#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rmcs_core::controller::identification {

// Physical-angle trajectories and count-domain probes. ADRC remains a separate
// component; this state machine neither computes nor consumes torque in Nm.
class CurrentSweep {
public:
    static constexpr std::size_t kJointCount = 4;
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    enum class Stage { kIdle, kMove, kSettle, kChirp, kValidation, kComplete, kAborted };

    struct Config {
        std::vector<double> angles_rad;
        std::vector<double> amplitudes_raw;
        std::array<double, kJointCount> current_to_physical_direction{-1., -1., -1., -1.};
        double min_angle_rad = 0.0;
        double max_angle_rad = 0.0;
        double move_speed_rad_s = 0.08;
        double settle_time_s = 1.0;
        double settle_timeout_s = 30.0;
        double move_timeout_s = 120.0;
        double angle_tolerance_rad = 0.01;
        double settle_velocity_rad_s = 0.04;
        double max_excursion_rad = 0.14;
        double max_velocity_rad_s = 3.0;
        double max_temperature_c = 60.0;
        double max_feedback_age_s = 0.05;
        double max_feedback_current_raw = 4096.0;
        double max_update_gap_s = 0.05;
        double start_frequency_hz = 0.2;
        double end_frequency_hz = 8.0;
        double chirp_duration_s = 20.0;
        double validation_duration_s = 6.0;
        double validation_step_s = 0.5;
        double probe_ramp_s = 0.3;
    };

    struct Feedback {
        double angle = kNan;
        double velocity = kNan;
        double current_raw = kNan;
        double temperature = kNan;
        double sequence = 0.0;
        double time_s = kNan;
    };

    explicit CurrentSweep(Config config)
        : config_(std::move(config)) {
        const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
        if (!std::isfinite(config_.min_angle_rad) || !std::isfinite(config_.max_angle_rad)
            || config_.min_angle_rad >= config_.max_angle_rad || config_.angles_rad.empty()
            || config_.amplitudes_raw.empty())
            throw std::invalid_argument("Current sweep requires physical-angle bounds and points");
        for (const double angle : config_.angles_rad)
            if (!std::isfinite(angle) || angle < config_.min_angle_rad
                || angle > config_.max_angle_rad)
                throw std::invalid_argument("Current sweep angle is outside physical bounds");
        for (const double amplitude : config_.amplitudes_raw)
            if (!positive(amplitude) || amplitude > 2048.0)
                throw std::invalid_argument("Sweep amplitudes must be 0..2048 protocol counts");
        for (const double direction : config_.current_to_physical_direction)
            if (direction != -1.0 && direction != 1.0)
                throw std::invalid_argument("Current-to-physical directions must be +/-1");
        for (const double value :
             {config_.move_speed_rad_s, config_.settle_time_s, config_.settle_timeout_s,
              config_.move_timeout_s, config_.angle_tolerance_rad, config_.settle_velocity_rad_s,
              config_.max_excursion_rad, config_.max_velocity_rad_s, config_.max_temperature_c,
              config_.max_feedback_age_s, config_.max_feedback_current_raw,
              config_.max_update_gap_s, config_.start_frequency_hz, config_.end_frequency_hz,
              config_.chirp_duration_s, config_.validation_duration_s, config_.validation_step_s,
              config_.probe_ramp_s})
            if (!positive(value))
                throw std::invalid_argument(
                    "Current sweep timing and protection values must be positive");
        if (config_.settle_timeout_s < config_.settle_time_s
            || 2.0 * config_.probe_ramp_s
                   >= std::min(config_.chirp_duration_s, config_.validation_duration_s))
            throw std::invalid_argument("Current sweep settle/ramp durations are inconsistent");
        reset();
    }

    void reset() {
        stage_ = Stage::kIdle;
        references_.fill(kNan);
        reference_velocities_.fill(kNan);
        probes_.fill(0.0);
        reason_.clear();
        point_ = joint_ = amplitude_ = 0;
        next_excitation_ = Stage::kChirp;
        stable_since_ = kNan;
        last_time_ = kNan;
        last_sequences_.fill(0.0);
    }

    bool start(double now, const std::array<Feedback, kJointCount>& feedback) {
        if (active())
            return false;
        reset();
        if (!healthy(now, feedback)) {
            abort(reason_);
            return false;
        }
        for (std::size_t i = 0; i < kJointCount; ++i) {
            references_[i] = feedback[i].angle;
            last_sequences_[i] = feedback[i].sequence;
        }
        reference_velocities_.fill(0.0);
        start_time_ = last_time_ = stage_start_ = now;
        stage_ = Stage::kMove;
        return true;
    }

    void stop(const std::string& reason = "stopped") { abort(reason); }

    void update(double now, const std::array<Feedback, kJointCount>& feedback, bool enabled) {
        if (!active())
            return;
        if (!enabled) {
            abort("remote reset or disabled");
            return;
        }
        const double dt = now - last_time_;
        if (!std::isfinite(dt) || dt <= 0.0 || dt > config_.max_update_gap_s) {
            abort("executor timing gap");
            return;
        }
        last_time_ = now;
        if (!healthy(now, feedback)) {
            abort(reason_);
            return;
        }
        probes_.fill(0.0);
        reference_velocities_.fill(0.0);
        if (stage_ == Stage::kMove) {
            bool arrived = true;
            for (std::size_t i = 0; i < kJointCount; ++i) {
                const double delta = config_.angles_rad[point_] - references_[i];
                const double step = std::clamp(
                    delta, -config_.move_speed_rad_s * dt, config_.move_speed_rad_s * dt);
                references_[i] += step;
                reference_velocities_[i] = step / dt;
                arrived &= std::abs(delta) <= config_.move_speed_rad_s * dt;
            }
            if (arrived)
                enter(Stage::kSettle, now);
            else if (now - stage_start_ > config_.move_timeout_s)
                abort("positioning timeout");
            return;
        }
        if (stage_ == Stage::kSettle) {
            bool stable = true;
            for (std::size_t i = 0; i < kJointCount; ++i)
                stable &=
                    std::abs(feedback[i].angle - references_[i]) <= config_.angle_tolerance_rad
                    && std::abs(feedback[i].velocity) <= config_.settle_velocity_rad_s;
            if (!stable)
                stable_since_ = kNan;
            else if (!std::isfinite(stable_since_))
                stable_since_ = now;
            else if (now - stable_since_ >= config_.settle_time_s)
                enter(next_excitation_, now);
            if (now - stage_start_ > config_.settle_timeout_s)
                abort("ADRC did not settle");
            return;
        }
        for (std::size_t i = 0; i < kJointCount; ++i)
            if (std::abs(feedback[i].angle - references_[i]) > config_.max_excursion_rad) {
                abort("physical-angle excursion");
                return;
            }
        const double elapsed = now - stage_start_;
        const double duration =
            stage_ == Stage::kChirp ? config_.chirp_duration_s : config_.validation_duration_s;
        if (elapsed >= duration) {
            advance(now);
            return;
        }
        const double edge = std::min(elapsed, duration - elapsed);
        const double ramp = std::clamp(edge / config_.probe_ramp_s, 0.0, 1.0);
        const double envelope = std::pow(std::sin(0.5 * std::numbers::pi * ramp), 2);
        double wave;
        if (stage_ == Stage::kChirp) {
            const double rate = (config_.end_frequency_hz - config_.start_frequency_hz) / duration;
            wave = std::sin(
                2.0 * std::numbers::pi
                * (config_.start_frequency_hz * elapsed + 0.5 * rate * elapsed * elapsed));
        } else {
            // Reserved for validation, never used to fit the training coefficients.
            wave = static_cast<int>(elapsed / config_.validation_step_s) % 2 ? -1.0 : 1.0;
        }
        probes_[joint_] = config_.current_to_physical_direction[joint_]
                        * config_.amplitudes_raw[amplitude_] * envelope * wave;
    }

    bool active() const {
        return stage_ == Stage::kMove || stage_ == Stage::kSettle || stage_ == Stage::kChirp
            || stage_ == Stage::kValidation;
    }
    Stage stage() const { return stage_; }
    std::size_t point() const { return point_; }
    std::size_t joint() const { return joint_; }
    double amplitude() const { return config_.amplitudes_raw[amplitude_]; }
    double elapsed(double now) const { return now - start_time_; }
    double frequency(double now) const {
        if (stage_ != Stage::kChirp)
            return 0.0;
        return config_.start_frequency_hz
             + (config_.end_frequency_hz - config_.start_frequency_hz) * (now - stage_start_)
                   / config_.chirp_duration_s;
    }
    const auto& references() const { return references_; }
    const auto& reference_velocities() const { return reference_velocities_; }
    const auto& probes() const { return probes_; }
    const auto& reason() const { return reason_; }

private:
    bool healthy(double now, const std::array<Feedback, kJointCount>& feedback) {
        if (!std::isfinite(now)) {
            reason_ = "invalid host clock";
            return false;
        }
        for (std::size_t i = 0; i < kJointCount; ++i) {
            const auto& f = feedback[i];
            if (!std::isfinite(f.angle) || !std::isfinite(f.velocity)
                || !std::isfinite(f.current_raw) || !std::isfinite(f.temperature)
                || !std::isfinite(f.sequence) || f.sequence < 1.0 || !std::isfinite(f.time_s)
                || now - f.time_s > config_.max_feedback_age_s || f.time_s > now
                || f.sequence < last_sequences_[i]) {
                reason_ = "invalid or stale feedback on joint " + std::to_string(i);
                return false;
            }
            if (f.angle < config_.min_angle_rad || f.angle > config_.max_angle_rad
                || std::abs(f.velocity) > config_.max_velocity_rad_s
                || f.temperature > config_.max_temperature_c
                || std::abs(f.current_raw) > config_.max_feedback_current_raw) {
                reason_ = "physical/current/temperature protection on joint " + std::to_string(i);
                return false;
            }
            last_sequences_[i] = f.sequence;
        }
        return true;
    }

    void abort(std::string reason) {
        reason_ = std::move(reason);
        stage_ = Stage::kAborted;
        references_.fill(kNan);
        reference_velocities_.fill(kNan);
        probes_.fill(0.0);
    }

    void enter(Stage stage, double now) {
        stage_ = stage;
        stage_start_ = now;
        stable_since_ = kNan;
    }

    void advance(double now) {
        if (stage_ == Stage::kChirp) {
            next_excitation_ = Stage::kValidation;
            enter(Stage::kSettle, now);
            return;
        }
        next_excitation_ = Stage::kChirp;
        if (++joint_ == kJointCount) {
            joint_ = 0;
            if (++amplitude_ == config_.amplitudes_raw.size()) {
                amplitude_ = 0;
                if (++point_ == config_.angles_rad.size()) {
                    point_ = config_.angles_rad.size() - 1;
                    stage_ = Stage::kComplete;
                    references_.fill(kNan);
                    reference_velocities_.fill(kNan);
                    return;
                }
                enter(Stage::kMove, now);
                return;
            }
        }
        enter(Stage::kSettle, now);
    }

    Config config_;
    Stage stage_ = Stage::kIdle;
    Stage next_excitation_ = Stage::kChirp;
    std::array<double, kJointCount> references_{}, reference_velocities_{}, probes_{},
        last_sequences_{};
    std::size_t point_ = 0, joint_ = 0, amplitude_ = 0;
    double start_time_ = 0.0, stage_start_ = 0.0, last_time_ = kNan, stable_since_ = kNan;
    std::string reason_;
};

struct CurrentCommand {
    double requested = 0.0;
    double encoded = 0.0;
    bool clipped = false;
    bool rate_limited = false;
};

inline CurrentCommand limit_current_command(
    double servo, double probe, double previous, double dt, double count_limit,
    double slew_counts_s = 0.0) {
    if (!std::isfinite(count_limit) || count_limit < 1.0 || count_limit > 2048.0
        || !std::isfinite(slew_counts_s) || slew_counts_s < 0.0)
        throw std::invalid_argument("Invalid native current limit");
    CurrentCommand result;
    result.requested = servo + probe;
    if (!std::isfinite(result.requested)) {
        result.clipped = true;
        return result;
    }
    const double limit = std::floor(count_limit);
    double current = std::clamp(result.requested, -limit, limit);
    result.clipped = current != result.requested;
    if (slew_counts_s > 0.0) {
        if (!std::isfinite(previous) || !std::isfinite(dt) || dt <= 0.0) {
            result.clipped = true;
            return result;
        }
        const double slewed =
            std::clamp(current, previous - slew_counts_s * dt, previous + slew_counts_s * dt);
        result.rate_limited = slewed != current;
        current = slewed;
    }
    result.encoded = std::round(std::clamp(current, -limit, limit));
    return result;
}

} // namespace rmcs_core::controller::identification

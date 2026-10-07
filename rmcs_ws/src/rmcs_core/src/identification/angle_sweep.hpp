#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace rmcs_core::controller::identification {

// Synchronized slow/fast chirps, a 1-Hz delay probe, and an optional step probe.
class AngleSweep {
public:
    struct Config {
        double min_angle = 0.0, max_angle = 0.0;
        double move_speed = 0.08;
        double slow_duration = 120.0, fast_duration = 60.0;
        double slow_start_hz = 0.02, slow_end_hz = 0.06;
        double fast_start_hz = 0.06, fast_end_hz = 0.12;
        double ultra_fast_frequency_hz = 1.0, ultra_fast_duration = 10.0;
        bool step_enable = false;
        double step_duration = 5.0;
        int step_cycles = 3;
    };
    explicit AngleSweep(Config config) : config_(config) {
        if (!std::isfinite(config.min_angle) || !std::isfinite(config.max_angle)
            || !(config.min_angle < config.max_angle))
            throw std::invalid_argument("Angle sweep requires min_angle < max_angle");
        for (double v : {config.move_speed, config.slow_duration, config.fast_duration,
                         config.slow_start_hz, config.slow_end_hz, config.fast_start_hz, config.fast_end_hz,
                         config.ultra_fast_frequency_hz, config.ultra_fast_duration})
            if (!std::isfinite(v) || v <= 0)
                throw std::invalid_argument("Angle sweep timing must be finite and positive");
        if (config.step_enable && (!std::isfinite(config.step_duration) || config.step_duration <= 0.0
                                   || config.step_cycles < 1))
            throw std::invalid_argument("Step response timing must be positive");
        reset();
    }
    void reset() { stage_ = 0; angle_.fill(NAN); velocity_.fill(0.0); frequency_ = 0.0; }
    bool start(double now, const std::array<double, 4>& measured) {
        if (active()) return false;
        for (std::size_t i = 0; i < 4; ++i)
            angle_[i] = std::isfinite(measured[i]) ? measured[i] : config_.min_angle;
        velocity_.fill(0.0);
        start_ = last_ = stage_start_ = now;
        frequency_ = 0.0;
        stage_ = 1;
        return true;
    }
    void stop() { reset(); stage_ = 6; }
    void update(double now, bool enabled) {
        if (!active()) return;
        if (!enabled) { stop(); return; }
        const double dt = now - last_;
        if (!std::isfinite(dt) || dt <= 0) return;
        last_ = now;
        if (stage_ == 1 || stage_ == 2 || stage_ == 4 || stage_ == 9) {
            bool arrived = true;
            for (std::size_t i = 0; i < 4; ++i) {
                const double delta = config_.min_angle - angle_[i];
                const double step = std::clamp(delta, -config_.move_speed * dt, config_.move_speed * dt);
                angle_[i] += step;
                velocity_[i] = step / dt;
                arrived &= std::abs(delta) <= config_.move_speed * dt;
            }
            if (arrived) {
                velocity_.fill(0.0);
                stage_ = stage_ == 1 ? 3 : stage_ == 2 ? 7 : stage_ == 4 ? 8
                    : (config_.step_enable ? 10 : 5);
                stage_start_ = now;
            }
            return;
        }
        if (stage_ == 10) {
            const double t = now - stage_start_;
            if (t >= config_.step_duration * 2.0 * config_.step_cycles) {
                angle_.fill(config_.min_angle);
                velocity_.fill(0.0);
                stage_ = 5;
                frequency_ = 0.0;
                return;
            }
            const auto phase = static_cast<int>(t / config_.step_duration) % 2;
            angle_.fill(phase == 0 ? config_.max_angle : config_.min_angle);
            velocity_.fill(0.0);
            frequency_ = 0.0;
            return;
        }
        const bool slow = stage_ == 3;
        const bool ultra_fast = stage_ == 8;
        const double duration = ultra_fast ? config_.ultra_fast_duration
            : slow ? config_.slow_duration : config_.fast_duration;
        const double f0 = ultra_fast ? config_.ultra_fast_frequency_hz
            : slow ? config_.slow_start_hz : config_.fast_start_hz;
        const double f1 = ultra_fast ? config_.ultra_fast_frequency_hz
            : slow ? config_.slow_end_hz : config_.fast_end_hz;
        const double t = std::min(now - stage_start_, duration);
        frequency_ = f0 + (f1 - f0) * t / duration;
        const double phase = 2.0 * std::numbers::pi * (f0 * t + 0.5 * (f1 - f0) * t * t / duration);
        const double radius = 0.5 * (config_.max_angle - config_.min_angle);
        angle_.fill(config_.min_angle + radius * (1.0 - std::cos(phase)));
        velocity_.fill(radius * std::sin(phase) * 2.0 * std::numbers::pi * frequency_);
        if (t >= duration) { stage_ = ultra_fast ? 9 : slow ? 2 : 4; frequency_ = 0.0; }
    }
    bool active() const { return stage_ != 0 && stage_ != 5 && stage_ != 6; }
    int stage() const { return stage_; }
    double elapsed(double now) const { return now - start_; }
    double frequency() const { return frequency_; }
    const auto& angles() const { return angle_; }
    const auto& velocities() const { return velocity_; }
private:
    Config config_;
    std::array<double, 4> angle_, velocity_;
    int stage_ = 0;
    double start_ = 0.0, last_ = 0.0, stage_start_ = 0.0, frequency_ = 0.0;
};
} // namespace rmcs_core::controller::identification

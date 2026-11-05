// Open-loop gimbal torque excitation for system identification.
//
// No PID cascade sits between this component and the drives: it writes
// /gimbal/{yaw,pitch}/control_torque directly, so the recorded input is the
// excitation itself.  Pitch supports a gravity/holding feed-forward recovered by
// the "calibration" experiment.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::identification {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

enum class Experiment : int {
    kHold = 0,
    kChirp = 1,
    kStaircase = 2,
    kSquare = 3,
    kCalibration = 4,
};

enum class State : int {
    kStopped = 0,
    kHolding = 1,
    kRunning = 2,
    kCompleted = 3,
    kFeedbackFault = 4,
    kLimitFault = 5,
    kExperimentMismatch = 6,
};

Experiment parse_experiment(const std::string& name) {
    if (name == "hold")
        return Experiment::kHold;
    if (name == "chirp")
        return Experiment::kChirp;
    if (name == "staircase")
        return Experiment::kStaircase;
    if (name == "square")
        return Experiment::kSquare;
    if (name == "calibration")
        return Experiment::kCalibration;
    throw std::invalid_argument("Unknown gimbal torque experiment: '" + name + "'");
}

const char* experiment_name(Experiment experiment) {
    switch (experiment) {
    case Experiment::kHold: return "hold";
    case Experiment::kChirp: return "chirp";
    case Experiment::kStaircase: return "staircase";
    case Experiment::kSquare: return "square";
    case Experiment::kCalibration: return "calibration";
    }
    return "unknown";
}

double wrap(double angle) { return std::remainder(angle, 2. * kPi); }

template <typename T>
T parameter_or(rclcpp::Node& node, const std::string& name, const T& fallback) {
    if (!node.has_parameter(name))
        node.declare_parameter<T>(name, fallback);
    return node.get_parameter(name).get_value<T>();
}

// Raised-cosine blend of one square-wave edge: 0 -> 1 over the edge time.
double edge_weight(double elapsed_in_half, double edge) {
    if (edge <= 0. || elapsed_in_half >= edge)
        return 1.;
    if (elapsed_in_half <= 0.)
        return 0.;
    return .5 - .5 * std::cos(kPi * elapsed_in_half / edge);
}

bool solve3(
    std::array<std::array<double, 3>, 3> matrix, std::array<double, 3> rhs,
    std::array<double, 3>& solution) {
    for (std::size_t column = 0; column < 3; ++column) {
        std::size_t pivot = column;
        for (std::size_t row = column + 1; row < 3; ++row)
            if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column]))
                pivot = row;
        if (std::abs(matrix[pivot][column]) < 1e-9)
            return false;
        std::swap(matrix[pivot], matrix[column]);
        std::swap(rhs[pivot], rhs[column]);
        for (std::size_t row = column + 1; row < 3; ++row) {
            const double factor = matrix[row][column] / matrix[column][column];
            for (std::size_t k = column; k < 3; ++k)
                matrix[row][k] -= factor * matrix[column][k];
            rhs[row] -= factor * rhs[column];
        }
    }
    for (std::size_t index = 3; index-- > 0;) {
        double value = rhs[index];
        for (std::size_t k = index + 1; k < 3; ++k)
            value -= matrix[index][k] * solution[k];
        solution[index] = value / matrix[index][index];
    }
    return true;
}

} // namespace

class GimbalTorqueSweepController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GimbalTorqueSweepController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        experiment_ = parse_experiment(parameter_or<std::string>(*this, "experiment", "hold"));
        torque_limit_ = parameter_or(*this, "torque_limit_nm", 1.5);
        amplitude_ = {
            parameter_or(*this, "yaw_torque_amplitude_nm", .5),
            parameter_or(*this, "pitch_torque_amplitude_nm", .5)};
        frequency_start_ = parameter_or(*this, "frequency_start_hz", .2);
        frequency_end_ = parameter_or(*this, "frequency_end_hz", 5.);
        const auto schedule = parameter_or<std::string>(*this, "chirp_schedule", "constant");
        if (schedule != "constant" && schedule != "velocity")
            throw std::invalid_argument("chirp_schedule must be 'constant' or 'velocity'");
        velocity_schedule_ = schedule == "velocity";
        duration_ = parameter_or(*this, "duration_s", 60.);
        ramp_ = parameter_or(*this, "ramp_s", 2.);
        settle_ = parameter_or(*this, "settle_s", 2.);
        staircase_steps_ = parameter_or(*this, "staircase_steps", 9);
        staircase_dwell_ = parameter_or(*this, "staircase_dwell_s", 2.);
        square_period_ = parameter_or(*this, "square_period_s", 10.);
        square_cycles_ = parameter_or(*this, "square_cycles", 6);
        square_edge_ = parameter_or(*this, "square_edge_s", .005);
        gravity_enabled_ = parameter_or(*this, "pitch_gravity_enabled", false);
        gravity_ = {
            parameter_or(*this, "pitch_gravity_cos_nm", 0.),
            parameter_or(*this, "pitch_gravity_sin_nm", 0.),
            parameter_or(*this, "pitch_gravity_const_nm", 0.)};
        calibration_angles_ = parameter_or<std::vector<double>>(
            *this, "calibration_angles_rad", std::vector<double>{});
        calibration_dwell_ = parameter_or(*this, "calibration_dwell_s", 1.5);
        calibration_kp_ = parameter_or(*this, "calibration_kp", 6.);
        calibration_ki_ = parameter_or(*this, "calibration_ki", 3.);
        calibration_rate_ = parameter_or(*this, "calibration_rate_rad_s", .6);
        pitch_min_ = parameter_or(*this, "pitch_min_rad", -.6109);
        pitch_max_ = parameter_or(*this, "pitch_max_rad", .5236);
        yaw_travel_limit_ = parameter_or(*this, "yaw_travel_limit_rad", .3490658503988659);
        max_velocity_ = parameter_or(*this, "max_velocity_rad_s", 4.);
        timeout_ = parameter_or(*this, "feedback_timeout_s", .1);

        validate();

        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/wheel_leg/dr16_fresh", remote_fresh_);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/gimbal/imu/last_steady_ns", imu_ns_);
        constexpr std::array axes{"yaw", "pitch"};
        for (std::size_t i = 0; i < axes.size(); ++i) {
            const auto prefix = std::string{"/gimbal/"} + axes[i];
            register_input(prefix + "/angle", angle_[i]);
            register_input(prefix + "/velocity", velocity_[i]);
            register_input(prefix + "/feedback_steady_ns", feedback_ns_[i]);
            register_output(prefix + "/control_torque", control_torque_[i], 0.);
            register_output(prefix + "/torque_sweep/excitation", excitation_[i], 0.);
        }
        register_output("/gimbal/pitch/torque_sweep/gravity", gravity_output_, 0.);
        register_output("/gimbal/pitch/torque_sweep/fit_cos", fit_[0], 0.);
        register_output("/gimbal/pitch/torque_sweep/fit_sin", fit_[1], 0.);
        register_output("/gimbal/pitch/torque_sweep/fit_const", fit_[2], 0.);
        register_output("/gimbal/pitch/torque_sweep/fit_residual", fit_residual_, 0.);
        register_output("/wheel_leg/enable_request", drive_enable_, false);
        register_output("/chassis/reset_count", reset_count_, std::size_t{0});
        constexpr std::array parked{"/wheel_leg/left_hip_joint",  "/wheel_leg/left_knee_joint",
                                    "/wheel_leg/right_hip_joint", "/wheel_leg/right_knee_joint",
                                    "/wheel_leg/left_wheel",      "/wheel_leg/right_wheel",
                                    "/gimbal/left_friction",      "/gimbal/right_friction",
                                    "/gimbal/bullet_feeder"};
        for (std::size_t i = 0; i < parked.size(); ++i)
            register_output(std::string{parked[i]} + "/control_torque", parked_torque_[i], 0.);
        register_output("/gimbal/torque_sweep/state", state_, 0);
        register_output("/gimbal/torque_sweep/mode", mode_, 0);
        register_output("/gimbal/torque_sweep/experiment", experiment_output_, 0);
        register_output("/gimbal/torque_sweep/axis", axis_, 0);
        register_output("/gimbal/torque_sweep/step", step_, 0);
        register_output("/gimbal/torque_sweep/enabled", enabled_, false);
        register_output("/gimbal/torque_sweep/completed", completed_, false);
        register_output("/gimbal/torque_sweep/elapsed_s", elapsed_, 0.);
        register_output("/gimbal/torque_sweep/frequency_hz", frequency_, 0.);
        register_output("/gimbal/torque_sweep/steady_ns", steady_ns_, std::uint64_t{0});
    }

    void update() override {
        // These outputs never participate in an enabled drive/shot graph.
        *drive_enable_ = false;
        for (auto& torque : parked_torque_)
            *torque = 0.;
        const auto now = std::chrono::steady_clock::now();
        *steady_ns_ =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        *experiment_output_ = static_cast<int>(experiment_);

        const int mode = remote_mode();
        *mode_ = mode;
        if (mode == 0) {
            previous_mode_ = 0;
            faulted_ = false;
            stop(State::kStopped);
            return;
        }
        if (mode != previous_mode_) {
            previous_mode_ = mode;
            faulted_ = false;
            stop(State::kStopped);
        }
        if (faulted_)
            return;
        // Calibration always drives pitch; selecting yaw or the sequence would
        // silently excite the wrong axis.
        if (experiment_ == Experiment::kCalibration && (mode == 2 || mode == 4)) {
            stop(State::kExperimentMismatch);
            faulted_ = true;
            return;
        }
        if (!feedback_valid()) {
            stop(State::kFeedbackFault);
            faulted_ = true;
            return;
        }
        if (std::abs(*velocity_[0]) > max_velocity_ || std::abs(*velocity_[1]) > max_velocity_) {
            stop(State::kLimitFault);
            faulted_ = true;
            return;
        }
        if (!*enabled_) {
            start();
            if (faulted_)
                return;
        }

        *elapsed_ = std::max(0., std::chrono::duration<double>(*timestamp_ - start_).count());
        const double pitch = wrap(*angle_[1]);
        if (std::abs(wrap(*angle_[0] - center_[0])) > yaw_travel_limit_ || pitch < pitch_min_
            || pitch > pitch_max_) {
            stop(State::kLimitFault);
            faulted_ = true;
            return;
        }

        std::array<double, 2> excitation{};
        *frequency_ = 0.;
        *axis_ = 0;
        *step_ = 0;
        *completed_ = false;
        set_state(State::kHolding);

        if (experiment_ == Experiment::kCalibration) {
            // Hold (middle/middle) never calibrates; the pitch switch owns it.
            if (mode == 3)
                run_calibration(*elapsed_, pitch, excitation);
        } else if (experiment_ != Experiment::kHold && mode != 1 && *elapsed_ >= settle_) {
            const double time = *elapsed_ - settle_;
            const double total = axis_duration() * (mode == 4 ? 2. : 1.);
            if (time >= total) {
                set_state(State::kCompleted);
                *completed_ = true;
            } else
                run_excitation(mode, time, excitation);
        }

        // Pitch holds its own weight whenever the model is available. During
        // calibration the PI output is the measurement, never a feed-forward.
        double gravity_torque = 0.;
        if (gravity_enabled_ && experiment_ != Experiment::kCalibration)
            gravity_torque =
                gravity_[0] * std::cos(pitch) + gravity_[1] * std::sin(pitch) + gravity_[2];
        *gravity_output_ = gravity_torque;

        for (std::size_t i = 0; i < excitation_.size(); ++i) {
            const double feedforward = i == 1 ? gravity_torque : 0.;
            *excitation_[i] = excitation[i];
            *control_torque_[i] =
                std::clamp(excitation[i] + feedforward, -torque_limit_, torque_limit_);
        }
    }

private:
    void set_state(State state) { *state_ = static_cast<int>(state); }

    void validate() {
        if (!std::isfinite(torque_limit_) || torque_limit_ <= 0. || torque_limit_ > 4.5)
            throw std::invalid_argument(
                "torque_limit_nm must be finite, positive and within the LK MG4010 4.5 Nm rating");
        for (const double amplitude : amplitude_)
            if (!std::isfinite(amplitude) || amplitude <= 0. || amplitude > torque_limit_)
                throw std::invalid_argument(
                    "Torque amplitudes must be finite, positive and within torque_limit_nm");
        if (!std::isfinite(frequency_start_) || frequency_start_ <= 0.
            || !std::isfinite(frequency_end_) || frequency_end_ < frequency_start_)
            throw std::invalid_argument("Invalid chirp frequency range");
        if (!std::isfinite(duration_) || duration_ <= 0. || !std::isfinite(ramp_) || ramp_ < 0.
            || ramp_ > duration_ / 2. || !std::isfinite(settle_) || settle_ < 0.)
            throw std::invalid_argument("Invalid chirp duration/ramp/settle");
        if (staircase_steps_ < 2 || !std::isfinite(staircase_dwell_) || staircase_dwell_ <= 0.)
            throw std::invalid_argument("staircase_steps must be >= 2 and the dwell positive");
        if (!std::isfinite(square_period_) || square_period_ <= 0. || square_cycles_ < 1
            || !std::isfinite(square_edge_) || square_edge_ < 0.
            || 2. * square_edge_ > square_period_)
            throw std::invalid_argument("Invalid square wave period/cycles/edge");
        if (!std::isfinite(pitch_min_) || !std::isfinite(pitch_max_) || pitch_min_ >= pitch_max_
            || !std::isfinite(yaw_travel_limit_) || yaw_travel_limit_ <= 0.)
            throw std::invalid_argument("Invalid angle limits");
        if (!std::isfinite(max_velocity_) || max_velocity_ <= 0. || !std::isfinite(timeout_)
            || timeout_ <= 0.)
            throw std::invalid_argument("Invalid velocity limit or feedback timeout");
        if (experiment_ == Experiment::kCalibration) {
            if (calibration_angles_.size() < 3)
                throw std::invalid_argument("Calibration needs at least three angles");
            if (!std::isfinite(calibration_dwell_) || calibration_dwell_ <= 0.
                || !std::isfinite(calibration_kp_) || calibration_kp_ <= 0.
                || !std::isfinite(calibration_ki_) || calibration_ki_ < 0.
                || !std::isfinite(calibration_rate_) || calibration_rate_ <= 0.)
                throw std::invalid_argument("Invalid calibration dwell, gains or rate");
            for (const double angle : calibration_angles_)
                if (!std::isfinite(angle) || angle < pitch_min_ || angle > pitch_max_)
                    throw std::invalid_argument(
                        "Calibration angles must lie within the pitch range");
        }
    }

    int remote_mode() const {
        using rmcs_msgs::Switch;
        if (!*remote_fresh_)
            return 0;
        const auto left = *switch_left_, right = *switch_right_;
        if (left == Switch::MIDDLE && right == Switch::MIDDLE)
            return 1;
        if (left == Switch::UP && right == Switch::MIDDLE)
            return 2;
        if (left == Switch::MIDDLE && right == Switch::UP)
            return 3;
        if (left == Switch::UP && right == Switch::UP)
            return 4;
        return 0;
    }

    bool feedback_valid() const {
        const auto now = *steady_ns_;
        const auto fresh = [this, now](std::uint64_t stamp) {
            return stamp != 0 && stamp <= now && (now - stamp) * 1e-9 < timeout_;
        };
        if (!fresh(*imu_ns_))
            return false;
        for (std::size_t i = 0; i < angle_.size(); ++i)
            if (!fresh(*feedback_ns_[i]) || !std::isfinite(*angle_[i])
                || !std::isfinite(*velocity_[i]))
                return false;
        return true;
    }

    void start() {
        center_ = {wrap(*angle_[0]), wrap(*angle_[1])};
        if (center_[1] < pitch_min_ || center_[1] > pitch_max_) {
            stop(State::kLimitFault);
            faulted_ = true;
            return;
        }
        start_ = *timestamp_;
        ++*reset_count_;
        *enabled_ = true;
        *completed_ = false;
        calibration_done_ = false;
        calibration_index_ = 0;
        calibration_integral_ = 0.;
        calibration_since_ = 0.;
        calibration_last_ = 0.;
        calibration_reference_ = center_[1];
        calibration_target_ = center_[1];
        last_sample_ = std::numeric_limits<std::size_t>::max();
        calibration_samples_.clear();
        for (auto& fit : fit_)
            *fit = kNan;
        *fit_residual_ = kNan;
        RCLCPP_INFO(
            get_logger(), "Gimbal torque experiment '%s' started at [%.4f, %.4f] rad",
            experiment_name(experiment_), center_[0], center_[1]);
    }

    double axis_duration() const {
        switch (experiment_) {
        case Experiment::kChirp: return duration_;
        case Experiment::kStaircase: return staircase_steps_ * staircase_dwell_;
        case Experiment::kSquare: return square_cycles_ * square_period_;
        case Experiment::kCalibration:
        case Experiment::kHold: return 0.;
        }
        return 0.;
    }

    // Returns {axis, local time}; axis < 0 means the whole sequence is over.
    std::pair<int, double> split_axis(int mode, double time) const {
        if (mode == 2)
            return {0, time};
        if (mode == 3)
            return {1, time};
        const double per_axis = axis_duration();
        if (time >= 2. * per_axis)
            return {-1, 0.};
        return time >= per_axis ? std::pair{1, time - per_axis} : std::pair{0, time};
    }

    void run_excitation(int mode, double time, std::array<double, 2>& excitation) {
        const auto [axis, local] = split_axis(mode, time);
        if (axis < 0) {
            set_state(State::kCompleted);
            *completed_ = true;
            return;
        }
        *axis_ = axis + 1;
        set_state(State::kRunning);
        switch (experiment_) {
        case Experiment::kChirp: {
            const auto wave = chirp(local, amplitude_[axis]);
            excitation[axis] = wave[0];
            *frequency_ = wave[1];
            return;
        }
        case Experiment::kStaircase: {
            const int index =
                std::clamp(int(local / staircase_dwell_), 0, int(staircase_steps_) - 1);
            const double level = -.5 + double(index) / double(staircase_steps_ - 1);
            excitation[axis] = 2. * level * amplitude_[axis];
            *step_ = index + 1;
            return;
        }
        case Experiment::kSquare: {
            const double half = square_period_ / 2.;
            const auto index = std::size_t(local / half);
            const double since = local - double(index) * half;
            const double level = index % 2 == 0 ? 1. : -1.;
            // The first edge rises from rest instead of jumping straight to -A.
            const double previous = index == 0 ? 0. : -level;
            const double weight = edge_weight(since, square_edge_);
            excitation[axis] = amplitude_[axis] * (previous + (level - previous) * weight);
            *step_ = int(index) + 1;
            return;
        }
        case Experiment::kCalibration:
        case Experiment::kHold: return;
        }
    }

    void run_calibration(double time, double pitch, std::array<double, 2>& excitation) {
        const std::size_t points = calibration_angles_.size();
        const double total = double(points * 2) * calibration_dwell_;
        *axis_ = 2;
        const double dt = std::max(0., time - calibration_last_);
        calibration_last_ = time;

        if (!calibration_done_ && time >= total) {
            finish_calibration();
            calibration_done_ = true;
        }
        if (calibration_done_) {
            // Keep holding the last angle: a pitch gimbal with no torque falls.
            excitation[1] = hold_torque(calibration_target_, pitch, dt);
            set_state(State::kCompleted);
            *completed_ = true;
            *step_ = int(points * 2);
            return;
        }

        const auto index = std::min(std::size_t(time / calibration_dwell_), points * 2 - 1);
        const std::size_t point = index < points ? index : points * 2 - 1 - index;
        if (index != calibration_index_) {
            calibration_index_ = index;
            calibration_since_ = time;
            calibration_target_ = calibration_angles_[point];
        }
        // Quasi-static reference: slew instead of stepping, so the PI never
        // launches the joint at a speed that trips the velocity guard.
        const double step = calibration_rate_ * dt;
        calibration_reference_ +=
            std::clamp(calibration_target_ - calibration_reference_, -step, step);

        excitation[1] = hold_torque(calibration_reference_, pitch, dt);
        set_state(State::kRunning);
        *step_ = int(index) + 1;

        // Sample only a settled, unsaturated equilibrium, and pair the torque
        // with the measured angle: a PI that lags the reference must not bias
        // the gravity model.
        const bool settled = time - calibration_since_ >= .98 * calibration_dwell_
                          && last_sample_ != index && std::abs(*velocity_[1]) < .03
                          && std::abs(excitation[1]) < torque_limit_ - 1e-3;
        if (settled) {
            calibration_samples_.push_back({pitch, excitation[1]});
            last_sample_ = index;
        }
    }

    double hold_torque(double setpoint, double pitch, double dt) {
        const double error = wrap(setpoint - pitch);
        const double unsaturated =
            calibration_kp_ * error + calibration_ki_ * calibration_integral_;
        const double torque = std::clamp(unsaturated, -torque_limit_, torque_limit_);
        // Anti-windup: integrate only while the command is not saturated.
        // Anti-windup on saturation only: an error gate would deadlock once the
        // steady tracking error sits above it.
        if (unsaturated == torque)
            calibration_integral_ += error * dt;
        return torque;
    }

    void finish_calibration() {
        std::array<std::array<double, 3>, 3> normal{};
        std::array<double, 3> rhs{};
        for (const auto& [angle, torque] : calibration_samples_) {
            const std::array<double, 3> basis{std::cos(angle), std::sin(angle), 1.};
            for (std::size_t row = 0; row < 3; ++row) {
                for (std::size_t column = 0; column < 3; ++column)
                    normal[row][column] += basis[row] * basis[column];
                rhs[row] += basis[row] * torque;
            }
        }
        std::array<double, 3> solution{};
        if (calibration_samples_.size() < 3 || !solve3(normal, rhs, solution)) {
            for (auto& fit : fit_)
                *fit = kNan;
            *fit_residual_ = kNan;
            RCLCPP_ERROR(get_logger(), "Pitch calibration fit failed: degenerate samples");
            return;
        }
        double squared = 0.;
        for (const auto& [angle, torque] : calibration_samples_) {
            const double predicted =
                solution[0] * std::cos(angle) + solution[1] * std::sin(angle) + solution[2];
            squared += (predicted - torque) * (predicted - torque);
        }
        *fit_residual_ = std::sqrt(squared / double(calibration_samples_.size()));
        for (std::size_t i = 0; i < fit_.size(); ++i)
            *fit_[i] = solution[i];
        RCLCPP_INFO(
            get_logger(),
            "Pitch holding model over %zu points: cos=%.4f sin=%.4f const=%.4f rms=%.4f Nm\n"
            "    pitch_gravity_cos_nm: %.4f\n    pitch_gravity_sin_nm: %.4f\n"
            "    pitch_gravity_const_nm: %.4f\n    pitch_gravity_enabled: true",
            calibration_samples_.size(), solution[0], solution[1], solution[2], *fit_residual_,
            solution[0], solution[1], solution[2]);
    }

    // Logarithmic chirp with a raised-cosine fade at both ends; returns
    // {torque, frequency}. The commanded torque is analytic, so the CSV can be
    // fitted without differentiating a sampled signal.
    std::array<double, 2> chirp(double time, double amplitude) const {
        const double growth = std::log(frequency_end_ / frequency_start_) / duration_;
        const double frequency = frequency_start_ * std::exp(growth * time);
        // A constant torque chirp drives a position-limited joint as 1/w^2, so
        // the low end can hit the travel guard before the sweep starts. Scaling
        // the torque with frequency keeps the displacement bounded.
        if (velocity_schedule_)
            amplitude *= frequency / frequency_end_;
        const double phase = 2. * kPi * frequency_start_
                           * (std::abs(growth) < 1e-12 ? time : std::expm1(growth * time) / growth);
        double fade = 1.;
        if (time < ramp_ || time > duration_ - ramp_) {
            const double local = time < ramp_ ? time : duration_ - time;
            const double theta = kPi * local / ramp_;
            fade = .5 - .5 * std::cos(theta);
        }
        return {amplitude * fade * std::sin(phase), frequency};
    }

    void stop(State state) {
        if (*enabled_)
            ++*reset_count_;
        *enabled_ = false;
        set_state(state);
        *axis_ = 0;
        *step_ = 0;
        *frequency_ = 0.;
        *completed_ = false;
        *elapsed_ = 0.;
        last_sample_ = std::numeric_limits<std::size_t>::max();
        calibration_done_ = false;
        calibration_index_ = 0;
        calibration_integral_ = 0.;
        calibration_since_ = 0.;
        calibration_last_ = 0.;
        calibration_samples_.clear();
        for (std::size_t i = 0; i < excitation_.size(); ++i) {
            *excitation_[i] = 0.;
            *control_torque_[i] = 0.;
        }
        *gravity_output_ = 0.;
    }

    Experiment experiment_ = Experiment::kHold;
    double torque_limit_ = 1.5, frequency_start_ = .2, frequency_end_ = 5.;
    double duration_ = 60., ramp_ = 2., settle_ = 2.;
    double staircase_dwell_ = 2., square_period_ = 10., square_edge_ = .005;
    double calibration_dwell_ = 1.5, calibration_kp_ = 6., calibration_ki_ = 3.;
    double calibration_rate_ = .6;
    double pitch_min_ = -.6109, pitch_max_ = .5236, yaw_travel_limit_ = .3490658503988659;
    double max_velocity_ = 4., timeout_ = .1;
    int staircase_steps_ = 9, square_cycles_ = 6;
    bool gravity_enabled_ = false, velocity_schedule_ = false;
    std::array<double, 2> amplitude_{.5, .5};
    std::array<double, 3> gravity_{0., 0., 0.};
    std::vector<double> calibration_angles_;

    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<bool> remote_fresh_;
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    InputInterface<std::uint64_t> imu_ns_;
    std::array<InputInterface<double>, 2> angle_, velocity_;
    std::array<InputInterface<std::uint64_t>, 2> feedback_ns_;
    std::array<OutputInterface<double>, 2> control_torque_, excitation_;
    OutputInterface<double> gravity_output_, fit_residual_;
    std::array<OutputInterface<double>, 3> fit_;
    OutputInterface<bool> drive_enable_, enabled_, completed_;
    OutputInterface<std::size_t> reset_count_;
    std::array<OutputInterface<double>, 9> parked_torque_;
    OutputInterface<int> state_, mode_, experiment_output_, axis_, step_;
    OutputInterface<double> elapsed_, frequency_;
    OutputInterface<std::uint64_t> steady_ns_;

    std::array<double, 2> center_{};
    std::chrono::steady_clock::time_point start_;
    int previous_mode_ = 0;
    bool faulted_ = false, calibration_done_ = false;
    std::size_t calibration_index_ = 0;
    double calibration_integral_ = 0., calibration_since_ = 0., calibration_last_ = 0.;
    double calibration_reference_ = 0., calibration_target_ = 0.;
    std::size_t last_sample_ = std::numeric_limits<std::size_t>::max();
    std::vector<std::pair<double, double>> calibration_samples_;
};

} // namespace rmcs_core::controller::identification

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::GimbalTorqueSweepController, rmcs_executor::Component)

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

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::identification {

class GimbalFrequencySweepController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GimbalFrequencySweepController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        // Double-middle drives the normal gimbal; a missing publisher must not
        // stop the sweep profile, so the stick defaults to centered.
        register_input("/remote/joystick/left", joystick_left_, false);
        register_input("/wheel_leg/dr16_fresh", remote_fresh_);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/gimbal/imu/last_steady_ns", imu_ns_);
        constexpr std::array axes{"yaw", "pitch"};
        for (std::size_t i = 0; i < axes.size(); ++i) {
            const auto prefix = std::string{"/gimbal/"} + axes[i];
            register_input(prefix + "/angle", angle_[i]);
            register_input(prefix + "/velocity_imu", velocity_[i]);
            register_input(prefix + "/feedback_steady_ns", feedback_ns_[i]);
            register_output(prefix + "/control_angle_error", error_[i], nan_);
            register_output(prefix + "/sweep/reference", reference_[i], nan_);
            register_output(prefix + "/sweep/reference_velocity", reference_velocity_[i], 0.);
        }
        register_output("/wheel_leg/enable_request", drive_enable_, false);
        register_output("/chassis/reset_count", reset_count_, std::size_t{0});
        constexpr std::array parked{"/wheel_leg/left_hip_joint",  "/wheel_leg/left_knee_joint",
                                    "/wheel_leg/right_hip_joint", "/wheel_leg/right_knee_joint",
                                    "/wheel_leg/left_wheel",      "/wheel_leg/right_wheel",
                                    "/gimbal/left_friction",      "/gimbal/right_friction",
                                    "/gimbal/bullet_feeder"};
        for (std::size_t i = 0; i < parked.size(); ++i)
            register_output(std::string{parked[i]} + "/control_torque", parked_torque_[i], 0.);
        register_output("/gimbal/sweep/state", state_, 0);
        register_output("/gimbal/sweep/mode", mode_, 0);
        register_output("/gimbal/sweep/axis", axis_, 0);
        register_output("/gimbal/sweep/enabled", enabled_, false);
        register_output("/gimbal/sweep/completed", completed_, false);
        register_output("/gimbal/sweep/elapsed_s", elapsed_, 0.);
        register_output("/gimbal/sweep/frequency_hz", frequency_, 0.);
        register_output("/gimbal/sweep/steady_ns", steady_ns_, std::uint64_t{0});

        frequency_start_ = get_parameter("frequency_start_hz").as_double();
        frequency_end_ = get_parameter("frequency_end_hz").as_double();
        duration_ = get_parameter("duration_s").as_double();
        ramp_ = get_parameter("ramp_s").as_double();
        settle_ = get_parameter("settle_s").as_double();
        amplitude_ = {
            get_parameter("yaw_amplitude_rad").as_double(),
            get_parameter("pitch_amplitude_rad").as_double()};
        timeout_ = get_parameter("feedback_timeout_s").as_double();
        get_parameter_or("manual_yaw_rate_rad_s", manual_yaw_rate_, 6.);
        get_parameter_or("manual_pitch_rate_rad_s", manual_pitch_rate_, 6.);
        get_parameter_or("manual_deadzone", manual_deadzone_, .02);
        for (const double value :
             {frequency_start_, frequency_end_, duration_, ramp_, amplitude_[0], amplitude_[1],
              timeout_})
            if (!std::isfinite(value) || value <= 0.)
                throw std::invalid_argument("Gimbal sweep ranges must be finite and positive");
        if (!std::isfinite(settle_) || settle_ < 0. || frequency_end_ < frequency_start_
            || ramp_ > duration_ / 2.)
            throw std::invalid_argument("Invalid gimbal sweep frequency, envelope or angle limits");
        for (const double rate : {manual_yaw_rate_, manual_pitch_rate_})
            if (!std::isfinite(rate) || rate <= 0.)
                throw std::invalid_argument("Manual gimbal rates must be finite and positive");
        if (!std::isfinite(manual_deadzone_) || manual_deadzone_ < 0. || manual_deadzone_ >= 1.)
            throw std::invalid_argument("manual_deadzone must be in [0, 1)");
    }

    void update() override {
        // These outputs never participate in an enabled drive/shot graph.
        *drive_enable_ = false;
        for (auto& torque : parked_torque_)
            *torque = 0.;
        const auto now = std::chrono::steady_clock::now();
        *steady_ns_ =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        const int mode = remote_mode();
        *mode_ = mode;
        if (mode == 0) {
            disable(0);
            previous_mode_ = 0;
            faulted_ = false;
            return;
        }
        if (mode != previous_mode_) {
            previous_mode_ = mode;
            faulted_ = false;
            disable(0);
        }
        if (faulted_)
            return;
        if (!feedback_valid(*steady_ns_)) {
            disable(4);
            faulted_ = true;
            return;
        }
        if (!*enabled_) {
            center_ = {signed_angle(*angle_[0]), signed_angle(*angle_[1])};
            start_ = *timestamp_;
            ++*reset_count_;
            *enabled_ = true;
        }
        *elapsed_ = std::max(0., std::chrono::duration<double>(*timestamp_ - start_).count());

        std::array<double, 2> target = center_;
        std::array<double, 2> target_velocity{};
        if (mode == 1)
            update_manual_reference(now, target, target_velocity);
        *frequency_ = 0.;
        *axis_ = 0;
        *completed_ = false;
        *state_ = 1;
        const double time = *elapsed_ - settle_;
        const double total = duration_ * (mode == 4 ? 2. : 1.);
        if (mode != 1 && time >= total) {
            *state_ = 3;
            *completed_ = true;
        } else if (mode != 1 && time >= 0.) {
            const std::size_t axis = mode == 3 || (mode == 4 && time >= duration_) ? 1 : 0;
            const double t = mode == 4 && axis == 1 ? time - duration_ : time;
            const auto wave = chirp(t, amplitude_[axis]);
            target[axis] += wave[0];
            target_velocity[axis] = wave[1];
            *frequency_ = wave[2];
            *axis_ = static_cast<int>(axis + 1);
            *state_ = 2;
        }
        for (std::size_t i = 0; i < target.size(); ++i) {
            *reference_[i] = target[i];
            *reference_velocity_[i] = target_velocity[i];
            *error_[i] = signed_angle(target[i] - *angle_[i]);
        }
    }

private:
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

    bool feedback_valid(std::uint64_t now) const {
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

    // Logarithmic chirp with a raised-cosine fade at both ends. The returned
    // derivative includes the fade, so recorded reference velocity is analytic.
    std::array<double, 3> chirp(double time, double amplitude) const {
        const double growth = std::log(frequency_end_ / frequency_start_) / duration_;
        const double frequency = frequency_start_ * std::exp(growth * time);
        const double phase = 2. * std::numbers::pi * frequency_start_
                           * (std::abs(growth) < 1e-12 ? time : std::expm1(growth * time) / growth);
        double fade = 1., fade_rate = 0.;
        if (time < ramp_ || time > duration_ - ramp_) {
            const bool entering = time < ramp_;
            const double t = entering ? time : duration_ - time;
            const double theta = std::numbers::pi * t / ramp_;
            fade = .5 - .5 * std::cos(theta);
            fade_rate = (entering ? 1. : -1.) * .5 * std::numbers::pi / ramp_ * std::sin(theta);
        }
        return {
            amplitude * fade * std::sin(phase),
            amplitude
                * (fade_rate * std::sin(phase)
                   + fade * 2. * std::numbers::pi * frequency * std::cos(phase)),
            frequency};
    }

    void disable(int state) {
        if (*enabled_)
            ++*reset_count_;
        *enabled_ = false;
        manual_initialized_ = false;
        *state_ = state;
        *axis_ = 0;
        *frequency_ = 0.;
        *completed_ = false;
        *elapsed_ = 0.;
        for (std::size_t i = 0; i < error_.size(); ++i) {
            *error_[i] = nan_;
            *reference_[i] = nan_;
            *reference_velocity_[i] = 0.;
        }
    }

    // Double-middle behaves like the normal gimbal: the left stick integrates a
    // reference angle that the existing PID cascade tracks.
    void update_manual_reference(
        const std::chrono::steady_clock::time_point& now, std::array<double, 2>& target,
        std::array<double, 2>& target_velocity) {
        const double dt =
            std::clamp(std::chrono::duration<double>(now - manual_last_).count(), 0., .01);
        manual_last_ = now;
        if (!manual_initialized_) {
            manual_reference_ = center_;
            manual_initialized_ = true;
            target = manual_reference_;
            return;
        }
        const Eigen::Vector2d stick =
            joystick_left_.ready() ? *joystick_left_ : Eigen::Vector2d{0., 0.};
        const auto deadband = [this](double value) {
            return std::abs(value) < manual_deadzone_ ? 0. : value;
        };
        const double yaw_rate = manual_yaw_rate_ * deadband(stick.y());
        const double pitch_rate = -manual_pitch_rate_ * deadband(stick.x());
        manual_reference_[0] = signed_angle(manual_reference_[0] + yaw_rate * dt);
        manual_reference_[1] = signed_angle(manual_reference_[1] + pitch_rate * dt);
        target = manual_reference_;
        target_velocity = {yaw_rate, pitch_rate};
    }

    static double signed_angle(double angle) {
        return std::remainder(angle, 2. * std::numbers::pi);
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<bool> remote_fresh_;
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    InputInterface<std::uint64_t> imu_ns_;
    std::array<InputInterface<double>, 2> angle_, velocity_;
    std::array<InputInterface<std::uint64_t>, 2> feedback_ns_;
    std::array<OutputInterface<double>, 2> error_, reference_, reference_velocity_;
    OutputInterface<bool> drive_enable_, enabled_, completed_;
    OutputInterface<std::size_t> reset_count_;
    std::array<OutputInterface<double>, 9> parked_torque_;
    OutputInterface<int> state_, mode_, axis_;
    OutputInterface<double> elapsed_, frequency_;
    OutputInterface<std::uint64_t> steady_ns_;

    double frequency_start_, frequency_end_, duration_, ramp_, settle_;
    std::array<double, 2> amplitude_, center_{};
    double timeout_;
    double manual_yaw_rate_ = 6., manual_pitch_rate_ = 6., manual_deadzone_ = .02;
    std::array<double, 2> manual_reference_{};
    std::chrono::steady_clock::time_point start_, manual_last_{};
    bool manual_initialized_ = false;
    int previous_mode_ = 0;
    bool faulted_ = false;
};

} // namespace rmcs_core::controller::identification

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::GimbalFrequencySweepController, rmcs_executor::Component)

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "hardware/device/actuator_latency_trace.hpp"
#include "identification/wheel_leg_identification_axes.hpp"
#include "identification/wheel_leg_latency_plan.hpp"
#include "identification/wheel_leg_load_compensation.hpp"
#include "identification/wheel_leg_sweep_session.hpp"

namespace rmcs_core::controller::identification {

class WheelLegLatencyProbeController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegLatencyProbeController()
        : Node{get_component_name(), node::options()}
        , plan_(
              get_parameter("probe_amplitude_nm").as_double(),
              get_parameter("bit_duration_s").as_double(),
              get_parameter("segment_duration_s").as_double()) {
        const auto name = get_parameter("axis").as_string();
        const auto found = std::ranges::find(kAxisNames, name);
        if (found == kAxisNames.end())
            throw std::invalid_argument("Unknown latency axis: " + name);
        axis_ = found - kAxisNames.begin();
        kp_ = get_parameter("joint_kp").as_double();
        kd_ = get_parameter("joint_kd").as_double();
        cap_ = get_parameter("torque_limit_nm").as_double();
        excursion_ = get_parameter("joint_excursion_limit_rad").as_double();
        speed_ = get_parameter("speed_limit_rad_s").as_double();
        target_velocity_ = get_parameter("wheel_velocity_rad_s").as_double();
        if (!std::isfinite(kp_) || kp_ < 0 || !std::isfinite(kd_) || kd_ <= 0
            || !std::isfinite(cap_) || cap_ <= 0 || cap_ > (axis_ < 4 ? 40. : 4.5)
            || !std::isfinite(excursion_) || excursion_ <= 0 || !std::isfinite(speed_)
            || speed_ <= 0 || !std::isfinite(target_velocity_)
            || std::abs(target_velocity_) >= speed_
            || get_parameter("probe_amplitude_nm").as_double() > cap_ * .5)
            throw std::invalid_argument("Invalid latency hold gains, amplitude or limits");
        register_input("/predefined/update_rate", rate_);
        register_input("/remote/switch/left", left_);
        register_input("/remote/switch/right", right_);
        register_input("/wheel_leg/dr16_fresh", remote_fresh_);
        register_input("/wheel_leg/dm_control_ready", ready_);
        register_output("/wheel_leg/enable_request", enable_, false);
        register_output("/wheel_leg/clear_error_request", clear_, false);
        register_output("/chassis/reset_count", reset_, std::size_t{0});
        register_output("/wheel_leg/identification/wheel_only", wheel_only_, axis_ >= 4);
        if (axis_ < 4)
            register_output("/wheel_leg/identification/selected_side", side_, axis_ / 2);
        register_output("/wheel_leg/latency/axis", axis_output_, axis_);
        register_output("/wheel_leg/latency/stage", stage_, -1);
        register_output("/wheel_leg/latency/validation", validation_, false);
        register_output("/wheel_leg/latency/active", active_, false);
        register_output("/wheel_leg/latency/completed", completed_, false);
        register_output("/wheel_leg/latency/fault", fault_, 0);
        register_output("/wheel_leg/latency/run_id", run_, std::uint64_t{0});
        register_output("/wheel_leg/latency/probe_nm", probe_, 0.);
        register_output("/wheel_leg/latency/saturated", saturated_, false);
        register_output("/wheel_leg/latency/control_steady_ns", start_ns_, std::uint64_t{0});
        register_output("/wheel_leg/latency/control_end_steady_ns", end_ns_, std::uint64_t{0});
        register_output("/wheel_leg/latency/preclip_nm", preclip_, 0.);
        for (std::size_t i = 0; i < 6; ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kAxisNames[i];
            register_input(prefix + "/angle", angle_[i]);
            register_input(prefix + "/velocity", velocity_[i]);
            register_input(prefix + "/feedback_steady_ns", feedback_ns_[i]);
            register_input(prefix + "/max_torque", motor_cap_[i]);
            register_output(prefix + "/control_torque", torque_[i], 0.);
        }
        constexpr std::array auxiliary{
            "/gimbal/yaw", "/gimbal/pitch", "/gimbal/left_friction", "/gimbal/right_friction",
            "/gimbal/bullet_feeder"};
        for (std::size_t i = 0; i < auxiliary.size(); ++i)
            register_output(std::string{auxiliary[i]} + "/control_torque", auxiliary_[i], 0.);
    }

    void update() override {
        using Clock = hardware::device::ActuatorLatencyTrace;
        *start_ns_ = Clock::now_ns();
        const double dt = previous_ns_ ? (*start_ns_ - previous_ns_) * 1e-9 : .001;
        previous_ns_ = *start_ns_;
        if (*rate_ != 1000.)
            throw std::invalid_argument("Latency probe requires 1000 Hz executor");
        for (auto& effort : torque_)
            *effort = 0.;
        *active_ = *saturated_ = false;
        *probe_ = *preclip_ = 0.;
        const bool down = *left_ == rmcs_msgs::Switch::DOWN && *right_ == rmcs_msgs::Switch::DOWN;
        if (down && !previous_down_) {
            ++*reset_;
            *fault_ = 0;
            for (auto& load : load_)
                load.reset();
        }
        previous_down_ = down;
        const double now = *start_ns_ * 1e-9;
        bool feedback_ok = true;
        for (int i = 0; i < 6; ++i) {
            if (!participates(i))
                continue;
            feedback_ok &= std::isfinite(*angle_[i]) && std::isfinite(*velocity_[i])
                        && *feedback_ns_[i] > 0 && *start_ns_ >= *feedback_ns_[i]
                        && *start_ns_ - *feedback_ns_[i] < 50'000'000;
        }
        const bool started = session_.update(
            now, *remote_fresh_, *left_, *right_,
            feedback_ok ? SweepStop::kNone : SweepStop::kFeedbackTimeout);
        if (down) {
            *stage_ = -1;
            *validation_ = false;
            *completed_ = false;
        }
        if (started) {
            origin_.fill(0.);
            for (int i = 0; i < 6; ++i) {
                origin_[i] = *angle_[i];
                load_[i].reset();
            }
            ready_since_ns_ = 0;
            was_ready_ = false;
        }
        *enable_ = session_.enabled() && !*fault_;
        *run_ = session_.run_id;
        if (session_.state == SweepState::kRunning && *ready_ && !*fault_) {
            if (!ready_since_ns_)
                ready_since_ns_ = *start_ns_;
            was_ready_ = true;
            const double elapsed = (*start_ns_ - ready_since_ns_) * 1e-9;
            const auto sample = plan_.at(elapsed);
            *stage_ = sample.stage;
            *validation_ = sample.validation;
            *active_ = sample.active;
            *probe_ = sample.probe;
            if (sample.completed) {
                session_.complete();
                *completed_ = true;
            } else {
                for (int i = 0; i < 6; ++i) {
                    if (!participates(i))
                        continue;
                    const bool wheel = i >= 4;
                    const double error = origin_[i] - *angle_[i];
                    if ((!wheel && std::abs(error) > excursion_)
                        || std::abs(*velocity_[i]) > speed_) {
                        *fault_ = 1;
                        break;
                    }
                    const double base = wheel ? kd_ * (target_velocity_ - *velocity_[i])
                                              : kp_ * error - kd_ * *velocity_[i];
                    const double limit = std::min(cap_, *motor_cap_[i]);
                    const double load = wheel         ? 0.
                                      : elapsed >= 2. ? load_[i].value()
                                                      : load_[i].update(
                                                            error, base, limit, dt, 20.,
                                                            std::min(16., cap_ * .5), 4., false);
                    const double preclip = base + load + (i == axis_ ? sample.probe : 0.);
                    *torque_[i] = std::clamp(preclip, -limit, limit);
                    if (i == axis_) {
                        *preclip_ = preclip;
                        *saturated_ = std::abs(preclip) >= limit;
                    }
                }
            }
        } else if (was_ready_ && session_.state == SweepState::kRunning) {
            *fault_ = 2;
        }
        if (*fault_) {
            *enable_ = *active_ = false;
            for (auto& effort : torque_)
                *effort = 0.;
        }
        *end_ns_ = Clock::now_ns();
    }

private:
    bool participates(int i) const { return axis_ >= 4 ? i == axis_ : i < 4 && i / 2 == axis_ / 2; }

    InputInterface<double> rate_;
    InputInterface<rmcs_msgs::Switch> left_, right_;
    InputInterface<bool> remote_fresh_, ready_;
    std::array<InputInterface<double>, 6> angle_, velocity_, motor_cap_;
    std::array<InputInterface<std::uint64_t>, 6> feedback_ns_;
    std::array<OutputInterface<double>, 6> torque_;
    std::array<OutputInterface<double>, 5> auxiliary_;
    OutputInterface<bool> enable_, clear_, wheel_only_, validation_, active_, completed_,
        saturated_;
    OutputInterface<int> side_, axis_output_, stage_, fault_;
    OutputInterface<std::size_t> reset_;
    OutputInterface<std::uint64_t> run_, start_ns_, end_ns_;
    OutputInterface<double> probe_, preclip_;
    WheelLegLatencyPlan plan_;
    WheelLegSweepSession session_{true, true, 1};
    std::array<WheelLegLoadCompensation, 6> load_;
    std::array<double, 6> origin_{};
    std::uint64_t ready_since_ns_ = 0;
    std::uint64_t previous_ns_ = 0;
    int axis_ = 0;
    double kp_, kd_, cap_, excursion_, speed_, target_velocity_;
    bool previous_down_ = false, was_ready_ = false;
};

} // namespace rmcs_core::controller::identification

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::WheelLegLatencyProbeController, rmcs_executor::Component)

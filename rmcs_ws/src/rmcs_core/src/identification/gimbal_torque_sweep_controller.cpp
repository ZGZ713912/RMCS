#include <array>
#include <cstddef>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::identification {

// Gates the standard deformable-style feed-forward; no PID, scan or fitting.
class GimbalTorqueSweepController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GimbalTorqueSweepController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/switch/left", left_);
        register_input("/remote/switch/right", right_);
        register_input("/gimbal/pitch/torque_ff", feedforward_);
        constexpr std::array axes{"yaw", "pitch"};
        for (std::size_t i = 0; i < axes.size(); ++i) {
            const auto prefix = std::string{"/gimbal/"} + axes[i];
            register_output(prefix + "/control_torque", torque_[i], 0.);
        }
        register_output("/wheel_leg/enable_request", drive_enable_, false);
        register_output("/chassis/reset_count", reset_, std::size_t{0});
        constexpr std::array parked{"/wheel_leg/left_hip_joint",  "/wheel_leg/left_knee_joint",
                                    "/wheel_leg/right_hip_joint", "/wheel_leg/right_knee_joint",
                                    "/wheel_leg/left_wheel",      "/wheel_leg/right_wheel",
                                    "/gimbal/left_friction",      "/gimbal/right_friction",
                                    "/gimbal/bullet_feeder"};
        for (std::size_t i = 0; i < parked.size(); ++i)
            register_output(std::string{parked[i]} + "/control_torque", parked_[i], 0.);
        register_output("/gimbal/torque_sweep/state", state_, 0);
        register_output("/gimbal/torque_sweep/enabled", enabled_, false);
    }

    void update() override {
        *drive_enable_ = false;
        for (auto& output : parked_)
            *output = 0.;
        if (*left_ != rmcs_msgs::Switch::MIDDLE || *right_ != rmcs_msgs::Switch::UP) {
            stop(0);
            return;
        }
        if (!*enabled_)
            ++*reset_;
        *enabled_ = true;
        *state_ = 1;
        *torque_[0] = 0.;
        *torque_[1] = *feedforward_;
    }

private:
    void stop(int state) {
        if (*enabled_)
            ++*reset_;
        *enabled_ = false;
        *state_ = state;
        for (auto& output : torque_)
            *output = 0.;
    }

    InputInterface<rmcs_msgs::Switch> left_, right_;
    InputInterface<double> feedforward_;
    std::array<OutputInterface<double>, 2> torque_;
    std::array<OutputInterface<double>, 9> parked_;
    OutputInterface<bool> drive_enable_, enabled_;
    OutputInterface<std::size_t> reset_;
    OutputInterface<int> state_;
};

} // namespace rmcs_core::controller::identification

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::GimbalTorqueSweepController, rmcs_executor::Component)

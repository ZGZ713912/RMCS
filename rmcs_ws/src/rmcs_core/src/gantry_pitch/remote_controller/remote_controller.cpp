#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::example {
class RemoteControllerExample
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    RemoteControllerExample()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
        register_input("/remote/joystick/left", remote_left_joystic_);
        register_input("/remote/joystick/right", remote_right_joystic_);
        register_input("/remote/switch/left", remote_left_switch_);
        register_input("/remote/switch/right", remote_right_switch_);

        register_input("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_imu_);
        register_input("/gimbal/row/velocity_imu", gimbal_row_velocity_imu_);

        register_output("/example/m2006_left/control_velocity", left_motor_control_velocity_);
        register_output("/example/m2006_right/control_velocity", right_motor_control_velocity_);
    }

    void update() override {
        using namespace rmcs_msgs;
        if ((*remote_left_switch_ == Switch::DOWN || *remote_left_switch_ == Switch::UNKNOWN)
            && (*remote_right_switch_ == Switch::DOWN || *remote_right_switch_ == Switch::UNKNOWN)) {
            // stop all !!
        } else {
            *left_motor_control_velocity_ = 20 * remote_left_joystic_->x() ;
            *right_motor_control_velocity_ = 20 * remote_left_joystic_->x();
            // RCLCPP_INFO(get_logger(), "%lf", *motor_control_velocity_);
        }
    }

private:
    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::Switch> remote_left_switch_;
    InputInterface<rmcs_msgs::Switch> remote_right_switch_;

    InputInterface<Eigen::Vector2d> remote_left_joystic_;
    InputInterface<Eigen::Vector2d> remote_right_joystic_;

    InputInterface<double> gimbal_row_velocity_imu_;
    InputInterface<double> gimbal_pitch_velocity_imu_;

    OutputInterface<double> left_motor_control_velocity_;
    OutputInterface<double> right_motor_control_velocity_;
};

} // namespace rmcs_core::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::example::RemoteControllerExample, rmcs_executor::Component)
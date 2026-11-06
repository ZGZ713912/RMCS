#include <cmath>
#include <limits>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::gimbal {

class PitchGravityFeedforward
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PitchGravityFeedforward()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        get_parameter_or("pitch_gravity_ff_gain", gain_, 0.);
        get_parameter_or("pitch_gravity_ff_phase", phase_, 0.);
        register_input("/gimbal/pitch/angle", pitch_angle_);
        register_output("/gimbal/pitch/torque_ff", torque_ff_, 0.);
    }

    void update() override {
        // Same two-parameter model as the deformable gimbal: gain * sin(angle - phase).
        *torque_ff_ = gain_ * std::sin(*pitch_angle_ - phase_);
    }

private:
    double gain_ = 0., phase_ = 0.;
    InputInterface<double> pitch_angle_;
    OutputInterface<double> torque_ff_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::PitchGravityFeedforward, rmcs_executor::Component)

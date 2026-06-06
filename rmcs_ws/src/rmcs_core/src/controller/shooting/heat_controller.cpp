#include <algorithm>
#include <chrono>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::shooting {

class HeatController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    static constexpr int64_t kHeatScale = 1000;

    HeatController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , heat_per_shot(get_parameter("heat_per_shot").as_int())
        , reserved_heat(get_parameter("reserved_heat").as_int()) {

        register_input("/referee/shooter/cooling", shooter_cooling_);
        register_input("/referee/shooter/heat_limit", shooter_heat_limit_);
        register_input("/referee/shooter/heat", referee_heat_);

        register_input("/gimbal/bullet_fired", bullet_fired_);

        register_output(
            "/gimbal/control_bullet_allowance/limited_by_heat", control_bullet_allowance_, 0);
    }

    void update() override {
        auto now = std::chrono::steady_clock::now();

        if (!first_update_) {
            double dt_seconds =
                std::chrono::duration<double>(now - last_update_time_).count();
            double cooling_delta_exact =
                *shooter_cooling_ * dt_seconds * 1000.0 + cooling_fraction_remainder_;
            int64_t cooling_delta = static_cast<int64_t>(cooling_delta_exact);
            cooling_fraction_remainder_ = cooling_delta_exact - cooling_delta;
            shooter_heat_ = std::max<int64_t>(0, shooter_heat_ - cooling_delta);
        }
        first_update_ = false;
        last_update_time_ = now;

        if (*bullet_fired_)
            shooter_heat_ += heat_per_shot;

        auto effective_heat = shooter_heat_;
        const auto referee_heat_scaled = *referee_heat_ * kHeatScale;
        if (*referee_heat_ >= 0 && referee_heat_scaled <= *shooter_heat_limit_) {
            effective_heat = std::max(effective_heat, referee_heat_scaled);
        }

        const auto over_limit = effective_heat > *shooter_heat_limit_;
        if (over_limit && !last_over_limit_) {
            RCLCPP_WARN(
                get_logger(),
                "Shooter heat exceeded limit: heat=%ld, limit=%ld, referee_heat=%ld, cooling=%ld",
                effective_heat, *shooter_heat_limit_, *referee_heat_, *shooter_cooling_);
        }
        last_over_limit_ = over_limit;

        *control_bullet_allowance_ = std::max<int64_t>(
            0, (*shooter_heat_limit_ - effective_heat - reserved_heat) / heat_per_shot);
    }

private:
    InputInterface<int64_t> shooter_cooling_;
    InputInterface<int64_t> shooter_heat_limit_;
    InputInterface<int64_t> referee_heat_;

    InputInterface<bool> bullet_fired_;

    const int64_t heat_per_shot;
    const int64_t reserved_heat;

    int64_t shooter_heat_ = 0;
    bool last_over_limit_ = false;

    std::chrono::steady_clock::time_point last_update_time_;
    bool first_update_ = true;
    double cooling_fraction_remainder_ = 0.0;

    OutputInterface<int64_t> control_bullet_allowance_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::shooting::HeatController, rmcs_executor::Component)

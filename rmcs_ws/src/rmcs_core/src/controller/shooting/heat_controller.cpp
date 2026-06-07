#include <algorithm>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::shooting {

class HeatController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
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
        if (*bullet_fired_)
            shooter_heat_ += heat_per_shot;

        ++cooling_tick_counter_;
        const auto cooling_delta_per_settlement = *shooter_cooling_ * kHeatScale / kCoolingRateHz;
        if (cooling_tick_counter_ >= kCoolingPeriodTicks) {
            cooling_tick_counter_ = 0;
            shooter_heat_ = std::max<int64_t>(0, shooter_heat_ - cooling_delta_per_settlement);
        }

        auto effective_heat = shooter_heat_;
        const auto referee_heat_scaled = *referee_heat_ * kHeatScale;
        const auto referee_heat_tolerance = cooling_delta_per_settlement * 2;
        const auto referee_heat_ahead =
            *referee_heat_ >= 0 && referee_heat_scaled > shooter_heat_ + referee_heat_tolerance;
        if (referee_heat_ahead && !last_referee_heat_ahead_) {
            RCLCPP_WARN(
                get_logger(),
                "Referee heat significantly ahead of local heat: local=%lld, referee=%lld, referee_scaled=%lld, tolerance=%lld, limit=%lld, cooling=%lld",
                static_cast<long long>(shooter_heat_), static_cast<long long>(*referee_heat_),
                static_cast<long long>(referee_heat_scaled),
                static_cast<long long>(referee_heat_tolerance),
                static_cast<long long>(*shooter_heat_limit_),
                static_cast<long long>(*shooter_cooling_));
        }
        last_referee_heat_ahead_ = referee_heat_ahead;

        if (*referee_heat_ >= 0 && referee_heat_scaled <= *shooter_heat_limit_) {
            effective_heat = std::max(effective_heat, referee_heat_scaled);
        }

        const auto over_limit = effective_heat > *shooter_heat_limit_;
        if (over_limit && !last_over_limit_) {
            RCLCPP_WARN(
                get_logger(),
                "Shooter heat exceeded limit: heat=%lld, limit=%lld, referee_heat=%lld, cooling=%lld",
                static_cast<long long>(effective_heat), static_cast<long long>(*shooter_heat_limit_),
                static_cast<long long>(*referee_heat_), static_cast<long long>(*shooter_cooling_));
        }
        last_over_limit_ = over_limit;

        *control_bullet_allowance_ = std::max<int64_t>(
            0, (*shooter_heat_limit_ - effective_heat - reserved_heat) / heat_per_shot);
    }

private:
    static constexpr int64_t kHeatScale = 1000;
    static constexpr int64_t kCoolingRateHz = 10;
    static constexpr int64_t kUpdateRateHz = 1000;
    static constexpr int64_t kCoolingPeriodTicks = kUpdateRateHz / kCoolingRateHz;

    InputInterface<int64_t> shooter_cooling_;
    InputInterface<int64_t> shooter_heat_limit_;
    InputInterface<int64_t> referee_heat_;

    InputInterface<bool> bullet_fired_;

    const int64_t heat_per_shot;
    const int64_t reserved_heat;

    int64_t shooter_heat_ = 0;
    bool last_over_limit_ = false;
    bool last_referee_heat_ahead_ = false;
    int64_t cooling_tick_counter_ = 0;

    OutputInterface<int64_t> control_bullet_allowance_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::shooting::HeatController, rmcs_executor::Component)

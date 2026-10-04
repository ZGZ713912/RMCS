#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "component_fixture.hpp"
#include "controller/chassis/deformable_joint_controller.cpp"
#include "hardware/device/lk_motor.hpp"
#include "identification/deformable_current_sweep.cpp"

namespace {
using Component = rmcs_executor::Component;
using rmcs_core::controller::chassis::DeformableJointController;
using rmcs_core::controller::identification::DeformableCurrentSweep;
using rmcs_core::hardware::device::CanPacket8;
using rmcs_core::hardware::device::LkMotor;

constexpr std::array<const char*, 4> kNames{"left_front", "left_back", "right_back", "right_front"};
std::string prefix(std::size_t i) { return std::string{"/chassis/"} + kNames[i] + "_joint"; }

class MockHardware : public Component {
public:
    MockHardware()
        : command_(create_partner_component<Command>("mock_command", *this)) {
        register_output("/remote/switch/left", left, rmcs_msgs::Switch::UNKNOWN);
        register_output("/remote/switch/right", right, rmcs_msgs::Switch::UNKNOWN);
        for (std::size_t i = 0; i < 4; ++i) {
            motors[i] = std::make_unique<LkMotor>(
                *this, *command_, prefix(i),
                LkMotor::Config{LkMotor::Type::kMG5010Ei36}.set_reversed());
            register_output(prefix(i) + "/physical_angle", angle[i], std::numbers::pi / 3);
            register_output(prefix(i) + "/physical_velocity", velocity[i], 0.0);
            packets[i] = LkMotor::generate_disable_command();
        }
    }
    void update() override {
        for (std::size_t i = 0; i < 4; ++i) {
            // Mock CAN state-2 feedback only. No USB or hardware plugin is constructed.
            CanPacket8 feedback{
                CanPacket8::Quarter{0x209C}, CanPacket8::Quarter{packets[i].data[2]},
                CanPacket8::Quarter{0}, CanPacket8::Quarter{0}};
            motors[i]->store_status(feedback.as_bytes());
            motors[i]->update_status();
        }
    }
    OutputInterface<rmcs_msgs::Switch> left, right;
    std::array<OutputInterface<double>, 4> angle, velocity;
    std::array<std::unique_ptr<LkMotor>, 4> motors;
    std::array<CanPacket8, 4> packets;

private:
    class Command : public Component {
    public:
        explicit Command(MockHardware& owner)
            : owner_(owner) {}
        void update() override {
            for (std::size_t i = 0; i < 4; ++i)
                owner_.packets[i] = owner_.motors[i]->generate_command();
        }

    private:
        MockHardware& owner_;
    };
    std::shared_ptr<Command> command_;
};

class SweepSink : public Component {
public:
    SweepSink() {
        register_input("/chassis/joint_control_active", active);
        register_input("/identification/stage", stage);
        for (std::size_t i = 0; i < 4; ++i) {
            register_input(prefix(i) + "/target_physical_angle", target[i]);
            register_input(prefix(i) + "/control_current_raw", command[i]);
        }
    }
    void update() override {}
    InputInterface<bool> active;
    InputInterface<double> stage;
    std::array<InputInterface<double>, 4> target, command;
};

rclcpp::NodeOptions sweep_options() {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides(
        {{"angles_deg", std::vector<double>{60.0}},
         {"amplitudes_raw", std::vector<double>{128.0}},
         {"current_to_physical_direction", std::vector<double>{-1, -1, -1, -1}},
         {"min_angle_deg", 17.0},
         {"max_angle_deg", 75.0},
         {"condition", "suspended"},
         {"settle_time_s", 0.003},
         {"chirp_duration_s", 0.03},
         {"validation_duration_s", 0.03},
         {"probe_ramp_s", 0.005},
         {"max_update_gap_s", 1.0}});
    return options;
}

rclcpp::NodeOptions servo_options(std::size_t i) {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    const auto p = prefix(i);
    options.parameter_overrides(
        {{"measurement_angle", p + "/physical_angle"},
         {"setpoint_angle", p + "/target_physical_angle"},
         {"setpoint_velocity", p + "/target_physical_velocity"},
         {"control", p + "/servo_current_raw"},
         {"output_domain", "current_raw"},
         {"controller_output_to_current_raw", 5.74635241301908},
         {"applied_current_raw", p + "/command_current_raw"},
         {"b0", -1.0}});
    return options;
}

TEST(DeformableCurrentSweep, WholeGraphPairsRunsStopsResetsAndCanRestart) {
    rclcpp::InitOptions init;
    init.set_domain_id(231); // Isolated mock test; no vehicle start/service calls.
    rclcpp::init(0, nullptr, init);
    {
        MockHardware hardware;
        Component::initializing_component_name = "mock_current_sweep";
        DeformableCurrentSweep sweep{sweep_options()};
        std::array<std::unique_ptr<DeformableJointController>, 4> servos;
        for (std::size_t i = 0; i < 4; ++i) {
            Component::initializing_component_name = std::string{"mock_"} + kNames[i];
            servos[i] = std::make_unique<DeformableJointController>(servo_options(i));
        }
        SweepSink sink;
        std::array<Component*, 7> roots{
            &hardware,       &sweep, servos[0].get(), servos[1].get(), servos[2].get(),
            servos[3].get(), &sink};
        auto components = rmcs_executor::Executor::with_partners(roots);
        const auto order = rmcs_executor::Executor::ordered(components);
        for (auto* component : order)
            component->before_updating();
        const auto tick = [&] {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            for (auto* component : order)
                component->update();
        };
        tick();
        EXPECT_FALSE(*sink.active);
        ASSERT_TRUE(sweep.request_start());
        tick();
        ASSERT_TRUE(*sink.active);
        EXPECT_FALSE(sweep.request_start());
        sweep.request_stop();
        tick();
        EXPECT_FALSE(*sink.active);
        for (const auto& packet : hardware.packets)
            EXPECT_EQ(std::bit_cast<int16_t>(packet.data[2]), 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        ASSERT_TRUE(sweep.request_start());
        tick();
        ASSERT_TRUE(*sink.active);
        *hardware.left = *hardware.right = rmcs_msgs::Switch::DOWN;
        tick();
        EXPECT_FALSE(*sink.active);
        for (const auto& packet : hardware.packets)
            EXPECT_EQ(std::bit_cast<int16_t>(packet.data[2]), 0);
        *hardware.left = *hardware.right = rmcs_msgs::Switch::UNKNOWN;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        ASSERT_TRUE(sweep.request_start());
        tick();
        ASSERT_TRUE(*sink.active);
        bool saw_probe = false;
        for (int k = 0; k < 1000 && *sink.active; ++k) {
            saw_probe |= std::abs(*sink.command[0]) > 1.0;
            tick();
        }
        EXPECT_TRUE(saw_probe);
        EXPECT_EQ(*sink.stage, 5.0);
        for (const auto& packet : hardware.packets)
            EXPECT_EQ(std::bit_cast<int16_t>(packet.data[2]), 0);
    }
    rclcpp::shutdown();
}
} // namespace

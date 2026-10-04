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
#include "hardware/device/supercap.hpp"
#include "debug/value_collector.cpp"
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
        supercap_ = std::make_unique<rmcs_core::hardware::device::Supercap>(*this, *command_, true);
        EXPECT_EQ(supercap_->generate_command().data, 0);
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
            : owner_(owner) {
            register_output("/chassis/joint_command_time_s", command_time_, 0.0);
        }
        void update() override {
            for (std::size_t i = 0; i < 4; ++i)
                owner_.packets[i] = owner_.motors[i]->generate_command();
            *command_time_ = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        }

    private:
        MockHardware& owner_;
        OutputInterface<double> command_time_;
    };
    std::shared_ptr<Command> command_;
    std::unique_ptr<rmcs_core::hardware::device::Supercap> supercap_;
};

class SweepSink : public Component {
public:
    SweepSink() {
        register_input("/chassis/joint_control_active", active);
        register_input("/identification/stage", stage);
        for (std::size_t i = 0; i < 4; ++i) {
            register_input(prefix(i) + "/target_physical_angle", target[i]);
            register_input(prefix(i) + "/control_current_raw", command[i]);
            register_input(prefix(i) + "/target_torque_proxy", target_torque[i]);
            register_input(prefix(i) + "/command_torque_proxy", command_torque[i]);
            register_input(prefix(i) + "/physical_torque_proxy", physical_torque[i]);
        }
    }
    void update() override {}
    InputInterface<bool> active;
    InputInterface<double> stage;
    std::array<InputInterface<double>, 4> target, command;
    std::array<InputInterface<double>, 4> target_torque, command_torque, physical_torque;
};

rclcpp::NodeOptions sweep_options() {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides(
        {{"min_angle_deg", 59.0},
         {"max_angle_deg", 61.0},
         {"condition", "suspended"},
         {"move_speed_rad_s", 20.0},
         {"slow_duration_s", 0.04}, {"fast_duration_s", 0.03},
         {"ultra_fast_frequency_hz", 1.0}, {"ultra_fast_duration_s", 0.03},
         {"slow_start_frequency_hz", 1.0}, {"slow_end_frequency_hz", 2.0},
         {"fast_start_frequency_hz", 2.0}, {"fast_end_frequency_hz", 4.0}});
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
        Component::initializing_component_name = "mock_collector";
        rclcpp::NodeOptions collector_options;
        collector_options.automatically_declare_parameters_from_overrides(true);
        const auto csv_path = std::filesystem::temp_directory_path() /
            ("rmcs_mock_collector_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".csv");
        const auto run_path = [&](int run) {
            return csv_path.parent_path() / (csv_path.stem().string() + "_run" + std::to_string(run) + ".csv.status.json");
        };
        collector_options.parameter_overrides({
            {"experiment_mode", true}, {"csv_path", csv_path.string()},
            {"signals", std::vector<std::string>{"/identification/host_time_s", "/identification/stage",
                "/chassis/left_front_joint/current_raw", "/chassis/left_front_joint/torque",
                "/chassis/left_front_joint/command_current_raw",
                "/chassis/left_front_joint/torque_per_current_raw",
                "/chassis/left_front_joint/command_torque_proxy"}},
            {"write_interval", 5}, {"flush_interval", 1000}});
        rmcs_core::debug::ValueCollector collector{collector_options};
        std::array<Component*, 8> roots{
            &hardware,       &sweep, servos[0].get(), servos[1].get(), servos[2].get(),
            servos[3].get(), &sink, &collector};
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
        *hardware.left = *hardware.right = rmcs_msgs::Switch::MIDDLE;
        ASSERT_TRUE(sweep.request_start());
        tick();
        ASSERT_TRUE(*sink.active);
        EXPECT_FALSE(sweep.request_start());
        sweep.request_stop();
        tick();
        EXPECT_FALSE(*sink.active);
        for (const auto& packet : hardware.packets)
            EXPECT_EQ(std::bit_cast<int16_t>(packet.data[2]), 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        EXPECT_TRUE(std::filesystem::exists(run_path(1)));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        ASSERT_TRUE(sweep.request_start());
        tick();
        ASSERT_TRUE(*sink.active);
        *hardware.left = *hardware.right = rmcs_msgs::Switch::DOWN;
        tick();
        EXPECT_FALSE(*sink.active);
        for (const auto& packet : hardware.packets)
            EXPECT_EQ(std::bit_cast<int16_t>(packet.data[2]), 0);
        *hardware.left = *hardware.right = rmcs_msgs::Switch::MIDDLE;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        ASSERT_TRUE(sweep.request_start());
        tick();
        ASSERT_TRUE(*sink.active);
        bool saw_motion_command = false;
        int run_ticks = 0;
        for (int k = 0; k < 1000 && *sink.active; ++k) {
            saw_motion_command |= std::abs(*sink.command[0]) > 1.0;
            for (std::size_t i = 0; i < 4; ++i)
                EXPECT_NEAR(*sink.target_torque[i], *sink.command[i] * 0.1740234375, 1e-9);
            tick();
            ++run_ticks;
        }
        EXPECT_TRUE(saw_motion_command);
        EXPECT_EQ(*sink.stage, 5.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::ifstream status{run_path(3)};
        std::string status_text;
        std::getline(status, status_text);
        EXPECT_TRUE(status_text.contains("\"completed\":true"));
        const auto csv = csv_path.parent_path() / (csv_path.stem().string() + "_run3.csv");
        std::ifstream samples{csv};
        std::string line;
        std::getline(samples, line);
        int rows = 0;
        bool saw_final = false;
        while (std::getline(samples, line)) {
            std::istringstream row{line};
            std::array<double, 7> values;
            for (auto& value : values) {
                std::string cell;
                std::getline(row, cell, ',');
                value = std::stod(cell);
            }
            EXPECT_NEAR(values[6], values[4] * values[5], 1e-10);
            saw_final |= values[1] == 5.0;
            ++rows;
        }
        EXPECT_TRUE(saw_final);
        EXPECT_LE(rows, run_ticks / 5 + 3);
        EXPECT_GT(rows, 2);
        for (const auto& packet : hardware.packets)
            EXPECT_EQ(std::bit_cast<int16_t>(packet.data[2]), 0);
        *hardware.left = *hardware.right = rmcs_msgs::Switch::DOWN;
        tick();
        *hardware.left = *hardware.right = rmcs_msgs::Switch::MIDDLE;
        tick();
        EXPECT_TRUE(*sink.active);
        for (int k = 0; k < 1000 && *sink.active; ++k)
            tick();
        EXPECT_EQ(*sink.stage, 5.0);
        for (int k = 0; k < 10; ++k)
            tick();
        EXPECT_FALSE(*sink.active); // Holding middle does not restart completed run.
        *hardware.left = *hardware.right = rmcs_msgs::Switch::DOWN;
        tick();
        *hardware.left = *hardware.right = rmcs_msgs::Switch::MIDDLE;
        tick();
        EXPECT_TRUE(*sink.active);
        sweep.request_stop();
        tick();
    }
    rclcpp::shutdown();
}
} // namespace

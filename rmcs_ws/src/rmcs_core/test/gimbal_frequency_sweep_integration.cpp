// Load the deployed YAML/controller/PIDs/CSV with memory-only hardware ports.
// Never instantiate a hardware plugin or open CAN/USB devices in this test.
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_msgs/switch.hpp>

#include "executor.hpp"

namespace {
using Component = rmcs_executor::Component;
using rmcs_msgs::Switch;
using namespace std::chrono_literals;
constexpr double yaw_center = -.01, pitch_center = .05;
constexpr std::array parked{
    "/wheel_leg/left_hip_joint",   "/wheel_leg/left_knee_joint", "/wheel_leg/right_hip_joint",
    "/wheel_leg/right_knee_joint", "/wheel_leg/left_wheel",      "/wheel_leg/right_wheel",
    "/gimbal/left_friction",       "/gimbal/right_friction",     "/gimbal/bullet_feeder"};

class Feedback final : public Component {
public:
    explicit Feedback(std::atomic<int>& phase)
        : phase_(phase) {
        register_output("/remote/switch/left", left_, Switch::DOWN);
        register_output("/remote/switch/right", right_, Switch::DOWN);
        register_output("/wheel_leg/dr16_fresh", fresh_, true);
        register_output("/wheel_leg/dm_control_ready", dm_ready_, false);
        register_output("/gimbal/imu/last_steady_ns", imu_ns_, std::uint64_t{0});
        for (const auto* axis : {"yaw", "pitch"}) {
            const auto prefix = std::string{"/gimbal/"} + axis;
            for (const auto* field : {"angle", "velocity", "velocity_imu", "torque", "temperature"})
                register_output(prefix + "/" + field, values_[prefix + "/" + field], 0.);
            register_output(prefix + "/feedback_steady_ns", stamps_[prefix], std::uint64_t{0});
        }
    }

    void update() override {
        const int phase = phase_.load();
        *left_ = phase == 0 ? Switch::DOWN : phase == 1 || phase == 3 ? Switch::MIDDLE : Switch::UP;
        *right_ = phase == 0               ? Switch::DOWN
                : phase == 3 || phase == 4 ? Switch::UP
                                           : Switch::MIDDLE;
        if (phase == 13)
            *left_ = Switch::UNKNOWN;
        *fresh_ = phase != 6;
        const std::uint64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count();
        *imu_ns_ = phase == 8 ? now - 500'000'000 : now;
        for (const auto* axis : {"yaw", "pitch"}) {
            const auto prefix = std::string{"/gimbal/"} + axis;
            *values_[prefix + "/angle"] =
                axis == std::string{"yaw"} ? 2. * std::numbers::pi + yaw_center : pitch_center;
            *values_[prefix + "/velocity_imu"] = 0.;
            *stamps_[prefix] = now;
        }
        if (phase == 7)
            *stamps_["/gimbal/yaw"] = now - 500'000'000;
        if (phase == 9)
            *values_["/gimbal/pitch/angle"] = std::numeric_limits<double>::quiet_NaN();
        if (phase == 10)
            *values_["/gimbal/yaw/angle"] += .5;
        if (phase == 11)
            *values_["/gimbal/pitch/angle"] = .7;
        if (phase == 12)
            *stamps_["/gimbal/yaw"] = 0;
        if (phase == 14)
            *values_["/gimbal/pitch/velocity_imu"] = std::numeric_limits<double>::infinity();
        if (phase == 15)
            *stamps_["/gimbal/pitch"] = now - 500'000'000;
        if (phase == 16)
            *stamps_["/gimbal/yaw"] = now + 10'000'000;
    }

private:
    std::atomic<int>& phase_;
    OutputInterface<Switch> left_, right_;
    OutputInterface<bool> fresh_, dm_ready_;
    OutputInterface<std::uint64_t> imu_ns_;
    std::map<std::string, OutputInterface<double>> values_;
    std::map<std::string, OutputInterface<std::uint64_t>> stamps_;
};

class Observe final : public Component {
public:
    Observe() {
        register_input("/gimbal/sweep/state", state_);
        register_input("/gimbal/sweep/axis", axis_);
        register_input("/gimbal/sweep/enabled", enabled_);
        register_input("/gimbal/sweep/completed", completed_);
        register_input("/gimbal/sweep/frequency_hz", frequency_);
        register_input("/wheel_leg/enable_request", drive_enable_);
        register_input("/chassis/reset_count", resets_);
        for (std::size_t i = 0; i < parked.size(); ++i)
            register_input(std::string{parked[i]} + "/control_torque", parked_torque_[i]);
        constexpr std::array axes{"yaw", "pitch"};
        for (std::size_t i = 0; i < axes.size(); ++i) {
            const auto prefix = std::string{"/gimbal/"} + axes[i];
            register_input(prefix + "/sweep/reference", reference_[i]);
            register_input(prefix + "/sweep/reference_velocity", reference_velocity_[i]);
            register_input(prefix + "/control_torque", torque_[i]);
        }
    }

    void update() override {
        if (*drive_enable_)
            ++violations;
        for (const auto& torque : parked_torque_)
            if (*torque != 0.)
                ++violations;
        constexpr std::array center{yaw_center, pitch_center};
        constexpr std::array amplitude{.05235987755982989, .03490658503988659};
        for (std::size_t i = 0; i < torque_.size(); ++i) {
            const double command = *torque_[i];
            if ((std::isfinite(command) && std::abs(command) > 2.)
                || (!*enabled_ && std::isfinite(command) && command != 0.))
                ++violations;
            if (*enabled_) {
                if (!std::isfinite(command) || !std::isfinite(*reference_velocity_[i])
                    || std::abs(*reference_[i] - center[i]) > amplitude[i] + 1e-10)
                    ++violations;
                if (*axis_ != static_cast<int>(i + 1)
                    && std::abs(*reference_[i] - center[i]) > 1e-10)
                    ++violations;
            }
        }
        enabled = *enabled_;
        completed = *completed_;
        state = *state_;
        axis = *axis_;
        frequency = *frequency_;
        resets = *resets_;
        yaw_reference = *reference_[0];
        pitch_reference = *reference_[1];
        ++ticks;
    }

    std::atomic<int> state{0}, axis{0}, violations{0};
    std::atomic<bool> enabled{false}, completed{false};
    std::atomic<double> frequency{0.}, yaw_reference{0.}, pitch_reference{0.};
    std::atomic<std::size_t> resets{0}, ticks{0};

private:
    InputInterface<int> state_, axis_;
    InputInterface<bool> enabled_, completed_, drive_enable_;
    InputInterface<double> frequency_;
    InputInterface<std::size_t> resets_;
    std::array<InputInterface<double>, 9> parked_torque_;
    std::array<InputInterface<double>, 2> reference_, reference_velocity_, torque_;
};

bool await(const std::function<bool()>& predicate, std::chrono::milliseconds timeout = 1000ms) {
    const auto until = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < until) {
        if (predicate())
            return true;
        std::this_thread::sleep_for(1ms);
    }
    return predicate();
}

TEST(GimbalSweepIntegration, DeployedDoublePidGraphSweepStopResetAndParkedOutputs) {
    std::atomic<int> phase{0};
    pluginlib::ClassLoader<Component> loader{"rmcs_executor", "rmcs_executor::Component"};
    EXPECT_TRUE(loader.isClassAvailable("rmcs_core::hardware::WheelLegDualBoard"));
    Component::initializing_component_name = "feedback";
    auto feedback = std::make_shared<Feedback>(phase);
    Component::initializing_component_name = "observe";
    auto observe = std::make_shared<Observe>();
    rclcpp::executors::SingleThreadedExecutor ros;
    {
        rmcs_executor::Executor executor{"rmcs_executor", ros};
        struct Shutdown {
            ~Shutdown() { rclcpp::shutdown(); }
        } shutdown;
        executor.add_component(feedback);
        constexpr std::array names{
            "gimbal_sweep",
            "yaw_angle_pid_controller",
            "yaw_velocity_pid_controller",
            "pitch_angle_pid_controller",
            "pitch_velocity_pid_controller",
            "value_collector"};
        constexpr std::array types{
            "rmcs_core::controller::identification::GimbalFrequencySweepController",
            "rmcs_core::controller::pid::ErrorPidController",
            "rmcs_core::controller::pid::PidController",
            "rmcs_core::controller::pid::ErrorPidController",
            "rmcs_core::controller::pid::PidController",
            "rmcs_core::debug::ValueCollector"};
        for (std::size_t i = 0; i < names.size(); ++i) {
            Component::initializing_component_name = names[i];
            executor.add_component(loader.createSharedInstance(types[i]));
        }
        executor.add_component(observe);
        ASSERT_NO_THROW(executor.start());
        ASSERT_TRUE(await([&] { return observe->ticks > 10; }));
        EXPECT_FALSE(observe->enabled);
        phase = 1;
        ASSERT_TRUE(await([&] { return observe->enabled && observe->state == 1; }));
        EXPECT_NEAR(observe->yaw_reference, yaw_center, 1e-10);
        EXPECT_NEAR(observe->pitch_reference, pitch_center, 1e-10);
        phase = 2;
        ASSERT_TRUE(await([&] { return observe->state == 2 && observe->axis == 1; }));
        EXPECT_GE(observe->frequency, .2);
        EXPECT_LE(observe->frequency, 5.);
        ASSERT_TRUE(await([&] { return observe->completed && observe->state == 3; }));
        std::this_thread::sleep_for(260ms);
        EXPECT_TRUE(observe->completed); // Held mode cannot restart a completed sweep.
        EXPECT_NEAR(observe->yaw_reference, yaw_center, 1e-10);
        phase = 3;
        ASSERT_TRUE(await([&] { return observe->state == 2 && observe->axis == 2; }));
        ASSERT_TRUE(await([&] { return observe->completed.load(); }));
        phase = 4;
        ASSERT_TRUE(await([&] { return observe->state == 2 && observe->axis == 1; }));
        ASSERT_TRUE(await([&] { return observe->state == 2 && observe->axis == 2; }));
        ASSERT_TRUE(await([&] { return observe->completed.load(); }));
        const auto resets = observe->resets.load();
        phase = 0;
        ASSERT_TRUE(await([&] { return !observe->enabled && observe->state == 0; }));
        EXPECT_GT(observe->resets, resets);
        for (const int fault : {6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}) {
            phase = 1;
            ASSERT_TRUE(await([&] { return observe->enabled && observe->state == 1; }));
            phase = 2;
            ASSERT_TRUE(await([&] { return observe->state == 2; }));
            phase = fault;
            const int expected = fault == 6 || fault == 13 ? 0 : fault == 10 || fault == 11 ? 5 : 4;
            ASSERT_TRUE(await([&] { return !observe->enabled && observe->state == expected; }));
            if (expected != 0) {
                phase = 2; // Healthy feedback alone cannot restart the same selected mode.
                std::this_thread::sleep_for(15ms);
                EXPECT_FALSE(observe->enabled);
            }
        }
        phase = 0;
        ASSERT_TRUE(await([&] { return observe->state == 0; }));
        EXPECT_EQ(observe->violations, 0);
    }
    std::ifstream csv{RMCS_GIMBAL_CSV};
    ASSERT_TRUE(csv.good());
    std::string header, row;
    ASSERT_TRUE(std::getline(csv, header));
    EXPECT_NE(header.find("/gimbal/pitch/control_velocity"), std::string::npos);
    EXPECT_NE(header.find("/gimbal/yaw/control_torque"), std::string::npos);
    EXPECT_NE(header.find("/gimbal/imu/last_steady_ns"), std::string::npos);
    std::size_t rows = 0;
    while (std::getline(csv, row))
        ++rows;
    EXPECT_EQ(rows, observe->ticks.load());
    EXPECT_GT(rows, 1000u);
}
} // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    const std::vector<const char*> args{
        "gimbal_frequency_sweep_integration",
        "--ros-args",
        "--params-file",
        RMCS_GIMBAL_PROFILE,
        "-p",
        "gimbal_sweep:duration_s:=0.2",
        "-p",
        "gimbal_sweep:ramp_s:=0.02",
        "-p",
        "gimbal_sweep:settle_s:=0.02",
        "-p",
        "value_collector:csv_path:=" RMCS_GIMBAL_CSV};
    rclcpp::init(static_cast<int>(args.size()), args.data());
    const int result = RUN_ALL_TESTS();
    if (rclcpp::ok())
        rclcpp::shutdown();
    return result;
}

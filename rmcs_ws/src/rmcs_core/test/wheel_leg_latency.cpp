#include <array>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "executor.hpp"
#include "hardware/device/actuator_latency_trace.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "identification/wheel_leg_identification_axes.hpp"
#include "identification/wheel_leg_latency_plan.hpp"
#include "identification/wheel_leg_sweep_session.hpp"

namespace {
using namespace rmcs_core::controller::identification;
using Trace = rmcs_core::hardware::device::ActuatorLatencyTrace;

class LatencyFeedback final : public rmcs_executor::Component {
public:
    LatencyFeedback() {
        register_output("/remote/switch/left", left_, rmcs_msgs::Switch::MIDDLE);
        register_output("/remote/switch/right", right_, rmcs_msgs::Switch::MIDDLE);
        register_output("/wheel_leg/dr16_fresh", remote_, true);
        register_output("/wheel_leg/dm_control_ready", ready_, true);
        for (int i = 0; i < 6; ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kAxisNames[i];
            register_output(prefix + "/angle", angle_[i], .1);
            register_output(prefix + "/velocity", velocity_[i], 0.);
            register_output(prefix + "/max_torque", caps_[i], i < 4 ? 40. : 4.5);
            register_output(prefix + "/feedback_steady_ns", stamps_[i], std::uint64_t{0});
        }
    }
    void update() override {
        *left_ = left.load();
        *right_ = right.load();
        *remote_ = remote.load();
        for (auto& stamp : stamps_)
            *stamp = Trace::now_ns();
    }
    std::atomic<rmcs_msgs::Switch> left{rmcs_msgs::Switch::MIDDLE},
        right{rmcs_msgs::Switch::MIDDLE};
    std::atomic<bool> remote{true};

private:
    OutputInterface<rmcs_msgs::Switch> left_, right_;
    OutputInterface<bool> remote_, ready_;
    std::array<OutputInterface<double>, 6> angle_, velocity_, caps_;
    std::array<OutputInterface<std::uint64_t>, 6> stamps_;
};

class LatencyCommand final : public rmcs_executor::Component {
public:
    LatencyCommand() {
        register_input("/wheel_leg/enable_request", enable_);
        register_input("/wheel_leg/latency/active", active_);
        register_input("/wheel_leg/latency/fault", fault_);
        for (int i = 0; i < 6; ++i)
            register_input(
                std::string{"/wheel_leg/"} + kAxisNames[i] + "/control_torque", torque_[i]);
    }
    void update() override {
        enabled.store(*enable_);
        double total = 0.;
        for (const auto& effort : torque_)
            total += std::abs(*effort);
        effort.store(total);
        fault.store(*fault_);
        if (*active_ && total > .01)
            ++probe_ticks;
    }
    std::atomic<bool> enabled{false};
    std::atomic<double> effort{0.};
    std::atomic<int> probe_ticks{0}, fault{0};

private:
    InputInterface<bool> enable_, active_;
    InputInterface<int> fault_;
    std::array<InputInterface<double>, 6> torque_;
};

class LatencyObservation final : public rmcs_executor::Component {
public:
    LatencyObservation() {
        register_output("/wheel_leg/rl/input/frame", frame_, std::vector<double>(35, 0.));
        register_output("/wheel_leg/rl/input/frame_sequence", sequence_, std::size_t{0});
        register_output("/wheel_leg/rl/input/frame_steady_ns", stamp_, std::uint64_t{0});
        register_output("/wheel_leg/rl/input/enable", enable_, true);
        register_output("/chassis/reset_count", reset_, std::size_t{0});
        (*frame_)[5] = -1.;
    }
    void update() override {
        if (tick_++ % 20 == 0) {
            ++*sequence_;
            *stamp_ = Trace::now_ns();
        }
    }

private:
    OutputInterface<std::vector<double>> frame_;
    OutputInterface<std::size_t> sequence_, reset_;
    OutputInterface<std::uint64_t> stamp_;
    OutputInterface<bool> enable_;
    std::size_t tick_ = 0;
};

class LatencyActionMonitor final : public rmcs_executor::Component {
public:
    LatencyActionMonitor() {
        register_input("/wheel_leg/rl/valid", valid_);
        register_input("/wheel_leg/rl/action_seq", sequence_);
        constexpr std::array names{"source_frame_ns",  "queued_ns",       "inference_start_ns",
                                   "inference_end_ns", "result_ready_ns", "first_publish_ns"};
        for (std::size_t i = 0; i < names.size(); ++i)
            register_input(std::string{"/wheel_leg/rl/latency/"} + names[i], stamps_[i]);
    }
    void update() override {
        if (*valid_ <= 0. || *sequence_ == previous_)
            return;
        previous_ = *sequence_;
        for (std::size_t i = 1; i < stamps_.size(); ++i)
            if (!*stamps_[i - 1] || *stamps_[i] < *stamps_[i - 1])
                ordered.store(false);
        ++received;
    }
    std::atomic<bool> ordered{true};
    std::atomic<int> received{0};

private:
    InputInterface<double> valid_;
    InputInterface<std::size_t> sequence_;
    std::array<InputInterface<std::uint64_t>, 6> stamps_;
    std::size_t previous_ = 0;
};

TEST(WheelLegLatency, ExcitationSeparatesAmplitudeLevelsAndHoldout) {
    WheelLegLatencyPlan plan(1., .01, 2.);
    EXPECT_FALSE(plan.at(1.).active);
    for (int stage = 0; stage < 6; ++stage) {
        double energy = 0;
        for (int tick = 0; tick < 2000; ++tick) {
            const auto sample = plan.at(2.5 + 3 * stage + tick * .001);
            EXPECT_EQ(sample.stage, stage);
            EXPECT_EQ(sample.validation, stage >= 3);
            EXPECT_LE(std::abs(sample.probe), (std::array{.5, .75, 1.}[stage % 3]));
            energy += sample.probe * sample.probe;
        }
        EXPECT_GT(energy, 200.);
    }
    int differences = 0;
    for (int i = 0; i < 100; ++i)
        differences += plan.at(3.51 + i * .01).probe != plan.at(12.51 + i * .01).probe;
    EXPECT_GT(differences, 20);
    EXPECT_TRUE(plan.at(20.).completed);
}

TEST(WheelLegLatency, ParameterValidation) {
    EXPECT_THROW(WheelLegLatencyPlan(0., .01, 2.), std::invalid_argument);
    EXPECT_THROW(WheelLegLatencyPlan(1., .001, 2.), std::invalid_argument);
    EXPECT_THROW(WheelLegLatencyPlan(1., .01, 1.), std::invalid_argument);
}

TEST(WheelLegLatency, CallbackTorqueDecodeMatchesCodecWithoutMutatingFeedback) {
    using namespace rmcs_core::hardware::device;
    rmcs_executor::Component::initializing_component_name = "latency_codec_fixture";
    LatencyFeedback fixture;
    DmMotor dm{
        fixture, fixture, "/codec_dm", DmMotor::Config{DmMotor::Type::kDM8009}.set_reversed()};
    DjiMotor wheel{
        fixture, fixture, "/codec_wheel",
        DjiMotor::Config{DjiMotor::Type::kM3508, 1}.set_reduction_ratio(15.62).set_reversed()};
    for (int value : {0, 512, 2048, 3500, 4095}) {
        CanPacket8 packet{std::uint64_t{0}};
        auto bytes = packet.as_writable_bytes();
        bytes[0] = std::byte{0x11};
        bytes[4] = static_cast<std::byte>((value >> 8) & 0x0F);
        bytes[5] = static_cast<std::byte>(value & 0xFF);
        const auto previous = dm.torque();
        const auto decoded = dm.feedback_frame_torque(packet);
        EXPECT_DOUBLE_EQ(previous, dm.torque());
        dm.update_status(packet);
        EXPECT_DOUBLE_EQ(decoded, dm.torque());
        const auto wheel_previous = wheel.torque();
        const auto wheel_decoded = wheel.feedback_frame_torque(packet);
        EXPECT_DOUBLE_EQ(wheel_previous, wheel.torque());
        wheel.update_status(packet);
        EXPECT_DOUBLE_EQ(wheel_decoded, wheel.torque());
    }
}

TEST(WheelLegLatency, CompletedSessionNeedsDownResetAndRemoteLossStops) {
    using rmcs_msgs::Switch;
    WheelLegSweepSession session{true, true, 1};
    EXPECT_TRUE(session.update(0, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone));
    session.complete();
    EXPECT_FALSE(session.update(1, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone));
    EXPECT_FALSE(session.update(2, false, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone));
    EXPECT_FALSE(session.enabled());
    session.update(3, true, Switch::DOWN, Switch::DOWN, SweepStop::kNone);
    EXPECT_TRUE(session.update(4, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone));
}

TEST(WheelLegLatency, ConcurrentEventTracePreservesIntegersAndAccountsForOverflow) {
    const auto path = std::filesystem::temp_directory_path() / "rmcs_latency_concurrency.csv";
    std::uint64_t dropped;
    {
        Trace trace{path.string(), 256};
        std::vector<std::jthread> producers;
        for (int axis = 0; axis < 3; ++axis)
            producers.emplace_back([&, axis] {
                for (std::uint64_t i = 1; i <= 10000; ++i)
                    trace.push(
                        {.event = 2,
                         .axis = axis,
                         .sequence = i,
                         .steady_ns = 576460752303423495ULL + i,
                         .value = .1});
            });
        for (auto& producer : producers)
            producer.join();
        dropped = trace.dropped();
        EXPECT_FALSE(trace.failed());
    }
    std::ifstream stream{path};
    std::string line;
    std::getline(stream, line);
    std::uint64_t count = 0;
    bool closed = false;
    while (std::getline(stream, line)) {
        if (line.starts_with("9,")) {
            closed = true;
            EXPECT_TRUE(line.starts_with("9,-1," + std::to_string(dropped) + ','));
        } else {
            ++count;
            EXPECT_NE(line.find("576460752303"), std::string::npos);
        }
    }
    EXPECT_EQ(count + dropped, 30000);
    EXPECT_TRUE(closed);
    std::filesystem::remove(path);
}

TEST(WheelLegLatency, EventTraceNeverOverwritesAnExistingCapture) {
    const auto path = std::filesystem::temp_directory_path()
                    / ("rmcs_latency_exclusive_" + std::to_string(Trace::now_ns()) + ".csv");
    {
        std::ofstream existing{path};
        existing << "keep original data\n";
    }
    EXPECT_THROW(Trace{path.string()}, std::invalid_argument);
    std::ifstream existing{path};
    std::string line;
    std::getline(existing, line);
    EXPECT_EQ(line, "keep original data");
    std::filesystem::remove(path);
}

TEST(WheelLegLatency, StandalonePluginConstructsForWheelsAndEachLegAxis) {
    for (const std::string axis :
         {"left_wheel", "right_wheel", "left_hip_joint", "left_knee_joint", "right_hip_joint",
          "right_knee_joint"}) {
        const std::vector<std::string> arguments{
            "latency_test",
            "--ros-args",
            "--params-file",
            std::string{RMCS_BRINGUP_CONFIG_DIR} + "/wheel-leg-latency.yaml",
            "-p",
            "latency_probe:axis:=" + axis};
        std::vector<const char*> argv;
        for (const auto& argument : arguments)
            argv.push_back(argument.c_str());
        rclcpp::init(argv.size(), argv.data());
        {
            pluginlib::ClassLoader<rmcs_executor::Component> loader{
                "rmcs_executor", "rmcs_executor::Component"};
            rmcs_executor::Component::initializing_component_name = "latency_probe";
            EXPECT_NO_THROW(loader.createSharedInstance(
                "rmcs_core::controller::identification::WheelLegLatencyProbeController"));
        }
        rclcpp::shutdown();
    }
}

TEST(WheelLegLatency, NativeExecutorPairsAndRunsProbeResetAndRemoteLossWithoutHardware) {
    for (const std::string axis : {"left_wheel", "right_knee_joint"}) {
        const std::vector<std::string> arguments{
            "latency_loop_test",
            "--ros-args",
            "--params-file",
            std::string{RMCS_BRINGUP_CONFIG_DIR} + "/wheel-leg-latency.yaml",
            "-p",
            "latency_probe:axis:=" + axis};
        std::vector<const char*> argv;
        for (const auto& argument : arguments)
            argv.push_back(argument.c_str());
        rclcpp::init(argv.size(), argv.data());
        {
            rclcpp::executors::SingleThreadedExecutor ros_executor;
            pluginlib::ClassLoader<rmcs_executor::Component> loader{
                "rmcs_executor", "rmcs_executor::Component"};
            rmcs_executor::Executor executor{"rmcs_executor", ros_executor};
            rmcs_executor::Component::initializing_component_name = "feedback_fixture";
            auto source = std::make_shared<LatencyFeedback>();
            rmcs_executor::Component::initializing_component_name = "latency_probe";
            auto probe = loader.createSharedInstance(
                "rmcs_core::controller::identification::WheelLegLatencyProbeController");
            rmcs_executor::Component::initializing_component_name = "command_fixture";
            auto command = std::make_shared<LatencyCommand>();
            executor.add_component(source);
            executor.add_component(probe);
            executor.add_component(command);
            executor.start();
            std::this_thread::sleep_for(std::chrono::milliseconds{2800});
            EXPECT_TRUE(rclcpp::ok());
            EXPECT_TRUE(command->enabled.load());
            EXPECT_GT(command->probe_ticks.load(), 0);
            EXPECT_EQ(command->fault.load(), 0);
            source->left = source->right = rmcs_msgs::Switch::DOWN;
            std::this_thread::sleep_for(std::chrono::milliseconds{20});
            EXPECT_FALSE(command->enabled.load());
            EXPECT_DOUBLE_EQ(command->effort.load(), 0.);
            source->left = source->right = rmcs_msgs::Switch::MIDDLE;
            std::this_thread::sleep_for(std::chrono::milliseconds{20});
            EXPECT_TRUE(command->enabled.load());
            source->remote = false;
            std::this_thread::sleep_for(std::chrono::milliseconds{20});
            EXPECT_FALSE(command->enabled.load());
            EXPECT_DOUBLE_EQ(command->effort.load(), 0.);
            rclcpp::shutdown();
        }
    }
}

TEST(WheelLegLatency, NativeDeploymentShadowPreservesSourceAndInferenceTimestampOrder) {
    const std::vector<std::string> arguments{
        "deployment_latency_test", "--ros-args", "--params-file",
        std::string{RMCS_BRINGUP_CONFIG_DIR} + "/wheel-leg-infantry-wheelbipe.yaml"};
    std::vector<const char*> argv;
    for (const auto& argument : arguments)
        argv.push_back(argument.c_str());
    rclcpp::init(argv.size(), argv.data());
    {
        rclcpp::executors::SingleThreadedExecutor ros_executor;
        pluginlib::ClassLoader<rmcs_executor::Component> loader{
            "rmcs_executor", "rmcs_executor::Component"};
        if (!loader.isClassAvailable("rmcs_rl::RlBridge")) {
            rclcpp::shutdown();
            GTEST_SKIP() << "rmcs_rl is not installed in this workspace";
        }
        rmcs_executor::Executor executor{"rmcs_executor", ros_executor};
        rmcs_executor::Component::initializing_component_name = "observation_fixture";
        auto source = std::make_shared<LatencyObservation>();
        rmcs_executor::Component::initializing_component_name = "rl_bridge";
        auto bridge = loader.createSharedInstance("rmcs_rl::RlBridge");
        rmcs_executor::Component::initializing_component_name = "action_monitor";
        auto monitor = std::make_shared<LatencyActionMonitor>();
        executor.add_component(source);
        executor.add_component(bridge);
        executor.add_component(monitor);
        executor.start();
        std::this_thread::sleep_for(std::chrono::milliseconds{900});
        EXPECT_TRUE(rclcpp::ok());
        EXPECT_GT(monitor->received.load(), 25);
        EXPECT_TRUE(monitor->ordered.load());
        rclcpp::shutdown();
    }
}
} // namespace

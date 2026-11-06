// Loads the deployed open-loop torque profile against a simulated gimbal.
// No hardware plugin, USB or CAN device is instantiated here.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <string>
#include <string_view>
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

constexpr std::string_view experiment{RMCS_TORQUE_EXPERIMENT};

// Simulated gimbal: J * domega = tau + gravity(theta) - c * omega.
// Gravity is an external torque, so the correct holding command is its negative.
constexpr double kJointInertia = .02;
constexpr double kJointDamping = .6;
constexpr double kGravityCos = .5;
constexpr double kGravitySin = .1;
constexpr double kGravityConst = .02;
constexpr double kPitchCenter = .05;

struct Model {
    double yaw_angle = 0., yaw_rate = 0.;
    double pitch_angle = kPitchCenter, pitch_rate = 0.;
    double yaw_torque = 0., pitch_torque = 0.;
    std::uint64_t last_ns = 0;
};

// Feedback-only component: publishes the state and never consumes the controller
// output, so the executor sees a one-way dependency instead of a cycle.
class PlantState final : public Component {
public:
    PlantState(std::shared_ptr<Model> model, std::atomic<int>& phase, std::atomic<bool>& lost)
        : model_(std::move(model))
        , phase_(phase)
        , lost_(lost) {
        register_output("/remote/switch/left", left_, Switch::DOWN);
        register_output("/remote/switch/right", right_, Switch::DOWN);
        register_output("/wheel_leg/dr16_fresh", fresh_, true);
        register_output("/wheel_leg/dm_control_ready", dm_ready_, false);
        register_output("/gimbal/imu/last_steady_ns", imu_ns_, std::uint64_t{0});
        for (const auto* axis : {"yaw", "pitch"}) {
            const auto prefix = std::string{"/gimbal/"} + axis;
            register_output(prefix + "/angle", angle_[prefix], 0.);
            register_output(prefix + "/velocity", velocity_[prefix], 0.);
            register_output(prefix + "/velocity_imu", imu_velocity_[prefix], 0.);
            register_output(prefix + "/torque", measured_[prefix], 0.);
            register_output(prefix + "/temperature", temperature_[prefix], 30.);
            register_output(prefix + "/feedback_steady_ns", stamps_[prefix], std::uint64_t{0});
        }
    }

    void update() override {
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count()
                      - 1'000'000;
        const auto phase = phase_.load();
        *left_ = phase == 1 || phase == 3 ? Switch::MIDDLE : Switch::UP;
        *right_ = phase == 3 || phase == 4 ? Switch::UP : Switch::MIDDLE;
        if (phase == 0) {
            *left_ = Switch::DOWN;
            *right_ = Switch::DOWN;
        }
        if (!lost_.load()) {
            *imu_ns_ = ns;
            for (auto& [name, stamp] : stamps_)
                *stamp = ns;
        }
        *angle_["/gimbal/yaw"] = model_->yaw_angle;
        *angle_["/gimbal/pitch"] = model_->pitch_angle;
        *velocity_["/gimbal/yaw"] = model_->yaw_rate;
        *velocity_["/gimbal/pitch"] = model_->pitch_rate;
        *imu_velocity_["/gimbal/yaw"] = model_->yaw_rate;
        *imu_velocity_["/gimbal/pitch"] = model_->pitch_rate;
        *measured_["/gimbal/yaw"] = model_->yaw_torque;
        *measured_["/gimbal/pitch"] = model_->pitch_torque;
    }

private:
    std::shared_ptr<Model> model_;
    std::atomic<int>& phase_;
    std::atomic<bool>& lost_;
    OutputInterface<Switch> left_, right_;
    OutputInterface<bool> fresh_, dm_ready_;
    OutputInterface<std::uint64_t> imu_ns_;
    std::map<std::string, OutputInterface<double>> angle_, velocity_, imu_velocity_, measured_,
        temperature_;
    std::map<std::string, OutputInterface<std::uint64_t>> stamps_;
};

// Consumes the controller output and integrates the plant; publishes no interface.
class PlantCommand final : public Component {
public:
    explicit PlantCommand(std::shared_ptr<Model> model)
        : model_(std::move(model)) {
        register_input("/gimbal/yaw/control_torque", yaw_torque_);
        register_input("/gimbal/pitch/control_torque", pitch_torque_);
    }

    void update() override {
        const auto now = std::chrono::steady_clock::now();
        const std::uint64_t ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        double dt = model_->last_ns == 0 ? 1e-3 : double(ns - model_->last_ns) * 1e-9;
        dt = std::clamp(dt, 1e-4, 2e-3);
        model_->last_ns = ns;

        const double yaw_torque = std::isfinite(*yaw_torque_) ? *yaw_torque_ : 0.;
        const double pitch_torque = std::isfinite(*pitch_torque_) ? *pitch_torque_ : 0.;
        model_->yaw_torque = yaw_torque;
        model_->pitch_torque = pitch_torque;

        const double yaw_acceleration =
            (yaw_torque - kJointDamping * model_->yaw_rate) / kJointInertia;
        model_->yaw_rate += yaw_acceleration * dt;
        model_->yaw_angle = wrap(model_->yaw_angle + model_->yaw_rate * dt);

        const double gravity = kGravityCos * std::cos(model_->pitch_angle)
                             + kGravitySin * std::sin(model_->pitch_angle) + kGravityConst;
        const double pitch_acceleration =
            (pitch_torque + gravity - kJointDamping * model_->pitch_rate) / kJointInertia;
        model_->pitch_rate += pitch_acceleration * dt;
        model_->pitch_angle = wrap(model_->pitch_angle + model_->pitch_rate * dt);
    }

private:
    static double wrap(double angle) { return std::remainder(angle, 2. * std::numbers::pi); }

    std::shared_ptr<Model> model_;
    InputInterface<double> yaw_torque_, pitch_torque_;
};

class Observe final : public Component {
public:
    Observe() {
        register_input("/gimbal/torque_sweep/state", state_);
        register_input("/gimbal/torque_sweep/experiment", experiment_);
        register_input("/gimbal/torque_sweep/axis", axis_);
        register_input("/gimbal/torque_sweep/step", step_);
        register_input("/gimbal/torque_sweep/enabled", enabled_);
        register_input("/gimbal/torque_sweep/completed", completed_);
        register_input("/gimbal/torque_sweep/frequency_hz", frequency_);
        register_input("/gimbal/pitch/torque_sweep/gravity", gravity_);
        register_input("/gimbal/pitch/torque_sweep/fit_cos", fit_[0]);
        register_input("/gimbal/pitch/torque_sweep/fit_sin", fit_[1]);
        register_input("/gimbal/pitch/torque_sweep/fit_const", fit_[2]);
        register_input("/gimbal/pitch/torque_sweep/fit_residual", residual_);
        register_input("/gimbal/yaw/control_torque", control_[0]);
        register_input("/gimbal/pitch/control_torque", control_[1]);
        register_input("/gimbal/yaw/torque_sweep/excitation", excitation_[0]);
        register_input("/gimbal/pitch/torque_sweep/excitation", excitation_[1]);
        register_input("/gimbal/yaw/angle", angle_[0]);
        register_input("/gimbal/pitch/angle", angle_[1]);
        register_input("/wheel_leg/enable_request", drive_enable_);
        register_input("/chassis/reset_count", resets_);
        for (std::size_t i = 0; i < parked.size(); ++i)
            register_input(std::string{parked[i]} + "/control_torque", parked_torque_[i]);
    }

    void update() override {
        if (*drive_enable_)
            ++violations;
        for (const auto& torque : parked_torque_)
            if (*torque != 0.)
                ++violations;
        for (std::size_t i = 0; i < control_.size(); ++i) {
            const double command = *control_[i];
            if (!std::isfinite(command) || std::abs(command) > 4.5 + 1e-9)
                ++violations;
            if (!*enabled_ && command != 0.)
                ++violations;
        }
        if (std::abs(*angle_[1]) > .62)
            ++violations;
        if (*experiment_ < 0 || *experiment_ > 5 || *state_ < 0 || *state_ > 6)
            ++violations;

        const double yaw_offset = std::abs(*control_[0] - *excitation_[0]);
        const double pitch_offset = std::abs(*control_[1] - *excitation_[1] - *gravity_);
        if (yaw_offset > 1e-12 || pitch_offset > 1e-12)
            ++violations;

        if (*enabled_ && *state_ == 2) {
            const double yaw = *excitation_[0], pitch = *excitation_[1];
            max_yaw_abs = std::max(max_yaw_abs.load(), std::abs(yaw));
            max_pitch_abs = std::max(max_pitch_abs.load(), std::abs(pitch));
            if (first_sample) {
                first_sample = false;
            } else {
                max_yaw_delta = std::max(max_yaw_delta.load(), std::abs(yaw - last_yaw_));
            }
            last_yaw_ = yaw;
            if (*step_ > 0 && *step_ != step) {
                step = *step_;
                const double value = *axis_ == 2 ? pitch : yaw;
                std::lock_guard lock{mutex_};
                levels.push_back(value);
            }
            yaw_min = std::min(yaw_min.load(), yaw);
            yaw_max = std::max(yaw_max.load(), yaw);
        }
        max_frequency = std::max(max_frequency.load(), *frequency_);
        state = *state_;
        axis = *axis_;
        pitch_angle = *angle_[1];
        step = *step_;
        experiment_id = *experiment_;
        completed = *completed_;
        enabled = *enabled_;
        frequency = *frequency_;
        resets = *resets_;
        gravity = *gravity_;
        residual = *residual_;
        for (std::size_t i = 0; i < fit_.size(); ++i)
            fit[i] = *fit_[i];
        ++ticks;
    }

    std::atomic<int> state{0}, axis{0}, step{0}, experiment_id{0}, violations{0};
    std::atomic<bool> completed{false}, enabled{false};
    std::atomic<double> frequency{0.}, max_frequency{0.}, gravity{0.}, residual{0.};
    std::atomic<double> max_yaw_abs{0.}, max_pitch_abs{0.}, max_yaw_delta{0.};
    std::atomic<double> yaw_min{0.}, yaw_max{0.}, pitch_angle{0.};
    std::atomic<std::size_t> resets{0}, ticks{0};
    std::atomic<double> fit[3];
    std::mutex mutex_;
    std::vector<double> levels;

private:
    static constexpr std::array parked{
        "/wheel_leg/left_hip_joint",   "/wheel_leg/left_knee_joint", "/wheel_leg/right_hip_joint",
        "/wheel_leg/right_knee_joint", "/wheel_leg/left_wheel",      "/wheel_leg/right_wheel",
        "/gimbal/left_friction",       "/gimbal/right_friction",     "/gimbal/bullet_feeder"};

    InputInterface<int> state_, experiment_, axis_, step_;
    InputInterface<bool> enabled_, completed_, drive_enable_;
    InputInterface<double> frequency_, gravity_, residual_;
    std::array<InputInterface<double>, 3> fit_;
    std::array<InputInterface<double>, 2> control_, excitation_, angle_;
    InputInterface<std::size_t> resets_;
    std::array<InputInterface<double>, 9> parked_torque_;
    double last_yaw_ = 0.;
    bool first_sample = true;
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

TEST(GimbalTorqueSweep, DeployedOpenLoopGraph) {
    std::atomic<int> phase{0};
    std::atomic<bool> lost{false};
    pluginlib::ClassLoader<Component> loader{"rmcs_executor", "rmcs_executor::Component"};
    EXPECT_TRUE(loader.isClassAvailable(
        "rmcs_core::controller::identification::GimbalTorqueSweepController"));
    auto model = std::make_shared<Model>();
    Component::initializing_component_name = "plant_state";
    auto plant_state = std::make_shared<PlantState>(model, phase, lost);
    Component::initializing_component_name = "plant_command";
    auto plant_command = std::make_shared<PlantCommand>(model);
    Component::initializing_component_name = "observe";
    auto observe = std::make_shared<Observe>();
    rclcpp::executors::SingleThreadedExecutor ros;
    {
        rmcs_executor::Executor executor{"rmcs_executor", ros};
        struct Shutdown {
            ~Shutdown() { rclcpp::shutdown(); }
        } shutdown;
        executor.add_component(plant_state);
        executor.add_component(plant_command);
        constexpr std::array names{"gimbal_torque_sweep", "value_collector"};
        constexpr std::array types{
            "rmcs_core::controller::identification::GimbalTorqueSweepController",
            "rmcs_core::debug::ValueCollector"};
        for (std::size_t i = 0; i < names.size(); ++i) {
            Component::initializing_component_name = names[i];
            executor.add_component(loader.createSharedInstance(types[i]));
        }
        executor.add_component(observe);
        ASSERT_NO_THROW(executor.start());
        ASSERT_TRUE(await([&] { return observe->ticks.load() > 10; }));
        EXPECT_FALSE(observe->enabled.load());
        EXPECT_EQ(observe->state.load(), 0);

        // Hold, feedback loss, explicit stop.
        phase = 1;
        ASSERT_TRUE(await([&] { return observe->enabled.load() && observe->state.load() == 1; }));
        const auto resets = observe->resets.load();
        lost = true;
        ASSERT_TRUE(
            await([&] { return observe->state.load() == 4 && !observe->enabled.load(); }, 2000ms));
        lost = false;
        phase = 0;
        ASSERT_TRUE(await([&] { return observe->state.load() == 0; }));
        EXPECT_GT(observe->resets.load(), resets);

        const int run_phase = experiment == "calibration" ? 3 : 2;
        phase = run_phase;
        ASSERT_TRUE(
            await([&] { return observe->enabled.load() && observe->state.load() == 2; }, 2000ms));
        const int expected_experiment = experiment == "chirp"       ? 1
                                      : experiment == "staircase"   ? 2
                                      : experiment == "square"      ? 3
                                      : experiment == "calibration" ? 4
                                                                    : 5;
        EXPECT_EQ(observe->experiment_id.load(), expected_experiment);
        EXPECT_EQ(observe->axis.load(), run_phase == 3 ? 2 : 1);

        const auto timeout = experiment == "calibration" ? 20000ms : 4000ms;
        ASSERT_TRUE(await([&] { return observe->completed.load(); }, timeout));
        EXPECT_EQ(observe->state.load(), 3);
        EXPECT_TRUE(observe->enabled.load());

        if (experiment == "chirp") {
            EXPECT_GE(observe->max_frequency.load(), .5);
            EXPECT_LE(observe->max_frequency.load(), 5. + 1e-9);
            EXPECT_GT(observe->max_yaw_abs.load(), .2);
            EXPECT_LE(observe->max_yaw_abs.load(), .35 + 1e-6);
            const double pitch = observe->pitch_angle.load();
            const double holding =
                -kGravityCos * std::cos(pitch) - kGravitySin * std::sin(pitch) - kGravityConst;
            EXPECT_NEAR(observe->gravity.load(), holding, 1e-6);
            EXPECT_LT(observe->max_pitch_abs.load(), 1e-9);
        } else if (experiment == "staircase") {
            std::lock_guard lock{observe->mutex_};
            ASSERT_EQ(observe->levels.size(), 9u);
            EXPECT_NEAR(observe->levels.front(), -.35, 1e-6);
            EXPECT_NEAR(observe->levels.back(), .35, 1e-6);
            for (std::size_t i = 1; i < observe->levels.size(); ++i)
                EXPECT_GT(observe->levels[i], observe->levels[i - 1]);
        } else if (experiment == "square") {
            EXPECT_NEAR(observe->yaw_max.load(), .35, 1e-6);
            EXPECT_NEAR(observe->yaw_min.load(), -.35, 1e-6);
            // Per-tick delta scales with scheduler jitter; a hard step is the full 0.35.
            EXPECT_LT(observe->max_yaw_delta.load(), .35 * .5);
            EXPECT_GT(observe->max_yaw_delta.load(), .0);
        } else if (experiment == "sine") {
            EXPECT_NEAR(observe->max_frequency.load(), 2.0, .01);
            EXPECT_GT(observe->max_yaw_abs.load(), .2);
            EXPECT_LE(observe->max_yaw_abs.load(), .35 + 1e-6);
        } else {
            // cos and the constant term are nearly collinear over +-30 deg, so
            // the coefficients themselves are only weakly identifiable; the
            // model prediction over the calibrated span is the real criterion.
            for (const double angle : {-.5, -.3, -.1, .1, .3, .45}) {
                const double predicted = observe->fit[0].load() * std::cos(angle)
                                       + observe->fit[1].load() * std::sin(angle)
                                       + observe->fit[2].load();
                const double truth = -(
                    kGravityCos * std::cos(angle) + kGravitySin * std::sin(angle) + kGravityConst);
                EXPECT_NEAR(predicted, truth, .04) << "at " << angle;
            }
            EXPECT_LT(observe->residual.load(), .04);
            EXPECT_NEAR(observe->gravity.load(), 0., 1e-9);
            // Calibration owns pitch only: selecting yaw must be refused.
            phase = 2;
            ASSERT_TRUE(
                await([&] { return observe->state.load() == 6 && !observe->enabled.load(); }));
        }

        phase = 0;
        ASSERT_TRUE(await([&] { return observe->state.load() == 0 && !observe->enabled.load(); }));
        EXPECT_EQ(observe->violations.load(), 0);
    }
    std::ifstream csv{RMCS_TORQUE_CSV};
    ASSERT_TRUE(csv.good());
    std::string header, row;
    ASSERT_TRUE(std::getline(csv, header));
    EXPECT_NE(header.find("/gimbal/pitch/torque_sweep/gravity"), std::string::npos);
    EXPECT_NE(header.find("/gimbal/yaw/control_torque"), std::string::npos);
    std::size_t rows = 0;
    while (std::getline(csv, row))
        ++rows;
    EXPECT_GT(rows, 200u);
}
} // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    const std::vector<const char*> args{
        "gimbal_torque_sweep_integration",
        "--ros-args",
        "--params-file",
        RMCS_TORQUE_PROFILE,
        "-p",
        "gimbal_torque_sweep:experiment:=" RMCS_TORQUE_EXPERIMENT,
        "-p",
        "value_collector:csv_path:=" RMCS_TORQUE_CSV,
        "-p",
        "gimbal_torque_sweep:settle_s:=0.02",
        "-p",
        "gimbal_torque_sweep:ramp_s:=0.02",
        "-p",
        "gimbal_torque_sweep:duration_s:=1.20",
        "-p",
        "gimbal_torque_sweep:staircase_dwell_s:=0.05",
        "-p",
        "gimbal_torque_sweep:square_period_s:=0.20",
        "-p",
        "gimbal_torque_sweep:square_cycles:=2",
        "-p",
        "gimbal_torque_sweep:square_edge_s:=0.02",
        "-p",
        "gimbal_torque_sweep:yaw_torque_amplitude_nm:=0.35",
        "-p",
        "gimbal_torque_sweep:sine_frequency_hz:=2.0",
        "-p",
        "gimbal_torque_sweep:sine_cycles:=4",
        "-p",
        "gimbal_torque_sweep:sine_ramp_s:=0.05",
        "-p",
        "gimbal_torque_sweep:pitch_gravity_enabled:=true",
        "-p",
        "gimbal_torque_sweep:pitch_gravity_cos_nm:=-0.5",
        "-p",
        "gimbal_torque_sweep:pitch_gravity_sin_nm:=-0.1",
        "-p",
        "gimbal_torque_sweep:pitch_gravity_const_nm:=-0.02",
        "-p",
        "gimbal_torque_sweep:calibration_dwell_s:=1.0",
        "-p",
        "gimbal_torque_sweep:calibration_rate_rad_s:=0.8",
        "-p",
        "gimbal_torque_sweep:calibration_kp:=6.0",
        "-p",
        "gimbal_torque_sweep:calibration_ki:=10.0",
        "-p",
        "gimbal_torque_sweep:max_velocity_rad_s:=20.0"};
    rclcpp::init(static_cast<int>(args.size()), args.data());
    const int result = RUN_ALL_TESTS();
    if (rclcpp::ok())
        rclcpp::shutdown();
    return result;
}
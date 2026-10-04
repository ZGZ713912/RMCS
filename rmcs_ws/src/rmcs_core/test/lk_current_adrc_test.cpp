#include <array>
#include <bit>
#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "component_fixture.hpp"
#include "controller/chassis/deformable_joint_controller.cpp"
#include "hardware/device/lk_motor.hpp"

namespace {
using rmcs_core::controller::chassis::DeformableJointController;
using rmcs_core::hardware::device::CanPacket8;
using rmcs_core::hardware::device::LkMotor;
using Component = rmcs_executor::Component;

class EmptyComponent : public Component {
public:
    void update() override {}
};

int16_t current(CanPacket8 packet) { return std::bit_cast<int16_t>(packet.data[2]); }

class Source : public Component {
public:
    Source() {
        register_output("/motor/control_current_raw", raw, 2000.0);
        register_output("/motor/control_torque", torque, 20.0);
        register_output("/motor/control_angle", angle, 1.0);
    }
    void update() override {}
    OutputInterface<double> raw, torque, angle;
};

class Sink : public Component {
public:
    Sink() {
        register_input("/motor/current_raw", iq);
        register_input("/motor/raw_current", bus_iq);
        register_input("/motor/feedback_sequence", sequence);
        register_input("/motor/feedback_time_s", receive_time);
        register_input("/motor/command_current_raw", encoded);
        register_input("/motor/command_prepare_time_s", prepare_time);
        register_input("/motor/command_limited", clipped);
    }
    void update() override {}
    InputInterface<double> iq, bus_iq, sequence, receive_time, encoded, prepare_time, clipped;
};

TEST(LkCurrent, DirectEncodingIsIndependentOfLegacyKtAndReduction) {
    EmptyComponent status, command;
    LkMotor m{status, command, "/motor"};
    for (auto type : {LkMotor::Type::kMG5010Ei36, LkMotor::Type::kMG4010Ei10}) {
        m.configure(LkMotor::Config{type}.set_reversed());
        EXPECT_EQ(current(m.generate_current_command(2000.0)), -2000);
        EXPECT_EQ(current(m.generate_current_command(9999.0)), -2048);
        EXPECT_EQ(current(m.generate_current_command(-9999.0)), 2048);
        for (double invalid :
             {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity()})
            EXPECT_EQ(current(m.generate_current_command(invalid)), 0);
    }
    m.configure(LkMotor::Config{LkMotor::Type::kMG5010Ei36}.set_current_command_limit(700.0));
    EXPECT_EQ(current(m.generate_current_command(2000.0)), 700);
}

TEST(LkCurrent, NativeInputWinsAndFeedbackHasSequenceAndActualEncodedCounts) {
    EmptyComponent status, command;
    LkMotor m{
        status, command, "/motor", LkMotor::Config{LkMotor::Type::kMG5010Ei36}.set_reversed()};
    Source source;
    Sink sink;
    std::array<Component*, 4> components{&status, &command, &source, &sink};
    rmcs_executor::Executor::ordered(components);
    EXPECT_EQ(current(m.generate_command()), -2000); // Angle/old torque inputs are also finite.
    EXPECT_EQ(*sink.encoded, 2000.0);
    EXPECT_TRUE(std::isfinite(*sink.prepare_time));
    *source.raw = 3000.0;
    EXPECT_EQ(current(m.generate_command()), -2048);
    EXPECT_EQ(*sink.encoded, 2048.0);
    EXPECT_EQ(*sink.clipped, 1.0);
    EXPECT_EQ(*sink.sequence, 0.0);
    EXPECT_TRUE(std::isnan(*sink.iq));

    auto frame = CanPacket8{
        CanPacket8::Quarter{0x209C}, CanPacket8::Quarter{2000}, CanPacket8::Quarter{36},
        CanPacket8::Quarter{1000}};
    m.store_status(frame.as_bytes());
    m.update_status();
    EXPECT_EQ(*sink.sequence, 1.0);
    EXPECT_EQ(*sink.bus_iq, 2000.0);
    EXPECT_EQ(*sink.iq, -2000.0);
    const double timestamp = *sink.receive_time;
    EXPECT_TRUE(std::isfinite(timestamp));
    for (uint8_t id : {0x9A, 0x92, 0x94, 0x80, 0x81, 0xA0}) {
        frame.as_writable_bytes()[0] = static_cast<std::byte>(id);
        m.store_status(frame.as_bytes());
        m.update_status();
        EXPECT_EQ(*sink.sequence, 1.0);
        EXPECT_EQ(*sink.receive_time, timestamp);
        EXPECT_EQ(*sink.iq, -2000.0);
    }
}

class AdrcSource : public Component {
public:
    AdrcSource() {
        register_output("/measurement", measurement, 0.0);
        register_output("/target", target, 1.0);
        register_output("/target_velocity", velocity, 0.0);
        register_output("/chassis/joint_control_active", enabled, true);
        register_output("/applied", applied, 0.0);
    }
    void update() override {}
    OutputInterface<double> measurement, target, velocity, applied;
    OutputInterface<bool> enabled;
};

class AdrcSink : public Component {
public:
    AdrcSink() { register_input("/output", output); }
    void update() override {}
    InputInterface<double> output;
};

rclcpp::NodeOptions options(const std::string& domain) {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides(
        {{"measurement_angle", "/measurement"},
         {"setpoint_angle", "/target"},
         {"setpoint_velocity", "/target_velocity"},
         {"control", "/output"},
         {"output_domain", domain},
         {"controller_output_to_current_raw", 10.0},
         {"applied_current_raw", "/applied"},
         {"b0", -1.0},
         {"k1", 30.0},
         {"k2", 1.0},
         {"alpha2", 1.0},
         {"u_min", -0.1},
         {"u_max", 0.1},
         {"output_min", -0.05},
         {"output_max", 0.05}});
    return options;
}

class AdrcTest : public testing::Test {
protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
        Component::initializing_component_name = "current_adrc_test";
    }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(AdrcTest, NativeModeIgnoresBothLegacyTorqueClampsAndResets) {
    for (const auto& domain : {std::string{"current_raw"}, std::string{"legacy_torque"}}) {
        Component::initializing_component_name = "current_adrc_test";
        DeformableJointController controller{options(domain)};
        AdrcSource source;
        AdrcSink sink;
        std::array<Component*, 3> components{&source, &controller, &sink};
        rmcs_executor::Executor::pair(components);
        controller.before_updating();
        controller.update();
        if (domain == "current_raw")
            EXPECT_NEAR(*sink.output, -300.0, 1e-9);
        else
            EXPECT_EQ(*sink.output, -0.05);
        *source.enabled = false;
        controller.update();
        EXPECT_TRUE(std::isnan(*sink.output));
        *source.enabled = true;
        controller.update();
        EXPECT_NEAR(*sink.output, domain == "current_raw" ? -300.0 : -0.05, 1e-9);
    }
}

TEST_F(AdrcTest, ObserverConsumesActualAppliedProbeAndSaturationAndRejectsNaN) {
    DeformableJointController controller{options("current_raw")};
    AdrcSource source;
    AdrcSink sink;
    std::array<Component*, 3> components{&source, &controller, &sink};
    rmcs_executor::Executor::pair(components);
    *source.target = 0.0;
    *source.applied = 1000.0;
    controller.update();
    // b0=-1, h=.001, applied=100 controller units => z2=-.1, e2=.1, u=-.1 => -1 count.
    EXPECT_NEAR(*sink.output, -1.0, 1e-9);
    *source.applied = std::numeric_limits<double>::quiet_NaN();
    controller.update();
    EXPECT_TRUE(std::isnan(*sink.output));
    *source.applied = 0.0;
    controller.update();
    EXPECT_NEAR(*sink.output, 0.0, 1e-9);
}
} // namespace

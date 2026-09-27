#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>

#include <gtest/gtest.h>
#include <rmcs_executor/component.hpp>

#include "controller/chassis/wheel_leg_joint_pair_geometry.hpp"
#include "hardware/device/dm_motor.hpp"

namespace {

class TestComponent : public rmcs_executor::Component {
public:
    void update() override {}
};

using rmcs_core::hardware::device::DmMotor;

struct MotorFixture {
    TestComponent status;
    TestComponent command;
    DmMotor motor{status, command, "/test_motor"};

    MotorFixture() {
        motor.configure(
            DmMotor::Config{DmMotor::Type::kDM8009}.set_id(1).set_feedback_id(0).set_limits(
                12.5, 45.0, 18.0));
    }
};

TEST(DmMotor, VelocityFrameEncodingAndLimits) {
    MotorFixture fixture;
    auto& motor = fixture.motor;

    constexpr auto kZero = std::array<std::byte, 8>{
        std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}};
    EXPECT_TRUE(std::ranges::equal(motor.generate_velocity_command(0.0).as_bytes(), kZero));

    // +1.0f = 0x3F800000, little-endian bytes
    constexpr auto kPlusOne = std::array<std::byte, 8>{
        std::byte{0x00}, std::byte{0x00}, std::byte{0x80}, std::byte{0x3F},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}};
    EXPECT_TRUE(std::ranges::equal(motor.generate_velocity_command(1.0).as_bytes(), kPlusOne));

    // +45.0f = 0x42340000, clamped to V_MAX
    constexpr auto kPlusMax = std::array<std::byte, 8>{
        std::byte{0x00}, std::byte{0x00}, std::byte{0x34}, std::byte{0x42},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}};
    EXPECT_TRUE(std::ranges::equal(motor.generate_velocity_command(100.0).as_bytes(), kPlusMax));

    const auto nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(std::ranges::equal(motor.generate_velocity_command(nan).as_bytes(), kZero));

    EXPECT_EQ(motor.send_id(), 1u);
    EXPECT_EQ(motor.velocity_send_id(), 0x201u);

    auto enable = motor.enable_command();
    EXPECT_EQ(enable.as_bytes()[7], std::byte{0xFC});
    auto disable = motor.disable_command();
    EXPECT_EQ(disable.as_bytes()[7], std::byte{0xFD});
    auto save_zero = motor.set_zero_command();
    EXPECT_EQ(save_zero.as_bytes()[7], std::byte{0xFE});
    auto clear = motor.clear_error_command();
    EXPECT_EQ(clear.as_bytes()[7], std::byte{0xFB});
}

TEST(DmMotor, ReversedVelocityFrame) {
    MotorFixture fixture;
    auto& motor = fixture.motor;
    motor.configure(
        DmMotor::Config{DmMotor::Type::kDM8009}
            .set_id(1)
            .set_feedback_id(0)
            .set_reversed()
            .set_limits(12.5, 45.0, 18.0));

    // -1.0f = 0xBF800000
    constexpr auto kMinusOne = std::array<std::byte, 8>{
        std::byte{0x00}, std::byte{0x00}, std::byte{0x80}, std::byte{0xBF},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}};
    EXPECT_TRUE(std::ranges::equal(motor.generate_velocity_command(1.0).as_bytes(), kMinusOne));
}

TEST(DmMotor, VelocityFrameClampsBothDirectionsAndRejectsNonfiniteValues) {
    MotorFixture fixture;
    auto& motor = fixture.motor;
    motor.configure(
        DmMotor::Config{DmMotor::Type::kDM8009}.set_id(2).set_reversed().set_limits(
            12.5, 4.0, 18.0));

    EXPECT_EQ(motor.send_id(), 2u);
    EXPECT_EQ(motor.velocity_send_id(), 0x202u);
    constexpr auto kNegativeLimit = std::array<std::byte, 8>{
        std::byte{0x00}, std::byte{0x00}, std::byte{0x80}, std::byte{0xC0},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}};
    constexpr auto kPositiveLimit = std::array<std::byte, 8>{
        std::byte{0x00}, std::byte{0x00}, std::byte{0x80}, std::byte{0x40},
        std::byte{0x00}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00}};
    constexpr auto kZero = std::array<std::byte, 8>{};
    EXPECT_TRUE(
        std::ranges::equal(motor.generate_velocity_command(100.0).as_bytes(), kNegativeLimit));
    EXPECT_TRUE(
        std::ranges::equal(motor.generate_velocity_command(-100.0).as_bytes(), kPositiveLimit));
    EXPECT_TRUE(
        std::ranges::equal(
            motor.generate_velocity_command(std::numeric_limits<double>::quiet_NaN()).as_bytes(),
            kZero));
    EXPECT_TRUE(
        std::ranges::equal(
            motor.generate_velocity_command(std::numeric_limits<double>::infinity()).as_bytes(),
            kZero));
    EXPECT_TRUE(
        std::ranges::equal(
            motor.generate_velocity_command(-std::numeric_limits<double>::infinity()).as_bytes(),
            kZero));
}

TEST(DmMotor, FeedbackStatusIsNotAlwaysFault) {
    MotorFixture fixture;
    auto& motor = fixture.motor;
    auto feedback = std::array<std::byte, 8>{std::byte{0x11}, std::byte{0x80}, std::byte{0x00},
                                             std::byte{0x80}, std::byte{0x08}, std::byte{0x00},
                                             std::byte{0x30}, std::byte{0x32}};

    EXPECT_FALSE(motor.match_then_store_status(1, feedback)); // Master ID defaults to 0
    EXPECT_TRUE(motor.match_then_store_status(0, feedback));
    motor.update_status();
    EXPECT_EQ(motor.status_code(), 1);                        // Official protocol: enabled
    EXPECT_EQ(motor.fault_code(), 0);
    EXPECT_EQ(motor.temperature_mos(), 48.0);
    EXPECT_EQ(motor.temperature_rotor(), 50.0);

    feedback[0] = std::byte{0xA1};
    EXPECT_TRUE(motor.match_then_store_status(0, feedback));
    motor.update_status();
    EXPECT_EQ(motor.fault_code(), 0xA);                       // overcurrent

    feedback[0] = std::byte{0x01};
    EXPECT_TRUE(motor.match_then_store_status(0, feedback));
    motor.update_status();
    EXPECT_EQ(motor.status_code(), 0);                        // disabled, not a fault
    EXPECT_EQ(motor.fault_code(), 0);
}

TEST(DmMotor, InvalidMappingIsRejected) {
    MotorFixture fixture;
    EXPECT_THROW(
        fixture.motor.configure(DmMotor::Config{DmMotor::Type::kDM8009}.set_id(16)),
        std::invalid_argument);
    EXPECT_THROW(
        fixture.motor.configure(
            DmMotor::Config{DmMotor::Type::kDM8009}.set_limits(12.5, 0.0, 18.0)),
        std::invalid_argument);
    EXPECT_THROW(
        fixture.motor.configure(
            DmMotor::Config{DmMotor::Type::kDM8009}.set_limits(12.5, 45.0, 0.0)),
        std::invalid_argument);
}

} // namespace

TEST(DmMotor, SingleTurnPhaseAcrossCalibrationZeroAndReconnect) {
    MotorFixture fixture;
    auto& motor = fixture.motor;
    motor.configure(
        DmMotor::Config{DmMotor::Type::kDM8009}
            .set_id(1)
            .set_feedback_id(0)
            .set_reversed()
            .set_angle_offset(-2.93));
    const auto observe = [&](double raw) {
        const auto encoded = static_cast<std::uint16_t>(std::round((raw + 12.5) / 25.0 * 65535.0));
        std::array<std::byte, 8> packet{
            std::byte{0x11}, std::byte{0}, std::byte{0}, std::byte{0x80}, std::byte{0x08}};
        packet[1] = static_cast<std::byte>(encoded >> 8);
        packet[2] = static_cast<std::byte>(encoded & 0xFF);
        EXPECT_TRUE(motor.match_then_store_status(0, packet));
        motor.update_status();
        return motor.angle();
    };
    const auto phase_error = [](double a, double b) {
        return std::remainder(a - b, 2.0 * std::numbers::pi);
    };
    EXPECT_NEAR(phase_error(observe(0.0), -2.93), 0.0, 0.0002);
    const double before = observe(2.0 * std::numbers::pi - 0.01);
    const double after = observe(0.01);
    EXPECT_NEAR(phase_error(after, before), -0.02, 0.0004);
    motor.reset_feedback_tracking();
    EXPECT_FALSE(motor.feedback_ready());
    EXPECT_NEAR(phase_error(observe(0.01), after), 0.0, 1e-9);
    EXPECT_NEAR(phase_error(observe(-2.0 * std::numbers::pi + 0.01), after), 0.0, 0.0004);
    // Large physical relocations while disabled never poison a turn counter.
    observe(3.0);
    observe(-2.0);
    EXPECT_NEAR(phase_error(observe(0.01), after), 0.0, 1e-9);
}

TEST(DmMotor, RawZeroGivesConfiguredPoseForAllFourMotors) {
    // Both adjacent POS16 codes represent zero to within half a quantization
    // step. Neither is a signed int16 value or a degree measurement.
    for (const double offset : {-1.6, -2.93, 1.6, 2.93}) {
        for (const std::uint16_t code : {0x7fffu, 0x8000u}) {
            MotorFixture fixture;
            auto& motor = fixture.motor;
            motor.configure(DmMotor::Config{DmMotor::Type::kDM8009}
                                .set_reversed().set_angle_offset(offset));
            const std::array<std::byte, 8> packet{
                std::byte{0x01}, static_cast<std::byte>(code >> 8),
                static_cast<std::byte>(code & 0xff), std::byte{0x80}, std::byte{0x08},
                std::byte{0x00}, std::byte{0x30}, std::byte{0x32}};
            ASSERT_TRUE(motor.match_then_store_status(0, packet));
            EXPECT_FALSE(motor.feedback_ready()); // received, but not yet decoded
            motor.update_status();
            EXPECT_TRUE(motor.feedback_ready());
            EXPECT_EQ(motor.position_code(), code);
            EXPECT_NEAR(motor.raw_angle(), 0.0, 0.000191);
            EXPECT_NEAR(motor.angle(), offset, 0.000191);
            EXPECT_EQ(motor.angle_offset(), offset);
            EXPECT_TRUE(motor.reversed());
            EXPECT_EQ(motor.position_max(), 12.5);
            auto observed_packet = motor.feedback_packet();
            EXPECT_TRUE(std::ranges::equal(observed_packet.as_bytes(), packet));
        }
    }
}

TEST(DmMotor, NewArrivalDoesNotRelabelOldDecodedFeedback) {
    MotorFixture fixture;
    auto& motor = fixture.motor;
    std::array<std::byte, 8> packet{
        std::byte{0x01}, std::byte{0x80}, std::byte{0}, std::byte{0x80}, std::byte{0x08}};
    ASSERT_TRUE(motor.match_then_store_status(0, packet));
    motor.update_status();
    const auto decoded_at = motor.last_feedback_time();
    packet[1] = std::byte{0x90};
    ASSERT_TRUE(motor.match_then_store_status(0, packet));
    EXPECT_EQ(motor.last_feedback_time(), decoded_at);
    EXPECT_EQ(motor.position_code(), 0x8000);
    motor.update_status();
    EXPECT_GE(motor.last_feedback_time(), decoded_at);
    EXPECT_EQ(motor.position_code(), 0x9000);
    motor.reset_feedback_tracking();
    EXPECT_FALSE(motor.feedback_ready());
}

TEST(DmMotor, ReportedCalibrationPoseIsNotAnAngleWrapAtZero) {
    // Representative POS16 values for the user's log, assuming the current
    // P_MAX=12.5 and the signed phase near zero. These are synthetic frames,
    // not a real CAN capture; the new diagnostics are needed to obtain that.
    constexpr std::array<double, 4> offsets{-1.6, -2.93, 1.6, 2.93};
    constexpr std::array<double, 4> logged_deg{-127.77, 150.19, 112.56, -171.09};
    constexpr std::array<double, 4> raw_deg{36.09675, 41.93337, -20.88675, -21.03337};
    constexpr std::array<std::uint16_t, 4> codes{0x8673, 0x877e, 0x7c44, 0x7c3d};
    for (std::size_t i = 0; i < offsets.size(); ++i) {
        MotorFixture fixture;
        auto& motor = fixture.motor;
        motor.configure(DmMotor::Config{DmMotor::Type::kDM8009}
                            .set_reversed().set_angle_offset(offsets[i]));
        const std::array<std::byte, 8> packet{
            std::byte{0x01}, static_cast<std::byte>(codes[i] >> 8),
            static_cast<std::byte>(codes[i] & 0xff), std::byte{0x80}, std::byte{0x08}};
        ASSERT_TRUE(motor.match_then_store_status(0, packet));
        motor.update_status();
        EXPECT_NEAR(motor.angle() * 180.0 / std::numbers::pi, logged_deg[i], 0.012);
        EXPECT_NEAR(motor.raw_phase() * 180.0 / std::numbers::pi, raw_deg[i], 0.012);
        EXPECT_GT(std::abs(motor.raw_phase()), 0.3);
    }
}

TEST(DmMotor, RealFeedbackCapturesKeepTheSameLeftOpeningAcrossTheEncoderSeam) {
    using Geometry = rmcs_core::controller::chassis::WheelLegJointPairGeometry;
    // Exact bytes supplied from two robot_status snapshots. The second pose
    // was described as only approximately at the calibration reference.
    constexpr std::array<std::array<std::array<std::uint8_t, 8>, 4>, 2> captures{{
        {{{0x11, 0x4b, 0x63, 0x7f, 0xf8, 0x00, 0x1b, 0x19},
          {0x11, 0x4c, 0x50, 0x7f, 0xf7, 0xff, 0x1e, 0x1b},
          {0x12, 0xb6, 0x66, 0x7f, 0xf7, 0xfa, 0x1c, 0x1a},
          {0x12, 0xb6, 0x76, 0x7f, 0xf7, 0xd9, 0x1d, 0x1a}}},
        {{{0x11, 0x81, 0x62, 0x7f, 0xf8, 0x3d, 0x1b, 0x19},
          {0x11, 0x82, 0x4f, 0x7f, 0xf8, 0x44, 0x1e, 0x1b},
          {0x12, 0x80, 0x4e, 0x7f, 0xe7, 0xcc, 0x1c, 0x1a},
          {0x12, 0x7f, 0xd7, 0x7f, 0xf7, 0xb2, 0x1d, 0x1a}}}}};
    constexpr std::array<double, 4> offsets{-1.6, -2.93, 1.6, 2.93};
    constexpr std::array<std::array<double, 4>, 2> expected_deg{{
        {-157.293066, 121.323452, 147.282588, -136.863736},
        {-99.421532, 179.194987, 89.957478, 168.761840}}};
    constexpr std::array<double, 2> expected_right{1.3238963912413215, 1.3753955901426718};
    for (std::size_t sample = 0; sample < captures.size(); ++sample) {
        std::array<double, 4> q;
        for (std::size_t i = 0; i < q.size(); ++i) {
            MotorFixture fixture;
            auto& motor = fixture.motor;
            const auto id = static_cast<std::uint8_t>(i / 2 + 1);
            motor.configure(DmMotor::Config{DmMotor::Type::kDM8009}
                                .set_id(id).set_feedback_id(id)
                                .set_reversed().set_angle_offset(offsets[i]));
            ASSERT_TRUE(motor.match_then_store_status(
                id, std::as_bytes(std::span{captures[sample][i]})));
            motor.update_status();
            EXPECT_TRUE(motor.feedback_ready());
            EXPECT_EQ(motor.status_code(), 1);
            EXPECT_EQ(motor.fault_code(), 0);
            q[i] = motor.angle();
            EXPECT_NEAR(q[i] * 180.0 / std::numbers::pi, expected_deg[sample][i], 0.000001);
        }
        EXPECT_NEAR(Geometry::decode(Geometry::Side::kLeft, q[0], q[1]).difference,
                    1.420409704737926, 1e-12);
        EXPECT_NEAR(Geometry::decode(Geometry::Side::kRight, q[2], q[3]).difference,
                    expected_right[sample], 1e-12);
        EXPECT_FALSE(Geometry::feedback(Geometry::Side::kLeft, q[0], q[1], 0.0, 1.33, 0.03));
        EXPECT_EQ(Geometry::feedback(Geometry::Side::kRight, q[2], q[3], 0.0, 1.33, 0.03)
                      .has_value(), sample == 0);
        // The requested 110-degree upper feedback allowance admits both real
        // captures without widening the lower bound or modifying the offsets.
        EXPECT_TRUE(Geometry::feedback(Geometry::Side::kLeft, q[0], q[1], 0.0, 1.33, 0.03, 0.0985));
        EXPECT_TRUE(Geometry::feedback(Geometry::Side::kRight, q[2], q[3], 0.0, 1.33, 0.03, 0.0985));
    }
}

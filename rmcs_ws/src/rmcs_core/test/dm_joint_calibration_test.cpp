#include <array>
#include <chrono>
#include <limits>

#include <gtest/gtest.h>

#include "hardware/device/dm_joint_calibration.hpp"

namespace {

using Calibration = rmcs_core::hardware::device::DmJointCalibration;
using namespace std::chrono_literals;
const auto kStart = Calibration::Clock::time_point{100s};

std::array<Calibration::Feedback, 4> feedback(int ms, double raw = 1.0) {
    std::array<Calibration::Feedback, 4> motors;
    for (auto& motor : motors)
        motor = {true, 0, raw, 0.010989, kStart + std::chrono::milliseconds{ms}};
    return motors;
}

TEST(DmJointCalibration, RequiresExplicitDisarmedStationaryRequest) {
    Calibration calibration;
    EXPECT_FALSE(calibration.blocks_control());
    EXPECT_EQ(calibration.update(feedback(0), kStart), 0);
    EXPECT_FALSE(calibration.request(true, feedback(0), kStart));
    auto motors = feedback(0);
    motors[2].status = 1;
    EXPECT_FALSE(calibration.request(false, motors, kStart));
    motors = feedback(0);
    motors[1].fresh = false;
    EXPECT_FALSE(calibration.request(false, motors, kStart));
    motors = feedback(0);
    motors[0].velocity = 0.2;
    EXPECT_FALSE(calibration.request(false, motors, kStart));
    motors = feedback(0);
    motors[3].raw_phase = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(calibration.request(false, motors, kStart));
    EXPECT_FALSE(calibration.blocks_control());
}

TEST(DmJointCalibration, StaggersWritesAndConfirmsOnlyFreshStableReadback) {
    Calibration calibration;
    ASSERT_TRUE(calibration.request(false, feedback(0), kStart));
    EXPECT_TRUE(calibration.blocks_control());
    EXPECT_EQ(calibration.update(feedback(0), kStart), 0x3);
    EXPECT_EQ(calibration.update(feedback(4), kStart + 4ms), 0);
    EXPECT_EQ(calibration.update(feedback(5), kStart + 5ms), 0xc);
    EXPECT_FALSE(calibration.request(false, feedback(6), kStart + 6ms));
    EXPECT_EQ(calibration.update(feedback(60, 0.000191), kStart + 60ms), 0);
    EXPECT_EQ(calibration.zero_mask(), 0xf);
    EXPECT_EQ(calibration.update(feedback(60, 0.000191), kStart + 160ms), 0);
    EXPECT_EQ(calibration.phase(), Calibration::Phase::kWaiting); // old sample repeated
    EXPECT_EQ(calibration.update(feedback(160, -0.000191), kStart + 160ms), 0);
    EXPECT_EQ(calibration.phase(), Calibration::Phase::kConfirmed);
    EXPECT_FALSE(calibration.blocks_control());
}

TEST(DmJointCalibration, DroppedWriteTimesOutWithoutRetryOrFalseSuccess) {
    Calibration calibration;
    ASSERT_TRUE(calibration.request(false, feedback(0), kStart));
    EXPECT_EQ(calibration.update(feedback(0), kStart), 0x3);
    EXPECT_EQ(calibration.update(feedback(5), kStart + 5ms), 0xc);
    for (int ms = 60; ms < 2000; ms += 20) {
        auto motors = feedback(ms, 0.0);
        motors[2].raw_phase = 0.365;
        EXPECT_EQ(calibration.update(motors, kStart + std::chrono::milliseconds{ms}), 0);
        EXPECT_EQ(calibration.zero_mask(), 0xb);
    }
    EXPECT_EQ(calibration.update(feedback(2000, 0.0), kStart + 2s), 0);
    EXPECT_EQ(calibration.phase(), Calibration::Phase::kFailed);
    EXPECT_TRUE(calibration.blocks_control());
    EXPECT_EQ(calibration.update(feedback(2100, 0.0), kStart + 2100ms), 0);
    EXPECT_EQ(calibration.phase(), Calibration::Phase::kFailed);
    EXPECT_TRUE(calibration.request(false, feedback(2200), kStart + 2200ms));
}

TEST(DmJointCalibration, PreWriteZeroAndStaleFeedbackCannotConfirm) {
    Calibration calibration;
    ASSERT_TRUE(calibration.request(false, feedback(0, 0.0), kStart));
    calibration.update(feedback(0, 0.0), kStart);
    calibration.update(feedback(5, 0.0), kStart + 5ms);
    for (const int ms : {60, 160, 500}) {
        auto motors = feedback(ms, 0.0);
        motors[0].received_at = kStart;
        motors[1].fresh = false;
        calibration.update(motors, kStart + std::chrono::milliseconds{ms});
        EXPECT_EQ(calibration.zero_mask(), 0xc);
        EXPECT_EQ(calibration.phase(), Calibration::Phase::kWaiting);
    }
}

TEST(DmJointCalibration, MotionInterruptsConfirmationAndPreventsRemainingWrites) {
    Calibration calibration;
    ASSERT_TRUE(calibration.request(false, feedback(0), kStart));
    calibration.update(feedback(0), kStart);
    auto moving = feedback(5);
    moving[3].velocity = 0.1;
    EXPECT_EQ(calibration.update(moving, kStart + 5ms), 0);
    EXPECT_EQ(calibration.sent_mask(), 0x3);
    EXPECT_EQ(calibration.phase(), Calibration::Phase::kFailed);
    EXPECT_TRUE(calibration.blocks_control());

    ASSERT_TRUE(calibration.request(false, feedback(10), kStart + 10ms));
    calibration.update(feedback(10), kStart + 10ms);
    calibration.update(feedback(15), kStart + 15ms);
    calibration.update(feedback(70, 0.0), kStart + 70ms);
    moving = feedback(100, 0.0);
    moving[3].velocity = 0.1;
    calibration.update(moving, kStart + 100ms);
    calibration.update(feedback(170, 0.0), kStart + 170ms);
    EXPECT_EQ(calibration.phase(), Calibration::Phase::kWaiting);
    calibration.update(feedback(270, 0.0), kStart + 270ms);
    EXPECT_EQ(calibration.phase(), Calibration::Phase::kConfirmed);
}

} // namespace

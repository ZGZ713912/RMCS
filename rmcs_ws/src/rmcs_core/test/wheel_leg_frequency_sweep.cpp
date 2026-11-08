#include <array>
#include <cmath>
#include <numbers>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/util/wheel_leg_drive.hpp"
#include "identification/wheel_leg_limit_probe.hpp"
#include "identification/wheel_leg_load_compensation.hpp"
#include "identification/wheel_leg_sweep_session.hpp"
#include "identification/wheel_leg_wheel_identification_plan.hpp"

namespace {
using namespace rmcs_core::controller::identification;
using rmcs_msgs::Switch;
constexpr double rad = PairGeometry::rad;

PairGeometry geometry(bool right = false) {
    std::vector<double> b,
        d{-.473731601, -.297071061, -.140638855, .001799635,  .134193202, .259151808,  .378464328,
          .493395522,  .604863832,  .713551877,  .819977446,  .924540437, 1.027554717, 1.129270232,
          1.229888687, 1.329574849, 1.428464858, 1.526672408, 1.624293425};
    for (int beta = 30; beta <= 120; beta += 5)
        b.push_back(beta * rad);
    if (right)
        for (auto& delta : d)
            delta = -delta;
    return {b, d};
}

void arm(WheelLegSweepSession& session) {
    EXPECT_FALSE(session.update(0, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone));
    EXPECT_TRUE(session.enabled());
}

TEST(FrequencySweepLoad, BoundedRateAntiWindupAndReset) {
    WheelLegLoadCompensation load;
    EXPECT_NEAR(load.update(1, 0, 40, .001, 20, 16, 4, false), .004, 1e-12);
    for (int i = 0; i < 10000; ++i)
        load.update(1, 0, 40, .001, 20, 16, 4, false);
    EXPECT_DOUBLE_EQ(load.update(1, 0, 40, .001, 20, 16, 4, false), 16);
    EXPECT_DOUBLE_EQ(load.update(1, 30, 40, .001, 20, 16, 4, false), 16);
    EXPECT_LT(load.update(-1, 0, 40, .001, 20, 16, 4, true), 16);
    load.reset();
    EXPECT_DOUBLE_EQ(load.update(1, 0, 40, .001, 20, 16, 4, true), 0);
    EXPECT_DOUBLE_EQ(load.update(1, 40, 40, .001, 20, 16, 4, false), 0);
    EXPECT_DOUBLE_EQ(load.update(-1, -40, 40, .001, 20, 16, 4, false), 0);
    load.update(1, 0, 40, .001, 20, 16, 4, false);
    EXPECT_DOUBLE_EQ(load.update(1, 0, 40, .1, 20, 16, 4, false), 0);
}

TEST(FrequencySweepLoad, RemovesStaticLoadOffsetWithinAvailableTorque) {
    WheelLegLoadCompensation load;
    double q = .1, dq = 0;
    for (int i = 0; i < 60000; ++i) {
        const double base = -160 * q - 4 * dq;
        const double compensation = load.update(-q, base, 40, .001, 20, 16, 4, false);
        const double torque = std::clamp(base + compensation, -40.0, 40.0);
        dq += (torque + 12 - 2 * dq) * .001;
        q += dq * .001;
    }
    EXPECT_LT(std::abs(q), .001);
    EXPECT_LT(std::abs(dq), .001);
}

TEST(FrequencySweepSession, CurrentSwitchesStartImmediatelyWithoutResetOrMiddleHistory) {
    constexpr std::array switches{Switch::DOWN, Switch::MIDDLE, Switch::UP};
    for (bool joints : {false, true})
        for (auto left : switches)
            for (auto right : switches) {
                WheelLegSweepSession session(joints);
                const bool down = left == Switch::DOWN && right == Switch::DOWN;
                const bool middle = left == Switch::MIDDLE && right == Switch::MIDDLE;
                const bool matching = joints
                                        ? (left == Switch::UP && right == Switch::MIDDLE)
                                              || (left == Switch::MIDDLE && right == Switch::UP)
                                              || (left == Switch::UP && right == Switch::UP)
                                        : !down && !middle;
                EXPECT_EQ(session.update(0, true, left, right, SweepStop::kNone), matching);
                EXPECT_EQ(session.state == SweepState::kRunning, matching);
                EXPECT_EQ(session.enabled(), !down);
                EXPECT_EQ(session.run_id, matching ? 1u : 0u);
            }
}

TEST(FrequencySweepSession, FixedYamlProjectsOnlyRunAtBothMiddleAndCompleteOnce) {
    for (int mode : {1, 2, 3}) {
        WheelLegSweepSession session(true, true, mode);
        EXPECT_FALSE(session.update(0, true, Switch::UP, Switch::UP, SweepStop::kNone));
        ASSERT_TRUE(session.update(1, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone));
        EXPECT_EQ(session.mode, mode);
        EXPECT_FALSE(session.update(2, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone));
        EXPECT_EQ(session.run_id, 1u);
        session.update(3, true, Switch::UP, Switch::MIDDLE, SweepStop::kNone);
        EXPECT_EQ(session.state, SweepState::kStandby);
        ASSERT_TRUE(session.update(4, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone));
        session.complete();
        EXPECT_FALSE(session.update(5, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone));
        EXPECT_EQ(session.state, SweepState::kComplete);
        session.update(6, true, Switch::UP, Switch::UP, SweepStop::kNone);
        EXPECT_FALSE(session.update(7, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone));
        EXPECT_EQ(session.state, SweepState::kComplete);
        session.update(8, false, Switch::UNKNOWN, Switch::UNKNOWN, SweepStop::kNone);
        EXPECT_FALSE(session.enabled());
        EXPECT_FALSE(session.update(9, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone));
        session.update(10, true, Switch::DOWN, Switch::DOWN, SweepStop::kNone);
        EXPECT_FALSE(session.enabled());
        EXPECT_TRUE(session.update(11, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone));
        EXPECT_EQ(session.mode, mode);
    }
}

TEST(FrequencySweepSession, StartupUnknownAndRemoteRecoveryNeverLatch) {
    WheelLegSweepSession session(false);
    EXPECT_FALSE(session.update(0, false, Switch::UNKNOWN, Switch::UNKNOWN, SweepStop::kNone));
    EXPECT_EQ(session.reason, SweepStop::kRemoteLost);
    ASSERT_TRUE(session.update(.004, true, Switch::UP, Switch::UP, SweepStop::kNone));
    EXPECT_TRUE(session.enabled());
    session.update(.005, false, Switch::UP, Switch::UP, SweepStop::kNone);
    EXPECT_FALSE(session.enabled());
    ASSERT_TRUE(session.update(.006, true, Switch::UP, Switch::UP, SweepStop::kNone));
    EXPECT_EQ(session.run_id, 2u);
    session.update(.007, true, Switch::DOWN, Switch::DOWN, SweepStop::kNone);
    EXPECT_FALSE(session.enabled());
    EXPECT_EQ(session.reason, SweepStop::kBothDown);
    ASSERT_TRUE(session.update(.008, true, Switch::UP, Switch::UP, SweepStop::kNone));
}

TEST(FrequencySweepSession, CurrentJointModeChangesImmediatelyAndSameModePreservesProgress) {
    WheelLegSweepSession session(true);
    ASSERT_TRUE(session.update(0, true, Switch::UP, Switch::MIDDLE, SweepStop::kNone));
    EXPECT_EQ(session.mode, 1);
    EXPECT_FALSE(session.update(.001, true, Switch::UP, Switch::MIDDLE, SweepStop::kNone));
    EXPECT_DOUBLE_EQ(session.start_time, 0);
    ASSERT_TRUE(session.update(.002, true, Switch::MIDDLE, Switch::UP, SweepStop::kNone));
    EXPECT_EQ(session.mode, 2);
    ASSERT_TRUE(session.update(.003, true, Switch::UP, Switch::UP, SweepStop::kNone));
    EXPECT_EQ(session.mode, 3);
    EXPECT_EQ(session.run_id, 3u);
    session.update(.004, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone);
    EXPECT_EQ(session.state, SweepState::kStandby);
    ASSERT_TRUE(session.update(.005, true, Switch::UP, Switch::UP, SweepStop::kNone));
}

TEST(FrequencySweepSession, MixedDownJointStandbyDoesNotLatchTheNextMode) {
    WheelLegSweepSession session(true);
    EXPECT_FALSE(session.update(0, true, Switch::DOWN, Switch::MIDDLE, SweepStop::kNone));
    EXPECT_EQ(session.state, SweepState::kStandby);
    ASSERT_TRUE(session.update(.001, true, Switch::UP, Switch::MIDDLE, SweepStop::kNone));
    session.update(.002, true, Switch::MIDDLE, Switch::DOWN, SweepStop::kNone);
    EXPECT_EQ(session.state, SweepState::kStandby);
    ASSERT_TRUE(session.update(.003, true, Switch::MIDDLE, Switch::UP, SweepStop::kNone));
    EXPECT_EQ(session.mode, 2);
}

TEST(FrequencySweepSession, WheelSwitchChangesDoNotRestartTheSameExperiment) {
    WheelLegSweepSession session(false);
    ASSERT_TRUE(session.update(0, true, Switch::UP, Switch::MIDDLE, SweepStop::kNone));
    EXPECT_FALSE(session.update(.001, true, Switch::UP, Switch::UP, SweepStop::kNone));
    EXPECT_FALSE(session.update(.002, true, Switch::MIDDLE, Switch::UP, SweepStop::kNone));
    EXPECT_FALSE(session.update(.003, true, Switch::MIDDLE, Switch::DOWN, SweepStop::kNone));
    EXPECT_EQ(session.run_id, 1u);
    EXPECT_DOUBLE_EQ(session.start_time, 0);
}

TEST(FrequencySweepSession, HeldModeRepeatsAfterCompletionWithoutACompletionLatch) {
    for (bool joints : {false, true}) {
        WheelLegSweepSession session(joints);
        ASSERT_TRUE(session.update(0, true, Switch::UP, Switch::UP, SweepStop::kNone));
        session.complete();
        EXPECT_EQ(session.state, SweepState::kComplete);
        EXPECT_TRUE(session.completed);
        EXPECT_TRUE(session.enabled());
        ASSERT_TRUE(session.update(10, true, Switch::UP, Switch::UP, SweepStop::kNone));
        EXPECT_EQ(session.state, SweepState::kRunning);
        EXPECT_FALSE(session.completed);
        EXPECT_EQ(session.run_id, 2u);
        EXPECT_DOUBLE_EQ(session.start_time, 10);
    }
}
TEST(FrequencySweepSession, Real2simCompletionRequiresMiddleOrResetEvenAfterFeedbackLoss) {
    WheelLegSweepSession session(false, true);
    ASSERT_TRUE(session.update(0, true, Switch::UP, Switch::MIDDLE, SweepStop::kNone));
    session.complete();
    EXPECT_FALSE(session.update(1, true, Switch::UP, Switch::UP, SweepStop::kNone));
    EXPECT_EQ(session.state, SweepState::kComplete);
    EXPECT_EQ(session.run_id, 1u);
    session.update(2, false, Switch::UNKNOWN, Switch::UNKNOWN, SweepStop::kNone);
    EXPECT_FALSE(session.enabled());
    EXPECT_FALSE(session.update(3, true, Switch::UP, Switch::UP, SweepStop::kNone));
    EXPECT_EQ(session.state, SweepState::kComplete);
    session.update(4, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone);
    EXPECT_TRUE(session.update(5, true, Switch::UP, Switch::MIDDLE, SweepStop::kNone));
    session.complete();
    session.update(6, true, Switch::DOWN, Switch::DOWN, SweepStop::kNone);
    EXPECT_FALSE(session.enabled());
    EXPECT_TRUE(session.update(7, true, Switch::UP, Switch::MIDDLE, SweepStop::kNone));
}

TEST(FrequencySweepSession, EitherIntermediateJointModeImmediatelyYieldsToBothUp) {
    for (const auto switches :
         {std::array{Switch::UP, Switch::MIDDLE}, std::array{Switch::MIDDLE, Switch::UP}}) {
        WheelLegSweepSession session(true);
        ASSERT_TRUE(session.update(0, true, switches[0], switches[1], SweepStop::kNone));
        ASSERT_TRUE(session.update(.001, true, Switch::UP, Switch::UP, SweepStop::kNone));
        EXPECT_EQ(session.mode, 3);
        EXPECT_EQ(session.run_id, 2u);
        EXPECT_FALSE(session.update(.002, true, Switch::UP, Switch::UP, SweepStop::kNone));
        EXPECT_DOUBLE_EQ(session.start_time, .001);
        session.update(.003, true, Switch::MIDDLE, Switch::MIDDLE, SweepStop::kNone);
        EXPECT_EQ(session.state, SweepState::kStandby);
        EXPECT_EQ(session.mode, 0);
    }
}

TEST(FrequencySweepSession, StaleRemoteNeverReplaysThePreviousSelection) {
    WheelLegSweepSession session(false);
    ASSERT_TRUE(session.update(0, true, Switch::UP, Switch::MIDDLE, SweepStop::kNone));
    session.update(.001, false, Switch::DOWN, Switch::DOWN, SweepStop::kNone);
    EXPECT_EQ(session.state, SweepState::kFault);
    EXPECT_EQ(session.reason, SweepStop::kRemoteLost);
    EXPECT_FALSE(session.enabled());
    ASSERT_TRUE(session.update(.002, true, Switch::UP, Switch::UP, SweepStop::kNone));
}

TEST(FrequencySweepSession, MissingActiveFeedbackRecoversOnNextValidTickWithoutReset) {
    for (auto issue : {SweepStop::kFeedbackTimeout, SweepStop::kNonfinite}) {
        WheelLegSweepSession session(false);
        EXPECT_FALSE(session.update(0, true, Switch::UP, Switch::UP, issue));
        EXPECT_FALSE(session.enabled());
        ASSERT_TRUE(session.update(.001, true, Switch::UP, Switch::UP, SweepStop::kNone));
        EXPECT_TRUE(session.enabled());
    }
}

TEST(FrequencySweepHardware, MissingOrFaultedFeedbackCannotArm) {
    using namespace rmcs_core::hardware;
    EXPECT_FALSE(wheel_leg_dm_pair_safe_to_arm({0, 0, 0, 0, false}));
    EXPECT_FALSE(wheel_leg_dm_pair_safe_to_arm({0, 0, 1, 0, true}));
    EXPECT_TRUE(wheel_leg_dm_pair_safe_to_arm({0, 0, 0, 0, true}));
}

TEST(FrequencySweepTrajectories, BothSidesKeepContinuousTurnsAndConfiguredBetaRange) {
    for (bool right : {false, true}) {
        auto g = geometry(right);
        const double sign = right ? 1 : -1,
                     h = right ? std::numbers::pi / 2 : -std::numbers::pi / 2;
        for (int mode : {1, 2, 3}) {
            WheelLegJointSweepPlan plan(g, sign);
            plan.start(mode, {h, h + g.at(70 * rad).position}, h, 35 * rad, 110 * rad);
            double max_theta = 0;
            auto previous = plan.at(0);
            for (double t = .005; t < plan.duration(); t += .02) {
                const auto x = plan.at(t);
                EXPECT_TRUE(std::isfinite(x.q[0]) && std::isfinite(x.q[1]));
                EXPECT_GE(x.beta, 35 * rad - 1e-8);
                EXPECT_LE(x.beta, 110 * rad + 1e-8);
                EXPECT_LT(std::abs(x.q[0] - previous.q[0]), .15);
                EXPECT_LT(std::abs(x.q[1] - previous.q[1]), .15);
                if (mode == 2) {
                    EXPECT_NEAR(x.q[0], h, 1e-12);
                }
                if (mode == 1) {
                    EXPECT_NEAR(x.beta, 70 * rad, 1e-10);
                }
                max_theta = std::max(max_theta, x.theta);
                previous = x;
            }
            if (mode != 2) {
                EXPECT_GT(max_theta, 2 * std::numbers::pi);
            }
            const auto end = plan.at(plan.duration());
            EXPECT_NEAR(end.q[0], h, 1e-9);
            EXPECT_NEAR(end.dq[0], 0, 1e-9);
            EXPECT_NEAR(end.dq[1], 0, 1e-9);
            for (const auto& segment : plan.segments()) {
                const auto before = plan.at(segment.start + segment.duration - 1e-7);
                const auto after = plan.at(segment.start + segment.duration);
                EXPECT_NEAR(before.q[0], after.q[0], 1e-7);
                EXPECT_NEAR(before.q[1], after.q[1], 1e-7);
                EXPECT_NEAR(before.dq[0], after.dq[0], 1e-6);
                EXPECT_NEAR(before.dq[1], after.dq[1], 1e-6);
            }
        }
    }
}

TEST(FrequencySweepTrajectories, InvalidInitialFkDoesNotProjectOrDiscardRawAngles) {
    auto g = geometry();
    WheelLegJointSweepPlan plan(g, -1);
    const std::array initial{25.0, 27.5};
    plan.start(1, initial, -std::numbers::pi / 2, 35 * rad, 105 * rad);
    const auto first = plan.at(0);
    EXPECT_EQ(first.q, initial);
    EXPECT_FALSE(first.beta_valid);
    EXPECT_TRUE(std::isnan(first.beta));
    EXPECT_FALSE(g.beta(2.5).has_value());
    EXPECT_THROW(g.at(29 * rad), std::out_of_range);
    EXPECT_NEAR(plan.at(5).q[0], 25, 1e-12);
    EXPECT_NEAR(plan.at(5).q[1] - 25, g.at(70 * rad).position, 1e-12);
}

TEST(FrequencySweepTrajectories, TwoSecondRevolutionsKeepLowSpeedBothDirections) {
    auto g = geometry();
    WheelLegJointSweepPlan plan(g, -1);
    plan.start(1, {0, g.at(70 * rad).position}, 0, 45 * rad, 105 * rad);
    std::vector<double> plateaus;
    for (const auto& segment : plan.segments()) {
        if (segment.kind != 2)
            continue;
        const auto sample = plan.at(segment.start + segment.duration / 2);
        plateaus.push_back(sample.dtheta);
        EXPECT_NEAR(sample.ddq[0], 0, 1e-12);
        EXPECT_NEAR(sample.dq[0], sample.dq[1], 1e-12);
        EXPECT_NEAR(std::abs(segment.to.theta - segment.from.theta), 4 * std::numbers::pi, 1e-12);
        if (std::abs(sample.dtheta) > 3) {
            EXPECT_NEAR(2 * std::numbers::pi / std::abs(sample.dtheta), 2, 1e-12);
            EXPECT_GE(segment.duration - 2, 2);
        }
    }
    ASSERT_EQ(plateaus.size(), 8u);
    EXPECT_NEAR(plateaus[0], .2, 1e-12);
    EXPECT_NEAR(plateaus[1], -.2, 1e-12);
    EXPECT_NEAR(plateaus[6], std::numbers::pi, 1e-12);
    EXPECT_NEAR(plateaus[7], -std::numbers::pi, 1e-12);
}

TEST(FrequencySweepTrajectories, KneeDynamicsHasContinuousSpeedPairsAndIndependentHoldout) {
    for (bool right : {false, true}) {
        auto g = geometry(right);
        WheelLegJointSweepPlan plan(g, right ? 1 : -1);
        plan.start(2, {0, g.at(70 * rad).position}, 0, 40 * rad, 110 * rad);
        EXPECT_NEAR(plan.duration(), 242, 1e-10);
        std::array<int, 2> counts{};
        std::array<int, 3> speed_counts{};
        double probe_energy = 0;
        int endpoint_holds = 0;
        for (const auto& segment : plan.segments()) {
            if (segment.kind == 1 && segment.duration == 3) {
                ++endpoint_holds;
                EXPECT_NEAR(segment.to.beta / rad, 40, 1e-9);
            }
            if (segment.kind != 7 && segment.kind != 3)
                continue;
            const auto start = plan.at(segment.start);
            const auto end = plan.at(segment.start + segment.duration - 1e-7);
            EXPECT_NEAR(start.probe_torque[1], 0, 1e-10);
            EXPECT_NEAR(end.probe_torque[1], 0, 1e-10);
            if (segment.kind == 7) {
                ++counts[segment.validation];
                const auto middle = plan.at(segment.start + segment.duration / 2);
                EXPECT_NEAR(std::abs(segment.to.beta - segment.from.beta) / rad, 70, 1e-9);
                EXPECT_NEAR(std::abs(middle.dbeta) / rad, 1.875 * 70 / segment.duration, 1e-8);
                const int speed = segment.duration == 4 ? 0 : segment.duration == 2 ? 1 : 2;
                ++speed_counts[speed];
            }
            for (double t = .005; t < segment.duration; t += .005) {
                const auto sample = plan.at(segment.start + t);
                EXPECT_NEAR(sample.q[0], 0, 1e-12);
                EXPECT_GE(sample.beta / rad, 40 - 1e-8);
                EXPECT_LE(sample.beta / rad, 110 + 1e-8);
                EXPECT_DOUBLE_EQ(sample.probe_torque[0], 0);
                EXPECT_LE(std::abs(sample.probe_torque[1]), 1);
                probe_energy += sample.probe_torque[1] * sample.probe_torque[1];
            }
        }
        EXPECT_EQ(counts, (std::array{18, 18}));
        EXPECT_EQ(endpoint_holds, 6);
        EXPECT_EQ(speed_counts, (std::array{12, 12, 12}));
        EXPECT_GT(probe_energy, 100);
    }
}

TEST(FrequencySweepTrajectories, KneeDynamicsRejectsUnboundedRangeAndAbruptPeriod) {
    auto g = geometry();
    WheelLegJointSweepPlan::Config config;
    config.knee_half_periods_s[2] = .1;
    EXPECT_THROW(WheelLegJointSweepPlan(g, -1, config), std::invalid_argument);
    WheelLegJointSweepPlan plan(g, -1);
    EXPECT_THROW(plan.start(2, {0, 0}, 0, 50 * rad, 105 * rad), std::invalid_argument);
}

TEST(FrequencySweepTrajectories, LocalProbesAreIndependentBoundedAndHaveCompleteHoldout) {
    auto g = geometry();
    WheelLegJointSweepPlan plan(g, -1);
    plan.start(3, {0, g.at(70 * rad).position}, 0, 45 * rad, 105 * rad);
    int train = 0, holdout = 0;
    double hip_energy = 0, knee_energy = 0, cross = 0;
    for (const auto& segment : plan.segments()) {
        const auto start = plan.at(segment.start);
        const auto end = plan.at(segment.start + segment.duration - 1e-7);
        for (std::size_t axis = 0; axis < 2; ++axis) {
            EXPECT_NEAR(start.probe_torque[axis], 0, 1e-10);
            EXPECT_NEAR(end.probe_torque[axis], 0, 1e-10);
        }
        if (segment.kind != 6)
            continue;
        segment.validation ? ++holdout : ++train;
        for (double t = 0; t < segment.duration; t += .005) {
            const auto sample = plan.at(segment.start + t);
            EXPECT_GE(sample.beta / rad, 45);
            EXPECT_LE(sample.beta / rad, 105);
            EXPECT_LE(std::abs(sample.probe_torque[0]), 1);
            EXPECT_LE(std::abs(sample.probe_torque[1]), 1);
            hip_energy += sample.probe_torque[0] * sample.probe_torque[0];
            knee_energy += sample.probe_torque[1] * sample.probe_torque[1];
            cross += sample.probe_torque[0] * sample.probe_torque[1];
        }
    }
    EXPECT_EQ(train, 3);
    EXPECT_EQ(holdout, 3);
    EXPECT_GT(hip_energy, 100);
    EXPECT_GT(knee_energy, 100);
    EXPECT_LT(std::abs(cross / std::sqrt(hip_energy * knee_energy)), .2);
}

TEST(FrequencySweepProbe, DrivesBothEndsAndHoldsTheHip) {
    auto g = geometry();
    WheelLegLimitProbe::Config c;
    c.speed = 90 * rad; // 360 deg in about 5 s, including the ramp-in.
    WheelLegLimitProbe probe(g, c);
    const std::array measured{0.0, 0.0};
    probe.start(0, measured);
    double t = 0;
    for (; t < 20 && probe.stage < 3; t += .005) {
        const auto sample = probe.update(t, measured);
        EXPECT_DOUBLE_EQ(sample.q[0], 0.0);
        EXPECT_DOUBLE_EQ(sample.dq[0], 0.0);
    }
    ASSERT_EQ(probe.stage, 3);
    EXPECT_EQ(probe.result[0], 1);
    EXPECT_EQ(probe.result[1], 1);
    EXPECT_DOUBLE_EQ(probe.boundary_q[0], measured[1]);
    EXPECT_DOUBLE_EQ(probe.boundary_q[1], measured[1]);
    EXPECT_DOUBLE_EQ(probe.boundary_hip[0], measured[0]);
}

TEST(FrequencySweepProbe, FinalReferenceIsTheConfiguredAngleEnd) {
    auto g = geometry();
    WheelLegLimitProbe::Config c;
    c.speed = 90 * rad;
    WheelLegLimitProbe probe(g, c);
    const std::array measured{-30 * rad, -30 * rad + g.at(70 * rad).position};
    probe.start(0, measured);
    WheelLegJointSweepPlan::Sample last;
    double t = 0;
    for (; t < 20 && probe.stage < 3; t += .005)
        last = probe.update(t, measured);
    ASSERT_EQ(probe.stage, 3);
    // The hip reference never moves; the knee ends exactly at the configured end.
    EXPECT_DOUBLE_EQ(last.q[0], measured[0]);
    EXPECT_NEAR(last.q[1], 360 * rad, 1e-9);
    EXPECT_NEAR(probe.boundary_beta[0], 70 * rad, 1e-9);
    EXPECT_DOUBLE_EQ(probe.boundary_q[1], measured[1]);
}

TEST(FrequencySweepProbe, RejectsDegenerateAngleSweep) {
    auto g = geometry();
    EXPECT_THROW(
        WheelLegLimitProbe(g, WheelLegLimitProbe::Config{0, 0, .5 * rad}), std::invalid_argument);
    EXPECT_THROW(
        WheelLegLimitProbe(g, WheelLegLimitProbe::Config{0, 360 * rad, 0}), std::invalid_argument);
    EXPECT_DOUBLE_EQ(WheelLegLimitProbe::Config{}.end, 360 * rad);
    EXPECT_DOUBLE_EQ(WheelLegLimitProbe::Config{}.speed, .5 * rad);
}

class PacketFixture final : public rmcs_executor::Component {
public:
    void update() override {}
};

TEST(FrequencySweepHardware, DjiEncodedTorquePreservesSignAndClips) {
    using Motor = rmcs_core::hardware::device::DjiMotor;
    rmcs_executor::Component::initializing_component_name = "dji_packet_fixture";
    PacketFixture fixture;
    Motor motor{fixture, fixture, "/test_wheel", Motor::Config{Motor::Type::kM3508, 1}};
    for (double torque : {-100., -1., 0., 1., 100.}) {
        const double decoded = motor.command_frame_torque(motor.generate_command(torque));
        EXPECT_NEAR(
            decoded, std::clamp(torque, -motor.max_torque(), motor.max_torque()),
            motor.max_torque() / 16384 + 1e-9);
    }
}

TEST(FrequencySweepCsv, StrictModeRejectsMissingAndUnsupportedSignals) {
    rclcpp::init(0, nullptr);
    {
        pluginlib::ClassLoader<rmcs_executor::Component> loader{
            "rmcs_executor", "rmcs_executor::Component"};
        const auto check = [&](bool strict, const rmcs_executor::Component::OutputInfoMap& map) {
            rmcs_executor::Component::initializing_component_name = "csv_strict_test";
            auto component = loader.createSharedInstance("rmcs_core::debug::ValueCollector");
            auto node = std::dynamic_pointer_cast<rclcpp::Node>(component);
            node->declare_parameter("csv_path", "/tmp/csv_strict_test.csv");
            node->declare_parameter("signals", std::vector<std::string>{"/missing"});
            node->declare_parameter("write_interval", 1);
            node->declare_parameter("flush_interval", 1);
            node->declare_parameter("strict_signals", strict);
            if (strict)
                EXPECT_THROW(component->before_pairing(map), std::runtime_error);
            else
                EXPECT_NO_THROW(component->before_pairing(map));
        };
        check(true, {});
        check(false, {});
        check(true, {{"/missing", {typeid(std::string), rmcs_executor::InterfaceKind::Normal}}});
        check(true, {{"/missing", {typeid(double), rmcs_executor::InterfaceKind::Event}}});
    }
    rclcpp::shutdown();
}

TEST(FrequencySweepHardware, EnabledSessionKeepsFcAndZeroGainMitWithoutFd) {
    using namespace rmcs_core::hardware;
    WheelLegSweepSession session(true);
    arm(session);
    session.complete();
    rmcs_executor::Component::initializing_component_name = "sweep_packet_fixture";
    PacketFixture fixture;
    using Motor = device::DmMotor;
    Motor motor{fixture, fixture, "/sweep_dm", Motor::Config{Motor::Type::kDM8009}};
    WheelLegDmCommandScheduler scheduler;
    int mit = 0, enable = 0;
    for (int tick = 0; tick < 2000; ++tick) {
        session.update(1 + tick * .001, true, Switch::UP, Switch::UP, SweepStop::kNone);
        ASSERT_TRUE(session.enabled());
        const auto action = scheduler.next(session.enabled());
        EXPECT_NE(action, WheelLegDmCommandScheduler::Action::kDisable);
        if (action == WheelLegDmCommandScheduler::Action::kEnable)
            ++enable;
        if (action == WheelLegDmCommandScheduler::Action::kMit
            || action == WheelLegDmCommandScheduler::Action::kFeedbackPoll) {
            ++mit;
            const auto packet = motor.generate_command(0.0);
            EXPECT_FALSE(Motor::command_frame_uses_pd(packet));
            EXPECT_LE(std::abs(motor.command_frame_torque(packet)), 54.0 / 4095.0 + 1e-12);
        }
    }
    EXPECT_GT(mit, 1800);
    EXPECT_GT(enable, 0);
}
WheelProbeConfig wheel_config() {
    WheelProbeConfig c;
    c.low_speeds = {.15, .3, .6};
    c.speeds = {1.5, 3, 6, 12, 20};
    c.ramp_s = 1;
    c.hold_s = 2;
    c.coast_s = 3;
    c.step_hold_s = .5;
    c.chirp_s = c.validation_s = 30;
    c.step_repetitions = 2;
    return c;
}

TEST(FrequencySweepWheels, HighSpeedHasBothDirectionsDirectCoastAndIndependentValidation) {
    auto c = wheel_config();
    c.high_speed_target = 65;
    c.high_speed_only = true;
    WheelProbePlan plan(c);
    EXPECT_DOUBLE_EQ(plan.max_target_speed(), 65);
    EXPECT_DOUBLE_EQ(plan.duration(), 691);
    int peaks = 0, train = 0, heldout = 0;
    std::array<std::array<bool, 2>, 2> covered{};
    const auto& segments = plan.segments();
    for (std::size_t i = 0; i < segments.size(); ++i) {
        const auto& segment = segments[i];
        if (segment.label.ends_with("peak_plateau")) {
            ++peaks;
            EXPECT_DOUBLE_EQ(std::abs(segment.to), 65);
            covered[segment.axis_scale[0] ? 0 : 1][segment.to > 0 ? 0 : 1] = true;
            ASSERT_LT(i + 1, segments.size());
            EXPECT_TRUE(segments[i + 1].coast);
            EXPECT_TRUE(segments[i + 1].label.ends_with("peak_coast"));
        }
        if (segment.waveform == 9) {
            segment.validation ? ++heldout : ++train;
            EXPECT_DOUBLE_EQ(segment.phase, segment.validation ? 1.27 : .31);
            double energy = 0;
            for (double t = 0; t < segment.duration_s; t += .013) {
                const auto sample = plan.at(segment.start_s + t);
                for (std::size_t axis = 0; axis < 2; ++axis) {
                    EXPECT_LE(std::abs(sample.probe_torque[axis]), .35 + 1e-12);
                    if (segment.axis_scale[axis] == 0) {
                        EXPECT_FALSE(sample.servo[axis]);
                        EXPECT_DOUBLE_EQ(sample.velocity_target[axis], 0);
                        EXPECT_DOUBLE_EQ(sample.probe_torque[axis], 0);
                    }
                    energy += sample.probe_torque[axis] * sample.probe_torque[axis];
                }
            }
            EXPECT_GT(energy, 1);
        }
        const auto sample = plan.at(segment.start_s + .5 * segment.duration_s);
        for (std::size_t axis = 0; axis < 2; ++axis) {
            EXPECT_LE(std::abs(sample.velocity_target[axis]), 65);
            if (segment.coast) {
                EXPECT_FALSE(sample.servo[axis]);
                EXPECT_DOUBLE_EQ(sample.velocity_target[axis], 0);
                EXPECT_DOUBLE_EQ(sample.probe_torque[axis], 0);
            }
        }
    }
    EXPECT_EQ(peaks, 4);
    EXPECT_EQ(train, 4);
    EXPECT_EQ(heldout, 4);
    for (auto side : covered)
        EXPECT_TRUE(side[0] && side[1]);
    EXPECT_DOUBLE_EQ(segments[2].start_s, 11);
}

TEST(FrequencySweepWheels, Real2simCoversFourConditionsAndHeldoutWithinTenMinutes) {
    auto c = wheel_config();
    c.high_speed_target = 65;
    c.real2sim = true;
    const WheelProbePlan plan(c);
    EXPECT_DOUBLE_EQ(plan.duration(), 555);
    EXPECT_DOUBLE_EQ(plan.max_target_speed(), 65);
    EXPECT_NE(plan.manifest_json().find("wheel_real2sim_v1"), std::string::npos);
    int platforms = 0, peaks = 0, coasts = 0, training = 0, validation = 0, both = 0;
    std::array<std::array<bool, 2>, 2> covered{};
    for (const auto& segment : plan.segments()) {
        if (segment.label.ends_with("platform_hold"))
            ++platforms;
        if (segment.label.ends_with("peak_plateau")) {
            ++peaks;
            covered[segment.axis_scale[0] ? 0 : 1][segment.to > 0 ? 0 : 1] = true;
        }
        if (segment.label.ends_with("peak_coast")) {
            ++coasts;
            EXPECT_TRUE(segment.coast);
        }
        if (segment.waveform == 9) {
            segment.validation ? ++validation : ++training;
            EXPECT_DOUBLE_EQ(segment.duration_s, 24);
            EXPECT_DOUBLE_EQ(segment.phase, segment.validation ? 1.27 : .31);
            EXPECT_DOUBLE_EQ(std::abs(segment.to), 35.75);
        }
        if (segment.axis_scale[0] != 0 && segment.axis_scale[1] != 0) {
            ++both;
            EXPECT_TRUE(segment.validation);
        }
        for (double t = 0; t < segment.duration_s; t += .017) {
            const auto sample = plan.at(segment.start_s + t);
            for (std::size_t axis = 0; axis < 2; ++axis) {
                EXPECT_LE(std::abs(sample.velocity_target[axis]), 65 + 1e-12);
                EXPECT_LE(std::abs(sample.probe_torque[axis]), .35 + 1e-12);
                if (segment.coast || segment.axis_scale[axis] == 0) {
                    EXPECT_FALSE(sample.servo[axis]);
                    EXPECT_DOUBLE_EQ(sample.velocity_target[axis], 0);
                    EXPECT_DOUBLE_EQ(sample.probe_torque[axis], 0);
                }
            }
        }
    }
    EXPECT_EQ(platforms, 12);
    EXPECT_EQ(peaks, 4);
    EXPECT_EQ(coasts, 4);
    EXPECT_EQ(training, 4);
    EXPECT_EQ(validation, 4);
    EXPECT_EQ(both, 8);
    for (const auto& side : covered)
        EXPECT_TRUE(side[0] && side[1]);
    c.high_speed_only = true;
    EXPECT_THROW(WheelProbePlan{c}, std::invalid_argument);
    c.high_speed_only = false;
    c.coast_s = 60;
    EXPECT_THROW(WheelProbePlan{c}, std::invalid_argument);
    c.coast_s = 3;
    c.high_speed_target = 0;
    EXPECT_THROW(WheelProbePlan{c}, std::invalid_argument);
}

TEST(FrequencySweepWheels, HighSpeedIsOptInAndFullSweepRetainsLowSpeedBand) {
    auto c = wheel_config();
    const WheelProbePlan legacy(c);
    EXPECT_DOUBLE_EQ(legacy.duration(), 931);
    EXPECT_DOUBLE_EQ(legacy.max_target_speed(), 20);
    EXPECT_NE(legacy.manifest_json().find("wheel_multiband_v2"), std::string::npos);
    EXPECT_NE(legacy.manifest_json().find("\"reference_frequency_hz\":50"), std::string::npos);
    c.high_speed_target = 65;
    const WheelProbePlan full(c);
    EXPECT_DOUBLE_EQ(full.duration(), 1619);
    EXPECT_DOUBLE_EQ(full.max_target_speed(), 65);
    EXPECT_NE(full.manifest_json().find("wheel_high_speed_v1"), std::string::npos);
    EXPECT_NE(full.manifest_json().find("\"reference_frequency_hz\":200"), std::string::npos);
    bool low_speed_found = false;
    for (const auto& segment : full.segments())
        low_speed_found = low_speed_found
                       || (segment.label.ends_with("low_steady") && std::abs(segment.to) == .15);
    EXPECT_TRUE(low_speed_found);
    c.high_speed_target = 71;
    EXPECT_THROW(WheelProbePlan{c}, std::invalid_argument);
    c.high_speed_target = 0;
    c.high_speed_only = true;
    EXPECT_THROW(WheelProbePlan{c}, std::invalid_argument);
    c.high_speed_target = 65;
    c.coast_s = 2000;
    EXPECT_THROW(WheelProbePlan{c}, std::invalid_argument);
}
} // namespace

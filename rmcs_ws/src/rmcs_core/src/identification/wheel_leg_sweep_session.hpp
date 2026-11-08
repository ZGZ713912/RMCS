#pragma once

#include <cstdint>

#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::identification {

enum class SweepState : int {
    kResetRequired = 0,
    kResetReady = 1,
    kStandby = 2,
    kRunning = 3,
    kComplete = 4,
    kInterrupted = 5,
    kFault = 6
};
enum class SweepStop : int {
    kNone = 0,
    kBothDown = 1,
    kRemoteLost = 2,
    kInvalidSwitch = 3,
    kOperator = 4,
    kModeChanged = 5,
    kFinished = 6,
    kFeedbackTimeout = 7,
    kNonfinite = 8,
    kDriveFault = 9,
    kEnableLost = 10
};

// Level-triggered remote handling with optional completion hold for wheel real2sim.
class WheelLegSweepSession {
public:
    explicit WheelLegSweepSession(
        bool joints, bool hold_completion = false, int fixed_joint_mode = 0)
        : joints_(joints)
        , hold_completion_(hold_completion)
        , fixed_joint_mode_(fixed_joint_mode) {}

    bool update(
        double now, bool remote_fresh, rmcs_msgs::Switch left, rmcs_msgs::Switch right,
        SweepStop health) {
        using rmcs_msgs::Switch;
        if (!remote_fresh || left == Switch::UNKNOWN || right == Switch::UNKNOWN) {
            fault(SweepStop::kRemoteLost);
            return false;
        }
        if (left == Switch::DOWN && right == Switch::DOWN) {
            completion_latched_ = false;
            state = SweepState::kResetReady;
            reason = SweepStop::kBothDown;
            mode = 0;
            completed = false;
            return false;
        }
        const bool middle_now = left == Switch::MIDDLE && right == Switch::MIDDLE;
        if (middle_now && !fixed_joint_mode_)
            completion_latched_ = false;
        if (fixed_joint_mode_ && !middle_now) {
            state = SweepState::kStandby;
            reason = SweepStop::kOperator;
            mode = 0;
            completed = false;
            return false;
        }
        if (health != SweepStop::kNone) {
            fault(health);
            return false;
        }
        if (completion_latched_) {
            state = SweepState::kComplete;
            reason = SweepStop::kFinished;
            completed = true;
            return false;
        }
        const int selected = fixed_joint_mode_ ? fixed_joint_mode_
                           : joints_ ? (left == Switch::UP && right == Switch::MIDDLE   ? 1
                                        : left == Switch::MIDDLE && right == Switch::UP ? 2
                                        : left == Switch::UP && right == Switch::UP     ? 3
                                                                                        : 0)
                                     : (middle_now ? 0 : 3);
        if (!selected) {
            reason = state == SweepState::kRunning ? SweepStop::kOperator : SweepStop::kNone;
            state = SweepState::kStandby;
            mode = 0;
            completed = false;
            return false;
        }
        if (state == SweepState::kRunning && mode == selected)
            return false;
        mode = selected;
        ++run_id;
        start_time = now;
        completed = false;
        reason = SweepStop::kNone;
        state = SweepState::kRunning;
        return true;
    }

    void complete() {
        completion_latched_ = hold_completion_;
        state = SweepState::kComplete;
        completed = true;
        reason = SweepStop::kFinished;
    }

    void fault(SweepStop why) {
        state = SweepState::kFault;
        reason = why;
        completed = false;
    }

    bool enabled() const {
        return state == SweepState::kStandby || state == SweepState::kRunning
            || state == SweepState::kComplete || state == SweepState::kInterrupted;
    }
    SweepState state = SweepState::kResetRequired;
    SweepStop reason = SweepStop::kNone;
    int mode = 0;
    std::uint64_t run_id = 0;
    double start_time = 0;
    bool completed = false;

private:
    bool joints_;
    bool hold_completion_ = false, completion_latched_ = false;
    int fixed_joint_mode_ = 0;
};

} // namespace rmcs_core::controller::identification

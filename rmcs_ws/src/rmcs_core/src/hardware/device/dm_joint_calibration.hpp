#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace rmcs_core::hardware::device {

// Explicit FE requests only. This verifies feedback at the requested pose; it
// cannot establish the physical reference or prove persistence across power loss.
class DmJointCalibration {
public:
    using Clock = std::chrono::steady_clock;
    enum class Phase { kNotRequested, kWaiting, kConfirmed, kFailed };
    struct Feedback {
        bool fresh = false;
        int status = 0;
        double raw_phase = 0.0;
        double velocity = 0.0;
        Clock::time_point received_at{};
    };

    bool request(bool enable_requested, const std::array<Feedback, 4>& feedback,
                 Clock::time_point now) {
        if (enable_requested || phase_ == Phase::kWaiting || !stationary_disabled_(feedback))
            return false;
        *this = DmJointCalibration{};
        phase_ = Phase::kWaiting;
        started_ = next_send_ = now;
        return true;
    }

    // Returns the FE mask in LH,LK,RH,RK order. One motor per CAN bus per slot,
    // once per explicit request; never retry a persistent zero write silently.
    std::uint8_t update(const std::array<Feedback, 4>& feedback, Clock::time_point now) {
        if (phase_ != Phase::kWaiting)
            return 0;
        if (now - started_ >= std::chrono::seconds{2}) {
            phase_ = Phase::kFailed;
            return 0;
        }
        if (sent_mask_ != 0xf) {
            if (!stationary_disabled_(feedback)) {
                phase_ = Phase::kFailed;
                return 0;
            }
            if (now < next_send_)
                return 0;
            const std::uint8_t mask = sent_mask_ == 0 ? 0x3 : 0xc;
            for (std::size_t i = 0; i < feedback.size(); ++i)
                if (mask & (1u << i))
                    sent_at_[i] = now;
            sent_mask_ |= mask;
            next_send_ = now + std::chrono::milliseconds{5};
            return mask;
        }

        zero_mask_ = 0;
        for (std::size_t i = 0; i < feedback.size(); ++i) {
            const auto& motor = feedback[i];
            if (motor.fresh && motor.status == 0 && std::isfinite(motor.raw_phase)
                && std::abs(motor.raw_phase) <= 0.01 && std::isfinite(motor.velocity)
                && std::abs(motor.velocity) <= 0.05
                && motor.received_at >= sent_at_[i] + std::chrono::milliseconds{50})
                zero_mask_ |= 1u << i;
        }
        if (zero_mask_ != 0xf) {
            confirming_ = false;
            return 0;
        }
        if (!confirming_) {
            stable_since_ = now;
            confirming_ = true;
        }
        for (const auto& motor : feedback)
            if (motor.received_at < stable_since_ + std::chrono::milliseconds{100})
                return 0;
        phase_ = Phase::kConfirmed;
        return 0;
    }

    Phase phase() const { return phase_; }
    bool blocks_control() const { return phase_ == Phase::kWaiting || phase_ == Phase::kFailed; }
    std::uint8_t sent_mask() const { return sent_mask_; }
    std::uint8_t zero_mask() const { return zero_mask_; }
    const char* phase_name() const {
        switch (phase_) {
        case Phase::kNotRequested: return "not_requested_in_RMCS";
        case Phase::kWaiting: return "waiting_for_zero_feedback";
        case Phase::kConfirmed: return "zero_feedback_confirmed";
        case Phase::kFailed: return "zero_feedback_failed";
        }
        return "unknown";
    }

private:
    static bool stationary_disabled_(const std::array<Feedback, 4>& feedback) {
        for (const auto& motor : feedback)
            if (!motor.fresh || motor.status != 0 || !std::isfinite(motor.velocity)
                || std::abs(motor.velocity) > 0.05 || !std::isfinite(motor.raw_phase))
                return false;
        return true;
    }

    Phase phase_ = Phase::kNotRequested;
    Clock::time_point started_{};
    Clock::time_point next_send_{};
    Clock::time_point stable_since_{};
    std::array<Clock::time_point, 4> sent_at_{};
    std::uint8_t sent_mask_ = 0;
    std::uint8_t zero_mask_ = 0;
    bool confirming_ = false;
};

} // namespace rmcs_core::hardware::device

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rmcs_core::controller::identification {

class WheelLegBreakawayProbe {
public:
    void
        start(double now, double angle, double velocity, double error, double bias, int direction) {
        origin = angle;
        bias_nm = bias;
        direction_ = direction;
        start_time_ = now;
        last_sequence_ = 0;
        moving_frames_ = 0;
        increment_nm = 0;
        phase = std::abs(velocity) <= .02 && std::abs(error) <= .02 ? 1 : 4;
        event_elapsed_s = -1;
    }

    void update(double now, double angle, double velocity, std::uint64_t sequence) {
        if (phase != 1)
            return;
        const double elapsed = now - start_time_;
        if (sequence != last_sequence_) {
            last_sequence_ = sequence;
            moving_frames_ = direction_ * velocity > .04 ? moving_frames_ + 1 : 0;
            if (direction_ * (angle - origin) >= .003 || moving_frames_ >= 3) {
                phase = 2;
                event_elapsed_s = elapsed;
            } else if (direction_ * (angle - origin) <= -.003 || direction_ * velocity < -.04) {
                phase = 5;
                event_elapsed_s = elapsed;
            }
        }
        if (phase == 1 && elapsed >= 6) {
            phase = 3;
            event_elapsed_s = elapsed;
        }
        increment_nm = phase == 1 ? direction_ * std::clamp(elapsed, 0.0, 6.0) : 0;
    }

    void reset() { *this = {}; }

    double origin = 0, bias_nm = 0, increment_nm = 0, event_elapsed_s = -1;
    int phase = 0;

private:
    double start_time_ = 0;
    int direction_ = 0, moving_frames_ = 0;
    std::uint64_t last_sequence_ = 0;
};

} // namespace rmcs_core::controller::identification

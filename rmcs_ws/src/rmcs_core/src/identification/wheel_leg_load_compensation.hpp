#pragma once

#include <algorithm>
#include <cmath>

namespace rmcs_core::controller::identification {

class WheelLegLoadCompensation {
public:
    double update(
        double error, double effort, double cap, double dt, double ki, double limit, double rate,
        bool velocity_limited) {
        if (!std::isfinite(error) || !std::isfinite(effort) || !std::isfinite(cap)
            || !std::isfinite(dt) || dt <= 0 || dt > .01) {
            reset();
            return 0;
        }
        const double bound = std::min(limit, cap);
        value_ = std::clamp(value_, -bound, bound);
        const double increment = std::clamp(ki * error, -rate, rate) * dt;
        const double total = effort + value_;
        const bool unloading = increment * value_ < 0;
        const bool drives_saturation =
            (total >= cap && increment > 0) || (total <= -cap && increment < 0);
        if ((!velocity_limited || unloading) && !drives_saturation)
            value_ = std::clamp(value_ + increment, -bound, bound);
        return value_;
    }

    void reset() { value_ = 0; }
    double value() const { return value_; }

private:
    double value_ = 0;
};

} // namespace rmcs_core::controller::identification

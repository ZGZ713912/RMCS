#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace rmcs_core::controller::identification {

class WheelLegLatencyPlan {
public:
    struct Sample {
        int stage = -1;
        bool validation = false, active = false, completed = false;
        double probe = 0;
    };

    WheelLegLatencyPlan(double amplitude, double bit_s, double segment_s)
        : amplitude_(amplitude)
        , bit_s_(bit_s)
        , segment_s_(segment_s) {
        if (!std::isfinite(amplitude) || amplitude <= 0 || !std::isfinite(bit_s) || bit_s < .002
            || !std::isfinite(segment_s) || segment_s < 2.)
            throw std::invalid_argument("Invalid latency excitation parameters");
    }

    Sample at(double elapsed) const {
        Sample sample;
        if (elapsed < 2.)
            return sample;
        const double local = elapsed - 2.;
        sample.stage = static_cast<int>(local / (segment_s_ + 1.));
        sample.completed = sample.stage >= 6;
        if (sample.completed)
            return sample;
        sample.validation = sample.stage >= 3;
        const double phase = local - sample.stage * (segment_s_ + 1.);
        sample.active = phase >= .5 && phase < segment_s_ + .5;
        if (!sample.active)
            return sample;
        const double t = phase - .5;
        const double scale = std::array{.5, .75, 1.}[sample.stage % 3];
        if (t < 1.) {
            constexpr std::array widths{.005, .01, .02, .05};
            const auto pulse = static_cast<int>(t / .125);
            sample.probe = std::fmod(t, .125) < widths[pulse % 4]
                             ? ((pulse / 4) % 2 ? -1. : 1.) * amplitude_ * scale
                             : 0.;
        } else {
            // Hashing the bit index makes the waveform independent of skipped host ticks.
            auto bits = static_cast<std::uint64_t>((t - 1.) / bit_s_)
                      + (sample.validation ? 0x51ed2705ULL : 0x9e3779b9ULL);
            bits = (bits ^ (bits >> 30)) * 0xbf58476d1ce4e5b9ULL;
            bits = (bits ^ (bits >> 27)) * 0x94d049bb133111ebULL;
            sample.probe = ((bits ^ (bits >> 31)) & 1 ? 1. : -1.) * amplitude_ * scale;
        }
        return sample;
    }

private:
    double amplitude_, bit_s_, segment_s_;
};

} // namespace rmcs_core::controller::identification

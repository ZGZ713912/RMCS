#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>

namespace rmcs_core::controller::chassis {

struct WheelLegJointPair {
    double hip = 0.0;
    double knee = 0.0;
};

// S1 x [d_min, d_max]: only the common orientation is circular. The opening
// stays on the V5 assembly branch, without any encoder turn state.
struct WheelLegPairPose {
    double orientation = 0.0;
    double difference = 0.0;
};

class WheelLegJointPairGeometry {
public:
    enum class Side { kLeft, kRight };
    static constexpr double kPeriod = 2.0 * std::numbers::pi;
    // URDF zero and the retained DM-zero offsets bound the real 45--105 deg
    // working range: d=0 and d=1.33. PhysX axis metrology gives 44.934/105.021
    // deg at these anchors; do not confuse this range with all valid assemblies.
    static constexpr double kDefaultMinDifference = 0.0;
    static constexpr double kDefaultMaxDifference = 1.33;

    static double wrap(double angle) { return std::remainder(angle, kPeriod); }
    static double sign(Side side) { return side == Side::kLeft ? 1.0 : -1.0; }

    static WheelLegPairPose decode(Side side, double hip, double knee) {
        const double d = wrap(sign(side) * (hip - knee));
        return {wrap(hip - sign(side) * d / 2.0), d};
    }

    static std::optional<WheelLegPairPose>
        feedback(Side side, double hip, double knee, double min, double max, double tolerance) {
        return feedback(side, hip, knee, min, max, tolerance, tolerance);
    }

    static std::optional<WheelLegPairPose> feedback(
        Side side, double hip, double knee, double min, double max, double lower_tolerance,
        double upper_tolerance) {
        if (!std::isfinite(hip) || !std::isfinite(knee))
            return std::nullopt;
        const auto pose = decode(side, hip, knee);
        if (pose.difference < min - lower_tolerance || pose.difference > max + upper_tolerance)
            return std::nullopt;
        return pose;
    }

    static WheelLegPairPose
        target(Side side, double hip, double knee, double min, double max, double margin) {
        auto pose = decode(side, hip, knee);
        pose.difference = std::clamp(pose.difference, min + margin, max - margin);
        return pose;
    }

    static WheelLegJointPair motors(Side side, const WheelLegPairPose& pose) {
        return {
            wrap(pose.orientation + sign(side) * pose.difference / 2.0),
            wrap(pose.orientation - sign(side) * pose.difference / 2.0)};
    }

    // Diagnostic only: hinge-plane axis measurements in the PhysX closed chain.
    // No extrapolation outside the calibrated range. Control uses d directly.
    static double inner_angle_degrees(double difference) {
        constexpr std::array<double, 15> kDifference{
            0.0,         0.001799166, 0.134191513, 0.259151399, 0.378464520,
            0.493397772, 0.604862034, 0.713550448, 0.819977403, 0.924540877,
            1.027554631, 1.129269719, 1.229888678, 1.329574585, 1.33};
        constexpr std::array<double, 15> kAngle{44.93430, 45., 50., 55., 60.,  65.,  70.,      75.,
                                                80.,      85., 90., 95., 100., 105., 105.02142};
        if (!std::isfinite(difference) || difference < kDifference.front()
            || difference > kDifference.back())
            return std::numeric_limits<double>::quiet_NaN();
        const auto upper = std::upper_bound(kDifference.begin(), kDifference.end(), difference);
        const auto index = std::clamp<std::ptrdiff_t>(upper - kDifference.begin(), 1, 14);
        const double fraction =
            (difference - kDifference[index - 1]) / (kDifference[index] - kDifference[index - 1]);
        return kAngle[index - 1] + fraction * (kAngle[index] - kAngle[index - 1]);
    }
};

struct WheelLegPairVelocityConfig {
    double angle_kp = 6.0;
    double max_velocity = 2.0;
    double max_acceleration = 4.0;
    double min_difference = WheelLegJointPairGeometry::kDefaultMinDifference;
    double max_difference = WheelLegJointPairGeometry::kDefaultMaxDifference;
};

// Shared by the RMCS component and the PhysX harness. No position reference
// advances independently while the hardware cannot follow it.
inline WheelLegJointPair wheel_leg_pair_velocity(
    WheelLegJointPairGeometry::Side side, const WheelLegPairPose& measured,
    const WheelLegPairPose& target, const WheelLegJointPair& previous,
    const WheelLegPairVelocityConfig& config, double dt) {
    using Geometry = WheelLegJointPairGeometry;
    double error = Geometry::wrap(target.orientation - measured.orientation);
    const double previous_common = (previous.hip + previous.knee) / 2.0;
    // Retain the selected route near the antipode despite encoder quantization.
    if (std::abs(error) > std::numbers::pi - 0.1 && error * previous_common < 0.0)
        error += std::copysign(Geometry::kPeriod, previous_common);
    // A slew limit alone brakes only after the proportional command falls,
    // which can overshoot the target by a large angle at high gains. Reserve
    // half the motor acceleration for common rotation and half for half-opening:
    // |a_common| + |a_half_opening| <= a_motor. Include one command interval in
    // the stopping distance, then retain the proportional law close to rest.
    const auto approach_speed = [&](double distance) {
        const double acceleration = config.max_acceleration / 2.0;
        const double interval = acceleration * dt;
        const double braking =
            std::sqrt(interval * interval + 2.0 * acceleration * std::abs(distance)) - interval;
        return std::copysign(std::min(config.angle_kp * std::abs(distance), braking), distance);
    };
    const double common = approach_speed(error);
    const double differential =
        2.0 * approach_speed((target.difference - measured.difference) / 2.0);
    const auto limit_speed = [&](WheelLegJointPair value) {
        const double scale = std::max(
            1.0, std::max(std::abs(value.hip), std::abs(value.knee)) / config.max_velocity);
        return WheelLegJointPair{value.hip / scale, value.knee / scale};
    };
    const auto desired = limit_speed(
        {common + Geometry::sign(side) * differential / 2.0,
         common - Geometry::sign(side) * differential / 2.0});
    const double change =
        std::max(std::abs(desired.hip - previous.hip), std::abs(desired.knee - previous.knee));
    const double fraction =
        change > 0.0 ? std::min(1.0, config.max_acceleration * dt / change) : 1.0;
    const WheelLegJointPair command{
        previous.hip + fraction * (desired.hip - previous.hip),
        previous.knee + fraction * (desired.knee - previous.knee)};

    // Stop outward motion at the mechanical boundary, before the normal ramp.
    const double shape_velocity = std::clamp(
        Geometry::sign(side) * (command.hip - command.knee),
        2.0 * approach_speed(std::min(0.0, config.min_difference - measured.difference) / 2.0),
        2.0 * approach_speed(std::max(0.0, config.max_difference - measured.difference) / 2.0));
    const double common_velocity = (command.hip + command.knee) / 2.0;
    return limit_speed(
        {common_velocity + Geometry::sign(side) * shape_velocity / 2.0,
         common_velocity - Geometry::sign(side) * shape_velocity / 2.0});
}

} // namespace rmcs_core::controller::chassis

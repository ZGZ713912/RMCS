#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>

#include <rclcpp/node.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class DeformableChassisActiveSuspension {
public:
    struct Corrections {
        std::array<double, 4> joint_angle_correction        = {0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> joint_velocity_correction     = {0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> joint_acceleration_correction = {0.0, 0.0, 0.0, 0.0};
        std::array<bool, 4> joint_correction_active         = {false, false, false, false};
    };

    void configure(rclcpp::Node& node) {
        load_pid_(
            node, "active_suspension_pitch_outer_", pitch_outer_pid_, //
            8.0, 0.35, 0.28, -2.0, 2.0, -3.0, 3.0);
        load_pid_(
            node, "active_suspension_pitch_inner_", pitch_inner_pid_, //
            2.0, 0.0, 0.0, -1.0, 1.0, -0.785, 0.785);
        load_pid_(
            node, "active_suspension_roll_outer_", roll_outer_pid_,   //
            8.0, 0.35, 0.28, -2.0, 2.0, -3.0, 3.0);
        load_pid_(
            node, "active_suspension_roll_inner_", roll_inner_pid_,   //
            2.0, 0.0, 0.0, -1.0, 1.0, -0.785, 0.785);

        active_correction_vel_limit_ = std::max(
            deg_to_rad_(
                std::abs(node.get_parameter_or(
                    "active_suspension_correction_velocity_limit_deg", 720.0))),
            1e-6);
        active_correction_acc_limit_ = std::max(
            deg_to_rad_(
                std::abs(node.get_parameter_or(
                    "active_suspension_correction_acceleration_limit_deg", 3600.0))),
            1e-6);

        passive_enabled_ = node.get_parameter_or("passive_suspension_enable", true);
        passive_support_margin_deadband_ = std::max(
            node.get_parameter_or("passive_suspension_support_margin_deadband", 0.08), 0.0);
        passive_support_unload_threshold_ = std::max(
            node.get_parameter_or("passive_suspension_support_unload_threshold", 0.18), 0.0);
        passive_unload_persistence_time_ = std::max(
            node.get_parameter_or("passive_suspension_unload_persistence_s", 0.06), 0.0);
        passive_impact_threshold_ = std::max(
            node.get_parameter_or("passive_suspension_impact_threshold", 8.0), 1e-6);
        passive_support_lpf_alpha_ = std::clamp(
            node.get_parameter_or("passive_suspension_support_lpf_alpha", 0.08), 0.0, 1.0);
        passive_payload_lpf_alpha_ = std::clamp(
            node.get_parameter_or("passive_suspension_payload_lpf_alpha", 0.02), 0.0, 1.0);
        passive_support_pitch_comp_gain_ = node.get_parameter_or(
            "passive_suspension_support_pitch_comp_gain", 0.6);
        passive_support_roll_comp_gain_ = node.get_parameter_or(
            "passive_suspension_support_roll_comp_gain", 0.6);
        passive_support_angle_comp_gain_ = node.get_parameter_or(
            "passive_suspension_support_angle_comp_gain", 0.25);
        passive_support_lever_arm_min_ = std::clamp(
            node.get_parameter_or("passive_suspension_support_lever_arm_min", 0.25), 1e-3, 1.0);
        passive_mode_gain_[kModeCommon] =
            node.get_parameter_or("passive_suspension_common_gain", 0.10);
        passive_mode_gain_[kModePitch] =
            node.get_parameter_or("passive_suspension_pitch_gain", 0.16);
        passive_mode_gain_[kModeRoll] =
            node.get_parameter_or("passive_suspension_roll_gain", 0.16);
        passive_mode_gain_[kModeTwist] =
            node.get_parameter_or("passive_suspension_twist_gain", 0.22);
        passive_mode_integral_gain_[kModeCommon] =
            node.get_parameter_or("passive_suspension_common_integral_gain", 0.005);
        passive_mode_integral_gain_[kModePitch] =
            node.get_parameter_or("passive_suspension_pitch_integral_gain", 0.008);
        passive_mode_integral_gain_[kModeRoll] =
            node.get_parameter_or("passive_suspension_roll_integral_gain", 0.008);
        passive_mode_integral_gain_[kModeTwist] =
            node.get_parameter_or("passive_suspension_twist_integral_gain", 0.010);
        passive_mode_integral_limit_ = std::max(
            node.get_parameter_or("passive_suspension_mode_integral_limit", 2.0), 0.0);
        passive_payload_common_gain_ = std::max(
            node.get_parameter_or("passive_suspension_payload_common_gain", 0.08), 0.0);
        passive_clearance_margin_rad_ = std::max(
            deg_to_rad_(std::abs(node.get_parameter_or(
                "passive_suspension_clearance_margin_deg", 10.0))),
            0.0);
        passive_clearance_recovery_gain_ = std::max(
            node.get_parameter_or("passive_suspension_clearance_recovery_gain", 0.9), 0.0);
        passive_clearance_recovery_integral_gain_ = std::max(
            node.get_parameter_or("passive_suspension_clearance_recovery_integral_gain", 0.05),
            0.0);
        passive_trusted_body_rate_threshold_rad_ = std::max(
            deg_to_rad_(std::abs(node.get_parameter_or(
                "passive_suspension_trusted_body_rate_threshold_deg", 18.0))),
            1e-6);
        passive_fast_recovery_gain_ = std::max(
            node.get_parameter_or("passive_suspension_fast_recovery_gain", 0.35), 0.0);
        passive_max_correction_rad_ = std::max(
            deg_to_rad_(
                std::abs(node.get_parameter_or("passive_suspension_max_angle_correction_deg", 8.0))),
            1e-6);
        passive_correction_vel_limit_ = std::max(
            deg_to_rad_(
                std::abs(node.get_parameter_or(
                    "passive_suspension_correction_velocity_limit_deg", 180.0))),
            1e-6);
        passive_correction_acc_limit_ = std::max(
            deg_to_rad_(
                std::abs(node.get_parameter_or(
                    "passive_suspension_correction_acceleration_limit_deg", 720.0))),
            1e-6);
        passive_fast_recovery_vel_limit_ = std::max(
            deg_to_rad_(std::abs(node.get_parameter_or(
                "passive_suspension_fast_recovery_velocity_limit_deg", 360.0))),
            passive_correction_vel_limit_);
        passive_fast_recovery_acc_limit_ = std::max(
            deg_to_rad_(std::abs(node.get_parameter_or(
                "passive_suspension_fast_recovery_acceleration_limit_deg", 1440.0))),
            passive_correction_acc_limit_);

        passive_load_sign_[kLeftFront] =
            node.get_parameter_or("passive_suspension_load_sign_left_front", -1.0);
        passive_load_sign_[kLeftBack] =
            node.get_parameter_or("passive_suspension_load_sign_left_back", -1.0);
        passive_load_sign_[kRightBack] =
            node.get_parameter_or("passive_suspension_load_sign_right_back", -1.0);
        passive_load_sign_[kRightFront] =
            node.get_parameter_or("passive_suspension_load_sign_right_front", -1.0);

        passive_load_bias_[kLeftFront] =
            node.get_parameter_or("passive_suspension_load_bias_left_front", 0.0);
        passive_load_bias_[kLeftBack] =
            node.get_parameter_or("passive_suspension_load_bias_left_back", 0.0);
        passive_load_bias_[kRightBack] =
            node.get_parameter_or("passive_suspension_load_bias_right_back", 0.0);
        passive_load_bias_[kRightFront] =
            node.get_parameter_or("passive_suspension_load_bias_right_front", 0.0);

        calibration_wait_time_ =
            std::max(node.get_parameter_or("chassis_imu_calibration_wait_s", 2.0), 0.0);
        calibration_sample_time_ =
            std::max(node.get_parameter_or("chassis_imu_calibration_sample_s", 3.0), 1e-6);
    }

    void calibrate(double pitch, double roll, bool symmetric_target, double dt) {

        if (calibrated_once_)
            return;

        if (!symmetric_target) {
            reset_calibration_window_();
            return;
        }

        if (!std::isfinite(pitch) || !std::isfinite(roll))
            return;

        calibration_hold_elapsed_ += dt;
        if (calibration_hold_elapsed_ < calibration_wait_time_)
            return;

        double calibration_end = calibration_wait_time_ + calibration_sample_time_;
        if (calibration_hold_elapsed_ < calibration_end) {
            pitch_sum_ += pitch;
            roll_sum_ += roll;
            ++sample_count_;
            return;
        }

        if (calibration_completed_for_window_)
            return;

        calibration_completed_for_window_ = true;
        if (sample_count_ == 0)
            return;

        pitch_offset_ = std::clamp(
            pitch_sum_ / static_cast<double>(sample_count_), -offset_limit_rad_, offset_limit_rad_);
        roll_offset_ = std::clamp(
            roll_sum_ / static_cast<double>(sample_count_), -offset_limit_rad_, offset_limit_rad_);
        calibrated_once_ = true;
    }

    bool calibrated() const { return calibrated_once_; }
    double pitch_offset() const { return pitch_offset_; }
    double roll_offset() const { return roll_offset_; }

    double scope_torque(bool suspension_active, bool is_spin) const {
        if (suspension_active && !is_spin)
            return -0.3;
        return 0.3;
    }

    Corrections update(
        double pitch, double roll, double pitch_rate, double roll_rate, bool suspension_active,
        bool passive_suspension_active, bool low_prone_override_active, double min_angle_deg,
        double max_angle_deg, double base_angle_deg, bool correction_inverted,
        const std::array<double, 4>& base_joint_angles,
        const std::array<double, 4>& current_joint_angles,
        const std::array<double, 4>& joint_torques, double dt) {

        Corrections corrections;

        if (passive_suspension_active && passive_enabled_) {
            reset_attitude_();
            update_passive_targets_(
                pitch, roll, pitch_rate, roll_rate, min_angle_deg, current_joint_angles,
                joint_torques, dt);
            run_correction_trajectory_(
                low_prone_override_active, min_angle_deg, max_angle_deg, base_angle_deg,
                base_joint_angles,
                passive_fast_recovery_active_ || passive_clearance_recovery_active_
                    ? passive_fast_recovery_vel_limit_
                    : passive_correction_vel_limit_,
                passive_fast_recovery_active_ || passive_clearance_recovery_active_
                    ? passive_fast_recovery_acc_limit_
                    : passive_correction_acc_limit_,
                dt,
                corrections);
            return corrections;
        }

        reset_passive_state_();

        if (!suspension_active) {
            reset_attitude_();
            run_correction_trajectory_(
                low_prone_override_active, min_angle_deg, max_angle_deg, base_angle_deg,
                base_joint_angles, active_correction_vel_limit_, active_correction_acc_limit_, dt,
                corrections);
            return corrections;
        }

        constexpr double max_attitude = 30.0 * std::numbers::pi / 180.0;
        double clamped_pitch          = std::clamp(pitch, -max_attitude, max_attitude);
        double clamped_roll           = std::clamp(roll, -max_attitude, max_attitude);

        double pitch_outer = pitch_outer_pid_.update(-clamped_pitch);
        double roll_outer  = roll_outer_pid_.update(clamped_roll);
        double pitch_diff  = pitch_inner_pid_.update(pitch_outer - pitch_rate);
        double roll_diff   = roll_inner_pid_.update(roll_outer + roll_rate);

        if (!std::isfinite(pitch_diff) || !std::isfinite(roll_diff)) {
            reset_attitude_();
            return corrections;
        }

        compute_correction_targets_(pitch_diff, roll_diff, correction_inverted);
        run_correction_trajectory_(
            low_prone_override_active, min_angle_deg, max_angle_deg, base_angle_deg,
            base_joint_angles, active_correction_vel_limit_, active_correction_acc_limit_, dt,
            corrections);
        return corrections;
    }

    void reset() {
        pitch_outer_pid_.reset();
        pitch_inner_pid_.reset();
        roll_outer_pid_.reset();
        roll_inner_pid_.reset();
        correction_target_rad_.fill(0.0);
        correction_state_rad_.fill(0.0);
        correction_velocity_state_rad_.fill(0.0);
        correction_acceleration_state_rad_.fill(0.0);
        reset_passive_state_();
        reset_calibration_window_();
    }

private:
    static constexpr double offset_limit_rad_ = 1.0 * std::numbers::pi / 180.0;

    static double deg_to_rad_(double deg) { return deg * std::numbers::pi / 180.0; }

    void load_pid_(
        rclcpp::Node& node, const std::string& prefix, pid::PidCalculator& pid, double kp_default,
        double ki_default, double kd_default, double integral_min_default,
        double integral_max_default, double output_min_default, double output_max_default) {

        pid.kp           = node.get_parameter_or(prefix + "kp", kp_default);
        pid.ki           = node.get_parameter_or(prefix + "ki", ki_default);
        pid.kd           = node.get_parameter_or(prefix + "kd", kd_default);
        pid.integral_min = node.get_parameter_or(prefix + "integral_min", integral_min_default);
        pid.integral_max = node.get_parameter_or(prefix + "integral_max", integral_max_default);
        pid.output_min   = node.get_parameter_or(prefix + "output_min", output_min_default);
        pid.output_max   = node.get_parameter_or(prefix + "output_max", output_max_default);
    }

    void reset_attitude_() {
        pitch_outer_pid_.reset();
        pitch_inner_pid_.reset();
        roll_outer_pid_.reset();
        roll_inner_pid_.reset();
        correction_target_rad_.fill(0.0);
    }

    void reset_passive_state_() {
        passive_load_initialized_ = false;
        passive_payload_initialized_ = false;
        passive_fast_recovery_active_ = false;
        passive_clearance_recovery_active_ = false;
        passive_filtered_support_proxy_.fill(0.0);
        passive_previous_support_proxy_.fill(0.0);
        passive_unload_timer_.fill(0.0);
        passive_mode_integral_.fill(0.0);
        passive_payload_state_ = 0.0;
        passive_payload_reference_ = 0.0;
        passive_clearance_integral_ = 0.0;
    }

    void reset_calibration_window_() {
        calibration_hold_elapsed_         = 0.0;
        sample_count_                     = 0;
        pitch_sum_                        = 0.0;
        roll_sum_                         = 0.0;
        calibration_completed_for_window_ = false;
    }

    void compute_correction_targets_(double pitch_diff, double roll_diff, bool inverted) {

        if (inverted) {
            double front_pitch_contribution = std::max(pitch_diff, 0.0);
            double back_pitch_contribution  = std::max(-pitch_diff, 0.0);
            double left_roll_contribution   = std::max(-roll_diff, 0.0);
            double right_roll_contribution  = std::max(roll_diff, 0.0);
            correction_target_rad_[kLeftFront] =
                -(front_pitch_contribution + left_roll_contribution);
            correction_target_rad_[kLeftBack] = -(back_pitch_contribution + left_roll_contribution);
            correction_target_rad_[kRightBack] =
                -(back_pitch_contribution + right_roll_contribution);
            correction_target_rad_[kRightFront] =
                -(front_pitch_contribution + right_roll_contribution);
        } else {
            double front_pitch_contribution    = std::max(-pitch_diff, 0.0);
            double back_pitch_contribution     = std::max(pitch_diff, 0.0);
            double left_roll_contribution      = std::max(roll_diff, 0.0);
            double right_roll_contribution     = std::max(-roll_diff, 0.0);
            correction_target_rad_[kLeftFront] = front_pitch_contribution + left_roll_contribution;
            correction_target_rad_[kLeftBack]  = back_pitch_contribution + left_roll_contribution;
            correction_target_rad_[kRightBack] = back_pitch_contribution + right_roll_contribution;
            correction_target_rad_[kRightFront] =
                front_pitch_contribution + right_roll_contribution;
        }
    }

    void update_passive_targets_(
        double pitch, double roll, double pitch_rate, double roll_rate, double min_angle_deg,
        const std::array<double, 4>& current_joint_angles,
        const std::array<double, 4>& joint_torques, double dt) {
        if (dt <= 0.0 || !std::isfinite(dt)) {
            correction_target_rad_.fill(0.0);
            return;
        }

        for (size_t i = 0; i < kJointCount; ++i) {
            if (!std::isfinite(joint_torques[i]) || !std::isfinite(current_joint_angles[i])) {
                correction_target_rad_.fill(0.0);
                reset_passive_state_();
                return;
            }
        }

        std::array<double, kJointCount> support_proxy{};
        std::array<double, kJointCount> support_deficit{};
        std::array<double, kJointCount> fast_deficit{};
        const double min_travel_reference = deg_to_rad_(min_angle_deg - 5.0);
        const double angle_mean = average_(current_joint_angles);
        const double clamped_pitch = std::clamp(pitch, -max_attitude_rad_, max_attitude_rad_);
        const double clamped_roll = std::clamp(roll, -max_attitude_rad_, max_attitude_rad_);
        bool impact_detected = false;

        for (size_t i = 0; i < kJointCount; ++i) {
            double support_model = passive_load_sign_[i] * (joint_torques[i] - passive_load_bias_[i]);
            support_model -= passive_support_pitch_comp_gain_ * kPitchWeights[i] * clamped_pitch;
            support_model -= passive_support_roll_comp_gain_ * kRollWeights[i] * clamped_roll;
            support_model -= passive_support_angle_comp_gain_ * (current_joint_angles[i] - angle_mean);

            double leverage =
                std::max(std::abs(std::cos(current_joint_angles[i])), passive_support_lever_arm_min_);
            support_proxy[i] = support_model / leverage;

            if (!passive_load_initialized_) {
                passive_filtered_support_proxy_[i] = support_proxy[i];
                passive_previous_support_proxy_[i] = support_proxy[i];
            } else {
                passive_filtered_support_proxy_[i] += passive_support_lpf_alpha_
                    * (support_proxy[i] - passive_filtered_support_proxy_[i]);
            }

            const double support_velocity =
                (passive_filtered_support_proxy_[i] - passive_previous_support_proxy_[i]) / dt;
            passive_previous_support_proxy_[i] = passive_filtered_support_proxy_[i];

            if (std::abs(support_velocity) > passive_impact_threshold_)
                impact_detected = true;
        }
        passive_load_initialized_ = true;

        const double common_support = average_(passive_filtered_support_proxy_);
        if (!passive_payload_initialized_) {
            passive_payload_state_ = common_support;
            passive_payload_reference_ = common_support;
            passive_payload_initialized_ = true;
        }

        const bool trusted_window =
            std::abs(pitch_rate) <= passive_trusted_body_rate_threshold_rad_
            && std::abs(roll_rate) <= passive_trusted_body_rate_threshold_rad_ && !impact_detected;

        if (trusted_window) {
            passive_payload_state_ += passive_payload_lpf_alpha_ * (common_support - passive_payload_state_);
            passive_payload_reference_ += 0.1 * passive_payload_lpf_alpha_
                * (passive_payload_state_ - passive_payload_reference_);
        }

        double min_travel_margin = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < kJointCount; ++i) {
            const double support_margin = common_support - passive_filtered_support_proxy_[i];
            support_deficit[i] = std::max(support_margin - passive_support_margin_deadband_, 0.0);

            if (support_margin > passive_support_unload_threshold_ && !impact_detected) {
                passive_unload_timer_[i] += dt;
            } else {
                passive_unload_timer_[i] = std::max(passive_unload_timer_[i] - 2.0 * dt, 0.0);
            }

            if (passive_unload_timer_[i] >= passive_unload_persistence_time_) {
                fast_deficit[i] = std::max(support_margin - passive_support_unload_threshold_, 0.0);
            }

            min_travel_margin = std::min(min_travel_margin, current_joint_angles[i] - min_travel_reference);
        }

        const auto slow_modes = project_modes_(support_deficit);
        if (trusted_window) {
            for (size_t mode = 0; mode < kModeCount; ++mode) {
                passive_mode_integral_[mode] = std::clamp(
                    passive_mode_integral_[mode] + slow_modes[mode] * dt,
                    -passive_mode_integral_limit_, passive_mode_integral_limit_);
            }
        }

        const double clearance_deficit =
            std::max(passive_clearance_margin_rad_ - min_travel_margin, 0.0);
        if (clearance_deficit > 0.0) {
            passive_clearance_integral_ = std::clamp(
                passive_clearance_integral_ + clearance_deficit * dt,
                0.0, passive_mode_integral_limit_);
        } else {
            passive_clearance_integral_ = std::max(passive_clearance_integral_ - 2.0 * dt, 0.0);
        }

        std::array<double, kModeCount> mode_commands{};
        for (size_t mode = 0; mode < kModeCount; ++mode) {
            mode_commands[mode] = passive_mode_gain_[mode] * slow_modes[mode]
                + passive_mode_integral_gain_[mode] * passive_mode_integral_[mode];
        }

        const double payload_excess = std::max(passive_payload_state_ - passive_payload_reference_, 0.0);
        mode_commands[kModeCommon] += passive_payload_common_gain_ * payload_excess;
        mode_commands[kModeCommon] += passive_clearance_recovery_gain_ * clearance_deficit;
        mode_commands[kModeCommon] +=
            passive_clearance_recovery_integral_gain_ * passive_clearance_integral_;

        const auto fast_modes = project_modes_(fast_deficit);
        for (size_t mode = 0; mode < kModeCount; ++mode)
            mode_commands[mode] += passive_fast_recovery_gain_ * fast_modes[mode];

        passive_fast_recovery_active_ = any_positive_(fast_deficit);
        passive_clearance_recovery_active_ = clearance_deficit > 1e-6 || payload_excess > 1e-6;

        const auto raw_targets = reconstruct_modes_(mode_commands);

        for (size_t i = 0; i < kJointCount; ++i) {
            correction_target_rad_[i] = std::clamp(raw_targets[i], 0.0, passive_max_correction_rad_);
        }
    }

    static double average_(const std::array<double, kJointCount>& values) {
        double sum = 0.0;
        for (double value : values)
            sum += value;
        return sum / static_cast<double>(kJointCount);
    }

    static bool any_positive_(const std::array<double, kJointCount>& values) {
        for (double value : values)
            if (value > 1e-6)
                return true;
        return false;
    }

    static std::array<double, kModeCount> project_modes_(
        const std::array<double, kJointCount>& values) {
        return {
            0.25 * (values[kLeftFront] + values[kLeftBack] + values[kRightBack] + values[kRightFront]),
            0.25 * (values[kLeftFront] + values[kRightFront] - values[kLeftBack] - values[kRightBack]),
            0.25 * (values[kLeftFront] + values[kLeftBack] - values[kRightFront] - values[kRightBack]),
            0.25 * (values[kLeftFront] + values[kRightBack] - values[kLeftBack] - values[kRightFront]),
        };
    }

    static std::array<double, kJointCount> reconstruct_modes_(
        const std::array<double, kModeCount>& modes) {
        return {
            modes[kModeCommon] + modes[kModePitch] + modes[kModeRoll] + modes[kModeTwist],
            modes[kModeCommon] - modes[kModePitch] + modes[kModeRoll] - modes[kModeTwist],
            modes[kModeCommon] - modes[kModePitch] - modes[kModeRoll] + modes[kModeTwist],
            modes[kModeCommon] + modes[kModePitch] - modes[kModeRoll] - modes[kModeTwist],
        };
    }

    void run_correction_trajectory_(
        bool low_prone_override_active, double min_angle_deg, double max_angle_deg,
        double base_angle_deg, const std::array<double, 4>& base_joint_angles,
        double correction_vel_limit, double correction_acc_limit, double dt, Corrections& corrections) {

        double max_target_rad = deg_to_rad_(max_angle_deg);
        double min_susp_rad   = deg_to_rad_(min_angle_deg - 5.0);

        for (size_t i = 0; i < kJointCount; ++i) {
            double base_angle = std::isfinite(base_joint_angles[i])
                                  ? base_joint_angles[i]
                                  : (low_prone_override_active ? min_susp_rad
                                                               : deg_to_rad_(base_angle_deg));

            double correction_min = min_susp_rad - base_angle;
            double correction_max = max_target_rad - base_angle;
            double target = std::clamp(correction_target_rad_[i], correction_min, correction_max);

            double& angle_state        = correction_state_rad_[i];
            double& velocity_state     = correction_velocity_state_rad_[i];
            double& acceleration_state = correction_acceleration_state_rad_[i];

            double position_error = target - angle_state;
            double stopping_distance = velocity_state * velocity_state / (2.0 * correction_acc_limit);

            double desired_velocity = 0.0;
            if (std::abs(position_error) > 1e-6 && std::abs(position_error) > stopping_distance) {
                desired_velocity = std::copysign(correction_vel_limit, position_error);
            }

            double velocity_error = desired_velocity - velocity_state;
            acceleration_state = std::clamp(velocity_error / dt, -correction_acc_limit, correction_acc_limit);

            velocity_state += acceleration_state * dt;
            velocity_state = std::clamp(velocity_state, -correction_vel_limit, correction_vel_limit);
            angle_state += velocity_state * dt;

            double next_error = target - angle_state;
            if ((position_error > 0.0 && next_error < 0.0)
                || (position_error < 0.0 && next_error > 0.0)
                || (std::abs(next_error) < 1e-5 && std::abs(velocity_state) < 1e-3)) {
                angle_state        = target;
                velocity_state     = 0.0;
                acceleration_state = 0.0;
            }

            corrections.joint_correction_active[i] = true;
        }

        corrections.joint_angle_correction        = correction_state_rad_;
        corrections.joint_velocity_correction     = correction_velocity_state_rad_;
        corrections.joint_acceleration_correction = correction_acceleration_state_rad_;
    }

    static constexpr size_t kLeftFront  = 0;
    static constexpr size_t kLeftBack   = 1;
    static constexpr size_t kRightBack  = 2;
    static constexpr size_t kRightFront = 3;
    static constexpr size_t kJointCount = 4;
    static constexpr size_t kModeCommon = 0;
    static constexpr size_t kModePitch  = 1;
    static constexpr size_t kModeRoll   = 2;
    static constexpr size_t kModeTwist  = 3;
    static constexpr size_t kModeCount  = 4;
    static constexpr double max_attitude_rad_ = 30.0 * std::numbers::pi / 180.0;
    static constexpr std::array<double, kJointCount> kPitchWeights = {1.0, -1.0, -1.0, 1.0};
    static constexpr std::array<double, kJointCount> kRollWeights  = {1.0, 1.0, -1.0, -1.0};

    pid::PidCalculator pitch_outer_pid_{};
    pid::PidCalculator pitch_inner_pid_{};
    pid::PidCalculator roll_outer_pid_{};
    pid::PidCalculator roll_inner_pid_{};

    double active_correction_vel_limit_ = 40.0;
    double active_correction_acc_limit_ = 200.0;
    bool passive_enabled_ = true;
    double passive_support_margin_deadband_ = 0.08;
    double passive_support_unload_threshold_ = 0.18;
    double passive_unload_persistence_time_ = 0.06;
    double passive_impact_threshold_ = 8.0;
    double passive_support_lpf_alpha_ = 0.08;
    double passive_payload_lpf_alpha_ = 0.02;
    double passive_support_pitch_comp_gain_ = 0.6;
    double passive_support_roll_comp_gain_ = 0.6;
    double passive_support_angle_comp_gain_ = 0.25;
    double passive_support_lever_arm_min_ = 0.25;
    std::array<double, kModeCount> passive_mode_gain_ = {0.10, 0.16, 0.16, 0.22};
    std::array<double, kModeCount> passive_mode_integral_gain_ = {0.005, 0.008, 0.008, 0.010};
    double passive_mode_integral_limit_ = 2.0;
    double passive_payload_common_gain_ = 0.08;
    double passive_clearance_margin_rad_ = deg_to_rad_(10.0);
    double passive_clearance_recovery_gain_ = 0.9;
    double passive_clearance_recovery_integral_gain_ = 0.05;
    double passive_trusted_body_rate_threshold_rad_ = deg_to_rad_(18.0);
    double passive_fast_recovery_gain_ = 0.35;
    double passive_max_correction_rad_ = deg_to_rad_(8.0);
    double passive_correction_vel_limit_ = deg_to_rad_(180.0);
    double passive_correction_acc_limit_ = deg_to_rad_(720.0);
    double passive_fast_recovery_vel_limit_ = deg_to_rad_(360.0);
    double passive_fast_recovery_acc_limit_ = deg_to_rad_(1440.0);

    double calibration_wait_time_          = 2.0;
    double calibration_sample_time_        = 3.0;
    double calibration_hold_elapsed_       = 0.0;
    size_t sample_count_                   = 0;
    double pitch_sum_                      = 0.0;
    double roll_sum_                       = 0.0;
    bool calibration_completed_for_window_ = false;
    bool calibrated_once_                  = false;
    double pitch_offset_                   = 0.0;
    double roll_offset_                    = 0.0;

    bool passive_load_initialized_ = false;
    bool passive_payload_initialized_ = false;
    bool passive_fast_recovery_active_ = false;
    bool passive_clearance_recovery_active_ = false;
    std::array<double, kJointCount> passive_load_sign_ = {-1.0, -1.0, -1.0, -1.0};
    std::array<double, kJointCount> passive_load_bias_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> passive_filtered_support_proxy_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> passive_previous_support_proxy_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> passive_unload_timer_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kModeCount> passive_mode_integral_ = {0.0, 0.0, 0.0, 0.0};
    double passive_payload_state_ = 0.0;
    double passive_payload_reference_ = 0.0;
    double passive_clearance_integral_ = 0.0;

    std::array<double, kJointCount> correction_target_rad_             = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> correction_state_rad_              = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> correction_velocity_state_rad_     = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> correction_acceleration_state_rad_ = {0.0, 0.0, 0.0, 0.0};
};

} // namespace rmcs_core::controller::chassis

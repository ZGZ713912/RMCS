#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "identification/angle_sweep.hpp"

namespace rmcs_core::controller::identification {

class DeformableCurrentSweep : public rmcs_executor::Component, public rclcpp::Node {
public:
    explicit DeformableCurrentSweep(const rclcpp::NodeOptions& options =
        rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        : Node(get_component_name(), options), config_(load_config()), sweep_(config_),
          command_(create_partner_component<Command>(get_component_name() + "_command", *this)) {
        condition_ = get_parameter("condition").as_string();
        register_input("/remote/switch/left", left_);
        register_input("/remote/switch/right", right_);
        for (std::size_t i = 0; i < 4; ++i) {
            register_input(prefix(i) + "/physical_angle", measured_[i]);
            register_output(prefix(i) + "/target_physical_angle", targets_[i], NAN);
            register_output(prefix(i) + "/target_physical_velocity", velocities_[i], 0.0);
        }
        register_output("/chassis/joint_control_active", active_, false);
        register_output("/identification/run", run_, 0.0);
        register_output("/identification/stage", stage_, 0.0);
        register_output("/identification/elapsed_s", elapsed_, 0.0);
        register_output("/identification/frequency_hz", frequency_, 0.0);
        register_output("/identification/remote_left", remote_left_, 0.0);
        register_output("/identification/remote_right", remote_right_, 0.0);
        register_output("/identification/metadata", metadata_, std::string{});
        using Trigger = std_srvs::srv::Trigger;
        start_service_ = create_service<Trigger>("/identification/start",
            [this](const Trigger::Request::SharedPtr&, const Trigger::Response::SharedPtr& r) {
                r->success = request_start(); r->message = "Start requested; DR16 double middle required";
            });
        stop_service_ = create_service<Trigger>("/identification/stop",
            [this](const Trigger::Request::SharedPtr&, const Trigger::Response::SharedPtr& r) {
                request_stop(); r->success = true; r->message = "Stop queued";
            });
        status_service_ = create_service<Trigger>("/identification/status",
            [this](const Trigger::Request::SharedPtr&, const Trigger::Response::SharedPtr& r) {
                std::lock_guard lock{mutex_}; r->success = true; r->message = status_;
            });
    }
    bool request_start() { return !active_snapshot_ && !start_requested_.exchange(true); }
    void request_stop() { stop_requested_ = true; }
    void before_updating() override {
        sweep_.reset(); *run_ = 0.0; latched_ = false;
        std::ostringstream m;
        m << "{\"schema_version\":2,\"command_domain\":\"angle_reference\",\"condition\":\""
          << condition_ << "\",\"excitation_mode\":\"simultaneous\",\"torque_calibrated\":false,"
          << "\"min_angle_deg\":" << config_.min_angle * 180 / std::numbers::pi
          << ",\"max_angle_deg\":" << config_.max_angle * 180 / std::numbers::pi
          << ",\"slow_duration_s\":" << config_.slow_duration
          << ",\"fast_duration_s\":" << config_.fast_duration
          << ",\"slow_start_frequency_hz\":" << config_.slow_start_hz
          << ",\"slow_end_frequency_hz\":" << config_.slow_end_hz
          << ",\"fast_start_frequency_hz\":" << config_.fast_start_hz
          << ",\"fast_end_frequency_hz\":" << config_.fast_end_hz
          << ",\"ultra_fast_frequency_hz\":" << config_.ultra_fast_frequency_hz
          << ",\"ultra_fast_duration_s\":" << config_.ultra_fast_duration
          << ",\"experiment_config\":" << get_parameter_or("experiment_config_json", std::string{"{}"}) << "}";
        *metadata_ = m.str();
    }
    void update() override {
        const double now = time_s();
        const bool middle = *left_ == rmcs_msgs::Switch::MIDDLE && *right_ == rmcs_msgs::Switch::MIDDLE;
        const bool requested = start_requested_.exchange(false);
        if (!middle) latched_ = false;
        if (stop_requested_.exchange(false)) {
            sweep_.stop(); start_requested_ = false; latched_ = middle;
        } else if ((middle && !latched_ && !sweep_.active()) || requested) {
            latched_ = middle;
            if (middle) {
                std::array<double, 4> values;
                for (std::size_t i = 0; i < 4; ++i) values[i] = *measured_[i];
                if (sweep_.start(now, values)) *run_ += 1.0;
            }
        } else sweep_.update(now, middle);
        *active_ = sweep_.active(); active_snapshot_ = *active_;
        *stage_ = sweep_.stage(); *frequency_ = sweep_.frequency();
        if (sweep_.active()) *elapsed_ = sweep_.elapsed(now);
        *remote_left_ = static_cast<double>(*left_); *remote_right_ = static_cast<double>(*right_);
        for (std::size_t i = 0; i < 4; ++i) {
            *targets_[i] = sweep_.active() ? sweep_.angles()[i] : NAN;
            *velocities_[i] = sweep_.velocities()[i];
        }
        std::ostringstream text;
        text << "condition=" << condition_ << " active=" << *active_ << " stage=" << *stage_
             << " run=" << *run_ << " elapsed_s=" << *elapsed_ << " frequency_hz=" << *frequency_
             << " remote=" << *remote_left_ << ',' << *remote_right_;
        std::lock_guard lock{mutex_}; status_ = text.str();
    }
private:
    static double time_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
    static std::string prefix(std::size_t i) {
        constexpr std::array names{"left_front", "left_back", "right_back", "right_front"};
        return std::string{"/chassis/"} + names[i] + "_joint";
    }
    AngleSweep::Config load_config() {
        AngleSweep::Config c;
        c.min_angle = get_parameter("min_angle_deg").as_double() * std::numbers::pi / 180;
        c.max_angle = get_parameter("max_angle_deg").as_double() * std::numbers::pi / 180;
        c.move_speed = get_parameter("move_speed_rad_s").as_double();
        c.slow_duration = get_parameter("slow_duration_s").as_double();
        c.fast_duration = get_parameter("fast_duration_s").as_double();
        c.slow_start_hz = get_parameter("slow_start_frequency_hz").as_double();
        c.slow_end_hz = get_parameter("slow_end_frequency_hz").as_double();
        c.fast_start_hz = get_parameter("fast_start_frequency_hz").as_double();
        c.fast_end_hz = get_parameter("fast_end_frequency_hz").as_double();
        c.ultra_fast_frequency_hz = get_parameter_or("ultra_fast_frequency_hz", 1.0);
        c.ultra_fast_duration = get_parameter_or("ultra_fast_duration_s", 10.0);
        return c;
    }
    class Command : public Component {
    public:
        explicit Command(DeformableCurrentSweep&) {
            register_input("/chassis/joint_control_active", active_);
            for (std::size_t i = 0; i < 4; ++i) {
                register_input(prefix(i) + "/servo_current_raw", servo_[i]);
                register_input(prefix(i) + "/torque_per_current_raw", scales_[i]);
                register_input(prefix(i) + "/torque", torque_[i]);
                register_input(prefix(i) + "/command_current_raw", command_current_[i]);
                register_output(prefix(i) + "/control_current_raw", controls_[i], 0.0);
                register_output(prefix(i) + "/target_torque_proxy", target_torque_[i], 0.0);
                register_output(prefix(i) + "/command_torque_proxy", command_torque_[i], 0.0);
                register_output(prefix(i) + "/physical_torque_proxy", physical_torque_[i], 0.0);
                register_output(prefix(i) + "/target_physical_torque_proxy", target_physical_torque_[i], 0.0);
                register_output(prefix(i) + "/command_physical_torque_proxy", command_physical_torque_[i], 0.0);
            }
            register_output("/identification/actuation_active", active_value_, 0.0);
            register_output("/identification/host_time_s", host_time_, 0.0);
        }
        void update() override {
            for (std::size_t i = 0; i < 4; ++i) {
                *controls_[i] = *active_ && std::isfinite(*servo_[i]) ? *servo_[i] : 0.0;
                *target_torque_[i] = *controls_[i] * *scales_[i];
                *command_torque_[i] = *command_current_[i] * *scales_[i];
                *physical_torque_[i] = -*torque_[i];
                *target_physical_torque_[i] = -*target_torque_[i];
                *command_physical_torque_[i] = -*command_torque_[i];
            }
            *active_value_ = *active_ ? 1.0 : 0.0; *host_time_ = time_s();
        }
    private:
        InputInterface<bool> active_;
        std::array<InputInterface<double>, 4> servo_, scales_, torque_, command_current_;
        std::array<OutputInterface<double>, 4> controls_;
        std::array<OutputInterface<double>, 4> target_torque_, command_torque_, physical_torque_,
            target_physical_torque_, command_physical_torque_;
        OutputInterface<double> active_value_, host_time_;
    };
    InputInterface<rmcs_msgs::Switch> left_, right_;
    std::array<InputInterface<double>, 4> measured_;
    std::array<OutputInterface<double>, 4> targets_, velocities_;
    OutputInterface<bool> active_;
    OutputInterface<double> run_, stage_, elapsed_, frequency_, remote_left_, remote_right_;
    OutputInterface<std::string> metadata_;
    AngleSweep::Config config_;
    AngleSweep sweep_;
    std::shared_ptr<Command> command_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> start_service_, stop_service_, status_service_;
    std::atomic<bool> start_requested_{false}, stop_requested_{false}, active_snapshot_{false};
    bool latched_ = false;
    std::mutex mutex_;
    std::string condition_, status_ = "idle";
};
} // namespace rmcs_core::controller::identification

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::identification::DeformableCurrentSweep, rmcs_executor::Component)

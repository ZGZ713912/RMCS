#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <format>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "identification/current_sweep.hpp"
#include "identification/current_sweep_recorder.hpp"

namespace rmcs_core::controller::identification {

namespace {
double host_time_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

CurrentSweep::Config sweep_config(rclcpp::Node& node) {
    CurrentSweep::Config c;
    c.angles_rad = node.get_parameter("angles_deg").as_double_array();
    for (auto& angle : c.angles_rad)
        angle *= std::numbers::pi / 180.0;
    c.amplitudes_raw = node.get_parameter("amplitudes_raw").as_double_array();
    const auto directions = node.get_parameter("current_to_physical_direction").as_double_array();
    if (directions.size() != CurrentSweep::kJointCount)
        throw std::invalid_argument("Four current-to-physical directions are required");
    std::copy(directions.begin(), directions.end(), c.current_to_physical_direction.begin());
    c.min_angle_rad = node.get_parameter("min_angle_deg").as_double() * std::numbers::pi / 180.0;
    c.max_angle_rad = node.get_parameter("max_angle_deg").as_double() * std::numbers::pi / 180.0;
    const auto read = [&](const char* name, double& value) { node.get_parameter(name, value); };
    read("move_speed_rad_s", c.move_speed_rad_s);
    read("settle_time_s", c.settle_time_s);
    read("settle_timeout_s", c.settle_timeout_s);
    read("move_timeout_s", c.move_timeout_s);
    read("angle_tolerance_rad", c.angle_tolerance_rad);
    read("settle_velocity_rad_s", c.settle_velocity_rad_s);
    read("max_excursion_rad", c.max_excursion_rad);
    read("max_velocity_rad_s", c.max_velocity_rad_s);
    read("max_temperature_c", c.max_temperature_c);
    read("max_feedback_age_s", c.max_feedback_age_s);
    read("max_feedback_current_raw", c.max_feedback_current_raw);
    read("max_update_gap_s", c.max_update_gap_s);
    read("start_frequency_hz", c.start_frequency_hz);
    read("end_frequency_hz", c.end_frequency_hz);
    read("chirp_duration_s", c.chirp_duration_s);
    read("validation_duration_s", c.validation_duration_s);
    read("validation_step_s", c.validation_step_s);
    read("probe_ramp_s", c.probe_ramp_s);
    return c;
}
} // namespace

class DeformableCurrentSweep
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit DeformableCurrentSweep(
        const rclcpp::NodeOptions& options =
            rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        : Node(get_component_name(), options)
        , config_(sweep_config(*this))
        , sweep_(config_)
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this)) {
        require_remote_ = get_parameter_or("require_remote_enable", false);
        condition_ = get_parameter("condition").as_string();
        if (condition_ != "suspended" && condition_ != "grounded")
            throw std::invalid_argument("Sweep condition must be suspended or grounded");
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        for (std::size_t i = 0; i < kNames.size(); ++i) {
            const auto p = prefix(i);
            register_input(p + "/physical_angle", angles_[i]);
            register_input(p + "/physical_velocity", velocities_[i]);
            register_input(p + "/current_raw", currents_[i]);
            register_input(p + "/temperature", temperatures_[i]);
            register_input(p + "/feedback_sequence", sequences_[i]);
            register_input(p + "/feedback_time_s", feedback_times_[i]);
            register_output(p + "/target_physical_angle", targets_[i], CurrentSweep::kNan);
            register_output(
                p + "/target_physical_velocity", target_velocities_[i], CurrentSweep::kNan);
            register_output(p + "/sweep_probe_current_raw", probes_[i], 0.0);
        }
        register_output("/chassis/joint_control_active", active_, false);
        register_output("/identification/run", run_, 0.0);
        register_output("/identification/stage", stage_, 0.0);
        register_output("/identification/point", point_, 0.0);
        register_output("/identification/joint", joint_, 0.0);
        register_output("/identification/amplitude_raw", amplitude_, 0.0);
        register_output("/identification/frequency_hz", frequency_, 0.0);
        register_output("/identification/elapsed_s", elapsed_, 0.0);
        register_output("/identification/dropped_samples", dropped_, 0.0);

        using Trigger = std_srvs::srv::Trigger;
        start_service_ = create_service<Trigger>(
            "/identification/start",
            [this](const Trigger::Request::SharedPtr&, const Trigger::Response::SharedPtr& r) {
                r->success = request_start();
                r->message = r->success ? "Start queued; check /identification/status for preflight"
                                        : "A run or start request is already active";
            });
        stop_service_ = create_service<Trigger>(
            "/identification/stop",
            [this](const Trigger::Request::SharedPtr&, const Trigger::Response::SharedPtr& r) {
                request_stop();
                r->success = true;
                r->message = "Stop queued; joint current will be zero on the next update";
            });
        status_service_ = create_service<Trigger>(
            "/identification/status",
            [this](const Trigger::Request::SharedPtr&, const Trigger::Response::SharedPtr& r) {
                std::lock_guard lock{status_mutex_};
                r->success = true;
                r->message = status_text_;
            });
    }

    bool request_start() { return !active_snapshot_.load() && !start_requested_.exchange(true); }

    void request_stop() { stop_requested_.store(true); }

    void before_updating() override {
        sweep_.reset();
        start_requested_ = stop_requested_ = recorder_fault_ = false;
        *run_ = *elapsed_ = *dropped_ = 0.0;
        publish(host_time_s());
    }

    void update() override {
        const double now = host_time_s();
        std::array<CurrentSweep::Feedback, 4> feedback;
        for (std::size_t i = 0; i < feedback.size(); ++i)
            feedback[i] = {*angles_[i],       *velocities_[i], *currents_[i],
                           *temperatures_[i], *sequences_[i],  *feedback_times_[i]};
        using rmcs_msgs::Switch;
        const bool known = *switch_left_ != Switch::UNKNOWN && *switch_right_ != Switch::UNKNOWN;
        const bool reset = *switch_left_ == Switch::DOWN && *switch_right_ == Switch::DOWN;
        const bool enabled = !reset && (!require_remote_ || known);
        if (stop_requested_.exchange(false)) {
            start_requested_ = false;
            sweep_.stop();
        } else if (start_requested_.exchange(false)) {
            if (!enabled)
                sweep_.stop("remote reset or unavailable");
            else if (sweep_.start(now, feedback)) {
                *run_ += 1.0;
                *dropped_ = 0.0;
                recorder_fault_ = false;
            }
        } else if (recorder_fault_.load()) {
            sweep_.stop("recorder or nonfinite ADRC output fault");
        } else {
            sweep_.update(now, feedback, enabled);
        }
        publish(now);
    }

private:
    static constexpr std::array<const char*, 4> kNames{
        "left_front", "left_back", "right_back", "right_front"};
    static std::string prefix(std::size_t i) {
        return std::string{"/chassis/"} + kNames[i] + "_joint";
    }

    void publish(double now) {
        *active_ = sweep_.active();
        active_snapshot_ = sweep_.active();
        *stage_ = static_cast<double>(sweep_.stage());
        *point_ = static_cast<double>(sweep_.point());
        *joint_ = static_cast<double>(sweep_.joint());
        *amplitude_ = sweep_.amplitude();
        *frequency_ = sweep_.frequency(now);
        *elapsed_ = sweep_.active() ? sweep_.elapsed(now) : *elapsed_;
        for (std::size_t i = 0; i < kNames.size(); ++i) {
            *targets_[i] = sweep_.references()[i];
            *target_velocities_[i] = sweep_.reference_velocities()[i];
            *probes_[i] = sweep_.probes()[i];
        }
        std::ostringstream text;
        text << "domain=raw_current condition=" << condition_ << " active=" << *active_
             << " stage=" << *stage_ << " run=" << *run_ << " point=" << *point_
             << " joint=" << kNames[sweep_.joint()] << " dropped=" << *dropped_
             << " reason=" << sweep_.reason() << " csv=" << csv_path_;
        std::lock_guard lock{status_mutex_};
        status_text_ = text.str();
    }

    class Command : public Component {
    public:
        explicit Command(DeformableCurrentSweep& owner)
            : owner_(owner) {
            limit_ = owner.get_parameter_or("max_command_current_raw", 2048.0);
            slew_ = owner.get_parameter_or("max_current_slew_raw_s", 0.0);
            (void)limit_current_command(0.0, 0.0, 0.0, 0.001, limit_, slew_);
            register_input("/chassis/joint_control_active", active_);
            register_input("/identification/run", run_);
            register_input("/identification/stage", stage_);
            register_input("/identification/point", point_);
            register_input("/identification/joint", joint_);
            register_input("/identification/amplitude_raw", amplitude_);
            register_input("/identification/frequency_hz", frequency_);
            register_input("/identification/elapsed_s", elapsed_);
            for (std::size_t i = 0; i < kNames.size(); ++i) {
                const auto p = prefix(i);
                register_input(p + "/servo_current_raw", servo_[i]);
                register_input(p + "/sweep_probe_current_raw", probe_[i]);
                register_input(p + "/target_physical_angle", target_[i]);
                register_input(p + "/target_physical_velocity", target_velocity_[i]);
                register_output(p + "/control_current_raw", controls_[i], 0.0);
                const std::array<const char*, 12> suffixes{
                    "physical_angle",
                    "physical_velocity",
                    "current_raw",
                    "raw_current",
                    "temperature",
                    "feedback_sequence",
                    "feedback_time_s",
                    "feedback_age_s",
                    "command_current_raw",
                    "command_sequence",
                    "command_prepare_time_s",
                    "command_limited"};
                for (std::size_t j = 0; j < suffixes.size(); ++j)
                    register_input(p + "/" + suffixes[j], feedback_[i][j]);
            }
            register_input("/chassis/imu/roll", roll_, false);
            register_input("/chassis/imu/pitch", pitch_, false);
        }

        void before_updating() override {
            recorder_.close();
            was_active_ = false;
            last_time_ = host_time_s();
            for (auto& control : controls_)
                *control = 0.0;
        }

        void update() override {
            const double now = host_time_s();
            const double dt = now - last_time_;
            last_time_ = now;
            bool active = *active_ && !owner_.recorder_fault_.load();
            if (active && !was_active_) {
                try {
                    if (!recorder_.finished())
                        throw std::runtime_error(
                            "Previous CSV is still draining; retry start after it finishes");
                    owner_.csv_path_ = std::format(
                        "/tmp/deformable_current_sweep_{}_{:.0f}.csv", owner_.condition_,
                        now * 1e6);
                    recorder_.open(owner_.csv_path_, columns(), metadata());
                } catch (const std::exception& e) {
                    recorder_.close();
                    RCLCPP_ERROR(
                        owner_.get_logger(), "Cannot open current sweep log: %s", e.what());
                    owner_.recorder_fault_ = true;
                    active = false;
                }
            }
            std::array<CurrentCommand, 4> commands;
            bool valid = active;
            if (active)
                for (std::size_t i = 0; i < kNames.size(); ++i) {
                    valid &= std::isfinite(*servo_[i]) && std::isfinite(*probe_[i]);
                    commands[i] = limit_current_command(
                        *servo_[i], *probe_[i], *controls_[i], dt, limit_, slew_);
                }
            if (active && (!valid || recorder_.failed())) {
                owner_.recorder_fault_ = true;
                valid = false;
            }
            for (std::size_t i = 0; i < kNames.size(); ++i)
                *controls_[i] = valid ? commands[i].encoded : 0.0;
            if (active || was_active_) {
                CurrentSweepRecorder::Row row{};
                std::size_t n = 0;
                const auto add = [&](double value) { row[n++] = value; };
                for (double value :
                     {1.0, now, *elapsed_, *run_, *stage_, *point_, *joint_, *amplitude_,
                      *frequency_, valid ? 1.0 : 0.0, static_cast<double>(recorder_.dropped())})
                    add(value);
                for (std::size_t i = 0; i < kNames.size(); ++i) {
                    for (const auto& signal : feedback_[i])
                        add(*signal);
                    for (double value :
                         {*servo_[i], *probe_[i], commands[i].requested, *controls_[i],
                          commands[i].clipped ? 1.0 : 0.0, commands[i].rate_limited ? 1.0 : 0.0,
                          *target_[i], *target_velocity_[i]})
                        add(value);
                }
                add(roll_.ready() ? *roll_ : CurrentSweep::kNan);
                add(pitch_.ready() ? *pitch_ : CurrentSweep::kNan);
                if (!recorder_.push(row)) {
                    owner_.recorder_fault_ = true;
                    for (auto& control : controls_)
                        *control = 0.0;
                }
                *owner_.dropped_ = static_cast<double>(recorder_.dropped());
            }
            if (!active || owner_.recorder_fault_.load())
                recorder_.request_finish(
                    static_cast<int>(*stage_) == static_cast<int>(CurrentSweep::Stage::kComplete)
                    && !owner_.recorder_fault_.load());
            was_active_ = active && !owner_.recorder_fault_.load();
        }

    private:
        std::vector<std::string> columns() const {
            std::vector<std::string> names{"schema_version",   "host_time_s",   "elapsed_s",
                                           "run_id",           "stage",         "point_index",
                                           "active_joint",     "amplitude_raw", "frequency_hz",
                                           "actuation_active", "queue_dropped"};
            for (std::size_t i = 0; i < kNames.size(); ++i)
                for (const char* suffix :
                     {"physical_angle",
                      "physical_velocity",
                      "current_raw",
                      "raw_current",
                      "temperature",
                      "feedback_sequence",
                      "feedback_time_s",
                      "feedback_age_s",
                      "command_current_raw",
                      "command_sequence",
                      "command_prepare_time_s",
                      "command_limited",
                      "servo_current_raw",
                      "sweep_probe_current_raw",
                      "requested_current_raw",
                      "limited_current_raw",
                      "current_clipped",
                      "current_rate_limited",
                      "target_physical_angle",
                      "target_physical_velocity"})
                    names.push_back(prefix(i) + "/" + suffix);
            names.emplace_back("/chassis/imu/roll");
            names.emplace_back("/chassis/imu/pitch");
            return names;
        }

        std::string metadata() const {
            std::ostringstream m;
            m << "{\"schema_version\":1,\"command_domain\":\"raw_current\",\"condition\":\""
              << owner_.condition_
              << "\",\"torque_calibrated\":false,\"max_command_current_raw\":" << limit_
              << ",\"max_current_slew_raw_s\":" << slew_
              << ",\"max_feedback_age_s\":" << owner_.config_.max_feedback_age_s
              << ",\"experiment_config\":"
              << owner_.get_parameter_or("experiment_config_json", std::string{"{}"})
              << ",\"driver_info\":\"MG5010i36 v3.0 / DG40C7 HW3.2 FW2.37 (user reported)\""
              << ",\"current_to_physical_direction\":[";
            for (std::size_t i = 0; i < kNames.size(); ++i)
                m << (i ? "," : "") << owner_.config_.current_to_physical_direction[i];
            m << "],\"targets\":[";
            for (std::size_t i = 0; i < kNames.size(); ++i)
                m << (i ? "," : "") << '"' << prefix(i) << '"';
            m << "],\"command_timestamp\":\"host packet preparation; not wire transmit time\""
              << ",\"feedback_timestamp\":\"host CAN receive; not motor sampling time\"}";
            return m.str();
        }

        DeformableCurrentSweep& owner_;
        InputInterface<bool> active_;
        InputInterface<double> run_, stage_, point_, joint_, amplitude_, frequency_, elapsed_,
            roll_, pitch_;
        std::array<InputInterface<double>, 4> servo_, probe_, target_, target_velocity_;
        std::array<std::array<InputInterface<double>, 12>, 4> feedback_;
        std::array<OutputInterface<double>, 4> controls_;
        CurrentSweepRecorder recorder_;
        double limit_ = 2048.0, slew_ = 0.0, last_time_ = 0.0;
        bool was_active_ = false;
    };

    std::array<InputInterface<double>, 4> angles_, velocities_, currents_, temperatures_,
        sequences_, feedback_times_;
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    std::array<OutputInterface<double>, 4> targets_, target_velocities_, probes_;
    OutputInterface<bool> active_;
    OutputInterface<double> run_, stage_, point_, joint_, amplitude_, frequency_, elapsed_,
        dropped_;
    CurrentSweep::Config config_;
    CurrentSweep sweep_;
    std::shared_ptr<Command> command_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> start_service_, stop_service_,
        status_service_;
    std::atomic<bool> start_requested_{false}, stop_requested_{false}, recorder_fault_{false},
        active_snapshot_{false};
    bool require_remote_ = false;
    std::string condition_, csv_path_,
        status_text_ = "idle; raw-current sweep requires explicit start";
    std::mutex status_mutex_;
};

} // namespace rmcs_core::controller::identification

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::DeformableCurrentSweep, rmcs_executor::Component)

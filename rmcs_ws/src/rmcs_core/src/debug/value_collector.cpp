#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <mutex>
#include <stdexcept>
#include <std_srvs/srv/trigger.hpp>

#include "identification/current_sweep_recorder.hpp"

namespace rmcs_core::debug {

class ValueCollector
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    explicit ValueCollector(const rclcpp::NodeOptions& options =
        rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        : Node(
              get_component_name(),
              options) {
        experiment_ = get_parameter_or("experiment_mode", false);
        if (experiment_) {
            register_input("/identification/run", run_);
            register_input("/identification/stage", stage_);
            register_input("/identification/actuation_active", active_);
            register_input("/identification/metadata", metadata_);
            register_input("/chassis/joint_command_time_s", command_time_);
            fault_ = create_partner_component<Fault>(get_component_name() + "_fault", *this);
            service_ = create_service<std_srvs::srv::Trigger>(
                "/identification/collector_status",
                [this](const std_srvs::srv::Trigger::Request::SharedPtr&,
                       const std_srvs::srv::Trigger::Response::SharedPtr& response) {
                    std::lock_guard lock{status_mutex_};
                    response->success = true;
                    response->message = status_;
                });
        }
    }

    ~ValueCollector() override { recorder_.close(); }

    auto before_pairing(const OutputInfoMap& output_map) -> void override {
        node::param("csv_path", csv_path_);

        {
            constexpr auto kPlaceholder = std::string_view{"<timestamp>"};
            const auto pos = csv_path_.find(kPlaceholder);
            if (pos != std::string::npos) {
                const auto now = std::chrono::system_clock::now();
                const auto time = std::chrono::system_clock::to_time_t(now);

                auto ss = std::ostringstream{};
                ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S");
                csv_path_.replace(pos, kPlaceholder.size(), ss.str());
            }
        }

        node::param("signals", signal_names_);
        node::param("write_interval", write_interval_);
        node::param("flush_interval", flush_interval_);

        if (csv_path_.empty() || signal_names_.empty()) {
            if (experiment_)
                throw std::invalid_argument("Experiment collector needs path and signals");
            return;
        }

        for (const auto& name : signal_names_) {
            const auto it = output_map.find(name);
            if (it == output_map.end()) {
                if (experiment_)
                    throw std::invalid_argument("Missing experiment signal " + name);
                node::error("signal '{}' not found", name);
                continue;
            }
            if (it->second.type.get() != typeid(double)) {
                if (experiment_)
                    throw std::invalid_argument("Experiment signal is not double: " + name);
                node::error("signal '{}' type is not double", name);
                continue;
            }

            auto unit = std::make_unique<SignalUnit>();
            unit->name = name;
            if (experiment_ && name == "/identification/stage")
                unit->source = &stage_;
            else if (experiment_ && name == "/identification/run")
                unit->source = &run_;
            else if (experiment_ && name == "/identification/actuation_active")
                unit->source = &active_;
            else {
                register_input(name, unit->value);
                unit->source = &unit->value;
            }
            units_.push_back(std::move(unit));
        }

        if (units_.empty())
            return;

        if (write_interval_ <= 0 || flush_interval_ <= 0)
            throw std::invalid_argument("Collector intervals must be positive");
        if (experiment_) {
            if (units_.size() > rmcs_core::controller::identification::CurrentSweepRecorder::kMaxColumns)
                throw std::invalid_argument("Too many experiment signals");
            return;
        }

        csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
        if (!csv_file_.is_open()) {
            node::error("failed to open {}", csv_path_);
            return;
        }

        csv_file_ << "index";
        for (const auto& unit : units_)
            csv_file_ << "," << unit->name;
        csv_file_ << "\n";
        csv_file_.flush();

        node::info("collecting {} signals to {}", units_.size(), csv_path_);
    }

    auto update() -> void override {
        if (experiment_) {
            update_experiment();
            return;
        }
        if (units_.empty() || !csv_file_.is_open())
            return;

        if (tick_++ % write_interval_ != 0)
            return;

        csv_file_ << std::setprecision(17) << sample_count_++;
        for (const auto& unit : units_)
            csv_file_ << "," << **unit->source;
        csv_file_ << "\n";

        if (sample_count_ % flush_interval_ == 0)
            csv_file_.flush();
    }

private:
    class Fault : public Component {
    public:
        explicit Fault(ValueCollector& owner) : owner_(owner) {
            register_output("/identification/collector_fault", fault_, false);
        }
        void update() override { *fault_ = owner_.failed_.load() || owner_.recorder_.failed(); }
    private:
        ValueCollector& owner_;
        OutputInterface<bool> fault_;
    };

    void update_experiment() {
        const double run = *run_;
        if (run != last_run_) {
            if (!recorder_.finished()) {
                failed_ = true;
                return;
            }
            last_run_ = run;
            failed_ = false;
            try {
                auto path = csv_path_;
                const auto dot = path.rfind(".csv");
                path.insert(dot == std::string::npos ? path.size() : dot,
                            "_run" + std::to_string(static_cast<int>(run)));
                recorder_.open(path, signal_names_, *metadata_, flush_interval_);
                tick_ = 0;
                std::lock_guard lock{status_mutex_};
                path_ = path;
            } catch (const std::exception& e) {
                failed_ = true;
                RCLCPP_ERROR(get_logger(), "Collector open failed: %s", e.what());
            }
            recording_ = !failed_;
        }
        if (recording_) {
            const bool final = *active_ == 0.0 || failed_;
            if (!final && tick_++ % write_interval_ != 0)
                return;
            rmcs_core::controller::identification::CurrentSweepRecorder::Row row{};
            for (std::size_t i = 0; i < units_.size(); ++i)
                row[i] = **units_[i]->source;
            const double now = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            for (std::size_t i = 0; i < units_.size(); ++i)
                if (units_[i]->name == "/identification/host_time_s")
                    row[i] = now;
            // Hardware prepares this cycle's encoded commands after the servo
            // component. Derive command proxies here so they match logged CAN counts.
            for (std::size_t i = 0; i < units_.size(); ++i) {
                const auto& name = units_[i]->name;
                const bool physical = name.ends_with("/command_physical_torque_proxy");
                if (!physical && !name.ends_with("/command_torque_proxy"))
                    continue;
                const auto prefix = name.substr(0, name.rfind('/'));
                double current = NAN, scale = NAN;
                for (std::size_t j = 0; j < units_.size(); ++j) {
                    if (units_[j]->name == prefix + "/command_current_raw") current = row[j];
                    if (units_[j]->name == prefix + "/torque_per_current_raw") scale = row[j];
                }
                row[i] = (physical ? -1.0 : 1.0) * current * scale;
            }
            if (!recorder_.push(row))
                failed_ = true;
            if (*active_ == 0.0 || failed_) {
                recorder_.request_finish(*stage_ == 5.0 && !failed_);
                recording_ = false;
            }
        }
        std::lock_guard lock{status_mutex_};
        status_ = "csv=" + path_ + " finished=" + (recorder_.finished() ? "1" : "0")
            + " failed=" + ((failed_ || recorder_.failed()) ? "1" : "0")
            + " dropped=" + std::to_string(recorder_.dropped());
    }

    struct SignalUnit {
        std::string name;
        InputInterface<double> value;
        const InputInterface<double>* source = nullptr;
    };

    std::string csv_path_;
    std::vector<std::string> signal_names_;
    int write_interval_ = 1;
    int flush_interval_ = 100;

    std::vector<std::unique_ptr<SignalUnit>> units_;
    std::ofstream csv_file_;

    int tick_ = 0;
    int sample_count_ = 0;
    InputInterface<double> run_, stage_, active_, command_time_;
    InputInterface<std::string> metadata_;
    rmcs_core::controller::identification::CurrentSweepRecorder recorder_;
    std::shared_ptr<Fault> fault_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_;
    std::atomic<bool> failed_{false};
    bool experiment_ = false, recording_ = false;
    double last_run_ = 0.0;
    std::mutex status_mutex_;
    std::string path_, status_ = "idle";
};

} // namespace rmcs_core::debug

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::debug::ValueCollector, rmcs_executor::Component)

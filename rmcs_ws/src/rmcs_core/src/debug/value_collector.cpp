#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>

namespace rmcs_core::debug {

class ValueCollector
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    ValueCollector()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {}

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

        if (csv_path_.empty() || signal_names_.empty())
            return;

        register_input("/predefined/timestamp", timestamp_);
        for (const auto& name : signal_names_) {
            if (name == "/wheel_leg/imu/pitch" || name == "/wheel_leg/imu/yaw"
                || name == "/wheel_leg/imu/pitch_velocity") {
                if (name == "/wheel_leg/imu/pitch" || name == "/wheel_leg/imu/yaw")
                    imu_angles_requested_ = true;
                if (name == "/wheel_leg/imu/pitch_velocity")
                    pitch_velocity_requested_ = true;
                auto unit = std::make_unique<SignalUnit>();
                unit->name = name;
                units_.push_back(std::move(unit));
                continue;
            }
            const auto it = output_map.find(name);
            if (it == output_map.end()) {
                node::error("signal '{}' not found", name);
                continue;
            }
            if (it->second.type.get() != typeid(double)) {
                node::error("signal '{}' type is not double", name);
                continue;
            }

            auto unit = std::make_unique<SignalUnit>();
            unit->name = name;
            register_input(name, unit->value);
            units_.push_back(std::move(unit));
        }

        if (imu_angles_requested_)
            register_input("/wheel_leg/imu/quaternion", imu_quaternion_);
        if (pitch_velocity_requested_)
            register_input("/wheel_leg/imu/sensor_gyro/y", pitch_velocity_);

        if (write_interval_ <= 0 || flush_interval_ <= 0)
            throw std::invalid_argument("write_interval and flush_interval must be positive");

        if (units_.empty())
            return;

        csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
        if (!csv_file_.is_open()) {
            node::error("failed to open {}", csv_path_);
            return;
        }

        csv_file_ << "timestamp_ns";
        for (const auto& unit : units_)
            csv_file_ << "," << unit->name;
        csv_file_ << "\n";
        csv_file_.flush();

        node::info("collecting {} signals to {}", units_.size(), csv_path_);
    }

    auto update() -> void override {
        if (units_.empty() || !csv_file_.is_open())
            return;

        if (tick_++ % write_interval_ != 0)
            return;

        if (!timestamp_.ready() || (imu_angles_requested_ && !imu_quaternion_.ready())
            || (pitch_velocity_requested_ && !pitch_velocity_.ready()))
            return;

        const auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      timestamp_->time_since_epoch())
                                      .count();
        double pitch = 0.0, yaw = 0.0;
        if (imu_angles_requested_) {
            const double sin_pitch = std::clamp(
                2.0 * (imu_quaternion_->w() * imu_quaternion_->y()
                       - imu_quaternion_->z() * imu_quaternion_->x()),
                -1.0, 1.0);
            pitch = std::asin(sin_pitch);
            yaw = std::atan2(
                2.0 * (imu_quaternion_->w() * imu_quaternion_->z()
                       + imu_quaternion_->x() * imu_quaternion_->y()),
                1.0 - 2.0 * (imu_quaternion_->y() * imu_quaternion_->y()
                             + imu_quaternion_->z() * imu_quaternion_->z()));
        }
        csv_file_ << timestamp_ns;
        for (const auto& unit : units_) {
            if (unit->name == "/wheel_leg/imu/pitch")
                csv_file_ << ',' << pitch;
            else if (unit->name == "/wheel_leg/imu/yaw")
                csv_file_ << ',' << yaw;
            else if (unit->name == "/wheel_leg/imu/pitch_velocity")
                csv_file_ << ',' << *pitch_velocity_;
            else
                csv_file_ << ',' << *unit->value;
        }
        csv_file_ << "\n";

        if (++sample_count_ % flush_interval_ == 0)
            csv_file_.flush();
    }

private:
    struct SignalUnit {
        std::string name;
        InputInterface<double> value;
    };

    std::string csv_path_;
    std::vector<std::string> signal_names_;
    int write_interval_ = 1;
    int flush_interval_ = 100;
    bool imu_angles_requested_ = false;
    bool pitch_velocity_requested_ = false;
    InputInterface<Eigen::Quaterniond> imu_quaternion_;
    InputInterface<double> pitch_velocity_;
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;

    std::vector<std::unique_ptr<SignalUnit>> units_;
    std::ofstream csv_file_;

    int tick_ = 0;
    int sample_count_ = 0;
};

} // namespace rmcs_core::debug

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::debug::ValueCollector, rmcs_executor::Component)

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <tuple>

#include <gtest/gtest.h>

#include "identification/current_sweep.hpp"
#include "identification/current_sweep_recorder.hpp"

namespace {
using rmcs_core::controller::identification::CurrentSweep;
using rmcs_core::controller::identification::CurrentSweepRecorder;
using rmcs_core::controller::identification::limit_current_command;

CurrentSweep::Config config() {
    CurrentSweep::Config c;
    c.angles_rad = {1.0, 0.7, 1.0};
    c.amplitudes_raw = {128.0, 384.0};
    c.min_angle_rad = 0.3;
    c.max_angle_rad = 1.3;
    c.move_speed_rad_s = 2.0;
    c.settle_time_s = 0.02;
    c.chirp_duration_s = 0.2;
    c.validation_duration_s = 0.15;
    c.probe_ramp_s = 0.02;
    c.validation_step_s = 0.03;
    return c;
}

std::array<CurrentSweep::Feedback, 4> feedback(double now) {
    std::array<CurrentSweep::Feedback, 4> f;
    f.fill({1.0, 0.0, 2000.0, 30.0, 1.0, now});
    return f;
}

TEST(CurrentSweep, VisitsEveryAngleJointAmplitudeWithIndependentValidation) {
    CurrentSweep s{config()};
    double now = 10.0;
    auto f = feedback(now);
    ASSERT_TRUE(s.start(now, f));
    std::set<std::tuple<std::size_t, std::size_t, double, int>> visited;
    for (int step = 0; step < 10000 && s.active(); ++step) {
        now += 0.005;
        for (std::size_t i = 0; i < f.size(); ++i) {
            f[i].angle = s.references()[i];
            f[i].time_s = now;
            f[i].sequence += 1.0;
        }
        const auto previous = s.references();
        s.update(now, f, true);
        if (s.stage() == CurrentSweep::Stage::kMove) {
            EXPECT_LE(std::abs(s.references()[0] - previous[0]), 0.01000001);
        }
        if (s.stage() == CurrentSweep::Stage::kChirp
            || s.stage() == CurrentSweep::Stage::kValidation) {
            visited.emplace(s.point(), s.joint(), s.amplitude(), static_cast<int>(s.stage()));
            for (std::size_t i = 0; i < f.size(); ++i) {
                EXPECT_LE(std::abs(s.probes()[i]), s.amplitude());
                if (i != s.joint()) {
                    EXPECT_EQ(s.probes()[i], 0.0);
                }
            }
        }
    }
    EXPECT_EQ(s.stage(), CurrentSweep::Stage::kComplete);
    EXPECT_EQ(visited.size(), 3u * 4u * 2u * 2u);
    for (double probe : s.probes())
        EXPECT_EQ(probe, 0.0);
    EXPECT_TRUE(std::isnan(s.references()[0]));
}

TEST(CurrentSweep, UsesFreshNativeCountsRatherThanLegacyTorqueProxy) {
    CurrentSweep s{config()};
    auto f = feedback(10.0); // raw 2000 is accepted without testing a 25-Nm bound.
    ASSERT_TRUE(s.start(10.0, f));
    f[0].time_s = 9.0;
    s.update(10.005, f, true);
    EXPECT_EQ(s.stage(), CurrentSweep::Stage::kAborted);
    EXPECT_TRUE(s.reason().contains("stale"));
    EXPECT_EQ(s.probes()[0], 0.0);
}

TEST(CurrentSweep, ResetTimingThermalAngleAndSequenceFaultsStopAllJoints) {
    for (int fault = 0; fault < 6; ++fault) {
        CurrentSweep s{config()};
        auto f = feedback(10.0);
        ASSERT_TRUE(s.start(10.0, f));
        double now = 10.005;
        switch (fault) {
        case 0: break;
        case 1: now = 10.1; break;
        case 2: f[1].temperature = 65.0; break;
        case 3: f[2].angle = 0.2; break;
        case 4: f[3].sequence = 0.0; break;
        case 5: f[1].current_raw = CurrentSweep::kNan; break;
        }
        s.update(now, f, fault != 0);
        EXPECT_FALSE(s.active());
        for (std::size_t i = 0; i < 4; ++i) {
            EXPECT_EQ(s.probes()[i], 0.0);
            EXPECT_TRUE(std::isnan(s.references()[i]));
        }
        f = feedback(now + 1.0);
        EXPECT_TRUE(s.start(now + 1.0, f));
        s.stop();
        EXPECT_FALSE(s.active());
    }
}

TEST(CurrentSweep, NativeCapIsFinalAndSlewIsDisabledByDefault) {
    const auto large = limit_current_command(1900.0, 384.0, 0.0, 0.001, 2048.0);
    EXPECT_EQ(large.requested, 2284.0);
    EXPECT_EQ(large.encoded, 2048.0);
    EXPECT_TRUE(large.clipped);
    EXPECT_FALSE(large.rate_limited);
    EXPECT_EQ(limit_current_command(-2000.0, 0.0, 0.0, 0.001, 2048.0).encoded, -2000.0);
    const auto slew = limit_current_command(100.0, 0.0, 0.0, 0.01, 2048.0, 200.0);
    EXPECT_EQ(slew.encoded, 2.0);
    EXPECT_TRUE(slew.rate_limited);
    EXPECT_EQ(limit_current_command(CurrentSweep::kNan, 128.0, 0.0, 0.001, 2048).encoded, 0.0);
    EXPECT_THROW(limit_current_command(0, 0, 0, 0.001, 2049), std::invalid_argument);
}

TEST(CurrentSweepRecorder, PreservesTimestampPrecisionOrderAndFinalStatus) {
    const auto dir = std::filesystem::temp_directory_path() / "rmcs_current_sweep_recorder_test";
    std::filesystem::create_directories(dir);
    const auto path = dir / "good.csv";
    CurrentSweepRecorder r;
    r.open(path, {"sequence", "time_s"}, "{\"command_domain\":\"raw_current\"}");
    for (int i = 0; i < 100; ++i) {
        CurrentSweepRecorder::Row row{};
        row[0] = i;
        row[1] = 123456.123456789 + i * 0.001;
        ASSERT_TRUE(r.push(row));
    }
    r.request_finish(true);
    r.close();
    EXPECT_TRUE(r.finished());
    EXPECT_FALSE(r.failed());
    std::ifstream in{path};
    std::string line;
    ASSERT_TRUE(static_cast<bool>(std::getline(in, line)));
    for (int i = 0; i < 100; ++i) {
        ASSERT_TRUE(static_cast<bool>(std::getline(in, line)));
        const auto comma = line.find(',');
        EXPECT_EQ(std::stoi(line.substr(0, comma)), i);
        EXPECT_DOUBLE_EQ(std::stod(line.substr(comma + 1)), 123456.123456789 + i * 0.001);
    }
    std::ifstream status{path.string() + ".status.json"};
    ASSERT_TRUE(static_cast<bool>(std::getline(status, line)));
    EXPECT_TRUE(line.contains("\"completed\":true"));
    EXPECT_TRUE(line.contains("\"write_failed\":false"));
    EXPECT_THROW(r.open(dir, {"bad"}, "{}"), std::ios_base::failure);
    r.close();
    r.open(dir / "restart.csv", {"restarted"}, "{}");
    EXPECT_TRUE(r.push({}));
    r.request_finish(false);
    r.close();
    EXPECT_FALSE(r.failed());
}
} // namespace

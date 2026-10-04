#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace rmcs_core::controller::identification {

// Single executor producer, single disk writer. No CSV formatting or disk flush
// in the control loop; a dropped row or write failure invalidates the experiment.
class CurrentSweepRecorder {
public:
    static constexpr std::size_t kMaxColumns = 128;
    static constexpr std::size_t kCapacity = 2048;
    using Row = std::array<double, kMaxColumns>;

    CurrentSweepRecorder()
        : rows_(std::make_unique<Row[]>(kCapacity)) {}
    ~CurrentSweepRecorder() { close(); }

    void open(
        const std::filesystem::path& path, const std::vector<std::string>& columns,
        const std::string& metadata) {
        close();
        if (columns.empty() || columns.size() > kMaxColumns)
            throw std::invalid_argument("Invalid sweep CSV column count");
        head_ = tail_ = dropped_ = 0;
        failed_ = false;
        stop_ = false;
        completed_ = false;
        finished_ = false;
        count_ = columns.size();
        path_ = path;
        if (!path.parent_path().empty())
            std::filesystem::create_directories(path.parent_path());
        stream_.exceptions(std::ios::badbit | std::ios::failbit);
        stream_.open(path);
        for (std::size_t i = 0; i < columns.size(); ++i)
            stream_ << (i ? "," : "") << columns[i];
        stream_ << '\n' << std::setprecision(17);
        std::ofstream sidecar{path.string() + ".json"};
        sidecar.exceptions(std::ios::badbit | std::ios::failbit);
        sidecar << metadata << '\n';
        sidecar.close();
        worker_ = std::thread([this] {
            try {
                auto next_flush = std::chrono::steady_clock::now();
                while (!stop_.load(std::memory_order_acquire)
                       || tail_.load(std::memory_order_relaxed)
                              != head_.load(std::memory_order_acquire)) {
                    const auto tail = tail_.load(std::memory_order_relaxed);
                    if (tail == head_.load(std::memory_order_acquire)) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                    const auto& row = rows_[tail % kCapacity];
                    for (std::size_t i = 0; i < count_; ++i)
                        stream_ << (i ? "," : "") << row[i];
                    stream_ << '\n';
                    tail_.store(tail + 1, std::memory_order_release);
                    const auto now = std::chrono::steady_clock::now();
                    if (now >= next_flush) {
                        stream_.flush();
                        next_flush = now + std::chrono::milliseconds(100);
                    }
                }
                stream_.flush();
            } catch (...) {
                failed_.store(true, std::memory_order_release);
            }
            try {
                stream_.close();
            } catch (...) {
                failed_.store(true, std::memory_order_release);
            }
            try {
                std::ofstream status{path_.string() + ".status.json"};
                status.exceptions(std::ios::badbit | std::ios::failbit);
                status << "{\"completed\":" << (completed_.load() ? "true" : "false")
                       << ",\"write_failed\":" << (failed() ? "true" : "false")
                       << ",\"dropped_samples\":" << dropped() << "}\n";
                status.close();
            } catch (...) {
                failed_.store(true, std::memory_order_release);
            }
            finished_.store(true, std::memory_order_release);
        });
    }

    bool push(const Row& row) {
        if (!worker_.joinable() || stop_.load(std::memory_order_acquire) || failed())
            return false;
        const auto head = head_.load(std::memory_order_relaxed);
        if (head - tail_.load(std::memory_order_acquire) >= kCapacity) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            failed_.store(true, std::memory_order_release);
            return false;
        }
        rows_[head % kCapacity] = row;
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    // Finishing a run must not delay the zero-current CAN frame while disk drains.
    void request_finish(bool completed) noexcept {
        if (stop_.load(std::memory_order_acquire))
            return;
        completed_.store(completed, std::memory_order_relaxed);
        stop_.store(true, std::memory_order_release);
    }

    void close() noexcept {
        request_finish(false);
        if (worker_.joinable())
            worker_.join();
        try {
            if (stream_.is_open())
                stream_.close();
        } catch (...) {
            failed_.store(true, std::memory_order_release);
        }
        stream_.clear();
        finished_.store(true, std::memory_order_release);
    }

    bool failed() const { return failed_.load(std::memory_order_acquire); }
    bool finished() const { return finished_.load(std::memory_order_acquire); }
    std::size_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

private:
    std::unique_ptr<Row[]> rows_;
    std::atomic<std::size_t> head_{0}, tail_{0}, dropped_{0};
    std::atomic<bool> stop_{false}, failed_{false}, completed_{false}, finished_{true};
    std::size_t count_ = 0;
    std::ofstream stream_;
    std::thread worker_;
    std::filesystem::path path_;
};

} // namespace rmcs_core::controller::identification

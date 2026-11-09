#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace rmcs_core::hardware::device {

class ActuatorLatencyTrace {
public:
    struct Event {
        int event = 0, axis = -1, code = 0;
        std::uint64_t sequence = 0, steady_ns = 0, end_steady_ns = 0;
        double value = std::numeric_limits<double>::quiet_NaN();
    };

    explicit ActuatorLatencyTrace(const std::string& path, std::size_t capacity = 65536)
        : queue_(capacity)
        , file_(path, std::ios::out | std::ios::noreplace) {
        if (capacity < 2 || !file_)
            throw std::invalid_argument("Cannot open latency trace or invalid capacity: " + path);
        for (std::size_t i = 0; i < capacity; ++i)
            queue_[i].turn.store(i, std::memory_order_relaxed);
        file_ << "event,axis,sequence,steady_ns,end_steady_ns,value,code\n"
              << std::setprecision(17);
        writer_ = std::jthread([this] { write(); });
    }

    ~ActuatorLatencyTrace() {
        stopping_.store(true, std::memory_order_release);
        writer_.join();
    }

    void push(Event event) noexcept {
        auto position = producer_.load(std::memory_order_relaxed);
        Cell* cell;
        while (true) {
            cell = &queue_[position % queue_.size()];
            const auto difference =
                static_cast<std::int64_t>(cell->turn.load(std::memory_order_acquire) - position);
            if (difference == 0) {
                if (producer_.compare_exchange_weak(position, position + 1))
                    break;
            } else if (difference < 0) {
                dropped_.fetch_add(1, std::memory_order_relaxed);
                return;
            } else {
                position = producer_.load(std::memory_order_relaxed);
            }
        }
        cell->event = event;
        cell->turn.store(position + 1, std::memory_order_release);
    }

    std::uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
    bool failed() const { return failed_.load(std::memory_order_relaxed); }

    static std::uint64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

private:
    struct Cell {
        std::atomic<std::uint64_t> turn{0};
        Event event;
    };

    void write() {
        std::array<Event, 512> batch;
        while (true) {
            std::size_t count = 0;
            while (count < batch.size()) {
                auto& cell = queue_[consumer_ % queue_.size()];
                if (cell.turn.load(std::memory_order_acquire) != consumer_ + 1)
                    break;
                batch[count++] = cell.event;
                cell.turn.store(consumer_ + queue_.size(), std::memory_order_release);
                ++consumer_;
            }
            for (std::size_t i = 0; i < count; ++i) {
                const auto& e = batch[i];
                file_ << e.event << ',' << e.axis << ',' << e.sequence << ',' << e.steady_ns << ','
                      << e.end_steady_ns << ',' << e.value << ',' << e.code << '\n';
            }
            if (!file_)
                failed_.store(true, std::memory_order_relaxed);
            if (!count) {
                if (stopping_.load(std::memory_order_acquire))
                    break;
                std::this_thread::sleep_for(std::chrono::milliseconds{1});
            }
        }
        // Event 9 certifies a clean close and the exact number of lost producer events.
        file_ << "9,-1," << dropped() << ',' << now_ns() << ",0,0,0\n";
        file_.flush();
        if (!file_)
            failed_.store(true, std::memory_order_relaxed);
    }

    std::vector<Cell> queue_;
    std::ofstream file_;
    std::atomic<std::uint64_t> producer_{0};
    std::uint64_t consumer_ = 0;
    std::atomic<std::uint64_t> dropped_{0};
    std::atomic<bool> stopping_{false}, failed_{false};
    std::jthread writer_;
};

} // namespace rmcs_core::hardware::device

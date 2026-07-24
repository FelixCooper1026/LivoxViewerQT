#ifndef FREEDOM_UTILS_CORE_H
#define FREEDOM_UTILS_CORE_H

#include "freedom/common_types.h"

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

namespace freedom {

class Timer {
public:
    struct TimerData {
        std::string name;
        std::chrono::high_resolution_clock::time_point start_time;
        double total_time = 0.0;
        double last_elapsed_ms = 0.0;
        int count = 0;
        bool is_running = false;

        explicit TimerData(const std::string& timer_name = {}) : name(timer_name) {}

        void start()
        {
            start_time = std::chrono::high_resolution_clock::now();
            is_running = true;
        }

        double stop()
        {
            if (!is_running) {
                return 0.0;
            }
            const auto end_time = std::chrono::high_resolution_clock::now();
            last_elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            total_time += last_elapsed_ms;
            ++count;
            is_running = false;
            return last_elapsed_ms;
        }

        void reset()
        {
            total_time = 0.0;
            last_elapsed_ms = 0.0;
            count = 0;
            is_running = false;
        }
    };

    TimerData& operator[](const std::string& timer_name)
    {
        for (TimerData& timer : timers_) {
            if (timer.name == timer_name) {
                return timer;
            }
        }
        timers_.emplace_back(timer_name);
        return timers_.back();
    }

private:
    std::vector<TimerData> timers_;
};

template <typename T>
class VectorElementGetter {
public:
    explicit VectorElementGetter(std::vector<T, Eigen::aligned_allocator<T>>& elements)
        : elements_(elements) {}

    bool get_ptr(T*& element)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_index_ >= elements_.size()) {
            return false;
        }
        element = &elements_[current_index_++];
        return true;
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_index_ = 0;
    }

private:
    std::mutex mutex_;
    std::vector<T, Eigen::aligned_allocator<T>>& elements_;
    std::size_t current_index_ = 0;
};

template <typename T>
class constVectorElementGetter {
public:
    explicit constVectorElementGetter(const std::vector<T, Eigen::aligned_allocator<T>>& elements)
        : elements_(elements) {}

    bool get_ptr(const T*& element)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_index_ >= elements_.size()) {
            return false;
        }
        element = &elements_[current_index_++];
        return true;
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_index_ = 0;
    }

private:
    std::mutex mutex_;
    const std::vector<T, Eigen::aligned_allocator<T>>& elements_;
    std::size_t current_index_ = 0;
};

inline void incrementIdx(Index& idx, const unsigned int& max_idx)
{
    ++idx.z();
    if (idx.z() >= static_cast<int>(max_idx)) {
        idx.z() = 0;
        ++idx.y();
        if (idx.y() >= static_cast<int>(max_idx)) {
            idx.y() = 0;
            ++idx.x();
        }
    }
}

inline void incrementIdx(Index& idx, const Index& max_idx)
{
    ++idx.z();
    if (idx.z() >= max_idx.z()) {
        idx.z() = 0;
        ++idx.y();
        if (idx.y() >= max_idx.y()) {
            idx.y() = 0;
            ++idx.x();
        }
    }
}

inline void incrementIdx(Index& idx, const Index& min_idx, const Index& max_idx)
{
    ++idx.z();
    if (idx.z() >= max_idx.z()) {
        idx.z() = min_idx.z();
        ++idx.y();
        if (idx.y() >= max_idx.y()) {
            idx.y() = min_idx.y();
            ++idx.x();
        }
    }
}

class Neighbours {
public:
    void set_params(unsigned int connectivity)
    {
        offsets.clear();
        offsets.reserve(26);
        offsets.emplace_back(-1, 0, 0);
        offsets.emplace_back(1, 0, 0);
        offsets.emplace_back(0, -1, 0);
        offsets.emplace_back(0, 1, 0);
        offsets.emplace_back(0, 0, -1);
        offsets.emplace_back(0, 0, 1);
        offsets.emplace_back(-1, -1, 0);
        offsets.emplace_back(-1, 1, 0);
        offsets.emplace_back(1, -1, 0);
        offsets.emplace_back(1, 1, 0);
        offsets.emplace_back(0, -1, -1);
        offsets.emplace_back(0, -1, 1);
        offsets.emplace_back(0, 1, -1);
        offsets.emplace_back(0, 1, 1);
        offsets.emplace_back(-1, 0, -1);
        offsets.emplace_back(1, 0, -1);
        offsets.emplace_back(-1, 0, 1);
        offsets.emplace_back(1, 0, 1);
        offsets.emplace_back(-1, -1, -1);
        offsets.emplace_back(-1, -1, 1);
        offsets.emplace_back(-1, 1, -1);
        offsets.emplace_back(-1, 1, 1);
        offsets.emplace_back(1, -1, -1);
        offsets.emplace_back(1, -1, 1);
        offsets.emplace_back(1, 1, -1);
        offsets.emplace_back(1, 1, 1);
        offsets.resize(connectivity);
    }

    std::vector<Index> offsets;
};

} // namespace freedom

#endif // FREEDOM_UTILS_CORE_H

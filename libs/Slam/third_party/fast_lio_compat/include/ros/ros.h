#pragma once

#include <cassert>
#include <cstdint>
#include <string>

using uint = unsigned int;

namespace ros {

class Time {
public:
    Time() = default;
    explicit Time(double seconds) : seconds_(seconds) {}

    double toSec() const { return seconds_; }
    Time fromSec(double seconds) const { return Time(seconds); }
    static Time now() { return Time(); }

private:
    double seconds_ = 0.0;
};

struct Header {
    Time stamp;
    std::string frame_id;
};

class Publisher {};

} // namespace ros

#define ROS_ASSERT(expr) assert(expr)
#define ROS_INFO(...) ((void)0)
#define ROS_WARN(...) ((void)0)

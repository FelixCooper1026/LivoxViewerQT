#pragma once

#include "ros/ros.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace livox_ros_driver {

struct CustomPoint {
    uint32_t offset_time = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    uint8_t reflectivity = 0;
    uint8_t tag = 0;
    uint8_t line = 0;
};

struct CustomMsg {
    using Ptr = std::shared_ptr<CustomMsg>;
    using ConstPtr = std::shared_ptr<const CustomMsg>;

    ros::Header header;
    uint32_t point_num = 0;
    std::vector<CustomPoint> points;
};

} // namespace livox_ros_driver

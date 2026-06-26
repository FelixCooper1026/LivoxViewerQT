#pragma once

#include "ros/ros.h"

#include <memory>

namespace sensor_msgs {

struct PointCloud2 {
    using Ptr = std::shared_ptr<PointCloud2>;
    using ConstPtr = std::shared_ptr<const PointCloud2>;

    ros::Header header;
};

} // namespace sensor_msgs

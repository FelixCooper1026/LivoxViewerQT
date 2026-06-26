#pragma once

#include <Eigen/Core>

#include <cstdint>

#ifndef PCL_ADD_POINT4D
#define PCL_ADD_POINT4D float x = 0.0f; float y = 0.0f; float z = 0.0f; float data[4] = {}
#endif

#ifndef POINT_CLOUD_REGISTER_POINT_STRUCT
#define POINT_CLOUD_REGISTER_POINT_STRUCT(...)
#endif

namespace pcl {

struct PointXYZI {
    PCL_ADD_POINT4D;
    float intensity = 0.0f;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointXYZINormal {
    PCL_ADD_POINT4D;
    float intensity = 0.0f;
    float normal_x = 0.0f;
    float normal_y = 0.0f;
    float normal_z = 0.0f;
    float curvature = 0.0f;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace pcl

#ifndef SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMTYPES_H
#define SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMTYPES_H

#include <deque>

#include "common_lib.h"

#include <pcl/point_types.h>

struct EIGEN_ALIGN16 PointTypePose {
    PCL_ADD_POINT4D;
    float intensity = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    double time = 0.0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointTypePose,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, roll, roll)
    (float, pitch, pitch)
    (float, yaw, yaw)
    (double, time, time))

#endif // SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMTYPES_H

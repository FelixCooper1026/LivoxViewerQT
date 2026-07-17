#pragma once

#include "pcl/point_cloud.h"

#include <Eigen/Geometry>

#include <cmath>

namespace pcl {

inline Eigen::Affine3f getTransformation(float x,
                                         float y,
                                         float z,
                                         float roll,
                                         float pitch,
                                         float yaw)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() = Eigen::Vector3f(x, y, z);
    transform.linear() = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                          Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                          Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()))
                             .toRotationMatrix();
    return transform;
}

inline void getTranslationAndEulerAngles(const Eigen::Affine3f& transform,
                                         float& x,
                                         float& y,
                                         float& z,
                                         float& roll,
                                         float& pitch,
                                         float& yaw)
{
    x = transform.translation().x();
    y = transform.translation().y();
    z = transform.translation().z();
    const Eigen::Vector3f euler = transform.linear().eulerAngles(2, 1, 0);
    roll = euler(2);
    pitch = euler(1);
    yaw = euler(0);
}

template <typename PointT>
void transformPointCloud(const PointCloud<PointT>& source,
                         PointCloud<PointT>& target,
                         const Eigen::Matrix4f& transform)
{
    target.resize(source.points.size());
    for (std::size_t i = 0; i < source.points.size(); ++i) {
        const PointT& sourcePoint = source.points[i];
        PointT& targetPoint = target.points[i];
        targetPoint = sourcePoint;
        const Eigen::Vector4f transformed =
            transform * Eigen::Vector4f(sourcePoint.x, sourcePoint.y, sourcePoint.z, 1.0f);
        targetPoint.x = transformed.x();
        targetPoint.y = transformed.y();
        targetPoint.z = transformed.z();
    }
    target.width = static_cast<uint32_t>(target.points.size());
    target.height = 1;
}

} // namespace pcl

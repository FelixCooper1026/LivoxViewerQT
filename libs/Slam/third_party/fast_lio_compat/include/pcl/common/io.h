#pragma once

#include "pcl/point_cloud.h"

namespace pcl {

template <typename SourcePointT, typename TargetPointT>
void copyPointCloud(const PointCloud<SourcePointT>& source, PointCloud<TargetPointT>& target)
{
    target.clear();
    target.reserve(source.points.size());
    for (const SourcePointT& sourcePoint : source.points) {
        TargetPointT targetPoint;
        targetPoint.x = sourcePoint.x;
        targetPoint.y = sourcePoint.y;
        targetPoint.z = sourcePoint.z;
        targetPoint.intensity = sourcePoint.intensity;
        target.push_back(targetPoint);
    }
    target.width = static_cast<uint32_t>(target.points.size());
    target.height = 1;
}

template <typename PointT>
void copyPointCloud(const PointCloud<PointT>& source, PointCloud<PointT>& target)
{
    target = source;
}

} // namespace pcl

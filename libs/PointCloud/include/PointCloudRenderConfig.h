#ifndef POINTCLOUD_POINTCLOUDRENDERCONFIG_H
#define POINTCLOUD_POINTCLOUDRENDERCONFIG_H

struct PointCloudEdlConfig
{
    bool enabled = true;
    float strength = 100.0f;
    float radiusPx = 3.0f;
    float renderScale = 1.0f;
    bool roundPointSplat = true;
};

#endif // POINTCLOUD_POINTCLOUDRENDERCONFIG_H

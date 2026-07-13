#ifndef POINTCLOUD_POINTCLOUDRENDERCONFIG_H
#define POINTCLOUD_POINTCLOUDRENDERCONFIG_H

struct PointCloudEdlConfig
{
    bool enabled = true;
    float strength = 2.5f;
    float radiusPx = 2.3f;
    float silhouetteStrength = 0.2f;
    float minimumShade = 0.35f;
    int sampleCount = 8;
    float renderScale = 1.0f;
    bool roundPointSplat = true;
};

#endif // POINTCLOUD_POINTCLOUDRENDERCONFIG_H

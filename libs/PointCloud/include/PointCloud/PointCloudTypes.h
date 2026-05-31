#ifndef POINTCLOUD_POINTCLOUDTYPES_H
#define POINTCLOUD_POINTCLOUDTYPES_H

#include <cstdint>

struct PointCloudPoint {
    float x, y, z;
    float r, g, b;
    uint8_t reflectivity;
    uint8_t tag;
    uint8_t line;
    bool spherical;
    float theta;
    float phi;
    float depth;
};

#endif // POINTCLOUD_POINTCLOUDTYPES_H

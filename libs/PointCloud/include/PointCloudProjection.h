#ifndef POINTCLOUD_POINTCLOUDPROJECTION_H
#define POINTCLOUD_POINTCLOUDPROJECTION_H

#include "PointCloudTypes.h"

#include <cstdint>

namespace PointCloudProjection {

struct SphericalConfig {
    bool depthProjectionEnabled = false;
    float depthMeters = 1.0f;
    bool planarProjectionEnabled = false;
    float planarRadius = 10.0f;
    int lineCount = 1;
};

PointCloudPoint projectSpherical(uint32_t depthRaw,
                                 uint16_t thetaRaw,
                                 uint16_t phiRaw,
                                 uint8_t reflectivity,
                                 uint8_t tag,
                                 const SphericalConfig& config);

} // namespace PointCloudProjection

#endif // POINTCLOUD_POINTCLOUDPROJECTION_H

#include "PointCloudProjection.h"

#include <cmath>

namespace PointCloudProjection {

namespace {

constexpr float kPi = 3.14159265358979323846f;

} // namespace

PointCloudPoint projectSpherical(uint32_t depthRaw,
                                 uint16_t thetaRaw,
                                 uint16_t phiRaw,
                                 uint8_t reflectivity,
                                 uint8_t tag,
                                 const SphericalConfig& config)
{
    PointCloudPoint point{};
    float depth = depthRaw / 1000.0f;
    point.spherical = true;
    point.theta = thetaRaw / 100.0f;
    point.phi = phiRaw / 100.0f;
    point.depth = depth;

    const float theta = thetaRaw / 100.0f * kPi / 180.0f;
    const float phi = phiRaw / 100.0f * kPi / 180.0f;

    if (config.depthProjectionEnabled && config.depthMeters > 0.0f) {
        depth = config.depthMeters;
    }

    if (config.planarProjectionEnabled && config.depthMeters <= 0.0f) {
        depth = config.planarRadius;
    }

    if (config.planarProjectionEnabled) {
        float phiDeg = phi * 180.0f / kPi;
        float thetaDeg = theta * 180.0f / kPi;
        if (phiDeg > 180.0f) {
            phiDeg -= 360.0f;
        }
        thetaDeg = 90.0f - thetaDeg;

        point.x = config.planarRadius * phiDeg / 180.0f;
        point.y = config.planarRadius * thetaDeg / 90.0f;
        point.z = 0.0f;
    } else {
        point.x = depth * std::sin(theta) * std::cos(phi);
        point.y = depth * std::sin(theta) * std::sin(phi);
        point.z = depth * std::cos(theta);
    }

    point.reflectivity = reflectivity;
    point.tag = tag;
    return point;
}

} // namespace PointCloudProjection

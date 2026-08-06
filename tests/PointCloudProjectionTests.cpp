#include "PointCloudProjection.h"

#include <cmath>
#include <iostream>

namespace {

int failureCount = 0;

void expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failureCount;
    }
}

bool nearlyEqual(float actual, float expected)
{
    return std::abs(actual - expected) < 0.0001f;
}

void expectZeroCoordinates(const PointCloudPoint& point, const char* mode)
{
    expect(point.x == 0.0f && point.y == 0.0f && point.z == 0.0f, mode);
    expect(point.depth == 0.0f, "zero-depth point keeps original depth");
}

} // namespace

int main()
{
    PointCloudProjection::SphericalConfig config;

    PointCloudPoint point = PointCloudProjection::projectSpherical(0, 4500, 9000, 12, 3, config);
    expectZeroCoordinates(point, "unprojected zero-depth point stays at origin");
    expect(point.spherical, "zero-depth point keeps spherical metadata");
    expect(nearlyEqual(point.theta, 45.0f) && nearlyEqual(point.phi, 90.0f),
           "zero-depth point keeps angles");
    expect(point.reflectivity == 12 && point.tag == 3,
           "zero-depth point keeps reflectivity and tag");

    config.depthProjectionEnabled = true;
    config.depthMeters = 12.0f;
    point = PointCloudProjection::projectSpherical(0, 4500, 9000, 12, 3, config);
    expectZeroCoordinates(point, "spherical projection does not project zero-depth point");

    point = PointCloudProjection::projectSpherical(2500, 9000, 0, 12, 3, config);
    expect(nearlyEqual(point.x, 12.0f) && nearlyEqual(point.y, 0.0f) && nearlyEqual(point.z, 0.0f),
           "spherical projection still projects valid point to configured radius");
    expect(nearlyEqual(point.depth, 2.5f), "projected valid point keeps original depth metadata");

    config.depthProjectionEnabled = false;
    config.planarProjectionEnabled = true;
    config.planarRadius = 10.0f;
    point = PointCloudProjection::projectSpherical(0, 4500, 9000, 12, 3, config);
    expectZeroCoordinates(point, "planar projection does not project zero-depth point");

    point = PointCloudProjection::projectSpherical(2500, 4500, 9000, 12, 3, config);
    expect(nearlyEqual(point.x, 5.0f) && nearlyEqual(point.y, 5.0f) && nearlyEqual(point.z, 0.0f),
           "planar projection still projects valid point");

    if (failureCount == 0) {
        std::cout << "PointCloudProjectionTests passed\n";
    }
    return failureCount == 0 ? 0 : 1;
}

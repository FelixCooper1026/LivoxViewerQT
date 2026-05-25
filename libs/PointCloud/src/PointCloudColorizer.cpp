#include "PointCloud/PointCloudColorizer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace PointCloudColorizer {

namespace {

constexpr int kColorByReflectivity = 0;
constexpr int kColorByDistance = 1;
constexpr int kColorByElevation = 2;
constexpr int kColorSolid = 3;
constexpr int kColorByPlanarProjection = 4;

void calculateReflectivityColor(uint8_t reflectivity, float& r, float& g, float& b)
{
    if (reflectivity < 30) {
        r = 0.0f;
        g = static_cast<float>(reflectivity * 255 / 30) / 255.0f;
        b = 1.0f;
    } else if (reflectivity < 90) {
        r = 0.0f;
        g = 1.0f;
        b = static_cast<float>((90 - reflectivity) * 255 / 60) / 255.0f;
    } else if (reflectivity < 150) {
        r = static_cast<float>((reflectivity - 90) * 255 / 60) / 255.0f;
        g = 1.0f;
        b = 0.0f;
    } else {
        r = 1.0f;
        g = static_cast<float>((255 - reflectivity) * 255 / (256 - 150)) / 255.0f;
        b = 0.0f;
    }
}

} // namespace

PointCloudPipelineLegend apply(QVector<PointCloudPoint>& points, const Config& config)
{
    PointCloudPipelineLegend legend;
    legend.mode = config.mode;
    legend.minValue = 0.0f;
    legend.maxValue = 1.0f;
    legend.visible = (config.mode != kColorSolid);

    if (points.isEmpty()) {
        return legend;
    }

    if (config.mode == kColorByReflectivity) {
        for (PointCloudPoint& point : points) {
            calculateReflectivityColor(point.reflectivity, point.r, point.g, point.b);
        }
        legend.minValue = 0.0f;
        legend.maxValue = 255.0f;
        legend.visible = true;
    } else if (config.mode == kColorByDistance) {
        const float minD = config.distanceColorMin;
        const float maxD = config.distanceColorMax;
        for (PointCloudPoint& point : points) {
            const float d = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            float t = (d - minD) / (maxD - minD);
            t = std::clamp(t, 0.0f, 1.0f);
            if (t < 0.25f) {
                point.r = 0.0f;
                point.g = t / 0.25f;
                point.b = 1.0f;
            } else if (t < 0.5f) {
                point.r = 0.0f;
                point.g = 1.0f;
                point.b = 1.0f - (t - 0.25f) / 0.25f;
            } else if (t < 0.75f) {
                point.r = (t - 0.5f) / 0.25f;
                point.g = 1.0f;
                point.b = 0.0f;
            } else {
                point.r = 1.0f;
                point.g = 1.0f - (t - 0.75f) / 0.25f;
                point.b = 0.0f;
            }
        }
        legend.minValue = minD;
        legend.maxValue = maxD;
        legend.visible = true;
    } else if (config.mode == kColorByElevation) {
        const float minZ = config.elevationColorMin;
        const float maxZ = config.elevationColorMax;
        for (PointCloudPoint& point : points) {
            float t = (point.z - minZ) / (maxZ - minZ);
            t = std::clamp(t, 0.0f, 1.0f);
            point.r = t;
            point.g = 0.0f;
            point.b = 1.0f - t;
        }
        legend.minValue = minZ;
        legend.maxValue = maxZ;
        legend.visible = true;
    } else if (config.mode == kColorSolid) {
        for (PointCloudPoint& point : points) {
            point.r = config.solidColor.redF();
            point.g = config.solidColor.greenF();
            point.b = config.solidColor.blueF();
        }
        legend.visible = false;
    } else if (config.mode == kColorByPlanarProjection) {
        float minX = std::numeric_limits<float>::max();
        float maxX = std::numeric_limits<float>::lowest();
        float minY = std::numeric_limits<float>::max();
        float maxY = std::numeric_limits<float>::lowest();
        for (const PointCloudPoint& point : points) {
            minX = std::min(minX, point.x);
            maxX = std::max(maxX, point.x);
            minY = std::min(minY, point.y);
            maxY = std::max(maxY, point.y);
        }
        if (!(maxX > minX)) {
            minX = -config.planarProjectionRadius;
            maxX = config.planarProjectionRadius;
        }
        if (!(maxY > minY)) {
            minY = 0.0f;
            maxY = config.planarProjectionRadius;
        }
        for (PointCloudPoint& point : points) {
            float tx = (point.x - minX) / (maxX - minX);
            float ty = (point.y - minY) / (maxY - minY);
            tx = std::clamp(tx, 0.0f, 1.0f);
            ty = std::clamp(ty, 0.0f, 1.0f);
            const float hue = tx * 360.0f;
            const float saturation = 0.8f;
            const float value = 0.5f + ty * 0.5f;
            const float c = value * saturation;
            const float x = c * (1.0f - std::abs(std::fmod(hue / 60.0f, 2.0f) - 1.0f));
            const float m = value - c;
            if (hue < 60.0f) {
                point.r = c + m;
                point.g = x + m;
                point.b = m;
            } else if (hue < 120.0f) {
                point.r = x + m;
                point.g = c + m;
                point.b = m;
            } else if (hue < 180.0f) {
                point.r = m;
                point.g = c + m;
                point.b = x + m;
            } else if (hue < 240.0f) {
                point.r = m;
                point.g = x + m;
                point.b = c + m;
            } else if (hue < 300.0f) {
                point.r = x + m;
                point.g = m;
                point.b = c + m;
            } else {
                point.r = c + m;
                point.g = m;
                point.b = x + m;
            }
            point.r = std::clamp(point.r, 0.0f, 1.0f);
            point.g = std::clamp(point.g, 0.0f, 1.0f);
            point.b = std::clamp(point.b, 0.0f, 1.0f);
        }
        legend.visible = true;
    }

    return legend;
}

} // namespace PointCloudColorizer

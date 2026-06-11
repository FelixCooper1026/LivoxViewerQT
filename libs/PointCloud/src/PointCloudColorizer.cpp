#include "PointCloud/PointCloudColorizer.h"

#include <algorithm>
#include <cmath>

namespace PointCloudColorizer {

namespace {

constexpr int kColorByReflectivity = 0;
constexpr int kColorByDistance = 1;
constexpr int kColorByElevation = 2;
constexpr int kColorSolid = 3;
constexpr int kColorByLine = 4;

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
    if (config.mode == kColorByLine) {
        legend.lineColors = config.lineColors;
        legend.lineNumbers.reserve(config.lineColors.size());
        for (int i = 0; i < config.lineColors.size(); ++i) {
            legend.lineNumbers.append(i);
        }
    }

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
    } else if (config.mode == kColorByLine) {
        QVector<int> usedLines;
        for (PointCloudPoint& point : points) {
            const int line = int(point.line);
            const QColor color = config.lineColors.at(line % config.lineColors.size());
            point.r = color.redF();
            point.g = color.greenF();
            point.b = color.blueF();
            if (!usedLines.contains(line)) {
                usedLines.append(line);
            }
        }
        std::sort(usedLines.begin(), usedLines.end());
        legend.lineNumbers = usedLines;
        legend.lineColors.clear();
        for (int line : usedLines) {
            legend.lineColors.append(config.lineColors.at(line % config.lineColors.size()));
        }
        legend.visible = true;
    }

    return legend;
}

} // namespace PointCloudColorizer

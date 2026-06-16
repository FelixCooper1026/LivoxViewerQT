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

QColor interpolateColor(const QVector<QColor>& colors, float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    const int colorCount = int(colors.size());
    const float scaled = t * float(colorCount - 1);
    const int index = std::min(int(std::floor(scaled)), colorCount - 2);
    const float localT = scaled - float(index);
    const QColor a = colors.at(index);
    const QColor b = colors.at(index + 1);
    return QColor::fromRgbF(a.redF() + (b.redF() - a.redF()) * localT,
                            a.greenF() + (b.greenF() - a.greenF()) * localT,
                            a.blueF() + (b.blueF() - a.blueF()) * localT);
}

void calculateReflectivityColor(uint8_t reflectivity, int scale, float& r, float& g, float& b)
{
    const QColor color = interpolateColor(reflectivityColorScaleStops(scale), float(reflectivity) / 255.0f);
    r = color.redF();
    g = color.greenF();
    b = color.blueF();
}

} // namespace

QVector<QColor> reflectivityColorScaleStops(int scale)
{
    switch (scale) {
    case ReflectivityRainbow:
        return {QColor("#0000FF"), QColor("#00FFFF"), QColor("#00FF00"), QColor("#FFFF00"), QColor("#FF0000")};
    case ReflectivityViridis:
        return {QColor("#440154"), QColor("#3B528B"), QColor("#21918C"), QColor("#5EC962"), QColor("#FDE725")};
    case ReflectivityTurbo:
        return {QColor("#30123B"), QColor("#4664D7"), QColor("#1AE4B6"), QColor("#A4FC3C"), QColor("#FABA39"), QColor("#C40000")};
    case ReflectivityCividis:
        return {QColor("#00204C"), QColor("#31446B"), QColor("#666870"), QColor("#A69D75"), QColor("#FFE945")};
    case ReflectivityHighContrast:
        return {QColor("#000000"), QColor("#0000FF"), QColor("#00FFFF"), QColor("#FFFF00"), QColor("#FF0000"), QColor("#FFFFFF")};
    case ReflectivityGrayscale:
        return {QColor("#000000"), QColor("#FFFFFF")};
    case ReflectivityBGYR:
    default:
        return {QColor("#0000FF"), QColor("#00FF00"), QColor("#FFFF00"), QColor("#FF0000")};
    }
}

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
            calculateReflectivityColor(point.reflectivity, config.reflectivityColorScale, point.r, point.g, point.b);
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

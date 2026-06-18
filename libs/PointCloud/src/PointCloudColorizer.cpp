#include "PointCloud/PointCloudColorizer.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace PointCloudColorizer {

namespace {

constexpr int kColorByReflectivity = 0;
constexpr int kColorByDistance = 1;
constexpr int kColorByElevation = 2;
constexpr int kColorSolid = 3;
constexpr int kColorByLine = 4;
constexpr int kReflectivityScaleCount = ReflectivityGrayscale + 1;
constexpr int kReflectivityValueCount = 256;

struct Rgb {
    float r;
    float g;
    float b;
};

struct ColorScaleDefinition {
    std::array<Rgb, 6> colors;
    int colorCount;
};

constexpr Rgb rgb(int r, int g, int b)
{
    return {float(r) / 255.0f, float(g) / 255.0f, float(b) / 255.0f};
}

constexpr std::array<ColorScaleDefinition, kReflectivityScaleCount> kReflectivityColorScales = {{
    {std::array<Rgb, 6>{rgb(0, 0, 255), rgb(0, 255, 0), rgb(255, 255, 0), rgb(255, 0, 0)}, 4},
    {std::array<Rgb, 6>{rgb(0, 0, 255), rgb(0, 255, 255), rgb(0, 255, 0), rgb(255, 255, 0), rgb(255, 0, 0)}, 5},
    {std::array<Rgb, 6>{rgb(68, 1, 84), rgb(59, 82, 139), rgb(33, 145, 140), rgb(94, 201, 98), rgb(253, 231, 37)}, 5},
    {std::array<Rgb, 6>{rgb(48, 18, 59), rgb(70, 100, 215), rgb(26, 228, 182), rgb(164, 252, 60), rgb(250, 186, 57), rgb(196, 0, 0)}, 6},
    {std::array<Rgb, 6>{rgb(0, 32, 76), rgb(49, 68, 107), rgb(102, 104, 112), rgb(166, 157, 117), rgb(255, 233, 69)}, 5},
    {std::array<Rgb, 6>{rgb(0, 0, 0), rgb(0, 0, 255), rgb(0, 255, 255), rgb(255, 255, 0), rgb(255, 0, 0), rgb(255, 255, 255)}, 6},
    {std::array<Rgb, 6>{rgb(0, 0, 0), rgb(255, 255, 255)}, 2}
}};

Rgb interpolateColor(const ColorScaleDefinition& scale, float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    const int colorCount = scale.colorCount;
    const float scaled = t * float(colorCount - 1);
    const int index = std::min(int(std::floor(scaled)), colorCount - 2);
    const float localT = scaled - float(index);
    const Rgb& a = scale.colors[size_t(index)];
    const Rgb& b = scale.colors[size_t(index + 1)];
    return {a.r + (b.r - a.r) * localT,
            a.g + (b.g - a.g) * localT,
            a.b + (b.b - a.b) * localT};
}

std::array<std::array<Rgb, kReflectivityValueCount>, kReflectivityScaleCount> buildReflectivityLookupTables()
{
    std::array<std::array<Rgb, kReflectivityValueCount>, kReflectivityScaleCount> tables;
    for (int scale = 0; scale < kReflectivityScaleCount; ++scale) {
        for (int reflectivity = 0; reflectivity < kReflectivityValueCount; ++reflectivity) {
            tables[size_t(scale)][size_t(reflectivity)] =
                interpolateColor(kReflectivityColorScales[size_t(scale)], float(reflectivity) / 255.0f);
        }
    }
    return tables;
}

const std::array<std::array<Rgb, kReflectivityValueCount>, kReflectivityScaleCount>& reflectivityLookupTables()
{
    static const std::array<std::array<Rgb, kReflectivityValueCount>, kReflectivityScaleCount> tables =
        buildReflectivityLookupTables();
    return tables;
}

void calculateReflectivityColor(uint8_t reflectivity, int scale, float& r, float& g, float& b)
{
    const Rgb& color = reflectivityLookupTables()[size_t(scale)][size_t(reflectivity)];
    r = color.r;
    g = color.g;
    b = color.b;
}

} // namespace

QVector<QColor> reflectivityColorScaleStops(int scale)
{
    const ColorScaleDefinition& scaleDefinition = kReflectivityColorScales[size_t(scale)];
    QVector<QColor> colors;
    colors.reserve(scaleDefinition.colorCount);
    for (int i = 0; i < scaleDefinition.colorCount; ++i) {
        const Rgb& color = scaleDefinition.colors[size_t(i)];
        colors.append(QColor::fromRgbF(color.r, color.g, color.b));
    }
    return colors;
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

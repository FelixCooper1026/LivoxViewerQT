#ifndef POINTCLOUD_POINTCLOUDCOLORIZER_H
#define POINTCLOUD_POINTCLOUDCOLORIZER_H

#include "PointCloud/PointCloudFrame.h"
#include "PointCloud/PointCloudPipeline.h"

#include <QColor>
#include <QVector>

namespace PointCloudColorizer {

enum ReflectivityColorScale {
    ReflectivityBGYR = 0,
    ReflectivityRainbow = 1,
    ReflectivityViridis = 2,
    ReflectivityTurbo = 3,
    ReflectivityCividis = 4,
    ReflectivityHighContrast = 5,
    ReflectivityGrayscale = 6,
    ReflectivityPlasma = 7,
    ReflectivityTerrain = 8,
    ReflectivityInferno = 9
};

struct Config {
    int mode = 0;
    int reflectivityColorScale = ReflectivityBGYR;
    QColor solidColor = QColor(255, 255, 255);
    QVector<QColor> lineColors = {
        QColor(33, 150, 243),
        QColor(46, 204, 113),
        QColor(255, 193, 7),
        QColor(233, 30, 99)
    };
    float distanceColorMin = 0.0f;
    float distanceColorMax = 100.0f;
    float elevationColorMin = -5.0f;
    float elevationColorMax = 5.0f;
};

QVector<QColor> reflectivityColorScaleStops(int scale);

PointCloudPipelineLegend apply(QVector<PointCloudPoint>& points, const Config& config);

} // namespace PointCloudColorizer

#endif // POINTCLOUD_POINTCLOUDCOLORIZER_H

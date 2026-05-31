#ifndef POINTCLOUD_POINTCLOUDCOLORIZER_H
#define POINTCLOUD_POINTCLOUDCOLORIZER_H

#include "PointCloud/PointCloudFrame.h"
#include "PointCloud/PointCloudPipeline.h"

#include <QColor>
#include <QVector>

namespace PointCloudColorizer {

struct Config {
    int mode = 0;
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
    float planarProjectionRadius = 10.0f;
};

PointCloudPipelineLegend apply(QVector<PointCloudPoint>& points, const Config& config);

} // namespace PointCloudColorizer

#endif // POINTCLOUD_POINTCLOUDCOLORIZER_H

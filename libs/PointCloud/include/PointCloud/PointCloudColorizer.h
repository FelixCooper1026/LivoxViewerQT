#ifndef POINTCLOUD_POINTCLOUDCOLORIZER_H
#define POINTCLOUD_POINTCLOUDCOLORIZER_H

#include "PointCloud/PointCloudFrame.h"
#include "PointCloud/PointCloudPipeline.h"

#include <QColor>

namespace PointCloudColorizer {

struct Config {
    int mode = 0;
    QColor solidColor = QColor(255, 255, 255);
    float distanceColorMin = 0.0f;
    float distanceColorMax = 100.0f;
    float elevationColorMin = -5.0f;
    float elevationColorMax = 5.0f;
    float planarProjectionRadius = 10.0f;
};

PointCloudPipelineLegend apply(QVector<PointCloudPoint>& points, const Config& config);

} // namespace PointCloudColorizer

#endif // POINTCLOUD_POINTCLOUDCOLORIZER_H

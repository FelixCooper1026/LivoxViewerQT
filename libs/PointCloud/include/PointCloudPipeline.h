#ifndef POINTCLOUD_POINTCLOUDPIPELINE_H
#define POINTCLOUD_POINTCLOUDPIPELINE_H

#include "PointCloudFrame.h"

#include <QColor>
#include <QVector>

enum class PointCloudProjectionMode {
    None = 0,
    SphericalDepth = 1,
    Planar = 2
};

struct PointCloudPipelineLegend {
    int mode = 0;
    float minValue = 0.0f;
    float maxValue = 1.0f;
    bool visible = false;
    QVector<QColor> lineColors;
    QVector<int> lineNumbers;
};

#endif // POINTCLOUD_POINTCLOUDPIPELINE_H

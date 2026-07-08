#ifndef POINTCLOUD_POINTCLOUDFRAME_H
#define POINTCLOUD_POINTCLOUDFRAME_H

#include "PointCloudTypes.h"

#include <QMap>
#include <QVector>
#include <cstdint>

struct PointCloudFrame {
    QVector<PointCloudPoint> points;
    QMap<uint32_t, QVector<PointCloudPoint>> pointsByLidar;
    uint64_t timestamp;
    uint32_t device_handle;
};

#endif // POINTCLOUD_POINTCLOUDFRAME_H

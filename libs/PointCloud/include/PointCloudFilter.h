#ifndef POINTCLOUD_POINTCLOUDFILTER_H
#define POINTCLOUD_POINTCLOUDFILTER_H

#include "PointCloudTypes.h"

#include <QVector>

namespace PointCloudFilter {

struct Config {
    bool showNoisePoints = false;
    bool removeNoisePoints = false;
    QVector<uint8_t> noiseTags;
};

QVector<PointCloudPoint> apply(const QVector<PointCloudPoint>& inputPoints, const Config& config);

} // namespace PointCloudFilter

#endif // POINTCLOUD_POINTCLOUDFILTER_H

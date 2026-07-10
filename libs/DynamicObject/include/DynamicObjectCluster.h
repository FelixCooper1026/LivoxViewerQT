#ifndef DYNAMICOBJECT_DYNAMICOBJECTCLUSTER_H
#define DYNAMICOBJECT_DYNAMICOBJECTCLUSTER_H

#include "DynamicObjectDetector.h"

#include <Eigen/Core>

#include <QVector>

struct DynamicObjectClusterResult {
    QVector<DynamicObjectPoint> dynamicPoints;
    QVector<DynamicObjectLabel> labels;
    int clusterCount = 0;
    int rejectedClusterCount = 0;
    int groundRemovedPointCount = 0;
    double clusterMs = 0.0;
};

class DynamicObjectCluster
{
public:
    explicit DynamicObjectCluster(const DynamicObjectDetectorConfig& config);

    DynamicObjectClusterResult processFrame(const QVector<DynamicObjectPoint>& points,
                                            const Eigen::Vector3d& worldUp) const;

private:
    DynamicObjectDetectorConfig config_;
};

#endif // DYNAMICOBJECT_DYNAMICOBJECTCLUSTER_H

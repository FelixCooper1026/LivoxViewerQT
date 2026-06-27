#ifndef SLAM_VISUALIZATION_SLAMRENDERSNAPSHOT_H
#define SLAM_VISUALIZATION_SLAMRENDERSNAPSHOT_H

#include <QMetaType>
#include <QVector>

struct SlamRenderVertex {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;
};

struct SlamRenderSnapshot {
    QVector<SlamRenderVertex> trajectoryVertices;
    QVector<SlamRenderVertex> poseAxisVertices;
    QVector<SlamRenderVertex> worldFrameVertices;
    QVector<SlamRenderVertex> bodyFrameVertices;
};

Q_DECLARE_METATYPE(SlamRenderSnapshot)

#endif // SLAM_VISUALIZATION_SLAMRENDERSNAPSHOT_H

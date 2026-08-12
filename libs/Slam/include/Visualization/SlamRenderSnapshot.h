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

struct SlamRenderPose {
    bool valid = false;
    float tx = 0.0f;
    float ty = 0.0f;
    float tz = 0.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
};

struct SlamRenderSnapshot {
    float worldFramePointSizePx = 2.0f;
    float bodyFramePointSizePx = 2.5f;
    float dynamicObjectPointSizePx = 4.0f;
    float freeDomVoxelSizeM = 0.0f;
    float freeDomEnhancedPointSizePx = 4.0f;
    quint64 freeDomScanVoxelRevision = 0;
    quint64 freeDomDynamicVoxelRevision = 0;
    quint64 freeDomRaycastedVoxelRevision = 0;
    quint64 freeDomFreeVoxelRevision = 0;
    quint64 freeDomStaticVoxelRevision = 0;
    float trajectoryLineWidthPx = 2.0f;
    float poseAxisLengthM = 0.8f;
    float poseAxisLineWidthPx = 3.0f;
    SlamRenderPose currentPose;
    QVector<SlamRenderVertex> trajectoryVertices;
    QVector<SlamRenderVertex> loopClosureVertices;
    QVector<SlamRenderVertex> poseAxisVertices;
    QVector<SlamRenderVertex> worldFrameVertices;
    QVector<SlamRenderVertex> bodyFrameVertices;
    QVector<SlamRenderVertex> dynamicObjectVertices;
    QVector<SlamRenderVertex> dynamicAggressiveVertices;
    QVector<SlamRenderVertex> dynamicModerateVertices;
    QVector<SlamRenderVertex> dynamicConservativeVertices;
    QVector<SlamRenderVertex> freeDomScanVoxelVertices;
    QVector<SlamRenderVertex> freeDomDynamicVoxelVertices;
    QVector<SlamRenderVertex> freeDomRaycastedVoxelVertices;
    QVector<SlamRenderVertex> freeDomFreeVoxelVertices;
    QVector<SlamRenderVertex> freeDomStaticVoxelVertices;
    QVector<SlamRenderVertex> freeDomEnhancedVertices;
};

Q_DECLARE_METATYPE(SlamRenderSnapshot)
Q_DECLARE_METATYPE(SlamRenderPose)

#endif // SLAM_VISUALIZATION_SLAMRENDERSNAPSHOT_H

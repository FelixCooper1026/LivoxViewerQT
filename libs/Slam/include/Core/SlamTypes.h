#ifndef SLAM_CORE_SLAMTYPES_H
#define SLAM_CORE_SLAMTYPES_H

#include "DynamicFilterBackend.h"

#include <QByteArray>
#include <QString>
#include <QVector>

#include <cstdint>

enum class SlamTimeSource {
    Unknown,
    LivoxPacketTimestamp,
    PcapCaptureTimestamp,
    SynthesizedFromPacketInterval
};

struct SlamPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    uint8_t reflectivity = 0;
    uint8_t tag = 0;
    uint8_t line = 0;
    bool hasLine = false;
    int64_t offsetNs = 0;
    bool hasOffsetTime = false;
};

enum class SlamDynamicPointLabel : uint8_t {
    Static = 0,
    Case1 = 1,
    Case2 = 2,
    Case3 = 3,
    FreeDomAggressive = 20,
    FreeDomModerate = 21,
    FreeDomConservative = 22,
    Invalid = 255
};

struct SlamDynamicPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    uint8_t reflectivity = 0;
    SlamDynamicPointLabel label = SlamDynamicPointLabel::Static;
};

struct SlamDynamicObjectStats {
    DynamicFilterBackend backend = DynamicFilterBackend::Disabled;
    bool enabled = false;
    bool clusterEnabled = false;
    int staticPointCount = 0;
    int dynamicPointCount = 0;
    int originDynamicPointCount = 0;
    int clusterCount = 0;
    int rejectedClusterCount = 0;
    int groundRemovedPointCount = 0;
    int case1PointCount = 0;
    int case2PointCount = 0;
    int case3PointCount = 0;
    int invalidPointCount = 0;
    int inputPointCount = 0;
    int freeDomAggressivePointCount = 0;
    int freeDomModeratePointCount = 0;
    int freeDomConservativePointCount = 0;
    int historyDepthMapCount = 0;
    double detectorMs = 0.0;
    double clusterMs = 0.0;
    double totalMs = 0.0;
    double freeDomBuildScanMapMs = 0.0;
    double freeDomScanRemovalMs = 0.0;
    double freeDomRaycastEnhancementMs = 0.0;
    double freeDomFreeSpaceEstimationMs = 0.0;
    double freeDomMapRemovalMs = 0.0;
    double freeDomStaticIntegrationMs = 0.0;
    int freeDomFreeBlockCount = 0;
    int freeDomFreeVoxelCount = 0;
    int freeDomStaticBlockCount = 0;
    int freeDomStaticVoxelCount = 0;
    int freeDomStaticSubvoxelCount = 0;
};

struct SlamImuSample {
    uint32_t lidarId = 0;
    int64_t timestampNs = 0;
    int64_t rawTimestampNs = 0;
    uint8_t timeType = 0;
    bool hasRawTimestamp = false;
    double gyroRadPerSec[3] = {};
    // Source acceleration scale. Livox packets use g; Fast-LIO normalizes it with the initialization gravity norm.
    double accelRaw[3] = {};
};

struct SlamInputFrame {
    uint64_t sequence = 0;
    uint32_t sourceId = 0;
    uint8_t deviceType = 0;
    int64_t frameStartNs = 0;
    int64_t frameEndNs = 0;
    int64_t rawFrameStartNs = 0;
    int64_t rawFrameEndNs = 0;
    uint8_t timeType = 0;
    bool hasRawFrameTimestamp = false;
    SlamTimeSource timeSource = SlamTimeSource::Unknown;
    QVector<SlamPoint> points;
    QVector<SlamImuSample> imuSamples;
    bool hasPointOffsetTime = false;
    bool hasCompleteImuCoverage = false;
    QString sourceName;
};

struct SlamPose {
    int64_t timestampNs = 0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    double covariance[36] = {};
    bool hasCovariance = false;
    QString poseFrame = "slam_world";
};

struct SlamTrajectoryPoint {
    SlamPose pose;
    double quality = 1.0;
};

struct SlamLoopClosureEdge {
    int currentKeyframeId = -1;
    int previousKeyframeId = -1;
};

enum class SlamStatusCode {
    Idle,
    Starting,
    InitializingImu,
    Running,
    Paused,
    Backpressure,
    MissingImu,
    TimeSyncError,
    Degraded,
    Failed,
    Stopped
};

struct SlamOutput {
    SlamStatusCode status = SlamStatusCode::Idle;
    SlamPose currentPose;
    bool currentPoseValid = false;
    QVector<SlamTrajectoryPoint> newTrajectoryPoints;
    bool optimizedTrajectoryReset = false;
    QVector<SlamTrajectoryPoint> optimizedTrajectory;
    QVector<SlamPoint> publishedWorldFramePoints;
    QVector<SlamPoint> publishedBodyFramePoints;
    QVector<SlamPoint> dynamicDetectionFrameWorldPoints;
    QVector<SlamDynamicPoint> dynamicWorldFramePoints;
    QVector<SlamPoint> freeDomScanVoxelPoints;
    QVector<SlamPoint> freeDomDynamicVoxelPoints;
    QVector<SlamPoint> freeDomRaycastedVoxelPoints;
    QVector<SlamPoint> freeDomFreeVoxelPoints;
    QVector<SlamPoint> freeDomStaticVoxelPoints;
    QVector<SlamPoint> freeDomStaticMapPoints;
    QVector<SlamPoint> freeDomEnhancedPoints;
    QByteArray freeDomDepthImage;
    QByteArray freeDomEnhancedDepthImage;
    int freeDomDepthImageRows = 0;
    int freeDomDepthImageColumns = 0;
    quint64 freeDomSnapshotVersion = 0;
    bool freeDomDebugSnapshotUpdated = false;
    bool freeDomMapSnapshotUpdated = false;
    QVector<SlamPoint> newGlobalMapPoints;
    bool optimizedGlobalMapReset = false;
    QVector<SlamPoint> optimizedGlobalMapPoints;
    QVector<SlamLoopClosureEdge> loopClosureEdges;
    SlamDynamicObjectStats dynamicObjectStats;
    QString message;
    double inputFps = 0.0;
    double backendMs = 0.0;
    int droppedFrameCount = 0;
    int mapPointCount = 0;
    int globalMapPointCount = 0;
    int trajectoryPointCount = 0;
    int keyframeCount = 0;
    int loopClosureCount = 0;
    int poseCorrectionEpoch = 0;
    bool imuHealthy = false;
    bool odometryOnly = false;
};

#endif // SLAM_CORE_SLAMTYPES_H

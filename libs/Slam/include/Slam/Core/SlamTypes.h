#ifndef SLAM_CORE_SLAMTYPES_H
#define SLAM_CORE_SLAMTYPES_H

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

struct SlamImuSample {
    uint32_t lidarId = 0;
    int64_t timestampNs = 0;
    double gyroRadPerSec[3] = {};
    double accelMps2[3] = {};
};

struct SlamInputFrame {
    uint64_t sequence = 0;
    uint32_t sourceId = 0;
    uint8_t deviceType = 0;
    int64_t frameStartNs = 0;
    int64_t frameEndNs = 0;
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
    QVector<SlamTrajectoryPoint> newTrajectoryPoints;
    QVector<SlamPoint> publishedWorldFramePoints;
    QVector<SlamPoint> publishedBodyFramePoints;
    QVector<SlamPoint> newGlobalMapPoints;
    QString message;
    double inputFps = 0.0;
    double backendMs = 0.0;
    int droppedFrameCount = 0;
    int mapPointCount = 0;
    int globalMapPointCount = 0;
    int trajectoryPointCount = 0;
    bool imuHealthy = false;
};

#endif // SLAM_CORE_SLAMTYPES_H

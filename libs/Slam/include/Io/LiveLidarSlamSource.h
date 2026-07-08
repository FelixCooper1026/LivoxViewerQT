#ifndef SLAM_IO_LIVELIDARSLAMSOURCE_H
#define SLAM_IO_LIVELIDARSLAMSOURCE_H

#include "Core/FastLioInputSynchronizer.h"
#include "Core/SlamInputQueue.h"
#include "livox_lidar_def.h"

#include <QElapsedTimer>
#include <QHash>
#include <QMutex>
#include <QString>
#include <QVector>

#include <cstdint>

struct LiveLidarSlamSourceStats {
    SlamStatusCode status = SlamStatusCode::Idle;
    QString message;
    uint64_t pointPacketCount = 0;
    uint64_t imuPacketCount = 0;
    uint64_t inputFrameCount = 0;
    uint64_t pointCount = 0;
    uint64_t imuSampleCount = 0;
    uint64_t incompleteImuCoverageFrameCount = 0;
    uint64_t missingPointOffsetPacketCount = 0;
    uint64_t missingImuTimingPacketCount = 0;
    uint64_t outOfOrderPointPacketCount = 0;
    uint64_t outOfOrderImuPacketCount = 0;
    uint64_t droppedPendingFrameCount = 0;
    uint64_t warmupDroppedFrameCount = 0;
    uint64_t paddedImuFrameCount = 0;
    uint64_t paddedImuSampleCount = 0;
    uint64_t timeResetCount = 0;
    uint64_t inputTimelineResetGeneration = 0;
    uint64_t backendHardResetGeneration = 0;
    uint64_t timeTypeMismatchCount = 0;
    uint64_t timestampJumpCount = 0;
    int queueCapacity = 0;
    int queueSize = 0;
    int pendingFrameCount = 0;
    uint64_t droppedFrameCount = 0;
    double inputFps = 0.0;
    int64_t lastFrameStartNs = 0;
    int64_t lastFrameEndNs = 0;
    int64_t latestPointTimestampNs = 0;
    int64_t latestImuTimestampNs = 0;
    int64_t waitingFrameStartNs = 0;
    int64_t waitingFrameEndNs = 0;
    int64_t imuCoverageGapNs = 0;
    int64_t lastTimestampJumpNs = 0;
    int64_t lastRawTimestampNs = 0;
    int64_t currentRawTimestampNs = 0;
    uint32_t activeHandle = 0;
    uint8_t pointTimeType = 0;
    uint8_t imuTimeType = 0;
    QString lastResetReason;
    SlamStatusCode lastResetStatus = SlamStatusCode::Idle;
    uint64_t lastResetGeneration = 0;
    int64_t lastResetJumpNs = 0;
    int64_t lastResetRawTimestampNs = 0;
    int64_t currentResetRawTimestampNs = 0;
    uint32_t lastResetHandle = 0;
    uint8_t lastResetTimeType = 0;
};

class LiveLidarSlamSource {
public:
    LiveLidarSlamSource();

    void reset();
    void setFrameDurationMs(int frameDurationMs);
    int frameDurationMs() const;
    void setQueueCapacity(int capacity);
    bool appendPointPacket(uint32_t handle,
                           uint8_t deviceType,
                           const LivoxLidarEthernetPacket* packet,
                           const QString& sourceName);
    bool appendImuPacket(uint32_t handle, const LivoxLidarEthernetPacket* packet);
    SlamInputQueue& inputQueue();
    const SlamInputQueue& inputQueue() const;
    LiveLidarSlamSourceStats stats() const;

private:
    struct HandleTimingState {
        bool axisInitialized = false;
        int64_t epochRawStartNs = 0;
        int64_t epochNormalizedStartNs = 0;
        bool hasPointTimeType = false;
        bool hasImuTimeType = false;
        uint8_t pointTimeType = 0;
        uint8_t imuTimeType = 0;
        bool hasLastPointRawTimestamp = false;
        bool hasLastImuRawTimestamp = false;
        int64_t lastPointRawTimestampNs = 0;
        int64_t lastImuRawTimestampNs = 0;
        bool hasLatestPointTimestamp = false;
        bool hasLatestImuTimestamp = false;
        int64_t latestPointTimestampNs = 0;
        int64_t latestImuTimestampNs = 0;
        int consecutiveTimestampJumpCount = 0;
        int consecutiveTimeTypeMismatchCount = 0;
    };

    bool normalizePacketTimestampLocked(uint32_t handle,
                                        uint8_t timeType,
                                        int64_t rawTimestampNs,
                                        bool pointStream,
                                        const QString& sourceName,
                                        int64_t* normalizedTimestampNs);
    void resetLiveTimelineLocked(uint32_t handle,
                                 uint8_t timeType,
                                 int64_t rawTimestampNs,
                                 bool requestBackendHardReset,
                                 SlamStatusCode status,
                                 const QString& message);
    void noteInputTimelineResetLocked(uint32_t handle,
                                      uint8_t timeType,
                                      int64_t lastRawTimestampNs,
                                      int64_t currentRawTimestampNs,
                                      int64_t jumpNs,
                                      SlamStatusCode status,
                                      const QString& message);
    void moveCurrentFrameToPendingLocked();
    void tryFinalizePendingFramesLocked();
    void updatePendingWaitStatusLocked(const SlamInputFrame& frame, int64_t latestImuTimestampNs);

    mutable QMutex mutex_;
    SlamInputQueue queue_;
    FastLioInputSynchronizer synchronizer_;
    SlamInputFrame currentFrame_;
    bool hasCurrentFrame_ = false;
    uint64_t nextSequence_ = 0;
    QHash<uint32_t, HandleTimingState> timingByHandle_;
    uint32_t activeHandle_ = 0;
    bool hasActiveHandle_ = false;
    LiveLidarSlamSourceStats stats_;
    QElapsedTimer statsTimer_;
    int64_t frameDurationNs_ = 100000000;
};

#endif // SLAM_IO_LIVELIDARSLAMSOURCE_H

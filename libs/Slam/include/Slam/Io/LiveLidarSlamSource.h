#ifndef SLAM_IO_LIVELIDARSLAMSOURCE_H
#define SLAM_IO_LIVELIDARSLAMSOURCE_H

#include "Slam/Core/SlamInputQueue.h"
#include "livox_lidar_def.h"

#include <QElapsedTimer>
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
    int queueCapacity = 0;
    int queueSize = 0;
    uint64_t droppedFrameCount = 0;
    double inputFps = 0.0;
    int64_t lastFrameStartNs = 0;
    int64_t lastFrameEndNs = 0;
};

class LiveLidarSlamSource {
public:
    LiveLidarSlamSource();

    void reset();
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
    void flushCurrentFrameLocked();
    void attachImuSamplesLocked(SlamInputFrame& frame) const;
    void pruneImuBufferLocked(int64_t latestTimestampNs);

    mutable QMutex mutex_;
    SlamInputQueue queue_;
    QVector<SlamImuSample> imuBuffer_;
    SlamInputFrame currentFrame_;
    bool hasCurrentFrame_ = false;
    uint64_t nextSequence_ = 0;
    int64_t lastPointPacketTimestampNs_ = 0;
    bool hasLastPointPacketTimestamp_ = false;
    LiveLidarSlamSourceStats stats_;
    QElapsedTimer statsTimer_;
};

#endif // SLAM_IO_LIVELIDARSLAMSOURCE_H

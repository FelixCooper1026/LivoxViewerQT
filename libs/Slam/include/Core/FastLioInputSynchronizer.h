#ifndef SLAM_CORE_FASTLIOINPUTSYNCHRONIZER_H
#define SLAM_CORE_FASTLIOINPUTSYNCHRONIZER_H

#include "Core/SlamTypes.h"

#include <QQueue>
#include <QVector>

#include <cstdint>

class FastLioInputSynchronizer {
public:
    void reset();
    void pushLidarFrame(SlamInputFrame&& frame);
    void pushImuSample(const SlamImuSample& sample);
    bool trySync(SlamInputFrame* frame);

    int pendingLidarFrameCount() const;
    int64_t waitingFrameStartNs() const;
    int64_t waitingFrameEndNs() const;
    int64_t latestImuTimestampNs() const;
    int64_t imuCoverageGapNs() const;

private:
    int64_t computePendingLidarEndNs(const SlamInputFrame& frame);

    QQueue<SlamInputFrame> lidarBuffer_;
    QVector<SlamImuSample> imuBuffer_;
    bool lidarPushed_ = false;
    int64_t pendingLidarEndNs_ = 0;
    double lidarMeanScanTimeNs_ = 0.0;
    int scanNum_ = 0;
    int64_t lastTimestampImuNs_ = 0;
};

QVector<SlamInputFrame> syncFastLioInputFrames(QVector<SlamInputFrame>&& frames,
                                               const QVector<SlamImuSample>& imuSamples);

#endif // SLAM_CORE_FASTLIOINPUTSYNCHRONIZER_H

#include "Core/FastLioInputSynchronizer.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace {

int64_t frameScanTimeNs(const SlamInputFrame& frame)
{
    return frame.frameEndNs - frame.frameStartNs;
}

} // namespace

void FastLioInputSynchronizer::reset()
{
    lidarBuffer_.clear();
    imuBuffer_.clear();
    lidarPushed_ = false;
    pendingLidarEndNs_ = 0;
    lidarMeanScanTimeNs_ = 0.0;
    scanNum_ = 0;
    lastTimestampImuNs_ = 0;
}

void FastLioInputSynchronizer::pushLidarFrame(SlamInputFrame&& frame)
{
    lidarBuffer_.enqueue(std::move(frame));
}

void FastLioInputSynchronizer::pushImuSample(const SlamImuSample& sample)
{
    const auto insertIt = std::upper_bound(imuBuffer_.begin(),
                                           imuBuffer_.end(),
                                           sample.timestampNs,
                                           [](int64_t timestampNs, const SlamImuSample& existing) {
                                               return timestampNs < existing.timestampNs;
                                           });
    imuBuffer_.insert(insertIt, sample);
    if (sample.timestampNs > lastTimestampImuNs_) {
        lastTimestampImuNs_ = sample.timestampNs;
    }
}

bool FastLioInputSynchronizer::trySync(SlamInputFrame* frame)
{
    if (lidarBuffer_.isEmpty() || imuBuffer_.isEmpty()) {
        return false;
    }

    if (!lidarPushed_) {
        pendingLidarEndNs_ = computePendingLidarEndNs(lidarBuffer_.head());
        lidarPushed_ = true;
    }

    if (lastTimestampImuNs_ < pendingLidarEndNs_) {
        return false;
    }

    SlamInputFrame syncedFrame = std::move(lidarBuffer_.head());
    lidarBuffer_.dequeue();
    syncedFrame.frameEndNs = pendingLidarEndNs_;
    syncedFrame.imuSamples.clear();

    int consumedImuCount = 0;
    while (consumedImuCount < imuBuffer_.size() &&
           imuBuffer_.at(consumedImuCount).timestampNs <= pendingLidarEndNs_) {
        syncedFrame.imuSamples.push_back(imuBuffer_.at(consumedImuCount));
        ++consumedImuCount;
    }
    if (consumedImuCount > 0) {
        imuBuffer_.remove(0, consumedImuCount);
    }

    syncedFrame.hasCompleteImuCoverage = !syncedFrame.imuSamples.isEmpty();
    lidarPushed_ = false;
    pendingLidarEndNs_ = 0;
    if (frame != nullptr) {
        *frame = std::move(syncedFrame);
    }
    return true;
}

int FastLioInputSynchronizer::pendingLidarFrameCount() const
{
    return lidarBuffer_.size();
}

int64_t FastLioInputSynchronizer::waitingFrameStartNs() const
{
    return lidarBuffer_.isEmpty() ? 0 : lidarBuffer_.head().frameStartNs;
}

int64_t FastLioInputSynchronizer::waitingFrameEndNs() const
{
    if (lidarBuffer_.isEmpty()) {
        return 0;
    }
    return lidarPushed_ ? pendingLidarEndNs_ : lidarBuffer_.head().frameEndNs;
}

int64_t FastLioInputSynchronizer::latestImuTimestampNs() const
{
    return lastTimestampImuNs_;
}

int64_t FastLioInputSynchronizer::imuCoverageGapNs() const
{
    const int64_t waitingEndNs = waitingFrameEndNs();
    return waitingEndNs > 0 ? waitingEndNs - lastTimestampImuNs_ : 0;
}

int64_t FastLioInputSynchronizer::computePendingLidarEndNs(const SlamInputFrame& frame)
{
    const int64_t scanTimeNs = frameScanTimeNs(frame);
    if (frame.points.size() <= 1) {
        return frame.frameStartNs + int64_t(std::llround(lidarMeanScanTimeNs_));
    }

    if (lidarMeanScanTimeNs_ > 0.0 && double(scanTimeNs) < 0.5 * lidarMeanScanTimeNs_) {
        return frame.frameStartNs + int64_t(std::llround(lidarMeanScanTimeNs_));
    }

    ++scanNum_;
    lidarMeanScanTimeNs_ += (double(scanTimeNs) - lidarMeanScanTimeNs_) / double(scanNum_);
    return frame.frameStartNs + scanTimeNs;
}

QVector<SlamInputFrame> syncFastLioInputFrames(QVector<SlamInputFrame>&& frames,
                                               const QVector<SlamImuSample>& imuSamples)
{
    std::sort(frames.begin(), frames.end(), [](const SlamInputFrame& lhs, const SlamInputFrame& rhs) {
        return lhs.frameStartNs < rhs.frameStartNs;
    });

    QVector<SlamImuSample> sortedImuSamples = imuSamples;
    std::sort(sortedImuSamples.begin(), sortedImuSamples.end(), [](const SlamImuSample& lhs, const SlamImuSample& rhs) {
        return lhs.timestampNs < rhs.timestampNs;
    });

    FastLioInputSynchronizer synchronizer;
    QVector<SlamInputFrame> syncedFrames;
    syncedFrames.reserve(frames.size());

    int frameIndex = 0;
    int imuIndex = 0;
    while (frameIndex < frames.size() || imuIndex < sortedImuSamples.size()) {
        if (frameIndex < frames.size() &&
            (imuIndex >= sortedImuSamples.size() ||
             frames.at(frameIndex).frameStartNs <= sortedImuSamples.at(imuIndex).timestampNs)) {
            synchronizer.pushLidarFrame(std::move(frames[frameIndex]));
            ++frameIndex;
        } else {
            synchronizer.pushImuSample(sortedImuSamples.at(imuIndex));
            ++imuIndex;
        }

        SlamInputFrame syncedFrame;
        while (synchronizer.trySync(&syncedFrame)) {
            syncedFrame.sequence = uint64_t(syncedFrames.size());
            syncedFrames.push_back(std::move(syncedFrame));
        }
    }

    SlamInputFrame syncedFrame;
    while (synchronizer.trySync(&syncedFrame)) {
        syncedFrame.sequence = uint64_t(syncedFrames.size());
        syncedFrames.push_back(std::move(syncedFrame));
    }

    return syncedFrames;
}

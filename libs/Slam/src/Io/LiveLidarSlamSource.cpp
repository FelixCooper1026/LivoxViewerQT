#include "Slam/Io/LiveLidarSlamSource.h"

#include "LivoxCore/LidarModelUtils.h"
#include "LivoxCore/LidarPacketUtils.h"

#include <QMutexLocker>

#include <algorithm>
#include <cmath>
#include <utility>

namespace {

constexpr int64_t kLivoxTimeIntervalUnitNs = 100;
constexpr int64_t kNsPerMs = 1000000;
constexpr int64_t kImuRetentionNs = 5000000000;
constexpr int64_t kImuCoverageMarginNs = 3000000;
constexpr int64_t kMaxPendingFrameWaitNs = 500000000;
constexpr int64_t kMaxImuTailPadNs = 8000000;
constexpr int64_t kMaxImuStartPadNs = 8000000;
constexpr int64_t kSmallTimestampBackstepNs = 5000000;
constexpr int64_t kTimestampJumpResetNs = 1000000000;
constexpr double kLivoxPi = 3.14159265358979323846;

bool hasOffsetTime(uint16_t dotNum, uint16_t timeIntervalRaw)
{
    return dotNum <= 1 || timeIntervalRaw != 0;
}

int64_t pointTimestampNs(int64_t packetTimestampNs, uint16_t timeIntervalRaw, uint32_t index)
{
    return packetTimestampNs + int64_t(index) * int64_t(timeIntervalRaw) * kLivoxTimeIntervalUnitNs;
}

int64_t packetEndTimestampNs(int64_t packetTimestampNs, const LivoxLidarEthernetPacket* packet)
{
    return pointTimestampNs(packetTimestampNs, packet->time_interval, packet->dot_num > 0 ? packet->dot_num - 1 : 0);
}

QString nsToMsText(int64_t ns)
{
    return QString::number(double(ns) / double(kNsPerMs), 'f', 3);
}

QString streamName(bool pointStream)
{
    return pointStream ? QStringLiteral("LiDAR") : QStringLiteral("IMU");
}

void appendPoint(SlamInputFrame& frame, const SlamPoint& point, int64_t timestampNs, bool hasPointOffsetTime)
{
    SlamPoint slamPoint = point;
    slamPoint.offsetNs = timestampNs - frame.frameStartNs;
    slamPoint.hasOffsetTime = hasPointOffsetTime;
    frame.points.push_back(slamPoint);
    if (timestampNs > frame.frameEndNs) {
        frame.frameEndNs = timestampNs;
    }
    if (!hasPointOffsetTime) {
        frame.hasPointOffsetTime = false;
    }
}

void appendCartesianHigh(const LivoxLidarEthernetPacket* packet,
                         int lineCount,
                         int64_t packetTimestampNs,
                         SlamInputFrame& frame)
{
    const auto* points = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(packet->data);
    const bool hasOffsets = hasOffsetTime(packet->dot_num, packet->time_interval);
    for (uint32_t i = 0; i < packet->dot_num; ++i) {
        SlamPoint point;
        point.x = points[i].x / 1000.0f;
        point.y = points[i].y / 1000.0f;
        point.z = points[i].z / 1000.0f;
        point.reflectivity = points[i].reflectivity;
        point.tag = points[i].tag;
        point.line = LivoxCore::lineForPointIndex(int(i), lineCount);
        point.hasLine = true;
        appendPoint(frame, point, pointTimestampNs(packetTimestampNs, packet->time_interval, i), hasOffsets);
    }
}

void appendCartesianLow(const LivoxLidarEthernetPacket* packet,
                        int lineCount,
                        int64_t packetTimestampNs,
                        SlamInputFrame& frame)
{
    const auto* points = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(packet->data);
    const bool hasOffsets = hasOffsetTime(packet->dot_num, packet->time_interval);
    for (uint32_t i = 0; i < packet->dot_num; ++i) {
        SlamPoint point;
        point.x = points[i].x / 100.0f;
        point.y = points[i].y / 100.0f;
        point.z = points[i].z / 100.0f;
        point.reflectivity = points[i].reflectivity;
        point.tag = points[i].tag;
        point.line = LivoxCore::lineForPointIndex(int(i), lineCount);
        point.hasLine = true;
        appendPoint(frame, point, pointTimestampNs(packetTimestampNs, packet->time_interval, i), hasOffsets);
    }
}

void appendSpherical(const LivoxLidarEthernetPacket* packet,
                     int lineCount,
                     int64_t packetTimestampNs,
                     SlamInputFrame& frame)
{
    const auto* points = reinterpret_cast<const LivoxLidarSpherPoint*>(packet->data);
    const bool hasOffsets = hasOffsetTime(packet->dot_num, packet->time_interval);
    for (uint32_t i = 0; i < packet->dot_num; ++i) {
        const float depth = points[i].depth / 1000.0f;
        const float thetaRad = points[i].theta * 0.01f * float(kLivoxPi) / 180.0f;
        const float phiRad = points[i].phi * 0.01f * float(kLivoxPi) / 180.0f;

        SlamPoint point;
        point.x = depth * std::sin(thetaRad) * std::cos(phiRad);
        point.y = depth * std::sin(thetaRad) * std::sin(phiRad);
        point.z = depth * std::cos(thetaRad);
        point.reflectivity = points[i].reflectivity;
        point.tag = points[i].tag;
        point.line = LivoxCore::lineForPointIndex(int(i), lineCount);
        point.hasLine = true;
        appendPoint(frame, point, pointTimestampNs(packetTimestampNs, packet->time_interval, i), hasOffsets);
    }
}

void appendDoubleEcho(const LivoxLidarEthernetPacket* packet,
                      int lineCount,
                      int64_t packetTimestampNs,
                      SlamInputFrame& frame)
{
    const auto* points = reinterpret_cast<const LivoxLidarDoubleEchoRawPoint*>(packet->data);
    const bool hasOffsets = hasOffsetTime(packet->dot_num, packet->time_interval);
    for (uint32_t i = 0; i < packet->dot_num; ++i) {
        const int64_t timestampNs = pointTimestampNs(packetTimestampNs, packet->time_interval, i);

        SlamPoint point1;
        point1.x = points[i].x1 / 1000.0f;
        point1.y = points[i].y1 / 1000.0f;
        point1.z = points[i].z1 / 1000.0f;
        point1.reflectivity = points[i].reflectivity1;
        point1.tag = points[i].tag1;
        point1.line = LivoxCore::lineForPointIndex(int(i) * 2, lineCount);
        point1.hasLine = true;
        appendPoint(frame, point1, timestampNs, hasOffsets);

        SlamPoint point2;
        point2.x = points[i].x2 / 1000.0f;
        point2.y = points[i].y2 / 1000.0f;
        point2.z = points[i].z2 / 1000.0f;
        point2.reflectivity = points[i].reflectivity2;
        point2.tag = points[i].tag2;
        point2.line = LivoxCore::lineForPointIndex(int(i) * 2 + 1, lineCount);
        point2.hasLine = true;
        appendPoint(frame, point2, timestampNs, hasOffsets);
    }
}

bool appendPacketPoints(const LivoxLidarEthernetPacket* packet,
                        int lineCount,
                        int64_t packetTimestampNs,
                        SlamInputFrame& frame)
{
    switch (packet->data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
        appendCartesianHigh(packet, lineCount, packetTimestampNs, frame);
        return true;
    case kLivoxLidarCartesianCoordinateLowData:
        appendCartesianLow(packet, lineCount, packetTimestampNs, frame);
        return true;
    case kLivoxLidarSphericalCoordinateData:
        appendSpherical(packet, lineCount, packetTimestampNs, frame);
        return true;
    case kLivoxLidarDoubleEchoData:
        appendDoubleEcho(packet, lineCount, packetTimestampNs, frame);
        return true;
    default:
        return false;
    }
}

} // namespace

LiveLidarSlamSource::LiveLidarSlamSource()
{
    statsTimer_.start();
}

void LiveLidarSlamSource::reset()
{
    QMutexLocker locker(&mutex_);
    queue_.clear();
    imuBuffer_.clear();
    pendingFrames_.clear();
    timingByHandle_.clear();
    currentFrame_ = SlamInputFrame();
    hasCurrentFrame_ = false;
    nextSequence_ = 0;
    activeHandle_ = 0;
    hasActiveHandle_ = false;
    stats_ = LiveLidarSlamSourceStats();
    statsTimer_.restart();
}

void LiveLidarSlamSource::setFrameDurationMs(int frameDurationMs)
{
    QMutexLocker locker(&mutex_);
    frameDurationNs_ = int64_t(std::max(1, frameDurationMs)) * kNsPerMs;
}

int LiveLidarSlamSource::frameDurationMs() const
{
    QMutexLocker locker(&mutex_);
    return int(frameDurationNs_ / kNsPerMs);
}

void LiveLidarSlamSource::setQueueCapacity(int capacity)
{
    queue_.setCapacity(capacity);
}

bool LiveLidarSlamSource::appendPointPacket(uint32_t handle,
                                            uint8_t deviceType,
                                            const LivoxLidarEthernetPacket* packet,
                                            const QString& sourceName)
{
    if (packet == nullptr || packet->dot_num == 0) {
        return false;
    }

    const int64_t rawPacketTimestampNs = int64_t(LivoxCore::parseLivoxTimestamp(packet->timestamp));
    QMutexLocker locker(&mutex_);

    if (hasActiveHandle_ && handle != activeHandle_) {
        resetLiveTimelineLocked(handle,
                                packet->time_type,
                                rawPacketTimestampNs,
                                true,
                                SlamStatusCode::Degraded,
                                QStringLiteral("实时 SLAM 输入设备切换，已重置输入源：oldHandle=%1, newHandle=%2, source=%3。")
                                    .arg(activeHandle_)
                                    .arg(handle)
                                    .arg(sourceName));
    }
    if (!hasActiveHandle_) {
        activeHandle_ = handle;
        hasActiveHandle_ = true;
        stats_.activeHandle = handle;
    }

    int64_t packetTimestampNs = 0;
    if (!normalizePacketTimestampLocked(handle,
                                        packet->time_type,
                                        rawPacketTimestampNs,
                                        true,
                                        sourceName,
                                        &packetTimestampNs)) {
        return false;
    }

    if (!hasOffsetTime(packet->dot_num, packet->time_interval)) {
        ++stats_.missingPointOffsetPacketCount;
        stats_.status = SlamStatusCode::TimeSyncError;
        stats_.message = QStringLiteral("实时点云 packet 缺少可用点内 offset 时间，无法进入 FAST_LIO：handle=%1, time_type=%2。")
                             .arg(handle)
                             .arg(int(packet->time_type));
        return false;
    }

    if (!hasCurrentFrame_) {
        currentFrame_ = SlamInputFrame();
        currentFrame_.sequence = nextSequence_++;
        currentFrame_.sourceId = handle;
        currentFrame_.deviceType = deviceType;
        currentFrame_.frameStartNs = packetTimestampNs;
        currentFrame_.frameEndNs = packetTimestampNs;
        currentFrame_.rawFrameStartNs = rawPacketTimestampNs;
        currentFrame_.rawFrameEndNs = rawPacketTimestampNs;
        currentFrame_.timeType = packet->time_type;
        currentFrame_.hasRawFrameTimestamp = true;
        currentFrame_.timeSource = SlamTimeSource::LivoxPacketTimestamp;
        currentFrame_.hasPointOffsetTime = true;
        currentFrame_.sourceName = sourceName;
        hasCurrentFrame_ = true;
    } else if (packetTimestampNs - currentFrame_.frameStartNs >= frameDurationNs_) {
        moveCurrentFrameToPendingLocked();
        currentFrame_ = SlamInputFrame();
        currentFrame_.sequence = nextSequence_++;
        currentFrame_.sourceId = handle;
        currentFrame_.deviceType = deviceType;
        currentFrame_.frameStartNs = packetTimestampNs;
        currentFrame_.frameEndNs = packetTimestampNs;
        currentFrame_.rawFrameStartNs = rawPacketTimestampNs;
        currentFrame_.rawFrameEndNs = rawPacketTimestampNs;
        currentFrame_.timeType = packet->time_type;
        currentFrame_.hasRawFrameTimestamp = true;
        currentFrame_.timeSource = SlamTimeSource::LivoxPacketTimestamp;
        currentFrame_.hasPointOffsetTime = true;
        currentFrame_.sourceName = sourceName;
        hasCurrentFrame_ = true;
    }

    const qsizetype pointCountBefore = currentFrame_.points.size();
    if (!appendPacketPoints(packet, LivoxCore::lineCountForDeviceType(deviceType), packetTimestampNs, currentFrame_)) {
        return false;
    }

    currentFrame_.rawFrameEndNs = std::max(currentFrame_.rawFrameEndNs, packetEndTimestampNs(rawPacketTimestampNs, packet));
    HandleTimingState& timingState = timingByHandle_[handle];
    if (!timingState.hasLatestPointTimestamp || currentFrame_.frameEndNs > timingState.latestPointTimestampNs) {
        timingState.latestPointTimestampNs = currentFrame_.frameEndNs;
        timingState.hasLatestPointTimestamp = true;
        stats_.latestPointTimestampNs = timingState.latestPointTimestampNs;
    }
    ++stats_.pointPacketCount;
    stats_.pointCount += uint64_t(currentFrame_.points.size() - pointCountBefore);
    stats_.lastFrameStartNs = currentFrame_.frameStartNs;
    stats_.lastFrameEndNs = currentFrame_.frameEndNs;
    if (stats_.status == SlamStatusCode::Idle) {
        stats_.status = SlamStatusCode::Starting;
    }

    tryFinalizePendingFramesLocked();
    return true;
}

bool LiveLidarSlamSource::appendImuPacket(uint32_t handle, const LivoxLidarEthernetPacket* packet)
{
    if (packet == nullptr || packet->data_type != kLivoxLidarImuData || packet->dot_num == 0) {
        return false;
    }

    const int64_t rawPacketTimestampNs = int64_t(LivoxCore::parseLivoxTimestamp(packet->timestamp));
    const bool hasTiming = hasOffsetTime(packet->dot_num, packet->time_interval);
    const auto* points = reinterpret_cast<const LivoxLidarImuRawPoint*>(packet->data);

    QMutexLocker locker(&mutex_);
    if (hasActiveHandle_ && handle != activeHandle_) {
        return false;
    }

    int64_t packetTimestampNs = 0;
    if (!normalizePacketTimestampLocked(handle,
                                        packet->time_type,
                                        rawPacketTimestampNs,
                                        false,
                                        QStringLiteral("live"),
                                        &packetTimestampNs)) {
        return false;
    }

    if (!hasTiming) {
        ++stats_.missingImuTimingPacketCount;
        stats_.status = SlamStatusCode::TimeSyncError;
        stats_.message = QStringLiteral("实时 IMU packet 缺少可用包内采样时间：handle=%1, time_type=%2。")
                             .arg(handle)
                             .arg(int(packet->time_type));
        return false;
    }

    int64_t latestSampleTimestampNs = packetTimestampNs;
    for (uint32_t i = 0; i < packet->dot_num; ++i) {
        SlamImuSample sample;
        sample.lidarId = handle;
        sample.timestampNs = pointTimestampNs(packetTimestampNs, packet->time_interval, i);
        sample.rawTimestampNs = pointTimestampNs(rawPacketTimestampNs, packet->time_interval, i);
        sample.timeType = packet->time_type;
        sample.hasRawTimestamp = true;
        sample.gyroRadPerSec[0] = points[i].gyro_x;
        sample.gyroRadPerSec[1] = points[i].gyro_y;
        sample.gyroRadPerSec[2] = points[i].gyro_z;
        sample.accelMps2[0] = points[i].acc_x;
        sample.accelMps2[1] = points[i].acc_y;
        sample.accelMps2[2] = points[i].acc_z;
        insertImuSampleLocked(sample);
        latestSampleTimestampNs = sample.timestampNs;
    }

    HandleTimingState& state = timingByHandle_[handle];
    if (!state.hasLatestImuTimestamp || latestSampleTimestampNs > state.latestImuTimestampNs) {
        state.latestImuTimestampNs = latestSampleTimestampNs;
        state.hasLatestImuTimestamp = true;
    }
    stats_.latestImuTimestampNs = state.latestImuTimestampNs;
    stats_.imuTimeType = packet->time_type;
    pruneImuBufferLocked(state.latestImuTimestampNs);
    ++stats_.imuPacketCount;
    stats_.imuSampleCount += packet->dot_num;
    tryFinalizePendingFramesLocked();
    return true;
}

SlamInputQueue& LiveLidarSlamSource::inputQueue()
{
    return queue_;
}

const SlamInputQueue& LiveLidarSlamSource::inputQueue() const
{
    return queue_;
}

LiveLidarSlamSourceStats LiveLidarSlamSource::stats() const
{
    QMutexLocker locker(&mutex_);
    LiveLidarSlamSourceStats result = stats_;
    const SlamInputQueueStats queueStats = queue_.stats();
    result.queueCapacity = queueStats.capacity;
    result.queueSize = queueStats.size;
    result.pendingFrameCount = pendingFrames_.size();
    result.droppedFrameCount = queueStats.droppedFrameCount + stats_.droppedPendingFrameCount;
    const qint64 elapsedMs = statsTimer_.elapsed();
    if (elapsedMs > 0) {
        result.inputFps = double(result.inputFrameCount) * 1000.0 / double(elapsedMs);
    }
    return result;
}

bool LiveLidarSlamSource::normalizePacketTimestampLocked(uint32_t handle,
                                                         uint8_t timeType,
                                                         int64_t rawTimestampNs,
                                                         bool pointStream,
                                                         const QString& sourceName,
                                                         int64_t* normalizedTimestampNs)
{
    HandleTimingState& state = timingByHandle_[handle];
    bool& hasStreamTimeType = pointStream ? state.hasPointTimeType : state.hasImuTimeType;
    uint8_t& streamTimeTypeValue = pointStream ? state.pointTimeType : state.imuTimeType;

    if (hasStreamTimeType && streamTimeTypeValue != timeType) {
        const QString message = QStringLiteral(
            "雷达时间源 time_type 发生切换，已重置实时 SLAM 输入源，等待时间稳定：stream=%1, last_time_type=%2, current_time_type=%3, handle=%4, source=%5。")
                                    .arg(streamName(pointStream))
                                    .arg(int(streamTimeTypeValue))
                                    .arg(int(timeType))
                                    .arg(handle)
                                    .arg(sourceName);
        resetLiveTimelineLocked(handle, timeType, rawTimestampNs, true, SlamStatusCode::Degraded, message);
    }

    HandleTimingState& typedState = timingByHandle_[handle];
    bool& typedHasStreamTimeType = pointStream ? typedState.hasPointTimeType : typedState.hasImuTimeType;
    uint8_t& typedStreamTimeTypeValue = pointStream ? typedState.pointTimeType : typedState.imuTimeType;
    typedHasStreamTimeType = true;
    typedStreamTimeTypeValue = timeType;

    if (typedState.hasPointTimeType &&
        typedState.hasImuTimeType &&
        typedState.pointTimeType != typedState.imuTimeType) {
        ++stats_.timeResetCount;
        ++stats_.backendResetGeneration;
        queue_.clear();
        imuBuffer_.clear();
        pendingFrames_.clear();
        currentFrame_ = SlamInputFrame();
        hasCurrentFrame_ = false;
        typedState = HandleTimingState();
        stats_.status = SlamStatusCode::TimeSyncError;
        stats_.message = QStringLiteral(
            "LiDAR 与 IMU 时间源不一致，无法形成有效 SLAM 输入帧：point_time_type=%1, imu_time_type=%2, handle=%3, source=%4。")
                             .arg(int(pointStream ? timeType : stats_.pointTimeType))
                             .arg(int(pointStream ? stats_.imuTimeType : timeType))
                             .arg(handle)
                             .arg(sourceName);
        stats_.pendingFrameCount = 0;
        stats_.activeHandle = handle;
        return false;
    }

    HandleTimingState& activeState = timingByHandle_[handle];
    if (!activeState.axisInitialized) {
        activeState.axisInitialized = true;
        activeState.epochRawStartNs = rawTimestampNs;
        activeState.epochNormalizedStartNs = 0;
    }

    bool& hasLastRawTimestamp = pointStream ? activeState.hasLastPointRawTimestamp : activeState.hasLastImuRawTimestamp;
    int64_t& lastRawTimestampNs = pointStream ? activeState.lastPointRawTimestampNs : activeState.lastImuRawTimestampNs;
    if (hasLastRawTimestamp) {
        const int64_t jumpNs = rawTimestampNs - lastRawTimestampNs;
        if (jumpNs < 0) {
            const int64_t backstepNs = -jumpNs;
            stats_.lastTimestampJumpNs = jumpNs;
            stats_.lastRawTimestampNs = lastRawTimestampNs;
            stats_.currentRawTimestampNs = rawTimestampNs;
            if (backstepNs <= kSmallTimestampBackstepNs) {
                stats_.status = SlamStatusCode::Degraded;
                stats_.message = QStringLiteral(
                    "实时输入轻微乱序，已处理异常 %1 packet：last=%2ms, current=%3ms, jump=%4ms, time_type=%5, handle=%6, source=%7。")
                                     .arg(streamName(pointStream))
                                     .arg(nsToMsText(lastRawTimestampNs))
                                     .arg(nsToMsText(rawTimestampNs))
                                     .arg(nsToMsText(jumpNs))
                                     .arg(int(timeType))
                                     .arg(handle)
                                     .arg(sourceName);
                if (pointStream) {
                    ++stats_.outOfOrderPointPacketCount;
                    stats_.message = QStringLiteral(
                        "实时输入轻微乱序，已丢弃异常 LiDAR packet：last=%1ms, current=%2ms, jump=%3ms, time_type=%4, handle=%5, source=%6。")
                                         .arg(nsToMsText(lastRawTimestampNs))
                                         .arg(nsToMsText(rawTimestampNs))
                                         .arg(nsToMsText(jumpNs))
                                         .arg(int(timeType))
                                         .arg(handle)
                                         .arg(sourceName);
                    return false;
                }
                ++stats_.outOfOrderImuPacketCount;
            } else {
                const QString message = QStringLiteral(
                    "雷达时间源发生跳变，已重置实时 SLAM 输入源，等待时间稳定：stream=%1, last=%2ms, current=%3ms, jump=%4ms, time_type=%5, handle=%6, source=%7。")
                                            .arg(streamName(pointStream))
                                            .arg(nsToMsText(lastRawTimestampNs))
                                            .arg(nsToMsText(rawTimestampNs))
                                            .arg(nsToMsText(jumpNs))
                                            .arg(int(timeType))
                                            .arg(handle)
                                            .arg(sourceName);
                resetLiveTimelineLocked(handle, timeType, rawTimestampNs, true, SlamStatusCode::Degraded, message);
                HandleTimingState& resetState = timingByHandle_[handle];
                resetState.hasPointTimeType = pointStream;
                resetState.hasImuTimeType = !pointStream;
                resetState.pointTimeType = timeType;
                resetState.imuTimeType = timeType;
            }
        } else if (jumpNs > kTimestampJumpResetNs) {
            stats_.lastTimestampJumpNs = jumpNs;
            stats_.lastRawTimestampNs = lastRawTimestampNs;
            stats_.currentRawTimestampNs = rawTimestampNs;
            const QString message = QStringLiteral(
                "雷达时间源发生大幅前跳，已重置实时 SLAM 输入源，等待时间稳定：stream=%1, last=%2ms, current=%3ms, jump=%4ms, time_type=%5, handle=%6, source=%7。")
                                        .arg(streamName(pointStream))
                                        .arg(nsToMsText(lastRawTimestampNs))
                                        .arg(nsToMsText(rawTimestampNs))
                                        .arg(nsToMsText(jumpNs))
                                        .arg(int(timeType))
                                        .arg(handle)
                                        .arg(sourceName);
            resetLiveTimelineLocked(handle, timeType, rawTimestampNs, true, SlamStatusCode::Degraded, message);
            HandleTimingState& resetState = timingByHandle_[handle];
            resetState.hasPointTimeType = pointStream;
            resetState.hasImuTimeType = !pointStream;
            resetState.pointTimeType = timeType;
            resetState.imuTimeType = timeType;
        }
    }

    HandleTimingState& finalState = timingByHandle_[handle];
    const int64_t normalized = rawTimestampNs - finalState.epochRawStartNs + finalState.epochNormalizedStartNs;
    if (normalizedTimestampNs != nullptr) {
        *normalizedTimestampNs = normalized;
    }

    bool& finalHasLastRawTimestamp = pointStream ? finalState.hasLastPointRawTimestamp : finalState.hasLastImuRawTimestamp;
    int64_t& finalLastRawTimestampNs = pointStream ? finalState.lastPointRawTimestampNs : finalState.lastImuRawTimestampNs;
    if (!finalHasLastRawTimestamp || rawTimestampNs > finalLastRawTimestampNs) {
        finalLastRawTimestampNs = rawTimestampNs;
        finalHasLastRawTimestamp = true;
    }

    if (pointStream) {
        if (!finalState.hasLatestPointTimestamp || normalized > finalState.latestPointTimestampNs) {
            finalState.latestPointTimestampNs = normalized;
            finalState.hasLatestPointTimestamp = true;
        }
        stats_.latestPointTimestampNs = finalState.latestPointTimestampNs;
        stats_.pointTimeType = timeType;
    } else {
        if (!finalState.hasLatestImuTimestamp || normalized > finalState.latestImuTimestampNs) {
            finalState.latestImuTimestampNs = normalized;
            finalState.hasLatestImuTimestamp = true;
        }
        stats_.latestImuTimestampNs = finalState.latestImuTimestampNs;
        stats_.imuTimeType = timeType;
    }
    stats_.activeHandle = handle;
    return true;
}

void LiveLidarSlamSource::resetLiveTimelineLocked(uint32_t handle,
                                                  uint8_t timeType,
                                                  int64_t rawTimestampNs,
                                                  bool requestBackendReset,
                                                  SlamStatusCode status,
                                                  const QString& message)
{
    queue_.clear();
    imuBuffer_.clear();
    pendingFrames_.clear();
    currentFrame_ = SlamInputFrame();
    hasCurrentFrame_ = false;
    timingByHandle_.clear();
    HandleTimingState state;
    state.axisInitialized = true;
    state.epochRawStartNs = rawTimestampNs;
    state.epochNormalizedStartNs = 0;
    timingByHandle_.insert(handle, state);
    activeHandle_ = handle;
    hasActiveHandle_ = true;
    ++stats_.timeResetCount;
    if (requestBackendReset) {
        ++stats_.backendResetGeneration;
    }
    stats_.status = status;
    stats_.message = message;
    stats_.activeHandle = handle;
    stats_.pendingFrameCount = 0;
    stats_.lastRawTimestampNs = rawTimestampNs;
    stats_.currentRawTimestampNs = rawTimestampNs;
    stats_.pointTimeType = timeType;
}

void LiveLidarSlamSource::moveCurrentFrameToPendingLocked()
{
    if (!hasCurrentFrame_ || currentFrame_.points.isEmpty()) {
        hasCurrentFrame_ = false;
        currentFrame_ = SlamInputFrame();
        return;
    }

    pendingFrames_.enqueue(std::move(currentFrame_));
    currentFrame_ = SlamInputFrame();
    hasCurrentFrame_ = false;
    stats_.pendingFrameCount = pendingFrames_.size();
}

void LiveLidarSlamSource::tryFinalizePendingFramesLocked()
{
    while (!pendingFrames_.isEmpty()) {
        const SlamInputFrame& pendingFrame = pendingFrames_.head();
        const auto stateIt = timingByHandle_.constFind(pendingFrame.sourceId);
        const int64_t latestObservedNs = latestObservedTimestampLocked(pendingFrame.sourceId);
        if (stateIt == timingByHandle_.constEnd() || !stateIt->hasLatestImuTimestamp) {
            if (latestObservedNs - pendingFrame.frameEndNs >= kMaxPendingFrameWaitNs) {
                const int64_t frameStartNs = pendingFrame.frameStartNs;
                const int64_t frameEndNs = pendingFrame.frameEndNs;
                const uint32_t sourceId = pendingFrame.sourceId;
                pendingFrames_.dequeue();
                ++stats_.droppedPendingFrameCount;
                ++stats_.warmupDroppedFrameCount;
                ++stats_.incompleteImuCoverageFrameCount;
                stats_.pendingFrameCount = pendingFrames_.size();
                stats_.status = SlamStatusCode::InitializingImu;
                stats_.message = QStringLiteral(
                    "实时 SLAM 暖机中：丢弃缺少 IMU 的点云帧，frameStart=%1ms, frameEnd=%2ms, handle=%3。")
                                      .arg(nsToMsText(frameStartNs))
                                      .arg(nsToMsText(frameEndNs))
                                      .arg(sourceId);
                continue;
            }
            updatePendingWaitStatusLocked(pendingFrame, 0);
            break;
        }

        const int64_t latestImuNs = stateIt->latestImuTimestampNs;
        if (latestImuNs < pendingFrame.frameEndNs + kImuCoverageMarginNs) {
            if (latestObservedNs - pendingFrame.frameEndNs >= kMaxPendingFrameWaitNs) {
                const int64_t frameStartNs = pendingFrame.frameStartNs;
                const int64_t frameEndNs = pendingFrame.frameEndNs;
                const uint8_t timeType = pendingFrame.timeType;
                const uint32_t sourceId = pendingFrame.sourceId;
                pendingFrames_.dequeue();
                ++stats_.droppedPendingFrameCount;
                ++stats_.incompleteImuCoverageFrameCount;
                stats_.pendingFrameCount = pendingFrames_.size();
                stats_.status = SlamStatusCode::InitializingImu;
                stats_.message = QStringLiteral(
                    "等待 IMU 覆盖超时，已丢弃点云帧：frameStart=%1ms, frameEnd=%2ms, latestImu=%3ms, tailGap=%4ms, time_type=%5, handle=%6。")
                                      .arg(nsToMsText(frameStartNs))
                                      .arg(nsToMsText(frameEndNs))
                                      .arg(nsToMsText(latestImuNs))
                                      .arg(nsToMsText(frameEndNs - latestImuNs))
                                      .arg(int(timeType))
                                      .arg(sourceId);
                continue;
            }
            updatePendingWaitStatusLocked(pendingFrame, latestImuNs);
            break;
        }

        SlamInputFrame frame = std::move(pendingFrames_.head());
        pendingFrames_.dequeue();
        const bool allowStartPadding = stats_.inputFrameCount > 0;
        if (!attachImuSamplesLocked(frame, allowStartPadding)) {
            ++stats_.droppedPendingFrameCount;
            ++stats_.incompleteImuCoverageFrameCount;
            if (stats_.inputFrameCount == 0) {
                ++stats_.warmupDroppedFrameCount;
            }
            stats_.pendingFrameCount = pendingFrames_.size();
            stats_.status = SlamStatusCode::InitializingImu;
            stats_.message = QStringLiteral(
                "实时 SLAM 暖机中：丢弃前置 IMU 覆盖不足的点云帧，frameStart=%1ms, frameEnd=%2ms, latestImu=%3ms, time_type=%4, handle=%5。")
                                  .arg(nsToMsText(frame.frameStartNs))
                                  .arg(nsToMsText(frame.frameEndNs))
                                  .arg(nsToMsText(latestImuNs))
                                  .arg(int(frame.timeType))
                                  .arg(frame.sourceId);
            continue;
        }

        stats_.lastFrameStartNs = frame.frameStartNs;
        stats_.lastFrameEndNs = frame.frameEndNs;
        queue_.push(std::move(frame));
        ++stats_.inputFrameCount;
        stats_.pendingFrameCount = pendingFrames_.size();
        stats_.imuCoverageGapNs = 0;
        stats_.waitingFrameStartNs = 0;
        stats_.waitingFrameEndNs = 0;
        stats_.status = SlamStatusCode::Running;
        stats_.message.clear();
    }
}

bool LiveLidarSlamSource::attachImuSamplesLocked(SlamInputFrame& frame, bool allowStartPadding)
{
    frame.imuSamples.clear();
    const auto firstInside = std::lower_bound(imuBuffer_.begin(),
                                              imuBuffer_.end(),
                                              frame.frameStartNs,
                                              [](const SlamImuSample& sample, int64_t timestampNs) {
                                                  return sample.timestampNs < timestampNs;
                                              });

    for (auto it = firstInside; it != imuBuffer_.begin();) {
        --it;
        if (it->lidarId == frame.sourceId) {
            frame.imuSamples.push_back(*it);
            break;
        }
    }

    for (auto it = firstInside; it != imuBuffer_.end() && it->timestampNs <= frame.frameEndNs; ++it) {
        if (it->lidarId == frame.sourceId) {
            frame.imuSamples.push_back(*it);
        }
    }

    const auto firstAfterEnd = std::upper_bound(imuBuffer_.begin(),
                                                imuBuffer_.end(),
                                                frame.frameEndNs,
                                                [](int64_t timestampNs, const SlamImuSample& sample) {
                                                    return timestampNs < sample.timestampNs;
                                                });
    for (auto it = firstAfterEnd; it != imuBuffer_.end(); ++it) {
        if (it->lidarId == frame.sourceId) {
            frame.imuSamples.push_back(*it);
            break;
        }
    }

    if (frame.imuSamples.isEmpty()) {
        frame.hasCompleteImuCoverage = false;
        return false;
    }

    bool padded = false;
    if (frame.imuSamples.first().timestampNs > frame.frameStartNs) {
        const int64_t startGapNs = frame.imuSamples.first().timestampNs - frame.frameStartNs;
        if (!allowStartPadding || startGapNs > kMaxImuStartPadNs) {
            frame.hasCompleteImuCoverage = false;
            stats_.imuCoverageGapNs = startGapNs;
            return false;
        }
        SlamImuSample paddedSample = frame.imuSamples.first();
        paddedSample.timestampNs = frame.frameStartNs;
        frame.imuSamples.prepend(paddedSample);
        ++stats_.paddedImuSampleCount;
        padded = true;
    }

    if (frame.imuSamples.last().timestampNs < frame.frameEndNs) {
        const int64_t tailGapNs = frame.frameEndNs - frame.imuSamples.last().timestampNs;
        if (tailGapNs > kMaxImuTailPadNs) {
            frame.hasCompleteImuCoverage = false;
            stats_.imuCoverageGapNs = tailGapNs;
            return false;
        }
        SlamImuSample paddedSample = frame.imuSamples.last();
        paddedSample.timestampNs = frame.frameEndNs;
        frame.imuSamples.push_back(paddedSample);
        ++stats_.paddedImuSampleCount;
        padded = true;
    }

    frame.hasCompleteImuCoverage =
        frame.imuSamples.first().timestampNs <= frame.frameStartNs &&
        frame.imuSamples.last().timestampNs >= frame.frameEndNs;
    if (frame.hasCompleteImuCoverage && padded) {
        ++stats_.paddedImuFrameCount;
    }
    return frame.hasCompleteImuCoverage;
}

void LiveLidarSlamSource::insertImuSampleLocked(const SlamImuSample& sample)
{
    const auto insertIt = std::upper_bound(imuBuffer_.begin(),
                                           imuBuffer_.end(),
                                           sample.timestampNs,
                                           [](int64_t timestampNs, const SlamImuSample& existing) {
                                               return timestampNs < existing.timestampNs;
                                           });
    imuBuffer_.insert(insertIt, sample);
}

void LiveLidarSlamSource::pruneImuBufferLocked(int64_t latestTimestampNs)
{
    const int64_t keepAfterNs = latestTimestampNs - kImuRetentionNs;
    int removeCount = 0;
    while (removeCount < imuBuffer_.size() && imuBuffer_.at(removeCount).timestampNs < keepAfterNs) {
        ++removeCount;
    }
    if (removeCount > 0) {
        imuBuffer_.remove(0, removeCount);
    }
}

void LiveLidarSlamSource::updatePendingWaitStatusLocked(const SlamInputFrame& frame, int64_t latestImuTimestampNs)
{
    stats_.status = stats_.imuPacketCount == 0 ? SlamStatusCode::Starting : SlamStatusCode::InitializingImu;
    stats_.waitingFrameStartNs = frame.frameStartNs;
    stats_.waitingFrameEndNs = frame.frameEndNs;
    stats_.imuCoverageGapNs = latestImuTimestampNs > 0 ? frame.frameEndNs - latestImuTimestampNs : frame.frameEndNs;
    stats_.message = QStringLiteral(
        "正在等待 IMU 覆盖点云帧：frameStart=%1ms, frameEnd=%2ms, latestImu=%3ms, tailGap=%4ms, time_type=%5, handle=%6。")
                         .arg(nsToMsText(frame.frameStartNs))
                         .arg(nsToMsText(frame.frameEndNs))
                         .arg(latestImuTimestampNs > 0 ? nsToMsText(latestImuTimestampNs) : QStringLiteral("n/a"))
                         .arg(latestImuTimestampNs > 0 ? nsToMsText(frame.frameEndNs - latestImuTimestampNs) : QStringLiteral("n/a"))
                         .arg(int(frame.timeType))
                         .arg(frame.sourceId);
}

int64_t LiveLidarSlamSource::latestObservedTimestampLocked(uint32_t handle) const
{
    const auto stateIt = timingByHandle_.constFind(handle);
    if (stateIt == timingByHandle_.constEnd()) {
        return 0;
    }

    int64_t latest = 0;
    if (stateIt->hasLatestPointTimestamp) {
        latest = stateIt->latestPointTimestampNs;
    }
    if (stateIt->hasLatestImuTimestamp && stateIt->latestImuTimestampNs > latest) {
        latest = stateIt->latestImuTimestampNs;
    }
    return latest;
}

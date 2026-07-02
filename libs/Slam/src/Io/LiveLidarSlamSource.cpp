#include "Slam/Io/LiveLidarSlamSource.h"

#include "LivoxCore/LidarModelUtils.h"
#include "LivoxCore/LidarPacketUtils.h"

#include <QMutexLocker>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>

namespace {

constexpr int64_t kLivoxTimeIntervalUnitNs = 100;
constexpr int64_t kNsPerMs = 1000000;
constexpr int64_t kSmallTimestampBackstepNs = 5000000;
constexpr int64_t kTimestampJumpResetNs = 1000000000;
constexpr int64_t kTimeTypeMismatchRawDeltaNs = 200000000;
constexpr int kTimestampJumpHardResetCount = 6;
constexpr int kTimeTypeMismatchErrorCount = 8;
constexpr double kLivoxPi = 3.14159265358979323846;

bool hasOffsetTime(uint16_t dotNum, uint16_t timeIntervalRaw)
{
    return dotNum <= 1 || timeIntervalRaw != 0;
}
int64_t sampleIntervalNs(uint16_t timeIntervalRaw, uint16_t dotNum)
{
    if (dotNum == 0) {
        return 0;
    }
    return int64_t(timeIntervalRaw) * kLivoxTimeIntervalUnitNs / int64_t(dotNum);
}

int64_t pointTimestampNs(int64_t packetTimestampNs, uint16_t timeIntervalRaw, uint16_t dotNum, uint32_t index)
{
    return packetTimestampNs + int64_t(index) * sampleIntervalNs(timeIntervalRaw, dotNum);
}

int64_t packetEndTimestampNs(int64_t packetTimestampNs, const LivoxLidarEthernetPacket* packet)
{
    return pointTimestampNs(packetTimestampNs,
                            packet->time_interval,
                            packet->dot_num,
                            packet->dot_num > 0 ? packet->dot_num - 1 : 0);
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
        appendPoint(frame,
                    point,
                    pointTimestampNs(packetTimestampNs, packet->time_interval, packet->dot_num, i),
                    hasOffsets);
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
        appendPoint(frame,
                    point,
                    pointTimestampNs(packetTimestampNs, packet->time_interval, packet->dot_num, i),
                    hasOffsets);
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
        appendPoint(frame,
                    point,
                    pointTimestampNs(packetTimestampNs, packet->time_interval, packet->dot_num, i),
                    hasOffsets);
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
        const int64_t timestampNs = pointTimestampNs(packetTimestampNs, packet->time_interval, packet->dot_num, i);

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
    synchronizer_.reset();
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
                                QStringLiteral("实时 SLAM 当前设备切换，已重置后端和输入源：oldHandle=%1, newHandle=%2, time_type=%3, raw=%4ms, source=%5。")
                                    .arg(activeHandle_)
                                    .arg(handle)
                                    .arg(int(packet->time_type))
                                    .arg(nsToMsText(rawPacketTimestampNs))
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
        sample.timestampNs = pointTimestampNs(packetTimestampNs, packet->time_interval, packet->dot_num, i);
        sample.rawTimestampNs = pointTimestampNs(rawPacketTimestampNs, packet->time_interval, packet->dot_num, i);
        sample.timeType = packet->time_type;
        sample.hasRawTimestamp = true;
        sample.gyroRadPerSec[0] = points[i].gyro_x;
        sample.gyroRadPerSec[1] = points[i].gyro_y;
        sample.gyroRadPerSec[2] = points[i].gyro_z;
        sample.accelRaw[0] = points[i].acc_x;
        sample.accelRaw[1] = points[i].acc_y;
        sample.accelRaw[2] = points[i].acc_z;
        synchronizer_.pushImuSample(sample);
        latestSampleTimestampNs = sample.timestampNs;
    }

    HandleTimingState& state = timingByHandle_[handle];
    if (!state.hasLatestImuTimestamp || latestSampleTimestampNs > state.latestImuTimestampNs) {
        state.latestImuTimestampNs = latestSampleTimestampNs;
        state.hasLatestImuTimestamp = true;
    }
    stats_.latestImuTimestampNs = state.latestImuTimestampNs;
    stats_.imuTimeType = packet->time_type;
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
    result.pendingFrameCount = synchronizer_.pendingLidarFrameCount();
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
        ++stats_.timeTypeMismatchCount;
        ++state.consecutiveTimeTypeMismatchCount;
        stats_.status = SlamStatusCode::Degraded;
        stats_.message = QStringLiteral(
            "实时 %1 time_type 短暂变化，继续使用归一化时间轴：last_time_type=%2, current_time_type=%3, raw=%4ms, handle=%5, source=%6。")
                             .arg(streamName(pointStream))
                             .arg(int(streamTimeTypeValue))
                             .arg(int(timeType))
                             .arg(nsToMsText(rawTimestampNs))
                             .arg(handle)
                             .arg(sourceName);
    }

    hasStreamTimeType = true;
    streamTimeTypeValue = timeType;

    if (state.hasPointTimeType && state.hasImuTimeType && state.pointTimeType != state.imuTimeType) {
        ++stats_.timeTypeMismatchCount;
        const bool hasBothRaw = state.hasLastPointRawTimestamp && state.hasLastImuRawTimestamp;
        const int64_t rawDeltaNs = hasBothRaw ? std::llabs(state.lastPointRawTimestampNs - state.lastImuRawTimestampNs) : 0;
        if (hasBothRaw &&
            rawDeltaNs > kTimeTypeMismatchRawDeltaNs &&
            state.consecutiveTimeTypeMismatchCount >= kTimeTypeMismatchErrorCount) {
            stats_.status = SlamStatusCode::TimeSyncError;
            stats_.message = QStringLiteral(
                "LiDAR 与 IMU time_type 持续不一致且 raw timestamp 差异过大，停止投递实时 SLAM 输入：pointTimeType=%1, imuTimeType=%2, rawDelta=%3ms, handle=%4, source=%5。")
                                 .arg(int(state.pointTimeType))
                                 .arg(int(state.imuTimeType))
                                 .arg(nsToMsText(rawDeltaNs))
                                 .arg(handle)
                                 .arg(sourceName);
            return false;
        }
        stats_.status = SlamStatusCode::Degraded;
        stats_.message = QStringLiteral(
            "LiDAR/IMU time_type 短暂不一致，归一化时间连续时继续运行：pointTimeType=%1, imuTimeType=%2, rawDelta=%3ms, handle=%4, source=%5。")
                             .arg(int(state.pointTimeType))
                             .arg(int(state.imuTimeType))
                             .arg(hasBothRaw ? nsToMsText(rawDeltaNs) : QStringLiteral("n/a"))
                             .arg(handle)
                             .arg(sourceName);
    }

    if (!state.axisInitialized) {
        state.axisInitialized = true;
        state.epochRawStartNs = rawTimestampNs;
        state.epochNormalizedStartNs = 0;
    }

    bool& hasLastRawTimestamp = pointStream ? state.hasLastPointRawTimestamp : state.hasLastImuRawTimestamp;
    int64_t& lastRawTimestampNs = pointStream ? state.lastPointRawTimestampNs : state.lastImuRawTimestampNs;
    if (hasLastRawTimestamp) {
        const int64_t jumpNs = rawTimestampNs - lastRawTimestampNs;
        if (jumpNs < 0) {
            const int64_t backstepNs = -jumpNs;
            stats_.lastTimestampJumpNs = jumpNs;
            stats_.lastRawTimestampNs = lastRawTimestampNs;
            stats_.currentRawTimestampNs = rawTimestampNs;
            if (backstepNs <= kSmallTimestampBackstepNs) {
                state.consecutiveTimestampJumpCount = 0;
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
                ++stats_.timestampJumpCount;
                ++state.consecutiveTimestampJumpCount;
                const QString message = QStringLiteral(
                    "实时 %1 packet 时间戳大幅回跳，已丢弃异常 packet 并清理未完成输入片段：last=%2ms, current=%3ms, jump=%4ms, time_type=%5, handle=%6, source=%7, consecutive=%8。")
                                            .arg(streamName(pointStream))
                                            .arg(nsToMsText(lastRawTimestampNs))
                                            .arg(nsToMsText(rawTimestampNs))
                                            .arg(nsToMsText(jumpNs))
                                            .arg(int(timeType))
                                            .arg(handle)
                                            .arg(sourceName)
                                            .arg(state.consecutiveTimestampJumpCount);
                if (state.consecutiveTimestampJumpCount >= kTimestampJumpHardResetCount) {
                    resetLiveTimelineLocked(handle, timeType, rawTimestampNs, true, SlamStatusCode::Degraded, message);
                } else {
                    noteInputTimelineResetLocked(handle, timeType, lastRawTimestampNs, rawTimestampNs, jumpNs, SlamStatusCode::Degraded, message);
                }
                return false;
            }
        } else if (jumpNs > kTimestampJumpResetNs) {
            ++stats_.timestampJumpCount;
            ++state.consecutiveTimestampJumpCount;
            stats_.lastTimestampJumpNs = jumpNs;
            stats_.lastRawTimestampNs = lastRawTimestampNs;
            stats_.currentRawTimestampNs = rawTimestampNs;
            const QString message = QStringLiteral(
                "实时 %1 packet 时间戳大幅前跳，已丢弃异常 packet 并清理未完成输入片段：last=%2ms, current=%3ms, jump=%4ms, time_type=%5, handle=%6, source=%7, consecutive=%8。")
                                        .arg(streamName(pointStream))
                                        .arg(nsToMsText(lastRawTimestampNs))
                                        .arg(nsToMsText(rawTimestampNs))
                                        .arg(nsToMsText(jumpNs))
                                        .arg(int(timeType))
                                        .arg(handle)
                                        .arg(sourceName)
                                        .arg(state.consecutiveTimestampJumpCount);
            if (state.consecutiveTimestampJumpCount >= kTimestampJumpHardResetCount) {
                resetLiveTimelineLocked(handle, timeType, rawTimestampNs, true, SlamStatusCode::Degraded, message);
            } else {
                noteInputTimelineResetLocked(handle, timeType, lastRawTimestampNs, rawTimestampNs, jumpNs, SlamStatusCode::Degraded, message);
            }
            return false;
        } else {
            state.consecutiveTimestampJumpCount = 0;
        }
    }

    const int64_t normalized = rawTimestampNs - state.epochRawStartNs + state.epochNormalizedStartNs;
    if (normalizedTimestampNs != nullptr) {
        *normalizedTimestampNs = normalized;
    }

    bool& finalHasLastRawTimestamp = pointStream ? state.hasLastPointRawTimestamp : state.hasLastImuRawTimestamp;
    int64_t& finalLastRawTimestampNs = pointStream ? state.lastPointRawTimestampNs : state.lastImuRawTimestampNs;
    if (!finalHasLastRawTimestamp || rawTimestampNs > finalLastRawTimestampNs) {
        finalLastRawTimestampNs = rawTimestampNs;
        finalHasLastRawTimestamp = true;
    }

    if (pointStream) {
        if (!state.hasLatestPointTimestamp || normalized > state.latestPointTimestampNs) {
            state.latestPointTimestampNs = normalized;
            state.hasLatestPointTimestamp = true;
        }
        stats_.latestPointTimestampNs = state.latestPointTimestampNs;
        stats_.pointTimeType = timeType;
    } else {
        if (!state.hasLatestImuTimestamp || normalized > state.latestImuTimestampNs) {
            state.latestImuTimestampNs = normalized;
            state.hasLatestImuTimestamp = true;
        }
        stats_.latestImuTimestampNs = state.latestImuTimestampNs;
        stats_.imuTimeType = timeType;
    }
    stats_.activeHandle = handle;
    return true;
}

void LiveLidarSlamSource::resetLiveTimelineLocked(uint32_t handle,
                                                  uint8_t timeType,
                                                  int64_t rawTimestampNs,
                                                  bool requestBackendHardReset,
                                                  SlamStatusCode status,
                                                  const QString& message)
{
    const int64_t resetLastRawTimestampNs = stats_.lastRawTimestampNs;
    const int64_t resetJumpNs = stats_.lastTimestampJumpNs;
    queue_.clear();
    synchronizer_.reset();
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
    ++stats_.inputTimelineResetGeneration;
    if (requestBackendHardReset) {
        ++stats_.backendHardResetGeneration;
    }
    stats_.status = status;
    stats_.message = message;
    stats_.activeHandle = handle;
    stats_.pendingFrameCount = 0;
    stats_.lastRawTimestampNs = rawTimestampNs;
    stats_.currentRawTimestampNs = rawTimestampNs;
    stats_.pointTimeType = timeType;
    stats_.lastResetReason = message;
    stats_.lastResetStatus = status;
    stats_.lastResetGeneration = requestBackendHardReset
        ? stats_.backendHardResetGeneration
        : stats_.inputTimelineResetGeneration;
    stats_.lastResetJumpNs = resetJumpNs;
    stats_.lastResetRawTimestampNs = resetLastRawTimestampNs != 0 ? resetLastRawTimestampNs : rawTimestampNs;
    stats_.currentResetRawTimestampNs = rawTimestampNs;
    stats_.lastResetHandle = handle;
    stats_.lastResetTimeType = timeType;
}

void LiveLidarSlamSource::noteInputTimelineResetLocked(uint32_t handle,
                                                       uint8_t timeType,
                                                       int64_t lastRawTimestampNs,
                                                       int64_t currentRawTimestampNs,
                                                       int64_t jumpNs,
                                                       SlamStatusCode status,
                                                       const QString& message)
{
    currentFrame_ = SlamInputFrame();
    hasCurrentFrame_ = false;
    synchronizer_.reset();
    ++stats_.timeResetCount;
    ++stats_.inputTimelineResetGeneration;
    stats_.status = status;
    stats_.message = message;
    stats_.pendingFrameCount = 0;
    stats_.lastTimestampJumpNs = jumpNs;
    stats_.lastRawTimestampNs = lastRawTimestampNs;
    stats_.currentRawTimestampNs = currentRawTimestampNs;
    stats_.activeHandle = handle;
    stats_.lastResetReason = message;
    stats_.lastResetStatus = status;
    stats_.lastResetGeneration = stats_.inputTimelineResetGeneration;
    stats_.lastResetJumpNs = jumpNs;
    stats_.lastResetRawTimestampNs = lastRawTimestampNs;
    stats_.currentResetRawTimestampNs = currentRawTimestampNs;
    stats_.lastResetHandle = handle;
    stats_.lastResetTimeType = timeType;
}

void LiveLidarSlamSource::moveCurrentFrameToPendingLocked()
{
    if (!hasCurrentFrame_ || currentFrame_.points.isEmpty()) {
        hasCurrentFrame_ = false;
        currentFrame_ = SlamInputFrame();
        return;
    }

    synchronizer_.pushLidarFrame(std::move(currentFrame_));
    currentFrame_ = SlamInputFrame();
    hasCurrentFrame_ = false;
    stats_.pendingFrameCount = synchronizer_.pendingLidarFrameCount();
}

void LiveLidarSlamSource::tryFinalizePendingFramesLocked()
{
    SlamInputFrame frame;
    while (synchronizer_.trySync(&frame)) {
        if (!frame.hasCompleteImuCoverage) {
            ++stats_.droppedPendingFrameCount;
            ++stats_.incompleteImuCoverageFrameCount;
            if (stats_.inputFrameCount == 0) {
                ++stats_.warmupDroppedFrameCount;
            }
            stats_.pendingFrameCount = synchronizer_.pendingLidarFrameCount();
            stats_.status = SlamStatusCode::InitializingImu;
            stats_.message = QStringLiteral("FAST_LIO sync output has no IMU samples: frameStart=%1ms, frameEnd=%2ms, latestImu=%3ms, time_type=%4, handle=%5.")
                                 .arg(nsToMsText(frame.frameStartNs))
                                 .arg(nsToMsText(frame.frameEndNs))
                                 .arg(nsToMsText(synchronizer_.latestImuTimestampNs()))
                                 .arg(int(frame.timeType))
                                 .arg(frame.sourceId);
            continue;
        }

        stats_.lastFrameStartNs = frame.frameStartNs;
        stats_.lastFrameEndNs = frame.frameEndNs;
        queue_.push(std::move(frame));
        ++stats_.inputFrameCount;
        stats_.pendingFrameCount = synchronizer_.pendingLidarFrameCount();
        stats_.imuCoverageGapNs = 0;
        stats_.waitingFrameStartNs = 0;
        stats_.waitingFrameEndNs = 0;
        stats_.status = SlamStatusCode::Running;
        stats_.message.clear();
    }

    if (synchronizer_.pendingLidarFrameCount() > 0) {
        SlamInputFrame waitingFrame;
        waitingFrame.frameStartNs = synchronizer_.waitingFrameStartNs();
        waitingFrame.frameEndNs = synchronizer_.waitingFrameEndNs();
        waitingFrame.timeType = stats_.pointTimeType;
        waitingFrame.sourceId = activeHandle_;
        updatePendingWaitStatusLocked(waitingFrame, synchronizer_.latestImuTimestampNs());
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

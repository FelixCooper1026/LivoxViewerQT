#include "Slam/Io/LiveLidarSlamSource.h"

#include "LivoxCore/LidarModelUtils.h"
#include "LivoxCore/LidarPacketUtils.h"

#include <QMutexLocker>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

constexpr int64_t kLivoxTimeIntervalUnitNs = 100;
constexpr int64_t kNsPerMs = 1000000;
constexpr int64_t kImuRetentionNs = 5000000000;
constexpr double kLivoxPi = 3.14159265358979323846;

bool hasOffsetTime(uint16_t dotNum, uint16_t timeIntervalRaw)
{
    return dotNum <= 1 || timeIntervalRaw != 0;
}

int64_t pointTimestampNs(int64_t packetTimestampNs, uint16_t timeIntervalRaw, uint32_t index)
{
    return packetTimestampNs + int64_t(index) * int64_t(timeIntervalRaw) * kLivoxTimeIntervalUnitNs;
}

void appendPoint(SlamInputFrame& frame, const SlamPoint& point, int64_t absoluteTimestampNs, bool hasPointOffsetTime)
{
    SlamPoint slamPoint = point;
    slamPoint.offsetNs = absoluteTimestampNs - frame.frameStartNs;
    slamPoint.hasOffsetTime = hasPointOffsetTime;
    frame.points.push_back(slamPoint);
    if (absoluteTimestampNs > frame.frameEndNs) {
        frame.frameEndNs = absoluteTimestampNs;
    }
    if (!hasPointOffsetTime) {
        frame.hasPointOffsetTime = false;
    }
}

void appendCartesianHigh(const LivoxLidarEthernetPacket* packet, int lineCount, SlamInputFrame& frame)
{
    const auto* points = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(packet->data);
    const int64_t packetTimestampNs = int64_t(LivoxCore::parseLivoxTimestamp(packet->timestamp));
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

void appendCartesianLow(const LivoxLidarEthernetPacket* packet, int lineCount, SlamInputFrame& frame)
{
    const auto* points = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(packet->data);
    const int64_t packetTimestampNs = int64_t(LivoxCore::parseLivoxTimestamp(packet->timestamp));
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

void appendSpherical(const LivoxLidarEthernetPacket* packet, int lineCount, SlamInputFrame& frame)
{
    const auto* points = reinterpret_cast<const LivoxLidarSpherPoint*>(packet->data);
    const int64_t packetTimestampNs = int64_t(LivoxCore::parseLivoxTimestamp(packet->timestamp));
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

void appendDoubleEcho(const LivoxLidarEthernetPacket* packet, int lineCount, SlamInputFrame& frame)
{
    const auto* points = reinterpret_cast<const LivoxLidarDoubleEchoRawPoint*>(packet->data);
    const int64_t packetTimestampNs = int64_t(LivoxCore::parseLivoxTimestamp(packet->timestamp));
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

bool appendPacketPoints(const LivoxLidarEthernetPacket* packet, int lineCount, SlamInputFrame& frame)
{
    switch (packet->data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
        appendCartesianHigh(packet, lineCount, frame);
        return true;
    case kLivoxLidarCartesianCoordinateLowData:
        appendCartesianLow(packet, lineCount, frame);
        return true;
    case kLivoxLidarSphericalCoordinateData:
        appendSpherical(packet, lineCount, frame);
        return true;
    case kLivoxLidarDoubleEchoData:
        appendDoubleEcho(packet, lineCount, frame);
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
    currentFrame_ = SlamInputFrame();
    hasCurrentFrame_ = false;
    nextSequence_ = 0;
    lastPointPacketTimestampNs_ = 0;
    hasLastPointPacketTimestamp_ = false;
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

    const int64_t packetTimestampNs = int64_t(LivoxCore::parseLivoxTimestamp(packet->timestamp));
    QMutexLocker locker(&mutex_);
    if (hasLastPointPacketTimestamp_ && packetTimestampNs < lastPointPacketTimestampNs_) {
        ++stats_.outOfOrderPointPacketCount;
        stats_.status = SlamStatusCode::TimeSyncError;
        stats_.message = QStringLiteral("实时点云 packet 时间戳乱序。");
        return false;
    }
    lastPointPacketTimestampNs_ = packetTimestampNs;
    hasLastPointPacketTimestamp_ = true;

    if (!hasCurrentFrame_) {
        currentFrame_ = SlamInputFrame();
        currentFrame_.sequence = nextSequence_++;
        currentFrame_.sourceId = handle;
        currentFrame_.deviceType = deviceType;
        currentFrame_.frameStartNs = packetTimestampNs;
        currentFrame_.frameEndNs = packetTimestampNs;
        currentFrame_.timeSource = SlamTimeSource::LivoxPacketTimestamp;
        currentFrame_.hasPointOffsetTime = true;
        currentFrame_.sourceName = sourceName;
        hasCurrentFrame_ = true;
    } else if (packetTimestampNs - currentFrame_.frameStartNs >= frameDurationNs_) {
        flushCurrentFrameLocked();
        currentFrame_ = SlamInputFrame();
        currentFrame_.sequence = nextSequence_++;
        currentFrame_.sourceId = handle;
        currentFrame_.deviceType = deviceType;
        currentFrame_.frameStartNs = packetTimestampNs;
        currentFrame_.frameEndNs = packetTimestampNs;
        currentFrame_.timeSource = SlamTimeSource::LivoxPacketTimestamp;
        currentFrame_.hasPointOffsetTime = true;
        currentFrame_.sourceName = sourceName;
        hasCurrentFrame_ = true;
    }

    if (!hasOffsetTime(packet->dot_num, packet->time_interval)) {
        ++stats_.missingPointOffsetPacketCount;
        stats_.status = SlamStatusCode::TimeSyncError;
        stats_.message = QStringLiteral("实时点云 packet 缺少可用点内 offset 时间。");
    }

    const qsizetype pointCountBefore = currentFrame_.points.size();
    if (!appendPacketPoints(packet, LivoxCore::lineCountForDeviceType(deviceType), currentFrame_)) {
        return false;
    }

    ++stats_.pointPacketCount;
    stats_.pointCount += uint64_t(currentFrame_.points.size() - pointCountBefore);
    if (stats_.status == SlamStatusCode::Idle) {
        stats_.status = SlamStatusCode::Running;
    }
    return true;
}

bool LiveLidarSlamSource::appendImuPacket(uint32_t handle, const LivoxLidarEthernetPacket* packet)
{
    if (packet == nullptr || packet->data_type != kLivoxLidarImuData || packet->dot_num == 0) {
        return false;
    }

    const int64_t packetTimestampNs = int64_t(LivoxCore::parseLivoxTimestamp(packet->timestamp));
    const bool hasTiming = hasOffsetTime(packet->dot_num, packet->time_interval);
    const auto* points = reinterpret_cast<const LivoxLidarImuRawPoint*>(packet->data);

    QMutexLocker locker(&mutex_);
    if (!hasTiming) {
        ++stats_.missingImuTimingPacketCount;
        stats_.status = SlamStatusCode::TimeSyncError;
        stats_.message = QStringLiteral("实时 IMU packet 缺少可用包内采样时间。");
    }

    for (uint32_t i = 0; i < packet->dot_num; ++i) {
        SlamImuSample sample;
        sample.lidarId = handle;
        sample.timestampNs = pointTimestampNs(packetTimestampNs, packet->time_interval, i);
        sample.gyroRadPerSec[0] = points[i].gyro_x;
        sample.gyroRadPerSec[1] = points[i].gyro_y;
        sample.gyroRadPerSec[2] = points[i].gyro_z;
        sample.accelMps2[0] = points[i].acc_x;
        sample.accelMps2[1] = points[i].acc_y;
        sample.accelMps2[2] = points[i].acc_z;
        imuBuffer_.push_back(sample);
    }

    pruneImuBufferLocked(packetTimestampNs);
    ++stats_.imuPacketCount;
    stats_.imuSampleCount += packet->dot_num;
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
    result.droppedFrameCount = queueStats.droppedFrameCount;
    const qint64 elapsedMs = statsTimer_.elapsed();
    if (elapsedMs > 0) {
        result.inputFps = double(result.inputFrameCount) * 1000.0 / double(elapsedMs);
    }
    return result;
}

void LiveLidarSlamSource::flushCurrentFrameLocked()
{
    if (!hasCurrentFrame_ || currentFrame_.points.isEmpty()) {
        hasCurrentFrame_ = false;
        return;
    }

    attachImuSamplesLocked(currentFrame_);
    if (!currentFrame_.hasCompleteImuCoverage) {
        ++stats_.incompleteImuCoverageFrameCount;
        if (stats_.status != SlamStatusCode::TimeSyncError) {
            stats_.status = currentFrame_.imuSamples.isEmpty() ? SlamStatusCode::MissingImu : SlamStatusCode::TimeSyncError;
            stats_.message = currentFrame_.imuSamples.isEmpty()
                ? QStringLiteral("实时输入帧缺少 IMU 样本。")
                : QStringLiteral("实时 IMU 样本未完整覆盖输入帧。");
        }
    }

    stats_.lastFrameStartNs = currentFrame_.frameStartNs;
    stats_.lastFrameEndNs = currentFrame_.frameEndNs;
    queue_.push(std::move(currentFrame_));
    ++stats_.inputFrameCount;
    hasCurrentFrame_ = false;
}

void LiveLidarSlamSource::attachImuSamplesLocked(SlamInputFrame& frame) const
{
    const auto firstInside = std::lower_bound(imuBuffer_.begin(),
                                              imuBuffer_.end(),
                                              frame.frameStartNs,
                                              [](const SlamImuSample& sample, int64_t timestampNs) {
                                                  return sample.timestampNs < timestampNs;
                                              });
    auto attachBegin = firstInside;
    if (attachBegin != imuBuffer_.begin()) {
        --attachBegin;
    }

    const auto firstAfterEnd = std::upper_bound(imuBuffer_.begin(),
                                                imuBuffer_.end(),
                                                frame.frameEndNs,
                                                [](int64_t timestampNs, const SlamImuSample& sample) {
                                                    return timestampNs < sample.timestampNs;
                                                });
    auto attachEnd = firstAfterEnd;
    if (attachEnd != imuBuffer_.end()) {
        ++attachEnd;
    }

    for (auto it = attachBegin; it != attachEnd; ++it) {
        frame.imuSamples.push_back(*it);
    }

    if (!frame.imuSamples.isEmpty()) {
        frame.hasCompleteImuCoverage =
            frame.imuSamples.first().timestampNs <= frame.frameStartNs &&
            frame.imuSamples.last().timestampNs >= frame.frameEndNs;
    }
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

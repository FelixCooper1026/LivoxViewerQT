#include "Rosbag/RosbagPlaybackSource.h"

#include <algorithm>
#include <cmath>

namespace {

uint32_t defaultLidarIdForFrames(const QVector<SlamInputFrame>& frames)
{
    for (const SlamInputFrame& frame : frames) {
        if (frame.sourceId != 0) {
            return frame.sourceId;
        }
    }
    return 1;
}

QString rosbagSourceDisplayName(const RosbagSlamSourceSummary& summary)
{
    const QString format = summary.format.contains(QStringLiteral("ROS2"), Qt::CaseInsensitive)
        ? QStringLiteral("ROS2")
        : QStringLiteral("ROS1");
    return summary.lidarFormat.isEmpty()
        ? QStringLiteral("%1 ROSbag").arg(format)
        : QStringLiteral("%1 %2").arg(format, summary.lidarFormat);
}

PointCloudPoint toPointCloudPoint(const SlamPoint& point)
{
    PointCloudPoint result{};
    result.x = point.x;
    result.y = point.y;
    result.z = point.z;
    result.r = 1.0f;
    result.g = 1.0f;
    result.b = 1.0f;
    result.reflectivity = point.reflectivity;
    result.tag = point.tag;
    result.line = point.line;
    result.spherical = false;
    result.theta = 0.0f;
    result.phi = 0.0f;
    result.depth = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    return result;
}

Playback::ImuSample toPlaybackImuSample(const SlamImuSample& sample, uint32_t defaultLidarId)
{
    Playback::ImuSample result;
    result.lidarId = sample.lidarId == 0 ? defaultLidarId : sample.lidarId;
    result.timestampNs = sample.timestampNs < 0 ? 0ULL : uint64_t(sample.timestampNs);
    result.gyroX = float(sample.gyroRadPerSec[0]);
    result.gyroY = float(sample.gyroRadPerSec[1]);
    result.gyroZ = float(sample.gyroRadPerSec[2]);
    result.accX = float(sample.accelRaw[0]);
    result.accY = float(sample.accelRaw[1]);
    result.accZ = float(sample.accelRaw[2]);
    return result;
}

uint64_t fallbackNominalFrameDurationNs(int frameDurationMs)
{
    return uint64_t(std::max(1, frameDurationMs)) * 1000000ULL;
}

uint64_t inferNominalFrameDurationNs(const QVector<SlamInputFrame>& frames, int frameDurationMs)
{
    QVector<int64_t> deltas;
    if (frames.size() > 1) {
        deltas.reserve(frames.size() - 1);
    }
    for (int i = 1; i < frames.size(); ++i) {
        const int64_t delta = frames.at(i).frameStartNs - frames.at(i - 1).frameStartNs;
        if (delta > 0) {
            deltas.push_back(delta);
        }
    }
    if (deltas.isEmpty()) {
        return fallbackNominalFrameDurationNs(frameDurationMs);
    }

    std::sort(deltas.begin(), deltas.end());
    return uint64_t(deltas.at(deltas.size() / 2));
}

bool hasValidFrameTimestamps(const QVector<SlamInputFrame>& frames)
{
    if (frames.isEmpty() || frames.first().frameStartNs <= 0) {
        return false;
    }
    if (frames.size() == 1) {
        return true;
    }
    for (int i = 1; i < frames.size(); ++i) {
        if (frames.at(i).frameStartNs > frames.at(i - 1).frameStartNs) {
            return true;
        }
    }
    return false;
}

} // namespace

RosbagPlaybackSource::RosbagPlaybackSource(int frameDurationMs)
    : frameDurationMs_(std::max(1, frameDurationMs))
{
    nominalFrameDurationNs_ = fallbackNominalFrameDurationNs(frameDurationMs_);
}

bool RosbagPlaybackSource::load(const QString& filePath)
{
    filePath_ = filePath;
    errorMessage_.clear();
    summaryText_.clear();
    summary_ = RosbagSlamSourceSummary();
    frames_.clear();
    imuSamples_.clear();
    devices_.clear();
    nominalFrameDurationNs_ = fallbackNominalFrameDurationNs(frameDurationMs_);
    hasFrameTimestamps_ = false;

    RosbagSlamSourceConfig config;
    config.frameDurationMs = frameDurationMs_;
    config.requireImu = false;
    config.allowLivoxDriver2PointCloud2 = true;
    config.allowLivoxDriverPointCloud2SynthesizedTime = true;
    config.synthesizePointOffsetTime = true;

    RosbagSlamSource source(config);
    QString error;
    if (!source.load(filePath, &error)) {
        errorMessage_ = error;
        return false;
    }

    frames_ = source.frames();
    nominalFrameDurationNs_ = inferNominalFrameDurationNs(frames_, frameDurationMs_);
    hasFrameTimestamps_ = hasValidFrameTimestamps(frames_);
    imuSamples_ = source.imuSamples();
    std::sort(imuSamples_.begin(), imuSamples_.end(), [](const SlamImuSample& lhs, const SlamImuSample& rhs) {
        return lhs.timestampNs < rhs.timestampNs;
    });
    summaryText_ = source.summaryText();

    if (frames_.isEmpty()) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：未生成可播放点云帧。");
        return false;
    }

    const RosbagSlamSourceSummary& summary = source.summary();
    summary_ = summary;
    Playback::DeviceInfo device;
    device.lidarId = defaultLidarIdForFrames(frames_);
    device.deviceType = 0;
    device.lidarSn.clear();
    device.modelDisplay = rosbagSourceDisplayName(summary);
    devices_.push_back(device);
    return true;
}

Playback::SourceKind RosbagPlaybackSource::kind() const
{
    return Playback::SourceKind::Rosbag;
}

QString RosbagPlaybackSource::path() const
{
    return filePath_;
}

QString RosbagPlaybackSource::errorMessage() const
{
    return errorMessage_;
}

int RosbagPlaybackSource::frameCount() const
{
    return frames_.size();
}

QVector<Playback::DeviceInfo> RosbagPlaybackSource::devices() const
{
    return devices_;
}

Playback::SourceInfo RosbagPlaybackSource::sourceInfo() const
{
    Playback::SourceInfo info;
    info.kind = Playback::SourceKind::Rosbag;
    info.displayName = rosbagSourceDisplayName(summary_);
    info.format = summary_.format;
    info.filePath = filePath_;
    info.frameCount = uint64_t(frames_.size());
    info.pointCount = summary_.pointCount;
    info.imuSampleCount = summary_.imuSampleCount;
    info.startTimestampNs = summary_.startTimestampNs;
    info.endTimestampNs = summary_.endTimestampNs;

    if (!summary_.lidarTopic.isEmpty()) {
        Playback::TopicInfo topic;
        topic.topic = summary_.lidarTopic;
        topic.type = summary_.lidarType;
        topic.messageCount = summary_.lidarMessageCount;
        topic.pointCount = summary_.pointCount;
        info.lidarTopics.push_back(topic);
    }
    if (!summary_.imuTopic.isEmpty()) {
        Playback::TopicInfo topic;
        topic.topic = summary_.imuTopic;
        topic.type = summary_.imuType;
        topic.messageCount = summary_.imuMessageCount;
        topic.pointCount = 0;
        info.imuTopics.push_back(topic);
    }
    return info;
}

bool RosbagPlaybackSource::readFrame(int frameIndex,
                                     const QMap<uint32_t, bool>& deviceVisibility,
                                     PointCloudFrame& frame)
{
    if (frameIndex < 0 || frameIndex >= frames_.size()) {
        errorMessage_ = QStringLiteral("ROSbag帧索引越界");
        return false;
    }

    const SlamInputFrame& sourceFrame = frames_.at(frameIndex);
    const uint32_t defaultLidarId = devices_.isEmpty() ? 1 : devices_.first().lidarId;
    const uint32_t lidarId = sourceFrame.sourceId == 0 ? defaultLidarId : sourceFrame.sourceId;

    frame = PointCloudFrame();
    frame.device_handle = lidarId;
    frame.timestamp = sourceFrame.frameStartNs < 0 ? 0ULL : uint64_t(sourceFrame.frameStartNs);

    if (!deviceVisibility.value(lidarId, true)) {
        return true;
    }

    frame.points.reserve(sourceFrame.points.size());
    for (const SlamPoint& point : sourceFrame.points) {
        frame.points.push_back(toPointCloudPoint(point));
    }
    frame.pointsByLidar.insert(lidarId, frame.points);
    return true;
}

QVector<Playback::ImuSample> RosbagPlaybackSource::readImuSamples(uint64_t startTimestampNs,
                                                                  uint64_t endTimestampNs) const
{
    QVector<Playback::ImuSample> samples;
    if (endTimestampNs <= startTimestampNs) {
        return samples;
    }

    const uint32_t defaultLidarId = devices_.isEmpty() ? 1 : devices_.first().lidarId;
    const auto begin = std::lower_bound(imuSamples_.constBegin(),
                                        imuSamples_.constEnd(),
                                        int64_t(startTimestampNs),
                                        [](const SlamImuSample& sample, int64_t timestampNs) {
                                            return sample.timestampNs < timestampNs;
                                        });
    for (auto it = begin; it != imuSamples_.constEnd(); ++it) {
        const SlamImuSample& sample = *it;
        if (sample.timestampNs < 0) {
            continue;
        }
        const uint64_t timestampNs = uint64_t(sample.timestampNs);
        if (timestampNs >= endTimestampNs) {
            break;
        }
        samples.push_back(toPlaybackImuSample(sample, defaultLidarId));
    }
    return samples;
}

uint64_t RosbagPlaybackSource::nominalFrameDurationNs() const
{
    return nominalFrameDurationNs_;
}

uint64_t RosbagPlaybackSource::frameTimestampNs(int frameIndex) const
{
    if (frameIndex < 0 || frameIndex >= frames_.size()) {
        return 0;
    }
    const int64_t timestampNs = frames_.at(frameIndex).frameStartNs;
    return timestampNs < 0 ? 0ULL : uint64_t(timestampNs);
}

bool RosbagPlaybackSource::hasFrameTimestamps() const
{
    return hasFrameTimestamps_;
}

void RosbagPlaybackSource::invalidateCache()
{
}

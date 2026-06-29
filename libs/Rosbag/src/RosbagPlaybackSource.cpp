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
    result.accX = float(sample.accelMps2[0]);
    result.accY = float(sample.accelMps2[1]);
    result.accZ = float(sample.accelMps2[2]);
    return result;
}

} // namespace

RosbagPlaybackSource::RosbagPlaybackSource(int frameDurationMs)
    : frameDurationMs_(std::max(1, frameDurationMs))
{
}

bool RosbagPlaybackSource::load(const QString& filePath)
{
    filePath_ = filePath;
    errorMessage_.clear();
    summaryText_.clear();
    frames_.clear();
    imuSamples_.clear();
    devices_.clear();

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
    Playback::DeviceInfo device;
    device.lidarId = defaultLidarIdForFrames(frames_);
    device.deviceType = 0;
    device.lidarSn = summary.lidarTopic;
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

void RosbagPlaybackSource::invalidateCache()
{
}

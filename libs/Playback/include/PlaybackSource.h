#ifndef PLAYBACK_PLAYBACKSOURCE_H
#define PLAYBACK_PLAYBACKSOURCE_H

#include "PointCloudFrame.h"

#include <QMap>
#include <QMatrix4x4>
#include <QString>
#include <QVector>

#include <algorithm>
#include <cstdint>

namespace Playback {

enum class SourceKind {
    Lvx,
    Lvx2,
    Pcap,
    Rosbag
};

struct FrameRef {
    uint64_t offset = 0;
    uint64_t nextOffset = 0;
    uint64_t frameIndex = 0;
};

struct Extrinsic {
    bool enabled = false;
    QMatrix4x4 transform;
};

struct DeviceInfo {
    uint32_t lidarId = 0;
    uint8_t deviceType = 0;
    QString lidarSn;
    QString modelDisplay;
    Extrinsic extrinsic;
};

struct TopicInfo {
    QString topic;
    QString type;
    uint64_t messageCount = 0;
    uint64_t pointCount = 0;
};

struct SourceInfo {
    SourceKind kind = SourceKind::Lvx2;
    QString displayName;
    QString format;
    QString filePath;
    QVector<TopicInfo> lidarTopics;
    QVector<TopicInfo> imuTopics;
    uint64_t frameCount = 0;
    uint64_t pointCount = 0;
    uint64_t imuSampleCount = 0;
    int64_t startTimestampNs = 0;
    int64_t endTimestampNs = 0;
};

struct ImuSample {
    uint32_t lidarId = 0;
    uint64_t timestampNs = 0;
    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;
    float accX = 0.0f;
    float accY = 0.0f;
    float accZ = 0.0f;
};

class Source {
public:
    virtual ~Source() = default;

    virtual bool load(const QString& filePath) = 0;
    virtual SourceKind kind() const = 0;
    virtual QString path() const = 0;
    virtual QString errorMessage() const = 0;
    virtual int frameCount() const = 0;
    virtual QVector<DeviceInfo> devices() const = 0;
    virtual bool readFrame(int frameIndex,
                           const QMap<uint32_t, bool>& deviceVisibility,
                           PointCloudFrame& frame) = 0;
    virtual QVector<ImuSample> readImuSamples(uint64_t startTimestampNs, uint64_t endTimestampNs) const
    {
        Q_UNUSED(startTimestampNs);
        Q_UNUSED(endTimestampNs);
        return {};
    }
    virtual uint64_t nominalFrameDurationNs() const
    {
        return 50000000ULL;
    }
    virtual uint64_t frameTimestampNs(int frameIndex) const
    {
        Q_UNUSED(frameIndex);
        return 0;
    }
    virtual bool hasFrameTimestamps() const
    {
        return false;
    }
    virtual SourceInfo sourceInfo() const
    {
        SourceInfo info;
        info.kind = kind();
        info.filePath = path();
        info.frameCount = uint64_t(std::max(0, frameCount()));
        return info;
    }
    virtual void invalidateCache() {}
};

} // namespace Playback

#endif // PLAYBACK_PLAYBACKSOURCE_H

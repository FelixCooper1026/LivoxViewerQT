#ifndef PLAYBACK_PLAYBACKSOURCE_H
#define PLAYBACK_PLAYBACKSOURCE_H

#include "PointCloud/PointCloudFrame.h"

#include <QMap>
#include <QMatrix4x4>
#include <QString>
#include <QVector>

#include <cstdint>

namespace Playback {

enum class SourceKind {
    Lvx2,
    Pcap
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
    virtual void invalidateCache() {}
};

} // namespace Playback

#endif // PLAYBACK_PLAYBACKSOURCE_H

#ifndef ROSBAG_ROSBAGPLAYBACKSOURCE_H
#define ROSBAG_ROSBAGPLAYBACKSOURCE_H

#include "Playback/PlaybackSource.h"
#include "Slam/Io/RosbagSlamSource.h"

class RosbagPlaybackSource final : public Playback::Source {
public:
    explicit RosbagPlaybackSource(int frameDurationMs = 100);

    bool load(const QString& filePath) override;
    Playback::SourceKind kind() const override;
    QString path() const override;
    QString errorMessage() const override;
    int frameCount() const override;
    QVector<Playback::DeviceInfo> devices() const override;
    bool readFrame(int frameIndex,
                   const QMap<uint32_t, bool>& deviceVisibility,
                   PointCloudFrame& frame) override;
    QVector<Playback::ImuSample> readImuSamples(uint64_t startTimestampNs, uint64_t endTimestampNs) const override;
    void invalidateCache() override;

private:
    QString filePath_;
    QString errorMessage_;
    QString summaryText_;
    QVector<SlamInputFrame> frames_;
    QVector<SlamImuSample> imuSamples_;
    QVector<Playback::DeviceInfo> devices_;
    int frameDurationMs_ = 100;
};

#endif // ROSBAG_ROSBAGPLAYBACKSOURCE_H

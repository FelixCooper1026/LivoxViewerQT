#ifndef ROSBAG_ROSBAGPLAYBACKSOURCE_H
#define ROSBAG_ROSBAGPLAYBACKSOURCE_H

#include "PlaybackSource.h"
#include "Io/RosbagSlamSource.h"

class RosbagPlaybackSource final : public Playback::Source {
public:
    explicit RosbagPlaybackSource(int frameDurationMs = 100);

    bool load(const QString& filePath) override;
    Playback::SourceKind kind() const override;
    QString path() const override;
    QString errorMessage() const override;
    int frameCount() const override;
    QVector<Playback::DeviceInfo> devices() const override;
    Playback::SourceInfo sourceInfo() const override;
    bool readFrame(int frameIndex,
                   const QMap<uint32_t, bool>& deviceVisibility,
                   PointCloudFrame& frame) override;
    QVector<Playback::ImuSample> readImuSamples(uint64_t startTimestampNs, uint64_t endTimestampNs) const override;
    uint64_t nominalFrameDurationNs() const override;
    uint64_t frameTimestampNs(int frameIndex) const override;
    bool hasFrameTimestamps() const override;
    void invalidateCache() override;

private:
    QString filePath_;
    QString errorMessage_;
    QString summaryText_;
    RosbagSlamSourceSummary summary_;
    QVector<SlamInputFrame> frames_;
    QVector<SlamImuSample> imuSamples_;
    QVector<Playback::DeviceInfo> devices_;
    uint64_t nominalFrameDurationNs_ = 100000000ULL;
    bool hasFrameTimestamps_ = false;
    int frameDurationMs_ = 100;
};

#endif // ROSBAG_ROSBAGPLAYBACKSOURCE_H

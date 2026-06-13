#ifndef PCAP_PCAPREADER_H
#define PCAP_PCAPREADER_H

#include "Playback/PlaybackSource.h"

namespace Pcap {

class PcapReader final : public Playback::Source {
public:
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

private:
    QString filePath_;
    QString errorMessage_;
    QVector<PointCloudFrame> frames_;
    QVector<Playback::ImuSample> imuSamples_;
    QVector<Playback::DeviceInfo> devices_;
    QMap<uint32_t, Playback::Extrinsic> extrinsics_;
};

} // namespace Pcap

#endif // PCAP_PCAPREADER_H

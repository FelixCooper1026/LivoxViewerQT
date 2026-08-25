#include "PcapReader.h"

#include "Lvx2PointParser.h"
#include "PcapParser.h"
#include "PushMsgParser.h"

#include <algorithm>

namespace Pcap {

bool PcapReader::load(const QString& filePath)
{
    filePath_ = filePath;
    errorMessage_.clear();
    frames_.clear();
    imuSamples_.clear();
    devices_.clear();
    extrinsics_.clear();

    const PcapParser::ParseResult parseResult = PcapParser::parseFileToFrames(filePath);
    if (!parseResult.ok) {
        errorMessage_ = parseResult.errorMessage;
        return false;
    }

    frames_ = parseResult.frames;
    imuSamples_.reserve(parseResult.imuSamples.size());
    for (const ImuParser::Sample& sample : parseResult.imuSamples) {
        Playback::ImuSample playbackSample;
        playbackSample.lidarId = sample.lidarId;
        playbackSample.timestampNs = sample.timestampNs;
        playbackSample.gyroX = sample.gyroX;
        playbackSample.gyroY = sample.gyroY;
        playbackSample.gyroZ = sample.gyroZ;
        playbackSample.accX = sample.accX;
        playbackSample.accY = sample.accY;
        playbackSample.accZ = sample.accZ;
        imuSamples_.push_back(playbackSample);
    }
    for (const PushMsgParser::PushDeviceRecord& device : parseResult.devices) {
        Playback::DeviceInfo uiInfo;
        uiInfo.lidarId = device.lidarId;
        uiInfo.deviceType = device.deviceType;
        uiInfo.lidarSn = device.lidarSn;
        uiInfo.modelDisplay = device.modelDisplay;

        if (device.hasExtrinsic) {
            Playback::Extrinsic extrinsic;
            extrinsic.enabled = true;
            extrinsic.transform.setToIdentity();
            extrinsic.transform.translate(device.offsetX, device.offsetY, device.offsetZ);
            extrinsic.transform.rotate(device.offsetYaw, 0.0f, 0.0f, 1.0f);
            extrinsic.transform.rotate(device.offsetPitch, 0.0f, 1.0f, 0.0f);
            extrinsic.transform.rotate(device.offsetRoll, 1.0f, 0.0f, 0.0f);
            extrinsics_.insert(device.lidarId, extrinsic);
            uiInfo.extrinsic = extrinsic;
        }
        devices_.push_back(uiInfo);
    }

    return true;
}

Playback::SourceKind PcapReader::kind() const
{
    return Playback::SourceKind::Pcap;
}

QString PcapReader::path() const
{
    return filePath_;
}

QString PcapReader::errorMessage() const
{
    return errorMessage_;
}

int PcapReader::frameCount() const
{
    return frames_.size();
}

QVector<Playback::DeviceInfo> PcapReader::devices() const
{
    return devices_;
}

bool PcapReader::readFrame(int frameIndex,
                           const QMap<uint32_t, bool>& deviceVisibility,
                           PointCloudFrame& frame)
{
    if (frameIndex < 0 || frameIndex >= frames_.size()) {
        errorMessage_ = "Pcap帧索引越界";
        return false;
    }

    const PointCloudFrame& raw = frames_.at(frameIndex);
    frame = PointCloudFrame();
    frame.device_handle = raw.device_handle;
    frame.timestamp = raw.timestamp;

    if (!raw.pointsByLidar.isEmpty()) {
        for (auto it = raw.pointsByLidar.constBegin(); it != raw.pointsByLidar.constEnd(); ++it) {
            if (!deviceVisibility.value(it.key(), true)) {
                continue;
            }

            QVector<PointCloudPoint> segment = it.value();
            const auto extrinsicIt = extrinsics_.constFind(it.key());
            const Playback::Extrinsic* extrinsic =
                (extrinsicIt == extrinsics_.constEnd()) ? nullptr : &extrinsicIt.value();
            if (extrinsic && extrinsic->enabled) {
                for (PointCloudPoint& point : segment) {
                    Lvx2PointParser::applyExtrinsicTransform(extrinsic, point);
                }
            }
            frame.points += segment;
        }
        return true;
    }

    frame.points = raw.points;
    return true;
}

QVector<Playback::ImuSample> PcapReader::readImuSamples(uint64_t startTimestampNs, uint64_t endTimestampNs) const
{
    QVector<Playback::ImuSample> samples;
    const auto begin = std::lower_bound(imuSamples_.constBegin(),
                                        imuSamples_.constEnd(),
                                        startTimestampNs,
                                        [](const Playback::ImuSample& sample, uint64_t timestampNs) {
                                            return sample.timestampNs < timestampNs;
                                        });
    for (auto it = begin; it != imuSamples_.constEnd() && it->timestampNs < endTimestampNs; ++it) {
        samples.push_back(*it);
    }
    return samples;
}

} // namespace Pcap

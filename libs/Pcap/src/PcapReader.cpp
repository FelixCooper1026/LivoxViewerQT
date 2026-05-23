#include "Pcap/PcapReader.h"

#include "Lvx2/Lvx2PointParser.h"
#include "Pcap/PcapParser.h"
#include "Pcap/PushMsgParser.h"

namespace Pcap {

bool PcapReader::load(const QString& filePath)
{
    filePath_ = filePath;
    errorMessage_.clear();
    frames_.clear();
    devices_.clear();
    extrinsics_.clear();

    const PcapParser::ParseResult parseResult = PcapParser::parseFileToFrames(filePath);
    if (!parseResult.ok) {
        errorMessage_ = parseResult.errorMessage;
        return false;
    }

    frames_ = parseResult.frames;
    for (const PushMsgParser::PushDeviceRecord& device : parseResult.devices) {
        Playback::DeviceInfo uiInfo;
        uiInfo.lidarId = device.lidarId;
        uiInfo.deviceType = device.deviceType;
        uiInfo.lidarSn = device.lidarSn;
        uiInfo.modelDisplay = device.modelDisplay;
        devices_.push_back(uiInfo);

        if (device.hasExtrinsic) {
            Playback::Extrinsic extrinsic;
            extrinsic.enabled = true;
            extrinsic.transform.setToIdentity();
            extrinsic.transform.translate(device.offsetX, device.offsetY, device.offsetZ);
            extrinsic.transform.rotate(device.offsetYaw, 0.0f, 0.0f, 1.0f);
            extrinsic.transform.rotate(device.offsetPitch, 0.0f, 1.0f, 0.0f);
            extrinsic.transform.rotate(device.offsetRoll, 1.0f, 0.0f, 0.0f);
            extrinsics_.insert(device.lidarId, extrinsic);
        }
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

} // namespace Pcap

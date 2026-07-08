#include "Lvx2Reader.h"

#include "LidarModelUtils.h"
#include "Lvx2PointParser.h"

#include <QFile>
#include <QtEndian>

namespace {

bool readExact(QFile& file, char* data, qint64 size)
{
    return file.read(data, size) == size;
}

uint64_t parseTimestampValue(uint64_t raw)
{
    return qFromBigEndian(raw);
}

} // namespace

namespace Lvx2 {

Lvx2Reader::Lvx2Reader() = default;
Lvx2Reader::~Lvx2Reader() = default;

bool Lvx2Reader::load(const QString& filePath)
{
    filePath_ = filePath;
    errorMessage_.clear();
    frames_.clear();
    extrinsics_.clear();
    lineCounts_.clear();
    devices_.clear();
    frameCache_.clear();
    frameCacheValid_.clear();
    playbackFile_.reset();

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        errorMessage_ = "无法打开LVX2文件";
        return false;
    }

    Lvx2PublicHeader publicHeader{};
    Lvx2PrivateHeader privateHeader{};
    if (!readExact(file, reinterpret_cast<char*>(&publicHeader), sizeof(publicHeader)) ||
        !readExact(file, reinterpret_cast<char*>(&privateHeader), sizeof(privateHeader))) {
        errorMessage_ = "LVX2文件头读取失败";
        return false;
    }
    if (std::memcmp(publicHeader.signature, "livox_tech", 10) != 0 ||
        publicHeader.magic_code != 0xAC0EA767) {
        errorMessage_ = "不是有效的LVX2文件";
        return false;
    }

    const int deviceCount = std::max(0, int(privateHeader.device_count));
    for (int i = 0; i < deviceCount; ++i) {
        Lvx2DeviceInfo deviceInfo{};
        if (!readExact(file, reinterpret_cast<char*>(&deviceInfo), sizeof(deviceInfo))) {
            errorMessage_ = "LVX2设备信息读取失败";
            return false;
        }

        extrinsics_.insert(deviceInfo.lidar_id, Lvx2PointParser::makeExtrinsic(deviceInfo));
        lineCounts_.insert(deviceInfo.lidar_id, LivoxCore::lineCountForDeviceType(deviceInfo.device_type));

        Playback::DeviceInfo uiInfo;
        uiInfo.lidarId = deviceInfo.lidar_id;
        uiInfo.deviceType = deviceInfo.device_type;
        uiInfo.lidarSn = QString::fromLatin1(deviceInfo.lidar_sn).trimmed();
        devices_.push_back(uiInfo);
    }

    const qint64 fileSize = file.size();
    while (file.pos() + qint64(sizeof(Lvx2FrameHeader)) <= fileSize) {
        Lvx2FrameHeader frameHeader{};
        const qint64 frameOffset = file.pos();
        if (!readExact(file, reinterpret_cast<char*>(&frameHeader), sizeof(frameHeader))) {
            break;
        }
        if (frameHeader.current_offset != uint64_t(frameOffset) ||
            frameHeader.next_offset <= frameHeader.current_offset ||
            frameHeader.next_offset > uint64_t(fileSize)) {
            break;
        }

        frames_.push_back({frameHeader.current_offset, frameHeader.next_offset, frameHeader.frame_index});
        if (!file.seek(qint64(frameHeader.next_offset))) {
            break;
        }
    }

    if (frames_.isEmpty()) {
        errorMessage_ = "LVX2文件中未找到可播放帧";
        return false;
    }

    frameCache_.resize(frames_.size());
    frameCacheValid_ = QVector<bool>(frames_.size(), false);
    return true;
}

Playback::SourceKind Lvx2Reader::kind() const
{
    return Playback::SourceKind::Lvx2;
}

QString Lvx2Reader::path() const
{
    return filePath_;
}

QString Lvx2Reader::errorMessage() const
{
    return errorMessage_;
}

int Lvx2Reader::frameCount() const
{
    return frames_.size();
}

QVector<Playback::DeviceInfo> Lvx2Reader::devices() const
{
    return devices_;
}

void Lvx2Reader::invalidateCache()
{
    frameCacheValid_.fill(false);
}

bool Lvx2Reader::ensurePlaybackFileOpen()
{
    if (playbackFile_ && playbackFile_->isOpen()) {
        return true;
    }

    playbackFile_ = std::make_unique<QFile>(filePath_);
    if (!playbackFile_->open(QIODevice::ReadOnly)) {
        errorMessage_ = "无法打开LVX2文件进行播放";
        return false;
    }
    return true;
}

bool Lvx2Reader::readFrame(int frameIndex,
                           const QMap<uint32_t, bool>& deviceVisibility,
                           PointCloudFrame& frame)
{
    if (frameIndex < 0 || frameIndex >= frames_.size()) {
        errorMessage_ = "LVX2帧索引越界";
        return false;
    }

    if (frameCacheValid_.value(frameIndex, false)) {
        frame = frameCache_.at(frameIndex);
        return true;
    }

    if (!ensurePlaybackFileOpen()) {
        return false;
    }

    PointCloudFrame parsed;
    parsed.device_handle = 0;
    parsed.timestamp = 0;

    const Playback::FrameRef& frameRef = frames_.at(frameIndex);
    qint64 cursor = qint64(frameRef.offset) + qint64(sizeof(Lvx2FrameHeader));
    while (cursor + qint64(sizeof(Lvx2PackageHeader)) <= qint64(frameRef.nextOffset)) {
        if (!playbackFile_->seek(cursor)) {
            break;
        }
        Lvx2PackageHeader packageHeader{};
        if (!readExact(*playbackFile_, reinterpret_cast<char*>(&packageHeader), sizeof(packageHeader))) {
            break;
        }
        cursor += qint64(sizeof(Lvx2PackageHeader));
        const qint64 dataLength = qint64(packageHeader.data_length);
        if (dataLength < 0 || cursor + dataLength > qint64(frameRef.nextOffset)) {
            break;
        }
        const QByteArray payload = playbackFile_->read(dataLength);
        if (payload.size() != dataLength) {
            break;
        }
        cursor += dataLength;

        if (!extrinsics_.contains(packageHeader.lidar_id)) {
            continue;
        }

        parsed.timestamp = std::max(parsed.timestamp, parseTimestampValue(packageHeader.timestamp));
        if (!deviceVisibility.value(packageHeader.lidar_id, true)) {
            continue;
        }

        const auto extrinsicIt = extrinsics_.constFind(packageHeader.lidar_id);
        const Playback::Extrinsic* extrinsic =
            (extrinsicIt == extrinsics_.constEnd()) ? nullptr : &extrinsicIt.value();
        Lvx2PointParser::appendPackagePoints(packageHeader,
                                             payload,
                                             extrinsic,
                                             parsed.points,
                                             lineCounts_.value(packageHeader.lidar_id, 1));
    }

    frameCache_[frameIndex] = parsed;
    frameCacheValid_[frameIndex] = true;
    frame = frameCache_.at(frameIndex);
    return true;
}

} // namespace Lvx2

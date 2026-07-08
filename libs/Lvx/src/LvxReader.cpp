#include "LvxReader.h"

#include "LidarPacketUtils.h"
#include "LidarSdkTypes.h"
#include "LvxPointParser.h"

#include <QFile>

#include <algorithm>
#include <cstring>

namespace {

bool readExact(QFile& file, char* data, qint64 size)
{
    return file.read(data, size) == size;
}

QString fixedString(const char* data, int size)
{
    const int length = int(strnlen(data, size));
    return QString::fromLatin1(data, length).trimmed();
}

QString lvxDeviceTypeToModel(uint8_t deviceType)
{
    switch (deviceType) {
    case kLivoxLidarTypeHub: return QStringLiteral("LiDAR Hub");
    case kLivoxLidarTypeMid40: return QStringLiteral("Mid-40/Mid-100");
    case kLivoxLidarTypeTele: return QStringLiteral("Tele-15");
    case kLivoxLidarTypeHorizon: return QStringLiteral("Horizon");
    case kLivoxLidarTypeMid70: return QStringLiteral("Mid-70");
    case kLivoxLidarTypeAvia: return QStringLiteral("Avia");
    case kLivoxLidarTypeMid360: return QStringLiteral("Mid360");
    case kLivoxLidarTypeIndustrialHAP: return QStringLiteral("Industrial HAP");
    case kLivoxLidarTypeHAP: return QStringLiteral("HAP");
    case kLivoxLidarTypePA: return QStringLiteral("PA");
    case kLivoxLidarTypeMid360s: return QStringLiteral("Mid360s");
    case kLivoxLidarTypeAvia2: return QStringLiteral("Avia2");
    case kLivoxLidarTypeMid360l: return QStringLiteral("Mid360l");
    default: return QStringLiteral("Unknown(%1)").arg(deviceType);
    }
}

int lvxLineCountForDeviceType(uint8_t deviceType)
{
    Q_UNUSED(deviceType);
    return 1;
}

uint64_t frameDurationToNs(uint32_t durationMs)
{
    return uint64_t(std::max<uint32_t>(1, durationMs)) * 1000000ULL;
}

bool isValidPackageAt(QFile& file,
                      qint64 packageOffset,
                      uint64_t frameEndOffset,
                      bool legacyFloatPoints,
                      const QMap<uint32_t, Playback::Extrinsic>& extrinsics)
{
    if (packageOffset + qint64(sizeof(LvxPackageHeader)) > qint64(frameEndOffset)) {
        return false;
    }
    if (!file.seek(packageOffset)) {
        return false;
    }

    LvxPackageHeader packageHeader{};
    if (!readExact(file, reinterpret_cast<char*>(&packageHeader), sizeof(packageHeader))) {
        return false;
    }
    if (!extrinsics.contains(packageHeader.device_index) || packageHeader.version == 0) {
        return false;
    }

    const int payloadSize = LvxPointParser::payloadSizeForDataType(packageHeader.data_type, legacyFloatPoints);
    return payloadSize >= 0 &&
           packageOffset + qint64(sizeof(LvxPackageHeader)) + qint64(payloadSize) <= qint64(frameEndOffset);
}

int detectFrameHeaderSize(QFile& file,
                          qint64 frameOffset,
                          uint64_t frameEndOffset,
                          bool legacyFloatPoints,
                          const QMap<uint32_t, Playback::Extrinsic>& extrinsics)
{
    constexpr int kFrameHeader24 = int(sizeof(LvxFrameHeader));
    constexpr int kFrameHeader32 = kFrameHeader24 + int(sizeof(uint64_t));
    if (isValidPackageAt(file, frameOffset + kFrameHeader24, frameEndOffset, legacyFloatPoints, extrinsics)) {
        return kFrameHeader24;
    }
    if (isValidPackageAt(file, frameOffset + kFrameHeader32, frameEndOffset, legacyFloatPoints, extrinsics)) {
        return kFrameHeader32;
    }
    return kFrameHeader24;
}

} // namespace

namespace Lvx {

LvxReader::LvxReader() = default;
LvxReader::~LvxReader() = default;

bool LvxReader::load(const QString& filePath)
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
    frameDurationNs_ = 50000000ULL;
    frameHeaderSize_ = int(sizeof(LvxFrameHeader));
    legacyFloatPoints_ = false;

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        errorMessage_ = "无法打开LVX文件";
        return false;
    }

    LvxPublicHeader publicHeader{};
    if (!readExact(file, reinterpret_cast<char*>(&publicHeader), sizeof(publicHeader))) {
        errorMessage_ = "LVX文件头读取失败";
        return false;
    }
    if (std::memcmp(publicHeader.signature, "livox_tech", 10) != 0 ||
        publicHeader.magic_code != 0xAC0EA767 ||
        publicHeader.version_a != 1) {
        errorMessage_ = "不是有效的LVX文件";
        return false;
    }

    legacyFloatPoints_ = (publicHeader.version_b == 0);
    if (legacyFloatPoints_) {
        uint8_t deviceCount = 0;
        if (!readExact(file, reinterpret_cast<char*>(&deviceCount), sizeof(deviceCount))) {
            errorMessage_ = "LVX设备数量读取失败";
            return false;
        }
        for (int i = 0; i < int(deviceCount); ++i) {
            LvxDeviceInfoV10 deviceInfo{};
            if (!readExact(file, reinterpret_cast<char*>(&deviceInfo), sizeof(deviceInfo))) {
                errorMessage_ = "LVX设备信息读取失败";
                return false;
            }

            const uint32_t deviceKey = deviceInfo.device_index;
            extrinsics_.insert(deviceKey, LvxPointParser::makeExtrinsic(deviceInfo));
            lineCounts_.insert(deviceKey, lvxLineCountForDeviceType(deviceInfo.device_type));

            Playback::DeviceInfo uiInfo;
            uiInfo.lidarId = deviceKey;
            uiInfo.deviceType = deviceInfo.device_type;
            uiInfo.lidarSn = fixedString(deviceInfo.lidar_sn, int(sizeof(deviceInfo.lidar_sn)));
            uiInfo.modelDisplay = lvxDeviceTypeToModel(deviceInfo.device_type);
            devices_.push_back(uiInfo);
        }
    } else {
        LvxPrivateHeader privateHeader{};
        if (!readExact(file, reinterpret_cast<char*>(&privateHeader), sizeof(privateHeader))) {
            errorMessage_ = "LVX私有头读取失败";
            return false;
        }
        frameDurationNs_ = frameDurationToNs(privateHeader.frame_duration);
        for (int i = 0; i < int(privateHeader.device_count); ++i) {
            LvxDeviceInfoV11 deviceInfo{};
            if (!readExact(file, reinterpret_cast<char*>(&deviceInfo), sizeof(deviceInfo))) {
                errorMessage_ = "LVX设备信息读取失败";
                return false;
            }

            const uint32_t deviceKey = deviceInfo.device_index;
            extrinsics_.insert(deviceKey, LvxPointParser::makeExtrinsic(deviceInfo));
            lineCounts_.insert(deviceKey, lvxLineCountForDeviceType(deviceInfo.device_type));

            Playback::DeviceInfo uiInfo;
            uiInfo.lidarId = deviceKey;
            uiInfo.deviceType = deviceInfo.device_type;
            uiInfo.lidarSn = fixedString(deviceInfo.lidar_sn, int(sizeof(deviceInfo.lidar_sn)));
            uiInfo.modelDisplay = lvxDeviceTypeToModel(deviceInfo.device_type);
            devices_.push_back(uiInfo);
        }
    }

    const qint64 fileSize = file.size();
    const qint64 firstFrameOffset = file.pos();
    if (firstFrameOffset + qint64(sizeof(LvxFrameHeader)) <= fileSize) {
        LvxFrameHeader frameHeader{};
        if (readExact(file, reinterpret_cast<char*>(&frameHeader), sizeof(frameHeader)) &&
            frameHeader.current_offset == uint64_t(firstFrameOffset) &&
            frameHeader.next_offset > frameHeader.current_offset &&
            frameHeader.next_offset <= uint64_t(fileSize)) {
            frameHeaderSize_ = detectFrameHeaderSize(file,
                                                     firstFrameOffset,
                                                     frameHeader.next_offset,
                                                     legacyFloatPoints_,
                                                     extrinsics_);
        }
        file.seek(firstFrameOffset);
    }

    while (file.pos() + qint64(sizeof(LvxFrameHeader)) <= fileSize) {
        LvxFrameHeader frameHeader{};
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
        errorMessage_ = "LVX文件中未找到可播放帧";
        return false;
    }

    frameCache_.resize(frames_.size());
    frameCacheValid_ = QVector<bool>(frames_.size(), false);
    return true;
}

Playback::SourceKind LvxReader::kind() const
{
    return Playback::SourceKind::Lvx;
}

QString LvxReader::path() const
{
    return filePath_;
}

QString LvxReader::errorMessage() const
{
    return errorMessage_;
}

int LvxReader::frameCount() const
{
    return frames_.size();
}

QVector<Playback::DeviceInfo> LvxReader::devices() const
{
    return devices_;
}

uint64_t LvxReader::nominalFrameDurationNs() const
{
    return frameDurationNs_;
}

void LvxReader::invalidateCache()
{
    frameCacheValid_.fill(false);
}

bool LvxReader::ensurePlaybackFileOpen()
{
    if (playbackFile_ && playbackFile_->isOpen()) {
        return true;
    }

    playbackFile_ = std::make_unique<QFile>(filePath_);
    if (!playbackFile_->open(QIODevice::ReadOnly)) {
        errorMessage_ = "无法打开LVX文件进行播放";
        return false;
    }
    return true;
}

bool LvxReader::readFrame(int frameIndex,
                          const QMap<uint32_t, bool>& deviceVisibility,
                          PointCloudFrame& frame)
{
    if (frameIndex < 0 || frameIndex >= frames_.size()) {
        errorMessage_ = "LVX帧索引越界";
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
    qint64 cursor = qint64(frameRef.offset) + qint64(frameHeaderSize_);
    while (cursor + qint64(sizeof(LvxPackageHeader)) <= qint64(frameRef.nextOffset)) {
        if (!playbackFile_->seek(cursor)) {
            break;
        }
        LvxPackageHeader packageHeader{};
        if (!readExact(*playbackFile_, reinterpret_cast<char*>(&packageHeader), sizeof(packageHeader))) {
            break;
        }
        cursor += qint64(sizeof(LvxPackageHeader));

        const int payloadSize = LvxPointParser::payloadSizeForDataType(packageHeader.data_type, legacyFloatPoints_);
        if (payloadSize < 0 || cursor + qint64(payloadSize) > qint64(frameRef.nextOffset)) {
            break;
        }
        const QByteArray payload = playbackFile_->read(payloadSize);
        if (payload.size() != payloadSize) {
            break;
        }
        cursor += payloadSize;

        const uint32_t deviceKey = packageHeader.device_index;
        parsed.timestamp = std::max(parsed.timestamp, LivoxCore::parseLivoxTimestamp(packageHeader.timestamp));
        if (!deviceVisibility.value(deviceKey, true)) {
            continue;
        }

        const auto extrinsicIt = extrinsics_.constFind(deviceKey);
        const Playback::Extrinsic* extrinsic =
            (extrinsicIt == extrinsics_.constEnd() || legacyFloatPoints_) ? nullptr : &extrinsicIt.value();
        LvxPointParser::appendPackagePoints(packageHeader,
                                            payload,
                                            legacyFloatPoints_,
                                            extrinsic,
                                            parsed.points,
                                            lineCounts_.value(deviceKey, 1));
    }

    frameCache_[frameIndex] = parsed;
    frameCacheValid_[frameIndex] = true;
    frame = frameCache_.at(frameIndex);
    return true;
}

} // namespace Lvx

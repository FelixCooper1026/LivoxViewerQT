#include "Lvx2Writer.h"

#include <cstring>

namespace Lvx2 {

namespace {

constexpr uint64_t kFrameDurationNs = 50000000ULL;

bool writeExact(QFile& file, const char* data, qint64 size)
{
    return file.write(data, size) == size;
}

} // namespace

bool Lvx2Writer::open(const QString& filePath,
                      const QVector<Lvx2DeviceInfo>& devices,
                      QString& errorMessage)
{
    file_.setFileName(filePath);
    if (!file_.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        errorMessage = QStringLiteral("无法创建 LVX2 文件：%1").arg(file_.errorString());
        return false;
    }

    Lvx2PublicHeader publicHeader{};
    Lvx2PrivateHeader privateHeader{};
    privateHeader.frame_duration = 50;
    privateHeader.device_count = uint8_t(devices.size());
    if (!writeExact(file_, reinterpret_cast<const char*>(&publicHeader), sizeof(publicHeader)) ||
        !writeExact(file_, reinterpret_cast<const char*>(&privateHeader), sizeof(privateHeader))) {
        errorMessage = QStringLiteral("LVX2 文件头写入失败：%1").arg(file_.errorString());
        abort();
        return false;
    }
    for (const Lvx2DeviceInfo& device : devices) {
        if (!writeExact(file_, reinterpret_cast<const char*>(&device), sizeof(device))) {
            errorMessage = QStringLiteral("LVX2 设备表写入失败：%1").arg(file_.errorString());
            abort();
            return false;
        }
    }
    return true;
}

bool Lvx2Writer::appendPacket(uint64_t captureTimestampNs,
                              const Lvx2PackageHeader& header,
                              const uint8_t* data,
                              size_t dataLength,
                              QString& errorMessage)
{
    if (frameStartNs_ == 0) {
        frameStartNs_ = captureTimestampNs;
    } else if (captureTimestampNs - frameStartNs_ >= kFrameDurationNs) {
        if (!flushFrame(errorMessage)) {
            return false;
        }
        frameStartNs_ = captureTimestampNs;
    }

    QByteArray package;
    package.reserve(int(sizeof(header) + dataLength));
    package.append(reinterpret_cast<const char*>(&header), sizeof(header));
    package.append(reinterpret_cast<const char*>(data), int(dataLength));
    packages_.push_back(std::move(package));
    return true;
}

bool Lvx2Writer::flushFrame(QString& errorMessage)
{
    if (packages_.isEmpty()) {
        return true;
    }

    const uint64_t currentOffset = uint64_t(file_.pos());
    uint64_t packageBytes = 0;
    for (const QByteArray& package : packages_) {
        packageBytes += uint64_t(package.size());
    }
    Lvx2FrameHeader frame{};
    frame.current_offset = currentOffset;
    frame.next_offset = currentOffset + sizeof(frame) + packageBytes;
    frame.frame_index = frameIndex_++;
    if (!writeExact(file_, reinterpret_cast<const char*>(&frame), sizeof(frame))) {
        errorMessage = QStringLiteral("LVX2 帧头写入失败：%1").arg(file_.errorString());
        return false;
    }
    for (const QByteArray& package : packages_) {
        if (!writeExact(file_, package.constData(), package.size())) {
            errorMessage = QStringLiteral("LVX2 点云数据写入失败：%1").arg(file_.errorString());
            return false;
        }
    }
    packages_.clear();
    return true;
}

bool Lvx2Writer::close(QString& errorMessage)
{
    if (!flushFrame(errorMessage)) {
        abort();
        return false;
    }
    file_.flush();
    if (file_.error() != QFile::NoError) {
        errorMessage = QStringLiteral("LVX2 文件刷新失败：%1").arg(file_.errorString());
        abort();
        return false;
    }
    file_.close();
    return true;
}

void Lvx2Writer::abort()
{
    packages_.clear();
    frameStartNs_ = 0;
    if (file_.isOpen()) {
        file_.close();
    }
    file_.remove();
}

} // namespace Lvx2

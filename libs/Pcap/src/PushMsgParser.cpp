#include "Pcap/PushMsgParser.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#endif

#include <cstring>

namespace PushMsgParser {

namespace {

// 定义设备配置结构体，方便后续扩展
struct DeviceTypeConfig {
    uint8_t typeId;
    QString modelName;
};

// 集中管理固件版本与设备型号的映射关系
DeviceTypeConfig getDeviceConfig(uint8_t firmwareMajor)
{
    switch (firmwareMajor) {
        case 13: return { 9,  QStringLiteral("Mid360") };
        case 35: return { 35, QStringLiteral("Mid360S") }; 
        case 40: return { 40, QStringLiteral("Avia2") }; 
        default: return { 0,  QStringLiteral("未知") };
    }
}

bool isValidPushSn(const QString& sn)
{
    return !sn.isEmpty() && sn != QLatin1String("DEFAULT_LIDAR");
}

// 与 LVX2 文件头/包内 lidar_id 字段一致的字节序（见 lvx2_writer 写入时的 swap）
uint32_t toLvx2LidarId(uint32_t hostOrderIp)
{
    return ((hostOrderIp & 0xFFu) << 24) |
           ((hostOrderIp & 0xFF00u) << 8) |
           ((hostOrderIp & 0xFF0000u) >> 8) |
           ((hostOrderIp & 0xFF000000u) >> 24);
}

} // namespace

uint32_t ipToLidarId(const QString& ip)
{
    uint32_t hostOrderIp = 0;
#ifdef _WIN32
    const unsigned long addr = inet_addr(ip.toLatin1().constData());
    if (addr == INADDR_NONE) {
        return 0;
    }
    hostOrderIp = ntohl(addr);
#else
    struct in_addr addr {};
    if (inet_pton(AF_INET, ip.toLatin1().constData(), &addr) != 1) {
        return 0;
    }
    hostOrderIp = ntohl(addr.s_addr);
#endif
    return toLvx2LidarId(hostOrderIp);
}

QString lidarIdToIpString(uint32_t lidarId)
{
    const uint32_t netOrder = htonl(lidarId);
    return QString("%1.%2.%3.%4")
        .arg((netOrder >> 24) & 0xFF)
        .arg((netOrder >> 16) & 0xFF)
        .arg((netOrder >> 8) & 0xFF)
        .arg(netOrder & 0xFF);
}

void parsePushPayload(const uint8_t* payload, size_t payloadLen, PushDeviceRecord& device)
{
    if (payload == nullptr || payloadLen < 32) {
        return;
    }

    size_t index = 28;
    while (index + 4 <= payloadLen) {
        const uint16_t key = uint16_t(payload[index]) | (uint16_t(payload[index + 1]) << 8);
        index += 2;
        const uint16_t length = uint16_t(payload[index]) | (uint16_t(payload[index + 1]) << 8);
        index += 2;
        if (index + length > payloadLen) {
            break;
        }

        if (key == 0x8000 && length > 0) {
            device.lidarSn = QString::fromLatin1(reinterpret_cast<const char*>(payload + index), int(length)).trimmed();
            const int nullPos = device.lidarSn.indexOf(QChar('\0'));
            if (nullPos >= 0) {
                device.lidarSn.truncate(nullPos);
            }
        } else if (key == 0x8002 && length >= 4) {
            // 解析固件版本号 app_version (aa.bb.cc.dd)，取第一个字节 aa
            const uint8_t firmwareMajor = payload[index];
            const auto config = getDeviceConfig(firmwareMajor);
            device.deviceType = config.typeId;
            device.modelDisplay = config.modelName;
        } else if (key == 0x0004 && length >= 4) {
            const QString ip = QString("%1.%2.%3.%4")
                                   .arg(payload[index + 0])
                                   .arg(payload[index + 1])
                                   .arg(payload[index + 2])
                                   .arg(payload[index + 3]);
            device.lidarId = ipToLidarId(ip);
        } else if (key == 0x0012 && length >= 24) {
            float roll = 0.0f;
            float pitch = 0.0f;
            float yaw = 0.0f;
            int32_t x = 0;
            int32_t y = 0;
            int32_t z = 0;
            std::memcpy(&roll, payload + index, 4);
            std::memcpy(&pitch, payload + index + 4, 4);
            std::memcpy(&yaw, payload + index + 8, 4);
            std::memcpy(&x, payload + index + 12, 4);
            std::memcpy(&y, payload + index + 16, 4);
            std::memcpy(&z, payload + index + 20, 4);
            device.hasExtrinsic = true;
            device.offsetRoll = roll;
            device.offsetPitch = pitch;
            device.offsetYaw = yaw;
            device.offsetX = x / 1000.0f;
            device.offsetY = y / 1000.0f;
            device.offsetZ = z / 1000.0f;
        }

        index += length;
    }
}

void mergePushPacket(const QString& srcIp,
                     const uint8_t* payload,
                     size_t payloadLen,
                     QMap<QString, PushDeviceRecord>& devicesByIp)
{
    if (srcIp.isEmpty() || payload == nullptr || payloadLen < 32) {
        return;
    }

    PushDeviceRecord parsed;
    parsed.deviceType = 0; // 默认未知
    parsePushPayload(payload, payloadLen, parsed);
    if (!isValidPushSn(parsed.lidarSn)) {
        return;
    }

    if (parsed.lidarId == 0) {
        parsed.lidarId = ipToLidarId(srcIp);
    }

    auto it = devicesByIp.find(srcIp);
    if (it == devicesByIp.end()) {
        if (parsed.modelDisplay.isEmpty()) {
            parsed.modelDisplay = QStringLiteral("未知");
        }
        devicesByIp.insert(srcIp, parsed);
        return;
    }

    PushDeviceRecord& existing = it.value();
    if (existing.lidarSn.isEmpty() || !isValidPushSn(existing.lidarSn)) {
        existing.lidarSn = parsed.lidarSn;
    }
    if (parsed.lidarId != 0) {
        existing.lidarId = parsed.lidarId;
    }
    if (parsed.deviceType != 0) {
        existing.deviceType = parsed.deviceType;
        existing.modelDisplay = parsed.modelDisplay;
    }
    if (parsed.hasExtrinsic) {
        existing.hasExtrinsic = true;
        existing.offsetRoll = parsed.offsetRoll;
        existing.offsetPitch = parsed.offsetPitch;
        existing.offsetYaw = parsed.offsetYaw;
        existing.offsetX = parsed.offsetX;
        existing.offsetY = parsed.offsetY;
        existing.offsetZ = parsed.offsetZ;
    }
}

QVector<PushDeviceRecord> finalizeDevices(const QMap<QString, PushDeviceRecord>& pushDevicesByIp,
                                          const QMap<QString, uint32_t>& pointCloudSourceIps)
{
    QVector<PushDeviceRecord> devices;
    QMap<uint32_t, bool> seenIds;

    for (auto it = pushDevicesByIp.constBegin(); it != pushDevicesByIp.constEnd(); ++it) {
        PushDeviceRecord record = it.value();
        if (record.lidarId == 0) {
            record.lidarId = ipToLidarId(it.key());
        }
        if (record.modelDisplay.isEmpty()) {
            record.modelDisplay = QStringLiteral("未知");
        }
        if (seenIds.contains(record.lidarId)) {
            continue;
        }
        seenIds.insert(record.lidarId, true);
        devices.push_back(record);
    }

    if (!pushDevicesByIp.isEmpty()) {
        return devices;
    }

    for (auto it = pointCloudSourceIps.constBegin(); it != pointCloudSourceIps.constEnd(); ++it) {
        const uint32_t lidarId = it.value() != 0 ? it.value() : ipToLidarId(it.key());
        if (lidarId == 0 || seenIds.contains(lidarId)) {
            continue;
        }
        seenIds.insert(lidarId, true);

        PushDeviceRecord fallback;
        fallback.lidarId = lidarId;
        fallback.deviceType = 0; // 点云兜底数据无法感知型号，设为未知
        fallback.lidarSn = QStringLiteral("未知");
        fallback.modelDisplay = QStringLiteral("未知");
        devices.push_back(fallback);
    }

    return devices;
}

} // namespace PushMsgParser
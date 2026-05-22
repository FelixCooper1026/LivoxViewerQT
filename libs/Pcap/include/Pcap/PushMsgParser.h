#ifndef PCAP_PUSHMSGPARSER_H
#define PCAP_PUSHMSGPARSER_H

#include <QMap>
#include <QString>
#include <QVector>
#include <cstdint>

namespace PushMsgParser {

struct PushDeviceRecord {
    uint32_t lidarId = 0;
    uint8_t deviceType = 0;
    QString lidarSn;
    QString modelDisplay;
    bool hasExtrinsic = false;
    float offsetRoll = 0.0f;
    float offsetPitch = 0.0f;
    float offsetYaw = 0.0f;
    float offsetX = 0.0f;
    float offsetY = 0.0f;
    float offsetZ = 0.0f;
};

void parsePushPayload(const uint8_t* payload, size_t payloadLen, PushDeviceRecord& device);

uint32_t ipToLidarId(const QString& ip);

QString lidarIdToIpString(uint32_t lidarId);

void mergePushPacket(const QString& srcIp, const uint8_t* payload, size_t payloadLen, QMap<QString, PushDeviceRecord>& devicesByIp);

QVector<PushDeviceRecord> finalizeDevices(const QMap<QString, PushDeviceRecord>& pushDevicesByIp,
                                          const QMap<QString, uint32_t>& pointCloudSourceIps);

} // namespace PushMsgParser

#endif // PCAP_PUSHMSGPARSER_H

#include "PcapParser.h"

#include "LidarModelUtils.h"
#include "ImuParser.h"
#include "PcapUdpPacket.h"
#include "PointParser.h"
#include "PushMsgParser.h"

#include <pcap.h>

#include <QMap>

namespace {

struct PcapMetadata {
    QMap<QString, PushMsgParser::PushDeviceRecord> pushDevicesByIp;
    QMap<QString, uint32_t> dataSourceIps;
    uint64_t totalPacketsScanned = 0;
    int datalinkType = 0;
};

pcap_t* openOfflinePcap(const QString& filePath, char* errbuf)
{
#ifdef _WIN32
    return pcap_open_offline(filePath.toLocal8Bit().constData(), errbuf);
#else
    return pcap_open_offline(filePath.toStdString().c_str(), errbuf);
#endif
}

void scanMetadata(pcap_t* handle, PcapMetadata& metadata)
{
    metadata.datalinkType = pcap_datalink(handle);

    struct pcap_pkthdr* header = nullptr;
    const u_char* packetData = nullptr;
    int res = 0;

    while ((res = pcap_next_ex(handle, &header, &packetData)) >= 0) {
        if (res == 0 || header == nullptr) {
            continue;
        }
        metadata.totalPacketsScanned++;

        PcapUdp::PacketInfo udpInfo;
        if (!PcapUdp::tryExtractUdp(packetData, header->caplen, udpInfo)) {
            continue;
        }

        const bool isPushPort = udpInfo.srcPort == PcapUdp::kLivoxPushDataPort;
        const bool isPointCloudPort = udpInfo.srcPort == PcapUdp::kLivoxPointCloudPort;
        const bool isImuPort = udpInfo.srcPort == PcapUdp::kLivoxIMUPort;

        if (isPushPort && udpInfo.payload != nullptr && udpInfo.payloadLen > 0) {
            PushMsgParser::mergePushPacket(udpInfo.srcIp,
                                           udpInfo.payload,
                                           udpInfo.payloadLen,
                                           metadata.pushDevicesByIp);
            continue;
        }

        if ((!isPointCloudPort && !isImuPort) || udpInfo.payload == nullptr) {
            continue;
        }

        const bool isValidDataPayload =
            isPointCloudPort ? PointParser::isLivoxPointCloudPayload(udpInfo.payload, udpInfo.payloadLen)
                             : ImuParser::isLivoxImuPayload(udpInfo.payload, udpInfo.payloadLen);
        if (!isValidDataPayload) {
            continue;
        }

        if (!udpInfo.srcIp.isEmpty()) {
            const uint32_t lidarId = PushMsgParser::ipToLidarId(udpInfo.srcIp);
            metadata.dataSourceIps.insert(udpInfo.srcIp, lidarId);
        }
    }
}

QMap<uint32_t, int> lineCountsByLidarId(const QVector<PushMsgParser::PushDeviceRecord>& devices)
{
    QMap<uint32_t, int> lineCounts;
    for (const PushMsgParser::PushDeviceRecord& device : devices) {
        lineCounts.insert(device.lidarId, LivoxCore::lineCountForDeviceType(device.deviceType));
    }
    return lineCounts;
}

void parseDataFramesAndImuSamples(pcap_t* handle,
                                  const QMap<uint32_t, int>& lineCounts,
                                  QVector<PointCloudFrame>& frames,
                                  QVector<ImuParser::Sample>& imuSamples)
{
    PointParser::FrameBuilder frameBuilder;

    struct pcap_pkthdr* header = nullptr;
    const u_char* packetData = nullptr;
    int res = 0;

    while ((res = pcap_next_ex(handle, &header, &packetData)) >= 0) {
        if (res == 0 || header == nullptr) {
            continue;
        }

        PcapUdp::PacketInfo udpInfo;
        if (!PcapUdp::tryExtractUdp(packetData, header->caplen, udpInfo)) {
            continue;
        }

        const bool isPointCloudPort = udpInfo.srcPort == PcapUdp::kLivoxPointCloudPort;
        const bool isImuPort = udpInfo.srcPort == PcapUdp::kLivoxIMUPort;
        if ((!isPointCloudPort && !isImuPort) || udpInfo.payload == nullptr) {
            continue;
        }

        const uint64_t packetTime = PointParser::pcapTimestampNs(header);
        const uint32_t lidarId =
            udpInfo.srcIp.isEmpty() ? 0u : PushMsgParser::ipToLidarId(udpInfo.srcIp);

        if (isImuPort) {
            ImuParser::appendImuPayload(udpInfo.payload, udpInfo.payloadLen, lidarId, packetTime, imuSamples);
            continue;
        }

        if (!PointParser::isLivoxPointCloudPayload(udpInfo.payload, udpInfo.payloadLen)) {
            continue;
        }

        if (frameBuilder.frameStartTime == 0) {
            frameBuilder.reset(packetTime);
        } else if (packetTime - frameBuilder.frameStartTime >= PointParser::FrameBuilder::kFrameDurationNs) {
            frameBuilder.flush(frames);
            frameBuilder.reset(packetTime);
        }

        PointParser::appendPointCloudPayload(udpInfo.payload,
                                             udpInfo.payloadLen,
                                             lidarId,
                                             lineCounts.value(lidarId, 1),
                                             frameBuilder);
    }

    frameBuilder.flush(frames);
}

} // namespace

namespace PcapParser {

ParseResult parseFileToFrames(const QString& filePath)
{
    ParseResult result;
    char errbuf[PCAP_ERRBUF_SIZE] = {};

    pcap_t* metadataHandle = openOfflinePcap(filePath, errbuf);
    if (metadataHandle == nullptr) {
        result.errorMessage = QString("无法打开抓包文件:\n%1").arg(errbuf);
        return result;
    }

    PcapMetadata metadata;
    scanMetadata(metadataHandle, metadata);
    pcap_close(metadataHandle);

    result.datalinkType = metadata.datalinkType;
    result.totalPacketsScanned = metadata.totalPacketsScanned;
    result.devices = PushMsgParser::finalizeDevices(metadata.pushDevicesByIp, metadata.dataSourceIps);

    char frameErrbuf[PCAP_ERRBUF_SIZE] = {};
    pcap_t* frameHandle = openOfflinePcap(filePath, frameErrbuf);
    if (frameHandle == nullptr) {
        result.errorMessage = QString("无法打开抓包文件:\n%1").arg(frameErrbuf);
        return result;
    }

    parseDataFramesAndImuSamples(frameHandle, lineCountsByLidarId(result.devices), result.frames, result.imuSamples);
    pcap_close(frameHandle);

    if (result.frames.isEmpty()) {
        result.errorMessage =
            QString("文件已解析，但未匹配到有效的点云数据包（源端口 %1）。\n\n"
                    "统计信息:\n"
                    "- 扫描数据包总数: %2\n"
                    "- 链路层类型(DLT): %3")
                .arg(PcapUdp::kLivoxPointCloudPort)
                .arg(result.totalPacketsScanned)
                .arg(result.datalinkType);
        return result;
    }

    result.ok = true;
    return result;
}

} // namespace PcapParser

#include "Pcap/PcapParser.h"
#include "Pcap/PcapUdpPacket.h"
#include "Pcap/PointParser.h"
#include "Pcap/PushMsgParser.h"

#include <pcap.h>

#include <QMap>

namespace PcapParser {

ParseResult parseFileToFrames(const QString& filePath)
{
    ParseResult result;
    char errbuf[PCAP_ERRBUF_SIZE] = {};

#ifdef _WIN32
    pcap_t* handle = pcap_open_offline(filePath.toLocal8Bit().constData(), errbuf);
#else
    pcap_t* handle = pcap_open_offline(filePath.toStdString().c_str(), errbuf);
#endif

    if (handle == nullptr) {
        result.errorMessage = QString("无法打开抓包文件:\n%1").arg(errbuf);
        return result;
    }

    result.datalinkType = pcap_datalink(handle);

    QMap<QString, PushMsgParser::PushDeviceRecord> pushDevicesByIp;
    QMap<QString, uint32_t> pointCloudSourceIps;
    PointParser::FrameBuilder frameBuilder;

    struct pcap_pkthdr* header = nullptr;
    const u_char* packetData = nullptr;
    int res = 0;

    while ((res = pcap_next_ex(handle, &header, &packetData)) >= 0) {
        if (res == 0 || header == nullptr) {
            continue;
        }
        result.totalPacketsScanned++;

        PcapUdp::PacketInfo udpInfo;
        if (!PcapUdp::tryExtractUdp(packetData, header->caplen, udpInfo)) {
            continue;
        }

        const bool isPushPort = udpInfo.srcPort == PcapUdp::kLivoxPushDataPort;
        const bool isPointCloudPort = udpInfo.srcPort == PcapUdp::kLivoxPointCloudPort;

        if (isPushPort && udpInfo.payload != nullptr && udpInfo.payloadLen > 0) {
            PushMsgParser::mergePushPacket(udpInfo.srcIp,
                                         udpInfo.payload,
                                         udpInfo.payloadLen,
                                         pushDevicesByIp);
            continue;
        }

        if (!isPointCloudPort || udpInfo.payload == nullptr || udpInfo.payloadLen < 36) {
            continue;
        }

        if (!PointParser::isLivoxPointCloudPayload(udpInfo.payload, udpInfo.payloadLen)) {
            continue;
        }

        if (!udpInfo.srcIp.isEmpty()) {
            const uint32_t lidarId = PushMsgParser::ipToLidarId(udpInfo.srcIp);
            pointCloudSourceIps.insert(udpInfo.srcIp, lidarId);
        }

        const uint64_t packetTime = PointParser::pcapTimestampNs(header);
        if (frameBuilder.frameStartTime == 0) {
            frameBuilder.reset(packetTime);
        } else if (packetTime - frameBuilder.frameStartTime >= PointParser::FrameBuilder::kFrameDurationNs) {
            frameBuilder.flush(result.frames);
            frameBuilder.reset(packetTime);
        }

        const uint32_t lidarId =
            udpInfo.srcIp.isEmpty() ? 0u : PushMsgParser::ipToLidarId(udpInfo.srcIp);
        PointParser::appendPointCloudPayload(udpInfo.payload, udpInfo.payloadLen, lidarId, frameBuilder);
    }

    frameBuilder.flush(result.frames);
    pcap_close(handle);

    result.devices = PushMsgParser::finalizeDevices(pushDevicesByIp, pointCloudSourceIps);

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

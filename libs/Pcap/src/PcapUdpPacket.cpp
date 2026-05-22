#include "Pcap/PcapUdpPacket.h"

namespace PcapUdp {

bool tryExtractUdp(const u_char* packetData, size_t caplen, PacketInfo& out)
{
    out = PacketInfo{};
    if (packetData == nullptr || caplen < 28) {
        return false;
    }

    const size_t candidateOffsets[] = {14, 16, 20, 18, 4, 0};
    for (size_t offset : candidateOffsets) {
        if (caplen < offset + 28) {
            continue;
        }
        if ((packetData[offset] & 0xF0) != 0x40) {
            continue;
        }

        const uint8_t ihl = (packetData[offset] & 0x0F) * 4;
        if (ihl < 20 || caplen < offset + ihl + 8) {
            continue;
        }
        if (packetData[offset + 9] != 17) {
            continue;
        }

        const size_t ipStart = offset;
        const size_t udpStart = offset + ihl;
        if (caplen < udpStart + 8) {
            continue;
        }

        out.srcPort = uint16_t(packetData[udpStart + 0] << 8) | packetData[udpStart + 1];
        out.dstPort = uint16_t(packetData[udpStart + 2] << 8) | packetData[udpStart + 3];

        const size_t payloadStart = udpStart + 8;
        if (caplen <= payloadStart) {
            continue;
        }

        const size_t srcIpOffset = ipStart + 12;
        const size_t dstIpOffset = ipStart + 16;
        if (caplen < dstIpOffset + 4) {
            continue;
        }

        out.srcIp = QString("%1.%2.%3.%4")
                        .arg(packetData[srcIpOffset + 0])
                        .arg(packetData[srcIpOffset + 1])
                        .arg(packetData[srcIpOffset + 2])
                        .arg(packetData[srcIpOffset + 3]);
        out.dstIp = QString("%1.%2.%3.%4")
                        .arg(packetData[dstIpOffset + 0])
                        .arg(packetData[dstIpOffset + 1])
                        .arg(packetData[dstIpOffset + 2])
                        .arg(packetData[dstIpOffset + 3]);
        out.payload = packetData + payloadStart;
        out.payloadLen = caplen - payloadStart;
        out.valid = true;
        return true;
    }

    return false;
}

} // namespace PcapUdp

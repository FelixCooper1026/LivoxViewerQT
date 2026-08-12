#include "PcapUdpPacket.h"

#include <pcap.h>

namespace PcapUdp {

namespace {

uint16_t readBe16(const u_char* data)
{
    return uint16_t(data[0] << 8) | uint16_t(data[1]);
}

bool ipv4Offset(const u_char* packetData, size_t caplen, int datalinkType, size_t& offset)
{
    if (datalinkType == DLT_RAW) {
        offset = 0;
        return true;
    }
    if (datalinkType == DLT_LINUX_SLL) {
        if (caplen < 16 || readBe16(packetData + 14) != 0x0800) {
            return false;
        }
        offset = 16;
        return true;
    }
#ifdef DLT_LINUX_SLL2
    if (datalinkType == DLT_LINUX_SLL2) {
        if (caplen < 20 || readBe16(packetData) != 0x0800) {
            return false;
        }
        offset = 20;
        return true;
    }
#endif
    if (datalinkType != DLT_EN10MB || caplen < 14) {
        return false;
    }

    offset = 14;
    uint16_t etherType = readBe16(packetData + 12);
    int vlanCount = 0;
    while ((etherType == 0x8100 || etherType == 0x88A8) && vlanCount < 2) {
        if (caplen < offset + 4) {
            return false;
        }
        etherType = readBe16(packetData + offset + 2);
        offset += 4;
        ++vlanCount;
    }
    return etherType == 0x0800;
}

} // namespace

bool tryExtractUdp(const u_char* packetData, size_t caplen, int datalinkType, PacketInfo& out)
{
    out = PacketInfo{};
    if (packetData == nullptr || caplen < 28) {
        return false;
    }

    size_t ipStart = 0;
    if (!ipv4Offset(packetData, caplen, datalinkType, ipStart) || caplen < ipStart + 28) {
        return false;
    }
    if ((packetData[ipStart] >> 4) != 4) {
        return false;
    }
    const size_t ihl = size_t(packetData[ipStart] & 0x0F) * 4;
    if (ihl < 20 || caplen < ipStart + ihl + 8 || packetData[ipStart + 9] != 17) {
        return false;
    }
    const uint16_t fragment = readBe16(packetData + ipStart + 6);
    if ((fragment & 0x3FFF) != 0) {
        return false;
    }
    const size_t ipLength = readBe16(packetData + ipStart + 2);
    if (ipLength < ihl + 8 || caplen < ipStart + ipLength) {
        return false;
    }

    const size_t udpStart = ipStart + ihl;
    const size_t udpLength = readBe16(packetData + udpStart + 4);
    if (udpLength < 8 || udpLength > ipLength - ihl || caplen < udpStart + udpLength) {
        return false;
    }

    out.srcPort = readBe16(packetData + udpStart);
    out.dstPort = readBe16(packetData + udpStart + 2);
    out.srcIp = QStringLiteral("%1.%2.%3.%4")
                    .arg(packetData[ipStart + 12]).arg(packetData[ipStart + 13])
                    .arg(packetData[ipStart + 14]).arg(packetData[ipStart + 15]);
    out.dstIp = QStringLiteral("%1.%2.%3.%4")
                    .arg(packetData[ipStart + 16]).arg(packetData[ipStart + 17])
                    .arg(packetData[ipStart + 18]).arg(packetData[ipStart + 19]);
    out.payload = packetData + udpStart + 8;
    out.payloadLen = udpLength - 8;
    out.valid = true;
    return true;
}

bool tryExtractUdp(const u_char* packetData, size_t caplen, PacketInfo& out)
{
    return tryExtractUdp(packetData, caplen, DLT_EN10MB, out);
}

} // namespace PcapUdp

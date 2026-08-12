#include "PcapUdpPacket.h"

#include <pcap.h>

#include <QByteArray>

#include <algorithm>
#include <iostream>

namespace {

void writeBe16(QByteArray& bytes, int offset, uint16_t value)
{
    bytes[offset] = char(value >> 8);
    bytes[offset + 1] = char(value & 0xFF);
}

QByteArray ethernetPacket(bool vlan, int padding)
{
    const int ipOffset = vlan ? 18 : 14;
    const QByteArray payload("livox", 5);
    QByteArray packet(ipOffset + 20 + 8 + payload.size() + padding, '\0');
    writeBe16(packet, 12, vlan ? 0x8100 : 0x0800);
    if (vlan) {
        writeBe16(packet, 14, 1);
        writeBe16(packet, 16, 0x0800);
    }
    packet[ipOffset] = 0x45;
    writeBe16(packet, ipOffset + 2, uint16_t(20 + 8 + payload.size()));
    packet[ipOffset + 8] = 64;
    packet[ipOffset + 9] = 17;
    packet[ipOffset + 12] = 192;
    packet[ipOffset + 13] = char(168);
    packet[ipOffset + 14] = 1;
    packet[ipOffset + 15] = 10;
    packet[ipOffset + 16] = 192;
    packet[ipOffset + 17] = char(168);
    packet[ipOffset + 18] = 1;
    packet[ipOffset + 19] = 20;
    const int udpOffset = ipOffset + 20;
    writeBe16(packet, udpOffset, 56300);
    writeBe16(packet, udpOffset + 2, 60000);
    writeBe16(packet, udpOffset + 4, uint16_t(8 + payload.size()));
    std::copy(payload.cbegin(), payload.cend(), packet.begin() + udpOffset + 8);
    return packet;
}

bool expect(bool condition, const char* message)
{
    if (!condition) std::cerr << message << '\n';
    return condition;
}

} // namespace

int main()
{
    bool ok = true;
    PcapUdp::PacketInfo info;
    const QByteArray plain = ethernetPacket(false, 12);
    ok &= expect(PcapUdp::tryExtractUdp(reinterpret_cast<const u_char*>(plain.constData()), plain.size(), DLT_EN10MB, info),
                 "Ethernet UDP should parse");
    ok &= expect(info.payloadLen == 5, "Ethernet padding must not be included in UDP payload");
    ok &= expect(info.srcPort == 56300 && info.srcIp == QStringLiteral("192.168.1.10"), "UDP fields mismatch");

    const QByteArray vlan = ethernetPacket(true, 0);
    ok &= expect(PcapUdp::tryExtractUdp(reinterpret_cast<const u_char*>(vlan.constData()), vlan.size(), DLT_EN10MB, info),
                 "VLAN UDP should parse");
    ok &= expect(info.payloadLen == 5, "VLAN payload length mismatch");

    const QByteArray truncated = plain.left(plain.size() - 13);
    ok &= expect(!PcapUdp::tryExtractUdp(reinterpret_cast<const u_char*>(truncated.constData()), truncated.size(), DLT_EN10MB, info),
                 "Truncated UDP must be rejected");
    ok &= expect(!PcapUdp::tryExtractUdp(reinterpret_cast<const u_char*>(plain.constData()), plain.size(), DLT_NULL, info),
                 "Unsupported DLT must be rejected");
    return ok ? 0 : 1;
}

#ifndef PCAP_PCAPUDPPACKET_H
#define PCAP_PCAPUDPPACKET_H

#include <QString>
#include <cstddef>
#include <cstdint>

struct pcap_pkthdr;
typedef unsigned char u_char;

namespace PcapUdp {

constexpr uint16_t kLivoxPointCloudPort = 56300;
constexpr uint16_t kLivoxPushDataPort = 56200;

struct PacketInfo {
    bool valid = false;
    QString srcIp;
    QString dstIp;
    uint16_t srcPort = 0;
    uint16_t dstPort = 0;
    const u_char* payload = nullptr;
    size_t payloadLen = 0;
};

bool tryExtractUdp(const u_char* packetData, size_t caplen, PacketInfo& out);

} // namespace PcapUdp

#endif // PCAP_PCAPUDPPACKET_H

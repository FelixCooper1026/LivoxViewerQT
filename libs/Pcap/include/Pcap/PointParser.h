#ifndef PCAP_POINTPARSER_H
#define PCAP_POINTPARSER_H

#include "PointCloud/PointCloudFrame.h"

#include <QVector>
#include <cstdint>

struct pcap_pkthdr;
typedef unsigned char u_char;

namespace PointParser {

struct FrameBuilder {
    PointCloudFrame currentFrame;
    uint64_t frameStartTime = 0;
    static constexpr uint64_t kFrameDurationNs = 50000000ULL;

    void reset(uint64_t packetTime);
    void flush(QVector<PointCloudFrame>& outFrames);
};

bool isLivoxPointCloudPayload(const uint8_t* payload, size_t payloadLen);

void appendPointCloudPayload(const uint8_t* payload,
                             size_t payloadLen,
                             uint32_t lidarId,
                             int lineCount,
                             FrameBuilder& builder);

uint64_t pcapTimestampNs(const pcap_pkthdr* header);

} // namespace PointParser

#endif // PCAP_POINTPARSER_H

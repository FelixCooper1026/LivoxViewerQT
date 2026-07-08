#ifndef PCAP_IMUPARSER_H
#define PCAP_IMUPARSER_H

#include <QVector>

#include <cstddef>
#include <cstdint>

namespace ImuParser {

struct Sample {
    uint32_t lidarId = 0;
    uint64_t timestampNs = 0;
    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;
    float accX = 0.0f;
    float accY = 0.0f;
    float accZ = 0.0f;
};

bool isLivoxImuPayload(const uint8_t* payload, size_t payloadLen);

void appendImuPayload(const uint8_t* payload,
                      size_t payloadLen,
                      uint32_t lidarId,
                      uint64_t timestampNs,
                      QVector<Sample>& samples);

} // namespace ImuParser

#endif // PCAP_IMUPARSER_H

#include "ImuParser.h"

namespace ImuParser {

namespace {

constexpr size_t kLivoxPayloadHeaderSize = 36;
constexpr size_t kLivoxImuSampleSize = 24;

} // namespace

bool isLivoxImuPayload(const uint8_t* payload, size_t payloadLen)
{
    if (payload == nullptr || payloadLen < kLivoxPayloadHeaderSize) {
        return false;
    }
    if (payload[0] != 0 && payload[0] != 1) {
        return false;
    }
    if (payload[10] != 0) {
        return false;
    }

    const uint16_t dotNum = *reinterpret_cast<const uint16_t*>(payload + 5);
    return payloadLen - kLivoxPayloadHeaderSize >= size_t(dotNum) * kLivoxImuSampleSize;
}

void appendImuPayload(const uint8_t* payload,
                      size_t payloadLen,
                      uint32_t lidarId,
                      uint64_t timestampNs,
                      QVector<Sample>& samples)
{
    if (!isLivoxImuPayload(payload, payloadLen)) {
        return;
    }

    const uint16_t dotNum = *reinterpret_cast<const uint16_t*>(payload + 5);
    const uint8_t* dataZone = payload + kLivoxPayloadHeaderSize;
    for (uint16_t i = 0; i < dotNum; ++i) {
        const uint8_t* p = dataZone + size_t(i) * kLivoxImuSampleSize;
        Sample sample;
        sample.lidarId = lidarId;
        sample.timestampNs = timestampNs;
        sample.gyroX = *reinterpret_cast<const float*>(p + 0);
        sample.gyroY = *reinterpret_cast<const float*>(p + 4);
        sample.gyroZ = *reinterpret_cast<const float*>(p + 8);
        sample.accX = *reinterpret_cast<const float*>(p + 12);
        sample.accY = *reinterpret_cast<const float*>(p + 16);
        sample.accZ = *reinterpret_cast<const float*>(p + 20);
        samples.push_back(sample);
    }
}

} // namespace ImuParser

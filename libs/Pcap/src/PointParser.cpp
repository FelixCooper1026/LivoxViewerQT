#include "PointParser.h"

#include "LidarModelUtils.h"

#include <pcap.h>

#include <cmath>
#include <cstring>

namespace PointParser {

namespace {

constexpr double kLivoxPi = 3.14159265358979323846;

static void appendCartesianHigh(const uint8_t* dataZone,
                                uint16_t dotNum,
                                int lineCount,
                                QVector<PointCloudPoint>& points)
{
    constexpr size_t stride = 14;
    for (uint16_t i = 0; i < dotNum; ++i) {
        const uint8_t* p = dataZone + (i * stride);
        PointCloudPoint point{};
        point.x = static_cast<float>(*reinterpret_cast<const int32_t*>(p + 0)) / 1000.0f;
        point.y = static_cast<float>(*reinterpret_cast<const int32_t*>(p + 4)) / 1000.0f;
        point.z = static_cast<float>(*reinterpret_cast<const int32_t*>(p + 8)) / 1000.0f;
        point.reflectivity = p[12];
        point.tag = p[13];
        point.line = LivoxCore::lineForPointIndex(int(i), lineCount);
        points.push_back(point);
    }
}

static void appendCartesianLow(const uint8_t* dataZone,
                               uint16_t dotNum,
                               int lineCount,
                               QVector<PointCloudPoint>& points)
{
    constexpr size_t stride = 8;
    for (uint16_t i = 0; i < dotNum; ++i) {
        const uint8_t* p = dataZone + (i * stride);
        PointCloudPoint point{};
        point.x = static_cast<float>(*reinterpret_cast<const int16_t*>(p + 0)) / 100.0f;
        point.y = static_cast<float>(*reinterpret_cast<const int16_t*>(p + 2)) / 100.0f;
        point.z = static_cast<float>(*reinterpret_cast<const int16_t*>(p + 4)) / 100.0f;
        point.reflectivity = p[6];
        point.tag = p[7];
        point.line = LivoxCore::lineForPointIndex(int(i), lineCount);
        points.push_back(point);
    }
}

static void appendSpherical(const uint8_t* dataZone,
                            uint16_t dotNum,
                            int lineCount,
                            QVector<PointCloudPoint>& points)
{
    constexpr size_t stride = 10;
    for (uint16_t i = 0; i < dotNum; ++i) {
        const uint8_t* p = dataZone + (i * stride);
        const float depth = static_cast<float>(*reinterpret_cast<const uint32_t*>(p + 0)) / 1000.0f;
        const float zenithRad =
            static_cast<float>(*reinterpret_cast<const uint16_t*>(p + 4)) * 0.01f * float(kLivoxPi) / 180.0f;
        const float azimuthRad =
            static_cast<float>(*reinterpret_cast<const uint16_t*>(p + 6)) * 0.01f * float(kLivoxPi) / 180.0f;

        PointCloudPoint point{};
        point.x = depth * std::sin(zenithRad) * std::cos(azimuthRad);
        point.y = depth * std::sin(zenithRad) * std::sin(azimuthRad);
        point.z = depth * std::cos(zenithRad);
        point.reflectivity = p[8];
        point.tag = p[9];
        point.line = LivoxCore::lineForPointIndex(int(i), lineCount);
        points.push_back(point);
    }
}

static void appendDoubleEcho(const uint8_t* dataZone,
                             uint16_t dotNum,
                             int lineCount,
                             QVector<PointCloudPoint>& points)
{
    constexpr size_t stride = 28;
    for (uint16_t i = 0; i < dotNum; ++i) {
        const uint8_t* p = dataZone + (i * stride);

        PointCloudPoint pt1{};
        pt1.x = static_cast<float>(*reinterpret_cast<const int32_t*>(p + 0)) / 1000.0f;
        pt1.y = static_cast<float>(*reinterpret_cast<const int32_t*>(p + 4)) / 1000.0f;
        pt1.z = static_cast<float>(*reinterpret_cast<const int32_t*>(p + 8)) / 1000.0f;
        pt1.reflectivity = p[12];
        pt1.tag = p[13];
        pt1.line = LivoxCore::lineForPointIndex(int(i) * 2, lineCount);
        points.push_back(pt1);

        PointCloudPoint pt2{};
        pt2.x = static_cast<float>(*reinterpret_cast<const int32_t*>(p + 14)) / 1000.0f;
        pt2.y = static_cast<float>(*reinterpret_cast<const int32_t*>(p + 18)) / 1000.0f;
        pt2.z = static_cast<float>(*reinterpret_cast<const int32_t*>(p + 22)) / 1000.0f;
        pt2.reflectivity = p[26];
        pt2.tag = p[27];
        pt2.line = LivoxCore::lineForPointIndex(int(i) * 2 + 1, lineCount);
        points.push_back(pt2);
    }
}

} // namespace

void FrameBuilder::reset(uint64_t packetTime)
{
    currentFrame = PointCloudFrame();
    frameStartTime = packetTime;
    currentFrame.device_handle = 0;
    currentFrame.timestamp = packetTime;
}

void FrameBuilder::flush(QVector<PointCloudFrame>& outFrames)
{
    if (!currentFrame.pointsByLidar.isEmpty()) {
        outFrames.push_back(std::move(currentFrame));
        currentFrame = PointCloudFrame();
        frameStartTime = 0;
    }
}

bool isLivoxPointCloudPayload(const uint8_t* payload, size_t payloadLen)
{
    if (payload == nullptr || payloadLen < 36) {
        return false;
    }
    if (payload[0] != 0 && payload[0] != 1) {
        return false;
    }
    const uint8_t dataType = payload[10];
    return (dataType >= 1 && dataType <= 3) || dataType == 17;
}

void appendPointCloudPayload(const uint8_t* payload,
                             size_t payloadLen,
                             uint32_t lidarId,
                             int lineCount,
                             FrameBuilder& builder)
{
    if (!isLivoxPointCloudPayload(payload, payloadLen)) {
        return;
    }

    const uint16_t dotNum = *reinterpret_cast<const uint16_t*>(payload + 5);
    const uint8_t dataType = payload[10];
    const uint8_t* dataZone = payload + 36;
    const size_t availableDataBytes = payloadLen - 36;
    QVector<PointCloudPoint>& bucket = builder.currentFrame.pointsByLidar[lidarId];
    if (bucket.isEmpty()) {
        bucket.reserve(20000);
    }

    if (dataType == 1) {
        constexpr size_t stride = 14;
        if (availableDataBytes < dotNum * stride) {
            return;
        }
        appendCartesianHigh(dataZone, dotNum, lineCount, bucket);
    } else if (dataType == 2) {
        constexpr size_t stride = 8;
        if (availableDataBytes < dotNum * stride) {
            return;
        }
        appendCartesianLow(dataZone, dotNum, lineCount, bucket);
    } else if (dataType == 3) {
        constexpr size_t stride = 10;
        if (availableDataBytes < dotNum * stride) {
            return;
        }
        appendSpherical(dataZone, dotNum, lineCount, bucket);
    } else if (dataType == 17) {
        constexpr size_t stride = 28;
        if (availableDataBytes < dotNum * stride) {
            return;
        }
        appendDoubleEcho(dataZone, dotNum, lineCount, bucket);
    }
}

uint64_t pcapTimestampNs(const pcap_pkthdr* header)
{
    return uint64_t(header->ts.tv_sec) * 1000000000ULL + uint64_t(header->ts.tv_usec) * 1000ULL;
}

} // namespace PointParser

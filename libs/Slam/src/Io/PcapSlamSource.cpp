#include "Slam/Io/PcapSlamSource.h"

#include "LivoxCore/LidarModelUtils.h"
#include "LivoxCore/LidarPacketUtils.h"
#include "Pcap/ImuParser.h"
#include "Pcap/PcapUdpPacket.h"
#include "Pcap/PointParser.h"
#include "Pcap/PushMsgParser.h"
#include "livox_lidar_def.h"

#include <pcap.h>

#include <QMap>
#include <QSet>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace {

constexpr size_t kLivoxPayloadHeaderSize = 36;
constexpr int64_t kLivoxTimeIntervalUnitNs = 100;
constexpr int64_t kNsPerMs = 1000000;
constexpr double kLivoxPi = 3.14159265358979323846;

struct PcapMetadata {
    QMap<QString, PushMsgParser::PushDeviceRecord> pushDevicesByIp;
    QMap<QString, uint32_t> dataSourceIps;
    QVector<PushMsgParser::PushDeviceRecord> devices;
};

struct LivoxPayloadHeader {
    uint16_t timeIntervalRaw = 0;
    uint16_t dotNum = 0;
    uint8_t dataType = 0;
    uint8_t timeType = 0;
    int64_t timestampNs = 0;
};

pcap_t* openOfflinePcap(const QString& filePath, char* errbuf)
{
#ifdef _WIN32
    return pcap_open_offline(filePath.toLocal8Bit().constData(), errbuf);
#else
    return pcap_open_offline(filePath.toStdString().c_str(), errbuf);
#endif
}

uint16_t readLe16(const uint8_t* data)
{
    return uint16_t(data[0]) | (uint16_t(data[1]) << 8);
}

int16_t readLeInt16(const uint8_t* data)
{
    const uint16_t value = readLe16(data);
    return static_cast<int16_t>(value);
}

uint32_t readLe32(const uint8_t* data)
{
    return uint32_t(data[0]) | (uint32_t(data[1]) << 8) | (uint32_t(data[2]) << 16) | (uint32_t(data[3]) << 24);
}

int32_t readLeInt32(const uint8_t* data)
{
    const uint32_t value = readLe32(data);
    return static_cast<int32_t>(value);
}

float readFloat(const uint8_t* data)
{
    float value = 0.0f;
    std::memcpy(&value, data, sizeof(float));
    return value;
}

bool parseLivoxPayloadHeader(const uint8_t* payload, size_t payloadLen, LivoxPayloadHeader& header)
{
    if (payload == nullptr || payloadLen < kLivoxPayloadHeaderSize) {
        return false;
    }

    header.timeIntervalRaw = readLe16(payload + 3);
    header.dotNum = readLe16(payload + 5);
    header.dataType = payload[10];
    header.timeType = payload[11];
    header.timestampNs = static_cast<int64_t>(LivoxCore::parseLivoxTimestamp(payload + 28));
    return header.dotNum > 0;
}

bool hasPointOffsetTime(const LivoxPayloadHeader& header)
{
    return header.dotNum <= 1 || header.timeIntervalRaw != 0;
}

int64_t sampleIntervalNs(const LivoxPayloadHeader& header)
{
    if (header.dotNum == 0) {
        return 0;
    }
    return int64_t(header.timeIntervalRaw) * kLivoxTimeIntervalUnitNs / int64_t(header.dotNum);
}

int64_t sampleOffsetNs(const LivoxPayloadHeader& header, uint16_t sampleIndex)
{
    return int64_t(sampleIndex) * sampleIntervalNs(header);
}

void addSlamPoint(SlamInputFrame& frame,
                  const SlamPoint& point,
                  int64_t absoluteTimestampNs,
                  bool hasOffsetTime)
{
    SlamPoint slamPoint = point;
    slamPoint.offsetNs = absoluteTimestampNs - frame.frameStartNs;
    slamPoint.hasOffsetTime = hasOffsetTime;
    frame.points.push_back(slamPoint);
    if (absoluteTimestampNs > frame.frameEndNs) {
        frame.frameEndNs = absoluteTimestampNs;
    }
    if (!hasOffsetTime) {
        frame.hasPointOffsetTime = false;
    }
}

void appendCartesianHighPoints(const uint8_t* dataZone,
                               const LivoxPayloadHeader& header,
                               int lineCount,
                               SlamInputFrame& frame)
{
    constexpr size_t stride = 14;
    const bool hasOffsets = hasPointOffsetTime(header);
    for (uint16_t i = 0; i < header.dotNum; ++i) {
        const uint8_t* p = dataZone + size_t(i) * stride;
        SlamPoint point;
        point.x = static_cast<float>(readLeInt32(p + 0)) / 1000.0f;
        point.y = static_cast<float>(readLeInt32(p + 4)) / 1000.0f;
        point.z = static_cast<float>(readLeInt32(p + 8)) / 1000.0f;
        point.reflectivity = p[12];
        point.tag = p[13];
        point.line = LivoxCore::lineForPointIndex(int(i), lineCount);
        point.hasLine = true;
        addSlamPoint(frame, point, header.timestampNs + sampleOffsetNs(header, i), hasOffsets);
    }
}

void appendCartesianLowPoints(const uint8_t* dataZone,
                              const LivoxPayloadHeader& header,
                              int lineCount,
                              SlamInputFrame& frame)
{
    constexpr size_t stride = 8;
    const bool hasOffsets = hasPointOffsetTime(header);
    for (uint16_t i = 0; i < header.dotNum; ++i) {
        const uint8_t* p = dataZone + size_t(i) * stride;
        SlamPoint point;
        point.x = static_cast<float>(readLeInt16(p + 0)) / 100.0f;
        point.y = static_cast<float>(readLeInt16(p + 2)) / 100.0f;
        point.z = static_cast<float>(readLeInt16(p + 4)) / 100.0f;
        point.reflectivity = p[6];
        point.tag = p[7];
        point.line = LivoxCore::lineForPointIndex(int(i), lineCount);
        point.hasLine = true;
        addSlamPoint(frame, point, header.timestampNs + sampleOffsetNs(header, i), hasOffsets);
    }
}

void appendSphericalPoints(const uint8_t* dataZone,
                           const LivoxPayloadHeader& header,
                           int lineCount,
                           SlamInputFrame& frame)
{
    constexpr size_t stride = 10;
    const bool hasOffsets = hasPointOffsetTime(header);
    for (uint16_t i = 0; i < header.dotNum; ++i) {
        const uint8_t* p = dataZone + size_t(i) * stride;
        const float depth = static_cast<float>(readLe32(p + 0)) / 1000.0f;
        const float zenithRad = static_cast<float>(readLe16(p + 4)) * 0.01f * float(kLivoxPi) / 180.0f;
        const float azimuthRad = static_cast<float>(readLe16(p + 6)) * 0.01f * float(kLivoxPi) / 180.0f;

        SlamPoint point;
        point.x = depth * std::sin(zenithRad) * std::cos(azimuthRad);
        point.y = depth * std::sin(zenithRad) * std::sin(azimuthRad);
        point.z = depth * std::cos(zenithRad);
        point.reflectivity = p[8];
        point.tag = p[9];
        point.line = LivoxCore::lineForPointIndex(int(i), lineCount);
        point.hasLine = true;
        addSlamPoint(frame, point, header.timestampNs + sampleOffsetNs(header, i), hasOffsets);
    }
}

void appendDoubleEchoPoints(const uint8_t* dataZone,
                            const LivoxPayloadHeader& header,
                            int lineCount,
                            SlamInputFrame& frame)
{
    constexpr size_t stride = 28;
    const bool hasOffsets = hasPointOffsetTime(header);
    for (uint16_t i = 0; i < header.dotNum; ++i) {
        const uint8_t* p = dataZone + size_t(i) * stride;
        const int64_t timestampNs = header.timestampNs + sampleOffsetNs(header, i);

        SlamPoint point1;
        point1.x = static_cast<float>(readLeInt32(p + 0)) / 1000.0f;
        point1.y = static_cast<float>(readLeInt32(p + 4)) / 1000.0f;
        point1.z = static_cast<float>(readLeInt32(p + 8)) / 1000.0f;
        point1.reflectivity = p[12];
        point1.tag = p[13];
        point1.line = LivoxCore::lineForPointIndex(int(i) * 2, lineCount);
        point1.hasLine = true;
        addSlamPoint(frame, point1, timestampNs, hasOffsets);

        SlamPoint point2;
        point2.x = static_cast<float>(readLeInt32(p + 14)) / 1000.0f;
        point2.y = static_cast<float>(readLeInt32(p + 18)) / 1000.0f;
        point2.z = static_cast<float>(readLeInt32(p + 22)) / 1000.0f;
        point2.reflectivity = p[26];
        point2.tag = p[27];
        point2.line = LivoxCore::lineForPointIndex(int(i) * 2 + 1, lineCount);
        point2.hasLine = true;
        addSlamPoint(frame, point2, timestampNs, hasOffsets);
    }
}

size_t pointStride(uint8_t dataType)
{
    switch (dataType) {
    case kLivoxLidarCartesianCoordinateHighData:
        return 14;
    case kLivoxLidarCartesianCoordinateLowData:
        return 8;
    case kLivoxLidarSphericalCoordinateData:
        return 10;
    case kLivoxLidarDoubleEchoData:
        return 28;
    default:
        return 0;
    }
}

bool appendPointPayload(const uint8_t* payload,
                        size_t payloadLen,
                        const LivoxPayloadHeader& header,
                        int lineCount,
                        SlamInputFrame& frame)
{
    const size_t stride = pointStride(header.dataType);
    if (stride == 0 || payloadLen - kLivoxPayloadHeaderSize < size_t(header.dotNum) * stride) {
        return false;
    }

    const uint8_t* dataZone = payload + kLivoxPayloadHeaderSize;
    switch (header.dataType) {
    case kLivoxLidarCartesianCoordinateHighData:
        appendCartesianHighPoints(dataZone, header, lineCount, frame);
        return true;
    case kLivoxLidarCartesianCoordinateLowData:
        appendCartesianLowPoints(dataZone, header, lineCount, frame);
        return true;
    case kLivoxLidarSphericalCoordinateData:
        appendSphericalPoints(dataZone, header, lineCount, frame);
        return true;
    case kLivoxLidarDoubleEchoData:
        appendDoubleEchoPoints(dataZone, header, lineCount, frame);
        return true;
    default:
        return false;
    }
}

void resetFrame(SlamInputFrame& frame,
                uint64_t sequence,
                uint32_t lidarId,
                uint8_t deviceType,
                int64_t startTimestampNs,
                const QString& sourceName)
{
    frame = SlamInputFrame();
    frame.sequence = sequence;
    frame.sourceId = lidarId;
    frame.deviceType = deviceType;
    frame.frameStartNs = startTimestampNs;
    frame.frameEndNs = startTimestampNs;
    frame.timeSource = SlamTimeSource::LivoxPacketTimestamp;
    frame.hasPointOffsetTime = true;
    frame.sourceName = sourceName;
}

void flushFrame(QVector<SlamInputFrame>& frames, SlamInputFrame& frame, bool& hasFrame)
{
    if (hasFrame && !frame.points.isEmpty()) {
        frames.push_back(std::move(frame));
    }
    frame = SlamInputFrame();
    hasFrame = false;
}

void scanMetadata(pcap_t* handle, PcapMetadata& metadata)
{
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

        if (udpInfo.srcPort == PcapUdp::kLivoxPushDataPort && udpInfo.payload != nullptr && udpInfo.payloadLen > 0) {
            PushMsgParser::mergePushPacket(udpInfo.srcIp,
                                           udpInfo.payload,
                                           udpInfo.payloadLen,
                                           metadata.pushDevicesByIp);
            continue;
        }

        const bool isDataPort =
            udpInfo.srcPort == PcapUdp::kLivoxPointCloudPort || udpInfo.srcPort == PcapUdp::kLivoxIMUPort;
        if (isDataPort && !udpInfo.srcIp.isEmpty() && udpInfo.payload != nullptr) {
            metadata.dataSourceIps.insert(udpInfo.srcIp, PushMsgParser::ipToLidarId(udpInfo.srcIp));
        }
    }

    metadata.devices = PushMsgParser::finalizeDevices(metadata.pushDevicesByIp, metadata.dataSourceIps);
}

QMap<uint32_t, int> lineCountsByLidarId(const QVector<PushMsgParser::PushDeviceRecord>& devices)
{
    QMap<uint32_t, int> lineCounts;
    for (const PushMsgParser::PushDeviceRecord& device : devices) {
        lineCounts.insert(device.lidarId, LivoxCore::lineCountForDeviceType(device.deviceType));
    }
    return lineCounts;
}

QMap<uint32_t, uint8_t> deviceTypesByLidarId(const QVector<PushMsgParser::PushDeviceRecord>& devices)
{
    QMap<uint32_t, uint8_t> deviceTypes;
    for (const PushMsgParser::PushDeviceRecord& device : devices) {
        deviceTypes.insert(device.lidarId, device.deviceType);
    }
    return deviceTypes;
}

QVector<PcapSlamDeviceInfo> toDeviceInfo(const QVector<PushMsgParser::PushDeviceRecord>& devices)
{
    QVector<PcapSlamDeviceInfo> result;
    result.reserve(devices.size());
    for (const PushMsgParser::PushDeviceRecord& device : devices) {
        PcapSlamDeviceInfo info;
        info.lidarId = device.lidarId;
        info.deviceType = device.deviceType;
        info.lidarSn = device.lidarSn;
        info.modelDisplay = device.modelDisplay;
        result.push_back(info);
    }
    return result;
}

void parseImuPayload(const uint8_t* payload,
                     size_t payloadLen,
                     uint32_t lidarId,
                     PcapSlamSourceSummary& summary,
                     QVector<SlamImuSample>& samples)
{
    LivoxPayloadHeader header;
    if (!parseLivoxPayloadHeader(payload, payloadLen, header) || header.dataType != kLivoxLidarImuData) {
        return;
    }

    constexpr size_t stride = 24;
    if (payloadLen - kLivoxPayloadHeaderSize < size_t(header.dotNum) * stride) {
        return;
    }

    const bool hasTiming = header.dotNum <= 1 || header.timeIntervalRaw != 0;
    if (!hasTiming) {
        summary.missingImuTimingPacketCount++;
    }

    const uint8_t* dataZone = payload + kLivoxPayloadHeaderSize;
    for (uint16_t i = 0; i < header.dotNum; ++i) {
        const uint8_t* p = dataZone + size_t(i) * stride;
        SlamImuSample sample;
        sample.lidarId = lidarId;
        sample.timestampNs = header.timestampNs + sampleOffsetNs(header, i);
        sample.gyroRadPerSec[0] = readFloat(p + 0);
        sample.gyroRadPerSec[1] = readFloat(p + 4);
        sample.gyroRadPerSec[2] = readFloat(p + 8);
        sample.accelRaw[0] = readFloat(p + 12);
        sample.accelRaw[1] = readFloat(p + 16);
        sample.accelRaw[2] = readFloat(p + 20);
        samples.push_back(sample);
    }

    summary.imuPacketCount++;
    summary.imuSampleCount += header.dotNum;
}

void attachImuSamples(QVector<SlamInputFrame>& frames,
                      const QVector<SlamImuSample>& imuSamples,
                      PcapSlamSourceSummary& summary)
{
    QVector<SlamImuSample> sortedSamples = imuSamples;
    std::sort(sortedSamples.begin(), sortedSamples.end(), [](const SlamImuSample& lhs, const SlamImuSample& rhs) {
        return lhs.timestampNs < rhs.timestampNs;
    });

    for (SlamInputFrame& frame : frames) {
        const auto firstInside = std::lower_bound(sortedSamples.begin(),
                                                  sortedSamples.end(),
                                                  frame.frameStartNs,
                                                  [](const SlamImuSample& sample, int64_t timestampNs) {
                                                      return sample.timestampNs < timestampNs;
                                                  });
        auto attachBegin = firstInside;
        if (attachBegin != sortedSamples.begin()) {
            --attachBegin;
        }

        auto firstAfterEnd = std::upper_bound(sortedSamples.begin(),
                                              sortedSamples.end(),
                                              frame.frameEndNs,
                                              [](int64_t timestampNs, const SlamImuSample& sample) {
                                                  return timestampNs < sample.timestampNs;
                                              });
        auto attachEnd = firstAfterEnd;
        if (attachEnd != sortedSamples.end()) {
            ++attachEnd;
        }

        for (auto it = attachBegin; it != attachEnd; ++it) {
            frame.imuSamples.push_back(*it);
        }

        if (!frame.imuSamples.isEmpty()) {
            const int64_t firstTimestamp = frame.imuSamples.first().timestampNs;
            const int64_t lastTimestamp = frame.imuSamples.last().timestampNs;
            frame.hasCompleteImuCoverage = firstTimestamp <= frame.frameStartNs && lastTimestamp >= frame.frameEndNs;
            if (frame.hasCompleteImuCoverage) {
                summary.framesWithCompleteImuCoverage++;
            }
        }
    }
}

void finalizeSummary(QVector<SlamInputFrame>& frames,
                     const QVector<SlamImuSample>& imuSamples,
                     PcapSlamSourceSummary& summary)
{
    summary.frameCount = frames.size();
    summary.hasImu = !imuSamples.isEmpty();
    summary.hasPointOffsetTime = summary.missingPointOffsetPacketCount == 0;

    if (!frames.isEmpty()) {
        summary.startTimestampNs = frames.first().frameStartNs;
        summary.endTimestampNs = frames.first().frameEndNs;
        for (const SlamInputFrame& frame : frames) {
            if (frame.frameStartNs < summary.startTimestampNs) {
                summary.startTimestampNs = frame.frameStartNs;
            }
            if (frame.frameEndNs > summary.endTimestampNs) {
                summary.endTimestampNs = frame.frameEndNs;
            }
        }
    }

    if (!summary.hasImu) {
        summary.status = SlamStatusCode::MissingImu;
        summary.messages.push_back(QStringLiteral("PCAP 未解析到 IMU payload，FAST_LIO 后端不得启动。"));
    } else if (summary.outOfOrderPointPacketCount > 0 || summary.missingPointOffsetPacketCount > 0 ||
               summary.missingImuTimingPacketCount > 0 || summary.framesWithCompleteImuCoverage < summary.frameCount) {
        summary.status = SlamStatusCode::TimeSyncError;
        if (summary.outOfOrderPointPacketCount > 0) {
            summary.messages.push_back(QStringLiteral("PCAP 点云 payload 时间戳存在乱序包。"));
        }
        if (summary.missingPointOffsetPacketCount > 0) {
            summary.messages.push_back(QStringLiteral("PCAP 点云 payload 缺少可用点内 offset 时间。"));
        }
        if (summary.missingImuTimingPacketCount > 0) {
            summary.messages.push_back(QStringLiteral("PCAP IMU payload 缺少可用包内采样时间。"));
        }
        if (summary.framesWithCompleteImuCoverage < summary.frameCount) {
            summary.messages.push_back(QStringLiteral("PCAP IMU 样本未完整覆盖所有 SLAM 输入帧。"));
        }
    }
}

} // namespace

PcapSlamSource::PcapSlamSource(int frameDurationMs)
{
    setFrameDurationMs(frameDurationMs);
}

void PcapSlamSource::setFrameDurationMs(int frameDurationMs)
{
    frameDurationNs_ = int64_t(std::max(1, frameDurationMs)) * kNsPerMs;
}

int PcapSlamSource::frameDurationMs() const
{
    return int(frameDurationNs_ / kNsPerMs);
}

bool PcapSlamSource::load(const QString& filePath, QString* error)
{
    clear();
    summary_.filePath = filePath;

    char metadataErrbuf[PCAP_ERRBUF_SIZE] = {};
    pcap_t* metadataHandle = openOfflinePcap(filePath, metadataErrbuf);
    if (metadataHandle == nullptr) {
        errorMessage_ = QString("无法打开抓包文件:\n%1").arg(metadataErrbuf);
        if (error != nullptr) {
            *error = errorMessage_;
        }
        summary_.status = SlamStatusCode::Failed;
        return false;
    }

    PcapMetadata metadata;
    scanMetadata(metadataHandle, metadata);
    pcap_close(metadataHandle);
    summary_.devices = toDeviceInfo(metadata.devices);

    const QMap<uint32_t, int> lineCounts = lineCountsByLidarId(metadata.devices);
    const QMap<uint32_t, uint8_t> deviceTypes = deviceTypesByLidarId(metadata.devices);

    char dataErrbuf[PCAP_ERRBUF_SIZE] = {};
    pcap_t* dataHandle = openOfflinePcap(filePath, dataErrbuf);
    if (dataHandle == nullptr) {
        errorMessage_ = QString("无法打开抓包文件:\n%1").arg(dataErrbuf);
        if (error != nullptr) {
            *error = errorMessage_;
        }
        summary_.status = SlamStatusCode::Failed;
        return false;
    }

    QVector<SlamImuSample> imuSamples;
    QSet<uint32_t> pointLidarIds;
    SlamInputFrame currentFrame;
    bool hasFrame = false;
    uint64_t nextSequence = 0;
    bool hasLastPointPacketTime = false;
    int64_t lastPointPacketTimeNs = 0;

    struct pcap_pkthdr* packetHeader = nullptr;
    const u_char* packetData = nullptr;
    int res = 0;

    while ((res = pcap_next_ex(dataHandle, &packetHeader, &packetData)) >= 0) {
        if (res == 0 || packetHeader == nullptr) {
            continue;
        }

        PcapUdp::PacketInfo udpInfo;
        if (!PcapUdp::tryExtractUdp(packetData, packetHeader->caplen, udpInfo)) {
            continue;
        }

        if (udpInfo.payload == nullptr) {
            continue;
        }

        const uint32_t lidarId = udpInfo.srcIp.isEmpty() ? 0u : PushMsgParser::ipToLidarId(udpInfo.srcIp);

        if (udpInfo.srcPort == PcapUdp::kLivoxIMUPort) {
            parseImuPayload(udpInfo.payload, udpInfo.payloadLen, lidarId, summary_, imuSamples);
            continue;
        }

        if (udpInfo.srcPort != PcapUdp::kLivoxPointCloudPort ||
            !PointParser::isLivoxPointCloudPayload(udpInfo.payload, udpInfo.payloadLen)) {
            continue;
        }

        LivoxPayloadHeader header;
        if (!parseLivoxPayloadHeader(udpInfo.payload, udpInfo.payloadLen, header)) {
            continue;
        }

        if (hasLastPointPacketTime && header.timestampNs < lastPointPacketTimeNs) {
            summary_.outOfOrderPointPacketCount++;
            continue;
        }
        lastPointPacketTimeNs = header.timestampNs;
        hasLastPointPacketTime = true;

        pointLidarIds.insert(lidarId);
        if (!hasFrame) {
            resetFrame(currentFrame,
                       nextSequence++,
                       lidarId,
                       deviceTypes.value(lidarId, 0),
                       header.timestampNs,
                       filePath);
            hasFrame = true;
        } else if (header.timestampNs - currentFrame.frameStartNs >= frameDurationNs_) {
            flushFrame(frames_, currentFrame, hasFrame);
            resetFrame(currentFrame,
                       nextSequence++,
                       lidarId,
                       deviceTypes.value(lidarId, 0),
                       header.timestampNs,
                       filePath);
            hasFrame = true;
        }

        if (!hasPointOffsetTime(header)) {
            summary_.missingPointOffsetPacketCount++;
        }

        const qsizetype pointCountBefore = currentFrame.points.size();
        if (appendPointPayload(udpInfo.payload,
                               udpInfo.payloadLen,
                               header,
                               lineCounts.value(lidarId, 1),
                               currentFrame)) {
            summary_.pointPacketCount++;
            summary_.pointCount += uint64_t(currentFrame.points.size() - pointCountBefore);
        }
    }

    pcap_close(dataHandle);
    flushFrame(frames_, currentFrame, hasFrame);

    if (pointLidarIds.size() > 1) {
        errorMessage_ = QStringLiteral("SLAM MVP 仅支持单雷达 PCAP，当前文件包含多个点云源。");
        if (error != nullptr) {
            *error = errorMessage_;
        }
        summary_.status = SlamStatusCode::Failed;
        return false;
    }

    if (frames_.isEmpty()) {
        errorMessage_ = QStringLiteral("文件已解析，但未生成有效的 SLAM 输入帧。");
        if (error != nullptr) {
            *error = errorMessage_;
        }
        summary_.status = SlamStatusCode::Failed;
        return false;
    }

    attachImuSamples(frames_, imuSamples, summary_);
    finalizeSummary(frames_, imuSamples, summary_);
    return true;
}

void PcapSlamSource::clear()
{
    frames_.clear();
    summary_ = PcapSlamSourceSummary();
    errorMessage_.clear();
}

int PcapSlamSource::frameCount() const
{
    return frames_.size();
}

const SlamInputFrame& PcapSlamSource::frameAt(int index) const
{
    return frames_.at(index);
}

const QVector<SlamInputFrame>& PcapSlamSource::frames() const
{
    return frames_;
}

const PcapSlamSourceSummary& PcapSlamSource::summary() const
{
    return summary_;
}

QString PcapSlamSource::errorMessage() const
{
    return errorMessage_;
}

QString PcapSlamSource::summaryText() const
{
    QStringList lines;
    lines << QStringLiteral("PCAP SLAM 输入摘要");
    lines << QStringLiteral("- 文件: %1").arg(summary_.filePath);
    lines << QStringLiteral("- 帧数: %1").arg(summary_.frameCount);
    lines << QStringLiteral("- 点数: %1").arg(QString::number(summary_.pointCount));
    lines << QStringLiteral("- IMU 样本数: %1").arg(QString::number(summary_.imuSampleCount));
    lines << QStringLiteral("- 点云包数: %1").arg(QString::number(summary_.pointPacketCount));
    lines << QStringLiteral("- IMU 包数: %1").arg(QString::number(summary_.imuPacketCount));
    lines << QStringLiteral("- 时间范围(ns): %1 - %2")
                 .arg(QString::number(summary_.startTimestampNs), QString::number(summary_.endTimestampNs));
    lines << QStringLiteral("- 完整 IMU 覆盖帧数: %1/%2")
                 .arg(summary_.framesWithCompleteImuCoverage)
                 .arg(summary_.frameCount);
    if (!summary_.messages.isEmpty()) {
        lines << QStringLiteral("- 状态:");
        for (const QString& message : summary_.messages) {
            lines << QStringLiteral("  - %1").arg(message);
        }
    }
    return lines.join(QLatin1Char('\n'));
}

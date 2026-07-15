#ifndef SLAM_IO_PCAPSLAMSOURCE_H
#define SLAM_IO_PCAPSLAMSOURCE_H

#include "Core/SlamTypes.h"

#include <QString>
#include <QStringList>
#include <QVector>

#include <cstdint>
#include <atomic>
#include <functional>

struct PcapSlamDeviceInfo {
    uint32_t lidarId = 0;
    uint8_t deviceType = 0;
    QString lidarSn;
    QString modelDisplay;
};

struct PcapSlamSourceSummary {
    QString filePath;
    QVector<PcapSlamDeviceInfo> devices;
    SlamStatusCode status = SlamStatusCode::Idle;
    QStringList messages;
    int frameCount = 0;
    uint64_t pointCount = 0;
    uint64_t imuSampleCount = 0;
    uint64_t pointPacketCount = 0;
    uint64_t imuPacketCount = 0;
    uint64_t outOfOrderPointPacketCount = 0;
    uint64_t missingPointOffsetPacketCount = 0;
    uint64_t missingImuTimingPacketCount = 0;
    int framesWithCompleteImuCoverage = 0;
    int64_t startTimestampNs = 0;
    int64_t endTimestampNs = 0;
    bool hasImu = false;
    bool hasPointOffsetTime = true;
};

class PcapSlamSource {
public:
    explicit PcapSlamSource(int frameDurationMs = 100);

    void setFrameDurationMs(int frameDurationMs);
    int frameDurationMs() const;
    bool load(const QString& filePath, QString* error);
    bool streamFrames(const QString& filePath,
                      bool requireImu,
                      const std::atomic_bool* cancellationRequested,
                      const std::function<bool(SlamInputFrame&&)>& consumer,
                      QString* error);
    void clear();

    int frameCount() const;
    const SlamInputFrame& frameAt(int index) const;
    const QVector<SlamInputFrame>& frames() const;
    const PcapSlamSourceSummary& summary() const;
    QString errorMessage() const;
    QString summaryText() const;

private:
    QVector<SlamInputFrame> frames_;
    PcapSlamSourceSummary summary_;
    QString errorMessage_;
    int64_t frameDurationNs_ = 100000000;
};

#endif // SLAM_IO_PCAPSLAMSOURCE_H

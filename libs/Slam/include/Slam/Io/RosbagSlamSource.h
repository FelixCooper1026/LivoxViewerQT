#ifndef SLAM_IO_ROSBAGSLAMSOURCE_H
#define SLAM_IO_ROSBAGSLAMSOURCE_H

#include "Slam/Core/SlamTypes.h"

#include <QString>
#include <QStringList>
#include <QVector>

#include <cstdint>

struct RosbagSlamTopicInfo {
    QString topic;
    QString type;
    int64_t messageCount = 0;
};

struct RosbagSlamSourceConfig {
    QString lidarTopic;
    QString imuTopic;
    bool autoDetectTopics = true;
    bool useLivoxCustomTimebase = false;
    bool useHeaderStamp = true;
    bool requirePointOffsetTime = true;
    bool synthesizePointOffsetTime = false;
    bool allowLivoxDriver2PointCloud2 = false;
    bool allowLivoxDriverPointCloud2SynthesizedTime = false;
    int64_t lidarToImuTimeOffsetNs = 0;
    int frameDurationMs = 100;
};

struct RosbagSlamSourceSummary {
    QString filePath;
    QString format;
    QVector<RosbagSlamTopicInfo> topics;
    QString lidarTopic;
    QString lidarType;
    QString lidarFormat;
    QString imuTopic;
    QString imuType;
    QString pointTimeMode;
    int frameCount = 0;
    uint64_t pointCount = 0;
    uint64_t imuSampleCount = 0;
    uint64_t lidarMessageCount = 0;
    uint64_t imuMessageCount = 0;
    uint64_t emptyLidarMessageCount = 0;
    int framesWithCompleteImuCoverage = 0;
    int64_t startTimestampNs = 0;
    int64_t endTimestampNs = 0;
    int64_t imuStartTimestampNs = 0;
    int64_t imuEndTimestampNs = 0;
    bool hasImu = false;
    bool hasPointOffsetTime = false;
    bool hasCompleteImuCoverage = false;
    QStringList messages;
};

class RosbagSlamSource {
public:
    explicit RosbagSlamSource(const RosbagSlamSourceConfig& config = {});

    void setConfig(const RosbagSlamSourceConfig& config);
    const RosbagSlamSourceConfig& config() const;

    bool load(const QString& filePath, QString* error);
    void clear();

    int frameCount() const;
    const SlamInputFrame& frameAt(int index) const;
    const QVector<SlamInputFrame>& frames() const;
    const RosbagSlamSourceSummary& summary() const;
    QString errorMessage() const;
    QString summaryText() const;

private:
    RosbagSlamSourceConfig config_;
    QVector<SlamInputFrame> frames_;
    RosbagSlamSourceSummary summary_;
    QString errorMessage_;
};

#endif // SLAM_IO_ROSBAGSLAMSOURCE_H

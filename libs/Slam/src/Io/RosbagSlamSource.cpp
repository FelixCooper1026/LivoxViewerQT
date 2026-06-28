#include "Slam/Io/RosbagSlamSource.h"

#include "Rosbag/RosMessageParsers.h"
#include "Rosbag/RosbagReader.h"

#include <QFileInfo>

#include <algorithm>
#include <limits>

namespace {

bool isLivoxCustomMsgType(const QString& type)
{
    return type == QStringLiteral("livox_ros_driver2/CustomMsg") ||
           type == QStringLiteral("livox_ros_driver/CustomMsg");
}

bool isPointCloud2Type(const QString& type)
{
    return type == QStringLiteral("sensor_msgs/PointCloud2");
}

bool isImuType(const QString& type)
{
    return type == QStringLiteral("sensor_msgs/Imu");
}

QString topicListText(const QVector<RosbagSlamTopicInfo>& topics)
{
    QStringList items;
    for (const RosbagSlamTopicInfo& topic : topics) {
        items << QStringLiteral("%1(%2)").arg(topic.topic, topic.type);
    }
    return items.join(QStringLiteral(", "));
}

int lidarTopicPriority(const QString& topic, const QString& type)
{
    if (!isLivoxCustomMsgType(type) && !isPointCloud2Type(type)) {
        return std::numeric_limits<int>::max();
    }
    int priority = isLivoxCustomMsgType(type) ? 0 : 100;
    if (topic == QStringLiteral("/livox/lidar") || topic == QStringLiteral("livox/lidar")) {
        return priority;
    }
    if (topic.startsWith(QStringLiteral("/livox/lidar_")) || topic.startsWith(QStringLiteral("livox/lidar_"))) {
        return priority + 1;
    }
    if (topic == QStringLiteral("/livox/lidar_msg") || topic == QStringLiteral("livox/lidar_msg")) {
        return priority + 2;
    }
    return priority + 10;
}

int imuTopicPriority(const QString& topic, const QString& type)
{
    if (!isImuType(type)) {
        return std::numeric_limits<int>::max();
    }
    if (topic == QStringLiteral("/livox/imu") || topic == QStringLiteral("livox/imu")) {
        return 0;
    }
    if (topic.startsWith(QStringLiteral("/livox/imu_")) || topic.startsWith(QStringLiteral("livox/imu_"))) {
        return 1;
    }
    if (topic == QStringLiteral("/livox/imu_data") || topic == QStringLiteral("livox/imu_data")) {
        return 2;
    }
    return 10;
}

const Rosbag::Connection* findConnectionByTopic(const QVector<Rosbag::Connection>& connections, const QString& topic)
{
    for (const Rosbag::Connection& connection : connections) {
        if (connection.topic == topic) {
            return &connection;
        }
    }
    return nullptr;
}

const Rosbag::Connection* autoSelectLidarConnection(const QVector<Rosbag::Connection>& connections)
{
    const Rosbag::Connection* selected = nullptr;
    int bestPriority = std::numeric_limits<int>::max();
    for (const Rosbag::Connection& connection : connections) {
        const int priority = lidarTopicPriority(connection.topic, connection.type);
        if (priority < bestPriority) {
            selected = &connection;
            bestPriority = priority;
        }
    }
    return selected;
}

const Rosbag::Connection* autoSelectImuConnection(const QVector<Rosbag::Connection>& connections)
{
    const Rosbag::Connection* selected = nullptr;
    int bestPriority = std::numeric_limits<int>::max();
    for (const Rosbag::Connection& connection : connections) {
        const int priority = imuTopicPriority(connection.topic, connection.type);
        if (priority < bestPriority) {
            selected = &connection;
            bestPriority = priority;
        }
    }
    return selected;
}

SlamPoint toSlamPoint(const Rosbag::LivoxCustomPoint& source)
{
    SlamPoint point;
    point.x = source.x;
    point.y = source.y;
    point.z = source.z;
    point.reflectivity = source.reflectivity;
    point.tag = source.tag;
    point.line = source.line;
    point.hasLine = true;
    point.offsetNs = source.offsetTimeNs;
    point.hasOffsetTime = true;
    return point;
}

SlamImuSample toSlamImuSample(const Rosbag::ImuMsg& source, int64_t offsetNs)
{
    SlamImuSample sample;
    sample.timestampNs = source.header.stampNs + offsetNs;
    sample.gyroRadPerSec[0] = source.angularVelocity[0];
    sample.gyroRadPerSec[1] = source.angularVelocity[1];
    sample.gyroRadPerSec[2] = source.angularVelocity[2];
    sample.accelMps2[0] = source.linearAcceleration[0];
    sample.accelMps2[1] = source.linearAcceleration[1];
    sample.accelMps2[2] = source.linearAcceleration[2];
    return sample;
}

void attachImuSamples(QVector<SlamInputFrame>& frames,
                      const QVector<SlamImuSample>& imuSamples,
                      RosbagSlamSourceSummary& summary)
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
    summary.hasCompleteImuCoverage = summary.framesWithCompleteImuCoverage == summary.frameCount && summary.frameCount > 0;
}

void finalizeTimestampRange(const QVector<SlamInputFrame>& frames, RosbagSlamSourceSummary& summary)
{
    if (frames.isEmpty()) {
        return;
    }
    summary.startTimestampNs = frames.first().frameStartNs;
    summary.endTimestampNs = frames.first().frameEndNs;
    for (const SlamInputFrame& frame : frames) {
        summary.startTimestampNs = std::min(summary.startTimestampNs, frame.frameStartNs);
        summary.endTimestampNs = std::max(summary.endTimestampNs, frame.frameEndNs);
    }
}

} // namespace

RosbagSlamSource::RosbagSlamSource(const RosbagSlamSourceConfig& config)
    : config_(config)
{
}

void RosbagSlamSource::setConfig(const RosbagSlamSourceConfig& config)
{
    config_ = config;
}

const RosbagSlamSourceConfig& RosbagSlamSource::config() const
{
    return config_;
}

bool RosbagSlamSource::load(const QString& filePath, QString* error)
{
    clear();
    summary_.filePath = filePath;
    summary_.format = QStringLiteral("ROS1 bag");

    Rosbag::Reader reader;
    if (!reader.read(filePath, &errorMessage_)) {
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }

    for (const Rosbag::Connection& connection : reader.connections()) {
        RosbagSlamTopicInfo topic;
        topic.topic = connection.topic;
        topic.type = connection.type;
        topic.messageCount = connection.messageCount;
        summary_.topics.push_back(topic);
    }

    const Rosbag::Connection* lidarConnection = config_.lidarTopic.isEmpty()
        ? autoSelectLidarConnection(reader.connections())
        : findConnectionByTopic(reader.connections(), config_.lidarTopic);
    if (lidarConnection == nullptr) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：未找到 LiDAR topic。已发现 topic: %1。").arg(topicListText(summary_.topics));
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }

    const Rosbag::Connection* imuConnection = config_.imuTopic.isEmpty()
        ? autoSelectImuConnection(reader.connections())
        : findConnectionByTopic(reader.connections(), config_.imuTopic);
    if (imuConnection == nullptr) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：未找到 IMU topic。已发现 topic: %1。").arg(topicListText(summary_.topics));
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }

    summary_.lidarTopic = lidarConnection->topic;
    summary_.lidarType = lidarConnection->type;
    summary_.imuTopic = imuConnection->topic;
    summary_.imuType = imuConnection->type;

    if (isPointCloud2Type(lidarConnection->type)) {
        errorMessage_ = config_.allowLivoxDriver2PointCloud2
            ? QStringLiteral("ROSbag 加载失败：driver2 PointCloud2 解析尚未实现。请使用 xfer_format=1 录制 livox_ros_driver2/CustomMsg。")
            : QStringLiteral("ROSbag 加载失败：LiDAR topic %1 是 sensor_msgs/PointCloud2。当前第一版仅启用 livox_ros_driver2/CustomMsg；请使用 xfer_format=1 录制。")
                  .arg(lidarConnection->topic);
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }
    if (!isLivoxCustomMsgType(lidarConnection->type)) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 类型 %2 不受支持。当前第一版仅支持 livox_ros_driver2/CustomMsg。")
                            .arg(lidarConnection->topic, lidarConnection->type);
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }

    summary_.lidarFormat = QStringLiteral("Livox CustomMsg");
    summary_.pointTimeMode = QStringLiteral("CustomMsg offset_time(ns)");

    QVector<SlamImuSample> imuSamples;
    uint64_t nextSequence = 0;

    for (const Rosbag::SerializedMessage& message : reader.messages()) {
        if (message.connectionId == imuConnection->id) {
            Rosbag::ImuMsg imu;
            QString parseError;
            if (!Rosbag::parseSensorImu(message.data, &imu, &parseError)) {
                errorMessage_ = QStringLiteral("ROSbag 加载失败：IMU topic %1 解析失败：%2")
                                    .arg(imuConnection->topic, parseError);
                if (error != nullptr) {
                    *error = errorMessage_;
                }
                return false;
            }
            imuSamples.push_back(toSlamImuSample(imu, config_.lidarToImuTimeOffsetNs));
            summary_.imuMessageCount++;
            summary_.imuSampleCount++;
            continue;
        }

        if (message.connectionId != lidarConnection->id) {
            continue;
        }

        Rosbag::LivoxCustomMsg livoxMsg;
        QString parseError;
        if (!Rosbag::parseLivoxCustomMsg(message.data, &livoxMsg, &parseError)) {
            errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 解析失败：%2")
                                .arg(lidarConnection->topic, parseError);
            if (error != nullptr) {
                *error = errorMessage_;
            }
            return false;
        }

        summary_.lidarMessageCount++;
        if (livoxMsg.points.isEmpty()) {
            summary_.emptyLidarMessageCount++;
            continue;
        }

        int64_t frameStartNs = 0;
        if (config_.useLivoxCustomTimebase && livoxMsg.timebaseNs > 0) {
            frameStartNs = int64_t(livoxMsg.timebaseNs);
        } else if (config_.useHeaderStamp && livoxMsg.header.stampNs > 0) {
            frameStartNs = livoxMsg.header.stampNs;
        } else {
            frameStartNs = message.timestampNs;
        }

        SlamInputFrame frame;
        frame.sequence = nextSequence++;
        frame.sourceId = livoxMsg.lidarId;
        frame.deviceType = 0;
        frame.frameStartNs = frameStartNs;
        frame.frameEndNs = frameStartNs;
        frame.timeSource = SlamTimeSource::LivoxPacketTimestamp;
        frame.hasPointOffsetTime = true;
        frame.sourceName = filePath;
        frame.points.reserve(livoxMsg.points.size());

        for (const Rosbag::LivoxCustomPoint& sourcePoint : livoxMsg.points) {
            SlamPoint point = toSlamPoint(sourcePoint);
            frame.frameEndNs = std::max(frame.frameEndNs, frameStartNs + int64_t(point.offsetNs));
            frame.points.push_back(point);
        }
        summary_.pointCount += uint64_t(frame.points.size());
        frames_.push_back(std::move(frame));
    }

    std::sort(frames_.begin(), frames_.end(), [](const SlamInputFrame& lhs, const SlamInputFrame& rhs) {
        return lhs.frameStartNs < rhs.frameStartNs;
    });
    for (int i = 0; i < frames_.size(); ++i) {
        frames_[i].sequence = uint64_t(i);
    }

    summary_.frameCount = frames_.size();
    summary_.hasImu = !imuSamples.isEmpty();
    summary_.hasPointOffsetTime = !frames_.isEmpty();
    finalizeTimestampRange(frames_, summary_);

    if (frames_.isEmpty()) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 未生成有效 SLAM 输入帧。").arg(lidarConnection->topic);
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }
    if (!summary_.hasImu) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：IMU topic %1 没有可用 sensor_msgs/Imu 样本。").arg(imuConnection->topic);
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }

    attachImuSamples(frames_, imuSamples, summary_);
    if (summary_.framesWithCompleteImuCoverage == 0) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：IMU 样本未覆盖任何 LiDAR 帧。请检查 %1 topic、时间戳和 LiDAR/IMU 时间偏移配置。")
                            .arg(imuConnection->topic);
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }
    if (summary_.framesWithCompleteImuCoverage < summary_.frameCount) {
        summary_.messages.push_back(QStringLiteral("ROSbag IMU 样本未完整覆盖所有 SLAM 输入帧，未覆盖帧将在离线 worker 中跳过。"));
    }
    return true;
}

void RosbagSlamSource::clear()
{
    frames_.clear();
    summary_ = RosbagSlamSourceSummary();
    errorMessage_.clear();
}

int RosbagSlamSource::frameCount() const
{
    return frames_.size();
}

const SlamInputFrame& RosbagSlamSource::frameAt(int index) const
{
    return frames_.at(index);
}

const QVector<SlamInputFrame>& RosbagSlamSource::frames() const
{
    return frames_;
}

const RosbagSlamSourceSummary& RosbagSlamSource::summary() const
{
    return summary_;
}

QString RosbagSlamSource::errorMessage() const
{
    return errorMessage_;
}

QString RosbagSlamSource::summaryText() const
{
    QStringList lines;
    lines << QStringLiteral("ROSbag SLAM 输入摘要");
    lines << QStringLiteral("- 文件: %1").arg(summary_.filePath);
    lines << QStringLiteral("- 格式: %1").arg(summary_.format);
    lines << QStringLiteral("- LiDAR topic: %1").arg(summary_.lidarTopic);
    lines << QStringLiteral("- LiDAR type: %1").arg(summary_.lidarType);
    lines << QStringLiteral("- LiDAR format: %1").arg(summary_.lidarFormat);
    lines << QStringLiteral("- IMU topic: %1").arg(summary_.imuTopic);
    lines << QStringLiteral("- IMU type: %1").arg(summary_.imuType);
    lines << QStringLiteral("- 点内时间: %1").arg(summary_.pointTimeMode);
    lines << QStringLiteral("- 帧数: %1").arg(summary_.frameCount);
    lines << QStringLiteral("- 点数: %1").arg(QString::number(summary_.pointCount));
    lines << QStringLiteral("- IMU 样本数: %1").arg(QString::number(summary_.imuSampleCount));
    lines << QStringLiteral("- LiDAR 消息数: %1").arg(QString::number(summary_.lidarMessageCount));
    lines << QStringLiteral("- IMU 消息数: %1").arg(QString::number(summary_.imuMessageCount));
    lines << QStringLiteral("- 空 LiDAR 消息数: %1").arg(QString::number(summary_.emptyLidarMessageCount));
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

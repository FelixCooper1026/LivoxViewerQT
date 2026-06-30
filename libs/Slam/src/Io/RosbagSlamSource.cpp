#include "Slam/Io/RosbagSlamSource.h"

#include "Rosbag/RosMessageParsers.h"
#include "Rosbag/Ros2BagReader.h"
#include "Rosbag/RosbagReader.h"

#include <QFileInfo>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {

constexpr int64_t kPointCloud2TimestampRoundingToleranceNs = int64_t{1000000};

bool isLivoxCustomMsgType(const QString& type)
{
    return type == QStringLiteral("livox_ros_driver2/CustomMsg") ||
           type == QStringLiteral("livox_ros_driver/CustomMsg") ||
           type == QStringLiteral("livox_ros_driver2/msg/CustomMsg");
}

bool isPointCloud2Type(const QString& type)
{
    return type == QStringLiteral("sensor_msgs/PointCloud2") ||
           type == QStringLiteral("sensor_msgs/msg/PointCloud2");
}

bool isImuType(const QString& type)
{
    return type == QStringLiteral("sensor_msgs/Imu") ||
           type == QStringLiteral("sensor_msgs/msg/Imu");
}

bool isRos2Type(const QString& type)
{
    return type.contains(QStringLiteral("/msg/"));
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

bool parseLivoxCustomMessage(const QString& type, const QByteArray& data, Rosbag::LivoxCustomMsg* out, QString* error)
{
    return isRos2Type(type)
        ? Rosbag::parseRos2LivoxCustomMsg(data, out, error)
        : Rosbag::parseLivoxCustomMsg(data, out, error);
}

bool parseImuMessage(const QString& type, const QByteArray& data, Rosbag::ImuMsg* out, QString* error)
{
    return isRos2Type(type)
        ? Rosbag::parseRos2SensorImu(data, out, error)
        : Rosbag::parseSensorImu(data, out, error);
}

bool parsePointCloud2Message(const QString& type, const QByteArray& data, Rosbag::PointCloud2Msg* out, QString* error)
{
    return isRos2Type(type)
        ? Rosbag::parseRos2SensorPointCloud2(data, out, error)
        : Rosbag::parseSensorPointCloud2(data, out, error);
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

const Rosbag::PointField* pointFieldByName(const Rosbag::PointCloud2Msg& cloud, const QString& name)
{
    for (const Rosbag::PointField& field : cloud.fields) {
        if (field.name == name) {
            return &field;
        }
    }
    return nullptr;
}

bool hasPointField(const Rosbag::PointCloud2Msg& cloud, const QString& name)
{
    return pointFieldByName(cloud, name) != nullptr;
}

qsizetype pointFieldDatatypeSize(uint8_t datatype)
{
    switch (datatype) {
    case 1:
    case 2:
        return 1;
    case 3:
    case 4:
        return 2;
    case 5:
    case 6:
    case 7:
        return 4;
    case 8:
        return 8;
    default:
        return 0;
    }
}

bool pointFieldMatchesTypeAndFits(const Rosbag::PointCloud2Msg& cloud, const QString& name, uint8_t datatype)
{
    const Rosbag::PointField* field = pointFieldByName(cloud, name);
    if (field == nullptr || field->datatype != datatype || field->count != 1) {
        return false;
    }
    const qsizetype size = pointFieldDatatypeSize(field->datatype);
    return size > 0 &&
           field->offset <= cloud.pointStep &&
           size <= qsizetype(cloud.pointStep) - qsizetype(field->offset);
}

bool isLivoxDriver2PointCloud2Layout(const Rosbag::PointCloud2Msg& cloud)
{
    return !cloud.isBigEndian &&
           cloud.height == 1 &&
           cloud.pointStep > 0 &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("x"), 7) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("y"), 7) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("z"), 7) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("intensity"), 7) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("tag"), 2) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("line"), 2) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("timestamp"), 8);
}

bool isLivoxDriverPointCloud2Layout(const Rosbag::PointCloud2Msg& cloud)
{
    return !cloud.isBigEndian &&
           cloud.height == 1 &&
           cloud.pointStep > 0 &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("x"), 7) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("y"), 7) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("z"), 7) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("intensity"), 7) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("tag"), 2) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("line"), 2) &&
           !hasPointField(cloud, QStringLiteral("timestamp")) &&
           !hasPointField(cloud, QStringLiteral("time")) &&
           !hasPointField(cloud, QStringLiteral("offset_time"));
}

bool readPointBytes(const Rosbag::PointCloud2Msg& cloud, int pointIndex, const Rosbag::PointField& field, const char** out)
{
    const qsizetype offset = qsizetype(pointIndex) * qsizetype(cloud.pointStep) + qsizetype(field.offset);
    const qsizetype size = field.datatype == 8 ? 8 : (field.datatype == 7 ? 4 : 1);
    if (offset < 0 || size <= 0 || offset > cloud.data.size() || size > cloud.data.size() - offset) {
        return false;
    }
    *out = cloud.data.constData() + offset;
    return true;
}

bool readPointFloat(const Rosbag::PointCloud2Msg& cloud, int pointIndex, const Rosbag::PointField& field, float* value)
{
    const char* bytes = nullptr;
    if (field.datatype != 7 || !readPointBytes(cloud, pointIndex, field, &bytes)) {
        return false;
    }
    std::memcpy(value, bytes, sizeof(float));
    return true;
}

bool readPointDouble(const Rosbag::PointCloud2Msg& cloud, int pointIndex, const Rosbag::PointField& field, double* value)
{
    const char* bytes = nullptr;
    if (field.datatype != 8 || !readPointBytes(cloud, pointIndex, field, &bytes)) {
        return false;
    }
    std::memcpy(value, bytes, sizeof(double));
    return true;
}

bool readPointU8(const Rosbag::PointCloud2Msg& cloud, int pointIndex, const Rosbag::PointField& field, uint8_t* value)
{
    const char* bytes = nullptr;
    if (field.datatype != 2 || !readPointBytes(cloud, pointIndex, field, &bytes)) {
        return false;
    }
    *value = uint8_t(*bytes);
    return true;
}

uint8_t reflectivityFromFloat(float value)
{
    if (!std::isfinite(value)) {
        return 0;
    }
    return uint8_t(std::clamp<int>(int(std::lround(value)), 0, 255));
}

int64_t pointCloud2PointCount(const Rosbag::PointCloud2Msg& cloud)
{
    return int64_t(cloud.width) * int64_t(std::max<uint32_t>(1, cloud.height));
}

bool buildPointCloud2Frame(const Rosbag::PointCloud2Msg& cloud,
                           const QString& filePath,
                           uint64_t sequence,
                           int64_t fallbackFrameStartNs,
                           int64_t nextFrameStartNs,
                           bool useDriver2Timestamp,
                           int fallbackFrameDurationMs,
                           SlamInputFrame* frame,
                           QString* error)
{
    const Rosbag::PointField* xField = pointFieldByName(cloud, QStringLiteral("x"));
    const Rosbag::PointField* yField = pointFieldByName(cloud, QStringLiteral("y"));
    const Rosbag::PointField* zField = pointFieldByName(cloud, QStringLiteral("z"));
    const Rosbag::PointField* intensityField = pointFieldByName(cloud, QStringLiteral("intensity"));
    const Rosbag::PointField* tagField = pointFieldByName(cloud, QStringLiteral("tag"));
    const Rosbag::PointField* lineField = pointFieldByName(cloud, QStringLiteral("line"));
    const Rosbag::PointField* timestampField = pointFieldByName(cloud, QStringLiteral("timestamp"));
    if (xField == nullptr || yField == nullptr || zField == nullptr ||
        intensityField == nullptr || tagField == nullptr || lineField == nullptr ||
        (useDriver2Timestamp && timestampField == nullptr)) {
        if (error != nullptr) {
            *error = QStringLiteral("PointCloud2 缺少 Livox 必需字段。");
        }
        return false;
    }

    const int64_t pointCount64 = pointCloud2PointCount(cloud);
    if (pointCount64 <= 0 || pointCount64 > std::numeric_limits<int>::max()) {
        if (error != nullptr) {
            *error = QStringLiteral("PointCloud2 点数无效：%1。").arg(pointCount64);
        }
        return false;
    }
    const int pointCount = int(pointCount64);
    const qsizetype requiredBytes = qsizetype(pointCount) * qsizetype(cloud.pointStep);
    if (requiredBytes > cloud.data.size()) {
        if (error != nullptr) {
            *error = QStringLiteral("PointCloud2 data 长度不足：需要 %1 字节，实际 %2 字节。")
                         .arg(requiredBytes)
                         .arg(cloud.data.size());
        }
        return false;
    }

    const int64_t frameStartNs = cloud.header.stampNs > 0 ? cloud.header.stampNs : fallbackFrameStartNs;
    const int64_t fallbackFrameDurationNs =
        static_cast<int64_t>(std::max<int>(1, fallbackFrameDurationMs)) * int64_t{1000000};
    int64_t synthesizedFrameDurationNs = nextFrameStartNs > frameStartNs
        ? nextFrameStartNs - frameStartNs
        : fallbackFrameDurationNs;

    *frame = SlamInputFrame();
    frame->sequence = sequence;
    frame->sourceId = 0;
    frame->deviceType = 0;
    frame->frameStartNs = frameStartNs;
    frame->frameEndNs = frameStartNs;
    frame->timeSource = useDriver2Timestamp
        ? SlamTimeSource::LivoxPacketTimestamp
        : SlamTimeSource::SynthesizedFromPacketInterval;
    frame->hasPointOffsetTime = true;
    frame->sourceName = filePath;
    frame->points.reserve(pointCount);

    for (int i = 0; i < pointCount; ++i) {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float intensity = 0.0f;
        uint8_t tag = 0;
        uint8_t line = 0;
        if (!readPointFloat(cloud, i, *xField, &x) ||
            !readPointFloat(cloud, i, *yField, &y) ||
            !readPointFloat(cloud, i, *zField, &z) ||
            !readPointFloat(cloud, i, *intensityField, &intensity) ||
            !readPointU8(cloud, i, *tagField, &tag) ||
            !readPointU8(cloud, i, *lineField, &line)) {
            if (error != nullptr) {
                *error = QStringLiteral("PointCloud2 第 %1 个点读取失败。").arg(i);
            }
            return false;
        }

        int64_t offsetNs = 0;
        if (useDriver2Timestamp) {
            double absoluteTimestamp = 0.0;
            if (!readPointDouble(cloud, i, *timestampField, &absoluteTimestamp) ||
                !std::isfinite(absoluteTimestamp)) {
                if (error != nullptr) {
                    *error = QStringLiteral("PointCloud2 第 %1 个点 timestamp 无效。").arg(i);
                }
                return false;
            }
            offsetNs = int64_t(std::llround(absoluteTimestamp)) - frameStartNs;
            if (offsetNs < 0 && offsetNs >= -kPointCloud2TimestampRoundingToleranceNs) {
                offsetNs = 0;
            }
            if (offsetNs < 0 || offsetNs > 10000000000LL) {
                if (error != nullptr) {
                    *error = QStringLiteral("PointCloud2 第 %1 个点 timestamp 与 header.stamp 不匹配。").arg(i);
                }
                return false;
            }
        } else {
            const int64_t divisor = static_cast<int64_t>(std::max<int>(1, pointCount));
            offsetNs = int64_t(i) * synthesizedFrameDurationNs / divisor;
        }

        SlamPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.reflectivity = reflectivityFromFloat(intensity);
        point.tag = tag;
        point.line = line;
        point.hasLine = true;
        point.offsetNs = offsetNs;
        point.hasOffsetTime = true;
        frame->frameEndNs = std::max<int64_t>(frame->frameEndNs, frameStartNs + offsetNs);
        frame->points.push_back(point);
    }
    return true;
}

SlamImuSample toSlamImuSample(const Rosbag::ImuMsg& source, int64_t offsetNs)
{
    SlamImuSample sample;
    sample.timestampNs = source.header.stampNs + offsetNs;
    sample.gyroRadPerSec[0] = source.angularVelocity[0];
    sample.gyroRadPerSec[1] = source.angularVelocity[1];
    sample.gyroRadPerSec[2] = source.angularVelocity[2];
    sample.accelRaw[0] = source.linearAcceleration[0];
    sample.accelRaw[1] = source.linearAcceleration[1];
    sample.accelRaw[2] = source.linearAcceleration[2];
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

void finalizeImuTimestampRange(const QVector<SlamImuSample>& imuSamples, RosbagSlamSourceSummary& summary)
{
    if (imuSamples.isEmpty()) {
        return;
    }
    summary.imuStartTimestampNs = imuSamples.first().timestampNs;
    summary.imuEndTimestampNs = imuSamples.first().timestampNs;
    for (const SlamImuSample& sample : imuSamples) {
        summary.imuStartTimestampNs = std::min(summary.imuStartTimestampNs, sample.timestampNs);
        summary.imuEndTimestampNs = std::max(summary.imuEndTimestampNs, sample.timestampNs);
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
    const QString suffix = QFileInfo(filePath).suffix().toLower();
    const bool useRos2Reader = suffix == QStringLiteral("db3") ||
                               suffix == QStringLiteral("yaml") ||
                               suffix == QStringLiteral("yml");
    summary_.format = useRos2Reader ? QStringLiteral("ROS2 db3") : QStringLiteral("ROS1 bag");

    QVector<Rosbag::Connection> connections;
    QVector<Rosbag::SerializedMessage> messages;
    if (useRos2Reader) {
        Rosbag::Ros2BagReader reader;
        if (!reader.read(filePath, &errorMessage_)) {
            if (error != nullptr) {
                *error = errorMessage_;
            }
            return false;
        }
        connections = reader.connections();
        messages = reader.messages();
    } else {
        Rosbag::Reader reader;
        if (!reader.read(filePath, &errorMessage_)) {
            if (error != nullptr) {
                *error = errorMessage_;
            }
            return false;
        }
        connections = reader.connections();
        messages = reader.messages();
    }

    if (messages.isEmpty()) {
        errorMessage_ = QStringLiteral("%1 加载失败：未找到 message data。").arg(summary_.format);
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }

    for (const Rosbag::Connection& connection : connections) {
        RosbagSlamTopicInfo topic;
        topic.topic = connection.topic;
        topic.type = connection.type;
        topic.messageCount = connection.messageCount;
        summary_.topics.push_back(topic);
    }

    const Rosbag::Connection* lidarConnection = config_.lidarTopic.isEmpty()
        ? autoSelectLidarConnection(connections)
        : findConnectionByTopic(connections, config_.lidarTopic);
    if (lidarConnection == nullptr) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：未找到 LiDAR topic。已发现 topic: %1。").arg(topicListText(summary_.topics));
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }

    const Rosbag::Connection* imuConnection = config_.imuTopic.isEmpty()
        ? autoSelectImuConnection(connections)
        : findConnectionByTopic(connections, config_.imuTopic);
    if (imuConnection == nullptr && config_.requireImu) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：未找到 IMU topic。已发现 topic: %1。").arg(topicListText(summary_.topics));
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }

    summary_.lidarTopic = lidarConnection->topic;
    summary_.lidarType = lidarConnection->type;
    if (imuConnection != nullptr) {
        summary_.imuTopic = imuConnection->topic;
        summary_.imuType = imuConnection->type;
    }

    const bool lidarIsCustomMsg = isLivoxCustomMsgType(lidarConnection->type);
    const bool lidarIsPointCloud2 = isPointCloud2Type(lidarConnection->type);
    if (!lidarIsCustomMsg && !lidarIsPointCloud2) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 类型 %2 不受支持。当前支持 Livox CustomMsg 和 Livox PointCloud2。")
                            .arg(lidarConnection->topic, lidarConnection->type);
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }

    QVector<SlamImuSample> imuSamples;
    QVector<const Rosbag::SerializedMessage*> lidarMessages;
    uint64_t nextSequence = 0;

    for (const Rosbag::SerializedMessage& message : messages) {
        if (imuConnection != nullptr && message.connectionId == imuConnection->id) {
            Rosbag::ImuMsg imu;
            QString parseError;
            if (!parseImuMessage(imuConnection->type, message.data, &imu, &parseError)) {
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

        if (message.connectionId == lidarConnection->id) {
            lidarMessages.push_back(&message);
        }
    }

    if (lidarIsCustomMsg) {
        summary_.lidarFormat = QStringLiteral("Livox CustomMsg");
        summary_.pointTimeMode = QStringLiteral("CustomMsg offset_time(ns)");

        for (const Rosbag::SerializedMessage* message : lidarMessages) {
            Rosbag::LivoxCustomMsg livoxMsg;
            QString parseError;
            if (!parseLivoxCustomMessage(lidarConnection->type, message->data, &livoxMsg, &parseError)) {
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
                frameStartNs = message->timestampNs;
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
                frame.frameEndNs = std::max<int64_t>(frame.frameEndNs, frameStartNs + int64_t(point.offsetNs));
                frame.points.push_back(point);
            }
            summary_.pointCount += uint64_t(frame.points.size());
            frames_.push_back(std::move(frame));
        }
    } else {
        enum class PointCloud2Mode {
            Unknown,
            Driver2Timestamp,
            DriverSynthesized
        };
        PointCloud2Mode pointCloud2Mode = PointCloud2Mode::Unknown;

        auto nextPointCloud2FrameStartNs = [&](int index) {
            for (int i = index + 1; i < lidarMessages.size(); ++i) {
                Rosbag::PointCloud2Msg nextCloud;
                QString ignoredError;
                if (!parsePointCloud2Message(lidarConnection->type, lidarMessages.at(i)->data, &nextCloud, &ignoredError)) {
                    continue;
                }
                if (pointCloud2PointCount(nextCloud) <= 0) {
                    continue;
                }
                return nextCloud.header.stampNs > 0 ? nextCloud.header.stampNs : lidarMessages.at(i)->timestampNs;
            }
            return int64_t(0);
        };

        for (int i = 0; i < lidarMessages.size(); ++i) {
            const Rosbag::SerializedMessage* message = lidarMessages.at(i);
            Rosbag::PointCloud2Msg cloud;
            QString parseError;
            if (!parsePointCloud2Message(lidarConnection->type, message->data, &cloud, &parseError)) {
                errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 PointCloud2 解析失败：%2")
                                    .arg(lidarConnection->topic, parseError);
                if (error != nullptr) {
                    *error = errorMessage_;
                }
                return false;
            }

            summary_.lidarMessageCount++;
            if (pointCloud2PointCount(cloud) <= 0) {
                summary_.emptyLidarMessageCount++;
                continue;
            }

            if (pointCloud2Mode == PointCloud2Mode::Unknown) {
                if (isLivoxDriver2PointCloud2Layout(cloud)) {
                    if (!config_.allowLivoxDriver2PointCloud2) {
                        errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 是 livox_ros_driver2 PointCloud2，但当前运行配置未允许该格式。")
                                            .arg(lidarConnection->topic);
                        if (error != nullptr) {
                            *error = errorMessage_;
                        }
                        return false;
                    }
                    pointCloud2Mode = PointCloud2Mode::Driver2Timestamp;
                    summary_.lidarFormat = QStringLiteral("Livox driver2 PointCloud2");
                    summary_.pointTimeMode = QStringLiteral("PointCloud2 absolute timestamp(ns)");
                } else if (isLivoxDriverPointCloud2Layout(cloud)) {
                    if (!config_.allowLivoxDriverPointCloud2SynthesizedTime &&
                        !config_.synthesizePointOffsetTime) {
                        errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 是 livox_ros_driver PointCloud2，但消息不包含真实点内时间，且当前运行配置未允许合成点内时间。")
                                            .arg(lidarConnection->topic);
                        if (error != nullptr) {
                            *error = errorMessage_;
                        }
                        return false;
                    }
                    pointCloud2Mode = PointCloud2Mode::DriverSynthesized;
                    summary_.lidarFormat = QStringLiteral("Livox driver PointCloud2");
                    summary_.pointTimeMode = QStringLiteral("Synthesized offset_time(ns)");
                    summary_.messages.push_back(QStringLiteral("ROSbag 提示：livox_ros_driver PointCloud2 不包含真实点内时间，已按帧周期合成 offset_time，建图精度可能低于 CustomMsg。"));
                } else {
                    errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 是 sensor_msgs/PointCloud2，但字段不匹配已支持的 Livox 布局。driver2 需要 x/y/z/intensity/tag/line/timestamp；driver1 需要 x/y/z/intensity/tag/line。")
                                        .arg(lidarConnection->topic);
                    if (error != nullptr) {
                        *error = errorMessage_;
                    }
                    return false;
                }
            }

            const bool useDriver2Timestamp = pointCloud2Mode == PointCloud2Mode::Driver2Timestamp;
            if ((useDriver2Timestamp && !isLivoxDriver2PointCloud2Layout(cloud)) ||
                (!useDriver2Timestamp && !isLivoxDriverPointCloud2Layout(cloud))) {
                errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 的 PointCloud2 字段布局在同一 bag 中发生变化。")
                                    .arg(lidarConnection->topic);
                if (error != nullptr) {
                    *error = errorMessage_;
                }
                return false;
            }

            SlamInputFrame frame;
            const int64_t nextFrameStartNs = useDriver2Timestamp ? 0 : nextPointCloud2FrameStartNs(i);
            if (!buildPointCloud2Frame(cloud,
                                       filePath,
                                       nextSequence++,
                                       message->timestampNs,
                                       nextFrameStartNs,
                                       useDriver2Timestamp,
                                       config_.frameDurationMs,
                                       &frame,
                                       &parseError)) {
                errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 PointCloud2 转换失败：%2")
                                    .arg(lidarConnection->topic, parseError);
                if (error != nullptr) {
                    *error = errorMessage_;
                }
                return false;
            }
            summary_.pointCount += uint64_t(frame.points.size());
            frames_.push_back(std::move(frame));
        }
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
    imuSamples_ = imuSamples;
    finalizeTimestampRange(frames_, summary_);
    finalizeImuTimestampRange(imuSamples, summary_);

    if (frames_.isEmpty()) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 未生成有效 SLAM 输入帧。").arg(lidarConnection->topic);
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }
    if (!summary_.hasImu && config_.requireImu) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：IMU topic %1 没有可用 sensor_msgs/Imu 样本。").arg(imuConnection->topic);
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }

    if (!imuSamples.isEmpty()) {
        attachImuSamples(frames_, imuSamples, summary_);
    } else {
        summary_.messages.push_back(QStringLiteral("ROSbag 未包含 IMU topic，普通点云播放将仅显示点云。"));
    }
    if (config_.requireImu && summary_.framesWithCompleteImuCoverage == 0) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：IMU 样本未覆盖任何 LiDAR 帧。LiDAR 时间范围(ns): %1 - %2，IMU 时间范围(ns): %3 - %4。请检查 %5 topic、时间戳和 LiDAR/IMU 时间偏移配置。")
                            .arg(QString::number(summary_.startTimestampNs),
                                 QString::number(summary_.endTimestampNs),
                                 QString::number(summary_.imuStartTimestampNs),
                                 QString::number(summary_.imuEndTimestampNs),
                                 imuConnection->topic);
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }
    if (config_.requireImu && summary_.framesWithCompleteImuCoverage < summary_.frameCount) {
        summary_.messages.push_back(QStringLiteral("ROSbag IMU 样本未完整覆盖所有 SLAM 输入帧，未覆盖帧将在离线 worker 中跳过。"));
    } else if (!config_.requireImu && summary_.hasImu && summary_.framesWithCompleteImuCoverage < summary_.frameCount) {
        summary_.messages.push_back(QStringLiteral("ROSbag IMU 样本未完整覆盖所有点云帧，普通播放仍会显示点云。"));
    }
    return true;
}

void RosbagSlamSource::clear()
{
    frames_.clear();
    imuSamples_.clear();
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

const QVector<SlamImuSample>& RosbagSlamSource::imuSamples() const
{
    return imuSamples_;
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
    lines << QStringLiteral("- IMU 时间范围(ns): %1 - %2")
                 .arg(QString::number(summary_.imuStartTimestampNs), QString::number(summary_.imuEndTimestampNs));
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

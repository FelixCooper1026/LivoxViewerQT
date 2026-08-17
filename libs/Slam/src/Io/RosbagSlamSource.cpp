#include "Io/RosbagSlamSource.h"

#include "RosMessageParsers.h"
#include "McapReader.h"
#include "Ros2BagReader.h"
#include "RosbagReader.h"
#include "Core/FastLioInputSynchronizer.h"

#include <QFileInfo>
#include <QSet>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {

constexpr int64_t kPointCloud2TimestampRoundingToleranceNs = int64_t{1000000};
constexpr uint8_t kPointFieldUint8 = 2;
constexpr uint8_t kPointFieldUint16 = 4;
constexpr uint8_t kPointFieldFloat32 = 7;
constexpr uint8_t kPointFieldFloat64 = 8;

enum class PointCloud2FrameTiming {
    Driver2AbsoluteNs,
    RelativeSeconds,
    RelativeMilliseconds,
    Synthesized
};

enum class PointCloud2PointLayout {
    Livox,
    GenericTimed,
    GenericXyz
};

bool isLivoxCustomMsgType(const QString& type)
{
    return type == QStringLiteral("livox_ros_driver2/CustomMsg") ||
           type == QStringLiteral("livox_ros_driver/CustomMsg") ||
           type == QStringLiteral("livox_msgs/CustomMsg") ||
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

QString mcapFormatName(const QVector<Rosbag::Connection>& connections)
{
    bool hasRos1 = false;
    bool hasRos2 = false;
    bool hasOther = false;
    for (const Rosbag::Connection& connection : connections) {
        const QString schemaEncoding = connection.schemaEncoding.toLower();
        const QString messageEncoding = connection.messageEncoding.toLower();
        if (messageEncoding == QStringLiteral("ros1") || schemaEncoding == QStringLiteral("ros1msg")) {
            hasRos1 = true;
        } else if (messageEncoding == QStringLiteral("cdr") || schemaEncoding.startsWith(QStringLiteral("ros2"))) {
            hasRos2 = true;
        } else {
            hasOther = true;
        }
    }
    if (hasRos1 && !hasRos2 && !hasOther) {
        return QStringLiteral("ROS1 MCAP");
    }
    if (hasRos2 && !hasRos1 && !hasOther) {
        return QStringLiteral("ROS2 MCAP");
    }
    return QStringLiteral("MCAP");
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

bool pointFieldFits(const Rosbag::PointCloud2Msg& cloud, const Rosbag::PointField& field)
{
    if (field.count != 1) {
        return false;
    }
    const qsizetype size = pointFieldDatatypeSize(field.datatype);
    return size > 0 &&
           field.offset <= cloud.pointStep &&
           size <= qsizetype(cloud.pointStep) - qsizetype(field.offset);
}

bool pointFieldMatchesTypeAndFits(const Rosbag::PointCloud2Msg& cloud, const QString& name, uint8_t datatype)
{
    const Rosbag::PointField* field = pointFieldByName(cloud, name);
    if (field == nullptr || field->datatype != datatype) {
        return false;
    }
    return pointFieldFits(cloud, *field);
}

bool pointFieldMatchesFloatAndFits(const Rosbag::PointCloud2Msg& cloud, const QString& name)
{
    return pointFieldMatchesTypeAndFits(cloud, name, kPointFieldFloat32) ||
           pointFieldMatchesTypeAndFits(cloud, name, kPointFieldFloat64);
}

bool pointFieldMatchesLineAndFits(const Rosbag::PointCloud2Msg& cloud, const QString& name)
{
    return pointFieldMatchesTypeAndFits(cloud, name, kPointFieldUint8) ||
           pointFieldMatchesTypeAndFits(cloud, name, kPointFieldUint16);
}

bool hasPointCloud2ReadableShape(const Rosbag::PointCloud2Msg& cloud)
{
    const uint64_t rowPointBytes = uint64_t(cloud.width) * uint64_t(cloud.pointStep);
    return !cloud.isBigEndian &&
           cloud.width > 0 &&
           cloud.height > 0 &&
           cloud.pointStep > 0 &&
           cloud.rowStep >= rowPointBytes;
}

bool isLivoxDriver2PointCloud2Layout(const Rosbag::PointCloud2Msg& cloud)
{
    return hasPointCloud2ReadableShape(cloud) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("x"), kPointFieldFloat32) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("y"), kPointFieldFloat32) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("z"), kPointFieldFloat32) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("intensity"), kPointFieldFloat32) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("tag"), kPointFieldUint8) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("line"), kPointFieldUint8) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("timestamp"), kPointFieldFloat64);
}

bool isLivoxDriverPointCloud2Layout(const Rosbag::PointCloud2Msg& cloud)
{
    return hasPointCloud2ReadableShape(cloud) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("x"), kPointFieldFloat32) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("y"), kPointFieldFloat32) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("z"), kPointFieldFloat32) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("intensity"), kPointFieldFloat32) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("tag"), kPointFieldUint8) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("line"), kPointFieldUint8) &&
           !hasPointField(cloud, QStringLiteral("timestamp")) &&
           !hasPointField(cloud, QStringLiteral("time")) &&
           !hasPointField(cloud, QStringLiteral("offset_time"));
}

bool isGenericTimedPointCloud2Layout(const Rosbag::PointCloud2Msg& cloud)
{
    return hasPointCloud2ReadableShape(cloud) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("x"), kPointFieldFloat32) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("y"), kPointFieldFloat32) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("z"), kPointFieldFloat32) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("intensity"), kPointFieldFloat32) &&
           (pointFieldMatchesLineAndFits(cloud, QStringLiteral("ring")) ||
            pointFieldMatchesLineAndFits(cloud, QStringLiteral("line"))) &&
           pointFieldMatchesFloatAndFits(cloud, QStringLiteral("time"));
}

bool isGenericXyzPointCloud2Layout(const Rosbag::PointCloud2Msg& cloud)
{
    return hasPointCloud2ReadableShape(cloud) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("x"), kPointFieldFloat32) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("y"), kPointFieldFloat32) &&
           pointFieldMatchesTypeAndFits(cloud, QStringLiteral("z"), kPointFieldFloat32);
}

bool readPointBytes(const Rosbag::PointCloud2Msg& cloud, int pointIndex, const Rosbag::PointField& field, const char** out)
{
    const uint32_t row = uint32_t(pointIndex) / cloud.width;
    const uint32_t column = uint32_t(pointIndex) % cloud.width;
    const uint64_t offset = uint64_t(row) * uint64_t(cloud.rowStep) +
                            uint64_t(column) * uint64_t(cloud.pointStep) +
                            uint64_t(field.offset);
    const uint64_t size = uint64_t(pointFieldDatatypeSize(field.datatype));
    if (size == 0 || offset > uint64_t(cloud.data.size()) || size > uint64_t(cloud.data.size()) - offset) {
        return false;
    }
    *out = cloud.data.constData() + qsizetype(offset);
    return true;
}

bool readPointFloat(const Rosbag::PointCloud2Msg& cloud, int pointIndex, const Rosbag::PointField& field, float* value)
{
    const char* bytes = nullptr;
    if (field.datatype != kPointFieldFloat32 || !readPointBytes(cloud, pointIndex, field, &bytes)) {
        return false;
    }
    std::memcpy(value, bytes, sizeof(float));
    return true;
}

bool readPointDouble(const Rosbag::PointCloud2Msg& cloud, int pointIndex, const Rosbag::PointField& field, double* value)
{
    const char* bytes = nullptr;
    if (field.datatype != kPointFieldFloat64 || !readPointBytes(cloud, pointIndex, field, &bytes)) {
        return false;
    }
    std::memcpy(value, bytes, sizeof(double));
    return true;
}

bool readPointU8(const Rosbag::PointCloud2Msg& cloud, int pointIndex, const Rosbag::PointField& field, uint8_t* value)
{
    const char* bytes = nullptr;
    if (field.datatype != kPointFieldUint8 || !readPointBytes(cloud, pointIndex, field, &bytes)) {
        return false;
    }
    *value = uint8_t(*bytes);
    return true;
}

bool readPointU16(const Rosbag::PointCloud2Msg& cloud, int pointIndex, const Rosbag::PointField& field, uint16_t* value)
{
    const char* bytes = nullptr;
    if (field.datatype != kPointFieldUint16 || !readPointBytes(cloud, pointIndex, field, &bytes)) {
        return false;
    }
    std::memcpy(value, bytes, sizeof(uint16_t));
    return true;
}

bool readPointLine(const Rosbag::PointCloud2Msg& cloud, int pointIndex, const Rosbag::PointField& field, uint8_t* value)
{
    if (field.datatype == kPointFieldUint8) {
        return readPointU8(cloud, pointIndex, field, value);
    }
    uint16_t line = 0;
    if (field.datatype != kPointFieldUint16 || !readPointU16(cloud, pointIndex, field, &line)) {
        return false;
    }
    *value = uint8_t(std::min<uint16_t>(line, 255));
    return true;
}

bool readPointFloatOrDouble(const Rosbag::PointCloud2Msg& cloud, int pointIndex, const Rosbag::PointField& field, double* value)
{
    if (field.datatype == kPointFieldFloat64) {
        return readPointDouble(cloud, pointIndex, field, value);
    }
    float floatValue = 0.0f;
    if (field.datatype != kPointFieldFloat32 || !readPointFloat(cloud, pointIndex, field, &floatValue)) {
        return false;
    }
    *value = double(floatValue);
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
    return int64_t(cloud.width) * int64_t(cloud.height);
}

uint64_t pointCloud2RequiredDataBytes(const Rosbag::PointCloud2Msg& cloud)
{
    if (cloud.height == 0) {
        return 0;
    }
    return uint64_t(cloud.rowStep) * uint64_t(cloud.height - 1) +
           uint64_t(cloud.pointStep) * uint64_t(cloud.width);
}

bool inferGenericPointCloud2Timing(const Rosbag::PointCloud2Msg& cloud,
                                   PointCloud2FrameTiming* timing,
                                   QString* error)
{
    const Rosbag::PointField* timeField = pointFieldByName(cloud, QStringLiteral("time"));
    if (timeField == nullptr) {
        if (error != nullptr) {
            *error = QStringLiteral("PointCloud2 缺少 time 字段。");
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

    double maxTime = 0.0;
    for (int i = 0; i < int(pointCount64); ++i) {
        double timeValue = 0.0;
        if (!readPointFloatOrDouble(cloud, i, *timeField, &timeValue) ||
            !std::isfinite(timeValue) ||
            timeValue < 0.0) {
            if (error != nullptr) {
                *error = QStringLiteral("PointCloud2 第 %1 个点 time 无效。").arg(i);
            }
            return false;
        }
        maxTime = std::max(maxTime, timeValue);
    }

    *timing = maxTime > 1.0
        ? PointCloud2FrameTiming::RelativeMilliseconds
        : PointCloud2FrameTiming::RelativeSeconds;
    return true;
}

bool buildPointCloud2Frame(const Rosbag::PointCloud2Msg& cloud,
                           const QString& filePath,
                           uint64_t sequence,
                           int64_t fallbackFrameStartNs,
                           int64_t nextFrameStartNs,
                           PointCloud2PointLayout pointLayout,
                           PointCloud2FrameTiming timing,
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
    if (lineField == nullptr) {
        lineField = pointFieldByName(cloud, QStringLiteral("ring"));
    }
    const Rosbag::PointField* timestampField = pointFieldByName(cloud, QStringLiteral("timestamp"));
    const Rosbag::PointField* timeField = pointFieldByName(cloud, QStringLiteral("time"));
    const bool useDriver2Timestamp = timing == PointCloud2FrameTiming::Driver2AbsoluteNs;
    const bool useRelativeTime = timing == PointCloud2FrameTiming::RelativeSeconds ||
                                 timing == PointCloud2FrameTiming::RelativeMilliseconds;
    const bool useGenericXyz = pointLayout == PointCloud2PointLayout::GenericXyz;
    const bool requireLivoxFields = pointLayout == PointCloud2PointLayout::Livox;
    const bool requireGenericTimedFields = pointLayout == PointCloud2PointLayout::GenericTimed;
    if (xField == nullptr || yField == nullptr || zField == nullptr ||
        ((requireLivoxFields || requireGenericTimedFields) && intensityField == nullptr) ||
        ((requireLivoxFields || requireGenericTimedFields) && lineField == nullptr) ||
        (requireLivoxFields && tagField == nullptr) ||
        (useDriver2Timestamp && timestampField == nullptr) ||
        (useRelativeTime && timeField == nullptr) ||
        (timing == PointCloud2FrameTiming::Synthesized && requireLivoxFields && tagField == nullptr)) {
        if (error != nullptr) {
            *error = QStringLiteral("PointCloud2 缺少当前布局必需字段。");
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
    const uint64_t requiredBytes = pointCloud2RequiredDataBytes(cloud);
    if (requiredBytes > uint64_t(cloud.data.size())) {
        if (error != nullptr) {
            *error = QStringLiteral("PointCloud2 data 长度不足：需要 %1 字节，实际 %2 字节。")
                         .arg(QString::number(requiredBytes))
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
    frame->timeSource = timing == PointCloud2FrameTiming::Synthesized
        ? SlamTimeSource::SynthesizedFromPacketInterval
        : SlamTimeSource::LivoxPacketTimestamp;
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
            !readPointFloat(cloud, i, *zField, &z)) {
            if (error != nullptr) {
                *error = QStringLiteral("PointCloud2 第 %1 个点读取失败。").arg(i);
            }
            return false;
        }
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }
        if (intensityField != nullptr && !readPointFloat(cloud, i, *intensityField, &intensity)) {
            if (error != nullptr) {
                *error = QStringLiteral("PointCloud2 第 %1 个点 intensity 读取失败。").arg(i);
            }
            return false;
        }
        if (lineField != nullptr && !readPointLine(cloud, i, *lineField, &line)) {
            if (error != nullptr) {
                *error = QStringLiteral("PointCloud2 第 %1 个点 line/ring 读取失败。").arg(i);
            }
            return false;
        }
        if (tagField != nullptr && !readPointU8(cloud, i, *tagField, &tag)) {
            if (error != nullptr) {
                *error = QStringLiteral("PointCloud2 第 %1 个点 tag 读取失败。").arg(i);
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
        } else if (useRelativeTime) {
            double relativeTime = 0.0;
            if (!readPointFloatOrDouble(cloud, i, *timeField, &relativeTime) ||
                !std::isfinite(relativeTime) ||
                relativeTime < 0.0) {
                if (error != nullptr) {
                    *error = QStringLiteral("PointCloud2 第 %1 个点 time 无效。").arg(i);
                }
                return false;
            }
            const double scale = timing == PointCloud2FrameTiming::RelativeMilliseconds
                ? 1000000.0
                : 1000000000.0;
            offsetNs = int64_t(std::llround(relativeTime * scale));
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
        point.hasLine = !useGenericXyz || lineField != nullptr;
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
    const bool useMcapReader = Rosbag::isMcapPath(filePath);
    const bool useRos2Reader = !useMcapReader &&
                               (suffix == QStringLiteral("db3") ||
                                suffix == QStringLiteral("yaml") ||
                                suffix == QStringLiteral("yml"));
    summary_.format = useMcapReader
        ? QStringLiteral("MCAP")
        : (useRos2Reader ? QStringLiteral("ROS2 db3") : QStringLiteral("ROS1 bag"));

    QVector<Rosbag::Connection> connections;
    QVector<Rosbag::SerializedMessage> messages;
    if (useMcapReader) {
        Rosbag::McapReader reader;
        if (!reader.read(filePath, &errorMessage_)) {
            if (error != nullptr) {
                *error = errorMessage_;
            }
            return false;
        }
        connections = reader.connections();
        messages = reader.messages();
        summary_.format = mcapFormatName(connections);
    } else if (useRos2Reader) {
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
            DriverSynthesized,
            GenericRelativeSeconds,
            GenericRelativeMilliseconds,
            GenericSynthesized
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
                } else if (isGenericTimedPointCloud2Layout(cloud)) {
                    PointCloud2FrameTiming genericTiming = PointCloud2FrameTiming::Synthesized;
                    QString timingError;
                    if (!inferGenericPointCloud2Timing(cloud, &genericTiming, &timingError)) {
                        errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 通用 PointCloud2 time 字段解析失败：%2")
                                            .arg(lidarConnection->topic, timingError);
                        if (error != nullptr) {
                            *error = errorMessage_;
                        }
                        return false;
                    }
                    if (genericTiming == PointCloud2FrameTiming::RelativeMilliseconds) {
                        pointCloud2Mode = PointCloud2Mode::GenericRelativeMilliseconds;
                        summary_.pointTimeMode = QStringLiteral("PointCloud2 relative time(ms)");
                    } else {
                        pointCloud2Mode = PointCloud2Mode::GenericRelativeSeconds;
                        summary_.pointTimeMode = QStringLiteral("PointCloud2 relative time(s)");
                    }
                    summary_.lidarFormat = QStringLiteral("Generic PointCloud2 x/y/z/intensity/ring/time");
                } else if (isGenericXyzPointCloud2Layout(cloud)) {
                    if (!config_.allowLivoxDriverPointCloud2SynthesizedTime &&
                        !config_.synthesizePointOffsetTime) {
                        errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 是通用 PointCloud2 x/y/z，但消息不包含点内时间，且当前运行配置未允许合成点内时间。")
                                            .arg(lidarConnection->topic);
                        if (error != nullptr) {
                            *error = errorMessage_;
                        }
                        return false;
                    }
                    pointCloud2Mode = PointCloud2Mode::GenericSynthesized;
                    summary_.lidarFormat = QStringLiteral("Generic PointCloud2 x/y/z");
                    summary_.pointTimeMode = QStringLiteral("Synthesized offset_time(ns)");
                    summary_.messages.push_back(QStringLiteral("ROSbag 提示：通用 PointCloud2 仅包含 x/y/z，已按帧周期合成 offset_time，并使用默认强度和线号；该模式用于点云播放，离线 SLAM 仍需要有效 IMU。"));
                } else {
                    errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 是 sensor_msgs/PointCloud2，但字段不匹配已支持的布局。driver2 需要 x/y/z/intensity/tag/line/timestamp；driver1 需要 x/y/z/intensity/tag/line；通用 timed PointCloud2 需要 x/y/z/intensity/ring/time；通用播放 PointCloud2 至少需要 x/y/z。")
                                        .arg(lidarConnection->topic);
                    if (error != nullptr) {
                        *error = errorMessage_;
                    }
                    return false;
                }
            }

            if ((pointCloud2Mode == PointCloud2Mode::Driver2Timestamp && !isLivoxDriver2PointCloud2Layout(cloud)) ||
                (pointCloud2Mode == PointCloud2Mode::DriverSynthesized && !isLivoxDriverPointCloud2Layout(cloud)) ||
                ((pointCloud2Mode == PointCloud2Mode::GenericRelativeSeconds ||
                  pointCloud2Mode == PointCloud2Mode::GenericRelativeMilliseconds) &&
                 !isGenericTimedPointCloud2Layout(cloud)) ||
                (pointCloud2Mode == PointCloud2Mode::GenericSynthesized &&
                 !isGenericXyzPointCloud2Layout(cloud))) {
                errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 的 PointCloud2 字段布局在同一 bag 中发生变化。")
                                    .arg(lidarConnection->topic);
                if (error != nullptr) {
                    *error = errorMessage_;
                }
                return false;
            }

            PointCloud2FrameTiming frameTiming = PointCloud2FrameTiming::Synthesized;
            if (pointCloud2Mode == PointCloud2Mode::Driver2Timestamp) {
                frameTiming = PointCloud2FrameTiming::Driver2AbsoluteNs;
            } else if (pointCloud2Mode == PointCloud2Mode::GenericRelativeSeconds) {
                frameTiming = PointCloud2FrameTiming::RelativeSeconds;
            } else if (pointCloud2Mode == PointCloud2Mode::GenericRelativeMilliseconds) {
                frameTiming = PointCloud2FrameTiming::RelativeMilliseconds;
            }
            PointCloud2PointLayout pointLayout = PointCloud2PointLayout::Livox;
            if (pointCloud2Mode == PointCloud2Mode::GenericRelativeSeconds ||
                pointCloud2Mode == PointCloud2Mode::GenericRelativeMilliseconds) {
                pointLayout = PointCloud2PointLayout::GenericTimed;
            } else if (pointCloud2Mode == PointCloud2Mode::GenericSynthesized) {
                pointLayout = PointCloud2PointLayout::GenericXyz;
            }

            SlamInputFrame frame;
            const int64_t nextFrameStartNs = (pointCloud2Mode == PointCloud2Mode::DriverSynthesized ||
                                              pointCloud2Mode == PointCloud2Mode::GenericSynthesized)
                ? nextPointCloud2FrameStartNs(i)
                : 0;
            if (!buildPointCloud2Frame(cloud,
                                       filePath,
                                       nextSequence++,
                                       message->timestampNs,
                                       nextFrameStartNs,
                                       pointLayout,
                                       frameTiming,
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

    if (!imuSamples.isEmpty() && config_.requireImu) {
        frames_ = syncFastLioInputFrames(std::move(frames_), imuSamples);
        summary_.frameCount = frames_.size();
        summary_.framesWithCompleteImuCoverage = 0;
        summary_.pointCount = 0;
        for (const SlamInputFrame& frame : frames_) {
            summary_.pointCount += uint64_t(frame.points.size());
            if (frame.hasCompleteImuCoverage) {
                ++summary_.framesWithCompleteImuCoverage;
            }
        }
        summary_.hasCompleteImuCoverage =
            summary_.framesWithCompleteImuCoverage == summary_.frameCount && summary_.frameCount > 0;
    } else if (imuSamples.isEmpty()) {
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

bool RosbagSlamSource::streamFrames(const QString& filePath,
                                    const std::atomic_bool* cancellationRequested,
                                    const std::function<bool(SlamInputFrame&&)>& consumer,
                                    const std::function<void(int64_t, int64_t)>& progress,
                                    QString* error)
{
    clear();
    summary_.filePath = filePath;
    const QString suffix = QFileInfo(filePath).suffix().toLower();
    const bool useMcapReader = Rosbag::isMcapPath(filePath);
    const bool useRos2Reader = !useMcapReader &&
                               (suffix == QStringLiteral("db3") ||
                                suffix == QStringLiteral("yaml") ||
                                suffix == QStringLiteral("yml"));
    summary_.format = useMcapReader
        ? QStringLiteral("MCAP")
        : (useRos2Reader ? QStringLiteral("ROS2 db3") : QStringLiteral("ROS1 bag"));

    Rosbag::Reader ros1Reader;
    Rosbag::Ros2BagReader ros2Reader;
    Rosbag::McapReader mcapReader;
    bool metadataOk = false;
    if (useMcapReader) {
        metadataOk = mcapReader.readConnections(filePath, &errorMessage_);
    } else if (useRos2Reader) {
        metadataOk = ros2Reader.readConnections(filePath, &errorMessage_);
    } else {
        metadataOk = ros1Reader.readConnections(filePath, &errorMessage_);
    }
    if (!metadataOk) {
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }

    QVector<Rosbag::Connection> connections;
    if (useMcapReader) {
        connections = mcapReader.connections();
        summary_.format = mcapFormatName(connections);
    } else if (useRos2Reader) {
        connections = ros2Reader.connections();
    } else {
        connections = ros1Reader.connections();
    }
    for (const Rosbag::Connection& connection : connections) {
        RosbagSlamTopicInfo topic;
        topic.topic = connection.topic;
        topic.type = connection.type;
        summary_.topics.push_back(topic);
    }

    const Rosbag::Connection* lidarConnection = config_.lidarTopic.isEmpty()
        ? autoSelectLidarConnection(connections)
        : findConnectionByTopic(connections, config_.lidarTopic);
    if (lidarConnection == nullptr) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：未找到 LiDAR topic。已发现 topic: %1。")
                            .arg(topicListText(summary_.topics));
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }
    const Rosbag::Connection* imuConnection = config_.imuTopic.isEmpty()
        ? autoSelectImuConnection(connections)
        : findConnectionByTopic(connections, config_.imuTopic);
    if (imuConnection == nullptr && config_.requireImu) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：未找到 IMU topic。已发现 topic: %1。")
                            .arg(topicListText(summary_.topics));
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
        errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 类型 %2 不受支持。")
                            .arg(lidarConnection->topic, lidarConnection->type);
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }

    enum class PointCloud2Mode {
        Unknown,
        Driver2Timestamp,
        DriverSynthesized,
        GenericRelativeSeconds,
        GenericRelativeMilliseconds,
        GenericSynthesized
    };
    struct PendingPointCloud2 {
        Rosbag::PointCloud2Msg cloud;
        int64_t fallbackFrameStartNs = 0;
    };

    PointCloud2Mode pointCloud2Mode = PointCloud2Mode::Unknown;
    PendingPointCloud2 pendingCloud;
    bool hasPendingCloud = false;
    FastLioInputSynchronizer synchronizer;
    uint64_t nextSequence = 0;
    bool consumerStopped = false;

    auto updateFrameSummary = [this](const SlamInputFrame& frame) {
        if (summary_.frameCount == 0) {
            summary_.startTimestampNs = frame.frameStartNs;
            summary_.endTimestampNs = frame.frameEndNs;
        } else {
            summary_.startTimestampNs = std::min(summary_.startTimestampNs, frame.frameStartNs);
            summary_.endTimestampNs = std::max(summary_.endTimestampNs, frame.frameEndNs);
        }
        ++summary_.frameCount;
        summary_.pointCount += uint64_t(frame.points.size());
        if (frame.hasCompleteImuCoverage) {
            ++summary_.framesWithCompleteImuCoverage;
        }
        summary_.hasPointOffsetTime = summary_.hasPointOffsetTime || frame.hasPointOffsetTime;
    };
    auto emitReadyFrames = [&]() {
        SlamInputFrame ready;
        while (synchronizer.trySync(&ready)) {
            ready.sequence = uint64_t(summary_.frameCount);
            updateFrameSummary(ready);
            if (!consumer(std::move(ready))) {
                consumerStopped = true;
                return false;
            }
        }
        return true;
    };
    auto submitFrame = [&](SlamInputFrame&& frame) {
        if (config_.requireImu) {
            synchronizer.pushLidarFrame(std::move(frame));
            return emitReadyFrames();
        }
        frame.sequence = uint64_t(summary_.frameCount);
        updateFrameSummary(frame);
        if (!consumer(std::move(frame))) {
            consumerStopped = true;
            return false;
        }
        return true;
    };
    auto configurePointCloud2Mode = [&](const Rosbag::PointCloud2Msg& cloud) {
        if (isLivoxDriver2PointCloud2Layout(cloud)) {
            if (!config_.allowLivoxDriver2PointCloud2) {
                errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 是 livox_ros_driver2 PointCloud2，但当前运行配置未允许该格式。")
                                    .arg(lidarConnection->topic);
                return false;
            }
            pointCloud2Mode = PointCloud2Mode::Driver2Timestamp;
            summary_.lidarFormat = QStringLiteral("Livox driver2 PointCloud2");
            summary_.pointTimeMode = QStringLiteral("PointCloud2 absolute timestamp(ns)");
            return true;
        }
        if (isLivoxDriverPointCloud2Layout(cloud)) {
            if (!config_.allowLivoxDriverPointCloud2SynthesizedTime &&
                !config_.synthesizePointOffsetTime) {
                errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 不允许合成点内时间。")
                                    .arg(lidarConnection->topic);
                return false;
            }
            pointCloud2Mode = PointCloud2Mode::DriverSynthesized;
            summary_.lidarFormat = QStringLiteral("Livox driver PointCloud2");
            summary_.pointTimeMode = QStringLiteral("Synthesized offset_time(ns)");
            summary_.messages.push_back(QStringLiteral("ROSbag 提示：livox_ros_driver PointCloud2 已按相邻帧周期合成 offset_time。"));
            return true;
        }
        if (isGenericTimedPointCloud2Layout(cloud)) {
            PointCloud2FrameTiming timing = PointCloud2FrameTiming::Synthesized;
            QString timingError;
            if (!inferGenericPointCloud2Timing(cloud, &timing, &timingError)) {
                errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 time 字段解析失败：%2")
                                    .arg(lidarConnection->topic, timingError);
                return false;
            }
            pointCloud2Mode = timing == PointCloud2FrameTiming::RelativeMilliseconds
                ? PointCloud2Mode::GenericRelativeMilliseconds
                : PointCloud2Mode::GenericRelativeSeconds;
            summary_.lidarFormat = QStringLiteral("Generic PointCloud2 x/y/z/intensity/ring/time");
            summary_.pointTimeMode = timing == PointCloud2FrameTiming::RelativeMilliseconds
                ? QStringLiteral("PointCloud2 relative time(ms)")
                : QStringLiteral("PointCloud2 relative time(s)");
            return true;
        }
        if (isGenericXyzPointCloud2Layout(cloud)) {
            if (!config_.allowLivoxDriverPointCloud2SynthesizedTime &&
                !config_.synthesizePointOffsetTime) {
                errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 不允许合成点内时间。")
                                    .arg(lidarConnection->topic);
                return false;
            }
            pointCloud2Mode = PointCloud2Mode::GenericSynthesized;
            summary_.lidarFormat = QStringLiteral("Generic PointCloud2 x/y/z");
            summary_.pointTimeMode = QStringLiteral("Synthesized offset_time(ns)");
            summary_.messages.push_back(QStringLiteral("ROSbag 提示：通用 PointCloud2 已按相邻帧周期合成 offset_time。"));
            return true;
        }
        errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 的 PointCloud2 字段布局不受支持。")
                            .arg(lidarConnection->topic);
        return false;
    };
    auto pointCloud2LayoutMatches = [&](const Rosbag::PointCloud2Msg& cloud) {
        return (pointCloud2Mode == PointCloud2Mode::Driver2Timestamp && isLivoxDriver2PointCloud2Layout(cloud)) ||
               (pointCloud2Mode == PointCloud2Mode::DriverSynthesized && isLivoxDriverPointCloud2Layout(cloud)) ||
               ((pointCloud2Mode == PointCloud2Mode::GenericRelativeSeconds ||
                 pointCloud2Mode == PointCloud2Mode::GenericRelativeMilliseconds) &&
                isGenericTimedPointCloud2Layout(cloud)) ||
               (pointCloud2Mode == PointCloud2Mode::GenericSynthesized && isGenericXyzPointCloud2Layout(cloud));
    };
    auto submitPointCloud2 = [&](const PendingPointCloud2& pending, int64_t nextFrameStartNs) {
        PointCloud2FrameTiming timing = PointCloud2FrameTiming::Synthesized;
        PointCloud2PointLayout layout = PointCloud2PointLayout::Livox;
        if (pointCloud2Mode == PointCloud2Mode::Driver2Timestamp) {
            timing = PointCloud2FrameTiming::Driver2AbsoluteNs;
        } else if (pointCloud2Mode == PointCloud2Mode::GenericRelativeSeconds) {
            timing = PointCloud2FrameTiming::RelativeSeconds;
            layout = PointCloud2PointLayout::GenericTimed;
        } else if (pointCloud2Mode == PointCloud2Mode::GenericRelativeMilliseconds) {
            timing = PointCloud2FrameTiming::RelativeMilliseconds;
            layout = PointCloud2PointLayout::GenericTimed;
        } else if (pointCloud2Mode == PointCloud2Mode::GenericSynthesized) {
            layout = PointCloud2PointLayout::GenericXyz;
        }

        SlamInputFrame frame;
        QString parseError;
        if (!buildPointCloud2Frame(pending.cloud,
                                   filePath,
                                   nextSequence++,
                                   pending.fallbackFrameStartNs,
                                   nextFrameStartNs,
                                   layout,
                                   timing,
                                   config_.frameDurationMs,
                                   &frame,
                                   &parseError)) {
            errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 PointCloud2 转换失败：%2")
                                .arg(lidarConnection->topic, parseError);
            return false;
        }
        return submitFrame(std::move(frame));
    };

    QSet<int> selectedConnectionIds;
    selectedConnectionIds.insert(lidarConnection->id);
    if (imuConnection != nullptr) {
        selectedConnectionIds.insert(imuConnection->id);
    }

    auto consumeMessage = [&](const Rosbag::SerializedMessage& message) {
        if (cancellationRequested && cancellationRequested->load()) {
            return false;
        }
        if (imuConnection != nullptr && message.connectionId == imuConnection->id) {
            Rosbag::ImuMsg imu;
            QString parseError;
            if (!parseImuMessage(imuConnection->type, message.data, &imu, &parseError)) {
                errorMessage_ = QStringLiteral("ROSbag 加载失败：IMU topic %1 解析失败：%2")
                                    .arg(imuConnection->topic, parseError);
                return false;
            }
            const SlamImuSample sample = toSlamImuSample(imu, config_.lidarToImuTimeOffsetNs);
            if (summary_.imuSampleCount == 0) {
                summary_.imuStartTimestampNs = sample.timestampNs;
                summary_.imuEndTimestampNs = sample.timestampNs;
            } else {
                summary_.imuStartTimestampNs = std::min(summary_.imuStartTimestampNs, sample.timestampNs);
                summary_.imuEndTimestampNs = std::max(summary_.imuEndTimestampNs, sample.timestampNs);
            }
            ++summary_.imuMessageCount;
            ++summary_.imuSampleCount;
            summary_.hasImu = true;
            if (config_.requireImu) {
                synchronizer.pushImuSample(sample);
                return emitReadyFrames();
            }
            return true;
        }
        if (message.connectionId != lidarConnection->id) {
            return true;
        }

        ++summary_.lidarMessageCount;
        if (lidarIsCustomMsg) {
            Rosbag::LivoxCustomMsg livoxMsg;
            QString parseError;
            if (!parseLivoxCustomMessage(lidarConnection->type, message.data, &livoxMsg, &parseError)) {
                errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 解析失败：%2")
                                    .arg(lidarConnection->topic, parseError);
                return false;
            }
            summary_.lidarFormat = QStringLiteral("Livox CustomMsg");
            summary_.pointTimeMode = QStringLiteral("CustomMsg offset_time(ns)");
            if (livoxMsg.points.isEmpty()) {
                ++summary_.emptyLidarMessageCount;
                return true;
            }
            int64_t frameStartNs = message.timestampNs;
            if (config_.useLivoxCustomTimebase && livoxMsg.timebaseNs > 0) {
                frameStartNs = int64_t(livoxMsg.timebaseNs);
            } else if (config_.useHeaderStamp && livoxMsg.header.stampNs > 0) {
                frameStartNs = livoxMsg.header.stampNs;
            }
            SlamInputFrame frame;
            frame.sequence = nextSequence++;
            frame.sourceId = livoxMsg.lidarId;
            frame.frameStartNs = frameStartNs;
            frame.frameEndNs = frameStartNs;
            frame.timeSource = SlamTimeSource::LivoxPacketTimestamp;
            frame.hasPointOffsetTime = true;
            frame.sourceName = filePath;
            frame.points.reserve(livoxMsg.points.size());
            for (const Rosbag::LivoxCustomPoint& sourcePoint : livoxMsg.points) {
                SlamPoint point = toSlamPoint(sourcePoint);
                frame.frameEndNs = std::max(frame.frameEndNs, frameStartNs + point.offsetNs);
                frame.points.push_back(point);
            }
            return submitFrame(std::move(frame));
        }

        Rosbag::PointCloud2Msg cloud;
        QString parseError;
        if (!parsePointCloud2Message(lidarConnection->type, message.data, &cloud, &parseError)) {
            errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 PointCloud2 解析失败：%2")
                                .arg(lidarConnection->topic, parseError);
            return false;
        }
        if (pointCloud2PointCount(cloud) <= 0) {
            ++summary_.emptyLidarMessageCount;
            return true;
        }
        if (pointCloud2Mode == PointCloud2Mode::Unknown && !configurePointCloud2Mode(cloud)) {
            return false;
        }
        if (!pointCloud2LayoutMatches(cloud)) {
            errorMessage_ = QStringLiteral("ROSbag 加载失败：LiDAR topic %1 的 PointCloud2 字段布局发生变化。")
                                .arg(lidarConnection->topic);
            return false;
        }

        PendingPointCloud2 current;
        current.fallbackFrameStartNs = message.timestampNs;
        current.cloud = std::move(cloud);
        const bool synthesized = pointCloud2Mode == PointCloud2Mode::DriverSynthesized ||
                                 pointCloud2Mode == PointCloud2Mode::GenericSynthesized;
        if (!synthesized) {
            return submitPointCloud2(current, 0);
        }
        if (hasPendingCloud) {
            const int64_t nextFrameStartNs = current.cloud.header.stampNs > 0
                ? current.cloud.header.stampNs
                : current.fallbackFrameStartNs;
            if (!submitPointCloud2(pendingCloud, nextFrameStartNs)) {
                return false;
            }
        }
        pendingCloud = std::move(current);
        hasPendingCloud = true;
        return true;
    };

    bool streamOk = false;
    if (useMcapReader) {
        streamOk = mcapReader.streamMessages(filePath,
                                             selectedConnectionIds,
                                             cancellationRequested,
                                             consumeMessage,
                                             progress,
                                             &errorMessage_);
    } else if (useRos2Reader) {
        streamOk = ros2Reader.streamMessages(filePath,
                                             selectedConnectionIds,
                                             cancellationRequested,
                                             consumeMessage,
                                             progress,
                                             &errorMessage_);
    } else {
        streamOk = ros1Reader.streamMessages(filePath,
                                             selectedConnectionIds,
                                             cancellationRequested,
                                             consumeMessage,
                                             progress,
                                             &errorMessage_);
    }
    if (!streamOk) {
        if (error != nullptr && !errorMessage_.isEmpty()) {
            *error = errorMessage_;
        }
        return false;
    }
    if (hasPendingCloud && !submitPointCloud2(pendingCloud, 0)) {
        if (error != nullptr && !errorMessage_.isEmpty()) {
            *error = errorMessage_;
        }
        return false;
    }
    if (!emitReadyFrames()) {
        return false;
    }
    if (consumerStopped) {
        return false;
    }

    summary_.hasCompleteImuCoverage = !config_.requireImu ||
        (summary_.frameCount > 0 && summary_.framesWithCompleteImuCoverage == summary_.frameCount);
    for (RosbagSlamTopicInfo& topic : summary_.topics) {
        if (topic.topic == summary_.lidarTopic) {
            topic.messageCount = int64_t(summary_.lidarMessageCount);
        } else if (topic.topic == summary_.imuTopic) {
            topic.messageCount = int64_t(summary_.imuMessageCount);
        }
    }
    if (config_.requireImu && !summary_.hasImu) {
        errorMessage_ = QStringLiteral("ROSbag 加载失败：IMU topic 没有可用样本。");
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }
    if (summary_.frameCount == 0) {
        errorMessage_ = summary_.lidarMessageCount > summary_.emptyLidarMessageCount
            ? QStringLiteral("ROSbag 加载失败：IMU 样本未覆盖任何 LiDAR 帧。")
            : QStringLiteral("ROSbag 加载失败：LiDAR topic 未生成有效 SLAM 输入帧。");
        if (error != nullptr) {
            *error = errorMessage_;
        }
        return false;
    }
    if (!config_.requireImu && !summary_.hasImu) {
        summary_.messages.push_back(QStringLiteral("ROSbag 未包含 IMU topic，已按纯激光模式流式处理。"));
    }
    if (error != nullptr) {
        error->clear();
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

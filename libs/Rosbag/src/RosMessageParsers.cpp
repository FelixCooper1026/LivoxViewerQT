#include "RosMessageParsers.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace {

class Reader {
public:
    explicit Reader(const QByteArray& data) : data_(data) {}

    bool readU8(uint8_t* value)
    {
        if (!canRead(1)) {
            return false;
        }
        *value = uint8_t(data_.at(offset_));
        ++offset_;
        return true;
    }

    bool readU32(uint32_t* value)
    {
        if (!canRead(4)) {
            return false;
        }
        const char* p = data_.constData() + offset_;
        *value = uint32_t(uint8_t(p[0])) |
                 (uint32_t(uint8_t(p[1])) << 8) |
                 (uint32_t(uint8_t(p[2])) << 16) |
                 (uint32_t(uint8_t(p[3])) << 24);
        offset_ += 4;
        return true;
    }

    bool readU64(uint64_t* value)
    {
        if (!canRead(8)) {
            return false;
        }
        const char* p = data_.constData() + offset_;
        uint64_t result = 0;
        for (int i = 7; i >= 0; --i) {
            result = (result << 8) | uint8_t(p[i]);
        }
        *value = result;
        offset_ += 8;
        return true;
    }

    bool readFloat(float* value)
    {
        if (!canRead(4)) {
            return false;
        }
        std::memcpy(value, data_.constData() + offset_, sizeof(float));
        offset_ += 4;
        return true;
    }

    bool readDouble(double* value)
    {
        if (!canRead(8)) {
            return false;
        }
        std::memcpy(value, data_.constData() + offset_, sizeof(double));
        offset_ += 8;
        return true;
    }

    bool readString(QString* value)
    {
        uint32_t len = 0;
        if (!readU32(&len) || !canRead(len)) {
            return false;
        }
        *value = QString::fromUtf8(data_.constData() + offset_, int(len));
        offset_ += int(len);
        return true;
    }

    bool readByteArray(QByteArray* value)
    {
        uint32_t len = 0;
        if (!readU32(&len) || !canRead(len)) {
            return false;
        }
        *value = data_.mid(offset_, int(len));
        offset_ += int(len);
        return true;
    }

    bool skip(qsizetype size)
    {
        if (!canRead(size)) {
            return false;
        }
        offset_ += int(size);
        return true;
    }

private:
    bool canRead(qsizetype size) const
    {
        return size >= 0 && offset_ <= data_.size() && size <= data_.size() - offset_;
    }

    const QByteArray& data_;
    int offset_ = 0;
};

bool parseHeader(Reader& reader, Rosbag::RosHeader* header)
{
    uint32_t sec = 0;
    uint32_t nsec = 0;
    return reader.readU32(&header->sequence) &&
           reader.readU32(&sec) &&
           reader.readU32(&nsec) &&
           reader.readString(&header->frameId) &&
           (header->stampNs = int64_t(sec) * 1000000000LL + int64_t(nsec), true);
}

bool parsePointField(Reader& reader, Rosbag::PointField* field)
{
    return reader.readString(&field->name) &&
           reader.readU32(&field->offset) &&
           reader.readU8(&field->datatype) &&
           reader.readU32(&field->count);
}

class CdrReader {
public:
    explicit CdrReader(const QByteArray& data) : data_(data)
    {
        if (data_.size() >= kEncapsulationSize) {
            const uint8_t b0 = uint8_t(data_.at(0));
            const uint8_t b1 = uint8_t(data_.at(1));
            littleEndian_ = (b0 == 0 && (b1 == 1 || b1 == 3)) ||
                            ((b0 == 1 || b0 == 3) && b1 == 0);
            valid_ = littleEndian_;
            offset_ = kEncapsulationSize;
        }
    }

    bool isValid() const { return valid_; }

    bool readBool(bool* value)
    {
        uint8_t raw = 0;
        if (!readU8(&raw)) {
            return false;
        }
        *value = raw != 0;
        return true;
    }

    bool readU8(uint8_t* value)
    {
        if (!canRead(1)) {
            return false;
        }
        *value = uint8_t(data_.at(offset_));
        ++offset_;
        return true;
    }

    bool readU32(uint32_t* value)
    {
        if (!align(4) || !canRead(4)) {
            return false;
        }
        const char* p = data_.constData() + offset_;
        *value = uint32_t(uint8_t(p[0])) |
                 (uint32_t(uint8_t(p[1])) << 8) |
                 (uint32_t(uint8_t(p[2])) << 16) |
                 (uint32_t(uint8_t(p[3])) << 24);
        offset_ += 4;
        return true;
    }

    bool readI32(int32_t* value)
    {
        uint32_t raw = 0;
        if (!readU32(&raw)) {
            return false;
        }
        *value = int32_t(raw);
        return true;
    }

    bool readU64(uint64_t* value)
    {
        if (!align(8) || !canRead(8)) {
            return false;
        }
        const char* p = data_.constData() + offset_;
        uint64_t result = 0;
        for (int i = 7; i >= 0; --i) {
            result = (result << 8) | uint8_t(p[i]);
        }
        *value = result;
        offset_ += 8;
        return true;
    }

    bool readFloat(float* value)
    {
        if (!align(4) || !canRead(4)) {
            return false;
        }
        std::memcpy(value, data_.constData() + offset_, sizeof(float));
        offset_ += 4;
        return true;
    }

    bool readDouble(double* value)
    {
        if (!align(8) || !canRead(8)) {
            return false;
        }
        std::memcpy(value, data_.constData() + offset_, sizeof(double));
        offset_ += 8;
        return true;
    }

    bool readString(QString* value)
    {
        uint32_t len = 0;
        if (!readU32(&len) || !canRead(len)) {
            return false;
        }
        QByteArray bytes = data_.mid(offset_, int(len));
        offset_ += int(len);
        if (!bytes.isEmpty() && bytes.endsWith('\0')) {
            bytes.chop(1);
        }
        *value = QString::fromUtf8(bytes);
        return true;
    }

    bool readU8Array(QByteArray* value)
    {
        uint32_t len = 0;
        if (!readU32(&len) || !canRead(len)) {
            return false;
        }
        *value = data_.mid(offset_, int(len));
        offset_ += int(len);
        return true;
    }

private:
    bool align(int alignment)
    {
        if (alignment <= 1) {
            return true;
        }
        const int relativeOffset = offset_ - kEncapsulationSize;
        const int alignedRelativeOffset = (relativeOffset + alignment - 1) & ~(alignment - 1);
        const int aligned = kEncapsulationSize + alignedRelativeOffset;
        if (aligned < offset_ || aligned > data_.size()) {
            return false;
        }
        offset_ = aligned;
        return true;
    }

    bool canRead(qsizetype size) const
    {
        return valid_ && size >= 0 && offset_ <= data_.size() && size <= data_.size() - offset_;
    }

    const QByteArray& data_;
    static constexpr int kEncapsulationSize = 4;
    int offset_ = 0;
    bool littleEndian_ = false;
    bool valid_ = false;
};

bool parseRos2Header(CdrReader& reader, Rosbag::RosHeader* header)
{
    int32_t sec = 0;
    uint32_t nsec = 0;
    return reader.readI32(&sec) &&
           reader.readU32(&nsec) &&
           reader.readString(&header->frameId) &&
           (header->sequence = 0,
            header->stampNs = int64_t(sec) * 1000000000LL + int64_t(nsec),
            true);
}

bool parseRos2PointField(CdrReader& reader, Rosbag::PointField* field)
{
    return reader.readString(&field->name) &&
           reader.readU32(&field->offset) &&
           reader.readU8(&field->datatype) &&
           reader.readU32(&field->count);
}

} // namespace

namespace Rosbag {

bool parseLivoxCustomMsg(const QByteArray& data, LivoxCustomMsg* out, QString* error)
{
    if (out == nullptr) {
        return false;
    }
    *out = LivoxCustomMsg();

    Reader reader(data);
    if (!parseHeader(reader, &out->header) ||
        !reader.readU64(&out->timebaseNs) ||
        !reader.readU32(&out->pointNum) ||
        !reader.readU8(&out->lidarId) ||
        !reader.skip(3)) {
        if (error != nullptr) {
            *error = QStringLiteral("Livox CustomMsg 反序列化失败：消息头不完整。");
        }
        return false;
    }

    uint32_t pointArrayLen = 0;
    if (!reader.readU32(&pointArrayLen)) {
        if (error != nullptr) {
            *error = QStringLiteral("Livox CustomMsg 反序列化失败：缺少 points 数组长度。");
        }
        return false;
    }
    out->points.reserve(int(pointArrayLen));
    for (uint32_t i = 0; i < pointArrayLen; ++i) {
        LivoxCustomPoint point;
        if (!reader.readU32(&point.offsetTimeNs) ||
            !reader.readFloat(&point.x) ||
            !reader.readFloat(&point.y) ||
            !reader.readFloat(&point.z) ||
            !reader.readU8(&point.reflectivity) ||
            !reader.readU8(&point.tag) ||
            !reader.readU8(&point.line)) {
            if (error != nullptr) {
                *error = QStringLiteral("Livox CustomMsg 反序列化失败：第 %1 个点数据不完整。").arg(i);
            }
            return false;
        }
        out->points.push_back(point);
    }
    return true;
}

bool parseSensorImu(const QByteArray& data, ImuMsg* out, QString* error)
{
    if (out == nullptr) {
        return false;
    }
    *out = ImuMsg();

    Reader reader(data);
    double ignored = 0.0;
    if (!parseHeader(reader, &out->header)) {
        if (error != nullptr) {
            *error = QStringLiteral("sensor_msgs/Imu 反序列化失败：Header 不完整。");
        }
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        if (!reader.readDouble(&ignored)) {
            if (error != nullptr) {
                *error = QStringLiteral("sensor_msgs/Imu 反序列化失败：orientation 不完整。");
            }
            return false;
        }
    }
    for (int i = 0; i < 9; ++i) {
        if (!reader.readDouble(&ignored)) {
            if (error != nullptr) {
                *error = QStringLiteral("sensor_msgs/Imu 反序列化失败：orientation covariance 不完整。");
            }
            return false;
        }
    }
    for (double& value : out->angularVelocity) {
        if (!reader.readDouble(&value)) {
            if (error != nullptr) {
                *error = QStringLiteral("sensor_msgs/Imu 反序列化失败：angular_velocity 不完整。");
            }
            return false;
        }
    }
    for (int i = 0; i < 9; ++i) {
        if (!reader.readDouble(&ignored)) {
            if (error != nullptr) {
                *error = QStringLiteral("sensor_msgs/Imu 反序列化失败：angular velocity covariance 不完整。");
            }
            return false;
        }
    }
    for (double& value : out->linearAcceleration) {
        if (!reader.readDouble(&value)) {
            if (error != nullptr) {
                *error = QStringLiteral("sensor_msgs/Imu 反序列化失败：linear_acceleration 不完整。");
            }
            return false;
        }
    }
    return true;
}

bool parseSensorPointCloud2(const QByteArray& data, PointCloud2Msg* out, QString* error)
{
    if (out == nullptr) {
        return false;
    }
    *out = PointCloud2Msg();

    Reader reader(data);
    if (!parseHeader(reader, &out->header) ||
        !reader.readU32(&out->height) ||
        !reader.readU32(&out->width)) {
        if (error != nullptr) {
            *error = QStringLiteral("sensor_msgs/PointCloud2 反序列化失败：Header/尺寸字段不完整。");
        }
        return false;
    }

    uint32_t fieldCount = 0;
    if (!reader.readU32(&fieldCount)) {
        if (error != nullptr) {
            *error = QStringLiteral("sensor_msgs/PointCloud2 反序列化失败：缺少 fields 数组长度。");
        }
        return false;
    }
    out->fields.reserve(int(fieldCount));
    for (uint32_t i = 0; i < fieldCount; ++i) {
        PointField field;
        if (!parsePointField(reader, &field)) {
            if (error != nullptr) {
                *error = QStringLiteral("sensor_msgs/PointCloud2 反序列化失败：第 %1 个 PointField 不完整。").arg(i);
            }
            return false;
        }
        out->fields.push_back(std::move(field));
    }

    uint8_t isBigEndian = 0;
    if (!reader.readU8(&isBigEndian) ||
        !reader.readU32(&out->pointStep) ||
        !reader.readU32(&out->rowStep)) {
        if (error != nullptr) {
            *error = QStringLiteral("sensor_msgs/PointCloud2 反序列化失败：point_step/row_step 不完整。");
        }
        return false;
    }
    out->isBigEndian = isBigEndian != 0;

    if (!reader.readByteArray(&out->data)) {
        if (error != nullptr) {
            *error = QStringLiteral("sensor_msgs/PointCloud2 反序列化失败：data 数组不完整。");
        }
        return false;
    }

    uint8_t isDense = 0;
    if (!reader.readU8(&isDense)) {
        if (error != nullptr) {
            *error = QStringLiteral("sensor_msgs/PointCloud2 反序列化失败：is_dense 缺失。");
        }
        return false;
    }
    out->isDense = isDense != 0;
    return true;
}

bool parseRos2LivoxCustomMsg(const QByteArray& data, LivoxCustomMsg* out, QString* error)
{
    if (out == nullptr) {
        return false;
    }
    *out = LivoxCustomMsg();

    CdrReader reader(data);
    if (!reader.isValid()) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 Livox CustomMsg 反序列化失败：不支持的 CDR 封装。");
        }
        return false;
    }
    if (!parseRos2Header(reader, &out->header) ||
        !reader.readU64(&out->timebaseNs) ||
        !reader.readU32(&out->pointNum) ||
        !reader.readU8(&out->lidarId)) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 Livox CustomMsg 反序列化失败：消息头不完整。");
        }
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        uint8_t ignored = 0;
        if (!reader.readU8(&ignored)) {
            if (error != nullptr) {
                *error = QStringLiteral("ROS2 Livox CustomMsg 反序列化失败：rsvd 字段不完整。");
            }
            return false;
        }
    }

    uint32_t pointArrayLen = 0;
    if (!reader.readU32(&pointArrayLen)) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 Livox CustomMsg 反序列化失败：缺少 points 数组长度。");
        }
        return false;
    }
    out->points.reserve(int(pointArrayLen));
    for (uint32_t i = 0; i < pointArrayLen; ++i) {
        LivoxCustomPoint point;
        if (!reader.readU32(&point.offsetTimeNs) ||
            !reader.readFloat(&point.x) ||
            !reader.readFloat(&point.y) ||
            !reader.readFloat(&point.z) ||
            !reader.readU8(&point.reflectivity) ||
            !reader.readU8(&point.tag) ||
            !reader.readU8(&point.line)) {
            if (error != nullptr) {
                *error = QStringLiteral("ROS2 Livox CustomMsg 反序列化失败：第 %1 个点数据不完整。").arg(i);
            }
            return false;
        }
        out->points.push_back(point);
    }
    return true;
}

bool parseRos2SensorImu(const QByteArray& data, ImuMsg* out, QString* error)
{
    if (out == nullptr) {
        return false;
    }
    *out = ImuMsg();

    CdrReader reader(data);
    double ignored = 0.0;
    if (!reader.isValid()) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 sensor_msgs/Imu 反序列化失败：不支持的 CDR 封装。");
        }
        return false;
    }
    if (!parseRos2Header(reader, &out->header)) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 sensor_msgs/Imu 反序列化失败：Header 不完整。");
        }
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        if (!reader.readDouble(&ignored)) {
            if (error != nullptr) {
                *error = QStringLiteral("ROS2 sensor_msgs/Imu 反序列化失败：orientation 不完整。");
            }
            return false;
        }
    }
    for (int i = 0; i < 9; ++i) {
        if (!reader.readDouble(&ignored)) {
            if (error != nullptr) {
                *error = QStringLiteral("ROS2 sensor_msgs/Imu 反序列化失败：orientation covariance 不完整。");
            }
            return false;
        }
    }
    for (double& value : out->angularVelocity) {
        if (!reader.readDouble(&value)) {
            if (error != nullptr) {
                *error = QStringLiteral("ROS2 sensor_msgs/Imu 反序列化失败：angular_velocity 不完整。");
            }
            return false;
        }
    }
    for (int i = 0; i < 9; ++i) {
        if (!reader.readDouble(&ignored)) {
            if (error != nullptr) {
                *error = QStringLiteral("ROS2 sensor_msgs/Imu 反序列化失败：angular velocity covariance 不完整。");
            }
            return false;
        }
    }
    for (double& value : out->linearAcceleration) {
        if (!reader.readDouble(&value)) {
            if (error != nullptr) {
                *error = QStringLiteral("ROS2 sensor_msgs/Imu 反序列化失败：linear_acceleration 不完整。");
            }
            return false;
        }
    }
    return true;
}

bool parseRos2SensorPointCloud2(const QByteArray& data, PointCloud2Msg* out, QString* error)
{
    if (out == nullptr) {
        return false;
    }
    *out = PointCloud2Msg();

    CdrReader reader(data);
    if (!reader.isValid()) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 sensor_msgs/PointCloud2 反序列化失败：不支持的 CDR 封装。");
        }
        return false;
    }
    if (!parseRos2Header(reader, &out->header) ||
        !reader.readU32(&out->height) ||
        !reader.readU32(&out->width)) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 sensor_msgs/PointCloud2 反序列化失败：Header/尺寸字段不完整。");
        }
        return false;
    }

    uint32_t fieldCount = 0;
    if (!reader.readU32(&fieldCount)) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 sensor_msgs/PointCloud2 反序列化失败：缺少 fields 数组长度。");
        }
        return false;
    }
    out->fields.reserve(int(fieldCount));
    for (uint32_t i = 0; i < fieldCount; ++i) {
        PointField field;
        if (!parseRos2PointField(reader, &field)) {
            if (error != nullptr) {
                *error = QStringLiteral("ROS2 sensor_msgs/PointCloud2 反序列化失败：第 %1 个 PointField 不完整。").arg(i);
            }
            return false;
        }
        out->fields.push_back(std::move(field));
    }

    if (!reader.readBool(&out->isBigEndian) ||
        !reader.readU32(&out->pointStep) ||
        !reader.readU32(&out->rowStep) ||
        !reader.readU8Array(&out->data) ||
        !reader.readBool(&out->isDense)) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 sensor_msgs/PointCloud2 反序列化失败：点云数据区不完整。");
        }
        return false;
    }
    return true;
}

} // namespace Rosbag

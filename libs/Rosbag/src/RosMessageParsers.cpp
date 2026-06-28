#include "Rosbag/RosMessageParsers.h"

#include <cstring>

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
    if (pointArrayLen != out->pointNum) {
        if (error != nullptr) {
            *error = QStringLiteral("Livox CustomMsg 反序列化失败：point_num=%1 与 points 数组长度=%2 不一致。")
                         .arg(out->pointNum)
                         .arg(pointArrayLen);
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

} // namespace Rosbag

#ifndef ROSBAG_ROSMESSAGEPARSERS_H
#define ROSBAG_ROSMESSAGEPARSERS_H

#include <QByteArray>
#include <QString>
#include <QVector>

#include <cstdint>

namespace Rosbag {

struct RosHeader {
    uint32_t sequence = 0;
    int64_t stampNs = 0;
    QString frameId;
};

struct LivoxCustomPoint {
    uint32_t offsetTimeNs = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    uint8_t reflectivity = 0;
    uint8_t tag = 0;
    uint8_t line = 0;
};

struct LivoxCustomMsg {
    RosHeader header;
    uint64_t timebaseNs = 0;
    uint32_t pointNum = 0;
    uint8_t lidarId = 0;
    QVector<LivoxCustomPoint> points;
};

struct ImuMsg {
    RosHeader header;
    double angularVelocity[3] = {};
    double linearAcceleration[3] = {};
};

struct PointField {
    QString name;
    uint32_t offset = 0;
    uint8_t datatype = 0;
    uint32_t count = 0;
};

struct PointCloud2Msg {
    RosHeader header;
    uint32_t height = 0;
    uint32_t width = 0;
    QVector<PointField> fields;
    bool isBigEndian = false;
    uint32_t pointStep = 0;
    uint32_t rowStep = 0;
    QByteArray data;
    bool isDense = false;
};

bool parseLivoxCustomMsg(const QByteArray& data, LivoxCustomMsg* out, QString* error);
bool parseSensorImu(const QByteArray& data, ImuMsg* out, QString* error);
bool parseSensorPointCloud2(const QByteArray& data, PointCloud2Msg* out, QString* error);

bool parseRos2LivoxCustomMsg(const QByteArray& data, LivoxCustomMsg* out, QString* error);
bool parseRos2SensorImu(const QByteArray& data, ImuMsg* out, QString* error);
bool parseRos2SensorPointCloud2(const QByteArray& data, PointCloud2Msg* out, QString* error);

} // namespace Rosbag

#endif // ROSBAG_ROSMESSAGEPARSERS_H

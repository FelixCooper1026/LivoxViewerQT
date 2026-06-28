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

bool parseLivoxCustomMsg(const QByteArray& data, LivoxCustomMsg* out, QString* error);
bool parseSensorImu(const QByteArray& data, ImuMsg* out, QString* error);

} // namespace Rosbag

#endif // ROSBAG_ROSMESSAGEPARSERS_H

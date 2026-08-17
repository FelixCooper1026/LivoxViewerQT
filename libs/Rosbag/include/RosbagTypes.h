#ifndef ROSBAG_ROSBAGTYPES_H
#define ROSBAG_ROSBAGTYPES_H

#include <QByteArray>
#include <QHash>
#include <QString>
#include <QVector>

#include <cstdint>

namespace Rosbag {

struct Connection {
    int id = -1;
    QString topic;
    QString type;
    QString md5sum;
    QString messageDefinition;
    QString schemaEncoding;
    QString messageEncoding;
    QHash<QString, QByteArray> fields;
    int64_t messageCount = 0;
};

struct SerializedMessage {
    int connectionId = -1;
    QString topic;
    QString type;
    int64_t timestampNs = 0;
    int64_t publishTimestampNs = 0;
    QByteArray data;
};

struct Summary {
    QString filePath;
    int connectionCount = 0;
    int chunkCount = 0;
    int64_t messageCount = 0;
    int64_t startTimestampNs = 0;
    int64_t endTimestampNs = 0;
};

} // namespace Rosbag

#endif // ROSBAG_ROSBAGTYPES_H

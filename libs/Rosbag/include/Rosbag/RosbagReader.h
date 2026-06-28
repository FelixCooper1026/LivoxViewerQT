#ifndef ROSBAG_ROSBAGREADER_H
#define ROSBAG_ROSBAGREADER_H

#include "Rosbag/RosbagTypes.h"

#include <QVector>

namespace Rosbag {

class Reader {
public:
    bool read(const QString& filePath, QString* error);
    void clear();

    const QVector<Connection>& connections() const;
    const QVector<SerializedMessage>& messages() const;
    const Summary& summary() const;

private:
    QVector<Connection> connections_;
    QVector<SerializedMessage> messages_;
    Summary summary_;
};

} // namespace Rosbag

#endif // ROSBAG_ROSBAGREADER_H

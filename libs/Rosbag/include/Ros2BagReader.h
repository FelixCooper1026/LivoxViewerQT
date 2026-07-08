#ifndef ROSBAG_ROS2BAGREADER_H
#define ROSBAG_ROS2BAGREADER_H

#include "RosbagTypes.h"

#include <QVector>

namespace Rosbag {

class Ros2BagReader {
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

#endif // ROSBAG_ROS2BAGREADER_H

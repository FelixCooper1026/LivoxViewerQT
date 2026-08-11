#ifndef ROSBAG_ROSBAGREADER_H
#define ROSBAG_ROSBAGREADER_H

#include "RosbagTypes.h"

#include <QSet>
#include <QVector>

#include <atomic>
#include <functional>

namespace Rosbag {

class Reader {
public:
    bool read(const QString& filePath, QString* error);
    bool readConnections(const QString& filePath, QString* error);
    bool streamMessages(const QString& filePath,
                        const QSet<int>& connectionIds,
                        const std::atomic_bool* cancellationRequested,
                        const std::function<bool(const SerializedMessage&)>& consumer,
                        const std::function<void(int64_t, int64_t)>& progress,
                        QString* error);
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

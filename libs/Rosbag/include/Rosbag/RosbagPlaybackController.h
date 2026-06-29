#ifndef ROSBAG_ROSBAGPLAYBACKCONTROLLER_H
#define ROSBAG_ROSBAGPLAYBACKCONTROLLER_H

#include <QString>

namespace RosbagPlayback {

inline bool isSupportedFile(const QString& filePath)
{
    return filePath.endsWith(QStringLiteral(".bag"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".db3"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".yaml"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".yml"), Qt::CaseInsensitive);
}

} // namespace RosbagPlayback

#endif // ROSBAG_ROSBAGPLAYBACKCONTROLLER_H

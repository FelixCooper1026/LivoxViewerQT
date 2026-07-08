#ifndef SLAM_EXPORT_SLAMTRAJECTORYEXPORT_H
#define SLAM_EXPORT_SLAMTRAJECTORYEXPORT_H

#include "Core/SlamTypes.h"

#include <QString>
#include <QVector>

namespace SlamTrajectoryExport {

enum class Format {
    Csv,
    Tum
};

bool save(const QString& filePath,
          const QVector<SlamTrajectoryPoint>& trajectory,
          Format format,
          QString* error);

bool saveCsv(const QString& filePath,
             const QVector<SlamTrajectoryPoint>& trajectory,
             QString* error);

bool saveTum(const QString& filePath,
             const QVector<SlamTrajectoryPoint>& trajectory,
             QString* error);

} // namespace SlamTrajectoryExport

#endif // SLAM_EXPORT_SLAMTRAJECTORYEXPORT_H

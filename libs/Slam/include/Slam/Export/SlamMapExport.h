#ifndef SLAM_EXPORT_SLAMMAPEXPORT_H
#define SLAM_EXPORT_SLAMMAPEXPORT_H

#include "Slam/Core/SlamTypes.h"

#include <QString>

namespace SlamMapExport {

enum class Format {
    Pcd,
    Las
};

bool save(const QString& filePath, const QVector<SlamPoint>& points, Format format, QString* error = nullptr);

} // namespace SlamMapExport

#endif // SLAM_EXPORT_SLAMMAPEXPORT_H

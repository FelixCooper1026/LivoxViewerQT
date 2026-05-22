#ifndef EXPORT_POINTCLOUDEXPORT_H
#define EXPORT_POINTCLOUDEXPORT_H

#include "PointCloud/PointCloudFrame.h"

#include <QIODevice>
#include <QString>

namespace PointCloudExport {

struct Bounds {
    double minX = 0.0;
    double minY = 0.0;
    double minZ = 0.0;
    double maxX = 0.0;
    double maxY = 0.0;
    double maxZ = 0.0;
};

Bounds calculateBounds(const QVector<PointCloudPoint>& points);

bool writePcdHeader(QIODevice& device, qint64 pointCount);
bool writePcdPoint(QIODevice& device, const PointCloudPoint& point);
bool writeLasHeader(QIODevice& device, quint32 pointCount, const Bounds& bounds, double scale = 0.001);
bool writeLasPoint(QIODevice& device, const PointCloudPoint& point, double scale = 0.001);

bool saveAsPCD(const QString& filePath, const QVector<PointCloudPoint>& points);
bool saveAsLAS(const QString& filePath, const QVector<PointCloudPoint>& points);
bool saveAsCSV(const QString& filePath, const QVector<PointCloudPoint>& points);
bool saveAsTXT(const QString& filePath, const QVector<PointCloudPoint>& points);

} // namespace PointCloudExport

#endif // EXPORT_POINTCLOUDEXPORT_H

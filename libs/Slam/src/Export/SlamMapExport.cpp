#include "Slam/Export/SlamMapExport.h"

#include "Export/PointCloudExport.h"

#include <QFile>

#include <algorithm>
#include <limits>

namespace SlamMapExport {

namespace {

void assignError(QString* error, const QString& message)
{
    if (error != nullptr) {
        *error = message;
    }
}

PointCloudPoint toPointCloudPoint(const SlamPoint& point)
{
    PointCloudPoint out{};
    out.x = point.x;
    out.y = point.y;
    out.z = point.z;
    out.r = 1.0f;
    out.g = 1.0f;
    out.b = 1.0f;
    out.reflectivity = point.reflectivity;
    out.tag = point.tag;
    out.line = point.line;
    out.spherical = false;
    out.theta = 0.0f;
    out.phi = 0.0f;
    out.depth = 0.0f;
    return out;
}

PointCloudExport::Bounds calculateBounds(const QVector<SlamPoint>& points)
{
    if (points.isEmpty()) {
        return {};
    }

    PointCloudExport::Bounds bounds;
    bounds.minX = bounds.minY = bounds.minZ = std::numeric_limits<double>::max();
    bounds.maxX = bounds.maxY = bounds.maxZ = -std::numeric_limits<double>::max();
    for (const SlamPoint& point : points) {
        bounds.minX = std::min(bounds.minX, double(point.x));
        bounds.minY = std::min(bounds.minY, double(point.y));
        bounds.minZ = std::min(bounds.minZ, double(point.z));
        bounds.maxX = std::max(bounds.maxX, double(point.x));
        bounds.maxY = std::max(bounds.maxY, double(point.y));
        bounds.maxZ = std::max(bounds.maxZ, double(point.z));
    }
    return bounds;
}

bool savePcd(const QString& filePath, const QVector<SlamPoint>& points, QString* error)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        assignError(error, QStringLiteral("无法创建 PCD 文件: %1").arg(file.errorString()));
        return false;
    }
    if (!PointCloudExport::writePcdHeader(file, points.size())) {
        assignError(error, QStringLiteral("写入 PCD 文件头失败。"));
        return false;
    }
    for (const SlamPoint& point : points) {
        if (!PointCloudExport::writePcdPoint(file, toPointCloudPoint(point))) {
            assignError(error, QStringLiteral("写入 PCD 点数据失败。"));
            return false;
        }
    }
    assignError(error, QString());
    return true;
}

bool saveLas(const QString& filePath, const QVector<SlamPoint>& points, QString* error)
{
    if (qulonglong(points.size()) > qulonglong(std::numeric_limits<quint32>::max())) {
        assignError(error, QStringLiteral("LAS 1.2 点数超过 32 位上限，无法导出。"));
        return false;
    }

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        assignError(error, QStringLiteral("无法创建 LAS 文件: %1").arg(file.errorString()));
        return false;
    }
    if (!PointCloudExport::writeLasHeader(file,
                                          static_cast<quint32>(points.size()),
                                          calculateBounds(points))) {
        assignError(error, QStringLiteral("写入 LAS 文件头失败。"));
        return false;
    }
    for (const SlamPoint& point : points) {
        if (!PointCloudExport::writeLasPoint(file, toPointCloudPoint(point))) {
            assignError(error, QStringLiteral("写入 LAS 点数据失败。"));
            return false;
        }
    }
    assignError(error, QString());
    return true;
}

} // namespace

bool save(const QString& filePath, const QVector<SlamPoint>& points, Format format, QString* error)
{
    if (points.isEmpty()) {
        assignError(error, QStringLiteral("完整全局地图为空，无法导出。"));
        return false;
    }
    switch (format) {
    case Format::Pcd:
        return savePcd(filePath, points, error);
    case Format::Las:
        return saveLas(filePath, points, error);
    }
    assignError(error, QStringLiteral("未知地图导出格式。"));
    return false;
}

} // namespace SlamMapExport

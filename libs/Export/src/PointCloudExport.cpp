#include "PointCloudExport.h"

#include <QFile>
#include <QTextStream>
#include <QtEndian>

#include <algorithm>
#include <cmath>
#include <limits>

namespace PointCloudExport {

namespace {

struct PcdBinaryPoint {
    float x;
    float y;
    float z;
    float intensity;
    float tag;
};
static_assert(sizeof(PcdBinaryPoint) == 20, "PcdBinaryPoint must be 20 bytes");

static inline quint16 clampU16(int value)
{
    return quint16(std::max(0, std::min(65535, value)));
}

} // namespace

Bounds calculateBounds(const QVector<PointCloudPoint>& points)
{
    if (points.isEmpty()) {
        return {};
    }

    Bounds bounds;
    bounds.minX = bounds.minY = bounds.minZ = std::numeric_limits<double>::max();
    bounds.maxX = bounds.maxY = bounds.maxZ = -std::numeric_limits<double>::max();

    for (const PointCloudPoint& point : points) {
        bounds.minX = std::min(bounds.minX, double(point.x));
        bounds.minY = std::min(bounds.minY, double(point.y));
        bounds.minZ = std::min(bounds.minZ, double(point.z));
        bounds.maxX = std::max(bounds.maxX, double(point.x));
        bounds.maxY = std::max(bounds.maxY, double(point.y));
        bounds.maxZ = std::max(bounds.maxZ, double(point.z));
    }

    return bounds;
}

bool writePcdHeader(QIODevice& device, qint64 pointCount)
{
    QTextStream header(&device);
    header.setRealNumberNotation(QTextStream::FixedNotation);
    header.setRealNumberPrecision(6);
    header << "# .PCD v0.7 - Point Cloud Data file\n";
    header << "VERSION 0.7\n";
    header << "FIELDS x y z intensity tag\n";
    header << "SIZE 4 4 4 4 4\n";
    header << "TYPE F F F F F\n";
    header << "COUNT 1 1 1 1 1\n";
    header << "WIDTH " << pointCount << "\n";
    header << "HEIGHT 1\n";
    header << "VIEWPOINT 0 0 0 1 0 0 0\n";
    header << "POINTS " << pointCount << "\n";
    header << "DATA binary\n";
    header.flush();
    return header.status() == QTextStream::Ok;
}

bool writePcdPoint(QIODevice& device, const PointCloudPoint& point)
{
    const PcdBinaryPoint out{
        point.x,
        point.y,
        point.z,
        float(point.reflectivity),
        float(point.tag)
    };
    return device.write(reinterpret_cast<const char*>(&out), sizeof(out)) == sizeof(out);
}

bool writeLasHeader(QIODevice& device, quint32 pointCount, const Bounds& bounds, double scale)
{
    QByteArray header(227, 0);
    header[0] = 'L';
    header[1] = 'A';
    header[2] = 'S';
    header[3] = 'F';
    header[24] = 1;
    header[25] = 2;

    QByteArray sys = QByteArray("LivoxViewerQT").leftJustified(32, '\0', true);
    std::copy(sys.begin(), sys.end(), header.begin() + 26);
    QByteArray gen = QByteArray("LVX").leftJustified(32, '\0', true);
    std::copy(gen.begin(), gen.end(), header.begin() + 58);

    qToLittleEndian<quint16>(227, reinterpret_cast<uchar*>(header.data() + 94));
    qToLittleEndian<quint32>(227, reinterpret_cast<uchar*>(header.data() + 96));
    qToLittleEndian<quint32>(0, reinterpret_cast<uchar*>(header.data() + 100));
    header[104] = 0;
    qToLittleEndian<quint16>(20, reinterpret_cast<uchar*>(header.data() + 105));
    qToLittleEndian<quint32>(pointCount, reinterpret_cast<uchar*>(header.data() + 107));
    qToLittleEndian<quint32>(pointCount, reinterpret_cast<uchar*>(header.data() + 111));

    qToLittleEndian<double>(scale, reinterpret_cast<uchar*>(header.data() + 131));
    qToLittleEndian<double>(scale, reinterpret_cast<uchar*>(header.data() + 139));
    qToLittleEndian<double>(scale, reinterpret_cast<uchar*>(header.data() + 147));
    qToLittleEndian<double>(0.0, reinterpret_cast<uchar*>(header.data() + 155));
    qToLittleEndian<double>(0.0, reinterpret_cast<uchar*>(header.data() + 163));
    qToLittleEndian<double>(0.0, reinterpret_cast<uchar*>(header.data() + 171));

    qToLittleEndian<double>(bounds.maxX, reinterpret_cast<uchar*>(header.data() + 179));
    qToLittleEndian<double>(bounds.minX, reinterpret_cast<uchar*>(header.data() + 187));
    qToLittleEndian<double>(bounds.maxY, reinterpret_cast<uchar*>(header.data() + 195));
    qToLittleEndian<double>(bounds.minY, reinterpret_cast<uchar*>(header.data() + 203));
    qToLittleEndian<double>(bounds.maxZ, reinterpret_cast<uchar*>(header.data() + 211));
    qToLittleEndian<double>(bounds.minZ, reinterpret_cast<uchar*>(header.data() + 219));

    return device.write(header) == header.size();
}

bool writeLasPoint(QIODevice& device, const PointCloudPoint& point, double scale)
{
    QByteArray record(20, 0);
    const qint32 xi = qint32(std::llround(double(point.x) / scale));
    const qint32 yi = qint32(std::llround(double(point.y) / scale));
    const qint32 zi = qint32(std::llround(double(point.z) / scale));
    qToLittleEndian<qint32>(xi, reinterpret_cast<uchar*>(record.data() + 0));
    qToLittleEndian<qint32>(yi, reinterpret_cast<uchar*>(record.data() + 4));
    qToLittleEndian<qint32>(zi, reinterpret_cast<uchar*>(record.data() + 8));
    qToLittleEndian<quint16>(clampU16(int(point.reflectivity)), reinterpret_cast<uchar*>(record.data() + 12));
    record[14] = 0;
    record[15] = 0;
    record[16] = 0;
    record[17] = static_cast<char>(point.tag);
    qToLittleEndian<quint16>(0, reinterpret_cast<uchar*>(record.data() + 18));
    return device.write(record) == record.size();
}

bool saveAsPCD(const QString& filePath, const QVector<PointCloudPoint>& points)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return false;
    }
    if (!writePcdHeader(file, points.size())) {
        return false;
    }
    for (const PointCloudPoint& point : points) {
        if (!writePcdPoint(file, point)) {
            return false;
        }
    }
    return true;
}

bool saveAsLAS(const QString& filePath, const QVector<PointCloudPoint>& points)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return false;
    }
    const quint32 count = static_cast<quint32>(
        std::min<qulonglong>(static_cast<qulonglong>(points.size()),
                             std::numeric_limits<quint32>::max()));
    if (!writeLasHeader(file, count, calculateBounds(points))) {
        return false;
    }
    for (const PointCloudPoint& point : points) {
        if (!writeLasPoint(file, point)) {
            return false;
        }
    }
    return true;
}

bool saveAsCSV(const QString& filePath, const QVector<PointCloudPoint>& points)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        return false;
    }
    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(6);
    out << "x,y,z,reflectivity,tag\n";
    for (const PointCloudPoint& point : points) {
        out << point.x << ',' << point.y << ',' << point.z << ','
            << int(point.reflectivity) << ',' << int(point.tag) << "\n";
    }
    return true;
}

bool saveAsTXT(const QString& filePath, const QVector<PointCloudPoint>& points)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        return false;
    }
    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(6);
    for (const PointCloudPoint& point : points) {
        out << point.x << ' ' << point.y << ' ' << point.z << ' '
            << int(point.reflectivity) << ' ' << int(point.tag) << "\n";
    }
    return true;
}

} // namespace PointCloudExport

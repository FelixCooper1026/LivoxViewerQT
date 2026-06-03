#include "plugins/StlModel/StlModelLoader.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <QFile>
#include <QTextStream>

namespace StlModel {
namespace {

float readFloat32(const char* data)
{
    float value = 0.0f;
    std::memcpy(&value, data, sizeof(float));
    return value;
}

quint32 readUInt32(const char* data)
{
    quint32 value = 0;
    std::memcpy(&value, data, sizeof(quint32));
    return value;
}

QVector3D triangleNormal(const QVector3D& a, const QVector3D& b, const QVector3D& c)
{
    QVector3D normal = QVector3D::crossProduct(b - a, c - a);
    if (normal.lengthSquared() > 0.0f) {
        normal.normalize();
    }
    return normal;
}

QVector3D colorForNormal(const QVector3D& normal)
{
    const QVector3D light = QVector3D(0.35f, -0.45f, 0.82f).normalized();
    const float intensity = 0.48f + 0.42f * std::max(0.0f, QVector3D::dotProduct(normal, light));
    return QVector3D(0.72f, 0.76f, 0.78f) * intensity;
}

void addTriangle(Mesh& mesh, const QVector3D& a, const QVector3D& b, const QVector3D& c)
{
    const QVector3D normal = triangleNormal(a, b, c);
    const QVector3D color = colorForNormal(normal);
    mesh.triangles.push_back({a.x(), a.y(), a.z(), color.x(), color.y(), color.z()});
    mesh.triangles.push_back({b.x(), b.y(), b.z(), color.x(), color.y(), color.z()});
    mesh.triangles.push_back({c.x(), c.y(), c.z(), color.x(), color.y(), color.z()});
}

void updateBounds(QVector3D& minPoint, QVector3D& maxPoint, const QVector3D& point)
{
    minPoint.setX(std::min(minPoint.x(), point.x()));
    minPoint.setY(std::min(minPoint.y(), point.y()));
    minPoint.setZ(std::min(minPoint.z(), point.z()));
    maxPoint.setX(std::max(maxPoint.x(), point.x()));
    maxPoint.setY(std::max(maxPoint.y(), point.y()));
    maxPoint.setZ(std::max(maxPoint.z(), point.z()));
}

void centerMesh(Mesh& mesh, const QVector3D& center)
{
    for (Vertex& vertex : mesh.triangles) {
        vertex.x -= center.x();
        vertex.y -= center.y();
        vertex.z -= center.z();
    }
    mesh.boundsMin -= center;
    mesh.boundsMax -= center;
}

bool loadBinary(const QByteArray& data, Mesh& mesh)
{
    const quint32 triangleCount = readUInt32(data.constData() + 80);
    mesh.triangles.clear();
    mesh.triangles.reserve(int(triangleCount) * 3);

    const char* cursor = data.constData() + 84;
    QVector3D minPoint;
    QVector3D maxPoint;
    bool first = true;
    for (quint32 i = 0; i < triangleCount; ++i) {
        cursor += 12;
        QVector3D points[3];
        for (QVector3D& point : points) {
            point = QVector3D(readFloat32(cursor), readFloat32(cursor + 4), readFloat32(cursor + 8));
            cursor += 12;
            if (first) {
                minPoint = point;
                maxPoint = point;
                first = false;
            } else {
                updateBounds(minPoint, maxPoint, point);
            }
        }
        addTriangle(mesh, points[0], points[1], points[2]);
        cursor += 2;
    }

    mesh.boundsMin = minPoint;
    mesh.boundsMax = maxPoint;
    centerMesh(mesh, (minPoint + maxPoint) * 0.5f);
    return true;
}

bool loadAscii(const QByteArray& data, Mesh& mesh)
{
    mesh.triangles.clear();
    QTextStream stream(data);
    QVector<QVector3D> pending;
    pending.reserve(3);

    QVector3D minPoint;
    QVector3D maxPoint;
    bool first = true;
    while (!stream.atEnd()) {
        const QString line = stream.readLine().trimmed();
        if (!line.startsWith(QStringLiteral("vertex"))) {
            continue;
        }
        const QStringList parts = line.split(QChar(' '), Qt::SkipEmptyParts);
        if (parts.size() != 4) {
            continue;
        }
        const QVector3D point(parts[1].toFloat(), parts[2].toFloat(), parts[3].toFloat());
        if (first) {
            minPoint = point;
            maxPoint = point;
            first = false;
        } else {
            updateBounds(minPoint, maxPoint, point);
        }
        pending.push_back(point);
        if (pending.size() == 3) {
            addTriangle(mesh, pending[0], pending[1], pending[2]);
            pending.clear();
        }
    }

    mesh.boundsMin = minPoint;
    mesh.boundsMax = maxPoint;
    centerMesh(mesh, (minPoint + maxPoint) * 0.5f);
    return !mesh.triangles.isEmpty();
}

} // namespace

bool load(const QString& filePath, Mesh& mesh, QString& errorMessage)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        errorMessage = QStringLiteral("无法打开STL文件");
        return false;
    }

    const QByteArray data = file.readAll();
    const bool binarySizeMatch = data.size() >= 84 &&
        data.size() == 84 + qsizetype(readUInt32(data.constData() + 80)) * 50;

    const bool ok = binarySizeMatch ? loadBinary(data, mesh) : loadAscii(data, mesh);
    if (!ok) {
        errorMessage = QStringLiteral("STL文件中没有可显示的三角面");
        return false;
    }
    return true;
}

} // namespace StlModel

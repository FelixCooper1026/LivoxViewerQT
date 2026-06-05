#include "plugins/StlModel/StlModelLoader.h"

#include <algorithm>
#include <cstring>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector4D>

namespace StlModel {
namespace {

constexpr quint32 kGlbMagic = 0x46546C67;
constexpr quint32 kGlbVersion = 2;
constexpr quint32 kJsonChunkType = 0x4E4F534A;
constexpr quint32 kBinChunkType = 0x004E4942;

quint32 readUInt32(const char* data)
{
    quint32 value = 0;
    std::memcpy(&value, data, sizeof(quint32));
    return value;
}

float readFloat32(const char* data)
{
    float value = 0.0f;
    std::memcpy(&value, data, sizeof(float));
    return value;
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

struct AccessorData {
    const char* data = nullptr;
    int count = 0;
    int stride = 0;
    int componentType = 0;
    int components = 0;
    bool normalized = false;
};

int componentSize(int componentType)
{
    switch (componentType) {
    case 5120:
    case 5121:
        return 1;
    case 5122:
    case 5123:
        return 2;
    case 5125:
    case 5126:
        return 4;
    default:
        return 0;
    }
}

int componentCount(const QString& type)
{
    if (type == QStringLiteral("SCALAR")) {
        return 1;
    }
    if (type == QStringLiteral("VEC2")) {
        return 2;
    }
    if (type == QStringLiteral("VEC3")) {
        return 3;
    }
    if (type == QStringLiteral("VEC4")) {
        return 4;
    }
    return 0;
}

bool accessorData(const QJsonObject& root, const QByteArray& bin, int accessorIndex, AccessorData& out, QString& errorMessage)
{
    const QJsonArray accessors = root.value(QStringLiteral("accessors")).toArray();
    if (accessorIndex < 0 || accessorIndex >= accessors.size()) {
        errorMessage = QStringLiteral("GLB accessor索引无效");
        return false;
    }

    const QJsonObject accessor = accessors.at(accessorIndex).toObject();
    const int bufferViewIndex = accessor.value(QStringLiteral("bufferView")).toInt(-1);
    const QJsonArray bufferViews = root.value(QStringLiteral("bufferViews")).toArray();
    if (bufferViewIndex < 0 || bufferViewIndex >= bufferViews.size()) {
        errorMessage = QStringLiteral("GLB bufferView索引无效");
        return false;
    }

    const QJsonObject bufferView = bufferViews.at(bufferViewIndex).toObject();
    if (bufferView.value(QStringLiteral("buffer")).toInt(0) != 0) {
        errorMessage = QStringLiteral("GLB仅支持内嵌BIN buffer");
        return false;
    }

    out.count = accessor.value(QStringLiteral("count")).toInt();
    out.componentType = accessor.value(QStringLiteral("componentType")).toInt();
    out.components = componentCount(accessor.value(QStringLiteral("type")).toString());
    out.normalized = accessor.value(QStringLiteral("normalized")).toBool(false);
    const int elementSize = componentSize(out.componentType) * out.components;
    out.stride = bufferView.value(QStringLiteral("byteStride")).toInt(elementSize);

    const int byteOffset = bufferView.value(QStringLiteral("byteOffset")).toInt() +
        accessor.value(QStringLiteral("byteOffset")).toInt();
    const int byteLength = bufferView.value(QStringLiteral("byteLength")).toInt();
    const int required = out.count > 0 ? byteOffset + (out.count - 1) * out.stride + elementSize : byteOffset;
    if (out.count <= 0 || elementSize <= 0 || byteOffset < 0 || byteLength <= 0 || required > bin.size()) {
        errorMessage = QStringLiteral("GLB accessor数据越界");
        return false;
    }

    out.data = bin.constData() + byteOffset;
    return true;
}

float readComponent(const char* p, int componentType, bool normalized)
{
    switch (componentType) {
    case 5120: {
        const qint8 value = *reinterpret_cast<const qint8*>(p);
        return normalized ? std::max(-1.0f, float(value) / 127.0f) : float(value);
    }
    case 5121: {
        const quint8 value = *reinterpret_cast<const quint8*>(p);
        return normalized ? float(value) / 255.0f : float(value);
    }
    case 5122: {
        qint16 value = 0;
        std::memcpy(&value, p, sizeof(qint16));
        return normalized ? std::max(-1.0f, float(value) / 32767.0f) : float(value);
    }
    case 5123: {
        quint16 value = 0;
        std::memcpy(&value, p, sizeof(quint16));
        return normalized ? float(value) / 65535.0f : float(value);
    }
    case 5125:
        return float(readUInt32(p));
    case 5126:
        return readFloat32(p);
    default:
        return 0.0f;
    }
}

QVector3D readPosition(const AccessorData& accessor, int index)
{
    const char* p = accessor.data + index * accessor.stride;
    const int size = componentSize(accessor.componentType);
    return QVector3D(readComponent(p, accessor.componentType, accessor.normalized),
                     readComponent(p + size, accessor.componentType, accessor.normalized),
                     readComponent(p + size * 2, accessor.componentType, accessor.normalized));
}

QVector4D readVec4(const AccessorData& accessor, int index)
{
    const char* p = accessor.data + index * accessor.stride;
    const int size = componentSize(accessor.componentType);
    return QVector4D(readComponent(p, accessor.componentType, accessor.normalized),
                     readComponent(p + size, accessor.componentType, accessor.normalized),
                     readComponent(p + size * 2, accessor.componentType, accessor.normalized),
                     readComponent(p + size * 3, accessor.componentType, accessor.normalized));
}

quint32 readIndex(const AccessorData& accessor, int index)
{
    const char* p = accessor.data + index * accessor.stride;
    switch (accessor.componentType) {
    case 5121:
        return quint8(*p);
    case 5123: {
        quint16 value = 0;
        std::memcpy(&value, p, sizeof(quint16));
        return value;
    }
    case 5125:
        return readUInt32(p);
    default:
        return 0;
    }
}

QMatrix4x4 nodeTransform(const QJsonObject& node)
{
    if (node.contains(QStringLiteral("matrix"))) {
        const QJsonArray m = node.value(QStringLiteral("matrix")).toArray();
        return QMatrix4x4(float(m.at(0).toDouble()), float(m.at(4).toDouble()), float(m.at(8).toDouble()), float(m.at(12).toDouble()),
                          float(m.at(1).toDouble()), float(m.at(5).toDouble()), float(m.at(9).toDouble()), float(m.at(13).toDouble()),
                          float(m.at(2).toDouble()), float(m.at(6).toDouble()), float(m.at(10).toDouble()), float(m.at(14).toDouble()),
                          float(m.at(3).toDouble()), float(m.at(7).toDouble()), float(m.at(11).toDouble()), float(m.at(15).toDouble()));
    }

    QMatrix4x4 transform;
    if (node.contains(QStringLiteral("translation"))) {
        const QJsonArray t = node.value(QStringLiteral("translation")).toArray();
        transform.translate(float(t.at(0).toDouble()), float(t.at(1).toDouble()), float(t.at(2).toDouble()));
    }
    if (node.contains(QStringLiteral("rotation"))) {
        const QJsonArray r = node.value(QStringLiteral("rotation")).toArray();
        transform.rotate(QQuaternion(float(r.at(3).toDouble()), float(r.at(0).toDouble()), float(r.at(1).toDouble()), float(r.at(2).toDouble())));
    }
    if (node.contains(QStringLiteral("scale"))) {
        const QJsonArray s = node.value(QStringLiteral("scale")).toArray();
        transform.scale(float(s.at(0).toDouble()), float(s.at(1).toDouble()), float(s.at(2).toDouble()));
    }
    return transform;
}

void addTriangle(Mesh& mesh,
                 const QVector3D& a,
                 const QVector3D& b,
                 const QVector3D& c,
                 QVector3D& minPoint,
                 QVector3D& maxPoint,
                 bool& firstPoint)
{
    const QVector3D normal = triangleNormal(a, b, c);
    const QVector3D color = colorForNormal(normal);
    const QVector3D points[3] = {a, b, c};
    for (const QVector3D& point : points) {
        if (firstPoint) {
            minPoint = point;
            maxPoint = point;
            firstPoint = false;
        } else {
            updateBounds(minPoint, maxPoint, point);
        }
        mesh.triangles.push_back({point.x(), point.y(), point.z(), color.x(), color.y(), color.z()});
    }
}

bool appendPrimitive(const QJsonObject& root,
                     const QByteArray& bin,
                     const QJsonObject& primitive,
                     const QMatrix4x4& transform,
                     Mesh& mesh,
                     QVector3D& minPoint,
                     QVector3D& maxPoint,
                     bool& firstPoint,
                     QString& errorMessage)
{
    if (primitive.value(QStringLiteral("mode")).toInt(4) != 4) {
        return true;
    }
    if (primitive.value(QStringLiteral("extensions")).toObject().contains(QStringLiteral("KHR_draco_mesh_compression"))) {
        errorMessage = QStringLiteral("GLB使用KHR_draco_mesh_compression压缩，当前加载器不支持Draco压缩网格，请导出未压缩GLB");
        return false;
    }

    const QJsonObject attributes = primitive.value(QStringLiteral("attributes")).toObject();
    const int positionAccessorIndex = attributes.value(QStringLiteral("POSITION")).toInt(-1);
    AccessorData positions;
    if (!accessorData(root, bin, positionAccessorIndex, positions, errorMessage)) {
        return false;
    }
    if (positions.components != 3) {
        errorMessage = QStringLiteral("GLB POSITION必须是VEC3");
        return false;
    }

    if (primitive.contains(QStringLiteral("indices"))) {
        AccessorData indices;
        if (!accessorData(root, bin, primitive.value(QStringLiteral("indices")).toInt(), indices, errorMessage)) {
            return false;
        }
        if (indices.componentType != 5121 && indices.componentType != 5123 && indices.componentType != 5125) {
            errorMessage = QStringLiteral("GLB indices类型不支持");
            return false;
        }
        for (int i = 0; i + 2 < indices.count; i += 3) {
            const int ia = int(readIndex(indices, i));
            const int ib = int(readIndex(indices, i + 1));
            const int ic = int(readIndex(indices, i + 2));
            if (ia >= positions.count || ib >= positions.count || ic >= positions.count) {
                errorMessage = QStringLiteral("GLB indices引用越界");
                return false;
            }
            addTriangle(mesh,
                        transform.map(readPosition(positions, ia)),
                        transform.map(readPosition(positions, ib)),
                        transform.map(readPosition(positions, ic)),
                        minPoint,
                        maxPoint,
                        firstPoint);
        }
    } else {
        for (int i = 0; i + 2 < positions.count; i += 3) {
            addTriangle(mesh,
                        transform.map(readPosition(positions, i)),
                        transform.map(readPosition(positions, i + 1)),
                        transform.map(readPosition(positions, i + 2)),
                        minPoint,
                        maxPoint,
                        firstPoint);
        }
    }

    return true;
}

bool appendMesh(const QJsonObject& root,
                const QByteArray& bin,
                int meshIndex,
                const QMatrix4x4& transform,
                Mesh& mesh,
                QVector3D& minPoint,
                QVector3D& maxPoint,
                bool& firstPoint,
                QString& errorMessage)
{
    const QJsonArray meshes = root.value(QStringLiteral("meshes")).toArray();
    if (meshIndex < 0 || meshIndex >= meshes.size()) {
        errorMessage = QStringLiteral("GLB mesh索引无效");
        return false;
    }

    const QJsonArray primitives = meshes.at(meshIndex).toObject().value(QStringLiteral("primitives")).toArray();
    for (const QJsonValue& primitiveValue : primitives) {
        if (!appendPrimitive(root, bin, primitiveValue.toObject(), transform, mesh, minPoint, maxPoint, firstPoint, errorMessage)) {
            return false;
        }
    }
    return true;
}

bool appendNode(const QJsonObject& root,
                const QByteArray& bin,
                int nodeIndex,
                const QMatrix4x4& parentTransform,
                Mesh& mesh,
                QVector3D& minPoint,
                QVector3D& maxPoint,
                bool& firstPoint,
                QString& errorMessage)
{
    const QJsonArray nodes = root.value(QStringLiteral("nodes")).toArray();
    if (nodeIndex < 0 || nodeIndex >= nodes.size()) {
        errorMessage = QStringLiteral("GLB node索引无效");
        return false;
    }

    const QJsonObject node = nodes.at(nodeIndex).toObject();
    const QMatrix4x4 transform = parentTransform * nodeTransform(node);
    if (node.contains(QStringLiteral("mesh"))) {
        const QJsonObject instancing = node.value(QStringLiteral("extensions")).toObject()
            .value(QStringLiteral("EXT_mesh_gpu_instancing")).toObject();
        const QJsonObject instanceAttributes = instancing.value(QStringLiteral("attributes")).toObject();
        if (!instanceAttributes.isEmpty()) {
            AccessorData translations;
            AccessorData rotations;
            AccessorData scales;
            const bool hasTranslations = instanceAttributes.contains(QStringLiteral("TRANSLATION"));
            const bool hasRotations = instanceAttributes.contains(QStringLiteral("ROTATION"));
            const bool hasScales = instanceAttributes.contains(QStringLiteral("SCALE"));
            int instanceCount = 0;
            if (hasTranslations) {
                if (!accessorData(root, bin, instanceAttributes.value(QStringLiteral("TRANSLATION")).toInt(), translations, errorMessage)) {
                    return false;
                }
                instanceCount = translations.count;
            }
            if (hasRotations) {
                if (!accessorData(root, bin, instanceAttributes.value(QStringLiteral("ROTATION")).toInt(), rotations, errorMessage)) {
                    return false;
                }
                instanceCount = rotations.count;
            }
            if (hasScales) {
                if (!accessorData(root, bin, instanceAttributes.value(QStringLiteral("SCALE")).toInt(), scales, errorMessage)) {
                    return false;
                }
                instanceCount = scales.count;
            }
            for (int i = 0; i < instanceCount; ++i) {
                QMatrix4x4 instanceTransform;
                if (hasTranslations) {
                    instanceTransform.translate(readPosition(translations, i));
                }
                if (hasRotations) {
                    const QVector4D rotation = readVec4(rotations, i);
                    instanceTransform.rotate(QQuaternion(rotation.w(), rotation.x(), rotation.y(), rotation.z()));
                }
                if (hasScales) {
                    instanceTransform.scale(readPosition(scales, i));
                }
                if (!appendMesh(root, bin, node.value(QStringLiteral("mesh")).toInt(), transform * instanceTransform, mesh, minPoint, maxPoint, firstPoint, errorMessage)) {
                    return false;
                }
            }
        } else {
            if (!appendMesh(root, bin, node.value(QStringLiteral("mesh")).toInt(), transform, mesh, minPoint, maxPoint, firstPoint, errorMessage)) {
                return false;
            }
        }
    }

    const QJsonArray children = node.value(QStringLiteral("children")).toArray();
    for (const QJsonValue& childValue : children) {
        if (!appendNode(root, bin, childValue.toInt(), transform, mesh, minPoint, maxPoint, firstPoint, errorMessage)) {
            return false;
        }
    }
    return true;
}

bool parseGlbChunks(const QByteArray& data, QByteArray& jsonChunk, QByteArray& binChunk, QString& errorMessage)
{
    if (data.size() < 20 || readUInt32(data.constData()) != kGlbMagic || readUInt32(data.constData() + 4) != kGlbVersion) {
        errorMessage = QStringLiteral("不是GLB 2.0文件");
        return false;
    }

    const quint32 declaredLength = readUInt32(data.constData() + 8);
    if (declaredLength != quint32(data.size())) {
        errorMessage = QStringLiteral("GLB文件长度无效");
        return false;
    }

    qsizetype offset = 12;
    while (offset + 8 <= data.size()) {
        const quint32 chunkLength = readUInt32(data.constData() + offset);
        const quint32 chunkType = readUInt32(data.constData() + offset + 4);
        offset += 8;
        if (offset + chunkLength > data.size()) {
            errorMessage = QStringLiteral("GLB chunk越界");
            return false;
        }
        if (chunkType == kJsonChunkType) {
            jsonChunk = data.mid(offset, chunkLength);
        } else if (chunkType == kBinChunkType) {
            binChunk = data.mid(offset, chunkLength);
        }
        offset += chunkLength;
    }

    if (jsonChunk.isEmpty() || binChunk.isEmpty()) {
        errorMessage = QStringLiteral("GLB缺少JSON或BIN chunk");
        return false;
    }
    return true;
}

} // namespace

bool load(const QString& filePath, Mesh& mesh, QString& errorMessage)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        errorMessage = QStringLiteral("无法打开GLB文件");
        return false;
    }

    QByteArray jsonChunk;
    QByteArray binChunk;
    if (!parseGlbChunks(file.readAll(), jsonChunk, binChunk, errorMessage)) {
        return false;
    }

    const QJsonDocument document = QJsonDocument::fromJson(jsonChunk);
    if (!document.isObject()) {
        errorMessage = QStringLiteral("GLB JSON无效");
        return false;
    }

    const QJsonObject root = document.object();
    mesh.triangles.clear();
    QVector3D minPoint;
    QVector3D maxPoint;
    bool firstPoint = true;

    const QJsonArray scenes = root.value(QStringLiteral("scenes")).toArray();
    const int sceneIndex = root.value(QStringLiteral("scene")).toInt(0);
    if (sceneIndex < 0 || sceneIndex >= scenes.size()) {
        errorMessage = QStringLiteral("GLB scene索引无效");
        return false;
    }

    const QJsonArray nodes = scenes.at(sceneIndex).toObject().value(QStringLiteral("nodes")).toArray();
    for (const QJsonValue& nodeValue : nodes) {
        if (!appendNode(root, binChunk, nodeValue.toInt(), QMatrix4x4(), mesh, minPoint, maxPoint, firstPoint, errorMessage)) {
            return false;
        }
    }

    if (mesh.triangles.isEmpty()) {
        errorMessage = QStringLiteral("GLB文件中没有可显示的三角面");
        return false;
    }

    mesh.boundsMin = minPoint;
    mesh.boundsMax = maxPoint;
    centerMesh(mesh, (minPoint + maxPoint) * 0.5f);
    return true;
}

} // namespace StlModel

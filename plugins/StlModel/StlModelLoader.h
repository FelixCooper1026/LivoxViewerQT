#ifndef PLUGINS_STLMODEL_STLMODELLOADER_H
#define PLUGINS_STLMODEL_STLMODELLOADER_H

#include <QString>
#include <QVector>
#include <QVector3D>

namespace StlModel {

struct Vertex {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float r = 0.72f;
    float g = 0.76f;
    float b = 0.78f;
};

struct Mesh {
    QVector<Vertex> triangles;
    QVector3D boundsMin;
    QVector3D boundsMax;
};

bool load(const QString& filePath, Mesh& mesh, QString& errorMessage);

} // namespace StlModel

#endif // PLUGINS_STLMODEL_STLMODELLOADER_H

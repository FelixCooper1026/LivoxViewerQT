#ifndef LIVOXVIEWER_WIDGETS_IMUORIENTATIONVIEW_H
#define LIVOXVIEWER_WIDGETS_IMUORIENTATIONVIEW_H

#include "plugins/StlModel/StlModelLoader.h"

#include <QColor>
#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QQuaternion>
#include <QString>
#include <QVector>

struct ImuRenderVertex {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float r = 0.72f;
    float g = 0.76f;
    float b = 0.78f;
    float nx = 0.0f;
    float ny = 0.0f;
    float nz = 1.0f;
};

class ImuOrientationView : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
public:
    explicit ImuOrientationView(QWidget* parent = nullptr);
    ~ImuOrientationView() override;

    void setDeviceModelName(const QString& modelName);
    void setOrientation(const QQuaternion& orientation);
    void setHasData(bool hasData);
    void clearScene();
    void refreshTheme();

protected:
    void initializeGL() override;
    void paintGL() override;

private:
    void setupShaders();
    void setupBuffers();
    void uploadModelVertices();
    void uploadAxisVertices();
    void clearModel();

    QOpenGLShaderProgram* m_program = nullptr;
    QOpenGLBuffer m_modelVbo;
    QOpenGLVertexArrayObject m_modelVao;
    QOpenGLBuffer m_axisVbo;
    QOpenGLVertexArrayObject m_axisVao;

    QVector<ImuRenderVertex> m_modelVertices;
    QVector<ImuRenderVertex> m_axisVertices;
    QString m_modelKey;
    QString m_statusText;
    QQuaternion m_orientation;
    QColor m_backgroundColor;
    QColor m_textColor;
    QColor m_hintColor;
    int m_axisLineVertexCount = 0;
    bool m_modelLoaded = false;
    bool m_hasData = false;
    float m_modelScale = 1.0f;
};

#endif // LIVOXVIEWER_WIDGETS_IMUORIENTATIONVIEW_H

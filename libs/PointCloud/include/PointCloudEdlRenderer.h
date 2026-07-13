#ifndef POINTCLOUD_POINTCLOUDEDLRENDERER_H
#define POINTCLOUD_POINTCLOUDEDLRENDERER_H

#include "PointCloudRenderConfig.h"

#include <QColor>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLVertexArrayObject>
#include <QSize>
#include <QString>

class QOpenGLShaderProgram;

class PointCloudEdlRenderer : protected QOpenGLFunctions_3_3_Core
{
public:
    bool initialize();
    void destroy();

    bool beginScene(const QSize& framebufferSize);
    void composite(GLuint targetFramebuffer,
                   const QSize& targetSize,
                   const PointCloudEdlConfig& config,
                   float physicalRadius,
                   const QColor& backgroundTop,
                   const QColor& backgroundBottom);
    void destroyFramebuffer();

    bool isSupported() const { return m_shaderValid; }
    QString errorMessage() const { return m_errorMessage; }
    QSize framebufferSize() const { return m_framebufferSize; }

private:
    bool setupCompositeShader();
    bool ensureFramebuffer(const QSize& size);
    bool recreateFramebuffer(const QSize& size);

    GLuint m_framebuffer = 0;
    GLuint m_colorTexture = 0;
    GLuint m_linearDepthTexture = 0;
    GLuint m_depthRenderbuffer = 0;
    QOpenGLShaderProgram* m_compositeProgram = nullptr;
    QOpenGLVertexArrayObject m_fullscreenVao;
    QSize m_framebufferSize;
    QString m_errorMessage;
    bool m_shaderValid = false;
    bool m_resourcesValid = false;
};

#endif // POINTCLOUD_POINTCLOUDEDLRENDERER_H

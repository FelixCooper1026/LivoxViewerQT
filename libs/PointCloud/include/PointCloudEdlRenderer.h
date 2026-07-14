#ifndef POINTCLOUD_POINTCLOUDEDLRENDERER_H
#define POINTCLOUD_POINTCLOUDEDLRENDERER_H

#include "PointCloudRenderConfig.h"

#include <QOpenGLBuffer>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLVertexArrayObject>
#include <QSize>
#include <QString>

#include <array>

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
                   float nearPlane,
                   float farPlane,
                   bool perspectiveMode);
    void destroyFramebuffer();

    bool isSupported() const { return m_shaderValid; }
    QString errorMessage() const { return m_errorMessage; }
    QSize framebufferSize() const { return m_framebufferSize; }

private:
    struct ScalePass
    {
        GLuint shadeFramebuffer = 0;
        GLuint shadeTexture = 0;
        GLuint filteredFramebuffer = 0;
        GLuint filteredTexture = 0;
        QSize size;
    };

    bool setupShaders();
    bool ensureFramebuffer(const QSize& size);
    bool recreateFramebuffer(const QSize& size);
    void drawFullscreenTriangle();

    GLuint m_sceneFramebuffer = 0;
    GLuint m_sceneColorTexture = 0;
    GLuint m_sceneDepthTexture = 0;
    std::array<ScalePass, 3> m_scalePasses;
    QOpenGLShaderProgram* m_shadeProgram = nullptr;
    QOpenGLShaderProgram* m_bilateralProgram = nullptr;
    QOpenGLShaderProgram* m_mixProgram = nullptr;
    QOpenGLVertexArrayObject m_fullscreenVao;
    QOpenGLBuffer m_fullscreenVbo{QOpenGLBuffer::VertexBuffer};
    QSize m_framebufferSize;
    QString m_errorMessage;
    bool m_shaderValid = false;
    bool m_resourcesValid = false;
};

#endif // POINTCLOUD_POINTCLOUDEDLRENDERER_H

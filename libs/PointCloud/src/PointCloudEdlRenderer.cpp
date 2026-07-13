#include "PointCloudEdlRenderer.h"

#include <QOpenGLShaderProgram>
#include <QVector2D>
#include <QVector3D>

bool PointCloudEdlRenderer::initialize()
{
    initializeOpenGLFunctions();
    m_shaderValid = setupCompositeShader();
    if (m_shaderValid) {
        m_fullscreenVao.create();
    }
    return m_shaderValid;
}

void PointCloudEdlRenderer::destroy()
{
    destroyFramebuffer();
    m_fullscreenVao.destroy();
    delete m_compositeProgram;
    m_compositeProgram = nullptr;
    m_shaderValid = false;
}

bool PointCloudEdlRenderer::setupCompositeShader()
{
    static const char* vertexShaderSource = R"(
        #version 330 core
        out vec2 vUv;

        void main()
        {
            vec2 positions[3] = vec2[](
                vec2(-1.0, -1.0),
                vec2( 3.0, -1.0),
                vec2(-1.0,  3.0)
            );
            vec2 position = positions[gl_VertexID];
            gl_Position = vec4(position, 0.0, 1.0);
            vUv = position * 0.5 + 0.5;
        }
    )";

    static const char* fragmentShaderSource = R"(
        #version 330 core
        in vec2 vUv;

        uniform sampler2D uSceneColor;
        uniform sampler2D uLinearDepth;
        uniform vec2 uInvFramebufferSize;
        uniform float uRadiusPx;
        uniform float uStrength;
        uniform float uSilhouetteStrength;
        uniform float uMinimumShade;
        uniform int uSampleCount;
        uniform vec3 uBackgroundTop;
        uniform vec3 uBackgroundBottom;

        out vec4 FragColor;

        float depthResponse(float centerDepth, float neighborDepth)
        {
            if (neighborDepth <= 0.0) {
                return uSilhouetteStrength;
            }
            float centerLog = log2(max(centerDepth, 0.0001));
            float neighborLog = log2(max(neighborDepth, 0.0001));
            return max(neighborLog - centerLog, 0.0);
        }

        void main()
        {
            vec4 scene = texture(uSceneColor, vUv);
            float centerDepth = texture(uLinearDepth, vUv).r;
            vec3 background = mix(uBackgroundBottom, uBackgroundTop, vUv.y);

            if (centerDepth <= 0.0) {
                FragColor = vec4(background, 1.0);
                return;
            }
            if (scene.a < 0.75) {
                FragColor = vec4(scene.rgb, 1.0);
                return;
            }

            const vec2 directions[8] = vec2[](
                vec2( 1.0,  0.0),
                vec2(-1.0,  0.0),
                vec2( 0.0,  1.0),
                vec2( 0.0, -1.0),
                vec2( 0.7071,  0.7071),
                vec2(-0.7071,  0.7071),
                vec2( 0.7071, -0.7071),
                vec2(-0.7071, -0.7071)
            );
            vec2 offset = uInvFramebufferSize * uRadiusPx;
            int count = clamp(uSampleCount, 4, 8);
            float response = 0.0;
            for (int i = 0; i < 8; ++i) {
                if (i >= count) {
                    break;
                }
                float neighborDepth = texture(uLinearDepth, vUv + directions[i] * offset).r;
                response += depthResponse(centerDepth, neighborDepth);
            }
            response /= float(count);

            float shade = exp(-uStrength * response);
            shade = clamp(shade, uMinimumShade, 1.0);
            FragColor = vec4(scene.rgb * shade, 1.0);
        }
    )";

    m_compositeProgram = new QOpenGLShaderProgram();
    if (!m_compositeProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource) ||
        !m_compositeProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource) ||
        !m_compositeProgram->link()) {
        m_errorMessage = m_compositeProgram->log();
        delete m_compositeProgram;
        m_compositeProgram = nullptr;
        return false;
    }
    return true;
}

bool PointCloudEdlRenderer::beginScene(const QSize& framebufferSize)
{
    if (!m_shaderValid || !ensureFramebuffer(framebufferSize)) {
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, m_framebuffer);
    glViewport(0, 0, framebufferSize.width(), framebufferSize.height());
    const GLfloat clearColor[] = {0.0f, 0.0f, 0.0f, 0.0f};
    const GLfloat clearLinearDepth[] = {0.0f};
    const GLfloat clearDepth = 1.0f;
    glClearBufferfv(GL_COLOR, 0, clearColor);
    glClearBufferfv(GL_COLOR, 1, clearLinearDepth);
    glClearBufferfv(GL_DEPTH, 0, &clearDepth);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glDisable(GL_BLEND);
    return true;
}

void PointCloudEdlRenderer::composite(GLuint targetFramebuffer,
                                      const QSize& targetSize,
                                      const PointCloudEdlConfig& config,
                                      float physicalRadius,
                                      const QColor& backgroundTop,
                                      const QColor& backgroundBottom)
{
    glBindFramebuffer(GL_FRAMEBUFFER, targetFramebuffer);
    glViewport(0, 0, targetSize.width(), targetSize.height());
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDisable(GL_BLEND);

    m_compositeProgram->bind();
    m_compositeProgram->setUniformValue("uSceneColor", 0);
    m_compositeProgram->setUniformValue("uLinearDepth", 1);
    m_compositeProgram->setUniformValue(
        "uInvFramebufferSize",
        QVector2D(1.0f / float(m_framebufferSize.width()),
                  1.0f / float(m_framebufferSize.height())));
    m_compositeProgram->setUniformValue("uRadiusPx", physicalRadius);
    m_compositeProgram->setUniformValue("uStrength", config.strength);
    m_compositeProgram->setUniformValue("uSilhouetteStrength", config.silhouetteStrength);
    m_compositeProgram->setUniformValue("uMinimumShade", config.minimumShade);
    m_compositeProgram->setUniformValue("uSampleCount", config.sampleCount);
    m_compositeProgram->setUniformValue(
        "uBackgroundTop",
        QVector3D(backgroundTop.redF(), backgroundTop.greenF(), backgroundTop.blueF()));
    m_compositeProgram->setUniformValue(
        "uBackgroundBottom",
        QVector3D(backgroundBottom.redF(), backgroundBottom.greenF(), backgroundBottom.blueF()));

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_colorTexture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_linearDepthTexture);
    m_fullscreenVao.bind();
    glDrawArrays(GL_TRIANGLES, 0, 3);
    m_fullscreenVao.release();
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
    m_compositeProgram->release();
}

bool PointCloudEdlRenderer::ensureFramebuffer(const QSize& size)
{
    if (size == m_framebufferSize) {
        return m_resourcesValid;
    }
    return recreateFramebuffer(size);
}

bool PointCloudEdlRenderer::recreateFramebuffer(const QSize& size)
{
    destroyFramebuffer();
    m_framebufferSize = size;

    glGenFramebuffers(1, &m_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, m_framebuffer);

    glGenTextures(1, &m_colorTexture);
    glBindTexture(GL_TEXTURE_2D, m_colorTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, size.width(), size.height(), 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_colorTexture, 0);

    glGenTextures(1, &m_linearDepthTexture);
    glBindTexture(GL_TEXTURE_2D, m_linearDepthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, size.width(), size.height(), 0,
                 GL_RED, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, m_linearDepthTexture, 0);

    glGenRenderbuffers(1, &m_depthRenderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, m_depthRenderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, size.width(), size.height());
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_depthRenderbuffer);

    const GLenum drawBuffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
    glDrawBuffers(2, drawBuffers);
    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    m_resourcesValid = status == GL_FRAMEBUFFER_COMPLETE;
    if (!m_resourcesValid) {
        m_errorMessage = QStringLiteral("EDL framebuffer incomplete: 0x%1")
                             .arg(static_cast<unsigned int>(status), 0, 16);
    }
    return m_resourcesValid;
}

void PointCloudEdlRenderer::destroyFramebuffer()
{
    if (m_depthRenderbuffer != 0) {
        glDeleteRenderbuffers(1, &m_depthRenderbuffer);
        m_depthRenderbuffer = 0;
    }
    if (m_linearDepthTexture != 0) {
        glDeleteTextures(1, &m_linearDepthTexture);
        m_linearDepthTexture = 0;
    }
    if (m_colorTexture != 0) {
        glDeleteTextures(1, &m_colorTexture);
        m_colorTexture = 0;
    }
    if (m_framebuffer != 0) {
        glDeleteFramebuffers(1, &m_framebuffer);
        m_framebuffer = 0;
    }
    m_framebufferSize = QSize();
    m_resourcesValid = false;
}

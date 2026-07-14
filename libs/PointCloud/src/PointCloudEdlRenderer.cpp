#include "PointCloudEdlRenderer.h"

#include <QOpenGLShaderProgram>
#include <QVector2D>

// qEDL pipeline adapted from CloudCompare's qEDL plugin.
// Original copyright: EDF R&D / TELECOM ParisTech (ENST-TSI), GPL v2 or later.

bool PointCloudEdlRenderer::initialize()
{
    initializeOpenGLFunctions();
    m_shaderValid = setupShaders();
    if (!m_shaderValid) {
        return false;
    }

    static const GLfloat fullscreenTriangle[] = {
        -1.0f, -1.0f,
         3.0f, -1.0f,
        -1.0f,  3.0f
    };
    m_fullscreenVao.create();
    m_fullscreenVbo.create();
    m_fullscreenVao.bind();
    m_fullscreenVbo.bind();
    m_fullscreenVbo.allocate(fullscreenTriangle, sizeof(fullscreenTriangle));
    m_shadeProgram->bind();
    m_shadeProgram->enableAttributeArray(0);
    m_shadeProgram->setAttributeBuffer(0, GL_FLOAT, 0, 2);
    m_shadeProgram->release();
    m_fullscreenVbo.release();
    m_fullscreenVao.release();
    return true;
}

void PointCloudEdlRenderer::destroy()
{
    destroyFramebuffer();
    m_fullscreenVbo.destroy();
    m_fullscreenVao.destroy();
    delete m_mixProgram;
    m_mixProgram = nullptr;
    delete m_bilateralProgram;
    m_bilateralProgram = nullptr;
    delete m_shadeProgram;
    m_shadeProgram = nullptr;
    m_shaderValid = false;
}

bool PointCloudEdlRenderer::setupShaders()
{
    static const char* vertexShaderSource = R"(
        #version 330 core
        layout(location = 0) in vec2 aPosition;
        out vec2 vUv;

        void main()
        {
            gl_Position = vec4(aPosition, 0.0, 1.0);
            vUv = aPosition * 0.5 + 0.5;
        }
    )";

    static const char* shadeFragmentShaderSource = R"(
        #version 330 core
        in vec2 vUv;
        layout(location = 0) out vec4 FragColor;

        uniform sampler2D uSceneColor;
        uniform sampler2D uSceneDepth;
        uniform vec2 uInvSceneSize;
        uniform float uPixelScale;
        uniform float uNeighborDistancePx;
        uniform float uStrength;
        uniform float uNearPlane;
        uniform float uFarPlane;
        uniform int uPerspectiveMode;

        bool excludesEdl(float category)
        {
            return category > 0.25 && category < 0.75;
        }

        float fixDepth(float depth)
        {
            if (uPerspectiveMode == 1) {
                depth = (2.0 * uFarPlane * uNearPlane)
                    / ((uFarPlane + uNearPlane)
                       - (2.0 * depth - 1.0) * (uFarPlane - uNearPlane));
                depth = (depth - uNearPlane) / (uFarPlane - uNearPlane);
            }
            return clamp(1.0 - depth, 0.0, 1.0);
        }

        void main()
        {
            vec4 source = texture(uSceneColor, vUv);
            float centerDepth = fixDepth(texture(uSceneDepth, vUv).r);
            if (centerDepth <= 0.001 || excludesEdl(source.a)) {
                FragColor = vec4(source.rgb, 1.0);
                return;
            }

            const vec2 directions[8] = vec2[](
                vec2( 1.0,  0.0),
                vec2( 0.70710678,  0.70710678),
                vec2( 0.0,  1.0),
                vec2(-0.70710678,  0.70710678),
                vec2(-1.0,  0.0),
                vec2(-0.70710678, -0.70710678),
                vec2( 0.0, -1.0),
                vec2( 0.70710678, -0.70710678)
            );
            const vec3 lightDirection = vec3(0.0, 0.0, 1.0);
            vec4 lightPlane = vec4(
                lightDirection,
                -dot(lightDirection, vec3(0.0, 0.0, centerDepth)));
            float obscurance = 0.0;

            for (int i = 0; i < 8; ++i) {
                vec2 relativePosition = uPixelScale
                    * uNeighborDistancePx
                    * uInvSceneSize
                    * directions[i];
                vec2 neighborUv = vUv + relativePosition;
                vec4 neighborColor = texture(uSceneColor, neighborUv);
                float neighborDepth = excludesEdl(neighborColor.a)
                    ? centerDepth
                    : fixDepth(texture(uSceneDepth, neighborUv).r);
                float planeResponse = dot(
                    vec4(relativePosition, neighborDepth, 1.0),
                    lightPlane);
                obscurance += max(planeResponse, 0.0) / uPixelScale;
            }

            float shade = exp(-uStrength * obscurance);
            FragColor = vec4(source.rgb * shade, 1.0);
        }
    )";

    static const char* bilateralFragmentShaderSource = R"(
        #version 330 core
        in vec2 vUv;
        layout(location = 0) out vec4 FragColor;

        uniform sampler2D uSourceTexture;
        uniform sampler2D uSceneDepth;
        uniform vec2 uInvFilterSize;

        void main()
        {
            const float sigmaDepth = 0.4;
            const float spatialDenominator = 32.0;
            float centerDepth = texture(uSceneDepth, vUv).r;
            float weightSum = 0.0;
            vec3 colorSum = vec3(0.0);

            for (int x = -2; x <= 2; ++x) {
                for (int y = -2; y <= 2; ++y) {
                    vec2 offset = vec2(float(x), float(y)) * uInvFilterSize;
                    float spatialWeight = exp(
                        -float(x * x + y * y) / spatialDenominator);
                    float neighborDepth = texture(uSceneDepth, vUv + offset).r;
                    float depthDelta = (centerDepth - neighborDepth) / sigmaDepth;
                    float weight = spatialWeight * exp(-depthDelta * depthDelta * 0.5);
                    colorSum += texture(uSourceTexture, vUv + offset).rgb * weight;
                    weightSum += weight;
                }
            }

            FragColor = vec4(colorSum / weightSum, 1.0);
        }
    )";

    static const char* mixFragmentShaderSource = R"(
        #version 330 core
        in vec2 vUv;
        layout(location = 0) out vec4 FragColor;

        uniform sampler2D uFullTexture;
        uniform sampler2D uHalfTexture;
        uniform sampler2D uQuarterTexture;
        uniform sampler2D uSceneDepth;
        uniform sampler2D uSceneColor;

        bool excludesEdl(float category)
        {
            return category > 0.25 && category < 0.75;
        }

        void main()
        {
            vec3 fullColor = texture(uFullTexture, vUv).rgb;
            float rawDepth = texture(uSceneDepth, vUv).r;
            float category = texture(uSceneColor, vUv).a;
            if (rawDepth > 0.999 || excludesEdl(category)) {
                FragColor = vec4(fullColor, 1.0);
                return;
            }

            vec3 halfColor = texture(uHalfTexture, vUv).rgb;
            vec3 quarterColor = texture(uQuarterTexture, vUv).rgb;
            FragColor = vec4(
                (fullColor + 0.5 * halfColor + 0.25 * quarterColor) / 1.75,
                1.0);
        }
    )";

    auto buildProgram = [this](QOpenGLShaderProgram*& program,
                               const char* fragmentSource,
                               const QString& label) {
        program = new QOpenGLShaderProgram();
        if (!program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource)
            || !program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentSource)
            || !program->link()) {
            m_errorMessage = QStringLiteral("%1 shader failed: %2").arg(label, program->log());
            delete program;
            program = nullptr;
            return false;
        }
        return true;
    };

    return buildProgram(m_shadeProgram, shadeFragmentShaderSource, QStringLiteral("EDL shade"))
        && buildProgram(m_bilateralProgram, bilateralFragmentShaderSource, QStringLiteral("EDL bilateral"))
        && buildProgram(m_mixProgram, mixFragmentShaderSource, QStringLiteral("EDL mix"));
}

bool PointCloudEdlRenderer::beginScene(const QSize& framebufferSize)
{
    if (!m_shaderValid || !ensureFramebuffer(framebufferSize)) {
        return false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, m_sceneFramebuffer);
    const GLenum drawBuffer = GL_COLOR_ATTACHMENT0;
    glDrawBuffers(1, &drawBuffer);
    glViewport(0, 0, framebufferSize.width(), framebufferSize.height());
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDisable(GL_RASTERIZER_DISCARD);
    glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
    glDisable(GL_SAMPLE_COVERAGE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glDepthRange(0.0, 1.0);
    const GLfloat clearColor[] = {0.0f, 0.0f, 0.0f, 1.0f};
    const GLfloat clearDepth = 1.0f;
    glClearBufferfv(GL_COLOR, 0, clearColor);
    glClearBufferfv(GL_DEPTH, 0, &clearDepth);
    return true;
}

void PointCloudEdlRenderer::composite(GLuint targetFramebuffer,
                                      const QSize& targetSize,
                                      const PointCloudEdlConfig& config,
                                      float physicalRadius,
                                      float nearPlane,
                                      float farPlane,
                                      bool perspectiveMode)
{
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glDisable(GL_RASTERIZER_DISCARD);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_FALSE);

    m_shadeProgram->bind();
    m_shadeProgram->setUniformValue("uSceneColor", 0);
    m_shadeProgram->setUniformValue("uSceneDepth", 1);
    m_shadeProgram->setUniformValue(
        "uInvSceneSize",
        QVector2D(1.0f / float(m_framebufferSize.width()),
                  1.0f / float(m_framebufferSize.height())));
    m_shadeProgram->setUniformValue("uNeighborDistancePx", physicalRadius);
    m_shadeProgram->setUniformValue("uStrength", config.strength);
    m_shadeProgram->setUniformValue("uNearPlane", nearPlane);
    m_shadeProgram->setUniformValue("uFarPlane", farPlane);
    m_shadeProgram->setUniformValue("uPerspectiveMode", perspectiveMode ? 1 : 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_sceneColorTexture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_sceneDepthTexture);

    for (int i = 0; i < int(m_scalePasses.size()); ++i) {
        const ScalePass& pass = m_scalePasses[std::size_t(i)];
        glBindFramebuffer(GL_FRAMEBUFFER, pass.shadeFramebuffer);
        const GLenum drawBuffer = GL_COLOR_ATTACHMENT0;
        glDrawBuffers(1, &drawBuffer);
        glViewport(0, 0, pass.size.width(), pass.size.height());
        m_shadeProgram->setUniformValue("uPixelScale", float(1 << i));
        drawFullscreenTriangle();
    }
    m_shadeProgram->release();

    m_bilateralProgram->bind();
    m_bilateralProgram->setUniformValue("uSourceTexture", 0);
    m_bilateralProgram->setUniformValue("uSceneDepth", 1);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_sceneDepthTexture);
    for (int i = 1; i < int(m_scalePasses.size()); ++i) {
        const ScalePass& pass = m_scalePasses[std::size_t(i)];
        glBindFramebuffer(GL_FRAMEBUFFER, pass.filteredFramebuffer);
        const GLenum drawBuffer = GL_COLOR_ATTACHMENT0;
        glDrawBuffers(1, &drawBuffer);
        glViewport(0, 0, pass.size.width(), pass.size.height());
        m_bilateralProgram->setUniformValue(
            "uInvFilterSize",
            QVector2D(1.0f / float(pass.size.width()),
                      1.0f / float(pass.size.height())));
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, pass.shadeTexture);
        drawFullscreenTriangle();
    }
    m_bilateralProgram->release();

    glBindFramebuffer(GL_FRAMEBUFFER, targetFramebuffer);
    if (targetFramebuffer == 0) {
        glDrawBuffer(GL_BACK);
    } else {
        const GLenum drawBuffer = GL_COLOR_ATTACHMENT0;
        glDrawBuffers(1, &drawBuffer);
    }
    glViewport(0, 0, targetSize.width(), targetSize.height());
    m_mixProgram->bind();
    m_mixProgram->setUniformValue("uFullTexture", 0);
    m_mixProgram->setUniformValue("uHalfTexture", 1);
    m_mixProgram->setUniformValue("uQuarterTexture", 2);
    m_mixProgram->setUniformValue("uSceneDepth", 3);
    m_mixProgram->setUniformValue("uSceneColor", 4);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_scalePasses[0].shadeTexture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, m_scalePasses[1].filteredTexture);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, m_scalePasses[2].filteredTexture);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, m_sceneDepthTexture);
    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_2D, m_sceneColorTexture);
    drawFullscreenTriangle();
    m_mixProgram->release();

    for (int unit = 4; unit >= 0; --unit) {
        glActiveTexture(GL_TEXTURE0 + unit);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
    glActiveTexture(GL_TEXTURE0);
}

void PointCloudEdlRenderer::drawFullscreenTriangle()
{
    m_fullscreenVao.bind();
    glDrawArrays(GL_TRIANGLES, 0, 3);
    m_fullscreenVao.release();
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

    auto createColorTexture = [this](GLuint& texture, const QSize& textureSize) {
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8,
                     textureSize.width(), textureSize.height(), 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    };
    auto framebufferComplete = [this](const QString& label) {
        const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE) {
            m_errorMessage = QStringLiteral("%1 framebuffer incomplete: 0x%2")
                                 .arg(label)
                                 .arg(static_cast<unsigned int>(status), 0, 16);
            return false;
        }
        return true;
    };

    glGenFramebuffers(1, &m_sceneFramebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, m_sceneFramebuffer);
    createColorTexture(m_sceneColorTexture, size);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, m_sceneColorTexture, 0);

    glGenTextures(1, &m_sceneDepthTexture);
    glBindTexture(GL_TEXTURE_2D, m_sceneDepthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F,
                 size.width(), size.height(), 0,
                 GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D, m_sceneDepthTexture, 0);
    const GLenum sceneDrawBuffer = GL_COLOR_ATTACHMENT0;
    glDrawBuffers(1, &sceneDrawBuffer);
    if (!framebufferComplete(QStringLiteral("EDL scene"))) {
        return false;
    }

    for (int i = 0; i < int(m_scalePasses.size()); ++i) {
        ScalePass& pass = m_scalePasses[std::size_t(i)];
        const int scale = 1 << i;
        pass.size = QSize(qMax(1, size.width() / scale),
                          qMax(1, size.height() / scale));
        glGenFramebuffers(1, &pass.shadeFramebuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, pass.shadeFramebuffer);
        createColorTexture(pass.shadeTexture, pass.size);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, pass.shadeTexture, 0);
        const GLenum drawBuffer = GL_COLOR_ATTACHMENT0;
        glDrawBuffers(1, &drawBuffer);
        if (!framebufferComplete(QStringLiteral("EDL 1:%1 shade").arg(scale))) {
            return false;
        }

        if (i == 0) {
            continue;
        }
        glGenFramebuffers(1, &pass.filteredFramebuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, pass.filteredFramebuffer);
        createColorTexture(pass.filteredTexture, pass.size);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, pass.filteredTexture, 0);
        glDrawBuffers(1, &drawBuffer);
        if (!framebufferComplete(QStringLiteral("EDL 1:%1 bilateral").arg(scale))) {
            return false;
        }
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    m_resourcesValid = true;
    m_errorMessage.clear();
    return true;
}

void PointCloudEdlRenderer::destroyFramebuffer()
{
    for (ScalePass& pass : m_scalePasses) {
        if (pass.filteredTexture != 0) {
            glDeleteTextures(1, &pass.filteredTexture);
            pass.filteredTexture = 0;
        }
        if (pass.filteredFramebuffer != 0) {
            glDeleteFramebuffers(1, &pass.filteredFramebuffer);
            pass.filteredFramebuffer = 0;
        }
        if (pass.shadeTexture != 0) {
            glDeleteTextures(1, &pass.shadeTexture);
            pass.shadeTexture = 0;
        }
        if (pass.shadeFramebuffer != 0) {
            glDeleteFramebuffers(1, &pass.shadeFramebuffer);
            pass.shadeFramebuffer = 0;
        }
        pass.size = QSize();
    }
    if (m_sceneDepthTexture != 0) {
        glDeleteTextures(1, &m_sceneDepthTexture);
        m_sceneDepthTexture = 0;
    }
    if (m_sceneColorTexture != 0) {
        glDeleteTextures(1, &m_sceneColorTexture);
        m_sceneColorTexture = 0;
    }
    if (m_sceneFramebuffer != 0) {
        glDeleteFramebuffers(1, &m_sceneFramebuffer);
        m_sceneFramebuffer = 0;
    }
    m_framebufferSize = QSize();
    m_resourcesValid = false;
}

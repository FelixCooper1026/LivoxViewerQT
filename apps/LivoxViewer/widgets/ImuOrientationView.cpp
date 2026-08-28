#include "widgets/ImuOrientationView.h"

#include "utils/DeviceModelResource.h"

#include <QFileInfo>
#include <QFontMetrics>
#include <QIcon>
#include <QMatrix3x3>
#include <QOpenGLContext>
#include <QPainter>
#include <QPalette>
#include <QPixmap>
#include <QVector3D>

#include <algorithm>
#include <cmath>

namespace {

constexpr float kTwoPi = 6.28318530717958647692f;
const QQuaternion kImuAxisAlignment = QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 0.0f, 1.0f), 180.0f);

class ScopedOpenGLContext
{
public:
    explicit ScopedOpenGLContext(QOpenGLWidget* widget)
        : m_widget(widget)
    {
        QOpenGLContext* widgetContext = widget ? widget->context() : nullptr;
        if (widgetContext && QOpenGLContext::currentContext() != widgetContext) {
            widget->makeCurrent();
            m_madeCurrent = true;
        }
    }

    ~ScopedOpenGLContext()
    {
        if (m_madeCurrent) {
            m_widget->doneCurrent();
        }
    }

private:
    QOpenGLWidget* m_widget = nullptr;
    bool m_madeCurrent = false;
};

ImuRenderVertex transformDeviceModelVertex(const StlModel::Vertex& vertex,
                                           bool sourceXReversed,
                                           float sourceUnitToMeters)
{
    return {
        (sourceXReversed ? vertex.x : -vertex.x) * sourceUnitToMeters,
        vertex.z * sourceUnitToMeters,
        vertex.y * sourceUnitToMeters,
        0.72f,
        0.76f,
        0.78f,
        0.0f,
        0.0f,
        1.0f
    };
}

QVector<ImuRenderVertex> transformDeviceModelVertices(const QVector<StlModel::Vertex>& vertices,
                                                      bool sourceXReversed,
                                                      float sourceUnitToMeters,
                                                      float& modelScale)
{
    QVector<ImuRenderVertex> transformed;
    transformed.reserve(vertices.size());

    QVector3D minPoint;
    QVector3D maxPoint;
    bool first = true;
    auto updateBounds = [&minPoint, &maxPoint, &first](const ImuRenderVertex& vertex) {
        const QVector3D point(vertex.x, vertex.y, vertex.z);
        if (first) {
            minPoint = point;
            maxPoint = point;
            first = false;
        } else {
            minPoint.setX(std::min(minPoint.x(), point.x()));
            minPoint.setY(std::min(minPoint.y(), point.y()));
            minPoint.setZ(std::min(minPoint.z(), point.z()));
            maxPoint.setX(std::max(maxPoint.x(), point.x()));
            maxPoint.setY(std::max(maxPoint.y(), point.y()));
            maxPoint.setZ(std::max(maxPoint.z(), point.z()));
        }
    };

    for (int i = 0; i + 2 < vertices.size(); i += 3) {
        ImuRenderVertex a = transformDeviceModelVertex(vertices.at(i), sourceXReversed, sourceUnitToMeters);
        ImuRenderVertex b = transformDeviceModelVertex(vertices.at(i + 1), sourceXReversed, sourceUnitToMeters);
        ImuRenderVertex c = transformDeviceModelVertex(vertices.at(i + 2), sourceXReversed, sourceUnitToMeters);
        QVector3D normal = QVector3D::crossProduct(
            QVector3D(b.x - a.x, b.y - a.y, b.z - a.z),
            QVector3D(c.x - a.x, c.y - a.y, c.z - a.z));
        if (normal.lengthSquared() > 0.0f) {
            normal.normalize();
        } else {
            normal = QVector3D(0.0f, 0.0f, 1.0f);
        }
        for (ImuRenderVertex* vertex : {&a, &b, &c}) {
            vertex->nx = normal.x();
            vertex->ny = normal.y();
            vertex->nz = normal.z();
            updateBounds(*vertex);
            transformed.push_back(*vertex);
        }
    }

    const QVector3D size = maxPoint - minPoint;
    const float maxDimension = std::max({size.x(), size.y(), size.z(), 0.1f});
    modelScale = 1.6f / maxDimension;
    return transformed;
}

QVector<ImuRenderVertex> buildAxisVertices()
{
    QVector<ImuRenderVertex> vertices = {
        {0.0f, 0.0f, 0.0f, 0.84f, 0.20f, 0.22f}, {1.0f, 0.0f, 0.0f, 0.84f, 0.20f, 0.22f},
        {0.0f, 0.0f, 0.0f, 0.18f, 0.62f, 0.36f}, {0.0f, 1.0f, 0.0f, 0.18f, 0.62f, 0.36f},
        {0.0f, 0.0f, 0.0f, 0.18f, 0.42f, 0.82f}, {0.0f, 0.0f, 1.0f, 0.18f, 0.42f, 0.82f}
    };
    auto appendCone = [&vertices](const QVector3D& direction, const QVector3D& color) {
        constexpr int kSegments = 18;
        constexpr float kTipDistance = 1.16f;
        constexpr float kBaseDistance = 0.94f;
        constexpr float kRadius = 0.055f;
        const QVector3D axis = direction.normalized();
        const QVector3D reference = std::abs(QVector3D::dotProduct(axis, QVector3D(0.0f, 0.0f, 1.0f))) > 0.9f
            ? QVector3D(0.0f, 1.0f, 0.0f)
            : QVector3D(0.0f, 0.0f, 1.0f);
        const QVector3D u = QVector3D::crossProduct(axis, reference).normalized();
        const QVector3D v = QVector3D::crossProduct(axis, u).normalized();
        const QVector3D tip = axis * kTipDistance;
        const QVector3D baseCenter = axis * kBaseDistance;
        auto makeVertex = [&color](const QVector3D& point) {
            return ImuRenderVertex{point.x(), point.y(), point.z(), color.x(), color.y(), color.z()};
        };
        for (int i = 0; i < kSegments; ++i) {
            const float a0 = float(i) * kTwoPi / float(kSegments);
            const float a1 = float(i + 1) * kTwoPi / float(kSegments);
            const QVector3D p0 = baseCenter + (std::cos(a0) * u + std::sin(a0) * v) * kRadius;
            const QVector3D p1 = baseCenter + (std::cos(a1) * u + std::sin(a1) * v) * kRadius;
            vertices.push_back(makeVertex(tip));
            vertices.push_back(makeVertex(p0));
            vertices.push_back(makeVertex(p1));
            vertices.push_back(makeVertex(baseCenter));
            vertices.push_back(makeVertex(p1));
            vertices.push_back(makeVertex(p0));
        }
    };
    appendCone(QVector3D(1.0f, 0.0f, 0.0f), QVector3D(0.84f, 0.20f, 0.22f));
    appendCone(QVector3D(0.0f, 1.0f, 0.0f), QVector3D(0.18f, 0.62f, 0.36f));
    appendCone(QVector3D(0.0f, 0.0f, 1.0f), QVector3D(0.18f, 0.42f, 0.82f));
    return vertices;
}

} // namespace

ImuOrientationView::ImuOrientationView(QWidget* parent)
    : QOpenGLWidget(parent)
    , m_modelVbo(QOpenGLBuffer::VertexBuffer)
    , m_axisVbo(QOpenGLBuffer::VertexBuffer)
{
    setMinimumSize(240, 180);
    m_axisVertices = buildAxisVertices();
    m_axisLineVertexCount = 6;
    refreshTheme();
}

ImuOrientationView::~ImuOrientationView()
{
    if (context()) {
        makeCurrent();
        m_modelVao.destroy();
        m_modelVbo.destroy();
        m_axisVao.destroy();
        m_axisVbo.destroy();
        delete m_program;
        m_program = nullptr;
        doneCurrent();
    }
}

void ImuOrientationView::setDeviceModelName(const QString& modelName)
{
    const QString modelKey = DeviceModelResource::modelKeyForName(modelName);
    if (modelKey == m_modelKey) {
        return;
    }

    m_modelKey = modelKey;
    if (modelKey.isEmpty()) {
        clearModel();
        m_statusText = QStringLiteral("未匹配到设备模型");
        return;
    }

    const QString filePath = DeviceModelResource::modelPathForKey(modelKey);
    if (!QFileInfo::exists(filePath)) {
        clearModel();
        m_statusText = QStringLiteral("未找到设备模型");
        return;
    }

    StlModel::Mesh mesh;
    QString errorMessage;
    if (!StlModel::load(filePath, mesh, errorMessage)) {
        clearModel();
        m_statusText = errorMessage;
        return;
    }

    m_modelVertices = transformDeviceModelVertices(
        mesh.triangles,
        DeviceModelResource::sourceXReversedForKey(modelKey),
        DeviceModelResource::sourceUnitToMetersForKey(modelKey),
        m_modelScale);
    m_modelLoaded = true;
    m_statusText.clear();
    uploadModelVertices();
    update();
}

void ImuOrientationView::setOrientation(const QQuaternion& orientation)
{
    if (m_orientation == orientation) {
        return;
    }
    m_orientation = orientation;
    update();
}

void ImuOrientationView::setHasData(bool hasData)
{
    if (m_hasData == hasData) {
        return;
    }
    m_hasData = hasData;
    update();
}

void ImuOrientationView::clearScene()
{
    m_hasData = false;
    m_orientation = QQuaternion();
    m_modelKey.clear();
    m_statusText.clear();
    clearModel();
}

void ImuOrientationView::refreshTheme()
{
    const QPalette pal = palette();
    m_backgroundColor = pal.color(QPalette::Base);
    m_textColor = pal.color(QPalette::WindowText);
    m_hintColor = pal.color(QPalette::Mid);
    update();
}

void ImuOrientationView::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    setupShaders();
    setupBuffers();
    uploadAxisVertices();
    uploadModelVertices();
}

void ImuOrientationView::paintGL()
{
    glViewport(0, 0, width() * devicePixelRatioF(), height() * devicePixelRatioF());
    glClearColor(m_backgroundColor.redF(), m_backgroundColor.greenF(), m_backgroundColor.blueF(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (m_hasData && m_program) {
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LESS);
        glDisable(GL_BLEND);

        m_program->bind();

        QMatrix4x4 projection;
        projection.perspective(38.0f, float(std::max(1, width())) / float(std::max(1, height())), 0.01f, 100.0f);

        QMatrix4x4 view;
        view.lookAt(QVector3D(2.15f, -2.75f, 1.7f),
                    QVector3D(0.0f, 0.0f, 0.0f),
                    QVector3D(0.0f, 0.0f, 1.0f));

        if (m_modelLoaded && !m_modelVertices.isEmpty()) {
            QMatrix4x4 model;
            model.rotate(m_orientation);
            model.rotate(DeviceModelResource::imuModelYawAlignmentDegreesForKey(m_modelKey),
                         QVector3D(0.0f, 0.0f, 1.0f));
            model.scale(m_modelScale);
            const QMatrix4x4 modelView = view * model;
            m_program->setUniformValue("mvp", projection * modelView);
            m_program->setUniformValue("modelView", modelView);
            m_program->setUniformValue("normalMatrix", modelView.normalMatrix());
            m_program->setUniformValue("useLighting", 1);
            m_modelVao.bind();
            glDrawArrays(GL_TRIANGLES, 0, m_modelVertices.size());
            m_modelVao.release();
        }

        QMatrix4x4 axesModel;
        axesModel.rotate(m_orientation);
        axesModel.rotate(kImuAxisAlignment);
        axesModel.scale(1.0f);
        const QMatrix4x4 axesModelView = view * axesModel;
        QMatrix3x3 axesNormalMatrix;
        axesNormalMatrix.setToIdentity();
        m_program->setUniformValue("mvp", projection * axesModelView);
        m_program->setUniformValue("modelView", axesModelView);
        m_program->setUniformValue("normalMatrix", axesNormalMatrix);
        m_program->setUniformValue("useLighting", 0);
        glLineWidth(2.2f);
        m_axisVao.bind();
        glDrawArrays(GL_LINES, 0, m_axisLineVertexCount);
        glDrawArrays(GL_TRIANGLES, m_axisLineVertexCount, m_axisVertices.size() - m_axisLineVertexCount);
        m_axisVao.release();
        glLineWidth(1.0f);

        m_program->release();
    }

    if (!m_hasData) {
        QPainter painter(this);
        painter.setRenderHint(QPainter::TextAntialiasing, true);
        painter.setPen(m_hintColor);
        const QSize iconSize(32, 32);
        const QPixmap pendingIcon = QIcon(QStringLiteral(":/icons/status_pending.svg")).pixmap(iconSize);
        const QString pendingText = QStringLiteral("等待 IMU 数据");
        const QFontMetrics fm(painter.font());
        constexpr int spacing = 6;
        const int textHeight = fm.boundingRect(pendingText).height();
        const int contentHeight = iconSize.height() + spacing + textHeight;
        const int top = rect().center().y() - contentHeight / 2;
        const int iconLeft = rect().center().x() - iconSize.width() / 2;
        painter.drawPixmap(QRect(iconLeft, top, iconSize.width(), iconSize.height()), pendingIcon);
        const QRect textRect(12, top + iconSize.height() + spacing, width() - 24, textHeight);
        painter.drawText(textRect, Qt::AlignCenter, pendingText);
        return;
    }

    const QString overlayText = m_modelLoaded ? QString() : m_statusText;
    if (!overlayText.isEmpty()) {
        QPainter painter(this);
        painter.setRenderHint(QPainter::TextAntialiasing, true);
        painter.setPen(m_textColor);
        painter.drawText(rect().adjusted(12, 12, -12, -12), Qt::AlignCenter | Qt::TextWordWrap, overlayText);
    }
}

void ImuOrientationView::setupShaders()
{
    m_program = new QOpenGLShaderProgram(this);
    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aColor;
        layout(location = 2) in vec3 aNormal;
        uniform mat4 mvp;
        uniform mat4 modelView;
        uniform mat3 normalMatrix;
        out vec3 vColor;
        out vec3 vNormal;
        out vec3 vViewPosition;
        void main() {
            vec4 viewPosition = modelView * vec4(aPos, 1.0);
            gl_Position = mvp * vec4(aPos, 1.0);
            vColor = aColor;
            vNormal = normalize(normalMatrix * aNormal);
            vViewPosition = viewPosition.xyz;
        }
    )");
    m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, R"(
        #version 330 core
        in vec3 vColor;
        in vec3 vNormal;
        in vec3 vViewPosition;
        uniform int useLighting;
        out vec4 FragColor;
        void main() {
            if (useLighting == 0) {
                FragColor = vec4(vColor, 1.0);
                return;
            }

            vec3 normal = normalize(vNormal);
            vec3 viewDir = normalize(-vViewPosition);
            vec3 keyLight = normalize(vec3(-0.45, 0.35, 0.82));
            vec3 fillLight = normalize(vec3(0.68, -0.32, 0.52));
            float diffuse = 0.46 * max(dot(normal, keyLight), 0.0)
                          + 0.22 * max(dot(normal, fillLight), 0.0);
            vec3 halfDir = normalize(keyLight + viewDir);
            float specular = 0.10 * pow(max(dot(normal, halfDir), 0.0), 32.0);
            vec3 color = vColor * (0.36 + diffuse) + vec3(specular);
            FragColor = vec4(clamp(color, 0.0, 1.0), 1.0);
        }
    )");
    m_program->link();
}

void ImuOrientationView::setupBuffers()
{
    m_modelVao.create();
    m_modelVao.bind();
    m_modelVbo.create();
    m_modelVbo.bind();
    m_modelVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_modelVbo.allocate(1);
    m_program->enableAttributeArray(0);
    m_program->setAttributeBuffer(0, GL_FLOAT, offsetof(ImuRenderVertex, x), 3, sizeof(ImuRenderVertex));
    m_program->enableAttributeArray(1);
    m_program->setAttributeBuffer(1, GL_FLOAT, offsetof(ImuRenderVertex, r), 3, sizeof(ImuRenderVertex));
    m_program->enableAttributeArray(2);
    m_program->setAttributeBuffer(2, GL_FLOAT, offsetof(ImuRenderVertex, nx), 3, sizeof(ImuRenderVertex));
    m_modelVao.release();
    m_modelVbo.release();

    m_axisVao.create();
    m_axisVao.bind();
    m_axisVbo.create();
    m_axisVbo.bind();
    m_axisVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_axisVbo.allocate(1);
    m_program->enableAttributeArray(0);
    m_program->setAttributeBuffer(0, GL_FLOAT, offsetof(ImuRenderVertex, x), 3, sizeof(ImuRenderVertex));
    m_program->enableAttributeArray(1);
    m_program->setAttributeBuffer(1, GL_FLOAT, offsetof(ImuRenderVertex, r), 3, sizeof(ImuRenderVertex));
    m_program->enableAttributeArray(2);
    m_program->setAttributeBuffer(2, GL_FLOAT, offsetof(ImuRenderVertex, nx), 3, sizeof(ImuRenderVertex));
    m_axisVao.release();
    m_axisVbo.release();
}

void ImuOrientationView::uploadModelVertices()
{
    if (!m_modelVbo.isCreated()) {
        return;
    }
    ScopedOpenGLContext current(this);
    m_modelVbo.bind();
    m_modelVbo.allocate(m_modelVertices.constData(), static_cast<int>(m_modelVertices.size() * qsizetype(sizeof(ImuRenderVertex))));
    m_modelVbo.release();
}

void ImuOrientationView::uploadAxisVertices()
{
    if (!m_axisVbo.isCreated()) {
        return;
    }
    ScopedOpenGLContext current(this);
    m_axisVbo.bind();
    m_axisVbo.allocate(m_axisVertices.constData(), static_cast<int>(m_axisVertices.size() * qsizetype(sizeof(ImuRenderVertex))));
    m_axisVbo.release();
}

void ImuOrientationView::clearModel()
{
    m_modelVertices.clear();
    m_modelLoaded = false;
    m_modelScale = 1.0f;
    uploadModelVertices();
    update();
}

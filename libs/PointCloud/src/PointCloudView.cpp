#include "PointCloudView.h"
#include "PointCloudEdlRenderer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <QMouseEvent>
#include <QPainter>
#include <QPointF>
#include <QRectF>
#include <QLinearGradient>
#include <QMatrix3x3>
#include <QCoreApplication>
#include <QOpenGLFunctions>
#include <QOpenGLContext>
#include <QMimeData>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QDebug>
#include <QPointer>
#include <QStringList>
#include <QThread>
#include <QUrl>
#include <QVector4D>
#include <QtConcurrent/QtConcurrentRun>

namespace {

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

constexpr float kModelMillimetersToMeters = 0.001f;
constexpr float kDefaultCameraDistance = 25.0f;
constexpr float kDefaultNearPlane = 0.1f;
constexpr float kMinimumNearPlane = 0.005f;
constexpr float kNearPlaneDistanceRatio = 0.05f;
constexpr float kSlamFollowDistanceM = 6.0f;
constexpr float kSlamFollowHeightM = 1.5f;
constexpr float kSlamFollowLookAheadM = 4.0f;
constexpr float kSlamFollowTargetHeightM = 0.5f;

bool isSupportedPlaybackDropFile(const QString& filePath)
{
    return filePath.endsWith(QStringLiteral(".lvx"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".lvx2"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".pcap"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".pcapng"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".cap"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".bag"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".db3"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".yaml"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".yml"), Qt::CaseInsensitive);
}

QStringList supportedPlaybackDropFiles(const QMimeData* mimeData)
{
    QStringList files;
    if (!mimeData || !mimeData->hasUrls()) {
        return files;
    }

    const QList<QUrl> urls = mimeData->urls();
    for (const QUrl& url : urls) {
        if (!url.isLocalFile()) {
            continue;
        }
        const QString localFile = url.toLocalFile();
        if (isSupportedPlaybackDropFile(localFile)) {
            files.append(localFile);
        }
    }
    return files;
}

StlRenderVertex transformDeviceModelVertex(const StlModel::Vertex& vertex, bool sourceXReversed)
{
    return {
        (sourceXReversed ? vertex.x : -vertex.x) * kModelMillimetersToMeters,
        vertex.z * kModelMillimetersToMeters,
        vertex.y * kModelMillimetersToMeters,
        0.72f,
        0.76f,
        0.78f,
        0.0f,
        0.0f,
        1.0f
    };
}

QVector<StlRenderVertex> transformDeviceModelVertices(const QVector<StlModel::Vertex>& vertices, bool sourceXReversed)
{
    QVector<StlRenderVertex> transformed;
    transformed.reserve(vertices.size());
    for (int i = 0; i + 2 < vertices.size(); i += 3) {
        StlRenderVertex a = transformDeviceModelVertex(vertices.at(i), sourceXReversed);
        StlRenderVertex b = transformDeviceModelVertex(vertices.at(i + 1), sourceXReversed);
        StlRenderVertex c = transformDeviceModelVertex(vertices.at(i + 2), sourceXReversed);
        QVector3D normal = QVector3D::crossProduct(
            QVector3D(b.x - a.x, b.y - a.y, b.z - a.z),
            QVector3D(c.x - a.x, c.y - a.y, c.z - a.z));
        if (normal.lengthSquared() > 0.0f) {
            normal.normalize();
        } else {
            normal = QVector3D(0.0f, 0.0f, 1.0f);
        }
        for (StlRenderVertex* vertex : {&a, &b, &c}) {
            vertex->nx = normal.x();
            vertex->ny = normal.y();
            vertex->nz = normal.z();
            transformed.push_back(*vertex);
        }
    }
    return transformed;
}

bool uploadPointCloudBuffer(QOpenGLWidget* widget,
                            QOpenGLShaderProgram* program,
                            const QVector<PointCloudPoint>& points,
                            QOpenGLBuffer& vbo,
                            QOpenGLVertexArrayObject& vao,
                            qsizetype& capacityBytes,
                            int& pointCount)
{
    pointCount = points.size();
    if (!program || !widget || !widget->context()) {
        return false;
    }

    ScopedOpenGLContext current(widget);
    if (QOpenGLContext::currentContext() != widget->context()) {
        return false;
    }

    if (!vao.isCreated()) {
        vao.create();
    }
    vao.bind();
    if (!vbo.isCreated()) {
        vbo.create();
        vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    }
    vbo.bind();
    program->bind();

    const qsizetype byteCount = points.size() * qsizetype(sizeof(PointCloudPoint));
    if (byteCount == 0) {
        vbo.allocate(0);
        capacityBytes = 0;
    } else if (byteCount > capacityBytes) {
        vbo.allocate(points.constData(), static_cast<int>(byteCount));
        capacityBytes = byteCount;
    } else {
        vbo.write(0, points.constData(), static_cast<int>(byteCount));
    }

    program->enableAttributeArray(0);
    program->setAttributeBuffer(0, GL_FLOAT, offsetof(PointCloudPoint, x), 3, sizeof(PointCloudPoint));
    program->enableAttributeArray(1);
    program->setAttributeBuffer(1, GL_FLOAT, offsetof(PointCloudPoint, r), 3, sizeof(PointCloudPoint));
    vbo.release();
    vao.release();
    program->release();
    return true;
}

bool uploadSlamRenderBuffer(QOpenGLWidget* widget,
                            QOpenGLShaderProgram* program,
                            const QVector<SlamRenderVertex>& vertices,
                            QOpenGLBuffer& vbo,
                            QOpenGLVertexArrayObject& vao,
                            qsizetype& capacityBytes,
                            int& vertexCount)
{
    vertexCount = vertices.size();
    if (!program || !widget || !widget->context()) {
        return false;
    }
    if (QOpenGLContext::currentContext() != widget->context()) {
        return false;
    }

    if (!vao.isCreated()) {
        vao.create();
    }
    vao.bind();
    if (!vbo.isCreated()) {
        vbo.create();
        vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    }
    vbo.bind();

    const qsizetype byteCount = vertices.size() * qsizetype(sizeof(SlamRenderVertex));
    if (byteCount == 0) {
        vbo.allocate(0);
        capacityBytes = 0;
    } else if (byteCount > capacityBytes) {
        vbo.allocate(vertices.constData(), static_cast<int>(byteCount));
        capacityBytes = byteCount;
    } else {
        vbo.write(0, vertices.constData(), static_cast<int>(byteCount));
    }

    program->enableAttributeArray(0);
    program->setAttributeBuffer(0, GL_FLOAT, offsetof(SlamRenderVertex, x), 3, sizeof(SlamRenderVertex));
    program->enableAttributeArray(1);
    program->setAttributeBuffer(1, GL_FLOAT, offsetof(SlamRenderVertex, r), 3, sizeof(SlamRenderVertex));
    vbo.release();
    vao.release();
    return true;
}

QVector<SlamRenderVertex> clipSlamRenderVertices(
    const QVector<SlamRenderVertex>& vertices,
    const PointCloudCrossSection::State& state)
{
    QVector<SlamRenderVertex> clipped;
    clipped.reserve(vertices.size());
    for (const SlamRenderVertex& vertex : vertices) {
        if (PointCloudCrossSection::containsPoint(QVector3D(vertex.x, vertex.y, vertex.z), state)) {
            clipped.push_back(vertex);
        }
    }
    return clipped;
}

bool slamWorldFrameBounds(const SlamRenderSnapshot& snapshot,
                          QVector3D& minPoint,
                          QVector3D& maxPoint)
{
    if (snapshot.worldFrameVertices.isEmpty()) {
        return false;
    }

    const SlamRenderVertex& first = snapshot.worldFrameVertices.first();
    minPoint = QVector3D(first.x, first.y, first.z);
    maxPoint = minPoint;
    for (const SlamRenderVertex& vertex : snapshot.worldFrameVertices) {
        minPoint.setX(std::min(minPoint.x(), vertex.x));
        minPoint.setY(std::min(minPoint.y(), vertex.y));
        minPoint.setZ(std::min(minPoint.z(), vertex.z));
        maxPoint.setX(std::max(maxPoint.x(), vertex.x));
        maxPoint.setY(std::max(maxPoint.y(), vertex.y));
        maxPoint.setZ(std::max(maxPoint.z(), vertex.z));
    }
    return true;
}

int clippedSlamWorldFramePointCount(const SlamRenderSnapshot& snapshot,
                                    const PointCloudCrossSection::State& state)
{
    int count = 0;
    for (const SlamRenderVertex& vertex : snapshot.worldFrameVertices) {
        if (PointCloudCrossSection::containsPoint(QVector3D(vertex.x, vertex.y, vertex.z), state)) {
            ++count;
        }
    }
    return count;
}

struct SegmentPointSnapshot {
    quint64 segmentId = 0;
    bool singleBuffer = false;
    QVector<PointCloudPoint> points;
};

struct ClipSegmentResult {
    quint64 segmentId = 0;
    bool singleBuffer = false;
    QVector<PointCloudPoint> clippedPoints;
};

struct ClipJobResult {
    quint64 generation = 0;
    bool fullWindow = false;
    QVector<ClipSegmentResult> segments;
};

struct SelectionSegmentResult {
    quint64 segmentId = 0;
    bool singleBuffer = false;
    QVector<PointCloudPoint> selectedPoints;
};

struct SelectionJobResult {
    quint64 generation = 0;
    QVector<SelectionSegmentResult> segments;
};

struct SelectionPublishResult {
    quint64 generation = 0;
    QVector<PointCloudPoint> points;
    int zeroPointCount = 0;
};

struct ProjectedScreenPoint {
    float x = 0.0f;
    float y = 0.0f;
};

float clipPlaneDistance(const QVector4D& point, int plane)
{
    switch (plane) {
    case 0:
        return point.x() + point.w();
    case 1:
        return point.w() - point.x();
    case 2:
        return point.y() + point.w();
    case 3:
        return point.w() - point.y();
    case 4:
        return point.z() + point.w();
    case 5:
        return point.w() - point.z();
    default:
        return -1.0f;
    }
}

bool clipHomogeneousSegment(QVector4D& a, QVector4D& b)
{
    for (int plane = 0; plane < 6; ++plane) {
        float da = clipPlaneDistance(a, plane);
        float db = clipPlaneDistance(b, plane);
        if (da < 0.0f && db < 0.0f) {
            return false;
        }
        if (da < 0.0f || db < 0.0f) {
            const float t = da / (da - db);
            const QVector4D p = a + (b - a) * t;
            if (da < 0.0f) {
                a = p;
            } else {
                b = p;
            }
        }
    }
    return a.w() > 0.0f && b.w() > 0.0f;
}

QPointF clipToScreenPoint(const QVector4D& clip, int viewportW, int viewportH)
{
    const QVector3D ndc = clip.toVector3DAffine();
    return QPointF((ndc.x() * 0.5f + 0.5f) * float(viewportW),
                   (1.0f - (ndc.y() * 0.5f + 0.5f)) * float(viewportH));
}

bool projectClippedSegmentToScreen(const QVector3D& worldA,
                                   const QVector3D& worldB,
                                   const QMatrix4x4& mvp,
                                   int viewportW,
                                   int viewportH,
                                   QPointF& screenA,
                                   QPointF& screenB)
{
    if (viewportW <= 0 || viewportH <= 0) {
        return false;
    }

    QVector4D clipA = mvp * QVector4D(worldA, 1.0f);
    QVector4D clipB = mvp * QVector4D(worldB, 1.0f);
    if (!clipHomogeneousSegment(clipA, clipB)) {
        return false;
    }

    screenA = clipToScreenPoint(clipA, viewportW, viewportH);
    screenB = clipToScreenPoint(clipB, viewportW, viewportH);
    return true;
}

bool projectVisiblePointToScreen(const QVector4D& hp,
                                 const QMatrix4x4& mvp,
                                 int viewportW,
                                 int viewportH,
                                 ProjectedScreenPoint& projected)
{
    if (viewportW <= 0 || viewportH <= 0) {
        return false;
    }

    const QVector4D clip = mvp * hp;
    if (clip.w() <= 0.0f) {
        return false;
    }

    const QVector3D ndc = clip.toVector3DAffine();
    if (ndc.x() < -1.0f || ndc.x() > 1.0f ||
        ndc.y() < -1.0f || ndc.y() > 1.0f ||
        ndc.z() < -1.0f || ndc.z() > 1.0f) {
        return false;
    }

    projected.x = (ndc.x() * 0.5f + 0.5f) * float(viewportW);
    projected.y = (1.0f - (ndc.y() * 0.5f + 0.5f)) * float(viewportH);
    return true;
}

bool containsSelectionPoint(const PointCloudPoint& point, const PointCloudView::SelectionRegion& region)
{
    if (!region.valid || region.viewportW <= 0 || region.viewportH <= 0) {
        return false;
    }
    const QVector4D hp(point.x, point.y, point.z, 1.0f);
    ProjectedScreenPoint projected;
    if (!projectVisiblePointToScreen(hp, region.mvp, region.viewportW, region.viewportH, projected)) {
        return false;
    }
    return projected.x >= region.rect.left() && projected.x <= region.rect.right() &&
           projected.y >= region.rect.top() && projected.y <= region.rect.bottom();
}

QVector<PointCloudPoint> selectPointsInRegion(const QVector<PointCloudPoint>& points,
                                              const PointCloudView::SelectionRegion& region)
{
    QVector<PointCloudPoint> selected;
    selected.reserve(std::min<int>(int(points.size()), 4096));
    for (const PointCloudPoint& point : points) {
        if (containsSelectionPoint(point, region)) {
            selected.push_back(point);
        }
    }
    return selected;
}

int zeroPointCount(const QVector<PointCloudPoint>& points)
{
    int count = 0;
    for (const PointCloudPoint& point : points) {
        if (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f) {
            ++count;
        }
    }
    return count;
}

} // namespace

// PointCloudView 实现
PointCloudView::PointCloudView(QWidget *parent)
    : QOpenGLWidget(parent)
    , m_program(nullptr)
    , m_distance(kDefaultCameraDistance)
    , m_rotation(0, 0, 0)
    , m_orientation() // identity
    , m_target(0, 0, 0)
    , m_viewerPosition(0, 0, 25)
    , m_activeButton(Qt::NoButton)
    , m_mousePressed(false)
    , m_pointSize(2.0f)
    , m_selecting(false)
    , m_selStart(QPoint())
    , m_selEnd(QPoint())
    , m_selectionModeEnabled(false)
    , m_gridVisible(true)
{
    setFocusPolicy(Qt::StrongFocus);
    setAcceptDrops(true);
    // 默认视角：X 向上、Y 向左、Z 向外（斜45°视角看向Z轴）
    m_orientation = QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), -60.0f)
                * QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), 90.0f);
    syncViewerPositionFromOrbit();

    m_crossSectionClipDebounceTimer = new QTimer(this);
    m_crossSectionClipDebounceTimer->setSingleShot(true);
    m_crossSectionClipDebounceTimer->setInterval(0);
    connect(m_crossSectionClipDebounceTimer, &QTimer::timeout, this, &PointCloudView::startCrossSectionClipJob);

    m_selectionPublishDebounceTimer = new QTimer(this);
    m_selectionPublishDebounceTimer->setSingleShot(true);
    m_selectionPublishDebounceTimer->setInterval(150);
    connect(m_selectionPublishDebounceTimer, &QTimer::timeout, this, &PointCloudView::startSelectionPublishJob);
}

PointCloudView::~PointCloudView()
{
    const bool hasContext = context() != nullptr;
    if (hasContext) {
        makeCurrent();
    }
    if (m_edlRenderer) {
        m_edlRenderer->destroy();
        m_edlRenderer.reset();
    }
    m_stlModelVbo.destroy();
    m_stlModelVao.destroy();
    m_crossSectionTriangleVbo.destroy();
    m_crossSectionTriangleVao.destroy();
    m_crossSectionVbo.destroy();
    m_crossSectionVao.destroy();
    destroySlamRenderOverlay();
    m_backgroundVbo.destroy();
    m_backgroundVao.destroy();
    m_gridVbo.destroy();
    m_gridVao.destroy();
    m_axesVbo.destroy();
    m_axesVao.destroy();
    destroyPointCloudSegments();
    m_selectionVbo.destroy();
    m_selectionVao.destroy();
    m_vbo.destroy();
    m_vao.destroy();
    if (m_program) {
        delete m_program;
        m_program = nullptr;
    }
    if (m_backgroundProgram) {
        delete m_backgroundProgram;
        m_backgroundProgram = nullptr;
    }
    if (hasContext) {
        doneCurrent();
    }
}

void PointCloudView::setSlamRenderSnapshot(const SlamRenderSnapshot& snapshot)
{
    Q_ASSERT(qApp);
    Q_ASSERT(QThread::currentThread() == qApp->thread());
    m_slamRenderSnapshot = snapshot;
    setSlamFollowPose(snapshot.currentPose);
    m_slamRenderUploadPending = true;
    bool hasPrimaryPointCloud = false;
    {
        QMutexLocker locker(&m_pointsMutex);
        hasPrimaryPointCloud = !m_points.isEmpty() || !m_pointCloudSegments.empty();
    }
    if (m_crossSectionState.enabled &&
        m_crossSectionState.initialized &&
        m_crossSectionState.sourcePoints.isEmpty() &&
        !hasPrimaryPointCloud) {
        emit crossSectionChanged(
            clippedSlamWorldFramePointCount(m_slamRenderSnapshot, m_crossSectionState),
            m_slamRenderSnapshot.worldFrameVertices.size());
    }
    update();
}

void PointCloudView::setSlamPoseAxisVertices(const QVector<SlamRenderVertex>& vertices)
{
    Q_ASSERT(qApp);
    Q_ASSERT(QThread::currentThread() == qApp->thread());
    m_slamRenderSnapshot.poseAxisVertices = vertices;
    m_slamPoseAxisUploadPending = true;
    update();
}

void PointCloudView::clearSlamRenderOverlay()
{
    Q_ASSERT(qApp);
    Q_ASSERT(QThread::currentThread() == qApp->thread());
    m_slamRenderSnapshot = SlamRenderSnapshot();
    setSlamFollowPose(SlamRenderPose());
    m_slamRenderUploadPending = true;
    update();
}

// 选点：在屏幕区域内找最近点（优先屏幕距离，其次视空间深度）
bool PointCloudView::pickNearestPoint(const QPoint& pos, QVector3D& outWorld, QPoint& outScreen, int pixelRadius)
{
    QMatrix4x4 mvp = m_projection * m_modelView;
    float dpr = devicePixelRatioF();
    int effectiveRadius = std::max(pixelRadius, int(std::round((m_pointSize / std::max(1.0f, dpr)) * 1.8f)));
    float radiusSq = float(effectiveRadius * effectiveRadius);
    float bestDistSq = std::numeric_limits<float>::max();
    float bestZ = std::numeric_limits<float>::max();
    bool found = false;
    forEachDisplayedPoint([&](const PointCloudPoint& p) {
        QVector4D hp(p.x, p.y, p.z, 1.0f);
        ProjectedScreenPoint projected;
        if (!projectVisiblePointToScreen(hp, mvp, width(), height(), projected)) {
            return true;
        }
        float dx = projected.x - pos.x();
        float dy = projected.y - pos.y();
        float distSq = dx*dx + dy*dy;
        if (distSq <= radiusSq) {
            float vz = (m_modelView * hp).z();
            if (distSq < bestDistSq || (std::abs(distSq - bestDistSq) < 1e-3f && vz < bestZ)) {
                bestDistSq = distSq;
                bestZ = vz;
                outWorld = QVector3D(p.x, p.y, p.z);
                outScreen = QPoint(int(std::round(projected.x)), int(std::round(projected.y)));
                found = true;
            }
        }
        return true;
    });
    return found;
}

void PointCloudView::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (m_projectionMode == ProjectionMode::SlamPoseFollow && m_slamFollowPoseValid) {
        return;
    }
    if (event->button() == Qt::LeftButton) {
        QVector3D pickedWorld;
        QPoint pickedScreen;
        
        // 调用你现有的拾取函数
        if (pickNearestPoint(event->pos(), pickedWorld, pickedScreen)) {
            if (m_projectionMode == ProjectionMode::ObserverPerspective) {
                const QVector3D toPoint = pickedWorld - m_viewerPosition;
                const float pickedDistance = toPoint.length();
                if (pickedDistance > 1e-3f) {
                    m_distance = qMax(1.0f, pickedDistance * 0.5f);
                    m_viewerPosition = pickedWorld - toPoint.normalized() * m_distance;
                    m_target = pickedWorld;
                    update();
                }
                return;
            }
            // 核心改变：将双击的点设为新的旋转和缩放中心
            m_target = pickedWorld;
            
            // 细节优化：双击通常是为了看局部，稍微拉近距离
            m_distance = qMax(1.0f, m_distance * 0.5f); 
            
            update();
        }
    }
}

void PointCloudView::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(m_backgroundTopColor.redF(), m_backgroundTopColor.greenF(), m_backgroundTopColor.blueF(), 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_MULTISAMPLE);
    glPointSize(2.0f);
    
    setupShaders();
    m_edlRenderer = std::make_unique<PointCloudEdlRenderer>();
    m_edlRenderer->initialize();
    setupBackgroundBuffers();
    setupBuffers();
    setupAxesBuffers();
    setupGridBuffers();
    setupCrossSectionBuffers();
    setupStlModelBuffers();
}

void PointCloudView::setupShaders()
{
    const char *vertexShaderSource = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aColor;
        layout(location = 2) in vec3 aNormal;

        // 正常渲染用的矩阵
        uniform mat4 modelView;
        uniform mat4 projection;
        uniform mat3 normalMatrix;
        uniform float uPointSize;
        uniform int uModelLighting;

        // 框选专用的 Uniform (来自 m_selectionLocked == true 时)
        uniform int uPersistEnabled;
        uniform mat4 uSelMVP;       // C++端传来的 m_selProjection * m_selModelView
        uniform vec4 uPersistRect;  // x0, y0, x1, y1
        uniform vec2 uViewport;     // 宽度, 高度

        // 传给片元着色器的变量
        out vec3 vColor;
        out vec3 vNormal;
        out vec3 vViewPosition;
        flat out int vIsSelected; // 【关键】flat 关键字表示这个整数不需要插值，原样传给片元

        void main() {
            // 1. 常规的顶点位置计算 (为了屏幕显示)
            vec4 viewPosition = modelView * vec4(aPos, 1.0);
            gl_Position = projection * viewPosition;
            gl_PointSize = uPointSize;
            vColor = aColor;
            vNormal = normalize(normalMatrix * aNormal);
            vViewPosition = viewPosition.xyz;
            
            // 2. 默认未选中
            vIsSelected = 0;

            // 3. 框选判断逻辑 (只在这里算一次！)
            if (uPersistEnabled == 1) {
                // 使用框选瞬间的视角矩阵计算该点的位置
                vec4 selClip = uSelMVP * vec4(aPos, 1.0);
                
                // 排除在摄像机背后的点
                if (selClip.w > 0.0) {
                    // 转换为归一化设备坐标 (NDC)
                    vec3 ndc = selClip.xyz / selClip.w;
                    
                    // 映射到屏幕坐标 (注意：这里的公式必须和 point_widget.cpp 中 pickNearestPoint 的逻辑完全一致)
                    float screenX = (ndc.x * 0.5 + 0.5) * uViewport.x;
                    float screenY = (1.0 - (ndc.y * 0.5 + 0.5)) * uViewport.y; // Y 轴翻转
                    
                    // 判断是否在鼠标画出的矩形范围内
                    if (screenX >= uPersistRect.x && screenX <= uPersistRect.z &&
                        screenY >= uPersistRect.y && screenY <= uPersistRect.w) {
                        vIsSelected = 1; // 标记为选中
                    }
                }
            }
        }
    )";

    const char *fragmentShaderSource = R"(
        #version 330 core
        in vec3 vColor;
        in vec3 vNormal;
        in vec3 vViewPosition;
        flat in int vIsSelected; // 接收顶点着色器传来的判断结果

        uniform int uModelLighting;
        uniform int uPointPrimitive;
        uniform int uEdlEligible;
        uniform int uEdlPass;
        uniform int uRoundPointSplat;

        layout(location = 0) out vec4 SceneColor;
        layout(location = 1) out float LinearDepth;

        void main() {
            if (uPointPrimitive == 1 && uRoundPointSplat == 1) {
                vec2 coord = gl_PointCoord * 2.0 - 1.0;
                if (dot(coord, coord) > 1.0) {
                    discard;
                }
            }

            vec3 color = vColor;
            if (vIsSelected == 1) {
                color = vec3(1.0, 0.0, 0.0);
            } else if (uModelLighting == 1) {
                vec3 normal = normalize(vNormal);
                vec3 viewDir = normalize(-vViewPosition);
                vec3 keyLight = normalize(vec3(-0.45, 0.35, 0.82));
                vec3 fillLight = normalize(vec3(0.68, -0.32, 0.52));
                float diffuse = 0.46 * max(dot(normal, keyLight), 0.0)
                              + 0.22 * max(dot(normal, fillLight), 0.0);
                vec3 halfDir = normalize(keyLight + viewDir);
                float specular = 0.10 * pow(max(dot(normal, halfDir), 0.0), 32.0);
                color = clamp(vColor * (0.36 + diffuse) + vec3(specular), 0.0, 1.0);
            }

            float category = uEdlPass == 1
                ? (uEdlEligible == 1 ? 1.0 : 0.5)
                : 1.0;
            SceneColor = vec4(color, category);
            LinearDepth = max(-vViewPosition.z, 0.0001);
        }
    )";

    m_program = new QOpenGLShaderProgram();
    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    m_program->link();
}

void PointCloudView::setupBackgroundBuffers()
{
    const char* vertexShaderSource = R"(
        #version 330 core
        layout(location = 0) in vec2 aPos;
        out float vPositionY;

        void main() {
            vPositionY = (aPos.y + 1.0) * 0.5;
            gl_Position = vec4(aPos, 0.0, 1.0);
        }
    )";

    const char* fragmentShaderSource = R"(
        #version 330 core
        in float vPositionY;
        uniform vec3 uTopColor;
        uniform vec3 uBottomColor;
        out vec4 FragColor;

        void main() {
            FragColor = vec4(mix(uBottomColor, uTopColor, vPositionY), 1.0);
        }
    )";

    m_backgroundProgram = new QOpenGLShaderProgram();
    m_backgroundProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    m_backgroundProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    m_backgroundProgram->link();

    const float vertices[] = {
        -1.0f, -1.0f,
         1.0f, -1.0f,
        -1.0f,  1.0f,
         1.0f,  1.0f
    };

    m_backgroundVao.create();
    m_backgroundVao.bind();
    m_backgroundVbo.create();
    m_backgroundVbo.bind();
    m_backgroundVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_backgroundVbo.allocate(vertices, int(sizeof(vertices)));
    m_backgroundProgram->enableAttributeArray(0);
    m_backgroundProgram->setAttributeBuffer(0, GL_FLOAT, 0, 2, 2 * int(sizeof(float)));
    m_backgroundVao.release();
}

void PointCloudView::setupBuffers()
{
    m_vao.create();
    m_vao.bind();
    
    m_vbo.create();
    m_vbo.bind();
    m_vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_pointCloudBufferCapacityBytes = qsizetype(1000) * qsizetype(sizeof(PointCloudPoint));
    m_vbo.allocate(static_cast<int>(m_pointCloudBufferCapacityBytes)); // 初始分配空间
    
    // 设置顶点属性
    m_program->enableAttributeArray(0);
    m_program->setAttributeBuffer(0, GL_FLOAT, offsetof(PointCloudPoint, x), 3, sizeof(PointCloudPoint));
    
    m_program->enableAttributeArray(1);
    m_program->setAttributeBuffer(1, GL_FLOAT, offsetof(PointCloudPoint, r), 3, sizeof(PointCloudPoint));
    
    m_vao.release();
}

//坐标轴属性
void PointCloudView::setupAxesBuffers()
{
    struct AxisVertex { float x, y, z, r, g, b; };
    std::vector<AxisVertex> vertices;

    const float axisLen = 1.0f;
    const QColor colX(255, 0, 0), colY(0, 255, 0), colZ(0, 0, 255);

    // 辅助 lambda：添加线段
    auto addLine = [&](const QVector3D& p1, const QVector3D& p2, const QColor& c) {
        vertices.push_back({p1.x(), p1.y(), p1.z(), c.redF(), c.greenF(), c.blueF()});
        vertices.push_back({p2.x(), p2.y(), p2.z(), c.redF(), c.greenF(), c.blueF()});
    };

    // 绘制轴线（从原点延伸到锥体底部）
    const float coneHeight = 0.2f;
    const float coneBasePos = axisLen - coneHeight;
    addLine(QVector3D(0,0,0), QVector3D(coneBasePos, 0, 0), colX);
    addLine(QVector3D(0,0,0), QVector3D(0, coneBasePos, 0), colY);
    addLine(QVector3D(0,0,0), QVector3D(0, 0, coneBasePos), colZ);

    // 锥体参数
    const float coneRadius = 0.08f;
    const int   coneSegments = 16;   // 十六边形底面
    const float angleStep = 2.0f * M_PI / coneSegments;

    // 生成锥体线框（X轴）
    {
        QVector3D tip(axisLen, 0, 0);
        QVector3D baseCenter(coneBasePos, 0, 0);
        QVector3D u(0, 1, 0), v(0, 0, 1); // 底面的两个正交轴
        std::vector<QVector3D> basePts;
        for (int i = 0; i < coneSegments; ++i) {
            float angle = i * angleStep;
            QVector3D pt = baseCenter + u * (coneRadius * cos(angle)) + v * (coneRadius * sin(angle));
            basePts.push_back(pt);
        }
        // 底面圆环线段
        for (int i = 0; i < coneSegments; ++i) {
            int j = (i + 1) % coneSegments;
            addLine(basePts[i], basePts[j], colX);
        }
        // 侧棱（锥尖到各顶点）
        for (const auto& pt : basePts) {
            addLine(tip, pt, colX);
        }
    }

    // Y轴锥体
    {
        QVector3D tip(0, axisLen, 0);
        QVector3D baseCenter(0, coneBasePos, 0);
        QVector3D u(1, 0, 0), v(0, 0, 1);
        std::vector<QVector3D> basePts;
        for (int i = 0; i < coneSegments; ++i) {
            float angle = i * angleStep;
            QVector3D pt = baseCenter + u * (coneRadius * cos(angle)) + v * (coneRadius * sin(angle));
            basePts.push_back(pt);
        }
        for (int i = 0; i < coneSegments; ++i) {
            int j = (i + 1) % coneSegments;
            addLine(basePts[i], basePts[j], colY);
        }
        for (const auto& pt : basePts) {
            addLine(tip, pt, colY);
        }
    }

    // Z轴锥体
    {
        QVector3D tip(0, 0, axisLen);
        QVector3D baseCenter(0, 0, coneBasePos);
        QVector3D u(1, 0, 0), v(0, 1, 0);
        std::vector<QVector3D> basePts;
        for (int i = 0; i < coneSegments; ++i) {
            float angle = i * angleStep;
            QVector3D pt = baseCenter + u * (coneRadius * cos(angle)) + v * (coneRadius * sin(angle));
            basePts.push_back(pt);
        }
        for (int i = 0; i < coneSegments; ++i) {
            int j = (i + 1) % coneSegments;
            addLine(basePts[i], basePts[j], colZ);
        }
        for (const auto& pt : basePts) {
            addLine(tip, pt, colZ);
        }
    }

    // 创建 VBO/VAO
    m_axesVao.create();
    m_axesVao.bind();

    m_axesVbo.create();
    m_axesVbo.bind();
    m_axesVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_axesVbo.allocate(vertices.data(), int(vertices.size() * sizeof(AxisVertex)));

    m_program->enableAttributeArray(0);
    m_program->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(AxisVertex));
    m_program->enableAttributeArray(1);
    m_program->setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 3, sizeof(AxisVertex));

    m_axesVao.release();
    m_axesVbo.release();

    m_axesVertexCount = int(vertices.size());
}

void PointCloudView::setupCrossSectionBuffers()
{
    m_crossSectionVao.create();
    m_crossSectionVao.bind();

    m_crossSectionVbo.create();
    m_crossSectionVbo.bind();
    m_crossSectionVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_crossSectionVbo.allocate(1);

    m_program->enableAttributeArray(0);
    m_program->setAttributeBuffer(0, GL_FLOAT, offsetof(PointCloudCrossSection::ColoredVertex, x), 3, sizeof(PointCloudCrossSection::ColoredVertex));
    m_program->enableAttributeArray(1);
    m_program->setAttributeBuffer(1, GL_FLOAT, offsetof(PointCloudCrossSection::ColoredVertex, r), 3, sizeof(PointCloudCrossSection::ColoredVertex));

    m_crossSectionVao.release();
    m_crossSectionVbo.release();

    m_crossSectionTriangleVao.create();
    m_crossSectionTriangleVao.bind();

    m_crossSectionTriangleVbo.create();
    m_crossSectionTriangleVbo.bind();
    m_crossSectionTriangleVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_crossSectionTriangleVbo.allocate(1);

    m_program->enableAttributeArray(0);
    m_program->setAttributeBuffer(0, GL_FLOAT, offsetof(PointCloudCrossSection::ColoredVertex, x), 3, sizeof(PointCloudCrossSection::ColoredVertex));
    m_program->enableAttributeArray(1);
    m_program->setAttributeBuffer(1, GL_FLOAT, offsetof(PointCloudCrossSection::ColoredVertex, r), 3, sizeof(PointCloudCrossSection::ColoredVertex));

    m_crossSectionTriangleVao.release();
    m_crossSectionTriangleVbo.release();
}

void PointCloudView::uploadSlamRenderOverlayIfNeeded()
{
    if ((!m_slamRenderUploadPending && !m_slamPoseAxisUploadPending) ||
        !m_program || QOpenGLContext::currentContext() != context()) {
        return;
    }

    if (!m_slamRenderUploadPending) {
        uploadSlamRenderBuffer(this,
                               m_program,
                               m_slamRenderSnapshot.poseAxisVertices,
                               m_slamPoseAxisVbo,
                               m_slamPoseAxisVao,
                               m_slamPoseAxisBufferCapacityBytes,
                               m_slamPoseAxisVertexCount);
        m_slamPoseAxisUploadPending = false;
        return;
    }

    const bool crossSectionEnabled =
        m_crossSectionState.enabled && m_crossSectionState.initialized;
    QVector<SlamRenderVertex> clippedWorldFrameVertices;
    QVector<SlamRenderVertex> clippedDynamicObjectVertices;
    const QVector<SlamRenderVertex>* worldFrameVertices =
        &m_slamRenderSnapshot.worldFrameVertices;
    const QVector<SlamRenderVertex>* dynamicObjectVertices =
        &m_slamRenderSnapshot.dynamicObjectVertices;
    if (crossSectionEnabled) {
        clippedWorldFrameVertices = clipSlamRenderVertices(
            m_slamRenderSnapshot.worldFrameVertices,
            m_crossSectionState);
        clippedDynamicObjectVertices = clipSlamRenderVertices(
            m_slamRenderSnapshot.dynamicObjectVertices,
            m_crossSectionState);
        worldFrameVertices = &clippedWorldFrameVertices;
        dynamicObjectVertices = &clippedDynamicObjectVertices;
    }

    uploadSlamRenderBuffer(this,
                           m_program,
                           m_slamRenderSnapshot.trajectoryVertices,
                           m_slamTrajectoryVbo,
                           m_slamTrajectoryVao,
                           m_slamTrajectoryBufferCapacityBytes,
                           m_slamTrajectoryVertexCount);
    uploadSlamRenderBuffer(this,
                           m_program,
                           m_slamRenderSnapshot.poseAxisVertices,
                           m_slamPoseAxisVbo,
                           m_slamPoseAxisVao,
                           m_slamPoseAxisBufferCapacityBytes,
                           m_slamPoseAxisVertexCount);
    uploadSlamRenderBuffer(this,
                           m_program,
                           *worldFrameVertices,
                           m_slamWorldFrameVbo,
                           m_slamWorldFrameVao,
                           m_slamWorldFrameBufferCapacityBytes,
                           m_slamWorldFrameVertexCount);
    uploadSlamRenderBuffer(this,
                           m_program,
                           m_slamRenderSnapshot.bodyFrameVertices,
                           m_slamBodyFrameVbo,
                           m_slamBodyFrameVao,
                           m_slamBodyFrameBufferCapacityBytes,
                           m_slamBodyFrameVertexCount);
    uploadSlamRenderBuffer(this,
                           m_program,
                           *dynamicObjectVertices,
                           m_slamDynamicObjectVbo,
                           m_slamDynamicObjectVao,
                           m_slamDynamicObjectBufferCapacityBytes,
                           m_slamDynamicObjectVertexCount);
    m_slamRenderUploadPending = false;
    m_slamPoseAxisUploadPending = false;
}

void PointCloudView::destroySlamRenderOverlay()
{
    m_slamWorldFrameVbo.destroy();
    m_slamWorldFrameVao.destroy();
    m_slamBodyFrameVbo.destroy();
    m_slamBodyFrameVao.destroy();
    m_slamDynamicObjectVbo.destroy();
    m_slamDynamicObjectVao.destroy();
    m_slamPoseAxisVbo.destroy();
    m_slamPoseAxisVao.destroy();
    m_slamTrajectoryVbo.destroy();
    m_slamTrajectoryVao.destroy();
    m_slamTrajectoryBufferCapacityBytes = 0;
    m_slamPoseAxisBufferCapacityBytes = 0;
    m_slamWorldFrameBufferCapacityBytes = 0;
    m_slamBodyFrameBufferCapacityBytes = 0;
    m_slamDynamicObjectBufferCapacityBytes = 0;
    m_slamTrajectoryVertexCount = 0;
    m_slamPoseAxisVertexCount = 0;
    m_slamWorldFrameVertexCount = 0;
    m_slamBodyFrameVertexCount = 0;
    m_slamDynamicObjectVertexCount = 0;
}

void PointCloudView::setupStlModelBuffers()
{
    m_stlModelVao.create();
    m_stlModelVao.bind();

    m_stlModelVbo.create();
    m_stlModelVbo.bind();
    m_stlModelVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_stlModelVbo.allocate(1);

    m_program->enableAttributeArray(0);
    m_program->setAttributeBuffer(0, GL_FLOAT, offsetof(StlRenderVertex, x), 3, sizeof(StlRenderVertex));
    m_program->enableAttributeArray(1);
    m_program->setAttributeBuffer(1, GL_FLOAT, offsetof(StlRenderVertex, r), 3, sizeof(StlRenderVertex));
    m_program->enableAttributeArray(2);
    m_program->setAttributeBuffer(2, GL_FLOAT, offsetof(StlRenderVertex, nx), 3, sizeof(StlRenderVertex));

    m_stlModelVao.release();
    m_stlModelVbo.release();
}

void PointCloudView::setGridVisible(bool visible)
{
    if (m_gridVisible != visible) {
        m_gridVisible = visible;
        update(); // 触发重绘
    }
}

void PointCloudView::setGridConfig(const GridConfig& config)
{
    GridConfig normalized = config;
    normalized.range = std::max(1.0f, normalized.range);
    normalized.step = std::max(0.1f, normalized.step);
    if (!normalized.color.isValid()) {
        normalized.color = QColor(77, 77, 77);
    }
    if (normalized.type != GridConfig::Square &&
        normalized.type != GridConfig::ConcentricCircles &&
        normalized.type != GridConfig::SquareAndConcentricCircles) {
        normalized.type = GridConfig::Square;
    }

    m_gridConfig = normalized;

    if (context()) {
        makeCurrent();
        setupGridBuffers();
        doneCurrent();
    }
    update();
}

void PointCloudView::setupGridBuffers()
{
    struct GridVertex {
        float x, y, z;
        float r, g, b;
    };

    std::vector<GridVertex> gridVertices;

    const float range = std::max(1.0f, m_gridConfig.range);
    const float step = std::max(0.1f, m_gridConfig.step);
    const QVector3D gridColor(float(m_gridConfig.color.redF()),
                              float(m_gridConfig.color.greenF()),
                              float(m_gridConfig.color.blueF()));
    const QVector3D xAxisColor(1.0f, 0.12f, 0.10f);
    const QVector3D yAxisColor(0.10f, 0.80f, 0.20f);
    const int ringSegments = 96;
    const float pi = 3.14159265358979323846f;

    auto addLine = [&](const QVector3D& p1, const QVector3D& p2, const QVector3D& color) {
        gridVertices.push_back({p1.x(), p1.y(), p1.z(), color.x(), color.y(), color.z()});
        gridVertices.push_back({p2.x(), p2.y(), p2.z(), color.x(), color.y(), color.z()});
    };

    const bool drawSquareGrid =
        m_gridConfig.type == GridConfig::Square ||
        m_gridConfig.type == GridConfig::SquareAndConcentricCircles;
    const bool drawConcentricCircles =
        m_gridConfig.type == GridConfig::ConcentricCircles ||
        m_gridConfig.type == GridConfig::SquareAndConcentricCircles;

    if (drawSquareGrid) {
        for (float i = -range; i <= range + 1e-4f; i += step) {
            if (std::abs(i) <= 1e-4f) {
                continue;
            }
            addLine(QVector3D(i, -range, 0.0f), QVector3D(i, range, 0.0f), gridColor);
            addLine(QVector3D(-range, i, 0.0f), QVector3D(range, i, 0.0f), gridColor);
        }
    }

    if (drawConcentricCircles) {
        for (float radius = step; radius <= range + 1e-4f; radius += step) {
            for (int i = 0; i < ringSegments; ++i) {
                const float a0 = (2.0f * pi * float(i)) / float(ringSegments);
                const float a1 = (2.0f * pi * float(i + 1)) / float(ringSegments);
                addLine(QVector3D(radius * std::cos(a0), radius * std::sin(a0), 0.0f),
                        QVector3D(radius * std::cos(a1), radius * std::sin(a1), 0.0f),
                        gridColor);
            }
        }
    }

    if (drawSquareGrid || drawConcentricCircles) {
        addLine(QVector3D(-range, 0.0f, 0.0f), QVector3D(range, 0.0f, 0.0f), xAxisColor);
        addLine(QVector3D(0.0f, -range, 0.0f), QVector3D(0.0f, range, 0.0f), yAxisColor);
    }

    m_gridVertexCount = int(gridVertices.size());

    if (m_gridVbo.isCreated()) {
        m_gridVbo.destroy();
    }
    if (m_gridVao.isCreated()) {
        m_gridVao.destroy();
    }

    m_gridVao.create();
    m_gridVao.bind();

    m_gridVbo.create();
    m_gridVbo.bind();
    m_gridVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_gridVbo.allocate(gridVertices.data(), int(gridVertices.size() * sizeof(GridVertex)));

    // 复用现有 shader 的 layout (loc 0=pos, loc 1=color)
    m_program->enableAttributeArray(0);
    m_program->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(GridVertex));
    
    m_program->enableAttributeArray(1);
    m_program->setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 3, sizeof(GridVertex));

    m_gridVao.release();
    m_gridVbo.release();
}

static QMatrix4x4 quaternionToMatrix(const QQuaternion& q)
{
    QMatrix4x4 m;
    m.setToIdentity();
    const float x = q.x(), y = q.y(), z = q.z(), w = q.scalar();
    const float xx = x * x, yy = y * y, zz = z * z;
    const float xy = x * y, xz = x * z, yz = y * z;
    const float wx = w * x, wy = w * y, wz = w * z;

    m(0,0) = 1.0f - 2.0f * (yy + zz);
    m(0,1) = 2.0f * (xy - wz);
    m(0,2) = 2.0f * (xz + wy);

    m(1,0) = 2.0f * (xy + wz);
    m(1,1) = 1.0f - 2.0f * (xx + zz);
    m(1,2) = 2.0f * (yz - wx);

    m(2,0) = 2.0f * (xz - wy);
    m(2,1) = 2.0f * (yz + wx);
    m(2,2) = 1.0f - 2.0f * (xx + yy);
    return m;
}

QVector3D PointCloudView::mapToArcball(const QPoint& p) const
{
    float x = (2.0f * p.x() - width()) / qMax(1, width());
    float y = (height() - 2.0f * p.y()) / qMax(1, height());
    float z2 = 1.0f - x * x - y * y;
    float z = z2 > 0.0f ? std::sqrt(z2) : 0.0f;
    return QVector3D(x, y, z).normalized();
}

QVector3D PointCloudView::cameraForward() const
{
    return m_orientation.conjugated().rotatedVector(QVector3D(0.0f, 0.0f, -1.0f));
}

QVector3D PointCloudView::cameraRight() const
{
    return m_orientation.conjugated().rotatedVector(QVector3D(1.0f, 0.0f, 0.0f));
}

QVector3D PointCloudView::cameraUp() const
{
    return m_orientation.conjugated().rotatedVector(QVector3D(0.0f, 1.0f, 0.0f));
}

QVector3D PointCloudView::orbitCameraPosition() const
{
    return m_target + m_orientation.conjugated().rotatedVector(QVector3D(0.0f, 0.0f, m_distance));
}

void PointCloudView::syncViewerPositionFromOrbit()
{
    m_viewerPosition = orbitCameraPosition();
}

void PointCloudView::syncOrbitTargetFromViewer()
{
    m_target = m_viewerPosition + cameraForward() * m_distance;
}

void PointCloudView::paintGL()
{
    if (!m_program) {
        return;
    }

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glClearDepth(1.0);

    bool edlActive = false;
    if (m_edlConfig.enabled && m_edlRenderer) {
        edlActive = m_edlRenderer->beginScene(edlPhysicalSize());
        if (!edlActive && !m_edlFallbackReported) {
            qWarning().noquote() << "EDL disabled for this view:" << m_edlRenderer->errorMessage();
            m_edlFallbackReported = true;
        } else if (edlActive) {
            m_edlFallbackReported = false;
        }
    }

    if (!edlActive) {
        glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());
        const QSize physicalSize = widgetPhysicalSize();
        glViewport(0, 0, physicalSize.width(), physicalSize.height());
        if (m_backgroundTopColor == m_backgroundBottomColor) {
            glClearColor(m_backgroundTopColor.redF(), m_backgroundTopColor.greenF(), m_backgroundTopColor.blueF(), 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        } else {
            glDisable(GL_DEPTH_TEST);
            m_backgroundProgram->bind();
            m_backgroundProgram->setUniformValue("uTopColor", QVector3D(m_backgroundTopColor.redF(),
                                                                        m_backgroundTopColor.greenF(),
                                                                        m_backgroundTopColor.blueF()));
            m_backgroundProgram->setUniformValue("uBottomColor", QVector3D(m_backgroundBottomColor.redF(),
                                                                           m_backgroundBottomColor.greenF(),
                                                                           m_backgroundBottomColor.blueF()));
            m_backgroundVao.bind();
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            m_backgroundVao.release();
            m_backgroundProgram->release();
            glClear(GL_DEPTH_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);
        }
    }
    
    m_program->bind();
    syncPendingPointCloudBuffers();
    m_program->bind();
    
    // 1. 设置基础变换矩阵 (目标中心轨道相机模型)
    m_modelView.setToIdentity();

    // 动作 3：最后，将整个场景沿着 Z 轴推远，为相机腾出视野空间 (Distance)
    m_modelView.translate(0.0f, 0.0f, -m_distance);

    // 动作 2：然后，围绕当前的原点进行旋转 (Rotation)
    QMatrix4x4 rot = quaternionToMatrix(m_orientation);
    m_modelView = m_modelView * rot;

    // 动作 1：首先，把我们要观察的“目标点”移动到世界坐标系的原点 (Translation)
    m_modelView.translate(-m_target);
    if (m_projectionMode == ProjectionMode::ObserverPerspective) {
        m_modelView.setToIdentity();
        m_modelView = m_modelView * rot;
        m_modelView.translate(-m_viewerPosition);
    } else if (m_projectionMode == ProjectionMode::SlamPoseFollow && m_slamFollowPoseValid) {
        const QVector3D forward = m_slamFollowOrientation.rotatedVector(QVector3D(1.0f, 0.0f, 0.0f));
        const QVector3D up = m_slamFollowOrientation.rotatedVector(QVector3D(0.0f, 0.0f, 1.0f));
        const QVector3D cameraPosition = m_slamFollowPosition
            - forward * kSlamFollowDistanceM
            + up * kSlamFollowHeightM;
        const QVector3D lookTarget = m_slamFollowPosition
            + forward * kSlamFollowLookAheadM
            + up * kSlamFollowTargetHeightM;
        const QVector3D viewDirection = (lookTarget - cameraPosition).normalized();
        m_modelView.setToIdentity();
        m_modelView.lookAt(cameraPosition, lookTarget, up);
        rot.setToIdentity();
        rot.lookAt(QVector3D(), viewDirection, up);
    }

    // 设置基础投影矩阵    
    m_projection.setToIdentity();
    const float nearPlane = std::clamp(m_distance * kNearPlaneDistanceRatio,
                                       kMinimumNearPlane,
                                       kDefaultNearPlane);
    float farPlane = qMax(1000.0f, m_distance * 10.0f); // 永远比当前视距大一个数量级
    const float fovY = 45.0f;
    const float aspect = float(qMax(1, width())) / float(qMax(1, height()));
    if (m_projectionMode != ProjectionMode::Orthographic) {
        m_projection.perspective(fovY, aspect, nearPlane, farPlane);
    } else {
        const float halfHeight = m_distance * std::tan((fovY * float(M_PI) / 180.0f) * 0.5f);
        const float halfWidth = halfHeight * aspect;
        m_projection.ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, nearPlane, farPlane);
    }
    QMatrix3x3 normalMatrix;
    normalMatrix.setToIdentity();
    m_program->setUniformValue("modelView", m_modelView);
    m_program->setUniformValue("projection", m_projection);
    m_program->setUniformValue("normalMatrix", normalMatrix);
    m_program->setUniformValue("uPointSize", m_pointSize);
    m_program->setUniformValue("uModelLighting", 0);
    m_program->setUniformValue("uEdlPass", edlActive ? 1 : 0);
    auto setPrimitiveState = [this, edlActive](bool pointPrimitive, bool edlEligible) {
        m_program->setUniformValue("uPointPrimitive", pointPrimitive ? 1 : 0);
        m_program->setUniformValue("uEdlEligible", edlEligible ? 1 : 0);
        m_program->setUniformValue(
            "uRoundPointSplat",
            pointPrimitive && edlActive && m_edlConfig.roundPointSplat ? 1 : 0);
    };

    // ==========================================
    // 2. 绘制基础图元：网格 (注：坐标轴被移到了后面作为 Overlay 绘制)
    // ==========================================
    m_program->setUniformValue("uSelectionEnabled", 0);
    m_program->setUniformValue("uPersistEnabled", 0);
    setPrimitiveState(false, false);

    // 绘制网格
    if (m_gridVisible) {
        if (edlActive) {
            const GLenum colorAttachment = GL_COLOR_ATTACHMENT0;
            glDrawBuffers(1, &colorAttachment);
            glDepthMask(GL_FALSE);
        } else {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable(GL_LINE_SMOOTH);
            glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
        }
        glLineWidth(1.0f);
        if (m_gridVertexCount > 0) {
            m_gridVao.bind();
            glDrawArrays(GL_LINES, 0, m_gridVertexCount);
            m_gridVao.release();
        }
        if (edlActive) {
            glDepthMask(GL_TRUE);
            const GLenum edlAttachments[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
            glDrawBuffers(2, edlAttachments);
        } else {
            glDisable(GL_LINE_SMOOTH);
            glDisable(GL_BLEND);
        }
    }
    glLineWidth(1.0f);

    // ==========================================
    // 3. 准备绘制点云：恢复并设置点云的选择状态逻辑
    // ==========================================
    m_program->setUniformValue("uPersistEnabled", 0);
    m_program->setUniformValue("uPersistRect", QVector4D(0,0,0,0));
    m_program->setUniformValue("uSelModelView", QMatrix4x4());
    m_program->setUniformValue("uSelProjection", QMatrix4x4());
    m_program->setUniformValue("uViewport", QVector2D(0,0));
    m_program->setUniformValue("uDepthRange", QVector2D(0,0));

    // 拖拽时的屏幕框高亮
    if (m_selectionModeEnabled && m_selecting) {
        QRect sel = m_selectionRect();
        if (!sel.isEmpty()) {
            float dpr = devicePixelRatioF();
            float x0 = float(qMin(sel.left(), sel.right())) * dpr;
            float x1 = float(qMax(sel.left(), sel.right())) * dpr;
            float y_top = float(qMin(sel.top(), sel.bottom())) * dpr;
            float y_bottom = float(qMax(sel.top(), sel.bottom())) * dpr;
            float y0 = float(height()) * dpr - y_bottom;
            float y1 = float(height()) * dpr - y_top;
            m_program->setUniformValue("uSelectionEnabled", 1);
            m_program->setUniformValue("uSelRect", QVector4D(x0, y0, x1, y1));
        }
    } else {
        m_program->setUniformValue("uSelectionEnabled", 0);
    }
    
    // 4. 绘制点云
    setPrimitiveState(true, true);
    if (!m_points.isEmpty()) {
        m_vao.bind();
        glDrawArrays(GL_POINTS, 0, m_points.size());
        m_vao.release();
    }
    const bool drawClippedSegments = m_crossSectionState.enabled && m_crossSectionState.initialized;
    for (const std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
        if (!segment) {
            continue;
        }
        if (drawClippedSegments) {
            if (segment->clippedPointCount <= 0 || !segment->clippedVao.isCreated()) {
                continue;
            }
            segment->clippedVao.bind();
            glDrawArrays(GL_POINTS, 0, segment->clippedPointCount);
            segment->clippedVao.release();
        } else {
            if (segment->pointCount <= 0 || !segment->vao.isCreated()) {
                continue;
            }
            segment->vao.bind();
            glDrawArrays(GL_POINTS, 0, segment->pointCount);
            segment->vao.release();
        }
    }

    if (m_selectionLocked) {
        m_program->setUniformValue("uSelectionEnabled", 0);
        m_program->setUniformValue("uPersistEnabled", 0);
        m_program->setUniformValue("uPointSize", m_pointSize + 1.0f);
        glDepthFunc(GL_LEQUAL);
        if (m_selectionPointCount > 0 && m_selectionVao.isCreated()) {
            m_selectionVao.bind();
            glDrawArrays(GL_POINTS, 0, m_selectionPointCount);
            m_selectionVao.release();
        }
        for (const std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
            if (!segment || segment->selectedPointCount <= 0 || !segment->selectedVao.isCreated()) {
                continue;
            }
            segment->selectedVao.bind();
            glDrawArrays(GL_POINTS, 0, segment->selectedPointCount);
            segment->selectedVao.release();
        }
        glDepthFunc(GL_LESS);
        m_program->setUniformValue("uPointSize", m_pointSize);
    }

    uploadSlamRenderOverlayIfNeeded();
    m_program->setUniformValue("uSelectionEnabled", 0);
    m_program->setUniformValue("uPersistEnabled", 0);
    m_program->setUniformValue("uModelLighting", 0);
    setPrimitiveState(true, true);
    if (m_slamWorldFrameVertexCount > 0 && m_slamWorldFrameVao.isCreated()) {
        glDepthFunc(GL_LEQUAL);
        m_program->setUniformValue("uPointSize", m_slamRenderSnapshot.worldFramePointSizePx);
        m_slamWorldFrameVao.bind();
        glDrawArrays(GL_POINTS, 0, m_slamWorldFrameVertexCount);
        m_slamWorldFrameVao.release();
        m_program->setUniformValue("uPointSize", m_pointSize);
        glDepthFunc(GL_LESS);
    }
    if (m_slamBodyFrameVertexCount > 0 && m_slamBodyFrameVao.isCreated()) {
        m_program->setUniformValue("uPointSize", m_slamRenderSnapshot.bodyFramePointSizePx);
        m_slamBodyFrameVao.bind();
        glDrawArrays(GL_POINTS, 0, m_slamBodyFrameVertexCount);
        m_slamBodyFrameVao.release();
        m_program->setUniformValue("uPointSize", m_pointSize);
    }
    if (m_slamDynamicObjectVertexCount > 0 && m_slamDynamicObjectVao.isCreated()) {
        setPrimitiveState(true, false);
        m_program->setUniformValue("uPointSize", m_slamRenderSnapshot.dynamicObjectPointSizePx);
        m_slamDynamicObjectVao.bind();
        glDrawArrays(GL_POINTS, 0, m_slamDynamicObjectVertexCount);
        m_slamDynamicObjectVao.release();
        m_program->setUniformValue("uPointSize", m_pointSize);
    }

    if (m_stlModelVisible && !m_stlModelVertices.isEmpty()) {
        setPrimitiveState(false, false);
        m_program->setUniformValue("uPersistEnabled", 0);
        m_program->setUniformValue("uSelectionEnabled", 0);
        m_program->setUniformValue("normalMatrix", m_modelView.normalMatrix());
        m_program->setUniformValue("uModelLighting", 1);
        glEnable(GL_DEPTH_TEST);
        m_stlModelVao.bind();
        glDrawArrays(GL_TRIANGLES, 0, m_stlModelVertices.size());
        m_stlModelVao.release();
        m_program->setUniformValue("uModelLighting", 0);
    }

    if (edlActive) {
        m_program->release();
        const QSize physicalSize = widgetPhysicalSize();
        const float physicalRadius = m_edlConfig.radiusPx * float(devicePixelRatioF()) * m_edlConfig.renderScale;
        m_edlRenderer->composite(defaultFramebufferObject(),
                                 physicalSize,
                                 m_edlConfig,
                                 physicalRadius,
                                 m_backgroundTopColor,
                                 m_backgroundBottomColor);
        glDepthMask(GL_TRUE);
        m_program->bind();
        m_program->setUniformValue("modelView", m_modelView);
        m_program->setUniformValue("projection", m_projection);
        m_program->setUniformValue("normalMatrix", normalMatrix);
        m_program->setUniformValue("uPointSize", m_pointSize);
        m_program->setUniformValue("uModelLighting", 0);
        m_program->setUniformValue("uEdlPass", 0);
        m_program->setUniformValue("uSelectionEnabled", 0);
        m_program->setUniformValue("uPersistEnabled", 0);
        setPrimitiveState(false, false);
    }

    if (m_slamTrajectoryVertexCount > 1 && m_slamTrajectoryVao.isCreated()) {
        glDisable(GL_DEPTH_TEST);
        glLineWidth(m_slamRenderSnapshot.trajectoryLineWidthPx);
        m_slamTrajectoryVao.bind();
        glDrawArrays(GL_LINE_STRIP, 0, m_slamTrajectoryVertexCount);
        m_slamTrajectoryVao.release();
        glLineWidth(1.0f);
        glEnable(GL_DEPTH_TEST);
    }
    if (m_slamPoseAxisVertexCount > 0 && m_slamPoseAxisVao.isCreated()) {
        glDisable(GL_DEPTH_TEST);
        m_slamPoseAxisVao.bind();
        glDrawArrays(GL_TRIANGLES, 0, m_slamPoseAxisVertexCount);
        m_slamPoseAxisVao.release();
        glEnable(GL_DEPTH_TEST);
    }

    if (m_crossSectionState.enabled && m_crossSectionState.initialized) {
        const PointCloudCrossSection::Camera camera = crossSectionCamera();
        QVector<PointCloudCrossSection::ColoredVertex> crossSectionLines =
            PointCloudCrossSection::buildBoxLines(m_crossSectionState);
        m_crossSectionBoxLineVertexCount = crossSectionLines.size();
        crossSectionLines += PointCloudCrossSection::buildGizmoLines(m_crossSectionState, camera);
        const QVector<PointCloudCrossSection::ColoredVertex> crossSectionTriangles =
            PointCloudCrossSection::buildGizmoTriangles(m_crossSectionState, camera);
        uploadCrossSectionLines(crossSectionLines);
        uploadCrossSectionTriangles(crossSectionTriangles);

        m_program->setUniformValue("uPersistEnabled", 0);
        setPrimitiveState(false, false);
        glDisable(GL_DEPTH_TEST);
        m_crossSectionTriangleVao.bind();
        glDrawArrays(GL_TRIANGLES, 0, m_crossSectionTriangleVertexCount);
        m_crossSectionTriangleVao.release();
        m_crossSectionVao.bind();
        glLineWidth(2.0f);
        glDrawArrays(GL_LINES, 0, m_crossSectionBoxLineVertexCount);
        glLineWidth(3.5f);
        glDrawArrays(GL_LINES,
                     m_crossSectionBoxLineVertexCount,
                     m_crossSectionVertexCount - m_crossSectionBoxLineVertexCount);
        m_crossSectionVao.release();
        glLineWidth(1.0f);
        glEnable(GL_DEPTH_TEST);
    }
    
    // ==========================================
    // 4.5 绘制 Overlay 坐标轴 (固定在左下角)
    // ==========================================
    // 临时关闭深度测试，确保坐标轴不会被网格或点云遮挡
    glDisable(GL_DEPTH_TEST);
    m_program->setUniformValue("uSelectionEnabled", 0); 
    m_program->setUniformValue("uPersistEnabled", 0);
    setPrimitiveState(false, false);

    QMatrix4x4 overlayProj;
    // 构建正交投影，左下角为(0,0)，宽高为像素分辨率
    overlayProj.ortho(0.0f, float(width()), 0.0f, float(height()), -1000.0f, 1000.0f);
    
    QMatrix4x4 overlayMV;
    float padding = 85.0f;     // 原点距离屏幕左下角的像素距离
    float axisLength = 40.0f;  // 轴在屏幕上的像素长度（假设 VAO 中线条长度是 1.0）
    
    overlayMV.translate(padding, padding, 0.0f); 
    overlayMV = overlayMV * quaternionToMatrix(m_orientation); // 只应用场景旋转，没有平移
    overlayMV.scale(axisLength);

    m_program->setUniformValue("projection", overlayProj);
    m_program->setUniformValue("modelView", overlayMV);

    glLineWidth(3.0f); // 使得叠加轴更清晰
    m_axesVao.bind();
    glDrawArrays(GL_LINES, 0, m_axesVertexCount);
    m_axesVao.release();
    glLineWidth(1.0f);

    // 恢复深度测试并解绑着色器
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    m_program->release();

    // ==========================================
    // 5. 绘制 2D 叠加层 (文本、UI组件等)
    // ==========================================

    // 绘制坐标轴标签 "X", "Y", "Z" (基于 Overlay 轴的位置)
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::TextAntialiasing, true);
        painter.setPen(Qt::white);
        QFont f = painter.font();
        f.setBold(true);
        painter.setFont(f);

        QVector3D tips[] = { QVector3D(1.0f, 0, 0), QVector3D(0, 1.0f, 0), QVector3D(0, 0, 1.0f) };
        QString labels[] = { "X", "Y", "Z" };

        float labelRadius = axisLength + 12.0f; // 标签离原点的距离比轴长一点

        for (int i = 0; i < 3; ++i) {
            // 仅使用场景的旋转矩阵来推算 X, Y 的偏移
            QVector3D rotatedPos = rot * (tips[i] * labelRadius);
            
            // OpenGL的正交矩阵原点在左下角，而QPainter原点在左上角
            // x = padding + rotatedPos.x()
            // y_gl = padding + rotatedPos.y()  ->  y_painter = height() - y_gl
            int sx = int(padding + rotatedPos.x());
            int sy = int(height() - (padding + rotatedPos.y()));

            // // 可选：利用 rotatedPos.z() 实现背面轴标签置灰或半透明
            // if (rotatedPos.z() < -10.0f) {
            //     painter.setPen(QColor(150, 150, 150, 180)); // 偏向内部变暗
            // } else {
            //     painter.setPen(Qt::white);
            // }
            painter.setPen(Qt::white);

            // 微调居中，让字母正中心对准坐标
            painter.drawText(sx - 4, sy + 4, labels[i]); 
        }
    }

    // 2D 覆盖层：绘制测距点、连线与距离
    if (m_measureMode && (m_haveP1 || m_haveP2)) {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(QPen(QColor(255,0,0), 2));
        const QMatrix4x4 mvp = m_projection * m_modelView;
        auto drawPoint = [&](const QPointF& s) {
            painter.setBrush(QColor(255,0,0));
            painter.drawEllipse(s, 4, 4);
        };
        auto projectToScreen = [&](const QVector3D& w, QPointF& screen) -> bool {
            QVector4D hp(w, 1.0f);
            ProjectedScreenPoint projected;
            if (!projectVisiblePointToScreen(hp, m_projection * m_modelView, width(), height(), projected)) {
                return false;
            }
            screen = QPointF(projected.x, projected.y);
            return true;
        };
        QPointF p1s;
        QPointF p2s;
        const bool p1Visible = m_haveP1 && projectToScreen(m_p1, p1s);
        const bool p2Visible = m_haveP2 && projectToScreen(m_p2, p2s);
        if (p1Visible) drawPoint(p1s);
        if (p2Visible) drawPoint(p2s);
        if (m_haveP1 && m_haveP2) {
            QPointF lineA;
            QPointF lineB;
            if (projectClippedSegmentToScreen(m_p1, m_p2, mvp, width(), height(), lineA, lineB)) {
                painter.drawLine(lineA, lineB);
                const QPointF mid = (lineA + lineB) * 0.5;
                const QString label = QString::number((m_p2 - m_p1).length(), 'f', 3) + " m";
                const QFontMetrics fm(painter.font());
                QRectF labelRect(mid + QPointF(8.0, -8.0 - fm.ascent()),
                                 QSizeF(fm.horizontalAdvance(label) + 8, fm.height() + 4));
                const QRectF bounds = rect().adjusted(4, 4, -4, -4);
                if (labelRect.right() > bounds.right()) {
                    labelRect.moveRight(bounds.right());
                }
                if (labelRect.left() < bounds.left()) {
                    labelRect.moveLeft(bounds.left());
                }
                if (labelRect.bottom() > bounds.bottom()) {
                    labelRect.moveBottom(bounds.bottom());
                }
                if (labelRect.top() < bounds.top()) {
                    labelRect.moveTop(bounds.top());
                }
                painter.setPen(QPen(QColor(255,0,0)));
                painter.drawText(labelRect, Qt::AlignCenter, label);
            }
        }
    }

    // 右下角图例 (保持不变)
    if (m_legendVisible) {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setRenderHint(QPainter::TextAntialiasing, true);
        painter.setPen(Qt::white);
 
         // 定义尺寸参数
         const int barWidth = 18;
         const int barHeight = 160;
         const int margin = 14;
         const int tickLen = 6;
         const int labelSpacing = 4;
 
         // 字体与标题
         painter.setPen(Qt::white);
         QFont f = painter.font();
         f.setPointSizeF(f.pointSizeF()*0.9f);
         f.setItalic(true);
         painter.setFont(f);
         QFontMetrics fm(painter.font());

         if (m_legendMode == 4) {
             const int swatchSize = 14;
             const int rowGap = 6;
             const int rowHeight = swatchSize + rowGap;
             const int titleHeight = 18;
             const int rightPadding = 10;
             int labelMaxWidth = 0;
             for (int i = 0; i < m_lineLegendColors.size(); ++i) {
                 labelMaxWidth = std::max(labelMaxWidth, fm.horizontalAdvance(QString("Line %1").arg(m_lineLegendNumbers.at(i))));
             }
             const int legendWidth = swatchSize + 8 + labelMaxWidth + rightPadding;
             const int legendHeight = titleHeight + 8 + m_lineLegendColors.size() * rowHeight - rowGap;
             const int left = width() - margin - legendWidth;
             const int top = height() - margin - legendHeight;
             painter.drawText(QRect(left, top, legendWidth, titleHeight), Qt::AlignLeft | Qt::AlignVCenter, "Line");
             int y = top + titleHeight + 8;
             for (int i = 0; i < m_lineLegendColors.size(); ++i) {
                 const QString label = QString("Line %1").arg(m_lineLegendNumbers.at(i));
                 const QRect swatch(left, y, swatchSize, swatchSize);
                 painter.fillRect(swatch, m_lineLegendColors.at(i));
                 painter.setPen(QColor(255,255,255,200));
                 painter.drawRect(swatch.adjusted(0,0,-1,-1));
                 painter.setPen(Qt::white);
                 painter.drawText(QRect(swatch.right() + 8, y - 2, labelMaxWidth + rightPadding, swatchSize + 4),
                                  Qt::AlignLeft | Qt::AlignVCenter,
                                  label);
                 y += rowHeight;
             }
         } else {
 
         // 准备标签文本并计算最大宽度
         QString title;
         switch (m_legendMode) {
             case 0: title = "Reflectivity"; break;
             case 1: title = "Distance"; break;
             case 2: title = "Elevation"; break;
             case 3: title = "Color"; break;
             case 4: title = "Line"; break;
             default: title = "Unknown"; break;
         }
         QStringList tickLabels;
         if (m_legendMode == 0) {
             tickLabels = {"255", "204", "153", "102", "51", "0"};
         } else if (m_legendMode == 1 || m_legendMode == 2) {
             auto fmt = [](float v) { return QString::number(v, 'f', 2); };
             tickLabels = { fmt(m_legendMax), fmt((m_legendMin + m_legendMax) * 0.5f), fmt(m_legendMin) };
         } else {
             tickLabels = {};
         }
         int labelMaxWidth = 0;
         for (const QString& s : tickLabels) labelMaxWidth = std::max(labelMaxWidth, fm.horizontalAdvance(s));
 
         // 计算图例总宽度并放置在右下角，保证标签不裁剪
         int legendWidth = barWidth + tickLen + labelSpacing + (tickLabels.isEmpty() ? 0 : labelMaxWidth);
         int barLeft = width() - margin - legendWidth;
         QRect barRect(barLeft, height() - margin - barHeight, barWidth, barHeight);
 
         // 背景透明，不绘制背景
 
         // 渐变：反射率/距离使用同一色标，高度修正为顶部红、底部蓝
         QLinearGradient grad(barRect.topLeft(), barRect.bottomLeft());
         auto addReflectivityStops = [this, &grad]() {
             const int colorCount = int(m_legendGradientColors.size());
             const int last = colorCount - 1;
             for (int i = 0; i < colorCount; ++i) {
                 grad.setColorAt(1.0 - double(i) / double(last), m_legendGradientColors.at(i));
             }
         };
         auto addDistanceStops = [&grad]() {
             grad.setColorAt(0.00, QColor(255,   0,   0));
             grad.setColorAt(0.25, QColor(255, 255,   0));
             grad.setColorAt(0.50, QColor(  0, 255,   0));
             grad.setColorAt(0.75, QColor(  0, 255, 255));
             grad.setColorAt(1.00, QColor(  0,   0, 255));
         };
         auto addElevationStops = [&grad]() {
             // 顶部为高值->红，底部为低值->蓝（修正方向）
             grad.setColorAt(0.00, QColor(255,   0,   0));
             grad.setColorAt(1.00, QColor(  0,   0, 255));
         };
         if (m_legendMode == 0) addReflectivityStops();
         else if (m_legendMode == 1) addDistanceStops();
         else if (m_legendMode == 2) addElevationStops();
         else { grad.setColorAt(0.0, Qt::white); grad.setColorAt(1.0, Qt::white); }
 
         painter.fillRect(barRect, QBrush(grad));
         painter.setPen(QColor(255,255,255,200));
         painter.drawRect(barRect.adjusted(0,0,-1,-1));
 
         // 标题
         painter.setPen(Qt::white);
         QRect titleRect(barLeft - 6, barRect.top() - 30, legendWidth + 12, 18);
         painter.drawText(titleRect, Qt::AlignLeft | Qt::AlignVCenter, title);
 
         // 刻度
         auto drawTick = [&](float norm, const QString& label) {
             int y = barRect.top() + int((1.0f - norm) * barRect.height());
             painter.drawLine(barRect.right() + 2, y, barRect.right() + 2 + tickLen, y);
             painter.drawText(barRect.right() + 2 + tickLen + labelSpacing, y + 4, label);
         };
         if (m_legendMode == 0) {
             drawTick(1.0f,  "255");
             drawTick(0.8f,  "204");
             drawTick(0.6f,  "153");
             drawTick(0.4f,  "102");
             drawTick(0.2f,  "51");
             drawTick(0.0f,  "0");
         } else if (m_legendMode == 1 || m_legendMode == 2) {
             auto fmt = [](float v) { return QString::number(v, 'f', 2); };
             drawTick(1.0f, fmt(m_legendMax));
             drawTick(0.5f, fmt((m_legendMin + m_legendMax) * 0.5f));
             drawTick(0.0f, fmt(m_legendMin));
         }
         }
     }

    // 屏幕框矩形（仅拖拽过程显示）
    if (m_selecting) {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        QRect r = m_selectionRect();
        QColor fill(0, 120, 215, 40);
        QColor border(0, 120, 215, 200);
        painter.fillRect(r, fill);
        QPen pen(border);
        pen.setWidth(1);
        painter.setPen(pen);
        painter.drawRect(r.adjusted(0,0,-1,-1));
    }

}

void PointCloudView::setEdlConfig(const EdlConfig& config)
{
    const bool releaseFramebuffer = m_edlConfig.enabled && !config.enabled &&
                                    m_edlRenderer && context();
    m_edlConfig = config;
    m_edlFallbackReported = false;
    if (releaseFramebuffer) {
        ScopedOpenGLContext current(this);
        m_edlRenderer->destroyFramebuffer();
    }
    update();
}

void PointCloudView::setSlamFollowPose(const SlamRenderPose& pose)
{
    Q_ASSERT(qApp);
    Q_ASSERT(QThread::currentThread() == qApp->thread());
    m_slamFollowPoseValid = pose.valid;
    if (pose.valid) {
        m_slamFollowPosition = QVector3D(pose.tx, pose.ty, pose.tz);
        m_slamFollowOrientation = QQuaternion(pose.qw, pose.qx, pose.qy, pose.qz).normalized();
    }
    if (m_projectionMode == ProjectionMode::SlamPoseFollow) {
        update();
    }
}

bool PointCloudView::isEdlSupported() const
{
    return m_edlRenderer && m_edlRenderer->isSupported();
}

QString PointCloudView::edlErrorMessage() const
{
    return m_edlRenderer ? m_edlRenderer->errorMessage()
                         : QStringLiteral("EDL renderer is not initialized");
}

QSize PointCloudView::widgetPhysicalSize() const
{
    const qreal dpr = devicePixelRatioF();
    return QSize(qMax(1, int(std::lround(width() * dpr))),
                 qMax(1, int(std::lround(height() * dpr))));
}

QSize PointCloudView::edlPhysicalSize() const
{
    const qreal scale = devicePixelRatioF() * m_edlConfig.renderScale;
    return QSize(qMax(1, int(std::lround(width() * scale))),
                 qMax(1, int(std::lround(height() * scale))));
}

void PointCloudView::resizeGL(int w, int h)
{
    glViewport(0, 0, w * devicePixelRatioF(), h * devicePixelRatioF());
}

void PointCloudView::hideEvent(QHideEvent* event)
{
    if (m_edlRenderer && context()) {
        ScopedOpenGLContext current(this);
        m_edlRenderer->destroyFramebuffer();
    }
    QOpenGLWidget::hideEvent(event);
}

void PointCloudView::mousePressEvent(QMouseEvent *event)
{
    m_lastMousePos = event->pos();
    m_activeButton = event->button();
    m_mousePressed = true;

    if (m_crossSectionState.enabled &&
        event->button() == Qt::LeftButton &&
        PointCloudCrossSection::beginDrag(m_crossSectionState, crossSectionCamera(), event->pos())) {
        setCursor(Qt::ClosedHandCursor);
        update();
        return;
    }

    // 测距：按住Ctrl+左键依次选择P1与P2
    if (m_measureMode && event->button() == Qt::LeftButton && (event->modifiers() & Qt::ControlModifier)) {
        QVector3D w; QPoint s;
        if (pickNearestPoint(event->pos(), w, s)) {
            if (!m_haveP1) {
                m_p1 = w; m_p1Screen = s; m_haveP1 = true; m_haveP2 = false;
            } else if (m_haveP1 && !m_haveP2) {
                m_p2 = w; m_p2Screen = s; m_haveP2 = true;
            } else { // m_haveP1 && m_haveP2 -> 重新开始新一轮测距
                m_p1 = w; m_p1Screen = s; m_haveP1 = true; m_haveP2 = false;
            }
            QWidget* wdt = window();
            if (wdt) QMetaObject::invokeMethod(wdt, "onMeasurementUpdated", Qt::QueuedConnection);
            update();
            return;
        }
    }

    if (m_selectionModeEnabled && event->button() == Qt::LeftButton && (event->modifiers() & Qt::ControlModifier)) {
        // 开启新的框选
        m_selectionLocked = false;
        m_selectionRegion = SelectionRegion();
        ++m_selectionGeneration;
        {
            QMutexLocker locker(&m_pointsMutex);
            clearSelectionCaches();
        }
        emit selectionPointsReady({}, 0);
        m_selecting = true;
        m_selStart = m_selEnd = event->pos();
        update();
        return;
    }
    // 相机操作：左键旋转 或 中键/右键平移
    if (!(m_projectionMode == ProjectionMode::SlamPoseFollow && m_slamFollowPoseValid) &&
        ((event->button() == Qt::LeftButton && !(event->modifiers() & Qt::ControlModifier)) ||
         event->button() == Qt::MiddleButton ||
         event->button() == Qt::RightButton)) {
        setCursor(Qt::ClosedHandCursor);
    }
}

void PointCloudView::mouseMoveEvent(QMouseEvent *event)
{
    if (!m_mousePressed) {
        if (m_crossSectionState.enabled) {
            const bool changed = PointCloudCrossSection::updateHover(m_crossSectionState, crossSectionCamera(), event->pos());
            if (m_crossSectionState.hoverHandle != PointCloudCrossSection::HandleType::None) {
                setCursor(Qt::PointingHandCursor);
            } else {
                setCursor(Qt::ArrowCursor);
            }
            if (changed) {
                update();
            }
        }
        return;
    }
    
    QPoint delta = event->pos() - m_lastMousePos;

    if (m_crossSectionState.dragging) {
        PointCloudCrossSection::updateDrag(m_crossSectionState, crossSectionCamera(), event->pos());
        requestCrossSectionClip(false);
        update();
        return;
    }

    // 移除拖动更新第二点的逻辑，改为仅在Ctrl+点击时更新

    if (m_selectionModeEnabled && m_selecting && m_activeButton == Qt::LeftButton && (event->modifiers() & Qt::ControlModifier)) {
        m_selEnd = event->pos();
        updateSelectionRegionFromRect(m_selectionRect());
        update();
        return;
    }
    if (m_projectionMode == ProjectionMode::SlamPoseFollow && m_slamFollowPoseValid) {
        m_lastMousePos = event->pos();
        return;
    }
    if (m_activeButton == Qt::LeftButton) {
        QVector3D va = mapToArcball(m_lastMousePos);
        QVector3D vb = mapToArcball(event->pos());
        QVector3D axis = QVector3D::crossProduct(va, vb);
        float dot = std::clamp(QVector3D::dotProduct(va, vb), -1.0f, 1.0f);
        float angle = std::acos(dot);
        if (axis.lengthSquared() > 1e-6f && angle > 1e-6f) {
            QQuaternion dq = QQuaternion::fromAxisAndAngle(axis.normalized(), angle * 180.0f / float(M_PI));
            m_orientation = dq * m_orientation;
        }
    } else if (m_activeButton == Qt::MiddleButton || m_activeButton == Qt::RightButton) {
        // --- 新的平移逻辑 ---
        float aspect = (float)qMax(1, width()) / (float)qMax(1, height());
        float fovy_rad = 45.0f * float(M_PI) / 180.0f;
        
        // 计算当前距离下，屏幕像素对应的世界坐标尺寸
        float factor = 2.0f * m_distance * std::tan(fovy_rad * 0.5f);
        float worldPerPixelY = factor / (float)qMax(1, height());
        float worldPerPixelX = worldPerPixelY * aspect;

        // 计算相机当前在世界坐标系下的局部坐标轴（右方向和上方向）
        // 这里使用 m_orientation 的共轭来获取逆变换方向
        QVector3D cameraRight = this->cameraRight();
        QVector3D cameraUp = this->cameraUp();

        // 更新观察中心或观察者相机位置
        const QVector3D cameraDelta = -cameraRight * (delta.x() * worldPerPixelX)
                                    + cameraUp * (delta.y() * worldPerPixelY);
        if (m_projectionMode == ProjectionMode::ObserverPerspective) {
            m_viewerPosition += cameraDelta;
        } else {
            m_target += cameraDelta;
        }
    }
    
    m_lastMousePos = event->pos();
    update();
}

void PointCloudView::mouseReleaseEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
    if (m_crossSectionState.dragging) {
        PointCloudCrossSection::endDrag(m_crossSectionState);
        requestCrossSectionClip(true);
        m_mousePressed = false;
        m_activeButton = Qt::NoButton;
        setCursor(Qt::ArrowCursor);
        update();
        return;
    }

    if (m_selectionModeEnabled && m_selecting && m_activeButton == Qt::LeftButton) {
        m_selecting = false;
        updateSelectionRegionFromRect(m_selectionRect());
        update();
    }
    m_mousePressed = false;
    m_activeButton = Qt::NoButton;
    setCursor(Qt::ArrowCursor);
}

void PointCloudView::leaveEvent(QEvent *event)
{
    Q_UNUSED(event);
    setCursor(Qt::ArrowCursor);
    if (m_crossSectionState.hoverHandle != PointCloudCrossSection::HandleType::None) {
        m_crossSectionState.hoverHandle = PointCloudCrossSection::HandleType::None;
        update();
    }
    QOpenGLWidget::leaveEvent(event);
}

void PointCloudView::wheelEvent(QWheelEvent *event)
{
    if (m_projectionMode == ProjectionMode::SlamPoseFollow && m_slamFollowPoseValid) {
        event->accept();
        return;
    }
    const float oldDistance = m_distance;
    const float factor = 1.0f - event->angleDelta().y() * 0.001f; // 每格约 1.5% 的比例变化
    m_distance = qMax(0.1f, m_distance * factor);                  // 最小距离 0.1
    if (m_projectionMode == ProjectionMode::ObserverPerspective) {
        m_viewerPosition += cameraForward() * (oldDistance - m_distance);
    }
    update();
}

void PointCloudView::dragEnterEvent(QDragEnterEvent* event)
{
    if (!event) {
        return;
    }

    if (!supportedPlaybackDropFiles(event->mimeData()).isEmpty()) {
        event->acceptProposedAction();
    }
}

void PointCloudView::dropEvent(QDropEvent* event)
{
    if (!event) {
        return;
    }

    const QStringList files = supportedPlaybackDropFiles(event->mimeData());
    if (files.isEmpty()) {
        return;
    }

    event->acceptProposedAction();
    for (const QString& file : files) {
        emit lvx2FileDropped(file);
    }
}

void PointCloudView::uploadPointCloudPoints(QVector<PointCloudPoint>&& points)
{
    QMutexLocker locker(&m_pointsMutex);
    destroyPointCloudSegments();
    m_points = std::move(points);
    m_selectedPoints.clear();
    m_selectionPointCount = 0;
    m_pointCloudGpuUploadPending = true;
    update();
}

void PointCloudView::uploadPointCloudSegment(PointCloudSegment& segment)
{
    if (!uploadPointCloudBuffer(this,
                                m_program,
                                segment.points,
                                segment.vbo,
                                segment.vao,
                                segment.bufferCapacityBytes,
                                segment.pointCount)) {
        m_pointCloudGpuUploadPending = true;
    }
}

void PointCloudView::uploadPointCloudSegmentClip(PointCloudSegment& segment)
{
    if (!uploadPointCloudBuffer(this,
                                m_program,
                                segment.clippedPoints,
                                segment.clippedVbo,
                                segment.clippedVao,
                                segment.clippedBufferCapacityBytes,
                                segment.clippedPointCount)) {
        m_pointCloudGpuUploadPending = true;
    }
}

void PointCloudView::uploadPointCloudSegmentSelection(PointCloudSegment& segment)
{
    QVector<PointCloudPoint> overlayPoints = segment.selectedPoints;
    for (PointCloudPoint& point : overlayPoints) {
        point.r = 1.0f;
        point.g = 0.0f;
        point.b = 0.0f;
    }
    if (!uploadPointCloudBuffer(this,
                                m_program,
                                overlayPoints,
                                segment.selectedVbo,
                                segment.selectedVao,
                                segment.selectedBufferCapacityBytes,
                                segment.selectedPointCount)) {
        m_pointCloudGpuUploadPending = true;
    }
}

void PointCloudView::uploadSelectionPoints(QVector<PointCloudPoint>&& points)
{
    m_selectedPoints = std::move(points);
    m_pointCloudGpuUploadPending = true;
    update();
}

void PointCloudView::syncPendingPointCloudBuffers()
{
    if (!m_pointCloudGpuUploadPending || !m_program || !context()) {
        return;
    }

    bool synced = true;
    {
        QMutexLocker locker(&m_pointsMutex);
        if (!m_points.isEmpty() || m_vbo.isCreated()) {
            int pointCount = m_points.size();
            synced = uploadPointCloudBuffer(this,
                                            m_program,
                                            m_points,
                                            m_vbo,
                                            m_vao,
                                            m_pointCloudBufferCapacityBytes,
                                            pointCount) && synced;
        }
        for (std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
            if (!segment) {
                continue;
            }
            synced = uploadPointCloudBuffer(this,
                                            m_program,
                                            segment->points,
                                            segment->vbo,
                                            segment->vao,
                                            segment->bufferCapacityBytes,
                                            segment->pointCount) && synced;
            if (!segment->clippedPoints.isEmpty() || segment->clippedVbo.isCreated()) {
                synced = uploadPointCloudBuffer(this,
                                                m_program,
                                                segment->clippedPoints,
                                                segment->clippedVbo,
                                                segment->clippedVao,
                                                segment->clippedBufferCapacityBytes,
                                                segment->clippedPointCount) && synced;
            }
            if (!segment->selectedPoints.isEmpty() || segment->selectedVbo.isCreated()) {
                QVector<PointCloudPoint> overlayPoints = segment->selectedPoints;
                for (PointCloudPoint& point : overlayPoints) {
                    point.r = 1.0f;
                    point.g = 0.0f;
                    point.b = 0.0f;
                }
                synced = uploadPointCloudBuffer(this,
                                                m_program,
                                                overlayPoints,
                                                segment->selectedVbo,
                                                segment->selectedVao,
                                                segment->selectedBufferCapacityBytes,
                                                segment->selectedPointCount) && synced;
            }
        }
    }

    if (!m_selectedPoints.isEmpty() || m_selectionVbo.isCreated()) {
        QVector<PointCloudPoint> overlayPoints = m_selectedPoints;
        for (PointCloudPoint& point : overlayPoints) {
            point.r = 1.0f;
            point.g = 0.0f;
            point.b = 0.0f;
        }
        synced = uploadPointCloudBuffer(this,
                                        m_program,
                                        overlayPoints,
                                        m_selectionVbo,
                                        m_selectionVao,
                                        m_selectionBufferCapacityBytes,
                                        m_selectionPointCount) && synced;
    }

    m_pointCloudGpuUploadPending = !synced;
}

void PointCloudView::destroyPointCloudSegments()
{
    if (m_pointCloudSegments.empty()) {
        return;
    }

    ScopedOpenGLContext current(this);
    for (std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
        if (!segment) {
            continue;
        }
        if (segment->vbo.isCreated()) {
            segment->vbo.destroy();
        }
        if (segment->vao.isCreated()) {
            segment->vao.destroy();
        }
        if (segment->clippedVbo.isCreated()) {
            segment->clippedVbo.destroy();
        }
        if (segment->clippedVao.isCreated()) {
            segment->clippedVao.destroy();
        }
        clearSegmentSelectionBuffers(*segment);
    }
    m_pointCloudSegments.clear();
}

void PointCloudView::clearSegmentSelectionBuffers(PointCloudSegment& segment)
{
    segment.selectedPoints.clear();
    segment.selectedPointCount = 0;
    segment.selectedBufferCapacityBytes = 0;
    if (segment.selectedVbo.isCreated() || segment.selectedVao.isCreated()) {
        ScopedOpenGLContext current(this);
        if (segment.selectedVbo.isCreated()) {
            segment.selectedVbo.destroy();
        }
        if (segment.selectedVao.isCreated()) {
            segment.selectedVao.destroy();
        }
    }
}

void PointCloudView::clearSelectionCaches()
{
    uploadSelectionPoints(QVector<PointCloudPoint>());
    for (std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
        if (segment) {
            clearSegmentSelectionBuffers(*segment);
        }
    }
}

void PointCloudView::clearPointCloudSegments()
{
    QMutexLocker locker(&m_pointsMutex);
    ++m_crossSectionClipGeneration;
    ++m_selectionGeneration;
    m_points.clear();
    m_selectedPoints.clear();
    m_selectionPointCount = 0;
    m_selectionBufferCapacityBytes = 0;
    destroyPointCloudSegments();
    if (m_vbo.isCreated()) {
        ScopedOpenGLContext current(this);
        m_vbo.bind();
        m_vbo.allocate(0);
        m_pointCloudBufferCapacityBytes = 0;
        m_vbo.release();
    }
    if (m_selectionVbo.isCreated() || m_selectionVao.isCreated()) {
        ScopedOpenGLContext current(this);
        if (m_selectionVbo.isCreated()) {
            m_selectionVbo.destroy();
        }
        if (m_selectionVao.isCreated()) {
            m_selectionVao.destroy();
        }
    }
    update();
}

void PointCloudView::appendPointCloudSegment(QVector<PointCloudPoint>&& points)
{
    quint64 segmentId = 0;
    QVector<PointCloudPoint> segmentPointsSnapshot;
    const bool crossSectionEnabled = m_crossSectionState.enabled && m_crossSectionState.initialized;
    const bool selectionEnabled = m_selectionLocked && m_selectionRegion.valid;
    {
        QMutexLocker locker(&m_pointsMutex);
        m_points.clear();
        auto segment = std::make_unique<PointCloudSegment>();
        segment->id = m_nextPointCloudSegmentId++;
        segment->points = std::move(points);
        m_pointCloudGpuUploadPending = true;
        segmentId = segment->id;
        segmentPointsSnapshot = segment->points;
        m_pointCloudSegments.push_back(std::move(segment));
    }
    if (crossSectionEnabled) {
        startCrossSectionSegmentClipJob(segmentId, segmentPointsSnapshot);
    } else if (selectionEnabled) {
        startSelectionSegmentJob(segmentId, segmentPointsSnapshot);
    }
    update();
}

void PointCloudView::removeFirstPointCloudSegment()
{
    int clippedPointCount = 0;
    int sourcePointCount = 0;
    const bool crossSectionEnabled = m_crossSectionState.enabled && m_crossSectionState.initialized;
    const bool selectionEnabled = m_selectionLocked && m_selectionRegion.valid;
    {
        QMutexLocker locker(&m_pointsMutex);
        if (m_pointCloudSegments.empty()) {
            return;
        }

        std::unique_ptr<PointCloudSegment>& segment = m_pointCloudSegments.front();
        if (segment) {
            ScopedOpenGLContext current(this);
            if (segment->vbo.isCreated()) {
                segment->vbo.destroy();
            }
            if (segment->vao.isCreated()) {
                segment->vao.destroy();
            }
            if (segment->clippedVbo.isCreated()) {
                segment->clippedVbo.destroy();
            }
            if (segment->clippedVao.isCreated()) {
                segment->clippedVao.destroy();
            }
            clearSegmentSelectionBuffers(*segment);
        }
        m_pointCloudSegments.pop_front();
        if (crossSectionEnabled) {
            for (const std::unique_ptr<PointCloudSegment>& currentSegment : m_pointCloudSegments) {
                if (!currentSegment) {
                    continue;
                }
                sourcePointCount += currentSegment->points.size();
                clippedPointCount += currentSegment->clippedPoints.size();
            }
        }
    }
    if (crossSectionEnabled) {
        emit crossSectionChanged(clippedPointCount, sourcePointCount);
    }
    if (selectionEnabled) {
        requestSelectionPublish();
    }
    update();
}

int PointCloudView::pointCloudSegmentCount() const
{
    QMutexLocker locker(const_cast<QMutex*>(&m_pointsMutex));
    return int(m_pointCloudSegments.size());
}

QVector<PointCloudPoint> PointCloudView::currentPoints() const
{
    QVector<PointCloudPoint> points;
    QMutexLocker locker(const_cast<QMutex*>(&m_pointsMutex));
    qsizetype pointCount = m_points.size();
    for (const std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
        if (segment) {
            pointCount += segment->points.size();
        }
    }
    points.reserve(pointCount);
    points += m_points;
    for (const std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
        if (segment) {
            points += segment->points;
        }
    }
    return points;
}

QVector<PointCloudPoint> PointCloudView::currentCrossSectionPoints() const
{
    QVector<PointCloudPoint> points;
    QMutexLocker locker(const_cast<QMutex*>(&m_pointsMutex));
    if (m_crossSectionState.enabled && m_crossSectionState.initialized && !m_pointCloudSegments.empty()) {
        qsizetype pointCount = 0;
        for (const std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
            if (segment) {
                pointCount += segment->clippedPoints.size();
            }
        }
        points.reserve(pointCount);
        for (const std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
            if (segment) {
                points += segment->clippedPoints;
            }
        }
        return points;
    }
    return m_crossSectionState.clippedPoints;
}

bool PointCloudView::pointCloudSegmentSourceBounds(QVector3D& minPoint, QVector3D& maxPoint) const
{
    QMutexLocker locker(const_cast<QMutex*>(&m_pointsMutex));
    bool hasPoint = false;
    auto includePoint = [&](const PointCloudPoint& point) {
        const QVector3D current(point.x, point.y, point.z);
        if (!hasPoint) {
            minPoint = current;
            maxPoint = current;
            hasPoint = true;
        } else {
            minPoint.setX(std::min(minPoint.x(), current.x()));
            minPoint.setY(std::min(minPoint.y(), current.y()));
            minPoint.setZ(std::min(minPoint.z(), current.z()));
            maxPoint.setX(std::max(maxPoint.x(), current.x()));
            maxPoint.setY(std::max(maxPoint.y(), current.y()));
            maxPoint.setZ(std::max(maxPoint.z(), current.z()));
        }
    };

    for (const std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
        if (!segment) {
            continue;
        }
        for (const PointCloudPoint& point : segment->points) {
            includePoint(point);
        }
    }
    return hasPoint;
}

void PointCloudView::forEachDisplayedPoint(const std::function<bool(const PointCloudPoint&)>& visitor) const
{
    QMutexLocker locker(const_cast<QMutex*>(&m_pointsMutex));
    for (const PointCloudPoint& point : m_points) {
        if (!visitor(point)) {
            return;
        }
    }
    for (const std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
        if (!segment) {
            continue;
        }
        const QVector<PointCloudPoint>& points =
            (m_crossSectionState.enabled && m_crossSectionState.initialized)
                ? segment->clippedPoints
                : segment->points;
        for (const PointCloudPoint& point : points) {
            if (!visitor(point)) {
                return;
            }
        }
    }
}

void PointCloudView::uploadCrossSectionLines(const QVector<PointCloudCrossSection::ColoredVertex>& vertices)
{
    m_crossSectionVertexCount = vertices.size();
    if (!m_crossSectionVbo.isCreated()) {
        return;
    }
    ScopedOpenGLContext current(this);
    m_crossSectionVbo.bind();
    m_crossSectionVbo.allocate(vertices.constData(), static_cast<int>(vertices.size() * qsizetype(sizeof(PointCloudCrossSection::ColoredVertex))));
    m_crossSectionVbo.release();
}

void PointCloudView::uploadCrossSectionTriangles(const QVector<PointCloudCrossSection::ColoredVertex>& vertices)
{
    m_crossSectionTriangleVertexCount = vertices.size();
    if (!m_crossSectionTriangleVbo.isCreated()) {
        return;
    }
    ScopedOpenGLContext current(this);
    m_crossSectionTriangleVbo.bind();
    m_crossSectionTriangleVbo.allocate(vertices.constData(), static_cast<int>(vertices.size() * qsizetype(sizeof(PointCloudCrossSection::ColoredVertex))));
    m_crossSectionTriangleVbo.release();
}

void PointCloudView::uploadStlModelVertices()
{
    if (!m_stlModelVbo.isCreated()) {
        return;
    }
    ScopedOpenGLContext current(this);
    m_stlModelVbo.bind();
    m_stlModelVbo.allocate(m_stlModelVertices.constData(), static_cast<int>(m_stlModelVertices.size() * qsizetype(sizeof(StlRenderVertex))));
    m_stlModelVbo.release();
}

PointCloudCrossSection::Camera PointCloudView::crossSectionCamera() const
{
    return PointCloudCrossSection::Camera{m_modelView, m_projection, QSize(width(), height())};
}

void PointCloudView::requestCrossSectionClip(bool immediate)
{
    if (!m_crossSectionState.enabled || !m_crossSectionState.initialized) {
        return;
    }
    if (immediate) {
        if (m_crossSectionClipDebounceTimer) {
            m_crossSectionClipDebounceTimer->stop();
        }
        if (m_crossSectionClipRunning) {
            m_crossSectionClipPending = true;
            return;
        }
        ++m_crossSectionClipGeneration;
        startCrossSectionClipJob();
    } else if (m_crossSectionClipRunning) {
        m_crossSectionClipPending = true;
    } else {
        ++m_crossSectionClipGeneration;
        startCrossSectionClipJob();
    }
}

void PointCloudView::startCrossSectionClipJob()
{
    if (!m_crossSectionState.enabled || !m_crossSectionState.initialized) {
        return;
    }
    if (m_crossSectionClipRunning) {
        m_crossSectionClipPending = true;
        return;
    }

    QVector<SegmentPointSnapshot> snapshots;
    const quint64 generation = m_crossSectionClipGeneration;
    const PointCloudCrossSection::State state = m_crossSectionState;
    {
        QMutexLocker locker(&m_pointsMutex);
        snapshots.reserve(int(m_pointCloudSegments.size()));
        for (const std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
            if (!segment) {
                continue;
            }
            snapshots.push_back(SegmentPointSnapshot{segment->id, false, segment->points});
        }
    }

    if (snapshots.isEmpty()) {
        if (state.sourcePoints.isEmpty() &&
            !m_slamRenderSnapshot.worldFrameVertices.isEmpty()) {
            m_slamRenderUploadPending = true;
            emit crossSectionChanged(
                clippedSlamWorldFramePointCount(m_slamRenderSnapshot, state),
                m_slamRenderSnapshot.worldFrameVertices.size());
            update();
            return;
        }
        if (state.sourcePoints.isEmpty()) {
            PointCloudCrossSection::updateClip(m_crossSectionState);
            uploadPointCloudPoints(QVector<PointCloudPoint>(m_crossSectionState.clippedPoints));
            emit crossSectionChanged(m_crossSectionState.clippedPoints.size(), m_crossSectionState.sourcePoints.size());
            return;
        }
        snapshots.push_back(SegmentPointSnapshot{0, true, state.sourcePoints});
    }

    m_crossSectionClipRunning = true;
    m_crossSectionClipPending = false;
    m_crossSectionClipRemaining = snapshots.size();
    const QPointer<PointCloudView> guard(this);
    for (const SegmentPointSnapshot& snapshot : snapshots) {
        QtConcurrent::run([guard, generation, state, snapshot]() mutable {
            ClipSegmentResult segmentResult{
                snapshot.segmentId,
                snapshot.singleBuffer,
                PointCloudCrossSection::clip(snapshot.points, state)
            };
            if (!guard) {
                return;
            }
            QMetaObject::invokeMethod(guard.data(), [guard, generation, segmentResult = std::move(segmentResult)]() mutable {
                if (!guard) {
                    return;
                }

                PointCloudView* view = guard.data();
                const bool stale = generation != view->m_crossSectionClipGeneration ||
                                   !view->m_crossSectionState.enabled ||
                                   !view->m_crossSectionState.initialized;

                QVector<PointCloudPoint> selectionSnapshot;
                quint64 appliedSegmentId = 0;
                if (!stale) {
                    if (segmentResult.singleBuffer) {
                        QVector<PointCloudPoint> clippedPoints = std::move(segmentResult.clippedPoints);
                        view->m_crossSectionState.clippedPoints = clippedPoints;
                        view->uploadPointCloudPoints(std::move(clippedPoints));
                    } else {
                        QMutexLocker locker(&view->m_pointsMutex);
                        for (std::unique_ptr<PointCloudSegment>& segment : view->m_pointCloudSegments) {
                            if (!segment || segment->id != segmentResult.segmentId) {
                                continue;
                            }
                            segment->clippedPoints = std::move(segmentResult.clippedPoints);
                            selectionSnapshot = segment->clippedPoints;
                            appliedSegmentId = segment->id;
                            view->uploadPointCloudSegmentClip(*segment);
                            break;
                        }
                        view->m_crossSectionState.sourcePoints.clear();
                        view->m_crossSectionState.clippedPoints.clear();
                    }
                    view->update();
                    if (appliedSegmentId != 0 && view->m_selectionLocked && view->m_selectionRegion.valid) {
                        view->startSelectionSegmentJob(appliedSegmentId, selectionSnapshot);
                    }
                }

                --view->m_crossSectionClipRemaining;
                if (view->m_crossSectionClipRemaining > 0) {
                    return;
                }

                int clippedPointCount = 0;
                int sourcePointCount = 0;
                if (!view->m_pointCloudSegments.empty()) {
                    QMutexLocker locker(&view->m_pointsMutex);
                    for (const std::unique_ptr<PointCloudSegment>& segment : view->m_pointCloudSegments) {
                        if (!segment) {
                            continue;
                        }
                        sourcePointCount += segment->points.size();
                        clippedPointCount += segment->clippedPoints.size();
                    }
                } else {
                    sourcePointCount = view->m_crossSectionState.sourcePoints.size();
                    clippedPointCount = view->m_crossSectionState.clippedPoints.size();
                }

                const bool restart = view->m_crossSectionClipPending ||
                                     generation != view->m_crossSectionClipGeneration;
                view->m_crossSectionClipRunning = false;
                view->m_crossSectionClipPending = false;
                emit view->crossSectionChanged(clippedPointCount, sourcePointCount);
                if (view->m_selectionLocked && view->m_selectionRegion.valid && !restart) {
                    view->requestSelectionUpdate();
                }
                if (restart && view->m_crossSectionState.enabled && view->m_crossSectionState.initialized) {
                    view->startCrossSectionClipJob();
                }
            }, Qt::QueuedConnection);
        });
    }
}

void PointCloudView::startCrossSectionSegmentClipJob(quint64 segmentId, const QVector<PointCloudPoint>& points)
{
    if (!m_crossSectionState.enabled || !m_crossSectionState.initialized || segmentId == 0) {
        return;
    }
    const quint64 generation = m_crossSectionClipGeneration;
    const PointCloudCrossSection::State state = m_crossSectionState;
    const QPointer<PointCloudView> guard(this);
    QtConcurrent::run([guard, generation, segmentId, state, points]() mutable {
        ClipJobResult result;
        result.generation = generation;
        result.fullWindow = false;
        result.segments.push_back(ClipSegmentResult{segmentId, false, PointCloudCrossSection::clip(points, state)});
        if (!guard) {
            return;
        }
        QMetaObject::invokeMethod(guard.data(), [guard, result = std::move(result)]() mutable {
            if (!guard) {
                return;
            }
            PointCloudView* view = guard.data();
            if (result.generation != view->m_crossSectionClipGeneration ||
                !view->m_crossSectionState.enabled ||
                !view->m_crossSectionState.initialized ||
                result.segments.isEmpty()) {
                return;
            }

            QVector<PointCloudPoint> selectionSnapshot;
            quint64 appliedSegmentId = 0;
            int clippedPointCount = 0;
            int sourcePointCount = 0;
            {
                QMutexLocker locker(&view->m_pointsMutex);
                const ClipSegmentResult& segmentResult = result.segments.first();
                for (std::unique_ptr<PointCloudSegment>& segment : view->m_pointCloudSegments) {
                    if (!segment || segment->id != segmentResult.segmentId) {
                        continue;
                    }
                    segment->clippedPoints = segmentResult.clippedPoints;
                    selectionSnapshot = segment->clippedPoints;
                    appliedSegmentId = segment->id;
                    view->uploadPointCloudSegmentClip(*segment);
                    break;
                }
                for (const std::unique_ptr<PointCloudSegment>& segment : view->m_pointCloudSegments) {
                    if (!segment) {
                        continue;
                    }
                    sourcePointCount += segment->points.size();
                    clippedPointCount += segment->clippedPoints.size();
                }
            }
            emit view->crossSectionChanged(clippedPointCount, sourcePointCount);
            view->update();
            if (appliedSegmentId != 0 && view->m_selectionLocked && view->m_selectionRegion.valid) {
                view->startSelectionSegmentJob(appliedSegmentId, selectionSnapshot);
            }
        }, Qt::QueuedConnection);
    });
}

void PointCloudView::updateCrossSectionPointCloud()
{
    requestCrossSectionClip(true);
}

void PointCloudView::updateSelectionRegionFromRect(const QRect& rect)
{
    if (rect.isEmpty()) {
        m_selectionLocked = false;
        m_selectionRegion = SelectionRegion();
        requestSelectionUpdate();
        return;
    }

    m_selModelView = m_modelView;
    m_selProjection = m_projection;
    m_selViewportW = width();
    m_selViewportH = height();
    m_selRectLogical = rect;
    m_selViewZMin = -std::numeric_limits<float>::max();
    m_selViewZMax = std::numeric_limits<float>::max();
    m_selectionRegion.valid = true;
    m_selectionRegion.mvp = m_selProjection * m_selModelView;
    m_selectionRegion.modelView = m_selModelView;
    m_selectionRegion.rect = rect;
    m_selectionRegion.viewportW = m_selViewportW;
    m_selectionRegion.viewportH = m_selViewportH;
    m_selectionLocked = true;
    requestSelectionUpdate();
}

void PointCloudView::requestSelectionUpdate()
{
    if (!m_selectionLocked || !m_selectionRegion.valid) {
        ++m_selectionGeneration;
        {
            QMutexLocker locker(&m_pointsMutex);
            clearSelectionCaches();
        }
        emit selectionPointsReady({}, 0);
        update();
        return;
    }

    if (m_selectionJobRunning) {
        m_selectionJobPending = true;
        return;
    }

    ++m_selectionGeneration;
    startSelectionJob();
}

void PointCloudView::startSelectionJob()
{
    if (!m_selectionLocked || !m_selectionRegion.valid) {
        return;
    }
    if (m_selectionJobRunning) {
        m_selectionJobPending = true;
        return;
    }

    QVector<SegmentPointSnapshot> snapshots;
    const quint64 generation = m_selectionGeneration;
    const SelectionRegion region = m_selectionRegion;
    {
        QMutexLocker locker(&m_pointsMutex);
        if (!m_points.isEmpty()) {
            snapshots.push_back(SegmentPointSnapshot{0, true, m_points});
        }
        snapshots.reserve(snapshots.size() + int(m_pointCloudSegments.size()));
        for (const std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
            if (!segment) {
                continue;
            }
            const QVector<PointCloudPoint>& displayedPoints =
                (m_crossSectionState.enabled && m_crossSectionState.initialized)
                    ? segment->clippedPoints
                    : segment->points;
            snapshots.push_back(SegmentPointSnapshot{segment->id, false, displayedPoints});
        }
    }

    if (snapshots.isEmpty()) {
        {
            QMutexLocker locker(&m_pointsMutex);
            clearSelectionCaches();
        }
        emit selectionPointsReady({}, 0);
        update();
        return;
    }

    m_selectionJobRunning = true;
    m_selectionJobPending = false;
    m_selectionJobRemaining = snapshots.size();
    const QPointer<PointCloudView> guard(this);
    for (const SegmentPointSnapshot& snapshot : snapshots) {
        QtConcurrent::run([guard, generation, region, snapshot]() mutable {
            SelectionSegmentResult result{
                snapshot.segmentId,
                snapshot.singleBuffer,
                selectPointsInRegion(snapshot.points, region)
            };
            if (!guard) {
                return;
            }
            QMetaObject::invokeMethod(guard.data(), [guard, generation, result = std::move(result)]() mutable {
                if (!guard) {
                    return;
                }
                PointCloudView* view = guard.data();
                const bool stale = generation != view->m_selectionGeneration ||
                                   !view->m_selectionLocked ||
                                   !view->m_selectionRegion.valid;
                if (!stale) {
                    {
                        QMutexLocker locker(&view->m_pointsMutex);
                        if (result.singleBuffer) {
                            view->uploadSelectionPoints(std::move(result.selectedPoints));
                        } else {
                            for (std::unique_ptr<PointCloudSegment>& segment : view->m_pointCloudSegments) {
                                if (!segment || segment->id != result.segmentId) {
                                    continue;
                                }
                                segment->selectedPoints = std::move(result.selectedPoints);
                                view->uploadPointCloudSegmentSelection(*segment);
                                break;
                            }
                        }
                    }
                    view->update();
                }

                --view->m_selectionJobRemaining;
                if (view->m_selectionJobRemaining > 0) {
                    return;
                }

                const bool restart = view->m_selectionJobPending ||
                                     generation != view->m_selectionGeneration;
                view->m_selectionJobRunning = false;
                view->m_selectionJobPending = false;
                if (generation == view->m_selectionGeneration &&
                    view->m_selectionLocked &&
                    view->m_selectionRegion.valid) {
                    view->requestSelectionPublish();
                }
                if (restart && view->m_selectionLocked && view->m_selectionRegion.valid) {
                    view->startSelectionJob();
                }
            }, Qt::QueuedConnection);
        });
    }
}

void PointCloudView::startSelectionSegmentJob(quint64 segmentId, const QVector<PointCloudPoint>& points)
{
    if (!m_selectionLocked || !m_selectionRegion.valid || segmentId == 0) {
        return;
    }
    const quint64 generation = m_selectionGeneration;
    const SelectionRegion region = m_selectionRegion;
    const QPointer<PointCloudView> guard(this);
    QtConcurrent::run([guard, generation, segmentId, region, points]() mutable {
        SelectionSegmentResult result{segmentId, false, selectPointsInRegion(points, region)};
        if (!guard) {
            return;
        }
        QMetaObject::invokeMethod(guard.data(), [guard, generation, result = std::move(result)]() mutable {
            if (!guard) {
                return;
            }
            PointCloudView* view = guard.data();
            if (generation != view->m_selectionGeneration ||
                !view->m_selectionLocked ||
                !view->m_selectionRegion.valid) {
                return;
            }
            {
                QMutexLocker locker(&view->m_pointsMutex);
                for (std::unique_ptr<PointCloudSegment>& segment : view->m_pointCloudSegments) {
                    if (!segment || segment->id != result.segmentId) {
                        continue;
                    }
                    segment->selectedPoints = std::move(result.selectedPoints);
                    view->uploadPointCloudSegmentSelection(*segment);
                    break;
                }
            }
            view->requestSelectionPublish();
            view->update();
        }, Qt::QueuedConnection);
    });
}

void PointCloudView::requestSelectionPublish()
{
    if (!m_selectionLocked || !m_selectionRegion.valid) {
        return;
    }
    if (m_selectionPublishRunning) {
        m_selectionPublishPending = true;
        return;
    }
    startSelectionPublishJob();
}

void PointCloudView::startSelectionPublishJob()
{
    if (!m_selectionLocked || !m_selectionRegion.valid) {
        return;
    }
    if (m_selectionPublishRunning) {
        m_selectionPublishPending = true;
        return;
    }

    QVector<QVector<PointCloudPoint>> selectedSnapshots;
    const quint64 generation = m_selectionGeneration;
    {
        QMutexLocker locker(&m_pointsMutex);
        if (!m_selectedPoints.isEmpty()) {
            selectedSnapshots.push_back(m_selectedPoints);
        }
        selectedSnapshots.reserve(selectedSnapshots.size() + int(m_pointCloudSegments.size()));
        for (const std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
            if (segment && !segment->selectedPoints.isEmpty()) {
                selectedSnapshots.push_back(segment->selectedPoints);
            }
        }
    }

    m_selectionPublishRunning = true;
    m_selectionPublishPending = false;
    const QPointer<PointCloudView> guard(this);
    QtConcurrent::run([guard, generation, selectedSnapshots = std::move(selectedSnapshots)]() mutable {
        SelectionPublishResult result;
        result.generation = generation;
        qsizetype totalCount = 0;
        for (const QVector<PointCloudPoint>& selectedPoints : selectedSnapshots) {
            totalCount += selectedPoints.size();
        }
        result.points.reserve(totalCount);
        for (const QVector<PointCloudPoint>& selectedPoints : selectedSnapshots) {
            result.points += selectedPoints;
        }
        result.zeroPointCount = zeroPointCount(result.points);
        if (!guard) {
            return;
        }
        QMetaObject::invokeMethod(guard.data(), [guard, result = std::move(result)]() mutable {
            if (!guard) {
                return;
            }
            PointCloudView* view = guard.data();
            const bool stale = result.generation != view->m_selectionGeneration ||
                               !view->m_selectionLocked ||
                               !view->m_selectionRegion.valid;
            if (!stale) {
                emit view->selectionPointsReady(std::move(result.points), result.zeroPointCount);
            }

            const bool restart = view->m_selectionPublishPending ||
                                 result.generation != view->m_selectionGeneration;
            view->m_selectionPublishRunning = false;
            view->m_selectionPublishPending = false;
            if (restart && view->m_selectionLocked && view->m_selectionRegion.valid) {
                view->startSelectionPublishJob();
            }
        }, Qt::QueuedConnection);
    });
}

void PointCloudView::updatePointCloud(const PointCloudFrame& frame)
{
    if (m_crossSectionState.enabled) {
        m_crossSectionState.sourcePoints = frame.points;
        m_crossSectionState.clippedPoints.clear();
        requestCrossSectionClip(true);
        return;
    }
    uploadPointCloudPoints(QVector<PointCloudPoint>(frame.points));
    if (m_selectionLocked && m_selectionRegion.valid) {
        requestSelectionUpdate();
    }
}

void PointCloudView::updatePointCloud(PointCloudFrame&& frame)
{
    if (m_crossSectionState.enabled) {
        m_crossSectionState.sourcePoints = std::move(frame.points);
        m_crossSectionState.clippedPoints.clear();
        requestCrossSectionClip(true);
        return;
    }
    uploadPointCloudPoints(std::move(frame.points));
    if (m_selectionLocked && m_selectionRegion.valid) {
        requestSelectionUpdate();
    }
}

void PointCloudView::recolorCurrentPointCloud(const std::function<void(QVector<PointCloudPoint>&)>& colorize)
{
    if (m_crossSectionState.enabled && pointCloudSegmentCount() > 0) {
        {
            QMutexLocker locker(&m_pointsMutex);
            for (std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
                if (!segment) {
                    continue;
                }
                colorize(segment->points);
                m_pointCloudGpuUploadPending = true;
                segment->clippedPoints.clear();
                segment->clippedPointCount = 0;
                clearSegmentSelectionBuffers(*segment);
            }
        }
        requestCrossSectionClip(true);
        update();
        return;
    }

    if (m_crossSectionState.enabled) {
        colorize(m_crossSectionState.sourcePoints);
        m_crossSectionState.clippedPoints.clear();
        requestCrossSectionClip(true);
        return;
    }

    if (pointCloudSegmentCount() > 0) {
        const bool selectionNeedsUpdate = m_selectionLocked && m_selectionRegion.valid;
        {
            QMutexLocker locker(&m_pointsMutex);
            for (std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
                if (!segment) {
                    continue;
                }
                colorize(segment->points);
                m_pointCloudGpuUploadPending = true;
                if (selectionNeedsUpdate) {
                    segment->selectedPoints.clear();
                    segment->selectedPointCount = 0;
                }
            }
        }
        if (selectionNeedsUpdate) {
            requestSelectionUpdate();
        }
        update();
        return;
    }

    QVector<PointCloudPoint> points;
    {
        QMutexLocker locker(&m_pointsMutex);
        points = m_points;
    }
    colorize(points);
    uploadPointCloudPoints(std::move(points));
    if (m_selectionLocked && m_selectionRegion.valid) {
        requestSelectionUpdate();
    }
}

void PointCloudView::clearPointCloud()
{
    uploadPointCloudPoints(QVector<PointCloudPoint>());
    m_crossSectionState = PointCloudCrossSection::State();
    m_selectionLocked = false;
    m_selectionRegion = SelectionRegion();
    ++m_crossSectionClipGeneration;
    ++m_selectionGeneration;
    emit selectionPointsReady({}, 0);
    printf("PointCloudView: cleared all points\n");
}

void PointCloudView::resetView()
{
    m_orientation = QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), -45.0f)
                * QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), 90.0f);
    m_target = QVector3D(0, 0, 0);
    m_distance = kDefaultCameraDistance;
    syncViewerPositionFromOrbit();
    update();
}

void PointCloudView::setPointSize(float sizePixels)
{
    m_pointSize = qBound(1.0f, sizePixels, 10.0f);
    update();
}

void PointCloudView::setBackgroundColors(const QColor& topColor, const QColor& bottomColor)
{
    m_backgroundTopColor = topColor;
    m_backgroundBottomColor = bottomColor;
    update();
}

void PointCloudView::setProjectionMode(ProjectionMode mode)
{
    if (m_projectionMode == mode) {
        return;
    }
    if (mode == ProjectionMode::ObserverPerspective) {
        syncViewerPositionFromOrbit();
    } else if (m_projectionMode == ProjectionMode::ObserverPerspective) {
        syncOrbitTargetFromViewer();
    }
    m_projectionMode = mode;
    update();
}

void PointCloudView::setLegend(int mode,
                               float minVal,
                               float maxVal,
                               bool visible,
                               const QVector<QColor>& lineColors,
                               const QVector<int>& lineNumbers,
                               const QVector<QColor>& gradientColors)
{
    m_legendMode = mode;
    m_legendMin = minVal;
    m_legendMax = maxVal;
    m_legendVisible = visible;
    m_lineLegendColors = lineColors;
    m_lineLegendNumbers = lineNumbers;
    m_legendGradientColors = gradientColors;
    update();
}

void PointCloudView::setTopDownView()
{
    setViewPreset(ViewPreset::Top);
}

void PointCloudView::setViewPreset(ViewPreset preset)
{
    const QQuaternion qFront = QQuaternion::fromAxisAndAngle(QVector3D(0, 1, 0), -90.0f)  // 再绕Y轴-90°
                         * QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), -90.0f); // 先绕X轴-90°
    const QQuaternion qBack = QQuaternion::fromAxisAndAngle(QVector3D(0, 1, 0), 90.0f)  // 再绕Y轴90°
                         * QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), -90.0f); // 先绕X轴-90°
    const QQuaternion qLeft = QQuaternion::fromAxisAndAngle(QVector3D(0, 1, 0), -180.0f)  // 再绕Y轴-180°
                         * QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), -90.0f); // 先绕X轴-90°
    const QQuaternion qRight = QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), -90.0f); // 绕X轴-90°
    const QQuaternion qTop = QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), 90.0f); // 绕Z轴90°
    const QQuaternion qWorld = QQuaternion();

    switch (preset) {
    case ViewPreset::World:
        m_orientation = qWorld;
        break;
    case ViewPreset::Front:
        m_orientation = qFront;
        break;
    case ViewPreset::Back:
        m_orientation = qBack;
        break;
    case ViewPreset::Left:
        m_orientation = qLeft;
        break;
    case ViewPreset::Right:
        m_orientation = qRight;
        break;
    case ViewPreset::Top:
        m_orientation = qTop;
        break;
    }
    m_target = QVector3D(0, 0, 0);
    m_distance = kDefaultCameraDistance;
    syncViewerPositionFromOrbit();
    update();
}

void PointCloudView::setSelectionModeEnabled(bool enabled)
{
    m_selectionModeEnabled = enabled;
    if (!enabled) {
        m_selecting = false;
        m_selStart = QPoint();
        m_selEnd = QPoint();
        m_selectionLocked = false;
        m_selRectLogical = QRect();
        m_selViewportW = 0;
        m_selViewportH = 0;
        m_selViewZMin = 0.0f;
        m_selViewZMax = 0.0f;
        m_selectionRegion = SelectionRegion();
        ++m_selectionGeneration;
        {
            QMutexLocker locker(&m_pointsMutex);
            clearSelectionCaches();
        }
        emit selectionPointsReady({}, 0);
        update();
    }
}

void PointCloudView::setCrossSectionModeEnabled(bool enabled)
{
    if (m_crossSectionState.enabled == enabled) {
        return;
    }

    ++m_crossSectionClipGeneration;
    if (m_crossSectionClipDebounceTimer) {
        m_crossSectionClipDebounceTimer->stop();
    }

    if (enabled) {
        if (pointCloudSegmentCount() > 0) {
            QVector3D minPoint;
            QVector3D maxPoint;
            if (!pointCloudSegmentSourceBounds(minPoint, maxPoint)) {
                m_crossSectionState.enabled = false;
                emit crossSectionChanged(0, 0);
                return;
            }
            PointCloudCrossSection::initializeBoxFromBounds(m_crossSectionState, minPoint, maxPoint);
            updateCrossSectionPointCloud();
            return;
        }

        QVector<PointCloudPoint> sourcePoints = currentPoints();
        if (sourcePoints.isEmpty()) {
            QVector3D minPoint;
            QVector3D maxPoint;
            if (!slamWorldFrameBounds(m_slamRenderSnapshot, minPoint, maxPoint)) {
                m_crossSectionState.enabled = false;
                emit crossSectionChanged(0, 0);
                return;
            }
            PointCloudCrossSection::initializeBoxFromBounds(m_crossSectionState, minPoint, maxPoint);
            m_slamRenderUploadPending = true;
            emit crossSectionChanged(m_slamRenderSnapshot.worldFrameVertices.size(),
                                     m_slamRenderSnapshot.worldFrameVertices.size());
            update();
            return;
        }
        if (!PointCloudCrossSection::initializeBox(m_crossSectionState, sourcePoints)) {
            m_crossSectionState.enabled = false;
            emit crossSectionChanged(0, 0);
            return;
        }
        uploadPointCloudPoints(QVector<PointCloudPoint>(m_crossSectionState.clippedPoints));
        emit crossSectionChanged(m_crossSectionState.clippedPoints.size(), m_crossSectionState.sourcePoints.size());
        update();
        return;
    }

    if (pointCloudSegmentCount() > 0) {
        int sourcePointCount = 0;
        {
            QMutexLocker locker(&m_pointsMutex);
            ScopedOpenGLContext current(this);
            for (std::unique_ptr<PointCloudSegment>& segment : m_pointCloudSegments) {
                if (!segment) {
                    continue;
                }
                sourcePointCount += segment->points.size();
                segment->clippedPoints.clear();
                segment->clippedPointCount = 0;
                segment->clippedBufferCapacityBytes = 0;
                if (segment->clippedVbo.isCreated()) {
                    segment->clippedVbo.destroy();
                }
                if (segment->clippedVao.isCreated()) {
                    segment->clippedVao.destroy();
                }
            }
        }
        m_crossSectionState = PointCloudCrossSection::State();
        emit crossSectionChanged(sourcePointCount, sourcePointCount);
        if (m_selectionLocked && m_selectionRegion.valid) {
            requestSelectionUpdate();
        }
        update();
        return;
    }

    if (m_crossSectionState.sourcePoints.isEmpty() &&
        !m_slamRenderSnapshot.worldFrameVertices.isEmpty()) {
        const int pointCount = m_slamRenderSnapshot.worldFrameVertices.size();
        m_crossSectionState = PointCloudCrossSection::State();
        m_slamRenderUploadPending = true;
        emit crossSectionChanged(pointCount, pointCount);
        update();
        return;
    }

    QVector<PointCloudPoint> restored = std::move(m_crossSectionState.sourcePoints);
    const int restoredCount = restored.size();
    m_crossSectionState = PointCloudCrossSection::State();
    uploadPointCloudPoints(std::move(restored));
    emit crossSectionChanged(restoredCount, restoredCount);
    if (m_selectionLocked && m_selectionRegion.valid) {
        requestSelectionUpdate();
    }
}

void PointCloudView::resetCrossSectionBoxToCurrentCloud()
{
    if (!m_crossSectionState.enabled) {
        return;
    }
    if (pointCloudSegmentCount() > 0) {
        QVector3D minPoint;
        QVector3D maxPoint;
        if (!pointCloudSegmentSourceBounds(minPoint, maxPoint)) {
            emit crossSectionChanged(0, 0);
            return;
        }
        PointCloudCrossSection::initializeBoxFromBounds(m_crossSectionState, minPoint, maxPoint);
        updateCrossSectionPointCloud();
        return;
    }
    if (m_crossSectionState.sourcePoints.isEmpty()) {
        QVector3D minPoint;
        QVector3D maxPoint;
        if (!slamWorldFrameBounds(m_slamRenderSnapshot, minPoint, maxPoint)) {
            emit crossSectionChanged(0, 0);
            return;
        }
        PointCloudCrossSection::initializeBoxFromBounds(m_crossSectionState, minPoint, maxPoint);
        m_slamRenderUploadPending = true;
        emit crossSectionChanged(m_slamRenderSnapshot.worldFrameVertices.size(),
                                 m_slamRenderSnapshot.worldFrameVertices.size());
        update();
        return;
    }
    PointCloudCrossSection::initializeBox(m_crossSectionState, m_crossSectionState.sourcePoints);
    updateCrossSectionPointCloud();
}

void PointCloudView::setCrossSectionControlsVisible(bool visible)
{
    PointCloudCrossSection::setControlsVisible(m_crossSectionState, visible);
    update();
}

void PointCloudView::setStlModelMesh(const StlModel::Mesh& mesh, bool sourceXReversed)
{
    m_stlModelVertices = transformDeviceModelVertices(mesh.triangles, sourceXReversed);
    m_stlModelVisible = true;
    uploadStlModelVertices();
    update();
}

void PointCloudView::setStlModelVisible(bool visible)
{
    m_stlModelVisible = visible && !m_stlModelVertices.isEmpty();
    update();
}

QVector<PointCloudPoint> PointCloudView::pointsInRect(const QRect& rect, int maxPoints)
{
    QVector<PointCloudPoint> result;
    if (rect.isEmpty()) return result;
    QMatrix4x4 mvp = m_projection * m_modelView;
    result.reserve(std::min(maxPoints, 4096));
    forEachDisplayedPoint([&](const PointCloudPoint& p) {
        QVector4D hp(p.x, p.y, p.z, 1.0f);
        ProjectedScreenPoint projected;
        if (!projectVisiblePointToScreen(hp, mvp, width(), height(), projected)) {
            return true;
        }
        if (rect.contains(QPoint(int(projected.x), int(projected.y)))) {
            result.push_back(p);
            if (result.size() >= maxPoints) {
                return false;
            }
        }
        return true;
    });
    return result;
}

QVector<PointCloudPoint> PointCloudView::pointsInAabb(const QVector3D& min, const QVector3D& max, int maxPoints)
{
    QVector<PointCloudPoint> result;
    result.reserve(std::min(maxPoints, 4096));
    forEachDisplayedPoint([&](const PointCloudPoint& p) {
        if (p.x >= min.x() && p.x <= max.x() &&
            p.y >= min.y() && p.y <= max.y() &&
            p.z >= min.z() && p.z <= max.z()) {
            result.push_back(p);
            if (result.size() >= maxPoints) {
                return false;
            }
        }
        return true;
    });
    return result;
}

QVector<PointCloudPoint> PointCloudView::pointsInPersistSelection(int maxPoints)
{
    QVector<PointCloudPoint> result;
    if (!m_selectionLocked) return result;
    QMatrix4x4 mvp = m_selProjection * m_selModelView;
    result.reserve(std::min(maxPoints, 4096));
    forEachDisplayedPoint([&](const PointCloudPoint& p) {
        QVector4D hp(p.x, p.y, p.z, 1.0f);
        ProjectedScreenPoint projected;
        if (!projectVisiblePointToScreen(hp, mvp, m_selViewportW, m_selViewportH, projected)) {
            return true;
        }
        float vz = (m_selModelView * hp).z();
        if (projected.x >= m_selRectLogical.left() && projected.x <= m_selRectLogical.right() &&
            projected.y >= m_selRectLogical.top() && projected.y <= m_selRectLogical.bottom() &&
            vz >= m_selViewZMin && vz <= m_selViewZMax) {
            result.push_back(p);
            if (result.size() >= maxPoints) {
                return false;
            }
        }
        return true;
    });
    return result;
} 

#include "mainwindow.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <QPainter>
#include <QLinearGradient>
#include <QOpenGLFunctions>

// PointCloudWidget 实现
PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QOpenGLWidget(parent)
    , m_program(nullptr)
    , m_distance(10.0f)
    , m_rotation(0, 0, 0)
    , m_orientation() // identity
    , m_panOffset(0, 0, 0)
    , m_activeButton(Qt::NoButton)
    , m_mousePressed(false)
    , m_pointSize(2.0f)
    , m_selecting(false)
    , m_selStart(QPoint())
    , m_selEnd(QPoint())
    , m_selectionModeEnabled(false)
{
    setFocusPolicy(Qt::StrongFocus);
    // 默认视角：X 向上、Y 向左、Z 向外（绕 Z 轴 +90°）
    m_orientation = QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), 90.0f);
}

PointCloudWidget::~PointCloudWidget()
{
    if (m_program) {
        delete m_program;
    }
}

// 选点：在屏幕区域内找最近点（优先屏幕距离，其次视空间深度）
bool PointCloudWidget::pickNearestPoint(const QPoint& pos, QVector3D& outWorld, QPoint& outScreen, int pixelRadius)
{
    QMatrix4x4 mvp = m_projection * m_modelView;
    QMutexLocker locker(&m_pointsMutex);
    float dpr = devicePixelRatioF();
    int effectiveRadius = std::max(pixelRadius, int(std::round((m_pointSize / std::max(1.0f, dpr)) * 1.8f)));
    float radiusSq = float(effectiveRadius * effectiveRadius);
    float bestDistSq = std::numeric_limits<float>::max();
    float bestZ = std::numeric_limits<float>::max();
    bool found = false;
    for (const Point3D& p : m_points) {
        QVector4D hp(p.x, p.y, p.z, 1.0f);
        QVector4D clip = mvp * hp;
        if (clip.w() == 0.0f) continue;
        QVector3D ndc = clip.toVector3DAffine();
        float sx = (ndc.x() * 0.5f + 0.5f) * width();
        float sy = (1.0f - (ndc.y() * 0.5f + 0.5f)) * height();
        float dx = sx - pos.x();
        float dy = sy - pos.y();
        float distSq = dx*dx + dy*dy;
        if (distSq <= radiusSq) {
            float vz = (m_modelView * hp).z();
            if (distSq < bestDistSq || (std::abs(distSq - bestDistSq) < 1e-3f && vz < bestZ)) {
                bestDistSq = distSq;
                bestZ = vz;
                outWorld = QVector3D(p.x, p.y, p.z);
                outScreen = QPoint(int(std::round(sx)), int(std::round(sy)));
                found = true;
            }
        }
    }
    return found;
}

void PointCloudWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(2.0f);
    
    setupShaders();
    setupBuffers();
    setupAxesBuffers();
}

void PointCloudWidget::setupShaders()
{
    const char *vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 position;
        layout (location = 1) in vec3 color;
        
        uniform mat4 modelView;
        uniform mat4 projection;
        uniform float uPointSize;
        
        out vec3 fragColor;
        out vec3 vWorld;
        
        void main()
        {
            vWorld = position;
            gl_Position = projection * modelView * vec4(position, 1.0);
            gl_PointSize = uPointSize;
            fragColor = color;
        }
    )";

    const char *fragmentShaderSource = R"(
        #version 330 core
        in vec3 fragColor;
        in vec3 vWorld;
        out vec4 outColor;
        
        uniform int uSelectionEnabled;
        uniform vec4 uSelRect; // x0, y0, x1, y1 (pixels)
        uniform int uPersistEnabled;
        uniform vec4 uPersistRect; // selection-time rect in logical pixels
        uniform mat4 uSelModelView;
        uniform mat4 uSelProjection;
        uniform vec2 uViewport; // width, height at selection time (logical)
        uniform vec2 uDepthRange; // minZ, maxZ in view space at selection time
        
        float viewZ(mat4 mv, vec3 world) {
            vec4 v = mv * vec4(world, 1.0);
            return v.z; // right-handed view space, negative forward
        }
        
        void main()
        {
            vec4 base = vec4(fragColor, 1.0);
            bool selected = false;
            if (uPersistEnabled == 1) {
                // project with selection-time matrices
                vec4 clip = uSelProjection * (uSelModelView * vec4(vWorld, 1.0));
                if (clip.w != 0.0) {
                    vec3 ndc = (clip.xyz / clip.w);
                    float sx = (ndc.x * 0.5 + 0.5) * uViewport.x;
                    float sy = (1.0 - (ndc.y * 0.5 + 0.5)) * uViewport.y;
                    float vz = viewZ(uSelModelView, vWorld);
                    if (sx >= uPersistRect.x && sx <= uPersistRect.z && sy >= uPersistRect.y && sy <= uPersistRect.w && vz >= uDepthRange.x && vz <= uDepthRange.y) {
                        selected = true;
                    }
                }
            }
            if (uSelectionEnabled == 1) {
                float x = gl_FragCoord.x;
                float y = gl_FragCoord.y;
                if (x >= uSelRect.x && x <= uSelRect.z && y >= uSelRect.y && y <= uSelRect.w) {
                    selected = true;
                }
            }
            outColor = selected ? vec4(1.0, 0.0, 0.0, 1.0) : base;
        }
    )";

    m_program = new QOpenGLShaderProgram();
    m_program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    m_program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    m_program->link();
}

void PointCloudWidget::setupBuffers()
{
    m_vao.create();
    m_vao.bind();
    
    m_vbo.create();
    m_vbo.bind();
    m_vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_vbo.allocate(1000 * sizeof(Point3D)); // 初始分配空间
    
    // 设置顶点属性
    m_program->enableAttributeArray(0);
    m_program->setAttributeBuffer(0, GL_FLOAT, offsetof(Point3D, x), 3, sizeof(Point3D));
    
    m_program->enableAttributeArray(1);
    m_program->setAttributeBuffer(1, GL_FLOAT, offsetof(Point3D, r), 3, sizeof(Point3D));
    
    m_vao.release();
}

void PointCloudWidget::setupAxesBuffers()
{
    struct AxisVertex { float x, y, z, r, g, b; };
    AxisVertex axes[6] = {
        {0.f, 0.f, 0.f, 1.f, 0.f, 0.f}, {1.0f, 0.f, 0.f, 1.f, 0.f, 0.f}, // X
        {0.f, 0.f, 0.f, 0.f, 1.f, 0.f}, {0.f, 1.0f, 0.f, 0.f, 1.f, 0.f}, // Y
        {0.f, 0.f, 0.f, 0.f, 0.f, 1.f}, {0.f, 0.f, 1.0f, 0.f, 0.f, 1.f}, // Z
    };

    m_axesVao.create();
    m_axesVao.bind();

    m_axesVbo.create();
    m_axesVbo.bind();
    m_axesVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_axesVbo.allocate(axes, sizeof(axes));

    m_program->enableAttributeArray(0);
    m_program->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(AxisVertex));
    m_program->enableAttributeArray(1);
    m_program->setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 3, sizeof(AxisVertex));

    m_axesVao.release();
    m_axesVbo.release();
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

QVector3D PointCloudWidget::mapToArcball(const QPoint& p) const
{
    float x = (2.0f * p.x() - width()) / qMax(1, width());
    float y = (height() - 2.0f * p.y()) / qMax(1, height());
    float z2 = 1.0f - x * x - y * y;
    float z = z2 > 0.0f ? std::sqrt(z2) : 0.0f;
    return QVector3D(x, y, z).normalized();
}

void PointCloudWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    if (!m_program) {
        return;
    }
    
    m_program->bind();
    
    // 设置变换矩阵：平移 -> 缩放距离 -> 旋转(四元数)
    m_modelView.setToIdentity();
    m_modelView.translate(m_panOffset);
    m_modelView.translate(0, 0, -m_distance);

    QMatrix4x4 rot = quaternionToMatrix(m_orientation);
    m_modelView = m_modelView * rot;
    
    m_projection.setToIdentity();
    m_projection.perspective(45.0f, float(width()) / float(height()), 0.1f, 1000.0f);
    
    m_program->setUniformValue("modelView", m_modelView);
    m_program->setUniformValue("projection", m_projection);
    m_program->setUniformValue("uPointSize", m_pointSize);
    // 临时屏幕框选（拖拽中）
    m_program->setUniformValue("uSelectionEnabled", 0);
    m_program->setUniformValue("uSelRect", QVector4D(0, 0, 0, 0));

    // 持久选择（使用选择当时的矩阵参数）
    if (m_selectionLocked) {
        m_program->setUniformValue("uPersistEnabled", 1);
        m_program->setUniformValue("uPersistRect", QVector4D(m_selRectLogical.left(), m_selRectLogical.top(), m_selRectLogical.right(), m_selRectLogical.bottom()));
        m_program->setUniformValue("uSelModelView", m_selModelView);
        m_program->setUniformValue("uSelProjection", m_selProjection);
        m_program->setUniformValue("uViewport", QVector2D(float(m_selViewportW), float(m_selViewportH)));
        m_program->setUniformValue("uDepthRange", QVector2D(m_selViewZMin, m_selViewZMax));
    } else {
        m_program->setUniformValue("uPersistEnabled", 0);
        m_program->setUniformValue("uPersistRect", QVector4D(0,0,0,0));
        m_program->setUniformValue("uSelModelView", QMatrix4x4());
        m_program->setUniformValue("uSelProjection", QMatrix4x4());
        m_program->setUniformValue("uViewport", QVector2D(0,0));
        m_program->setUniformValue("uDepthRange", QVector2D(0,0));
    }

    // 先绘制坐标轴
    glLineWidth(2.0f);
    m_axesVao.bind();
    glDrawArrays(GL_LINES, 0, 6);
    m_axesVao.release();
    glLineWidth(1.0f);

    // 拖拽时的屏幕框高亮
    if (m_selectionModeEnabled && m_selecting && !m_selectionLocked) {
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
    }
    
    // 绘制点云
    if (!m_points.isEmpty()) {
        m_vao.bind();
        glDrawArrays(GL_POINTS, 0, m_points.size());
        m_vao.release();
    }
    
    m_program->release();

    // 2D 覆盖层：绘制测距点、连线与距离
    if (m_measureMode && (m_haveP1 || m_haveP2)) {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(QPen(QColor(255,0,0), 2));
        auto drawPoint = [&](const QPoint& s) {
            painter.setBrush(QColor(255,0,0));
            painter.drawEllipse(s, 4, 4);
        };
        auto projectToScreen = [&](const QVector3D& w) -> QPoint {
            QVector4D hp(w, 1.0f);
            QVector4D clip = m_projection * (m_modelView * hp);
            if (clip.w() == 0.0f) return QPoint(-10000, -10000);
            QVector3D ndc = clip.toVector3DAffine();
            int sx = int(std::round((ndc.x() * 0.5f + 0.5f) * width()));
            int sy = int(std::round((1.0f - (ndc.y() * 0.5f + 0.5f)) * height()));
            return QPoint(sx, sy);
        };
        QPoint p1s = m_haveP1 ? projectToScreen(m_p1) : QPoint();
        QPoint p2s = m_haveP2 ? projectToScreen(m_p2) : QPoint();
        if (m_haveP1) drawPoint(p1s);
        if (m_haveP2) drawPoint(p2s);
        if (m_haveP1 && m_haveP2) {
            painter.drawLine(p1s, p2s);
            // 中点标注距离（米）
            QPoint mid((p1s.x()+p2s.x())/2, (p1s.y()+p2s.y())/2);
            double dist = (m_p2 - m_p1).length();
            painter.setPen(QPen(QColor(255,0,0)));
            painter.drawText(mid + QPoint(8,-8), QString::number(dist, 'f', 3) + " m");
        }
    }

    // 右下角图例
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
 
         // 准备标签文本并计算最大宽度
         QString title;
         switch (m_legendMode) {
             case 0: title = "Reflectivity"; break;
             case 1: title = "Distance"; break;
             case 2: title = "Elevation"; break;
             case 3: title = "Color"; break;
             case 4: title = "Planar Projection"; break;
             default: title = "Unknown"; break;
         }
         QStringList tickLabels;
         if (m_legendMode == 0) {
             tickLabels = {"255", "153", "102", "51", "0"};
         } else if (m_legendMode == 1 || m_legendMode == 2 || m_legendMode == 4) {
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
         auto addRefDistStops = [&grad]() {
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
         if (m_legendMode == 0 || m_legendMode == 1) addRefDistStops();
         else if (m_legendMode == 2) addElevationStops();
         else if (m_legendMode == 4) {
             // 平面投影：使用HSV渐变
             grad.setColorAt(0.0, QColor(255, 0, 0));      // 红色
             grad.setColorAt(0.2, QColor(255, 255, 0));    // 黄色
             grad.setColorAt(0.4, QColor(0, 255, 0));      // 绿色
             grad.setColorAt(0.6, QColor(0, 255, 255));    // 青色
             grad.setColorAt(0.8, QColor(0, 0, 255));      // 蓝色
             grad.setColorAt(1.0, QColor(255, 0, 255));    // 紫色
         }
         else { grad.setColorAt(0.0, Qt::white); grad.setColorAt(1.0, Qt::white); }
 
         painter.fillRect(barRect, QBrush(grad));
         painter.setPen(QColor(255,255,255,200));
         painter.drawRect(barRect.adjusted(0,0,-1,-1));
 
         // 标题
         painter.setPen(Qt::white);
         QRect titleRect(barLeft - 6, barRect.top() - 20, legendWidth + 12, 18);
         painter.drawText(titleRect, Qt::AlignLeft | Qt::AlignVCenter, title);
 
         // 刻度
         auto drawTick = [&](float norm, const QString& label) {
             int y = barRect.top() + int((1.0f - norm) * barRect.height());
             painter.drawLine(barRect.right() + 2, y, barRect.right() + 2 + tickLen, y);
             painter.drawText(barRect.right() + 2 + tickLen + labelSpacing, y + 4, label);
         };
         if (m_legendMode == 0) {
             drawTick(1.0f, "255");
             drawTick(0.6f,  "153");
             drawTick(0.4f,  "102");
             drawTick(0.2f,  "51");
             drawTick(0.0f,  "0");
         } else if (m_legendMode == 1 || m_legendMode == 2 || m_legendMode == 4) {
             auto fmt = [](float v) { return QString::number(v, 'f', 2); };
             drawTick(1.0f, fmt(m_legendMax));
             drawTick(0.5f, fmt((m_legendMin + m_legendMax) * 0.5f));
             drawTick(0.0f, fmt(m_legendMin));
         }
     }

    // 屏幕框矩形（仅拖拽过程显示）
    if (m_selecting && !m_selectionLocked) {
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

void PointCloudWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w * devicePixelRatioF(), h * devicePixelRatioF());
}

void PointCloudWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastMousePos = event->pos();
    m_activeButton = event->button();
    m_mousePressed = true;

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
        m_selecting = true;
        m_selStart = m_selEnd = event->pos();
        update();
        return;
    }
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (!m_mousePressed) return;
    
    QPoint delta = event->pos() - m_lastMousePos;

    // 移除拖动更新第二点的逻辑，改为仅在Ctrl+点击时更新

    if (m_selectionModeEnabled && m_selecting && m_activeButton == Qt::LeftButton && (event->modifiers() & Qt::ControlModifier)) {
        m_selEnd = event->pos();
        update();
        return;
    } else if (m_activeButton == Qt::LeftButton) {
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
        float aspect = (float)qMax(1, width()) / (float)qMax(1, height());
        float fovy_rad = 45.0f * float(M_PI) / 180.0f;
        float worldPerPixelY = 2.0f * m_distance * std::tan(fovy_rad * 0.5f) / (float)qMax(1, height());
        float worldPerPixelX = worldPerPixelY * aspect;
        m_panOffset.setX(m_panOffset.x() + delta.x() * worldPerPixelX);
        m_panOffset.setY(m_panOffset.y() - delta.y() * worldPerPixelY);
    }
    
    m_lastMousePos = event->pos();
    update();
}

void PointCloudWidget::mouseReleaseEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
    if (m_selectionModeEnabled && m_selecting && m_activeButton == Qt::LeftButton) {
        // 完成框选：记录选择当时的矩阵、视口以及选择矩形与深度范围
        m_selecting = false;
        QRect sel = m_selectionRect();
        if (!sel.isEmpty()) {
            // 捕获选择时的矩阵（当前帧的）
            m_selModelView = m_modelView;
            m_selProjection = m_projection;
            m_selViewportW = width();
            m_selViewportH = height();
            m_selRectLogical = sel;
            // 计算深度范围（基于选择时矩阵）
            QMatrix4x4 mvp = m_selProjection * m_selModelView;
            float zmin =  std::numeric_limits<float>::max();
            float zmax = -std::numeric_limits<float>::max();
            {
                QMutexLocker locker(&m_pointsMutex);
                for (const Point3D& p : m_points) {
                    QVector4D hp(p.x, p.y, p.z, 1.0f);
                    QVector4D clip = mvp * hp;
                    if (clip.w() == 0.0f) continue;
                    QVector3D ndc = clip.toVector3DAffine();
                    float sx = (ndc.x() * 0.5f + 0.5f) * float(m_selViewportW);
                    float sy = (1.0f - (ndc.y() * 0.5f + 0.5f)) * float(m_selViewportH);
                    if (sel.contains(QPoint(int(sx), int(sy)))) {
                        float vz = (m_selModelView * hp).z();
                        if (vz < zmin) zmin = vz;
                        if (vz > zmax) zmax = vz;
                    }
                }
            }
            if (zmin <= zmax) {
                m_selViewZMin = zmin;
                m_selViewZMax = zmax;
                m_selectionLocked = true;
            } else {
                m_selectionLocked = false;
            }
        }
        QWidget* w = window();
        if (w) {
            QMetaObject::invokeMethod(w, "onSelectionFinished", Qt::QueuedConnection);
        }
    }
    m_mousePressed = false;
    m_activeButton = Qt::NoButton;
}

void PointCloudWidget::wheelEvent(QWheelEvent *event)
{
    m_distance -= event->angleDelta().y() * 0.01f;
    m_distance = qMax(1.0f, m_distance);
    update();
}

void PointCloudWidget::updatePointCloud(const PointCloudFrame& frame)
{
    QMutexLocker locker(&m_pointsMutex);
    m_points = frame.points;
    m_vbo.bind();
    m_vbo.allocate(m_points.constData(), m_points.size() * sizeof(Point3D));
    m_vbo.release();
    update();
}

void PointCloudWidget::clearPointCloud()
{
    QMutexLocker locker(&m_pointsMutex);
    m_points.clear();
    printf("PointCloudWidget: cleared all points\n");
    update();
}

void PointCloudWidget::resetView()
{
    m_distance = 10.0f;
    m_rotation = QVector3D(0, 0, 0);
    // 重置视角：X 向上、Y 向左、Z 向外（绕 Z 轴 +90°）
    m_orientation = QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), 90.0f);
    m_panOffset = QVector3D(0, 0, 0);
    update();
}

void PointCloudWidget::setPointSize(float sizePixels)
{
    m_pointSize = qBound(1.0f, sizePixels, 10.0f);
    update();
}

void PointCloudWidget::setLegend(int mode, float minVal, float maxVal, bool visible)
{
    m_legendMode = mode;
    m_legendMin = minVal;
    m_legendMax = maxVal;
    m_legendVisible = visible;
    update();
}

void PointCloudWidget::setTopDownView()
{
    // 设置平面投影视角：Z轴朝上，Y轴朝向屏幕外，X轴朝右
    m_orientation = QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), 0.0f);
    m_distance = 15.0f;  // 稍微拉远一点以便观察整个平面
    m_panOffset = QVector3D(0, 0, 0);
    update();
} 

void PointCloudWidget::setSelectionModeEnabled(bool enabled)
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
        update();
    }
}

QVector<Point3D> PointCloudWidget::pointsInRect(const QRect& rect, int maxPoints)
{
    QVector<Point3D> result;
    if (rect.isEmpty()) return result;
    QMatrix4x4 mvp = m_projection * m_modelView;
    QMutexLocker locker(&m_pointsMutex);
    result.reserve(qMin(maxPoints, m_points.size()));
    for (const Point3D& p : m_points) {
        QVector4D hp(p.x, p.y, p.z, 1.0f);
        QVector4D clip = mvp * hp;
        if (clip.w() == 0.0f) continue;
        QVector3D ndc = clip.toVector3DAffine();
        float sx = (ndc.x() * 0.5f + 0.5f) * width();
        float sy = (1.0f - (ndc.y() * 0.5f + 0.5f)) * height();
        if (rect.contains(QPoint(int(sx), int(sy)))) {
            result.push_back(p);
            if (result.size() >= maxPoints) break;
        }
    }
    return result;
}

QVector<Point3D> PointCloudWidget::pointsInAabb(const QVector3D& min, const QVector3D& max, int maxPoints)
{
    QVector<Point3D> result;
    QMutexLocker locker(&m_pointsMutex);
    result.reserve(qMin(maxPoints, m_points.size()));
    for (const Point3D& p : m_points) {
        if (p.x >= min.x() && p.x <= max.x() &&
            p.y >= min.y() && p.y <= max.y() &&
            p.z >= min.z() && p.z <= max.z()) {
            result.push_back(p);
            if (result.size() >= maxPoints) break;
        }
    }
    return result;
}

QVector<Point3D> PointCloudWidget::pointsInPersistSelection(int maxPoints)
{
    QVector<Point3D> result;
    if (!m_selectionLocked) return result;
    QMatrix4x4 mvp = m_selProjection * m_selModelView;
    QMutexLocker locker(&m_pointsMutex);
    result.reserve(qMin(maxPoints, m_points.size()));
    for (const Point3D& p : m_points) {
        QVector4D hp(p.x, p.y, p.z, 1.0f);
        QVector4D clip = mvp * hp;
        if (clip.w() == 0.0f) continue;
        QVector3D ndc = clip.toVector3DAffine();
        float sx = (ndc.x() * 0.5f + 0.5f) * float(m_selViewportW);
        float sy = (1.0f - (ndc.y() * 0.5f + 0.5f)) * float(m_selViewportH);
        float vz = (m_selModelView * hp).z();
        if (sx >= m_selRectLogical.left() && sx <= m_selRectLogical.right() && sy >= m_selRectLogical.top() && sy <= m_selRectLogical.bottom() && vz >= m_selViewZMin && vz <= m_selViewZMax) {
            result.push_back(p);
            if (result.size() >= maxPoints) break;
        }
    }
    return result;
} 
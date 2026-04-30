#include "mainwindow.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <QPainter>
#include <QLinearGradient>
#include <QOpenGLFunctions>
#include <QMimeData>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QUrl>

// PointCloudWidget 实现
PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QOpenGLWidget(parent)
    , m_program(nullptr)
    , m_distance(10.0f)
    , m_rotation(0, 0, 0)
    , m_orientation() // identity
    , m_target(0, 0, 0)
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

void PointCloudWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        QVector3D pickedWorld;
        QPoint pickedScreen;
        
        // 调用你现有的拾取函数
        if (pickNearestPoint(event->pos(), pickedWorld, pickedScreen)) {
            // 核心改变：将双击的点设为新的旋转和缩放中心
            m_target = pickedWorld;
            
            // 细节优化：双击通常是为了看局部，稍微拉近距离
            m_distance = qMax(1.0f, m_distance * 0.5f); 
            
            update();
        }
    }
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
    setupGridBuffers();
}

void PointCloudWidget::setupShaders()
{
    const char *vertexShaderSource = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aColor;

        // 正常渲染用的矩阵
        uniform mat4 modelView;
        uniform mat4 projection;
        uniform float uPointSize;

        // 框选专用的 Uniform (来自 m_selectionLocked == true 时)
        uniform int uPersistEnabled;
        uniform mat4 uSelMVP;       // C++端传来的 m_selProjection * m_selModelView
        uniform vec4 uPersistRect;  // x0, y0, x1, y1
        uniform vec2 uViewport;     // 宽度, 高度

        // 传给片元着色器的变量
        out vec3 vColor;
        flat out int vIsSelected; // 【关键】flat 关键字表示这个整数不需要插值，原样传给片元

        void main() {
            // 1. 常规的顶点位置计算 (为了屏幕显示)
            gl_Position = projection * modelView * vec4(aPos, 1.0);
            gl_PointSize = uPointSize;
            vColor = aColor;
            
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
        flat in int vIsSelected; // 接收顶点着色器传来的判断结果

        out vec4 FragColor;

        void main() {
            // 片元着色器只做最简单的 IF 判断，不涉及任何矩阵运算！
            if (vIsSelected == 1) {
                FragColor = vec4(1.0, 0.0, 0.0, 1.0); // 框选的点涂成纯红色
            } else {
                FragColor = vec4(vColor, 1.0);        // 没选中的点保持原色
            }
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

//坐标轴属性
void PointCloudWidget::setupAxesBuffers()
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

void PointCloudWidget::setGridVisible(bool visible)
{
    if (m_gridVisible != visible) {
        m_gridVisible = visible;
        update(); // 触发重绘
    }
}

void PointCloudWidget::setGridConfig(const GridConfig& config)
{
    GridConfig normalized = config;
    normalized.range = std::max(1.0f, normalized.range);
    normalized.step = std::max(0.1f, normalized.step);
    if (!normalized.color.isValid()) {
        normalized.color = QColor(77, 77, 77);
    }

    m_gridConfig = normalized;

    if (context()) {
        makeCurrent();
        setupGridBuffers();
        doneCurrent();
    }
    update();
}

void PointCloudWidget::setupGridBuffers()
{
    struct GridVertex {
        float x, y, z;
        float r, g, b;
    };

    std::vector<GridVertex> gridVertices;

    const float range = std::max(1.0f, m_gridConfig.range);
    const float step = std::max(0.1f, m_gridConfig.step);
    const float r = float(m_gridConfig.color.redF());
    const float g = float(m_gridConfig.color.greenF());
    const float b = float(m_gridConfig.color.blueF());
    const int ringSegments = 96;
    const float pi = 3.14159265358979323846f;

    auto addLine = [&](const QVector3D& p1, const QVector3D& p2) {
        gridVertices.push_back({p1.x(), p1.y(), p1.z(), r, g, b});
        gridVertices.push_back({p2.x(), p2.y(), p2.z(), r, g, b});
    };

    if (m_gridConfig.type == GridConfig::ConcentricCircles) {
        addLine(QVector3D(-range, 0.0f, 0.0f), QVector3D(range, 0.0f, 0.0f));
        addLine(QVector3D(0.0f, -range, 0.0f), QVector3D(0.0f, range, 0.0f));

        for (float radius = step; radius <= range + 1e-4f; radius += step) {
            for (int i = 0; i < ringSegments; ++i) {
                const float a0 = (2.0f * pi * float(i)) / float(ringSegments);
                const float a1 = (2.0f * pi * float(i + 1)) / float(ringSegments);
                addLine(QVector3D(radius * std::cos(a0), radius * std::sin(a0), 0.0f),
                        QVector3D(radius * std::cos(a1), radius * std::sin(a1), 0.0f));
            }
        }
    } else {
        for (float i = -range; i <= range + 1e-4f; i += step) {
            addLine(QVector3D(i, -range, 0.0f), QVector3D(i, range, 0.0f));
            addLine(QVector3D(-range, i, 0.0f), QVector3D(range, i, 0.0f));
        }
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
    
    // 1. 设置基础变换矩阵
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

    // 1. 设置基础变换矩阵    
    m_projection.setToIdentity();
    const float nearPlane = 0.1f;
    float farPlane = qMax(1000.0f, m_distance * 10.0f); // 永远比当前视距大一个数量级
    m_projection.perspective(45.0f, float(width()) / float(height()), nearPlane, farPlane);    
    m_program->setUniformValue("modelView", m_modelView);
    m_program->setUniformValue("projection", m_projection);
    m_program->setUniformValue("uPointSize", m_pointSize);

    // ==========================================
    // 2. 绘制基础图元：坐标轴与网格
    // 强制关闭选择逻辑，确保它们保持原始颜色
    // ==========================================
    m_program->setUniformValue("uSelectionEnabled", 0);
    m_program->setUniformValue("uPersistEnabled", 0);

    // 绘制坐标轴
    glLineWidth(2.0f);
    m_axesVao.bind();
    glDrawArrays(GL_LINES, 0, m_axesVertexCount);
    m_axesVao.release();
    glLineWidth(1.0f);

    // 绘制网格
    if (m_gridVisible) {
        glLineWidth(1.0f); 
        if (m_gridVertexCount > 0) {
            m_gridVao.bind();
            glDrawArrays(GL_LINES, 0, m_gridVertexCount);
            m_gridVao.release();
        }
    }
    glLineWidth(1.0f);

    // ==========================================
    // 3. 准备绘制点云：恢复并设置点云的选择状态逻辑
    // ==========================================
    
    // 持久选择（使用选择当时的矩阵参数）
    if (m_selectionLocked) {
        m_program->setUniformValue("uPersistEnabled", 1);
        m_program->setUniformValue("uPersistRect", QVector4D(m_selRectLogical.left(), m_selRectLogical.top(), m_selRectLogical.right(), m_selRectLogical.bottom()));
// 【优化】：在 CPU 端提前计算好 MVP 矩阵，只传一个矩阵给 GPU
QMatrix4x4 selMVP = m_selProjection * m_selModelView;
m_program->setUniformValue("uSelMVP", selMVP);
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
    } else {
        // 确保未拖拽时关闭临时高亮
        m_program->setUniformValue("uSelectionEnabled", 0);
    }
    
    // 4. 绘制点云
    if (!m_points.isEmpty()) {
        m_vao.bind();
        glDrawArrays(GL_POINTS, 0, m_points.size());
        m_vao.release();
    }
    
    m_program->release();

    // ==========================================
    // 5. 绘制 2D 叠加层 (文本、UI组件等)
    // ==========================================

    // 绘制坐标轴标签 "X", "Y", "Z"
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::TextAntialiasing, true);
        painter.setPen(Qt::white);
        QFont f = painter.font();
        f.setBold(true);
        painter.setFont(f);

        QVector3D tips[] = { QVector3D(1.0f, 0, 0), QVector3D(0, 1.0f, 0), QVector3D(0, 0, 1.0f) };
        QString labels[] = { "X", "Y", "Z" };

        for (int i = 0; i < 3; ++i) {
            QVector4D hp(tips[i], 1.0f);
            // 建议分步计算，以便检查视图坐标系下的 Z 值
            QVector4D viewPos = m_modelView * hp;
            
            // 在 OpenGL 视图空间中，相机看向 -Z 方向。
            // 如果 viewPos.z() > 0，说明点在相机后面，或者在近裁剪面外
            if (viewPos.z() >= -0.1f) continue; 
        
            QVector4D clip = m_projection * viewPos;
            
            // 标准裁剪检查：NDC 坐标必须在 [-1, 1] 之间
            if (clip.z() < -clip.w() || clip.z() > clip.w()) continue;
            if (std::abs(clip.x()) > clip.w() || std::abs(clip.y()) > clip.w()) continue;
        
            QVector3D ndc = clip.toVector3DAffine();
            int sx = int((ndc.x() * 0.5f + 0.5f) * width());
            int sy = int((1.0f - (ndc.y() * 0.5f + 0.5f)) * height());
            painter.drawText(sx + 8, sy - 8, labels[i]);
        }
    }

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
        // --- 新的平移逻辑 ---
        float aspect = (float)qMax(1, width()) / (float)qMax(1, height());
        float fovy_rad = 45.0f * float(M_PI) / 180.0f;
        
        // 计算当前距离下，屏幕像素对应的世界坐标尺寸
        float factor = 2.0f * m_distance * std::tan(fovy_rad * 0.5f);
        float worldPerPixelY = factor / (float)qMax(1, height());
        float worldPerPixelX = worldPerPixelY * aspect;

        // 计算相机当前在世界坐标系下的局部坐标轴（右方向和上方向）
        // 这里使用 m_orientation 的共轭来获取逆变换方向
        QVector3D cameraRight = m_orientation.conjugated().rotatedVector(QVector3D(1, 0, 0));
        QVector3D cameraUp    = m_orientation.conjugated().rotatedVector(QVector3D(0, 1, 0));

        // 更新 m_target：鼠标向右，目标向左移；鼠标向上，目标向下移
        m_target -= cameraRight * (delta.x() * worldPerPixelX);
        m_target += cameraUp * (delta.y() * worldPerPixelY);
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
    const float factor = 1.0f - event->angleDelta().y() * 0.001f; // 每格约 1.5% 的比例变化
    m_distance = qMax(0.1f, m_distance * factor);                  // 最小距离 0.1
    update();
}

void PointCloudWidget::dragEnterEvent(QDragEnterEvent* event)
{
    if (!event || !event->mimeData() || !event->mimeData()->hasUrls()) {
        return;
    }

    const QList<QUrl> urls = event->mimeData()->urls();
    for (const QUrl& url : urls) {
        if (url.isLocalFile() && url.toLocalFile().endsWith(".lvx2", Qt::CaseInsensitive)) {
            event->acceptProposedAction();
            return;
        }
    }
}

void PointCloudWidget::dropEvent(QDropEvent* event)
{
    if (!event || !event->mimeData() || !event->mimeData()->hasUrls()) {
        return;
    }

    const QList<QUrl> urls = event->mimeData()->urls();
    for (const QUrl& url : urls) {
        const QString localFile = url.toLocalFile();
        if (url.isLocalFile() && localFile.endsWith(".lvx2", Qt::CaseInsensitive)) {
            emit lvx2FileDropped(localFile);
            event->acceptProposedAction();
            return;
        }
    }
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
    m_orientation = QQuaternion::fromAxisAndAngle(QVector3D(0, 0, 1), 90.0f);
    m_target = QVector3D(0, 0, 0); // 重置观察中心
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
    m_orientation = QQuaternion::fromAxisAndAngle(QVector3D(1, 0, 0), 0.0f);
    m_distance = 15.0f;
    m_target = QVector3D(0, 0, 0); // 重置观察中心
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

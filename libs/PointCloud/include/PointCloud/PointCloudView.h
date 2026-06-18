#ifndef POINTCLOUD_POINTCLOUDVIEW_H
#define POINTCLOUD_POINTCLOUDVIEW_H

#include "PointCloud/PointCloudFrame.h"
#include "PointCloud/PointCloudCrossSection.h"
#include "plugins/StlModel/StlModelLoader.h"

#include <QColor>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMatrix4x4>
#include <QMutex>
#include <QMutexLocker>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <QQuaternion>
#include <QRect>
#include <QVector>
#include <QVector3D>
#include <QWheelEvent>

#include <functional>

class PointCloudView : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT

public:
    enum class ViewPreset {
        World = 0,
        Front = 1,
        Back = 2,
        Left = 3,
        Right = 4,
        Top = 5
    };

    enum class ProjectionMode {
        Perspective = 0,
        Orthographic = 1
    };

    struct GridConfig {
        enum Type {
            Square = 0,
            ConcentricCircles = 1
        };

        float range = 100.0f;
        float step = 1.0f;
        QColor color = QColor(77, 77, 77);
        Type type = Square;
    };

    explicit PointCloudView(QWidget *parent = nullptr);
    ~PointCloudView();

    void setGridVisible(bool visible);
    void setGridConfig(const GridConfig& config);
    GridConfig gridConfig() const { return m_gridConfig; }
    void updatePointCloud(const PointCloudFrame& frame);
    void updatePointCloud(PointCloudFrame&& frame);
    void recolorCurrentPointCloud(const std::function<void(QVector<PointCloudPoint>&)>& colorize);
    void clearPointCloud();
    void resetView();
    void setPointSize(float sizePixels);
    void setBackgroundColors(const QColor& topColor, const QColor& bottomColor);
    void setLegend(int mode,
                   float minVal,
                   float maxVal,
                   bool visible,
                   const QVector<QColor>& lineColors = {},
                   const QVector<int>& lineNumbers = {},
                   const QVector<QColor>& gradientColors = {});
    QRect currentSelectionRect() const { return m_selectionRect(); }
    QVector<PointCloudPoint> currentPoints() const { QMutexLocker locker(const_cast<QMutex*>(&m_pointsMutex)); return m_points; }
    void setSelectionModeEnabled(bool enabled);
    bool isSelectionModeEnabled() const { return m_selectionModeEnabled; }
    QVector<PointCloudPoint> pointsInRect(const QRect& rect, int maxPoints = 5000);
    QVector<PointCloudPoint> pointsInAabb(const QVector3D& min, const QVector3D& max, int maxPoints = 5000);
    QVector<PointCloudPoint> pointsInPersistSelection(int maxPoints = 5000);

    void setCrossSectionModeEnabled(bool enabled);
    bool isCrossSectionModeEnabled() const { return m_crossSectionState.enabled; }
    QVector<PointCloudPoint> currentCrossSectionPoints() const { return m_crossSectionState.clippedPoints; }
    void resetCrossSectionBoxToCurrentCloud();
    void setCrossSectionControlsVisible(bool visible);
    bool crossSectionControlsVisible() const { return m_crossSectionState.controlsVisible; }
    void setStlModelMesh(const StlModel::Mesh& mesh, bool sourceXReversed = false);
    void setStlModelVisible(bool visible);
    bool isStlModelVisible() const { return m_stlModelVisible; }
    bool hasStlModel() const { return !m_stlModelVertices.isEmpty(); }

    void setSelectionAabb(const QVector3D& min, const QVector3D& max) { m_aabbMin = min; m_aabbMax = max; m_selectionLocked = true; update(); }
    void clearSelectionAabb() { m_selectionLocked = false; update(); }
    bool hasSelectionAabb() const { return m_selectionLocked; }
    QPair<QVector3D,QVector3D> selectionAabb() const { return { m_aabbMin, m_aabbMax }; }

    void setMeasurementModeEnabled(bool enabled) { m_measureMode = enabled; if (!enabled) { m_haveP1=false; m_haveP2=false; update(); } }
    bool isMeasurementModeEnabled() const { return m_measureMode; }
    bool hasMeasureP1() const { return m_haveP1; }
    bool hasMeasureP2() const { return m_haveP2; }
    QVector3D getMeasureP1() const { return m_p1; }
    QVector3D getMeasureP2() const { return m_p2; }
    double getMeasureDistance() const { return m_haveP1 && m_haveP2 ? (m_p2 - m_p1).length() : 0.0; }

    void setTopDownView();
    void setViewPreset(ViewPreset preset);
    void setProjectionMode(ProjectionMode mode);
    ProjectionMode projectionMode() const { return m_projectionMode; }

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dropEvent(QDropEvent* event) override;
    void leaveEvent(QEvent *event) override;
    void setupGridBuffers();

signals:
    void lvx2FileDropped(const QString& filePath);
    void crossSectionChanged(int clippedPointCount, int sourcePointCount);

private:
    void setupShaders();
    void setupBackgroundBuffers();
    void setupBuffers();
    void setupAxesBuffers();
    void setupCrossSectionBuffers();
    QVector3D mapToArcball(const QPoint& p) const;
    bool pickNearestPoint(const QPoint& pos, QVector3D& outWorld, QPoint& outScreen, int pixelRadius = 10);
    void uploadPointCloudPoints(QVector<PointCloudPoint>&& points);
    void updateCrossSectionPointCloud();
    void uploadCrossSectionLines(const QVector<PointCloudCrossSection::ColoredVertex>& vertices);
    void uploadCrossSectionTriangles(const QVector<PointCloudCrossSection::ColoredVertex>& vertices);
    void setupStlModelBuffers();
    void uploadStlModelVertices();
    PointCloudCrossSection::Camera crossSectionCamera() const;

    QOpenGLShaderProgram *m_program;
    QOpenGLShaderProgram* m_backgroundProgram = nullptr;
    QOpenGLBuffer m_backgroundVbo;
    QOpenGLVertexArrayObject m_backgroundVao;
    QOpenGLBuffer m_vbo;
    QOpenGLVertexArrayObject m_vao;
    qsizetype m_pointCloudBufferCapacityBytes = 0;

    QOpenGLBuffer m_axesVbo;
    QOpenGLVertexArrayObject m_axesVao;
    int m_axesVertexCount = 0;

    QOpenGLBuffer m_crossSectionVbo;
    QOpenGLVertexArrayObject m_crossSectionVao;
    int m_crossSectionVertexCount = 0;
    int m_crossSectionBoxLineVertexCount = 0;
    QOpenGLBuffer m_crossSectionTriangleVbo;
    QOpenGLVertexArrayObject m_crossSectionTriangleVao;
    int m_crossSectionTriangleVertexCount = 0;
    QOpenGLBuffer m_stlModelVbo;
    QOpenGLVertexArrayObject m_stlModelVao;
    QVector<StlModel::Vertex> m_stlModelVertices;
    bool m_stlModelVisible = false;

    QMatrix4x4 m_projection;
    QMatrix4x4 m_modelView;

    QVector<PointCloudPoint> m_points;
    QMutex m_pointsMutex;

    bool m_gridVisible;
    QOpenGLVertexArrayObject m_gridVao;
    QOpenGLBuffer m_gridVbo;
    int m_gridVertexCount = 0;
    GridConfig m_gridConfig;

    float m_distance;
    QVector3D m_rotation;
    QQuaternion m_orientation;
    QVector3D m_target;
    Qt::MouseButton m_activeButton;
    QPoint m_lastMousePos;
    bool m_mousePressed;

    float m_pointSize = 2.0f;
    ProjectionMode m_projectionMode = ProjectionMode::Perspective;

    bool m_legendVisible = false;
    int m_legendMode = 0;
    float m_legendMin = 0.0f;
    float m_legendMax = 1.0f;
    QVector<QColor> m_lineLegendColors;
    QVector<int> m_lineLegendNumbers;
    QVector<QColor> m_legendGradientColors;
    QColor m_backgroundTopColor = QColor(26, 26, 26);
    QColor m_backgroundBottomColor = QColor(26, 26, 26);

    bool m_selecting = false;
    QPoint m_selStart;
    QPoint m_selEnd;
    QRect m_selectionRect() const { return QRect(m_selStart, m_selEnd).normalized(); }
    bool m_selectionModeEnabled = false;

    PointCloudCrossSection::State m_crossSectionState;

    bool m_selectionLocked = false;
    QVector3D m_aabbMin;
    QVector3D m_aabbMax;
    QMatrix4x4 m_selModelView;
    QMatrix4x4 m_selProjection;
    QRect m_selRectLogical;
    int m_selViewportW = 0;
    int m_selViewportH = 0;
    float m_selViewZMin = 0.0f;
    float m_selViewZMax = 0.0f;

    bool m_measureMode = false;
    bool m_haveP1 = false;
    bool m_haveP2 = false;
    QVector3D m_p1;
    QVector3D m_p2;
    QPoint m_p1Screen;
    QPoint m_p2Screen;
};

#endif // POINTCLOUD_POINTCLOUDVIEW_H

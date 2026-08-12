#ifndef POINTCLOUD_POINTCLOUDVIEW_H
#define POINTCLOUD_POINTCLOUDVIEW_H

#include "PointCloudFrame.h"
#include "PointCloudCrossSection.h"
#include "PointCloudRenderConfig.h"
#include "Visualization/SlamRenderSnapshot.h"
#include "plugins/StlModel/StlModelLoader.h"

#include <QColor>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QElapsedTimer>
#include <QHideEvent>
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
#include <QTimer>
#include <QVector>
#include <QVector3D>
#include <QWheelEvent>

#include <deque>
#include <functional>
#include <memory>
#include <limits>

struct StlRenderVertex {
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

class PointCloudEdlRenderer;

class PointCloudView : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT

public:
    using EdlConfig = PointCloudEdlConfig;

    struct SelectionRegion {
        bool valid = false;
        QMatrix4x4 mvp;
        QMatrix4x4 modelView;
        QRect rect;
        int viewportW = 0;
        int viewportH = 0;
    };

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
        Orthographic = 1,
        ObserverPerspective = 2,
        SlamPoseFollow = 3
    };

    struct GridConfig {
        enum Type {
            Square = 0,
            ConcentricCircles = 1,
            SquareAndConcentricCircles = 2
        };

        float range = 100.0f;
        float step = 1.0f;
        QColor color = QColor(77, 77, 77);
        Type type = Square;
    };

    explicit PointCloudView(QWidget *parent = nullptr);
    ~PointCloudView();

    void setGridVisible(bool visible);
    bool isGridVisible() const { return m_gridVisible; }
    void setGridConfig(const GridConfig& config);
    GridConfig gridConfig() const { return m_gridConfig; }
    void updatePointCloud(const PointCloudFrame& frame);
    void updatePointCloud(PointCloudFrame&& frame);
    void clearPointCloudSegments();
    void appendPointCloudSegment(QVector<PointCloudPoint>&& points);
    void removeFirstPointCloudSegment();
    int pointCloudSegmentCount() const;
    void recolorCurrentPointCloud(const std::function<void(QVector<PointCloudPoint>&)>& colorize);
    void clearPointCloud();
    void resetView();
    void setPointSize(float sizePixels);
    void setEdlConfig(const EdlConfig& config);
    EdlConfig edlConfig() const { return m_edlConfig; }
    bool isEdlSupported() const;
    QString edlErrorMessage() const;
    void setBackgroundColors(const QColor& topColor, const QColor& bottomColor);
    void setLegend(int mode,
                   float minVal,
                   float maxVal,
                   bool visible,
                   const QVector<QColor>& lineColors = {},
                   const QVector<int>& lineNumbers = {},
                   const QVector<QColor>& gradientColors = {});
    QRect currentSelectionRect() const { return m_selectionRect(); }
    QVector<PointCloudPoint> currentPoints() const;
    void setSelectionModeEnabled(bool enabled);
    bool isSelectionModeEnabled() const { return m_selectionModeEnabled; }
    void requestSelectionUpdate();
    QVector<PointCloudPoint> pointsInRect(const QRect& rect, int maxPoints = 5000);
    QVector<PointCloudPoint> pointsInAabb(const QVector3D& min, const QVector3D& max, int maxPoints = 5000);
    QVector<PointCloudPoint> pointsInPersistSelection(int maxPoints = 5000);

    void setCrossSectionModeEnabled(bool enabled);
    bool isCrossSectionModeEnabled() const { return m_crossSectionState.enabled; }
    QVector<PointCloudPoint> currentCrossSectionPoints() const;
    void resetCrossSectionBoxToCurrentCloud();
    void setCrossSectionControlsVisible(bool visible);
    bool crossSectionControlsVisible() const { return m_crossSectionState.controlsVisible; }
    void setStlModelMesh(const StlModel::Mesh& mesh,
                         bool sourceXReversed = false,
                         float sourceUnitToMeters = 0.001f);
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
    void setSlamRenderSnapshot(const SlamRenderSnapshot& snapshot);
    void setSlamFollowPose(const SlamRenderPose& pose);
    void setSlamPoseAxisVertices(const QVector<SlamRenderVertex>& vertices);
    void clearSlamRenderOverlay();

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
    void hideEvent(QHideEvent* event) override;
    void leaveEvent(QEvent *event) override;
    void setupGridBuffers();

signals:
    void lvx2FileDropped(const QString& filePath);
    void crossSectionChanged(int clippedPointCount, int sourcePointCount);
    void selectionPointsReady(QVector<PointCloudPoint> points, int zeroPointCount);

private:
    struct PointCloudSegment;

    void setupShaders();
    void setupBackgroundBuffers();
    void setupBuffers();
    void setupAxesBuffers();
    void setupCrossSectionBuffers();
    void setupFreeDomVoxelBuffers();
    void uploadFreeDomVoxelInstances(const QVector<SlamRenderVertex>& instances,
                                     QOpenGLBuffer& instanceVbo,
                                     QOpenGLVertexArrayObject& vao,
                                     qsizetype& capacityBytes,
                                     int& instanceCount);
    void uploadSlamRenderOverlayIfNeeded();
    void destroySlamRenderOverlay();
    QVector3D mapToArcball(const QPoint& p) const;
    bool pickNearestPoint(const QPoint& pos, QVector3D& outWorld, QPoint& outScreen, int pixelRadius = 10);
    void uploadPointCloudPoints(QVector<PointCloudPoint>&& points);
    void uploadPointCloudSegment(PointCloudSegment& segment);
    void uploadPointCloudSegmentClip(PointCloudSegment& segment);
    void uploadPointCloudSegmentSelection(PointCloudSegment& segment);
    void uploadSelectionPoints(QVector<PointCloudPoint>&& points);
    void syncPendingPointCloudBuffers();
    void destroyPointCloudSegments();
    bool pointCloudSegmentSourceBounds(QVector3D& minPoint, QVector3D& maxPoint) const;
    void forEachDisplayedPoint(const std::function<bool(const PointCloudPoint&)>& visitor) const;
    void updateCrossSectionPointCloud();
    void requestCrossSectionClip(bool immediate);
    void startCrossSectionClipJob();
    void startCrossSectionSegmentClipJob(quint64 segmentId, const QVector<PointCloudPoint>& points);
    void clearSegmentSelectionBuffers(PointCloudSegment& segment);
    void clearSelectionCaches();
    void updateSelectionRegionFromRect(const QRect& rect);
    void startSelectionJob();
    void startSelectionSegmentJob(quint64 segmentId, const QVector<PointCloudPoint>& points);
    void requestSelectionPublish();
    void startSelectionPublishJob();
    void uploadCrossSectionLines(const QVector<PointCloudCrossSection::ColoredVertex>& vertices);
    void uploadCrossSectionTriangles(const QVector<PointCloudCrossSection::ColoredVertex>& vertices);
    void setupStlModelBuffers();
    void uploadStlModelVertices();
    PointCloudCrossSection::Camera crossSectionCamera() const;
    QVector3D cameraForward() const;
    QVector3D cameraRight() const;
    QVector3D cameraUp() const;
    QVector3D orbitCameraPosition() const;
    void syncViewerPositionFromOrbit();
    void syncOrbitTargetFromViewer();
    void advanceSlamFollowSmoothing();
    QSize edlPhysicalSize() const;
    QSize widgetPhysicalSize() const;

    QOpenGLShaderProgram *m_program;
    QOpenGLShaderProgram* m_voxelProgram = nullptr;
    QOpenGLShaderProgram* m_backgroundProgram = nullptr;
    std::unique_ptr<PointCloudEdlRenderer> m_edlRenderer;
    EdlConfig m_edlConfig;
    bool m_edlFallbackReported = false;
    QOpenGLBuffer m_backgroundVbo;
    QOpenGLVertexArrayObject m_backgroundVao;
    QOpenGLBuffer m_vbo;
    QOpenGLVertexArrayObject m_vao;
    qsizetype m_pointCloudBufferCapacityBytes = 0;

    QOpenGLBuffer m_axesVbo;
    QOpenGLVertexArrayObject m_axesVao;
    int m_axesLineVertexCount = 0;
    int m_axesTriangleVertexCount = 0;

    QOpenGLBuffer m_slamTrajectoryVbo;
    QOpenGLVertexArrayObject m_slamTrajectoryVao;
    qsizetype m_slamTrajectoryBufferCapacityBytes = 0;
    int m_slamTrajectoryVertexCount = 0;
    QOpenGLBuffer m_slamLoopClosureVbo;
    QOpenGLVertexArrayObject m_slamLoopClosureVao;
    qsizetype m_slamLoopClosureBufferCapacityBytes = 0;
    int m_slamLoopClosureVertexCount = 0;
    QOpenGLBuffer m_slamPoseAxisVbo;
    QOpenGLVertexArrayObject m_slamPoseAxisVao;
    qsizetype m_slamPoseAxisBufferCapacityBytes = 0;
    int m_slamPoseAxisVertexCount = 0;
    QOpenGLBuffer m_slamWorldFrameVbo;
    QOpenGLVertexArrayObject m_slamWorldFrameVao;
    qsizetype m_slamWorldFrameBufferCapacityBytes = 0;
    int m_slamWorldFrameVertexCount = 0;
    QOpenGLBuffer m_slamBodyFrameVbo;
    QOpenGLVertexArrayObject m_slamBodyFrameVao;
    qsizetype m_slamBodyFrameBufferCapacityBytes = 0;
    int m_slamBodyFrameVertexCount = 0;
    QOpenGLBuffer m_slamDynamicObjectVbo;
    QOpenGLVertexArrayObject m_slamDynamicObjectVao;
    qsizetype m_slamDynamicObjectBufferCapacityBytes = 0;
    int m_slamDynamicObjectVertexCount = 0;
    QOpenGLBuffer m_slamFreeDomDynamicPointsVbo;
    QOpenGLVertexArrayObject m_slamFreeDomDynamicPointsVao;
    qsizetype m_slamFreeDomDynamicPointsBufferCapacityBytes = 0;
    int m_slamFreeDomDynamicPointsVertexCount = 0;
    QOpenGLBuffer m_freeDomVoxelCubeVbo;
    int m_freeDomVoxelCubeVertexCount = 0;
    QOpenGLBuffer m_slamFreeDomScanVoxelVbo;
    QOpenGLVertexArrayObject m_slamFreeDomScanVoxelVao;
    qsizetype m_slamFreeDomScanVoxelBufferCapacityBytes = 0;
    int m_slamFreeDomScanVoxelVertexCount = 0;
    QOpenGLBuffer m_slamFreeDomDynamicVoxelVbo;
    QOpenGLVertexArrayObject m_slamFreeDomDynamicVoxelVao;
    qsizetype m_slamFreeDomDynamicVoxelBufferCapacityBytes = 0;
    int m_slamFreeDomDynamicVoxelVertexCount = 0;
    QOpenGLBuffer m_slamFreeDomRaycastedVoxelVbo;
    QOpenGLVertexArrayObject m_slamFreeDomRaycastedVoxelVao;
    qsizetype m_slamFreeDomRaycastedVoxelBufferCapacityBytes = 0;
    int m_slamFreeDomRaycastedVoxelVertexCount = 0;
    QOpenGLBuffer m_slamFreeDomFreeVoxelVbo;
    QOpenGLVertexArrayObject m_slamFreeDomFreeVoxelVao;
    qsizetype m_slamFreeDomFreeVoxelBufferCapacityBytes = 0;
    int m_slamFreeDomFreeVoxelVertexCount = 0;
    QOpenGLBuffer m_slamFreeDomStaticVoxelVbo;
    QOpenGLVertexArrayObject m_slamFreeDomStaticVoxelVao;
    qsizetype m_slamFreeDomStaticVoxelBufferCapacityBytes = 0;
    int m_slamFreeDomStaticVoxelVertexCount = 0;
    QOpenGLBuffer m_slamFreeDomEnhancedVbo;
    QOpenGLVertexArrayObject m_slamFreeDomEnhancedVao;
    qsizetype m_slamFreeDomEnhancedBufferCapacityBytes = 0;
    int m_slamFreeDomEnhancedVertexCount = 0;
    SlamRenderSnapshot m_slamRenderSnapshot;
    bool m_slamRenderUploadPending = false;
    bool m_slamPoseAxisUploadPending = false;
    bool m_slamVoxelClipUploadPending = false;
    quint64 m_uploadedFreeDomScanVoxelRevision = std::numeric_limits<quint64>::max();
    quint64 m_uploadedFreeDomDynamicVoxelRevision = std::numeric_limits<quint64>::max();
    quint64 m_uploadedFreeDomRaycastedVoxelRevision = std::numeric_limits<quint64>::max();
    quint64 m_uploadedFreeDomFreeVoxelRevision = std::numeric_limits<quint64>::max();
    quint64 m_uploadedFreeDomStaticVoxelRevision = std::numeric_limits<quint64>::max();

    QOpenGLBuffer m_crossSectionVbo;
    QOpenGLVertexArrayObject m_crossSectionVao;
    int m_crossSectionVertexCount = 0;
    int m_crossSectionBoxLineVertexCount = 0;
    QOpenGLBuffer m_crossSectionTriangleVbo;
    QOpenGLVertexArrayObject m_crossSectionTriangleVao;
    int m_crossSectionTriangleVertexCount = 0;
    QOpenGLBuffer m_stlModelVbo;
    QOpenGLVertexArrayObject m_stlModelVao;
    QVector<StlRenderVertex> m_stlModelVertices;
    bool m_stlModelVisible = false;

    QMatrix4x4 m_projection;
    QMatrix4x4 m_modelView;

    QVector<PointCloudPoint> m_points;
    QVector<PointCloudPoint> m_selectedPoints;
    struct PointCloudSegment {
        quint64 id = 0;
        QVector<PointCloudPoint> points;
        QVector<PointCloudPoint> clippedPoints;
        QVector<PointCloudPoint> selectedPoints;
        QOpenGLBuffer vbo;
        QOpenGLVertexArrayObject vao;
        QOpenGLBuffer clippedVbo;
        QOpenGLVertexArrayObject clippedVao;
        QOpenGLBuffer selectedVbo;
        QOpenGLVertexArrayObject selectedVao;
        qsizetype bufferCapacityBytes = 0;
        qsizetype clippedBufferCapacityBytes = 0;
        qsizetype selectedBufferCapacityBytes = 0;
        int pointCount = 0;
        int clippedPointCount = 0;
        int selectedPointCount = 0;
        bool sourceUploadPending = true;

        PointCloudSegment()
            : vbo(QOpenGLBuffer::VertexBuffer)
            , clippedVbo(QOpenGLBuffer::VertexBuffer)
            , selectedVbo(QOpenGLBuffer::VertexBuffer)
        {}
    };
    std::deque<std::unique_ptr<PointCloudSegment>> m_pointCloudSegments;
    QMutex m_pointsMutex;
    quint64 m_nextPointCloudSegmentId = 1;

    QOpenGLBuffer m_selectionVbo;
    QOpenGLVertexArrayObject m_selectionVao;
    qsizetype m_selectionBufferCapacityBytes = 0;
    int m_selectionPointCount = 0;
    bool m_pointCloudGpuUploadPending = false;

    SelectionRegion m_selectionRegion;
    QTimer* m_crossSectionClipDebounceTimer = nullptr;
    QTimer* m_selectionPublishDebounceTimer = nullptr;
    quint64 m_crossSectionClipGeneration = 0;
    bool m_crossSectionClipRunning = false;
    bool m_crossSectionClipPending = false;
    int m_crossSectionClipRemaining = 0;
    quint64 m_selectionGeneration = 0;
    bool m_selectionJobRunning = false;
    bool m_selectionJobPending = false;
    int m_selectionJobRemaining = 0;
    bool m_selectionPublishRunning = false;
    bool m_selectionPublishPending = false;

    bool m_gridVisible;
    QOpenGLVertexArrayObject m_gridVao;
    QOpenGLBuffer m_gridVbo;
    int m_gridVertexCount = 0;
    GridConfig m_gridConfig;

    float m_distance;
    QVector3D m_rotation;
    QQuaternion m_orientation;
    QVector3D m_target;
    QVector3D m_viewerPosition;
    QVector3D m_slamFollowPosition;
    QQuaternion m_slamFollowOrientation;
    QVector3D m_slamFollowTargetPosition;
    QQuaternion m_slamFollowTargetOrientation;
    float m_slamFollowDistanceM = 10.05f;
    float m_slamFollowYawDeg = 180.0f;
    float m_slamFollowPitchDeg = 5.71f;
    bool m_slamFollowPoseValid = false;
    QTimer* m_slamFollowSmoothingTimer = nullptr;
    QElapsedTimer m_slamFollowSmoothingClock;
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

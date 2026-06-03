#ifndef POINTCLOUD_POINTCLOUDCROSSSECTION_H
#define POINTCLOUD_POINTCLOUDCROSSSECTION_H

#include "PointCloud/PointCloudTypes.h"

#include <QMatrix4x4>
#include <QPoint>
#include <QPointF>
#include <QQuaternion>
#include <QSize>
#include <QVector>
#include <QVector3D>

namespace PointCloudCrossSection {

struct ColoredVertex {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;
};

struct Box {
    QVector3D center;
    QVector3D halfExtents;
    QQuaternion orientation;
};

struct Camera {
    QMatrix4x4 modelView;
    QMatrix4x4 projection;
    QSize viewport;
};

enum class HandleType {
    None = 0,
    Center,
    TranslateX,
    TranslateY,
    TranslateZ,
    ScaleMinX,
    ScaleMaxX,
    ScaleMinY,
    ScaleMaxY,
    ScaleMinZ,
    ScaleMaxZ,
    RotateMinX,
    RotateMaxX,
    RotateMinY,
    RotateMaxY,
    RotateMinZ,
    RotateMaxZ
};

struct HitResult {
    HandleType handle = HandleType::None;
    float distancePixels = 0.0f;
};

struct State {
    bool enabled = false;
    bool initialized = false;
    Box box;
    QVector<PointCloudPoint> sourcePoints;
    QVector<PointCloudPoint> clippedPoints;
    HandleType activeHandle = HandleType::None;
    HandleType hoverHandle = HandleType::None;
    bool controlsVisible = true;
    bool dragging = false;
    QPoint dragStart;
    Box dragStartBox;
    QVector3D dragAxis;
    QPointF dragAxisScreen;
    QPointF dragRingCenterScreen;
    float dragUnitsPerPixel = 0.0f;
    float dragRotationSign = 1.0f;
    QVector3D dragCameraRight;
    QVector3D dragCameraUp;
};

bool initializeBox(State& state, const QVector<PointCloudPoint>& points);
void setSourcePoints(State& state, const QVector<PointCloudPoint>& points);
QVector<PointCloudPoint> clip(const QVector<PointCloudPoint>& points, const State& state);
void updateClip(State& state);

HitResult hitTest(const State& state, const Camera& camera, const QPoint& mousePos);
bool updateHover(State& state, const Camera& camera, const QPoint& mousePos);
bool beginDrag(State& state, const Camera& camera, const QPoint& mousePos);
bool updateDrag(State& state, const Camera& camera, const QPoint& mousePos);
void endDrag(State& state);
void setControlsVisible(State& state, bool visible);

QVector<ColoredVertex> buildBoxLines(const State& state);
QVector<ColoredVertex> buildGizmoLines(const State& state, const Camera& camera);

} // namespace PointCloudCrossSection

#endif // POINTCLOUD_POINTCLOUDCROSSSECTION_H

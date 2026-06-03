#include "PointCloud/PointCloudCrossSection.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QVector4D>

namespace PointCloudCrossSection {
namespace {

constexpr float kMinHalfExtent = 0.05f;
constexpr float kHitThresholdPixels = 12.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kCenterHitRadiusPixels = 14.0f;
constexpr float kTranslateOffsetPixels = 18.0f;
constexpr float kTranslateLengthPixels = 84.0f;
constexpr float kScaleHandlePixels = 48.0f;
constexpr float kRotationRingRadiusPixels = 28.0f;
constexpr float kArrowHeadPixels = 16.0f;
constexpr float kScaleCrossPixels = 11.0f;
constexpr float kCenterCrossPixels = 16.0f;
constexpr float kCenterRingPixels = 18.0f;

QVector3D axisForHandle(const Box& box, HandleType handle)
{
    switch (handle) {
    case HandleType::Center:
        return {};
    case HandleType::TranslateX:
    case HandleType::ScaleMaxX:
    case HandleType::RotateMaxX:
        return box.orientation.rotatedVector(QVector3D(1.0f, 0.0f, 0.0f)).normalized();
    case HandleType::ScaleMinX:
    case HandleType::RotateMinX:
        return box.orientation.rotatedVector(QVector3D(-1.0f, 0.0f, 0.0f)).normalized();
    case HandleType::TranslateY:
    case HandleType::ScaleMaxY:
    case HandleType::RotateMaxY:
        return box.orientation.rotatedVector(QVector3D(0.0f, 1.0f, 0.0f)).normalized();
    case HandleType::ScaleMinY:
    case HandleType::RotateMinY:
        return box.orientation.rotatedVector(QVector3D(0.0f, -1.0f, 0.0f)).normalized();
    case HandleType::TranslateZ:
    case HandleType::ScaleMaxZ:
    case HandleType::RotateMaxZ:
        return box.orientation.rotatedVector(QVector3D(0.0f, 0.0f, 1.0f)).normalized();
    case HandleType::ScaleMinZ:
    case HandleType::RotateMinZ:
        return box.orientation.rotatedVector(QVector3D(0.0f, 0.0f, -1.0f)).normalized();
    case HandleType::None:
        return {};
    }
    return {};
}

float maxHalfExtent(const Box& box)
{
    return std::max({box.halfExtents.x(), box.halfExtents.y(), box.halfExtents.z(), 0.1f});
}

bool isScaleHandle(HandleType handle)
{
    return handle == HandleType::ScaleMinX || handle == HandleType::ScaleMaxX ||
           handle == HandleType::ScaleMinY || handle == HandleType::ScaleMaxY ||
           handle == HandleType::ScaleMinZ || handle == HandleType::ScaleMaxZ;
}

bool isTranslateHandle(HandleType handle)
{
    return handle == HandleType::TranslateX ||
           handle == HandleType::TranslateY ||
           handle == HandleType::TranslateZ;
}

bool isRotateHandle(HandleType handle)
{
    return handle == HandleType::RotateMinX || handle == HandleType::RotateMaxX ||
           handle == HandleType::RotateMinY || handle == HandleType::RotateMaxY ||
           handle == HandleType::RotateMinZ || handle == HandleType::RotateMaxZ;
}

QVector3D colorForHandle(HandleType handle, bool highlighted)
{
    QVector3D color(1.0f, 0.82f, 0.15f);
    if (handle == HandleType::TranslateX || handle == HandleType::ScaleMinX || handle == HandleType::ScaleMaxX ||
        handle == HandleType::RotateMinX || handle == HandleType::RotateMaxX) {
        color = QVector3D(1.0f, 0.1f, 0.08f);
    } else if (handle == HandleType::TranslateY || handle == HandleType::ScaleMinY || handle == HandleType::ScaleMaxY ||
               handle == HandleType::RotateMinY || handle == HandleType::RotateMaxY) {
        color = QVector3D(0.1f, 0.9f, 0.15f);
    } else if (handle == HandleType::TranslateZ || handle == HandleType::ScaleMinZ || handle == HandleType::ScaleMaxZ ||
               handle == HandleType::RotateMinZ || handle == HandleType::RotateMaxZ) {
        color = QVector3D(0.15f, 0.45f, 1.0f);
    } else if (handle == HandleType::Center) {
        color = QVector3D(0.1f, 0.15f, 1.0f);
    }
    if (highlighted) {
        color = color * 0.35f + QVector3D(1.0f, 1.0f, 1.0f) * 0.65f;
    }
    return color;
}

int scaleAxisIndex(HandleType handle)
{
    switch (handle) {
    case HandleType::ScaleMinX:
    case HandleType::ScaleMaxX:
        return 0;
    case HandleType::ScaleMinY:
    case HandleType::ScaleMaxY:
        return 1;
    case HandleType::ScaleMinZ:
    case HandleType::ScaleMaxZ:
        return 2;
    default:
        return -1;
    }
}

QVector3D localAxis(const Box& box, int index)
{
    if (index == 0) {
        return box.orientation.rotatedVector(QVector3D(1.0f, 0.0f, 0.0f)).normalized();
    }
    if (index == 1) {
        return box.orientation.rotatedVector(QVector3D(0.0f, 1.0f, 0.0f)).normalized();
    }
    return box.orientation.rotatedVector(QVector3D(0.0f, 0.0f, 1.0f)).normalized();
}

float halfExtentComponent(const QVector3D& halfExtents, int index)
{
    if (index == 0) {
        return halfExtents.x();
    }
    if (index == 1) {
        return halfExtents.y();
    }
    return halfExtents.z();
}

void setHalfExtentComponent(QVector3D& halfExtents, int index, float value)
{
    if (index == 0) {
        halfExtents.setX(value);
    } else if (index == 1) {
        halfExtents.setY(value);
    } else {
        halfExtents.setZ(value);
    }
}

QVector3D worldFromLocal(const Box& box, const QVector3D& local)
{
    return box.center + box.orientation.rotatedVector(local);
}

QVector3D localFromWorld(const Box& box, const QVector3D& world)
{
    return box.orientation.conjugated().rotatedVector(world - box.center);
}

bool projectPoint(const Camera& camera, const QVector3D& world, QPointF& screen)
{
    const QVector4D clip = camera.projection * camera.modelView * QVector4D(world, 1.0f);
    if (clip.w() <= 0.0f || camera.viewport.width() <= 0 || camera.viewport.height() <= 0) {
        return false;
    }
    const QVector3D ndc = clip.toVector3DAffine();
    screen = QPointF((ndc.x() * 0.5f + 0.5f) * float(camera.viewport.width()),
                     (1.0f - (ndc.y() * 0.5f + 0.5f)) * float(camera.viewport.height()));
    return true;
}

QVector3D cameraAxis(const Camera& camera, const QVector3D& cameraDirection)
{
    const QMatrix4x4 inverse = camera.modelView.inverted();
    const QVector4D mapped = inverse * QVector4D(cameraDirection, 0.0f);
    return mapped.toVector3D().normalized();
}

float worldUnitsPerPixel(const Camera& camera, const QVector3D& world)
{
    QPointF centerScreen;
    QPointF rightScreen;
    QPointF upScreen;
    const QVector3D right = cameraAxis(camera, QVector3D(1.0f, 0.0f, 0.0f));
    const QVector3D up = cameraAxis(camera, QVector3D(0.0f, 1.0f, 0.0f));
    if (!projectPoint(camera, world, centerScreen) ||
        !projectPoint(camera, world + right, rightScreen) ||
        !projectPoint(camera, world + up, upScreen)) {
        return 0.0f;
    }

    const QPointF dr = rightScreen - centerScreen;
    const QPointF du = upScreen - centerScreen;
    const float rightPixels = std::sqrt(float(dr.x() * dr.x() + dr.y() * dr.y()));
    const float upPixels = std::sqrt(float(du.x() * du.x() + du.y() * du.y()));
    const float pixelsPerWorldUnit = std::max(rightPixels, upPixels);
    if (pixelsPerWorldUnit <= 0.0f) {
        return 0.0f;
    }
    return 1.0f / pixelsPerWorldUnit;
}

float worldSizeForPixels(const Camera& camera, const QVector3D& world, float pixels)
{
    return worldUnitsPerPixel(camera, world) * pixels;
}

float distanceToSegment(const QPointF& point, const QPointF& a, const QPointF& b)
{
    const QPointF ab = b - a;
    const float lenSq = float(ab.x() * ab.x() + ab.y() * ab.y());
    if (lenSq <= 1e-5f) {
        const QPointF d = point - a;
        return std::sqrt(float(d.x() * d.x() + d.y() * d.y()));
    }
    const QPointF ap = point - a;
    const float t = std::clamp(float((ap.x() * ab.x() + ap.y() * ab.y()) / lenSq), 0.0f, 1.0f);
    const QPointF closest = a + ab * t;
    const QPointF d = point - closest;
    return std::sqrt(float(d.x() * d.x() + d.y() * d.y()));
}

void appendLine(QVector<ColoredVertex>& vertices, const QVector3D& a, const QVector3D& b, const QVector3D& color)
{
    vertices.push_back({a.x(), a.y(), a.z(), color.x(), color.y(), color.z()});
    vertices.push_back({b.x(), b.y(), b.z(), color.x(), color.y(), color.z()});
}

void appendCross(QVector<ColoredVertex>& vertices,
                 const Box& box,
                 const QVector3D& center,
                 float radius,
                 const QVector3D& color)
{
    appendLine(vertices, center - localAxis(box, 0) * radius, center + localAxis(box, 0) * radius, color);
    appendLine(vertices, center - localAxis(box, 1) * radius, center + localAxis(box, 1) * radius, color);
    appendLine(vertices, center - localAxis(box, 2) * radius, center + localAxis(box, 2) * radius, color);
}

void appendArrowHead(QVector<ColoredVertex>& vertices,
                     const Box& box,
                     const QVector3D& tip,
                     const QVector3D& axis,
                     float size,
                     const QVector3D& color)
{
    QVector3D side = QVector3D::crossProduct(axis, localAxis(box, 2));
    if (side.lengthSquared() < 1e-5f) {
        side = QVector3D::crossProduct(axis, localAxis(box, 1));
    }
    side.normalize();
    const QVector3D base = tip - axis * size;
    appendLine(vertices, tip, base + side * size * 0.45f, color);
    appendLine(vertices, tip, base - side * size * 0.45f, color);
}

void appendCircle(QVector<ColoredVertex>& vertices,
                  const Box& box,
                  const QVector3D& center,
                  const QVector3D& normal,
                  float radius,
                  const QVector3D& color)
{
    QVector3D u;
    QVector3D v;
    if (std::abs(QVector3D::dotProduct(normal, localAxis(box, 0))) < 0.9f) {
        u = QVector3D::crossProduct(normal, localAxis(box, 0)).normalized();
    } else {
        u = QVector3D::crossProduct(normal, localAxis(box, 1)).normalized();
    }
    v = QVector3D::crossProduct(normal, u).normalized();

    constexpr int kSegments = 72;
    QVector3D previous = center + u * radius;
    for (int i = 1; i <= kSegments; ++i) {
        const float angle = (2.0f * kPi * float(i)) / float(kSegments);
        const QVector3D current = center + (u * std::cos(angle) + v * std::sin(angle)) * radius;
        appendLine(vertices, previous, current, color);
        previous = current;
    }
}

struct RingHandle {
    HandleType handle = HandleType::None;
    QVector3D center;
    QVector3D normal;
    float radius = 0.0f;
};

QVector<QPair<HandleType, QPair<QVector3D, QVector3D>>> handleSegments(const State& state, const Camera& camera)
{
    QVector<QPair<HandleType, QPair<QVector3D, QVector3D>>> segments;
    const Box& box = state.box;
    const QVector3D axes[] = {
        localAxis(box, 0),
        localAxis(box, 1),
        localAxis(box, 2)
    };
    const HandleType translateHandles[] = {
        HandleType::TranslateX,
        HandleType::TranslateY,
        HandleType::TranslateZ
    };
    const HandleType minHandles[] = {
        HandleType::ScaleMinX,
        HandleType::ScaleMinY,
        HandleType::ScaleMinZ
    };
    const HandleType maxHandles[] = {
        HandleType::ScaleMaxX,
        HandleType::ScaleMaxY,
        HandleType::ScaleMaxZ
    };

    for (int i = 0; i < 3; ++i) {
        const QVector3D axis = axes[i];
        const float extent = halfExtentComponent(box.halfExtents, i);
        const QVector3D maxFace = box.center + axis * extent;
        const QVector3D minFace = box.center - axis * extent;
        const float maxHandleLength = worldSizeForPixels(camera, maxFace, kScaleHandlePixels);
        const float minHandleLength = worldSizeForPixels(camera, minFace, kScaleHandlePixels);
        const float translateOffset = worldSizeForPixels(camera, maxFace, kTranslateOffsetPixels);
        const float translateLength = worldSizeForPixels(camera, maxFace, kTranslateLengthPixels);
        segments.push_back({translateHandles[i], {maxFace + axis * translateOffset,
                                                  maxFace + axis * (translateOffset + translateLength)}});

        segments.push_back({maxHandles[i], {maxFace, maxFace + axis * maxHandleLength}});
        segments.push_back({minHandles[i], {minFace, minFace - axis * minHandleLength}});
    }

    return segments;
}

QVector<RingHandle> rotationRings(const State& state, const Camera& camera)
{
    QVector<RingHandle> rings;
    const Box& box = state.box;
    const QVector3D axes[] = {
        localAxis(box, 0),
        localAxis(box, 1),
        localAxis(box, 2)
    };
    const HandleType minHandles[] = {
        HandleType::RotateMinX,
        HandleType::RotateMinY,
        HandleType::RotateMinZ
    };
    const HandleType maxHandles[] = {
        HandleType::RotateMaxX,
        HandleType::RotateMaxY,
        HandleType::RotateMaxZ
    };

    rings.reserve(6);
    for (int i = 0; i < 3; ++i) {
        const QVector3D axis = axes[i];
        const float extent = halfExtentComponent(box.halfExtents, i);
        const QVector3D maxFace = box.center + axis * extent;
        const QVector3D minFace = box.center - axis * extent;
        const QVector3D maxCenter = maxFace + axis * worldSizeForPixels(camera, maxFace, kScaleHandlePixels);
        const QVector3D minCenter = minFace - axis * worldSizeForPixels(camera, minFace, kScaleHandlePixels);
        rings.push_back({maxHandles[i], maxCenter, axis, worldSizeForPixels(camera, maxCenter, kRotationRingRadiusPixels)});
        rings.push_back({minHandles[i], minCenter, -axis, worldSizeForPixels(camera, minCenter, kRotationRingRadiusPixels)});
    }
    return rings;
}

RingHandle rotationRingForHandle(const State& state, const Camera& camera, HandleType handle)
{
    for (const RingHandle& ring : rotationRings(state, camera)) {
        if (ring.handle == handle) {
            return ring;
        }
    }
    return {};
}

void considerHit(HitResult& best, HandleType handle, float distance)
{
    if (distance <= kHitThresholdPixels &&
        (best.handle == HandleType::None || distance < best.distancePixels)) {
        best.handle = handle;
        best.distancePixels = distance;
    }
}

} // namespace

bool initializeBox(State& state, const QVector<PointCloudPoint>& points)
{
    if (points.isEmpty()) {
        state.initialized = false;
        state.sourcePoints.clear();
        state.clippedPoints.clear();
        return false;
    }

    QVector3D minPoint(points.first().x, points.first().y, points.first().z);
    QVector3D maxPoint = minPoint;
    for (const PointCloudPoint& point : points) {
        minPoint.setX(std::min(minPoint.x(), point.x));
        minPoint.setY(std::min(minPoint.y(), point.y));
        minPoint.setZ(std::min(minPoint.z(), point.z));
        maxPoint.setX(std::max(maxPoint.x(), point.x));
        maxPoint.setY(std::max(maxPoint.y(), point.y));
        maxPoint.setZ(std::max(maxPoint.z(), point.z));
    }

    state.box.center = (minPoint + maxPoint) * 0.5f;
    state.box.halfExtents = (maxPoint - minPoint) * 0.5f;
    state.box.halfExtents.setX(std::max(kMinHalfExtent, state.box.halfExtents.x()));
    state.box.halfExtents.setY(std::max(kMinHalfExtent, state.box.halfExtents.y()));
    state.box.halfExtents.setZ(std::max(kMinHalfExtent, state.box.halfExtents.z()));
    state.box.orientation = QQuaternion();
    state.enabled = true;
    state.initialized = true;
    state.sourcePoints = points;
    state.clippedPoints = points;
    state.dragging = false;
    state.activeHandle = HandleType::None;
    return true;
}

void setSourcePoints(State& state, const QVector<PointCloudPoint>& points)
{
    state.sourcePoints = points;
    if (state.enabled && state.initialized) {
        updateClip(state);
    }
}

QVector<PointCloudPoint> clip(const QVector<PointCloudPoint>& points, const State& state)
{
    if (!state.enabled || !state.initialized) {
        return points;
    }

    QVector<PointCloudPoint> clipped;
    clipped.reserve(points.size());
    const QVector3D half = state.box.halfExtents;
    for (const PointCloudPoint& point : points) {
        const QVector3D local = localFromWorld(state.box, QVector3D(point.x, point.y, point.z));
        if (std::abs(local.x()) <= half.x() &&
            std::abs(local.y()) <= half.y() &&
            std::abs(local.z()) <= half.z()) {
            clipped.push_back(point);
        }
    }
    return clipped;
}

void updateClip(State& state)
{
    state.clippedPoints = clip(state.sourcePoints, state);
}

HitResult hitTest(const State& state, const Camera& camera, const QPoint& mousePos)
{
    HitResult best;
    if (!state.enabled || !state.initialized || !state.controlsVisible) {
        return best;
    }

    const QPointF mouse(mousePos);
    QPointF centerScreen;
    if (projectPoint(camera, state.box.center, centerScreen)) {
        const QPointF delta = mouse - centerScreen;
        const float distance = std::sqrt(float(delta.x() * delta.x() + delta.y() * delta.y()));
        if (distance <= kCenterHitRadiusPixels) {
            best.handle = HandleType::Center;
            best.distancePixels = distance;
        }
    }

    for (const auto& segment : handleSegments(state, camera)) {
        QPointF a;
        QPointF b;
        if (projectPoint(camera, segment.second.first, a) && projectPoint(camera, segment.second.second, b)) {
            considerHit(best, segment.first, distanceToSegment(mouse, a, b));
        }
    }

    constexpr int kSegments = 72;
    for (const RingHandle& ring : rotationRings(state, camera)) {
        QVector3D u;
        if (std::abs(QVector3D::dotProduct(ring.normal, localAxis(state.box, 0))) < 0.9f) {
            u = QVector3D::crossProduct(ring.normal, localAxis(state.box, 0)).normalized();
        } else {
            u = QVector3D::crossProduct(ring.normal, localAxis(state.box, 1)).normalized();
        }
        const QVector3D v = QVector3D::crossProduct(ring.normal, u).normalized();
        QVector3D previous = ring.center + u * ring.radius;
        for (int i = 1; i <= kSegments; ++i) {
            const float angle = (2.0f * kPi * float(i)) / float(kSegments);
            const QVector3D current = ring.center + (u * std::cos(angle) + v * std::sin(angle)) * ring.radius;
            QPointF a;
            QPointF b;
            if (projectPoint(camera, previous, a) && projectPoint(camera, current, b)) {
                considerHit(best, ring.handle, distanceToSegment(mouse, a, b));
            }
            previous = current;
        }
    }

    return best;
}

bool updateHover(State& state, const Camera& camera, const QPoint& mousePos)
{
    const HandleType previous = state.hoverHandle;
    state.hoverHandle = hitTest(state, camera, mousePos).handle;
    return previous != state.hoverHandle;
}

bool beginDrag(State& state, const Camera& camera, const QPoint& mousePos)
{
    const HitResult hit = hitTest(state, camera, mousePos);
    if (hit.handle == HandleType::None) {
        return false;
    }

    state.activeHandle = hit.handle;
    state.dragging = true;
    state.dragStart = mousePos;
    state.dragStartBox = state.box;
    state.dragAxis = axisForHandle(state.box, hit.handle);
    state.dragCameraRight = cameraAxis(camera, QVector3D(1.0f, 0.0f, 0.0f));
    state.dragCameraUp = cameraAxis(camera, QVector3D(0.0f, 1.0f, 0.0f));
    state.dragUnitsPerPixel = worldUnitsPerPixel(camera, state.box.center);
    QPointF centerScreen;
    QPointF axisScreen;
    const float guideLength = worldSizeForPixels(camera, state.box.center, 80.0f);
    if (isRotateHandle(hit.handle)) {
        const RingHandle ring = rotationRingForHandle(state, camera, hit.handle);
        projectPoint(camera, ring.center, state.dragRingCenterScreen);
        const QVector3D viewDirection = cameraAxis(camera, QVector3D(0.0f, 0.0f, -1.0f));
        state.dragRotationSign = QVector3D::dotProduct(state.dragAxis, viewDirection) > 0.0f ? -1.0f : 1.0f;
    } else if (projectPoint(camera, state.box.center, centerScreen) &&
               projectPoint(camera, state.box.center + state.dragAxis * guideLength, axisScreen)) {
        const QPointF screenVector = axisScreen - centerScreen;
        const float screenLength = std::sqrt(float(screenVector.x() * screenVector.x() + screenVector.y() * screenVector.y()));
        if (screenLength > 1.0f) {
            state.dragAxisScreen = QPointF(screenVector.x() / screenLength, screenVector.y() / screenLength);
            state.dragUnitsPerPixel = guideLength / screenLength;
        }
    }
    return true;
}

bool updateDrag(State& state, const Camera& camera, const QPoint& mousePos)
{
    Q_UNUSED(camera);
    if (!state.dragging || state.activeHandle == HandleType::None) {
        return false;
    }

    const QPointF delta = QPointF(mousePos - state.dragStart);
    const float alongPixels = float(delta.x() * state.dragAxisScreen.x() + delta.y() * state.dragAxisScreen.y());
    const float units = alongPixels * state.dragUnitsPerPixel;

    if (state.activeHandle == HandleType::Center) {
        state.box = state.dragStartBox;
        state.box.center = state.dragStartBox.center +
                           state.dragCameraRight * (float(delta.x()) * state.dragUnitsPerPixel) -
                           state.dragCameraUp * (float(delta.y()) * state.dragUnitsPerPixel);
    } else if (isTranslateHandle(state.activeHandle)) {
        state.box.center = state.dragStartBox.center + state.dragAxis * units;
    } else if (isScaleHandle(state.activeHandle)) {
        const int index = scaleAxisIndex(state.activeHandle);
        const float oldHalf = halfExtentComponent(state.dragStartBox.halfExtents, index);
        const float proposedHalf = std::max(kMinHalfExtent, oldHalf + units * 0.5f);
        const float faceDelta = (proposedHalf - oldHalf) * 2.0f;
        state.box = state.dragStartBox;
        setHalfExtentComponent(state.box.halfExtents, index, proposedHalf);
        state.box.center = state.dragStartBox.center + state.dragAxis * (faceDelta * 0.5f);
    } else if (isRotateHandle(state.activeHandle)) {
        const QPointF startVector = QPointF(state.dragStart) - state.dragRingCenterScreen;
        const QPointF currentVector = QPointF(mousePos) - state.dragRingCenterScreen;
        const float cross = float(startVector.x() * currentVector.y() - startVector.y() * currentVector.x());
        const float dot = float(startVector.x() * currentVector.x() + startVector.y() * currentVector.y());
        const float angleDegrees = -std::atan2(cross, dot) * 180.0f / kPi * state.dragRotationSign;
        state.box = state.dragStartBox;
        state.box.orientation = (QQuaternion::fromAxisAndAngle(state.dragAxis, angleDegrees) *
                                 state.dragStartBox.orientation).normalized();
    }

    updateClip(state);
    return true;
}

void endDrag(State& state)
{
    state.dragging = false;
    state.activeHandle = HandleType::None;
}

void setControlsVisible(State& state, bool visible)
{
    state.controlsVisible = visible;
    if (!visible) {
        state.dragging = false;
        state.activeHandle = HandleType::None;
        state.hoverHandle = HandleType::None;
    }
}

QVector<ColoredVertex> buildBoxLines(const State& state)
{
    QVector<ColoredVertex> vertices;
    if (!state.enabled || !state.initialized || !state.controlsVisible) {
        return vertices;
    }

    const QVector3D h = state.box.halfExtents;
    QVector<QVector3D> corners;
    corners.reserve(8);
    for (float x : {-h.x(), h.x()}) {
        for (float y : {-h.y(), h.y()}) {
            for (float z : {-h.z(), h.z()}) {
                corners.push_back(worldFromLocal(state.box, QVector3D(x, y, z)));
            }
        }
    }

    const QVector3D boxColor(1.0f, 0.82f, 0.15f);
    const int edges[][2] = {
        {0, 1}, {0, 2}, {0, 4}, {3, 1}, {3, 2}, {3, 7},
        {5, 1}, {5, 4}, {5, 7}, {6, 2}, {6, 4}, {6, 7}
    };
    for (const auto& edge : edges) {
        appendLine(vertices, corners[edge[0]], corners[edge[1]], boxColor);
    }
    return vertices;
}

QVector<ColoredVertex> buildGizmoLines(const State& state, const Camera& camera)
{
    QVector<ColoredVertex> vertices;
    if (!state.enabled || !state.initialized || !state.controlsVisible) {
        return vertices;
    }

    const auto segments = handleSegments(state, camera);
    for (const auto& segment : segments) {
        const bool highlighted = state.hoverHandle == segment.first || state.activeHandle == segment.first;
        const QVector3D color = colorForHandle(segment.first, highlighted);
        appendLine(vertices, segment.second.first, segment.second.second, color);
        if (isTranslateHandle(segment.first)) {
            appendArrowHead(vertices,
                            state.box,
                            segment.second.second,
                            (segment.second.second - segment.second.first).normalized(),
                            worldSizeForPixels(camera, segment.second.second, kArrowHeadPixels),
                            color);
        } else if (isScaleHandle(segment.first)) {
            appendCross(vertices,
                        state.box,
                        segment.second.second,
                        worldSizeForPixels(camera, segment.second.second, kScaleCrossPixels),
                        color);
        }
    }

    for (const RingHandle& ring : rotationRings(state, camera)) {
        appendCircle(vertices,
                     state.box,
                     ring.center,
                     ring.normal,
                     ring.radius,
                     colorForHandle(ring.handle, state.hoverHandle == ring.handle || state.activeHandle == ring.handle));
    }
    appendCross(vertices,
                state.box,
                state.box.center,
                worldSizeForPixels(camera, state.box.center, kCenterCrossPixels),
                colorForHandle(HandleType::Center, state.hoverHandle == HandleType::Center || state.activeHandle == HandleType::Center));
    const QVector3D centerColor = colorForHandle(HandleType::Center, state.hoverHandle == HandleType::Center || state.activeHandle == HandleType::Center);
    const float centerRadius = worldSizeForPixels(camera, state.box.center, kCenterRingPixels);
    appendCircle(vertices, state.box, state.box.center, localAxis(state.box, 0), centerRadius, centerColor);
    appendCircle(vertices, state.box, state.box.center, localAxis(state.box, 1), centerRadius, centerColor);
    appendCircle(vertices, state.box, state.box.center, localAxis(state.box, 2), centerRadius, centerColor);
    return vertices;
}

} // namespace PointCloudCrossSection

#include "PointCloudCrossSection.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QVector4D>

namespace PointCloudCrossSection {
namespace {

constexpr float kMinHalfExtent = 0.05f;
constexpr float kHitThresholdPixels = 18.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kCenterHitRadiusPixels = 24.0f;
constexpr float kTranslateGapFactor = 0.12f;
constexpr float kTranslateLengthFactor = 0.20f;
constexpr float kScaleHandleFactor = 0.32f;
constexpr float kRotationRingRadiusFactor = 0.18f;
constexpr float kArrowHeadFactor = 0.10f;
constexpr float kScaleCrossFactor = 0.07f;
constexpr float kCenterCrossFactor = 0.10f;
constexpr float kCenterRingFactor = 0.12f;
constexpr float kDragGuideFactor = 0.50f;
constexpr float kRotationSensitivity = 0.35f;

QVector3D axisForHandle(const Box& box, HandleType handle)
{
    switch (handle) {
    case HandleType::Center:
        return {};
    case HandleType::TranslateX:
    case HandleType::ScaleMaxX:
    case HandleType::RotateMaxX:
        return box.orientation.rotatedVector(QVector3D(1.0f, 0.0f, 0.0f)).normalized();
    case HandleType::TranslateMinX:
    case HandleType::ScaleMinX:
    case HandleType::RotateMinX:
        return box.orientation.rotatedVector(QVector3D(-1.0f, 0.0f, 0.0f)).normalized();
    case HandleType::TranslateY:
    case HandleType::ScaleMaxY:
    case HandleType::RotateMaxY:
        return box.orientation.rotatedVector(QVector3D(0.0f, 1.0f, 0.0f)).normalized();
    case HandleType::TranslateMinY:
    case HandleType::ScaleMinY:
    case HandleType::RotateMinY:
        return box.orientation.rotatedVector(QVector3D(0.0f, -1.0f, 0.0f)).normalized();
    case HandleType::TranslateZ:
    case HandleType::ScaleMaxZ:
    case HandleType::RotateMaxZ:
        return box.orientation.rotatedVector(QVector3D(0.0f, 0.0f, 1.0f)).normalized();
    case HandleType::TranslateMinZ:
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

float boxScaledSize(const Box& box, float factor)
{
    return maxHalfExtent(box) * factor;
}

bool isScaleHandle(HandleType handle)
{
    return handle == HandleType::ScaleMinX || handle == HandleType::ScaleMaxX ||
           handle == HandleType::ScaleMinY || handle == HandleType::ScaleMaxY ||
           handle == HandleType::ScaleMinZ || handle == HandleType::ScaleMaxZ;
}

bool isTranslateHandle(HandleType handle)
{
    return handle == HandleType::TranslateMinX || handle == HandleType::TranslateX ||
           handle == HandleType::TranslateMinY || handle == HandleType::TranslateY ||
           handle == HandleType::TranslateMinZ || handle == HandleType::TranslateZ;
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
    if (handle == HandleType::TranslateMinX || handle == HandleType::TranslateX ||
        handle == HandleType::ScaleMinX || handle == HandleType::ScaleMaxX ||
        handle == HandleType::RotateMinX || handle == HandleType::RotateMaxX) {
        color = QVector3D(1.0f, 0.1f, 0.08f);
    } else if (handle == HandleType::TranslateMinY || handle == HandleType::TranslateY ||
               handle == HandleType::ScaleMinY || handle == HandleType::ScaleMaxY ||
               handle == HandleType::RotateMinY || handle == HandleType::RotateMaxY) {
        color = QVector3D(0.1f, 0.9f, 0.15f);
    } else if (handle == HandleType::TranslateMinZ || handle == HandleType::TranslateZ ||
               handle == HandleType::ScaleMinZ || handle == HandleType::ScaleMaxZ ||
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
    case HandleType::TranslateMinX:
    case HandleType::TranslateX:
    case HandleType::ScaleMinX:
    case HandleType::ScaleMaxX:
        return 0;
    case HandleType::TranslateMinY:
    case HandleType::TranslateY:
    case HandleType::ScaleMinY:
    case HandleType::ScaleMaxY:
        return 1;
    case HandleType::TranslateMinZ:
    case HandleType::TranslateZ:
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

void appendVertex(QVector<ColoredVertex>& vertices, const QVector3D& point, const QVector3D& color)
{
    vertices.push_back({point.x(), point.y(), point.z(), color.x(), color.y(), color.z()});
}

void appendCone(QVector<ColoredVertex>& vertices,
                const Box& box,
                const QVector3D& tip,
                const QVector3D& axis,
                float length,
                const QVector3D& color)
{
    QVector3D u = QVector3D::crossProduct(axis, localAxis(box, 2));
    if (u.lengthSquared() < 1e-5f) {
        u = QVector3D::crossProduct(axis, localAxis(box, 1));
    }
    u.normalize();
    const QVector3D v = QVector3D::crossProduct(axis, u).normalized();
    const QVector3D baseCenter = tip - axis * length;
    const float radius = length * 0.42f;

    constexpr int kSegments = 24;
    QVector<QVector3D> base;
    base.reserve(kSegments);
    for (int i = 0; i < kSegments; ++i) {
        const float angle = (2.0f * kPi * float(i)) / float(kSegments);
        base.push_back(baseCenter + (u * std::cos(angle) + v * std::sin(angle)) * radius);
    }

    for (int i = 0; i < kSegments; ++i) {
        const QVector3D a = base[i];
        const QVector3D b = base[(i + 1) % kSegments];
        const float shade = 0.72f + 0.28f * std::abs(std::cos((2.0f * kPi * float(i)) / float(kSegments)));
        const QVector3D sideColor = color * shade;
        appendVertex(vertices, tip, sideColor);
        appendVertex(vertices, a, sideColor);
        appendVertex(vertices, b, sideColor);
    }

    const QVector3D baseColor = color * 0.65f;
    for (int i = 0; i < kSegments; ++i) {
        appendVertex(vertices, baseCenter, baseColor);
        appendVertex(vertices, base[(i + 1) % kSegments], baseColor);
        appendVertex(vertices, base[i], baseColor);
    }
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
    Q_UNUSED(camera);
    QVector<QPair<HandleType, QPair<QVector3D, QVector3D>>> segments;
    const Box& box = state.box;
    const QVector3D axes[] = {
        localAxis(box, 0),
        localAxis(box, 1),
        localAxis(box, 2)
    };
    const HandleType minTranslateHandles[] = {
        HandleType::TranslateMinX,
        HandleType::TranslateMinY,
        HandleType::TranslateMinZ
    };
    const HandleType maxTranslateHandles[] = {
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
        const float handleLength = boxScaledSize(box, kScaleHandleFactor);
        const float translateOffset = handleLength + boxScaledSize(box, kTranslateGapFactor);
        const float translateLength = boxScaledSize(box, kTranslateLengthFactor);
        segments.push_back({maxTranslateHandles[i], {maxFace + axis * translateOffset,
                                                     maxFace + axis * (translateOffset + translateLength)}});
        segments.push_back({minTranslateHandles[i], {minFace - axis * translateOffset,
                                                     minFace - axis * (translateOffset + translateLength)}});

        segments.push_back({maxHandles[i], {maxFace, maxFace + axis * handleLength}});
        segments.push_back({minHandles[i], {minFace, minFace - axis * handleLength}});
    }

    return segments;
}

QVector<RingHandle> rotationRings(const State& state, const Camera& camera)
{
    Q_UNUSED(camera);
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
        const float handleLength = boxScaledSize(box, kScaleHandleFactor);
        const float ringRadius = boxScaledSize(box, kRotationRingRadiusFactor);
        const QVector3D maxCenter = maxFace + axis * handleLength;
        const QVector3D minCenter = minFace - axis * handleLength;
        rings.push_back({maxHandles[i], maxCenter, axis, ringRadius});
        rings.push_back({minHandles[i], minCenter, -axis, ringRadius});
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

void initializeBoxGeometry(State& state, const QVector3D& minPoint, const QVector3D& maxPoint)
{
    state.box.center = (minPoint + maxPoint) * 0.5f;
    state.box.halfExtents = (maxPoint - minPoint) * 0.5f;
    state.box.halfExtents.setX(std::max(kMinHalfExtent, state.box.halfExtents.x()));
    state.box.halfExtents.setY(std::max(kMinHalfExtent, state.box.halfExtents.y()));
    state.box.halfExtents.setZ(std::max(kMinHalfExtent, state.box.halfExtents.z()));
    state.box.orientation = QQuaternion();
    state.enabled = true;
    state.initialized = true;
    state.dragging = false;
    state.activeHandle = HandleType::None;
    state.hoverHandle = HandleType::None;
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

    initializeBoxGeometry(state, minPoint, maxPoint);
    state.sourcePoints = points;
    state.clippedPoints = points;
    return true;
}

bool initializeBoxFromBounds(State& state, const QVector3D& minPoint, const QVector3D& maxPoint)
{
    initializeBoxGeometry(state, minPoint, maxPoint);
    state.sourcePoints.clear();
    state.clippedPoints.clear();
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
    const QVector3D center = state.box.center;
    const QVector3D half = state.box.halfExtents;
    const QVector3D axisX = state.box.orientation.rotatedVector(QVector3D(1.0f, 0.0f, 0.0f)).normalized();
    const QVector3D axisY = state.box.orientation.rotatedVector(QVector3D(0.0f, 1.0f, 0.0f)).normalized();
    const QVector3D axisZ = state.box.orientation.rotatedVector(QVector3D(0.0f, 0.0f, 1.0f)).normalized();
    for (const PointCloudPoint& point : points) {
        const QVector3D delta(point.x - center.x(), point.y - center.y(), point.z - center.z());
        const float localX = QVector3D::dotProduct(delta, axisX);
        const float localY = QVector3D::dotProduct(delta, axisY);
        const float localZ = QVector3D::dotProduct(delta, axisZ);
        if (std::abs(localX) <= half.x() &&
            std::abs(localY) <= half.y() &&
            std::abs(localZ) <= half.z()) {
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
    state.dragPrevious = mousePos;
    state.dragStartBox = state.box;
    state.dragAxis = axisForHandle(state.box, hit.handle);
    state.dragCameraRight = cameraAxis(camera, QVector3D(1.0f, 0.0f, 0.0f));
    state.dragCameraUp = cameraAxis(camera, QVector3D(0.0f, 1.0f, 0.0f));
    state.dragUnitsPerPixel = worldUnitsPerPixel(camera, state.box.center);
    state.dragRotationDegrees = 0.0f;
    QPointF centerScreen;
    QPointF axisScreen;
    const float guideLength = boxScaledSize(state.box, kDragGuideFactor);
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
        const int index = scaleAxisIndex(state.activeHandle);
        const float oldHalf = halfExtentComponent(state.dragStartBox.halfExtents, index);
        const float proposedHalf = std::max(kMinHalfExtent, oldHalf + units * 0.5f);
        const float faceDelta = (proposedHalf - oldHalf) * 2.0f;
        state.box = state.dragStartBox;
        setHalfExtentComponent(state.box.halfExtents, index, proposedHalf);
        state.box.center = state.dragStartBox.center + state.dragAxis * (faceDelta * 0.5f);
    } else if (isScaleHandle(state.activeHandle)) {
        state.box.center = state.dragStartBox.center + state.dragAxis * units;
    } else if (isRotateHandle(state.activeHandle)) {
        const QPointF startVector = QPointF(state.dragPrevious) - state.dragRingCenterScreen;
        const QPointF currentVector = QPointF(mousePos) - state.dragRingCenterScreen;
        const float cross = float(startVector.x() * currentVector.y() - startVector.y() * currentVector.x());
        const float dot = float(startVector.x() * currentVector.x() + startVector.y() * currentVector.y());
        state.dragRotationDegrees += -std::atan2(cross, dot) * 180.0f / kPi * state.dragRotationSign * kRotationSensitivity;
        state.dragPrevious = mousePos;
        state.box = state.dragStartBox;
        state.box.orientation = (QQuaternion::fromAxisAndAngle(state.dragAxis, state.dragRotationDegrees) *
                                 state.dragStartBox.orientation).normalized();
    }

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
        if (isTranslateHandle(segment.first)) {
            const QVector3D axis = (segment.second.second - segment.second.first).normalized();
            const float coneLength = boxScaledSize(state.box, kArrowHeadFactor);
            appendLine(vertices, segment.second.first, segment.second.second - axis * coneLength, color);
        } else if (isScaleHandle(segment.first)) {
            appendLine(vertices, segment.second.first, segment.second.second, color);
            appendCross(vertices,
                        state.box,
                        segment.second.second,
                        boxScaledSize(state.box, kScaleCrossFactor),
                        color);
        } else {
            appendLine(vertices, segment.second.first, segment.second.second, color);
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
                boxScaledSize(state.box, kCenterCrossFactor),
                colorForHandle(HandleType::Center, state.hoverHandle == HandleType::Center || state.activeHandle == HandleType::Center));
    const QVector3D centerColor = colorForHandle(HandleType::Center, state.hoverHandle == HandleType::Center || state.activeHandle == HandleType::Center);
    const float centerRadius = boxScaledSize(state.box, kCenterRingFactor);
    appendCircle(vertices, state.box, state.box.center, localAxis(state.box, 0), centerRadius, centerColor);
    appendCircle(vertices, state.box, state.box.center, localAxis(state.box, 1), centerRadius, centerColor);
    appendCircle(vertices, state.box, state.box.center, localAxis(state.box, 2), centerRadius, centerColor);
    return vertices;
}

QVector<ColoredVertex> buildGizmoTriangles(const State& state, const Camera& camera)
{
    QVector<ColoredVertex> vertices;
    if (!state.enabled || !state.initialized || !state.controlsVisible) {
        return vertices;
    }

    for (const auto& segment : handleSegments(state, camera)) {
        if (!isTranslateHandle(segment.first)) {
            continue;
        }
        const bool highlighted = state.hoverHandle == segment.first || state.activeHandle == segment.first;
        const QVector3D color = colorForHandle(segment.first, highlighted);
        const QVector3D axis = (segment.second.second - segment.second.first).normalized();
        appendCone(vertices,
                   state.box,
                   segment.second.second,
                   axis,
                   boxScaledSize(state.box, kArrowHeadFactor),
                   color);
    }
    return vertices;
}

} // namespace PointCloudCrossSection

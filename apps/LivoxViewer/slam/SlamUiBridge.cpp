#include "slam/SlamUiBridge.h"

#include <QCoreApplication>
#include <QQuaternion>
#include <QThread>
#include <QVector3D>

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace {

constexpr int kMaxTrajectoryPoints = 200000;

QString statusName(SlamStatusCode status)
{
    switch (status) {
    case SlamStatusCode::Idle: return QStringLiteral("Idle");
    case SlamStatusCode::Starting: return QStringLiteral("Starting");
    case SlamStatusCode::InitializingImu: return QStringLiteral("Initializing IMU");
    case SlamStatusCode::Running: return QStringLiteral("Running");
    case SlamStatusCode::Paused: return QStringLiteral("Paused");
    case SlamStatusCode::Backpressure: return QStringLiteral("Backpressure");
    case SlamStatusCode::MissingImu: return QStringLiteral("Missing IMU");
    case SlamStatusCode::TimeSyncError: return QStringLiteral("Time sync error");
    case SlamStatusCode::Degraded: return QStringLiteral("Degraded");
    case SlamStatusCode::Failed: return QStringLiteral("Failed");
    case SlamStatusCode::Stopped: return QStringLiteral("Stopped");
    }
    return QStringLiteral("Unknown");
}

QString dynamicBackendName(DynamicFilterBackend backend)
{
    switch (backend) {
    case DynamicFilterBackend::Disabled: return QStringLiteral("关闭");
    case DynamicFilterBackend::MDetector: return QStringLiteral("M-detector");
    case DynamicFilterBackend::FreeDOM: return QStringLiteral("FreeDOM");
    }
    return QStringLiteral("关闭");
}

SlamRenderVertex renderVertex(const QVector3D& point, float r, float g, float b)
{
    return {point.x(), point.y(), point.z(), r, g, b};
}

SlamRenderVertex renderSlamPoint(const SlamPoint& point, const QColor& color)
{
    return {point.x, point.y, point.z, color.redF(), color.greenF(), color.blueF()};
}

SlamRenderVertex renderDynamicPoint(const SlamDynamicPoint& point, const QColor& color)
{
    return {point.x, point.y, point.z, color.redF(), color.greenF(), color.blueF()};
}

QVector3D posePosition(const SlamPose& pose)
{
    return QVector3D(float(pose.tx), float(pose.ty), float(pose.tz));
}

Eigen::Isometry3d poseTransform(const SlamPose& pose)
{
    Eigen::Quaterniond rotation(pose.qw, pose.qx, pose.qy, pose.qz);
    rotation.normalize();
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = rotation.toRotationMatrix();
    transform.translation() = Eigen::Vector3d(pose.tx, pose.ty, pose.tz);
    return transform;
}

void setPoseTransform(SlamPose& pose, const Eigen::Isometry3d& transform)
{
    const Eigen::Quaterniond rotation(transform.rotation());
    pose.tx = transform.translation().x();
    pose.ty = transform.translation().y();
    pose.tz = transform.translation().z();
    pose.qx = rotation.x();
    pose.qy = rotation.y();
    pose.qz = rotation.z();
    pose.qw = rotation.w();
}

SlamRenderPose renderPose(const SlamPose& pose, bool valid)
{
    SlamRenderPose result;
    result.valid = valid;
    result.tx = float(pose.tx);
    result.ty = float(pose.ty);
    result.tz = float(pose.tz);
    result.qx = float(pose.qx);
    result.qy = float(pose.qy);
    result.qz = float(pose.qz);
    result.qw = float(pose.qw);
    return result;
}

QString formatPose(const SlamPose& pose)
{
    return QStringLiteral("t=[%1, %2, %3]\nq=[%4, %5, %6, %7]")
        .arg(pose.tx, 0, 'f', 3)
        .arg(pose.ty, 0, 'f', 3)
        .arg(pose.tz, 0, 'f', 3)
        .arg(pose.qx, 0, 'f', 4)
        .arg(pose.qy, 0, 'f', 4)
        .arg(pose.qz, 0, 'f', 4)
        .arg(pose.qw, 0, 'f', 4);
}

bool isErrorStatus(SlamStatusCode status)
{
    return status == SlamStatusCode::Failed ||
           status == SlamStatusCode::MissingImu ||
           status == SlamStatusCode::TimeSyncError;
}

QString appendDetail(const QString& hint, const QString& detail)
{
    return detail.isEmpty() ? hint : hint + QStringLiteral(" 详细信息: ") + detail;
}

QString errorDisplayMessage(const SlamOutput& output)
{
    switch (output.status) {
    case SlamStatusCode::MissingImu:
        return appendDetail(QStringLiteral("缺少 IMU 数据或 IMU 覆盖不完整。请确认 PCAP 包含 Livox IMU payload，并且点云帧时间范围内有连续 IMU 样本。"),
                            output.message);
    case SlamStatusCode::TimeSyncError:
        return appendDetail(QStringLiteral("点云与 IMU 时间戳不同步，或缺少可用的点内/包内采样时间。请确认使用原始时间戳录制，且 IMU 与点云时间范围重叠。"),
                            output.message);
    case SlamStatusCode::Failed:
        return output.message.isEmpty()
            ? QStringLiteral("SLAM 后端启动或处理失败。请检查输入数据、IMU、时间戳和 LiDAR-IMU 外参配置。")
            : output.message;
    default:
        return output.message;
    }
}

float clampOverlayPointSize(float sizePx)
{
    return std::clamp(sizePx, 1.0f, 10.0f);
}

float clampOverlayLineWidth(float widthPx)
{
    return std::clamp(widthPx, 1.0f, 10.0f);
}

float clampPoseAxisLength(float lengthM)
{
    return std::clamp(lengthM, 0.1f, 10.0f);
}

float poseAxisDiameterFromLineWidth(float widthPx)
{
    return std::clamp(widthPx * 0.02f, 0.01f, 0.3f);
}

QString formatMemoryBytes(quint64 bytes)
{
    constexpr double kKb = 1024.0;
    constexpr double kMb = kKb * 1024.0;
    constexpr double kGb = kMb * 1024.0;
    const double value = double(bytes);
    if (value >= kGb) {
        return QStringLiteral("%1 GB").arg(value / kGb, 0, 'f', 2);
    }
    if (value >= kMb) {
        return QStringLiteral("%1 MB").arg(value / kMb, 0, 'f', 1);
    }
    if (value >= kKb) {
        return QStringLiteral("%1 KB").arg(value / kKb, 0, 'f', 1);
    }
    return QStringLiteral("%1 B").arg(bytes);
}

quint64 vectorBytes(qsizetype size, qsizetype elementSize)
{
    if (size <= 0 || elementSize <= 0) {
        return 0;
    }
    return quint64(size) * quint64(elementSize);
}

void appendTriangle(QVector<SlamRenderVertex>& vertices,
                    const QVector3D& a,
                    const QVector3D& b,
                    const QVector3D& c,
                    float r,
                    float g,
                    float bColor)
{
    vertices.push_back(renderVertex(a, r, g, bColor));
    vertices.push_back(renderVertex(b, r, g, bColor));
    vertices.push_back(renderVertex(c, r, g, bColor));
}

void appendPoseAxisMesh(QVector<SlamRenderVertex>& vertices,
                        const QVector3D& origin,
                        const QVector3D& direction,
                        float cylinderLength,
                        float diameter,
                        float r,
                        float g,
                        float b)
{
    constexpr int kSegments = 16;
    constexpr float kPi = 3.14159265358979323846f;
    const QVector3D axis = direction.normalized();
    const QVector3D reference = std::abs(QVector3D::dotProduct(axis, QVector3D(0.0f, 0.0f, 1.0f))) > 0.9f
        ? QVector3D(0.0f, 1.0f, 0.0f)
        : QVector3D(0.0f, 0.0f, 1.0f);
    const QVector3D u = QVector3D::crossProduct(axis, reference).normalized();
    const QVector3D v = QVector3D::crossProduct(axis, u).normalized();
    const float radius = diameter * 0.5f;
    const QVector3D cylinderEnd = origin + axis * cylinderLength;

    for (int i = 0; i < kSegments; ++i) {
        const float a0 = 2.0f * kPi * float(i) / float(kSegments);
        const float a1 = 2.0f * kPi * float(i + 1) / float(kSegments);
        const QVector3D offset0 = (u * std::cos(a0) + v * std::sin(a0)) * radius;
        const QVector3D offset1 = (u * std::cos(a1) + v * std::sin(a1)) * radius;
        const QVector3D base0 = origin + offset0;
        const QVector3D base1 = origin + offset1;
        const QVector3D end0 = cylinderEnd + offset0;
        const QVector3D end1 = cylinderEnd + offset1;

        appendTriangle(vertices, base0, end0, base1, r, g, b);
        appendTriangle(vertices, base1, end0, end1, r, g, b);
    }
}

void appendPoseAxes(QVector<SlamRenderVertex>& vertices,
                    const SlamPose& pose,
                    float axisLength,
                    float diameter)
{
    const QVector3D origin = posePosition(pose);
    const QQuaternion rotation(float(pose.qw),
                               float(pose.qx),
                               float(pose.qy),
                               float(pose.qz));
    appendPoseAxisMesh(vertices, origin, rotation.rotatedVector(QVector3D(1.0f, 0.0f, 0.0f)),
                       axisLength, diameter, 1.0f, 0.05f, 0.05f);
    appendPoseAxisMesh(vertices, origin, rotation.rotatedVector(QVector3D(0.0f, 1.0f, 0.0f)),
                       axisLength, diameter, 0.05f, 1.0f, 0.05f);
    appendPoseAxisMesh(vertices, origin, rotation.rotatedVector(QVector3D(0.0f, 0.0f, 1.0f)),
                       axisLength, diameter, 0.1f, 0.35f, 1.0f);
}

} // namespace

SlamUiBridge::SlamUiBridge(QObject* parent)
    : QObject(parent)
{
    m_refreshTimer.setInterval(100);
    connect(&m_refreshTimer, &QTimer::timeout, this, &SlamUiBridge::refreshStatus);
    m_refreshTimer.start();
    refreshStatus();
}

void SlamUiBridge::receiveSlamOutput(const SlamOutput& output)
{
    Q_ASSERT(qApp);
    Q_ASSERT(QThread::currentThread() == qApp->thread());
    if (output.odometryOnly) {
        m_latestOutput.currentPose = output.currentPose;
        m_latestOutput.currentPoseValid = output.currentPoseValid;
        m_hasCurrentPose = output.currentPoseValid;
        if (output.currentPoseValid && !m_hasInitialPose) {
            m_initialPose = output.currentPose;
            m_hasInitialPose = true;
        }
        m_displayState.currentPose = formatPose(output.currentPose);
        emit displayStateChanged();
        emit renderPoseReady(renderPose(output.currentPose, m_hasCurrentPose));
        emit poseAxisVerticesReady(buildPoseAxisVertices());
        return;
    }
    const SlamPose retainedPose = m_latestOutput.currentPose;
    const bool retainedPoseValid = m_hasCurrentPose;
    m_latestOutput = output;
    if (output.freeDomDebugSnapshotUpdated) {
        m_freeDomScanVoxelPoints = output.freeDomScanVoxelPoints;
        m_freeDomDynamicVoxelPoints = output.freeDomDynamicVoxelPoints;
        m_freeDomRaycastedVoxelPoints = output.freeDomRaycastedVoxelPoints;
        m_freeDomEnhancedPoints = output.freeDomEnhancedPoints;
        const int rows = output.freeDomDepthImageRows;
        const int columns = output.freeDomDepthImageColumns;
        if (rows > 0 && columns > 0) {
            m_freeDomDepthImage = QImage(
                reinterpret_cast<const uchar*>(output.freeDomDepthImage.constData()),
                columns,
                rows,
                columns,
                QImage::Format_Grayscale8).copy();
            m_freeDomEnhancedDepthImage = QImage(
                reinterpret_cast<const uchar*>(output.freeDomEnhancedDepthImage.constData()),
                columns,
                rows,
                columns,
                QImage::Format_Grayscale8).copy();
        }
    }
    if (output.freeDomMapSnapshotUpdated) {
        m_freeDomFreeVoxelPoints = output.freeDomFreeVoxelPoints;
        m_freeDomStaticVoxelPoints = output.freeDomStaticVoxelPoints;
        m_freeDomStaticMapPoints = output.freeDomStaticMapPoints;
        if (!output.freeDomRaycastedVoxelPoints.isEmpty()) {
            m_freeDomRaycastedVoxelPoints = output.freeDomRaycastedVoxelPoints;
        }
    }
    if (output.currentPoseValid) {
        m_hasCurrentPose = true;
        if (!m_hasInitialPose) {
            m_initialPose = output.currentPose;
            m_hasInitialPose = true;
        }
    } else {
        m_latestOutput.currentPose = retainedPose;
        m_latestOutput.currentPoseValid = retainedPoseValid;
        m_hasCurrentPose = retainedPoseValid;
    }
    emit renderPoseReady(renderPose(m_latestOutput.currentPose, m_hasCurrentPose));
    if (isErrorStatus(output.status)) {
        m_errorMessage = errorDisplayMessage(output);
    } else if (output.status == SlamStatusCode::Starting ||
               output.status == SlamStatusCode::Running ||
               output.status == SlamStatusCode::Stopped ||
               output.status == SlamStatusCode::Idle) {
        m_errorMessage.clear();
    }
    appendTrajectory(output);
    appendGlobalMap(output);
    if (!output.loopClosureEdges.isEmpty()) {
        m_loopClosureEdges += output.loopClosureEdges;
    }
    if (output.status == SlamStatusCode::Running) {
        m_worldFramePointTotal += quint64(output.publishedWorldFramePoints.size());
    }
    m_latestOutput.newGlobalMapPoints.clear();
    m_latestOutput.optimizedTrajectory.clear();
    m_latestOutput.optimizedGlobalMapPoints.clear();
    m_latestOutput.loopClosureEdges.clear();
}

void SlamUiBridge::setWorldFrameColor(const QColor& color)
{
    if (!color.isValid() || m_worldFrameColor == color) {
        return;
    }
    m_worldFrameColor = color;
    refreshStatus();
}

void SlamUiBridge::setBodyFrameColor(const QColor& color)
{
    if (!color.isValid() || m_bodyFrameColor == color) {
        return;
    }
    m_bodyFrameColor = color;
    refreshStatus();
}

void SlamUiBridge::setDynamicObjectColor(const QColor& color)
{
    if (!color.isValid() || m_dynamicObjectColor == color) {
        return;
    }
    m_dynamicObjectColor = color;
    refreshStatus();
}

void SlamUiBridge::setDynamicAggressiveColor(const QColor& color)
{
    if (!color.isValid() || m_dynamicAggressiveColor == color) {
        return;
    }
    m_dynamicAggressiveColor = color;
    refreshStatus();
}

void SlamUiBridge::setDynamicModerateColor(const QColor& color)
{
    if (!color.isValid() || m_dynamicModerateColor == color) {
        return;
    }
    m_dynamicModerateColor = color;
    refreshStatus();
}

void SlamUiBridge::setDynamicConservativeColor(const QColor& color)
{
    if (!color.isValid() || m_dynamicConservativeColor == color) {
        return;
    }
    m_dynamicConservativeColor = color;
    refreshStatus();
}

void SlamUiBridge::setFreeDomScanVoxelColor(const QColor& color)
{
    if (!color.isValid() || m_freeDomScanVoxelColor == color) {
        return;
    }
    m_freeDomScanVoxelColor = color;
    refreshStatus();
}

void SlamUiBridge::setFreeDomDynamicVoxelColor(const QColor& color)
{
    if (!color.isValid() || m_freeDomDynamicVoxelColor == color) {
        return;
    }
    m_freeDomDynamicVoxelColor = color;
    refreshStatus();
}

void SlamUiBridge::setFreeDomRaycastedVoxelColor(const QColor& color)
{
    if (!color.isValid() || m_freeDomRaycastedVoxelColor == color) {
        return;
    }
    m_freeDomRaycastedVoxelColor = color;
    refreshStatus();
}

void SlamUiBridge::setFreeDomFreeVoxelColor(const QColor& color)
{
    if (!color.isValid() || m_freeDomFreeVoxelColor == color) {
        return;
    }
    m_freeDomFreeVoxelColor = color;
    refreshStatus();
}

void SlamUiBridge::setFreeDomStaticVoxelColor(const QColor& color)
{
    if (!color.isValid() || m_freeDomStaticVoxelColor == color) {
        return;
    }
    m_freeDomStaticVoxelColor = color;
    refreshStatus();
}

void SlamUiBridge::setFreeDomEnhancedColor(const QColor& color)
{
    if (!color.isValid() || m_freeDomEnhancedColor == color) {
        return;
    }
    m_freeDomEnhancedColor = color;
    refreshStatus();
}

void SlamUiBridge::setTrajectoryColor(const QColor& color)
{
    if (!color.isValid() || m_trajectoryColor == color) {
        return;
    }
    m_trajectoryColor = color;
    refreshStatus();
}

void SlamUiBridge::setWorldFramePointSize(float sizePx)
{
    const float clampedSize = clampOverlayPointSize(sizePx);
    if (m_worldFramePointSizePx == clampedSize) {
        return;
    }
    m_worldFramePointSizePx = clampedSize;
    refreshStatus();
}

void SlamUiBridge::setBodyFramePointSize(float sizePx)
{
    const float clampedSize = clampOverlayPointSize(sizePx);
    if (m_bodyFramePointSizePx == clampedSize) {
        return;
    }
    m_bodyFramePointSizePx = clampedSize;
    refreshStatus();
}

void SlamUiBridge::setDynamicObjectPointSize(float sizePx)
{
    const float clampedSize = clampOverlayPointSize(sizePx);
    if (m_dynamicObjectPointSizePx == clampedSize) {
        return;
    }
    m_dynamicObjectPointSizePx = clampedSize;
    refreshStatus();
}

void SlamUiBridge::setFreeDomScanVoxelPointSize(float sizePx)
{
    const float clampedSize = clampOverlayPointSize(sizePx);
    if (m_freeDomScanVoxelPointSizePx == clampedSize) {
        return;
    }
    m_freeDomScanVoxelPointSizePx = clampedSize;
    refreshStatus();
}

void SlamUiBridge::setFreeDomDynamicVoxelPointSize(float sizePx)
{
    const float clampedSize = clampOverlayPointSize(sizePx);
    if (m_freeDomDynamicVoxelPointSizePx == clampedSize) {
        return;
    }
    m_freeDomDynamicVoxelPointSizePx = clampedSize;
    refreshStatus();
}

void SlamUiBridge::setFreeDomRaycastedVoxelPointSize(float sizePx)
{
    const float clampedSize = clampOverlayPointSize(sizePx);
    if (m_freeDomRaycastedVoxelPointSizePx == clampedSize) {
        return;
    }
    m_freeDomRaycastedVoxelPointSizePx = clampedSize;
    refreshStatus();
}

void SlamUiBridge::setFreeDomFreeVoxelPointSize(float sizePx)
{
    const float clampedSize = clampOverlayPointSize(sizePx);
    if (m_freeDomFreeVoxelPointSizePx == clampedSize) {
        return;
    }
    m_freeDomFreeVoxelPointSizePx = clampedSize;
    refreshStatus();
}

void SlamUiBridge::setFreeDomStaticVoxelPointSize(float sizePx)
{
    const float clampedSize = clampOverlayPointSize(sizePx);
    if (m_freeDomStaticVoxelPointSizePx == clampedSize) {
        return;
    }
    m_freeDomStaticVoxelPointSizePx = clampedSize;
    refreshStatus();
}

void SlamUiBridge::setFreeDomEnhancedPointSize(float sizePx)
{
    const float clampedSize = clampOverlayPointSize(sizePx);
    if (m_freeDomEnhancedPointSizePx == clampedSize) {
        return;
    }
    m_freeDomEnhancedPointSizePx = clampedSize;
    refreshStatus();
}

void SlamUiBridge::setTrajectoryLineWidth(float widthPx)
{
    const float clampedWidth = clampOverlayLineWidth(widthPx);
    if (m_trajectoryLineWidthPx == clampedWidth) {
        return;
    }
    m_trajectoryLineWidthPx = clampedWidth;
    refreshStatus();
}

void SlamUiBridge::setPoseAxisLength(float lengthM)
{
    const float clampedLength = clampPoseAxisLength(lengthM);
    if (m_poseAxisLengthM == clampedLength) {
        return;
    }
    m_poseAxisLengthM = clampedLength;
    refreshStatus();
}

void SlamUiBridge::setPoseAxisLineWidth(float widthPx)
{
    const float clampedWidth = clampOverlayLineWidth(widthPx);
    if (m_poseAxisLineWidthPx == clampedWidth) {
        return;
    }
    m_poseAxisLineWidthPx = clampedWidth;
    refreshStatus();
}

void SlamUiBridge::setRenderLayerVisibility(bool trajectoryVisible,
                                            bool poseAxisVisible,
                                            bool worldFrameVisible,
                                            bool bodyFrameVisible,
                                            bool dynamicObjectVisible,
                                            bool freeDomScanVoxelVisible,
                                            bool freeDomDynamicVoxelVisible,
                                            bool freeDomRaycastedVoxelVisible,
                                            bool freeDomFreeVoxelVisible,
                                            bool freeDomStaticVoxelVisible,
                                            bool freeDomEnhancedVisible)
{
    if (m_trajectoryVisible == trajectoryVisible &&
        m_poseAxisVisible == poseAxisVisible &&
        m_worldFrameVisible == worldFrameVisible &&
        m_bodyFrameVisible == bodyFrameVisible &&
        m_dynamicObjectVisible == dynamicObjectVisible &&
        m_freeDomScanVoxelVisible == freeDomScanVoxelVisible &&
        m_freeDomDynamicVoxelVisible == freeDomDynamicVoxelVisible &&
        m_freeDomRaycastedVoxelVisible == freeDomRaycastedVoxelVisible &&
        m_freeDomFreeVoxelVisible == freeDomFreeVoxelVisible &&
        m_freeDomStaticVoxelVisible == freeDomStaticVoxelVisible &&
        m_freeDomEnhancedVisible == freeDomEnhancedVisible) {
        return;
    }
    m_trajectoryVisible = trajectoryVisible;
    m_poseAxisVisible = poseAxisVisible;
    m_worldFrameVisible = worldFrameVisible;
    m_bodyFrameVisible = bodyFrameVisible;
    m_dynamicObjectVisible = dynamicObjectVisible;
    m_freeDomScanVoxelVisible = freeDomScanVoxelVisible;
    m_freeDomDynamicVoxelVisible = freeDomDynamicVoxelVisible;
    m_freeDomRaycastedVoxelVisible = freeDomRaycastedVoxelVisible;
    m_freeDomFreeVoxelVisible = freeDomFreeVoxelVisible;
    m_freeDomStaticVoxelVisible = freeDomStaticVoxelVisible;
    m_freeDomEnhancedVisible = freeDomEnhancedVisible;
    refreshStatus();
}

void SlamUiBridge::setModeAndBackend(const QString& mode, const QString& backend)
{
    m_mode = mode;
    m_backend = backend;
    refreshStatus();
}

void SlamUiBridge::setErrorMessage(const QString& message)
{
    m_errorMessage = message;
    refreshStatus();
}

void SlamUiBridge::clearErrorMessage()
{
    if (m_errorMessage.isEmpty()) {
        return;
    }
    m_errorMessage.clear();
    refreshStatus();
}

void SlamUiBridge::resetCurrentPose()
{
    m_latestOutput.currentPose = SlamPose();
    m_latestOutput.currentPoseValid = false;
    m_hasCurrentPose = false;
    emit renderPoseReady(SlamRenderPose());
    emit poseAxisVerticesReady(buildPoseAxisVertices());
    refreshStatus();
}

void SlamUiBridge::clearDisplay()
{
    m_trajectory.clear();
    m_unoptimizedTrajectory.clear();
    m_globalMapPoints.clear();
    m_freeDomScanVoxelPoints.clear();
    m_freeDomDynamicVoxelPoints.clear();
    m_freeDomRaycastedVoxelPoints.clear();
    m_freeDomFreeVoxelPoints.clear();
    m_freeDomStaticVoxelPoints.clear();
    m_freeDomStaticMapPoints.clear();
    m_freeDomEnhancedPoints.clear();
    m_freeDomDepthImage = QImage();
    m_freeDomEnhancedDepthImage = QImage();
    m_denseGlobalMapSegments.clear();
    m_loopClosureEdges.clear();
    m_hasOptimizedTrajectory = false;
    m_hasOptimizedGlobalMap = false;
    m_worldFramePointTotal = 0;
    m_latestOutput.publishedWorldFramePoints.clear();
    m_latestOutput.publishedBodyFramePoints.clear();
    m_latestOutput.dynamicDetectionFrameWorldPoints.clear();
    m_latestOutput.dynamicWorldFramePoints.clear();
    m_latestOutput.newGlobalMapPoints.clear();
    m_latestOutput.globalMapPointCount = 0;
    m_latestOutput.dynamicObjectStats = SlamDynamicObjectStats();
    m_latestOutput.currentPose = SlamPose();
    m_latestOutput.currentPoseValid = false;
    m_hasCurrentPose = false;
    m_initialPose = SlamPose();
    m_hasInitialPose = false;
    m_errorMessage.clear();
    emit renderPoseReady(SlamRenderPose());
    emit renderSnapshotReady(buildRenderSnapshot());
    refreshStatus();
}

void SlamUiBridge::refreshStatus()
{
    SlamRenderSnapshot snapshot = buildRenderSnapshot();
    m_displayState.status = statusName(m_latestOutput.status);
    m_displayState.mode = m_mode;
    m_displayState.backend = m_backend;
    m_displayState.imuState = m_backend == QStringLiteral("FAST_LO")
        ? QStringLiteral("Not used (LO)")
        : (m_latestOutput.imuHealthy ? QStringLiteral("Healthy") : QStringLiteral("Unavailable"));
    m_displayState.inputFps = QString::number(m_latestOutput.inputFps, 'f', 1);
    m_displayState.backendMs = QString::number(m_latestOutput.backendMs, 'f', 2);
    m_displayState.droppedFrames = QString::number(m_latestOutput.droppedFrameCount);
    m_displayState.currentPose = formatPose(m_latestOutput.currentPose);
    m_displayState.trajectoryPoints = QString::number(m_trajectory.size());
    m_displayState.keyframeCount = QString::number(m_latestOutput.keyframeCount);
    m_displayState.loopClosureCount = QString::number(m_latestOutput.loopClosureCount);
    const quint64 snapshotBytes =
        vectorBytes(snapshot.trajectoryVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.loopClosureVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.poseAxisVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.worldFrameVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.bodyFrameVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.dynamicObjectVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.dynamicAggressiveVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.dynamicModerateVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.dynamicConservativeVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.freeDomScanVoxelVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.freeDomDynamicVoxelVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.freeDomRaycastedVoxelVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.freeDomFreeVoxelVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.freeDomStaticVoxelVertices.size(), sizeof(SlamRenderVertex)) +
        vectorBytes(snapshot.freeDomEnhancedVertices.size(), sizeof(SlamRenderVertex));
    const quint64 uiBytes =
        vectorBytes(m_trajectory.size(), sizeof(SlamTrajectoryPoint)) +
        vectorBytes(m_unoptimizedTrajectory.size(), sizeof(SlamTrajectoryPoint)) +
        vectorBytes(m_globalMapPoints.size(), sizeof(SlamPoint)) +
        vectorBytes(m_freeDomStaticMapPoints.size(), sizeof(SlamPoint)) +
        vectorBytes(denseGlobalMapPointCount(), sizeof(SlamPoint)) +
        vectorBytes(m_latestOutput.publishedWorldFramePoints.size(), sizeof(SlamPoint)) +
        vectorBytes(m_latestOutput.publishedBodyFramePoints.size(), sizeof(SlamPoint)) +
        vectorBytes(m_latestOutput.dynamicDetectionFrameWorldPoints.size(), sizeof(SlamPoint)) +
        vectorBytes(m_latestOutput.dynamicWorldFramePoints.size(), sizeof(SlamDynamicPoint)) +
        snapshotBytes;
    m_displayState.memoryUsage = QStringLiteral("UI %1").arg(formatMemoryBytes(uiBytes));
    m_displayState.mapPoints = QString::number(m_latestOutput.mapPointCount);
    m_displayState.worldFramePoints = QString::number(m_worldFramePointTotal);
    m_displayState.bodyFramePoints = QString::number(m_latestOutput.publishedBodyFramePoints.size());
    m_displayState.globalMapPoints = QString::number(m_globalMapPoints.size());
    m_displayState.dynamicMode = m_latestOutput.dynamicObjectStats.enabled
        ? dynamicBackendName(m_latestOutput.dynamicObjectStats.backend)
        : QStringLiteral("关闭");
    if (m_latestOutput.dynamicObjectStats.enabled) {
        const SlamDynamicObjectStats& stats = m_latestOutput.dynamicObjectStats;
        m_displayState.dynamicPoints = stats.backend == DynamicFilterBackend::FreeDOM
            ? QStringLiteral("动态 %1 / 静态 %2 / 无效 %3 / A %4 M %5 C %6")
                  .arg(stats.dynamicPointCount)
                  .arg(stats.staticPointCount)
                  .arg(stats.invalidPointCount)
                  .arg(stats.freeDomAggressivePointCount)
                  .arg(stats.freeDomModeratePointCount)
                  .arg(stats.freeDomConservativePointCount)
            : stats.clusterEnabled
            ? QStringLiteral("聚类 %1 / 原始 %2 / 对象 %3 拒绝 %4 / 地面剔除 %5 / C1 %6 C2 %7 C3 %8")
                  .arg(stats.dynamicPointCount)
                  .arg(stats.originDynamicPointCount)
                  .arg(stats.clusterCount)
                  .arg(stats.rejectedClusterCount)
                  .arg(stats.groundRemovedPointCount)
                  .arg(stats.case1PointCount)
                  .arg(stats.case2PointCount)
                  .arg(stats.case3PointCount)
            : QStringLiteral("%1 / 静态 %2 / C1 %3 C2 %4 C3 %5")
                  .arg(stats.dynamicPointCount)
                  .arg(stats.staticPointCount)
                  .arg(stats.case1PointCount)
                  .arg(stats.case2PointCount)
                  .arg(stats.case3PointCount);
    } else {
        m_displayState.dynamicPoints = QStringLiteral("0");
    }
    m_displayState.dynamicDetectorMs = m_latestOutput.dynamicObjectStats.enabled
        ? QString::number(m_latestOutput.dynamicObjectStats.detectorMs, 'f', 2)
        : QStringLiteral("0.00");
    m_displayState.dynamicClusterMs = m_latestOutput.dynamicObjectStats.clusterEnabled
        ? QString::number(m_latestOutput.dynamicObjectStats.clusterMs, 'f', 2)
        : QStringLiteral("0.00");
    const SlamDynamicObjectStats& dynamicStats = m_latestOutput.dynamicObjectStats;
    m_displayState.freeDomMap = dynamicStats.backend == DynamicFilterBackend::FreeDOM
        ? QStringLiteral("Free %1 块/%2 体素；Static %3 块/%4 体素/%5 子体素")
              .arg(dynamicStats.freeDomFreeBlockCount)
              .arg(dynamicStats.freeDomFreeVoxelCount)
              .arg(dynamicStats.freeDomStaticBlockCount)
              .arg(dynamicStats.freeDomStaticVoxelCount)
              .arg(dynamicStats.freeDomStaticSubvoxelCount)
        : QStringLiteral("-");
    m_displayState.freeDomStages = dynamicStats.backend == DynamicFilterBackend::FreeDOM
        ? QStringLiteral("Scan %1 / ScanRm %2 / Enhance %3 / Free %4 / MapRm %5 / Integrate %6 ms")
              .arg(dynamicStats.freeDomBuildScanMapMs, 0, 'f', 2)
              .arg(dynamicStats.freeDomScanRemovalMs, 0, 'f', 2)
              .arg(dynamicStats.freeDomRaycastEnhancementMs, 0, 'f', 2)
              .arg(dynamicStats.freeDomFreeSpaceEstimationMs, 0, 'f', 2)
              .arg(dynamicStats.freeDomMapRemovalMs, 0, 'f', 2)
              .arg(dynamicStats.freeDomStaticIntegrationMs, 0, 'f', 2)
        : QStringLiteral("-");
    m_displayState.error = m_errorMessage.isEmpty() ? QStringLiteral("无") : m_errorMessage;

    const QString statusText = QStringLiteral("SLAM: %1 | %2 | %3 fps | %4 ms")
                                   .arg(m_displayState.status,
                                        m_displayState.backend,
                                        m_displayState.inputFps,
                                        m_displayState.backendMs);
    emit statusTextReady(statusText);
    emit displayStateChanged();
    emit renderSnapshotReady(snapshot);
}

SlamRenderSnapshot SlamUiBridge::buildRenderSnapshot()
{
    SlamRenderSnapshot snapshot;
    snapshot.worldFramePointSizePx = m_worldFramePointSizePx;
    snapshot.bodyFramePointSizePx = m_bodyFramePointSizePx;
    snapshot.dynamicObjectPointSizePx = m_dynamicObjectPointSizePx;
    snapshot.freeDomScanVoxelPointSizePx = m_freeDomScanVoxelPointSizePx;
    snapshot.freeDomDynamicVoxelPointSizePx = m_freeDomDynamicVoxelPointSizePx;
    snapshot.freeDomRaycastedVoxelPointSizePx = m_freeDomRaycastedVoxelPointSizePx;
    snapshot.freeDomFreeVoxelPointSizePx = m_freeDomFreeVoxelPointSizePx;
    snapshot.freeDomStaticVoxelPointSizePx = m_freeDomStaticVoxelPointSizePx;
    snapshot.freeDomEnhancedPointSizePx = m_freeDomEnhancedPointSizePx;
    snapshot.trajectoryLineWidthPx = m_trajectoryLineWidthPx;
    snapshot.poseAxisLengthM = m_poseAxisLengthM;
    snapshot.poseAxisLineWidthPx = m_poseAxisLineWidthPx;
    snapshot.currentPose = renderPose(m_latestOutput.currentPose, m_hasCurrentPose);

    if (m_trajectoryVisible) {
        snapshot.trajectoryVertices.reserve(m_trajectory.size());
        const float r = float(m_trajectoryColor.redF());
        const float g = float(m_trajectoryColor.greenF());
        const float b = float(m_trajectoryColor.blueF());
        for (const SlamTrajectoryPoint& point : m_trajectory) {
            snapshot.trajectoryVertices.push_back(renderVertex(posePosition(point.pose), r, g, b));
        }
        snapshot.loopClosureVertices.reserve(m_loopClosureEdges.size() * 2);
        for (const SlamLoopClosureEdge& edge : m_loopClosureEdges) {
            if (edge.currentKeyframeId < 0 || edge.previousKeyframeId < 0 ||
                edge.currentKeyframeId >= m_trajectory.size() ||
                edge.previousKeyframeId >= m_trajectory.size()) {
                continue;
            }
            snapshot.loopClosureVertices.push_back(renderVertex(
                posePosition(m_trajectory.at(edge.currentKeyframeId).pose), 0.9f, 0.9f, 0.0f));
            snapshot.loopClosureVertices.push_back(renderVertex(
                posePosition(m_trajectory.at(edge.previousKeyframeId).pose), 0.9f, 0.9f, 0.0f));
        }
    }

    if (m_poseAxisVisible) {
        snapshot.poseAxisVertices = buildPoseAxisVertices();
    }

    const QVector<SlamPoint>* worldFramePoints = nullptr;
    if (m_worldFrameVisible && !m_latestOutput.publishedWorldFramePoints.isEmpty()) {
        worldFramePoints = &m_latestOutput.publishedWorldFramePoints;
    } else if (m_latestOutput.publishedWorldFramePoints.isEmpty() &&
               m_latestOutput.dynamicObjectStats.enabled) {
        worldFramePoints = &m_latestOutput.dynamicDetectionFrameWorldPoints;
    }
    if (worldFramePoints) {
        snapshot.worldFrameVertices.reserve(worldFramePoints->size());
        for (const SlamPoint& point : *worldFramePoints) {
            snapshot.worldFrameVertices.push_back(renderSlamPoint(point, m_worldFrameColor));
        }
    }

    if (m_bodyFrameVisible) {
        snapshot.bodyFrameVertices.reserve(m_latestOutput.publishedBodyFramePoints.size());
        for (const SlamPoint& point : m_latestOutput.publishedBodyFramePoints) {
            snapshot.bodyFrameVertices.push_back(renderSlamPoint(point, m_bodyFrameColor));
        }
    }
    if (m_dynamicObjectVisible && m_latestOutput.dynamicObjectStats.enabled) {
        snapshot.dynamicObjectVertices.reserve(m_latestOutput.dynamicWorldFramePoints.size());
        for (const SlamDynamicPoint& point : m_latestOutput.dynamicWorldFramePoints) {
            switch (point.label) {
            case SlamDynamicPointLabel::FreeDomAggressive:
                snapshot.dynamicAggressiveVertices.push_back(
                    renderDynamicPoint(point, m_dynamicAggressiveColor));
                break;
            case SlamDynamicPointLabel::FreeDomModerate:
                snapshot.dynamicModerateVertices.push_back(
                    renderDynamicPoint(point, m_dynamicModerateColor));
                break;
            case SlamDynamicPointLabel::FreeDomConservative:
                snapshot.dynamicConservativeVertices.push_back(
                    renderDynamicPoint(point, m_dynamicConservativeColor));
                break;
            default:
                snapshot.dynamicObjectVertices.push_back(
                    renderDynamicPoint(point, m_dynamicObjectColor));
                break;
            }
        }
    }
    if (m_freeDomScanVoxelVisible && m_latestOutput.dynamicObjectStats.enabled) {
        snapshot.freeDomScanVoxelVertices.reserve(m_freeDomScanVoxelPoints.size());
        for (const SlamPoint& point : m_freeDomScanVoxelPoints) {
            snapshot.freeDomScanVoxelVertices.push_back(
                renderSlamPoint(point, m_freeDomScanVoxelColor));
        }
    }
    if (m_freeDomDynamicVoxelVisible && m_latestOutput.dynamicObjectStats.enabled) {
        snapshot.freeDomDynamicVoxelVertices.reserve(m_freeDomDynamicVoxelPoints.size());
        for (const SlamPoint& point : m_freeDomDynamicVoxelPoints) {
            snapshot.freeDomDynamicVoxelVertices.push_back(
                renderSlamPoint(point, m_freeDomDynamicVoxelColor));
        }
    }
    if (m_freeDomRaycastedVoxelVisible && m_latestOutput.dynamicObjectStats.enabled) {
        snapshot.freeDomRaycastedVoxelVertices.reserve(m_freeDomRaycastedVoxelPoints.size());
        for (const SlamPoint& point : m_freeDomRaycastedVoxelPoints) {
            snapshot.freeDomRaycastedVoxelVertices.push_back(
                renderSlamPoint(point, m_freeDomRaycastedVoxelColor));
        }
    }
    if (m_freeDomFreeVoxelVisible && m_latestOutput.dynamicObjectStats.enabled) {
        snapshot.freeDomFreeVoxelVertices.reserve(m_freeDomFreeVoxelPoints.size());
        for (const SlamPoint& point : m_freeDomFreeVoxelPoints) {
            snapshot.freeDomFreeVoxelVertices.push_back(
                renderSlamPoint(point, m_freeDomFreeVoxelColor));
        }
    }
    if (m_freeDomStaticVoxelVisible && m_latestOutput.dynamicObjectStats.enabled) {
        snapshot.freeDomStaticVoxelVertices.reserve(m_freeDomStaticVoxelPoints.size());
        for (const SlamPoint& point : m_freeDomStaticVoxelPoints) {
            snapshot.freeDomStaticVoxelVertices.push_back(
                renderSlamPoint(point, m_freeDomStaticVoxelColor));
        }
    }
    if (m_freeDomEnhancedVisible && m_latestOutput.dynamicObjectStats.enabled) {
        snapshot.freeDomEnhancedVertices.reserve(m_freeDomEnhancedPoints.size());
        for (const SlamPoint& point : m_freeDomEnhancedPoints) {
            snapshot.freeDomEnhancedVertices.push_back(
                renderSlamPoint(point, m_freeDomEnhancedColor));
        }
    }
    return snapshot;
}

QVector<SlamRenderVertex> SlamUiBridge::buildPoseAxisVertices() const
{
    QVector<SlamRenderVertex> vertices;
    if (!m_poseAxisVisible) {
        return vertices;
    }

    const float diameter = poseAxisDiameterFromLineWidth(m_poseAxisLineWidthPx);
    vertices.reserve((m_hasInitialPose ? 288 : 0) + (m_hasCurrentPose ? 288 : 0));
    if (m_hasInitialPose) {
        appendPoseAxes(vertices, m_initialPose, m_poseAxisLengthM, diameter);
    }
    if (m_hasCurrentPose) {
        appendPoseAxes(vertices, m_latestOutput.currentPose, m_poseAxisLengthM, diameter);
    }
    return vertices;
}

void SlamUiBridge::appendTrajectory(const SlamOutput& output)
{
    if (!output.newTrajectoryPoints.isEmpty()) {
        m_unoptimizedTrajectory += output.newTrajectoryPoints;
        if (m_unoptimizedTrajectory.size() > kMaxTrajectoryPoints) {
            m_unoptimizedTrajectory.erase(
                m_unoptimizedTrajectory.begin(),
                m_unoptimizedTrajectory.begin() +
                    (m_unoptimizedTrajectory.size() - kMaxTrajectoryPoints));
        }
        if (!m_hasOptimizedTrajectory) {
            m_trajectory += output.newTrajectoryPoints;
        }
    }
    if (output.optimizedTrajectoryReset) {
        m_trajectory.clear();
        m_hasOptimizedTrajectory = true;
    }
    if (!output.optimizedTrajectory.isEmpty()) {
        if (!m_hasOptimizedTrajectory) {
            m_trajectory.clear();
            m_hasOptimizedTrajectory = true;
        }
        m_trajectory += output.optimizedTrajectory;
    }
}

void SlamUiBridge::appendGlobalMap(const SlamOutput& output)
{
    if (!output.newGlobalMapPoints.isEmpty()) {
        DenseGlobalMapSegment segment;
        segment.poseCorrectionEpoch = output.poseCorrectionEpoch;
        segment.pose = output.currentPose;
        segment.points = output.newGlobalMapPoints;
        m_denseGlobalMapSegments.push_back(std::move(segment));
        m_globalMapPoints += output.newGlobalMapPoints;
    }

    if (!m_denseGlobalMapSegments.isEmpty()) {
        if (output.optimizedTrajectoryReset && !output.optimizedTrajectory.isEmpty()) {
            correctDenseGlobalMap(output.optimizedTrajectory);
            rebuildDenseGlobalMapSnapshot();
            m_hasOptimizedGlobalMap = true;
        }
        return;
    }

    if (output.optimizedGlobalMapReset) {
        m_globalMapPoints.clear();
        m_hasOptimizedGlobalMap = true;
    }
    if (!output.optimizedGlobalMapPoints.isEmpty()) {
        if (!m_hasOptimizedGlobalMap) {
            m_globalMapPoints.clear();
            m_hasOptimizedGlobalMap = true;
        }
        m_globalMapPoints += output.optimizedGlobalMapPoints;
    }
}

void SlamUiBridge::correctDenseGlobalMap(
    const QVector<SlamTrajectoryPoint>& optimizedTrajectory)
{
    struct PoseCorrection {
        int64_t timestampNs = 0;
        Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    };

    QVector<QVector<PoseCorrection>> correctionsByEpoch;
    int segmentIndex = 0;
    for (const SlamTrajectoryPoint& optimizedPoint : optimizedTrajectory) {
        while (segmentIndex + 1 < m_denseGlobalMapSegments.size() &&
               std::abs(m_denseGlobalMapSegments.at(segmentIndex + 1).pose.timestampNs -
                        optimizedPoint.pose.timestampNs) <
                   std::abs(m_denseGlobalMapSegments.at(segmentIndex).pose.timestampNs -
                            optimizedPoint.pose.timestampNs)) {
            ++segmentIndex;
        }
        const Eigen::Isometry3d correction =
            poseTransform(optimizedPoint.pose) *
            poseTransform(m_denseGlobalMapSegments.at(segmentIndex).pose).inverse();
        const int epoch = m_denseGlobalMapSegments.at(segmentIndex).poseCorrectionEpoch;
        if (correctionsByEpoch.size() <= epoch) {
            correctionsByEpoch.resize(epoch + 1);
        }
        correctionsByEpoch[epoch].push_back({optimizedPoint.pose.timestampNs,
                                             Eigen::Quaterniond(correction.rotation()),
                                             correction.translation()});
    }

    DenseGlobalMapSegment* segments = m_denseGlobalMapSegments.data();
    const int segmentCount = m_denseGlobalMapSegments.size();
#pragma omp parallel for
    for (int index = 0; index < segmentCount; ++index) {
        DenseGlobalMapSegment& segment = segments[index];
        const QVector<PoseCorrection>& corrections =
            correctionsByEpoch.at(segment.poseCorrectionEpoch);
        const auto upper = std::lower_bound(
            corrections.cbegin(),
            corrections.cend(),
            segment.pose.timestampNs,
            [](const PoseCorrection& correction, int64_t timestampNs) {
                return correction.timestampNs < timestampNs;
            });

        Eigen::Quaterniond rotation;
        Eigen::Vector3d translation;
        if (upper == corrections.cbegin()) {
            rotation = upper->rotation;
            translation = upper->translation;
        } else if (upper == corrections.cend()) {
            rotation = corrections.back().rotation;
            translation = corrections.back().translation;
        } else {
            const PoseCorrection& before = *(upper - 1);
            const double alpha = double(segment.pose.timestampNs - before.timestampNs) /
                                 double(upper->timestampNs - before.timestampNs);
            rotation = before.rotation.slerp(alpha, upper->rotation).normalized();
            translation = before.translation * (1.0 - alpha) + upper->translation * alpha;
        }

        const Eigen::Matrix3f rotationMatrix = rotation.toRotationMatrix().cast<float>();
        const Eigen::Vector3f translationVector = translation.cast<float>();
        for (SlamPoint& point : segment.points) {
            const Eigen::Vector3f corrected =
                rotationMatrix * Eigen::Vector3f(point.x, point.y, point.z) + translationVector;
            point.x = corrected.x();
            point.y = corrected.y();
            point.z = corrected.z();
        }

        Eigen::Isometry3d correctionTransform = Eigen::Isometry3d::Identity();
        correctionTransform.linear() = rotation.toRotationMatrix();
        correctionTransform.translation() = translation;
        setPoseTransform(segment.pose, correctionTransform * poseTransform(segment.pose));
    }
}

void SlamUiBridge::rebuildDenseGlobalMapSnapshot()
{
    m_globalMapPoints.clear();
    m_globalMapPoints.reserve(denseGlobalMapPointCount());
    for (const DenseGlobalMapSegment& segment : m_denseGlobalMapSegments) {
        m_globalMapPoints += segment.points;
    }
}

qsizetype SlamUiBridge::denseGlobalMapPointCount() const
{
    qsizetype pointCount = 0;
    for (const DenseGlobalMapSegment& segment : m_denseGlobalMapSegments) {
        pointCount += segment.points.size();
    }
    return pointCount;
}

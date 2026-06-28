#include "slam/SlamUiBridge.h"

#include <QCoreApplication>
#include <QQuaternion>
#include <QThread>
#include <QVector3D>

#include <algorithm>
#include <cmath>

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

SlamRenderVertex renderVertex(const QVector3D& point, float r, float g, float b)
{
    return {point.x(), point.y(), point.z(), r, g, b};
}

SlamRenderVertex renderSlamPoint(const SlamPoint& point, const QColor& color)
{
    return {point.x, point.y, point.z, color.redF(), color.greenF(), color.blueF()};
}

QVector3D posePosition(const SlamPose& pose)
{
    return QVector3D(float(pose.tx), float(pose.ty), float(pose.tz));
}

QString formatPose(const SlamPose& pose)
{
    return QStringLiteral("t=[%1, %2, %3], q=[%4, %5, %6, %7]")
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
    m_latestOutput = output;
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
    if (output.status == SlamStatusCode::Running) {
        m_worldFramePointTotal += quint64(output.publishedWorldFramePoints.size());
    }
    m_latestOutput.newGlobalMapPoints.clear();
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
                                            bool bodyFrameVisible)
{
    if (m_trajectoryVisible == trajectoryVisible &&
        m_poseAxisVisible == poseAxisVisible &&
        m_worldFrameVisible == worldFrameVisible &&
        m_bodyFrameVisible == bodyFrameVisible) {
        return;
    }
    m_trajectoryVisible = trajectoryVisible;
    m_poseAxisVisible = poseAxisVisible;
    m_worldFrameVisible = worldFrameVisible;
    m_bodyFrameVisible = bodyFrameVisible;
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

void SlamUiBridge::clearDisplay()
{
    m_trajectory.clear();
    m_globalMapPoints.clear();
    m_worldFramePointTotal = 0;
    m_latestOutput.publishedWorldFramePoints.clear();
    m_latestOutput.publishedBodyFramePoints.clear();
    m_latestOutput.newGlobalMapPoints.clear();
    m_latestOutput.globalMapPointCount = 0;
    m_errorMessage.clear();
    emit renderSnapshotReady(buildRenderSnapshot());
    refreshStatus();
}

void SlamUiBridge::refreshStatus()
{
    SlamRenderSnapshot snapshot = buildRenderSnapshot();
    m_displayState.status = statusName(m_latestOutput.status);
    m_displayState.mode = m_mode;
    m_displayState.backend = m_backend;
    m_displayState.imuState = m_latestOutput.imuHealthy ? QStringLiteral("Healthy") : QStringLiteral("Unavailable");
    m_displayState.inputFps = QString::number(m_latestOutput.inputFps, 'f', 1);
    m_displayState.backendMs = QString::number(m_latestOutput.backendMs, 'f', 2);
    m_displayState.droppedFrames = QString::number(m_latestOutput.droppedFrameCount);
    m_displayState.currentPose = formatPose(m_latestOutput.currentPose);
    m_displayState.trajectoryPoints = QString::number(m_trajectory.size());
    m_displayState.mapPoints = QString::number(m_latestOutput.mapPointCount);
    m_displayState.worldFramePoints = QString::number(m_worldFramePointTotal);
    m_displayState.bodyFramePoints = QString::number(m_latestOutput.publishedBodyFramePoints.size());
    m_displayState.globalMapPoints = QString::number(m_globalMapPoints.size());
    m_displayState.error = m_errorMessage;

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
    snapshot.trajectoryLineWidthPx = m_trajectoryLineWidthPx;
    snapshot.poseAxisLengthM = m_poseAxisLengthM;
    snapshot.poseAxisLineWidthPx = m_poseAxisLineWidthPx;

    if (m_trajectoryVisible) {
        snapshot.trajectoryVertices.reserve(m_trajectory.size());
        const float r = float(m_trajectoryColor.redF());
        const float g = float(m_trajectoryColor.greenF());
        const float b = float(m_trajectoryColor.blueF());
        for (const SlamTrajectoryPoint& point : m_trajectory) {
            snapshot.trajectoryVertices.push_back(renderVertex(posePosition(point.pose), r, g, b));
        }
    }

    if (m_poseAxisVisible) {
        const QVector3D origin = posePosition(m_latestOutput.currentPose);
        const QQuaternion rotation(float(m_latestOutput.currentPose.qw),
                                   float(m_latestOutput.currentPose.qx),
                                   float(m_latestOutput.currentPose.qy),
                                   float(m_latestOutput.currentPose.qz));
        const float diameter = poseAxisDiameterFromLineWidth(m_poseAxisLineWidthPx);
        snapshot.poseAxisVertices.reserve(288);
        appendPoseAxisMesh(snapshot.poseAxisVertices,
                           origin,
                           rotation.rotatedVector(QVector3D(1.0f, 0.0f, 0.0f)),
                           m_poseAxisLengthM,
                           diameter,
                           1.0f,
                           0.05f,
                           0.05f);
        appendPoseAxisMesh(snapshot.poseAxisVertices,
                           origin,
                           rotation.rotatedVector(QVector3D(0.0f, 1.0f, 0.0f)),
                           m_poseAxisLengthM,
                           diameter,
                           0.05f,
                           1.0f,
                           0.05f);
        appendPoseAxisMesh(snapshot.poseAxisVertices,
                           origin,
                           rotation.rotatedVector(QVector3D(0.0f, 0.0f, 1.0f)),
                           m_poseAxisLengthM,
                           diameter,
                           0.1f,
                           0.35f,
                           1.0f);
    }

    if (m_worldFrameVisible) {
        snapshot.worldFrameVertices.reserve(m_latestOutput.publishedWorldFramePoints.size());
        for (const SlamPoint& point : m_latestOutput.publishedWorldFramePoints) {
            snapshot.worldFrameVertices.push_back(renderSlamPoint(point, m_worldFrameColor));
        }
    }

    if (m_bodyFrameVisible) {
        snapshot.bodyFrameVertices.reserve(m_latestOutput.publishedBodyFramePoints.size());
        for (const SlamPoint& point : m_latestOutput.publishedBodyFramePoints) {
            snapshot.bodyFrameVertices.push_back(renderSlamPoint(point, m_bodyFrameColor));
        }
    }
    return snapshot;
}

void SlamUiBridge::appendTrajectory(const SlamOutput& output)
{
    if (!output.newTrajectoryPoints.isEmpty()) {
        m_trajectory += output.newTrajectoryPoints;
        if (m_trajectory.size() > kMaxTrajectoryPoints) {
            m_trajectory.erase(m_trajectory.begin(), m_trajectory.begin() + (m_trajectory.size() - kMaxTrajectoryPoints));
        }
    }
}

void SlamUiBridge::appendGlobalMap(const SlamOutput& output)
{
    if (output.newGlobalMapPoints.isEmpty()) {
        return;
    }
    m_globalMapPoints += output.newGlobalMapPoints;
}

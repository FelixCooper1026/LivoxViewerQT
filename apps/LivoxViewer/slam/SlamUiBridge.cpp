#include "slam/SlamUiBridge.h"

#include <QCoreApplication>
#include <QQuaternion>
#include <QThread>
#include <QVector3D>

#include <algorithm>

namespace {

constexpr int kMaxTrajectoryPoints = 200000;
constexpr int kMaxMapPointsPerChunk = 1000;
constexpr int kMaxMapPointsPerTick = 5000;

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
    appendTrajectory(output);
    appendMapPreview(output);
}

void SlamUiBridge::setModeAndBackend(const QString& mode, const QString& backend)
{
    m_mode = mode;
    m_backend = backend;
    refreshStatus();
}

void SlamUiBridge::setMapPreviewEnabled(bool enabled)
{
    if (m_mapPreviewEnabled == enabled) {
        return;
    }
    m_mapPreviewEnabled = enabled;
    if (!enabled) {
        m_mapPreviewPoints.clear();
        m_pendingMapPreviewPoints.clear();
    }
    emit renderSnapshotReady(buildRenderSnapshot());
    refreshStatus();
}

void SlamUiBridge::clearDisplay()
{
    m_trajectory.clear();
    m_mapPreviewPoints.clear();
    m_pendingMapPreviewPoints.clear();
    emit renderSnapshotReady(buildRenderSnapshot());
    refreshStatus();
}

void SlamUiBridge::refreshStatus()
{
    flushPendingMapPreview();

    m_displayState.status = statusName(m_latestOutput.status);
    m_displayState.mode = m_mode;
    m_displayState.backend = m_backend;
    m_displayState.imuState = m_latestOutput.imuHealthy ? QStringLiteral("Healthy") : QStringLiteral("Unavailable");
    m_displayState.inputFps = QString::number(m_latestOutput.inputFps, 'f', 1);
    m_displayState.backendMs = QString::number(m_latestOutput.backendMs, 'f', 2);
    m_displayState.droppedFrames = QString::number(m_latestOutput.droppedFrameCount);
    m_displayState.currentPose = formatPose(m_latestOutput.currentPose);
    m_displayState.trajectoryPoints = QString::number(m_trajectory.size());
    m_displayState.mapPoints = QString::number(m_mapPreviewEnabled ? m_mapPreviewPoints.size()
                                                                    : m_latestOutput.mapPointCount);
    m_displayState.error = isErrorStatus(m_latestOutput.status) ? m_latestOutput.message : QString();

    const QString statusText = QStringLiteral("SLAM: %1 | %2 | %3 fps | %4 ms")
                                   .arg(m_displayState.status,
                                        m_displayState.backend,
                                        m_displayState.inputFps,
                                        m_displayState.backendMs);
    emit statusTextReady(statusText);
    emit displayStateChanged();
    emit renderSnapshotReady(buildRenderSnapshot());
}

SlamRenderSnapshot SlamUiBridge::buildRenderSnapshot()
{
    SlamRenderSnapshot snapshot;
    snapshot.mapPreviewEnabled = m_mapPreviewEnabled;
    snapshot.maxMapPreviewPoints = m_maxMapPreviewPoints;
    snapshot.trajectoryVertices.reserve(m_trajectory.size());
    for (const SlamTrajectoryPoint& point : m_trajectory) {
        snapshot.trajectoryVertices.push_back(renderVertex(posePosition(point.pose), 0.1f, 0.75f, 1.0f));
    }

    const QVector3D origin = posePosition(m_latestOutput.currentPose);
    const QQuaternion rotation(float(m_latestOutput.currentPose.qw),
                               float(m_latestOutput.currentPose.qx),
                               float(m_latestOutput.currentPose.qy),
                               float(m_latestOutput.currentPose.qz));
    constexpr float kAxisLength = 0.8f;
    snapshot.poseAxisVertices.reserve(6);
    snapshot.poseAxisVertices.push_back(renderVertex(origin, 1.0f, 0.05f, 0.05f));
    snapshot.poseAxisVertices.push_back(renderVertex(origin + rotation.rotatedVector(QVector3D(kAxisLength, 0.0f, 0.0f)), 1.0f, 0.05f, 0.05f));
    snapshot.poseAxisVertices.push_back(renderVertex(origin, 0.05f, 1.0f, 0.05f));
    snapshot.poseAxisVertices.push_back(renderVertex(origin + rotation.rotatedVector(QVector3D(0.0f, kAxisLength, 0.0f)), 0.05f, 1.0f, 0.05f));
    snapshot.poseAxisVertices.push_back(renderVertex(origin, 0.1f, 0.35f, 1.0f));
    snapshot.poseAxisVertices.push_back(renderVertex(origin + rotation.rotatedVector(QVector3D(0.0f, 0.0f, kAxisLength)), 0.1f, 0.35f, 1.0f));

    if (m_mapPreviewEnabled) {
        snapshot.mapPreviewPoints = m_mapPreviewPoints;
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

void SlamUiBridge::appendMapPreview(const SlamOutput& output)
{
    if (!m_mapPreviewEnabled) {
        return;
    }

    for (const SlamMapChunk& chunk : output.newMapChunks) {
        const int pointCount = int(chunk.pointsWorld.size());
        const int stride = std::max(1, pointCount / kMaxMapPointsPerChunk);
        for (int i = 0; i < pointCount; i += stride) {
            const SlamPoint& point = chunk.pointsWorld.at(i);
            m_pendingMapPreviewPoints.push_back({point.x, point.y, point.z, 0.72f, 0.72f, 0.72f});
        }
    }
}

void SlamUiBridge::flushPendingMapPreview()
{
    if (!m_mapPreviewEnabled || m_pendingMapPreviewPoints.isEmpty()) {
        return;
    }

    const int pendingCount = int(m_pendingMapPreviewPoints.size());
    const int currentCount = int(m_mapPreviewPoints.size());
    const int flushCount = std::min(kMaxMapPointsPerTick, pendingCount);
    m_mapPreviewPoints.reserve(std::min(m_maxMapPreviewPoints, currentCount + flushCount));
    for (int i = 0; i < flushCount; ++i) {
        m_mapPreviewPoints.push_back(m_pendingMapPreviewPoints.at(i));
    }
    m_pendingMapPreviewPoints.erase(m_pendingMapPreviewPoints.begin(),
                                    m_pendingMapPreviewPoints.begin() + flushCount);

    if (m_mapPreviewPoints.size() > m_maxMapPreviewPoints) {
        m_mapPreviewPoints.erase(m_mapPreviewPoints.begin(),
                                 m_mapPreviewPoints.begin() + (m_mapPreviewPoints.size() - m_maxMapPreviewPoints));
    }
}

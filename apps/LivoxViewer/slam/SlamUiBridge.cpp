#include "slam/SlamUiBridge.h"

#include <QCoreApplication>
#include <QQuaternion>
#include <QThread>
#include <QVector3D>

#include <algorithm>

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
    appendMapPreview(output);
}

void SlamUiBridge::setModeAndBackend(const QString& mode, const QString& backend)
{
    m_mode = mode;
    m_backend = backend;
    refreshStatus();
}

void SlamUiBridge::setMapPreviewConfig(const SlamMapPreviewConfig& config)
{
    const bool changed = config.mode != m_mapPreviewConfig.mode ||
                         config.globalSparseMaxPoints != m_mapPreviewConfig.globalSparseMaxPoints ||
                         config.globalSparseVoxelSizeM != m_mapPreviewConfig.globalSparseVoxelSizeM ||
                         config.globalSparseUploadPointsPerTick != m_mapPreviewConfig.globalSparseUploadPointsPerTick ||
                         config.globalDenseMaxPoints != m_mapPreviewConfig.globalDenseMaxPoints ||
                         config.globalDenseVoxelSizeM != m_mapPreviewConfig.globalDenseVoxelSizeM ||
                         config.globalDenseUploadPointsPerTick != m_mapPreviewConfig.globalDenseUploadPointsPerTick;
    m_mapPreviewConfig = config;
    m_mapPreviewStore.configure(config);
    if (changed) {
        m_mapPreviewResetPending = true;
    }
    emit renderSnapshotReady(buildRenderSnapshot());
    refreshStatus();
}

void SlamUiBridge::setMapPreviewMode(SlamMapPreviewMode mode)
{
    SlamMapPreviewConfig config = m_mapPreviewConfig;
    config.mode = mode;
    setMapPreviewConfig(config);
}

void SlamUiBridge::setMapPreviewEnabled(bool enabled)
{
    setMapPreviewMode(enabled ? SlamMapPreviewMode::GlobalSparse : SlamMapPreviewMode::Off);
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
    m_mapPreviewStore.clear();
    m_mapPreviewResetPending = true;
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
    m_displayState.mapPoints = QString::number(mapPreviewEnabled() ? m_mapPreviewStore.previewPointCount()
                                                                   : m_latestOutput.mapPointCount);
    m_displayState.mapPreviewMode = slamMapPreviewModeName(m_mapPreviewConfig.mode);
    m_displayState.mapPreviewPoints = QString::number(m_mapPreviewStore.previewPointCount());
    m_displayState.mapPreviewLimit = QString::number(m_mapPreviewStore.maxPreviewPoints());
    m_displayState.mapPreviewVoxelSize = m_mapPreviewStore.enabled()
        ? QStringLiteral("%1 m").arg(m_mapPreviewStore.voxelSizeM(), 0, 'f', 3)
        : QStringLiteral("-");
    m_displayState.mapPreviewPendingPoints = QString::number(m_mapPreviewStore.pendingPointCount());
    m_displayState.mapPreviewLimitReached = m_mapPreviewStore.reachedPointLimit()
        ? QStringLiteral("是")
        : QStringLiteral("否");
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
    snapshot.mapPreviewEnabled = mapPreviewEnabled();
    snapshot.mapPreviewReset = m_mapPreviewResetPending;
    snapshot.maxMapPreviewPoints = m_mapPreviewStore.maxPreviewPoints();
    snapshot.mapPreviewPointCount = m_mapPreviewStore.previewPointCount();
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

    if (snapshot.mapPreviewEnabled) {
        const GlobalMapPreviewStore::PendingUpload upload =
            m_mapPreviewStore.takePendingUploadPoints(m_mapPreviewStore.uploadPointsPerTick());
        snapshot.mapPreviewAppendPoints = upload.appendedPoints;
        snapshot.mapPreviewPointUpdates = upload.updatedPoints;
    }
    m_mapPreviewResetPending = false;
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
    if (!mapPreviewEnabled()) {
        return;
    }

    for (const SlamMapChunk& chunk : output.newMapChunks) {
        m_mapPreviewStore.appendMapChunk(chunk);
    }
}

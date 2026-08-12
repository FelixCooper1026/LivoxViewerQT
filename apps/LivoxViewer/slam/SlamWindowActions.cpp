#include "LivoxViewerWindow.h"

#include "Backends/FastLio/FastLioSlamBackend.h"
#include "Export/SlamMapExport.h"
#include "Export/SlamTrajectoryExport.h"
#include "Io/LvxSlamSource.h"
#include "Io/PcapSlamSource.h"
#include "Io/RosbagSlamSource.h"
#include "slam/SlamControlDialog.h"
#include "slam/SlamUiBridge.h"

#include <QDateTime>
#include <QApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QFileInfo>
#include <QMetaObject>
#include <QMessageBox>
#include <QSettings>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QStandardPaths>
#include <QStringList>
#include <QThread>

#include <algorithm>
#include <chrono>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include <Eigen/Geometry>
#include <omp.h>

namespace {

constexpr int64_t kSlamWorldHistoryRetentionNs = int64_t{600000} * int64_t{1000000};
constexpr std::chrono::milliseconds kOdometryPublishPeriod(5);
constexpr qint64 kFastReplayUiPublishIntervalMs = 100;
constexpr int kStreamingProgressMaximum = 1000;

bool isLidarOnlyConfig(const SlamRuntimeConfig& config)
{
    return !config.imuEnabled && config.allowPureLidar;
}

QString slamBackendDisplayName(const SlamRuntimeConfig& config)
{
    return isLidarOnlyConfig(config) ? QStringLiteral("FAST_LO") : QStringLiteral("FAST_LIO");
}

class HighRateOdometryPredictor {
public:
    void appendSamples(const QVector<SlamImuSample>& samples)
    {
        for (const SlamImuSample& sample : samples) {
            history_.push_back(sample);
        }
    }

    void applyCorrection(const FastLioPredictionState& correction)
    {
        position_ = Eigen::Vector3d(correction.position[0], correction.position[1], correction.position[2]);
        orientation_ = Eigen::Quaterniond(correction.orientation[3],
                                           correction.orientation[0],
                                           correction.orientation[1],
                                           correction.orientation[2]);
        velocity_ = Eigen::Vector3d(correction.velocity[0], correction.velocity[1], correction.velocity[2]);
        angularVelocity_ = Eigen::Vector3d(correction.angularVelocity[0],
                                           correction.angularVelocity[1],
                                           correction.angularVelocity[2]);
        gyroBias_ = Eigen::Vector3d(correction.gyroBias[0], correction.gyroBias[1], correction.gyroBias[2]);
        accelBias_ = Eigen::Vector3d(correction.accelBias[0], correction.accelBias[1], correction.accelBias[2]);
        gravity_ = Eigen::Vector3d(correction.gravity[0], correction.gravity[1], correction.gravity[2]);
        accelerationScale_ = correction.accelerationScale;
        lidarOnly_ = correction.lidarOnly;
        timestampNs_ = correction.timestampNs;
        initialized_ = true;

        while (!history_.empty() && history_.front().timestampNs <= timestampNs_) {
            history_.pop_front();
        }
        integratePendingSamples();
    }

    void integratePendingSamples()
    {
        integrateUntil(std::numeric_limits<int64_t>::max());
    }

    void integrateUntil(int64_t maximumTimestampNs)
    {
        if (!initialized_ || lidarOnly_) {
            return;
        }
        for (const SlamImuSample& sample : history_) {
            if (sample.timestampNs <= timestampNs_) {
                continue;
            }
            if (sample.timestampNs > maximumTimestampNs) {
                break;
            }
            const double dt = double(sample.timestampNs - timestampNs_) * 1.0e-9;
            const Eigen::Vector3d gyro(sample.gyroRadPerSec[0],
                                       sample.gyroRadPerSec[1],
                                       sample.gyroRadPerSec[2]);
            const Eigen::Vector3d accel(sample.accelRaw[0], sample.accelRaw[1], sample.accelRaw[2]);
            const Eigen::Vector3d angularVelocity = gyro - gyroBias_;
            const Eigen::Vector3d worldAcceleration =
                orientation_ * (accel * accelerationScale_ - accelBias_) + gravity_;
            position_ += velocity_ * dt + 0.5 * worldAcceleration * dt * dt;
            velocity_ += worldAcceleration * dt;
            const double angle = angularVelocity.norm() * dt;
            if (angle > 0.0) {
                orientation_ = (orientation_ *
                    Eigen::Quaterniond(Eigen::AngleAxisd(angle, angularVelocity.normalized()))).normalized();
            }
            timestampNs_ = sample.timestampNs;
        }
    }

    void predictTo(int64_t timestampNs)
    {
        if (!initialized_ || !lidarOnly_ || timestampNs <= timestampNs_) {
            return;
        }
        const double dt = double(timestampNs - timestampNs_) * 1.0e-9;
        position_ += velocity_ * dt;
        const double angle = angularVelocity_.norm() * dt;
        if (angle > 0.0) {
            orientation_ = (orientation_ *
                Eigen::Quaterniond(Eigen::AngleAxisd(angle, angularVelocity_.normalized()))).normalized();
        }
        timestampNs_ = timestampNs;
    }

    bool initialized() const { return initialized_; }
    bool lidarOnly() const { return lidarOnly_; }
    int64_t timestampNs() const { return timestampNs_; }

    SlamPose pose() const
    {
        SlamPose result;
        result.timestampNs = timestampNs_;
        result.tx = position_.x();
        result.ty = position_.y();
        result.tz = position_.z();
        result.qx = orientation_.x();
        result.qy = orientation_.y();
        result.qz = orientation_.z();
        result.qw = orientation_.w();
        result.poseFrame = QStringLiteral("slam_world");
        return result;
    }

private:
    std::deque<SlamImuSample> history_;
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroBias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d accelBias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_ = Eigen::Vector3d::Zero();
    double accelerationScale_ = 1.0;
    int64_t timestampNs_ = 0;
    bool initialized_ = false;
    bool lidarOnly_ = false;
};

struct HighRateOdometryChannel {
    std::mutex mutex;
    FastLioPredictionState correction;
    uint64_t generation = 0;
};

SlamOutput statusOutput(SlamStatusCode status, const QString& message)
{
    SlamOutput output;
    output.status = status;
    output.message = message;
    output.imuHealthy = status == SlamStatusCode::Running ||
                        status == SlamStatusCode::InitializingImu;
    return output;
}

bool isSlamErrorStatus(SlamStatusCode status)
{
    return status == SlamStatusCode::Failed ||
           status == SlamStatusCode::MissingImu ||
           status == SlamStatusCode::TimeSyncError ||
           status == SlamStatusCode::Degraded;
}

bool isSlamDialogFailureStatus(SlamStatusCode status)
{
    return status == SlamStatusCode::Failed ||
           status == SlamStatusCode::MissingImu ||
           status == SlamStatusCode::TimeSyncError;
}

PointCloudPoint toPointCloudPoint(const SlamPoint& point)
{
    PointCloudPoint result;
    result.x = point.x;
    result.y = point.y;
    result.z = point.z;
    result.r = 1.0f;
    result.g = 1.0f;
    result.b = 1.0f;
    result.reflectivity = point.reflectivity;
    result.tag = point.tag;
    result.line = point.line;
    result.spherical = false;
    result.theta = 0.0f;
    result.phi = 0.0f;
    result.depth = 0.0f;
    return result;
}

Eigen::Isometry3d slamPoseTransform(const SlamPose& pose)
{
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = Eigen::Quaterniond(pose.qw, pose.qx, pose.qy, pose.qz)
                             .normalized()
                             .toRotationMatrix();
    transform.translation() = Eigen::Vector3d(pose.tx, pose.ty, pose.tz);
    return transform;
}

void setSlamPoseTransform(SlamPose& pose, const Eigen::Isometry3d& transform)
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

qint64 replayTargetMs(int64_t firstFrameStartNs, int64_t frameStartNs)
{
    const int64_t elapsedNs = frameStartNs - firstFrameStartNs;
    if (elapsedNs <= 0) {
        return 0;
    }
    return qint64(elapsedNs / int64_t{1000000});
}

QString formatSlamTimeNs(uint64_t timeNs, uint64_t durationNs)
{
    const uint64_t totalMs = timeNs / uint64_t{1000000};
    const uint64_t durationMs = std::max<uint64_t>(uint64_t{1}, durationNs / uint64_t{1000000});
    const uint64_t totalTenths = totalMs / uint64_t{100};
    const uint64_t totalSeconds = totalTenths / uint64_t{10};
    const uint64_t tenths = totalTenths % uint64_t{10};

    if (durationMs < uint64_t{60000}) {
        return QStringLiteral("%1.%2s").arg(totalSeconds).arg(tenths);
    }

    const uint64_t seconds = totalSeconds % uint64_t{60};
    const uint64_t totalMinutes = totalSeconds / uint64_t{60};
    if (durationMs < uint64_t{3600000}) {
        return QStringLiteral("%1:%2.%3")
            .arg(totalMinutes)
            .arg(seconds, 2, 10, QLatin1Char('0'))
            .arg(tenths);
    }

    const uint64_t minutes = totalMinutes % uint64_t{60};
    const uint64_t hours = totalMinutes / uint64_t{60};
    return QStringLiteral("%1:%2:%3.%4")
        .arg(hours)
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 2, 10, QLatin1Char('0'))
        .arg(tenths);
}

QString trajectoryExportExtension(SlamTrajectoryExport::Format format)
{
    return format == SlamTrajectoryExport::Format::Csv ? QStringLiteral("csv") : QStringLiteral("tum");
}

QString trajectoryExportFilter(SlamTrajectoryExport::Format format)
{
    return format == SlamTrajectoryExport::Format::Csv
        ? QStringLiteral("CSV 轨迹 (*.csv)")
        : QStringLiteral("TUM 轨迹 (*.tum *.txt)");
}

QStringList trajectoryExportFilters()
{
    return {trajectoryExportFilter(SlamTrajectoryExport::Format::Csv),
            trajectoryExportFilter(SlamTrajectoryExport::Format::Tum)};
}

SlamTrajectoryExport::Format trajectoryExportFormatFromSelection(const QString& selectedFilter, const QString& filePath)
{
    if (selectedFilter == trajectoryExportFilter(SlamTrajectoryExport::Format::Tum)) {
        return SlamTrajectoryExport::Format::Tum;
    }
    const QString suffix = QFileInfo(filePath).suffix().toLower();
    return (suffix == QStringLiteral("tum") || suffix == QStringLiteral("txt"))
        ? SlamTrajectoryExport::Format::Tum
        : SlamTrajectoryExport::Format::Csv;
}

QString mapExportExtension(bool lasFormat)
{
    return lasFormat ? QStringLiteral("las") : QStringLiteral("pcd");
}

QString mapExportFilter(bool lasFormat)
{
    return lasFormat ? QStringLiteral("LAS 点云 (*.las)") : QStringLiteral("PCD 点云 (*.pcd)");
}

QStringList mapExportFilters()
{
    return {mapExportFilter(false), mapExportFilter(true)};
}

bool mapExportLasFormatFromSelection(const QString& selectedFilter, const QString& filePath)
{
    if (selectedFilter == mapExportFilter(true)) {
        return true;
    }
    return QFileInfo(filePath).suffix().compare(QStringLiteral("las"), Qt::CaseInsensitive) == 0;
}

QString normalizedExportPath(const QString& filePath,
                             const QStringList& allowedSuffixes,
                             const QString& primarySuffix,
                             const QStringList& replaceableSuffixes)
{
    QFileInfo info(filePath);
    const QString suffix = info.suffix().toLower();
    if (allowedSuffixes.contains(suffix)) {
        return filePath;
    }
    if (suffix.isEmpty()) {
        return filePath + QStringLiteral(".") + primarySuffix;
    }
    if (replaceableSuffixes.contains(suffix)) {
        return info.dir().filePath(info.completeBaseName() + QStringLiteral(".") + primarySuffix);
    }
    return filePath + QStringLiteral(".") + primarySuffix;
}

QString normalizedTrajectoryExportPath(const QString& filePath, SlamTrajectoryExport::Format format)
{
    if (format == SlamTrajectoryExport::Format::Tum) {
        return normalizedExportPath(filePath,
                                    {QStringLiteral("tum"), QStringLiteral("txt")},
                                    QStringLiteral("tum"),
                                    {QStringLiteral("csv"), QStringLiteral("tum"), QStringLiteral("txt")});
    }
    return normalizedExportPath(filePath,
                                {QStringLiteral("csv")},
                                QStringLiteral("csv"),
                                {QStringLiteral("csv"), QStringLiteral("tum"), QStringLiteral("txt")});
}

QString normalizedMapExportPath(const QString& filePath, bool lasFormat)
{
    return normalizedExportPath(filePath,
                                {mapExportExtension(lasFormat)},
                                mapExportExtension(lasFormat),
                                {QStringLiteral("pcd"), QStringLiteral("las")});
}

QString liveInputWaitingMessage(const LiveLidarSlamSourceStats& stats)
{
    if (!stats.message.isEmpty()) {
        return stats.message;
    }
    if (stats.pointPacketCount == 0 && stats.imuPacketCount == 0) {
        return QStringLiteral("未收到实时 LiDAR/IMU 数据。请确认雷达已连接、SDK 已启动、网络接口正确，并且设备正在输出点云和 IMU。");
    }
    if (stats.pointPacketCount == 0) {
        return QStringLiteral("未收到实时点云数据。请确认设备正在输出点云，且数据回调已进入 SLAM 输入源。");
    }
    if (stats.imuPacketCount == 0) {
        return QStringLiteral("已收到实时点云，但未收到 IMU 数据。FAST_LIO 需要连续 IMU 覆盖当前帧。");
    }
    if (stats.inputFrameCount == 0) {
        return QStringLiteral("实时 SLAM 暖机中，正在等待 IMU 覆盖点云帧。");
    }
    return QStringLiteral("正在等待实时 SLAM 输入帧。");
}

SlamStatusCode liveInputWaitingStatus(const LiveLidarSlamSourceStats& stats, bool alreadyRunning)
{
    if (stats.status == SlamStatusCode::TimeSyncError ||
        stats.status == SlamStatusCode::MissingImu ||
        stats.status == SlamStatusCode::Degraded) {
        return stats.status;
    }
    if (alreadyRunning) {
        return SlamStatusCode::Running;
    }
    if (stats.pointPacketCount > 0 && stats.imuPacketCount > 0) {
        return SlamStatusCode::InitializingImu;
    }
    return SlamStatusCode::Starting;
}

QString skippedLiveFrameMessage(const SlamInputFrame& frame, const LiveLidarSlamSourceStats& stats)
{
    if (!stats.message.isEmpty()) {
        return stats.message;
    }
    if (!frame.hasPointOffsetTime) {
        return QStringLiteral("实时点云帧缺少可用点内 offset 时间，无法执行 FAST_LIO 去畸变。");
    }
    if (!frame.hasCompleteImuCoverage) {
        return frame.imuSamples.isEmpty()
            ? QStringLiteral("实时输入帧缺少 IMU 样本，已跳过当前帧。")
            : QStringLiteral("实时 IMU 样本未完整覆盖当前点云帧，已跳过当前帧。");
    }
    return QStringLiteral("实时输入帧不完整，已跳过当前帧。");
}

QString offlineSourceKindForPath(const QString& filePath)
{
    const QString suffix = QFileInfo(filePath).suffix().toLower();
    if (suffix == QStringLiteral("bag")) {
        return QStringLiteral("ROSbag");
    }
    if (suffix == QStringLiteral("db3") ||
        suffix == QStringLiteral("yaml") ||
        suffix == QStringLiteral("yml")) {
        return QStringLiteral("ROS2 db3");
    }
    if (suffix == QStringLiteral("lvx2")) {
        return QStringLiteral("LVX2");
    }
    if (suffix == QStringLiteral("lvx")) {
        return QStringLiteral("LVX");
    }
    return QStringLiteral("PCAP");
}

} // namespace

SlamUiBridge* LivoxViewerWindow::ensureSlamUiBridge()
{
    if (slamUiBridge) {
        return slamUiBridge;
    }

    qRegisterMetaType<SlamRenderSnapshot>("SlamRenderSnapshot");
    qRegisterMetaType<SlamRenderPose>("SlamRenderPose");
    slamUiBridge = new SlamUiBridge(this);
    slamUiBridge->setWorldFrameColor(slamWorldCurrentFrameColor);
    slamUiBridge->setBodyFrameColor(slamBodyFrameColor);
    slamUiBridge->setDynamicObjectColor(slamDynamicObjectColor);
    slamUiBridge->setDynamicAggressiveColor(slamDynamicAggressiveColor);
    slamUiBridge->setDynamicModerateColor(slamDynamicModerateColor);
    slamUiBridge->setDynamicConservativeColor(slamDynamicConservativeColor);
    slamUiBridge->setFreeDomScanVoxelColor(slamFreeDomScanVoxelColor);
    slamUiBridge->setFreeDomDynamicVoxelColor(slamFreeDomDynamicVoxelColor);
    slamUiBridge->setFreeDomRaycastedVoxelColor(slamFreeDomRaycastedVoxelColor);
    slamUiBridge->setFreeDomFreeVoxelColor(slamFreeDomFreeVoxelColor);
    slamUiBridge->setFreeDomStaticVoxelColor(slamFreeDomStaticVoxelColor);
    slamUiBridge->setFreeDomEnhancedColor(slamFreeDomEnhancedColor);
    slamUiBridge->setTrajectoryColor(slamTrajectoryColor);
    slamUiBridge->setWorldFramePointSize(slamWorldCurrentFramePointSizePx);
    slamUiBridge->setBodyFramePointSize(slamBodyFramePointSizePx);
    slamUiBridge->setDynamicObjectPointSize(slamDynamicObjectPointSizePx);
    slamUiBridge->setFreeDomEnhancedPointSize(slamFreeDomEnhancedPointSizePx);
    slamUiBridge->setTrajectoryLineWidth(slamTrajectoryLineWidthPx);
    slamUiBridge->setPoseAxisLength(slamPoseAxisLengthM);
    slamUiBridge->setPoseAxisLineWidth(slamPoseAxisLineWidthPx);
    syncSlamRenderLayerVisibility();
    connect(slamUiBridge, &SlamUiBridge::statusTextReady, this, [this](const QString& text) {
        if (statusBar()) {
            statusBar()->showMessage(text, 2000);
        }
    });
    connect(slamUiBridge, &SlamUiBridge::renderSnapshotReady, this, [this](const SlamRenderSnapshot& snapshot) {
        if (!slamRenderOverlayEnabled) {
            return;
        }
        if (slamPointCloudView) {
            slamPointCloudView->setSlamRenderSnapshot(snapshot);
        }
    });
    connect(slamUiBridge, &SlamUiBridge::renderPoseReady, this, [this](const SlamRenderPose& pose) {
        if (slamPointCloudView) {
            slamPointCloudView->setSlamFollowPose(pose);
        }
    });
    connect(slamUiBridge, &SlamUiBridge::poseAxisVerticesReady, this,
            [this](const QVector<SlamRenderVertex>& vertices) {
                if (slamRenderOverlayEnabled && slamPointCloudView) {
                    slamPointCloudView->setSlamPoseAxisVertices(vertices);
                }
            });
    connect(slamUiBridge, &SlamUiBridge::displayStateChanged, this, &LivoxViewerWindow::updateSlamStatusPanel);
    showSlamStatusPanel();
    updateSlamStatusPanel();
    return slamUiBridge;
}

void LivoxViewerWindow::showSlamControlDialog()
{
    SlamUiBridge* bridge = ensureSlamUiBridge();
    if (!slamControlDialog) {
        slamControlDialog = new SlamControlDialog(this, bridge, this);
    }
    slamControlDialog->show();
    slamControlDialog->raise();
    slamControlDialog->activateWindow();
}

void LivoxViewerWindow::startOnlineSlamFromMenu()
{
    setSlamInputModeOnline();
    ensureSlamVisualizationTab(QStringLiteral("online"));
    showSlamInfoPanel();
    showSlamStatusPanel();
    if (!slamWorkerActive.load() && !slamWorker.joinable()) {
        postSlamStatus(SlamStatusCode::Idle, QStringLiteral("在线 SLAM 已准备，点击浮动控制条“开始”开始。"));
    }
    updateSlamControlBarUi();
}

void LivoxViewerWindow::startOfflineSlamFromMenu()
{
    setSlamInputModeOffline();
    if (!loadOfflineSlamSource()) {
        return;
    }
    showSlamInfoPanel();
    showSlamStatusPanel();
    if (!slamWorkerActive.load() && !slamWorker.joinable()) {
        postSlamStatus(SlamStatusCode::Idle,
                       QStringLiteral("离线 SLAM %1 已加载，点击浮动控制条“开始”开始。")
                           .arg(slamOfflineSourceDisplayName));
    }
    updateSlamControlBarUi();
}

void LivoxViewerWindow::setSlamInputModeOffline()
{
    slamInputMode = SlamInputMode::Offline;
    ensureSlamUiBridge()->setModeAndBackend(QStringLiteral("离线 SLAM"),
                                            slamBackendDisplayName(slamRuntimeConfig));
    updateSlamControlBarUi();
}

void LivoxViewerWindow::setSlamInputModeOnline()
{
    slamInputMode = SlamInputMode::Online;
    ensureSlamUiBridge()->setModeAndBackend(QStringLiteral("在线 SLAM"),
                                            slamBackendDisplayName(slamRuntimeConfig));
    updateSlamControlBarUi();
}

bool LivoxViewerWindow::isOfflineSlamMode() const
{
    return slamInputMode == SlamInputMode::Offline;
}

QString LivoxViewerWindow::offlineSlamPcapPath() const
{
    return slamOfflineSourcePath;
}

QString LivoxViewerWindow::offlineSlamSourcePath() const
{
    return slamOfflineSourcePath;
}

void LivoxViewerWindow::toggleSlamPrimaryAction()
{
    if (slamWorkerStopping.load()) {
        return;
    }
    if (slamWorkerActive.load() && !slamWorkerPaused.load()) {
        pauseSlamProcessing();
        return;
    }
    if (slamWorkerActive.load() && slamWorkerPaused.load()) {
        slamWorkerPaused.store(false);
        postSlamStatus(SlamStatusCode::Running, QStringLiteral("SLAM 已继续。"));
        updateSlamControlBarUi();
        return;
    }
    startSlamProcessing();
}

void LivoxViewerWindow::handleSlamReplayModeChanged(int index)
{
    if (!slamReplayModeCombo || index < 0) {
        return;
    }
    if (!isOfflineSlamMode() || slamWorkerActive.load()) {
        QSignalBlocker blocker(slamReplayModeCombo);
        slamReplayModeCombo->setCurrentIndex(slamReplayMode == SlamReplayMode::Fast ? 1 : 0);
        return;
    }

    const SlamReplayMode mode = static_cast<SlamReplayMode>(slamReplayModeCombo->itemData(index).toInt());
    slamReplayMode = mode;
    updateSlamControlBarUi();
}

void LivoxViewerWindow::syncSlamTemplateControl()
{
    if (slamControlTemplateCombo) {
        QSignalBlocker blocker(slamControlTemplateCombo);
        const int value = static_cast<int>(slamRuntimeConfig.lidarTemplate);
        const int index = slamControlTemplateCombo->findData(value);
        if (index >= 0) {
            slamControlTemplateCombo->setCurrentIndex(index);
        }
    }
    if (slamReplayModeCombo) {
        QSignalBlocker blocker(slamReplayModeCombo);
        slamReplayModeCombo->setCurrentIndex(slamReplayMode == SlamReplayMode::Fast ? 1 : 0);
    }
    if (slamControlModeCombo) {
        QSignalBlocker blocker(slamControlModeCombo);
        slamControlModeCombo->setCurrentIndex(
            slamRuntimeConfig.allowPureLidar && !slamRuntimeConfig.imuEnabled ? 1 : 0);
    }
}

void LivoxViewerWindow::handleSlamTemplateControlChanged(int index)
{
    if (!slamControlTemplateCombo || index < 0) {
        return;
    }

    const SlamLidarTemplate selectedTemplate =
        slamLidarTemplateFromInt(slamControlTemplateCombo->itemData(index).toInt());
    if (selectedTemplate == slamRuntimeConfig.lidarTemplate) {
        return;
    }

    if (slamWorkerActive.load()) {
        syncSlamTemplateControl();
        logMessage(QStringLiteral("[SLAM] 运行中不能切换 LiDAR 模板。"));
        return;
    }

    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    slamRuntimeConfig = loadSlamRuntimeConfigForTemplate(settings,
                                                         QStringLiteral("slam/runtime"),
                                                         selectedTemplate);
    settings.setValue(QStringLiteral("slam/runtime/lidarTemplate"), static_cast<int>(selectedTemplate));
    liveSlamSource.setFrameDurationMs(slamRuntimeConfig.inputFrameDurationMs);
    rebuildSlamInfoPanel();
    syncSlamRenderLayerVisibility();
    syncSlamTemplateControl();
    updateSlamControlBarUi();
    logMessage(QStringLiteral("[SLAM] LiDAR 模板已切换为 %1。").arg(slamLidarTemplateDisplayName(selectedTemplate)));
}

void LivoxViewerWindow::handleSlamModeControlChanged(int index)
{
    if (!slamControlModeCombo || index < 0) {
        return;
    }

    const bool lidarOnly = slamControlModeCombo->itemData(index).toInt() == 1;
    if (lidarOnly == (slamRuntimeConfig.allowPureLidar && !slamRuntimeConfig.imuEnabled)) {
        return;
    }

    if (slamWorkerActive.load()) {
        syncSlamTemplateControl();
        logMessage(QStringLiteral("[SLAM] 运行中不能切换 LIO/LO 模式。"));
        return;
    }

    slamRuntimeConfig.allowPureLidar = lidarOnly;
    slamRuntimeConfig.imuEnabled = !lidarOnly;
    slamRuntimeConfig.backendType = lidarOnly ? QStringLiteral("FAST_LO") : QStringLiteral("FAST_LIO");
    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    saveSlamRuntimeConfig(settings, slamRuntimeConfig, QStringLiteral("slam/runtime"));
    syncSlamTemplateControl();
    updateSlamControlBarUi();
    logMessage(lidarOnly
        ? QStringLiteral("[SLAM] 已切换为 LO 模式。")
        : QStringLiteral("[SLAM] 已切换为 LIO 模式。"));
}

bool LivoxViewerWindow::loadOfflineSlamSource()
{
    setSlamInputModeOffline();
    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    QString lastDir = settings.value(QStringLiteral("slam/lastOfflineSourceDir"),
                                     QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
    if (lastDir.isEmpty()) {
        lastDir = QDir::homePath();
    }

    QFileDialog dialog(this);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setWindowTitle(QStringLiteral("加载离线 SLAM 数据源"));
    dialog.setDirectory(lastDir);
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setNameFilters({
        QStringLiteral("SLAM 数据源 (*.pcap *.pcapng *.bag *.db3 *.yaml *.yml *.lvx *.lvx2)"),
        QStringLiteral("PCAP 文件 (*.pcap *.pcapng)"),
        QStringLiteral("ROS1 Bag 文件 (*.bag)"),
        QStringLiteral("ROS2 db3 文件 (*.db3 *.yaml *.yml)"),
        QStringLiteral("Livox LVX/LVX2 文件 (*.lvx *.lvx2)"),
        QStringLiteral("所有文件 (*.*)")
    });
    if (dialog.exec() != QDialog::Accepted || dialog.selectedFiles().isEmpty()) {
        return false;
    }

    const QString filePath = dialog.selectedFiles().first();
    if (slamWorker.joinable()) {
        stopSlamProcessing();
    }
    const QString suffix = QFileInfo(filePath).suffix().toLower();
    if (suffix == QStringLiteral("lvx") || suffix == QStringLiteral("lvx2")) {
        slamOfflineSourceKind = SlamOfflineSourceKind::Lvx;
    } else if (suffix == QStringLiteral("bag") ||
               suffix == QStringLiteral("db3") ||
               suffix == QStringLiteral("yaml") ||
               suffix == QStringLiteral("yml")) {
        slamOfflineSourceKind = SlamOfflineSourceKind::Rosbag;
    } else {
        slamOfflineSourceKind = SlamOfflineSourceKind::Pcap;
    }
    slamOfflineSourcePath = filePath;
    slamOfflineSourceDisplayName = offlineSourceKindForPath(filePath);
    settings.setValue(QStringLiteral("slam/lastOfflineSourceDir"), QFileInfo(filePath).absolutePath());
    ensureSlamVisualizationTab(filePath);
    clearSlamDisplay();
    ensureSlamUiBridge()->setModeAndBackend(QStringLiteral("离线 SLAM"),
                                            slamBackendDisplayName(slamRuntimeConfig));
    logMessage(QStringLiteral("[SLAM] 已加载离线 %1: %2")
                   .arg(slamOfflineSourceDisplayName, QDir::toNativeSeparators(filePath)));
    if (statusBar()) {
        statusBar()->showMessage(QStringLiteral("离线 SLAM %1 已加载").arg(slamOfflineSourceDisplayName), 3000);
    }
    updateSlamControlBarUi();
    return true;
}

bool LivoxViewerWindow::loadOfflineSlamPcap()
{
    return loadOfflineSlamSource();
}

int LivoxViewerWindow::ensureSlamVisualizationTab(const QString& sourcePath)
{
    if (slamVisualizationTabId >= 0 && slamPointCloudView) {
        if (!sourcePath.isEmpty()) {
            visualizationWorkspace->setTabToolTip(slamVisualizationTabId, QDir::toNativeSeparators(sourcePath));
        }
        visualizationWorkspace->activateTab(slamVisualizationTabId);
        updateSlamControlBarUi();
        return slamVisualizationTabId;
    }

    slamPointCloudView = new PointCloudView(visualizationWorkspace);
    slamPointCloudView->installEventFilter(this);
    slamPointCloudView->setMinimumSize(200, 200);
    slamPointCloudView->setPointSize(pointSizePx);
    slamPointCloudView->setEdlConfig(pointCloudEdlConfig);
    slamPointCloudView->setGridConfig(pointCloudGridConfig);
    slamPointCloudView->setGridVisible(realtimePointCloudView->isGridVisible());
    slamPointCloudView->setMeasurementModeEnabled(measurementModeActive);
    slamPointCloudView->setSelectionModeEnabled(selectionRealtimeEnabled);
    connect(slamPointCloudView, &PointCloudView::lvx2FileDropped, this, &LivoxViewerWindow::onLvx2PlaybackFileDropped);
    connect(slamPointCloudView, &PointCloudView::selectionPointsReady, this, &LivoxViewerWindow::onSelectionPointsReady);
    connect(slamPointCloudView, &PointCloudView::crossSectionChanged, this, [this](int clippedPointCount, int sourcePointCount) {
        if (crossSectionModeActive && statusLabelBar) {
            statusLabelBar->setText(QString("点云裁切：%1 / %2 点").arg(clippedPointCount).arg(sourcePointCount));
        }
    });

    slamVisualizationTabId = visualizationWorkspace->addTab(
        VisualizationWorkspace::TabKind::SlamPointCloud,
        QStringLiteral("SLAM"),
        slamPointCloudView,
        true);
    if (!sourcePath.isEmpty()) {
        visualizationWorkspace->setTabToolTip(slamVisualizationTabId, QDir::toNativeSeparators(sourcePath));
    }
    pointCloudViewsByTab.insert(slamVisualizationTabId, slamPointCloudView);
    applyPointCloudBackground();
    updatePointCloudLegend();
    visualizationWorkspace->activateTab(slamVisualizationTabId);
    updateSlamControlBarUi();
    return slamVisualizationTabId;
}

bool LivoxViewerWindow::isSlamPointCloudTab(int tabId) const
{
    return visualizationWorkspace &&
           tabId >= 0 &&
           tabId == slamVisualizationTabId &&
           visualizationWorkspace->tabKind(tabId) == VisualizationWorkspace::TabKind::SlamPointCloud;
}

void LivoxViewerWindow::startSlamProcessing()
{
    SlamUiBridge* bridge = ensureSlamUiBridge();
    bridge->setModeAndBackend(isOfflineSlamMode() ? QStringLiteral("离线 SLAM") : QStringLiteral("在线 SLAM"),
                              slamBackendDisplayName(slamRuntimeConfig));

    if (slamWorkerStopping.load()) {
        postSlamStatus(SlamStatusCode::Stopped, QStringLiteral("SLAM 正在停止，请等待后端资源释放完成。"));
        return;
    }

    if (slamWorker.joinable() && !slamWorkerActive.load()) {
        slamWorker.join();
    }

    if (slamWorker.joinable()) {
        if (slamWorkerPaused.load()) {
            slamWorkerPaused.store(false);
            postSlamStatus(SlamStatusCode::Running, QStringLiteral("SLAM 已恢复。"));
        } else {
            postSlamStatus(SlamStatusCode::Running, QStringLiteral("SLAM 已在运行。"));
        }
        updateSlamControlBarUi();
        return;
    }

    slamFailureDialogShown = false;

    if (slamInputMode == SlamInputMode::Online) {
        if (connectedLidarDevicesSnapshot().isEmpty()) {
            const QString message = QStringLiteral("没有可用的已连接设备，无法启动在线 SLAM。");
            showSlamStatusPanel();
            postSlamStatus(SlamStatusCode::Failed, message);
            QMessageBox::warning(this, QStringLiteral("在线 SLAM"), message);
            updateSlamControlBarUi();
            return;
        }

        const SlamRuntimeConfig config = slamRuntimeConfig;
        liveSlamSource.reset();
        liveSlamSource.setFrameDurationMs(config.inputFrameDurationMs);
        liveSlamSource.setQueueCapacity(config.maxInputQueueFrames);
        slamWorkerCancel.store(false);
        slamWorkerPaused.store(false);
        slamWorkerStopping.store(false);
        slamWorkerActive.store(true);
        slamProgressValue = 0;
        slamProgressMaximum = 0;
        slamProgressIndeterminate = true;
        slamProgressSourceText = QStringLiteral("在线 SLAM");
        slamProgressTimeText = QStringLiteral("输入 FPS: 0.0");
        slamProgressFrameText = QStringLiteral("已处理帧: 0");
        slamRenderOverlayEnabled = true;
        ensureSlamVisualizationTab(QStringLiteral("online"));
        bridge->clearDisplay();
        clearSlamWorldPointCloud();
        showSlamInfoPanel();
        showSlamStatusPanel();
        postSlamStatus(SlamStatusCode::Starting,
                       QStringLiteral("正在启动在线 SLAM：%1 Hz / %2 ms。")
                           .arg(config.preprocessScanRateHz, 0, 'f', 1)
                           .arg(config.inputFrameDurationMs));
        updateSlamControlBarUi();

        slamWorker = std::thread([this, config]() {
            auto postOutput = [this](SlamOutput output) {
                QMetaObject::invokeMethod(this, [this, output = std::move(output)]() mutable {
                    submitSlamOutputForUi(output);
                    updateSlamControlBarUi();
                }, Qt::QueuedConnection);
            };
            auto postOnlineProgress = [this](double inputFps, int processedFrames) {
                QMetaObject::invokeMethod(this, [this, inputFps, processedFrames]() {
                    slamProgressSourceText = QStringLiteral("在线 SLAM");
                    slamProgressTimeText = QStringLiteral("输入 FPS: %1").arg(inputFps, 0, 'f', 1);
                    slamProgressFrameText = QStringLiteral("已处理帧: %1").arg(processedFrames);
                    updateSlamControlBarUi();
                }, Qt::QueuedConnection);
            };
            auto postControlUpdate = [this]() {
                QMetaObject::invokeMethod(this, [this]() {
                    finishSlamWorkerUi();
                }, Qt::QueuedConnection);
            };
            auto postSlamLog = [this](QString message) {
                QMetaObject::invokeMethod(this, [this, message = std::move(message)]() {
                    logMessage(QStringLiteral("[SLAM] %1").arg(message));
                }, Qt::QueuedConnection);
            };
            auto postDisplayReset = [this]() {
                QMetaObject::invokeMethod(this, [this]() {
                    if (SlamUiBridge* bridge = ensureSlamUiBridge()) {
                        bridge->clearDisplay();
                    }
                    clearSlamWorldPointCloud();
                    updateSlamControlBarUi();
                }, Qt::QueuedConnection);
            };

            FastLioSlamBackend backend;
            QString error;
            if (!backend.start(config, &error)) {
                postOutput(statusOutput(SlamStatusCode::Failed, error));
                slamWorkerActive.store(false);
                postControlUpdate();
                return;
            }

            HighRateOdometryChannel odometryChannel;
            std::atomic_bool odometryStop{false};
            liveSlamSource.setOdometryImuEnabled(true);
            std::thread odometryWorker([this, &postOutput, &odometryChannel, &odometryStop]() {
                HighRateOdometryPredictor predictor;
                uint64_t correctionGeneration = 0;
                auto nextPublish = std::chrono::steady_clock::now();
                while (!odometryStop.load()) {
                    nextPublish += kOdometryPublishPeriod;
                    predictor.appendSamples(liveSlamSource.takeOdometryImuSamples());

                    FastLioPredictionState correction;
                    uint64_t generation = 0;
                    {
                        std::lock_guard<std::mutex> lock(odometryChannel.mutex);
                        correction = odometryChannel.correction;
                        generation = odometryChannel.generation;
                    }
                    if (generation != correctionGeneration) {
                        correctionGeneration = generation;
                        if (correction.valid) {
                            predictor.applyCorrection(correction);
                        }
                    } else {
                        predictor.integratePendingSamples();
                    }
                    if (predictor.lidarOnly()) {
                        predictor.predictTo(predictor.timestampNs() + int64_t{kOdometryPublishPeriod.count()} *
                                                                        int64_t{1000000});
                    }

                    if (!slamWorkerPaused.load() && correction.valid && predictor.initialized()) {
                        SlamOutput output;
                        output.status = SlamStatusCode::Running;
                        output.currentPose = predictor.pose();
                        output.currentPoseValid = true;
                        output.imuHealthy = !predictor.lidarOnly();
                        output.odometryOnly = true;
                        postOutput(std::move(output));
                    }
                    std::this_thread::sleep_until(nextPublish);
                }
            });

            QElapsedTimer elapsed;
            elapsed.start();
            int processedFrames = 0;
            int skippedFrames = 0;
            SlamOutput lastOutput;
            bool hasLastOutput = false;
            qint64 lastInputWarningMs = -1000;
            uint64_t observedBackendHardResetGeneration = liveSlamSource.stats().backendHardResetGeneration;

            while (!slamWorkerCancel.load()) {
                if (slamWorkerPaused.load()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }

                const LiveLidarSlamSourceStats loopStats = liveSlamSource.stats();
                if (loopStats.backendHardResetGeneration != observedBackendHardResetGeneration) {
                    observedBackendHardResetGeneration = loopStats.backendHardResetGeneration;
                    error.clear();
                    if (!backend.reset(&error)) {
                        postOutput(statusOutput(SlamStatusCode::Failed, error));
                        break;
                    }
                    {
                        std::lock_guard<std::mutex> lock(odometryChannel.mutex);
                        odometryChannel.correction = FastLioPredictionState();
                        ++odometryChannel.generation;
                    }
                    hasLastOutput = false;
                    lastOutput = SlamOutput();
                    const QString resetReason = loopStats.lastResetReason.isEmpty()
                        ? liveInputWaitingMessage(loopStats)
                        : loopStats.lastResetReason;
                    const QString resetMessage = QStringLiteral(
                        "实时 SLAM 后端已重置：%1 last=%2ms, current=%3ms, jump=%4ms, handle=%5, time_type=%6, generation=%7。")
                                                     .arg(resetReason)
                                                     .arg(QString::number(double(loopStats.lastResetRawTimestampNs) / 1000000.0, 'f', 3))
                                                     .arg(QString::number(double(loopStats.currentResetRawTimestampNs) / 1000000.0, 'f', 3))
                                                     .arg(QString::number(double(loopStats.lastResetJumpNs) / 1000000.0, 'f', 3))
                                                     .arg(loopStats.lastResetHandle)
                                                     .arg(int(loopStats.lastResetTimeType))
                                                     .arg(loopStats.backendHardResetGeneration);
                    SlamOutput resetOutput = statusOutput(loopStats.lastResetStatus == SlamStatusCode::Idle
                                                              ? SlamStatusCode::Degraded
                                                              : loopStats.lastResetStatus,
                                                          resetMessage);
                    resetOutput.inputFps = loopStats.inputFps;
                    resetOutput.droppedFrameCount = int(loopStats.droppedFrameCount) + skippedFrames;
                    postDisplayReset();
                    postSlamLog(resetMessage);
                    postOutput(resetOutput);
                    postOnlineProgress(loopStats.inputFps, processedFrames);
                    lastInputWarningMs = elapsed.elapsed();
                }

                SlamInputFrame frame;
                if (!liveSlamSource.inputQueue().tryPop(frame)) {
                    const qint64 nowMs = elapsed.elapsed();
                    const LiveLidarSlamSourceStats stats = liveSlamSource.stats();
                    if (hasLastOutput &&
                        stats.status != SlamStatusCode::TimeSyncError &&
                        stats.status != SlamStatusCode::MissingImu &&
                        stats.status != SlamStatusCode::Degraded) {
                        postOnlineProgress(stats.inputFps, processedFrames);
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                        continue;
                    }
                    const bool shouldPostWaiting =
                        !hasLastOutput ||
                        stats.status == SlamStatusCode::Starting ||
                        stats.status == SlamStatusCode::InitializingImu ||
                        stats.status == SlamStatusCode::Degraded ||
                        stats.status == SlamStatusCode::TimeSyncError ||
                        stats.status == SlamStatusCode::MissingImu;
                    if (shouldPostWaiting && nowMs - lastInputWarningMs >= 1000) {
                        SlamOutput waitingOutput = statusOutput(liveInputWaitingStatus(stats, hasLastOutput || processedFrames > 0),
                                                                liveInputWaitingMessage(stats));
                        waitingOutput.inputFps = stats.inputFps;
                        waitingOutput.droppedFrameCount = int(stats.droppedFrameCount) + skippedFrames;
                        lastOutput = waitingOutput;
                        postOutput(waitingOutput);
                        postOnlineProgress(stats.inputFps, processedFrames);
                        lastInputWarningMs = nowMs;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }

                if (!frame.hasPointOffsetTime || !frame.hasCompleteImuCoverage) {
                    ++skippedFrames;
                    const qint64 nowMs = elapsed.elapsed();
                    if (nowMs - lastInputWarningMs >= 1000) {
                        const LiveLidarSlamSourceStats stats = liveSlamSource.stats();
                        SlamOutput skippedOutput = statusOutput(liveInputWaitingStatus(stats, hasLastOutput || processedFrames > 0),
                                                               skippedLiveFrameMessage(frame, stats));
                        skippedOutput.inputFps = stats.inputFps;
                        skippedOutput.droppedFrameCount = int(stats.droppedFrameCount) + skippedFrames;
                        lastOutput = skippedOutput;
                        postOutput(skippedOutput);
                        postOnlineProgress(stats.inputFps, processedFrames);
                        lastInputWarningMs = nowMs;
                    }
                    continue;
                }

                SlamOutput output;
                error.clear();
                const bool accepted = backend.processFrame(frame, &output, &error);
                ++processedFrames;
                const LiveLidarSlamSourceStats stats = liveSlamSource.stats();
                output.inputFps = stats.inputFps;
                output.droppedFrameCount = int(stats.droppedFrameCount) + skippedFrames;
                if (!error.isEmpty() && output.message.isEmpty()) {
                    output.message = error;
                }
                lastOutput = output;
                hasLastOutput = true;
                const FastLioPredictionState correction = backend.predictionState(frame.frameEndNs);
                if (correction.valid) {
                    std::lock_guard<std::mutex> lock(odometryChannel.mutex);
                    odometryChannel.correction = correction;
                    ++odometryChannel.generation;
                }
                postOutput(output);
                postOnlineProgress(output.inputFps, processedFrames);

                if (!accepted &&
                    output.status != SlamStatusCode::InitializingImu &&
                    output.status != SlamStatusCode::TimeSyncError) {
                    break;
                }
            }

            odometryStop.store(true);
            odometryWorker.join();
            liveSlamSource.setOdometryImuEnabled(false);
            SlamOutput finalOutput = hasLastOutput ? lastOutput : statusOutput(SlamStatusCode::Stopped, QString());
            error.clear();
            backend.finalize(&finalOutput, &error);
            backend.stop();
            const bool cancelled = slamWorkerCancel.load();
            finalOutput.status = SlamStatusCode::Stopped;
            finalOutput.message = error.isEmpty()
                ? (cancelled ? QStringLiteral("SLAM 已停止。") : QStringLiteral("在线 SLAM 已结束。"))
                : error;
            finalOutput.newTrajectoryPoints.clear();
            finalOutput.newGlobalMapPoints.clear();
            finalOutput.publishedWorldFramePoints.clear();
            finalOutput.publishedBodyFramePoints.clear();
            finalOutput.dynamicDetectionFrameWorldPoints.clear();
            finalOutput.dynamicWorldFramePoints.clear();
            const LiveLidarSlamSourceStats stats = liveSlamSource.stats();
            finalOutput.inputFps = stats.inputFps;
            finalOutput.droppedFrameCount = int(stats.droppedFrameCount) + skippedFrames;
            if (processedFrames > 0 && finalOutput.inputFps <= 0.0) {
                const double elapsedSec = qMax(0.001, double(elapsed.elapsed()) / 1000.0);
                finalOutput.inputFps = double(processedFrames) / elapsedSec;
            }
            postOnlineProgress(finalOutput.inputFps, processedFrames);
            postOutput(finalOutput);
            slamWorkerActive.store(false);
            postControlUpdate();
        });
        return;
    }

    if (slamOfflineSourcePath.isEmpty() || slamOfflineSourceKind == SlamOfflineSourceKind::None) {
        postSlamStatus(SlamStatusCode::Failed, QStringLiteral("请先加载离线 SLAM 数据源。"));
        return;
    }

    const QString sourcePath = slamOfflineSourcePath;
    const SlamOfflineSourceKind sourceKind = slamOfflineSourceKind;
    const QString sourceDisplayName = slamOfflineSourceDisplayName.isEmpty()
        ? offlineSourceKindForPath(sourcePath)
        : slamOfflineSourceDisplayName;
    SlamRuntimeConfig config = slamRuntimeConfig;
    config.deterministicOfflineLoopClosure = true;
    const SlamReplayMode replayMode = slamReplayMode;
    slamWorkerCancel.store(false);
    slamWorkerPaused.store(false);
    slamWorkerStopping.store(false);
    slamWorkerActive.store(true);
    slamProgressValue = 0;
    slamProgressMaximum = 0;
    slamProgressIndeterminate = true;
    slamFinalLoopClosureActive = false;
    slamProgressSourceText = QDir::toNativeSeparators(sourcePath);
    slamProgressTimeText = QStringLiteral("时间 - / -");
    slamProgressFrameText = QStringLiteral("帧 0 / 0");
    slamRenderOverlayEnabled = true;
    bridge->clearDisplay();
    clearSlamWorldPointCloud();
    ensureSlamVisualizationTab(sourcePath);
    showSlamInfoPanel();
    showSlamStatusPanel();
    postSlamStatus(SlamStatusCode::Starting,
                   QStringLiteral("正在启动离线 SLAM（%1）：%2 Hz / %3 ms。")
                       .arg(sourceDisplayName)
                       .arg(config.preprocessScanRateHz, 0, 'f', 1)
                       .arg(config.inputFrameDurationMs));
    updateSlamControlBarUi();

    slamWorker = std::thread([this, sourcePath, sourceKind, sourceDisplayName, config, replayMode]() {
        auto postOutput = [this](SlamOutput output) {
            QMetaObject::invokeMethod(this, [this, output = std::move(output)]() mutable {
                submitSlamOutputForUi(output);
                updateSlamControlBarUi();
            }, Qt::QueuedConnection);
        };
        auto postOutputBatch = [this](QVector<SlamOutput> outputs) {
            QMetaObject::invokeMethod(this, [this, outputs = std::move(outputs)]() mutable {
                submitSlamOutputsForUi(outputs);
                updateSlamControlBarUi();
            }, Qt::QueuedConnection);
        };
        auto postLog = [this](QString message) {
            QMetaObject::invokeMethod(this, [this, message = std::move(message)]() {
                logMessage(message);
            }, Qt::QueuedConnection);
        };
        auto postProgress = [this](int value,
                                   int maximum,
                                   bool indeterminate,
                                   QString sourceText,
                                   QString timeText,
                                   QString frameText) {
            QMetaObject::invokeMethod(this,
                                      [this,
                                       value,
                                       maximum,
                                       indeterminate,
                                       sourceText = std::move(sourceText),
                                       timeText = std::move(timeText),
                                       frameText = std::move(frameText)]() {
                slamProgressValue = value;
                slamProgressMaximum = maximum;
                slamProgressIndeterminate = indeterminate;
                slamProgressSourceText = sourceText;
                slamProgressTimeText = timeText;
                slamProgressFrameText = frameText;
                updateSlamControlBarUi();
            }, Qt::QueuedConnection);
        };
        auto postControlUpdate = [this]() {
            QMetaObject::invokeMethod(this, [this]() {
                finishSlamWorkerUi();
            }, Qt::QueuedConnection);
        };

        QString error;
        std::unique_ptr<PcapSlamSource> pcapSource;
        std::unique_ptr<RosbagSlamSource> rosbagSource;
        std::unique_ptr<LvxSlamSource> lvxSource;
        QString summaryText;
        int totalFrameCount = 0;
        if (sourceKind == SlamOfflineSourceKind::Pcap) {
            pcapSource = std::make_unique<PcapSlamSource>(config.inputFrameDurationMs);
        } else if (sourceKind == SlamOfflineSourceKind::Rosbag) {
            RosbagSlamSourceConfig rosbagConfig;
            rosbagConfig.frameDurationMs = config.inputFrameDurationMs;
            rosbagConfig.requireImu = !isLidarOnlyConfig(config);
            rosbagConfig.requirePointOffsetTime = !isLidarOnlyConfig(config);
            rosbagConfig.allowLivoxDriver2PointCloud2 = config.allowRosbagDriver2PointCloud2;
            rosbagConfig.allowLivoxDriverPointCloud2SynthesizedTime =
                config.allowRosbagDriverPointCloud2SynthesizedTime;
            rosbagConfig.lidarToImuTimeOffsetNs = config.lidarToImuTimeOffsetNs;
            rosbagSource = std::make_unique<RosbagSlamSource>(rosbagConfig);
        } else if (sourceKind == SlamOfflineSourceKind::Lvx) {
            if (!isLidarOnlyConfig(config)) {
                postOutput(statusOutput(SlamStatusCode::MissingImu,
                                        QStringLiteral("LVX/LVX2 离线 SLAM 需要在首选项中启用纯激光里程计。")));
                slamWorkerActive.store(false);
                postControlUpdate();
                return;
            }
            lvxSource = std::make_unique<LvxSlamSource>(config.inputFrameDurationMs);
            if (!lvxSource->open(sourcePath, &slamWorkerCancel, &error)) {
                if (slamWorkerCancel.load()) {
                    slamWorkerActive.store(false);
                    postControlUpdate();
                    return;
                }
                postOutput(statusOutput(SlamStatusCode::Failed, error));
                slamWorkerActive.store(false);
                postControlUpdate();
                return;
            }
            totalFrameCount = lvxSource->estimatedFrameCount();
            summaryText = lvxSource->summaryText();
        } else {
            postOutput(statusOutput(SlamStatusCode::Failed, QStringLiteral("未知离线 SLAM 数据源类型。")));
            slamWorkerActive.store(false);
            postControlUpdate();
            return;
        }
        if (!summaryText.isEmpty()) {
            postLog(QStringLiteral("[SLAM] %1").arg(summaryText));
        }
        const QString sourceText = QDir::toNativeSeparators(sourcePath);
        const bool streamingSource = pcapSource || rosbagSource;
        int streamingProgressValue = 0;
        int64_t firstFrameStartNs = 0;
        int64_t lastFrameEndNs = 0;
        uint64_t totalDurationNs = streamingSource
            ? 0
            : static_cast<uint64_t>(std::max(1, totalFrameCount)) *
                  static_cast<uint64_t>(std::max(1, config.inputFrameDurationMs)) * uint64_t{1000000};
        auto makeTimeText = [&totalDurationNs](uint64_t elapsedNs) {
            if (totalDurationNs == 0) {
                return QStringLiteral("时间 %1")
                    .arg(formatSlamTimeNs(elapsedNs, std::max<uint64_t>(elapsedNs, uint64_t{1})));
            }
            const uint64_t clampedElapsedNs = std::min(elapsedNs, totalDurationNs);
            return QStringLiteral("时间 %1 / %2")
                .arg(formatSlamTimeNs(clampedElapsedNs, totalDurationNs))
                .arg(formatSlamTimeNs(totalDurationNs, totalDurationNs));
        };
        auto makeFrameText = [](int value, int maximum) {
            return maximum > 0
                ? QStringLiteral("帧 %1 / %2").arg(value).arg(maximum)
                : QStringLiteral("帧 %1").arg(value);
        };
        postProgress(0,
                     streamingSource ? kStreamingProgressMaximum : totalFrameCount,
                     !streamingSource && totalFrameCount <= 0,
                     sourceText,
                     makeTimeText(0),
                     makeFrameText(0, totalFrameCount));

        FastLioSlamBackend backend;
        if (!backend.start(config, &error)) {
            postOutput(statusOutput(SlamStatusCode::Failed, error));
            slamWorkerActive.store(false);
            postControlUpdate();
            return;
        }

        QElapsedTimer elapsed;
        elapsed.start();
        int processedFrames = 0;
        int droppedFrames = 0;
        bool hasFirstFrameStart = false;
        int64_t firstReplayFrameEndNs = 0;
        int64_t lastVisitedFrameStartNs = 0;
        qint64 pausedReplayMs = 0;
        QElapsedTimer replayClock;
        SlamOutput lastOutput;
        bool hasLastOutput = false;
        int progressFrames = 0;
        QString sourceReadError;
        HighRateOdometryPredictor offlineOdometryPredictor;
        QElapsedTimer fastOutputPublishClock;
        QElapsedTimer fastProgressPublishClock;
        fastOutputPublishClock.start();
        fastProgressPublishClock.start();
        QVector<SlamOutput> pendingFastOutputs;

        auto publishOfflineOutput = [replayMode,
                                     &postOutput,
                                     &postOutputBatch,
                                     &fastOutputPublishClock,
                                     &pendingFastOutputs](SlamOutput output, bool force) {
            if (replayMode == SlamReplayMode::Fast) {
                pendingFastOutputs.push_back(std::move(output));
                if (!force && fastOutputPublishClock.elapsed() < kFastReplayUiPublishIntervalMs) {
                    return;
                }
                postOutputBatch(std::move(pendingFastOutputs));
                pendingFastOutputs.clear();
                fastOutputPublishClock.restart();
                return;
            }
            postOutput(std::move(output));
        };

        auto publishOfflineProgress = [replayMode,
                                       streamingSource,
                                       &postProgress,
                                       &fastProgressPublishClock,
                                       &sourceText,
                                       &makeTimeText,
                                       &makeFrameText,
                                       &progressFrames,
                                       &totalFrameCount,
                                       &streamingProgressValue](uint64_t elapsedFrameNs, bool force) {
            if (replayMode == SlamReplayMode::Fast &&
                !force &&
                fastProgressPublishClock.elapsed() < kFastReplayUiPublishIntervalMs) {
                return;
            }
            postProgress(streamingSource ? streamingProgressValue : progressFrames,
                         streamingSource ? kStreamingProgressMaximum : totalFrameCount,
                         !streamingSource && totalFrameCount <= 0,
                         sourceText,
                         makeTimeText(elapsedFrameNs),
                         makeFrameText(progressFrames, streamingSource ? 0 : totalFrameCount));
            if (replayMode == SlamReplayMode::Fast) {
                fastProgressPublishClock.restart();
            }
        };
        auto updateStreamingProgress = [&streamingProgressValue](int64_t value, int64_t maximum) {
            const int64_t clampedValue = std::clamp(value, int64_t{0}, maximum);
            streamingProgressValue = std::max(
                streamingProgressValue,
                static_cast<int>(double(clampedValue) * double(kStreamingProgressMaximum) / double(maximum)));
        };

        auto waitUntilRunning = [this, &pausedReplayMs]() {
            if (!slamWorkerPaused.load()) {
                return !slamWorkerCancel.load();
            }
            QElapsedTimer pauseTimer;
            pauseTimer.start();
            while (slamWorkerPaused.load() && !slamWorkerCancel.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            pausedReplayMs += pauseTimer.elapsed();
            return !slamWorkerCancel.load();
        };

        auto publishHighRatePose = [&postOutput,
                                    &offlineOdometryPredictor,
                                    &firstReplayFrameEndNs,
                                    &config](qint64 replayElapsedMs) {
            if (isLidarOnlyConfig(config) || !offlineOdometryPredictor.initialized()) {
                return;
            }
            offlineOdometryPredictor.integrateUntil(
                firstReplayFrameEndNs + int64_t(replayElapsedMs) * int64_t{1000000});
            SlamOutput odometryOutput;
            odometryOutput.status = SlamStatusCode::Running;
            odometryOutput.currentPose = offlineOdometryPredictor.pose();
            odometryOutput.currentPoseValid = true;
            odometryOutput.imuHealthy = true;
            odometryOutput.odometryOnly = true;
            postOutput(std::move(odometryOutput));
        };

        auto waitForReplayTarget = [this,
                                    &waitUntilRunning,
                                    &pausedReplayMs,
                                    &replayClock,
                                    &publishHighRatePose](qint64 targetMs) {
            while (!slamWorkerCancel.load()) {
                if (!waitUntilRunning()) {
                    return false;
                }
                const qint64 replayElapsedMs = replayClock.elapsed() - pausedReplayMs;
                publishHighRatePose(replayElapsedMs);
                const qint64 remainingMs = targetMs - replayElapsedMs;
                if (remainingMs <= 0) {
                    return true;
                }
                const qint64 sleepMs = std::min<qint64>(remainingMs, kOdometryPublishPeriod.count());
                std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
            }
            return false;
        };

        auto processFrame = [this,
                             &backend,
                             &config,
                             replayMode,
                             &elapsed,
                             &processedFrames,
                             &droppedFrames,
                             &hasFirstFrameStart,
                             &firstFrameStartNs,
                             &firstReplayFrameEndNs,
                             &lastVisitedFrameStartNs,
                             &lastFrameEndNs,
                             &replayClock,
                             &offlineOdometryPredictor,
                             &waitForReplayTarget,
                             &waitUntilRunning,
                             &publishOfflineOutput,
                             &publishOfflineProgress,
                             &totalFrameCount,
                             &lastOutput,
                             &hasLastOutput,
                             &progressFrames,
                             &error](SlamInputFrame&& frame) {
            if (slamWorkerCancel.load()) {
                return false;
            }
            ++progressFrames;
            lastVisitedFrameStartNs = frame.frameStartNs;
            lastFrameEndNs = frame.frameEndNs > frame.frameStartNs
                ? frame.frameEndNs
                : frame.frameStartNs + int64_t(config.inputFrameDurationMs) * int64_t{1000000};
            if (!hasFirstFrameStart) {
                hasFirstFrameStart = true;
                firstFrameStartNs = frame.frameStartNs;
                firstReplayFrameEndNs = lastFrameEndNs;
                replayClock.start();
            }
            if (!isLidarOnlyConfig(config)) {
                offlineOdometryPredictor.appendSamples(frame.imuSamples);
            }
            if (replayMode == SlamReplayMode::Timed &&
                !waitForReplayTarget(replayTargetMs(firstReplayFrameEndNs, lastFrameEndNs))) {
                return false;
            }
            if (!waitUntilRunning()) {
                return false;
            }
            const uint64_t elapsedFrameNs =
                frame.frameStartNs >= firstFrameStartNs ? uint64_t(frame.frameStartNs - firstFrameStartNs) : 0ULL;
            if (!isLidarOnlyConfig(config) && !frame.hasCompleteImuCoverage) {
                ++droppedFrames;
                publishOfflineProgress(elapsedFrameNs, false);
                return true;
            }

            SlamOutput output;
            error.clear();
            const bool accepted = backend.processFrame(frame, &output, &error);
            ++processedFrames;
            const double elapsedSec = qMax(0.001, double(elapsed.elapsed()) / 1000.0);
            output.inputFps = double(processedFrames) / elapsedSec;
            output.droppedFrameCount = droppedFrames;
            if (!error.isEmpty() && output.message.isEmpty()) {
                output.message = error;
            }
            lastOutput = output;
            hasLastOutput = true;
            if (!isLidarOnlyConfig(config)) {
                const FastLioPredictionState correction = backend.predictionState(frame.frameEndNs);
                if (correction.valid) {
                    offlineOdometryPredictor.applyCorrection(correction);
                }
            }
            const bool continueProcessing = accepted ||
                output.status == SlamStatusCode::InitializingImu ||
                output.status == SlamStatusCode::TimeSyncError;
            publishOfflineOutput(std::move(output), !continueProcessing);
            publishOfflineProgress(elapsedFrameNs, !continueProcessing);

            if (!continueProcessing) {
                return false;
            }
            return true;
        };

        if (pcapSource) {
            pcapSource->streamFrames(sourcePath,
                                     !isLidarOnlyConfig(config),
                                     &slamWorkerCancel,
                                     processFrame,
                                     updateStreamingProgress,
                                     &sourceReadError);
            summaryText = pcapSource->summaryText();
        } else if (rosbagSource) {
            rosbagSource->streamFrames(sourcePath,
                                       &slamWorkerCancel,
                                       processFrame,
                                       updateStreamingProgress,
                                       &sourceReadError);
            summaryText = rosbagSource->summaryText();
        } else {
            while (!slamWorkerCancel.load()) {
                SlamInputFrame frame;
                if (!lvxSource->readNextFrame(&frame, &slamWorkerCancel, &sourceReadError)) {
                    break;
                }
                if (!processFrame(std::move(frame))) {
                    break;
                }
            }
            summaryText = lvxSource->summaryText();
        }
        if (!summaryText.isEmpty()) {
            postLog(QStringLiteral("[SLAM] %1").arg(summaryText));
        }

        uint64_t lastElapsedFrameNs = 0;
        if (hasFirstFrameStart && lastVisitedFrameStartNs > firstFrameStartNs) {
            lastElapsedFrameNs = uint64_t(lastVisitedFrameStartNs - firstFrameStartNs);
        }
        if (replayMode == SlamReplayMode::Fast && !pendingFastOutputs.isEmpty()) {
            postOutputBatch(std::move(pendingFastOutputs));
            pendingFastOutputs.clear();
            fastOutputPublishClock.restart();
        }
        if (replayMode == SlamReplayMode::Fast) {
            publishOfflineProgress(lastElapsedFrameNs, true);
        }

        const bool finalLoopClosure = !slamWorkerCancel.load() &&
            sourceReadError.isEmpty() &&
            config.loopClosureEnableFlag;
        if (finalLoopClosure) {
            QMetaObject::invokeMethod(this, [this]() {
                slamFinalLoopClosureActive = true;
                slamProgressIndeterminate = true;
                slamProgressTimeText = QStringLiteral("正在执行最终回环优化并重建全局地图…");
                postSlamStatus(SlamStatusCode::Running,
                               QStringLiteral("正在执行最终回环优化并重建全局地图…"));
                updateSlamControlBarUi();
            }, Qt::QueuedConnection);
        }

        SlamOutput finalOutput = hasLastOutput ? lastOutput : statusOutput(SlamStatusCode::Stopped, QString());
        error.clear();
        backend.finalize(&finalOutput, &error);
        backend.stop();
        const bool cancelled = slamWorkerCancel.load();
        const bool sourceReadFailed = !cancelled && !sourceReadError.isEmpty();
        if (!cancelled && !sourceReadFailed) {
            totalFrameCount = progressFrames;
            if (streamingSource) {
                streamingProgressValue = kStreamingProgressMaximum;
            }
            if (hasFirstFrameStart && lastFrameEndNs > firstFrameStartNs) {
                totalDurationNs = uint64_t(lastFrameEndNs - firstFrameStartNs);
            }
        }
        finalOutput.status = sourceReadFailed ? SlamStatusCode::Failed : SlamStatusCode::Stopped;
        finalOutput.message = !error.isEmpty()
            ? error
            : (sourceReadFailed
            ? sourceReadError
            : (cancelled ? QStringLiteral("SLAM 已停止。")
                         : QStringLiteral("%1 SLAM 已完成。").arg(sourceDisplayName)));
        finalOutput.droppedFrameCount = droppedFrames;
        if (processedFrames > 0) {
            const double elapsedSec = qMax(0.001, double(elapsed.elapsed()) / 1000.0);
            finalOutput.inputFps = double(processedFrames) / elapsedSec;
        }
        finalOutput.newTrajectoryPoints.clear();
        finalOutput.newGlobalMapPoints.clear();
        finalOutput.publishedWorldFramePoints.clear();
        finalOutput.publishedBodyFramePoints.clear();
        finalOutput.dynamicDetectionFrameWorldPoints.clear();
        finalOutput.dynamicWorldFramePoints.clear();
        const int finalProgress = streamingSource
            ? streamingProgressValue
            : (cancelled || sourceReadFailed ? progressFrames : totalFrameCount);
        const int finalMaximum = streamingSource ? kStreamingProgressMaximum : totalFrameCount;
        const int finalFrameCount = cancelled || sourceReadFailed ? progressFrames : totalFrameCount;
        uint64_t finalElapsedNs = totalDurationNs;
        if (cancelled || sourceReadFailed) {
            finalElapsedNs = 0;
            if (hasFirstFrameStart && progressFrames > 0) {
                const int64_t elapsedNs = lastVisitedFrameStartNs - firstFrameStartNs;
                if (elapsedNs > 0) {
                    finalElapsedNs = uint64_t(elapsedNs);
                }
            }
        }
        postProgress(finalProgress,
                     finalMaximum,
                     false,
                     sourceText,
                     makeTimeText(finalElapsedNs),
                     makeFrameText(finalFrameCount, totalFrameCount));
        postOutput(finalOutput);
        if (finalLoopClosure) {
            QMetaObject::invokeMethod(this, [this]() {
                slamFinalLoopClosureActive = false;
                updateSlamControlBarUi();
            }, Qt::QueuedConnection);
        }
        slamWorkerActive.store(false);
        postControlUpdate();
    });
}

void LivoxViewerWindow::pauseSlamProcessing()
{
    ensureSlamUiBridge();
    if (slamWorkerStopping.load()) {
        return;
    }
    if (!slamWorker.joinable() || !slamWorkerActive.load()) {
        postSlamStatus(SlamStatusCode::Paused, QStringLiteral("当前没有正在运行的 SLAM。"));
        updateSlamControlBarUi();
        return;
    }
    slamWorkerPaused.store(true);
    postSlamStatus(SlamStatusCode::Paused, QStringLiteral("SLAM 已暂停。"));
    updateSlamControlBarUi();
}

void LivoxViewerWindow::stopSlamProcessing()
{
    ensureSlamUiBridge();
    if (!slamWorker.joinable()) {
        slamProgressValue = 0;
        slamProgressMaximum = 0;
        slamProgressIndeterminate = false;
        postSlamStatus(SlamStatusCode::Stopped, QStringLiteral("SLAM 已停止。"));
        updateSlamControlBarUi();
        return;
    }
    if (slamWorkerStopping.exchange(true)) {
        return;
    }
    slamWorkerCancel.store(true);
    slamWorkerPaused.store(false);
    postSlamStatus(SlamStatusCode::Stopped, QStringLiteral("正在停止 SLAM 并释放后端地图资源…"));
    updateSlamControlBarUi();
}

void LivoxViewerWindow::finishSlamWorkerUi()
{
    Q_ASSERT(QThread::currentThread() == qApp->thread());
    if (slamWorker.joinable()) {
        slamWorker.join();
    }
    const bool stoppedByUser = slamWorkerStopping.exchange(false);
    slamFinalLoopClosureActive = false;
    if (stoppedByUser) {
        if (slamResetPending) {
            slamResetPending = false;
            clearSlamDisplay();
            postSlamStatus(SlamStatusCode::Idle, QStringLiteral("SLAM 已重置。"));
        } else {
            postSlamStatus(SlamStatusCode::Stopped, QStringLiteral("SLAM 已停止。"));
            ensureSlamUiBridge()->resetCurrentPose();
        }
        slamProgressValue = 0;
        slamProgressMaximum = 0;
        slamProgressIndeterminate = false;
    }
    updateSlamControlBarUi();
}

void LivoxViewerWindow::resetSlamProcessing()
{
    if (slamWorker.joinable()) {
        slamResetPending = true;
        stopSlamProcessing();
        return;
    }
    clearSlamDisplay();
    postSlamStatus(SlamStatusCode::Idle, QStringLiteral("SLAM 已重置。"));
    updateSlamControlBarUi();
}

void LivoxViewerWindow::clearSlamDisplay()
{
    slamProgressValue = 0;
    slamProgressMaximum = 0;
    slamProgressIndeterminate = false;
    slamFinalLoopClosureActive = false;
    slamProgressSourceText.clear();
    slamProgressTimeText.clear();
    slamProgressFrameText.clear();
    if (SlamUiBridge* bridge = ensureSlamUiBridge()) {
        bridge->clearDisplay();
    }
    clearSlamWorldPointCloud();
    forEachPointCloudView([](PointCloudView* view) {
        if (view) {
            view->clearSlamRenderOverlay();
        }
    });
    updateSlamControlBarUi();
}

void LivoxViewerWindow::appendSlamWorldFramePoints(const SlamOutput& output, bool refreshDisplay)
{
    if (!slamRuntimeConfig.publishWorldFrameCloud || !slamPointCloudView) {
        return;
    }

    if (output.optimizedTrajectoryReset && !output.optimizedTrajectory.isEmpty()) {
        correctSlamWorldPointSegments(output.optimizedTrajectory);
        slamWorldDisplayedSegmentStart = 0;
        slamWorldDisplayedSegmentEnd = 0;
        slamPointCloudView->clearPointCloudSegments();
        if (refreshDisplay && slamWorldFrameVisible) {
            refreshSlamWorldPointCloud();
        }
    }

    if (output.publishedWorldFramePoints.isEmpty()) {
        return;
    }

    SlamWorldPointSegment segment;
    segment.timestampNs = output.currentPose.timestampNs;
    segment.poseCorrectionEpoch = output.poseCorrectionEpoch;
    segment.pose = output.currentPose;
    if (segment.timestampNs <= 0 && !slamWorldPointSegments.isEmpty()) {
        segment.timestampNs =
            slamWorldPointSegments.back().timestampNs +
            static_cast<int64_t>(slamRuntimeConfig.inputFrameDurationMs) * int64_t{1000000};
    }
    segment.points.reserve(output.publishedWorldFramePoints.size());
    for (const SlamPoint& point : output.publishedWorldFramePoints) {
        segment.points.push_back(toPointCloudPoint(point));
    }
    slamWorldPointSegments.push_back(std::move(segment));

    const int64_t latestTimestampNs = slamWorldPointSegments.back().timestampNs;
    int removedHistorySegmentCount = 0;
    while (!slamWorldPointSegments.isEmpty() &&
           slamWorldPointSegments.front().timestampNs < latestTimestampNs - kSlamWorldHistoryRetentionNs) {
        ++removedHistorySegmentCount;
        slamWorldPointSegments.remove(0);
    }
    if (removedHistorySegmentCount > 0) {
        const int displayedSegmentCount = std::max(0, slamWorldDisplayedSegmentEnd - slamWorldDisplayedSegmentStart);
        const int displayedRemovedCount =
            std::clamp(removedHistorySegmentCount - slamWorldDisplayedSegmentStart, 0, displayedSegmentCount);
        for (int i = 0; i < displayedRemovedCount && slamPointCloudView; ++i) {
            slamPointCloudView->removeFirstPointCloudSegment();
        }
        slamWorldDisplayedSegmentStart = std::max(0, slamWorldDisplayedSegmentStart - removedHistorySegmentCount);
        slamWorldDisplayedSegmentEnd = std::max(slamWorldDisplayedSegmentStart,
                                                slamWorldDisplayedSegmentEnd - removedHistorySegmentCount);
    }
    if (refreshDisplay && slamWorldFrameVisible) {
        refreshSlamWorldPointCloud();
    }
}

void LivoxViewerWindow::correctSlamWorldPointSegments(
    const QVector<SlamTrajectoryPoint>& optimizedTrajectory)
{
    if (slamWorldPointSegments.isEmpty()) {
        return;
    }

    struct PoseCorrection {
        int64_t timestampNs = 0;
        Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    };

    QVector<QVector<PoseCorrection>> correctionsByEpoch;
    int segmentIndex = 0;
    for (const SlamTrajectoryPoint& optimizedPoint : optimizedTrajectory) {
        while (segmentIndex + 1 < slamWorldPointSegments.size() &&
               std::abs(slamWorldPointSegments.at(segmentIndex + 1).timestampNs -
                        optimizedPoint.pose.timestampNs) <
                   std::abs(slamWorldPointSegments.at(segmentIndex).timestampNs -
                            optimizedPoint.pose.timestampNs)) {
            ++segmentIndex;
        }
        const Eigen::Isometry3d correction =
            slamPoseTransform(optimizedPoint.pose) *
            slamPoseTransform(slamWorldPointSegments.at(segmentIndex).pose).inverse();
        const int epoch = slamWorldPointSegments.at(segmentIndex).poseCorrectionEpoch;
        if (correctionsByEpoch.size() <= epoch) {
            correctionsByEpoch.resize(epoch + 1);
        }
        correctionsByEpoch[epoch].push_back({optimizedPoint.pose.timestampNs,
                                             Eigen::Quaterniond(correction.rotation()),
                                             correction.translation()});
    }

    SlamWorldPointSegment* segments = slamWorldPointSegments.data();
    const int segmentCount = slamWorldPointSegments.size();
    const int threadCount = std::max(1, slamRuntimeConfig.numberOfCores);
#pragma omp parallel for num_threads(threadCount)
    for (int segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex) {
        SlamWorldPointSegment& segment = segments[segmentIndex];
        const QVector<PoseCorrection>& corrections =
            correctionsByEpoch.at(segment.poseCorrectionEpoch);
        const auto upper = std::lower_bound(
            corrections.cbegin(),
            corrections.cend(),
            segment.timestampNs,
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
            const double alpha = double(segment.timestampNs - before.timestampNs) /
                                 double(upper->timestampNs - before.timestampNs);
            rotation = before.rotation.slerp(alpha, upper->rotation).normalized();
            translation = before.translation * (1.0 - alpha) + upper->translation * alpha;
        }

        const Eigen::Matrix3f rotationMatrix = rotation.toRotationMatrix().cast<float>();
        const Eigen::Vector3f translationVector = translation.cast<float>();
        for (PointCloudPoint& point : segment.points) {
            const Eigen::Vector3f corrected =
                rotationMatrix * Eigen::Vector3f(point.x, point.y, point.z) + translationVector;
            point.x = corrected.x();
            point.y = corrected.y();
            point.z = corrected.z();
        }

        Eigen::Isometry3d correctionTransform = Eigen::Isometry3d::Identity();
        correctionTransform.linear() = rotation.toRotationMatrix();
        correctionTransform.translation() = translation;
        setSlamPoseTransform(segment.pose, correctionTransform * slamPoseTransform(segment.pose));
    }
}

void LivoxViewerWindow::refreshSlamWorldPointCloud()
{
    if (!slamPointCloudView) {
        return;
    }
    if (!slamRuntimeConfig.publishWorldFrameCloud || !slamWorldFrameVisible) {
        slamPointCloudView->clearPointCloudSegments();
        slamWorldDisplayedSegmentStart = 0;
        slamWorldDisplayedSegmentEnd = 0;
        return;
    }
    if (slamWorldPointSegments.isEmpty()) {
        slamPointCloudView->clearPointCloudSegments();
        slamWorldDisplayedSegmentStart = 0;
        slamWorldDisplayedSegmentEnd = 0;
        return;
    }

    const int64_t latestTimestampNs = slamWorldPointSegments.back().timestampNs;
    const int64_t windowNs =
        static_cast<int64_t>(std::max<uint64_t>(uint64_t{1}, slamWorldDisplayWindowMs)) * int64_t{1000000};
    const int64_t windowStartNs = latestTimestampNs > windowNs ? latestTimestampNs - windowNs : 0;

    int targetStart = 0;
    while (targetStart < slamWorldPointSegments.size() &&
           slamWorldPointSegments.at(targetStart).timestampNs < windowStartNs) {
        ++targetStart;
    }
    const int targetEnd = slamWorldPointSegments.size();

    auto appendDisplayedSegment = [this](const SlamWorldPointSegment& segment) {
        PointCloudFrame frame;
        frame.timestamp = static_cast<uint64_t>(std::max<int64_t>(int64_t{0}, segment.timestampNs));
        frame.device_handle = 0;
        frame.points = segment.points;
        applyPointCloudPipeline(frame, slamPointCloudView);
        slamPointCloudView->appendPointCloudSegment(std::move(frame.points));
    };

    const bool needsRebuild =
        slamWorldDisplayedSegmentStart > slamWorldDisplayedSegmentEnd ||
        slamWorldDisplayedSegmentEnd > slamWorldPointSegments.size() ||
        (slamWorldDisplayedSegmentStart == 0 &&
         slamWorldDisplayedSegmentEnd == 0 &&
         targetStart > 0) ||
        targetStart < slamWorldDisplayedSegmentStart ||
        targetEnd < slamWorldDisplayedSegmentEnd;
    if (needsRebuild) {
        slamPointCloudView->clearPointCloudSegments();
        slamWorldDisplayedSegmentStart = targetStart;
        slamWorldDisplayedSegmentEnd = targetStart;
    }

    while (slamWorldDisplayedSegmentStart < targetStart) {
        slamPointCloudView->removeFirstPointCloudSegment();
        ++slamWorldDisplayedSegmentStart;
    }
    while (slamWorldDisplayedSegmentEnd < targetEnd) {
        appendDisplayedSegment(slamWorldPointSegments.at(slamWorldDisplayedSegmentEnd));
        ++slamWorldDisplayedSegmentEnd;
    }
}

void LivoxViewerWindow::clearSlamWorldPointCloud()
{
    slamWorldPointSegments.clear();
    slamWorldDisplayedSegmentStart = 0;
    slamWorldDisplayedSegmentEnd = 0;
    if (slamPointCloudView) {
        slamPointCloudView->clearPointCloudSegments();
    }
}

void LivoxViewerWindow::setSlamWorldFrameVisible(bool visible)
{
    if (slamWorldFrameVisible == visible) {
        return;
    }
    slamWorldFrameVisible = visible;
    if (!visible && slamPointCloudView) {
        slamPointCloudView->clearPointCloudSegments();
        slamWorldDisplayedSegmentStart = 0;
        slamWorldDisplayedSegmentEnd = 0;
        return;
    }
    refreshSlamWorldPointCloud();
}

void LivoxViewerWindow::setSlamWorldCurrentFrameVisible(bool visible)
{
    if (slamWorldCurrentFrameVisible == visible) {
        return;
    }
    slamWorldCurrentFrameVisible = visible;
    syncSlamRenderLayerVisibility();
}

void LivoxViewerWindow::setSlamBodyFrameVisible(bool visible)
{
    if (slamBodyFrameVisible == visible) {
        return;
    }
    slamBodyFrameVisible = visible;
    syncSlamRenderLayerVisibility();
}

void LivoxViewerWindow::setSlamDynamicObjectVisible(bool visible)
{
    if (slamDynamicObjectVisible == visible) {
        return;
    }
    slamDynamicObjectVisible = visible;
    syncSlamRenderLayerVisibility();
}

void LivoxViewerWindow::setSlamFreeDomScanVoxelVisible(bool visible)
{
    if (slamFreeDomScanVoxelVisible == visible) {
        return;
    }
    slamFreeDomScanVoxelVisible = visible;
    syncSlamRenderLayerVisibility();
}

void LivoxViewerWindow::setSlamFreeDomDynamicVoxelVisible(bool visible)
{
    if (slamFreeDomDynamicVoxelVisible == visible) {
        return;
    }
    slamFreeDomDynamicVoxelVisible = visible;
    syncSlamRenderLayerVisibility();
}

void LivoxViewerWindow::setSlamFreeDomRaycastedVoxelVisible(bool visible)
{
    if (slamFreeDomRaycastedVoxelVisible == visible) {
        return;
    }
    slamFreeDomRaycastedVoxelVisible = visible;
    syncSlamRenderLayerVisibility();
}

void LivoxViewerWindow::setSlamFreeDomFreeVoxelVisible(bool visible)
{
    if (slamFreeDomFreeVoxelVisible == visible) {
        return;
    }
    slamFreeDomFreeVoxelVisible = visible;
    syncSlamRenderLayerVisibility();
}

void LivoxViewerWindow::setSlamFreeDomStaticVoxelVisible(bool visible)
{
    if (slamFreeDomStaticVoxelVisible == visible) {
        return;
    }
    slamFreeDomStaticVoxelVisible = visible;
    syncSlamRenderLayerVisibility();
}

void LivoxViewerWindow::setSlamFreeDomEnhancedVisible(bool visible)
{
    if (slamFreeDomEnhancedVisible == visible) {
        return;
    }
    slamFreeDomEnhancedVisible = visible;
    syncSlamRenderLayerVisibility();
}

void LivoxViewerWindow::setSlamTrajectoryVisible(bool visible)
{
    if (slamTrajectoryVisible == visible) {
        return;
    }
    slamTrajectoryVisible = visible;
    syncSlamRenderLayerVisibility();
}

void LivoxViewerWindow::setSlamPoseAxisVisible(bool visible)
{
    if (slamPoseAxisVisible == visible) {
        return;
    }
    slamPoseAxisVisible = visible;
    syncSlamRenderLayerVisibility();
}

void LivoxViewerWindow::syncSlamRenderLayerVisibility()
{
    if (!slamUiBridge) {
        return;
    }
    const bool freeDomLayersAvailable =
        slamRuntimeConfig.dynamicFilterEnabled &&
        slamRuntimeConfig.dynamicFilterBackend == DynamicFilterBackend::FreeDOM &&
        slamRuntimeConfig.dynamicDebugVisualizationEnabled;
    slamUiBridge->setRenderLayerVisibility(slamTrajectoryVisible,
                                           slamPoseAxisVisible,
                                           slamRuntimeConfig.publishWorldFrameCloud && slamWorldCurrentFrameVisible,
                                           slamRuntimeConfig.publishWorldFrameCloud &&
                                               slamRuntimeConfig.publishBodyFrameCloud &&
                                               slamBodyFrameVisible,
                                           slamRuntimeConfig.dynamicFilterEnabled &&
                                               slamDynamicObjectVisible,
                                           freeDomLayersAvailable && slamFreeDomScanVoxelVisible,
                                           freeDomLayersAvailable && slamFreeDomDynamicVoxelVisible,
                                           freeDomLayersAvailable && slamFreeDomRaycastedVoxelVisible,
                                           freeDomLayersAvailable && slamFreeDomFreeVoxelVisible,
                                           freeDomLayersAvailable && slamFreeDomStaticVoxelVisible,
                                           freeDomLayersAvailable &&
                                               slamRuntimeConfig.freeDom.raycastEnhancementEnabled &&
                                               slamFreeDomEnhancedVisible);
}

void LivoxViewerWindow::exportSlamTrajectoryFromDialog()
{
    SlamUiBridge* bridge = ensureSlamUiBridge();
    const QVector<SlamTrajectoryPoint> trajectory = bridge->trajectorySnapshot();
    if (trajectory.isEmpty()) {
        const QString message = QStringLiteral("当前没有可导出的 SLAM 轨迹。");
        bridge->setErrorMessage(message);
        logMessage(QStringLiteral("[SLAM] %1").arg(message));
        if (statusBar()) {
            statusBar()->showMessage(message, 3000);
        }
        QMessageBox::warning(this, QStringLiteral("保存 SLAM 轨迹失败"), message);
        return;
    }

    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    QString lastDir = settings.value(QStringLiteral("slam/lastTrajectoryExportDir"),
                                     QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
    if (lastDir.isEmpty()) {
        lastDir = QDir::homePath();
    }

    const QString defaultName = QStringLiteral("slam_trajectory_%1.csv")
                                    .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")));
    QFileDialog dialog(this);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setWindowTitle(QStringLiteral("保存 SLAM 轨迹"));
    dialog.setDirectory(lastDir);
    dialog.selectFile(defaultName);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setDefaultSuffix(QStringLiteral("csv"));
    dialog.setNameFilters(trajectoryExportFilters());
    dialog.selectNameFilter(trajectoryExportFilter(SlamTrajectoryExport::Format::Csv));
    if (dialog.exec() != QDialog::Accepted || dialog.selectedFiles().isEmpty()) {
        return;
    }

    const SlamTrajectoryExport::Format format =
        trajectoryExportFormatFromSelection(dialog.selectedNameFilter(), dialog.selectedFiles().first());
    const QString filePath = normalizedTrajectoryExportPath(dialog.selectedFiles().first(), format);
    QString error;
    if (!SlamTrajectoryExport::save(filePath, trajectory, format, &error)) {
        bridge->setErrorMessage(error);
        logMessage(QStringLiteral("[SLAM] 轨迹保存失败: %1").arg(error));
        if (statusBar()) {
            statusBar()->showMessage(error, 3000);
        }
        QMessageBox::warning(this, QStringLiteral("保存 SLAM 轨迹失败"), error);
        return;
    }

    settings.setValue(QStringLiteral("slam/lastTrajectoryExportDir"), QFileInfo(filePath).absolutePath());
    bridge->clearErrorMessage();
    logMessage(QStringLiteral("[SLAM] 轨迹保存完成: %1").arg(QDir::toNativeSeparators(filePath)));
    if (statusBar()) {
        statusBar()->showMessage(QStringLiteral("SLAM 轨迹已保存"), 3000);
    }
}

void LivoxViewerWindow::exportSlamGlobalMapFromDialog()
{
    SlamUiBridge* bridge = ensureSlamUiBridge();
    if (slamMapExportWorker.joinable() && !slamMapExportActive.load()) {
        slamMapExportWorker.join();
    }
    if (slamMapExportWorker.joinable()) {
        const QString message = QStringLiteral("完整全局地图正在导出，请等待当前导出完成。");
        bridge->setErrorMessage(message);
        logMessage(QStringLiteral("[SLAM] %1").arg(message));
        if (statusBar()) {
            statusBar()->showMessage(message, 3000);
        }
        QMessageBox::warning(this, QStringLiteral("保存 SLAM 全局地图失败"), message);
        return;
    }

    QVector<SlamPoint> points = bridge->globalMapSnapshot();
    if (points.isEmpty()) {
        const QString message = QStringLiteral("当前没有可导出的 SLAM 完整全局地图。请在 SLAM 设置中启用完整地图保存并重新运行 SLAM。");
        bridge->setErrorMessage(message);
        logMessage(QStringLiteral("[SLAM] %1").arg(message));
        if (statusBar()) {
            statusBar()->showMessage(message, 3000);
        }
        QMessageBox::warning(this, QStringLiteral("保存 SLAM 全局地图失败"), message);
        return;
    }

    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    QString lastDir = settings.value(QStringLiteral("slam/lastMapExportDir"),
                                     QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
    if (lastDir.isEmpty()) {
        lastDir = QDir::homePath();
    }

    const QString defaultName = QStringLiteral("slam_global_map_%1.pcd")
                                    .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")));
    QFileDialog dialog(this);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setWindowTitle(QStringLiteral("保存 SLAM 完整全局地图"));
    dialog.setDirectory(lastDir);
    dialog.selectFile(defaultName);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setDefaultSuffix(QStringLiteral("pcd"));
    dialog.setNameFilters(mapExportFilters());
    dialog.selectNameFilter(mapExportFilter(false));
    if (dialog.exec() != QDialog::Accepted || dialog.selectedFiles().isEmpty()) {
        return;
    }

    const bool lasFormat = mapExportLasFormatFromSelection(dialog.selectedNameFilter(), dialog.selectedFiles().first());
    const QString filePath = normalizedMapExportPath(dialog.selectedFiles().first(), lasFormat);
    settings.setValue(QStringLiteral("slam/lastMapExportDir"), QFileInfo(filePath).absolutePath());
    const int pointCount = points.size();
    const SlamMapExport::Format format = lasFormat ? SlamMapExport::Format::Las : SlamMapExport::Format::Pcd;
    slamMapExportActive.store(true);
    updateSlamControlBarUi();
    bridge->clearErrorMessage();
    logMessage(QStringLiteral("[SLAM] 开始保存完整全局地图: points=%1, file=%2")
                   .arg(QString::number(pointCount), QDir::toNativeSeparators(filePath)));
    if (statusBar()) {
        statusBar()->showMessage(QStringLiteral("正在保存 SLAM 完整全局地图"), 3000);
    }

    slamMapExportWorker = std::thread([this, filePath, points = std::move(points), format, pointCount]() {
        QString error;
        const bool ok = SlamMapExport::save(filePath, points, format, &error);
        QMetaObject::invokeMethod(this, [this, ok, error, filePath, pointCount]() {
            slamMapExportActive.store(false);
            updateSlamControlBarUi();
            if (!ok) {
                if (SlamUiBridge* bridge = ensureSlamUiBridge()) {
                    bridge->setErrorMessage(error);
                }
                logMessage(QStringLiteral("[SLAM] 完整全局地图保存失败: %1").arg(error));
                if (statusBar()) {
                    statusBar()->showMessage(error, 3000);
                }
                QMessageBox::warning(this, QStringLiteral("保存 SLAM 全局地图失败"), error);
                return;
            }
            if (SlamUiBridge* bridge = ensureSlamUiBridge()) {
                bridge->clearErrorMessage();
            }
            logMessage(QStringLiteral("[SLAM] 完整全局地图保存完成: points=%1, file=%2")
                           .arg(QString::number(pointCount), QDir::toNativeSeparators(filePath)));
            if (statusBar()) {
                statusBar()->showMessage(QStringLiteral("SLAM 完整全局地图已保存"), 3000);
            }
        }, Qt::QueuedConnection);
    });
}

void LivoxViewerWindow::exportFreeDomStaticPointMapFromDialog()
{
    exportFreeDomMapFromDialog(false);
}

void LivoxViewerWindow::exportFreeDomStaticVoxelMapFromDialog()
{
    exportFreeDomMapFromDialog(true);
}

void LivoxViewerWindow::exportFreeDomMapFromDialog(bool voxelCenters)
{
    SlamUiBridge* bridge = ensureSlamUiBridge();
    if (slamMapExportWorker.joinable() && !slamMapExportActive.load()) {
        slamMapExportWorker.join();
    }
    if (slamMapExportWorker.joinable()) {
        const QString message = QStringLiteral("地图正在导出，请等待当前导出完成。");
        bridge->setErrorMessage(message);
        logMessage(QStringLiteral("[FreeDOM] %1").arg(message));
        if (statusBar()) {
            statusBar()->showMessage(message, 3000);
        }
        return;
    }

    QVector<SlamPoint> points = voxelCenters
        ? bridge->freeDomStaticVoxelSnapshot()
        : bridge->freeDomStaticMapSnapshot();
    const QString mapName = voxelCenters
        ? QStringLiteral("静态 Voxel 地图")
        : QStringLiteral("静态点地图");
    if (points.isEmpty()) {
        const QString message = QStringLiteral("当前没有可导出的 FreeDOM %1。请启用 FreeDOM 并运行到地图快照生成。")
                                    .arg(mapName);
        bridge->setErrorMessage(message);
        logMessage(QStringLiteral("[FreeDOM] %1").arg(message));
        if (statusBar()) {
            statusBar()->showMessage(message, 3000);
        }
        return;
    }

    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    QString lastDir = settings.value(
        QStringLiteral("slam/lastMapExportDir"),
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
    if (lastDir.isEmpty()) {
        lastDir = QDir::homePath();
    }
    const QString baseName = voxelCenters
        ? QStringLiteral("freedom_static_voxel_map")
        : QStringLiteral("freedom_static_point_map");
    QFileDialog dialog(this);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setWindowTitle(QStringLiteral("保存 FreeDOM %1").arg(mapName));
    dialog.setDirectory(lastDir);
    dialog.selectFile(QStringLiteral("%1_%2.pcd")
                          .arg(baseName,
                               QDateTime::currentDateTime().toString(
                                   QStringLiteral("yyyyMMdd_HHmmss"))));
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setDefaultSuffix(QStringLiteral("pcd"));
    dialog.setNameFilters(mapExportFilters());
    dialog.selectNameFilter(mapExportFilter(false));
    if (dialog.exec() != QDialog::Accepted || dialog.selectedFiles().isEmpty()) {
        return;
    }

    const bool lasFormat = mapExportLasFormatFromSelection(
        dialog.selectedNameFilter(), dialog.selectedFiles().first());
    const QString filePath = normalizedMapExportPath(
        dialog.selectedFiles().first(), lasFormat);
    settings.setValue(QStringLiteral("slam/lastMapExportDir"),
                      QFileInfo(filePath).absolutePath());
    const int pointCount = points.size();
    const SlamMapExport::Format format = lasFormat
        ? SlamMapExport::Format::Las
        : SlamMapExport::Format::Pcd;
    slamMapExportActive.store(true);
    updateSlamControlBarUi();
    bridge->clearErrorMessage();
    logMessage(QStringLiteral("[FreeDOM] 开始保存%1: points=%2, file=%3")
                   .arg(mapName,
                        QString::number(pointCount),
                        QDir::toNativeSeparators(filePath)));

    slamMapExportWorker = std::thread(
        [this, filePath, points = std::move(points), format, pointCount, mapName]() {
            QString error;
            const bool ok = SlamMapExport::save(filePath, points, format, &error);
            QMetaObject::invokeMethod(
                this,
                [this, ok, error, filePath, pointCount, mapName]() {
                    slamMapExportActive.store(false);
                    updateSlamControlBarUi();
                    SlamUiBridge* bridge = ensureSlamUiBridge();
                    if (!ok) {
                        bridge->setErrorMessage(error);
                        logMessage(QStringLiteral("[FreeDOM] %1保存失败: %2")
                                       .arg(mapName, error));
                        return;
                    }
                    bridge->clearErrorMessage();
                    logMessage(QStringLiteral("[FreeDOM] %1保存完成: points=%2, file=%3")
                                   .arg(mapName,
                                        QString::number(pointCount),
                                        QDir::toNativeSeparators(filePath)));
                    if (statusBar()) {
                        statusBar()->showMessage(
                            QStringLiteral("FreeDOM %1已保存").arg(mapName),
                            3000);
                    }
                },
                Qt::QueuedConnection);
        });
}

void LivoxViewerWindow::exportSlamTrajectoryCsv()
{
    exportSlamTrajectory(SlamTrajectoryExport::Format::Csv);
}

void LivoxViewerWindow::exportSlamTrajectoryTum()
{
    exportSlamTrajectory(SlamTrajectoryExport::Format::Tum);
}

void LivoxViewerWindow::exportSlamMapPcd()
{
    exportSlamGlobalMap(false);
}

void LivoxViewerWindow::exportSlamMapLas()
{
    exportSlamGlobalMap(true);
}

void LivoxViewerWindow::exportSlamTrajectory(SlamTrajectoryExport::Format format)
{
    SlamUiBridge* bridge = ensureSlamUiBridge();
    const QVector<SlamTrajectoryPoint> trajectory = bridge->trajectorySnapshot();
    if (trajectory.isEmpty()) {
        const QString message = QStringLiteral("当前没有可导出的 SLAM 轨迹。");
        bridge->setErrorMessage(message);
        logMessage(QStringLiteral("[SLAM] %1").arg(message));
        if (statusBar()) {
            statusBar()->showMessage(message, 3000);
        }
        QMessageBox::warning(this, QStringLiteral("导出 SLAM 轨迹失败"), message);
        return;
    }

    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    QString lastDir = settings.value(QStringLiteral("slam/lastTrajectoryExportDir"),
                                     QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
    if (lastDir.isEmpty()) {
        lastDir = QDir::homePath();
    }

    const QString extension = trajectoryExportExtension(format);
    const QString defaultName = QStringLiteral("slam_trajectory_%1.%2")
                                    .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")),
                                         extension);
    QFileDialog dialog(this);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setWindowTitle(QStringLiteral("导出 SLAM 轨迹"));
    dialog.setDirectory(lastDir);
    dialog.selectFile(defaultName);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setDefaultSuffix(extension);
    dialog.setNameFilter(trajectoryExportFilter(format));
    if (dialog.exec() != QDialog::Accepted || dialog.selectedFiles().isEmpty()) {
        return;
    }

    const QString filePath = dialog.selectedFiles().first();
    QString error;
    if (!SlamTrajectoryExport::save(filePath, trajectory, format, &error)) {
        bridge->setErrorMessage(error);
        logMessage(QStringLiteral("[SLAM] 轨迹导出失败: %1").arg(error));
        if (statusBar()) {
            statusBar()->showMessage(error, 3000);
        }
        QMessageBox::warning(this, QStringLiteral("导出 SLAM 轨迹失败"), error);
        return;
    }

    settings.setValue(QStringLiteral("slam/lastTrajectoryExportDir"), QFileInfo(filePath).absolutePath());
    bridge->clearErrorMessage();
    logMessage(QStringLiteral("[SLAM] 轨迹导出完成: %1").arg(QDir::toNativeSeparators(filePath)));
    if (statusBar()) {
        statusBar()->showMessage(QStringLiteral("SLAM 轨迹已导出"), 3000);
    }
}

void LivoxViewerWindow::exportSlamGlobalMap(bool lasFormat)
{
    SlamUiBridge* bridge = ensureSlamUiBridge();
    if (slamMapExportWorker.joinable() && !slamMapExportActive.load()) {
        slamMapExportWorker.join();
    }
    if (slamMapExportWorker.joinable()) {
        const QString message = QStringLiteral("完整全局地图正在导出，请等待当前导出完成。");
        bridge->setErrorMessage(message);
        logMessage(QStringLiteral("[SLAM] %1").arg(message));
        if (statusBar()) {
            statusBar()->showMessage(message, 3000);
        }
        QMessageBox::warning(this, QStringLiteral("导出 SLAM 全局地图失败"), message);
        return;
    }

    QVector<SlamPoint> points = bridge->globalMapSnapshot();
    if (points.isEmpty()) {
        const QString message = QStringLiteral("当前没有可导出的 SLAM 完整全局地图。请在 SLAM 设置中启用完整地图保存并重新运行 SLAM。");
        bridge->setErrorMessage(message);
        logMessage(QStringLiteral("[SLAM] %1").arg(message));
        if (statusBar()) {
            statusBar()->showMessage(message, 3000);
        }
        QMessageBox::warning(this, QStringLiteral("导出 SLAM 全局地图失败"), message);
        return;
    }

    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    QString lastDir = settings.value(QStringLiteral("slam/lastMapExportDir"),
                                     QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
    if (lastDir.isEmpty()) {
        lastDir = QDir::homePath();
    }

    const QString extension = mapExportExtension(lasFormat);
    const QString defaultName = QStringLiteral("slam_global_map_%1.%2")
                                    .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")),
                                         extension);
    QFileDialog dialog(this);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setWindowTitle(QStringLiteral("导出 SLAM 完整全局地图"));
    dialog.setDirectory(lastDir);
    dialog.selectFile(defaultName);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setDefaultSuffix(extension);
    dialog.setNameFilter(mapExportFilter(lasFormat));
    if (dialog.exec() != QDialog::Accepted || dialog.selectedFiles().isEmpty()) {
        return;
    }

    const QString filePath = dialog.selectedFiles().first();
    settings.setValue(QStringLiteral("slam/lastMapExportDir"), QFileInfo(filePath).absolutePath());
    const int pointCount = points.size();
    const SlamMapExport::Format format = lasFormat ? SlamMapExport::Format::Las : SlamMapExport::Format::Pcd;
    slamMapExportActive.store(true);
    bridge->clearErrorMessage();
    logMessage(QStringLiteral("[SLAM] 开始导出完整全局地图: points=%1, file=%2")
                   .arg(QString::number(pointCount), QDir::toNativeSeparators(filePath)));
    if (statusBar()) {
        statusBar()->showMessage(QStringLiteral("正在导出 SLAM 完整全局地图"), 3000);
    }

    slamMapExportWorker = std::thread([this, filePath, points = std::move(points), format, pointCount]() {
        QString error;
        const bool ok = SlamMapExport::save(filePath, points, format, &error);
        QMetaObject::invokeMethod(this, [this, ok, error, filePath, pointCount]() {
            slamMapExportActive.store(false);
            if (!ok) {
                if (SlamUiBridge* bridge = ensureSlamUiBridge()) {
                    bridge->setErrorMessage(error);
                }
                logMessage(QStringLiteral("[SLAM] 完整全局地图导出失败: %1").arg(error));
                if (statusBar()) {
                    statusBar()->showMessage(error, 3000);
                }
                QMessageBox::warning(this, QStringLiteral("导出 SLAM 全局地图失败"), error);
                return;
            }
            if (SlamUiBridge* bridge = ensureSlamUiBridge()) {
                bridge->clearErrorMessage();
            }
            logMessage(QStringLiteral("[SLAM] 完整全局地图导出完成: points=%1, file=%2")
                           .arg(QString::number(pointCount), QDir::toNativeSeparators(filePath)));
            if (statusBar()) {
                statusBar()->showMessage(QStringLiteral("SLAM 完整全局地图已导出"), 3000);
            }
        }, Qt::QueuedConnection);
    });
}

void LivoxViewerWindow::submitSlamOutputForUi(const SlamOutput& output)
{
    if (isSlamErrorStatus(output.status) && !output.message.isEmpty()) {
        logMessage(QStringLiteral("[SLAM] %1").arg(output.message));
    }
    appendSlamWorldFramePoints(output);
    ensureSlamUiBridge()->receiveSlamOutput(output);
    showOfflineSlamFailureDialog(output.status, output.message);
}

void LivoxViewerWindow::submitSlamOutputsForUi(const QVector<SlamOutput>& outputs)
{
    SlamUiBridge* bridge = ensureSlamUiBridge();
    SlamStatusCode failureStatus = SlamStatusCode::Idle;
    QString failureMessage;
    for (const SlamOutput& output : outputs) {
        if (isSlamErrorStatus(output.status) && !output.message.isEmpty()) {
            logMessage(QStringLiteral("[SLAM] %1").arg(output.message));
        }
        appendSlamWorldFramePoints(output, false);
        bridge->receiveSlamOutput(output);
        if (failureMessage.isEmpty() && isSlamDialogFailureStatus(output.status) && !output.message.isEmpty()) {
            failureStatus = output.status;
            failureMessage = output.message;
        }
    }
    if (slamWorldFrameVisible && !outputs.isEmpty()) {
        refreshSlamWorldPointCloud();
    }
    showOfflineSlamFailureDialog(failureStatus, failureMessage);
}

void LivoxViewerWindow::showOfflineSlamFailureDialog(SlamStatusCode status, const QString& message)
{
    if (isOfflineSlamMode() && isSlamDialogFailureStatus(status) &&
        !message.isEmpty() && !slamFailureDialogShown) {
        slamFailureDialogShown = true;
        QMessageBox::warning(this, QStringLiteral("离线 SLAM 失败"), message);
    }
}

void LivoxViewerWindow::postSlamStatus(SlamStatusCode status, const QString& message)
{
    submitSlamOutputForUi(statusOutput(status, message));
}

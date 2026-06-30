#include "LivoxViewerWindow.h"

#include "Slam/Backends/FastLio/FastLioSlamBackend.h"
#include "Slam/Export/SlamMapExport.h"
#include "Slam/Export/SlamTrajectoryExport.h"
#include "Slam/Io/PcapSlamSource.h"
#include "Slam/Io/RosbagSlamSource.h"
#include "slam/SlamControlDialog.h"
#include "slam/SlamUiBridge.h"

#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QFileInfo>
#include <QMetaObject>
#include <QSettings>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QStandardPaths>
#include <QStringList>

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

namespace {

constexpr int64_t kSlamWorldHistoryRetentionNs = int64_t{600000} * int64_t{1000000};

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
    return QStringLiteral("PCAP");
}

} // namespace

SlamUiBridge* LivoxViewerWindow::ensureSlamUiBridge()
{
    if (slamUiBridge) {
        return slamUiBridge;
    }

    qRegisterMetaType<SlamRenderSnapshot>("SlamRenderSnapshot");
    slamUiBridge = new SlamUiBridge(this);
    slamUiBridge->setWorldFrameColor(slamWorldCurrentFrameColor);
    slamUiBridge->setBodyFrameColor(slamBodyFrameColor);
    slamUiBridge->setTrajectoryColor(slamTrajectoryColor);
    slamUiBridge->setWorldFramePointSize(slamWorldCurrentFramePointSizePx);
    slamUiBridge->setBodyFramePointSize(slamBodyFramePointSizePx);
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
    ensureSlamUiBridge()->setModeAndBackend(QStringLiteral("离线 SLAM"), QStringLiteral("FAST_LIO"));
    updateSlamControlBarUi();
}

void LivoxViewerWindow::setSlamInputModeOnline()
{
    slamInputMode = SlamInputMode::Online;
    ensureSlamUiBridge()->setModeAndBackend(QStringLiteral("在线 SLAM"), QStringLiteral("FAST_LIO"));
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
        QStringLiteral("SLAM 数据源 (*.pcap *.pcapng *.bag *.db3 *.yaml *.yml)"),
        QStringLiteral("PCAP 文件 (*.pcap *.pcapng)"),
        QStringLiteral("ROS1 Bag 文件 (*.bag)"),
        QStringLiteral("ROS2 db3 文件 (*.db3 *.yaml *.yml)"),
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
    slamOfflineSourceKind = (suffix == QStringLiteral("bag") ||
                             suffix == QStringLiteral("db3") ||
                             suffix == QStringLiteral("yaml") ||
                             suffix == QStringLiteral("yml"))
        ? SlamOfflineSourceKind::Rosbag
        : SlamOfflineSourceKind::Pcap;
    slamOfflineSourcePath = filePath;
    slamOfflineSourceDisplayName = offlineSourceKindForPath(filePath);
    settings.setValue(QStringLiteral("slam/lastOfflineSourceDir"), QFileInfo(filePath).absolutePath());
    ensureSlamVisualizationTab(filePath);
    clearSlamDisplay();
    ensureSlamUiBridge()->setModeAndBackend(QStringLiteral("离线 SLAM"), QStringLiteral("FAST_LIO"));
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
    slamPointCloudView->setMeasurementModeEnabled(measurementModeActive);
    slamPointCloudView->setSelectionModeEnabled(selectionRealtimeEnabled);
    connect(slamPointCloudView, &PointCloudView::lvx2FileDropped, this, &LivoxViewerWindow::onLvx2PlaybackFileDropped);
    connect(slamPointCloudView, &PointCloudView::selectionPointsReady, this, &LivoxViewerWindow::onSelectionPointsReady);
    connect(slamPointCloudView, &PointCloudView::crossSectionChanged, this, [this](int clippedPointCount, int sourcePointCount) {
        if (crossSectionModeActive && statusLabelBar) {
            statusLabelBar->setText(QString("Cross Section: %1 / %2 点").arg(clippedPointCount).arg(sourcePointCount));
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
                              QStringLiteral("FAST_LIO"));

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

    if (slamInputMode == SlamInputMode::Online) {
        const SlamRuntimeConfig config = slamRuntimeConfig;
        liveSlamSource.reset();
        liveSlamSource.setFrameDurationMs(config.inputFrameDurationMs);
        liveSlamSource.setQueueCapacity(config.maxInputQueueFrames);
        slamWorkerCancel.store(false);
        slamWorkerPaused.store(false);
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
                    updateSlamControlBarUi();
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
                postOutput(output);
                postOnlineProgress(output.inputFps, processedFrames);

                if (!accepted &&
                    output.status != SlamStatusCode::InitializingImu &&
                    output.status != SlamStatusCode::TimeSyncError) {
                    break;
                }
            }

            backend.stop();
            const bool cancelled = slamWorkerCancel.load();
            SlamOutput finalOutput = hasLastOutput ? lastOutput : statusOutput(SlamStatusCode::Stopped, QString());
            finalOutput.status = SlamStatusCode::Stopped;
            finalOutput.message = cancelled ? QStringLiteral("SLAM 已停止。") : QStringLiteral("在线 SLAM 已结束。");
            finalOutput.newTrajectoryPoints.clear();
            finalOutput.newGlobalMapPoints.clear();
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
            QMetaObject::invokeMethod(this, [this]() {
                updateSlamControlBarUi();
            }, Qt::QueuedConnection);
        });
        return;
    }

    if (slamOfflineSourcePath.isEmpty() || slamOfflineSourceKind == SlamOfflineSourceKind::None) {
        postSlamStatus(SlamStatusCode::Failed, QStringLiteral("请先加载离线 SLAM 数据源（PCAP 或 ROSbag）。"));
        return;
    }

    const QString sourcePath = slamOfflineSourcePath;
    const SlamOfflineSourceKind sourceKind = slamOfflineSourceKind;
    const QString sourceDisplayName = slamOfflineSourceDisplayName.isEmpty()
        ? offlineSourceKindForPath(sourcePath)
        : slamOfflineSourceDisplayName;
    const SlamRuntimeConfig config = slamRuntimeConfig;
    const SlamReplayMode replayMode = slamReplayMode;
    slamWorkerCancel.store(false);
    slamWorkerPaused.store(false);
    slamWorkerActive.store(true);
    slamProgressValue = 0;
    slamProgressMaximum = 0;
    slamProgressIndeterminate = true;
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
                updateSlamControlBarUi();
            }, Qt::QueuedConnection);
        };

        QString error;
        QVector<SlamInputFrame> frames;
        QString summaryText;
        if (sourceKind == SlamOfflineSourceKind::Pcap) {
            PcapSlamSource source(config.inputFrameDurationMs);
            if (!source.load(sourcePath, &error)) {
                postOutput(statusOutput(SlamStatusCode::Failed, error));
                slamWorkerActive.store(false);
                postControlUpdate();
                return;
            }
            frames = source.frames();
            summaryText = source.summaryText();
        } else if (sourceKind == SlamOfflineSourceKind::Rosbag) {
            RosbagSlamSourceConfig rosbagConfig;
            rosbagConfig.frameDurationMs = config.inputFrameDurationMs;
            rosbagConfig.allowLivoxDriver2PointCloud2 = config.allowRosbagDriver2PointCloud2;
            rosbagConfig.allowLivoxDriverPointCloud2SynthesizedTime =
                config.allowRosbagDriverPointCloud2SynthesizedTime;
            RosbagSlamSource source(rosbagConfig);
            if (!source.load(sourcePath, &error)) {
                postOutput(statusOutput(SlamStatusCode::Failed, error));
                slamWorkerActive.store(false);
                postControlUpdate();
                return;
            }
            frames = source.frames();
            summaryText = source.summaryText();
        } else {
            postOutput(statusOutput(SlamStatusCode::Failed, QStringLiteral("未知离线 SLAM 数据源类型。")));
            slamWorkerActive.store(false);
            postControlUpdate();
            return;
        }
        postLog(QStringLiteral("[SLAM] %1").arg(summaryText));
        const QString sourceText = QDir::toNativeSeparators(sourcePath);
        const int64_t firstFrameStartNs = frames.isEmpty()
            ? int64_t{0}
            : frames.first().frameStartNs;

        int64_t lastFrameEndNs = int64_t{0};
        if (!frames.isEmpty()) {
            const int64_t fallbackFrameDurationNs =
                static_cast<int64_t>(std::max<int>(1, config.inputFrameDurationMs)) * int64_t{1000000};
            const int64_t fallbackFrameEndNs = frames.last().frameStartNs + fallbackFrameDurationNs;
            lastFrameEndNs = std::max<int64_t>(frames.last().frameEndNs, fallbackFrameEndNs);
        }

        const uint64_t fallbackFrameCount =
            static_cast<uint64_t>(std::max<qsizetype>(qsizetype{1}, frames.size()));
        const uint64_t fallbackFrameDurationMs =
            static_cast<uint64_t>(std::max<int>(1, config.inputFrameDurationMs));
        const uint64_t totalDurationNs =
            firstFrameStartNs >= 0 && lastFrameEndNs > firstFrameStartNs
                ? static_cast<uint64_t>(lastFrameEndNs - firstFrameStartNs)
                : fallbackFrameCount * fallbackFrameDurationMs * uint64_t{1000000};
        auto makeTimeText = [totalDurationNs](uint64_t elapsedNs) {
            const uint64_t clampedElapsedNs = std::min(elapsedNs, totalDurationNs);
            return QStringLiteral("时间 %1 / %2")
                .arg(formatSlamTimeNs(clampedElapsedNs, totalDurationNs))
                .arg(formatSlamTimeNs(totalDurationNs, totalDurationNs));
        };
        auto makeFrameText = [](int value, int maximum) {
            return QStringLiteral("帧 %1 / %2").arg(value).arg(maximum);
        };
        postProgress(0, frames.size(), false, sourceText, makeTimeText(0), makeFrameText(0, frames.size()));

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
        qint64 pausedReplayMs = 0;
        QElapsedTimer replayClock;
        SlamOutput lastOutput;
        bool hasLastOutput = false;
        int progressFrames = 0;

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

        auto waitForReplayTarget = [this, &waitUntilRunning, &pausedReplayMs, &replayClock](qint64 targetMs) {
            while (!slamWorkerCancel.load()) {
                if (!waitUntilRunning()) {
                    return false;
                }
                const qint64 remainingMs = targetMs - (replayClock.elapsed() - pausedReplayMs);
                if (remainingMs <= 0) {
                    return true;
                }
                const qint64 sleepMs = std::min<qint64>(remainingMs, qint64{10});
                std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
            }
            return false;
        };

        for (const SlamInputFrame& frame : frames) {
            if (slamWorkerCancel.load()) {
                break;
            }
            ++progressFrames;
            if (!hasFirstFrameStart) {
                hasFirstFrameStart = true;
                replayClock.start();
            }
            if (replayMode == SlamReplayMode::Timed &&
                !waitForReplayTarget(replayTargetMs(firstFrameStartNs, frame.frameStartNs))) {
                break;
            }
            if (!waitUntilRunning()) {
                break;
            }
            const uint64_t elapsedFrameNs =
                frame.frameStartNs >= firstFrameStartNs ? uint64_t(frame.frameStartNs - firstFrameStartNs) : 0ULL;
            if (!frame.hasCompleteImuCoverage) {
                ++droppedFrames;
                postProgress(progressFrames,
                             frames.size(),
                             false,
                             sourceText,
                             makeTimeText(elapsedFrameNs),
                             makeFrameText(progressFrames, frames.size()));
                continue;
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
            postOutput(output);
            postProgress(progressFrames,
                         frames.size(),
                         false,
                         sourceText,
                         makeTimeText(elapsedFrameNs),
                         makeFrameText(progressFrames, frames.size()));

            if (!accepted &&
                output.status != SlamStatusCode::InitializingImu &&
                output.status != SlamStatusCode::TimeSyncError) {
                break;
            }
        }

        backend.stop();
        const bool cancelled = slamWorkerCancel.load();
        SlamOutput finalOutput = hasLastOutput ? lastOutput : statusOutput(SlamStatusCode::Stopped, QString());
        finalOutput.status = SlamStatusCode::Stopped;
        finalOutput.message = cancelled ? QStringLiteral("SLAM 已停止。") : QStringLiteral("%1 SLAM 已完成。").arg(sourceDisplayName);
        finalOutput.droppedFrameCount = droppedFrames;
        if (processedFrames > 0) {
            const double elapsedSec = qMax(0.001, double(elapsed.elapsed()) / 1000.0);
            finalOutput.inputFps = double(processedFrames) / elapsedSec;
        }
        finalOutput.newTrajectoryPoints.clear();
        finalOutput.newGlobalMapPoints.clear();
        const int finalProgress = cancelled ? progressFrames : frames.size();
        uint64_t finalElapsedNs = totalDurationNs;
        if (cancelled) {
            finalElapsedNs = 0;
            if (progressFrames > 0 && progressFrames <= frames.size()) {
                const int64_t elapsedNs = frames.at(progressFrames - 1).frameStartNs - firstFrameStartNs;
                if (elapsedNs > 0) {
                    finalElapsedNs = std::min<uint64_t>(uint64_t(elapsedNs), totalDurationNs);
                }
            }
        }
        postProgress(finalProgress,
                     frames.size(),
                     false,
                     sourceText,
                     makeTimeText(finalElapsedNs),
                     makeFrameText(finalProgress, frames.size()));
        postOutput(finalOutput);
        slamWorkerActive.store(false);
        QMetaObject::invokeMethod(this, [this]() {
            updateSlamControlBarUi();
        }, Qt::QueuedConnection);
    });
}

void LivoxViewerWindow::pauseSlamProcessing()
{
    ensureSlamUiBridge();
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
    slamWorkerCancel.store(true);
    slamWorkerPaused.store(false);
    if (slamWorker.joinable()) {
        slamWorker.join();
    }
    slamWorkerActive.store(false);
    postSlamStatus(SlamStatusCode::Stopped, QStringLiteral("SLAM 已停止。"));
    updateSlamControlBarUi();
}

void LivoxViewerWindow::resetSlamProcessing()
{
    stopSlamProcessing();
    clearSlamDisplay();
    postSlamStatus(SlamStatusCode::Idle, QStringLiteral("SLAM 已重置。"));
    updateSlamControlBarUi();
}

void LivoxViewerWindow::clearSlamDisplay()
{
    slamProgressValue = 0;
    slamProgressMaximum = 0;
    slamProgressIndeterminate = false;
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

void LivoxViewerWindow::appendSlamWorldFramePoints(const SlamOutput& output)
{
    if (output.publishedWorldFramePoints.isEmpty() || !slamPointCloudView) {
        return;
    }

    SlamWorldPointSegment segment;
    segment.timestampNs = output.currentPose.timestampNs;
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
    if (slamWorldFrameVisible) {
        refreshSlamWorldPointCloud();
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
    slamUiBridge->setRenderLayerVisibility(slamTrajectoryVisible,
                                           slamPoseAxisVisible,
                                           slamRuntimeConfig.publishWorldFrameCloud && slamWorldCurrentFrameVisible,
                                           slamRuntimeConfig.publishWorldFrameCloud &&
                                               slamRuntimeConfig.publishBodyFrameCloud &&
                                               slamBodyFrameVisible);
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
}

void LivoxViewerWindow::postSlamStatus(SlamStatusCode status, const QString& message)
{
    submitSlamOutputForUi(statusOutput(status, message));
}

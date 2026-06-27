#include "LivoxViewerWindow.h"

#include "Export/PointCloudExport.h"
#include "Slam/Backends/FastLio/FastLioSlamBackend.h"
#include "Slam/Export/SlamTrajectoryExport.h"
#include "Slam/Io/PcapSlamSource.h"
#include "slam/SlamControlDialog.h"
#include "slam/SlamUiBridge.h"

#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QFileInfo>
#include <QMetaObject>
#include <QSettings>
#include <QStatusBar>
#include <QStandardPaths>

#include <algorithm>
#include <chrono>
#include <thread>

namespace {

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
           status == SlamStatusCode::TimeSyncError;
}

const PlaybackControllerState* activePlaybackState(const LivoxViewerWindow* window,
                                                   const PlaybackControllerState* mirror,
                                                   const PlaybackControllerState* bound)
{
    Q_UNUSED(window);
    if (bound && bound->active) {
        return bound;
    }
    return mirror && mirror->active ? mirror : nullptr;
}

qint64 replayTargetMs(int64_t firstFrameStartNs, int64_t frameStartNs)
{
    const int64_t elapsedNs = frameStartNs - firstFrameStartNs;
    if (elapsedNs <= 0) {
        return 0;
    }
    return qint64(elapsedNs / 1000000);
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

QString mapExportExtension(bool lasFormat)
{
    return lasFormat ? QStringLiteral("las") : QStringLiteral("pcd");
}

QString mapExportFilter(bool lasFormat)
{
    return lasFormat ? QStringLiteral("LAS 点云 (*.las)") : QStringLiteral("PCD 点云 (*.pcd)");
}

QVector<PointCloudPoint> toPointCloudPoints(const QVector<SlamRenderVertex>& vertices)
{
    QVector<PointCloudPoint> points;
    points.reserve(vertices.size());
    for (const SlamRenderVertex& vertex : vertices) {
        PointCloudPoint point{};
        point.x = vertex.x;
        point.y = vertex.y;
        point.z = vertex.z;
        point.r = vertex.r;
        point.g = vertex.g;
        point.b = vertex.b;
        point.reflectivity = uint8_t(std::clamp(int((vertex.r + vertex.g + vertex.b) * 85.0f), 0, 255));
        point.tag = 0;
        point.line = 0;
        point.spherical = false;
        point.theta = 0.0f;
        point.phi = 0.0f;
        point.depth = 0.0f;
        points.append(point);
    }
    return points;
}

bool saveMapPreview(const QString& filePath, const QVector<PointCloudPoint>& points, bool lasFormat)
{
    return lasFormat ? PointCloudExport::saveAsLAS(filePath, points)
                     : PointCloudExport::saveAsPCD(filePath, points);
}

} // namespace

SlamUiBridge* LivoxViewerWindow::ensureSlamUiBridge()
{
    if (slamUiBridge) {
        return slamUiBridge;
    }

    qRegisterMetaType<SlamRenderSnapshot>("SlamRenderSnapshot");
    slamUiBridge = new SlamUiBridge(this);
    slamUiBridge->setMapPreviewConfig(slamMapPreviewConfig);
    connect(slamUiBridge, &SlamUiBridge::statusTextReady, this, [this](const QString& text) {
        if (statusBar()) {
            statusBar()->showMessage(text, 2000);
        }
    });
    connect(slamUiBridge, &SlamUiBridge::renderSnapshotReady, this, [this](const SlamRenderSnapshot& snapshot) {
        if (!slamRenderOverlayEnabled) {
            return;
        }
        if (PointCloudView* view = currentPointCloudView()) {
            view->setSlamRenderSnapshot(snapshot);
        }
    });
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

void LivoxViewerWindow::startSlamProcessing()
{
    SlamUiBridge* bridge = ensureSlamUiBridge();
    bridge->setModeAndBackend(QStringLiteral("PCAP 原始时间"), QStringLiteral("FAST_LIO"));

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
        return;
    }

    const PlaybackControllerState* boundState = playbackStateForTab(boundPlaybackTabId);
    const PlaybackControllerState* state = activePlaybackState(this, &playbackState, boundState);
    if (!state || !state->source || state->source->kind() != Playback::SourceKind::Pcap || state->path.isEmpty()) {
        postSlamStatus(SlamStatusCode::Failed, QStringLiteral("请先打开并激活一个 PCAP 播放源。"));
        return;
    }

    const QString pcapPath = state->path;
    const SlamRuntimeConfig config = slamRuntimeConfig;
    slamWorkerCancel.store(false);
    slamWorkerPaused.store(false);
    slamWorkerActive.store(true);
    slamRenderOverlayEnabled = true;
    bridge->clearDisplay();
    postSlamStatus(SlamStatusCode::Starting, QStringLiteral("正在启动 PCAP SLAM 原始时间回放。"));

    slamWorker = std::thread([this, pcapPath, config]() {
        auto postOutput = [this](SlamOutput output) {
            if (slamMapPreviewModeValue.load() == int(SlamMapPreviewMode::Off)) {
                output.newMapChunks.clear();
            }
            QMetaObject::invokeMethod(this, [this, output = std::move(output)]() mutable {
                submitSlamOutputForUi(output);
            }, Qt::QueuedConnection);
        };

        PcapSlamSource source;
        QString error;
        if (!source.load(pcapPath, &error)) {
            postOutput(statusOutput(SlamStatusCode::Failed, error));
            slamWorkerActive.store(false);
            return;
        }

        FastLioSlamBackend backend;
        if (!backend.start(config, &error)) {
            postOutput(statusOutput(SlamStatusCode::Failed, error));
            slamWorkerActive.store(false);
            return;
        }

        QElapsedTimer elapsed;
        elapsed.start();
        int processedFrames = 0;
        int droppedFrames = 0;
        int64_t firstFrameStartNs = 0;
        bool hasFirstFrameStart = false;
        qint64 pausedReplayMs = 0;
        QElapsedTimer replayClock;
        SlamOutput lastOutput;
        bool hasLastOutput = false;

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
                const qint64 sleepMs = std::min<qint64>(remainingMs, 10);
                std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
            }
            return false;
        };

        for (const SlamInputFrame& frame : source.frames()) {
            if (slamWorkerCancel.load()) {
                break;
            }
            if (!hasFirstFrameStart) {
                firstFrameStartNs = frame.frameStartNs;
                hasFirstFrameStart = true;
                replayClock.start();
            }
            if (!waitForReplayTarget(replayTargetMs(firstFrameStartNs, frame.frameStartNs))) {
                break;
            }
            if (!waitUntilRunning()) {
                break;
            }
            if (!frame.hasCompleteImuCoverage) {
                ++droppedFrames;
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
        finalOutput.message = cancelled ? QStringLiteral("SLAM 已停止。") : QStringLiteral("PCAP SLAM 已完成。");
        finalOutput.droppedFrameCount = droppedFrames;
        if (processedFrames > 0) {
            const double elapsedSec = qMax(0.001, double(elapsed.elapsed()) / 1000.0);
            finalOutput.inputFps = double(processedFrames) / elapsedSec;
        }
        finalOutput.newTrajectoryPoints.clear();
        finalOutput.newMapChunks.clear();
        postOutput(finalOutput);
        slamWorkerActive.store(false);
    });
}

void LivoxViewerWindow::pauseSlamProcessing()
{
    ensureSlamUiBridge();
    if (!slamWorker.joinable() || !slamWorkerActive.load()) {
        postSlamStatus(SlamStatusCode::Paused, QStringLiteral("当前没有正在运行的 SLAM。"));
        return;
    }
    slamWorkerPaused.store(true);
    postSlamStatus(SlamStatusCode::Paused, QStringLiteral("SLAM 已暂停。"));
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
}

void LivoxViewerWindow::resetSlamProcessing()
{
    stopSlamProcessing();
    clearSlamDisplay();
    postSlamStatus(SlamStatusCode::Idle, QStringLiteral("SLAM 已重置。"));
}

void LivoxViewerWindow::clearSlamDisplay()
{
    if (SlamUiBridge* bridge = ensureSlamUiBridge()) {
        bridge->clearDisplay();
    }
    forEachPointCloudView([](PointCloudView* view) {
        if (view) {
            view->clearSlamRenderOverlay();
        }
    });
}

void LivoxViewerWindow::setSlamMapPreviewMode(SlamMapPreviewMode mode)
{
    SlamMapPreviewConfig config = slamMapPreviewConfig;
    config.mode = mode;
    setSlamMapPreviewConfig(config);
}

void LivoxViewerWindow::setSlamMapPreviewConfig(const SlamMapPreviewConfig& config)
{
    slamMapPreviewConfig = config;
    slamMapPreviewModeValue.store(int(config.mode));
    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    saveSlamMapPreviewConfig(settings, slamMapPreviewConfig, QStringLiteral("slam/mapPreview"));
    if (SlamUiBridge* bridge = slamUiBridge) {
        bridge->setMapPreviewConfig(slamMapPreviewConfig);
    }
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
    exportSlamMapPreview(false);
}

void LivoxViewerWindow::exportSlamMapLas()
{
    exportSlamMapPreview(true);
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

void LivoxViewerWindow::exportSlamMapPreview(bool lasFormat)
{
    SlamUiBridge* bridge = ensureSlamUiBridge();
    const QVector<SlamRenderVertex> vertices = bridge->mapPreviewSnapshot();
    if (vertices.isEmpty()) {
        const QString message =
            QStringLiteral("当前没有可导出的 SLAM 预览地图。请先选择全局稀疏或全局稠密预览并运行 SLAM。");
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
    const QString defaultName = QStringLiteral("slam_preview_map_%1.%2")
                                    .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")),
                                         extension);
    QFileDialog dialog(this);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setWindowTitle(QStringLiteral("导出 SLAM 当前预览地图"));
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
    const QVector<PointCloudPoint> points = toPointCloudPoints(vertices);
    if (!saveMapPreview(filePath, points, lasFormat)) {
        const QString message = QStringLiteral("SLAM 当前预览地图导出失败。");
        bridge->setErrorMessage(message);
        logMessage(QStringLiteral("[SLAM] %1").arg(message));
        if (statusBar()) {
            statusBar()->showMessage(message, 3000);
        }
        return;
    }

    settings.setValue(QStringLiteral("slam/lastMapExportDir"), QFileInfo(filePath).absolutePath());
    bridge->clearErrorMessage();
    logMessage(QStringLiteral("[SLAM] 当前预览地图导出完成: mode=%1, points=%2, file=%3")
                   .arg(slamMapPreviewModeLogName(bridge->mapPreviewMode()),
                        QString::number(vertices.size()),
                        QDir::toNativeSeparators(filePath)));
    if (statusBar()) {
        statusBar()->showMessage(QStringLiteral("SLAM 当前预览地图已导出"), 3000);
    }
}

void LivoxViewerWindow::submitSlamOutputForUi(const SlamOutput& output)
{
    if (isSlamErrorStatus(output.status) && !output.message.isEmpty()) {
        logMessage(QStringLiteral("[SLAM] %1").arg(output.message));
    }
    ensureSlamUiBridge()->receiveSlamOutput(output);
}

void LivoxViewerWindow::postSlamStatus(SlamStatusCode status, const QString& message)
{
    submitSlamOutputForUi(statusOutput(status, message));
}

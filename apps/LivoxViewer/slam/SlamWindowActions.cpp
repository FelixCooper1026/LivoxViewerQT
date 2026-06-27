#include "LivoxViewerWindow.h"

#include "Slam/Backends/FastLio/FastLioSlamBackend.h"
#include "Slam/Export/SlamMapExport.h"
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

} // namespace

SlamUiBridge* LivoxViewerWindow::ensureSlamUiBridge()
{
    if (slamUiBridge) {
        return slamUiBridge;
    }

    qRegisterMetaType<SlamRenderSnapshot>("SlamRenderSnapshot");
    slamUiBridge = new SlamUiBridge(this);
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
    postSlamStatus(SlamStatusCode::Starting,
                   QStringLiteral("正在启动 PCAP SLAM 原始时间回放：%1 Hz / %2 ms。")
                       .arg(config.preprocessScanRateHz, 0, 'f', 1)
                       .arg(config.inputFrameDurationMs));

    slamWorker = std::thread([this, pcapPath, config]() {
        auto postOutput = [this](SlamOutput output) {
            QMetaObject::invokeMethod(this, [this, output = std::move(output)]() mutable {
                submitSlamOutputForUi(output);
            }, Qt::QueuedConnection);
        };

        PcapSlamSource source(config.inputFrameDurationMs);
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
        finalOutput.newGlobalMapPoints.clear();
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
    ensureSlamUiBridge()->receiveSlamOutput(output);
}

void LivoxViewerWindow::postSlamStatus(SlamStatusCode status, const QString& message)
{
    submitSlamOutputForUi(statusOutput(status, message));
}

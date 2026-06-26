#include "LivoxViewerWindow.h"

#include "Slam/Backends/FastLio/FastLioSlamBackend.h"
#include "Slam/Io/PcapSlamSource.h"
#include "slam/SlamControlDialog.h"
#include "slam/SlamUiBridge.h"

#include <QElapsedTimer>
#include <QMetaObject>
#include <QStatusBar>

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
    slamWorkerCancel.store(false);
    slamWorkerPaused.store(false);
    slamWorkerActive.store(true);
    slamRenderOverlayEnabled = true;
    bridge->clearDisplay();
    postSlamStatus(SlamStatusCode::Starting, QStringLiteral("正在启动 PCAP SLAM 原始时间回放。"));

    slamWorker = std::thread([this, pcapPath]() {
        auto postOutput = [this](SlamOutput output) {
            if (!slamMapPreviewEnabled.load()) {
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
        SlamRuntimeConfig config;
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

void LivoxViewerWindow::setSlamMapPreviewEnabled(bool enabled)
{
    slamMapPreviewEnabled.store(enabled);
    if (SlamUiBridge* bridge = ensureSlamUiBridge()) {
        bridge->setMapPreviewEnabled(enabled);
    }
}

void LivoxViewerWindow::submitSlamOutputForUi(const SlamOutput& output)
{
    ensureSlamUiBridge()->receiveSlamOutput(output);
}

void LivoxViewerWindow::postSlamStatus(SlamStatusCode status, const QString& message)
{
    submitSlamOutputForUi(statusOutput(status, message));
}

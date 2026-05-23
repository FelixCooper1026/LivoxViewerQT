#include "LivoxViewerWindow.h"

#include "Lvx2/Lvx2Reader.h"
#include "Pcap/PcapPlaybackController.h"
#include "Pcap/PushMsgParser.h"

#include <QDir>
#include <QFileInfo>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStandardPaths>

#include <algorithm>
#include <memory>
#include <thread>

namespace {

static int rawFramesPerPlaybackFrame(uint64_t frameIntervalMs)
{
    return std::max(1, int((frameIntervalMs + 49ULL) / 50ULL));
}

static int visiblePlaybackFrameCount(const Playback::Source* source,
                                     LivoxViewerWindow::Lvx2PlaybackMode mode,
                                     uint64_t frameIntervalMs)
{
    if (!source) {
        return 0;
    }

    const int rawFrameCount = source->frameCount();
    if (mode == LivoxViewerWindow::Lvx2PlaybackMode::SlidingWindow) {
        return rawFrameCount;
    }

    const int rawFramesPerStep = rawFramesPerPlaybackFrame(frameIntervalMs);
    return (rawFrameCount + rawFramesPerStep - 1) / rawFramesPerStep;
}

} // namespace

void LivoxViewerWindow::finishPlaybackSourceLoad(const std::shared_ptr<Playback::Source>& source)
{
    playbackSource = source;
    playbackDevices = playbackSource ? playbackSource->devices() : QVector<PlaybackDeviceInfo>();
    playbackDeviceVisible.clear();
    for (const PlaybackDeviceInfo& device : playbackDevices) {
        playbackDeviceVisible.insert(device.lidarId, true);
    }
    rebuildLvx2DeviceTab();

    playbackSlidingWindowStart = -1;
    playbackSlidingWindowEnd = -1;
    playbackSlidingWindowPoints.clear();
    playbackSlidingWindowSegmentPointCounts.clear();
    playbackSlidingWindowTimestamp = 0;
    playbackPath = playbackSource ? playbackSource->path() : QString();
    playbackActive = (playbackSource && playbackSource->frameCount() > 0);
    playbackFrameCount = visiblePlaybackFrameCount(playbackSource.get(), playbackMode, frameIntervalMs);
    if (playbackActive && playbackFrameCount <= 0) {
        playbackFrameCount = 1;
    }
    playbackFrame = -1;

    {
        QMutexLocker locker(&frameMutex);
        pendingFrames.clear();
        lastSeenTimestamp.clear();
    }

    setLvx2PlaybackPlaying(false);
    updateLvx2PlaybackUi();
    showLvx2PlaybackFrame(0);

    const QString fileName = playbackPath.isEmpty() ? QString() : QFileInfo(playbackPath).fileName();
    if (statusLabelBar) {
        if (playbackSource && playbackSource->kind() == Playback::SourceKind::Pcap) {
            statusLabelBar->setText(QString("Pcap播放: %1").arg(fileName));
        } else {
            statusLabelBar->setText(QString("LVX2播放: %1").arg(fileName));
        }
    }
    if (playbackSource && playbackSource->kind() == Playback::SourceKind::Pcap) {
        logMessage(QString("已加载Pcap文件: %1 (共 %2 帧)")
                       .arg(QDir::toNativeSeparators(playbackPath))
                       .arg(playbackSource->frameCount()));
    } else {
        logMessage(QString("已加载LVX2文件: %1").arg(QDir::toNativeSeparators(playbackPath)));
    }
}

bool LivoxViewerWindow::loadLvx2PlaybackFile(const QString& filePath)
{
    closeLvx2Playback(false);

    playbackLoading = true;
    playbackPath = filePath;
    playbackLoadToken++;
    const quint64 currentToken = playbackLoadToken;

    setLvx2PlaybackPlaying(false);
    updateLvx2PlaybackUi();
    if (statusLabelBar) {
        statusLabelBar->setText(QString("正在加载LVX2: %1").arg(QFileInfo(filePath).fileName()));
    }

    std::thread([this, filePath, currentToken]() {
        auto source = std::make_shared<Lvx2::Lvx2Reader>();
        const bool ok = source->load(filePath);
        const QString errorMessage = source->errorMessage();

        QMetaObject::invokeMethod(this, [this, currentToken, source, ok, errorMessage]() {
            if (currentToken != playbackLoadToken) {
                return;
            }

            playbackLoading = false;
            if (!ok) {
                playbackPath.clear();
                updateLvx2PlaybackUi();
                if (statusLabelBar) {
                    statusLabelBar->setText(sdk_started ? "已连接 - 采样中" : "就绪");
                }
                QMessageBox::warning(this, "播放LVX2点云", errorMessage);
                return;
            }

            finishPlaybackSourceLoad(source);
        }, Qt::QueuedConnection);
    }).detach();

    return true;
}

void LivoxViewerWindow::closeLvx2Playback(bool clearView)
{
    setLvx2PlaybackPlaying(false);

    playbackLoadToken++;
    playbackSource.reset();
    playbackActive = false;
    playbackLoading = false;
    playbackFrame = -1;
    playbackFrameCount = 0;
    playbackDevices.clear();
    playbackDeviceVisible.clear();
    rebuildLvx2DeviceTab();
    playbackSlidingWindowStart = -1;
    playbackSlidingWindowEnd = -1;
    playbackSlidingWindowPoints.clear();
    playbackSlidingWindowSegmentPointCounts.clear();
    playbackSlidingWindowTimestamp = 0;
    playbackPath.clear();

    updateLvx2PlaybackUi();
    if (clearView && pointCloudView) {
        pointCloudView->clearPointCloud();
    }

    if (statusLabelBar) {
        statusLabelBar->setText(sdk_started ? "已连接 - 采样中" : "就绪");
    }
}

void LivoxViewerWindow::showLvx2PlaybackFrame(int playbackFrameIndex)
{
    if (!playbackActive || !playbackSource || playbackFrameCount <= 0) {
        return;
    }

    playbackFrameIndex = std::clamp(playbackFrameIndex, 0, playbackFrameCount - 1);
    const int sourceFrameCount = playbackSource->frameCount();
    const int rawFrameCount = rawFramesPerPlaybackFrame(frameIntervalMs);
    int rawStartIndex = 0;
    int rawEndIndex = 0;
    if (playbackMode == Lvx2PlaybackMode::SlidingWindow) {
        rawEndIndex = std::min(playbackFrameIndex + 1, sourceFrameCount);
        rawStartIndex = std::max(0, rawEndIndex - rawFrameCount);
    } else {
        rawStartIndex = playbackFrameIndex * rawFrameCount;
        rawEndIndex = std::min(rawStartIndex + rawFrameCount, sourceFrameCount);
    }
    if (rawStartIndex >= rawEndIndex) {
        return;
    }

    auto readRawFrame = [this](int rawIndex) -> PointCloudFrame {
        PointCloudFrame frame;
        if (!playbackSource || !playbackSource->readFrame(rawIndex, playbackDeviceVisible, frame)) {
            return {};
        }
        return frame;
    };

    if (playbackMode == Lvx2PlaybackMode::SlidingWindow) {
        const bool canIncrementalAdvance =
            (playbackSlidingWindowStart >= 0 && playbackSlidingWindowEnd >= 0 &&
             rawStartIndex >= playbackSlidingWindowStart && rawEndIndex >= playbackSlidingWindowEnd &&
             rawStartIndex - playbackSlidingWindowStart <= 1 && rawEndIndex - playbackSlidingWindowEnd <= 1);

        if (!canIncrementalAdvance) {
            playbackSlidingWindowPoints.clear();
            playbackSlidingWindowSegmentPointCounts.clear();
            for (int i = rawStartIndex; i < rawEndIndex; ++i) {
                const PointCloudFrame rawFrame = readRawFrame(i);
                playbackSlidingWindowPoints += rawFrame.points;
                playbackSlidingWindowSegmentPointCounts.push_back(rawFrame.points.size());
            }
        } else {
            if (rawStartIndex > playbackSlidingWindowStart && !playbackSlidingWindowSegmentPointCounts.isEmpty()) {
                const int removeCount = playbackSlidingWindowSegmentPointCounts.front();
                if (removeCount > 0) {
                    playbackSlidingWindowPoints.remove(0, removeCount);
                }
                playbackSlidingWindowSegmentPointCounts.remove(0);
            }
            if (rawEndIndex > playbackSlidingWindowEnd) {
                const PointCloudFrame addedFrame = readRawFrame(rawEndIndex - 1);
                playbackSlidingWindowPoints += addedFrame.points;
                playbackSlidingWindowSegmentPointCounts.push_back(addedFrame.points.size());
            }
        }
        playbackSlidingWindowStart = rawStartIndex;
        playbackSlidingWindowEnd = rawEndIndex;
        playbackSlidingWindowTimestamp = 0;
        for (int i = playbackSlidingWindowStart; i < playbackSlidingWindowEnd; ++i) {
            playbackSlidingWindowTimestamp =
                std::max(playbackSlidingWindowTimestamp, readRawFrame(i).timestamp);
        }
    } else {
        playbackSlidingWindowStart = rawStartIndex;
        playbackSlidingWindowEnd = rawEndIndex;
        playbackSlidingWindowPoints.clear();
        playbackSlidingWindowSegmentPointCounts.clear();
        playbackSlidingWindowTimestamp = 0;
        for (int i = rawStartIndex; i < rawEndIndex; ++i) {
            const PointCloudFrame rawFrame = readRawFrame(i);
            playbackSlidingWindowPoints += rawFrame.points;
            playbackSlidingWindowTimestamp = std::max(playbackSlidingWindowTimestamp, rawFrame.timestamp);
        }
    }

    PointCloudFrame frame;
    frame.device_handle = 0;
    frame.timestamp = playbackSlidingWindowTimestamp;
    frame.points = playbackSlidingWindowPoints;

    applyPointCloudPipeline(frame);
    presentPointCloudFrame(frame);

    playbackFrame = playbackFrameIndex;
    updateLvx2PlaybackUi();
}

QString LivoxViewerWindow::lvx2DeviceTypeToModel(uint8_t deviceType) const
{
    switch (deviceType) {
    case kLivoxLidarTypeMid40: return "Mid40";
    case kLivoxLidarTypeMid70: return "Mid70";
    case kLivoxLidarTypeMid360: return "Mid360";
    case kLivoxLidarTypeMid360s: return "Mid360s";
    case kLivoxLidarTypeHorizon: return "Horizon";
    case kLivoxLidarTypeAvia: return "Avia";
    case kLivoxLidarTypeAvia2: return "Avia2";
    case kLivoxLidarTypeTele: return "Tele";
    case kLivoxLidarTypeHAP: return "HAP";
    case kLivoxLidarTypePA: return "PA";
    default: return QString("Unknown(%1)").arg(deviceType);
    }
}

void LivoxViewerWindow::rebuildLvx2DeviceTab()
{
    if (!lvx2DeviceTable) {
        return;
    }
    QSignalBlocker blocker(lvx2DeviceTable);
    lvx2DeviceTable->clearContents();
    lvx2DeviceTable->setRowCount(playbackDevices.size());
    for (int row = 0; row < playbackDevices.size(); ++row) {
        const auto& info = playbackDevices[row];
        auto* visibleItem = new QTableWidgetItem();
        visibleItem->setFlags((visibleItem->flags() | Qt::ItemIsUserCheckable) & ~Qt::ItemIsEditable);
        const bool visible = playbackDeviceVisible.value(info.lidarId, true);
        visibleItem->setCheckState(visible ? Qt::Checked : Qt::Unchecked);
        visibleItem->setData(Qt::UserRole, static_cast<qulonglong>(info.lidarId));
        lvx2DeviceTable->setItem(row, 0, visibleItem);
        const QString modelName =
            info.modelDisplay.isEmpty() ? lvx2DeviceTypeToModel(info.deviceType) : info.modelDisplay;
        lvx2DeviceTable->setItem(row, 1, new QTableWidgetItem(modelName));
        lvx2DeviceTable->setItem(row, 2, new QTableWidgetItem(info.lidarSn));
        lvx2DeviceTable->setItem(row, 3, new QTableWidgetItem(PushMsgParser::lidarIdToIpString(info.lidarId)));
    }

    disconnect(lvx2DeviceTable, &QTableWidget::itemChanged, this, nullptr);
    connect(lvx2DeviceTable, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item) {
        if (!item || item->column() != 0) {
            return;
        }
        const uint32_t lidarId = static_cast<uint32_t>(item->data(Qt::UserRole).toULongLong());
        playbackDeviceVisible[lidarId] = (item->checkState() == Qt::Checked);
        if (playbackSource) {
            playbackSource->invalidateCache();
        }
        playbackSlidingWindowStart = -1;
        playbackSlidingWindowEnd = -1;
        playbackSlidingWindowPoints.clear();
        playbackSlidingWindowSegmentPointCounts.clear();
        if (playbackActive && playbackFrame >= 0) {
            showLvx2PlaybackFrame(playbackFrame);
        }
    });
}

void LivoxViewerWindow::updateLvx2PlaybackUi()
{
    if (lvx2FileDock) {
        const bool showDock = playbackActive || playbackLoading;
        lvx2FileDock->setVisible(showDock);
        if (showDock) {
            lvx2FileDock->raise();
        }
    }

    if (!lvx2PlaybackBar) {
        return;
    }

    lvx2PlaybackBar->setVisible(playbackActive || playbackLoading);
    if (!playbackActive && !playbackLoading) {
        return;
    }

    if (lvx2PlayPauseButton) {
        QIcon icon = QIcon::fromTheme(playbackPlaying ? "media-playback-pause" : "media-playback-start");
        if (icon.isNull()) {
            icon = style()->standardIcon(playbackPlaying ? QStyle::SP_MediaPause : QStyle::SP_MediaPlay);
        }
        lvx2PlayPauseButton->setIcon(icon);
        lvx2PlayPauseButton->setToolTip(playbackPlaying ? "暂停" : "播放");
        lvx2PlayPauseButton->setEnabled(playbackActive && !playbackLoading);
    }
    if (lvx2PrevFrameButton) {
        lvx2PrevFrameButton->setEnabled(playbackActive && !playbackLoading && playbackFrame > 0);
    }
    if (lvx2NextFrameButton) {
        lvx2NextFrameButton->setEnabled(playbackActive && !playbackLoading &&
                                        playbackFrame >= 0 && playbackFrame < playbackFrameCount - 1);
    }
    if (lvx2ProgressSlider) {
        playbackUpdatingSlider = true;
        lvx2ProgressSlider->setRange(0, std::max(0, playbackFrameCount - 1));
        lvx2ProgressSlider->setEnabled(playbackActive && !playbackLoading && playbackFrameCount > 0);
        lvx2ProgressSlider->setValue(std::max(0, playbackFrame));
        playbackUpdatingSlider = false;
    }
    if (lvx2CloseButton) {
        lvx2CloseButton->setEnabled(!playbackLoading);
    }
    if (lvx2PlaybackModeCombo) {
        lvx2PlaybackModeCombo->setEnabled(playbackActive && !playbackLoading);
        lvx2PlaybackModeCombo->setCurrentIndex(playbackMode == Lvx2PlaybackMode::SlidingWindow ? 1 : 0);
    }
    if (lvx2PlaybackLabel) {
        const QString name = playbackPath.isEmpty() ? QString() : QFileInfo(playbackPath).fileName();
        if (playbackLoading) {
            lvx2PlaybackLabel->setText(QString("%1  加载中...").arg(name));
        } else {
            lvx2PlaybackLabel->setText(QString("%1  %2/%3")
                                       .arg(name)
                                       .arg(std::max(0, playbackFrame) + 1)
                                       .arg(std::max(1, playbackFrameCount)));
        }
    }
}

void LivoxViewerWindow::setLvx2PlaybackPlaying(bool playing)
{
    playbackPlaying = playing && playbackActive && !playbackLoading && playbackFrameCount > 0;

    if (lvx2PlaybackTimer) {
        if (playbackPlaying) {
            const double baseStepMs =
                (playbackMode == Lvx2PlaybackMode::SlidingWindow) ? 50.0 : static_cast<double>(frameIntervalMs);
            const int interval = std::max(1, static_cast<int>(baseStepMs / playbackSpeed));
            lvx2PlaybackTimer->start(interval);
        } else {
            lvx2PlaybackTimer->stop();
        }
    }
    updateLvx2PlaybackUi();
}

void LivoxViewerWindow::onLvx2PlaybackTick()
{
    if (!playbackActive) {
        setLvx2PlaybackPlaying(false);
        return;
    }

    const int nextFrame = playbackFrame + 1;
    if (nextFrame >= playbackFrameCount) {
        setLvx2PlaybackPlaying(false);
        return;
    }
    showLvx2PlaybackFrame(nextFrame);
}

void LivoxViewerWindow::onLvx2PlaybackSliderMoved(int value)
{
    if (playbackUpdatingSlider || !playbackActive) {
        return;
    }
    setLvx2PlaybackPlaying(false);
    showLvx2PlaybackFrame(value);
}

void LivoxViewerWindow::onLvx2PlaybackFileDropped(const QString& filePath)
{
    if (filePath.isEmpty()) {
        return;
    }
    if (PcapPlayback::isSupportedFile(filePath)) {
        loadPcapPlaybackFile(filePath);
        return;
    }
    loadLvx2PlaybackFile(filePath);
}

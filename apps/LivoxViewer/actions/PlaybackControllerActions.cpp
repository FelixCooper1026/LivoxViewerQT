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
    playbackState.source = source;
    playbackState.devices = playbackState.source ? playbackState.source->devices() : QVector<PlaybackDeviceInfo>();
    playbackState.deviceVisible.clear();
    for (const PlaybackDeviceInfo& device : playbackState.devices) {
        playbackState.deviceVisible.insert(device.lidarId, true);
    }
    rebuildLvx2DeviceTab();

    playbackState.slidingWindowStart = -1;
    playbackState.slidingWindowEnd = -1;
    playbackState.slidingWindowPoints.clear();
    playbackState.slidingWindowSegmentPointCounts.clear();
    playbackState.slidingWindowTimestamp = 0;
    playbackState.path = playbackState.source ? playbackState.source->path() : QString();
    playbackState.active = (playbackState.source && playbackState.source->frameCount() > 0);
    playbackState.frameCount = visiblePlaybackFrameCount(playbackState.source.get(), playbackState.mode, frameIntervalMs);
    if (playbackState.active && playbackState.frameCount <= 0) {
        playbackState.frameCount = 1;
    }
    playbackState.frame = -1;

    {
        QMutexLocker locker(&frameMutex);
        pendingFrames.clear();
        lastSeenTimestamp.clear();
    }

    setLvx2PlaybackPlaying(false);
    updateLvx2PlaybackUi();
    showLvx2PlaybackFrame(0);

    const QString fileName = playbackState.path.isEmpty() ? QString() : QFileInfo(playbackState.path).fileName();
    if (statusLabelBar) {
        if (playbackState.source && playbackState.source->kind() == Playback::SourceKind::Pcap) {
            statusLabelBar->setText(QString("Pcap播放: %1").arg(fileName));
        } else {
            statusLabelBar->setText(QString("LVX2播放: %1").arg(fileName));
        }
    }
    if (playbackState.source && playbackState.source->kind() == Playback::SourceKind::Pcap) {
        logMessage(QString("已加载Pcap文件: %1 (共 %2 帧)")
                       .arg(QDir::toNativeSeparators(playbackState.path))
                       .arg(playbackState.source->frameCount()));
    } else {
        logMessage(QString("已加载LVX2文件: %1").arg(QDir::toNativeSeparators(playbackState.path)));
    }
}

bool LivoxViewerWindow::loadLvx2PlaybackFile(const QString& filePath)
{
    closeLvx2Playback(false);

    playbackState.loading = true;
    playbackState.path = filePath;
    playbackState.loadToken++;
    const quint64 currentToken = playbackState.loadToken;

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
            if (currentToken != playbackState.loadToken) {
                return;
            }

            playbackState.loading = false;
            if (!ok) {
                playbackState.path.clear();
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

    playbackState.loadToken++;
    playbackState.source.reset();
    playbackState.active = false;
    playbackState.loading = false;
    playbackState.frame = -1;
    playbackState.frameCount = 0;
    playbackState.devices.clear();
    playbackState.deviceVisible.clear();
    rebuildLvx2DeviceTab();
    playbackState.slidingWindowStart = -1;
    playbackState.slidingWindowEnd = -1;
    playbackState.slidingWindowPoints.clear();
    playbackState.slidingWindowSegmentPointCounts.clear();
    playbackState.slidingWindowTimestamp = 0;
    playbackState.path.clear();

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
    if (!playbackState.active || !playbackState.source || playbackState.frameCount <= 0) {
        return;
    }

    playbackFrameIndex = std::clamp(playbackFrameIndex, 0, playbackState.frameCount - 1);
    const int sourceFrameCount = playbackState.source->frameCount();
    const int rawFrameCount = rawFramesPerPlaybackFrame(frameIntervalMs);
    int rawStartIndex = 0;
    int rawEndIndex = 0;
    if (playbackState.mode == Lvx2PlaybackMode::SlidingWindow) {
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
        if (!playbackState.source || !playbackState.source->readFrame(rawIndex, playbackState.deviceVisible, frame)) {
            return {};
        }
        return frame;
    };

    if (playbackState.mode == Lvx2PlaybackMode::SlidingWindow) {
        const bool canIncrementalAdvance =
            (playbackState.slidingWindowStart >= 0 && playbackState.slidingWindowEnd >= 0 &&
             rawStartIndex >= playbackState.slidingWindowStart && rawEndIndex >= playbackState.slidingWindowEnd &&
             rawStartIndex - playbackState.slidingWindowStart <= 1 && rawEndIndex - playbackState.slidingWindowEnd <= 1);

        if (!canIncrementalAdvance) {
            playbackState.slidingWindowPoints.clear();
            playbackState.slidingWindowSegmentPointCounts.clear();
            for (int i = rawStartIndex; i < rawEndIndex; ++i) {
                const PointCloudFrame rawFrame = readRawFrame(i);
                playbackState.slidingWindowPoints += rawFrame.points;
                playbackState.slidingWindowSegmentPointCounts.push_back(rawFrame.points.size());
            }
        } else {
            if (rawStartIndex > playbackState.slidingWindowStart && !playbackState.slidingWindowSegmentPointCounts.isEmpty()) {
                const int removeCount = playbackState.slidingWindowSegmentPointCounts.front();
                if (removeCount > 0) {
                    playbackState.slidingWindowPoints.remove(0, removeCount);
                }
                playbackState.slidingWindowSegmentPointCounts.remove(0);
            }
            if (rawEndIndex > playbackState.slidingWindowEnd) {
                const PointCloudFrame addedFrame = readRawFrame(rawEndIndex - 1);
                playbackState.slidingWindowPoints += addedFrame.points;
                playbackState.slidingWindowSegmentPointCounts.push_back(addedFrame.points.size());
            }
        }
        playbackState.slidingWindowStart = rawStartIndex;
        playbackState.slidingWindowEnd = rawEndIndex;
        playbackState.slidingWindowTimestamp = 0;
        for (int i = playbackState.slidingWindowStart; i < playbackState.slidingWindowEnd; ++i) {
            playbackState.slidingWindowTimestamp =
                std::max(playbackState.slidingWindowTimestamp, readRawFrame(i).timestamp);
        }
    } else {
        playbackState.slidingWindowStart = rawStartIndex;
        playbackState.slidingWindowEnd = rawEndIndex;
        playbackState.slidingWindowPoints.clear();
        playbackState.slidingWindowSegmentPointCounts.clear();
        playbackState.slidingWindowTimestamp = 0;
        for (int i = rawStartIndex; i < rawEndIndex; ++i) {
            const PointCloudFrame rawFrame = readRawFrame(i);
            playbackState.slidingWindowPoints += rawFrame.points;
            playbackState.slidingWindowTimestamp = std::max(playbackState.slidingWindowTimestamp, rawFrame.timestamp);
        }
    }

    PointCloudFrame frame;
    frame.device_handle = 0;
    frame.timestamp = playbackState.slidingWindowTimestamp;
    frame.points = playbackState.slidingWindowPoints;

    applyPointCloudPipeline(frame);
    presentPointCloudFrame(frame);

    playbackState.frame = playbackFrameIndex;
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
    lvx2DeviceTable->setRowCount(playbackState.devices.size());
    for (int row = 0; row < playbackState.devices.size(); ++row) {
        const auto& info = playbackState.devices[row];
        auto* visibleItem = new QTableWidgetItem();
        visibleItem->setFlags((visibleItem->flags() | Qt::ItemIsUserCheckable) & ~Qt::ItemIsEditable);
        const bool visible = playbackState.deviceVisible.value(info.lidarId, true);
        visibleItem->setCheckState(visible ? Qt::Checked : Qt::Unchecked);
        visibleItem->setData(Qt::UserRole, static_cast<qulonglong>(info.lidarId));
        lvx2DeviceTable->setItem(row, 0, visibleItem);
        const QString modelName =
            info.modelDisplay.isEmpty() ? lvx2DeviceTypeToModel(info.deviceType) : info.modelDisplay;
        const QString lidarIp = PushMsgParser::lidarIdToIpString(info.lidarId);
        QTableWidgetItem* modelItem = new QTableWidgetItem(modelName);
        QTableWidgetItem* snItem = new QTableWidgetItem(info.lidarSn);
        QTableWidgetItem* ipItem = new QTableWidgetItem(lidarIp);
        const QString deviceTip = QString("型号: %1\nSN: %2\nIP: %3").arg(modelName, info.lidarSn, lidarIp);
        visibleItem->setToolTip(deviceTip);
        modelItem->setToolTip(deviceTip);
        snItem->setToolTip(deviceTip);
        ipItem->setToolTip(deviceTip);
        lvx2DeviceTable->setItem(row, 1, modelItem);
        lvx2DeviceTable->setItem(row, 2, snItem);
        lvx2DeviceTable->setItem(row, 3, ipItem);
    }

    disconnect(lvx2DeviceTable, &QTableWidget::itemChanged, this, nullptr);
    connect(lvx2DeviceTable, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item) {
        if (!item || item->column() != 0) {
            return;
        }
        const uint32_t lidarId = static_cast<uint32_t>(item->data(Qt::UserRole).toULongLong());
        playbackState.deviceVisible[lidarId] = (item->checkState() == Qt::Checked);
        if (playbackState.source) {
            playbackState.source->invalidateCache();
        }
        playbackState.slidingWindowStart = -1;
        playbackState.slidingWindowEnd = -1;
        playbackState.slidingWindowPoints.clear();
        playbackState.slidingWindowSegmentPointCounts.clear();
        if (playbackState.active && playbackState.frame >= 0) {
            showLvx2PlaybackFrame(playbackState.frame);
        }
    });
}

void LivoxViewerWindow::updateLvx2PlaybackUi()
{
    if (lvx2FileDock) {
        const bool showDock = playbackState.active;
        lvx2FileDock->setVisible(showDock);
        if (showDock) {
            lvx2FileDock->raise();
        }
    }

    if (!playbackState.bar) {
        return;
    }

    playbackState.bar->setVisible(playbackState.active || playbackState.loading);
    if (!playbackState.active && !playbackState.loading) {
        return;
    }

    if (playbackState.playPauseButton) {
        QIcon icon = QIcon::fromTheme(playbackState.playing ? "media-playback-pause" : "media-playback-start");
        if (icon.isNull()) {
            icon = style()->standardIcon(playbackState.playing ? QStyle::SP_MediaPause : QStyle::SP_MediaPlay);
        }
        playbackState.playPauseButton->setIcon(icon);
        playbackState.playPauseButton->setToolTip(playbackState.playing ? "暂停" : "播放");
        playbackState.playPauseButton->setEnabled(playbackState.active && !playbackState.loading);
    }
    if (playbackState.prevFrameButton) {
        playbackState.prevFrameButton->setEnabled(playbackState.active && !playbackState.loading && playbackState.frame > 0);
    }
    if (playbackState.nextFrameButton) {
        playbackState.nextFrameButton->setEnabled(playbackState.active && !playbackState.loading &&
                                        playbackState.frame >= 0 && playbackState.frame < playbackState.frameCount - 1);
    }
    if (playbackState.progressSlider) {
        playbackState.updatingSlider = true;
        playbackState.progressSlider->setRange(0, std::max(0, playbackState.frameCount - 1));
        playbackState.progressSlider->setEnabled(playbackState.active && !playbackState.loading && playbackState.frameCount > 0);
        playbackState.progressSlider->setValue(std::max(0, playbackState.frame));
        playbackState.updatingSlider = false;
    }
    if (playbackState.closeButton) {
        playbackState.closeButton->setEnabled(!playbackState.loading);
    }
    if (playbackState.modeCombo) {
        playbackState.modeCombo->setEnabled(playbackState.active && !playbackState.loading);
        playbackState.modeCombo->setCurrentIndex(playbackState.mode == Lvx2PlaybackMode::SlidingWindow ? 1 : 0);
    }
    if (playbackState.label) {
        const QString name = playbackState.path.isEmpty() ? QString() : QFileInfo(playbackState.path).fileName();
        if (playbackState.loading) {
            playbackState.label->setText(QString("%1  加载中...").arg(name));
        } else {
            playbackState.label->setText(QString("%1  %2/%3")
                                       .arg(name)
                                       .arg(std::max(0, playbackState.frame) + 1)
                                       .arg(std::max(1, playbackState.frameCount)));
        }
    }
}

void LivoxViewerWindow::setLvx2PlaybackPlaying(bool playing)
{
    playbackState.playing = playing && playbackState.active && !playbackState.loading && playbackState.frameCount > 0;

    if (playbackState.timer) {
        if (playbackState.playing) {
            const double baseStepMs =
                (playbackState.mode == Lvx2PlaybackMode::SlidingWindow) ? 50.0 : static_cast<double>(frameIntervalMs);
            const int interval = std::max(1, static_cast<int>(baseStepMs / playbackState.speed));
            playbackState.timer->start(interval);
        } else {
            playbackState.timer->stop();
        }
    }
    updateLvx2PlaybackUi();
}

void LivoxViewerWindow::onLvx2PlaybackTick()
{
    if (!playbackState.active) {
        setLvx2PlaybackPlaying(false);
        return;
    }

    const int nextFrame = playbackState.frame + 1;
    if (nextFrame >= playbackState.frameCount) {
        setLvx2PlaybackPlaying(false);
        return;
    }
    showLvx2PlaybackFrame(nextFrame);
}

void LivoxViewerWindow::onLvx2PlaybackSliderMoved(int value)
{
    if (playbackState.updatingSlider || !playbackState.active) {
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

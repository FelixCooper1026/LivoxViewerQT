#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"

#include "Lvx2/Lvx2Reader.h"
#include "Pcap/PcapPlaybackController.h"
#include "Pcap/PushMsgParser.h"
#include "PointCloud/PointCloudColorizer.h"

#include <QDir>
#include <QFileInfo>
#include <QFontMetrics>
#include <QMessageBox>
#include <QStandardPaths>
#include <QStyleOptionSlider>
#include <QToolButton>

#include <algorithm>
#include <memory>
#include <thread>

namespace {

constexpr uint64_t kPlaybackDisplayFrameMs = 100;
constexpr uint64_t kPcapRawFrameDurationNs = 50000000ULL;
constexpr int kRawFramesPerDisplayFrame = 2;
constexpr const char* kPlaybackLabelFullTextProperty = "playbackFullText";

void setElidedPlaybackLabelText(QLabel* label, const QString& text)
{
    if (!label) {
        return;
    }

    label->setProperty(kPlaybackLabelFullTextProperty, text);
    label->setToolTip(text);
    label->setText(label->fontMetrics().elidedText(text, Qt::ElideMiddle, label->width()));
}

void refreshElidedPlaybackLabelText(QLabel* label)
{
    if (!label) {
        return;
    }

    const QString text = label->property(kPlaybackLabelFullTextProperty).toString();
    label->setText(label->fontMetrics().elidedText(text, Qt::ElideMiddle, label->width()));
}

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

static int displayFrameNumberForRawEndIndex(int rawEndIndex)
{
    return (std::max(1, rawEndIndex) + kRawFramesPerDisplayFrame - 1) / kRawFramesPerDisplayFrame;
}

static int displayPlaybackFrameCount(const Playback::Source* source)
{
    if (!source) {
        return 1;
    }

    const int rawFrameCount = source->frameCount();
    return std::max(1, (rawFrameCount + kRawFramesPerDisplayFrame - 1) / kRawFramesPerDisplayFrame);
}

static QVector<int> lineNumbersForPoints(const QVector<PointCloudPoint>& points)
{
    QVector<int> lineNumbers;
    for (const PointCloudPoint& point : points) {
        const int line = int(point.line);
        if (!lineNumbers.contains(line)) {
            lineNumbers.push_back(line);
        }
    }
    std::sort(lineNumbers.begin(), lineNumbers.end());
    return lineNumbers;
}

static QString formatPlaybackTime(int frameNumber, int totalFrameCount)
{
    const uint64_t totalMs = uint64_t(frameNumber) * kPlaybackDisplayFrameMs;
    const uint64_t durationMs = uint64_t(std::max(1, totalFrameCount)) * kPlaybackDisplayFrameMs;
    const uint64_t totalTenths = totalMs / 100ULL;
    const uint64_t totalSeconds = totalTenths / 10ULL;
    const uint64_t tenths = totalTenths % 10ULL;

    if (durationMs < 60000ULL) {
        return QString("%1.%2s").arg(totalSeconds).arg(tenths);
    }

    const uint64_t seconds = totalSeconds % 60ULL;
    const uint64_t totalMinutes = totalSeconds / 60ULL;
    if (durationMs < 3600000ULL) {
        return QString("%1:%2.%3")
            .arg(totalMinutes)
            .arg(seconds, 2, 10, QLatin1Char('0'))
            .arg(tenths);
    }

    const uint64_t minutes = totalMinutes % 60ULL;
    const uint64_t hours = totalMinutes / 60ULL;
    return QString("%1:%2:%3.%4")
        .arg(hours)
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 2, 10, QLatin1Char('0'))
        .arg(tenths);
}

static void updatePlaybackDeviceCardState(bool visible,
                                          QToolButton* visibleButton,
                                          QLabel* modelLabel,
                                          QLabel* snLabel,
                                          QLabel* ipLabel)
{
    ThemeIconUtils::setThemedSvgIcon(visibleButton,
        visible ? QStringLiteral(":/icons/eye.svg") : QStringLiteral(":/icons/eye_off.svg"));
    visibleButton->setToolTip(visible ? QStringLiteral("隐藏设备") : QStringLiteral("显示设备"));
    const QString textStyle = visible ? QString() : QStringLiteral("color: palette(mid);");
    for (QLabel* label : {modelLabel, snLabel, ipLabel}) {
        label->setStyleSheet(textStyle);
    }
}

} // namespace

void LivoxViewerWindow::finishPlaybackSourceLoad(int tabId, const std::shared_ptr<Playback::Source>& source)
{
    PlaybackControllerState* state = playbackStateForTab(tabId);
    if (!state) {
        return;
    }

    clearPlaybackImuSamples(*state);
    state->resetPlaybackImuHandles();
    state->source = source;
    state->devices = state->source ? state->source->devices() : QVector<PlaybackDeviceInfo>();
    state->deviceVisible.clear();
    for (const PlaybackDeviceInfo& device : state->devices) {
        state->deviceVisible.insert(device.lidarId, true);
        if (state->source && state->source->kind() == Playback::SourceKind::Pcap) {
            playbackImuHandleForLidarId(*state, device.lidarId);
        }
    }

    state->resetSlidingWindow();
    state->path = state->source ? state->source->path() : QString();
    state->active = (state->source && state->source->frameCount() > 0);
    state->loading = false;
    state->frameCount = visiblePlaybackFrameCount(state->source.get(), state->mode, frameIntervalMs);
    if (state->active && state->frameCount <= 0) {
        state->frameCount = 1;
    }
    state->frame = -1;

    if (boundPlaybackTabId != tabId) {
        return;
    }

    rebuildLvx2DeviceTab();
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
        logMessage(QString("已加载Pcap文件: %1 (共%2帧)")
                       .arg(QDir::toNativeSeparators(playbackState.path))
                       .arg(playbackState.source->frameCount()));
    } else {
        logMessage(QString("已加载LVX2文件: %1").arg(QDir::toNativeSeparators(playbackState.path)));
    }
}

bool LivoxViewerWindow::loadLvx2PlaybackFile(const QString& filePath)
{
    const int tabId = createOfflinePointCloudTab(filePath);

    playbackState.loading = true;
    playbackState.path = filePath;
    playbackState.loadToken++;
    const quint64 currentToken = playbackState.loadToken;

    setLvx2PlaybackPlaying(false);
    updateLvx2PlaybackUi();
    if (statusLabelBar) {
        statusLabelBar->setText(QString("正在加载LVX2: %1").arg(QFileInfo(filePath).fileName()));
    }
    saveBoundPlaybackState();

    std::thread([this, filePath, tabId, currentToken]() {
        auto source = std::make_shared<Lvx2::Lvx2Reader>();
        const bool ok = source->load(filePath);
        const QString errorMessage = source->errorMessage();

        QMetaObject::invokeMethod(this, [this, tabId, currentToken, source, ok, errorMessage]() {
            PlaybackControllerState* state = playbackStateForTab(tabId);
            if (!state || currentToken != state->loadToken) {
                return;
            }

            state->loading = false;
            if (!ok) {
                state->path.clear();
                updateLvx2PlaybackUi();
                updateStatus();
                QMessageBox::warning(this, "播放LVX2点云", errorMessage);
                closeVisualizationTab(tabId);
                return;
            }

            finishPlaybackSourceLoad(tabId, source);
        }, Qt::QueuedConnection);
    }).detach();

    return true;
}

void LivoxViewerWindow::closeLvx2Playback(bool clearView)
{
    if (boundPlaybackTabId >= 0) {
        closeVisualizationTab(boundPlaybackTabId);
        return;
    }

    setLvx2PlaybackPlaying(false);

    clearPlaybackImuSamples();
    playbackState.resetPlaybackImuHandles();
    playbackState.loadToken++;
    playbackState.source.reset();
    playbackState.active = false;
    playbackState.loading = false;
    playbackState.frame = -1;
    playbackState.frameCount = 0;
    playbackState.devices.clear();
    playbackState.deviceVisible.clear();
    rebuildLvx2DeviceTab();
    playbackState.resetSlidingWindow();
    playbackState.path.clear();

    updateLvx2PlaybackUi();
    if (clearView && pointCloudView) {
        pointCloudView->clearPointCloud();
    }

    updateStatus();
}

int LivoxViewerWindow::playbackRawEndIndexForFrame(int playbackFrameIndex,
                                                   Lvx2PlaybackMode mode,
                                                   uint64_t intervalMs) const
{
    const int sourceFrameCount = playbackState.source ? playbackState.source->frameCount() : 0;
    if (sourceFrameCount <= 0) {
        return 0;
    }

    const int frameIndex = std::clamp(playbackFrameIndex, 0, std::max(0, playbackState.frameCount - 1));
    if (mode == Lvx2PlaybackMode::SlidingWindow) {
        return std::min(frameIndex + 1, sourceFrameCount);
    }

    const int rawFrameCount = rawFramesPerPlaybackFrame(intervalMs);
    return std::min((frameIndex + 1) * rawFrameCount, sourceFrameCount);
}

int LivoxViewerWindow::playbackFrameIndexForRawEndIndex(int rawEndIndex,
                                                        Lvx2PlaybackMode mode,
                                                        uint64_t intervalMs) const
{
    const int sourceFrameCount = playbackState.source ? playbackState.source->frameCount() : 0;
    if (sourceFrameCount <= 0 || playbackState.frameCount <= 0) {
        return 0;
    }

    const int rawEnd = std::clamp(rawEndIndex, 1, sourceFrameCount);
    if (mode == Lvx2PlaybackMode::SlidingWindow) {
        return std::clamp(rawEnd - 1, 0, playbackState.frameCount - 1);
    }

    const int rawFrameCount = rawFramesPerPlaybackFrame(intervalMs);
    return std::clamp((rawEnd - 1) / rawFrameCount, 0, playbackState.frameCount - 1);
}

void LivoxViewerWindow::showLvx2PlaybackFrame(int playbackFrameIndex)
{
    if (!playbackState.active || !playbackState.source || playbackState.frameCount <= 0) {
        return;
    }

    playbackFrameIndex = std::clamp(playbackFrameIndex, 0, playbackState.frameCount - 1);
    const int previousPlaybackFrame = playbackState.frame;
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

    uint64_t windowStartTimestamp = 0;
    uint64_t windowEndTimestamp = 0;
    auto refreshWindowEndpointTimestamps = [&]() {
        if (!playbackState.slidingWindowSegmentTimestamps.isEmpty()) {
            windowStartTimestamp = playbackState.slidingWindowSegmentTimestamps.front();
            windowEndTimestamp = playbackState.slidingWindowSegmentTimestamps.back();
        }
    };
    auto recomputeSlidingWindowTimestamp = [&]() {
        uint64_t timestamp = 0;
        for (uint64_t segmentTimestamp : playbackState.slidingWindowSegmentTimestamps) {
            timestamp = std::max(timestamp, segmentTimestamp);
        }
        playbackState.slidingWindowTimestamp = timestamp;
    };
    auto updateSegmentedLegend = [this]() {
        if (!pointCloudView) {
            return;
        }

        const int mode = effectiveColorMode();
        if (mode == ColorByReflectivity) {
            pointCloudView->setLegend(ColorByReflectivity,
                                      0.0f,
                                      255.0f,
                                      true,
                                      {},
                                      {},
                                      PointCloudColorizer::reflectivityColorScaleStops(reflectivityColorScale));
        } else if (mode == ColorByDistance) {
            pointCloudView->setLegend(ColorByDistance, distanceLegendMin, distanceLegendMax, true);
        } else if (mode == ColorByElevation) {
            pointCloudView->setLegend(ColorByElevation, elevationLegendMin, elevationLegendMax, true);
        } else if (mode == ColorSolid) {
            pointCloudView->setLegend(ColorSolid, 0.0f, 1.0f, false);
        } else if (mode == ColorByLine) {
            QVector<int> usedLines;
            for (const QVector<int>& segmentLines : playbackState.slidingWindowSegmentLineNumbers) {
                for (int line : segmentLines) {
                    if (!usedLines.contains(line)) {
                        usedLines.push_back(line);
                    }
                }
            }
            std::sort(usedLines.begin(), usedLines.end());

            QVector<QColor> usedColors;
            usedColors.reserve(usedLines.size());
            for (int line : usedLines) {
                usedColors.push_back(lineColors.at(line % lineColors.size()));
            }
            pointCloudView->setLegend(ColorByLine, 0.0f, 1.0f, true, usedColors, usedLines);
        }
    };

    const bool useSegmentedSlidingWindow =
        playbackState.mode == Lvx2PlaybackMode::SlidingWindow &&
        pointCloudView;

    if (useSegmentedSlidingWindow) {
        const bool canIncrementalAdvance =
            (playbackState.slidingWindowStart >= 0 && playbackState.slidingWindowEnd >= 0 &&
             playbackState.slidingWindowSegmentPointCounts.size() == playbackState.slidingWindowSegmentTimestamps.size() &&
             playbackState.slidingWindowSegmentLineNumbers.size() == playbackState.slidingWindowSegmentTimestamps.size() &&
             pointCloudView->pointCloudSegmentCount() == playbackState.slidingWindowSegmentTimestamps.size() &&
             rawStartIndex >= playbackState.slidingWindowStart && rawEndIndex >= playbackState.slidingWindowEnd &&
             rawStartIndex - playbackState.slidingWindowStart <= 1 && rawEndIndex - playbackState.slidingWindowEnd <= 1);

        auto appendProcessedSegment = [&](int rawIndex) {
            PointCloudFrame segmentFrame = readRawFrame(rawIndex);
            applyPointCloudPipeline(segmentFrame, pointCloudView);
            playbackState.slidingWindowSegmentPointCounts.push_back(segmentFrame.points.size());
            playbackState.slidingWindowSegmentTimestamps.push_back(segmentFrame.timestamp);
            playbackState.slidingWindowSegmentLineNumbers.push_back(lineNumbersForPoints(segmentFrame.points));
            playbackState.slidingWindowTimestamp = std::max(playbackState.slidingWindowTimestamp, segmentFrame.timestamp);
            pointCloudView->appendPointCloudSegment(std::move(segmentFrame.points));
        };

        if (!canIncrementalAdvance) {
            playbackState.resetSlidingWindow();
            pointCloudView->clearPointCloudSegments();
            for (int i = rawStartIndex; i < rawEndIndex; ++i) {
                appendProcessedSegment(i);
            }
        } else {
            if (rawStartIndex > playbackState.slidingWindowStart && !playbackState.slidingWindowSegmentPointCounts.isEmpty()) {
                const uint64_t removedTimestamp = playbackState.slidingWindowSegmentTimestamps.isEmpty()
                    ? 0
                    : playbackState.slidingWindowSegmentTimestamps.front();
                playbackState.slidingWindowSegmentPointCounts.remove(0);
                if (!playbackState.slidingWindowSegmentTimestamps.isEmpty()) {
                    playbackState.slidingWindowSegmentTimestamps.remove(0);
                }
                if (!playbackState.slidingWindowSegmentLineNumbers.isEmpty()) {
                    playbackState.slidingWindowSegmentLineNumbers.remove(0);
                }
                pointCloudView->removeFirstPointCloudSegment();
                if (removedTimestamp == playbackState.slidingWindowTimestamp) {
                    recomputeSlidingWindowTimestamp();
                }
            }
            if (rawEndIndex > playbackState.slidingWindowEnd) {
                appendProcessedSegment(rawEndIndex - 1);
            }
        }
        playbackState.slidingWindowStart = rawStartIndex;
        playbackState.slidingWindowEnd = rawEndIndex;
        refreshWindowEndpointTimestamps();
        updateSegmentedLegend();
    } else {
        playbackState.resetSlidingWindow();
        playbackState.slidingWindowStart = rawStartIndex;
        playbackState.slidingWindowEnd = rawEndIndex;
        PointCloudFrame frame;
        frame.device_handle = 0;
        frame.timestamp = 0;
        for (int i = rawStartIndex; i < rawEndIndex; ++i) {
            const PointCloudFrame rawFrame = readRawFrame(i);
            if (playbackState.slidingWindowSegmentTimestamps.isEmpty()) {
                windowStartTimestamp = rawFrame.timestamp;
            }
            frame.points += rawFrame.points;
            frame.timestamp = std::max(frame.timestamp, rawFrame.timestamp);
            playbackState.slidingWindowSegmentPointCounts.push_back(rawFrame.points.size());
            playbackState.slidingWindowSegmentTimestamps.push_back(rawFrame.timestamp);
            playbackState.slidingWindowSegmentLineNumbers.push_back(lineNumbersForPoints(rawFrame.points));
        }
        playbackState.slidingWindowTimestamp = frame.timestamp;
        refreshWindowEndpointTimestamps();

        applyPointCloudPipeline(frame, pointCloudView);
        if (pointCloudView) {
            pointCloudView->updatePointCloud(std::move(frame));
        }
    }
    if (selectionRealtimeEnabled && pointCloudView && (attrTable || selectionTable)) {
        updateSelectionTableAndLog();
    }

    if (playbackState.source->kind() == Playback::SourceKind::Pcap) {
        const bool rebuildImuHistory = previousPlaybackFrame < 0 || playbackFrameIndex != previousPlaybackFrame + 1;
        const uint64_t imuStartTimestamp =
            rebuildImuHistory ? 0ULL
                              : (playbackState.mode == Lvx2PlaybackMode::SlidingWindow
                                     ? windowEndTimestamp
                                     : windowStartTimestamp);
        const QVector<Playback::ImuSample> imuSamples =
            playbackState.source->readImuSamples(imuStartTimestamp, windowEndTimestamp + kPcapRawFrameDurationNs);
        appendPlaybackImuSamples(playbackState, imuSamples, rebuildImuHistory);
    }

    playbackState.frame = playbackFrameIndex;
    updateLvx2PlaybackUi();
}

QString LivoxViewerWindow::lvx2DeviceTypeToModel(uint8_t deviceType) const
{
    switch (deviceType) {
    case kLivoxLidarTypeMid360: return "Mid360";
    case kLivoxLidarTypeMid360s: return "Mid360s";
    case kLivoxLidarTypeMid360l: return "Mid360l";
    case kLivoxLidarTypeAvia2: return "Avia2";
    case kLivoxLidarTypeHAP: return "HAP";
    case kLivoxLidarTypePA: return "PA";
    default: return QString("Unknown(%1)").arg(deviceType);
    }
}

void LivoxViewerWindow::rebuildLvx2DeviceTab()
{
    if (!lvx2DeviceListWidget) {
        return;
    }
    QVBoxLayout* deviceListLayout = static_cast<QVBoxLayout*>(lvx2DeviceListWidget->layout());

    while (QLayoutItem* item = deviceListLayout->takeAt(0)) {
        delete item->widget();
        delete item;
    }
    for (int row = 0; row < playbackState.devices.size(); ++row) {
        const auto& info = playbackState.devices[row];
        const QString modelName =
            info.modelDisplay.isEmpty() ? lvx2DeviceTypeToModel(info.deviceType) : info.modelDisplay;
        const QString lidarIp = PushMsgParser::lidarIdToIpString(info.lidarId);
        const QString deviceTip = QString("型号: %1\nSN: %2\nIP: %3").arg(modelName, info.lidarSn, lidarIp);
        const uint32_t lidarId = info.lidarId;

        QFrame* card = new QFrame(lvx2DeviceListWidget);
        card->setObjectName("PlaybackDeviceCard");
        card->setFrameShape(QFrame::StyledPanel);
        card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        card->setToolTip(deviceTip);
        card->setStyleSheet("QFrame#PlaybackDeviceCard { border: 1px solid palette(mid); border-radius: 6px; background: palette(base); }");

        QVBoxLayout* cardLayout = new QVBoxLayout(card);
        cardLayout->setContentsMargins(8, 6, 8, 6);
        cardLayout->setSpacing(4);

        QHBoxLayout* headerLayout = new QHBoxLayout();
        headerLayout->setContentsMargins(0, 0, 0, 0);
        headerLayout->setSpacing(6);
        const bool visible = playbackState.deviceVisible.value(lidarId, true);
        QToolButton* visibleButton = new QToolButton(card);
        visibleButton->setCheckable(true);
        visibleButton->setChecked(visible);
        visibleButton->setAutoRaise(true);
        visibleButton->setIconSize(QSize(20, 20));
        visibleButton->setFixedSize(28, 28);
        QLabel* modelLabel = new QLabel(modelName, card);
        QFont modelFont = modelLabel->font();
        modelFont.setBold(true);
        modelLabel->setFont(modelFont);
        modelLabel->setToolTip(deviceTip);
        modelLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        modelLabel->setWordWrap(true);
        headerLayout->addWidget(visibleButton);
        headerLayout->addWidget(modelLabel, 1);
        cardLayout->addLayout(headerLayout);

        QLabel* snLabel = new QLabel(QStringLiteral("SN: %1").arg(info.lidarSn), card);
        QLabel* ipLabel = new QLabel(QStringLiteral("IP: %1").arg(lidarIp), card);
        for (QLabel* label : {snLabel, ipLabel}) {
            label->setToolTip(deviceTip);
            label->setWordWrap(true);
        }
        cardLayout->addWidget(snLabel);
        cardLayout->addWidget(ipLabel);
        updatePlaybackDeviceCardState(visible, visibleButton, modelLabel, snLabel, ipLabel);

        const std::shared_ptr<Playback::Source> source = playbackState.source;
        connect(visibleButton, &QToolButton::toggled, this, [this, source, lidarId, visibleButton, modelLabel, snLabel, ipLabel](bool checked) {
            updatePlaybackDeviceCardState(checked, visibleButton, modelLabel, snLabel, ipLabel);
            playbackState.deviceVisible[lidarId] = checked;
            source->invalidateCache();
            playbackState.resetSlidingWindow();
            if (playbackState.active && playbackState.frame >= 0) {
                showLvx2PlaybackFrame(playbackState.frame);
            }
        });

        deviceListLayout->addWidget(card);
    }

    deviceListLayout->addStretch();
}

void LivoxViewerWindow::updateLvx2PlaybackUi()
{
    if (lvx2FileDock) {
        if (playbackState.active) {
            if (!playbackState.fileInfoDockVisible) {
                lvx2FileDock->show();
                lvx2FileDock->raise();
                playbackState.fileInfoDockVisible = true;
            }
        } else {
            lvx2FileDock->hide();
            playbackState.fileInfoDockVisible = false;
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
        const int displayFrameCount = displayPlaybackFrameCount(playbackState.source.get());
        const int rawEndIndex = playbackRawEndIndexForFrame(playbackState.frame, playbackState.mode, frameIntervalMs);
        const int displayFrameIndex = displayFrameNumberForRawEndIndex(rawEndIndex) - 1;
        playbackState.progressSlider->setRange(0, std::max(0, displayFrameCount - 1));
        playbackState.progressSlider->setEnabled(playbackState.active && !playbackState.loading && playbackState.frameCount > 0);
        playbackState.progressSlider->setValue(std::clamp(displayFrameIndex, 0, std::max(0, displayFrameCount - 1)));
        playbackState.updatingSlider = false;
    }
    if (playbackState.modeCombo) {
        playbackState.modeCombo->setEnabled(playbackState.active && !playbackState.loading);
        playbackState.modeCombo->setCurrentIndex(playbackState.mode == Lvx2PlaybackMode::SlidingWindow ? 1 : 0);
    }
    if (playbackState.label) {
        const QString path = QDir::toNativeSeparators(playbackState.path);
        QString playbackInfo;
        if (playbackState.loading) {
            playbackInfo = QString("%1    加载中...").arg(path);
        } else {
            const int rawEndIndex = playbackRawEndIndexForFrame(playbackState.frame, playbackState.mode, frameIntervalMs);
            const int currentFrame = displayFrameNumberForRawEndIndex(rawEndIndex);
            const int totalFrames = displayPlaybackFrameCount(playbackState.source.get());
            playbackInfo = QString("%1    时间 %2 / %3    帧 %4 / %5")
                               .arg(path)
                               .arg(formatPlaybackTime(currentFrame, totalFrames))
                               .arg(formatPlaybackTime(totalFrames, totalFrames))
                               .arg(currentFrame)
                               .arg(totalFrames);
        }
        setElidedPlaybackLabelText(playbackState.label, playbackInfo);
    }
}

bool LivoxViewerWindow::eventFilter(QObject* watched, QEvent* event)
{
    if (watched == playbackState.progressSlider &&
        playbackState.progressSlider &&
        playbackState.progressSlider->isEnabled()) {
        auto sliderValueAt = [this](const QPoint& pos) {
            QStyleOptionSlider option;
            option.initFrom(playbackState.progressSlider);
            option.orientation = playbackState.progressSlider->orientation();
            option.minimum = playbackState.progressSlider->minimum();
            option.maximum = playbackState.progressSlider->maximum();
            option.sliderPosition = playbackState.progressSlider->sliderPosition();
            option.sliderValue = playbackState.progressSlider->value();
            option.singleStep = playbackState.progressSlider->singleStep();
            option.pageStep = playbackState.progressSlider->pageStep();
            option.upsideDown = playbackState.progressSlider->invertedAppearance();

            const int span = std::max(1, playbackState.progressSlider->width());
            return QStyle::sliderValueFromPosition(option.minimum,
                                                   option.maximum,
                                                   pos.x(),
                                                   span,
                                                   option.upsideDown);
        };

        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton) {
                playbackState.progressSliderDragging = true;
                playbackState.progressSlider->setSliderDown(true);
                playbackState.progressSlider->grabMouse();
                playbackState.progressSlider->setValue(sliderValueAt(mouseEvent->pos()));
                return true;
            }
        } else if (event->type() == QEvent::MouseMove && playbackState.progressSliderDragging) {
            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
            playbackState.progressSlider->setValue(sliderValueAt(mouseEvent->pos()));
            return true;
        } else if (event->type() == QEvent::MouseButtonRelease && playbackState.progressSliderDragging) {
            QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton) {
                playbackState.progressSlider->setValue(sliderValueAt(mouseEvent->pos()));
                playbackState.progressSlider->releaseMouse();
                playbackState.progressSlider->setSliderDown(false);
                playbackState.progressSliderDragging = false;
                return true;
            }
        }
    } else if (watched == playbackState.progressSlider &&
               event->type() == QEvent::MouseButtonRelease &&
               playbackState.progressSliderDragging) {
        if (playbackState.progressSlider) {
            playbackState.progressSlider->releaseMouse();
            playbackState.progressSlider->setSliderDown(false);
        }
        playbackState.progressSliderDragging = false;
        return true;
    } else if (watched == playbackState.label &&
               event->type() == QEvent::Resize) {
        refreshElidedPlaybackLabelText(playbackState.label);
    }

    return QMainWindow::eventFilter(watched, event);
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
    const int sourceFrameCount = playbackState.source ? playbackState.source->frameCount() : 0;
    const int rawEndIndex = std::min((value + 1) * kRawFramesPerDisplayFrame, sourceFrameCount);
    showLvx2PlaybackFrame(playbackFrameIndexForRawEndIndex(rawEndIndex, playbackState.mode, frameIntervalMs));
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

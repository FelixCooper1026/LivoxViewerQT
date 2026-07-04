#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"

#include "Lvx/LvxReader.h"
#include "Lvx2/Lvx2Reader.h"
#include "Pcap/PcapPlaybackController.h"
#include "Pcap/PushMsgParser.h"
#include "PointCloud/PointCloudColorizer.h"
#include "Rosbag/RosbagPlaybackController.h"
#include "Rosbag/RosbagPlaybackSource.h"

#include <QDir>
#include <QFileInfo>
#include <QFontMetrics>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStandardPaths>
#include <QStyleOptionSlider>
#include <QToolButton>

#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>

namespace {

constexpr uint64_t kPlaybackDisplayFrameMs = 100;
constexpr uint64_t kPlaybackDisplayFrameNs = kPlaybackDisplayFrameMs * 1000000ULL;
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

static uint64_t sourceNominalFrameDurationNs(const Playback::Source* source)
{
    return source ? std::max<uint64_t>(1, source->nominalFrameDurationNs()) : 50000000ULL;
}

static int rawFramesForDurationNs(const Playback::Source* source, uint64_t durationNs)
{
    const uint64_t nominalNs = sourceNominalFrameDurationNs(source);
    if (source && source->hasFrameTimestamps()) {
        return std::max(1, int(std::llround(double(durationNs) / double(nominalNs))));
    }
    return std::max(1, int((durationNs + nominalNs - 1ULL) / nominalNs));
}

static int rawFramesPerPlaybackFrame(const Playback::Source* source, uint64_t frameIntervalMs)
{
    return rawFramesForDurationNs(source, std::max<uint64_t>(1, frameIntervalMs) * 1000000ULL);
}

static int rawFramesPerDisplayFrame(const Playback::Source* source)
{
    return rawFramesForDurationNs(source, kPlaybackDisplayFrameNs);
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

    const int rawFramesPerStep = rawFramesPerPlaybackFrame(source, frameIntervalMs);
    return (rawFrameCount + rawFramesPerStep - 1) / rawFramesPerStep;
}

static int displayFrameNumberForRawEndIndex(const Playback::Source* source, int rawEndIndex)
{
    const int rawFramesPerDisplay = rawFramesPerDisplayFrame(source);
    return (std::max(1, rawEndIndex) + rawFramesPerDisplay - 1) / rawFramesPerDisplay;
}

static int displayPlaybackFrameCount(const Playback::Source* source)
{
    if (!source) {
        return 1;
    }

    const int rawFrameCount = source->frameCount();
    const int rawFramesPerDisplay = rawFramesPerDisplayFrame(source);
    return std::max(1, (rawFrameCount + rawFramesPerDisplay - 1) / rawFramesPerDisplay);
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

static uint64_t playbackElapsedNsForRawEndIndex(const Playback::Source* source, int rawEndIndex)
{
    if (!source || rawEndIndex <= 0) {
        return 0;
    }

    const int sourceFrameCount = source->frameCount();
    const int frameIndex = std::clamp(rawEndIndex - 1, 0, std::max(0, sourceFrameCount - 1));
    const uint64_t nominalNs = sourceNominalFrameDurationNs(source);
    if (source->hasFrameTimestamps() && sourceFrameCount > 0) {
        const uint64_t firstTimestampNs = source->frameTimestampNs(0);
        const uint64_t frameTimestampNs = source->frameTimestampNs(frameIndex);
        if (frameTimestampNs >= firstTimestampNs) {
            return frameTimestampNs - firstTimestampNs + nominalNs;
        }
    }
    return uint64_t(rawEndIndex) * nominalNs;
}

static uint64_t playbackDurationNs(const Playback::Source* source)
{
    if (!source || source->frameCount() <= 0) {
        return kPlaybackDisplayFrameNs;
    }

    const int sourceFrameCount = source->frameCount();
    const uint64_t nominalNs = sourceNominalFrameDurationNs(source);
    if (source->hasFrameTimestamps()) {
        const uint64_t firstTimestampNs = source->frameTimestampNs(0);
        const uint64_t lastTimestampNs = source->frameTimestampNs(sourceFrameCount - 1);
        if (lastTimestampNs >= firstTimestampNs) {
            return lastTimestampNs - firstTimestampNs + nominalNs;
        }
    }
    return uint64_t(sourceFrameCount) * nominalNs;
}

static QString formatPlaybackTimeNs(uint64_t timeNs, uint64_t durationNs)
{
    const uint64_t totalMs = timeNs / 1000000ULL;
    const uint64_t durationMs = std::max<uint64_t>(1, durationNs / 1000000ULL);
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

static QString playbackSourceDisplayName(Playback::SourceKind kind)
{
    switch (kind) {
    case Playback::SourceKind::Lvx:
        return QStringLiteral("LVX");
    case Playback::SourceKind::Pcap:
        return QStringLiteral("Pcap");
    case Playback::SourceKind::Rosbag:
        return QStringLiteral("ROSbag");
    case Playback::SourceKind::Lvx2:
    default:
        return QStringLiteral("LVX2");
    }
}

static bool playbackSourceProvidesImu(Playback::SourceKind kind)
{
    return kind == Playback::SourceKind::Pcap || kind == Playback::SourceKind::Rosbag;
}

static uint64_t playbackImuQueryPaddingNs(const Playback::Source* source)
{
    return sourceNominalFrameDurationNs(source);
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

LivoxViewerWindow::PlaybackUiSnapshot LivoxViewerWindow::playbackUiSnapshot() const
{
    PlaybackUiSnapshot snapshot;
    snapshot.active = playbackState.active;
    snapshot.loading = playbackState.loading;
    snapshot.playing = playbackState.playing;
    snapshot.available = boundPlaybackTabId >= 0 && (playbackState.active || playbackState.loading);
    snapshot.canPlayPause = playbackState.active && !playbackState.loading && playbackState.frameCount > 0;
    snapshot.canFirst = playbackState.active && !playbackState.loading && playbackState.frameCount > 0;
    snapshot.canPrevious = snapshot.canFirst && playbackState.frame > 0;
    snapshot.canNext = snapshot.canFirst &&
                       playbackState.frame >= 0 &&
                       playbackState.frame < playbackState.frameCount - 1;
    snapshot.canLast = snapshot.canFirst;
    snapshot.speedText = QStringLiteral("x%1").arg(playbackState.speed, 0, 'f', 1);
    snapshot.modeIndex = playbackState.mode == Lvx2PlaybackMode::SlidingWindow ? 1 : 0;

    const int displayFrameCount = displayPlaybackFrameCount(playbackState.source.get());
    const int rawEndIndex = playbackRawEndIndexForFrame(playbackState.frame, playbackState.mode, frameIntervalMs);
    const int displayFrameIndex = displayFrameNumberForRawEndIndex(playbackState.source.get(), rawEndIndex) - 1;
    snapshot.sliderMinimum = 0;
    snapshot.sliderMaximum = std::max(0, displayFrameCount - 1);
    snapshot.sliderValue = std::clamp(displayFrameIndex, 0, snapshot.sliderMaximum);

    const QString path = QDir::toNativeSeparators(playbackState.path);
    snapshot.pathText = path;
    if (playbackState.loading) {
        snapshot.timeText = QStringLiteral("加载中...");
        snapshot.infoText = QString("%1    加载中...").arg(path);
    } else if (playbackState.active) {
        const int currentFrame = displayFrameNumberForRawEndIndex(playbackState.source.get(), rawEndIndex);
        const int totalFrames = displayPlaybackFrameCount(playbackState.source.get());
        const uint64_t elapsedNs = playbackElapsedNsForRawEndIndex(playbackState.source.get(), rawEndIndex);
        const uint64_t durationNs = playbackDurationNs(playbackState.source.get());
        snapshot.timeText = QStringLiteral("时间 %1 / %2")
                                .arg(formatPlaybackTimeNs(elapsedNs, durationNs))
                                .arg(formatPlaybackTimeNs(durationNs, durationNs));
        snapshot.frameText = QStringLiteral("帧 %1 / %2")
                                 .arg(currentFrame)
                                 .arg(totalFrames);
        snapshot.infoText = QString("%1    时间 %2 / %3    帧 %4 / %5")
                                .arg(path)
                                .arg(formatPlaybackTimeNs(elapsedNs, durationNs))
                                .arg(formatPlaybackTimeNs(durationNs, durationNs))
                                .arg(currentFrame)
                                .arg(totalFrames);
    }

    return snapshot;
}

void LivoxViewerWindow::playbackToggle()
{
    if (!playbackState.active) {
        return;
    }
    if (playbackState.playing) {
        setLvx2PlaybackPlaying(false);
        return;
    }
    if (playbackState.frameCount <= 0) {
        return;
    }
    if (playbackState.frame < 0 || playbackState.frame >= playbackState.frameCount - 1) {
        showLvx2PlaybackFrame(0);
    }
    setLvx2PlaybackPlaying(true);
}

void LivoxViewerWindow::playbackShowFirstFrame()
{
    if (!playbackState.active || playbackState.frameCount <= 0) {
        return;
    }
    setLvx2PlaybackPlaying(false);
    showLvx2PlaybackFrame(0);
}

void LivoxViewerWindow::playbackShowPreviousFrame()
{
    if (!playbackState.active || playbackState.frameCount <= 0) {
        return;
    }
    setLvx2PlaybackPlaying(false);
    showLvx2PlaybackFrame(std::max(0, playbackState.frame - 1));
}

void LivoxViewerWindow::playbackShowNextFrame()
{
    if (!playbackState.active || playbackState.frameCount <= 0) {
        return;
    }
    setLvx2PlaybackPlaying(false);
    showLvx2PlaybackFrame(std::min(playbackState.frameCount - 1, playbackState.frame + 1));
}

void LivoxViewerWindow::playbackShortcutPreviousFrame()
{
    if (!playbackState.active || playbackState.frameCount <= 0 || !playbackState.source) {
        return;
    }

    const int displayFrameCount = displayPlaybackFrameCount(playbackState.source.get());
    const int rawEndIndex = playbackRawEndIndexForFrame(playbackState.frame, playbackState.mode, frameIntervalMs);
    const int displayFrameIndex = std::clamp(displayFrameNumberForRawEndIndex(playbackState.source.get(), rawEndIndex) - 1,
                                            0,
                                            std::max(0, displayFrameCount - 1));
    playbackSeekToDisplayFrame(std::max(0, displayFrameIndex - 1));
}

void LivoxViewerWindow::playbackShortcutNextFrame()
{
    if (!playbackState.active || playbackState.frameCount <= 0 || !playbackState.source) {
        return;
    }

    const int displayFrameCount = displayPlaybackFrameCount(playbackState.source.get());
    const int rawEndIndex = playbackRawEndIndexForFrame(playbackState.frame, playbackState.mode, frameIntervalMs);
    const int displayFrameIndex = std::clamp(displayFrameNumberForRawEndIndex(playbackState.source.get(), rawEndIndex) - 1,
                                            0,
                                            std::max(0, displayFrameCount - 1));
    playbackSeekToDisplayFrame(std::min(std::max(0, displayFrameCount - 1), displayFrameIndex + 1));
}

void LivoxViewerWindow::playbackShowLastFrame()
{
    if (!playbackState.active || playbackState.frameCount <= 0) {
        return;
    }
    setLvx2PlaybackPlaying(false);
    showLvx2PlaybackFrame(playbackState.frameCount - 1);
}

void LivoxViewerWindow::playbackSeekToDisplayFrame(int value)
{
    if (playbackState.updatingSlider || !playbackState.active) {
        return;
    }
    setLvx2PlaybackPlaying(false);
    const int sourceFrameCount = playbackState.source ? playbackState.source->frameCount() : 0;
    const int rawEndIndex = std::min((value + 1) * rawFramesPerDisplayFrame(playbackState.source.get()), sourceFrameCount);
    showLvx2PlaybackFrame(playbackFrameIndexForRawEndIndex(rawEndIndex, playbackState.mode, frameIntervalMs));
}

void LivoxViewerWindow::playbackSetSpeedText(const QString& text)
{
    QString speedText = text;
    speedText.remove('x');
    bool ok = false;
    const double speed = speedText.toDouble(&ok);
    if (!ok || speed <= 0.0) {
        return;
    }
    playbackState.speed = speed;
    if (playbackState.playing) {
        setLvx2PlaybackPlaying(true);
    } else {
        updateLvx2PlaybackUi();
    }
}

void LivoxViewerWindow::playbackSetModeIndex(int index)
{
    const int rawEndIndex = playbackRawEndIndexForFrame(playbackState.frame, playbackState.mode, frameIntervalMs);
    playbackState.mode = (index == 1) ? Lvx2PlaybackMode::SlidingWindow : Lvx2PlaybackMode::FrameByFrame;
    playbackState.resetSlidingWindow();
    if (!playbackState.active) {
        updateLvx2PlaybackUi();
        return;
    }

    const int sourceFrameCount = playbackState.source ? playbackState.source->frameCount() : 0;
    const int rawFramesPerStep = rawFramesPerPlaybackFrame(playbackState.source.get(), frameIntervalMs);
    if (playbackState.mode == Lvx2PlaybackMode::SlidingWindow) {
        playbackState.frameCount = sourceFrameCount;
    } else {
        playbackState.frameCount = (sourceFrameCount + rawFramesPerStep - 1) / rawFramesPerStep;
    }
    if (playbackState.frameCount <= 0) {
        playbackState.frameCount = 1;
    }
    const int targetFrame = playbackFrameIndexForRawEndIndex(rawEndIndex, playbackState.mode, frameIntervalMs);
    showLvx2PlaybackFrame(targetFrame);
    if (playbackState.playing) {
        setLvx2PlaybackPlaying(true);
    } else {
        updateLvx2PlaybackUi();
    }
}

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
        if (state->source && playbackSourceProvidesImu(state->source->kind())) {
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
    const QString sourceName = playbackState.source
        ? playbackSourceDisplayName(playbackState.source->kind())
        : QStringLiteral("点云");
    if (statusLabelBar) {
        statusLabelBar->setText(QString("%1播放: %2").arg(sourceName, fileName));
    }
    logMessage(QString("已加载%1文件: %2 (共%3帧)")
                   .arg(sourceName)
                   .arg(QDir::toNativeSeparators(playbackState.path))
                   .arg(playbackState.source ? playbackState.source->frameCount() : 0));
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

bool LivoxViewerWindow::loadLvxPlaybackFile(const QString& filePath)
{
    const int tabId = createOfflinePointCloudTab(filePath);

    playbackState.loading = true;
    playbackState.path = filePath;
    playbackState.loadToken++;
    const quint64 currentToken = playbackState.loadToken;

    setLvx2PlaybackPlaying(false);
    updateLvx2PlaybackUi();
    if (statusLabelBar) {
        statusLabelBar->setText(QString("正在加载LVX: %1").arg(QFileInfo(filePath).fileName()));
    }
    saveBoundPlaybackState();

    std::thread([this, filePath, tabId, currentToken]() {
        auto source = std::make_shared<Lvx::LvxReader>();
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
                QMessageBox::warning(this, "播放LVX点云", errorMessage);
                closeVisualizationTab(tabId);
                return;
            }

            finishPlaybackSourceLoad(tabId, source);
        }, Qt::QueuedConnection);
    }).detach();

    return true;
}

bool LivoxViewerWindow::loadRosbagPlaybackFile(const QString& filePath)
{
    const int tabId = createOfflinePointCloudTab(filePath);

    playbackState.loading = true;
    playbackState.path = filePath;
    playbackState.loadToken++;
    const quint64 currentToken = playbackState.loadToken;

    setLvx2PlaybackPlaying(false);
    updateLvx2PlaybackUi();
    if (statusLabelBar) {
        statusLabelBar->setText(QString("正在加载ROSbag: %1").arg(QFileInfo(filePath).fileName()));
    }
    saveBoundPlaybackState();

    std::thread([this, filePath, tabId, currentToken]() {
        auto source = std::make_shared<RosbagPlaybackSource>(int(frameIntervalMs));
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
                QMessageBox::warning(this, "播放ROSbag点云", errorMessage);
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

    const int rawFrameCount = rawFramesPerPlaybackFrame(playbackState.source.get(), intervalMs);
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

    const int rawFrameCount = rawFramesPerPlaybackFrame(playbackState.source.get(), intervalMs);
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
    const int rawFrameCount = rawFramesPerPlaybackFrame(playbackState.source.get(), frameIntervalMs);
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
    if (pointCloudView) {
        pointCloudView->applyPendingFitViewRequest();
    }
    if (playbackSourceProvidesImu(playbackState.source->kind())) {
        const bool rebuildImuHistory = previousPlaybackFrame < 0 || playbackFrameIndex != previousPlaybackFrame + 1;
        const uint64_t imuStartTimestamp =
            rebuildImuHistory ? 0ULL
                              : (playbackState.mode == Lvx2PlaybackMode::SlidingWindow
                                     ? windowEndTimestamp
                                     : windowStartTimestamp);
        const QVector<Playback::ImuSample> imuSamples =
            playbackState.source->readImuSamples(
                imuStartTimestamp,
                windowEndTimestamp + playbackImuQueryPaddingNs(playbackState.source.get()));
        appendPlaybackImuSamples(playbackState, imuSamples, rebuildImuHistory);
    }

    playbackState.frame = playbackFrameIndex;
    updateLvx2PlaybackUi();
}

QString LivoxViewerWindow::lvx2DeviceTypeToModel(uint8_t deviceType) const
{
    switch (deviceType) {
    case kLivoxLidarTypeHub: return "LiDAR Hub";
    case kLivoxLidarTypeMid40: return "Mid-40";
    case kLivoxLidarTypeTele: return "Tele-15";
    case kLivoxLidarTypeHorizon: return "Horizon";
    case kLivoxLidarTypeMid70: return "Mid-70";
    case kLivoxLidarTypeAvia: return "Avia";
    case kLivoxLidarTypeMid360: return "Mid360";
    case kLivoxLidarTypeIndustrialHAP: return "Industrial HAP";
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

    if (playbackState.source && playbackState.source->kind() == Playback::SourceKind::Rosbag) {
        const Playback::SourceInfo sourceInfo = playbackState.source->sourceInfo();
        auto addInfoCard = [this, deviceListLayout](const QString& title,
                                                    const QVector<QPair<QString, QString>>& rows) {
            QFrame* card = new QFrame(lvx2DeviceListWidget);
            card->setObjectName("PlaybackDeviceCard");
            card->setFrameShape(QFrame::StyledPanel);
            card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            card->setStyleSheet("QFrame#PlaybackDeviceCard { border: 1px solid palette(mid); border-radius: 6px; background: palette(base); }");

            QVBoxLayout* cardLayout = new QVBoxLayout(card);
            cardLayout->setContentsMargins(8, 6, 8, 6);
            cardLayout->setSpacing(4);

            QLabel* titleLabel = new QLabel(title, card);
            QFont titleFont = titleLabel->font();
            titleFont.setBold(true);
            titleLabel->setFont(titleFont);
            titleLabel->setWordWrap(true);
            cardLayout->addWidget(titleLabel);

            for (const QPair<QString, QString>& row : rows) {
                QLabel* rowLabel = new QLabel(QStringLiteral("%1: %2").arg(row.first, row.second), card);
                rowLabel->setWordWrap(true);
                cardLayout->addWidget(rowLabel);
            }
            deviceListLayout->addWidget(card);
        };

        const uint64_t durationNs = sourceInfo.endTimestampNs > sourceInfo.startTimestampNs
            ? uint64_t(sourceInfo.endTimestampNs - sourceInfo.startTimestampNs)
            : 0ULL;
        const QString timeRange = sourceInfo.startTimestampNs > 0 && sourceInfo.endTimestampNs > 0
            ? QStringLiteral("%1 - %2 (%3)")
                  .arg(QString::number(sourceInfo.startTimestampNs),
                       QString::number(sourceInfo.endTimestampNs),
                       formatPlaybackTimeNs(durationNs, std::max<uint64_t>(1, durationNs)))
            : QStringLiteral("-");
        addInfoCard(QStringLiteral("ROSbag 文件"),
                    {
                        {QStringLiteral("格式"), sourceInfo.format.isEmpty() ? QStringLiteral("-") : sourceInfo.format},
                        {QStringLiteral("帧数"), QString::number(sourceInfo.frameCount)},
                        {QStringLiteral("点数"), QString::number(sourceInfo.pointCount)},
                        {QStringLiteral("IMU 样本数"), QString::number(sourceInfo.imuSampleCount)},
                        {QStringLiteral("时间范围"), timeRange}
                    });
        for (const Playback::TopicInfo& topic : sourceInfo.lidarTopics) {
            addInfoCard(QStringLiteral("LiDAR Topic"),
                        {
                            {QStringLiteral("Topic"), topic.topic},
                            {QStringLiteral("类型"), topic.type},
                            {QStringLiteral("消息数"), QString::number(topic.messageCount)},
                            {QStringLiteral("点数"), QString::number(topic.pointCount)}
                        });
        }
        for (const Playback::TopicInfo& topic : sourceInfo.imuTopics) {
            addInfoCard(QStringLiteral("IMU Topic"),
                        {
                            {QStringLiteral("Topic"), topic.topic},
                            {QStringLiteral("类型"), topic.type},
                            {QStringLiteral("消息数"), QString::number(topic.messageCount)}
                        });
        }
        deviceListLayout->addStretch();
        return;
    }

    for (int row = 0; row < playbackState.devices.size(); ++row) {
        const auto& info = playbackState.devices[row];
        const QString modelName =
            info.modelDisplay.isEmpty() ? lvx2DeviceTypeToModel(info.deviceType) : info.modelDisplay;
        const bool lvxSource = playbackState.source && playbackState.source->kind() == Playback::SourceKind::Lvx;
        const QString deviceIdText = lvxSource
            ? QStringLiteral("设备索引: %1").arg(info.lidarId)
            : QStringLiteral("IP: %1").arg(PushMsgParser::lidarIdToIpString(info.lidarId));
        const QString deviceTip = QString("型号: %1\nSN: %2\n%3").arg(modelName, info.lidarSn, deviceIdText);
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
        QLabel* ipLabel = new QLabel(deviceIdText, card);
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
    updatePlaybackBarGeometry();
    if (!playbackState.active && !playbackState.loading) {
        return;
    }

    if (playbackState.playPauseButton) {
        const QString iconPath = playbackState.playing
            ? QStringLiteral(":/icons/playback_pause.svg")
            : QStringLiteral(":/icons/playback_play.svg");
        if (playbackState.playPauseButton->property(ThemeIconUtils::kSvgIconPathProperty).toString() != iconPath) {
            ThemeIconUtils::setThemedSvgIcon(playbackState.playPauseButton, iconPath);
        }
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
        const int displayFrameIndex = displayFrameNumberForRawEndIndex(playbackState.source.get(), rawEndIndex) - 1;
        playbackState.progressSlider->setRange(0, std::max(0, displayFrameCount - 1));
        playbackState.progressSlider->setEnabled(playbackState.active && !playbackState.loading && playbackState.frameCount > 0);
        playbackState.progressSlider->setValue(std::clamp(displayFrameIndex, 0, std::max(0, displayFrameCount - 1)));
        playbackState.updatingSlider = false;
    }
    if (playbackState.speedCombo) {
        QSignalBlocker blocker(playbackState.speedCombo);
        playbackState.speedCombo->setEnabled(playbackState.active && !playbackState.loading);
        playbackState.speedCombo->setCurrentText(QStringLiteral("x%1").arg(playbackState.speed, 0, 'f', 1));
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
            const int currentFrame = displayFrameNumberForRawEndIndex(playbackState.source.get(), rawEndIndex);
            const int totalFrames = displayPlaybackFrameCount(playbackState.source.get());
            const uint64_t elapsedNs = playbackElapsedNsForRawEndIndex(playbackState.source.get(), rawEndIndex);
            const uint64_t durationNs = playbackDurationNs(playbackState.source.get());
            playbackInfo = QString("%1    时间 %2 / %3    帧 %4 / %5")
                               .arg(path)
                               .arg(formatPlaybackTimeNs(elapsedNs, durationNs))
                               .arg(formatPlaybackTimeNs(durationNs, durationNs))
                               .arg(currentFrame)
                               .arg(totalFrames);
        }
        setElidedPlaybackLabelText(playbackState.label, playbackInfo);
    }
    updatePlaybackBarGeometry();
}

void LivoxViewerWindow::updatePlaybackBarGeometry()
{
    if (!playbackState.bar || !visualizationWorkspace) {
        return;
    }

    if (!playbackState.bar->isVisible() || !pointCloudView || !isOfflinePointCloudTab(activeVisualizationTabId)) {
        return;
    }

    if (playbackState.bar->parentWidget() != visualizationWorkspace) {
        playbackState.bar->setParent(visualizationWorkspace);
    }

    const QPoint viewTopLeft = pointCloudView->mapTo(visualizationWorkspace, QPoint(0, 0));
    const int margin = 10;
    const int barWidth = std::max(1, pointCloudView->width() - margin * 2);
    const int barHeight = playbackState.bar->sizeHint().height();
    playbackState.bar->setGeometry(viewTopLeft.x() + margin,
                                   viewTopLeft.y() + margin,
                                   barWidth,
                                   barHeight);
    playbackState.bar->raise();
}

void LivoxViewerWindow::updateSlamControlBarUi()
{
    if (!slamControlBar) {
        return;
    }

    const bool visible = isSlamPointCloudTab(activeVisualizationTabId) && slamPointCloudView != nullptr;
    slamControlBar->setVisible(visible);
    if (!visible) {
        return;
    }

    const bool active = slamWorkerActive.load();
    const bool hasWorker = slamWorker.joinable();
    const bool paused = active && slamWorkerPaused.load();
    if (slamStartButton) {
        if (active && !paused) {
            slamStartButton->setText(QStringLiteral("暂停"));
            slamStartButton->setToolTip(QStringLiteral("暂停 SLAM worker"));
        } else if (paused) {
            slamStartButton->setText(QStringLiteral("继续"));
            slamStartButton->setToolTip(QStringLiteral("继续 SLAM worker"));
        } else {
            slamStartButton->setText(QStringLiteral("开始"));
            slamStartButton->setToolTip(QStringLiteral("开始 SLAM worker"));
        }
        slamStartButton->setEnabled(true);
    }
    if (slamStopButton) {
        slamStopButton->setEnabled(active || hasWorker);
    }
    if (slamClearButton) {
        slamClearButton->setEnabled(!active);
        slamClearButton->setToolTip(active
                                        ? QStringLiteral("运行中不能清空显示")
                                        : QStringLiteral("清空 SLAM 显示"));
    }
    if (slamExportTrajectoryButton) {
        slamExportTrajectoryButton->setEnabled(true);
    }
    if (slamExportMapButton) {
        slamExportMapButton->setEnabled(!slamMapExportActive.load());
    }
    if (slamProgressBar) {
        if (slamProgressIndeterminate && active) {
            slamProgressBar->setRange(0, 0);
        } else {
            const int maximum = std::max(1, slamProgressMaximum);
            slamProgressBar->setRange(0, maximum);
            slamProgressBar->setValue(std::clamp(slamProgressValue, 0, maximum));
        }
        slamProgressBar->setEnabled(active || hasWorker || slamProgressValue > 0);
        slamProgressBar->setToolTip(slamProgressMaximum > 0
                                        ? QStringLiteral("%1 / %2").arg(slamProgressValue).arg(slamProgressMaximum)
                                        : QStringLiteral("SLAM 进度"));
    }
    if (slamReplayModeCombo) {
        QSignalBlocker blocker(slamReplayModeCombo);
        slamReplayModeCombo->setEnabled(isOfflineSlamMode() && !active);
        slamReplayModeCombo->setCurrentIndex(slamReplayMode == SlamReplayMode::Fast ? 1 : 0);
        slamReplayModeCombo->setToolTip(isOfflineSlamMode()
                                            ? QStringLiteral("离线 SLAM 回放速度")
                                            : QStringLiteral("在线 SLAM 不使用回放速度"));
    }
    if (slamControlTemplateCombo) {
        QSignalBlocker blocker(slamControlTemplateCombo);
        const int index = slamControlTemplateCombo->findData(static_cast<int>(slamRuntimeConfig.lidarTemplate));
        if (index >= 0) {
            slamControlTemplateCombo->setCurrentIndex(index);
        }
        slamControlTemplateCombo->setEnabled(!active);
        slamControlTemplateCombo->setToolTip(active
                                                 ? QStringLiteral("SLAM 运行中不能切换 LiDAR 模板")
                                                 : QStringLiteral("LiDAR 模板"));
    }
    const QString modeText = isOfflineSlamMode() ? QStringLiteral("离线SLAM") : QStringLiteral("在线SLAM");
    const QString stateText = active
        ? (paused ? QStringLiteral("已暂停") : QStringLiteral("运行中"))
        : (hasWorker ? QStringLiteral("已结束") : QStringLiteral("未运行"));
    const QString sourceText = isOfflineSlamMode()
        ? (!slamProgressSourceText.isEmpty()
               ? slamProgressSourceText
               : (slamOfflineSourcePath.isEmpty() ? QString() : QDir::toNativeSeparators(slamOfflineSourcePath)))
        : QString();
    const QString sourceLine = sourceText.isEmpty()
        ? QStringLiteral("%1 | %2").arg(modeText, stateText)
        : QStringLiteral("%1 | %2 | %3").arg(modeText, stateText, sourceText);
    if (slamSourceLabel) {
        slamSourceLabel->setToolTip(sourceLine);
        slamSourceLabel->setText(slamSourceLabel->fontMetrics().elidedText(sourceLine,
                                                                           Qt::ElideMiddle,
                                                                           slamSourceLabel->width()));
    }
    if (slamTimeLabel) {
        QString timeText = slamProgressTimeText;
        if (timeText.isEmpty()) {
            timeText = isOfflineSlamMode() ? QStringLiteral("时间 - / -") : QStringLiteral("输入 FPS: 0.0");
        }
        slamTimeLabel->setText(timeText);
        slamTimeLabel->setToolTip(timeText);
    }
    if (slamFrameLabel) {
        QString frameText = slamProgressFrameText;
        if (frameText.isEmpty()) {
            frameText = isOfflineSlamMode() ? QStringLiteral("帧 0 / 0") : QStringLiteral("已处理帧: 0");
        }
        slamFrameLabel->setText(frameText);
        slamFrameLabel->setToolTip(frameText);
    }
    updateSlamControlBarGeometry();
}

void LivoxViewerWindow::updateSlamControlBarGeometry()
{
    if (!slamControlBar || !visualizationWorkspace || !slamPointCloudView) {
        return;
    }

    if (!slamControlBar->isVisible() || !isSlamPointCloudTab(activeVisualizationTabId)) {
        return;
    }

    if (slamControlBar->parentWidget() != visualizationWorkspace) {
        slamControlBar->setParent(visualizationWorkspace);
    }

    const QPoint viewTopLeft = slamPointCloudView->mapTo(visualizationWorkspace, QPoint(0, 0));
    const int margin = 10;
    const int barWidth = std::max(1, slamPointCloudView->width() - margin * 2);
    const int barHeight = slamControlBar->sizeHint().height();
    slamControlBar->setGeometry(viewTopLeft.x() + margin,
                                viewTopLeft.y() + margin,
                                barWidth,
                                barHeight);
    slamControlBar->raise();
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
        updatePlaybackBarGeometry();
    } else if ((watched == visualizationWorkspace || qobject_cast<PointCloudView*>(watched)) &&
               (event->type() == QEvent::Resize ||
                event->type() == QEvent::Move ||
                event->type() == QEvent::Show)) {
        QMetaObject::invokeMethod(this, [this]() {
            updatePlaybackBarGeometry();
            updateSlamControlBarGeometry();
        }, Qt::QueuedConnection);
    }

    return QMainWindow::eventFilter(watched, event);
}

void LivoxViewerWindow::setLvx2PlaybackPlaying(bool playing)
{
    playbackState.playing = playing && playbackState.active && !playbackState.loading && playbackState.frameCount > 0;

    if (playbackState.timer) {
        if (playbackState.playing) {
            const double baseStepMs =
                (playbackState.mode == Lvx2PlaybackMode::SlidingWindow)
                    ? double(sourceNominalFrameDurationNs(playbackState.source.get())) / 1000000.0
                    : static_cast<double>(frameIntervalMs);
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
    playbackSeekToDisplayFrame(value);
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
    if (RosbagPlayback::isSupportedFile(filePath)) {
        loadRosbagPlaybackFile(filePath);
        return;
    }
    if (filePath.endsWith(QStringLiteral(".lvx"), Qt::CaseInsensitive)) {
        loadLvxPlaybackFile(filePath);
        return;
    }
    loadLvx2PlaybackFile(filePath);
}

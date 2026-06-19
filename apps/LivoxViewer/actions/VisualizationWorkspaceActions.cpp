#include "LivoxViewerWindow.h"

#include <QDir>
#include <QFileInfo>

#include <algorithm>

namespace {

void clearPlaybackUiPointers(PlaybackControllerState& state)
{
    state.timer = nullptr;
    state.bar = nullptr;
    state.playPauseButton = nullptr;
    state.firstFrameButton = nullptr;
    state.prevFrameButton = nullptr;
    state.nextFrameButton = nullptr;
    state.lastFrameButton = nullptr;
    state.progressSlider = nullptr;
    state.label = nullptr;
    state.speedCombo = nullptr;
    state.modeCombo = nullptr;
}

void restorePlaybackUiPointers(PlaybackControllerState& state, const PlaybackControllerState& controls)
{
    state.timer = controls.timer;
    state.bar = controls.bar;
    state.playPauseButton = controls.playPauseButton;
    state.firstFrameButton = controls.firstFrameButton;
    state.prevFrameButton = controls.prevFrameButton;
    state.nextFrameButton = controls.nextFrameButton;
    state.lastFrameButton = controls.lastFrameButton;
    state.progressSlider = controls.progressSlider;
    state.label = controls.label;
    state.speedCombo = controls.speedCombo;
    state.modeCombo = controls.modeCombo;
}

} // namespace

void LivoxViewerWindow::copyPlaybackSession(PlaybackControllerState& dst, const PlaybackControllerState& src) const
{
    dst.source = src.source;
    dst.devices = src.devices;
    dst.deviceVisible = src.deviceVisible;
    dst.path = src.path;
    dst.active = src.active;
    dst.loading = src.loading;
    dst.playing = src.playing;
    dst.fileInfoDockVisible = false;
    dst.frame = src.frame;
    dst.frameCount = src.frameCount;
    dst.loadToken = src.loadToken;
    dst.updatingSlider = false;
    dst.progressSliderDragging = false;
    dst.speed = src.speed;
    dst.mode = src.mode;
    dst.slidingWindowStart = src.slidingWindowStart;
    dst.slidingWindowEnd = src.slidingWindowEnd;
    dst.slidingWindowPoints = src.slidingWindowPoints;
    dst.slidingWindowSegmentPointCounts = src.slidingWindowSegmentPointCounts;
    dst.slidingWindowSegmentTimestamps = src.slidingWindowSegmentTimestamps;
    dst.slidingWindowSegmentLineNumbers = src.slidingWindowSegmentLineNumbers;
    dst.slidingWindowTimestamp = src.slidingWindowTimestamp;
    dst.imuHandleByLidarId = src.imuHandleByLidarId;
    dst.nextPlaybackImuHandle = src.nextPlaybackImuHandle;
    clearPlaybackUiPointers(dst);
}

void LivoxViewerWindow::saveBoundPlaybackState()
{
    if (boundPlaybackTabId >= 0) {
        copyPlaybackSession(playbackStatesByTab[boundPlaybackTabId], playbackState);
    }
}

void LivoxViewerWindow::bindPlaybackStateToTab(int tabId)
{
    const PlaybackControllerState controls = playbackState;
    copyPlaybackSession(playbackState, playbackStatesByTab[tabId]);
    restorePlaybackUiPointers(playbackState, controls);
    boundPlaybackTabId = tabId;
}

void LivoxViewerWindow::clearPlaybackStateMirror()
{
    const PlaybackControllerState controls = playbackState;
    playbackState = PlaybackControllerState();
    restorePlaybackUiPointers(playbackState, controls);
    boundPlaybackTabId = -1;
}

void LivoxViewerWindow::rebindPlaybackControls()
{
}

bool LivoxViewerWindow::isOfflinePointCloudTab(int tabId) const
{
    return visualizationWorkspace &&
           tabId >= 0 &&
           playbackStatesByTab.contains(tabId) &&
           visualizationWorkspace->tabKind(tabId) == VisualizationWorkspace::TabKind::OfflinePointCloud;
}

bool LivoxViewerWindow::isPointCloudTab(int tabId) const
{
    return tabId >= 0 && pointCloudViewsByTab.contains(tabId);
}

PlaybackControllerState* LivoxViewerWindow::playbackStateForTab(int tabId)
{
    if (boundPlaybackTabId == tabId) {
        return &playbackState;
    }
    auto it = playbackStatesByTab.find(tabId);
    return it == playbackStatesByTab.end() ? nullptr : &it.value();
}

const PlaybackControllerState* LivoxViewerWindow::playbackStateForTab(int tabId) const
{
    if (boundPlaybackTabId == tabId) {
        return &playbackState;
    }
    auto it = playbackStatesByTab.constFind(tabId);
    return it == playbackStatesByTab.constEnd() ? nullptr : &it.value();
}

PointCloudView* LivoxViewerWindow::pointCloudViewForTab(int tabId) const
{
    auto it = pointCloudViewsByTab.constFind(tabId);
    return it == pointCloudViewsByTab.constEnd() ? nullptr : it.value();
}

PointCloudView* LivoxViewerWindow::currentPointCloudView() const
{
    return pointCloudViewForTab(activeVisualizationTabId);
}

QVector<PointCloudView*> LivoxViewerWindow::pointCloudViews() const
{
    QVector<PointCloudView*> views;
    for (PointCloudView* view : pointCloudViewsByTab) {
        if (view && !views.contains(view)) {
            views.append(view);
        }
    }
    return views;
}

void LivoxViewerWindow::forEachPointCloudView(const std::function<void(PointCloudView*)>& callback) const
{
    for (PointCloudView* view : pointCloudViews()) {
        callback(view);
    }
}

PlaybackControllerState* LivoxViewerWindow::imuPlaybackState()
{
    return playbackStateForTab(imuSourceVisualizationTabId);
}

const PlaybackControllerState* LivoxViewerWindow::imuPlaybackState() const
{
    return playbackStateForTab(imuSourceVisualizationTabId);
}

uint32_t LivoxViewerWindow::playbackImuHandleForLidarId(PlaybackControllerState& state, uint32_t lidarId)
{
    auto it = state.imuHandleByLidarId.find(lidarId);
    if (it != state.imuHandleByLidarId.end()) {
        return it.value();
    }
    const uint32_t handle = nextPlaybackImuHandle++;
    state.imuHandleByLidarId.insert(lidarId, handle);
    return handle;
}

int LivoxViewerWindow::createOfflinePointCloudTab(const QString& filePath)
{
    PointCloudView* view = new PointCloudView(visualizationWorkspace);
    view->setMinimumSize(200, 200);
    view->setPointSize(pointSizePx);
    view->setMeasurementModeEnabled(measurementModeActive);
    view->setSelectionModeEnabled(selectionRealtimeEnabled);
    connect(view, &PointCloudView::lvx2FileDropped, this, &LivoxViewerWindow::onLvx2PlaybackFileDropped);
    connect(view, &PointCloudView::selectionPointsReady, this, &LivoxViewerWindow::onSelectionPointsReady);
    connect(view, &PointCloudView::crossSectionChanged, this, [this](int clippedPointCount, int sourcePointCount) {
        if (crossSectionModeActive && statusLabelBar) {
            statusLabelBar->setText(QString("Cross Section: %1 / %2 点").arg(clippedPointCount).arg(sourcePointCount));
        }
    });

    int tabNumber = 0;
    if (!reusableOfflinePointCloudTabNumbers.isEmpty()) {
        tabNumber = *std::min_element(reusableOfflinePointCloudTabNumbers.constBegin(),
                                      reusableOfflinePointCloudTabNumbers.constEnd());
        reusableOfflinePointCloudTabNumbers.remove(tabNumber);
    } else {
        tabNumber = nextOfflinePointCloudTabNumber++;
    }

    const QString fileName = QFileInfo(filePath).fileName();
    const int tabId = visualizationWorkspace->addTab(
        VisualizationWorkspace::TabKind::OfflinePointCloud,
        fileName.isEmpty() ? QStringLiteral("离线点云%1").arg(tabNumber) : fileName,
        view,
        true);
    visualizationWorkspace->setTabToolTip(tabId, QDir::toNativeSeparators(filePath));
    pointCloudViewsByTab.insert(tabId, view);
    offlinePointCloudTabNumbersByTab.insert(tabId, tabNumber);
    applyPointCloudBackground();

    PlaybackControllerState state;
    state.path = filePath;
    playbackStatesByTab.insert(tabId, state);
    visualizationWorkspace->activateTab(tabId);
    return tabId;
}

void LivoxViewerWindow::closeVisualizationTab(int tabId)
{
    if (tabId == realtimeVisualizationTabId) {
        return;
    }

    if (isOfflinePointCloudTab(tabId)) {
        const int tabNumber = offlinePointCloudTabNumbersByTab.value(tabId, 0);
        if (tabNumber > 0) {
            reusableOfflinePointCloudTabNumbers.insert(tabNumber);
        }
        offlinePointCloudTabNumbersByTab.remove(tabId);

        PlaybackControllerState* state = playbackStateForTab(tabId);
        if (state) {
            if (boundPlaybackTabId == tabId) {
                setLvx2PlaybackPlaying(false);
            }
            clearPlaybackImuSamples(*state);
            state->loadToken++;
        }
        playbackStatesByTab.remove(tabId);
        pointCloudViewsByTab.remove(tabId);
        if (imuSourceVisualizationTabId == tabId) {
            imuSourceVisualizationTabId = -1;
        }
        if (boundPlaybackTabId == tabId) {
            clearPlaybackStateMirror();
        }
    }

    visualizationWorkspace->removeTab(tabId);
}

void LivoxViewerWindow::onVisualizationFocusedTabChanged(int tabId)
{
    if (boundPlaybackTabId >= 0 && boundPlaybackTabId != tabId) {
        saveBoundPlaybackState();
    }

    activeVisualizationTabId = tabId;
    pointCloudView = isPointCloudTab(tabId) ? currentPointCloudView() : nullptr;

    if (isOfflinePointCloudTab(tabId)) {
        bindPlaybackStateToTab(tabId);
        imuSourceVisualizationTabId = tabId;
        if (playbackState.playing) {
            setLvx2PlaybackPlaying(true);
        } else if (playbackState.timer) {
            playbackState.timer->stop();
        }
    } else if (isPointCloudTab(tabId)) {
        clearPlaybackStateMirror();
        if (playbackState.timer) {
            playbackState.timer->stop();
        }
    } else {
        clearPlaybackStateMirror();
        if (playbackState.timer) {
            playbackState.timer->stop();
        }
        activeVisualizationTabId = -1;
    }

    updatePointCloudLegend();
    syncPointCloudStlModelAction();
    syncPointCloudToolActions();
    rebuildLvx2DeviceTab();
    updateLvx2PlaybackUi();

    if (isOfflinePointCloudTab(tabId) && playbackState.active && playbackState.frame < 0) {
        showLvx2PlaybackFrame(0);
    }
}

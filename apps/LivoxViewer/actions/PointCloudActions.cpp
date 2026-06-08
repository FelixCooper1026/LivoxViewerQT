#include "LivoxViewerWindow.h"

#include <QIcon>
#include <QSignalBlocker>

#include <algorithm>

void LivoxViewerWindow::onFrameIntervalChanged(int ms)
{
    if (ms < 50) ms = 50;
    const uint64_t previousFrameIntervalMs = frameIntervalMs;
    const int rawEndIndex = playbackRawEndIndexForFrame(playbackState.frame,
                                                        playbackState.mode,
                                                        previousFrameIntervalMs);
    frameIntervalMs = static_cast<uint64_t>(ms);
    if (captureState.pointCloudTask.status == CaptureTaskStatus::Running &&
        (captureState.pcdSaveActive || captureState.lasSaveActive)) {
        captureState.pointCloudTask.integrationMs = ms;
        captureState.pointCloudSaveIntervalNs = frameIntervalMs * 1000000ULL;
    }
    logMessage(QString("点云积分时间已设置为 %1 ms").arg(ms));
    if (playbackState.active) {
        playbackState.slidingWindowStart = -1;
        playbackState.slidingWindowEnd = -1;
        playbackState.slidingWindowPoints.clear();
        playbackState.slidingWindowSegmentPointCounts.clear();
        playbackState.slidingWindowTimestamp = 0;
        const int sourceFrameCount = playbackState.source ? playbackState.source->frameCount() : 0;
        const int rawFramesPerStep = std::max(1, int((frameIntervalMs + 49ULL) / 50ULL));
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
}

void LivoxViewerWindow::onPointSizeChanged(int px)
{
    pointSizePx = static_cast<float>(px);
    if (pointCloudView) pointCloudView->setPointSize(pointSizePx);
}

void LivoxViewerWindow::onColorModeClicked(int index)
{
    colorMode = index;
    if (colorModeCombo) {
        colorModeCombo->setEnabled(!planarProjectionEnabled);
    }
    updatePointCloudLegend();
    if (playbackState.active && playbackState.frame >= 0) {
        showLvx2PlaybackFrame(playbackState.frame);
    }
}

int LivoxViewerWindow::effectiveColorMode() const
{
    return planarProjectionEnabled ? ColorByPlanarProjection : colorMode;
}

void LivoxViewerWindow::updatePointCloudLegend()
{
    if (!pointCloudView) {
        return;
    }

    const int mode = effectiveColorMode();
    if (mode == ColorByReflectivity) {
        pointCloudView->setLegend(ColorByReflectivity, 0.0f, 255.0f, true);
    } else if (mode == ColorByDistance) {
        pointCloudView->setLegend(ColorByDistance, distanceLegendMin, distanceLegendMax, true);
    } else if (mode == ColorByElevation) {
        pointCloudView->setLegend(ColorByElevation, elevationLegendMin, elevationLegendMax, true);
    } else if (mode == ColorSolid) {
        pointCloudView->setLegend(ColorSolid, 0.0f, 1.0f, false);
    } else if (mode == ColorByLine) {
        QVector<int> lineNumbers;
        lineNumbers.reserve(lineColors.size());
        for (int i = 0; i < lineColors.size(); ++i) {
            lineNumbers.append(i);
        }
        pointCloudView->setLegend(ColorByLine, 0.0f, 1.0f, true, lineColors, lineNumbers);
    } else if (mode == ColorByPlanarProjection) {
        pointCloudView->setLegend(ColorByPlanarProjection, 0.0f, 1.0f, true);
    }
}

void LivoxViewerWindow::onProjectionDepthChanged(double meters)
{
    if (meters < 0.0) meters = 0.0;
    projectionDepthMeters = static_cast<float>(meters);
}

void LivoxViewerWindow::onProjectionDepthToggled(bool enabled)
{
    projectionDepthEnabled = enabled;
    if (projectionDepthSpin) {
        projectionDepthSpin->setEnabled(enabled);
    }
    logMessage(enabled ? "深度投影已启用" : "深度投影已关闭");
}

void LivoxViewerWindow::onPlanarProjectionToggled(bool enabled)
{
    planarProjectionEnabled = enabled;
    if (enabled) {
        logMessage("平面投影模式已启用");
        statusLabelBar->setText("平面投影模式已启用");
        if (pointCloudView) {
            pointCloudView->resetView();
            pointCloudView->setTopDownView();
        }
    } else {
        logMessage("平面投影模式已关闭");
        statusLabelBar->setText("平面投影模式已关闭");
        if (pointCloudView) {
            pointCloudView->resetView();
        }
    }
    if (planarRadiusSpin) {
        planarRadiusSpin->setEnabled(enabled);
    }
    if (colorModeCombo) {
        colorModeCombo->setEnabled(!planarProjectionEnabled);
    }
    updatePointCloudLegend();
}

void LivoxViewerWindow::onPlanarProjectionRadiusChanged(double radius)
{
    if (radius < 1.0) radius = 1.0;
    planarProjectionRadius = static_cast<float>(radius);
    if (planarProjectionEnabled) {
        updatePointCloudLegend();
    }
    logMessage(QString("平面投影半径已设置为 %1 m").arg(radius));
}

void LivoxViewerWindow::onPointCloudVisualizationToggled(bool enabled)
{
    pointCloudVisualizationEnabled = enabled;
    syncPointCloudVisualizationAction();
    if (enabled) {
        logMessage("实时点云显示已恢复");
    } else {
        logMessage("实时点云显示已冻结");
    }
}

void LivoxViewerWindow::syncPointCloudVisualizationAction()
{
    if (!actionPointCloudVisualization) {
        return;
    }

    actionPointCloudVisualization->setText(pointCloudVisualizationEnabled ? "冻结实时点云" : "恢复实时点云");
    actionPointCloudVisualization->setToolTip(pointCloudVisualizationEnabled
        ? "冻结实时点云显示，不控制离线播放"
        : "恢复实时点云显示，不控制离线播放");
    actionPointCloudVisualization->setIcon(QIcon(pointCloudVisualizationEnabled
        ? ":/icons/point_cloud_live.svg"
        : ":/icons/point_cloud_frozen.svg"));
    actionPointCloudVisualization->setEnabled(!measurementModeActive);
    QSignalBlocker blocker(actionPointCloudVisualization);
    actionPointCloudVisualization->setChecked(pointCloudVisualizationEnabled);
}

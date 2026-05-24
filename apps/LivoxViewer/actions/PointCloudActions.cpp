#include "LivoxViewerWindow.h"

#include <QColorDialog>
#include <algorithm>

void LivoxViewerWindow::onFrameIntervalChanged(int ms)
{
    if (ms < 50) ms = 50;
    frameIntervalMs = static_cast<uint64_t>(ms);
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
        const int targetFrame = std::clamp(playbackState.frame, 0, playbackState.frameCount - 1);
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

void LivoxViewerWindow::onColorModeChanged(int index)
{
    colorMode = index;
    if (solidColorRow) {
        solidColorRow->setEnabled(colorMode == ColorSolid);
    }
    if (pointCloudView) {
        if (colorMode == ColorByReflectivity) {
            pointCloudView->setLegend(ColorByReflectivity, 0.0f, 255.0f, true);
        } else if (colorMode == ColorByDistance) {
            pointCloudView->setLegend(ColorByDistance, 0.0f, 1.0f, true);
        } else if (colorMode == ColorByElevation) {
            pointCloudView->setLegend(ColorByElevation, -1.0f, 1.0f, true);
        } else if (colorMode == ColorSolid) {
            pointCloudView->setLegend(ColorSolid, 0.0f, 1.0f, false);
        } else if (colorMode == ColorByPlanarProjection) {
            pointCloudView->setLegend(ColorByPlanarProjection, 0.0f, 1.0f, true);
        }
    }
}

void LivoxViewerWindow::onSolidColorClicked()
{
    QColor c = QColorDialog::getColor(solidColor, this, "选择点云颜色");
    if (!c.isValid()) return;
    solidColor = c;
    if (solidColorPreview) {
        solidColorPreview->setStyleSheet(QString("background-color: %1;").arg(solidColor.name()));
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
}

void LivoxViewerWindow::onPlanarProjectionRadiusChanged(double radius)
{
    if (radius < 1.0) radius = 1.0;
    planarProjectionRadius = static_cast<float>(radius);
    logMessage(QString("平面投影半径已设置为 %1 m").arg(radius));
}

void LivoxViewerWindow::onPointCloudVisualizationToggled(bool enabled)
{
    pointCloudVisualizationEnabled = enabled;
    if (enabled) {
        logMessage("点云可视化已开启");
    } else {
        logMessage("点云可视化已暂停");
    }
}

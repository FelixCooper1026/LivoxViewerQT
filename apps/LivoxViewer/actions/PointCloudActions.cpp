#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"

#include "PointCloud/PointCloudColorizer.h"

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
    forEachPointCloudView([this](PointCloudView* view) {
        view->setPointSize(pointSizePx);
    });
}

void LivoxViewerWindow::onColorModeClicked(int index)
{
    colorMode = index;
    recolorPointCloudViews();
}

int LivoxViewerWindow::effectiveColorMode() const
{
    return colorMode;
}

void LivoxViewerWindow::updatePointCloudLegend()
{
    const QVector<PointCloudView*> views = pointCloudViews();
    if (views.isEmpty()) {
        return;
    }

    const int mode = effectiveColorMode();
    if (mode == ColorByReflectivity) {
        for (PointCloudView* view : views) {
            view->setLegend(ColorByReflectivity, 0.0f, 255.0f, true);
        }
    } else if (mode == ColorByDistance) {
        for (PointCloudView* view : views) {
            view->setLegend(ColorByDistance, distanceLegendMin, distanceLegendMax, true);
        }
    } else if (mode == ColorByElevation) {
        for (PointCloudView* view : views) {
            view->setLegend(ColorByElevation, elevationLegendMin, elevationLegendMax, true);
        }
    } else if (mode == ColorSolid) {
        for (PointCloudView* view : views) {
            view->setLegend(ColorSolid, 0.0f, 1.0f, false);
        }
    } else if (mode == ColorByLine) {
        QVector<int> lineNumbers;
        lineNumbers.reserve(lineColors.size());
        for (int i = 0; i < lineColors.size(); ++i) {
            lineNumbers.append(i);
        }
        for (PointCloudView* view : views) {
            view->setLegend(ColorByLine, 0.0f, 1.0f, true, lineColors, lineNumbers);
        }
    }
}

void LivoxViewerWindow::recolorPointCloudViews()
{
    PointCloudColorizer::Config colorConfig;
    colorConfig.mode = effectiveColorMode();
    colorConfig.solidColor = solidColor;
    colorConfig.lineColors = lineColors;
    colorConfig.distanceColorMin = distanceLegendMin;
    colorConfig.distanceColorMax = distanceLegendMax;
    colorConfig.elevationColorMin = elevationLegendMin;
    colorConfig.elevationColorMax = elevationLegendMax;

    forEachPointCloudView([&colorConfig](PointCloudView* view) {
        view->recolorCurrentPointCloud([view, &colorConfig](QVector<PointCloudPoint>& points) {
            const PointCloudPipelineLegend legend = PointCloudColorizer::apply(points, colorConfig);
            view->setLegend(legend.mode,
                            legend.minValue,
                            legend.maxValue,
                            legend.visible,
                            legend.lineColors,
                            legend.lineNumbers);
        });
    });
}

void LivoxViewerWindow::onProjectionDepthChanged(double meters)
{
    if (meters < 0.0) meters = 0.0;
    projectionDepthMeters = static_cast<float>(meters);
    if (playbackState.active && playbackState.frame >= 0) {
        showLvx2PlaybackFrame(playbackState.frame);
    }
}

void LivoxViewerWindow::onProjectionDepthToggled(bool enabled)
{
    if (enabled && planarProjectionCheck && planarProjectionCheck->defaultAction()) {
        planarProjectionCheck->defaultAction()->setChecked(false);
    }
    projectionDepthEnabled = enabled;
    if (projectionDepthSpin) {
        projectionDepthSpin->setEnabled(enabled);
    }
    if (playbackState.active && playbackState.frame >= 0) {
        showLvx2PlaybackFrame(playbackState.frame);
    }
    logMessage(enabled ? "深度投影已启用" : "深度投影已关闭");
}

void LivoxViewerWindow::onPlanarProjectionToggled(bool enabled)
{
    if (enabled && projectionDepthCheck && projectionDepthCheck->defaultAction()) {
        projectionDepthCheck->defaultAction()->setChecked(false);
    }
    planarProjectionEnabled = enabled;
    if (enabled) {
        logMessage("平面投影模式已启用");
        statusLabelBar->setText("平面投影模式已启用");
        forEachPointCloudView([](PointCloudView* view) {
            view->resetView();
            view->setTopDownView();
        });
    } else {
        logMessage("平面投影模式已关闭");
        statusLabelBar->setText("平面投影模式已关闭");
        forEachPointCloudView([](PointCloudView* view) {
            view->resetView();
        });
    }
    if (planarRadiusSpin) {
        planarRadiusSpin->setEnabled(enabled);
    }
    if (playbackState.active && playbackState.frame >= 0) {
        showLvx2PlaybackFrame(playbackState.frame);
    }
    updatePointCloudLegend();
}

void LivoxViewerWindow::onPlanarProjectionRadiusChanged(double radius)
{
    if (radius < 1.0) radius = 1.0;
    planarProjectionRadius = static_cast<float>(radius);
    if (planarProjectionEnabled) {
        updatePointCloudLegend();
        if (playbackState.active && playbackState.frame >= 0) {
            showLvx2PlaybackFrame(playbackState.frame);
        }
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
    ThemeIconUtils::setThemedSvgIcon(actionPointCloudVisualization,
        pointCloudVisualizationEnabled
            ? QStringLiteral(":/icons/point_cloud_live.svg")
            : QStringLiteral(":/icons/point_cloud_frozen.svg"));
    actionPointCloudVisualization->setEnabled(!measurementModeActive);
    QSignalBlocker blocker(actionPointCloudVisualization);
    actionPointCloudVisualization->setChecked(pointCloudVisualizationEnabled);
}

void LivoxViewerWindow::syncPointCloudStlModelAction()
{
    if (!pointCloudStlModelAction) {
        return;
    }

    const bool checked = pointCloudView && pointCloudView->hasStlModel() && pointCloudView->isStlModelVisible();
    QSignalBlocker blocker(pointCloudStlModelAction);
    pointCloudStlModelAction->setChecked(checked);
    ThemeIconUtils::setThemedSvgIcon(pointCloudStlModelAction,
        checked ? QStringLiteral(":/icons/3d_model_on.svg") : QStringLiteral(":/icons/3d_model_off.svg"));
}

void LivoxViewerWindow::syncPointCloudToolActions()
{
    if (pointCloudMeasureAction) {
        QSignalBlocker blocker(pointCloudMeasureAction);
        pointCloudMeasureAction->setChecked(measurementModeActive);
    }
    if (pointCloudSelectionAction) {
        QSignalBlocker blocker(pointCloudSelectionAction);
        pointCloudSelectionAction->setChecked(selectionRealtimeEnabled);
    }
}

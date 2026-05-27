#include "LivoxViewerWindow.h"

#include "PointCloud/PointCloudColorizer.h"
#include "PointCloud/PointCloudDecoder.h"
#include "PointCloud/PointCloudFilter.h"

void LivoxViewerWindow::decodePointCloudPacket(uint32_t handle, const LivoxLidarEthernetPacket* packet)
{
    PointCloudDecoder::DecodeOptions options;
    options.depthProjectionEnabled = projectionDepthEnabled;
    options.depthMeters = projectionDepthMeters;
    options.planarProjectionEnabled = planarProjectionEnabled;
    options.planarRadius = planarProjectionRadius;

    PointCloudFrame frame;
    if (!PointCloudDecoder::decodeLivoxPacket(handle, packet, options, frame)) {
        return;
    }

    {
        QMutexLocker locker(&frameMutex);
        pendingFrames[handle].enqueue(frame);
        lastSeenTimestamp[handle] = frame.timestamp;
    }
}

void LivoxViewerWindow::presentPointCloudFrame(const PointCloudFrame& frame)
{
    QMetaObject::invokeMethod(this, [this, frame]() mutable {
        pointCloudView->updatePointCloud(std::move(frame));
        if (selectionRealtimeEnabled && pointCloudView && (attrTable || selectionTable)) {
            updateSelectionTableAndLog();
        }
    }, Qt::QueuedConnection);
}

void LivoxViewerWindow::applyPointCloudPipeline(PointCloudFrame& frame)
{
    PointCloudColorizer::Config colorConfig;
    colorConfig.mode = effectiveColorMode();
    colorConfig.solidColor = solidColor;
    colorConfig.distanceColorMin = distanceLegendMin;
    colorConfig.distanceColorMax = distanceLegendMax;
    colorConfig.elevationColorMin = elevationLegendMin;
    colorConfig.elevationColorMax = elevationLegendMax;
    colorConfig.planarProjectionRadius = planarProjectionRadius;

    const PointCloudPipelineLegend legend = PointCloudColorizer::apply(frame.points, colorConfig);
    if (pointCloudView) {
        pointCloudView->setLegend(legend.mode, legend.minValue, legend.maxValue, legend.visible);
    }

    PointCloudFilter::Config filterConfig;
    filterConfig.showNoisePoints = filterState.showNoisePoints;
    filterConfig.removeNoisePoints = filterState.removeNoisePoints;
    filterConfig.noiseTags = filterState.noiseFilterTags;
    frame.points = PointCloudFilter::apply(frame.points, filterConfig);
}

void LivoxViewerWindow::onRenderTick()
{
    if (playbackState.active) {
        {
            QMutexLocker locker(&frameMutex);
            pendingFrames.clear();
            lastSeenTimestamp.clear();
        }
        if (pointCloudView) {
            pointCloudView->update();
        }
        return;
    }

    if (!pointCloudVisualizationEnabled) {
        {
            QMutexLocker locker(&frameMutex);
            for (auto it = pendingFrames.begin(); it != pendingFrames.end(); ++it) {
                it.value().clear();
            }
        }
        if (pointCloudView) {
            pointCloudView->update();
        }
        return;
    }

    if (pointCloudView && pointCloudView->isMeasurementModeEnabled()) {
        {
            QMutexLocker locker(&frameMutex);
            for (auto it = pendingFrames.begin(); it != pendingFrames.end(); ++it) {
                it.value().clear();
            }
        }
        pointCloudView->update();
        return;
    }

    uint32_t targetHandle = 0;
    bool hasTarget = false;
    {
        QMutexLocker devLocker(&lidarDeviceMutex);
        if (hasCurrentLidarHandle) {
            targetHandle = currentLidarHandle;
            hasTarget = true;
        }
    }

    uint64_t now_ns = 0;
    {
        QMutexLocker locker(&frameMutex);
        if (hasTarget && lastSeenTimestamp.contains(targetHandle)) {
            now_ns = lastSeenTimestamp[targetHandle];
        } else if (!hasTarget) {
            for (auto it = lastSeenTimestamp.begin(); it != lastSeenTimestamp.end(); ++it) {
                if (it.value() > now_ns) now_ns = it.value();
            }
        }
    }
    if (now_ns == 0) return;

    uint64_t window_ns = frameIntervalMs * 1000000ULL;
    uint64_t window_begin = (now_ns > window_ns) ? (now_ns - window_ns) : 0ULL;

    PointCloudFrame merged;
    merged.timestamp = now_ns;
    merged.device_handle = 0;

    bool hasAnyPoint = false;
    {
        QMutexLocker locker(&frameMutex);
        for (auto it = pendingFrames.begin(); it != pendingFrames.end(); ++it) {
            if (hasTarget && it.key() != targetHandle) {
                continue;
            }

            QQueue<PointCloudFrame>& q = it.value();
            while (!q.isEmpty() && q.head().timestamp < window_begin) {
                q.dequeue();
            }
            for (int i = 0; i < q.size(); ++i) {
                const PointCloudFrame& f = q.at(i);
                if (f.timestamp >= window_begin && f.timestamp <= now_ns) {
                    merged.points += f.points;
                    hasAnyPoint = true;
                }
            }
        }
    }

    if (hasAnyPoint) {
        applyPointCloudPipeline(merged);
        handlePointCloudRecording(merged, now_ns);
        presentPointCloudFrame(merged);
    }

    if (selectionRealtimeEnabled && pointCloudView && (attrTable || selectionTable)) {
        updateSelectionTableAndLog();
    }
}

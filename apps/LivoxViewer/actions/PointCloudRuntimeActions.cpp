#include "LivoxViewerWindow.h"

#include "LidarModelUtils.h"
#include "PointCloudColorizer.h"
#include "PointCloudDecoder.h"
#include "PointCloudFilter.h"

#include <algorithm>

void LivoxViewerWindow::decodePointCloudPacket(uint32_t handle, uint8_t dev_type, const LivoxLidarEthernetPacket* packet)
{
    PointCloudDecoder::DecodeOptions options;
    options.depthProjectionEnabled = projectionDepthEnabled;
    options.depthMeters = projectionDepthMeters;
    options.planarProjectionEnabled = planarProjectionEnabled;
    options.planarRadius = planarProjectionRadius;
    options.lineCount = LivoxCore::lineCountForDeviceType(dev_type);

    PointCloudFrame frame;
    if (!PointCloudDecoder::decodeLivoxPacket(handle, packet, options, frame)) {
        return;
    }

    {
        QMutexLocker locker(&frameMutex);
        pendingFrames[handle].enqueue(PendingPointCloudFrame{nextPendingPointCloudSequence++, std::move(frame)});
        lastSeenTimestamp[handle] = pendingFrames[handle].back().frame.timestamp;
    }
}

void LivoxViewerWindow::resetRealtimePointCloudWindow()
{
    realtimeLastPresentedSequence = 0;
    realtimePointCloudSegmentTimestamps.clear();
    if (realtimePointCloudView) {
        realtimePointCloudView->clearPointCloudSegments();
    }
}

void LivoxViewerWindow::applyPointCloudPipeline(PointCloudFrame& frame, PointCloudView* targetView)
{
    PointCloudColorizer::Config colorConfig;
    colorConfig.mode = effectiveColorMode();
    colorConfig.reflectivityColorScale = reflectivityColorScale;
    colorConfig.solidColor = solidColor;
    colorConfig.lineColors = lineColors;
    colorConfig.distanceColorMin = distanceLegendMin;
    colorConfig.distanceColorMax = distanceLegendMax;
    colorConfig.elevationColorMin = elevationLegendMin;
    colorConfig.elevationColorMax = elevationLegendMax;

    const PointCloudPipelineLegend legend = PointCloudColorizer::apply(frame.points, colorConfig);
    PointCloudView* legendView = targetView ? targetView : pointCloudView;
    if (legendView) {
        legendView->setLegend(legend.mode,
                              legend.minValue,
                              legend.maxValue,
                              legend.visible,
                              legend.lineColors,
                              legend.lineNumbers,
                              colorConfig.mode == ColorByReflectivity
                                  ? PointCloudColorizer::reflectivityColorScaleStops(reflectivityColorScale)
                                  : (colorConfig.mode == ColorByElevation
                                         ? PointCloudColorizer::elevationColorScaleStops()
                                         : QVector<QColor>()));
    }

    PointCloudFilter::Config filterConfig;
    filterConfig.showNoisePoints = filterState.showNoisePoints;
    filterConfig.removeNoisePoints = filterState.removeNoisePoints;
    filterConfig.noiseTags = filterState.noiseFilterTags;
    frame.points = PointCloudFilter::apply(frame.points, filterConfig);
}

void LivoxViewerWindow::onRenderTick()
{
    const bool pointCloudFileCaptureActive = captureState.pcdSaveActive || captureState.lasSaveActive;

    if (!pointCloudVisualizationEnabled && !pointCloudFileCaptureActive) {
        {
            QMutexLocker locker(&frameMutex);
            for (auto it = pendingFrames.begin(); it != pendingFrames.end(); ++it) {
                it.value().clear();
            }
        }
        if (realtimePointCloudView) {
            realtimePointCloudView->update();
        }
        return;
    }

    const bool measurementViewActive = realtimePointCloudView && realtimePointCloudView->isMeasurementModeEnabled();
    if (measurementViewActive && !pointCloudFileCaptureActive) {
        {
            QMutexLocker locker(&frameMutex);
            for (auto it = pendingFrames.begin(); it != pendingFrames.end(); ++it) {
                it.value().clear();
            }
        }
        realtimePointCloudView->update();
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

    constexpr uint64_t displayTimeBucketNs = 50000000ULL;
    const uint64_t window_ns = frameIntervalMs * 1000000ULL;
    const uint64_t window_begin = (now_ns > window_ns) ? (now_ns - window_ns) : 0ULL;
    const uint64_t display_end = (now_ns / displayTimeBucketNs) * displayTimeBucketNs;
    const uint64_t display_window_begin = (display_end > window_ns) ? (display_end - window_ns) : 0ULL;
    if (pointCloudFileCaptureActive && captureState.pointCloudNextSaveTimestamp == 0) {
        captureState.pointCloudNextSaveTimestamp = now_ns + captureState.pointCloudSaveIntervalNs;
    }
    const bool pointCloudRecordingDue = pointCloudFileCaptureActive &&
                                        now_ns >= captureState.pointCloudNextSaveTimestamp;

    const bool sourceChanged = realtimePointCloudSourceHasTarget != hasTarget ||
                               (hasTarget && realtimePointCloudSourceHandle != targetHandle);
    if (sourceChanged) {
        resetRealtimePointCloudWindow();
        realtimePointCloudSourceHasTarget = hasTarget;
        realtimePointCloudSourceHandle = targetHandle;
    }

    while (!realtimePointCloudSegmentTimestamps.isEmpty() &&
           realtimePointCloudSegmentTimestamps.head() < display_window_begin) {
        realtimePointCloudSegmentTimestamps.dequeue();
        realtimePointCloudView->removeFirstPointCloudSegment();
    }

    QMap<uint64_t, PointCloudFrame> displaySegments;

    PointCloudFrame recordingFrame;
    recordingFrame.timestamp = now_ns;
    recordingFrame.device_handle = 0;

    quint64 latestPresentedSequence = realtimeLastPresentedSequence;
    {
        QMutexLocker locker(&frameMutex);
        for (auto it = pendingFrames.begin(); it != pendingFrames.end(); ++it) {
            if (hasTarget && it.key() != targetHandle) {
                continue;
            }

            QQueue<PendingPointCloudFrame>& q = it.value();
            while (!q.isEmpty() && q.head().frame.timestamp < display_window_begin) {
                q.dequeue();
            }

            if (pointCloudRecordingDue) {
                for (const PendingPointCloudFrame& pending : q) {
                    if (pending.frame.timestamp >= window_begin) {
                        recordingFrame.points += pending.frame.points;
                    }
                }
            }

            if (pointCloudVisualizationEnabled && !measurementViewActive) {
                int firstNewFrame = q.size();
                while (firstNewFrame > 0 &&
                       q.at(firstNewFrame - 1).sequence > realtimeLastPresentedSequence) {
                    --firstNewFrame;
                }
                for (int i = firstNewFrame; i < q.size(); ++i) {
                    const PendingPointCloudFrame& pending = q.at(i);
                    if (pending.frame.timestamp >= display_end) {
                        continue;
                    }
                    PointCloudFrame& segment = displaySegments[pending.frame.timestamp / displayTimeBucketNs];
                    if (segment.points.isEmpty()) {
                        segment.timestamp = (pending.frame.timestamp / displayTimeBucketNs) * displayTimeBucketNs;
                    }
                    segment.points += pending.frame.points;
                    latestPresentedSequence = std::max(latestPresentedSequence, pending.sequence);
                }
            }
        }
    }

    if (pointCloudRecordingDue && !recordingFrame.points.isEmpty()) {
        applyPointCloudPipeline(recordingFrame, realtimePointCloudView);
        handlePointCloudRecording(recordingFrame, now_ns);
    }

    for (auto it = displaySegments.begin(); it != displaySegments.end(); ++it) {
        PointCloudFrame& segment = it.value();
        applyPointCloudPipeline(segment, realtimePointCloudView);
        realtimePointCloudView->appendPointCloudSegment(std::move(segment.points));
        realtimePointCloudSegmentTimestamps.enqueue(segment.timestamp);
    }
    if (!displaySegments.isEmpty()) {
        realtimeLastPresentedSequence = latestPresentedSequence;
    }

}

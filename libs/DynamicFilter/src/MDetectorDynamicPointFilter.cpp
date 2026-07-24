#include "MDetectorDynamicPointFilter.h"

#include "DynamicObjectDetector.h"

#include <algorithm>
#include <chrono>

namespace {

DynamicFilterReason reasonFromLabel(DynamicObjectLabel label)
{
    switch(label)
    {
    case DynamicObjectLabel::Case1:
        return DynamicFilterReason::MDetectorCase1;
    case DynamicObjectLabel::Case2:
        return DynamicFilterReason::MDetectorCase2;
    case DynamicObjectLabel::Case3:
        return DynamicFilterReason::MDetectorCase3;
    case DynamicObjectLabel::Invalid:
        return DynamicFilterReason::Invalid;
    case DynamicObjectLabel::Static:
    default:
        return DynamicFilterReason::Static;
    }
}

bool dynamicLabel(DynamicObjectLabel label)
{
    return label == DynamicObjectLabel::Case1 ||
           label == DynamicObjectLabel::Case2 ||
           label == DynamicObjectLabel::Case3;
}

} // namespace

bool MDetectorDynamicPointFilter::configure(const DynamicFilterRuntimeConfig& config,
                                            std::string* error)
{
    config_ = config.mDetector;
    detector_ = std::make_unique<DynamicObjectDetector>(config_);
    if(error)
        error->clear();
    return true;
}

void MDetectorDynamicPointFilter::reset()
{
    detector_->reset();
}

bool MDetectorDynamicPointFilter::processFrame(const DynamicFilterFrame& frame,
                                               DynamicFilterResult* result,
                                               std::string* error)
{
    if(frame.lidarFrameCloud == nullptr || result == nullptr)
    {
        if(error)
            *error = "M-detector received a null frame cloud or result.";
        return false;
    }

    const auto startedAt = std::chrono::steady_clock::now();
    DynamicObjectDetectionFrame detectorFrame;
    detectorFrame.timestampNs = frame.timestampNs;
    detectorFrame.worldFromBodyRotation =
        Eigen::Quaterniond(frame.worldFromBody.linear());
    detectorFrame.worldFromBodyTranslation = frame.worldFromBody.translation();
    detectorFrame.points.reserve(
        static_cast<int>(frame.lidarFrameCloud->points.size()));

    for(const pcl::PointXYZINormal& lidarPoint : frame.lidarFrameCloud->points)
    {
        const Eigen::Vector3d bodyPoint =
            frame.bodyFromLidar * Eigen::Vector3d(lidarPoint.x, lidarPoint.y, lidarPoint.z);
        DynamicObjectPoint point;
        point.x = static_cast<float>(bodyPoint.x());
        point.y = static_cast<float>(bodyPoint.y());
        point.z = static_cast<float>(bodyPoint.z());
        point.reflectivity = static_cast<std::uint8_t>(
            std::clamp(lidarPoint.intensity, 0.0f, 255.0f));
        detectorFrame.points.push_back(point);
    }

    const DynamicObjectDetectionResult detection =
        detector_->processFrame(detectorFrame);
    if(detection.clusterLabels.size() !=
       static_cast<int>(frame.lidarFrameCloud->points.size()))
    {
        if(error)
            *error = "M-detector returned a label count that does not match the input cloud.";
        return false;
    }
    result->timestampNs = frame.timestampNs;
    result->labels.resize(frame.lidarFrameCloud->points.size());
    for(int index = 0; index < detection.clusterLabels.size(); ++index)
    {
        const DynamicObjectLabel label = detection.clusterLabels.at(index);
        result->labels[static_cast<std::size_t>(index)] =
            DynamicFilterLabel{dynamicLabel(label), reasonFromLabel(label)};
    }

    result->dynamicWorldPoints.clear();
    result->dynamicWorldPoints.reserve(
        static_cast<std::size_t>(detection.dynamicPoints.size()));
    for(const DynamicObjectPoint& point : detection.dynamicPoints)
    {
        result->dynamicWorldPoints.push_back(DynamicFilterPoint{
            point.worldX,
            point.worldY,
            point.worldZ,
            point.reflectivity,
            reasonFromLabel(point.label)});
    }

    result->stats = DynamicFilterStats{};
    result->stats.backend = DynamicFilterBackend::MDetector;
    result->stats.enabled = true;
    result->stats.clusterEnabled = detection.stats.clusterEnabled;
    result->stats.inputPointCount = detection.stats.inputPointCount;
    result->stats.staticPointCount = detection.stats.staticPointCount;
    result->stats.dynamicPointCount = detection.stats.dynamicPointCount;
    result->stats.invalidPointCount = detection.stats.invalidPointCount;
    result->stats.case1PointCount = detection.stats.case1PointCount;
    result->stats.case2PointCount = detection.stats.case2PointCount;
    result->stats.case3PointCount = detection.stats.case3PointCount;
    result->stats.originDynamicPointCount = detection.stats.originDynamicPointCount;
    result->stats.clusterCount = detection.stats.clusterCount;
    result->stats.rejectedClusterCount = detection.stats.rejectedClusterCount;
    result->stats.groundRemovedPointCount = detection.stats.groundRemovedPointCount;
    result->stats.historyDepthMapCount = detection.stats.historyDepthMapCount;
    result->stats.detectorMs = detection.stats.detectorMs;
    result->stats.clusterMs = detection.stats.clusterMs;
    result->stats.totalMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - startedAt).count();
    result->freeDomDebugSnapshot.reset();
    result->freeDomMapSnapshot.reset();
    if(error)
        error->clear();
    return true;
}

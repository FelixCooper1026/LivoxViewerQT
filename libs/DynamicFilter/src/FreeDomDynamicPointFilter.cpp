#include "FreeDomDynamicPointFilter.h"

#include "freedom/freedom.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>

#include <opencv2/imgcodecs.hpp>

namespace {

bool supportedConnectivity(unsigned int value)
{
    return value == 6 || value == 18 || value == 26;
}

bool validate(const FreeDomRuntimeConfig& config, std::string* error)
{
    const bool valid =
        std::isfinite(config.subVoxelSizeM) && config.subVoxelSizeM > 0.0 &&
        config.blockDepth >= config.voxelDepth &&
        config.numThreads > 0 && config.numThreads <= 64 &&
        std::isfinite(config.sensorMinRangeM) &&
        std::isfinite(config.sensorMaxRangeM) &&
        config.sensorMinRangeM < config.sensorMaxRangeM &&
        config.sensorMinZM < config.sensorMaxZM &&
        (!config.localMapEnabled ||
         (config.localMapRangeM > 0.0 &&
          config.localMapMinZM < config.localMapMaxZM)) &&
        config.raycastMaxRangeM > 0.0 &&
        config.raycastMinZM < config.raycastMaxZM &&
        config.countsToFree > 0 &&
        config.countsToRevert > 0 &&
        supportedConnectivity(config.conservativeConnectivity) &&
        (supportedConnectivity(config.aggressiveConnectivity) ||
         config.aggressiveConnectivity == 80 ||
         config.aggressiveConnectivity == 124) &&
        config.lidarHorizontalFovDeg > 0.0 &&
        config.lidarHorizontalFovDeg <= 360.0 &&
        config.lidarVerticalFovUpperDeg > config.lidarVerticalFovLowerDeg &&
        config.depthImageVerticalLines > 1 &&
        config.depthImageMinRangeM >= 0.0 &&
        config.maxRaycastEnhancementRangeM > 0.0 &&
        config.inpaintSize > 0;
    if(!valid)
    {
        if(error)
            *error = "FreeDOM configuration contains an invalid range, map resolution, connectivity, FOV or thread count.";
        return false;
    }
    if(config.learnFov && config.fovMaskPath.empty())
    {
        if(error)
            *error = "FreeDOM FOV learning requires an output mask path.";
        return false;
    }
    if(config.fovMaskEnabled && !config.learnFov)
    {
        const cv::Mat mask = cv::imread(config.fovMaskPath, cv::IMREAD_GRAYSCALE);
        const double radiansPerDegree = std::acos(-1.0) / 180.0;
        const double verticalFov =
            (config.lidarVerticalFovUpperDeg -
             config.lidarVerticalFovLowerDeg) * radiansPerDegree;
        const double verticalResolution =
            verticalFov / double(config.depthImageVerticalLines - 1);
        const int expectedColumns = int(std::ceil(
            config.lidarHorizontalFovDeg * radiansPerDegree /
            verticalResolution));
        if(mask.rows != int(config.depthImageVerticalLines) ||
           mask.cols != expectedColumns)
        {
            if(error)
                *error = "FreeDOM FOV mask cannot be read or its dimensions do not match the configured depth image.";
            return false;
        }
    }
    if(config.learnFov)
    {
        const std::filesystem::path outputPath(config.fovMaskPath);
        const std::filesystem::path parent = outputPath.parent_path();
        if(!parent.empty() && !std::filesystem::exists(parent))
        {
            if(error)
                *error = "FreeDOM FOV mask output directory does not exist.";
            return false;
        }
    }
    return true;
}

freedom::FreeDOM::Config toCoreConfig(const FreeDomRuntimeConfig& config)
{
    const double radiansPerDegree = std::acos(-1.0) / 180.0;
    return freedom::FreeDOM::Config{
        config.sensorMinRangeM,
        config.sensorMaxRangeM,
        config.sensorMinZM,
        config.sensorMaxZM,
        config.subVoxelSizeM,
        config.voxelDepth,
        config.blockDepth,
        config.localMapEnabled,
        config.localMapRangeM,
        config.localMapMinZM,
        config.localMapMaxZM,
        config.raycastMaxRangeM,
        config.raycastMinZM,
        config.raycastMaxZM,
        config.countsToFree,
        config.countsToRevert,
        config.conservativeConnectivity,
        config.aggressiveConnectivity,
        config.raycastEnhancementEnabled,
        config.lidarHorizontalFovDeg * radiansPerDegree,
        config.lidarVerticalFovUpperDeg * radiansPerDegree,
        config.lidarVerticalFovLowerDeg * radiansPerDegree,
        config.depthImageVerticalLines,
        config.depthImageMinRangeM,
        config.maxRaycastEnhancementRangeM,
        config.raycastEnhancementDepthMarginM,
        config.inpaintSize,
        config.erosionSize,
        config.minRaycastEnhancementArea,
        config.depthImageTopMargin,
        config.learnFov,
        config.fovMaskEnabled,
        config.fovMaskPath,
        config.numThreads};
}

DynamicFilterReason reasonFromLevel(freedom::DynamicLevel level)
{
    switch(level)
    {
    case freedom::DynamicLevel::AGGRESSIVE_DYNAMIC:
        return DynamicFilterReason::FreeDomAggressive;
    case freedom::DynamicLevel::MODERATE_DYNAMIC:
        return DynamicFilterReason::FreeDomModerate;
    case freedom::DynamicLevel::CONSERVATIVE_DYNAMIC:
        return DynamicFilterReason::FreeDomConservative;
    case freedom::DynamicLevel::STATIC:
    default:
        return DynamicFilterReason::Static;
    }
}

} // namespace

FreeDomDynamicPointFilter::FreeDomDynamicPointFilter() = default;
FreeDomDynamicPointFilter::~FreeDomDynamicPointFilter() = default;

bool FreeDomDynamicPointFilter::configure(const DynamicFilterRuntimeConfig& config,
                                          std::string* error)
{
    if(!validate(config.freeDom, error))
        return false;
    if(config.debugSnapshotIntervalFrames == 0 ||
       config.mapSnapshotIntervalFrames == 0)
    {
        if(error)
            *error = "FreeDOM snapshot intervals must be greater than zero.";
        return false;
    }

    config_ = config.freeDom;
    debugEnabled_ = config.debugVisualizationEnabled;
    debugSnapshotIntervalFrames_ = config.debugSnapshotIntervalFrames;
    mapSnapshotIntervalFrames_ = config.mapSnapshotIntervalFrames;
    configured_ = true;
    frameVersion_ = 0;
    createEngine();
    if(error)
        error->clear();
    return true;
}

void FreeDomDynamicPointFilter::createEngine()
{
    engine_ = std::make_unique<freedom::FreeDOM>();
    engine_->set_params(toCoreConfig(config_));
    engine_->set_scan_removal_callback([this](const freedom::ScanMap& scan) {
        if(buildingDebugSnapshot_)
            FreeDomSnapshotBuilder::appendScan(scan, *buildingDebugSnapshot_);
    });
    engine_->set_raycast_enhancement_callback([this](const freedom::DepthImage& depthImage) {
        if(buildingDebugSnapshot_)
            FreeDomSnapshotBuilder::appendDepthImage(depthImage, *buildingDebugSnapshot_);
    });
    engine_->set_map_removal_callback([this](const freedom::MRMap& map) {
        if(buildingDebugSnapshot_)
            FreeDomSnapshotBuilder::appendMap(map, *buildingDebugSnapshot_);
    });
    buildingDebugSnapshot_.reset();
    latestDebugSnapshot_.reset();
    latestMapSnapshot_.reset();
}

void FreeDomDynamicPointFilter::reset()
{
    frameVersion_ = 0;
    createEngine();
}

bool FreeDomDynamicPointFilter::processFrame(const DynamicFilterFrame& frame,
                                             DynamicFilterResult* result,
                                             std::string* error)
{
    if(!configured_ || frame.lidarFrameCloud == nullptr || result == nullptr)
    {
        if(error)
            *error = "FreeDOM is not configured or received a null frame cloud/result.";
        return false;
    }

    const auto startedAt = std::chrono::steady_clock::now();
    ++frameVersion_;
    if(debugEnabled_ &&
       (frameVersion_ == 1 ||
        (frameVersion_ % debugSnapshotIntervalFrames_) == 0))
    {
        buildingDebugSnapshot_ = std::make_shared<FreeDomDebugSnapshot>();
        buildingDebugSnapshot_->version = frameVersion_;
    }
    else
    {
        buildingDebugSnapshot_.reset();
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.reserve(frame.lidarFrameCloud->points.size());
    for(const pcl::PointXYZINormal& point : frame.lidarFrameCloud->points)
    {
        pcl::PointXYZ target;
        target.x = point.x;
        target.y = point.y;
        target.z = point.z;
        cloud.points.emplace_back(target);
    }
    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.height = 1;

    try
    {
        engine_->pointcloud_integrate(cloud, frame.worldFromLidar);
    }
    catch(const std::exception& exception)
    {
        if(error)
            *error = exception.what();
        buildingDebugSnapshot_.reset();
        return false;
    }

    if(buildingDebugSnapshot_)
    {
        latestDebugSnapshot_ = buildingDebugSnapshot_;
        buildingDebugSnapshot_.reset();
    }
    if(frameVersion_ == 1 ||
       (frameVersion_ % mapSnapshotIntervalFrames_) == 0)
    {
        auto mapSnapshot = std::make_shared<FreeDomMapSnapshot>();
        FreeDomSnapshotBuilder::buildMap(*engine_, frameVersion_, *mapSnapshot);
        if(latestDebugSnapshot_)
            mapSnapshot->raycastedVoxelCenters =
                latestDebugSnapshot_->raycastedVoxelCenters;
        latestMapSnapshot_ = std::move(mapSnapshot);
    }

    result->timestampNs = frame.timestampNs;
    result->labels.resize(frame.lidarFrameCloud->points.size());
    result->dynamicWorldPoints.clear();
    const auto& levels = engine_->last_point_dynamic_levels();
    const auto& validity = engine_->last_point_validity();
    if(levels.size() != frame.lidarFrameCloud->points.size() ||
       validity.size() != frame.lidarFrameCloud->points.size())
    {
        if(error)
            *error = "FreeDOM returned a label count that does not match the input cloud.";
        return false;
    }
    for(std::size_t index = 0; index < levels.size(); ++index)
    {
        if(!validity[index])
        {
            result->labels[index] =
                DynamicFilterLabel{false, DynamicFilterReason::Invalid};
            continue;
        }
        const bool dynamic = levels[index] != freedom::DynamicLevel::STATIC;
        const DynamicFilterReason reason = reasonFromLevel(levels[index]);
        result->labels[index] = DynamicFilterLabel{dynamic, reason};
        if(!dynamic)
            continue;
        const pcl::PointXYZINormal& point = frame.lidarFrameCloud->points[index];
        const Eigen::Vector3d worldPoint =
            frame.worldFromLidar * Eigen::Vector3d(point.x, point.y, point.z);
        result->dynamicWorldPoints.push_back(DynamicFilterPoint{
            static_cast<float>(worldPoint.x()),
            static_cast<float>(worldPoint.y()),
            static_cast<float>(worldPoint.z()),
            static_cast<std::uint8_t>(
                std::clamp(point.intensity, 0.0f, 255.0f)),
            reason});
    }

    result->stats = DynamicFilterStats{};
    result->stats.backend = DynamicFilterBackend::FreeDOM;
    result->stats.enabled = true;
    result->stats.inputPointCount =
        static_cast<int>(engine_->last_frame_stats().input_point_count);
    result->stats.staticPointCount =
        static_cast<int>(engine_->last_frame_stats().static_point_count);
    result->stats.dynamicPointCount =
        static_cast<int>(
            engine_->last_frame_stats().aggressive_point_count +
            engine_->last_frame_stats().moderate_point_count +
            engine_->last_frame_stats().conservative_point_count);
    result->stats.invalidPointCount =
        static_cast<int>(engine_->last_frame_stats().invalid_point_count);
    result->stats.freeDom = engine_->last_frame_stats();
    result->stats.totalMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - startedAt).count();
    result->stats.detectorMs = result->stats.totalMs;
    result->freeDomDebugSnapshot = latestDebugSnapshot_;
    result->freeDomMapSnapshot = latestMapSnapshot_;
    if(error)
        error->clear();
    return true;
}

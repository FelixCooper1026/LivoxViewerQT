#include "DynamicObjectDetector.h"
#include "DynamicObjectCluster.h"

#include <QElapsedTimer>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kSecondsToNanoseconds = 1.0e9;

bool isFinitePoint(const Eigen::Vector3d& point)
{
    return std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z());
}

int effectiveVoteThreshold(int configuredThreshold, int historyDepthMapCount)
{
    return std::max(1, std::min(configuredThreshold, historyDepthMapCount));
}

} // namespace

DynamicObjectDetector::DynamicObjectDetector(const DynamicObjectDetectorConfig& config)
    : config_(config),
      imageWidth_(static_cast<int>(std::ceil(2.0 * kPi / config.horizontalResolutionRad))),
      imageHeight_(static_cast<int>(std::ceil(
          (config.verticalFovUpDeg - config.verticalFovDownDeg) * kDegToRad /
          config.verticalResolutionRad)))
{
    if (config_.clusterEnabled) {
        cluster_ = std::make_unique<DynamicObjectCluster>(config_);
    }
}

DynamicObjectDetector::~DynamicObjectDetector() = default;

void DynamicObjectDetector::reset()
{
    bufferedFrames_.clear();
    depthMaps_.clear();
}

DynamicObjectDetectionResult DynamicObjectDetector::processFrame(const DynamicObjectDetectionFrame& frame)
{
    QElapsedTimer timer;
    timer.start();

    DynamicObjectDetectionResult result;
    result.stats.inputPointCount = frame.points.size();
    result.stats.clusterEnabled = config_.clusterEnabled;

    promoteBufferedFrames(frame.timestampNs);

    QVector<PointState> states;
    states.reserve(frame.points.size());

    const int historyCount = depthMaps_.size();
    const bool hasEnoughHistory = historyCount >= config_.minHistoryMaps;
    const int case1Threshold = effectiveVoteThreshold(config_.case1VoteThreshold, historyCount);
    const int case2Threshold = effectiveVoteThreshold(config_.case2VoteThreshold, historyCount);
    const int case3Threshold = effectiveVoteThreshold(config_.case3VoteThreshold, historyCount);

    for (const DynamicObjectPoint& sourcePoint : frame.points) {
        PointState state;
        state.point = sourcePoint;

        const Eigen::Vector3d bodyPoint(sourcePoint.x, sourcePoint.y, sourcePoint.z);
        state.valid = projectBodyPoint(bodyPoint, &state.currentProjection);
        if (!state.valid) {
            state.label = DynamicObjectLabel::Invalid;
            ++result.stats.invalidPointCount;
            states.push_back(state);
            continue;
        }

        const Eigen::Vector3d worldPoint =
            frame.worldFromBodyRotation * bodyPoint + frame.worldFromBodyTranslation;
        state.point.worldX = static_cast<float>(worldPoint.x());
        state.point.worldY = static_cast<float>(worldPoint.y());
        state.point.worldZ = static_cast<float>(worldPoint.z());

        if (!hasEnoughHistory) {
            ++result.stats.staticPointCount;
            states.push_back(state);
            continue;
        }

        int case1Votes = 0;
        int case2Votes = 0;
        int case3Votes = 0;
        for (const DepthMap& map : depthMaps_) {
            const Eigen::Vector3d historicalBodyPoint =
                map.worldFromBodyRotation.conjugate() * (worldPoint - map.worldFromBodyTranslation);
            PixelProjection historicalProjection;
            if (!projectBodyPoint(historicalBodyPoint, &historicalProjection)) {
                continue;
            }

            PixelStats stats;
            if (!gatherPixelStats(map, historicalProjection.x, historicalProjection.y, &stats)) {
                continue;
            }

            if (stats.hasStatic &&
                historicalProjection.depth < stats.minStaticDepth - static_cast<float>(config_.case1DepthMarginM)) {
                ++case1Votes;
            }
            if (stats.hasAny &&
                historicalProjection.depth > stats.maxAnyDepth + static_cast<float>(config_.case2DepthMarginM)) {
                ++case2Votes;
            }
            if (stats.hasAny &&
                historicalProjection.depth < stats.minAnyDepth - static_cast<float>(config_.case3DepthMarginM)) {
                ++case3Votes;
            }
        }

        if (case1Votes >= case1Threshold) {
            state.label = DynamicObjectLabel::Case1;
            ++result.stats.case1PointCount;
        } else if (case2Votes >= case2Threshold) {
            state.label = DynamicObjectLabel::Case2;
            ++result.stats.case2PointCount;
        } else if (case3Votes >= case3Threshold) {
            state.label = DynamicObjectLabel::Case3;
            ++result.stats.case3PointCount;
        } else {
            ++result.stats.staticPointCount;
        }

        if (state.label == DynamicObjectLabel::Case1 ||
            state.label == DynamicObjectLabel::Case2 ||
            state.label == DynamicObjectLabel::Case3) {
            state.point.label = state.label;
            result.originDynamicPoints.push_back(state.point);
        }
        states.push_back(state);
    }

    result.stats.originDynamicPointCount = result.originDynamicPoints.size();
    result.dynamicPoints = result.originDynamicPoints;
    result.originLabels.reserve(states.size());
    for (PointState& state : states) {
        state.point.label = state.label;
        result.originLabels.push_back(state.label);
    }
    result.clusterLabels = result.originLabels;
    result.stats.detectorMs = double(timer.nsecsElapsed()) / 1000000.0;

    if (cluster_) {
        QVector<DynamicObjectPoint> currentPoints;
        currentPoints.reserve(states.size());
        for (const PointState& state : states) {
            currentPoints.push_back(state.point);
        }
        const DynamicObjectClusterResult clusterResult = cluster_->processFrame(
            currentPoints,
            frame.worldFromBodyRotation * Eigen::Vector3d::UnitZ());
        result.dynamicPoints = clusterResult.dynamicPoints;
        result.clusterLabels = clusterResult.labels;
        result.stats.clusterCount = clusterResult.clusterCount;
        result.stats.rejectedClusterCount = clusterResult.rejectedClusterCount;
        result.stats.groundRemovedPointCount = clusterResult.groundRemovedPointCount;
        result.stats.clusterMs = clusterResult.clusterMs;
        result.stats.staticPointCount = int(frame.points.size()) -
            result.stats.invalidPointCount - int(result.dynamicPoints.size());
        for (int index = 0; index < states.size(); ++index) {
            states[index].label = result.clusterLabels.at(index);
            states[index].point.label = states[index].label;
        }
    }
    result.stats.dynamicPointCount = result.dynamicPoints.size();

    BufferedFrame bufferedFrame;
    bufferedFrame.timestampNs = frame.timestampNs;
    bufferedFrame.worldFromBodyRotation = frame.worldFromBodyRotation;
    bufferedFrame.worldFromBodyTranslation = frame.worldFromBodyTranslation;
    bufferedFrame.states = std::move(states);
    bufferedFrames_.push_back(std::move(bufferedFrame));
    result.stats.historyDepthMapCount = depthMaps_.size();
    return result;
}

bool DynamicObjectDetector::projectBodyPoint(const Eigen::Vector3d& point, PixelProjection* projection) const
{
    if (!projection || !isFinitePoint(point)) {
        return false;
    }

    const double range = point.norm();
    if (!std::isfinite(range) || range < config_.minRangeM || range > config_.maxRangeM) {
        return false;
    }

    const double horizontalRange = std::sqrt(point.x() * point.x() + point.y() * point.y());
    const double elevationDeg = std::atan2(point.z(), horizontalRange) * kRadToDeg;
    if (elevationDeg < config_.verticalFovDownDeg || elevationDeg > config_.verticalFovUpDeg) {
        return false;
    }

    const double azimuth = std::atan2(point.y(), point.x());
    const double azimuthDeg = azimuth * kRadToDeg;
    const double horizontalFovDeg = config_.horizontalFovLeftDeg - config_.horizontalFovRightDeg;
    if (horizontalFovDeg < 360.0 &&
        (azimuthDeg < config_.horizontalFovRightDeg || azimuthDeg > config_.horizontalFovLeftDeg)) {
        return false;
    }

    projection->x = std::clamp(
        static_cast<int>(std::floor((azimuth + kPi) / config_.horizontalResolutionRad)),
        0,
        imageWidth_ - 1);
    projection->y = std::clamp(
        static_cast<int>(std::floor(
            (config_.verticalFovUpDeg - elevationDeg) * kDegToRad / config_.verticalResolutionRad)),
        0,
        imageHeight_ - 1);
    projection->depth = static_cast<float>(range);
    return true;
}

bool DynamicObjectDetector::gatherPixelStats(const DepthMap& map, int x, int y, PixelStats* stats) const
{
    if (!stats) {
        return false;
    }

    const bool wrapHorizontal =
        config_.horizontalFovLeftDeg - config_.horizontalFovRightDeg >= 360.0;
    for (int dy = -config_.neighborPixelRadius; dy <= config_.neighborPixelRadius; ++dy) {
        const int py = y + dy;
        if (py < 0 || py >= imageHeight_) {
            continue;
        }
        for (int dx = -config_.neighborPixelRadius; dx <= config_.neighborPixelRadius; ++dx) {
            int px = x + dx;
            if (wrapHorizontal) {
                px = (px + imageWidth_) % imageWidth_;
            } else if (px < 0 || px >= imageWidth_) {
                continue;
            }
            const DepthPixel& pixel = map.pixels.at(pixelIndex(px, py));
            if (pixel.hasStatic) {
                if (!stats->hasStatic) {
                    stats->minStaticDepth = pixel.minStaticDepth;
                    stats->maxStaticDepth = pixel.maxStaticDepth;
                    stats->hasStatic = true;
                } else {
                    stats->minStaticDepth = std::min(stats->minStaticDepth, pixel.minStaticDepth);
                    stats->maxStaticDepth = std::max(stats->maxStaticDepth, pixel.maxStaticDepth);
                }
            }
            if (pixel.hasAny) {
                if (!stats->hasAny) {
                    stats->minAnyDepth = pixel.minAnyDepth;
                    stats->maxAnyDepth = pixel.maxAnyDepth;
                    stats->hasAny = true;
                } else {
                    stats->minAnyDepth = std::min(stats->minAnyDepth, pixel.minAnyDepth);
                    stats->maxAnyDepth = std::max(stats->maxAnyDepth, pixel.maxAnyDepth);
                }
            }
        }
    }
    return stats->hasStatic || stats->hasAny;
}

DynamicObjectDetector::DepthMap DynamicObjectDetector::buildDepthMap(const DynamicObjectDetectionFrame& frame,
                                                                     const QVector<PointState>& states) const
{
    DepthMap map;
    map.timestampNs = frame.timestampNs;
    map.worldFromBodyRotation = frame.worldFromBodyRotation;
    map.worldFromBodyTranslation = frame.worldFromBodyTranslation;
    map.pixels.resize(imageWidth_ * imageHeight_);

    for (const PointState& state : states) {
        if (!state.valid) {
            continue;
        }
        appendProjectedPoint(map, state.currentProjection, state.label);
    }
    return map;
}

void DynamicObjectDetector::mergeBufferedFrame(DepthMap& map, const BufferedFrame& frame) const
{
    for (const PointState& state : frame.states) {
        if (!state.valid) {
            continue;
        }
        const Eigen::Vector3d bodyPoint(state.point.x, state.point.y, state.point.z);
        const Eigen::Vector3d worldPoint =
            frame.worldFromBodyRotation * bodyPoint + frame.worldFromBodyTranslation;
        const Eigen::Vector3d mapBodyPoint =
            map.worldFromBodyRotation.conjugate() * (worldPoint - map.worldFromBodyTranslation);
        PixelProjection projection;
        if (projectBodyPoint(mapBodyPoint, &projection)) {
            appendProjectedPoint(map, projection, state.label);
        }
    }
}

void DynamicObjectDetector::appendProjectedPoint(DepthMap& map,
                                                 const PixelProjection& projection,
                                                 DynamicObjectLabel label) const
{
    DepthPixel& pixel = map.pixels[pixelIndex(projection.x, projection.y)];
    const float depth = projection.depth;
    if (!pixel.hasAny) {
        pixel.minAnyDepth = depth;
        pixel.maxAnyDepth = depth;
        pixel.hasAny = true;
    } else {
        pixel.minAnyDepth = std::min(pixel.minAnyDepth, depth);
        pixel.maxAnyDepth = std::max(pixel.maxAnyDepth, depth);
    }
    if (label != DynamicObjectLabel::Static) {
        return;
    }
    if (!pixel.hasStatic) {
        pixel.minStaticDepth = depth;
        pixel.maxStaticDepth = depth;
        pixel.hasStatic = true;
    } else {
        pixel.minStaticDepth = std::min(pixel.minStaticDepth, depth);
        pixel.maxStaticDepth = std::max(pixel.maxStaticDepth, depth);
    }
}

void DynamicObjectDetector::promoteBufferedFrames(int64_t currentTimestampNs)
{
    const int64_t bufferDelayNs = static_cast<int64_t>(
        std::llround(config_.bufferDelaySec * kSecondsToNanoseconds));
    const int64_t depthMapDurationNs = static_cast<int64_t>(
        std::llround(config_.depthMapDurationSec * kSecondsToNanoseconds));
    while (!bufferedFrames_.isEmpty() &&
           currentTimestampNs - bufferedFrames_.front().timestampNs >= bufferDelayNs) {
        BufferedFrame frame = std::move(bufferedFrames_.front());
        bufferedFrames_.removeFirst();
        if (depthMaps_.isEmpty() ||
            frame.timestampNs - depthMaps_.back().timestampNs >= depthMapDurationNs) {
            DynamicObjectDetectionFrame detectionFrame;
            detectionFrame.timestampNs = frame.timestampNs;
            detectionFrame.worldFromBodyRotation = frame.worldFromBodyRotation;
            detectionFrame.worldFromBodyTranslation = frame.worldFromBodyTranslation;
            appendDepthMap(buildDepthMap(detectionFrame, frame.states));
        } else {
            mergeBufferedFrame(depthMaps_.back(), frame);
        }
    }
}

void DynamicObjectDetector::appendDepthMap(DepthMap&& map)
{
    depthMaps_.push_back(std::move(map));
    while (depthMaps_.size() > config_.maxDepthMaps) {
        depthMaps_.removeFirst();
    }
}

int DynamicObjectDetector::pixelIndex(int x, int y) const
{
    return y * imageWidth_ + x;
}

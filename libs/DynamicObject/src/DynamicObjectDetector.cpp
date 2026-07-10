#include "DynamicObjectDetector.h"
#include "DynamicObjectCluster.h"

#include <QElapsedTimer>

#include <algorithm>
#include <cmath>
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

bool isDynamicLabel(DynamicObjectLabel label)
{
    return label == DynamicObjectLabel::Case1 ||
        label == DynamicObjectLabel::Case2 ||
        label == DynamicObjectLabel::Case3;
}

int effectiveVoteThreshold(int configuredThreshold, int historyDepthMapCount)
{
    return std::max(1, std::min(configuredThreshold, historyDepthMapCount));
}

double secondsBetween(int64_t newerTimestampNs, int64_t olderTimestampNs)
{
    return double(newerTimestampNs - olderTimestampNs) / kSecondsToNanoseconds;
}

double angularDistance(double left, double right)
{
    return std::abs(std::remainder(left - right, 2.0 * kPi));
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
    nextDepthMapId_ = 0;
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

    for (const DynamicObjectPoint& sourcePoint : frame.points) {
        PointState state;
        state.point = sourcePoint;

        const Eigen::Vector3d bodyPoint(sourcePoint.x, sourcePoint.y, sourcePoint.z);
        state.valid = projectBodyPoint(bodyPoint, &state.currentProjection);
        if (!state.valid) {
            state.label = DynamicObjectLabel::Invalid;
            ++result.stats.invalidPointCount;
            states.push_back(std::move(state));
            continue;
        }

        const Eigen::Vector3d worldPoint =
            frame.worldFromBodyRotation * bodyPoint + frame.worldFromBodyTranslation;
        state.point.worldX = static_cast<float>(worldPoint.x());
        state.point.worldY = static_cast<float>(worldPoint.y());
        state.point.worldZ = static_cast<float>(worldPoint.z());
        state.historicalPoint = std::make_shared<HistoricalPoint>();
        state.historicalPoint->worldPoint = worldPoint;
        state.historicalPoint->timestampNs = frame.timestampNs;

        if (hasEnoughHistory) {
            int case1Votes = 0;
            for (const DepthMap& map : depthMaps_) {
                PixelProjection historicalProjection;
                if (!projectWorldPoint(worldPoint, map, &historicalProjection)) {
                    continue;
                }

                PixelStats stats;
                if (gatherPixelStats(map, historicalProjection.x, historicalProjection.y, &stats) &&
                    stats.hasStatic &&
                    historicalProjection.depth <
                        stats.minStaticDepth - static_cast<float>(config_.case1DepthMarginM)) {
                    ++case1Votes;
                }
            }

            if (case1Votes >= case1Threshold) {
                state.label = DynamicObjectLabel::Case1;
                ++result.stats.case1PointCount;
            } else if (detectCase2(state.historicalPoint)) {
                state.label = DynamicObjectLabel::Case2;
                ++result.stats.case2PointCount;
            } else if (detectCase3(state.historicalPoint)) {
                state.label = DynamicObjectLabel::Case3;
                ++result.stats.case3PointCount;
            }
        }

        if (isDynamicLabel(state.label)) {
            state.point.label = state.label;
            result.originDynamicPoints.push_back(state.point);
        } else {
            ++result.stats.staticPointCount;
        }
        state.historicalPoint->label = state.label;
        states.push_back(std::move(state));
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
            PointState& state = states[index];
            const DynamicObjectLabel originLabel = state.label;
            state.label = result.clusterLabels.at(index);
            state.point.label = state.label;
            if (!state.historicalPoint) {
                continue;
            }
            state.historicalPoint->label = state.label;
            if (state.label == DynamicObjectLabel::Case1 ||
                (!isDynamicLabel(state.label) && isDynamicLabel(originLabel))) {
                state.historicalPoint->case2Predecessor.reset();
                state.historicalPoint->case2PredecessorMapId = -1;
                state.historicalPoint->case3Predecessor.reset();
                state.historicalPoint->case3PredecessorMapId = -1;
            } else if (state.label == DynamicObjectLabel::Case2) {
                state.historicalPoint->case3Predecessor.reset();
                state.historicalPoint->case3PredecessorMapId = -1;
            } else if (state.label == DynamicObjectLabel::Case3) {
                state.historicalPoint->case2Predecessor.reset();
                state.historicalPoint->case2PredecessorMapId = -1;
            }
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

bool DynamicObjectDetector::projectBodyPoint(const Eigen::Vector3d& point,
                                             PixelProjection* projection) const
{
    if (!projection || !isFinitePoint(point)) {
        return false;
    }

    const double range = point.norm();
    if (!std::isfinite(range) || range < config_.minRangeM || range > config_.maxRangeM) {
        return false;
    }

    const double horizontalRange = std::sqrt(point.x() * point.x() + point.y() * point.y());
    const double elevation = std::atan2(point.z(), horizontalRange);
    const double elevationDeg = elevation * kRadToDeg;
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
            (config_.verticalFovUpDeg - elevationDeg) * kDegToRad /
            config_.verticalResolutionRad)),
        0,
        imageHeight_ - 1);
    projection->azimuthRad = static_cast<float>(azimuth);
    projection->elevationRad = static_cast<float>(elevation);
    projection->depth = static_cast<float>(range);
    return true;
}

bool DynamicObjectDetector::projectWorldPoint(const Eigen::Vector3d& point,
                                              const DepthMap& map,
                                              PixelProjection* projection) const
{
    const Eigen::Vector3d bodyPoint =
        map.worldFromBodyRotation.conjugate() * (point - map.worldFromBodyTranslation);
    return projectBodyPoint(bodyPoint, projection);
}

bool DynamicObjectDetector::gatherPixelStats(const DepthMap& map,
                                             int x,
                                             int y,
                                             PixelStats* stats) const
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

QVector<std::shared_ptr<DynamicObjectDetector::HistoricalPoint>>
DynamicObjectDetector::gatherNeighborPoints(const DepthMap& map,
                                            const PixelProjection& projection,
                                            double horizontalThresholdRad,
                                            double verticalThresholdRad) const
{
    QVector<std::shared_ptr<HistoricalPoint>> points;
    const int horizontalRadius =
        static_cast<int>(std::ceil(horizontalThresholdRad / config_.horizontalResolutionRad));
    const int verticalRadius =
        static_cast<int>(std::ceil(verticalThresholdRad / config_.verticalResolutionRad));
    const bool wrapHorizontal =
        config_.horizontalFovLeftDeg - config_.horizontalFovRightDeg >= 360.0;

    for (int dy = -verticalRadius; dy <= verticalRadius; ++dy) {
        const int py = projection.y + dy;
        if (py < 0 || py >= imageHeight_) {
            continue;
        }
        for (int dx = -horizontalRadius; dx <= horizontalRadius; ++dx) {
            int px = projection.x + dx;
            if (wrapHorizontal) {
                px = (px + imageWidth_) % imageWidth_;
            } else if (px < 0 || px >= imageWidth_) {
                continue;
            }
            const DepthPixel& pixel = map.pixels.at(pixelIndex(px, py));
            for (const std::shared_ptr<HistoricalPoint>& point : pixel.points) {
                if (angularDistance(projection.azimuthRad,
                                    point->depthMapProjection.azimuthRad) < horizontalThresholdRad &&
                    std::abs(double(projection.elevationRad) -
                             double(point->depthMapProjection.elevationRad)) < verticalThresholdRad) {
                    points.push_back(point);
                }
            }
        }
    }
    return points;
}

bool DynamicObjectDetector::detectCase2(const std::shared_ptr<HistoricalPoint>& point)
{
    DepthMap& latestMap = depthMaps_.back();
    PixelProjection latestProjection;
    if (!projectWorldPoint(point->worldPoint, latestMap, &latestProjection) ||
        !case2Enter(*point, latestProjection, latestMap) ||
        case2MapConsistent(*point, latestProjection, latestMap)) {
        return false;
    }

    const QVector<std::shared_ptr<HistoricalPoint>> candidates = gatherNeighborPoints(
        latestMap,
        latestProjection,
        config_.case2OcclusionHorizontalRad,
        config_.case2OcclusionVerticalRad);
    for (const std::shared_ptr<HistoricalPoint>& predecessor : candidates) {
        if (!case2IsOccluded(*point, latestProjection, *predecessor) ||
            !case2DepthConsistent(*predecessor, latestMap)) {
            continue;
        }

        point->case2Predecessor = predecessor;
        point->case2PredecessorMapId = latestMap.id;
        point->case2QueryProjection = latestProjection;
        int chainLength = 1;
        if (chainLength >= config_.case2OcclusionChainLength) {
            return true;
        }

        double segmentDuration = secondsBetween(point->timestampNs, predecessor->timestampNs);
        double previousVelocity =
            (latestProjection.depth - predecessor->depthMapProjection.depth) / segmentDuration;
        double previousMidpointNs = 0.5 * double(point->timestampNs + predecessor->timestampNs);
        std::shared_ptr<HistoricalPoint> cursor = predecessor;
        int searchMapIndex = depthMaps_.size() - 2;
        bool mapConsistent = false;

        while (chainLength < config_.case2OcclusionChainLength) {
            std::shared_ptr<HistoricalPoint> next = cursor->case2Predecessor.lock();
            int edgeMapIndex = depthMapIndex(cursor->case2PredecessorMapId);
            PixelProjection cursorProjection = cursor->case2QueryProjection;
            if (!next || edgeMapIndex < 0) {
                if (searchMapIndex < 0 ||
                    !findCase2Occluder(*cursor,
                                       depthMaps_.at(searchMapIndex),
                                       &next,
                                       &cursorProjection)) {
                    break;
                }
                edgeMapIndex = searchMapIndex;
                cursor->case2Predecessor = next;
                cursor->case2PredecessorMapId = depthMaps_.at(edgeMapIndex).id;
                cursor->case2QueryProjection = cursorProjection;
            }

            const DepthMap& edgeMap = depthMaps_.at(edgeMapIndex);
            if (next->depthMapId != edgeMap.id) {
                break;
            }
            PixelProjection pointProjection;
            if (!projectWorldPoint(point->worldPoint, edgeMap, &pointProjection)) {
                break;
            }
            if (case2MapConsistent(*point, pointProjection, edgeMap)) {
                mapConsistent = true;
                break;
            }
            if (!case2IsOccluded(*point, pointProjection, *next) ||
                !case2DepthConsistent(*next, edgeMap)) {
                break;
            }

            segmentDuration = secondsBetween(cursor->timestampNs, next->timestampNs);
            const double currentVelocity =
                (cursorProjection.depth - next->depthMapProjection.depth) / segmentDuration;
            const double currentMidpointNs = 0.5 * double(cursor->timestampNs + next->timestampNs);
            const double midpointDeltaSec =
                (previousMidpointNs - currentMidpointNs) / kSecondsToNanoseconds;
            if (std::abs(previousVelocity - currentVelocity) >=
                midpointDeltaSec * config_.case2AccelerationLimitMps2) {
                break;
            }

            ++chainLength;
            previousVelocity = currentVelocity;
            previousMidpointNs = currentMidpointNs;
            cursor = next;
            searchMapIndex = edgeMapIndex - 1;
        }
        if (chainLength >= config_.case2OcclusionChainLength) {
            return true;
        }
        if (mapConsistent) {
            return false;
        }
    }
    return false;
}

bool DynamicObjectDetector::detectCase3(const std::shared_ptr<HistoricalPoint>& point)
{
    DepthMap& latestMap = depthMaps_.back();
    PixelProjection latestProjection;
    if (!projectWorldPoint(point->worldPoint, latestMap, &latestProjection) ||
        !case3Enter(*point, latestProjection, latestMap) ||
        case3MapConsistent(*point, latestProjection, latestMap)) {
        return false;
    }

    const QVector<std::shared_ptr<HistoricalPoint>> candidates = gatherNeighborPoints(
        latestMap,
        latestProjection,
        config_.case3OcclusionHorizontalRad,
        config_.case3OcclusionVerticalRad);
    for (const std::shared_ptr<HistoricalPoint>& predecessor : candidates) {
        if (!case3IsOccluding(*point, latestProjection, *predecessor) ||
            !case3DepthConsistent(*predecessor, latestMap)) {
            continue;
        }

        point->case3Predecessor = predecessor;
        point->case3PredecessorMapId = latestMap.id;
        point->case3QueryProjection = latestProjection;
        int chainLength = 1;
        if (chainLength >= config_.case3OcclusionChainLength) {
            return true;
        }

        double segmentDuration = secondsBetween(point->timestampNs, predecessor->timestampNs);
        double previousVelocity =
            (predecessor->depthMapProjection.depth - latestProjection.depth) / segmentDuration;
        double previousMidpointNs = 0.5 * double(point->timestampNs + predecessor->timestampNs);
        std::shared_ptr<HistoricalPoint> cursor = predecessor;
        int searchMapIndex = depthMaps_.size() - 2;
        bool mapConsistent = false;

        while (chainLength < config_.case3OcclusionChainLength) {
            std::shared_ptr<HistoricalPoint> next = cursor->case3Predecessor.lock();
            int edgeMapIndex = depthMapIndex(cursor->case3PredecessorMapId);
            PixelProjection cursorProjection = cursor->case3QueryProjection;
            if (!next || edgeMapIndex < 0) {
                if (searchMapIndex < 0 ||
                    !findCase3OccludedPoint(*cursor,
                                            depthMaps_.at(searchMapIndex),
                                            &next,
                                            &cursorProjection)) {
                    break;
                }
                edgeMapIndex = searchMapIndex;
                cursor->case3Predecessor = next;
                cursor->case3PredecessorMapId = depthMaps_.at(edgeMapIndex).id;
                cursor->case3QueryProjection = cursorProjection;
            }

            const DepthMap& edgeMap = depthMaps_.at(edgeMapIndex);
            if (next->depthMapId != edgeMap.id) {
                break;
            }
            PixelProjection pointProjection;
            if (!projectWorldPoint(point->worldPoint, edgeMap, &pointProjection)) {
                break;
            }
            if (case3MapConsistent(*point, pointProjection, edgeMap)) {
                mapConsistent = true;
                break;
            }
            if (!case3IsOccluding(*point, pointProjection, *next) ||
                !case3DepthConsistent(*next, edgeMap)) {
                break;
            }

            segmentDuration = secondsBetween(cursor->timestampNs, next->timestampNs);
            const double currentVelocity =
                (next->depthMapProjection.depth - cursorProjection.depth) / segmentDuration;
            const double currentMidpointNs = 0.5 * double(cursor->timestampNs + next->timestampNs);
            const double midpointDeltaSec =
                (previousMidpointNs - currentMidpointNs) / kSecondsToNanoseconds;
            if (std::abs(previousVelocity - currentVelocity) >=
                midpointDeltaSec * config_.case3AccelerationLimitMps2) {
                break;
            }

            ++chainLength;
            previousVelocity = currentVelocity;
            previousMidpointNs = currentMidpointNs;
            cursor = next;
            searchMapIndex = edgeMapIndex - 1;
        }
        if (chainLength >= config_.case3OcclusionChainLength) {
            return true;
        }
        if (mapConsistent) {
            return false;
        }
    }
    return false;
}

bool DynamicObjectDetector::case2Enter(const HistoricalPoint& point,
                                       const PixelProjection& projection,
                                       const DepthMap& map) const
{
    const DepthPixel& pixel = map.pixels.at(pixelIndex(projection.x, projection.y));
    if (!pixel.maxAnyPoint) {
        return false;
    }
    const double deltaTime = secondsBetween(point.timestampNs, pixel.maxAnyPoint->timestampNs);
    const double threshold =
        std::min(config_.case2DepthMarginM, config_.case2MinVelocityMps * deltaTime);
    return deltaTime > 0.0 && projection.depth > pixel.maxAnyDepth + threshold;
}

bool DynamicObjectDetector::case3Enter(const HistoricalPoint& point,
                                       const PixelProjection& projection,
                                       const DepthMap& map) const
{
    const DepthPixel& pixel = map.pixels.at(pixelIndex(projection.x, projection.y));
    if (!pixel.minAnyPoint) {
        return false;
    }
    const double deltaTime = secondsBetween(point.timestampNs, pixel.minAnyPoint->timestampNs);
    const double threshold =
        std::min(config_.case3DepthMarginM, config_.case3MinVelocityMps * deltaTime);
    return deltaTime > 0.0 && projection.depth < pixel.minAnyDepth - threshold;
}

bool DynamicObjectDetector::case2MapConsistent(const HistoricalPoint& point,
                                               const PixelProjection& projection,
                                               const DepthMap& map) const
{
    const double depthThreshold = config_.case2MapConsistencyDepthM;
    const int horizontalRadius = static_cast<int>(std::ceil(
        config_.case2MapConsistencyHorizontalRad / config_.horizontalResolutionRad));
    const int verticalRadius = static_cast<int>(std::ceil(
        config_.case2MapConsistencyVerticalRad / config_.verticalResolutionRad));
    const int64_t frameDurationNs =
        static_cast<int64_t>(std::llround(config_.frameDurationSec * kSecondsToNanoseconds));
    const bool wrapHorizontal =
        config_.horizontalFovLeftDeg - config_.horizontalFovRightDeg >= 360.0;

    for (int dy = -verticalRadius; dy <= verticalRadius; ++dy) {
        const int py = projection.y + dy;
        if (py < 0 || py >= imageHeight_) {
            continue;
        }
        for (int dx = -horizontalRadius; dx <= horizontalRadius; ++dx) {
            int px = projection.x + dx;
            if (wrapHorizontal) {
                px = (px + imageWidth_) % imageWidth_;
            } else if (px < 0 || px >= imageWidth_) {
                continue;
            }
            const DepthPixel& pixel = map.pixels.at(pixelIndex(px, py));
            if (pixel.hasAny &&
                pixel.maxAnyDepth > projection.depth + depthThreshold &&
                pixel.minAnyDepth < projection.depth - depthThreshold) {
                continue;
            }
            for (const std::shared_ptr<HistoricalPoint>& candidate : pixel.points) {
                if (candidate->label == DynamicObjectLabel::Static &&
                    std::abs(point.timestampNs - candidate->timestampNs) > frameDurationNs &&
                    std::abs(double(projection.depth) - candidate->depthMapProjection.depth) < depthThreshold &&
                    angularDistance(projection.azimuthRad,
                                    candidate->depthMapProjection.azimuthRad) <
                        config_.case2MapConsistencyHorizontalRad &&
                    std::abs(double(projection.elevationRad) -
                             double(candidate->depthMapProjection.elevationRad)) <
                        config_.case2MapConsistencyVerticalRad) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool DynamicObjectDetector::case3MapConsistent(const HistoricalPoint& point,
                                               const PixelProjection& projection,
                                               const DepthMap& map) const
{
    const double depthThreshold = config_.case3MapConsistencyDepthM;
    const int horizontalRadius = static_cast<int>(std::ceil(
        config_.case3MapConsistencyHorizontalRad / config_.horizontalResolutionRad));
    const int verticalRadius = static_cast<int>(std::ceil(
        config_.case3MapConsistencyVerticalRad / config_.verticalResolutionRad));
    const int64_t frameDurationNs =
        static_cast<int64_t>(std::llround(config_.frameDurationSec * kSecondsToNanoseconds));
    const bool wrapHorizontal =
        config_.horizontalFovLeftDeg - config_.horizontalFovRightDeg >= 360.0;

    for (int dy = -verticalRadius; dy <= verticalRadius; ++dy) {
        const int py = projection.y + dy;
        if (py < 0 || py >= imageHeight_) {
            continue;
        }
        for (int dx = -horizontalRadius; dx <= horizontalRadius; ++dx) {
            int px = projection.x + dx;
            if (wrapHorizontal) {
                px = (px + imageWidth_) % imageWidth_;
            } else if (px < 0 || px >= imageWidth_) {
                continue;
            }
            const DepthPixel& pixel = map.pixels.at(pixelIndex(px, py));
            if (pixel.hasAny &&
                pixel.maxAnyDepth > projection.depth + depthThreshold &&
                pixel.minAnyDepth < projection.depth - depthThreshold) {
                continue;
            }
            for (const std::shared_ptr<HistoricalPoint>& candidate : pixel.points) {
                if (candidate->label == DynamicObjectLabel::Static &&
                    std::abs(point.timestampNs - candidate->timestampNs) > frameDurationNs &&
                    candidate->depthMapProjection.depth - projection.depth < depthThreshold &&
                    angularDistance(projection.azimuthRad,
                                    candidate->depthMapProjection.azimuthRad) <
                        config_.case3MapConsistencyHorizontalRad &&
                    std::abs(double(projection.elevationRad) -
                             double(candidate->depthMapProjection.elevationRad)) <
                        config_.case3MapConsistencyVerticalRad) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool DynamicObjectDetector::case2DepthConsistent(const HistoricalPoint& point,
                                                 const DepthMap& map) const
{
    const QVector<std::shared_ptr<HistoricalPoint>> neighbors = gatherNeighborPoints(
        map,
        point.depthMapProjection,
        config_.case2DepthConsistencyHorizontalRad,
        config_.case2DepthConsistencyVerticalRad);
    const int64_t frameDurationNs =
        static_cast<int64_t>(std::llround(config_.frameDurationSec * kSecondsToNanoseconds));
    double totalDifference = 0.0;
    int closeCount = 0;
    int closerCount = 0;
    int fartherCount = 0;
    int allCount = 0;
    for (const std::shared_ptr<HistoricalPoint>& neighbor : neighbors) {
        if (std::abs(neighbor->timestampNs - point.timestampNs) >= frameDurationNs) {
            continue;
        }
        ++allCount;
        if (neighbor->label != DynamicObjectLabel::Static) {
            continue;
        }
        const double difference =
            double(point.depthMapProjection.depth) - neighbor->depthMapProjection.depth;
        if (std::abs(difference) < config_.case2DepthConsistencyMaxM) {
            ++closeCount;
            totalDifference += std::abs(difference);
        } else if (difference > 0.0) {
            ++closerCount;
        } else {
            ++fartherCount;
        }
    }
    if (allCount == 0) {
        return false;
    }
    if (closeCount > 1) {
        const double meanThreshold = std::max(
            config_.case2DepthConsistencyMeanM,
            config_.case2DepthConsistencyScale * point.depthMapProjection.depth);
        if (totalDifference / double(closeCount - 1) > meanThreshold) {
            return false;
        }
    }
    return closerCount == 0 || fartherCount == 0;
}

bool DynamicObjectDetector::case3DepthConsistent(const HistoricalPoint& point,
                                                 const DepthMap& map) const
{
    const QVector<std::shared_ptr<HistoricalPoint>> neighbors = gatherNeighborPoints(
        map,
        point.depthMapProjection,
        config_.case3DepthConsistencyHorizontalRad,
        config_.case3DepthConsistencyVerticalRad);
    const int64_t frameDurationNs =
        static_cast<int64_t>(std::llround(config_.frameDurationSec * kSecondsToNanoseconds));
    double totalDifference = 0.0;
    int closeCount = 0;
    int closerCount = 0;
    int fartherCount = 0;
    int allCount = 0;
    for (const std::shared_ptr<HistoricalPoint>& neighbor : neighbors) {
        if (std::abs(neighbor->timestampNs - point.timestampNs) >= frameDurationNs) {
            continue;
        }
        ++allCount;
        if (neighbor->label != DynamicObjectLabel::Static) {
            continue;
        }
        const double difference =
            double(point.depthMapProjection.depth) - neighbor->depthMapProjection.depth;
        if (std::abs(difference) < config_.case3DepthConsistencyMaxM) {
            ++closeCount;
            totalDifference += std::abs(difference);
        } else if (difference > 0.0) {
            ++closerCount;
        } else {
            ++fartherCount;
        }
    }
    if (allCount == 0) {
        return false;
    }
    if (closeCount > 1) {
        const double meanThreshold = std::max(
            config_.case3DepthConsistencyMeanM,
            config_.case3DepthConsistencyScale * point.depthMapProjection.depth);
        if (totalDifference / double(closeCount - 1) > meanThreshold) {
            return false;
        }
    }
    return closerCount == 0 || fartherCount == 0;
}

bool DynamicObjectDetector::case2IsOccluded(const HistoricalPoint& point,
                                            const PixelProjection& projection,
                                            const HistoricalPoint& occluder) const
{
    if (occluder.label == DynamicObjectLabel::Invalid) {
        return false;
    }
    const double deltaTime = secondsBetween(point.timestampNs, occluder.timestampNs);
    const double depthThreshold =
        std::min(config_.case2DepthMarginM, config_.case2MinVelocityMps * deltaTime);
    return deltaTime > 0.0 &&
        projection.depth > occluder.depthMapProjection.depth + depthThreshold &&
        angularDistance(projection.azimuthRad, occluder.depthMapProjection.azimuthRad) <
            config_.case2OcclusionHorizontalRad &&
        std::abs(double(projection.elevationRad) -
                 double(occluder.depthMapProjection.elevationRad)) <
            config_.case2OcclusionVerticalRad;
}

bool DynamicObjectDetector::case3IsOccluding(const HistoricalPoint& point,
                                             const PixelProjection& projection,
                                             const HistoricalPoint& occludedPoint) const
{
    if (occludedPoint.label == DynamicObjectLabel::Invalid) {
        return false;
    }
    const double deltaTime = secondsBetween(point.timestampNs, occludedPoint.timestampNs);
    const double depthThreshold = std::min(
        config_.case3MapConsistencyDepthM,
        config_.case3MinVelocityMps * deltaTime);
    return deltaTime > 0.0 &&
        occludedPoint.depthMapProjection.depth > projection.depth + depthThreshold &&
        angularDistance(projection.azimuthRad, occludedPoint.depthMapProjection.azimuthRad) <
            config_.case3OcclusionHorizontalRad &&
        std::abs(double(projection.elevationRad) -
                 double(occludedPoint.depthMapProjection.elevationRad)) <
            config_.case3OcclusionVerticalRad;
}

bool DynamicObjectDetector::findCase2Occluder(
    const HistoricalPoint& point,
    const DepthMap& map,
    std::shared_ptr<HistoricalPoint>* occluder,
    PixelProjection* queryProjection) const
{
    PixelProjection projection;
    if (!projectWorldPoint(point.worldPoint, map, &projection)) {
        return false;
    }
    const QVector<std::shared_ptr<HistoricalPoint>> candidates = gatherNeighborPoints(
        map,
        projection,
        config_.case2OcclusionHorizontalRad,
        config_.case2OcclusionVerticalRad);
    for (const std::shared_ptr<HistoricalPoint>& candidate : candidates) {
        if (case2IsOccluded(point, projection, *candidate) &&
            case2DepthConsistent(*candidate, map)) {
            *occluder = candidate;
            *queryProjection = projection;
            return true;
        }
    }
    return false;
}

bool DynamicObjectDetector::findCase3OccludedPoint(
    const HistoricalPoint& point,
    const DepthMap& map,
    std::shared_ptr<HistoricalPoint>* occludedPoint,
    PixelProjection* queryProjection) const
{
    PixelProjection projection;
    if (!projectWorldPoint(point.worldPoint, map, &projection)) {
        return false;
    }
    const QVector<std::shared_ptr<HistoricalPoint>> candidates = gatherNeighborPoints(
        map,
        projection,
        config_.case3OcclusionHorizontalRad,
        config_.case3OcclusionVerticalRad);
    for (const std::shared_ptr<HistoricalPoint>& candidate : candidates) {
        if (case3IsOccluding(point, projection, *candidate) &&
            case3DepthConsistent(*candidate, map)) {
            *occludedPoint = candidate;
            *queryProjection = projection;
            return true;
        }
    }
    return false;
}

int DynamicObjectDetector::depthMapIndex(int id) const
{
    for (int index = 0; index < depthMaps_.size(); ++index) {
        if (depthMaps_.at(index).id == id) {
            return index;
        }
    }
    return -1;
}

DynamicObjectDetector::DepthMap DynamicObjectDetector::buildDepthMap(
    const DynamicObjectDetectionFrame& frame,
    const QVector<PointState>& states,
    int mapId) const
{
    DepthMap map;
    map.id = mapId;
    map.timestampNs = frame.timestampNs;
    map.worldFromBodyRotation = frame.worldFromBodyRotation;
    map.worldFromBodyTranslation = frame.worldFromBodyTranslation;
    map.pixels.resize(imageWidth_ * imageHeight_);

    for (const PointState& state : states) {
        if (state.valid) {
            appendProjectedPoint(map, state.currentProjection, state.historicalPoint);
        }
    }
    return map;
}

void DynamicObjectDetector::mergeBufferedFrame(DepthMap& map,
                                               const BufferedFrame& frame) const
{
    for (const PointState& state : frame.states) {
        if (!state.valid) {
            continue;
        }
        PixelProjection projection;
        if (projectWorldPoint(state.historicalPoint->worldPoint, map, &projection)) {
            appendProjectedPoint(map, projection, state.historicalPoint);
        }
    }
}

void DynamicObjectDetector::appendProjectedPoint(
    DepthMap& map,
    const PixelProjection& projection,
    const std::shared_ptr<HistoricalPoint>& point) const
{
    DepthPixel& pixel = map.pixels[pixelIndex(projection.x, projection.y)];
    if (pixel.points.size() >= config_.maxPointsPerPixel) {
        return;
    }

    point->depthMapId = map.id;
    point->depthMapProjection = projection;
    pixel.points.push_back(point);
    const float depth = projection.depth;
    if (!pixel.hasAny) {
        pixel.minAnyDepth = depth;
        pixel.maxAnyDepth = depth;
        pixel.minAnyPoint = point;
        pixel.maxAnyPoint = point;
        pixel.hasAny = true;
    } else {
        if (depth < pixel.minAnyDepth) {
            pixel.minAnyDepth = depth;
            pixel.minAnyPoint = point;
        }
        if (depth > pixel.maxAnyDepth) {
            pixel.maxAnyDepth = depth;
            pixel.maxAnyPoint = point;
        }
    }
    if (point->label != DynamicObjectLabel::Static) {
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
            appendDepthMap(buildDepthMap(detectionFrame, frame.states, nextDepthMapId_++));
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

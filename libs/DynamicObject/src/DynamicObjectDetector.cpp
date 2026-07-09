#include "DynamicObjectDetector.h"

#include <QElapsedTimer>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

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
    : config_(config)
{
}

void DynamicObjectDetector::reset()
{
    depthMaps_.clear();
}

DynamicObjectDetectionResult DynamicObjectDetector::processFrame(const DynamicObjectDetectionFrame& frame)
{
    QElapsedTimer timer;
    timer.start();

    DynamicObjectDetectionResult result;
    result.stats.inputPointCount = frame.points.size();
    result.stats.clusterEnabled = config_.clusterEnabled;

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
            result.dynamicPoints.push_back(state.point);
            ++result.stats.dynamicPointCount;
        }
        states.push_back(state);
    }

    appendDepthMap(buildDepthMap(frame, states));
    result.stats.historyDepthMapCount = depthMaps_.size();
    result.stats.detectorMs = double(timer.nsecsElapsed()) / 1000000.0;
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
    const double u = (azimuth + kPi) / (2.0 * kPi);
    const double v = (config_.verticalFovUpDeg - elevationDeg) /
        (config_.verticalFovUpDeg - config_.verticalFovDownDeg);

    projection->x = std::clamp(static_cast<int>(std::floor(u * double(config_.imageWidth))),
                               0,
                               config_.imageWidth - 1);
    projection->y = std::clamp(static_cast<int>(std::floor(v * double(config_.imageHeight))),
                               0,
                               config_.imageHeight - 1);
    projection->depth = static_cast<float>(range);
    return true;
}

bool DynamicObjectDetector::gatherPixelStats(const DepthMap& map, int x, int y, PixelStats* stats) const
{
    if (!stats) {
        return false;
    }

    for (int dy = -config_.neighborPixelRadius; dy <= config_.neighborPixelRadius; ++dy) {
        const int py = y + dy;
        if (py < 0 || py >= config_.imageHeight) {
            continue;
        }
        for (int dx = -config_.neighborPixelRadius; dx <= config_.neighborPixelRadius; ++dx) {
            const int px = x + dx;
            if (px < 0 || px >= config_.imageWidth) {
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
    map.pixels.resize(config_.imageWidth * config_.imageHeight);

    for (const PointState& state : states) {
        if (!state.valid) {
            continue;
        }
        const bool staticPoint = state.label == DynamicObjectLabel::Static;
        DepthPixel& pixel = map.pixels[pixelIndex(state.currentProjection.x, state.currentProjection.y)];
        const float depth = state.currentProjection.depth;
        if (!pixel.hasAny) {
            pixel.minAnyDepth = depth;
            pixel.maxAnyDepth = depth;
            pixel.hasAny = true;
        } else {
            pixel.minAnyDepth = std::min(pixel.minAnyDepth, depth);
            pixel.maxAnyDepth = std::max(pixel.maxAnyDepth, depth);
        }
        if (!staticPoint) {
            continue;
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
    return map;
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
    return y * config_.imageWidth + x;
}

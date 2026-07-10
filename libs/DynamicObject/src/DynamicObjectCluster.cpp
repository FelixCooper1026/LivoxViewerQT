#include "DynamicObjectCluster.h"

#include <Eigen/Eigenvalues>

#include <QElapsedTimer>

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct VoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const VoxelKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const
    {
        std::size_t value = std::hash<int>{}(key.x);
        value ^= std::hash<int>{}(key.y) + 0x9e3779b9 + (value << 6) + (value >> 2);
        value ^= std::hash<int>{}(key.z) + 0x9e3779b9 + (value << 6) + (value >> 2);
        return value;
    }
};

using VoxelPointMap = std::unordered_map<VoxelKey, std::vector<int>, VoxelKeyHash>;
using VoxelSet = std::unordered_set<VoxelKey, VoxelKeyHash>;

struct PlaneModel {
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double offset = 0.0;
};

bool isDynamicLabel(DynamicObjectLabel label)
{
    return label == DynamicObjectLabel::Case1 ||
        label == DynamicObjectLabel::Case2 ||
        label == DynamicObjectLabel::Case3;
}

Eigen::Vector3d worldPoint(const DynamicObjectPoint& point)
{
    return {point.worldX, point.worldY, point.worldZ};
}

VoxelKey voxelKey(const DynamicObjectPoint& point, double voxelSize)
{
    return {
        static_cast<int>(std::floor(double(point.worldX) / voxelSize)),
        static_cast<int>(std::floor(double(point.worldY) / voxelSize)),
        static_cast<int>(std::floor(double(point.worldZ) / voxelSize))
    };
}

double pointPlaneDistance(const DynamicObjectPoint& point, const PlaneModel& plane)
{
    return std::abs(plane.normal.dot(worldPoint(point)) + plane.offset);
}

bool fitPlane(const QVector<DynamicObjectPoint>& points,
              const std::vector<int>& pointIndices,
              PlaneModel* plane)
{
    if (pointIndices.size() < 3) {
        return false;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int index : pointIndices) {
        centroid += worldPoint(points.at(index));
    }
    centroid /= double(pointIndices.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int index : pointIndices) {
        const Eigen::Vector3d centered = worldPoint(points.at(index)) - centroid;
        covariance += centered * centered.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    plane->normal = solver.eigenvectors().col(0).normalized();
    plane->offset = -plane->normal.dot(centroid);
    return true;
}

bool estimateGroundPlane(const QVector<DynamicObjectPoint>& points,
                         std::vector<int> pointIndices,
                         const Eigen::Vector3d& worldUp,
                         double distanceThreshold,
                         double maxAngleDeg,
                         PlaneModel* groundPlane)
{
    if (pointIndices.size() < 4) {
        return false;
    }

    std::sort(pointIndices.begin(), pointIndices.end());
    const int blockSize = std::max(4, int(pointIndices.size() / 100));
    const double minUpDot = std::cos(maxAngleDeg * kPi / 180.0);
    int bestInlierCount = 0;
    PlaneModel bestPlane;

    for (int begin = 0; begin + blockSize <= int(pointIndices.size()); begin += blockSize) {
        const std::vector<int> block(pointIndices.begin() + begin,
                                     pointIndices.begin() + begin + blockSize);
        PlaneModel candidate;
        if (!fitPlane(points, block, &candidate)) {
            continue;
        }
        if (candidate.normal.dot(worldUp) < 0.0) {
            candidate.normal = -candidate.normal;
            candidate.offset = -candidate.offset;
        }
        if (candidate.normal.dot(worldUp) < minUpDot) {
            continue;
        }

        bool planarBlock = true;
        for (int index : block) {
            if (pointPlaneDistance(points.at(index), candidate) > distanceThreshold) {
                planarBlock = false;
                break;
            }
        }
        if (!planarBlock) {
            continue;
        }

        int inlierCount = 0;
        for (int index : pointIndices) {
            if (pointPlaneDistance(points.at(index), candidate) <= distanceThreshold) {
                ++inlierCount;
            }
        }
        if (inlierCount > bestInlierCount) {
            bestInlierCount = inlierCount;
            bestPlane = candidate;
        }
        if (bestInlierCount > int(0.6 * double(pointIndices.size()))) {
            break;
        }
    }

    if (bestInlierCount <= int(0.2 * double(pointIndices.size())) && bestInlierCount <= 500) {
        return false;
    }

    std::vector<int> inliers;
    inliers.reserve(bestInlierCount);
    for (int index : pointIndices) {
        if (pointPlaneDistance(points.at(index), bestPlane) <= distanceThreshold) {
            inliers.push_back(index);
        }
    }
    fitPlane(points, inliers, groundPlane);
    if (groundPlane->normal.dot(worldUp) < 0.0) {
        groundPlane->normal = -groundPlane->normal;
        groundPlane->offset = -groundPlane->offset;
    }
    return groundPlane->normal.dot(worldUp) >= minUpDot;
}

std::vector<VoxelKey> neighborOffsets(int range)
{
    std::vector<VoxelKey> offsets;
    for (int x = -range; x <= range; ++x) {
        for (int y = -range; y <= range; ++y) {
            for (int z = -range; z <= range; ++z) {
                if (x == 0 && y == 0 && z == 0) {
                    continue;
                }
                if (std::sqrt(double(x * x + y * y + z * z)) <= double(range) + 0.001) {
                    offsets.push_back({x, y, z});
                }
            }
        }
    }
    return offsets;
}

std::vector<std::vector<VoxelKey>> connectedComponents(const VoxelSet& voxels,
                                                       const std::vector<VoxelKey>& offsets)
{
    VoxelSet unvisited = voxels;
    std::vector<std::vector<VoxelKey>> components;
    while (!unvisited.empty()) {
        std::queue<VoxelKey> pending;
        std::vector<VoxelKey> component;
        const VoxelKey first = *unvisited.begin();
        unvisited.erase(first);
        pending.push(first);
        while (!pending.empty()) {
            const VoxelKey current = pending.front();
            pending.pop();
            component.push_back(current);
            for (const VoxelKey& offset : offsets) {
                const VoxelKey neighbor{
                    current.x + offset.x,
                    current.y + offset.y,
                    current.z + offset.z
                };
                const auto found = unvisited.find(neighbor);
                if (found != unvisited.end()) {
                    pending.push(neighbor);
                    unvisited.erase(found);
                }
            }
        }
        components.push_back(std::move(component));
    }
    return components;
}

bool touchesObjectVoxel(const VoxelKey& key, const VoxelSet& objectVoxels)
{
    const VoxelKey neighbors[] = {
        {key.x + 1, key.y, key.z}, {key.x - 1, key.y, key.z},
        {key.x, key.y + 1, key.z}, {key.x, key.y - 1, key.z},
        {key.x, key.y, key.z + 1}, {key.x, key.y, key.z - 1}
    };
    for (const VoxelKey& neighbor : neighbors) {
        if (objectVoxels.find(neighbor) != objectVoxels.end()) {
            return true;
        }
    }
    return false;
}

} // namespace

DynamicObjectCluster::DynamicObjectCluster(const DynamicObjectDetectorConfig& config)
    : config_(config)
{
}

DynamicObjectClusterResult DynamicObjectCluster::processFrame(
    const QVector<DynamicObjectPoint>& points,
    const Eigen::Vector3d& worldUp) const
{
    QElapsedTimer timer;
    timer.start();

    DynamicObjectClusterResult result;
    result.labels.fill(DynamicObjectLabel::Static, points.size());

    VoxelPointMap rawVoxels;
    VoxelPointMap eventVoxels;
    for (int index = 0; index < points.size(); ++index) {
        const DynamicObjectPoint& point = points.at(index);
        if (point.label == DynamicObjectLabel::Invalid) {
            result.labels[index] = DynamicObjectLabel::Invalid;
            continue;
        }
        const VoxelKey key = voxelKey(point, config_.clusterVoxelSizeM);
        rawVoxels[key].push_back(index);
        if (isDynamicLabel(point.label)) {
            eventVoxels[key].push_back(index);
        }
    }

    VoxelSet eventVoxelSet;
    eventVoxelSet.reserve(eventVoxels.size());
    for (const auto& entry : eventVoxels) {
        eventVoxelSet.insert(entry.first);
    }
    const std::vector<VoxelKey> offsets = neighborOffsets(config_.clusterExtendVoxel);
    const std::vector<std::vector<VoxelKey>> eventComponents =
        connectedComponents(eventVoxelSet, offsets);

    for (const std::vector<VoxelKey>& component : eventComponents) {
        if (int(component.size()) < config_.clusterMinVoxelCount) {
            ++result.rejectedClusterCount;
            continue;
        }

        VoxelKey minKey = component.front();
        VoxelKey maxKey = component.front();
        int seedPointCount = 0;
        for (const VoxelKey& key : component) {
            minKey.x = std::min(minKey.x, key.x);
            minKey.y = std::min(minKey.y, key.y);
            minKey.z = std::min(minKey.z, key.z);
            maxKey.x = std::max(maxKey.x, key.x);
            maxKey.y = std::max(maxKey.y, key.y);
            maxKey.z = std::max(maxKey.z, key.z);
            seedPointCount += int(eventVoxels.at(key).size());
        }

        const int varyingAxes = int(maxKey.x > minKey.x) +
            int(maxKey.y > minKey.y) + int(maxKey.z > minKey.z);
        if (config_.clusterMinVoxelCount > 1 && varyingAxes < 2) {
            ++result.rejectedClusterCount;
            continue;
        }

        const Eigen::Vector3d boundsMin(
            double(minKey.x) * config_.clusterVoxelSizeM,
            double(minKey.y) * config_.clusterVoxelSizeM,
            double(minKey.z) * config_.clusterVoxelSizeM);
        const Eigen::Vector3d boundsMax(
            double(maxKey.x + 1) * config_.clusterVoxelSizeM,
            double(maxKey.y + 1) * config_.clusterVoxelSizeM,
            double(maxKey.z + 1) * config_.clusterVoxelSizeM);
        const Eigen::Vector3d center = 0.5 * (boundsMin + boundsMax);
        const Eigen::Vector3d size = boundsMax - boundsMin;
        const Eigen::Vector3d outerHalf(
            std::max(1.0, size.x()),
            std::max(1.0, size.y()),
            std::max(1.0, size.z()));

        std::unordered_set<int> clusterPointIndices;
        std::vector<int> groundCandidateIndices;
        VoxelSet objectVoxels;
        VoxelSet surroundingVoxels;
        for (const auto& entry : rawVoxels) {
            const VoxelKey& key = entry.first;
            const Eigen::Vector3d voxelCenter(
                (double(key.x) + 0.5) * config_.clusterVoxelSizeM,
                (double(key.y) + 0.5) * config_.clusterVoxelSizeM,
                (double(key.z) + 0.5) * config_.clusterVoxelSizeM);
            const Eigen::Vector3d centerDistance = (voxelCenter - center).cwiseAbs();
            if ((centerDistance.array() > outerHalf.array()).any()) {
                continue;
            }

            const bool inside = key.x >= minKey.x && key.x <= maxKey.x &&
                key.y >= minKey.y && key.y <= maxKey.y &&
                key.z >= minKey.z && key.z <= maxKey.z;
            if (inside) {
                objectVoxels.insert(key);
                clusterPointIndices.insert(entry.second.begin(), entry.second.end());
            } else {
                surroundingVoxels.insert(key);
                groundCandidateIndices.insert(groundCandidateIndices.end(),
                                              entry.second.begin(),
                                              entry.second.end());
            }
        }

        PlaneModel groundPlane;
        const bool groundDetected = estimateGroundPlane(
            points,
            groundCandidateIndices,
            worldUp.normalized(),
            config_.clusterGroundDistanceThresholdM,
            config_.clusterGroundMaxAngleDeg,
            &groundPlane);

        if (groundDetected) {
            VoxelSet groundVoxels;
            for (const VoxelKey& key : surroundingVoxels) {
                const std::vector<int>& indices = rawVoxels.at(key);
                const bool groundVoxel = std::any_of(
                    indices.begin(), indices.end(), [&](int index) {
                        return pointPlaneDistance(points.at(index), groundPlane) <=
                            config_.clusterGroundDistanceThresholdM;
                    });
                if (groundVoxel) {
                    groundVoxels.insert(key);
                }
            }

            for (const VoxelKey& key : surroundingVoxels) {
                const std::vector<int>& indices = rawVoxels.at(key);
                if (indices.size() <= 2 || groundVoxels.find(key) != groundVoxels.end() ||
                    !touchesObjectVoxel(key, objectVoxels)) {
                    continue;
                }
                PlaneModel localPlane;
                if (!fitPlane(points, indices, &localPlane)) {
                    continue;
                }
                bool planarVoxel = true;
                for (int index : indices) {
                    if (pointPlaneDistance(points.at(index), localPlane) >
                        config_.clusterGroundDistanceThresholdM) {
                        planarVoxel = false;
                        break;
                    }
                }
                if (planarVoxel && std::abs(localPlane.normal.dot(groundPlane.normal)) < 0.8) {
                    objectVoxels.insert(key);
                    clusterPointIndices.insert(indices.begin(), indices.end());
                }
            }

            for (auto it = clusterPointIndices.begin(); it != clusterPointIndices.end();) {
                if (pointPlaneDistance(points.at(*it), groundPlane) <=
                    config_.clusterGroundDistanceThresholdM) {
                    it = clusterPointIndices.erase(it);
                    ++result.groundRemovedPointCount;
                } else {
                    ++it;
                }
            }
        }

        VoxelPointMap clusteredVoxels;
        VoxelSet clusteredVoxelSet;
        for (int index : clusterPointIndices) {
            const VoxelKey key = voxelKey(points.at(index), config_.clusterVoxelSizeM);
            clusteredVoxels[key].push_back(index);
            clusteredVoxelSet.insert(key);
        }
        const std::vector<std::vector<VoxelKey>> isolatedComponents =
            connectedComponents(clusteredVoxelSet, offsets);
        if (isolatedComponents.empty()) {
            ++result.rejectedClusterCount;
            continue;
        }

        const auto largest = std::max_element(
            isolatedComponents.begin(), isolatedComponents.end(),
            [](const std::vector<VoxelKey>& left, const std::vector<VoxelKey>& right) {
                return left.size() < right.size();
            });
        std::vector<int> finalPointIndices;
        for (const VoxelKey& key : *largest) {
            const std::vector<int>& indices = clusteredVoxels.at(key);
            finalPointIndices.insert(finalPointIndices.end(), indices.begin(), indices.end());
        }
        if (finalPointIndices.empty() ||
            double(seedPointCount) / double(finalPointIndices.size()) < config_.clusterTrustThreshold) {
            ++result.rejectedClusterCount;
            continue;
        }

        ++result.clusterCount;
        for (int index : finalPointIndices) {
            result.labels[index] = isDynamicLabel(points.at(index).label)
                ? points.at(index).label
                : DynamicObjectLabel::Case1;
        }
    }

    for (int index = 0; index < points.size(); ++index) {
        if (!isDynamicLabel(result.labels.at(index))) {
            continue;
        }
        DynamicObjectPoint point = points.at(index);
        point.label = result.labels.at(index);
        result.dynamicPoints.push_back(point);
    }
    result.clusterMs = double(timer.nsecsElapsed()) / 1000000.0;
    return result;
}

#pragma once


#include "pcl/point_cloud.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <utility>
#include <vector>

namespace pcl {

template <typename PointT>
class KdTreeFLANN {
public:
    using Ptr = std::shared_ptr<KdTreeFLANN<PointT>>;

    void setInputCloud(const typename PointCloud<PointT>::Ptr& cloud)
    {
        cloud_ = cloud;
        std::vector<int> indices(cloud_->points.size());
        std::iota(indices.begin(), indices.end(), 0);
        root_ = build(indices, 0, static_cast<int>(indices.size()), 0);
    }

    int radiusSearch(const PointT& query,
                     double radius,
                     std::vector<int>& indices,
                     std::vector<float>& squaredDistances,
                     unsigned int maxNearestNeighbors = 0) const
    {
        std::vector<std::pair<float, int>> matches;
        radiusSearch(root_.get(), query, static_cast<float>(radius * radius), matches);
        std::sort(matches.begin(), matches.end());
        if (maxNearestNeighbors > 0 && matches.size() > maxNearestNeighbors) {
            matches.resize(maxNearestNeighbors);
        }
        indices.resize(matches.size());
        squaredDistances.resize(matches.size());
        for (std::size_t i = 0; i < matches.size(); ++i) {
            squaredDistances[i] = matches[i].first;
            indices[i] = matches[i].second;
        }
        return static_cast<int>(matches.size());
    }

    int nearestKSearch(const PointT& query,
                       int k,
                       std::vector<int>& indices,
                       std::vector<float>& squaredDistances) const
    {
        if (k == 1) {
            int bestIndex = -1;
            float bestDistance = std::numeric_limits<float>::infinity();
            nearestOneSearch(root_.get(), query, bestIndex, bestDistance);
            if (bestIndex < 0) {
                indices.clear();
                squaredDistances.clear();
                return 0;
            }
            indices.resize(1);
            squaredDistances.resize(1);
            indices.front() = bestIndex;
            squaredDistances.front() = bestDistance;
            return 1;
        }

        using Candidate = std::pair<float, int>;
        std::priority_queue<Candidate> candidates;
        nearestKSearch(root_.get(), query, k, candidates);

        const int count = static_cast<int>(candidates.size());
        indices.resize(static_cast<std::size_t>(count));
        squaredDistances.resize(static_cast<std::size_t>(count));
        for (int i = count - 1; i >= 0; --i) {
            squaredDistances[static_cast<std::size_t>(i)] = candidates.top().first;
            indices[static_cast<std::size_t>(i)] = candidates.top().second;
            candidates.pop();
        }
        return count;
    }

private:
    struct Node {
        int index = -1;
        int axis = 0;
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;
    };

    static float coordinate(const PointT& point, int axis)
    {
        if (axis == 0) {
            return point.x;
        }
        if (axis == 1) {
            return point.y;
        }
        return point.z;
    }

    static float squaredDistance(const PointT& lhs, const PointT& rhs)
    {
        const float dx = lhs.x - rhs.x;
        const float dy = lhs.y - rhs.y;
        const float dz = lhs.z - rhs.z;
        return dx * dx + dy * dy + dz * dz;
    }

    std::unique_ptr<Node> build(std::vector<int>& indices, int begin, int end, int depth)
    {
        if (begin >= end) {
            return nullptr;
        }
        const int axis = depth % 3;
        const int middle = begin + (end - begin) / 2;
        std::nth_element(indices.begin() + begin,
                         indices.begin() + middle,
                         indices.begin() + end,
                         [this, axis](int lhs, int rhs) {
                             return coordinate(cloud_->points[static_cast<std::size_t>(lhs)], axis) <
                                 coordinate(cloud_->points[static_cast<std::size_t>(rhs)], axis);
                         });
        auto node = std::make_unique<Node>();
        node->index = indices[static_cast<std::size_t>(middle)];
        node->axis = axis;
        node->left = build(indices, begin, middle, depth + 1);
        node->right = build(indices, middle + 1, end, depth + 1);
        return node;
    }

    void radiusSearch(const Node* node,
                      const PointT& query,
                      float squaredRadius,
                      std::vector<std::pair<float, int>>& matches) const
    {
        if (node == nullptr) {
            return;
        }
        const PointT& point = cloud_->points[static_cast<std::size_t>(node->index)];
        const float distance = squaredDistance(query, point);
        if (distance <= squaredRadius) {
            matches.emplace_back(distance, node->index);
        }

        const float delta = coordinate(query, node->axis) - coordinate(point, node->axis);
        const Node* nearNode = delta < 0.0f ? node->left.get() : node->right.get();
        const Node* farNode = delta < 0.0f ? node->right.get() : node->left.get();
        radiusSearch(nearNode, query, squaredRadius, matches);
        if (delta * delta <= squaredRadius) {
            radiusSearch(farNode, query, squaredRadius, matches);
        }
    }

    void nearestKSearch(const Node* node,
                        const PointT& query,
                        int k,
                        std::priority_queue<std::pair<float, int>>& candidates) const
    {
        if (node == nullptr) {
            return;
        }
        const PointT& point = cloud_->points[static_cast<std::size_t>(node->index)];
        const float distance = squaredDistance(query, point);
        if (static_cast<int>(candidates.size()) < k) {
            candidates.emplace(distance, node->index);
        } else if (distance < candidates.top().first) {
            candidates.pop();
            candidates.emplace(distance, node->index);
        }

        const float delta = coordinate(query, node->axis) - coordinate(point, node->axis);
        const Node* nearNode = delta < 0.0f ? node->left.get() : node->right.get();
        const Node* farNode = delta < 0.0f ? node->right.get() : node->left.get();
        nearestKSearch(nearNode, query, k, candidates);
        const float worstDistance = static_cast<int>(candidates.size()) < k
            ? std::numeric_limits<float>::infinity()
            : candidates.top().first;
        if (delta * delta < worstDistance) {
            nearestKSearch(farNode, query, k, candidates);
        }
    }

    void nearestOneSearch(const Node* node,
                          const PointT& query,
                          int& bestIndex,
                          float& bestDistance) const
    {
        if (node == nullptr) {
            return;
        }
        const PointT& point = cloud_->points[static_cast<std::size_t>(node->index)];
        const float distance = squaredDistance(query, point);
        if (distance < bestDistance) {
            bestDistance = distance;
            bestIndex = node->index;
        }

        const float delta = coordinate(query, node->axis) - coordinate(point, node->axis);
        const Node* nearNode = delta < 0.0f ? node->left.get() : node->right.get();
        const Node* farNode = delta < 0.0f ? node->right.get() : node->left.get();
        nearestOneSearch(nearNode, query, bestIndex, bestDistance);
        if (delta * delta < bestDistance) {
            nearestOneSearch(farNode, query, bestIndex, bestDistance);
        }
    }

    typename PointCloud<PointT>::Ptr cloud_;
    std::unique_ptr<Node> root_;
};

} // namespace pcl

#pragma once

#include "pcl/point_cloud.h"

#include <cassert>
#include <cmath>
#include <unordered_map>

namespace pcl {

template <typename PointT>
class VoxelGrid {
public:
    void setLeafSize(float leafX, float leafY, float leafZ)
    {
        leafX_ = leafX;
        leafY_ = leafY;
        leafZ_ = leafZ;
    }

    void setInputCloud(const typename PointCloud<PointT>::Ptr& input)
    {
        input_ = input;
    }

    void filter(PointCloud<PointT>& output) const
    {
        assert(input_);
        assert(leafX_ > 0.0f && leafY_ > 0.0f && leafZ_ > 0.0f);

        struct VoxelKey {
            long long x = 0;
            long long y = 0;
            long long z = 0;

            bool operator==(const VoxelKey& other) const
            {
                return x == other.x && y == other.y && z == other.z;
            }
        };

        struct VoxelKeyHash {
            std::size_t operator()(const VoxelKey& key) const
            {
                std::size_t seed = std::hash<long long>{}(key.x);
                seed ^= std::hash<long long>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                seed ^= std::hash<long long>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                return seed;
            }
        };

        struct Accumulator {
            PointT point;
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            int count = 0;
        };

        std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash> voxels;
        voxels.reserve(input_->points.size());
        for (const PointT& point : input_->points) {
            const VoxelKey key{
                static_cast<long long>(std::floor(point.x / leafX_)),
                static_cast<long long>(std::floor(point.y / leafY_)),
                static_cast<long long>(std::floor(point.z / leafZ_))
            };
            Accumulator& accumulator = voxels[key];
            if (accumulator.count == 0) {
                accumulator.point = point;
            }
            accumulator.x += point.x;
            accumulator.y += point.y;
            accumulator.z += point.z;
            ++accumulator.count;
        }

        output.clear();
        output.reserve(voxels.size());
        for (const auto& entry : voxels) {
            PointT point = entry.second.point;
            const double invCount = 1.0 / static_cast<double>(entry.second.count);
            point.x = static_cast<float>(entry.second.x * invCount);
            point.y = static_cast<float>(entry.second.y * invCount);
            point.z = static_cast<float>(entry.second.z * invCount);
            output.push_back(point);
        }
        output.width = static_cast<uint32_t>(output.points.size());
        output.height = 1;
    }

private:
    typename PointCloud<PointT>::Ptr input_;
    float leafX_ = 1.0f;
    float leafY_ = 1.0f;
    float leafZ_ = 1.0f;
};

} // namespace pcl

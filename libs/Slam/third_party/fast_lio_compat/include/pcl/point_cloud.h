#pragma once

#include <Eigen/Core>

#include <memory>
#include <vector>

namespace pcl {

template <typename PointT>
class PointCloud {
public:
    using Ptr = std::shared_ptr<PointCloud<PointT>>;
    using ConstPtr = std::shared_ptr<const PointCloud<PointT>>;
    using VectorType = std::vector<PointT, Eigen::aligned_allocator<PointT>>;
    using iterator = typename VectorType::iterator;
    using const_iterator = typename VectorType::const_iterator;

    PointCloud() = default;
    PointCloud(std::size_t widthValue, std::size_t heightValue)
        : points(widthValue * heightValue), width(static_cast<uint32_t>(widthValue)), height(static_cast<uint32_t>(heightValue))
    {
    }

    iterator begin() { return points.begin(); }
    iterator end() { return points.end(); }
    const_iterator begin() const { return points.begin(); }
    const_iterator end() const { return points.end(); }

    void clear() { points.clear(); }
    void reserve(std::size_t size) { points.reserve(size); }
    void resize(std::size_t size) { points.resize(size); }
    void push_back(const PointT& point) { points.push_back(point); }
    bool empty() const { return points.empty(); }
    std::size_t size() const { return points.size(); }

    PointT& operator[](std::size_t index) { return points[index]; }
    const PointT& operator[](std::size_t index) const { return points[index]; }

    PointCloud& operator+=(const PointCloud& other)
    {
        points.insert(points.end(), other.points.begin(), other.points.end());
        return *this;
    }

    VectorType points;
    uint32_t width = 0;
    uint32_t height = 1;
};

} // namespace pcl

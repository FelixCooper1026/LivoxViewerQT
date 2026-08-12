#include "FreeDomSnapshotBuilder.h"

#include "freedom/depth_image.h"
#include "freedom/freedom.h"
#include "freedom/mrmap.h"
#include "freedom/scanmap.h"
#include "freedom/utils_core.h"

#include <opencv2/core.hpp>

#include <algorithm>

namespace {

Eigen::Vector3f voxelCenter(const freedom::Index& index, double voxelSize)
{
    return ((index.cast<double>().array() + 0.5) * voxelSize).matrix().cast<float>();
}

freedom::Index localVoxelIndex(unsigned int linearIndex, unsigned int side)
{
    const unsigned int sideSquared = side * side;
    return freedom::Index(
        static_cast<int>(linearIndex / sideSquared),
        static_cast<int>((linearIndex % sideSquared) / side),
        static_cast<int>(linearIndex % side));
}

freedom::Index raycastBlockIndex(std::size_t linearIndex,
                                 const freedom::IndexBias& minimum,
                                 const freedom::IndexBias& size)
{
    const std::size_t yz = static_cast<std::size_t>(size.y()) *
                           static_cast<std::size_t>(size.z());
    const int x = static_cast<int>(linearIndex / yz);
    const std::size_t remainder = linearIndex % yz;
    const int y = static_cast<int>(remainder / static_cast<std::size_t>(size.z()));
    const int z = static_cast<int>(remainder % static_cast<std::size_t>(size.z()));
    return minimum + freedom::Index(x, y, z);
}

void copyImage(const cv::Mat& source, FreeDomImageSnapshot& output)
{
    output.rows = source.rows;
    output.columns = source.cols;
    output.pixels.resize(source.total());
    if(source.isContinuous())
    {
        std::copy(source.ptr<std::uint8_t>(),
                  source.ptr<std::uint8_t>() + source.total(),
                  output.pixels.begin());
        return;
    }
    for(int row = 0; row < source.rows; ++row)
    {
        std::copy(source.ptr<std::uint8_t>(row),
                  source.ptr<std::uint8_t>(row) + source.cols,
                  output.pixels.begin() + static_cast<std::size_t>(row * source.cols));
    }
}

void appendFreeVoxels(const freedom::MRMap& map,
                      std::vector<Eigen::Vector3f>& output)
{
    const unsigned int side = map.getVoxel2blockMultiples();
    const unsigned int count = map.getVoxel2blockMultiplesCubed();
    for(const auto& blockPair : map.get_free_blocks())
    {
        const freedom::FreeBlock& block = blockPair.second;
        for(unsigned int linear = 0; linear < count; ++linear)
        {
            if(!block.getFreeVoxel(linear).is_free)
                continue;
            const freedom::Index worldIndex =
                blockPair.first * static_cast<int>(side) +
                localVoxelIndex(linear, side);
            output.emplace_back(voxelCenter(worldIndex, map.getVoxelSize()));
        }
    }
}

void appendStaticVoxels(const freedom::MRMap& map,
                        std::vector<Eigen::Vector3f>& output)
{
    const unsigned int side = map.getVoxel2blockMultiples();
    const unsigned int count = map.getVoxel2blockMultiplesCubed();
    for(const auto& blockPair : map.get_static_blocks())
    {
        const freedom::StaticBlock& block = blockPair.second;
        for(unsigned int linear = 0; linear < count; ++linear)
        {
            if(!block.is_static_voxel_allocated(linear) ||
               block.getStaticVoxel(linear).static_occ_count == 0)
                continue;
            const freedom::Index worldIndex =
                blockPair.first * static_cast<int>(side) +
                localVoxelIndex(linear, side);
            output.emplace_back(voxelCenter(worldIndex, map.getVoxelSize()));
        }
    }
}

void appendRaycastedVoxels(const freedom::MRMap& map,
                           std::vector<Eigen::Vector3f>& output)
{
    const auto& flags = map.get_raycasted_flags();
    const auto& blocks = map.get_raycast_blocks();
    const freedom::IndexBias minimum = map.get_raycast_map_min_idx();
    const freedom::IndexBias size = map.get_raycast_map_idx_size();
    const unsigned int side = map.getVoxel2blockMultiples();
    const unsigned int count = map.getVoxel2blockMultiplesCubed();
    for(std::size_t blockLinear = 0; blockLinear < flags.size(); ++blockLinear)
    {
        if(!flags[blockLinear].load(std::memory_order_relaxed))
            continue;
        const freedom::Index blockIndex =
            raycastBlockIndex(blockLinear, minimum, size);
        for(unsigned int voxelLinear = 0; voxelLinear < count; ++voxelLinear)
        {
            const auto& words = blocks[blockLinear].trversed_voxels;
            if((words[voxelLinear / 64].load(std::memory_order_relaxed) &
                (1ULL << (voxelLinear % 64))) == 0)
                continue;
            const freedom::Index worldIndex =
                blockIndex * static_cast<int>(side) +
                localVoxelIndex(voxelLinear, side);
            output.emplace_back(voxelCenter(worldIndex, map.getVoxelSize()));
        }
    }
}

} // namespace

void FreeDomSnapshotBuilder::appendScan(const freedom::ScanMap& scan,
                                        FreeDomDebugSnapshot& output)
{
    output.voxelSizeM = static_cast<float>(scan.getVoxelSize());
    output.scanVoxelCenters.clear();
    output.dynamicVoxelCenters.clear();
    for(const freedom::ScanMap::ScanBlock& block : scan.get_scan_blocks())
    {
        for(const freedom::ScanMap::ScanVoxel& voxel : block.scan_voxels)
        {
            const Eigen::Vector3f center = voxelCenter(voxel.voxel_idx, scan.getVoxelSize());
            output.scanVoxelCenters.emplace_back(center);
            if(voxel.dynamic_level != freedom::DynamicLevel::STATIC)
                output.dynamicVoxelCenters.emplace_back(center);
        }
    }
}

void FreeDomSnapshotBuilder::appendDepthImage(const freedom::DepthImage& depthImage,
                                              FreeDomDebugSnapshot& output)
{
    output.enhancedPoints.clear();
    output.enhancedPoints.reserve(depthImage.get_enhanced_pointcloud().size());
    for(const freedom::Point& point : depthImage.get_enhanced_pointcloud())
        output.enhancedPoints.emplace_back(point.cast<float>());
    copyImage(depthImage.get_depth_image(), output.depthImage);
    copyImage(depthImage.get_inpainted_image(), output.enhancedDepthImage);
}

void FreeDomSnapshotBuilder::appendMap(const freedom::MRMap& map,
                                       FreeDomDebugSnapshot& output)
{
    output.raycastedVoxelCenters.clear();
    output.freeVoxelCenters.clear();
    output.staticVoxelCenters.clear();
    appendRaycastedVoxels(map, output.raycastedVoxelCenters);
    appendFreeVoxels(map, output.freeVoxelCenters);
    appendStaticVoxels(map, output.staticVoxelCenters);
}

void FreeDomSnapshotBuilder::buildMap(const freedom::FreeDOM& engine,
                                      std::uint64_t version,
                                      FreeDomMapSnapshot& output)
{
    output = FreeDomMapSnapshot{};
    output.version = version;
    output.voxelSizeM = static_cast<float>(engine.map_state().getVoxelSize());
    freedom::FreeDOM::StaticMapSnapshot coreSnapshot;
    engine.build_static_map_snapshot(coreSnapshot);
    output.staticPoints.reserve(coreSnapshot.point_map.size());
    for(const freedom::Point& point : coreSnapshot.point_map)
        output.staticPoints.emplace_back(point.cast<float>());
    appendStaticVoxels(engine.map_state(), output.staticVoxelCenters);
    appendFreeVoxels(engine.map_state(), output.freeVoxelCenters);
    appendRaycastedVoxels(engine.map_state(), output.raycastedVoxelCenters);
}

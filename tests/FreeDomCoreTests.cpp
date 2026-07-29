#include "FreeDomDynamicPointFilter.h"
#include "freedom/depth_image.h"
#include "freedom/map.h"
#include "freedom/raycast.h"
#include "freedom/scanmap.h"

#include <opencv2/core.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

void expect(bool condition, const std::string& message)
{
    if (condition) {
        return;
    }
    ++failures;
    std::cerr << "FAILED: " << message << '\n';
}

bool sameIndex(const freedom::Index& actual, int x, int y, int z)
{
    return actual.x() == x && actual.y() == y && actual.z() == z;
}

class TestMap final : public freedom::Map
{
public:
    using freedom::Map::set_params;
};

freedom::ScanMap::ScanMapConfig scanConfig(unsigned int threads)
{
    freedom::ScanMap::ScanMapConfig config{};
    config.sub_voxel_size = 0.1;
    config.voxel_depth = 2;
    config.block_depth = 4;
    config.enable_local_map = true;
    config.local_map_range = 20.0;
    config.local_map_min_z = -5.0;
    config.local_map_max_z = 5.0;
    config.num_threads = threads;
    config.sensor_min_range = 0.3;
    config.sensor_max_range = 10.0;
    config.sensor_min_z = -2.0;
    config.sensor_max_z = 2.0;
    return config;
}

pcl::PointXYZ xyz(float x, float y, float z)
{
    pcl::PointXYZ point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

void testMap()
{
    TestMap map;
    freedom::Map::Config config{};
    config.sub_voxel_size = 0.1;
    config.voxel_depth = 2;
    config.block_depth = 4;
    config.enable_local_map = false;
    config.num_threads = 1;
    map.set_params(config);

    expect(std::abs(map.getVoxelSize() - 0.4) < 1e-12, "Map voxel size");
    expect(std::abs(map.getBlockSize() - 1.6) < 1e-12, "Map block size");
    freedom::Index index;
    map.getSubVoxelIdxFromPoint(freedom::Point(-0.001, 0.199, -0.101), index);
    expect(sameIndex(index, -1, 1, -2), "Map floor for negative coordinates");
    map.getVoxelIdxFromPoint(freedom::Point(-0.001, 0.799, -0.401), index);
    expect(sameIndex(index, -1, 1, -2), "Map voxel floor");
    freedom::LinearIndex linear = 0;
    map.getLocalSubVoxelLinearIdxFromSubvoxelIdx(freedom::Index(-1, 1, 2), linear);
    expect(linear == 54, "Map local sub-voxel linear index");
    map.getLocalVoxelLinearIdxFromVoxelIdx(freedom::Index(-1, 1, 2), linear);
    expect(linear == 54, "Map local voxel linear index");

    freedom::Map::RoboCentricMap local;
    local.set_params(4.0, -2.0, 2.0, 1.0);
    const freedom::Index initialCenter = local.center;
    local.setLocalMapBound(freedom::Point(3.1, -2.2, 1.4));
    expect(local.center != initialCenter, "Local map center moves with sensor");
    local.getLocalBlockLinearIdxFromBlockIdx(local.min_idx, linear);
    expect(linear == 0, "Local map minimum has zero linear index");
}

std::vector<freedom::Index> traceRay(const freedom::Point& start,
                                     const freedom::Point& end,
                                     double resolution)
{
    freedom::Index startIndex(
        int(std::floor(start.x() / resolution)),
        int(std::floor(start.y() / resolution)),
        int(std::floor(start.z() / resolution)));
    freedom::Index endIndex(
        int(std::floor(end.x() / resolution)),
        int(std::floor(end.y() / resolution)),
        int(std::floor(end.z() / resolution)));
    freedom::Index current;
    freedom::RayCaster caster(start, startIndex, current, resolution);
    caster.setRayEnd(end, endIndex);
    std::vector<freedom::Index> result;
    do {
        result.push_back(current);
    } while (caster.step());
    return result;
}

void testRayCaster()
{
    const auto positiveX = traceRay({0.1, 0.1, 0.1}, {3.2, 0.1, 0.1}, 1.0);
    expect(positiveX.size() == 3 && sameIndex(positiveX.front(), 0, 0, 0) &&
               sameIndex(positiveX.back(), 2, 0, 0),
           "RayCaster +X traversal and endpoint exclusion");
    const auto negativeX = traceRay({-0.1, 0.1, 0.1}, {-3.2, 0.1, 0.1}, 1.0);
    expect(negativeX.size() == 3 && sameIndex(negativeX.front(), -1, 0, 0) &&
               sameIndex(negativeX.back(), -3, 0, 0),
           "RayCaster -X across negative boundary");
    const auto positiveY = traceRay({0.1, 0.1, 0.1}, {0.1, 2.2, 0.1}, 1.0);
    expect(positiveY.size() == 2 && sameIndex(positiveY.back(), 0, 1, 0),
           "RayCaster +Y");
    const auto negativeZ = traceRay({0.1, 0.1, -0.1}, {0.1, 0.1, -2.2}, 1.0);
    expect(negativeZ.size() == 2 && sameIndex(negativeZ.back(), 0, 0, -2),
           "RayCaster -Z");
    const auto diagonal = traceRay({0.1, 0.1, 0.1}, {2.2, 2.2, 2.2}, 1.0);
    expect(diagonal.size() == 6 && sameIndex(diagonal.back(), 1, 2, 2),
           "RayCaster diagonal supercover");
    const auto sameVoxel = traceRay({0.1, 0.1, 0.1}, {0.8, 0.8, 0.8}, 1.0);
    expect(sameVoxel.size() == 1 && sameIndex(sameVoxel.front(), 0, 0, 0),
           "RayCaster same voxel");
}

std::vector<freedom::Index> scanVoxelIndices(const freedom::ScanMap& scan)
{
    std::vector<freedom::Index> indices;
    for (const auto& block : scan.get_scan_blocks()) {
        for (const auto& voxel : block.scan_voxels) {
            indices.push_back(voxel.voxel_idx);
        }
    }
    std::sort(indices.begin(), indices.end(), [](const auto& left, const auto& right) {
        if (left.x() != right.x()) return left.x() < right.x();
        if (left.y() != right.y()) return left.y() < right.y();
        return left.z() < right.z();
    });
    return indices;
}

void testScanMap()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points = {
        xyz(0.1f, 0.0f, 0.0f),
        xyz(1.0f, 0.0f, 0.0f),
        xyz(2.0f, 0.0f, 3.0f),
        xyz(-1.0f, 0.0f, 0.0f),
        xyz(2.0f, 0.2f, 0.0f)};
    cloud.width = std::uint32_t(cloud.points.size());
    cloud.height = 1;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(10.0, -2.0, 1.0);

    freedom::ScanMap singleThread;
    singleThread.set_params(scanConfig(1));
    singleThread.build_scan_map(cloud, transform);
    expect((singleThread.get_sensor_origin() - transform.translation()).norm() < 1e-12,
           "ScanMap world transform sensor origin");
    std::vector<freedom::DynamicLevel> levels;
    std::vector<bool> valid;
    singleThread.write_point_dynamic_levels(cloud.size(), levels, valid);
    expect(valid.size() == cloud.size() && !valid[0] && valid[1] && !valid[2] &&
               valid[3] && valid[4],
           "ScanMap range/z filtering and source indices");
    singleThread.get_scan_blocks().front().scan_voxels.front().dynamic_level =
        freedom::DynamicLevel::AGGRESSIVE_DYNAMIC;
    singleThread.write_point_dynamic_levels(cloud.size(), levels, valid);
    int dynamicCount = 0;
    for (std::size_t index = 0; index < levels.size(); ++index) {
        if (valid[index] && levels[index] != freedom::DynamicLevel::STATIC) {
            ++dynamicCount;
        }
    }
    expect(dynamicCount > 0, "ScanMap voxel level maps back to source points");

    freedom::ScanMap parallel;
    parallel.set_params(scanConfig(4));
    parallel.build_scan_map(cloud, transform);
    const auto expectedIndices = scanVoxelIndices(singleThread);
    expect(scanVoxelIndices(parallel) == expectedIndices,
           "ScanMap single-thread and parallel voxel equivalence");
    parallel.reset();
    parallel.build_scan_map(cloud, transform);
    expect(scanVoxelIndices(parallel) == expectedIndices,
           "ScanMap reset and reuse");
}

void testDepthImage()
{
    freedom::DepthImage image;
    const double pi = std::acos(-1.0);
    freedom::DepthImage::DepthImageConfig config{
        2.0 * pi, 0.6, -0.6, 24,
        0.3, 20.0, 0.2,
        3, 0, 0.0, 0.0,
        false, false, std::string(), 2};
    image.set_params(config);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points = {
        xyz(5.0f, 0.01f, 0.0f),
        xyz(5.0f, -0.01f, 0.0f),
        xyz(-5.0f, 0.01f, 0.0f),
        xyz(-5.0f, -0.01f, 0.0f),
        xyz(4.0f, 0.0f, 1.0f)};
    cloud.width = std::uint32_t(cloud.points.size());
    cloud.height = 1;
    image.raycast_enhancement(cloud, Eigen::Isometry3d::Identity());
    expect(image.get_depth_image().rows == 24 && image.get_depth_image().cols > 100,
           "DepthImage configured dimensions");
    expect(cv::countNonZero(image.get_depth_image()) >= 4,
           "DepthImage projection, quantization and panorama wrap");
    expect(image.get_inpainted_image().size() == image.get_depth_image().size(),
           "DepthImage inpaint output dimensions");
}

void testFreeDomAdapter()
{
    DynamicFilterRuntimeConfig config;
    config.backend = DynamicFilterBackend::FreeDOM;
    config.debugVisualizationEnabled = true;
    config.debugSnapshotIntervalFrames = 1;
    config.mapSnapshotIntervalFrames = 1;
    config.freeDom.raycastEnhancementEnabled = false;
    config.freeDom.numThreads = 2;

    FreeDomDynamicPointFilter filter;
    std::string error;
    expect(filter.configure(config, &error), "FreeDOM adapter valid configuration");

    pcl::PointCloud<pcl::PointXYZINormal> cloud;
    for (int y = -2; y <= 2; ++y) {
        for (int z = -1; z <= 1; ++z) {
            pcl::PointXYZINormal point;
            point.x = 4.0f;
            point.y = float(y) * 0.3f;
            point.z = float(z) * 0.3f;
            point.intensity = 80.0f;
            cloud.points.push_back(point);
        }
    }
    cloud.width = std::uint32_t(cloud.points.size());
    cloud.height = 1;
    DynamicFilterFrame frame;
    frame.timestampNs = 100000000;
    frame.lidarFrameCloud = &cloud;
    frame.worldFromLidar = Eigen::Isometry3d::Identity();
    frame.worldFromBody = Eigen::Isometry3d::Identity();
    frame.bodyFromLidar = Eigen::Isometry3d::Identity();
    DynamicFilterResult result;
    expect(filter.processFrame(frame, &result, &error), "FreeDOM adapter processes frame");
    expect(result.labels.size() == cloud.size(), "FreeDOM per-input-point label count");
    expect(result.stats.backend == DynamicFilterBackend::FreeDOM,
           "FreeDOM adapter reports backend");
    expect(result.freeDomDebugSnapshot && result.freeDomMapSnapshot,
           "FreeDOM versioned debug/map snapshots");

    DynamicFilterRuntimeConfig invalid = config;
    invalid.freeDom.blockDepth = invalid.freeDom.voxelDepth - 1;
    expect(!filter.configure(invalid, &error), "FreeDOM rejects blockDepth < voxelDepth");
    invalid = config;
    invalid.freeDom.fovMaskEnabled = true;
    invalid.freeDom.fovMaskPath = "missing_freedom_mask.png";
    expect(!filter.configure(invalid, &error), "FreeDOM rejects missing FOV mask");
}

void testFovLearningWithoutRaycastEnhancement()
{
    const std::filesystem::path outputPath = "freedom_fov_learning_test.png";
    std::filesystem::remove(outputPath);

    pcl::PointCloud<pcl::PointXYZINormal> cloud;
    pcl::PointXYZINormal point;
    point.x = 4.0f;
    cloud.points.push_back(point);
    cloud.width = 1;
    cloud.height = 1;

    DynamicFilterFrame frame;
    frame.timestampNs = 100000000;
    frame.lidarFrameCloud = &cloud;
    frame.worldFromLidar = Eigen::Isometry3d::Identity();
    frame.worldFromBody = Eigen::Isometry3d::Identity();
    frame.bodyFromLidar = Eigen::Isometry3d::Identity();

    {
        DynamicFilterRuntimeConfig config;
        config.backend = DynamicFilterBackend::FreeDOM;
        config.freeDom.raycastEnhancementEnabled = false;
        config.freeDom.learnFov = true;
        config.freeDom.fovMaskPath = outputPath.string();
        config.freeDom.numThreads = 1;

        FreeDomDynamicPointFilter filter;
        DynamicFilterResult result;
        std::string error;
        expect(filter.configure(config, &error),
               "FreeDOM configures FOV learning without raycast enhancement");
        expect(filter.processFrame(frame, &result, &error),
               "FreeDOM learns FOV without raycast enhancement");
    }

    expect(std::filesystem::exists(outputPath),
           "FreeDOM writes the learned FOV mask when the backend is destroyed");
    std::filesystem::remove(outputPath);
}

} // namespace

int main()
{
    testMap();
    testRayCaster();
    testScanMap();
    testDepthImage();
    testFreeDomAdapter();
    testFovLearningWithoutRaycastEnhancement();
    if (failures != 0) {
        std::cerr << failures << " FreeDOM test(s) failed\n";
        return 1;
    }
    std::cout << "All FreeDOM core and adapter tests passed\n";
    return 0;
}

#ifndef _FREEDOM_H
#define _FREEDOM_H

#include <pcl/point_cloud.h>
#include <Eigen/Eigen>
#include <future>
#include <functional>
#include <vector>

#include "freedom/utils_core.h"
#include "freedom/common_types.h"
#include "freedom/mrmap.h"
#include "freedom/scanmap.h"
#include "freedom/depth_image.h"

namespace freedom{
class FreeDOM{
public:
    struct StageTimings
    {
        double build_scan_map_ms = 0.0;
        double scan_removal_ms = 0.0;
        double raycast_enhancement_ms = 0.0;
        double free_space_estimation_ms = 0.0;
        double map_removal_ms = 0.0;
        double static_integration_ms = 0.0;
        double remove_out_of_bound_ms = 0.0;
        double reset_ms = 0.0;
    };

    struct FrameStats
    {
        std::size_t input_point_count = 0;
        std::size_t valid_point_count = 0;
        std::size_t static_point_count = 0;
        std::size_t aggressive_point_count = 0;
        std::size_t moderate_point_count = 0;
        std::size_t conservative_point_count = 0;
        std::size_t invalid_point_count = 0;
        std::size_t scan_voxel_count = 0;
        std::size_t new_free_voxel_count = 0;
        std::size_t free_block_count = 0;
        std::size_t free_voxel_count = 0;
        std::size_t static_block_count = 0;
        std::size_t static_voxel_count = 0;
        std::size_t static_subvoxel_count = 0;
        StageTimings timings;
    };

    struct StaticMapSnapshot
    {
        Points point_map;
        Points voxel_map;
    };

    struct Config
    {
        double sensor_min_range;
        double sensor_max_range;
        double sensor_min_z;
        double sensor_max_z;

        double sub_voxel_size;
        unsigned int voxel_depth;
        unsigned int block_depth;

        bool enable_local_map;
        double local_map_range;
        double local_map_min_z;
        double local_map_max_z;

        double raycast_max_range;
        double raycast_min_z;
        double raycast_max_z;

        unsigned int counts_to_free;
        unsigned int counts_to_revert;

        unsigned int conservative_connectivity;
        unsigned int aggressive_connectivity;

        bool enable_raycast_enhancement;

        double lidar_horizon_fov;
        double lidar_vertical_fov_upper;
        double lidar_vertical_fov_lower;
        unsigned int depth_image_vertical_lines;

        double depth_image_min_range;
        double max_raycast_enhancement_range;
        double raycast_enhancement_depth_margin;

        unsigned int inpaint_size;
        unsigned int erosion_size;
        double min_raycast_enhancement_area;
        double depth_image_top_margin;

        bool learn_fov;
        bool enable_fov_mask;
        std::string fov_mask_path;

        unsigned int num_threads;
    };

    FreeDOM(){}

    void set_params(const Config& config);
    void set_scan_removal_callback(std::function<void(const ScanMap&)> callback);
    void set_raycast_enhancement_callback(std::function<void(const DepthImage&)> callback);
    void set_map_removal_callback(std::function<void(const MRMap&)> callback);

    void pointcloud_integrate(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform);
    void build_static_map_snapshot(StaticMapSnapshot& output) const;

    const std::vector<DynamicLevel>& last_point_dynamic_levels() const noexcept
    {
        return last_point_dynamic_levels_;
    }
    const std::vector<bool>& last_point_validity() const noexcept
    {
        return last_point_validity_;
    }
    const FrameStats& last_frame_stats() const noexcept { return last_frame_stats_; }
    const MRMap& map_state() const noexcept { return map; }
    const DepthImage& depth_image_state() const noexcept { return depth_image; }

private:
    // params
    bool enable_raycast_enhancement;

    // variables
    ScanMap scan;
    DepthImage depth_image;
    MRMap map;

    unsigned int scan_seq;
    std::vector<DynamicLevel> last_point_dynamic_levels_;
    std::vector<bool> last_point_validity_;
    FrameStats last_frame_stats_;

    // scan-removal callback
    std::function<void(const ScanMap&)> scan_removal_callback;
    // raycast-enhancement callback
    std::function<void(const DepthImage&)> raycast_enhancement_callback;
    // map-removal callback
    std::function<void(const MRMap&)> map_removal_callback;

    Timer timer;
};
}
#endif

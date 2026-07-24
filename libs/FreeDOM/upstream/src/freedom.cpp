#include "freedom/freedom.h"

namespace freedom{
void FreeDOM::set_params(const Config& config)
{
    // set params
    ScanMap::ScanMapConfig scan_config{
        config.sub_voxel_size,
        config.voxel_depth,
        config.block_depth,
        config.enable_local_map,
        config.local_map_range,
        config.local_map_min_z,
        config.local_map_max_z,
        config.num_threads,
        config.sensor_min_range,
        config.sensor_max_range,
        config.sensor_min_z,
        config.sensor_max_z
    };
    scan.set_params(scan_config);
    
    MRMap::MRMapConfig map_config{
        config.sub_voxel_size,
        config.voxel_depth,
        config.block_depth,
        config.enable_local_map,
        config.local_map_range,
        config.local_map_min_z,
        config.local_map_max_z,
        config.num_threads,
        config.sensor_max_range,
        config.sensor_min_z,
        config.sensor_max_z,
        config.raycast_max_range,
        config.raycast_min_z,
        config.raycast_max_z,
        config.counts_to_free,
        config.counts_to_revert,
        config.conservative_connectivity,
        config.aggressive_connectivity
    };
    map.set_params(map_config);

    enable_raycast_enhancement = config.enable_raycast_enhancement;
    if(enable_raycast_enhancement)
    {
        DepthImage::DepthImageConfig depth_image_config{
            config.lidar_horizon_fov,
            config.lidar_vertical_fov_upper,
            config.lidar_vertical_fov_lower,
            config.depth_image_vertical_lines,
            config.depth_image_min_range,
            config.max_raycast_enhancement_range,
            config.raycast_enhancement_depth_margin,
            config.inpaint_size,
            config.erosion_size,
            config.min_raycast_enhancement_area,
            config.depth_image_top_margin,
            config.learn_fov,
            config.enable_fov_mask,
            config.fov_mask_path,
            config.num_threads
        };
        depth_image.set_params(depth_image_config);
    }

    // init variables
    scan_seq = 0;
}

void FreeDOM::set_scan_removal_callback(std::function<void(const ScanMap&)> callback)
{
    scan_removal_callback = callback;
}

void FreeDOM::set_raycast_enhancement_callback(std::function<void(const DepthImage&)> callback)
{
    raycast_enhancement_callback = callback;
}

void FreeDOM::set_map_removal_callback(std::function<void(const MRMap&)> callback)
{
    map_removal_callback = callback;
}

void FreeDOM::pointcloud_integrate(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform)
{
    last_frame_stats_ = FrameStats{};
    last_frame_stats_.input_point_count = cloud.size();

    // 构建scan map,为了快速随机查找与快速遍历
    timer["build scan map"].start();
    scan.build_scan_map(cloud,transform);
    last_frame_stats_.timings.build_scan_map_ms = timer["build scan map"].stop();

    // 遍历scan voxel，查找对应的free voxel并设置scan voxel为CONSERVATIVE_DYNAMIC且加入occ_in_freespace
    // 对occ_in_freespace进行聚类并完善其DynamicLevel
    timer["scan removal"].start();
    map.scan_removal(scan);
    last_frame_stats_.timings.scan_removal_ms = timer["scan removal"].stop();

    scan.write_point_dynamic_levels(
        cloud.size(), last_point_dynamic_levels_, last_point_validity_);
    for(std::size_t index = 0; index < last_point_dynamic_levels_.size(); ++index)
    {
        if(!last_point_validity_[index])
        {
            ++ last_frame_stats_.invalid_point_count;
            continue;
        }
        ++ last_frame_stats_.valid_point_count;
        switch(last_point_dynamic_levels_[index])
        {
        case DynamicLevel::AGGRESSIVE_DYNAMIC:
            ++ last_frame_stats_.aggressive_point_count;
            break;
        case DynamicLevel::MODERATE_DYNAMIC:
            ++ last_frame_stats_.moderate_point_count;
            break;
        case DynamicLevel::CONSERVATIVE_DYNAMIC:
            ++ last_frame_stats_.conservative_point_count;
            break;
        case DynamicLevel::STATIC:
            ++ last_frame_stats_.static_point_count;
            break;
        }
    }
    for(const ScanMap::ScanBlock& scan_block : scan.get_scan_blocks())
        last_frame_stats_.scan_voxel_count += scan_block.scan_voxels.size();

    // 滤除当前帧动态点云后立即触发回调
    if(scan_removal_callback)
        scan_removal_callback(scan);

    if(enable_raycast_enhancement)
    {
        timer["raycast enhancement"].start();
        depth_image.raycast_enhancement(cloud,transform);
        last_frame_stats_.timings.raycast_enhancement_ms =
            timer["raycast enhancement"].stop();

        if(raycast_enhancement_callback)
            raycast_enhancement_callback(depth_image);
    }

    Indices freespace_incremental;
    // FreeSpace estimation
    timer["raycast"].start();
    map.freespace_estimation(scan,depth_image,freespace_incremental);
    last_frame_stats_.timings.free_space_estimation_ms = timer["raycast"].stop();
    last_frame_stats_.new_free_voxel_count = freespace_incremental.size();

    // map clearing
    timer["map removal"].start();
    map.map_removal(freespace_incremental);
    last_frame_stats_.timings.map_removal_ms = timer["map removal"].stop();

    // map integration
    timer["staticspace integration"].start();
    map.staticspace_integration(scan,scan_seq);
    last_frame_stats_.timings.static_integration_ms =
        timer["staticspace integration"].stop();

    timer["remove map out of bound"].start();
    map.remove_map_out_of_bound();
    last_frame_stats_.timings.remove_out_of_bound_ms =
        timer["remove map out of bound"].stop();

    last_frame_stats_.free_block_count = map.get_free_blocks().size();
    for(const auto& block_pair : map.get_free_blocks())
        last_frame_stats_.free_voxel_count += block_pair.second.free_count;
    last_frame_stats_.static_block_count = map.get_static_blocks().size();
    for(const auto& block_pair : map.get_static_blocks())
    {
        const StaticBlock& static_block = block_pair.second;
        last_frame_stats_.static_voxel_count += static_block.occ_count;
        for(unsigned int voxel_index = 0;
            voxel_index < map.getVoxel2blockMultiplesCubed();
            ++voxel_index)
        {
            if(static_block.is_static_voxel_allocated(voxel_index))
                last_frame_stats_.static_subvoxel_count +=
                    static_block.getStaticVoxel(voxel_index).occ_count;
        }
    }

    // 集成地图后立即触发回调
    if(map_removal_callback)
        map_removal_callback(map);

    timer["reset"].start();
    scan.reset();
    map.reset();
    last_frame_stats_.timings.reset_ms = timer["reset"].stop();
    ++ scan_seq;
}

void FreeDOM::build_static_map_snapshot(StaticMapSnapshot& output) const
{
    output.point_map.clear();
    output.voxel_map.clear();

    unsigned int block_idx_size = map.getVoxel2blockMultiples();
    unsigned int voxel_idx_size = map.getSubvoxel2voxelMultiples();
    unsigned int voxel_num = map.getVoxel2blockMultiplesCubed();
    unsigned int sub_voxel_num = map.getSubvoxel2voxelMultiplesCubed();

    double block_size = map.getBlockSize();
    double voxel_size = map.getVoxelSize();
    double sub_voxel_size = map.getSubVoxelSize();
    PointBias half_sub_voxel_bias = PointBias(sub_voxel_size,sub_voxel_size,sub_voxel_size) / 2.0;

    for(const auto& block_pair : map.get_static_blocks())
    {
        PointBias block_bias = block_size * block_pair.first.cast<double>();
        const StaticBlock& static_block = block_pair.second;

        Index local_voxel_idx(0,0,0);
        for(unsigned int i = 0; i < voxel_num; ++i)
        {
            if(!static_block.is_static_voxel_allocated(i))
            {
                incrementIdx(local_voxel_idx,block_idx_size);
                continue;
            }

            const StaticVoxel& static_voxel = block_pair.second.getStaticVoxel(i);

            PointBias voxel_bias = voxel_size * local_voxel_idx.cast<double>();

            Index local_subvoxel_idx(0,0,0);
            for(unsigned int j = 0; j < sub_voxel_num; ++j)
            {
                if(static_voxel.scan_in_subvoxel[j] == StaticVoxel::NOT_A_SCAN || static_voxel.dynamic_level[j] > DynamicLevel::STATIC)
                {
                    incrementIdx(local_subvoxel_idx,voxel_idx_size);
                    continue;
                }

                PointBias subvoxel_bias = sub_voxel_size * local_subvoxel_idx.cast<double>();
                Point point_voxel = block_bias + voxel_bias + subvoxel_bias + half_sub_voxel_bias;
                Point point = block_bias + voxel_bias + static_voxel.points[j].cast<double>();

                output.voxel_map.emplace_back(point_voxel);
                output.point_map.emplace_back(point);
                incrementIdx(local_subvoxel_idx,voxel_idx_size);
            }
            incrementIdx(local_voxel_idx,block_idx_size);
        }
    }

}
}

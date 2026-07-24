#ifndef FREEDOM_FREEDOMFRAMERESULT_H
#define FREEDOM_FREEDOMFRAMERESULT_H

#include "freedom/freedom.h"

#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <vector>

struct FreeDomImageSnapshot {
    int rows = 0;
    int columns = 0;
    std::vector<std::uint8_t> pixels;
};

struct FreeDomDebugSnapshot {
    std::uint64_t version = 0;
    std::vector<Eigen::Vector3f> scanVoxelCenters;
    std::vector<Eigen::Vector3f> dynamicVoxelCenters;
    std::vector<Eigen::Vector3f> raycastedVoxelCenters;
    std::vector<Eigen::Vector3f> freeVoxelCenters;
    std::vector<Eigen::Vector3f> staticVoxelCenters;
    std::vector<Eigen::Vector3f> staticSubvoxelPoints;
    std::vector<Eigen::Vector3f> enhancedPoints;
    FreeDomImageSnapshot depthImage;
    FreeDomImageSnapshot enhancedDepthImage;
};

struct FreeDomMapSnapshot {
    std::uint64_t version = 0;
    std::vector<Eigen::Vector3f> staticPoints;
    std::vector<Eigen::Vector3f> staticVoxelCenters;
    std::vector<Eigen::Vector3f> freeVoxelCenters;
    std::vector<Eigen::Vector3f> raycastedVoxelCenters;
};

struct FreeDomFrameResult {
    std::int64_t timestampNs = 0;
    std::vector<freedom::DynamicLevel> levels;
    std::vector<bool> valid;
    freedom::FreeDOM::FrameStats stats;
    std::shared_ptr<const FreeDomDebugSnapshot> debugSnapshot;
};

#endif // FREEDOM_FREEDOMFRAMERESULT_H

#ifndef DYNAMICFILTER_DYNAMICFILTERTYPES_H
#define DYNAMICFILTER_DYNAMICFILTERTYPES_H

#include "DynamicFilterBackend.h"
#include "DynamicObjectDetector.h"
#include "FreeDomFrameResult.h"
#include "FreeDomRuntimeConfig.h"

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <memory>
#include <vector>

enum class DynamicFilterReason : std::uint8_t {
    Static = 0,
    Invalid = 1,
    MDetectorCase1 = 10,
    MDetectorCase2 = 11,
    MDetectorCase3 = 12,
    FreeDomAggressive = 20,
    FreeDomModerate = 21,
    FreeDomConservative = 22
};

struct DynamicFilterLabel {
    bool dynamic = false;
    DynamicFilterReason reason = DynamicFilterReason::Static;
};

struct DynamicFilterPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    std::uint8_t reflectivity = 0;
    DynamicFilterReason reason = DynamicFilterReason::Static;
};

struct DynamicFilterFrame {
    std::int64_t timestampNs = 0;
    const pcl::PointCloud<pcl::PointXYZINormal>* lidarFrameCloud = nullptr;
    Eigen::Isometry3d worldFromLidar = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d worldFromBody = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d bodyFromLidar = Eigen::Isometry3d::Identity();
};

struct DynamicFilterStats {
    DynamicFilterBackend backend = DynamicFilterBackend::Disabled;
    bool enabled = false;
    bool clusterEnabled = false;
    int inputPointCount = 0;
    int staticPointCount = 0;
    int dynamicPointCount = 0;
    int invalidPointCount = 0;
    int case1PointCount = 0;
    int case2PointCount = 0;
    int case3PointCount = 0;
    int originDynamicPointCount = 0;
    int clusterCount = 0;
    int rejectedClusterCount = 0;
    int groundRemovedPointCount = 0;
    int historyDepthMapCount = 0;
    double totalMs = 0.0;
    double detectorMs = 0.0;
    double clusterMs = 0.0;
    freedom::FreeDOM::FrameStats freeDom;
};

struct DynamicFilterResult {
    std::int64_t timestampNs = 0;
    std::vector<DynamicFilterLabel> labels;
    std::vector<DynamicFilterPoint> dynamicWorldPoints;
    DynamicFilterStats stats;
    std::shared_ptr<const FreeDomDebugSnapshot> freeDomDebugSnapshot;
    std::shared_ptr<const FreeDomMapSnapshot> freeDomMapSnapshot;
};

struct DynamicFilterRuntimeConfig {
    DynamicFilterBackend backend = DynamicFilterBackend::Disabled;
    DynamicObjectDetectorConfig mDetector;
    FreeDomRuntimeConfig freeDom;
    bool debugVisualizationEnabled = false;
    unsigned int debugSnapshotIntervalFrames = 5;
    unsigned int mapSnapshotIntervalFrames = 10;
};

#endif // DYNAMICFILTER_DYNAMICFILTERTYPES_H

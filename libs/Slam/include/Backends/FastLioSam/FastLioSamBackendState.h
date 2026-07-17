#ifndef SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMBACKENDSTATE_H
#define SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMBACKENDSTATE_H

#include <omp.h>

#include "Backends/FastLioSam/FastLioSamTypes.h"
#include "Core/SlamRuntimeConfig.h"
#include "Core/SlamTypes.h"
#include "DynamicObjectDetector.h"
#include "IMU_Processing.hpp"
#include "ikd-Tree/ikd_Tree.h"
#include "use-ikfom.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <QVector>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

struct FastLioSamBackendState {
    FastLioSamBackendState();

    std::vector<PointCloudXYZI::Ptr> cornerCloudKeyFrames;
    std::vector<PointCloudXYZI::Ptr> surfCloudKeyFrames;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copyCloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copyCloudKeyPoses6D;
    pcl::PointCloud<PointTypePose>::Ptr fastlioUnoptimizedCloudKeyPoses6D;

    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::Values optimizedEstimate;
    std::unique_ptr<gtsam::ISAM2> isam;
    gtsam::Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    bool aLoopIsClosed = false;
    bool potentialLoopFlag = false;
    std::map<int, int> loopIndexContainer;
    std::vector<std::pair<int, int>> loopIndexQueue;
    std::vector<gtsam::Pose3> loopPoseQueue;
    std::vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;

    float transformTobeMapped[6] = {};
    bool reconstructKdTree = false;
    int updateKdtreeCount = 0;

    std::mutex poseMutex;
    std::mutex loopInfoMutex;
    std::atomic_bool stopRequested{false};
    std::thread loopThread;
    QVector<SlamLoopClosureEdge> pendingLoopClosureEdges;
};

struct FastLioAlgorithmState {
    explicit FastLioAlgorithmState(const SlamRuntimeConfig& runtimeConfig);

    SlamRuntimeConfig config;
    FastLioSamBackendState sam;
    double filterSizeSurf = 0.5;
    double filterSizeMap = 0.5;
    double cubeLen = 200.0;
    float detRange = 300.0f;
    double fovDeg = 180.0;
    double fovDegUsed = 179.9;
    double halfFovCos = 0.0;
    double totalResidual = 0.0;
    double resMeanLast = 0.05;
    double firstLidarTime = 0.0;
    bool firstScan = true;
    bool ekfInited = false;
    bool localMapInitialized = false;
    bool lidarOnly = false;
    bool currentFrameHasPointOffsetTime = false;
    bool hasLoReferencePose = false;
    int64_t loReferenceTimestampNs = 0;
    V3D loReferencePosition = Zero3d;
    M3D loReferenceRotation = Eye3d;
    int effectiveFeatureNum = 0;
    int featsDownSize = 0;
    int trajectoryPointCount = 0;
    int globalMapPointCount = 0;
    int unoptimizedPoseFrameCount = 0;

    PointCloudXYZI::Ptr featsFromMap;
    PointCloudXYZI::Ptr featsUndistort;
    PointCloudXYZI::Ptr featsDownBody;
    PointCloudXYZI::Ptr featsDownWorld;
    PointCloudXYZI::Ptr normvec;
    PointCloudXYZI::Ptr laserCloudOri;
    PointCloudXYZI::Ptr corrNormvect;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;
    KD_TREE<PointType> ikdtree;

    V3F xAxisPointBody;
    V3F xAxisPointWorld;
    V3D eulerCur;
    V3D positionLast;
    V3D lidarTWrImu;
    M3D lidarRWrImu;
    vect3 posLid;
    MeasureGroup measures;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
    state_ikfom statePoint;
    BoxPointType localMapPoints;
    std::vector<BoxPointType> cubNeedRemove;
    std::vector<PointVector> nearestPoints;
    std::vector<char> pointSelectedSurf;
    std::vector<float> resLast;
    std::unique_ptr<ImuProcess> imuProcessor;
    std::unique_ptr<DynamicObjectDetector> dynamicObjectDetector;

    QVector<SlamTrajectoryPoint> optimizedPath;
    QVector<SlamTrajectoryPoint> pendingOptimizedTrajectory;
    QVector<SlamPoint> pendingOptimizedGlobalMapPoints;
    bool optimizedTrajectoryResetPending = false;
    bool optimizedGlobalMapResetPending = false;

    static void hShareModel(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfomData);
};

#endif // SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMBACKENDSTATE_H

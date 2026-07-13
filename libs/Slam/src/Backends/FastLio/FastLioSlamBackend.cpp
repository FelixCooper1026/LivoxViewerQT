#include "Backends/FastLio/FastLioSlamBackend.h"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <omp.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

#include <pcl/filters/voxel_grid.h>

#include "DynamicObjectDetector.h"
#include "IMU_Processing.hpp"
#include "ikd-Tree/ikd_Tree.h"
#include "use-ikfom.hpp"

namespace {

constexpr double kNsToSeconds = 1.0e-9;
constexpr double kNsToMilliseconds = 1.0e-6;
constexpr double kInitTime = 0.1;
constexpr double kLaserPointCov = 0.001;
constexpr double kLoFixedStateCov = 1.0e-9;
constexpr double kLoInitialPoseCov = 1.0e-3;
constexpr double kLoInitialMotionCov = 1.0;
constexpr float kMoveThreshold = 1.5f;
constexpr int kMinMapInitPoints = 5;

void assignError(QString* error, const QString& message);

bool isFiniteArray(const double* values, int count)
{
    for (int i = 0; i < count; ++i) {
        if (!std::isfinite(values[i])) {
            return false;
        }
    }
    return true;
}

bool isUsableRotationMatrix(const double* values)
{
    if (!isFiniteArray(values, 9)) {
        return false;
    }
    for (int row = 0; row < 3; ++row) {
        const double x = values[row * 3 + 0];
        const double y = values[row * 3 + 1];
        const double z = values[row * 3 + 2];
        if ((x * x + y * y + z * z) <= 1.0e-9) {
            return false;
        }
    }
    return true;
}

bool validateRuntimeConfig(const SlamRuntimeConfig& config, QString* error)
{
    if (!std::isfinite(config.gravityNorm) || config.gravityNorm <= 0.0) {
        assignError(error, QStringLiteral("SLAM 配置无效：重力加速度必须大于 0。"));
        return false;
    }
    if (!std::isfinite(config.filterSizeSurfM) || config.filterSizeSurfM <= 0.0 ||
        !std::isfinite(config.filterSizeMapM) || config.filterSizeMapM <= 0.0) {
        assignError(error, QStringLiteral("SLAM 配置无效：FAST_LIO 滤波体素尺寸必须大于 0。"));
        return false;
    }
    if (!std::isfinite(config.cubeSideLengthM) || config.cubeSideLengthM <= 0.0 ||
        !std::isfinite(config.detRangeM) || config.detRangeM <= 0.0 ||
        !std::isfinite(config.fovDegree) || config.fovDegree <= 0.0 ||
        !std::isfinite(config.blindMinRangeM) || config.blindMinRangeM < 0.0 ||
        config.pointFilterNum <= 0 ||
        config.maxIterations <= 0) {
        assignError(error, QStringLiteral("SLAM 配置无效：FAST_LIO 局部地图、视场角、探测距离、近距离盲区和最大迭代次数必须有效。"));
        return false;
    }
    if (!std::isfinite(config.gyrCov) || config.gyrCov <= 0.0 ||
        !std::isfinite(config.accCov) || config.accCov <= 0.0 ||
        !std::isfinite(config.bGyrCov) || config.bGyrCov <= 0.0 ||
        !std::isfinite(config.bAccCov) || config.bAccCov <= 0.0) {
        assignError(error, QStringLiteral("SLAM 配置无效：IMU 噪声协方差必须大于 0。"));
        return false;
    }
    if (!std::isfinite(config.preprocessScanRateHz) || config.preprocessScanRateHz <= 0.0 ||
        config.inputFrameDurationMs <= 0) {
        assignError(error, QStringLiteral("SLAM 配置无效：扫描频率和聚帧周期必须大于 0。"));
        return false;
    }
    if (!isFiniteArray(config.extrinsicT_L_I, 3) ||
        !isUsableRotationMatrix(config.extrinsicR_L_I)) {
        assignError(error, QStringLiteral("SLAM 外参配置无效：LiDAR-IMU 平移或旋转矩阵缺失/非法。"));
        return false;
    }
    if (config.dynamicObjectDetectionEnabled &&
        (!std::isfinite(config.dynamicObjectBufferDelaySec) || config.dynamicObjectBufferDelaySec < 0.0 ||
         !std::isfinite(config.dynamicObjectDepthMapDurationSec) || config.dynamicObjectDepthMapDurationSec <= 0.0 ||
         config.dynamicObjectMaxDepthMaps <= 0 || config.dynamicObjectMinHistoryMaps <= 0 ||
         !std::isfinite(config.dynamicObjectHorizontalResolutionRad) || config.dynamicObjectHorizontalResolutionRad <= 0.0 ||
         !std::isfinite(config.dynamicObjectVerticalResolutionRad) || config.dynamicObjectVerticalResolutionRad <= 0.0 ||
         config.dynamicObjectVerticalFovDownDeg >= config.dynamicObjectVerticalFovUpDeg ||
         config.dynamicObjectHorizontalFovRightDeg >= config.dynamicObjectHorizontalFovLeftDeg ||
         config.dynamicObjectMinRangeM < 0.0 || config.dynamicObjectMinRangeM >= config.dynamicObjectMaxRangeM ||
         config.dynamicObjectNeighborPixelRadius < 0 ||
         config.dynamicObjectCase1DepthMarginM <= 0.0 ||
         config.dynamicObjectCase2DepthMarginM <= 0.0 ||
         config.dynamicObjectCase3DepthMarginM <= 0.0 ||
         config.dynamicObjectCase1VoteThreshold <= 0 ||
         config.dynamicObjectCase2OcclusionChainLength <= 0 ||
         config.dynamicObjectCase3OcclusionChainLength <= 0 ||
         (config.dynamicObjectClusterEnabled &&
          (config.dynamicObjectClusterVoxelSizeM <= 0.0 ||
           config.dynamicObjectClusterExtendVoxel <= 0 ||
           config.dynamicObjectClusterMinVoxelCount <= 0 ||
           config.dynamicObjectClusterTrustThreshold <= 0.0 ||
           config.dynamicObjectClusterTrustThreshold > 1.0 ||
           config.dynamicObjectClusterGroundDistanceThresholdM <= 0.0 ||
           config.dynamicObjectClusterGroundMaxAngleDeg <= 0.0 ||
           config.dynamicObjectClusterGroundMaxAngleDeg >= 90.0)))) {
        assignError(error, QStringLiteral("SLAM 配置无效：动态检测时间窗口、球面深度图或 Case 阈值参数非法。"));
        return false;
    }
    assignError(error, QString());
    return true;
}

DynamicObjectDetectorConfig dynamicObjectConfigFromRuntime(const SlamRuntimeConfig& config)
{
    const bool aviaFamily = config.lidarTemplate == SlamLidarTemplate::Avia ||
                            config.lidarTemplate == SlamLidarTemplate::Avia2;
    DynamicObjectDetectorConfig detectorConfig;
    detectorConfig.horizontalResolutionRad = config.dynamicObjectHorizontalResolutionRad;
    detectorConfig.verticalResolutionRad = config.dynamicObjectVerticalResolutionRad;
    detectorConfig.verticalFovDownDeg = config.dynamicObjectVerticalFovDownDeg;
    detectorConfig.verticalFovUpDeg = config.dynamicObjectVerticalFovUpDeg;
    detectorConfig.horizontalFovRightDeg = config.dynamicObjectHorizontalFovRightDeg;
    detectorConfig.horizontalFovLeftDeg = config.dynamicObjectHorizontalFovLeftDeg;
    detectorConfig.minRangeM = config.dynamicObjectMinRangeM;
    detectorConfig.maxRangeM = config.dynamicObjectMaxRangeM;
    detectorConfig.bufferDelaySec = config.dynamicObjectBufferDelaySec;
    detectorConfig.depthMapDurationSec = config.dynamicObjectDepthMapDurationSec;
    detectorConfig.frameDurationSec = aviaFamily ? 0.1 : 0.02;
    detectorConfig.maxDepthMaps = config.dynamicObjectMaxDepthMaps;
    detectorConfig.maxPointsPerPixel = aviaFamily ? 50 : 20;
    detectorConfig.minHistoryMaps = config.dynamicObjectMinHistoryMaps;
    detectorConfig.neighborPixelRadius = config.dynamicObjectNeighborPixelRadius;
    detectorConfig.case1DepthMarginM = config.dynamicObjectCase1DepthMarginM;
    detectorConfig.case2DepthMarginM = config.dynamicObjectCase2DepthMarginM;
    detectorConfig.case3DepthMarginM = config.dynamicObjectCase3DepthMarginM;
    detectorConfig.case1VoteThreshold = config.dynamicObjectCase1VoteThreshold;
    detectorConfig.case2OcclusionChainLength = config.dynamicObjectCase2OcclusionChainLength;
    detectorConfig.case3OcclusionChainLength = config.dynamicObjectCase3OcclusionChainLength;
    if (aviaFamily) {
        detectorConfig.case2MapConsistencyDepthM = 0.1;
        detectorConfig.case2MapConsistencyHorizontalRad = 0.01;
        detectorConfig.case2MapConsistencyVerticalRad = 0.01;
        detectorConfig.case2OcclusionHorizontalRad = 0.02;
        detectorConfig.case2OcclusionVerticalRad = 0.03;
        detectorConfig.case2DepthConsistencyHorizontalRad = 0.01;
        detectorConfig.case2DepthConsistencyVerticalRad = 0.01;
        detectorConfig.case3MapConsistencyDepthM = 0.2;
        detectorConfig.case3MapConsistencyHorizontalRad = 0.015;
        detectorConfig.case3MapConsistencyVerticalRad = 0.015;
        detectorConfig.case3OcclusionHorizontalRad = 0.01;
        detectorConfig.case3OcclusionVerticalRad = 0.01;
        detectorConfig.case3DepthConsistencyHorizontalRad = 0.01;
        detectorConfig.case3DepthConsistencyVerticalRad = 0.01;
    }
    detectorConfig.clusterEnabled = config.dynamicObjectClusterEnabled;
    detectorConfig.clusterVoxelSizeM = config.dynamicObjectClusterVoxelSizeM;
    detectorConfig.clusterExtendVoxel = config.dynamicObjectClusterExtendVoxel;
    detectorConfig.clusterMinVoxelCount = config.dynamicObjectClusterMinVoxelCount;
    detectorConfig.clusterTrustThreshold = config.dynamicObjectClusterTrustThreshold;
    detectorConfig.clusterGroundDistanceThresholdM =
        config.dynamicObjectClusterGroundDistanceThresholdM;
    detectorConfig.clusterGroundMaxAngleDeg = config.dynamicObjectClusterGroundMaxAngleDeg;
    return detectorConfig;
}

struct FastLioAlgorithmState {
    explicit FastLioAlgorithmState(const SlamRuntimeConfig& runtimeConfig)
        : config(runtimeConfig),
          featsFromMap(new PointCloudXYZI()),
          featsUndistort(new PointCloudXYZI()),
          featsDownBody(new PointCloudXYZI()),
          featsDownWorld(new PointCloudXYZI()),
          normvec(new PointCloudXYZI()),
          laserCloudOri(new PointCloudXYZI()),
          corrNormvect(new PointCloudXYZI()),
          imuProcessor(new ImuProcess())
    {
        lidarOnly = !config.imuEnabled && config.allowPureLidar;
        filterSizeSurf = config.filterSizeSurfM;
        filterSizeMap = config.filterSizeMapM;
        cubeLen = config.cubeSideLengthM;
        detRange = static_cast<float>(config.detRangeM);
        fovDeg = config.fovDegree;
        fovDegUsed = (fovDeg + 10.0) > 179.9 ? 179.9 : (fovDeg + 10.0);
        halfFovCos = std::cos(fovDegUsed * 0.5 * PI_M / 180.0);

        xAxisPointBody = V3F(LIDAR_SP_LEN, 0.0f, 0.0f);
        xAxisPointWorld = V3F(LIDAR_SP_LEN, 0.0f, 0.0f);
        positionLast = Zero3d;
        lidarTWrImu << VEC_FROM_ARRAY(config.extrinsicT_L_I);
        lidarRWrImu << MAT_FROM_ARRAY(config.extrinsicR_L_I);

        downSizeFilterSurf.setLeafSize(static_cast<float>(filterSizeSurf),
                                       static_cast<float>(filterSizeSurf),
                                       static_cast<float>(filterSizeSurf));
        downSizeFilterMap.setLeafSize(static_cast<float>(filterSizeMap),
                                      static_cast<float>(filterSizeMap),
                                      static_cast<float>(filterSizeMap));

        imuProcessor->set_extrinsic(lidarTWrImu, lidarRWrImu);
        imuProcessor->set_gravity_norm(config.gravityNorm);
        imuProcessor->set_gyr_cov(V3D(config.gyrCov, config.gyrCov, config.gyrCov));
        imuProcessor->set_acc_cov(V3D(config.accCov, config.accCov, config.accCov));
        imuProcessor->set_gyr_bias_cov(V3D(config.bGyrCov, config.bGyrCov, config.bGyrCov));
        imuProcessor->set_acc_bias_cov(V3D(config.bAccCov, config.bAccCov, config.bAccCov));
        imuProcessor->lidar_type = AVIA;
        if (config.dynamicObjectDetectionEnabled) {
            dynamicObjectDetector = std::make_unique<DynamicObjectDetector>(
                dynamicObjectConfigFromRuntime(config));
        }

        double epsi[23] = {};
        std::fill(epsi, epsi + 23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, hShareModel, config.maxIterations, epsi);
        if (lidarOnly) {
            statePoint.offset_T_L_I = lidarTWrImu;
            statePoint.offset_R_L_I = SO3(lidarRWrImu);
            kf.change_x(statePoint);

            auto covariance = kf.get_P();
            covariance.setIdentity();
            covariance *= kLoFixedStateCov;
            covariance.block<6, 6>(0, 0) =
                Eigen::Matrix<double, 6, 6>::Identity() * kLoInitialPoseCov;
            covariance.block<3, 3>(12, 12) =
                Eigen::Matrix3d::Identity() * kLoInitialMotionCov;
            covariance.block<3, 3>(15, 15) =
                Eigen::Matrix3d::Identity() * kLoInitialMotionCov;
            kf.change_P(covariance);
        }
    }

    SlamRuntimeConfig config;
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

    static void hShareModel(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfomData);
};

thread_local FastLioAlgorithmState* activeFastLioState = nullptr;

void predictWithoutImu(FastLioAlgorithmState& state, int64_t timestampNs)
{
    if (!state.hasLoReferencePose) {
        return;
    }

    const double dt = double(timestampNs - state.loReferenceTimestampNs) * kNsToSeconds;
    state.statePoint = state.kf.get_x();
    const V3D velocity(state.statePoint.vel(0), state.statePoint.vel(1), state.statePoint.vel(2));
    const V3D angularVelocity(state.statePoint.bg(0), state.statePoint.bg(1), state.statePoint.bg(2));
    state.statePoint.pos = state.statePoint.pos + velocity * dt;
    state.statePoint.rot = SO3(state.statePoint.rot.toRotationMatrix() * Exp(angularVelocity, dt));

    auto covariance = state.kf.get_P();
    Eigen::Matrix<double, 23, 23> transition = Eigen::Matrix<double, 23, 23>::Identity();
    transition.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity() * dt;
    transition.block<3, 3>(3, 3) = Exp(angularVelocity, -dt);
    transition.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
    covariance = transition * covariance * transition.transpose();
    covariance.block<3, 3>(12, 12) +=
        Eigen::Matrix3d::Identity() * state.config.accCov * dt * dt;
    covariance.block<3, 3>(15, 15) +=
        Eigen::Matrix3d::Identity() * state.config.gyrCov * dt * dt;
    state.kf.change_x(state.statePoint);
    state.kf.change_P(covariance);
}

void undistortWithoutImu(FastLioAlgorithmState& state)
{
    *state.featsUndistort = *state.measures.lidar;
    if (!state.currentFrameHasPointOffsetTime) {
        return;
    }

    const double frameDurationSec = state.measures.lidar_end_time - state.measures.lidar_beg_time;
    const M3D worldFromImu = state.statePoint.rot.toRotationMatrix();
    const M3D lidarToImu = state.statePoint.offset_R_L_I.toRotationMatrix();
    const V3D lidarInImu(state.statePoint.offset_T_L_I(0),
                         state.statePoint.offset_T_L_I(1),
                         state.statePoint.offset_T_L_I(2));
    const V3D velocityWorld(state.statePoint.vel(0),
                            state.statePoint.vel(1),
                            state.statePoint.vel(2));
    const V3D angularVelocity(state.statePoint.bg(0),
                              state.statePoint.bg(1),
                              state.statePoint.bg(2));

    for (PointType& point : state.featsUndistort->points) {
        const double pointOffsetSec = double(point.curvature) * 0.001;
        const double dt = frameDurationSec - pointOffsetSec;
        const V3D pointLidar(point.x, point.y, point.z);
        const V3D pointImu = lidarToImu * pointLidar + lidarInImu;
        const V3D pointImuAtEnd =
            Exp(angularVelocity, -dt) * pointImu - worldFromImu.transpose() * velocityWorld * dt;
        const V3D pointLidarAtEnd = lidarToImu.transpose() * (pointImuAtEnd - lidarInImu);
        point.x = static_cast<float>(pointLidarAtEnd(0));
        point.y = static_cast<float>(pointLidarAtEnd(1));
        point.z = static_cast<float>(pointLidarAtEnd(2));
    }
}

void commitLidarOnlyPose(FastLioAlgorithmState& state, int64_t timestampNs)
{
    state.statePoint = state.kf.get_x();
    if (state.hasLoReferencePose) {
        const double dt = double(timestampNs - state.loReferenceTimestampNs) * kNsToSeconds;
        state.statePoint.vel = (state.statePoint.pos - state.loReferencePosition) / dt;
        const M3D relativeRotation =
            state.loReferenceRotation.transpose() * state.statePoint.rot.toRotationMatrix();
        state.statePoint.bg = Log(relativeRotation) / dt;
        state.kf.change_x(state.statePoint);
    }
    state.loReferenceTimestampNs = timestampNs;
    state.loReferencePosition = state.statePoint.pos;
    state.loReferenceRotation = state.statePoint.rot.toRotationMatrix();
    state.hasLoReferencePose = true;
}

void assignError(QString* error, const QString& message)
{
    if (error != nullptr) {
        *error = message;
    }
}

void resetOutput(SlamOutput* output)
{
    if (output != nullptr) {
        *output = SlamOutput();
    }
}

void assignOutputStatus(SlamOutput* output, SlamStatusCode status, const QString& message)
{
    if (output != nullptr) {
        output->status = status;
        output->message = message;
        output->imuHealthy = status == SlamStatusCode::Running || status == SlamStatusCode::InitializingImu;
    }
}

MeasureGroup toFastLioMeasureGroup(const SlamInputFrame& frame, double blindMinRangeM, int pointFilterNum)
{
    MeasureGroup measure;
    measure.lidar_beg_time = static_cast<double>(frame.frameStartNs) * kNsToSeconds;
    measure.lidar_end_time = static_cast<double>(frame.frameEndNs) * kNsToSeconds;
    measure.lidar->points.reserve(static_cast<std::size_t>(frame.points.size()));
    const double blindRangeSq = blindMinRangeM * blindMinRangeM;
    int validPointCount = 0;

    for (const SlamPoint& sourcePoint : frame.points) {
        ++validPointCount;
        if (validPointCount % pointFilterNum != 0) {
            continue;
        }
        const double x = sourcePoint.x;
        const double y = sourcePoint.y;
        const double z = sourcePoint.z;
        if (x * x + y * y + z * z < blindRangeSq) {
            continue;
        }

        PointType targetPoint;
        targetPoint.x = sourcePoint.x;
        targetPoint.y = sourcePoint.y;
        targetPoint.z = sourcePoint.z;
        targetPoint.intensity = static_cast<float>(sourcePoint.reflectivity);
        targetPoint.curvature = static_cast<float>(static_cast<double>(sourcePoint.offsetNs) * kNsToMilliseconds);
        measure.lidar->points.push_back(targetPoint);
    }
    measure.lidar->width = static_cast<uint32_t>(measure.lidar->points.size());
    measure.lidar->height = 1;

    for (const SlamImuSample& sourceImu : frame.imuSamples) {
        auto targetImu = std::make_shared<sensor_msgs::Imu>();
        targetImu->header.stamp = ros::Time().fromSec(static_cast<double>(sourceImu.timestampNs) * kNsToSeconds);
        targetImu->angular_velocity.x = sourceImu.gyroRadPerSec[0];
        targetImu->angular_velocity.y = sourceImu.gyroRadPerSec[1];
        targetImu->angular_velocity.z = sourceImu.gyroRadPerSec[2];
        targetImu->linear_acceleration.x = sourceImu.accelRaw[0];
        targetImu->linear_acceleration.y = sourceImu.accelRaw[1];
        targetImu->linear_acceleration.z = sourceImu.accelRaw[2];
        measure.imu.push_back(targetImu);
    }

    return measure;
}

void pointBodyToWorld(const PointType* pi, PointType* po, const state_ikfom& state)
{
    V3D pBody(pi->x, pi->y, pi->z);
    V3D pGlobal(state.rot * (state.offset_R_L_I * pBody + state.offset_T_L_I) + state.pos);

    po->x = static_cast<float>(pGlobal(0));
    po->y = static_cast<float>(pGlobal(1));
    po->z = static_cast<float>(pGlobal(2));
    po->intensity = pi->intensity;
}

void pointBodyLidarToImu(const PointType* pi, PointType* po, const state_ikfom& state)
{
    V3D pBodyLidar(pi->x, pi->y, pi->z);
    V3D pBodyImu(state.offset_R_L_I * pBodyLidar + state.offset_T_L_I);

    po->x = static_cast<float>(pBodyImu(0));
    po->y = static_cast<float>(pBodyImu(1));
    po->z = static_cast<float>(pBodyImu(2));
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Eigen::Matrix<T, 3, 1>& pi, Eigen::Matrix<T, 3, 1>& po, const state_ikfom& state)
{
    V3D pBody(pi[0], pi[1], pi[2]);
    V3D pGlobal(state.rot * (state.offset_R_L_I * pBody + state.offset_T_L_I) + state.pos);

    po[0] = static_cast<T>(pGlobal(0));
    po[1] = static_cast<T>(pGlobal(1));
    po[2] = static_cast<T>(pGlobal(2));
}

SlamPoint toSlamPoint(const PointType& sourcePoint)
{
    SlamPoint targetPoint;
    targetPoint.x = sourcePoint.x;
    targetPoint.y = sourcePoint.y;
    targetPoint.z = sourcePoint.z;
    targetPoint.reflectivity = static_cast<uint8_t>(std::clamp(sourcePoint.intensity, 0.0f, 255.0f));
    targetPoint.hasOffsetTime = false;
    return targetPoint;
}

SlamDynamicPointLabel toSlamDynamicPointLabel(DynamicObjectLabel label)
{
    switch (label) {
    case DynamicObjectLabel::Case1:
        return SlamDynamicPointLabel::Case1;
    case DynamicObjectLabel::Case2:
        return SlamDynamicPointLabel::Case2;
    case DynamicObjectLabel::Case3:
        return SlamDynamicPointLabel::Case3;
    case DynamicObjectLabel::Invalid:
        return SlamDynamicPointLabel::Invalid;
    case DynamicObjectLabel::Static:
    default:
        return SlamDynamicPointLabel::Static;
    }
}

void collectRemovedPoints(FastLioAlgorithmState& state)
{
    PointVector pointsHistory;
    state.ikdtree.acquire_removed_points(pointsHistory);
}

void laserMapFovSegment(FastLioAlgorithmState& state)
{
    state.cubNeedRemove.clear();
    pointBodyToWorld(state.xAxisPointBody, state.xAxisPointWorld, state.statePoint);
    const V3D posLiD = state.posLid;
    if (!state.localMapInitialized) {
        for (int i = 0; i < 3; i++) {
            state.localMapPoints.vertex_min[i] = static_cast<float>(posLiD(i) - state.cubeLen / 2.0);
            state.localMapPoints.vertex_max[i] = static_cast<float>(posLiD(i) + state.cubeLen / 2.0);
        }
        state.localMapInitialized = true;
        return;
    }

    float distToMapEdge[3][2];
    bool needMove = false;
    for (int i = 0; i < 3; i++) {
        distToMapEdge[i][0] = static_cast<float>(std::fabs(posLiD(i) - state.localMapPoints.vertex_min[i]));
        distToMapEdge[i][1] = static_cast<float>(std::fabs(posLiD(i) - state.localMapPoints.vertex_max[i]));
        if (distToMapEdge[i][0] <= kMoveThreshold * state.detRange || distToMapEdge[i][1] <= kMoveThreshold * state.detRange) {
            needMove = true;
        }
    }
    if (!needMove) {
        return;
    }

    BoxPointType newLocalMapPoints = state.localMapPoints;
    BoxPointType tmpBoxPoints;
    const float moveDist = static_cast<float>(std::max((state.cubeLen - 2.0 * kMoveThreshold * state.detRange) * 0.5 * 0.9,
                                                       static_cast<double>(state.detRange * (kMoveThreshold - 1.0f))));
    for (int i = 0; i < 3; i++) {
        tmpBoxPoints = state.localMapPoints;
        if (distToMapEdge[i][0] <= kMoveThreshold * state.detRange) {
            newLocalMapPoints.vertex_max[i] -= moveDist;
            newLocalMapPoints.vertex_min[i] -= moveDist;
            tmpBoxPoints.vertex_min[i] = state.localMapPoints.vertex_max[i] - moveDist;
            state.cubNeedRemove.push_back(tmpBoxPoints);
        } else if (distToMapEdge[i][1] <= kMoveThreshold * state.detRange) {
            newLocalMapPoints.vertex_max[i] += moveDist;
            newLocalMapPoints.vertex_min[i] += moveDist;
            tmpBoxPoints.vertex_max[i] = state.localMapPoints.vertex_min[i] + moveDist;
            state.cubNeedRemove.push_back(tmpBoxPoints);
        }
    }
    state.localMapPoints = newLocalMapPoints;

    collectRemovedPoints(state);
    if (!state.cubNeedRemove.empty()) {
        state.ikdtree.Delete_Point_Boxes(state.cubNeedRemove);
    }
}

void mapIncremental(FastLioAlgorithmState& state, PointVector& addedPoints)
{
    PointVector pointToAdd;
    PointVector pointNoNeedDownsample;
    pointToAdd.reserve(static_cast<std::size_t>(state.featsDownSize));
    pointNoNeedDownsample.reserve(static_cast<std::size_t>(state.featsDownSize));
    for (int i = 0; i < state.featsDownSize; i++) {
        pointBodyToWorld(&(state.featsDownBody->points[static_cast<std::size_t>(i)]),
                         &(state.featsDownWorld->points[static_cast<std::size_t>(i)]),
                         state.statePoint);
        if (!state.nearestPoints[static_cast<std::size_t>(i)].empty() && state.ekfInited) {
            const PointVector& pointsNear = state.nearestPoints[static_cast<std::size_t>(i)];
            bool needAdd = true;
            PointType midPoint;
            midPoint.x = static_cast<float>(std::floor(state.featsDownWorld->points[static_cast<std::size_t>(i)].x / state.filterSizeMap) * state.filterSizeMap + 0.5 * state.filterSizeMap);
            midPoint.y = static_cast<float>(std::floor(state.featsDownWorld->points[static_cast<std::size_t>(i)].y / state.filterSizeMap) * state.filterSizeMap + 0.5 * state.filterSizeMap);
            midPoint.z = static_cast<float>(std::floor(state.featsDownWorld->points[static_cast<std::size_t>(i)].z / state.filterSizeMap) * state.filterSizeMap + 0.5 * state.filterSizeMap);
            const float dist = calc_dist(state.featsDownWorld->points[static_cast<std::size_t>(i)], midPoint);
            if (std::fabs(pointsNear[0].x - midPoint.x) > 0.5 * state.filterSizeMap &&
                std::fabs(pointsNear[0].y - midPoint.y) > 0.5 * state.filterSizeMap &&
                std::fabs(pointsNear[0].z - midPoint.z) > 0.5 * state.filterSizeMap) {
                pointNoNeedDownsample.push_back(state.featsDownWorld->points[static_cast<std::size_t>(i)]);
                continue;
            }
            for (int readdI = 0; readdI < NUM_MATCH_POINTS; readdI++) {
                if (pointsNear.size() < NUM_MATCH_POINTS) {
                    break;
                }
                if (calc_dist(pointsNear[static_cast<std::size_t>(readdI)], midPoint) < dist) {
                    needAdd = false;
                    break;
                }
            }
            if (needAdd) {
                pointToAdd.push_back(state.featsDownWorld->points[static_cast<std::size_t>(i)]);
            }
        } else {
            pointToAdd.push_back(state.featsDownWorld->points[static_cast<std::size_t>(i)]);
        }
    }

    state.ikdtree.Add_Points(pointToAdd, true);
    state.ikdtree.Add_Points(pointNoNeedDownsample, false);

    addedPoints = pointToAdd;
    addedPoints.insert(addedPoints.end(), pointNoNeedDownsample.begin(), pointNoNeedDownsample.end());
}

void FastLioAlgorithmState::hShareModel(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfomData)
{
    FastLioAlgorithmState& state = *activeFastLioState;
    state.laserCloudOri->clear();
    state.corrNormvect->clear();
    state.totalResidual = 0.0;

    for (int i = 0; i < state.featsDownSize; i++) {
        PointType& pointBody = state.featsDownBody->points[static_cast<std::size_t>(i)];
        PointType& pointWorld = state.featsDownWorld->points[static_cast<std::size_t>(i)];

        V3D pBody(pointBody.x, pointBody.y, pointBody.z);
        V3D pGlobal(s.rot * (s.offset_R_L_I * pBody + s.offset_T_L_I) + s.pos);
        pointWorld.x = static_cast<float>(pGlobal(0));
        pointWorld.y = static_cast<float>(pGlobal(1));
        pointWorld.z = static_cast<float>(pGlobal(2));
        pointWorld.intensity = pointBody.intensity;

        std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        PointVector& pointsNear = state.nearestPoints[static_cast<std::size_t>(i)];

        if (ekfomData.converge) {
            state.ikdtree.Nearest_Search(pointWorld, NUM_MATCH_POINTS, pointsNear, pointSearchSqDis);
            state.pointSelectedSurf[static_cast<std::size_t>(i)] =
                pointsNear.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] <= 5;
        }

        if (!state.pointSelectedSurf[static_cast<std::size_t>(i)]) {
            continue;
        }

        VF(4) pabcd;
        state.pointSelectedSurf[static_cast<std::size_t>(i)] = false;
        if (esti_plane(pabcd, pointsNear, 0.1f)) {
            const float pd2 = pabcd(0) * pointWorld.x + pabcd(1) * pointWorld.y + pabcd(2) * pointWorld.z + pabcd(3);
            const float score = 1.0f - 0.9f * std::fabs(pd2) / std::sqrt(pBody.norm());

            if (score > 0.9f) {
                state.pointSelectedSurf[static_cast<std::size_t>(i)] = true;
                state.normvec->points[static_cast<std::size_t>(i)].x = pabcd(0);
                state.normvec->points[static_cast<std::size_t>(i)].y = pabcd(1);
                state.normvec->points[static_cast<std::size_t>(i)].z = pabcd(2);
                state.normvec->points[static_cast<std::size_t>(i)].intensity = pd2;
                state.resLast[static_cast<std::size_t>(i)] = std::fabs(pd2);
            }
        }
    }

    state.effectiveFeatureNum = 0;
    for (int i = 0; i < state.featsDownSize; i++) {
        if (state.pointSelectedSurf[static_cast<std::size_t>(i)]) {
            state.laserCloudOri->push_back(state.featsDownBody->points[static_cast<std::size_t>(i)]);
            state.corrNormvect->push_back(state.normvec->points[static_cast<std::size_t>(i)]);
            state.totalResidual += state.resLast[static_cast<std::size_t>(i)];
            ++state.effectiveFeatureNum;
        }
    }

    if (state.effectiveFeatureNum < 1) {
        ekfomData.valid = false;
        return;
    }

    state.resMeanLast = state.totalResidual / state.effectiveFeatureNum;
    ekfomData.h_x = Eigen::MatrixXd::Zero(state.effectiveFeatureNum, 12);
    ekfomData.h.resize(state.effectiveFeatureNum);

    for (int i = 0; i < state.effectiveFeatureNum; i++) {
        const PointType& laserP = state.laserCloudOri->points[static_cast<std::size_t>(i)];
        V3D pointThisBe(laserP.x, laserP.y, laserP.z);
        M3D pointBeCrossmat;
        pointBeCrossmat << SKEW_SYM_MATRX(pointThisBe);
        V3D pointThis = s.offset_R_L_I * pointThisBe + s.offset_T_L_I;
        M3D pointCrossmat;
        pointCrossmat << SKEW_SYM_MATRX(pointThis);

        const PointType& normP = state.corrNormvect->points[static_cast<std::size_t>(i)];
        V3D normVec(normP.x, normP.y, normP.z);

        V3D C(s.rot.conjugate() * normVec);
        V3D A(pointCrossmat * C);
        if (state.config.extrinsicEstimationEnabled) {
            V3D B(pointBeCrossmat * s.offset_R_L_I.conjugate() * C);
            ekfomData.h_x.block<1, 12>(i, 0) << normP.x, normP.y, normP.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        } else {
            ekfomData.h_x.block<1, 12>(i, 0) << normP.x, normP.y, normP.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        ekfomData.h(i) = -normP.intensity;
    }
}

SlamPose toSlamPose(const FastLioAlgorithmState& state, int64_t timestampNs)
{
    SlamPose pose;
    pose.timestampNs = timestampNs;
    pose.tx = state.statePoint.pos(0);
    pose.ty = state.statePoint.pos(1);
    pose.tz = state.statePoint.pos(2);
    pose.qx = state.statePoint.rot.coeffs()[0];
    pose.qy = state.statePoint.rot.coeffs()[1];
    pose.qz = state.statePoint.rot.coeffs()[2];
    pose.qw = state.statePoint.rot.coeffs()[3];
    pose.poseFrame = QStringLiteral("slam_world");
    pose.hasCovariance = true;

    const auto covariance = state.kf.get_P();
    for (int i = 0; i < 6; i++) {
        const int k = i < 3 ? i + 3 : i - 3;
        pose.covariance[i * 6 + 0] = covariance(k, 3);
        pose.covariance[i * 6 + 1] = covariance(k, 4);
        pose.covariance[i * 6 + 2] = covariance(k, 5);
        pose.covariance[i * 6 + 3] = covariance(k, 0);
        pose.covariance[i * 6 + 4] = covariance(k, 1);
        pose.covariance[i * 6 + 5] = covariance(k, 2);
    }

    return pose;
}

void appendTrajectoryOutput(FastLioAlgorithmState& state, SlamOutput* output, int64_t timestampNs)
{
    if (output == nullptr) {
        return;
    }

    output->currentPose = toSlamPose(state, timestampNs);
    output->currentPoseValid = true;
    SlamTrajectoryPoint trajectoryPoint;
    trajectoryPoint.pose = output->currentPose;
    trajectoryPoint.quality = state.effectiveFeatureNum > 0 ? 1.0 : 0.0;
    output->newTrajectoryPoints.push_back(trajectoryPoint);
    ++state.trajectoryPointCount;
    output->trajectoryPointCount = state.trajectoryPointCount;
}

void appendPublishedWorldFrameOutput(FastLioAlgorithmState& state, SlamOutput* output)
{
    if (output == nullptr || !state.config.publishWorldFrameCloud) {
        return;
    }

    const PointCloudXYZI::Ptr& sourceCloud = state.config.publishDenseFrameCloud
        ? state.featsUndistort
        : state.featsDownBody;
    output->publishedWorldFramePoints.reserve(static_cast<int>(sourceCloud->points.size()));
    PointType worldPoint;
    for (const PointType& bodyPoint : sourceCloud->points) {
        pointBodyToWorld(&bodyPoint, &worldPoint, state.statePoint);
        output->publishedWorldFramePoints.push_back(toSlamPoint(worldPoint));
    }
}

void appendPublishedBodyFrameOutput(FastLioAlgorithmState& state, SlamOutput* output)
{
    if (output == nullptr || !state.config.publishWorldFrameCloud || !state.config.publishBodyFrameCloud) {
        return;
    }

    output->publishedBodyFramePoints.reserve(static_cast<int>(state.featsUndistort->points.size()));
    PointType bodyImuPoint;
    for (const PointType& bodyLidarPoint : state.featsUndistort->points) {
        pointBodyLidarToImu(&bodyLidarPoint, &bodyImuPoint, state.statePoint);
        output->publishedBodyFramePoints.push_back(toSlamPoint(bodyImuPoint));
    }
}

void appendGlobalMapOutput(FastLioAlgorithmState& state, SlamOutput* output)
{
    if (output == nullptr || !state.config.saveMap) {
        return;
    }

    output->newGlobalMapPoints.reserve(static_cast<int>(state.featsUndistort->points.size()));
    PointType worldPoint;
    for (const PointType& bodyPoint : state.featsUndistort->points) {
        pointBodyToWorld(&bodyPoint, &worldPoint, state.statePoint);
        output->newGlobalMapPoints.push_back(toSlamPoint(worldPoint));
    }
    state.globalMapPointCount += output->newGlobalMapPoints.size();
}

QVector<DynamicObjectLabel> appendDynamicObjectOutput(FastLioAlgorithmState& state,
                                                      SlamOutput* output,
                                                      int64_t timestampNs)
{
    if (output == nullptr) {
        return {};
    }

    output->dynamicObjectStats.enabled = state.config.dynamicObjectDetectionEnabled;
    output->dynamicObjectStats.clusterEnabled = state.config.dynamicObjectClusterEnabled;
    if (!state.config.dynamicObjectDetectionEnabled || !state.dynamicObjectDetector) {
        return {};
    }

    DynamicObjectDetectionFrame detectorFrame;
    detectorFrame.timestampNs = timestampNs;
    detectorFrame.worldFromBodyRotation = Eigen::Quaterniond(state.statePoint.rot.toRotationMatrix());
    detectorFrame.worldFromBodyTranslation =
        Eigen::Vector3d(state.statePoint.pos(0), state.statePoint.pos(1), state.statePoint.pos(2));
    detectorFrame.points.reserve(static_cast<int>(state.featsUndistort->points.size()));
    output->dynamicDetectionFrameWorldPoints.reserve(static_cast<int>(state.featsUndistort->points.size()));

    PointType bodyImuPoint;
    PointType worldPoint;
    for (const PointType& bodyLidarPoint : state.featsUndistort->points) {
        pointBodyLidarToImu(&bodyLidarPoint, &bodyImuPoint, state.statePoint);
        pointBodyToWorld(&bodyLidarPoint, &worldPoint, state.statePoint);
        DynamicObjectPoint detectorPoint;
        detectorPoint.x = bodyImuPoint.x;
        detectorPoint.y = bodyImuPoint.y;
        detectorPoint.z = bodyImuPoint.z;
        detectorPoint.reflectivity = static_cast<uint8_t>(std::clamp(bodyImuPoint.intensity, 0.0f, 255.0f));
        detectorFrame.points.push_back(detectorPoint);
        output->dynamicDetectionFrameWorldPoints.push_back(toSlamPoint(worldPoint));
    }

    const DynamicObjectDetectionResult detection = state.dynamicObjectDetector->processFrame(detectorFrame);
    output->dynamicObjectStats.staticPointCount = detection.stats.staticPointCount;
    output->dynamicObjectStats.dynamicPointCount = detection.stats.dynamicPointCount;
    output->dynamicObjectStats.originDynamicPointCount = detection.stats.originDynamicPointCount;
    output->dynamicObjectStats.clusterCount = detection.stats.clusterCount;
    output->dynamicObjectStats.rejectedClusterCount = detection.stats.rejectedClusterCount;
    output->dynamicObjectStats.groundRemovedPointCount = detection.stats.groundRemovedPointCount;
    output->dynamicObjectStats.case1PointCount = detection.stats.case1PointCount;
    output->dynamicObjectStats.case2PointCount = detection.stats.case2PointCount;
    output->dynamicObjectStats.case3PointCount = detection.stats.case3PointCount;
    output->dynamicObjectStats.invalidPointCount = detection.stats.invalidPointCount;
    output->dynamicObjectStats.historyDepthMapCount = detection.stats.historyDepthMapCount;
    output->dynamicObjectStats.detectorMs = detection.stats.detectorMs;
    output->dynamicObjectStats.clusterMs = detection.stats.clusterMs;
    output->dynamicObjectStats.clusterEnabled = detection.stats.clusterEnabled;
    output->dynamicWorldFramePoints.reserve(detection.dynamicPoints.size());

    for (const DynamicObjectPoint& detectorPoint : detection.dynamicPoints) {
        SlamDynamicPoint outputPoint;
        outputPoint.x = detectorPoint.worldX;
        outputPoint.y = detectorPoint.worldY;
        outputPoint.z = detectorPoint.worldZ;
        outputPoint.reflectivity = detectorPoint.reflectivity;
        outputPoint.label = toSlamDynamicPointLabel(detectorPoint.label);
        output->dynamicWorldFramePoints.push_back(outputPoint);
    }
    return detection.clusterLabels;
}

bool isDynamicObjectLabel(DynamicObjectLabel label)
{
    return label == DynamicObjectLabel::Case1 ||
        label == DynamicObjectLabel::Case2 ||
        label == DynamicObjectLabel::Case3;
}

void removeDynamicPointsFromBackendFrame(FastLioAlgorithmState& state,
                                         const QVector<DynamicObjectLabel>& labels)
{
    if (!state.config.dynamicObjectRemovalEnabled ||
        labels.size() != static_cast<int>(state.featsUndistort->points.size())) {
        return;
    }

    PointCloudXYZI::Ptr staticCloud(new PointCloudXYZI());
    staticCloud->points.reserve(state.featsUndistort->points.size());
    for (int index = 0; index < labels.size(); ++index) {
        if (!isDynamicObjectLabel(labels.at(index))) {
            staticCloud->points.push_back(state.featsUndistort->points[static_cast<std::size_t>(index)]);
        }
    }
    staticCloud->width = static_cast<std::uint32_t>(staticCloud->points.size());
    staticCloud->height = 1;
    state.featsUndistort = std::move(staticCloud);

    state.downSizeFilterSurf.setInputCloud(state.featsUndistort);
    state.downSizeFilterSurf.filter(*state.featsDownBody);
    state.featsDownSize = static_cast<int>(state.featsDownBody->points.size());
    state.featsDownWorld->resize(static_cast<std::size_t>(state.featsDownSize));
    state.nearestPoints.assign(static_cast<std::size_t>(state.featsDownSize), PointVector());
    for (int index = 0; index < state.featsDownSize; ++index) {
        pointBodyToWorld(&state.featsDownBody->points[static_cast<std::size_t>(index)],
                         &state.featsDownWorld->points[static_cast<std::size_t>(index)],
                         state.statePoint);
        std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        state.ikdtree.Nearest_Search(state.featsDownWorld->points[static_cast<std::size_t>(index)],
                                     NUM_MATCH_POINTS,
                                     state.nearestPoints[static_cast<std::size_t>(index)],
                                     pointSearchSqDis);
    }
}

void fillRunningOutput(FastLioAlgorithmState& state, SlamOutput* output, int64_t timestampNs, double backendMs)
{
    if (output == nullptr) {
        return;
    }
    output->status = SlamStatusCode::Running;
    output->message = state.lidarOnly && !state.currentFrameHasPointOffsetTime
        ? QStringLiteral("FAST_LO is running with rigid scans because point offset time is unavailable.")
        : QString();
    output->imuHealthy = !state.lidarOnly;
    output->backendMs = backendMs;
    output->mapPointCount = state.ikdtree.validnum();
    output->globalMapPointCount = state.globalMapPointCount;
    output->trajectoryPointCount = state.trajectoryPointCount;
    output->currentPose = toSlamPose(state, timestampNs);
    output->currentPoseValid = true;
    output->dynamicObjectStats.enabled = state.config.dynamicObjectDetectionEnabled;
    output->dynamicObjectStats.clusterEnabled = state.config.dynamicObjectClusterEnabled;
}

} // namespace

struct FastLioSlamBackend::FastLioState : FastLioAlgorithmState {
    explicit FastLioState(const SlamRuntimeConfig& runtimeConfig)
        : FastLioAlgorithmState(runtimeConfig)
    {
    }
};

FastLioSlamBackend::FastLioSlamBackend() = default;

FastLioSlamBackend::~FastLioSlamBackend() = default;

bool FastLioSlamBackend::start(const SlamRuntimeConfig& config, QString* error)
{
    config_ = config;
    QString configError;
    if (!validateRuntimeConfig(config_, &configError)) {
        message_ = configError;
        status_ = SlamStatusCode::Failed;
        assignError(error, message_);
        return false;
    }
    if (!config_.imuEnabled && !config_.allowPureLidar) {
        message_ = QStringLiteral("FAST_LIO requires IMU input.");
        status_ = SlamStatusCode::MissingImu;
        assignError(error, message_);
        return false;
    }

    state_ = std::make_unique<FastLioState>(config_);
    message_.clear();
    status_ = SlamStatusCode::Starting;
    assignError(error, QString());
    return true;
}

void FastLioSlamBackend::stop()
{
    state_.reset();
    status_ = SlamStatusCode::Stopped;
}

bool FastLioSlamBackend::reset(QString* error)
{
    state_ = std::make_unique<FastLioState>(config_);
    message_.clear();
    status_ = SlamStatusCode::Idle;
    assignError(error, QString());
    return true;
}

bool FastLioSlamBackend::processFrame(const SlamInputFrame& frame, SlamOutput* output, QString* error)
{
    resetOutput(output);
    const double startedAt = omp_get_wtime();

    if (!state_) {
        message_ = QStringLiteral("FAST_LIO backend has not been started.");
        status_ = SlamStatusCode::Failed;
        assignOutputStatus(output, status_, message_);
        assignError(error, message_);
        return false;
    }

    const bool lidarOnly = state_->lidarOnly;
    if (!lidarOnly && frame.imuSamples.isEmpty()) {
        message_ = QStringLiteral("FAST_LIO requires IMU samples for every lidar frame.");
        status_ = SlamStatusCode::MissingImu;
        assignOutputStatus(output, status_, message_);
        assignError(error, message_);
        return false;
    }

    if (!lidarOnly && !frame.hasCompleteImuCoverage) {
        message_ = QStringLiteral("FAST_LIO requires IMU coverage through the lidar frame end.");
        status_ = SlamStatusCode::TimeSyncError;
        assignOutputStatus(output, status_, message_);
        assignError(error, message_);
        return false;
    }

    if (!lidarOnly && !frame.hasPointOffsetTime) {
        message_ = QStringLiteral("FAST_LIO requires per-point offset time.");
        status_ = SlamStatusCode::TimeSyncError;
        assignOutputStatus(output, status_, message_);
        assignError(error, message_);
        return false;
    }

    FastLioState& state = *state_;
    state.measures = toFastLioMeasureGroup(frame, state.config.blindMinRangeM, state.config.pointFilterNum);
    state.currentFrameHasPointOffsetTime = frame.hasPointOffsetTime;

    if (state.firstScan) {
        state.firstLidarTime = state.measures.lidar_beg_time;
        state.firstScan = false;
        if (!state.lidarOnly) {
            state.imuProcessor->first_lidar_time = state.firstLidarTime;
            status_ = SlamStatusCode::InitializingImu;
            assignOutputStatus(output, status_, QStringLiteral("FAST_LIO accepted first lidar frame; initializing IMU."));
            assignError(error, QString());
            return true;
        }
        state.statePoint = state.kf.get_x();
        undistortWithoutImu(state);
    } else if (state.lidarOnly) {
        predictWithoutImu(state, frame.frameEndNs);
        state.statePoint = state.kf.get_x();
        undistortWithoutImu(state);
    } else {
        state.imuProcessor->Process(state.measures, state.kf, state.featsUndistort);
        state.statePoint = state.kf.get_x();
    }

    state.posLid = state.statePoint.pos + state.statePoint.rot * state.statePoint.offset_T_L_I;

    if (state.featsUndistort->empty()) {
        status_ = state.lidarOnly ? SlamStatusCode::Degraded : SlamStatusCode::InitializingImu;
        assignOutputStatus(output,
                           status_,
                           state.lidarOnly
                               ? QStringLiteral("FAST_LO received a lidar frame without usable points.")
                               : QStringLiteral("FAST_LIO IMU initialization is still running."));
        assignError(error, QString());
        return true;
    }

    state.ekfInited = state.lidarOnly
        ? state.hasLoReferencePose
        : (state.measures.lidar_beg_time - state.firstLidarTime) >= kInitTime;
    laserMapFovSegment(state);

    state.downSizeFilterSurf.setInputCloud(state.featsUndistort);
    state.downSizeFilterSurf.filter(*state.featsDownBody);
    state.featsDownSize = static_cast<int>(state.featsDownBody->points.size());

    if (state.ikdtree.Root_Node == nullptr) {
        if (state.featsDownSize > kMinMapInitPoints) {
            state.ikdtree.set_downsample_param(static_cast<float>(state.filterSizeMap));
            state.featsDownWorld->resize(static_cast<std::size_t>(state.featsDownSize));
            for (int i = 0; i < state.featsDownSize; i++) {
                pointBodyToWorld(&(state.featsDownBody->points[static_cast<std::size_t>(i)]),
                                 &(state.featsDownWorld->points[static_cast<std::size_t>(i)]),
                                 state.statePoint);
            }
            state.ikdtree.Build(state.featsDownWorld->points);
        }
        if (state.lidarOnly) {
            commitLidarOnlyPose(state, frame.frameEndNs);
        }
        appendTrajectoryOutput(state, output, frame.frameEndNs);
        appendDynamicObjectOutput(state, output, frame.frameEndNs);
        appendPublishedWorldFrameOutput(state, output);
        appendPublishedBodyFrameOutput(state, output);
        appendGlobalMapOutput(state, output);
        fillRunningOutput(state, output, frame.frameEndNs, (omp_get_wtime() - startedAt) * 1000.0);
        status_ = SlamStatusCode::Running;
        assignError(error, QString());
        return true;
    }

    if (state.featsDownSize < kMinMapInitPoints) {
        if (state.lidarOnly) {
            commitLidarOnlyPose(state, frame.frameEndNs);
        }
        appendTrajectoryOutput(state, output, frame.frameEndNs);
        appendDynamicObjectOutput(state, output, frame.frameEndNs);
        appendPublishedWorldFrameOutput(state, output);
        appendPublishedBodyFrameOutput(state, output);
        appendGlobalMapOutput(state, output);
        fillRunningOutput(state, output, frame.frameEndNs, (omp_get_wtime() - startedAt) * 1000.0);
        status_ = SlamStatusCode::Running;
        assignError(error, QString());
        return true;
    }

    state.normvec->resize(static_cast<std::size_t>(state.featsDownSize));
    state.featsDownWorld->resize(static_cast<std::size_t>(state.featsDownSize));
    state.nearestPoints.assign(static_cast<std::size_t>(state.featsDownSize), PointVector());
    state.pointSelectedSurf.assign(static_cast<std::size_t>(state.featsDownSize), true);
    state.resLast.assign(static_cast<std::size_t>(state.featsDownSize), -1000.0f);

    double solveHTime = 0.0;
    activeFastLioState = &state;
    state.kf.update_iterated_dyn_share_modified(kLaserPointCov, solveHTime);
    activeFastLioState = nullptr;

    state.statePoint = state.kf.get_x();
    if (state.lidarOnly) {
        commitLidarOnlyPose(state, frame.frameEndNs);
    }
    state.eulerCur = SO3ToEuler(state.statePoint.rot);
    state.posLid = state.statePoint.pos + state.statePoint.rot * state.statePoint.offset_T_L_I;

    const QVector<DynamicObjectLabel> dynamicLabels =
        appendDynamicObjectOutput(state, output, frame.frameEndNs);
    removeDynamicPointsFromBackendFrame(state, dynamicLabels);

    PointVector addedPoints;
    mapIncremental(state, addedPoints);

    appendTrajectoryOutput(state, output, frame.frameEndNs);
    appendPublishedWorldFrameOutput(state, output);
    appendPublishedBodyFrameOutput(state, output);
    appendGlobalMapOutput(state, output);
    fillRunningOutput(state, output, frame.frameEndNs, (omp_get_wtime() - startedAt) * 1000.0);
    status_ = SlamStatusCode::Running;
    assignError(error, QString());
    return true;
}

SlamStatusCode FastLioSlamBackend::status() const
{
    return status_;
}

FastLioPredictionState FastLioSlamBackend::predictionState(int64_t timestampNs) const
{
    FastLioPredictionState result;
    if (!state_ || status_ != SlamStatusCode::Running) {
        return result;
    }

    result.valid = true;
    result.lidarOnly = state_->lidarOnly;
    result.timestampNs = timestampNs;
    for (int i = 0; i < 3; ++i) {
        result.position[i] = state_->statePoint.pos(i);
        result.velocity[i] = state_->statePoint.vel(i);
        result.angularVelocity[i] = state_->lidarOnly ? state_->statePoint.bg(i) : 0.0;
        result.gyroBias[i] = state_->lidarOnly ? 0.0 : state_->statePoint.bg(i);
        result.accelBias[i] = state_->statePoint.ba(i);
        result.gravity[i] = state_->statePoint.grav[i];
    }
    result.orientation[0] = state_->statePoint.rot.coeffs()[0];
    result.orientation[1] = state_->statePoint.rot.coeffs()[1];
    result.orientation[2] = state_->statePoint.rot.coeffs()[2];
    result.orientation[3] = state_->statePoint.rot.coeffs()[3];
    result.accelerationScale = state_->lidarOnly ? 1.0 : state_->imuProcessor->acceleration_scale();
    return result;
}

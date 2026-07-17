#ifndef SLAM_CORE_SLAMRUNTIMECONFIG_H
#define SLAM_CORE_SLAMRUNTIMECONFIG_H

#include <QString>
#include <QSettings>

#include <cstdint>
#include <limits>

enum class SlamLidarTemplate {
    Mid360Mid360S = 0,
    Avia = 1,
    Mid360L = 2,
    Avia2 = 3,
    Custom = 4
};

struct SlamRuntimeConfig {
    QString backendType = "FAST_LIO";
    QString lidarModel;
    SlamLidarTemplate lidarTemplate = SlamLidarTemplate::Mid360Mid360S;
    bool imuEnabled = true;
    bool allowPureLidar = false;
    int64_t lidarToImuTimeOffsetNs = 0;
    double gravityNorm = 9.81;
    double extrinsicT_L_I[3] = {-0.011, -0.02329, 0.04412};
    double extrinsicR_L_I[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    bool extrinsicEstimationEnabled = false;
    double cubeSideLengthM = 1000.0;
    double detRangeM = 100.0;
    double fovDegree = 360.0;
    double blindMinRangeM = 0.5;
    int pointFilterNum = 1;
    double gyrCov = 0.1;
    double accCov = 0.1;
    double bGyrCov = 0.0001;
    double bAccCov = 0.0001;
    int maxIterations = 3;
    double filterSizeSurfM = 0.5;
    double filterSizeMapM = 0.5;
    float odometrySurfLeafSize = 0.2f;
    float mappingCornerLeafSize = 0.2f;
    float mappingSurfLeafSize = 0.2f;
    float zTolerance = std::numeric_limits<float>::max();
    float rotationTolerance = std::numeric_limits<float>::max();
    int numberOfCores = 2;
    double mappingProcessInterval = 0.15;
    float surroundingKeyframeAddingDistThreshold = 20.0f;
    float surroundingKeyframeAddingAngleThreshold = 0.2f;
    float surroundingKeyframeDensity = 1.0f;
    float surroundingKeyframeSearchRadius = 50.0f;
    bool loopClosureEnableFlag = false;
    float loopClosureFrequency = 1.0f;
    int surroundingKeyframeSize = 50;
    float historyKeyframeSearchRadius = 10.0f;
    float historyKeyframeSearchTimeDiff = 30.0f;
    int historyKeyframeSearchNum = 25;
    float historyKeyframeFitnessScore = 0.3f;
    float globalMapVisualizationSearchRadius = 1000.0f;
    float globalMapVisualizationPoseDensity = 10.0f;
    float globalMapVisualizationLeafSize = 1.0f;
    bool visualizeIkdtreeMap = false;
    bool reconstructKdTree = false;
    double preprocessScanRateHz = 10.0;
    int inputFrameDurationMs = 100;
    bool allowRosbagDriver2PointCloud2 = true;
    bool allowRosbagDriverPointCloud2SynthesizedTime = true;
    bool publishWorldFrameCloud = true;
    bool publishDenseFrameCloud = true;
    bool publishBodyFrameCloud = true;
    bool dynamicObjectDetectionEnabled = false;
    bool dynamicObjectRemovalEnabled = false;
    bool dynamicObjectClusterEnabled = false;
    double dynamicObjectClusterVoxelSizeM = 0.1;
    int dynamicObjectClusterExtendVoxel = 3;
    int dynamicObjectClusterMinVoxelCount = 1;
    double dynamicObjectClusterTrustThreshold = 0.1;
    double dynamicObjectClusterGroundDistanceThresholdM = 0.1;
    double dynamicObjectClusterGroundMaxAngleDeg = 30.0;
    double dynamicObjectBufferDelaySec = 0.1;
    double dynamicObjectDepthMapDurationSec = 0.4;
    int dynamicObjectMaxDepthMaps = 5;
    int dynamicObjectMinHistoryMaps = 2;
    double dynamicObjectHorizontalResolutionRad = 0.025;
    double dynamicObjectVerticalResolutionRad = 0.04;
    double dynamicObjectVerticalFovDownDeg = -7.0;
    double dynamicObjectVerticalFovUpDeg = 52.0;
    double dynamicObjectHorizontalFovRightDeg = -180.0;
    double dynamicObjectHorizontalFovLeftDeg = 180.0;
    double dynamicObjectMinRangeM = 0.3;
    double dynamicObjectMaxRangeM = 100.0;
    int dynamicObjectNeighborPixelRadius = 1;
    double dynamicObjectCase1DepthMarginM = 0.5;
    double dynamicObjectCase2DepthMarginM = 0.3;
    double dynamicObjectCase3DepthMarginM = 0.15;
    int dynamicObjectCase1VoteThreshold = 3;
    int dynamicObjectCase2OcclusionChainLength = 3;
    int dynamicObjectCase3OcclusionChainLength = 3;
    double mapVoxelSizeM = 0.1;
    int maxMapPoints = 2000000;
    int maxTrajectoryPoints = 200000;
    int maxInputQueueFrames = 8;
    bool saveTrajectory = false;
    bool saveMap = true;
    QString logLevel = "info";
};

QString slamLidarTemplateDisplayName(SlamLidarTemplate lidarTemplate);
SlamLidarTemplate slamLidarTemplateFromInt(int value);
void applySlamLidarTemplateDefaults(SlamRuntimeConfig& config, SlamLidarTemplate lidarTemplate);
SlamRuntimeConfig loadSlamRuntimeConfig(const QSettings& settings, const QString& prefix);
SlamRuntimeConfig loadSlamRuntimeConfigForTemplate(const QSettings& settings,
                                                   const QString& prefix,
                                                   SlamLidarTemplate lidarTemplate);
void saveSlamRuntimeConfig(QSettings& settings, const SlamRuntimeConfig& config, const QString& prefix);
void saveSlamRuntimeConfigForTemplate(QSettings& settings, const SlamRuntimeConfig& config, const QString& prefix);

#endif // SLAM_CORE_SLAMRUNTIMECONFIG_H

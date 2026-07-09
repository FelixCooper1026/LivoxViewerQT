#ifndef SLAM_CORE_SLAMRUNTIMECONFIG_H
#define SLAM_CORE_SLAMRUNTIMECONFIG_H

#include <QString>
#include <QSettings>

#include <cstdint>

enum class SlamLidarTemplate {
    Mid360Mid360S = 0,
    Avia = 1
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
    double preprocessScanRateHz = 10.0;
    int inputFrameDurationMs = 100;
    bool allowRosbagDriver2PointCloud2 = true;
    bool allowRosbagDriverPointCloud2SynthesizedTime = true;
    bool publishWorldFrameCloud = true;
    bool publishDenseFrameCloud = true;
    bool publishBodyFrameCloud = true;
    bool dynamicObjectDetectionEnabled = false;
    bool dynamicObjectClusterEnabled = false;
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

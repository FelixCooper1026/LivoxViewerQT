#ifndef SLAM_CORE_SLAMRUNTIMECONFIG_H
#define SLAM_CORE_SLAMRUNTIMECONFIG_H

#include <QString>
#include <QSettings>

#include <cstdint>

struct SlamRuntimeConfig {
    QString backendType = "FAST_LIO";
    QString lidarModel;
    bool imuEnabled = true;
    bool allowPureLidar = false;
    int64_t lidarToImuTimeOffsetNs = 0;
    double gravityNorm = 9.81;
    double extrinsicT_L_I[3] = {-0.011, -0.02329, 0.04412};
    double extrinsicR_L_I[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    bool extrinsicEstimationEnabled = false;
    double filterSizeSurfM = 0.5;
    double filterSizeMapM = 0.5;
    double preprocessScanRateHz = 10.0;
    int inputFrameDurationMs = 100;
    bool publishWorldFrameCloud = true;
    bool publishDenseFrameCloud = true;
    bool publishBodyFrameCloud = true;
    double mapVoxelSizeM = 0.1;
    int maxMapPoints = 2000000;
    int maxTrajectoryPoints = 200000;
    int maxInputQueueFrames = 8;
    bool saveTrajectory = false;
    bool saveMap = true;
    QString logLevel = "info";
};

SlamRuntimeConfig loadSlamRuntimeConfig(const QSettings& settings, const QString& prefix);
void saveSlamRuntimeConfig(QSettings& settings, const SlamRuntimeConfig& config, const QString& prefix);

#endif // SLAM_CORE_SLAMRUNTIMECONFIG_H

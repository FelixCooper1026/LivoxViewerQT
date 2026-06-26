#ifndef SLAM_CORE_SLAMRUNTIMECONFIG_H
#define SLAM_CORE_SLAMRUNTIMECONFIG_H

#include <QString>

#include <cstdint>

struct SlamRuntimeConfig {
    QString backendType = "FAST_LIO";
    QString lidarModel;
    bool imuEnabled = true;
    bool allowPureLidar = false;
    int64_t lidarToImuTimeOffsetNs = 0;
    double gravityNorm = 9.81;
    double extrinsicT_L_I[3] = {};
    double extrinsicR_L_I[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    double filterSizeSurfM = 0.5;
    double filterSizeMapM = 0.5;
    double mapVoxelSizeM = 0.1;
    int maxMapPoints = 2000000;
    int maxTrajectoryPoints = 200000;
    int maxInputQueueFrames = 8;
    bool saveTrajectory = false;
    bool saveMap = false;
    QString logLevel = "info";
};

#endif // SLAM_CORE_SLAMRUNTIMECONFIG_H

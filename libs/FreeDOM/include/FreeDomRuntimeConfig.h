#ifndef FREEDOM_FREEDOMRUNTIMECONFIG_H
#define FREEDOM_FREEDOMRUNTIMECONFIG_H

#include <string>

struct FreeDomRuntimeConfig {
    double sensorMinRangeM = 0.3;
    double sensorMaxRangeM = 100.0;
    double sensorMinZM = -20.0;
    double sensorMaxZM = 20.0;

    double subVoxelSizeM = 0.1;
    unsigned int voxelDepth = 2;
    unsigned int blockDepth = 5;

    bool localMapEnabled = false;
    double localMapRangeM = 100.0;
    double localMapMinZM = -20.0;
    double localMapMaxZM = 20.0;

    double raycastMaxRangeM = 100.0;
    double raycastMinZM = -20.0;
    double raycastMaxZM = 20.0;

    unsigned int countsToFree = 6;
    unsigned int countsToRevert = 20;
    unsigned int conservativeConnectivity = 26;
    unsigned int aggressiveConnectivity = 124;

    bool raycastEnhancementEnabled = true;
    double lidarHorizontalFovDeg = 360.0;
    double lidarVerticalFovUpperDeg = 52.0;
    double lidarVerticalFovLowerDeg = -7.0;
    unsigned int depthImageVerticalLines = 64;
    double depthImageMinRangeM = 0.3;
    double maxRaycastEnhancementRangeM = 100.0;
    double raycastEnhancementDepthMarginM = 0.2;
    unsigned int inpaintSize = 3;
    unsigned int erosionSize = 0;
    double minRaycastEnhancementArea = 0.0;
    double depthImageTopMargin = 0.0;
    bool learnFov = false;
    bool fovMaskEnabled = false;
    std::string fovMaskPath;
    unsigned int numThreads = 8;
};

#endif // FREEDOM_FREEDOMRUNTIMECONFIG_H

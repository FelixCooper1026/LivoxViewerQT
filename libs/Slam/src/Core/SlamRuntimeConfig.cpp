#include "Core/SlamRuntimeConfig.h"

#include <QVariant>

#include <algorithm>
#include <cmath>

namespace {

constexpr double kMid360ExtrinsicT_L_I[3] = {-0.011, -0.02329, 0.04412};
constexpr double kMid360LExtrinsicT_L_I[3] = {-0.023680, 0.0220500, 0.0438600};
constexpr double kAviaExtrinsicT_L_I[3] = {0.04165, 0.02326, -0.0284};
// Avia2 specifies the IMU origin in the LiDAR frame as
// (-0.0563149, 0.0367201, -0.0297031) m. FAST-LIO uses
// p_I = R_L_I * p_L + T_L_I, so identity R_L_I requires the inverse translation.
constexpr double kAvia2ExtrinsicT_L_I[3] = {0.0563149, -0.0367201, 0.0297031};
constexpr double kIdentityExtrinsicR_L_I[9] = {1.0, 0.0, 0.0,
                                               0.0, 1.0, 0.0,
                                               0.0, 0.0, 1.0};

QString key(const QString& prefix, const QString& name)
{
    return prefix + QLatin1Char('/') + name;
}

int frameDurationMsFromScanRate(double scanRateHz)
{
    return std::clamp(static_cast<int>(std::lround(1000.0 / scanRateHz)), 1, 10000);
}

double scanRateFromFrameDurationMs(int frameDurationMs)
{
    return 1000.0 / double(frameDurationMs);
}

double validScanRateHz(double value, double fallback)
{
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

int validFrameDurationMs(int value, int fallback)
{
    return value > 0 ? value : fallback;
}

double validPositiveDouble(double value, double fallback)
{
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

double validNonNegativeDouble(double value, double fallback)
{
    return std::isfinite(value) && value >= 0.0 ? value : fallback;
}

int validPositiveInt(int value, int fallback)
{
    return value > 0 ? value : fallback;
}

DynamicFilterBackend dynamicFilterBackendFromInt(int value)
{
    switch(value)
    {
    case int(DynamicFilterBackend::MDetector):
        return DynamicFilterBackend::MDetector;
    case int(DynamicFilterBackend::FreeDOM):
        return DynamicFilterBackend::FreeDOM;
    case int(DynamicFilterBackend::Disabled):
    default:
        return DynamicFilterBackend::Disabled;
    }
}

bool isZeroTranslation(const double* values)
{
    for (int i = 0; i < 3; ++i) {
        if (std::abs(values[i]) > 1.0e-12) {
            return false;
        }
    }
    return true;
}

bool isIdentityRotation(const double* values)
{
    for (int i = 0; i < 9; ++i) {
        if (std::abs(values[i] - kIdentityExtrinsicR_L_I[i]) > 1.0e-12) {
            return false;
        }
    }
    return true;
}

void assignTemplateDefaultExtrinsic(SlamRuntimeConfig& config, SlamLidarTemplate lidarTemplate)
{
    const double* extrinsicT = kMid360ExtrinsicT_L_I;
    if (lidarTemplate == SlamLidarTemplate::Mid360L) {
        extrinsicT = kMid360LExtrinsicT_L_I;
    } else if (lidarTemplate == SlamLidarTemplate::Avia) {
        extrinsicT = kAviaExtrinsicT_L_I;
    } else if (lidarTemplate == SlamLidarTemplate::Avia2) {
        extrinsicT = kAvia2ExtrinsicT_L_I;
    }
    for (int i = 0; i < 3; ++i) {
        config.extrinsicT_L_I[i] = extrinsicT[i];
    }
    for (int i = 0; i < 9; ++i) {
        config.extrinsicR_L_I[i] = kIdentityExtrinsicR_L_I[i];
    }
}

void applyMid360FreeDomDefaults(SlamRuntimeConfig& config)
{
    config.dynamicFilterBackend = DynamicFilterBackend::FreeDOM;
    config.dynamicDebugSnapshotIntervalFrames = 1;
    config.freeDomMapSnapshotIntervalFrames = 1;

    FreeDomRuntimeConfig& freeDom = config.freeDom;
    freeDom.sensorMinRangeM = 0.2;
    freeDom.sensorMaxRangeM = 40.0;
    freeDom.sensorMinZM = -5.0;
    freeDom.sensorMaxZM = 10.0;
    freeDom.subVoxelSizeM = 0.05;
    freeDom.voxelDepth = 2;
    freeDom.blockDepth = 4;
    freeDom.localMapEnabled = true;
    freeDom.localMapRangeM = 100.0;
    freeDom.localMapMinZM = -20.0;
    freeDom.localMapMaxZM = 20.0;
    freeDom.raycastMaxRangeM = 8.0;
    freeDom.raycastMinZM = -5.0;
    freeDom.raycastMaxZM = 5.0;
    freeDom.countsToFree = 3;
    freeDom.countsToRevert = 20;
    freeDom.conservativeConnectivity = 26;
    freeDom.aggressiveConnectivity = 124;
    freeDom.raycastEnhancementEnabled = true;
    freeDom.lidarHorizontalFovDeg = 360.0;
    freeDom.lidarVerticalFovLowerDeg = -7.0;
    freeDom.lidarVerticalFovUpperDeg = 52.0;
    freeDom.depthImageVerticalLines = 64;
    freeDom.depthImageMinRangeM = 0.5;
    freeDom.maxRaycastEnhancementRangeM = 100.0;
    freeDom.raycastEnhancementDepthMarginM = 0.2;
    freeDom.inpaintSize = 4;
    freeDom.erosionSize = 0;
    freeDom.minRaycastEnhancementArea = 0.0;
    freeDom.depthImageTopMargin = 0.0;
    freeDom.learnFov = false;
    freeDom.fovMaskEnabled = false;
    freeDom.fovMaskPath.clear();
    freeDom.numThreads = 4;
}

void applyMid360LFreeDomDefaults(SlamRuntimeConfig& config)
{
    applyMid360FreeDomDefaults(config);

    FreeDomRuntimeConfig& freeDom = config.freeDom;
    freeDom.subVoxelSizeM = 0.08;
    freeDom.raycastEnhancementEnabled = false;
    freeDom.lidarVerticalFovLowerDeg = -10.0;
    freeDom.lidarVerticalFovUpperDeg = 35.0;
    freeDom.depthImageVerticalLines = 32;
    freeDom.maxRaycastEnhancementRangeM = 40.0;
}

} // namespace

QString slamLidarTemplateDisplayName(SlamLidarTemplate lidarTemplate)
{
    switch (lidarTemplate) {
    case SlamLidarTemplate::Avia2:
        return QStringLiteral("Avia2");
    case SlamLidarTemplate::Avia:
        return QStringLiteral("Avia");
    case SlamLidarTemplate::Mid360L:
        return QStringLiteral("Mid-360L");
    case SlamLidarTemplate::Custom:
        return QStringLiteral("自定义");
    case SlamLidarTemplate::Mid360Mid360S:
    default:
        return QStringLiteral("Mid360/Mid360S");
    }
}

SlamLidarTemplate slamLidarTemplateFromInt(int value)
{
    switch (value) {
    case int(SlamLidarTemplate::Avia2):
        return SlamLidarTemplate::Avia2;
    case int(SlamLidarTemplate::Avia):
        return SlamLidarTemplate::Avia;
    case int(SlamLidarTemplate::Mid360L):
        return SlamLidarTemplate::Mid360L;
    case int(SlamLidarTemplate::Custom):
        return SlamLidarTemplate::Custom;
    case int(SlamLidarTemplate::Mid360Mid360S):
    default:
        return SlamLidarTemplate::Mid360Mid360S;
    }
}

void applySlamLidarTemplateDefaults(SlamRuntimeConfig& config, SlamLidarTemplate lidarTemplate)
{
    config.lidarTemplate = lidarTemplate;
    assignTemplateDefaultExtrinsic(config, lidarTemplate);
    config.cubeSideLengthM = 1000.0;
    config.maxIterations = 3;
    config.pointFilterNum = 1;
    config.filterSizeSurfM = 0.5;
    config.filterSizeMapM = 0.5;
    config.freeDom = FreeDomRuntimeConfig{};
    config.dynamicObjectBufferDelaySec = 0.1;
    config.dynamicObjectMaxDepthMaps = 5;
    config.dynamicObjectMinHistoryMaps = 2;
    config.dynamicObjectNeighborPixelRadius = 1;
    config.dynamicObjectClusterGroundDistanceThresholdM = 0.1;
    config.dynamicObjectClusterGroundMaxAngleDeg = 30.0;
    if (lidarTemplate == SlamLidarTemplate::Avia || lidarTemplate == SlamLidarTemplate::Avia2) {
        config.dynamicObjectHorizontalResolutionRad = 0.005;
        config.dynamicObjectVerticalResolutionRad = 0.005;
        const bool avia2 = lidarTemplate == SlamLidarTemplate::Avia2;
        config.detRangeM = avia2 ? 400.0 : 450.0;
        config.fovDegree = avia2 ? 80.0 : 90.0;
        config.blindMinRangeM = avia2 ? 5.0 : 4.0;
        config.dynamicObjectDepthMapDurationSec = 0.2;
        config.dynamicObjectVerticalFovDownDeg = avia2 ? -40.0 : -38.6;
        config.dynamicObjectVerticalFovUpDeg = avia2 ? 40.0 : 38.6;
        config.dynamicObjectHorizontalFovRightDeg = avia2 ? -40.0 : -34.0;
        config.dynamicObjectHorizontalFovLeftDeg = avia2 ? 40.0 : 34.0;
        config.dynamicObjectMinRangeM = avia2 ? 5.0 : 0.5;
        config.dynamicObjectMaxRangeM = avia2 ? 400.0 : 450.0;
        config.dynamicObjectCase1DepthMarginM = 0.15;
        config.dynamicObjectCase2DepthMarginM = 0.15;
        config.dynamicObjectCase3DepthMarginM = 0.15;
        config.dynamicObjectCase1VoteThreshold = 3;
        config.dynamicObjectCase2OcclusionChainLength = 3;
        config.dynamicObjectCase3OcclusionChainLength = 3;
        config.dynamicObjectClusterVoxelSizeM = 0.3;
        config.dynamicObjectClusterExtendVoxel = 5;
        config.dynamicObjectClusterMinVoxelCount = 2;
        config.dynamicObjectClusterTrustThreshold = 0.1;
        config.freeDom.sensorMinRangeM = config.dynamicObjectMinRangeM;
        config.freeDom.sensorMaxRangeM = config.dynamicObjectMaxRangeM;
        config.freeDom.sensorMinZM = -100.0;
        config.freeDom.sensorMaxZM = 100.0;
        config.freeDom.localMapRangeM = config.dynamicObjectMaxRangeM;
        config.freeDom.localMapMinZM = -100.0;
        config.freeDom.localMapMaxZM = 100.0;
        config.freeDom.raycastMaxRangeM = config.dynamicObjectMaxRangeM;
        config.freeDom.raycastMinZM = -100.0;
        config.freeDom.raycastMaxZM = 100.0;
        config.freeDom.lidarHorizontalFovDeg =
            config.dynamicObjectHorizontalFovLeftDeg -
            config.dynamicObjectHorizontalFovRightDeg;
        config.freeDom.lidarVerticalFovLowerDeg =
            config.dynamicObjectVerticalFovDownDeg;
        config.freeDom.lidarVerticalFovUpperDeg =
            config.dynamicObjectVerticalFovUpDeg;
        config.freeDom.depthImageMinRangeM = config.dynamicObjectMinRangeM;
        config.freeDom.maxRaycastEnhancementRangeM =
            config.dynamicObjectMaxRangeM;
        return;
    }

    const bool mid360L = lidarTemplate == SlamLidarTemplate::Mid360L;
    config.dynamicObjectHorizontalResolutionRad = 0.025;
    config.dynamicObjectVerticalResolutionRad = 0.04;
    config.detRangeM = 100.0;
    config.fovDegree = 360.0;
    config.blindMinRangeM = 0.5;
    config.dynamicObjectDepthMapDurationSec = 0.4;
    config.dynamicObjectVerticalFovDownDeg = mid360L ? -10.0 : -7.0;
    config.dynamicObjectVerticalFovUpDeg = mid360L ? 35.0 : 52.0;
    config.dynamicObjectHorizontalFovRightDeg = -180.0;
    config.dynamicObjectHorizontalFovLeftDeg = 180.0;
    config.dynamicObjectMinRangeM = 0.3;
    config.dynamicObjectMaxRangeM = 100.0;
    config.dynamicObjectCase1DepthMarginM = 0.5;
    config.dynamicObjectCase2DepthMarginM = 0.3;
    config.dynamicObjectCase3DepthMarginM = 0.15;
    config.dynamicObjectCase1VoteThreshold = 3;
    config.dynamicObjectCase2OcclusionChainLength = 3;
    config.dynamicObjectCase3OcclusionChainLength = 3;
    config.dynamicObjectClusterVoxelSizeM = 0.1;
    config.dynamicObjectClusterExtendVoxel = 3;
    config.dynamicObjectClusterMinVoxelCount = 1;
    config.dynamicObjectClusterTrustThreshold = 0.1;
    if (mid360L) {
        applyMid360LFreeDomDefaults(config);
    } else if (lidarTemplate == SlamLidarTemplate::Mid360Mid360S) {
        applyMid360FreeDomDefaults(config);
    }
}

namespace {

QString templateConfigPrefix(const QString& prefix, SlamLidarTemplate lidarTemplate)
{
    return key(key(prefix, QStringLiteral("templates")), QString::number(int(lidarTemplate)));
}

bool hasTemplateConfig(const QSettings& settings, const QString& prefix, SlamLidarTemplate lidarTemplate)
{
    return settings.contains(key(templateConfigPrefix(prefix, lidarTemplate), QStringLiteral("configured")));
}

void loadSlamRuntimeConfigValues(const QSettings& settings, const QString& prefix, SlamRuntimeConfig& config)
{
    config.imuEnabled = settings.value(key(prefix, QStringLiteral("imuEnabled")), config.imuEnabled).toBool();
    config.allowPureLidar = settings.value(key(prefix, QStringLiteral("allowPureLidar")), config.allowPureLidar).toBool();
    config.lidarToImuTimeOffsetNs = settings.value(key(prefix, QStringLiteral("lidarToImuTimeOffsetNs")),
                                                   qlonglong(config.lidarToImuTimeOffsetNs)).toLongLong();
    config.gravityNorm = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("gravityNorm")), config.gravityNorm).toDouble(),
        config.gravityNorm);
    for (int i = 0; i < 3; ++i) {
        config.extrinsicT_L_I[i] = settings.value(key(prefix, QStringLiteral("extrinsicT_L_I/%1").arg(i)),
                                                  config.extrinsicT_L_I[i]).toDouble();
    }
    for (int i = 0; i < 9; ++i) {
        config.extrinsicR_L_I[i] = settings.value(key(prefix, QStringLiteral("extrinsicR_L_I/%1").arg(i)),
                                                  config.extrinsicR_L_I[i]).toDouble();
    }
    const bool hasExtrinsicEstimation = settings.contains(key(prefix, QStringLiteral("extrinsicEstimationEnabled")));
    config.extrinsicEstimationEnabled = settings.value(key(prefix, QStringLiteral("extrinsicEstimationEnabled")),
                                                       config.extrinsicEstimationEnabled).toBool();
    if (!hasExtrinsicEstimation &&
        isZeroTranslation(config.extrinsicT_L_I) &&
        isIdentityRotation(config.extrinsicR_L_I)) {
        assignTemplateDefaultExtrinsic(config, config.lidarTemplate);
    }
    config.cubeSideLengthM = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("cubeSideLengthM")), config.cubeSideLengthM).toDouble(),
        config.cubeSideLengthM);
    config.detRangeM = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("detRangeM")), config.detRangeM).toDouble(),
        config.detRangeM);
    config.fovDegree = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("fovDegree")), config.fovDegree).toDouble(),
        config.fovDegree);
    config.blindMinRangeM = validNonNegativeDouble(
        settings.value(key(prefix, QStringLiteral("blindMinRangeM")), config.blindMinRangeM).toDouble(),
        config.blindMinRangeM);
    config.pointFilterNum = validPositiveInt(
        settings.value(key(prefix, QStringLiteral("pointFilterNum")), config.pointFilterNum).toInt(),
        config.pointFilterNum);
    config.gyrCov = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("gyrCov")), config.gyrCov).toDouble(),
        config.gyrCov);
    config.accCov = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("accCov")), config.accCov).toDouble(),
        config.accCov);
    config.bGyrCov = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("bGyrCov")), config.bGyrCov).toDouble(),
        config.bGyrCov);
    config.bAccCov = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("bAccCov")), config.bAccCov).toDouble(),
        config.bAccCov);
    config.maxIterations = validPositiveInt(
        settings.value(key(prefix, QStringLiteral("maxIterations")), config.maxIterations).toInt(),
        config.maxIterations);
    config.filterSizeSurfM = settings.value(key(prefix, QStringLiteral("filterSizeSurfM")), config.filterSizeSurfM).toDouble();
    config.filterSizeMapM = settings.value(key(prefix, QStringLiteral("filterSizeMapM")), config.filterSizeMapM).toDouble();
    config.odometrySurfLeafSize = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("odometrySurfLeafSize")), config.odometrySurfLeafSize).toDouble(),
        config.odometrySurfLeafSize));
    config.mappingCornerLeafSize = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("mappingCornerLeafSize")), config.mappingCornerLeafSize).toDouble(),
        config.mappingCornerLeafSize));
    config.mappingSurfLeafSize = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("mappingSurfLeafSize")), config.mappingSurfLeafSize).toDouble(),
        config.mappingSurfLeafSize));
    config.zTolerance = settings.value(key(prefix, QStringLiteral("zTolerance")), config.zTolerance).toFloat();
    config.rotationTolerance = settings.value(key(prefix, QStringLiteral("rotationTolerance")), config.rotationTolerance).toFloat();
    config.numberOfCores = validPositiveInt(
        settings.value(key(prefix, QStringLiteral("numberOfCores")), config.numberOfCores).toInt(),
        config.numberOfCores);
    config.mappingProcessInterval = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("mappingProcessInterval")), config.mappingProcessInterval).toDouble(),
        config.mappingProcessInterval);
    config.surroundingKeyframeAddingDistThreshold = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("surroundingKeyframeAddingDistThreshold")),
                       config.surroundingKeyframeAddingDistThreshold).toDouble(),
        config.surroundingKeyframeAddingDistThreshold));
    config.surroundingKeyframeAddingAngleThreshold = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("surroundingKeyframeAddingAngleThreshold")),
                       config.surroundingKeyframeAddingAngleThreshold).toDouble(),
        config.surroundingKeyframeAddingAngleThreshold));
    config.surroundingKeyframeDensity = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("surroundingKeyframeDensity")), config.surroundingKeyframeDensity).toDouble(),
        config.surroundingKeyframeDensity));
    config.surroundingKeyframeSearchRadius = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("surroundingKeyframeSearchRadius")),
                       config.surroundingKeyframeSearchRadius).toDouble(),
        config.surroundingKeyframeSearchRadius));
    config.loopClosureEnableFlag = settings.value(
        key(prefix, QStringLiteral("loopClosureEnableFlag")), config.loopClosureEnableFlag).toBool();
    config.loopClosureFrequency = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("loopClosureFrequency")), config.loopClosureFrequency).toDouble(),
        config.loopClosureFrequency));
    config.surroundingKeyframeSize = validPositiveInt(
        settings.value(key(prefix, QStringLiteral("surroundingKeyframeSize")), config.surroundingKeyframeSize).toInt(),
        config.surroundingKeyframeSize);
    config.historyKeyframeSearchRadius = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("historyKeyframeSearchRadius")),
                       config.historyKeyframeSearchRadius).toDouble(),
        config.historyKeyframeSearchRadius));
    config.historyKeyframeSearchTimeDiff = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("historyKeyframeSearchTimeDiff")),
                       config.historyKeyframeSearchTimeDiff).toDouble(),
        config.historyKeyframeSearchTimeDiff));
    config.historyKeyframeSearchNum = validPositiveInt(
        settings.value(key(prefix, QStringLiteral("historyKeyframeSearchNum")), config.historyKeyframeSearchNum).toInt(),
        config.historyKeyframeSearchNum);
    config.historyKeyframeFitnessScore = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("historyKeyframeFitnessScore")),
                       config.historyKeyframeFitnessScore).toDouble(),
        config.historyKeyframeFitnessScore));
    config.globalMapVisualizationSearchRadius = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("globalMapVisualizationSearchRadius")),
                       config.globalMapVisualizationSearchRadius).toDouble(),
        config.globalMapVisualizationSearchRadius));
    config.globalMapVisualizationPoseDensity = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("globalMapVisualizationPoseDensity")),
                       config.globalMapVisualizationPoseDensity).toDouble(),
        config.globalMapVisualizationPoseDensity));
    config.globalMapVisualizationLeafSize = static_cast<float>(validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("globalMapVisualizationLeafSize")),
                       config.globalMapVisualizationLeafSize).toDouble(),
        config.globalMapVisualizationLeafSize));
    config.visualizeIkdtreeMap = settings.value(
        key(prefix, QStringLiteral("visualizeIkdtreeMap")), config.visualizeIkdtreeMap).toBool();
    config.reconstructKdTree = settings.value(
        key(prefix, QStringLiteral("reconstructKdTree")), config.reconstructKdTree).toBool();
    const bool hasScanRate = settings.contains(key(prefix, QStringLiteral("preprocessScanRateHz")));
    const bool hasFrameDuration = settings.contains(key(prefix, QStringLiteral("inputFrameDurationMs")));
    config.preprocessScanRateHz = validScanRateHz(
        settings.value(key(prefix, QStringLiteral("preprocessScanRateHz")), config.preprocessScanRateHz).toDouble(),
        config.preprocessScanRateHz);
    config.inputFrameDurationMs = validFrameDurationMs(
        settings.value(key(prefix, QStringLiteral("inputFrameDurationMs")), config.inputFrameDurationMs).toInt(),
        config.inputFrameDurationMs);
    if (hasScanRate && !hasFrameDuration) {
        config.inputFrameDurationMs = frameDurationMsFromScanRate(config.preprocessScanRateHz);
    } else if (!hasScanRate && hasFrameDuration) {
        config.preprocessScanRateHz = scanRateFromFrameDurationMs(config.inputFrameDurationMs);
    }
    config.allowRosbagDriver2PointCloud2 = true;
    config.allowRosbagDriverPointCloud2SynthesizedTime = true;
    const bool hasPublishConfig = settings.contains(key(prefix, QStringLiteral("publishWorldFrameCloud")));
    config.publishWorldFrameCloud = settings.value(key(prefix, QStringLiteral("publishWorldFrameCloud")),
                                                   config.publishWorldFrameCloud).toBool();
    config.publishDenseFrameCloud = settings.value(key(prefix, QStringLiteral("publishDenseFrameCloud")),
                                                   config.publishDenseFrameCloud).toBool();
    config.publishBodyFrameCloud = settings.value(key(prefix, QStringLiteral("publishBodyFrameCloud")),
                                                  config.publishBodyFrameCloud).toBool();
    config.dynamicObjectDetectionEnabled = settings.value(key(prefix, QStringLiteral("dynamicObjectDetectionEnabled")),
                                                          config.dynamicObjectDetectionEnabled).toBool();
    config.dynamicObjectRemovalEnabled = settings.value(key(prefix, QStringLiteral("dynamicObjectRemovalEnabled")),
                                                        config.dynamicObjectRemovalEnabled).toBool();
    const bool hasDynamicFilterBackend =
        settings.contains(key(prefix, QStringLiteral("dynamicFilterBackend")));
    if(hasDynamicFilterBackend)
    {
        config.dynamicFilterBackend = dynamicFilterBackendFromInt(
            settings.value(key(prefix, QStringLiteral("dynamicFilterBackend")),
                           int(config.dynamicFilterBackend)).toInt());
        config.dynamicFilterEnabled =
            settings.value(key(prefix, QStringLiteral("dynamicFilterEnabled")),
                           config.dynamicFilterEnabled).toBool();
        config.dynamicPointRemovalEnabled =
            settings.value(key(prefix, QStringLiteral("dynamicPointRemovalEnabled")),
                           config.dynamicPointRemovalEnabled).toBool();
    }
    else
    {
        config.dynamicFilterEnabled = config.dynamicObjectDetectionEnabled;
        config.dynamicPointRemovalEnabled = config.dynamicObjectRemovalEnabled;
        config.dynamicFilterBackend = config.dynamicFilterEnabled
            ? DynamicFilterBackend::MDetector
            : DynamicFilterBackend::Disabled;
    }
    config.dynamicDebugVisualizationEnabled =
        settings.value(key(prefix, QStringLiteral("dynamicDebugVisualizationEnabled")),
                       config.dynamicDebugVisualizationEnabled).toBool();
    config.dynamicDebugSnapshotIntervalFrames = static_cast<unsigned int>(
        validPositiveInt(
            settings.value(key(prefix, QStringLiteral("dynamicDebugSnapshotIntervalFrames")),
                           int(config.dynamicDebugSnapshotIntervalFrames)).toInt(),
            int(config.dynamicDebugSnapshotIntervalFrames)));
    config.freeDomMapSnapshotIntervalFrames = static_cast<unsigned int>(
        validPositiveInt(
            settings.value(key(prefix, QStringLiteral("freeDomMapSnapshotIntervalFrames")),
                           int(config.freeDomMapSnapshotIntervalFrames)).toInt(),
            int(config.freeDomMapSnapshotIntervalFrames)));
    config.dynamicObjectDetectionEnabled = config.dynamicFilterEnabled;
    config.dynamicObjectRemovalEnabled = config.dynamicPointRemovalEnabled;
    config.dynamicObjectClusterEnabled = settings.value(key(prefix, QStringLiteral("dynamicObjectClusterEnabled")),
                                                        config.dynamicObjectClusterEnabled).toBool();
    config.dynamicObjectClusterVoxelSizeM = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("dynamicObjectClusterVoxelSizeM")), config.dynamicObjectClusterVoxelSizeM).toDouble(),
        config.dynamicObjectClusterVoxelSizeM);
    config.dynamicObjectClusterExtendVoxel = validPositiveInt(
        settings.value(key(prefix, QStringLiteral("dynamicObjectClusterExtendVoxel")), config.dynamicObjectClusterExtendVoxel).toInt(),
        config.dynamicObjectClusterExtendVoxel);
    config.dynamicObjectClusterMinVoxelCount = validPositiveInt(
        settings.value(key(prefix, QStringLiteral("dynamicObjectClusterMinVoxelCount")), config.dynamicObjectClusterMinVoxelCount).toInt(),
        config.dynamicObjectClusterMinVoxelCount);
    config.dynamicObjectClusterTrustThreshold = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("dynamicObjectClusterTrustThreshold")), config.dynamicObjectClusterTrustThreshold).toDouble(),
        config.dynamicObjectClusterTrustThreshold);
    config.dynamicObjectClusterGroundDistanceThresholdM = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("dynamicObjectClusterGroundDistanceThresholdM")), config.dynamicObjectClusterGroundDistanceThresholdM).toDouble(),
        config.dynamicObjectClusterGroundDistanceThresholdM);
    config.dynamicObjectClusterGroundMaxAngleDeg = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("dynamicObjectClusterGroundMaxAngleDeg")), config.dynamicObjectClusterGroundMaxAngleDeg).toDouble(),
        config.dynamicObjectClusterGroundMaxAngleDeg);
    config.dynamicObjectBufferDelaySec = validNonNegativeDouble(
        settings.value(key(prefix, QStringLiteral("dynamicObjectBufferDelaySec")), config.dynamicObjectBufferDelaySec).toDouble(),
        config.dynamicObjectBufferDelaySec);
    config.dynamicObjectDepthMapDurationSec = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("dynamicObjectDepthMapDurationSec")), config.dynamicObjectDepthMapDurationSec).toDouble(),
        config.dynamicObjectDepthMapDurationSec);
    config.dynamicObjectMaxDepthMaps = validPositiveInt(
        settings.value(key(prefix, QStringLiteral("dynamicObjectMaxDepthMaps")), config.dynamicObjectMaxDepthMaps).toInt(),
        config.dynamicObjectMaxDepthMaps);
    config.dynamicObjectMinHistoryMaps = validPositiveInt(
        settings.value(key(prefix, QStringLiteral("dynamicObjectMinHistoryMaps")), config.dynamicObjectMinHistoryMaps).toInt(),
        config.dynamicObjectMinHistoryMaps);
    config.dynamicObjectHorizontalResolutionRad = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("dynamicObjectHorizontalResolutionRad")), config.dynamicObjectHorizontalResolutionRad).toDouble(),
        config.dynamicObjectHorizontalResolutionRad);
    config.dynamicObjectVerticalResolutionRad = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("dynamicObjectVerticalResolutionRad")), config.dynamicObjectVerticalResolutionRad).toDouble(),
        config.dynamicObjectVerticalResolutionRad);
    config.dynamicObjectVerticalFovDownDeg = settings.value(
        key(prefix, QStringLiteral("dynamicObjectVerticalFovDownDeg")), config.dynamicObjectVerticalFovDownDeg).toDouble();
    config.dynamicObjectVerticalFovUpDeg = settings.value(
        key(prefix, QStringLiteral("dynamicObjectVerticalFovUpDeg")), config.dynamicObjectVerticalFovUpDeg).toDouble();
    config.dynamicObjectHorizontalFovRightDeg = settings.value(
        key(prefix, QStringLiteral("dynamicObjectHorizontalFovRightDeg")), config.dynamicObjectHorizontalFovRightDeg).toDouble();
    config.dynamicObjectHorizontalFovLeftDeg = settings.value(
        key(prefix, QStringLiteral("dynamicObjectHorizontalFovLeftDeg")), config.dynamicObjectHorizontalFovLeftDeg).toDouble();
    config.dynamicObjectMinRangeM = validNonNegativeDouble(
        settings.value(key(prefix, QStringLiteral("dynamicObjectMinRangeM")), config.dynamicObjectMinRangeM).toDouble(),
        config.dynamicObjectMinRangeM);
    config.dynamicObjectMaxRangeM = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("dynamicObjectMaxRangeM")), config.dynamicObjectMaxRangeM).toDouble(),
        config.dynamicObjectMaxRangeM);
    config.dynamicObjectNeighborPixelRadius = settings.value(
        key(prefix, QStringLiteral("dynamicObjectNeighborPixelRadius")), config.dynamicObjectNeighborPixelRadius).toInt();
    config.dynamicObjectCase1DepthMarginM = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("dynamicObjectCase1DepthMarginM")), config.dynamicObjectCase1DepthMarginM).toDouble(),
        config.dynamicObjectCase1DepthMarginM);
    config.dynamicObjectCase2DepthMarginM = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("dynamicObjectCase2DepthMarginM")), config.dynamicObjectCase2DepthMarginM).toDouble(),
        config.dynamicObjectCase2DepthMarginM);
    config.dynamicObjectCase3DepthMarginM = validPositiveDouble(
        settings.value(key(prefix, QStringLiteral("dynamicObjectCase3DepthMarginM")), config.dynamicObjectCase3DepthMarginM).toDouble(),
        config.dynamicObjectCase3DepthMarginM);
    config.dynamicObjectCase1VoteThreshold = validPositiveInt(
        settings.value(key(prefix, QStringLiteral("dynamicObjectCase1VoteThreshold")), config.dynamicObjectCase1VoteThreshold).toInt(),
        config.dynamicObjectCase1VoteThreshold);
    config.dynamicObjectCase2OcclusionChainLength = validPositiveInt(
        settings.value(key(prefix, QStringLiteral("dynamicObjectCase2OcclusionChainLength")),
                       config.dynamicObjectCase2OcclusionChainLength).toInt(),
        config.dynamicObjectCase2OcclusionChainLength);
    config.dynamicObjectCase3OcclusionChainLength = validPositiveInt(
        settings.value(key(prefix, QStringLiteral("dynamicObjectCase3OcclusionChainLength")),
                       config.dynamicObjectCase3OcclusionChainLength).toInt(),
        config.dynamicObjectCase3OcclusionChainLength);
    const QString freeDomPrefix = key(prefix, QStringLiteral("freeDom"));
    config.freeDom.sensorMinRangeM = settings.value(
        key(freeDomPrefix, QStringLiteral("sensorMinRangeM")),
        config.freeDom.sensorMinRangeM).toDouble();
    config.freeDom.sensorMaxRangeM = settings.value(
        key(freeDomPrefix, QStringLiteral("sensorMaxRangeM")),
        config.freeDom.sensorMaxRangeM).toDouble();
    config.freeDom.sensorMinZM = settings.value(
        key(freeDomPrefix, QStringLiteral("sensorMinZM")),
        config.freeDom.sensorMinZM).toDouble();
    config.freeDom.sensorMaxZM = settings.value(
        key(freeDomPrefix, QStringLiteral("sensorMaxZM")),
        config.freeDom.sensorMaxZM).toDouble();
    config.freeDom.subVoxelSizeM = settings.value(
        key(freeDomPrefix, QStringLiteral("subVoxelSizeM")),
        config.freeDom.subVoxelSizeM).toDouble();
    config.freeDom.voxelDepth = settings.value(
        key(freeDomPrefix, QStringLiteral("voxelDepth")),
        config.freeDom.voxelDepth).toUInt();
    config.freeDom.blockDepth = settings.value(
        key(freeDomPrefix, QStringLiteral("blockDepth")),
        config.freeDom.blockDepth).toUInt();
    config.freeDom.localMapEnabled = settings.value(
        key(freeDomPrefix, QStringLiteral("localMapEnabled")),
        config.freeDom.localMapEnabled).toBool();
    config.freeDom.localMapRangeM = settings.value(
        key(freeDomPrefix, QStringLiteral("localMapRangeM")),
        config.freeDom.localMapRangeM).toDouble();
    config.freeDom.localMapMinZM = settings.value(
        key(freeDomPrefix, QStringLiteral("localMapMinZM")),
        config.freeDom.localMapMinZM).toDouble();
    config.freeDom.localMapMaxZM = settings.value(
        key(freeDomPrefix, QStringLiteral("localMapMaxZM")),
        config.freeDom.localMapMaxZM).toDouble();
    config.freeDom.raycastMaxRangeM = settings.value(
        key(freeDomPrefix, QStringLiteral("raycastMaxRangeM")),
        config.freeDom.raycastMaxRangeM).toDouble();
    config.freeDom.raycastMinZM = settings.value(
        key(freeDomPrefix, QStringLiteral("raycastMinZM")),
        config.freeDom.raycastMinZM).toDouble();
    config.freeDom.raycastMaxZM = settings.value(
        key(freeDomPrefix, QStringLiteral("raycastMaxZM")),
        config.freeDom.raycastMaxZM).toDouble();
    config.freeDom.countsToFree = settings.value(
        key(freeDomPrefix, QStringLiteral("countsToFree")),
        config.freeDom.countsToFree).toUInt();
    config.freeDom.countsToRevert = settings.value(
        key(freeDomPrefix, QStringLiteral("countsToRevert")),
        config.freeDom.countsToRevert).toUInt();
    config.freeDom.conservativeConnectivity = settings.value(
        key(freeDomPrefix, QStringLiteral("conservativeConnectivity")),
        config.freeDom.conservativeConnectivity).toUInt();
    config.freeDom.aggressiveConnectivity = settings.value(
        key(freeDomPrefix, QStringLiteral("aggressiveConnectivity")),
        config.freeDom.aggressiveConnectivity).toUInt();
    config.freeDom.raycastEnhancementEnabled = settings.value(
        key(freeDomPrefix, QStringLiteral("raycastEnhancementEnabled")),
        config.freeDom.raycastEnhancementEnabled).toBool();
    config.freeDom.lidarHorizontalFovDeg = settings.value(
        key(freeDomPrefix, QStringLiteral("lidarHorizontalFovDeg")),
        config.freeDom.lidarHorizontalFovDeg).toDouble();
    config.freeDom.lidarVerticalFovUpperDeg = settings.value(
        key(freeDomPrefix, QStringLiteral("lidarVerticalFovUpperDeg")),
        config.freeDom.lidarVerticalFovUpperDeg).toDouble();
    config.freeDom.lidarVerticalFovLowerDeg = settings.value(
        key(freeDomPrefix, QStringLiteral("lidarVerticalFovLowerDeg")),
        config.freeDom.lidarVerticalFovLowerDeg).toDouble();
    config.freeDom.depthImageVerticalLines = settings.value(
        key(freeDomPrefix, QStringLiteral("depthImageVerticalLines")),
        config.freeDom.depthImageVerticalLines).toUInt();
    config.freeDom.depthImageMinRangeM = settings.value(
        key(freeDomPrefix, QStringLiteral("depthImageMinRangeM")),
        config.freeDom.depthImageMinRangeM).toDouble();
    config.freeDom.maxRaycastEnhancementRangeM = settings.value(
        key(freeDomPrefix, QStringLiteral("maxRaycastEnhancementRangeM")),
        config.freeDom.maxRaycastEnhancementRangeM).toDouble();
    config.freeDom.raycastEnhancementDepthMarginM = settings.value(
        key(freeDomPrefix, QStringLiteral("raycastEnhancementDepthMarginM")),
        config.freeDom.raycastEnhancementDepthMarginM).toDouble();
    config.freeDom.inpaintSize = settings.value(
        key(freeDomPrefix, QStringLiteral("inpaintSize")),
        config.freeDom.inpaintSize).toUInt();
    config.freeDom.erosionSize = settings.value(
        key(freeDomPrefix, QStringLiteral("erosionSize")),
        config.freeDom.erosionSize).toUInt();
    config.freeDom.minRaycastEnhancementArea = settings.value(
        key(freeDomPrefix, QStringLiteral("minRaycastEnhancementArea")),
        config.freeDom.minRaycastEnhancementArea).toDouble();
    config.freeDom.depthImageTopMargin = settings.value(
        key(freeDomPrefix, QStringLiteral("depthImageTopMargin")),
        config.freeDom.depthImageTopMargin).toDouble();
    config.freeDom.learnFov = settings.value(
        key(freeDomPrefix, QStringLiteral("learnFov")),
        config.freeDom.learnFov).toBool();
    config.freeDom.fovMaskEnabled = settings.value(
        key(freeDomPrefix, QStringLiteral("fovMaskEnabled")),
        config.freeDom.fovMaskEnabled).toBool();
    config.freeDom.fovMaskPath = settings.value(
        key(freeDomPrefix, QStringLiteral("fovMaskPath")),
        QString::fromStdString(config.freeDom.fovMaskPath)).toString().toStdString();
    config.freeDom.numThreads = settings.value(
        key(freeDomPrefix, QStringLiteral("numThreads")),
        config.freeDom.numThreads).toUInt();
    config.mapVoxelSizeM = settings.value(key(prefix, QStringLiteral("mapVoxelSizeM")), config.mapVoxelSizeM).toDouble();
    config.maxMapPoints = settings.value(key(prefix, QStringLiteral("maxMapPoints")), config.maxMapPoints).toInt();
    config.maxTrajectoryPoints = settings.value(key(prefix, QStringLiteral("maxTrajectoryPoints")), config.maxTrajectoryPoints).toInt();
    config.maxInputQueueFrames = settings.value(key(prefix, QStringLiteral("maxInputQueueFrames")), config.maxInputQueueFrames).toInt();
    config.saveTrajectory = settings.value(key(prefix, QStringLiteral("saveTrajectory")), config.saveTrajectory).toBool();
    config.saveMap = settings.value(key(prefix, QStringLiteral("saveMap")), config.saveMap).toBool();
    if (!hasPublishConfig) {
        config.saveMap = true;
    }
    config.logLevel = settings.value(key(prefix, QStringLiteral("logLevel")), config.logLevel).toString();
}

void saveSlamRuntimeConfigValues(QSettings& settings, const SlamRuntimeConfig& config, const QString& prefix)
{
    settings.setValue(key(prefix, QStringLiteral("backendType")), config.backendType);
    settings.setValue(key(prefix, QStringLiteral("lidarModel")), config.lidarModel);
    settings.setValue(key(prefix, QStringLiteral("lidarTemplate")), int(config.lidarTemplate));
    settings.setValue(key(prefix, QStringLiteral("imuEnabled")), config.imuEnabled);
    settings.setValue(key(prefix, QStringLiteral("allowPureLidar")), config.allowPureLidar);
    settings.setValue(key(prefix, QStringLiteral("lidarToImuTimeOffsetNs")), qlonglong(config.lidarToImuTimeOffsetNs));
    settings.setValue(key(prefix, QStringLiteral("gravityNorm")), config.gravityNorm);
    for (int i = 0; i < 3; ++i) {
        settings.setValue(key(prefix, QStringLiteral("extrinsicT_L_I/%1").arg(i)), config.extrinsicT_L_I[i]);
    }
    for (int i = 0; i < 9; ++i) {
        settings.setValue(key(prefix, QStringLiteral("extrinsicR_L_I/%1").arg(i)), config.extrinsicR_L_I[i]);
    }
    settings.setValue(key(prefix, QStringLiteral("extrinsicEstimationEnabled")), config.extrinsicEstimationEnabled);
    settings.setValue(key(prefix, QStringLiteral("cubeSideLengthM")), config.cubeSideLengthM);
    settings.setValue(key(prefix, QStringLiteral("detRangeM")), config.detRangeM);
    settings.setValue(key(prefix, QStringLiteral("fovDegree")), config.fovDegree);
    settings.setValue(key(prefix, QStringLiteral("blindMinRangeM")), config.blindMinRangeM);
    settings.setValue(key(prefix, QStringLiteral("pointFilterNum")), config.pointFilterNum);
    settings.setValue(key(prefix, QStringLiteral("gyrCov")), config.gyrCov);
    settings.setValue(key(prefix, QStringLiteral("accCov")), config.accCov);
    settings.setValue(key(prefix, QStringLiteral("bGyrCov")), config.bGyrCov);
    settings.setValue(key(prefix, QStringLiteral("bAccCov")), config.bAccCov);
    settings.setValue(key(prefix, QStringLiteral("maxIterations")), config.maxIterations);
    settings.setValue(key(prefix, QStringLiteral("filterSizeSurfM")), config.filterSizeSurfM);
    settings.setValue(key(prefix, QStringLiteral("filterSizeMapM")), config.filterSizeMapM);
    settings.setValue(key(prefix, QStringLiteral("odometrySurfLeafSize")), config.odometrySurfLeafSize);
    settings.setValue(key(prefix, QStringLiteral("mappingCornerLeafSize")), config.mappingCornerLeafSize);
    settings.setValue(key(prefix, QStringLiteral("mappingSurfLeafSize")), config.mappingSurfLeafSize);
    settings.setValue(key(prefix, QStringLiteral("zTolerance")), config.zTolerance);
    settings.setValue(key(prefix, QStringLiteral("rotationTolerance")), config.rotationTolerance);
    settings.setValue(key(prefix, QStringLiteral("numberOfCores")), config.numberOfCores);
    settings.setValue(key(prefix, QStringLiteral("mappingProcessInterval")), config.mappingProcessInterval);
    settings.setValue(key(prefix, QStringLiteral("surroundingKeyframeAddingDistThreshold")),
                      config.surroundingKeyframeAddingDistThreshold);
    settings.setValue(key(prefix, QStringLiteral("surroundingKeyframeAddingAngleThreshold")),
                      config.surroundingKeyframeAddingAngleThreshold);
    settings.setValue(key(prefix, QStringLiteral("surroundingKeyframeDensity")), config.surroundingKeyframeDensity);
    settings.setValue(key(prefix, QStringLiteral("surroundingKeyframeSearchRadius")),
                      config.surroundingKeyframeSearchRadius);
    settings.setValue(key(prefix, QStringLiteral("loopClosureEnableFlag")), config.loopClosureEnableFlag);
    settings.setValue(key(prefix, QStringLiteral("loopClosureFrequency")), config.loopClosureFrequency);
    settings.setValue(key(prefix, QStringLiteral("surroundingKeyframeSize")), config.surroundingKeyframeSize);
    settings.setValue(key(prefix, QStringLiteral("historyKeyframeSearchRadius")), config.historyKeyframeSearchRadius);
    settings.setValue(key(prefix, QStringLiteral("historyKeyframeSearchTimeDiff")), config.historyKeyframeSearchTimeDiff);
    settings.setValue(key(prefix, QStringLiteral("historyKeyframeSearchNum")), config.historyKeyframeSearchNum);
    settings.setValue(key(prefix, QStringLiteral("historyKeyframeFitnessScore")), config.historyKeyframeFitnessScore);
    settings.setValue(key(prefix, QStringLiteral("globalMapVisualizationSearchRadius")),
                      config.globalMapVisualizationSearchRadius);
    settings.setValue(key(prefix, QStringLiteral("globalMapVisualizationPoseDensity")),
                      config.globalMapVisualizationPoseDensity);
    settings.setValue(key(prefix, QStringLiteral("globalMapVisualizationLeafSize")),
                      config.globalMapVisualizationLeafSize);
    settings.setValue(key(prefix, QStringLiteral("visualizeIkdtreeMap")), config.visualizeIkdtreeMap);
    settings.setValue(key(prefix, QStringLiteral("reconstructKdTree")), config.reconstructKdTree);
    settings.setValue(key(prefix, QStringLiteral("preprocessScanRateHz")), config.preprocessScanRateHz);
    settings.setValue(key(prefix, QStringLiteral("inputFrameDurationMs")), config.inputFrameDurationMs);
    settings.setValue(key(prefix, QStringLiteral("allowRosbagDriver2PointCloud2")), true);
    settings.setValue(key(prefix, QStringLiteral("allowRosbagDriverPointCloud2SynthesizedTime")), true);
    settings.setValue(key(prefix, QStringLiteral("publishWorldFrameCloud")), config.publishWorldFrameCloud);
    settings.setValue(key(prefix, QStringLiteral("publishDenseFrameCloud")), config.publishDenseFrameCloud);
    settings.setValue(key(prefix, QStringLiteral("publishBodyFrameCloud")), config.publishBodyFrameCloud);
    settings.setValue(key(prefix, QStringLiteral("dynamicFilterBackend")), int(config.dynamicFilterBackend));
    settings.setValue(key(prefix, QStringLiteral("dynamicFilterEnabled")), config.dynamicFilterEnabled);
    settings.setValue(key(prefix, QStringLiteral("dynamicPointRemovalEnabled")), config.dynamicPointRemovalEnabled);
    settings.setValue(key(prefix, QStringLiteral("dynamicDebugVisualizationEnabled")), config.dynamicDebugVisualizationEnabled);
    settings.setValue(key(prefix, QStringLiteral("dynamicDebugSnapshotIntervalFrames")),
                      config.dynamicDebugSnapshotIntervalFrames);
    settings.setValue(key(prefix, QStringLiteral("freeDomMapSnapshotIntervalFrames")),
                      config.freeDomMapSnapshotIntervalFrames);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectDetectionEnabled")), config.dynamicObjectDetectionEnabled);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectRemovalEnabled")), config.dynamicObjectRemovalEnabled);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectClusterEnabled")), config.dynamicObjectClusterEnabled);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectClusterVoxelSizeM")), config.dynamicObjectClusterVoxelSizeM);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectClusterExtendVoxel")), config.dynamicObjectClusterExtendVoxel);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectClusterMinVoxelCount")), config.dynamicObjectClusterMinVoxelCount);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectClusterTrustThreshold")), config.dynamicObjectClusterTrustThreshold);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectClusterGroundDistanceThresholdM")), config.dynamicObjectClusterGroundDistanceThresholdM);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectClusterGroundMaxAngleDeg")), config.dynamicObjectClusterGroundMaxAngleDeg);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectBufferDelaySec")), config.dynamicObjectBufferDelaySec);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectDepthMapDurationSec")), config.dynamicObjectDepthMapDurationSec);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectMaxDepthMaps")), config.dynamicObjectMaxDepthMaps);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectMinHistoryMaps")), config.dynamicObjectMinHistoryMaps);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectHorizontalResolutionRad")), config.dynamicObjectHorizontalResolutionRad);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectVerticalResolutionRad")), config.dynamicObjectVerticalResolutionRad);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectVerticalFovDownDeg")), config.dynamicObjectVerticalFovDownDeg);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectVerticalFovUpDeg")), config.dynamicObjectVerticalFovUpDeg);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectHorizontalFovRightDeg")), config.dynamicObjectHorizontalFovRightDeg);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectHorizontalFovLeftDeg")), config.dynamicObjectHorizontalFovLeftDeg);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectMinRangeM")), config.dynamicObjectMinRangeM);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectMaxRangeM")), config.dynamicObjectMaxRangeM);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectNeighborPixelRadius")), config.dynamicObjectNeighborPixelRadius);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectCase1DepthMarginM")), config.dynamicObjectCase1DepthMarginM);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectCase2DepthMarginM")), config.dynamicObjectCase2DepthMarginM);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectCase3DepthMarginM")), config.dynamicObjectCase3DepthMarginM);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectCase1VoteThreshold")), config.dynamicObjectCase1VoteThreshold);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectCase2OcclusionChainLength")),
                      config.dynamicObjectCase2OcclusionChainLength);
    settings.setValue(key(prefix, QStringLiteral("dynamicObjectCase3OcclusionChainLength")),
                      config.dynamicObjectCase3OcclusionChainLength);
    const QString freeDomPrefix = key(prefix, QStringLiteral("freeDom"));
    settings.setValue(key(freeDomPrefix, QStringLiteral("sensorMinRangeM")), config.freeDom.sensorMinRangeM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("sensorMaxRangeM")), config.freeDom.sensorMaxRangeM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("sensorMinZM")), config.freeDom.sensorMinZM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("sensorMaxZM")), config.freeDom.sensorMaxZM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("subVoxelSizeM")), config.freeDom.subVoxelSizeM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("voxelDepth")), config.freeDom.voxelDepth);
    settings.setValue(key(freeDomPrefix, QStringLiteral("blockDepth")), config.freeDom.blockDepth);
    settings.setValue(key(freeDomPrefix, QStringLiteral("localMapEnabled")), config.freeDom.localMapEnabled);
    settings.setValue(key(freeDomPrefix, QStringLiteral("localMapRangeM")), config.freeDom.localMapRangeM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("localMapMinZM")), config.freeDom.localMapMinZM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("localMapMaxZM")), config.freeDom.localMapMaxZM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("raycastMaxRangeM")), config.freeDom.raycastMaxRangeM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("raycastMinZM")), config.freeDom.raycastMinZM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("raycastMaxZM")), config.freeDom.raycastMaxZM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("countsToFree")), config.freeDom.countsToFree);
    settings.setValue(key(freeDomPrefix, QStringLiteral("countsToRevert")), config.freeDom.countsToRevert);
    settings.setValue(key(freeDomPrefix, QStringLiteral("conservativeConnectivity")), config.freeDom.conservativeConnectivity);
    settings.setValue(key(freeDomPrefix, QStringLiteral("aggressiveConnectivity")), config.freeDom.aggressiveConnectivity);
    settings.setValue(key(freeDomPrefix, QStringLiteral("raycastEnhancementEnabled")), config.freeDom.raycastEnhancementEnabled);
    settings.setValue(key(freeDomPrefix, QStringLiteral("lidarHorizontalFovDeg")), config.freeDom.lidarHorizontalFovDeg);
    settings.setValue(key(freeDomPrefix, QStringLiteral("lidarVerticalFovUpperDeg")), config.freeDom.lidarVerticalFovUpperDeg);
    settings.setValue(key(freeDomPrefix, QStringLiteral("lidarVerticalFovLowerDeg")), config.freeDom.lidarVerticalFovLowerDeg);
    settings.setValue(key(freeDomPrefix, QStringLiteral("depthImageVerticalLines")), config.freeDom.depthImageVerticalLines);
    settings.setValue(key(freeDomPrefix, QStringLiteral("depthImageMinRangeM")), config.freeDom.depthImageMinRangeM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("maxRaycastEnhancementRangeM")), config.freeDom.maxRaycastEnhancementRangeM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("raycastEnhancementDepthMarginM")), config.freeDom.raycastEnhancementDepthMarginM);
    settings.setValue(key(freeDomPrefix, QStringLiteral("inpaintSize")), config.freeDom.inpaintSize);
    settings.setValue(key(freeDomPrefix, QStringLiteral("erosionSize")), config.freeDom.erosionSize);
    settings.setValue(key(freeDomPrefix, QStringLiteral("minRaycastEnhancementArea")), config.freeDom.minRaycastEnhancementArea);
    settings.setValue(key(freeDomPrefix, QStringLiteral("depthImageTopMargin")), config.freeDom.depthImageTopMargin);
    settings.setValue(key(freeDomPrefix, QStringLiteral("learnFov")), config.freeDom.learnFov);
    settings.setValue(key(freeDomPrefix, QStringLiteral("fovMaskEnabled")), config.freeDom.fovMaskEnabled);
    settings.setValue(key(freeDomPrefix, QStringLiteral("fovMaskPath")), QString::fromStdString(config.freeDom.fovMaskPath));
    settings.setValue(key(freeDomPrefix, QStringLiteral("numThreads")), config.freeDom.numThreads);
    settings.setValue(key(prefix, QStringLiteral("mapVoxelSizeM")), config.mapVoxelSizeM);
    settings.setValue(key(prefix, QStringLiteral("maxMapPoints")), config.maxMapPoints);
    settings.setValue(key(prefix, QStringLiteral("maxTrajectoryPoints")), config.maxTrajectoryPoints);
    settings.setValue(key(prefix, QStringLiteral("maxInputQueueFrames")), config.maxInputQueueFrames);
    settings.setValue(key(prefix, QStringLiteral("saveTrajectory")), config.saveTrajectory);
    settings.setValue(key(prefix, QStringLiteral("saveMap")), config.saveMap);
    settings.setValue(key(prefix, QStringLiteral("logLevel")), config.logLevel);
}

} // namespace

SlamRuntimeConfig loadSlamRuntimeConfig(const QSettings& settings, const QString& prefix)
{
    const SlamLidarTemplate selectedTemplate = slamLidarTemplateFromInt(settings.value(
        key(prefix, QStringLiteral("lidarTemplate")),
        int(SlamLidarTemplate::Mid360Mid360S)).toInt());
    if (hasTemplateConfig(settings, prefix, selectedTemplate)) {
        return loadSlamRuntimeConfigForTemplate(settings, prefix, selectedTemplate);
    }

    SlamRuntimeConfig config;
    config.backendType = settings.value(key(prefix, QStringLiteral("backendType")), config.backendType).toString();
    config.lidarModel = settings.value(key(prefix, QStringLiteral("lidarModel")), config.lidarModel).toString();
    applySlamLidarTemplateDefaults(config, selectedTemplate);
    loadSlamRuntimeConfigValues(settings, prefix, config);
    config.lidarTemplate = selectedTemplate;
    return config;
}

SlamRuntimeConfig loadSlamRuntimeConfigForTemplate(const QSettings& settings,
                                                   const QString& prefix,
                                                   SlamLidarTemplate lidarTemplate)
{
    SlamRuntimeConfig config;
    config.backendType = settings.value(key(prefix, QStringLiteral("backendType")), config.backendType).toString();
    config.lidarModel = settings.value(key(prefix, QStringLiteral("lidarModel")), config.lidarModel).toString();
    applySlamLidarTemplateDefaults(config, lidarTemplate);
    const QString tPrefix = templateConfigPrefix(prefix, lidarTemplate);
    if (hasTemplateConfig(settings, prefix, lidarTemplate)) {
        loadSlamRuntimeConfigValues(settings, tPrefix, config);
        config.lidarTemplate = lidarTemplate;
    }
    return config;
}

void saveSlamRuntimeConfig(QSettings& settings, const SlamRuntimeConfig& config, const QString& prefix)
{
    saveSlamRuntimeConfigValues(settings, config, prefix);
    saveSlamRuntimeConfigForTemplate(settings, config, prefix);
}

void saveSlamRuntimeConfigForTemplate(QSettings& settings, const SlamRuntimeConfig& config, const QString& prefix)
{
    const QString tPrefix = templateConfigPrefix(prefix, config.lidarTemplate);
    saveSlamRuntimeConfigValues(settings, config, tPrefix);
    settings.setValue(key(tPrefix, QStringLiteral("configured")), true);
}

#include "Slam/Core/SlamRuntimeConfig.h"

#include <QVariant>

#include <algorithm>
#include <cmath>

namespace {

constexpr double kMid360ExtrinsicT_L_I[3] = {-0.011, -0.02329, 0.04412};
constexpr double kAviaExtrinsicT_L_I[3] = {0.04165, 0.02326, -0.0284};
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
    const double* extrinsicT = lidarTemplate == SlamLidarTemplate::Avia
        ? kAviaExtrinsicT_L_I
        : kMid360ExtrinsicT_L_I;
    for (int i = 0; i < 3; ++i) {
        config.extrinsicT_L_I[i] = extrinsicT[i];
    }
    for (int i = 0; i < 9; ++i) {
        config.extrinsicR_L_I[i] = kIdentityExtrinsicR_L_I[i];
    }
}

} // namespace

QString slamLidarTemplateDisplayName(SlamLidarTemplate lidarTemplate)
{
    switch (lidarTemplate) {
    case SlamLidarTemplate::Avia:
        return QStringLiteral("Avia");
    case SlamLidarTemplate::Mid360Mid360S:
    default:
        return QStringLiteral("Mid360/Mid360S");
    }
}

SlamLidarTemplate slamLidarTemplateFromInt(int value)
{
    switch (value) {
    case int(SlamLidarTemplate::Avia):
        return SlamLidarTemplate::Avia;
    case int(SlamLidarTemplate::Mid360Mid360S):
    default:
        return SlamLidarTemplate::Mid360Mid360S;
    }
}

void applySlamLidarTemplateDefaults(SlamRuntimeConfig& config, SlamLidarTemplate lidarTemplate)
{
    config.lidarTemplate = lidarTemplate;
    assignTemplateDefaultExtrinsic(config, lidarTemplate);
    if (lidarTemplate == SlamLidarTemplate::Avia) {
        config.detRangeM = 450.0;
        config.fovDegree = 90.0;
        config.blindMinRangeM = 4.0;
        return;
    }

    config.detRangeM = 100.0;
    config.fovDegree = 360.0;
    config.blindMinRangeM = 0.5;
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
    settings.setValue(key(prefix, QStringLiteral("gyrCov")), config.gyrCov);
    settings.setValue(key(prefix, QStringLiteral("accCov")), config.accCov);
    settings.setValue(key(prefix, QStringLiteral("bGyrCov")), config.bGyrCov);
    settings.setValue(key(prefix, QStringLiteral("bAccCov")), config.bAccCov);
    settings.setValue(key(prefix, QStringLiteral("maxIterations")), config.maxIterations);
    settings.setValue(key(prefix, QStringLiteral("filterSizeSurfM")), config.filterSizeSurfM);
    settings.setValue(key(prefix, QStringLiteral("filterSizeMapM")), config.filterSizeMapM);
    settings.setValue(key(prefix, QStringLiteral("preprocessScanRateHz")), config.preprocessScanRateHz);
    settings.setValue(key(prefix, QStringLiteral("inputFrameDurationMs")), config.inputFrameDurationMs);
    settings.setValue(key(prefix, QStringLiteral("allowRosbagDriver2PointCloud2")), true);
    settings.setValue(key(prefix, QStringLiteral("allowRosbagDriverPointCloud2SynthesizedTime")), true);
    settings.setValue(key(prefix, QStringLiteral("publishWorldFrameCloud")), config.publishWorldFrameCloud);
    settings.setValue(key(prefix, QStringLiteral("publishDenseFrameCloud")), config.publishDenseFrameCloud);
    settings.setValue(key(prefix, QStringLiteral("publishBodyFrameCloud")), config.publishBodyFrameCloud);
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

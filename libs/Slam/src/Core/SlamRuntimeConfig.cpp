#include "Slam/Core/SlamRuntimeConfig.h"

#include <QVariant>

namespace {

QString key(const QString& prefix, const QString& name)
{
    return prefix + QLatin1Char('/') + name;
}

} // namespace

SlamRuntimeConfig loadSlamRuntimeConfig(const QSettings& settings, const QString& prefix)
{
    SlamRuntimeConfig config;
    config.backendType = settings.value(key(prefix, QStringLiteral("backendType")), config.backendType).toString();
    config.lidarModel = settings.value(key(prefix, QStringLiteral("lidarModel")), config.lidarModel).toString();
    config.imuEnabled = settings.value(key(prefix, QStringLiteral("imuEnabled")), config.imuEnabled).toBool();
    config.allowPureLidar = settings.value(key(prefix, QStringLiteral("allowPureLidar")), config.allowPureLidar).toBool();
    config.lidarToImuTimeOffsetNs = settings.value(key(prefix, QStringLiteral("lidarToImuTimeOffsetNs")),
                                                   qlonglong(config.lidarToImuTimeOffsetNs)).toLongLong();
    config.gravityNorm = settings.value(key(prefix, QStringLiteral("gravityNorm")), config.gravityNorm).toDouble();
    for (int i = 0; i < 3; ++i) {
        config.extrinsicT_L_I[i] = settings.value(key(prefix, QStringLiteral("extrinsicT_L_I/%1").arg(i)),
                                                  config.extrinsicT_L_I[i]).toDouble();
    }
    for (int i = 0; i < 9; ++i) {
        config.extrinsicR_L_I[i] = settings.value(key(prefix, QStringLiteral("extrinsicR_L_I/%1").arg(i)),
                                                  config.extrinsicR_L_I[i]).toDouble();
    }
    config.filterSizeSurfM = settings.value(key(prefix, QStringLiteral("filterSizeSurfM")), config.filterSizeSurfM).toDouble();
    config.filterSizeMapM = settings.value(key(prefix, QStringLiteral("filterSizeMapM")), config.filterSizeMapM).toDouble();
    config.mapVoxelSizeM = settings.value(key(prefix, QStringLiteral("mapVoxelSizeM")), config.mapVoxelSizeM).toDouble();
    config.maxMapPoints = settings.value(key(prefix, QStringLiteral("maxMapPoints")), config.maxMapPoints).toInt();
    config.maxTrajectoryPoints = settings.value(key(prefix, QStringLiteral("maxTrajectoryPoints")), config.maxTrajectoryPoints).toInt();
    config.maxInputQueueFrames = settings.value(key(prefix, QStringLiteral("maxInputQueueFrames")), config.maxInputQueueFrames).toInt();
    config.saveTrajectory = settings.value(key(prefix, QStringLiteral("saveTrajectory")), config.saveTrajectory).toBool();
    config.saveMap = settings.value(key(prefix, QStringLiteral("saveMap")), config.saveMap).toBool();
    config.logLevel = settings.value(key(prefix, QStringLiteral("logLevel")), config.logLevel).toString();
    return config;
}

void saveSlamRuntimeConfig(QSettings& settings, const SlamRuntimeConfig& config, const QString& prefix)
{
    settings.setValue(key(prefix, QStringLiteral("backendType")), config.backendType);
    settings.setValue(key(prefix, QStringLiteral("lidarModel")), config.lidarModel);
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
    settings.setValue(key(prefix, QStringLiteral("filterSizeSurfM")), config.filterSizeSurfM);
    settings.setValue(key(prefix, QStringLiteral("filterSizeMapM")), config.filterSizeMapM);
    settings.setValue(key(prefix, QStringLiteral("mapVoxelSizeM")), config.mapVoxelSizeM);
    settings.setValue(key(prefix, QStringLiteral("maxMapPoints")), config.maxMapPoints);
    settings.setValue(key(prefix, QStringLiteral("maxTrajectoryPoints")), config.maxTrajectoryPoints);
    settings.setValue(key(prefix, QStringLiteral("maxInputQueueFrames")), config.maxInputQueueFrames);
    settings.setValue(key(prefix, QStringLiteral("saveTrajectory")), config.saveTrajectory);
    settings.setValue(key(prefix, QStringLiteral("saveMap")), config.saveMap);
    settings.setValue(key(prefix, QStringLiteral("logLevel")), config.logLevel);
}

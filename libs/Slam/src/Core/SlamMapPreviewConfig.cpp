#include "Slam/Core/SlamMapPreviewConfig.h"

#include <algorithm>

namespace {

QString key(const QString& prefix, const QString& name)
{
    return prefix + QLatin1Char('/') + name;
}

int positiveInt(const QSettings& settings, const QString& name, int fallback)
{
    return std::max(1, settings.value(name, fallback).toInt());
}

double positiveDouble(const QSettings& settings, const QString& name, double fallback)
{
    return std::max(0.001, settings.value(name, fallback).toDouble());
}

} // namespace

SlamMapPreviewMode slamMapPreviewModeFromInt(int value)
{
    switch (value) {
    case int(SlamMapPreviewMode::GlobalSparse):
        return SlamMapPreviewMode::GlobalSparse;
    case int(SlamMapPreviewMode::GlobalDense):
        return SlamMapPreviewMode::GlobalDense;
    case int(SlamMapPreviewMode::Off):
    default:
        return SlamMapPreviewMode::Off;
    }
}

int slamMapPreviewModeToInt(SlamMapPreviewMode mode)
{
    return int(mode);
}

QString slamMapPreviewModeName(SlamMapPreviewMode mode)
{
    switch (mode) {
    case SlamMapPreviewMode::GlobalSparse:
        return QStringLiteral("全局稀疏");
    case SlamMapPreviewMode::GlobalDense:
        return QStringLiteral("全局稠密");
    case SlamMapPreviewMode::Off:
    default:
        return QStringLiteral("关闭");
    }
}

QString slamMapPreviewModeLogName(SlamMapPreviewMode mode)
{
    switch (mode) {
    case SlamMapPreviewMode::GlobalSparse:
        return QStringLiteral("全局稀疏预览");
    case SlamMapPreviewMode::GlobalDense:
        return QStringLiteral("全局稠密预览");
    case SlamMapPreviewMode::Off:
    default:
        return QStringLiteral("关闭");
    }
}

int slamMapPreviewMaxPoints(const SlamMapPreviewConfig& config)
{
    switch (config.mode) {
    case SlamMapPreviewMode::GlobalSparse:
        return std::max(1, config.globalSparseMaxPoints);
    case SlamMapPreviewMode::GlobalDense:
        return std::max(1, config.globalDenseMaxPoints);
    case SlamMapPreviewMode::Off:
    default:
        return 0;
    }
}

double slamMapPreviewVoxelSizeM(const SlamMapPreviewConfig& config)
{
    switch (config.mode) {
    case SlamMapPreviewMode::GlobalSparse:
        return std::max(0.001, config.globalSparseVoxelSizeM);
    case SlamMapPreviewMode::GlobalDense:
        return std::max(0.001, config.globalDenseVoxelSizeM);
    case SlamMapPreviewMode::Off:
    default:
        return 0.0;
    }
}

int slamMapPreviewUploadPointsPerTick(const SlamMapPreviewConfig& config)
{
    switch (config.mode) {
    case SlamMapPreviewMode::GlobalSparse:
        return std::max(1, config.globalSparseUploadPointsPerTick);
    case SlamMapPreviewMode::GlobalDense:
        return std::max(1, config.globalDenseUploadPointsPerTick);
    case SlamMapPreviewMode::Off:
    default:
        return 0;
    }
}

SlamMapPreviewConfig loadSlamMapPreviewConfig(const QSettings& settings, const QString& prefix)
{
    SlamMapPreviewConfig config;
    if (settings.contains(key(prefix, QStringLiteral("mode")))) {
        config.mode = slamMapPreviewModeFromInt(settings.value(key(prefix, QStringLiteral("mode")),
                                                               slamMapPreviewModeToInt(config.mode)).toInt());
    } else {
        const bool legacyEnabled = settings.value(QStringLiteral("slam/ui/mapPreviewDefaultEnabled"), false).toBool();
        config.mode = legacyEnabled ? SlamMapPreviewMode::GlobalSparse : SlamMapPreviewMode::Off;
    }

    config.globalSparseMaxPoints = positiveInt(settings,
                                               key(prefix, QStringLiteral("globalSparseMaxPoints")),
                                               settings.value(QStringLiteral("slam/ui/mapPreviewMaxPoints"),
                                                              config.globalSparseMaxPoints).toInt());
    config.globalSparseVoxelSizeM = positiveDouble(settings,
                                                   key(prefix, QStringLiteral("globalSparseVoxelSizeM")),
                                                   config.globalSparseVoxelSizeM);
    config.globalSparseUploadPointsPerTick = positiveInt(settings,
                                                         key(prefix, QStringLiteral("globalSparseUploadPointsPerTick")),
                                                         config.globalSparseUploadPointsPerTick);
    config.globalDenseMaxPoints = positiveInt(settings,
                                              key(prefix, QStringLiteral("globalDenseMaxPoints")),
                                              config.globalDenseMaxPoints);
    config.globalDenseVoxelSizeM = positiveDouble(settings,
                                                  key(prefix, QStringLiteral("globalDenseVoxelSizeM")),
                                                  config.globalDenseVoxelSizeM);
    config.globalDenseUploadPointsPerTick = positiveInt(settings,
                                                        key(prefix, QStringLiteral("globalDenseUploadPointsPerTick")),
                                                        config.globalDenseUploadPointsPerTick);
    return config;
}

void saveSlamMapPreviewConfig(QSettings& settings, const SlamMapPreviewConfig& config, const QString& prefix)
{
    settings.setValue(key(prefix, QStringLiteral("mode")), slamMapPreviewModeToInt(config.mode));
    settings.setValue(key(prefix, QStringLiteral("globalSparseMaxPoints")), config.globalSparseMaxPoints);
    settings.setValue(key(prefix, QStringLiteral("globalSparseVoxelSizeM")), config.globalSparseVoxelSizeM);
    settings.setValue(key(prefix, QStringLiteral("globalSparseUploadPointsPerTick")), config.globalSparseUploadPointsPerTick);
    settings.setValue(key(prefix, QStringLiteral("globalDenseMaxPoints")), config.globalDenseMaxPoints);
    settings.setValue(key(prefix, QStringLiteral("globalDenseVoxelSizeM")), config.globalDenseVoxelSizeM);
    settings.setValue(key(prefix, QStringLiteral("globalDenseUploadPointsPerTick")), config.globalDenseUploadPointsPerTick);
}

#ifndef SLAM_CORE_SLAMMAPPREVIEWCONFIG_H
#define SLAM_CORE_SLAMMAPPREVIEWCONFIG_H

#include <QSettings>
#include <QString>

enum class SlamMapPreviewMode {
    Off = 0,
    GlobalSparse = 1,
    GlobalDense = 2
};

struct SlamMapPreviewConfig {
    SlamMapPreviewMode mode = SlamMapPreviewMode::Off;
    int globalSparseMaxPoints = 2000000;
    double globalSparseVoxelSizeM = 0.10;
    int globalSparseUploadPointsPerTick = 20000;
    int globalDenseMaxPoints = 20000000;
    double globalDenseVoxelSizeM = 0.05;
    int globalDenseUploadPointsPerTick = 20000;
};

SlamMapPreviewMode slamMapPreviewModeFromInt(int value);
int slamMapPreviewModeToInt(SlamMapPreviewMode mode);
QString slamMapPreviewModeName(SlamMapPreviewMode mode);
QString slamMapPreviewModeLogName(SlamMapPreviewMode mode);
int slamMapPreviewMaxPoints(const SlamMapPreviewConfig& config);
double slamMapPreviewVoxelSizeM(const SlamMapPreviewConfig& config);
int slamMapPreviewUploadPointsPerTick(const SlamMapPreviewConfig& config);

SlamMapPreviewConfig loadSlamMapPreviewConfig(const QSettings& settings, const QString& prefix);
void saveSlamMapPreviewConfig(QSettings& settings, const SlamMapPreviewConfig& config, const QString& prefix);

#endif // SLAM_CORE_SLAMMAPPREVIEWCONFIG_H

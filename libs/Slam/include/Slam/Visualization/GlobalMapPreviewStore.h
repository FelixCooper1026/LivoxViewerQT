#ifndef SLAM_VISUALIZATION_GLOBALMAPPREVIEWSTORE_H
#define SLAM_VISUALIZATION_GLOBALMAPPREVIEWSTORE_H

#include "Slam/Core/SlamMapPreviewConfig.h"
#include "Slam/Core/SlamTypes.h"
#include "Slam/Visualization/SlamRenderSnapshot.h"

#include <QHash>
#include <QVector>

#include <cstdint>

struct SlamMapPreviewVoxelKey {
    qint64 x = 0;
    qint64 y = 0;
    qint64 z = 0;
    bool operator==(const SlamMapPreviewVoxelKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

size_t qHash(const SlamMapPreviewVoxelKey& key, size_t seed = 0);

class GlobalMapPreviewStore
{
public:
    struct PendingUpload {
        QVector<SlamRenderVertex> appendedPoints;
        QVector<SlamRenderPointUpdate> updatedPoints;
    };

    void configure(const SlamMapPreviewConfig& config);
    void clear();
    void appendMapChunk(const SlamMapChunk& chunk);
    PendingUpload takePendingUploadPoints(int maxPoints);

    SlamMapPreviewMode mode() const { return m_config.mode; }
    int previewPointCount() const { return m_points.size(); }
    int pendingPointCount() const;
    int maxPreviewPoints() const { return slamMapPreviewMaxPoints(m_config); }
    double voxelSizeM() const { return slamMapPreviewVoxelSizeM(m_config); }
    int uploadPointsPerTick() const { return slamMapPreviewUploadPointsPerTick(m_config); }
    bool reachedPointLimit() const { return m_reachedPointLimit; }
    bool enabled() const { return m_config.mode != SlamMapPreviewMode::Off; }
    QVector<SlamRenderVertex> pointsSnapshot() const { return m_points; }

private:
    SlamMapPreviewVoxelKey voxelKey(const SlamPoint& point) const;
    void appendVoxelPoint(const SlamMapPreviewVoxelKey& key, const SlamPoint& point);
    bool shouldReplaceExistingVoxel(const SlamMapPreviewVoxelKey& key) const;

    SlamMapPreviewConfig m_config;
    QVector<SlamRenderVertex> m_points;
    QVector<SlamMapPreviewVoxelKey> m_indexToVoxelKey;
    QHash<SlamMapPreviewVoxelKey, int> m_voxelToIndex;
    QVector<SlamRenderVertex> m_pendingAppends;
    QVector<SlamRenderPointUpdate> m_pendingUpdates;
    int m_pendingAppendHead = 0;
    int m_pendingUpdateHead = 0;
    int m_replacementCursor = 0;
    quint64 m_seenUniqueVoxelCount = 0;
    bool m_reachedPointLimit = false;
};

#endif // SLAM_VISUALIZATION_GLOBALMAPPREVIEWSTORE_H

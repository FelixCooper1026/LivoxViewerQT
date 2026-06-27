#include "Slam/Visualization/GlobalMapPreviewStore.h"

#include <algorithm>
#include <cmath>

namespace {

quint64 mix64(quint64 value)
{
    value ^= value >> 30;
    value *= 0xbf58476d1ce4e5b9ULL;
    value ^= value >> 27;
    value *= 0x94d049bb133111ebULL;
    value ^= value >> 31;
    return value;
}

quint64 voxelHash64(const SlamMapPreviewVoxelKey& key)
{
    quint64 h = 0x9e3779b97f4a7c15ULL;
    h ^= mix64(quint64(key.x) + 0x9e3779b97f4a7c15ULL);
    h = mix64(h);
    h ^= mix64(quint64(key.y) + 0xbf58476d1ce4e5b9ULL);
    h = mix64(h);
    h ^= mix64(quint64(key.z) + 0x94d049bb133111ebULL);
    return mix64(h);
}

SlamRenderVertex renderVertex(const SlamPoint& point)
{
    const float intensity = float(point.reflectivity) / 255.0f;
    const float c = 0.45f + 0.45f * intensity;
    return {point.x, point.y, point.z, c, c, c};
}

} // namespace

size_t qHash(const SlamMapPreviewVoxelKey& key, size_t seed)
{
    return size_t(voxelHash64(key) ^ quint64(seed));
}

void GlobalMapPreviewStore::configure(const SlamMapPreviewConfig& config)
{
    const bool changed = config.mode != m_config.mode ||
                         config.globalSparseMaxPoints != m_config.globalSparseMaxPoints ||
                         config.globalSparseVoxelSizeM != m_config.globalSparseVoxelSizeM ||
                         config.globalSparseUploadPointsPerTick != m_config.globalSparseUploadPointsPerTick ||
                         config.globalDenseMaxPoints != m_config.globalDenseMaxPoints ||
                         config.globalDenseVoxelSizeM != m_config.globalDenseVoxelSizeM ||
                         config.globalDenseUploadPointsPerTick != m_config.globalDenseUploadPointsPerTick;
    m_config = config;
    if (changed) {
        clear();
    }
}

void GlobalMapPreviewStore::clear()
{
    m_points.clear();
    m_indexToVoxelKey.clear();
    m_voxelToIndex.clear();
    m_pendingAppends.clear();
    m_pendingUpdates.clear();
    m_pendingAppendHead = 0;
    m_pendingUpdateHead = 0;
    m_replacementCursor = 0;
    m_seenUniqueVoxelCount = 0;
    m_reachedPointLimit = false;
}

void GlobalMapPreviewStore::appendMapChunk(const SlamMapChunk& chunk)
{
    if (!enabled()) {
        return;
    }
    for (const SlamPoint& point : chunk.pointsWorld) {
        const SlamMapPreviewVoxelKey key = voxelKey(point);
        if (m_voxelToIndex.contains(key)) {
            continue;
        }
        ++m_seenUniqueVoxelCount;
        appendVoxelPoint(key, point);
    }
}

GlobalMapPreviewStore::PendingUpload GlobalMapPreviewStore::takePendingUploadPoints(int maxPoints)
{
    PendingUpload upload;
    int remaining = std::max(0, maxPoints);
    while (remaining > 0 && m_pendingAppendHead < m_pendingAppends.size()) {
        upload.appendedPoints.push_back(m_pendingAppends.at(m_pendingAppendHead));
        ++m_pendingAppendHead;
        --remaining;
    }
    while (remaining > 0 && m_pendingUpdateHead < m_pendingUpdates.size()) {
        upload.updatedPoints.push_back(m_pendingUpdates.at(m_pendingUpdateHead));
        ++m_pendingUpdateHead;
        --remaining;
    }
    if (m_pendingAppendHead == m_pendingAppends.size()) {
        m_pendingAppends.clear();
        m_pendingAppendHead = 0;
    }
    if (m_pendingUpdateHead == m_pendingUpdates.size()) {
        m_pendingUpdates.clear();
        m_pendingUpdateHead = 0;
    }
    return upload;
}

int GlobalMapPreviewStore::pendingPointCount() const
{
    return (m_pendingAppends.size() - m_pendingAppendHead) +
           (m_pendingUpdates.size() - m_pendingUpdateHead);
}

SlamMapPreviewVoxelKey GlobalMapPreviewStore::voxelKey(const SlamPoint& point) const
{
    const double voxelSize = voxelSizeM();
    return {
        qint64(std::floor(double(point.x) / voxelSize)),
        qint64(std::floor(double(point.y) / voxelSize)),
        qint64(std::floor(double(point.z) / voxelSize))
    };
}

void GlobalMapPreviewStore::appendVoxelPoint(const SlamMapPreviewVoxelKey& key, const SlamPoint& point)
{
    const int maxPoints = maxPreviewPoints();
    if (maxPoints <= 0) {
        return;
    }

    const SlamRenderVertex vertex = renderVertex(point);
    if (m_points.size() < maxPoints) {
        const int index = m_points.size();
        m_points.push_back(vertex);
        m_indexToVoxelKey.push_back(key);
        m_voxelToIndex.insert(key, index);
        m_pendingAppends.push_back(vertex);
        return;
    }

    m_reachedPointLimit = true;
    if (!shouldReplaceExistingVoxel(key)) {
        return;
    }

    const int index = m_replacementCursor;
    m_replacementCursor = (m_replacementCursor + 1) % maxPoints;
    m_voxelToIndex.remove(m_indexToVoxelKey.at(index));
    m_indexToVoxelKey[index] = key;
    m_voxelToIndex.insert(key, index);
    m_points[index] = vertex;
    m_pendingUpdates.push_back({index, vertex});
}

bool GlobalMapPreviewStore::shouldReplaceExistingVoxel(const SlamMapPreviewVoxelKey& key) const
{
    const int maxPoints = maxPreviewPoints();
    if (maxPoints <= 0 || m_seenUniqueVoxelCount == 0) {
        return false;
    }
    return (voxelHash64(key) % m_seenUniqueVoxelCount) < quint64(maxPoints);
}

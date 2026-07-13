#ifndef LVX2_LVX2READER_H
#define LVX2_LVX2READER_H

#include "PlaybackSource.h"

#include <atomic>
#include <memory>

class QFile;

namespace Lvx2 {

class Lvx2Reader final : public Playback::Source {
public:
    explicit Lvx2Reader(bool frameCacheEnabled = true,
                        const std::atomic_bool* cancellationRequested = nullptr);
    ~Lvx2Reader() override;

    bool load(const QString& filePath) override;
    Playback::SourceKind kind() const override;
    QString path() const override;
    QString errorMessage() const override;
    int frameCount() const override;
    QVector<Playback::DeviceInfo> devices() const override;
    bool readFrame(int frameIndex,
                   const QMap<uint32_t, bool>& deviceVisibility,
                   PointCloudFrame& frame) override;
    uint64_t nominalFrameDurationNs() const override;
    void invalidateCache() override;

private:
    bool ensurePlaybackFileOpen();

    QString filePath_;
    QString errorMessage_;
    QVector<Playback::FrameRef> frames_;
    QMap<uint32_t, Playback::Extrinsic> extrinsics_;
    QMap<uint32_t, int> lineCounts_;
    QVector<Playback::DeviceInfo> devices_;
    QVector<PointCloudFrame> frameCache_;
    QVector<bool> frameCacheValid_;
    std::unique_ptr<QFile> playbackFile_;
    uint64_t frameDurationNs_ = 50000000ULL;
    bool frameCacheEnabled_ = true;
    const std::atomic_bool* cancellationRequested_ = nullptr;
};

} // namespace Lvx2

#endif // LVX2_LVX2READER_H

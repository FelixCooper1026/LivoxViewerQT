#ifndef SLAM_IO_LVXSLAMSOURCE_H
#define SLAM_IO_LVXSLAMSOURCE_H

#include "Core/SlamTypes.h"

#include <QMap>
#include <QString>

#include <atomic>
#include <memory>

namespace Playback {
class Source;
}

class LvxSlamSource {
public:
    explicit LvxSlamSource(int frameDurationMs = 100);
    ~LvxSlamSource();

    bool open(const QString& filePath,
              const std::atomic_bool* cancellationRequested,
              QString* error);
    bool readNextFrame(SlamInputFrame* frame,
                       const std::atomic_bool* cancellationRequested,
                       QString* error);
    int estimatedFrameCount() const;
    QString summaryText() const;

private:
    std::unique_ptr<Playback::Source> source_;
    QMap<uint32_t, bool> visibility_;
    SlamInputFrame pendingFrame_;
    QString filePath_;
    int64_t frameDurationNs_ = 100000000;
    int64_t sourceFrameDurationNs_ = 50000000;
    int sourceFrameIndex_ = 0;
    int estimatedFrameCount_ = 0;
    uint32_t sourceId_ = 0;
    uint8_t deviceType_ = 0;
    uint64_t emittedFrameCount_ = 0;
    uint64_t pointCount_ = 0;
    bool lvx2_ = false;
};

#endif // SLAM_IO_LVXSLAMSOURCE_H

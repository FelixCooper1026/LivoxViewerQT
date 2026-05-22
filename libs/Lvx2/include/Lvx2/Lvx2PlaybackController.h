#ifndef LVX2_LVX2PLAYBACKCONTROLLER_H
#define LVX2_LVX2PLAYBACKCONTROLLER_H

#include "Lvx2/Lvx2Types.h"

#include <QMatrix4x4>
#include <cstdint>

namespace Lvx2Playback {

struct FrameIndex {
    uint64_t offset = 0;
    uint64_t nextOffset = 0;
    uint64_t frameIndex = 0;
};

struct Extrinsic {
    bool enabled = false;
    QMatrix4x4 transform;
};

enum class Mode {
    FrameByFrame = 0,
    SlidingWindow = 1
};

} // namespace Lvx2Playback

#endif // LVX2_LVX2PLAYBACKCONTROLLER_H

#ifndef LVX2_LVX2PLAYBACKCONTROLLER_H
#define LVX2_LVX2PLAYBACKCONTROLLER_H

#include "PlaybackSource.h"

namespace Lvx2Playback {

using FrameIndex = Playback::FrameRef;
using Extrinsic = Playback::Extrinsic;

enum class Mode {
    FrameByFrame = 0,
    SlidingWindow = 1
};

} // namespace Lvx2Playback

#endif // LVX2_LVX2PLAYBACKCONTROLLER_H

#ifndef FREEDOM_FREEDOMSNAPSHOTBUILDER_H
#define FREEDOM_FREEDOMSNAPSHOTBUILDER_H

#include "FreeDomFrameResult.h"

namespace freedom {
class DepthImage;
class FreeDOM;
class MRMap;
class ScanMap;
}

class FreeDomSnapshotBuilder {
public:
    static void appendScan(const freedom::ScanMap& scan, FreeDomDebugSnapshot& output);
    static void appendDepthImage(const freedom::DepthImage& depthImage,
                                 FreeDomDebugSnapshot& output);
    static void appendMap(const freedom::MRMap& map, FreeDomDebugSnapshot& output);
    static void buildMap(const freedom::FreeDOM& engine,
                         std::uint64_t version,
                         FreeDomMapSnapshot& output);
};

#endif // FREEDOM_FREEDOMSNAPSHOTBUILDER_H

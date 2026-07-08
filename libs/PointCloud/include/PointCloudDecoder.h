#ifndef POINTCLOUD_POINTCLOUDDECODER_H
#define POINTCLOUD_POINTCLOUDDECODER_H

#include "LidarSdkTypes.h"
#include "PointCloudFrame.h"
#include "PointCloudProjection.h"

#include <cstdint>

namespace PointCloudDecoder {

using DecodeOptions = PointCloudProjection::SphericalConfig;

bool decodeLivoxPacket(uint32_t handle,
                       const LivoxLidarEthernetPacket* packet,
                       const DecodeOptions& options,
                       PointCloudFrame& frame);

} // namespace PointCloudDecoder

#endif // POINTCLOUD_POINTCLOUDDECODER_H

#include "PointCloud/PointCloudDecoder.h"

#include "LivoxCore/LidarPacketUtils.h"

namespace PointCloudDecoder {

bool decodeLivoxPacket(uint32_t handle,
                       const LivoxLidarEthernetPacket* packet,
                       const DecodeOptions& options,
                       PointCloudFrame& frame)
{
    if (!packet || packet->dot_num == 0) {
        return false;
    }

    frame = PointCloudFrame();
    frame.timestamp = LivoxCore::parseLivoxTimestamp(packet->timestamp);
    frame.device_handle = handle;

    switch (packet->data_type) {
    case kLivoxLidarCartesianCoordinateHighData: {
        const auto* pointData = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(packet->data);
        for (uint32_t i = 0; i < packet->dot_num; ++i) {
            PointCloudPoint point{};
            point.x = pointData[i].x / 1000.0f;
            point.y = pointData[i].y / 1000.0f;
            point.z = pointData[i].z / 1000.0f;
            point.reflectivity = pointData[i].reflectivity;
            point.tag = pointData[i].tag;
            frame.points.append(point);
        }
        return true;
    }
    case kLivoxLidarCartesianCoordinateLowData: {
        const auto* pointData = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(packet->data);
        for (uint32_t i = 0; i < packet->dot_num; ++i) {
            PointCloudPoint point{};
            point.x = pointData[i].x / 100.0f;
            point.y = pointData[i].y / 100.0f;
            point.z = pointData[i].z / 100.0f;
            point.reflectivity = pointData[i].reflectivity;
            point.tag = pointData[i].tag;
            frame.points.append(point);
        }
        return true;
    }
    case kLivoxLidarSphericalCoordinateData: {
        const auto* pointData = reinterpret_cast<const LivoxLidarSpherPoint*>(packet->data);
        for (uint32_t i = 0; i < packet->dot_num; ++i) {
            frame.points.append(PointCloudProjection::projectSpherical(pointData[i].depth,
                                                                       pointData[i].theta,
                                                                       pointData[i].phi,
                                                                       pointData[i].reflectivity,
                                                                       pointData[i].tag,
                                                                       options));
        }
        return true;
    }
    case kLivoxLidarDoubleEchoData: {
        const auto* pointData = reinterpret_cast<const LivoxLidarDoubleEchoRawPoint*>(packet->data);
        for (uint32_t i = 0; i < packet->dot_num; ++i) {
            PointCloudPoint point1{};
            point1.x = pointData[i].x1 / 1000.0f;
            point1.y = pointData[i].y1 / 1000.0f;
            point1.z = pointData[i].z1 / 1000.0f;
            point1.reflectivity = pointData[i].reflectivity1;
            point1.tag = pointData[i].tag1;

            PointCloudPoint point2{};
            point2.x = pointData[i].x2 / 1000.0f;
            point2.y = pointData[i].y2 / 1000.0f;
            point2.z = pointData[i].z2 / 1000.0f;
            point2.reflectivity = pointData[i].reflectivity2;
            point2.tag = pointData[i].tag2;

            frame.points.append(point1);
            frame.points.append(point2);
        }
        return true;
    }
    default:
        return false;
    }
}

} // namespace PointCloudDecoder

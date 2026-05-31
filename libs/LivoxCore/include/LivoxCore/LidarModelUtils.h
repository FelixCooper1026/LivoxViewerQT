#ifndef LIVOXCORE_LIDARMODELUTILS_H
#define LIVOXCORE_LIDARMODELUTILS_H

#include "LivoxCore/LidarSdkTypes.h"

#include <cstdint>

namespace LivoxCore {

inline int lineCountForDeviceType(uint8_t devType)
{
    switch (devType) {
    case kLivoxLidarTypeMid360:
    case kLivoxLidarTypeMid360s:
        return 4;
    case kLivoxLidarTypeAvia2:
        return 1;
    default:
        return 1;
    }
}

inline uint8_t lineForPointIndex(int pointIndex, int lineCount)
{
    return static_cast<uint8_t>(pointIndex % lineCount);
}

} // namespace LivoxCore

#endif // LIVOXCORE_LIDARMODELUTILS_H

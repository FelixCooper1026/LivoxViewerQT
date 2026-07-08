#ifndef LIVOXCORE_LIDARPACKETUTILS_H
#define LIVOXCORE_LIDARPACKETUTILS_H

#include <cstdint>

namespace LivoxCore {

inline uint64_t parseLivoxTimestamp(const uint8_t* timestamp)
{
    uint64_t result = 0;
    for (int i = 7; i >= 0; --i) {
        result = (result << 8) | timestamp[i];
    }
    return result;
}

} // namespace LivoxCore

#endif // LIVOXCORE_LIDARPACKETUTILS_H

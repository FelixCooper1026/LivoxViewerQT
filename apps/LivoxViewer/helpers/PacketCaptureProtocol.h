#ifndef LIVOXVIEWER_HELPERS_PACKETCAPTUREPROTOCOL_H
#define LIVOXVIEWER_HELPERS_PACKETCAPTUREPROTOCOL_H

#include <cstdint>

namespace PacketCaptureProtocol {

inline constexpr std::uint32_t kStreamMagic = 0x4C564350;
inline constexpr std::uint32_t kCaptureStopped = 0xFFFFFFFFu;

struct StreamHeader {
    std::uint32_t magic = kStreamMagic;
    std::int32_t dataLinkType = 0;
};

struct PacketHeader {
    std::int64_t timestampSec = 0;
    std::int64_t timestampUsec = 0;
    std::uint32_t capturedLength = 0;
    std::uint32_t originalLength = 0;
};

} // namespace PacketCaptureProtocol

#endif // LIVOXVIEWER_HELPERS_PACKETCAPTUREPROTOCOL_H

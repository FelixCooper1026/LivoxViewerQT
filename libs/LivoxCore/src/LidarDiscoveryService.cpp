#include "LivoxCore/LidarDiscoveryService.h"

namespace LidarDiscoveryService {

QByteArray discoveryCommand()
{
    return QByteArray::fromHex("aa00180002000000000000000000000000000a9200000000");
}

bool isLocalDatagram(const QString& senderIp,
                     const QString& boundLocalIp,
                     const QSet<QString>& localIpv4Addresses)
{
    const bool boundToSpecificInterface = !(boundLocalIp == "0.0.0.0" || boundLocalIp.startsWith("169.254."));
    return boundToSpecificInterface && localIpv4Addresses.contains(senderIp);
}

bool parseDiscoveryResponse(const QByteArray& data, DiscoveryResponse* response)
{
    if (!response || data.size() < 48) {
        return false;
    }

    if (static_cast<unsigned char>(data[0]) != 0xAA ||
        static_cast<unsigned char>(data[1]) != 0x00) {
        return false;
    }

    const uint16_t length = (static_cast<unsigned char>(data[3]) << 8) |
                            static_cast<unsigned char>(data[2]);
    if (data.size() < length) {
        return false;
    }

    const uint16_t cmdId = (static_cast<unsigned char>(data[8]) << 8) |
                           static_cast<unsigned char>(data[9]);
    const uint8_t cmdType = static_cast<unsigned char>(data[10]);
    const uint8_t senderType = static_cast<unsigned char>(data[11]);
    if (cmdId != 0x0000 || cmdType != 0x01 || senderType != 0x01) {
        return false;
    }

    constexpr int dataOffset = 24;
    const uint8_t retCode = static_cast<unsigned char>(data[dataOffset]);
    if (retCode != 0x00) {
        return false;
    }

    const int ipOffset = dataOffset + 18;
    if (ipOffset + 4 > data.size()) {
        return false;
    }

    response->deviceType = static_cast<unsigned char>(data[dataOffset + 1]);
    response->serialNumber = QString::fromLatin1(data.mid(dataOffset + 2, 16)).trimmed();
    response->deviceIp = QString("%1.%2.%3.%4")
                             .arg(static_cast<unsigned char>(data[ipOffset]))
                             .arg(static_cast<unsigned char>(data[ipOffset + 1]))
                             .arg(static_cast<unsigned char>(data[ipOffset + 2]))
                             .arg(static_cast<unsigned char>(data[ipOffset + 3]));
    if (ipOffset + 6 <= data.size()) {
        response->commandPort = (static_cast<unsigned char>(data[ipOffset + 5]) << 8) |
                                static_cast<unsigned char>(data[ipOffset + 4]);
    }
    return true;
}

} // namespace LidarDiscoveryService

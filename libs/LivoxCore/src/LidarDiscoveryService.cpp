#include "LidarDiscoveryService.h"

namespace {

LidarDiscoveryService::DiscoveryResponse invalidResponse(const QString& message)
{
    LidarDiscoveryService::DiscoveryResponse response;
    response.errorMessage = message;
    return response;
}

uint8_t byteAt(const QByteArray& data, int index)
{
    return static_cast<uint8_t>(static_cast<unsigned char>(data.at(index)));
}

} // namespace

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

DiscoveryResponse parseDiscoveryResponse(const QByteArray& data)
{
    if (data.size() < 48) {
        return invalidResponse("packet too short");
    }

    if (byteAt(data, 0) != 0xAA) {
        return invalidResponse("invalid SOF");
    }
    if (byteAt(data, 1) != 0x00) {
        return invalidResponse("invalid protocol version");
    }

    const uint16_t length = (static_cast<uint16_t>(byteAt(data, 3)) << 8) |
                            static_cast<uint16_t>(byteAt(data, 2));
    if (length < 48 || data.size() < length) {
        return invalidResponse("invalid packet length");
    }

    const uint16_t cmdId = (static_cast<uint16_t>(byteAt(data, 8)) << 8) |
                           static_cast<uint16_t>(byteAt(data, 9));
    if (cmdId != 0x0000) {
        return invalidResponse("unexpected command id");
    }

    if (byteAt(data, 10) != 0x01) {
        return invalidResponse("unexpected command type");
    }
    if (byteAt(data, 11) != 0x01) {
        return invalidResponse("unexpected sender type");
    }

    constexpr int dataOffset = 24;
    if (data.size() < dataOffset + 24) {
        return invalidResponse("payload too short");
    }

    const uint8_t retCode = byteAt(data, dataOffset);
    if (retCode != 0x00) {
        return invalidResponse(QString("ret_code=0x%1").arg(retCode, 2, 16, QChar('0')));
    }

    constexpr int ipOffset = dataOffset + 18;
    if (data.size() < ipOffset + 6) {
        return invalidResponse("address payload too short");
    }

    DiscoveryResponse response;
    response.valid = true;
    response.deviceType = byteAt(data, dataOffset + 1);
    response.serialNumber = QString::fromLatin1(data.mid(dataOffset + 2, 16)).trimmed();
    response.deviceIp = QString("%1.%2.%3.%4")
                            .arg(byteAt(data, ipOffset))
                            .arg(byteAt(data, ipOffset + 1))
                            .arg(byteAt(data, ipOffset + 2))
                            .arg(byteAt(data, ipOffset + 3));
    response.commandPort = (static_cast<uint16_t>(byteAt(data, ipOffset + 5)) << 8) |
                           static_cast<uint16_t>(byteAt(data, ipOffset + 4));
    return response;
}

bool parseDiscoveryResponse(const QByteArray& data, DiscoveryResponse* response)
{
    if (!response) {
        return false;
    }
    *response = parseDiscoveryResponse(data);
    return response->valid;
}

} // namespace LidarDiscoveryService

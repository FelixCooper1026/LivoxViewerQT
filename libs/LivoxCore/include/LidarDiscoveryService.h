#ifndef LIVOXCORE_LIDARDISCOVERYSERVICE_H
#define LIVOXCORE_LIDARDISCOVERYSERVICE_H

#include <QByteArray>
#include <QHostAddress>
#include <QSet>
#include <QString>

#include <cstdint>

namespace LidarDiscoveryService {

struct DiscoveryResponse {
    bool valid = false;
    QString errorMessage;
    uint8_t deviceType = 0;
    QString serialNumber;
    QString deviceIp;
    uint16_t commandPort = 0;
};

QByteArray discoveryCommand();
bool isLocalDatagram(const QString& senderIp, const QString& boundLocalIp, const QSet<QString>& localIpv4Addresses);
DiscoveryResponse parseDiscoveryResponse(const QByteArray& data);
bool parseDiscoveryResponse(const QByteArray& data, DiscoveryResponse* response);

} // namespace LidarDiscoveryService

#endif // LIVOXCORE_LIDARDISCOVERYSERVICE_H

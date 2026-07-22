#ifndef APPCONFIG_NETWORKINTERFACESERVICE_H
#define APPCONFIG_NETWORKINTERFACESERVICE_H

#include <QSet>
#include <QString>
#include <QList>
#include <QNetworkInterface>

#include <optional>

namespace NetworkInterfaceService {

struct NetworkInterfaceInfo {
    QString systemName;
    QString displayName;
    QString ipv4;
    QString netmask;
    QString broadcast;
    bool isValidForLidar = false;
};

bool isInSameSubnet(const QString& hostIp, const QString& currentIp, const QString& subnetMask);
QString currentHostIp(const QString& selectedIp = QString());
bool hasWiredNetworkDeviceConnected();
QString firstWiredInterfaceHumanName();
QSet<QString> localIpv4Addresses();
QSet<QString> existingIpv4Addresses();
QString calculateCompatibleHostIp(const QString& deviceIp);
bool isLikelyVirtualOrWirelessInterface(const QNetworkInterface& iface);
bool isValidLidarInterface(const QNetworkInterface& iface);
QList<NetworkInterfaceInfo> availableLidarInterfaces();
QList<NetworkInterfaceInfo> availableCaptureInterfaces();
std::optional<NetworkInterfaceInfo> findInterfaceByName(const QString& name);
std::optional<NetworkInterfaceInfo> findInterfaceByIp(const QString& ip);

} // namespace NetworkInterfaceService

#endif // APPCONFIG_NETWORKINTERFACESERVICE_H

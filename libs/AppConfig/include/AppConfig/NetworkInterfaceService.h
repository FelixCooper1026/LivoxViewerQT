#ifndef APPCONFIG_NETWORKINTERFACESERVICE_H
#define APPCONFIG_NETWORKINTERFACESERVICE_H

#include <QSet>
#include <QString>
#include <QNetworkInterface>

namespace NetworkInterfaceService {

bool isInSameSubnet(const QString& hostIp, const QString& currentIp, const QString& subnetMask);
QString currentHostIp(const QString& selectedIp = QString());
bool hasWiredNetworkDeviceConnected();
QString firstWiredInterfaceHumanName();
QSet<QString> localIpv4Addresses();
QSet<QString> existingIpv4Addresses();
QString calculateCompatibleHostIp(const QString& deviceIp);

} // namespace NetworkInterfaceService

#endif // APPCONFIG_NETWORKINTERFACESERVICE_H

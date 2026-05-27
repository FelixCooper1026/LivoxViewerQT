#include "AppConfig/NetworkInterfaceService.h"

#include <QAbstractSocket>
#include <QHostAddress>
#include <QNetworkAddressEntry>
#include <QDebug>

namespace {

bool fillIpv4Info(const QNetworkInterface& iface, NetworkInterfaceService::NetworkInterfaceInfo* info = nullptr)
{
    for (const QNetworkAddressEntry& entry : iface.addressEntries()) {
        const QHostAddress& addr = entry.ip();
        if (addr.protocol() == QAbstractSocket::IPv4Protocol &&
            addr.toString() != "0.0.0.0" &&
            !addr.toString().startsWith("169.254.")) {
            if (info) {
                info->systemName = iface.name();
                info->displayName = iface.humanReadableName();
                info->ipv4 = addr.toString();
                info->netmask = entry.netmask().toString();
                info->broadcast = entry.broadcast().isNull() ? QString() : entry.broadcast().toString();
                info->isValidForLidar = true;
            }
            return true;
        }
    }
    return false;
}

} // namespace

namespace NetworkInterfaceService {

bool isLikelyVirtualOrWirelessInterface(const QNetworkInterface& iface)
{
    const QString text = QString("%1 %2")
        .arg(iface.name().toLower(), iface.humanReadableName().toLower());
    const QStringList excluded = {
        "wifi", "wi-fi", "wireless", "wlan", "bluetooth",
        "docker", "veth", "vmnet", "virtualbox", "vbox", "hyper-v",
        "tap", "tun", "tailscale", "zerotier", "loopback", "bridge"
    };

    for (const QString& keyword : excluded) {
        if (text.contains(keyword)) {
            return true;
        }
    }
    return iface.name().toLower().startsWith("br-");
}

bool isValidLidarInterface(const QNetworkInterface& iface)
{
    const auto flags = iface.flags();
    if (!(flags & QNetworkInterface::IsUp) ||
        !(flags & QNetworkInterface::IsRunning) ||
        (flags & QNetworkInterface::IsLoopBack) ||
        (flags & QNetworkInterface::IsPointToPoint)) {
        return false;
    }
    if (isLikelyVirtualOrWirelessInterface(iface)) {
        return false;
    }
    return fillIpv4Info(iface);
}

QList<NetworkInterfaceInfo> availableLidarInterfaces()
{
    QList<NetworkInterfaceInfo> interfaces;
    for (const QNetworkInterface& iface : QNetworkInterface::allInterfaces()) {
        if (!isValidLidarInterface(iface)) {
            continue;
        }
        NetworkInterfaceInfo info;
        if (fillIpv4Info(iface, &info)) {
            interfaces.append(info);
        }
    }
    return interfaces;
}

std::optional<NetworkInterfaceInfo> findInterfaceByName(const QString& name)
{
    for (const NetworkInterfaceInfo& info : availableLidarInterfaces()) {
        if (info.systemName == name) {
            return info;
        }
    }
    return std::nullopt;
}

std::optional<NetworkInterfaceInfo> findInterfaceByIp(const QString& ip)
{
    for (const NetworkInterfaceInfo& info : availableLidarInterfaces()) {
        if (info.ipv4 == ip) {
            return info;
        }
    }
    return std::nullopt;
}

bool isInSameSubnet(const QString& hostIp, const QString& currentIp, const QString& subnetMask)
{
    const QHostAddress hostAddr(hostIp);
    const QHostAddress currentAddr(currentIp);
    const QHostAddress maskAddr(subnetMask);
    if (hostAddr.isNull() || currentAddr.isNull() || maskAddr.isNull()) {
        return false;
    }
    return (hostAddr.toIPv4Address() & maskAddr.toIPv4Address()) ==
           (currentAddr.toIPv4Address() & maskAddr.toIPv4Address());
}

QString currentHostIp(const QString& selectedIp)
{
    if (!selectedIp.isEmpty()) {
        const auto selected = findInterfaceByIp(selectedIp);
        if (selected.has_value()) {
            return selected->ipv4;
        }
    }

    const QList<NetworkInterfaceInfo> interfaces = availableLidarInterfaces();
    if (!interfaces.isEmpty()) {
        return interfaces.first().ipv4;
    }
    return QString();
}

bool hasWiredNetworkDeviceConnected()
{
    const QList<NetworkInterfaceInfo> interfaces = availableLidarInterfaces();
    if (!interfaces.isEmpty()) {
        const NetworkInterfaceInfo& iface = interfaces.first();
        qDebug() << "[wired-interface] active interface:" << iface.displayName << "(" << iface.systemName << ")";
        return true;
    }
    qDebug() << "[wired-interface] no active wired interface detected";
    return false;
}

QString firstWiredInterfaceHumanName()
{
    const QList<NetworkInterfaceInfo> interfaces = availableLidarInterfaces();
    if (!interfaces.isEmpty()) {
        return interfaces.first().displayName;
    }
    return QString();
}

QSet<QString> localIpv4Addresses()
{
    QSet<QString> addresses;
    for (const QNetworkInterface& iface : QNetworkInterface::allInterfaces()) {
        const auto flags = iface.flags();
        if (!(flags & QNetworkInterface::IsUp) ||
            !(flags & QNetworkInterface::IsRunning) ||
            (flags & QNetworkInterface::IsLoopBack)) {
            continue;
        }
        for (const QNetworkAddressEntry& entry : iface.addressEntries()) {
            if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol) {
                addresses.insert(entry.ip().toString());
            }
        }
    }
    return addresses;
}

QSet<QString> existingIpv4Addresses()
{
    QSet<QString> addresses;
    for (const QNetworkInterface& iface : QNetworkInterface::allInterfaces()) {
        for (const QNetworkAddressEntry& entry : iface.addressEntries()) {
            if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol) {
                addresses.insert(entry.ip().toString());
            }
        }
    }
    return addresses;
}

QString calculateCompatibleHostIp(const QString& deviceIp)
{
    const QHostAddress deviceAddr(deviceIp);
    if (deviceAddr.isNull()) {
        return QString();
    }

    const quint32 deviceIpInt = deviceAddr.toIPv4Address();
    const quint32 networkPart = deviceIpInt & 0xFFFFFF00;
    for (int host = 2; host <= 254; ++host) {
        if (host == static_cast<int>(deviceIpInt & 0xFF)) {
            continue;
        }
        return QHostAddress(networkPart | host).toString();
    }
    return QString();
}

} // namespace NetworkInterfaceService

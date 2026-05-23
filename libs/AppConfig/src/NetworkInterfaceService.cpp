#include "AppConfig/NetworkInterfaceService.h"

#include <QAbstractSocket>
#include <QHostAddress>
#include <QNetworkAddressEntry>
#include <QDebug>

namespace {

bool isUsableInterface(const QNetworkInterface& iface)
{
    const auto flags = iface.flags();
    if (!(flags & QNetworkInterface::IsUp) ||
        !(flags & QNetworkInterface::IsRunning) ||
        (flags & QNetworkInterface::IsLoopBack) ||
        (flags & QNetworkInterface::IsPointToPoint)) {
        return false;
    }

    const QString name = iface.humanReadableName().toLower();
    const QString sysName = iface.name().toLower();
    if (name.contains("wifi") || name.contains("wi-fi") || name.contains("wlan") ||
        name.contains("wireless") || name.contains("bluetooth") || name.contains("bt") ||
        sysName.contains("wifi") || sysName.contains("wireless") || sysName.contains("wlan") ||
        sysName.contains("bt") || sysName.contains("bluetooth")) {
        return false;
    }

    if (sysName.contains("docker") || sysName.contains("veth") || sysName.startsWith("br-") ||
        sysName.contains("virbr") || sysName.contains("vmnet") || sysName.contains("vboxnet") ||
        sysName.contains("tap") || sysName.contains("tun") ||
        sysName.contains("loopback") || name.contains("virtual")) {
        return false;
    }

    return true;
}

bool hasValidIpv4(const QNetworkInterface& iface, QString* ip = nullptr)
{
    for (const QNetworkAddressEntry& entry : iface.addressEntries()) {
        const QHostAddress& addr = entry.ip();
        if (addr.protocol() == QAbstractSocket::IPv4Protocol &&
            addr.toString() != "0.0.0.0" &&
            !addr.toString().startsWith("169.254.")) {
            if (ip) {
                *ip = addr.toString();
            }
            return true;
        }
    }
    return false;
}

} // namespace

namespace NetworkInterfaceService {

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
        return selectedIp;
    }

    for (const QNetworkInterface& iface : QNetworkInterface::allInterfaces()) {
        if (!isUsableInterface(iface)) {
            continue;
        }
        QString ip;
        if (hasValidIpv4(iface, &ip)) {
            return ip;
        }
    }
    return QString();
}

bool hasWiredNetworkDeviceConnected()
{
    for (const QNetworkInterface& iface : QNetworkInterface::allInterfaces()) {
        if (!isUsableInterface(iface)) {
            continue;
        }
        if (hasValidIpv4(iface)) {
            qDebug() << "[wired-interface] active interface:"
                     << iface.humanReadableName() << "(" << iface.name() << ")";
            return true;
        }
        if (iface.flags() & QNetworkInterface::IsRunning) {
            qDebug() << "[wired-interface] physical link:" << iface.humanReadableName();
            return true;
        }
    }
    qDebug() << "[wired-interface] no active wired interface detected";
    return false;
}

QString firstWiredInterfaceHumanName()
{
    for (const QNetworkInterface& iface : QNetworkInterface::allInterfaces()) {
        if (!isUsableInterface(iface)) {
            continue;
        }
        const QString name = iface.humanReadableName().toLower();
        const QString sysName = iface.name().toLower();
        if (sysName.startsWith("ethernet") || name.contains("ethernet") || name.contains("以太网")) {
            return iface.humanReadableName();
        }
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

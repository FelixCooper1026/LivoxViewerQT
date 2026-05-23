#ifndef APPCONFIG_LIDARCONFIGSERVICE_H
#define APPCONFIG_LIDARCONFIGSERVICE_H

#include <QString>

#include <cstdint>

namespace LidarConfigService {

QString resolveConfigJsonPath(QString* migrationNote = nullptr);

bool checkNetworkCompatibility(const QString& configPath,
                               QString* details = nullptr,
                               const QString& selectedIp = QString());

bool updateHostIp(const QString& configPath,
                  const QString& newHostIp,
                  QString* message = nullptr);

bool updateResolvedConfigHostIp(const QString& newHostIp,
                                QString* message = nullptr,
                                QString* resolvedPath = nullptr);

bool updateDeviceTypeIfNeeded(const QString& configPath,
                              bool hasDiscoveredDeviceType,
                              uint8_t discoveredDeviceType,
                              QString* message = nullptr);

} // namespace LidarConfigService

#endif // APPCONFIG_LIDARCONFIGSERVICE_H

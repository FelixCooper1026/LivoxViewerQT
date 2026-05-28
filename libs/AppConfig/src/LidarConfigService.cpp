#include "AppConfig/LidarConfigService.h"

#include "AppConfig/NetworkInterfaceService.h"
#include "LivoxCore/LidarSdkTypes.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QMap>
#include <QStandardPaths>
#include <QStringList>
#include <QDebug>

namespace {

QString defaultConfigDir()
{
    QString dir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    if (dir.isEmpty()) {
        dir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    }
    return dir;
}

bool readJsonObject(const QString& path, QJsonObject* root, QString* message)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (message) {
            *message = QString("Cannot open config.json: %1").arg(QDir::toNativeSeparators(path));
        }
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError) {
        if (message) {
            *message = QString("config.json parse error: %1").arg(parseError.errorString());
        }
        return false;
    }

    *root = doc.object();
    return true;
}

bool writeJsonObject(const QString& path, const QJsonObject& root, QString* message)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (message) {
            *message = QString("Cannot write config.json: %1").arg(QDir::toNativeSeparators(path));
        }
        return false;
    }

    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return true;
}

QString expectedDeviceKey(uint8_t deviceType)
{
    switch (deviceType) {
    case kLivoxLidarTypeMid360:
        return "MID360";
    case kLivoxLidarTypeMid360s:
        return "Mid360s";
    case kLivoxLidarTypeHAP:
        return "HAP";
    case kLivoxLidarTypeAvia2:
        return "Avia2";
    default:
        return QString();
    }
}

bool isDeviceConfigKey(const QString& key, const QJsonValue& value)
{
    if (!value.isObject()) {
        return false;
    }
    if (key == "lidar_log_enable" || key == "lidar_log_cache_size_MB" || key == "lidar_log_path") {
        return false;
    }
    return value.toObject().contains("host_net_info");
}

QJsonObject defaultLidarNetInfo(const QString& deviceKey)
{
    QJsonObject lidarNet;
    if (deviceKey == "MID360" || deviceKey == "Mid360s" || deviceKey == "Avia2") {
        lidarNet.insert("cmd_data_port", 56100);
        lidarNet.insert("push_msg_port", 56200);
        lidarNet.insert("point_data_port", 56300);
        lidarNet.insert("imu_data_port", 56400);
        lidarNet.insert("log_data_port", 56500);
    } else {
        lidarNet.insert("cmd_data_port", 56000);
        lidarNet.insert("push_msg_port", 0);
        lidarNet.insert("point_data_port", 57000);
        lidarNet.insert("imu_data_port", 58000);
        lidarNet.insert("log_data_port", 59000);
    }
    return lidarNet;
}

QStringList orderedDeviceKeys()
{
    return {"Mid360s", "MID360", "Avia2", "HAP"};
}

} // namespace

namespace LidarConfigService {

QString resolveConfigJsonPath(QString* migrationNote)
{
    const QString cfgDir = defaultConfigDir();
    const QString standardPath = cfgDir.isEmpty() ? QString() : QDir(cfgDir).filePath("config.json");

    if (!standardPath.isEmpty() && QFile::exists(standardPath)) {
        return standardPath;
    }

    const QString appDirPath = QCoreApplication::applicationDirPath();
    const QStringList legacyCandidates = {
        QDir::currentPath() + "/config.json",
        appDirPath + "/config.json",
        appDirPath + "/../config.json",
    };

    QString legacyFound;
    for (const QString& path : legacyCandidates) {
        if (QFile::exists(path)) {
            legacyFound = path;
            break;
        }
    }

    if (!legacyFound.isEmpty() && !standardPath.isEmpty()) {
        QDir().mkpath(QFileInfo(standardPath).absolutePath());
        if (!QFile::exists(standardPath) && QFile::copy(legacyFound, standardPath)) {
            if (migrationNote) {
                *migrationNote = QString("config.json migrated to %1").arg(QDir::toNativeSeparators(standardPath));
            }
            return standardPath;
        }
        if (migrationNote) {
            *migrationNote = QString("Using legacy config.json: %1").arg(QDir::toNativeSeparators(legacyFound));
        }
        return legacyFound;
    }

    if (!legacyFound.isEmpty()) {
        if (migrationNote) {
            *migrationNote = QString("Using legacy config.json: %1").arg(QDir::toNativeSeparators(legacyFound));
        }
        return legacyFound;
    }

    if (!standardPath.isEmpty()) {
        QDir().mkpath(QFileInfo(standardPath).absolutePath());
    }
    return standardPath;
}

bool checkNetworkCompatibility(const QString& configPath, QString* details, const QString& selectedIp)
{
    QString message;
    QJsonObject root;
    if (!readJsonObject(configPath, &root, &message)) {
        qDebug() << message;
        if (details) {
            *details = message;
        }
        return false;
    }

    const QString currentHostIp = NetworkInterfaceService::currentHostIp(selectedIp);
    if (currentHostIp.isEmpty()) {
        if (details) {
            *details = "Cannot determine current host IP address";
        }
        return false;
    }

    QList<QPair<QString, QString>> deviceIpPairs;
    for (auto it = root.begin(); it != root.end(); ++it) {
        if (!it.value().isObject()) {
            continue;
        }
        const QJsonObject deviceObj = it.value().toObject();
        if (!deviceObj.value("host_net_info").isArray()) {
            continue;
        }
        for (const QJsonValue& value : deviceObj.value("host_net_info").toArray()) {
            if (!value.isObject()) {
                continue;
            }
            const QString hostIp = value.toObject().value("host_ip").toString();
            if (!hostIp.isEmpty()) {
                deviceIpPairs.append({it.key(), hostIp});
            }
        }
    }

    if (deviceIpPairs.isEmpty()) {
        if (details) {
            *details = "config.json does not contain any host_ip fields";
        }
        return false;
    }

    QStringList mismatchDetails;
    for (const auto& pair : deviceIpPairs) {
        if (pair.second != currentHostIp) {
            mismatchDetails << QString("[%1:%2]").arg(pair.first, pair.second);
        }
    }

    if (!mismatchDetails.isEmpty()) {
        if (details) {
            *details = QString("host_ip mismatch: %1; current host IP: %2")
                           .arg(mismatchDetails.join(", "), currentHostIp);
        }
        return false;
    }

    if (details) {
        *details = QString("config.json host_ip matches current host IP: %1").arg(currentHostIp);
    }
    return true;
}

bool updateHostIp(const QString& configPath, const QString& newHostIp, QString* message)
{
    QJsonObject root;
    if (!readJsonObject(configPath, &root, message)) {
        return false;
    }

    bool updated = false;
    for (auto it = root.begin(); it != root.end(); ++it) {
        if (!it.value().isObject()) {
            continue;
        }
        QJsonObject deviceObj = it.value().toObject();
        if (!deviceObj.value("host_net_info").isArray()) {
            continue;
        }
        QJsonArray hostInfoArray = deviceObj.value("host_net_info").toArray();
        for (int i = 0; i < hostInfoArray.size(); ++i) {
            QJsonObject hostInfo = hostInfoArray[i].toObject();
            if (hostInfo.contains("host_ip")) {
                hostInfo["host_ip"] = newHostIp;
                hostInfoArray[i] = hostInfo;
                updated = true;
            }
        }
        deviceObj["host_net_info"] = hostInfoArray;
        root[it.key()] = deviceObj;
    }

    if (!updated) {
        if (message) {
            *message = "config.json does not contain host_ip fields";
        }
        return false;
    }

    if (!writeJsonObject(configPath, root, message)) {
        return false;
    }
    if (message) {
        *message = QString("config.json host_ip updated to %1").arg(newHostIp);
    }
    return true;
}

bool updateResolvedConfigHostIp(const QString& newHostIp, QString* message, QString* resolvedPath)
{
    QString migrationNote;
    const QString configPath = resolveConfigJsonPath(&migrationNote);
    if (resolvedPath) {
        *resolvedPath = configPath;
    }
    if (configPath.isEmpty()) {
        if (message) {
            *message = "config.json path is empty";
        }
        return false;
    }
    return updateHostIp(configPath, newHostIp, message);
}

bool updateDeviceTypeIfNeeded(const QString& configPath,
                              bool hasDiscoveredDeviceType,
                              uint8_t discoveredDeviceType,
                              QString* message)
{
    if (!hasDiscoveredDeviceType) {
        return true;
    }

    const QString expectedKey = expectedDeviceKey(discoveredDeviceType);
    if (expectedKey.isEmpty()) {
        return true;
    }

    QJsonObject root;
    if (!readJsonObject(configPath, &root, message)) {
        return false;
    }

    QStringList deviceKeys;
    for (auto it = root.begin(); it != root.end(); ++it) {
        if (isDeviceConfigKey(it.key(), it.value())) {
            deviceKeys.append(it.key());
        }
    }

    if (deviceKeys.isEmpty() || deviceKeys.contains(expectedKey)) {
        return true;
    }

    if (deviceKeys.size() != 1) {
        if (message) {
            *message = "config.json has multiple device blocks; Device type was not changed automatically";
        }
        return true;
    }

    const QString oldKey = deviceKeys.first();
    const QJsonObject oldDevice = root.value(oldKey).toObject();
    if (!oldDevice.value("host_net_info").isArray()) {
        if (message) {
            *message = "device block does not contain host_net_info";
        }
        return false;
    }

    QJsonObject orderedRoot;
    if (root.contains("lidar_log_enable")) {
        orderedRoot.insert("lidar_log_enable", root.value("lidar_log_enable"));
    }
    if (root.contains("lidar_log_cache_size_MB")) {
        orderedRoot.insert("lidar_log_cache_size_MB", root.value("lidar_log_cache_size_MB"));
    }
    if (root.contains("lidar_log_path")) {
        orderedRoot.insert("lidar_log_path", root.value("lidar_log_path"));
    }

    QJsonObject newDevice;
    newDevice.insert("lidar_net_info", defaultLidarNetInfo(expectedKey));
    newDevice.insert("host_net_info", oldDevice.value("host_net_info").toArray());
    orderedRoot.insert(expectedKey, newDevice);

    if (!writeJsonObject(configPath, orderedRoot, message)) {
        return false;
    }
    if (message) {
        *message = QString("config.json Device type changed from %1 to %2").arg(oldKey, expectedKey);
    }
    return true;
}

bool updateDeviceTypesForDiscoveredTypes(const QString& configPath,
                                         const QSet<uint8_t>& discoveredDeviceTypes,
                                         QString* message)
{
    QSet<QString> expectedKeys;
    for (uint8_t deviceType : discoveredDeviceTypes) {
        const QString key = expectedDeviceKey(deviceType);
        if (!key.isEmpty()) {
            expectedKeys.insert(key);
        }
    }
    if (expectedKeys.isEmpty()) {
        return true;
    }

    QJsonObject root;
    if (!readJsonObject(configPath, &root, message)) {
        return false;
    }

    QMap<QString, QJsonObject> existingDevices;
    QJsonArray templateHostInfo;
    for (auto it = root.begin(); it != root.end(); ++it) {
        if (!isDeviceConfigKey(it.key(), it.value())) {
            continue;
        }
        const QJsonObject deviceObj = it.value().toObject();
        existingDevices.insert(it.key(), deviceObj);
        if (templateHostInfo.isEmpty() && deviceObj.value("host_net_info").isArray()) {
            templateHostInfo = deviceObj.value("host_net_info").toArray();
        }
    }

    if (templateHostInfo.isEmpty()) {
        if (message) {
            *message = "config.json does not contain host_net_info for discovered device types";
        }
        return false;
    }

    bool changed = false;
    QJsonObject orderedRoot;
    if (root.contains("lidar_log_enable")) {
        orderedRoot.insert("lidar_log_enable", root.value("lidar_log_enable"));
    }
    if (root.contains("lidar_log_cache_size_MB")) {
        orderedRoot.insert("lidar_log_cache_size_MB", root.value("lidar_log_cache_size_MB"));
    }
    if (root.contains("lidar_log_path")) {
        orderedRoot.insert("lidar_log_path", root.value("lidar_log_path"));
    }

    QStringList writtenKeys;
    for (const QString& key : orderedDeviceKeys()) {
        if (!expectedKeys.contains(key)) {
            continue;
        }

        QJsonObject deviceObj;
        if (existingDevices.contains(key)) {
            deviceObj = existingDevices.value(key);
            if (!deviceObj.value("host_net_info").isArray()) {
                deviceObj.insert("host_net_info", templateHostInfo);
                changed = true;
            }
            if (!deviceObj.value("lidar_net_info").isObject()) {
                deviceObj.insert("lidar_net_info", defaultLidarNetInfo(key));
                changed = true;
            }
        } else {
            deviceObj.insert("lidar_net_info", defaultLidarNetInfo(key));
            deviceObj.insert("host_net_info", templateHostInfo);
            changed = true;
        }
        orderedRoot.insert(key, deviceObj);
        writtenKeys.append(key);
    }

    for (const QString& key : existingDevices.keys()) {
        if (!expectedKeys.contains(key)) {
            changed = true;
            break;
        }
    }

    if (!changed) {
        return true;
    }

    if (!writeJsonObject(configPath, orderedRoot, message)) {
        return false;
    }
    if (message) {
        *message = QString("config.json device blocks updated for discovered types: %1").arg(writtenKeys.join(", "));
    }
    return true;
}

} // namespace LidarConfigService

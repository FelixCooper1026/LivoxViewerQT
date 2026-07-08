#ifndef LIVOXCORE_LIDARSDKSERVICE_H
#define LIVOXCORE_LIDARSDKSERVICE_H

#include "LidarSdkTypes.h"

#include <QString>

namespace LidarSdkService {

struct CallbackSet {
    LivoxLidarInfoChangeCallback infoChange = nullptr;
    LivoxLidarPointCloudCallBack pointCloud = nullptr;
    LivoxLidarImuDataCallback imu = nullptr;
    LivoxLidarInfoCallback statusInfo = nullptr;
    void* clientData = nullptr;
};

bool initialize(const QString& configPath);
void registerCallbacks(const CallbackSet& callbacks);
void clearCallbacks();
void shutdown();

QString versionString();
QString statusString(livox_status status);
QString retCodeString(uint8_t retCode);

} // namespace LidarSdkService

#endif // LIVOXCORE_LIDARSDKSERVICE_H

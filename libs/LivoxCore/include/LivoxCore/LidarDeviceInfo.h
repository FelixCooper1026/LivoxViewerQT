#ifndef LIVOXCORE_LIDARDEVICEINFO_H
#define LIVOXCORE_LIDARDEVICEINFO_H

#include <QString>
#include <cstdint>

struct LidarDeviceInfo {
    uint32_t handle;
    uint8_t dev_type;
    QString sn;
    QString lidar_ip;
    QString product_info;
    bool is_connected;
    bool is_streaming;
    bool parameter_query_ready = false;
};

#endif // LIVOXCORE_LIDARDEVICEINFO_H

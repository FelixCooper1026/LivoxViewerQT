#ifndef LIVOXCORE_LVX2TYPES_H
#define LIVOXCORE_LVX2TYPES_H

#include <cstdint>

#pragma pack(push, 1)
struct Lvx2PublicHeader {
    char signature[16] = "livox_tech";
    uint8_t version_a = 2;
    uint8_t version_b = 0;
    uint8_t version_c = 0;
    uint8_t version_d = 0;
    uint32_t magic_code = 0xAC0EA767;
};

struct Lvx2PrivateHeader {
    uint32_t frame_duration = 50;
    uint8_t device_count = 1;
};

struct Lvx2DeviceInfo {
    char lidar_sn[16] = {};
    char hub_sn[16] = {};
    uint32_t lidar_id = 0;
    uint8_t lidar_type = 247;
    uint8_t device_type = 0;
    uint8_t extrinsic_enable = 1;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Lvx2FrameHeader {
    uint64_t current_offset = 0;
    uint64_t next_offset = 0;
    uint64_t frame_index = 0;
};

struct Lvx2PackageHeader {
    uint8_t version = 0;
    uint32_t lidar_id = 0;
    uint8_t lidar_type = 8;
    uint8_t timestamp_type = 0;
    uint64_t timestamp = 0;
    uint16_t udp_counter = 0;
    uint8_t data_type = 0;
    uint32_t data_length = 0;
    uint8_t frame_counter = 0;
    uint8_t reserve[4] = {0};
};
#pragma pack(pop)

#endif // LIVOXCORE_LVX2TYPES_H

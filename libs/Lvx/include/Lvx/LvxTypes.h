#ifndef LVX_LVXTYPES_H
#define LVX_LVXTYPES_H

#include <cstdint>

#pragma pack(push, 1)
struct LvxPublicHeader {
    char signature[16] = "livox_tech";
    uint8_t version_a = 1;
    uint8_t version_b = 1;
    uint8_t version_c = 0;
    uint8_t version_d = 0;
    uint32_t magic_code = 0xAC0EA767;
};

struct LvxPrivateHeader {
    uint32_t frame_duration = 50;
    uint8_t device_count = 0;
};

struct LvxDeviceInfoV10 {
    char lidar_sn[16] = {};
    char hub_sn[16] = {};
    uint8_t device_index = 0;
    uint8_t device_type = 0;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct LvxDeviceInfoV11 {
    char lidar_sn[16] = {};
    char hub_sn[16] = {};
    uint8_t device_index = 0;
    uint8_t device_type = 0;
    uint8_t extrinsic_enable = 0;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct LvxFrameHeader {
    uint64_t current_offset = 0;
    uint64_t next_offset = 0;
    uint64_t frame_index = 0;
};

struct LvxPackageHeader {
    uint8_t device_index = 0;
    uint8_t version = 5;
    uint8_t slot_id = 0;
    uint8_t lidar_id = 0;
    uint8_t reserved = 0;
    uint32_t status_code = 0;
    uint8_t timestamp_type = 0;
    uint8_t data_type = 0;
    uint8_t timestamp[8] = {};
};

struct LvxLegacyCartesianPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    uint8_t reflectivity = 0;
};

struct LvxCartesianPoint {
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
    uint8_t reflectivity = 0;
};

struct LvxSphericalPoint {
    uint32_t depth = 0;
    uint16_t theta = 0;
    uint16_t phi = 0;
    uint8_t reflectivity = 0;
};

struct LvxCartesianPointWithTag {
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
    uint8_t reflectivity = 0;
    uint8_t tag = 0;
};

struct LvxSphericalPointWithTag {
    uint32_t depth = 0;
    uint16_t theta = 0;
    uint16_t phi = 0;
    uint8_t reflectivity = 0;
    uint8_t tag = 0;
};

struct LvxDoubleCartesianPoint {
    int32_t x1 = 0;
    int32_t y1 = 0;
    int32_t z1 = 0;
    uint8_t reflectivity1 = 0;
    uint8_t tag1 = 0;
    int32_t x2 = 0;
    int32_t y2 = 0;
    int32_t z2 = 0;
    uint8_t reflectivity2 = 0;
    uint8_t tag2 = 0;
};

struct LvxDoubleSphericalPoint {
    uint16_t theta = 0;
    uint16_t phi = 0;
    uint32_t depth1 = 0;
    uint8_t reflectivity1 = 0;
    uint8_t tag1 = 0;
    uint32_t depth2 = 0;
    uint8_t reflectivity2 = 0;
    uint8_t tag2 = 0;
};

struct LvxTripleCartesianPoint {
    int32_t x1 = 0;
    int32_t y1 = 0;
    int32_t z1 = 0;
    uint8_t reflectivity1 = 0;
    uint8_t tag1 = 0;
    int32_t x2 = 0;
    int32_t y2 = 0;
    int32_t z2 = 0;
    uint8_t reflectivity2 = 0;
    uint8_t tag2 = 0;
    int32_t x3 = 0;
    int32_t y3 = 0;
    int32_t z3 = 0;
    uint8_t reflectivity3 = 0;
    uint8_t tag3 = 0;
};

struct LvxTripleSphericalPoint {
    uint16_t theta = 0;
    uint16_t phi = 0;
    uint32_t depth1 = 0;
    uint8_t reflectivity1 = 0;
    uint8_t tag1 = 0;
    uint32_t depth2 = 0;
    uint8_t reflectivity2 = 0;
    uint8_t tag2 = 0;
    uint32_t depth3 = 0;
    uint8_t reflectivity3 = 0;
    uint8_t tag3 = 0;
};

struct LvxImuPoint {
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;
    float acc_x = 0.0f;
    float acc_y = 0.0f;
    float acc_z = 0.0f;
};
#pragma pack(pop)

#endif // LVX_LVXTYPES_H

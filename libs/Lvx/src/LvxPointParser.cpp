#include "Lvx/LvxPointParser.h"

#include "LivoxCore/LidarModelUtils.h"

#include <QVector4D>

#include <cmath>
#include <cstring>

namespace LvxPointParser {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr uint8_t kDataTypeCartesianMid = 0;
constexpr uint8_t kDataTypeSphericalMid = 1;
constexpr uint8_t kDataTypeCartesian = 2;
constexpr uint8_t kDataTypeSpherical = 3;
constexpr uint8_t kDataTypeDoubleCartesian = 4;
constexpr uint8_t kDataTypeDoubleSpherical = 5;
constexpr uint8_t kDataTypeImu = 6;
constexpr uint8_t kDataTypeTripleCartesian = 7;
constexpr uint8_t kDataTypeTripleSpherical = 8;

template <typename T>
T readPoint(const char* data, int index)
{
    T point;
    std::memcpy(&point, data + index * sizeof(T), sizeof(T));
    return point;
}

void configureTransform(Extrinsic& extrinsic,
                        bool enabled,
                        float roll,
                        float pitch,
                        float yaw,
                        float x,
                        float y,
                        float z)
{
    extrinsic.enabled = enabled;
    extrinsic.transform.setToIdentity();
    if (!extrinsic.enabled) {
        return;
    }

    extrinsic.transform.translate(x, y, z);
    extrinsic.transform.rotate(yaw, 0.0f, 0.0f, 1.0f);
    extrinsic.transform.rotate(pitch, 0.0f, 1.0f, 0.0f);
    extrinsic.transform.rotate(roll, 1.0f, 0.0f, 0.0f);
}

void applyExtrinsicTransform(const Extrinsic* extrinsic, PointCloudPoint& point)
{
    if (!extrinsic || !extrinsic->enabled) {
        return;
    }

    const QVector4D transformed = extrinsic->transform * QVector4D(point.x, point.y, point.z, 1.0f);
    point.x = transformed.x();
    point.y = transformed.y();
    point.z = transformed.z();
}

PointCloudPoint sphericalPoint(uint32_t depthMm,
                               uint16_t thetaCentideg,
                               uint16_t phiCentideg,
                               uint8_t reflectivity,
                               uint8_t tag,
                               uint8_t line)
{
    PointCloudPoint point{};
    const float depth = depthMm / 1000.0f;
    point.spherical = true;
    point.theta = thetaCentideg / 100.0f;
    point.phi = phiCentideg / 100.0f;
    point.depth = depth;
    const float theta = point.theta * kPi / 180.0f;
    const float phi = point.phi * kPi / 180.0f;
    point.x = depth * std::sin(theta) * std::cos(phi);
    point.y = depth * std::sin(theta) * std::sin(phi);
    point.z = depth * std::cos(theta);
    point.reflectivity = reflectivity;
    point.tag = tag;
    point.line = line;
    return point;
}

PointCloudPoint cartesianPoint(int32_t xMm,
                               int32_t yMm,
                               int32_t zMm,
                               uint8_t reflectivity,
                               uint8_t tag,
                               uint8_t line)
{
    PointCloudPoint point{};
    point.x = xMm / 1000.0f;
    point.y = yMm / 1000.0f;
    point.z = zMm / 1000.0f;
    point.reflectivity = reflectivity;
    point.tag = tag;
    point.line = line;
    return point;
}

} // namespace

Extrinsic makeExtrinsic(const LvxDeviceInfoV10& info)
{
    Extrinsic extrinsic;
    configureTransform(extrinsic, true, info.roll, info.pitch, info.yaw, info.x, info.y, info.z);
    return extrinsic;
}

Extrinsic makeExtrinsic(const LvxDeviceInfoV11& info)
{
    Extrinsic extrinsic;
    configureTransform(extrinsic,
                       info.extrinsic_enable != 0,
                       info.roll,
                       info.pitch,
                       info.yaw,
                       info.x,
                       info.y,
                       info.z);
    return extrinsic;
}

int payloadSizeForDataType(uint8_t dataType, bool legacyFloatPoints)
{
    if (legacyFloatPoints) {
        return 100 * int(sizeof(LvxLegacyCartesianPoint));
    }

    switch (dataType) {
    case kDataTypeCartesianMid:
        return 100 * int(sizeof(LvxCartesianPoint));
    case kDataTypeSphericalMid:
        return 100 * int(sizeof(LvxSphericalPoint));
    case kDataTypeCartesian:
        return 96 * int(sizeof(LvxCartesianPointWithTag));
    case kDataTypeSpherical:
        return 96 * int(sizeof(LvxSphericalPointWithTag));
    case kDataTypeDoubleCartesian:
        return 48 * int(sizeof(LvxDoubleCartesianPoint));
    case kDataTypeDoubleSpherical:
        return 48 * int(sizeof(LvxDoubleSphericalPoint));
    case kDataTypeImu:
        return int(sizeof(LvxImuPoint));
    case kDataTypeTripleCartesian:
        return 30 * int(sizeof(LvxTripleCartesianPoint));
    case kDataTypeTripleSpherical:
        return 30 * int(sizeof(LvxTripleSphericalPoint));
    default:
        return -1;
    }
}

void appendPackagePoints(const LvxPackageHeader& header,
                         const QByteArray& payload,
                         bool legacyFloatPoints,
                         const Extrinsic* extrinsic,
                         QVector<PointCloudPoint>& points,
                         int lineCount)
{
    if (payload.isEmpty()) {
        return;
    }

    const char* rawData = payload.constData();
    if (legacyFloatPoints) {
        const int pointCount = int(payload.size() / int(sizeof(LvxLegacyCartesianPoint)));
        for (int i = 0; i < pointCount; ++i) {
            const auto raw = readPoint<LvxLegacyCartesianPoint>(rawData, i);
            PointCloudPoint point{};
            point.x = raw.x;
            point.y = raw.y;
            point.z = raw.z;
            point.reflectivity = raw.reflectivity;
            point.line = LivoxCore::lineForPointIndex(i, lineCount);
            points.push_back(point);
        }
        return;
    }

    switch (header.data_type) {
    case kDataTypeCartesianMid: {
        const int pointCount = int(payload.size() / int(sizeof(LvxCartesianPoint)));
        for (int i = 0; i < pointCount; ++i) {
            const auto raw = readPoint<LvxCartesianPoint>(rawData, i);
            PointCloudPoint point{};
            point.x = raw.x / 1000.0f;
            point.y = raw.y / 1000.0f;
            point.z = raw.z / 1000.0f;
            point.reflectivity = raw.reflectivity;
            point.line = LivoxCore::lineForPointIndex(i, lineCount);
            applyExtrinsicTransform(extrinsic, point);
            points.push_back(point);
        }
        break;
    }
    case kDataTypeSphericalMid: {
        const int pointCount = int(payload.size() / int(sizeof(LvxSphericalPoint)));
        for (int i = 0; i < pointCount; ++i) {
            const auto raw = readPoint<LvxSphericalPoint>(rawData, i);
            PointCloudPoint point = sphericalPoint(raw.depth,
                                                   raw.theta,
                                                   raw.phi,
                                                   raw.reflectivity,
                                                   0,
                                                   LivoxCore::lineForPointIndex(i, lineCount));
            applyExtrinsicTransform(extrinsic, point);
            points.push_back(point);
        }
        break;
    }
    case kDataTypeCartesian: {
        const int pointCount = int(payload.size() / int(sizeof(LvxCartesianPointWithTag)));
        for (int i = 0; i < pointCount; ++i) {
            const auto raw = readPoint<LvxCartesianPointWithTag>(rawData, i);
            PointCloudPoint point{};
            point.x = raw.x / 1000.0f;
            point.y = raw.y / 1000.0f;
            point.z = raw.z / 1000.0f;
            point.reflectivity = raw.reflectivity;
            point.tag = raw.tag;
            point.line = LivoxCore::lineForPointIndex(i, lineCount);
            applyExtrinsicTransform(extrinsic, point);
            points.push_back(point);
        }
        break;
    }
    case kDataTypeSpherical: {
        const int pointCount = int(payload.size() / int(sizeof(LvxSphericalPointWithTag)));
        for (int i = 0; i < pointCount; ++i) {
            const auto raw = readPoint<LvxSphericalPointWithTag>(rawData, i);
            PointCloudPoint point = sphericalPoint(raw.depth,
                                                   raw.theta,
                                                   raw.phi,
                                                   raw.reflectivity,
                                                   raw.tag,
                                                   LivoxCore::lineForPointIndex(i, lineCount));
            applyExtrinsicTransform(extrinsic, point);
            points.push_back(point);
        }
        break;
    }
    case kDataTypeDoubleCartesian: {
        const int pointCount = int(payload.size() / int(sizeof(LvxDoubleCartesianPoint)));
        for (int i = 0; i < pointCount; ++i) {
            const auto raw = readPoint<LvxDoubleCartesianPoint>(rawData, i);
            PointCloudPoint p1{};
            p1.x = raw.x1 / 1000.0f;
            p1.y = raw.y1 / 1000.0f;
            p1.z = raw.z1 / 1000.0f;
            p1.reflectivity = raw.reflectivity1;
            p1.tag = raw.tag1;
            p1.line = LivoxCore::lineForPointIndex(i * 2, lineCount);
            applyExtrinsicTransform(extrinsic, p1);
            points.push_back(p1);

            PointCloudPoint p2{};
            p2.x = raw.x2 / 1000.0f;
            p2.y = raw.y2 / 1000.0f;
            p2.z = raw.z2 / 1000.0f;
            p2.reflectivity = raw.reflectivity2;
            p2.tag = raw.tag2;
            p2.line = LivoxCore::lineForPointIndex(i * 2 + 1, lineCount);
            applyExtrinsicTransform(extrinsic, p2);
            points.push_back(p2);
        }
        break;
    }
    case kDataTypeDoubleSpherical: {
        const int pointCount = int(payload.size() / int(sizeof(LvxDoubleSphericalPoint)));
        for (int i = 0; i < pointCount; ++i) {
            const auto raw = readPoint<LvxDoubleSphericalPoint>(rawData, i);
            PointCloudPoint p1 = sphericalPoint(raw.depth1,
                                                raw.theta,
                                                raw.phi,
                                                raw.reflectivity1,
                                                raw.tag1,
                                                LivoxCore::lineForPointIndex(i * 2, lineCount));
            applyExtrinsicTransform(extrinsic, p1);
            points.push_back(p1);

            PointCloudPoint p2 = sphericalPoint(raw.depth2,
                                                raw.theta,
                                                raw.phi,
                                                raw.reflectivity2,
                                                raw.tag2,
                                                LivoxCore::lineForPointIndex(i * 2 + 1, lineCount));
            applyExtrinsicTransform(extrinsic, p2);
            points.push_back(p2);
        }
        break;
    }
    case kDataTypeTripleCartesian: {
        const int pointCount = int(payload.size() / int(sizeof(LvxTripleCartesianPoint)));
        for (int i = 0; i < pointCount; ++i) {
            const auto raw = readPoint<LvxTripleCartesianPoint>(rawData, i);
            PointCloudPoint p1 = cartesianPoint(raw.x1,
                                                raw.y1,
                                                raw.z1,
                                                raw.reflectivity1,
                                                raw.tag1,
                                                LivoxCore::lineForPointIndex(i * 3, lineCount));
            applyExtrinsicTransform(extrinsic, p1);
            points.push_back(p1);

            PointCloudPoint p2 = cartesianPoint(raw.x2,
                                                raw.y2,
                                                raw.z2,
                                                raw.reflectivity2,
                                                raw.tag2,
                                                LivoxCore::lineForPointIndex(i * 3 + 1, lineCount));
            applyExtrinsicTransform(extrinsic, p2);
            points.push_back(p2);

            PointCloudPoint p3 = cartesianPoint(raw.x3,
                                                raw.y3,
                                                raw.z3,
                                                raw.reflectivity3,
                                                raw.tag3,
                                                LivoxCore::lineForPointIndex(i * 3 + 2, lineCount));
            applyExtrinsicTransform(extrinsic, p3);
            points.push_back(p3);
        }
        break;
    }
    case kDataTypeTripleSpherical: {
        const int pointCount = int(payload.size() / int(sizeof(LvxTripleSphericalPoint)));
        for (int i = 0; i < pointCount; ++i) {
            const auto raw = readPoint<LvxTripleSphericalPoint>(rawData, i);
            PointCloudPoint p1 = sphericalPoint(raw.depth1,
                                                raw.theta,
                                                raw.phi,
                                                raw.reflectivity1,
                                                raw.tag1,
                                                LivoxCore::lineForPointIndex(i * 3, lineCount));
            applyExtrinsicTransform(extrinsic, p1);
            points.push_back(p1);

            PointCloudPoint p2 = sphericalPoint(raw.depth2,
                                                raw.theta,
                                                raw.phi,
                                                raw.reflectivity2,
                                                raw.tag2,
                                                LivoxCore::lineForPointIndex(i * 3 + 1, lineCount));
            applyExtrinsicTransform(extrinsic, p2);
            points.push_back(p2);

            PointCloudPoint p3 = sphericalPoint(raw.depth3,
                                                raw.theta,
                                                raw.phi,
                                                raw.reflectivity3,
                                                raw.tag3,
                                                LivoxCore::lineForPointIndex(i * 3 + 2, lineCount));
            applyExtrinsicTransform(extrinsic, p3);
            points.push_back(p3);
        }
        break;
    }
    default:
        break;
    }
}

} // namespace LvxPointParser

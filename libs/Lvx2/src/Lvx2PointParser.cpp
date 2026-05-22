#include "Lvx2/Lvx2PointParser.h"

#include "LivoxCore/LidarSdkTypes.h"

#include <QVector4D>

#include <cmath>
#include <cstring>

namespace Lvx2PointParser {

namespace {

constexpr float kPi = 3.14159265358979323846f;

template <typename T>
static T safeReadPoint(const char* data, int index)
{
    T point;
    std::memcpy(&point, data + index * sizeof(T), sizeof(T));
    return point;
}

} // namespace

Extrinsic makeExtrinsic(const Lvx2DeviceInfo& info)
{
    Extrinsic extrinsic;
    extrinsic.enabled = (info.extrinsic_enable != 0);
    extrinsic.transform.setToIdentity();
    if (!extrinsic.enabled) {
        return extrinsic;
    }

    extrinsic.transform.translate(info.x / 100.0f, info.y / 100.0f, info.z / 100.0f);
    extrinsic.transform.rotate(info.yaw, 0.0f, 0.0f, 1.0f);
    extrinsic.transform.rotate(info.pitch, 0.0f, 1.0f, 0.0f);
    extrinsic.transform.rotate(info.roll, 1.0f, 0.0f, 0.0f);
    return extrinsic;
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

void appendPackagePoints(const Lvx2PackageHeader& header,
                         const QByteArray& payload,
                         const Extrinsic* extrinsic,
                         QVector<PointCloudPoint>& points)
{
    if (payload.isEmpty()) {
        return;
    }

    const char* rawData = payload.constData();

    switch (header.data_type) {
    case kLivoxLidarCartesianCoordinateHighData: {
        const int pointCount = int(payload.size() / int(sizeof(LivoxLidarCartesianHighRawPoint)));
        for (int i = 0; i < pointCount; ++i) {
            auto raw = safeReadPoint<LivoxLidarCartesianHighRawPoint>(rawData, i);
            PointCloudPoint point{};
            point.x = raw.x / 1000.0f;
            point.y = raw.y / 1000.0f;
            point.z = raw.z / 1000.0f;
            point.reflectivity = raw.reflectivity;
            point.tag = raw.tag;
            applyExtrinsicTransform(extrinsic, point);
            points.push_back(point);
        }
        break;
    }
    case kLivoxLidarCartesianCoordinateLowData: {
        const int pointCount = int(payload.size() / int(sizeof(LivoxLidarCartesianLowRawPoint)));
        for (int i = 0; i < pointCount; ++i) {
            auto raw = safeReadPoint<LivoxLidarCartesianLowRawPoint>(rawData, i);
            PointCloudPoint point{};
            point.x = raw.x / 100.0f;
            point.y = raw.y / 100.0f;
            point.z = raw.z / 100.0f;
            point.reflectivity = raw.reflectivity;
            point.tag = raw.tag;
            applyExtrinsicTransform(extrinsic, point);
            points.push_back(point);
        }
        break;
    }
    case kLivoxLidarSphericalCoordinateData: {
        const int pointCount = int(payload.size() / int(sizeof(LivoxLidarSpherPoint)));
        for (int i = 0; i < pointCount; ++i) {
            auto raw = safeReadPoint<LivoxLidarSpherPoint>(rawData, i);
            PointCloudPoint point{};
            const float depth = raw.depth / 1000.0f;
            const float theta = raw.theta / 100.0f * kPi / 180.0f;
            const float phi = raw.phi / 100.0f * kPi / 180.0f;
            point.x = depth * std::sin(theta) * std::cos(phi);
            point.y = depth * std::sin(theta) * std::sin(phi);
            point.z = depth * std::cos(theta);
            point.reflectivity = raw.reflectivity;
            point.tag = raw.tag;
            applyExtrinsicTransform(extrinsic, point);
            points.push_back(point);
        }
        break;
    }
    case kLivoxLidarDoubleEchoData: {
        const int pointCount = int(payload.size() / int(sizeof(LivoxLidarDoubleEchoRawPoint)));
        for (int i = 0; i < pointCount; ++i) {
            auto raw = safeReadPoint<LivoxLidarDoubleEchoRawPoint>(rawData, i);
            PointCloudPoint p1{};
            p1.x = raw.x1 / 1000.0f;
            p1.y = raw.y1 / 1000.0f;
            p1.z = raw.z1 / 1000.0f;
            p1.reflectivity = raw.reflectivity1;
            p1.tag = raw.tag1;
            applyExtrinsicTransform(extrinsic, p1);
            points.push_back(p1);

            PointCloudPoint p2{};
            p2.x = raw.x2 / 1000.0f;
            p2.y = raw.y2 / 1000.0f;
            p2.z = raw.z2 / 1000.0f;
            p2.reflectivity = raw.reflectivity2;
            p2.tag = raw.tag2;
            applyExtrinsicTransform(extrinsic, p2);
            points.push_back(p2);
        }
        break;
    }
    default:
        break;
    }
}

} // namespace Lvx2PointParser

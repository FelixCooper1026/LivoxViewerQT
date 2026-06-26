#include "Slam/Backends/FastLio/FastLioSlamBackend.h"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <omp.h>

#include <memory>

#include "IMU_Processing.hpp"
#include "use-ikfom.hpp"

namespace {

constexpr double kNsToSeconds = 1.0e-9;
constexpr double kNsToMilliseconds = 1.0e-6;

double fastLioProcessNoiseTrace()
{
    const auto covariance = process_noise_cov();
    return covariance.trace();
}

MeasureGroup toFastLioMeasureGroup(const SlamInputFrame& frame)
{
    MeasureGroup measure;
    measure.lidar_beg_time = static_cast<double>(frame.frameStartNs) * kNsToSeconds;
    measure.lidar_end_time = static_cast<double>(frame.frameEndNs) * kNsToSeconds;
    measure.lidar->points.reserve(static_cast<std::size_t>(frame.points.size()));

    for (const SlamPoint& sourcePoint : frame.points) {
        PointType targetPoint;
        targetPoint.x = sourcePoint.x;
        targetPoint.y = sourcePoint.y;
        targetPoint.z = sourcePoint.z;
        targetPoint.intensity = static_cast<float>(sourcePoint.reflectivity);
        targetPoint.curvature = static_cast<float>(static_cast<double>(sourcePoint.offsetNs) * kNsToMilliseconds);
        measure.lidar->points.push_back(targetPoint);
    }

    for (const SlamImuSample& sourceImu : frame.imuSamples) {
        auto targetImu = std::make_shared<sensor_msgs::Imu>();
        targetImu->header.stamp = ros::Time().fromSec(static_cast<double>(sourceImu.timestampNs) * kNsToSeconds);
        targetImu->angular_velocity.x = sourceImu.gyroRadPerSec[0];
        targetImu->angular_velocity.y = sourceImu.gyroRadPerSec[1];
        targetImu->angular_velocity.z = sourceImu.gyroRadPerSec[2];
        targetImu->linear_acceleration.x = sourceImu.accelMps2[0];
        targetImu->linear_acceleration.y = sourceImu.accelMps2[1];
        targetImu->linear_acceleration.z = sourceImu.accelMps2[2];
        measure.imu.push_back(targetImu);
    }

    return measure;
}

QString fastLioAdapterPendingMessage()
{
    return QStringLiteral("FAST_LIO source snapshot is present under libs/Slam/third_party/fast_lio; backend adapter is not complete yet.");
}

void assignError(QString* error, const QString& message)
{
    if (error != nullptr) {
        *error = message;
    }
}

void assignFailedOutput(SlamOutput* output, const QString& message)
{
    if (output != nullptr) {
        output->status = SlamStatusCode::Failed;
        output->message = message;
        output->imuHealthy = false;
    }
}

} // namespace

bool FastLioSlamBackend::start(const SlamRuntimeConfig& config, QString* error)
{
    (void)config;
    (void)fastLioProcessNoiseTrace();
    message_ = fastLioAdapterPendingMessage();
    status_ = SlamStatusCode::Failed;
    assignError(error, message_);
    return false;
}

void FastLioSlamBackend::stop()
{
    status_ = SlamStatusCode::Stopped;
}

bool FastLioSlamBackend::reset(QString* error)
{
    message_.clear();
    status_ = SlamStatusCode::Idle;
    assignError(error, QString());
    return true;
}

bool FastLioSlamBackend::processFrame(const SlamInputFrame& frame, SlamOutput* output, QString* error)
{
    const MeasureGroup measure = toFastLioMeasureGroup(frame);
    (void)measure;
    if (message_.isEmpty()) {
        message_ = fastLioAdapterPendingMessage();
    }
    status_ = SlamStatusCode::Failed;
    assignFailedOutput(output, message_);
    assignError(error, message_);
    return false;
}

SlamStatusCode FastLioSlamBackend::status() const
{
    return status_;
}

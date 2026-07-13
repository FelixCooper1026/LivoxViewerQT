#ifndef SLAM_BACKENDS_FASTLIO_FASTLIOSLAMBACKEND_H
#define SLAM_BACKENDS_FASTLIO_FASTLIOSLAMBACKEND_H

#include "Core/ISlamBackend.h"

#include <memory>

struct FastLioPredictionState {
    bool valid = false;
    bool lidarOnly = false;
    int64_t timestampNs = 0;
    double position[3] = {};
    double orientation[4] = {0.0, 0.0, 0.0, 1.0};
    double velocity[3] = {};
    double angularVelocity[3] = {};
    double gyroBias[3] = {};
    double accelBias[3] = {};
    double gravity[3] = {};
    double accelerationScale = 1.0;
};

class FastLioSlamBackend final : public ISlamBackend {
public:
    FastLioSlamBackend();
    ~FastLioSlamBackend() override;

    bool start(const SlamRuntimeConfig& config, QString* error) override;
    void stop() override;
    bool reset(QString* error) override;
    bool processFrame(const SlamInputFrame& frame, SlamOutput* output, QString* error) override;
    SlamStatusCode status() const override;
    FastLioPredictionState predictionState(int64_t timestampNs) const;

private:
    struct FastLioState;

    SlamRuntimeConfig config_;
    std::unique_ptr<FastLioState> state_;
    SlamStatusCode status_ = SlamStatusCode::Idle;
    QString message_;
};

#endif // SLAM_BACKENDS_FASTLIO_FASTLIOSLAMBACKEND_H

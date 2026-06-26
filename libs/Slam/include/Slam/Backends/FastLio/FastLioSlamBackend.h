#ifndef SLAM_BACKENDS_FASTLIO_FASTLIOSLAMBACKEND_H
#define SLAM_BACKENDS_FASTLIO_FASTLIOSLAMBACKEND_H

#include "Slam/Core/ISlamBackend.h"

class FastLioSlamBackend final : public ISlamBackend {
public:
    bool start(const SlamRuntimeConfig& config, QString* error) override;
    void stop() override;
    bool reset(QString* error) override;
    bool processFrame(const SlamInputFrame& frame, SlamOutput* output, QString* error) override;
    SlamStatusCode status() const override;

private:
    SlamStatusCode status_ = SlamStatusCode::Idle;
    QString message_;
};

#endif // SLAM_BACKENDS_FASTLIO_FASTLIOSLAMBACKEND_H

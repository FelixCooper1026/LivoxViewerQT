#ifndef SLAM_CORE_ISLAMBACKEND_H
#define SLAM_CORE_ISLAMBACKEND_H

#include "Core/SlamRuntimeConfig.h"
#include "Core/SlamTypes.h"

#include <QString>

class ISlamBackend {
public:
    virtual ~ISlamBackend() = default;

    virtual bool start(const SlamRuntimeConfig& config, QString* error) = 0;
    virtual void stop() = 0;
    virtual bool reset(QString* error) = 0;
    virtual bool processFrame(const SlamInputFrame& frame, SlamOutput* output, QString* error) = 0;
    virtual SlamStatusCode status() const = 0;
};

#endif // SLAM_CORE_ISLAMBACKEND_H

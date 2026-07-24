#include "DynamicFilterController.h"

#include "FreeDomDynamicPointFilter.h"
#include "MDetectorDynamicPointFilter.h"

bool DynamicFilterController::configure(const DynamicFilterRuntimeConfig& config,
                                        std::string* error)
{
    config_ = config;
    backend_ = config.backend;
    switch(backend_)
    {
    case DynamicFilterBackend::MDetector:
        filter_ = std::make_unique<MDetectorDynamicPointFilter>();
        break;
    case DynamicFilterBackend::FreeDOM:
        filter_ = std::make_unique<FreeDomDynamicPointFilter>();
        break;
    case DynamicFilterBackend::Disabled:
        filter_.reset();
        if(error)
            error->clear();
        return true;
    }
    return filter_->configure(config_, error);
}

void DynamicFilterController::reset()
{
    if(filter_)
        filter_->reset();
}

bool DynamicFilterController::processFrame(const DynamicFilterFrame& frame,
                                           DynamicFilterResult* result,
                                           std::string* error)
{
    if(result == nullptr || frame.lidarFrameCloud == nullptr)
    {
        if(error)
            *error = "Dynamic filter received a null frame cloud or result.";
        return false;
    }
    if(filter_)
        return filter_->processFrame(frame, result, error);

    *result = DynamicFilterResult{};
    result->timestampNs = frame.timestampNs;
    result->labels.resize(frame.lidarFrameCloud->points.size());
    result->stats.backend = DynamicFilterBackend::Disabled;
    result->stats.inputPointCount =
        static_cast<int>(frame.lidarFrameCloud->points.size());
    result->stats.staticPointCount = result->stats.inputPointCount;
    if(error)
        error->clear();
    return true;
}

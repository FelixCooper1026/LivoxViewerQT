#ifndef DYNAMICFILTER_MDETECTORDYNAMICPOINTFILTER_H
#define DYNAMICFILTER_MDETECTORDYNAMICPOINTFILTER_H

#include "IDynamicPointFilter.h"

#include <memory>

class DynamicObjectDetector;

class MDetectorDynamicPointFilter final : public IDynamicPointFilter {
public:
    bool configure(const DynamicFilterRuntimeConfig& config,
                   std::string* error) override;
    void reset() override;
    bool processFrame(const DynamicFilterFrame& frame,
                      DynamicFilterResult* result,
                      std::string* error) override;
    DynamicFilterBackend backend() const noexcept override
    {
        return DynamicFilterBackend::MDetector;
    }

private:
    DynamicObjectDetectorConfig config_;
    std::unique_ptr<DynamicObjectDetector> detector_;
};

#endif // DYNAMICFILTER_MDETECTORDYNAMICPOINTFILTER_H

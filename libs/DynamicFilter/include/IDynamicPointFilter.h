#ifndef DYNAMICFILTER_IDYNAMICPOINTFILTER_H
#define DYNAMICFILTER_IDYNAMICPOINTFILTER_H

#include "DynamicFilterTypes.h"

#include <string>

class IDynamicPointFilter {
public:
    virtual ~IDynamicPointFilter() = default;
    virtual bool configure(const DynamicFilterRuntimeConfig& config,
                           std::string* error) = 0;
    virtual void reset() = 0;
    virtual bool processFrame(const DynamicFilterFrame& frame,
                              DynamicFilterResult* result,
                              std::string* error) = 0;
    virtual DynamicFilterBackend backend() const noexcept = 0;
};

#endif // DYNAMICFILTER_IDYNAMICPOINTFILTER_H

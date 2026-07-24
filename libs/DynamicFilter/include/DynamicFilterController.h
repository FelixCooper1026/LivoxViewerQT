#ifndef DYNAMICFILTER_DYNAMICFILTERCONTROLLER_H
#define DYNAMICFILTER_DYNAMICFILTERCONTROLLER_H

#include "IDynamicPointFilter.h"

#include <memory>

class DynamicFilterController {
public:
    bool configure(const DynamicFilterRuntimeConfig& config,
                   std::string* error);
    void reset();
    bool processFrame(const DynamicFilterFrame& frame,
                      DynamicFilterResult* result,
                      std::string* error);
    DynamicFilterBackend backend() const noexcept { return backend_; }

private:
    DynamicFilterRuntimeConfig config_;
    DynamicFilterBackend backend_ = DynamicFilterBackend::Disabled;
    std::unique_ptr<IDynamicPointFilter> filter_;
};

#endif // DYNAMICFILTER_DYNAMICFILTERCONTROLLER_H

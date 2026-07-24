#ifndef DYNAMICFILTER_FREEDOMDYNAMICPOINTFILTER_H
#define DYNAMICFILTER_FREEDOMDYNAMICPOINTFILTER_H

#include "FreeDomSnapshotBuilder.h"
#include "IDynamicPointFilter.h"

#include <memory>

namespace freedom {
class FreeDOM;
}

class FreeDomDynamicPointFilter final : public IDynamicPointFilter {
public:
    FreeDomDynamicPointFilter();
    ~FreeDomDynamicPointFilter() override;

    bool configure(const DynamicFilterRuntimeConfig& config,
                   std::string* error) override;
    void reset() override;
    bool processFrame(const DynamicFilterFrame& frame,
                      DynamicFilterResult* result,
                      std::string* error) override;
    DynamicFilterBackend backend() const noexcept override
    {
        return DynamicFilterBackend::FreeDOM;
    }

private:
    void createEngine();

    FreeDomRuntimeConfig config_;
    bool configured_ = false;
    bool debugEnabled_ = false;
    unsigned int debugSnapshotIntervalFrames_ = 5;
    unsigned int mapSnapshotIntervalFrames_ = 10;
    std::uint64_t frameVersion_ = 0;
    std::unique_ptr<freedom::FreeDOM> engine_;
    std::shared_ptr<FreeDomDebugSnapshot> buildingDebugSnapshot_;
    std::shared_ptr<const FreeDomDebugSnapshot> latestDebugSnapshot_;
    std::shared_ptr<const FreeDomMapSnapshot> latestMapSnapshot_;
};

#endif // DYNAMICFILTER_FREEDOMDYNAMICPOINTFILTER_H

#ifndef DYNAMICFILTER_DYNAMICFILTERBACKEND_H
#define DYNAMICFILTER_DYNAMICFILTERBACKEND_H

#include <cstdint>

enum class DynamicFilterBackend : std::uint8_t {
    Disabled = 0,
    MDetector = 1,
    FreeDOM = 2
};

#endif // DYNAMICFILTER_DYNAMICFILTERBACKEND_H

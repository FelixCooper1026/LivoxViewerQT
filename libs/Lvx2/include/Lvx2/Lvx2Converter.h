#ifndef LVX2_LVX2CONVERTER_H
#define LVX2_LVX2CONVERTER_H

#include "Lvx2/Lvx2Types.h"

namespace Lvx2Convert {

enum class Mode {
    MergeAllToOne = 0,
    SplitBy100ms = 1
};

enum class Format {
    PCD = 0,
    LAS = 1,
    CSV = 2,
    TXT = 3
};

} // namespace Lvx2Convert

#endif // LVX2_LVX2CONVERTER_H

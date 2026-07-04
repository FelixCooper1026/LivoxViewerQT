#ifndef LVX_LVXPOINTPARSER_H
#define LVX_LVXPOINTPARSER_H

#include "Lvx/LvxTypes.h"
#include "Playback/PlaybackSource.h"

#include <QByteArray>
#include <QVector>

namespace LvxPointParser {

using Extrinsic = Playback::Extrinsic;

Extrinsic makeExtrinsic(const LvxDeviceInfoV10& info);
Extrinsic makeExtrinsic(const LvxDeviceInfoV11& info);

int payloadSizeForDataType(uint8_t dataType, bool legacyFloatPoints);

void appendPackagePoints(const LvxPackageHeader& header,
                         const QByteArray& payload,
                         bool legacyFloatPoints,
                         const Extrinsic* extrinsic,
                         QVector<PointCloudPoint>& points,
                         int lineCount);

} // namespace LvxPointParser

#endif // LVX_LVXPOINTPARSER_H

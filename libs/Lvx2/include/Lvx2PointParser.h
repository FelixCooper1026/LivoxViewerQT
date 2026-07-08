#ifndef LVX2_LVX2POINTPARSER_H
#define LVX2_LVX2POINTPARSER_H

#include "Lvx2PlaybackController.h"
#include "Lvx2Types.h"
#include "PointCloudTypes.h"

#include <QByteArray>
#include <QVector>

namespace Lvx2PointParser {

using Extrinsic = Lvx2Playback::Extrinsic;

Extrinsic makeExtrinsic(const Lvx2DeviceInfo& info);

void applyExtrinsicTransform(const Extrinsic* extrinsic, PointCloudPoint& point);

void appendPackagePoints(const Lvx2PackageHeader& header,
                         const QByteArray& payload,
                         const Extrinsic* extrinsic,
                         QVector<PointCloudPoint>& points,
                         int lineCount = 1);

} // namespace Lvx2PointParser

#endif // LVX2_LVX2POINTPARSER_H

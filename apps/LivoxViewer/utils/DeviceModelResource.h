#ifndef LIVOXVIEWER_UTILS_DEVICEMODELRESOURCE_H
#define LIVOXVIEWER_UTILS_DEVICEMODELRESOURCE_H

#include <QString>

namespace DeviceModelResource {

QString modelKeyForName(QString modelName);
QString modelPathForKey(const QString& modelKey);
QString modelPathForName(const QString& modelName);
bool sourceXReversedForKey(const QString& modelKey);
float sourceUnitToMetersForKey(const QString& modelKey);

} // namespace DeviceModelResource

#endif // LIVOXVIEWER_UTILS_DEVICEMODELRESOURCE_H

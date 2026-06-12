#include "utils/DeviceModelResource.h"

#include <QDir>

namespace DeviceModelResource {

QString modelKeyForName(QString modelName)
{
    modelName = modelName.trimmed();
    const QString lower = modelName.toLower();
    if (lower.contains(QStringLiteral("mid360s"))) {
        return QStringLiteral("Mid360S");
    }
    if (lower.contains(QStringLiteral("mid360l"))) {
        return QStringLiteral("Mid360l");
    }
    if (lower.contains(QStringLiteral("mid360"))) {
        return QStringLiteral("Mid360");
    }
    if (lower.contains(QStringLiteral("avia2"))) {
        return QStringLiteral("Avia2");
    }
    if (lower.contains(QStringLiteral("hap"))) {
        return QStringLiteral("HAP");
    }
    if (lower == QStringLiteral("pa") || lower.contains(QStringLiteral("livox pa"))) {
        return QStringLiteral("PA");
    }
    return {};
}

QString modelPathForKey(const QString& modelKey)
{
    return QDir(QStringLiteral(LIVOX_VIEWER_SOURCE_DIR))
        .filePath(QStringLiteral("plugins/StlModel/models/%1.glb").arg(modelKey));
}

QString modelPathForName(const QString& modelName)
{
    const QString modelKey = modelKeyForName(modelName);
    return modelKey.isEmpty() ? QString() : modelPathForKey(modelKey);
}

bool sourceXReversedForKey(const QString& modelKey)
{
    return modelKey == QStringLiteral("Avia2");
}

} // namespace DeviceModelResource

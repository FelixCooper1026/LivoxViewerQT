#include "utils/DeviceModelResource.h"

#include <QDir>

namespace DeviceModelResource {

QString modelKeyForName(QString modelName)
{
    const QString lower = modelName.trimmed().toLower();
    QString normalized;
    normalized.reserve(lower.size());
    for (const QChar character : lower) {
        if (character.isLetterOrNumber()) {
            normalized.append(character);
        }
    }
    if (normalized.contains(QStringLiteral("mid360s"))) {
        return QStringLiteral("Mid360S");
    }
    if (normalized.contains(QStringLiteral("mid360l")) || normalized == QStringLiteral("360l")) {
        return QStringLiteral("Mid360L");
    }
    if (normalized.contains(QStringLiteral("mid360"))) {
        return QStringLiteral("Mid360");
    }
    if (normalized.contains(QStringLiteral("avia2"))) {
        return QStringLiteral("Avia2");
    }
    if (normalized.contains(QStringLiteral("hap"))) {
        return QStringLiteral("HAP");
    }
    if (normalized == QStringLiteral("pa") || normalized.contains(QStringLiteral("livoxpa"))) {
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

float sourceUnitToMetersForKey(const QString& modelKey)
{
    return modelKey == QStringLiteral("Mid360L") ? 1.0f : 0.001f;
}

float imuModelYawAlignmentDegreesForKey(const QString& modelKey)
{
    return modelKey == QStringLiteral("Mid360L") ? 0.0f : 180.0f;
}

float pointCloudModelYawAlignmentDegreesForKey(const QString& modelKey)
{
    return modelKey == QStringLiteral("Mid360L") ? 180.0f : 0.0f;
}

} // namespace DeviceModelResource

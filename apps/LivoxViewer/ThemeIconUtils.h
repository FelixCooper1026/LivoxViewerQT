#ifndef LIVOXVIEWER_THEMEICONUTILS_H
#define LIVOXVIEWER_THEMEICONUTILS_H

#include <QAbstractButton>
#include <QAction>
#include <QApplication>
#include <QGuiApplication>
#include <QHash>
#include <QIcon>
#include <QLabel>
#include <QList>
#include <QPainter>
#include <QPalette>
#include <QPixmap>
#include <QScreen>
#include <QSize>
#include <QSvgRenderer>
#include <QVariant>
#include <QtGlobal>

namespace ThemeIconUtils {

inline constexpr const char* kSvgIconPathProperty = "themedSvgIconPath";
inline constexpr const char* kSvgIconSizeProperty = "themedSvgIconSize";

inline QColor iconColor()
{
    return QApplication::palette().color(QPalette::ButtonText);
}

inline qreal devicePixelRatio()
{
    if (QScreen* screen = QGuiApplication::primaryScreen()) {
        return screen->devicePixelRatio();
    }
    return qApp ? qApp->devicePixelRatio() : 1.0;
}

inline QPixmap themedSvgPixmap(const QString& iconPath, const QSize& size)
{
    QSvgRenderer renderer(iconPath);
    const qreal dpr = devicePixelRatio();
    const QSize physicalSize(qMax(1, qRound(size.width() * dpr)),
                             qMax(1, qRound(size.height() * dpr)));
    QPixmap pixmap(physicalSize);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    renderer.render(&painter, QRectF(QPointF(0, 0), QSizeF(physicalSize)));
    painter.setCompositionMode(QPainter::CompositionMode_SourceIn);
    painter.fillRect(QRectF(QPointF(0, 0), QSizeF(physicalSize)), iconColor());
    painter.end();
    pixmap.setDevicePixelRatio(dpr);
    return pixmap;
}

inline QIcon themedSvgIcon(const QString& iconPath)
{
    static QHash<QString, QIcon> iconCache;
    const QString cacheKey = QStringLiteral("%1|%2|%3")
                                 .arg(iconPath,
                                      iconColor().name(QColor::HexArgb),
                                      QString::number(devicePixelRatio()));
    const auto cachedIcon = iconCache.constFind(cacheKey);
    if (cachedIcon != iconCache.cend()) {
        return cachedIcon.value();
    }

    QIcon icon;
    for (int size : {16, 20, 22, 24, 32, 48, 64}) {
        icon.addPixmap(themedSvgPixmap(iconPath, QSize(size, size)));
    }
    iconCache.insert(cacheKey, icon);
    return icon;
}

inline void setThemedSvgIcon(QAction* action, const QString& iconPath)
{
    action->setProperty(kSvgIconPathProperty, iconPath);
    action->setIcon(themedSvgIcon(iconPath));
}

inline void setThemedSvgIcon(QAbstractButton* button, const QString& iconPath)
{
    button->setProperty(kSvgIconPathProperty, iconPath);
    button->setIcon(themedSvgIcon(iconPath));
}

inline void setThemedSvgPixmap(QLabel* label, const QString& iconPath, const QSize& size)
{
    label->setProperty(kSvgIconPathProperty, iconPath);
    label->setProperty(kSvgIconSizeProperty, size);
    label->setPixmap(themedSvgPixmap(iconPath, size));
}

inline void refreshObject(QObject* object)
{
    const QString iconPath = object->property(kSvgIconPathProperty).toString();
    if (!iconPath.isEmpty()) {
        if (QAction* action = qobject_cast<QAction*>(object)) {
            action->setIcon(themedSvgIcon(iconPath));
        } else if (QAbstractButton* button = qobject_cast<QAbstractButton*>(object)) {
            button->setIcon(themedSvgIcon(iconPath));
        } else if (QLabel* label = qobject_cast<QLabel*>(object)) {
            const QSize size = label->property(kSvgIconSizeProperty).toSize();
            label->setPixmap(themedSvgPixmap(iconPath, size));
        }
    }

    const QObjectList children = object->children();
    for (QObject* child : children) {
        refreshObject(child);
    }
}

} // namespace ThemeIconUtils

#endif // LIVOXVIEWER_THEMEICONUTILS_H

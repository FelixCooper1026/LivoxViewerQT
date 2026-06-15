#ifndef LIVOXVIEWER_DIALOGS_DIALOGWINDOWUTILS_H
#define LIVOXVIEWER_DIALOGS_DIALOGWINDOWUTILS_H

#include <QDialog>

namespace DialogWindowUtils {

inline void enableTopLevelWindowControls(QDialog* dialog)
{
    dialog->setWindowFlags((dialog->windowFlags() & ~Qt::WindowType_Mask)
        | Qt::Window
        | Qt::WindowTitleHint
        | Qt::WindowSystemMenuHint
        | Qt::WindowMinimizeButtonHint
        | Qt::WindowMaximizeButtonHint
        | Qt::WindowCloseButtonHint);
}

} // namespace DialogWindowUtils

#endif // LIVOXVIEWER_DIALOGS_DIALOGWINDOWUTILS_H

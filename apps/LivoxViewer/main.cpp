#include <QApplication>
#include <QIcon>
#include <QSurfaceFormat>
#include "AppVersion.h"
#include "LivoxViewerWindow.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

int main(int argc, char *argv[])
{
#ifdef _WIN32
    SetDllDirectoryA("C:\\Windows\\System32\\Npcap");
#endif

    QSurfaceFormat glFormat = QSurfaceFormat::defaultFormat();
    glFormat.setDepthBufferSize(24);
    glFormat.setStencilBufferSize(8);
    glFormat.setSamples(4);
    QSurfaceFormat::setDefaultFormat(glFormat);

    QApplication app(argc, argv);

    app.setApplicationName("LivoxViewerQT");
    app.setApplicationDisplayName("LivoxViewerQT");
    app.setApplicationVersion(QStringLiteral(LIVOX_VIEWER_VERSION));
    app.setOrganizationName("FelixCooper1026");

#ifndef _WIN32
    app.setDesktopFileName("LivoxViewerQT");
#endif

    const QIcon appIcon(":/icons/app_icon.png");
    app.setWindowIcon(appIcon);

    LivoxViewerWindow w;
    w.setWindowIcon(appIcon);
    w.show();

    return app.exec();
}

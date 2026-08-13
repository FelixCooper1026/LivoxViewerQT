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
#include <string>

namespace {

void addNpcapDllDirectory()
{
    wchar_t systemDirectory[MAX_PATH] = {};
    const UINT length = GetSystemDirectoryW(systemDirectory, MAX_PATH);
    if (length > 0 && length < MAX_PATH) {
        std::wstring npcapDirectory(systemDirectory, length);
        npcapDirectory += L"\\Npcap";
        SetDllDirectoryW(npcapDirectory.c_str());
    }
}

} // namespace
#endif

int main(int argc, char *argv[])
{
#ifdef _WIN32
    addNpcapDllDirectory();
#endif

    QApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

    QSurfaceFormat glFormat = QSurfaceFormat::defaultFormat();
    glFormat.setVersion(3, 3);
    glFormat.setProfile(QSurfaceFormat::CoreProfile);
    glFormat.setDepthBufferSize(24);
    glFormat.setStencilBufferSize(8);
    glFormat.setSamples(0);
    glFormat.setSwapInterval(0);
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

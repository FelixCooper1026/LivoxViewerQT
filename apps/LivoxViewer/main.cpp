#include <QApplication>
#include <QIcon>
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

    QApplication app(argc, argv);

    // 设置应用程序信息
    app.setApplicationName("LivoxViewerQT");
    app.setApplicationVersion("1.3.0");
    app.setOrganizationName("FelixCooper1026");
    app.setWindowIcon(QIcon(":/resources/app_icon.ico"));

    LivoxViewerWindow w;
    w.show();

    return app.exec();
}

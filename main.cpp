#include <QApplication>
#include <QIcon>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);



    // 设置应用程序信息
    app.setApplicationName("LivoxViewerQT");
    app.setApplicationVersion("1.0");
    app.setOrganizationName("融汇定位（北京）");
    app.setWindowIcon(QIcon(":/resources/app_icon.ico"));

    MainWindow w;
    w.show();

    return app.exec();
}

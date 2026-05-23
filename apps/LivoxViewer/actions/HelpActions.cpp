#include "LivoxViewerWindow.h"
#include <QDesktopServices>
#include <QUrl>
void LivoxViewerWindow::createHelpActions()
{
    // 1. Livox 官网
    QAction* actionLivoxWebsite = helpMenu->addAction("Livox 官网");
    // 2. Livox Wiki
    QAction* actionLivoxWiki = helpMenu->addAction("Livox Wiki");
    // 3. Mid-360 HMS 故障诊断码说明
    QAction* actionHmsCode = helpMenu->addAction("HMS 故障诊断码说明");
    // 4. 时间同步说明
    QAction* actionTimeSync = helpMenu->addAction("时间同步说明");
    // 5. 产品知识库(暂未实现)
    QAction* actionKnowledgeBase = helpMenu->addAction("产品知识库");
    // 6. 下载中心
    QAction* actionDownloadCenter = helpMenu->addAction("下载中心");

    // 关于
    aboutAction = helpMenu->addAction("关于");

    // 退出
    // 关于
    connect(aboutAction, &QAction::triggered, [this]() {
        QMessageBox msgBox(this);
        msgBox.setWindowTitle("关于 LivoxViewerQT");
        msgBox.setTextFormat(Qt::RichText);  // 支持富文本
        msgBox.setText(
            "<h3>LivoxViewerQT - Livox 激光雷达可视化配置软件</h3>"
            "<p><b>版本:</b> 1.3.0</p>"
            "<p><b>编译日期:</b> " __DATE__ " </p>"
            "<p><b>作者:</b> FelixCooper1026</p>"
            "<p><b>功能特性:</b></p>"
            "<ul>"
            "<li>Livox 激光雷达设备连接与管理</li>"
            "<li>实时点云数据可视化</li>"
            "<li>设备参数配置与状态监控</li>"
            "<li>点云数据采集与保存</li>"
            "<li>IMU 数据显示与记录</li>"
            "<li>设备LOG数据采集与保存</li>"
            "<li>设备固件升级</li>"
            "</ul>"
            "<p><b>项目地址:</b> <a href=\"https://github.com/FelixCooper1026/LivoxViewerQT\">https://github.com/FelixCooper1026/LivoxViewerQT</a></p>"
            "<p>基于 Qt " QT_VERSION_STR " 和 Livox SDK2 开发</p>"
        );
        msgBox.exec();
    });

    // Livox 官网
    connect(actionLivoxWebsite, &QAction::triggered, []() {
        QDesktopServices::openUrl(QUrl("https://www.livoxtech.com/cn"));
    });

    // Livox Wiki
    connect(actionLivoxWiki, &QAction::triggered, []() {
        QDesktopServices::openUrl(QUrl("https://livox-wiki-cn.readthedocs.io/zh-cn/latest/tutorials/index.html"));
    });

    // Mid-360 HMS 故障诊断码说明
    connect(actionHmsCode, &QAction::triggered, []() {
        QDesktopServices::openUrl(QUrl("https://livox-wiki-cn.readthedocs.io/zh-cn/latest/tutorials/new_product/mid360/hms_code_mid360.html"));
    });

    // 时间同步说明
    connect(actionTimeSync, &QAction::triggered, []() {
        QDesktopServices::openUrl(QUrl("https://livox-wiki-cn.readthedocs.io/zh-cn/latest/tutorials/new_product/common/time_sync.html#id1"));
    });

    // 产品知识库（弹出对话框）
    connect(actionKnowledgeBase, &QAction::triggered, [this]() {
        QDialog dlg(this);
        dlg.setWindowTitle("产品知识库（暂未实现）");
        dlg.resize(600, 400);

        QVBoxLayout* layout = new QVBoxLayout(&dlg);
        QLabel* lbl = new QLabel("请选择需要查看的帮助文档：", &dlg);
        layout->addWidget(lbl);

        QListWidget* list = new QListWidget(&dlg);
        list->addItem("用户手册.pdf");
        list->addItem("快速入门.pdf");
        list->addItem("常见问题.pdf");
        layout->addWidget(list, 1);

        QPushButton* btnOpen = new QPushButton("打开文档", &dlg);
        layout->addWidget(btnOpen);

        connect(btnOpen, &QPushButton::clicked, [&]() {
            if (list->currentItem()) {
                QString fileName = list->currentItem()->text();
                QString filePath = QCoreApplication::applicationDirPath() + "/help/" + fileName;
                if (QFile::exists(filePath)) {
                    QDesktopServices::openUrl(QUrl::fromLocalFile(filePath));
                } else {
                    QMessageBox::warning(&dlg, "文件不存在", "未找到文档: " + filePath);
                }
            }
        });

        dlg.exec();
    });

    // 下载中心
    connect(actionDownloadCenter, &QAction::triggered, []() {
        QDesktopServices::openUrl(QUrl("https://www.livoxtech.com/cn/downloads"));
    });
}

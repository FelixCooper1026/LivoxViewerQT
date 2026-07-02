#include "LivoxViewerWindow.h"
#include "LivoxCore/LidarSdkService.h"
#include <QCoreApplication>
#include <QDesktopServices>
#include <QDialogButtonBox>
#include <QFrame>
#include <QGridLayout>
#include <QListWidget>
#include <QUrl>

namespace {

QLabel* createAboutMutedLabel(const QString& text, QWidget* parent)
{
    QLabel* label = new QLabel(text, parent);
    label->setWordWrap(true);
    label->setStyleSheet(QStringLiteral("color: palette(mid);"));
    return label;
}

QLabel* createAboutValueLabel(const QString& text, QWidget* parent)
{
    QLabel* label = new QLabel(text, parent);
    label->setWordWrap(true);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    return label;
}

void addAboutInfoRow(QGridLayout* layout, int row, const QString& name, const QString& value, QWidget* parent)
{
    QLabel* nameLabel = createAboutMutedLabel(name, parent);
    QLabel* valueLabel = createAboutValueLabel(value, parent);
    layout->addWidget(nameLabel, row, 0, Qt::AlignTop);
    layout->addWidget(valueLabel, row, 1);
}

} // namespace

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
        QDialog dlg(this);
        dlg.setWindowTitle("关于 LivoxViewerQT");
        dlg.resize(720, 480);

        QVBoxLayout* rootLayout = new QVBoxLayout(&dlg);
        rootLayout->setContentsMargins(18, 18, 18, 16);
        rootLayout->setSpacing(12);

        QFrame* infoFrame = new QFrame(&dlg);
        infoFrame->setObjectName(QStringLiteral("AboutInfoFrame"));
        infoFrame->setFrameShape(QFrame::StyledPanel);
        infoFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
        infoFrame->setStyleSheet(
            "QFrame#AboutInfoFrame {"
            "  border: 1px solid palette(mid);"
            "  border-radius: 8px;"
            "  background: palette(base);"
            "}"
        );
        QVBoxLayout* frameLayout = new QVBoxLayout(infoFrame);
        frameLayout->setContentsMargins(18, 16, 18, 16);
        frameLayout->setSpacing(14);

        QHBoxLayout* headerLayout = new QHBoxLayout();
        headerLayout->setSpacing(14);
        QLabel* iconLabel = new QLabel(infoFrame);
        iconLabel->setPixmap(QIcon(QStringLiteral(":/icons/app_icon.png")).pixmap(QSize(56, 56)));
        iconLabel->setFixedSize(62, 62);
        iconLabel->setAlignment(Qt::AlignCenter);
        headerLayout->addWidget(iconLabel, 0, Qt::AlignTop);

        QVBoxLayout* titleLayout = new QVBoxLayout();
        titleLayout->setSpacing(4);
        QLabel* titleLabel = new QLabel(QStringLiteral("LivoxViewerQT"), infoFrame);
        QFont titleFont = titleLabel->font();
        titleFont.setPointSize(titleFont.pointSize() + 7);
        titleFont.setBold(true);
        titleLabel->setFont(titleFont);
        QLabel* descriptionLabel = createAboutMutedLabel(
            QStringLiteral("集 Livox 设备接入、点云查看、参数调试、数据采集、离线回放、ROSbag 解析与 FAST_LIO SLAM 于一体的桌面工具。"),
            infoFrame);
        titleLayout->addWidget(titleLabel);
        titleLayout->addWidget(descriptionLabel);
        headerLayout->addLayout(titleLayout, 1);
        frameLayout->addLayout(headerLayout);

        QFrame* separator = new QFrame(infoFrame);
        separator->setFrameShape(QFrame::HLine);
        separator->setFrameShadow(QFrame::Plain);
        separator->setStyleSheet(QStringLiteral("color: palette(mid);"));
        frameLayout->addWidget(separator);

        QGridLayout* infoLayout = new QGridLayout();
        infoLayout->setContentsMargins(0, 0, 0, 0);
        infoLayout->setHorizontalSpacing(18);
        infoLayout->setVerticalSpacing(8);
        infoLayout->setColumnStretch(1, 1);
        addAboutInfoRow(infoLayout, 0, QStringLiteral("应用版本"), QCoreApplication::applicationVersion(), infoFrame);
        addAboutInfoRow(infoLayout, 1, QStringLiteral("编译时间"), QStringLiteral(__DATE__) + QStringLiteral(" ") + QStringLiteral(__TIME__), infoFrame);
        addAboutInfoRow(infoLayout, 2, QStringLiteral("Qt 版本"), QStringLiteral(QT_VERSION_STR), infoFrame);
        addAboutInfoRow(infoLayout, 3, QStringLiteral("Livox SDK"), LidarSdkService::versionString(), infoFrame);
        addAboutInfoRow(infoLayout, 4, QStringLiteral("作者"), QStringLiteral("FelixCooper1026"), infoFrame);
        addAboutInfoRow(infoLayout, 5, QStringLiteral("主要能力"),
            QStringLiteral("设备接入：发现设备、自动配置主机IP\n"
                           "点云查看：实时渲染、着色、投影、视角控制\n"
                           "点云分析：测距、框选、点属性、点云裁切\n"
                           "参数配置：基础、网络、FOV、外参与状态参数\n"
                           "数据采集：点云录制、PCD/LAS 导出、IMU/LOG/Debug 采集\n"
                           "离线工具：LVX2/PCAP/ROSbag 回放、LVX2/PCD/LAS/TXT 点云格式转换\n"
                           "SLAM集成：在线/离线建图、轨迹显示/导出、地图显示/导出、性能诊断"),
            infoFrame);
        frameLayout->addLayout(infoLayout);
        rootLayout->addWidget(infoFrame);
        rootLayout->addStretch();

        QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, &dlg);
        if (QPushButton* closeButton = buttonBox->button(QDialogButtonBox::Close)) {
            closeButton->setText(QStringLiteral("关闭"));
        }
        connect(buttonBox, &QDialogButtonBox::rejected, &dlg, &QDialog::accept);
        rootLayout->addWidget(buttonBox);

        dlg.exec();
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

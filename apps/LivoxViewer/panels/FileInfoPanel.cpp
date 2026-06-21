#include "LivoxViewerWindow.h"
#include <QFrame>
#include <QSizePolicy>

void LivoxViewerWindow::createFileInfoPanel()
{
    // 左侧：文件信息 Dock（播放 LVX2 / Pcap 时显示）
    lvx2FileDock = new QDockWidget(QStringLiteral("文件信息"), this);
    lvx2FileDock->setObjectName("Lvx2FileDock");
    lvx2FileDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    lvx2FileDock->setStyleSheet(QStringLiteral(
        "QDockWidget#Lvx2FileDock {"
        "  border: none;"
        "}"
        "QDockWidget#Lvx2FileDock::title {"
        "  border: none;"
        "}"
    ));
    QWidget* hiddenFileTitleBar = new QWidget(lvx2FileDock);
    hiddenFileTitleBar->setFixedHeight(0);
    lvx2FileDock->setTitleBarWidget(hiddenFileTitleBar);

    QWidget* lvx2FileContent = new QWidget(lvx2FileDock);
    QVBoxLayout* lvx2FileLayout = new QVBoxLayout(lvx2FileContent);
    lvx2FileLayout->setContentsMargins(8, 8, 8, 8);
    lvx2FileLayout->setSpacing(6);
    lvx2DeviceListWidget = new QWidget(lvx2FileContent);
    lvx2DeviceListWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    QVBoxLayout* deviceListLayout = new QVBoxLayout(lvx2DeviceListWidget);
    deviceListLayout->setContentsMargins(0, 0, 0, 0);
    deviceListLayout->setSpacing(6);

    QScrollArea* deviceScroll = new QScrollArea(lvx2FileContent);
    deviceScroll->setWidgetResizable(true);
    deviceScroll->setFrameShape(QFrame::NoFrame);
    deviceScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    deviceScroll->setWidget(lvx2DeviceListWidget);
    lvx2FileLayout->addWidget(deviceScroll);
    lvx2FileContent->setLayout(lvx2FileLayout);
    lvx2FileDock->setWidget(lvx2FileContent);
    lvx2FileDock->setMinimumWidth(0);
    addDockWidget(Qt::LeftDockWidgetArea, lvx2FileDock);
    tabifyDockWidget(lidarDevicesDock, imuDock);
    tabifyDockWidget(lidarDevicesDock, lvx2FileDock);
    lidarDevicesDock->raise();
    lvx2FileDock->hide();
}

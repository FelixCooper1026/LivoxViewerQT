#include "LivoxViewerWindow.h"
#include <QHeaderView>
#include <QSizePolicy>

void LivoxViewerWindow::createFileInfoPanel()
{
    // 左侧：文件信息 Dock（播放 LVX2 / Pcap 时显示）
    lvx2FileDock = new QDockWidget(QStringLiteral("文件信息"), this);
    lvx2FileDock->setObjectName("Lvx2FileDock");
    lvx2FileDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    QWidget* lvx2FileContent = new QWidget(lvx2FileDock);
    QVBoxLayout* lvx2FileLayout = new QVBoxLayout(lvx2FileContent);
    QLabel* lvx2Hint = new QLabel(QStringLiteral("文件设备列表（勾选=可见）"), lvx2FileContent);
    lvx2FileLayout->addWidget(lvx2Hint);
    lvx2DeviceTable = new QTableWidget(lvx2FileContent);
    lvx2DeviceTable->setColumnCount(4);
    lvx2DeviceTable->setHorizontalHeaderLabels({"显示", "雷达型号", "Lidar SN", "Lidar IP"});
    lvx2DeviceTable->verticalHeader()->setVisible(false);
    lvx2DeviceTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    lvx2DeviceTable->setSelectionMode(QAbstractItemView::NoSelection);
    lvx2DeviceTable->horizontalHeader()->setStretchLastSection(true);
    lvx2FileLayout->addWidget(lvx2DeviceTable);
    lvx2FileContent->setLayout(lvx2FileLayout);
    lvx2FileDock->setWidget(lvx2FileContent);
    addDockWidget(Qt::LeftDockWidgetArea, lvx2FileDock);
    tabifyDockWidget(imuDock, lvx2FileDock);
    imuDock->raise();
    lvx2FileDock->hide();
}

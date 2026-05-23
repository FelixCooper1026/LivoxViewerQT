#include "LivoxViewerWindow.h"
#include <QHeaderView>
#include <QSizePolicy>

void LivoxViewerWindow::createImuPanel()
{
    // 右侧：IMU数据 Dock
    imuDock = new QDockWidget("IMU数据", this);
    imuDock->setObjectName("ImuDock");
    imuDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    QWidget* imuContent = new QWidget(imuDock);
    QVBoxLayout* imuLayout = new QVBoxLayout(imuContent);
    imuDataTable = new QTableWidget(imuContent);
    imuDataTable->setColumnCount(2);
    imuDataTable->setRowCount(3);
    imuDataTable->setHorizontalHeaderLabels({"Gyro(rad/s)", "Acc(g)"});
    imuDataTable->setVerticalHeaderLabels({"X", "Y", "Z"});
    imuDataTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    imuDataTable->setSelectionMode(QAbstractItemView::NoSelection);
    imuDataTable->horizontalHeader()->setStretchLastSection(true);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 2; ++c) {
            auto* item = new QTableWidgetItem("0.000");
            item->setTextAlignment(Qt::AlignCenter);
            imuDataTable->setItem(r, c, item);
        }
    }
    imuLayout->addWidget(imuDataTable);

    imuAsciiLabel = new QLabel("状态: 未开始", imuContent);
    imuLayout->addWidget(imuAsciiLabel);

    // Display toggle button (2 Hz text refresh)
    imuDisplayButton = new QPushButton("显示IMU数据", imuContent);
    imuLayout->addWidget(imuDisplayButton);
    connect(imuDisplayButton, &QPushButton::clicked, this, &LivoxViewerWindow::onImuDisplayButtonClicked);
    imuLayout->addStretch();
    imuContent->setLayout(imuLayout);
    imuDock->setWidget(imuContent);
    addDockWidget(Qt::LeftDockWidgetArea, imuDock);
}

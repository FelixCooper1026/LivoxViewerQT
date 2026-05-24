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
    imuState.dataTable = new QTableWidget(imuContent);
    imuState.dataTable->setColumnCount(2);
    imuState.dataTable->setRowCount(3);
    imuState.dataTable->setHorizontalHeaderLabels({"Gyro(rad/s)", "Acc(g)"});
    imuState.dataTable->setVerticalHeaderLabels({"X", "Y", "Z"});
    imuState.dataTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    imuState.dataTable->setSelectionMode(QAbstractItemView::NoSelection);
    imuState.dataTable->horizontalHeader()->setStretchLastSection(true);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 2; ++c) {
            auto* item = new QTableWidgetItem("0.000");
            item->setTextAlignment(Qt::AlignCenter);
            imuState.dataTable->setItem(r, c, item);
        }
    }
    imuLayout->addWidget(imuState.dataTable);

    // Display toggle button (2 Hz text refresh)
    imuState.displayButton = new QPushButton("显示IMU数据", imuContent);
    imuLayout->addWidget(imuState.displayButton);
    connect(imuState.displayButton, &QPushButton::clicked, this, &LivoxViewerWindow::onImuDisplayButtonClicked);
    imuLayout->addStretch();
    imuContent->setLayout(imuLayout);
    imuDock->setWidget(imuContent);
    addDockWidget(Qt::LeftDockWidgetArea, imuDock);
}

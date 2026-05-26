#include "LivoxViewerWindow.h"
#include <QSizePolicy>

#include <algorithm>

void LivoxViewerWindow::createImuPanel()
{
    // 右侧：IMU数据 Dock
    imuDock = new QDockWidget("IMU数据", this);
    imuDock->setObjectName("ImuDock");
    imuDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    QWidget* imuContent = new QWidget(imuDock);
    QVBoxLayout* imuLayout = new QVBoxLayout(imuContent);
    imuLayout->setContentsMargins(8, 8, 8, 8);
    imuLayout->setSpacing(8);

    QWidget* valuePanel = new QWidget(imuContent);
    QGridLayout* valueGrid = new QGridLayout(valuePanel);
    valueGrid->setContentsMargins(0, 0, 0, 0);
    valueGrid->setHorizontalSpacing(6);
    valueGrid->setVerticalSpacing(6);
    valueGrid->setColumnStretch(1, 1);
    valueGrid->setColumnStretch(2, 1);

    auto makeHeader = [valuePanel](const QString& title, const QString& unit) {
        QWidget* header = new QWidget(valuePanel);
        QVBoxLayout* layout = new QVBoxLayout(header);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(0);
        QLabel* titleLabel = new QLabel(title, header);
        titleLabel->setAlignment(Qt::AlignCenter);
        QLabel* unitLabel = new QLabel(unit, header);
        unitLabel->setAlignment(Qt::AlignCenter);
        QFont unitFont = unitLabel->font();
        unitFont.setPointSizeF(std::max(7.0, unitFont.pointSizeF() * 0.85));
        unitLabel->setFont(unitFont);
        unitLabel->setStyleSheet("color: palette(mid);");
        layout->addWidget(titleLabel);
        layout->addWidget(unitLabel);
        return header;
    };

    auto makeValueLabel = [valuePanel]() {
        QLabel* label = new QLabel("0.000", valuePanel);
        label->setAlignment(Qt::AlignCenter);
        label->setMinimumWidth(0);
        label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        label->setStyleSheet("QLabel { background-color: palette(base); border: 1px solid palette(mid); padding: 3px 4px; }");
        return label;
    };

    valueGrid->addWidget(makeHeader("Gyro", "rad/s"), 0, 1);
    valueGrid->addWidget(makeHeader("Acc", "g"), 0, 2);
    const QStringList axisNames = {"X", "Y", "Z"};
    for (int r = 0; r < 3; ++r) {
        QLabel* axisLabel = new QLabel(axisNames.at(r), valuePanel);
        axisLabel->setAlignment(Qt::AlignCenter);
        valueGrid->addWidget(axisLabel, r + 1, 0);
        for (int c = 0; c < 2; ++c) {
            imuState.dataValueLabels[r][c] = makeValueLabel();
            valueGrid->addWidget(imuState.dataValueLabels[r][c], r + 1, c + 1);
        }
    }
    imuLayout->addWidget(valuePanel);

    // Display toggle button (2 Hz text refresh)
    imuState.displayButton = new QPushButton("显示IMU数据", imuContent);
    imuLayout->addWidget(imuState.displayButton);
    connect(imuState.displayButton, &QPushButton::clicked, this, &LivoxViewerWindow::onImuDisplayButtonClicked);
    imuLayout->addStretch();
    imuContent->setLayout(imuLayout);

    QScrollArea* imuScroll = new QScrollArea(imuDock);
    imuScroll->setWidgetResizable(true);
    imuScroll->setWidget(imuContent);
    imuDock->setWidget(imuScroll);
    imuDock->setMinimumWidth(0);
    addDockWidget(Qt::LeftDockWidgetArea, imuDock);
}

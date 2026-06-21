#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"
#include <QIcon>
#include <QSize>
#include <QSizePolicy>

void LivoxViewerWindow::createDevicePanel()
{
    networkDock = new QDockWidget(QStringLiteral("网卡"), this);
    networkDock->setObjectName(QStringLiteral("NetworkDock"));
    networkDock->setAllowedAreas(Qt::LeftDockWidgetArea);
    networkDock->setFeatures(QDockWidget::NoDockWidgetFeatures);
    networkDock->setStyleSheet(QStringLiteral(
        "QDockWidget#NetworkDock {"
        "  border: none;"
        "}"
        "QDockWidget#NetworkDock::title {"
        "  border: none;"
        "}"
    ));
    QWidget* hiddenNetworkTitleBar = new QWidget(networkDock);
    hiddenNetworkTitleBar->setFixedHeight(0);
    networkDock->setTitleBarWidget(hiddenNetworkTitleBar);

    lidarDevicesDock = new QDockWidget("设备", this);
    lidarDevicesDock->setObjectName("DevicesDock");
    lidarDevicesDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    lidarDevicesDock->setStyleSheet(QStringLiteral(
        "QDockWidget#DevicesDock {"
        "  border: none;"
        "}"
        "QDockWidget#DevicesDock::title {"
        "  border: none;"
        "}"
    ));
    QWidget* networkBlock = new QWidget(networkDock);
    QVBoxLayout* hostnetworkLayout = new QVBoxLayout(networkBlock);
    hostnetworkLayout->setContentsMargins(8, 8, 8, 8);
    hostnetworkLayout->setSpacing(4);
    QWidget* networkRow = new QWidget(networkBlock);
    QHBoxLayout* networkSelectLayout = new QHBoxLayout(networkRow);
    networkSelectLayout->setContentsMargins(0, 0, 0, 0);
    networkSelectLayout->setSpacing(6);
    QLabel* networkLabel = new QLabel("网卡:", networkRow);
    networkInterfaceCombo = new QComboBox(networkRow);
    networkInterfaceCombo->setMinimumWidth(0);
    networkInterfaceCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    networkInterfaceCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    networkInterfaceCombo->setToolTip("选择用于雷达通信的网络接口");
    QPushButton* btnRefreshNetwork = new QPushButton(networkRow);
    ThemeIconUtils::setThemedSvgIcon(btnRefreshNetwork, QStringLiteral(":/icons/refresh.svg"));
    btnRefreshNetwork->setIconSize(QSize(fontMetrics().height() + 4, fontMetrics().height() + 4));
    btnRefreshNetwork->setFixedWidth(fontMetrics().height() + 18);
    btnRefreshNetwork->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    btnRefreshNetwork->setToolTip("刷新网卡列表并重启设备发现");
    networkSelectLayout->addWidget(networkLabel);
    networkSelectLayout->addWidget(networkInterfaceCombo, 1);
    networkSelectLayout->addWidget(btnRefreshNetwork);
    hostnetworkLayout->addWidget(networkRow);
    networkDock->setWidget(networkBlock);
    const int networkDockHeight = fontMetrics().height() + 32;
    networkDock->setMinimumHeight(networkDockHeight);
    networkDock->setMaximumHeight(networkDockHeight);

    QWidget* hiddenDeviceTitleBar = new QWidget(lidarDevicesDock);
    hiddenDeviceTitleBar->setFixedHeight(0);
    lidarDevicesDock->setTitleBarWidget(hiddenDeviceTitleBar);

    QWidget* lidarDevicesDockContent = new QWidget(lidarDevicesDock);
    QVBoxLayout* lidarDevicesLayout = new QVBoxLayout(lidarDevicesDockContent);
    lidarDevicesLayout->setContentsMargins(8, 8, 8, 8);
    lidarDevicesLayout->setSpacing(8);

    connect(networkInterfaceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &LivoxViewerWindow::onNetworkInterfaceChanged);
    connect(btnRefreshNetwork, &QPushButton::clicked, this, [this]() {
        refreshNetworkInterfaces();
        if (sdk_started || sdk_initialized || realtimeState == RealtimeConnectionState::Running) {
            restartRealtimeConnectionForNetworkChange();
            return;
        }
        stopLidarDiscovery();
        startLidarDiscovery();
    });

    realtimeDeviceListWidget = new QWidget(lidarDevicesDockContent);
    realtimeDeviceListWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    QVBoxLayout* realtimeDeviceListLayout = new QVBoxLayout(realtimeDeviceListWidget);
    realtimeDeviceListLayout->setContentsMargins(0, 0, 0, 0);
    realtimeDeviceListLayout->setSpacing(6);

    QScrollArea* realtimeDeviceScroll = new QScrollArea(lidarDevicesDockContent);
    realtimeDeviceScroll->setWidgetResizable(true);
    realtimeDeviceScroll->setFrameShape(QFrame::NoFrame);
    realtimeDeviceScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    realtimeDeviceScroll->setMinimumHeight(80);
    realtimeDeviceScroll->setWidget(realtimeDeviceListWidget);
    lidarDevicesLayout->addWidget(realtimeDeviceScroll);

    lidarDevicesLayout->addStretch();
    lidarDevicesDockContent->setLayout(lidarDevicesLayout);

    QScrollArea* lidarDevicesScroll = new QScrollArea(lidarDevicesDock);
    lidarDevicesScroll->setObjectName(QStringLiteral("DevicesDockScroll"));
    lidarDevicesScroll->setWidgetResizable(true);
    lidarDevicesScroll->setFrameShape(QFrame::NoFrame);
    lidarDevicesScroll->setStyleSheet(QStringLiteral(
        "QScrollArea#DevicesDockScroll {"
        "  border: 0;"
        "  background: palette(window);"
        "}"
    ));
    lidarDevicesScroll->setWidget(lidarDevicesDockContent);
    lidarDevicesDock->setWidget(lidarDevicesScroll);
    lidarDevicesDock->setMinimumWidth(0);

    addDockWidget(Qt::LeftDockWidgetArea, networkDock);
    addDockWidget(Qt::LeftDockWidgetArea, lidarDevicesDock);
    splitDockWidget(networkDock, lidarDevicesDock, Qt::Vertical);
}

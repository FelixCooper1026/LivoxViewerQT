#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"
#include <QIcon>
#include <QSize>
#include <QSizePolicy>

void LivoxViewerWindow::createDevicePanel()
{
    // 左侧：设备与状态 Dock
    lidarDevicesDock = new QDockWidget("设备", this);
    lidarDevicesDock->setObjectName("DevicesDock");
    lidarDevicesDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    QWidget* lidarDevicesDockContent = new QWidget(lidarDevicesDock);
    QVBoxLayout* lidarDevicesLayout = new QVBoxLayout(lidarDevicesDockContent);
    lidarDevicesLayout->setContentsMargins(8, 8, 8, 8);
    lidarDevicesLayout->setSpacing(8);

    // 网络接口选择
    QWidget* networkBlock = new QWidget(lidarDevicesDockContent);
    QVBoxLayout* hostnetworkLayout = new QVBoxLayout(networkBlock);
    hostnetworkLayout->setContentsMargins(0, 0, 0, 0);
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
    lidarDevicesLayout->addWidget(networkBlock);
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
    lidarDevicesScroll->setWidgetResizable(true);
    lidarDevicesScroll->setWidget(lidarDevicesDockContent);
    lidarDevicesDock->setWidget(lidarDevicesScroll);
    lidarDevicesDock->setMinimumWidth(0);

    addDockWidget(Qt::LeftDockWidgetArea, lidarDevicesDock);
}

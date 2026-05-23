#include "LivoxViewerWindow.h"
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

    // 网络接口选择（放在设备管理dock上方）
    QWidget* networkRow = new QWidget(lidarDevicesDockContent);
    QHBoxLayout* hostnetworkLayout = new QHBoxLayout(networkRow);
    hostnetworkLayout->setContentsMargins(0, 0, 0, 0);
    hostnetworkLayout->setSpacing(6);
    QLabel* networkLabel = new QLabel("网卡选择:", networkRow);
    networkInterfaceCombo = new QComboBox(networkRow);
    networkInterfaceCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    networkInterfaceCombo->setToolTip("选择用于雷达通信的网络接口");
    QPushButton* btnRefreshNetwork = new QPushButton("刷新", networkRow);
    btnRefreshNetwork->setIcon(QIcon(":/icons/refresh.svg"));
    btnRefreshNetwork->setIconSize(QSize(fontMetrics().height() + 4, fontMetrics().height() + 4));
    btnRefreshNetwork->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    hostnetworkLayout->addWidget(networkLabel);
    hostnetworkLayout->addWidget(networkInterfaceCombo, 1);
    hostnetworkLayout->addWidget(btnRefreshNetwork);
    lidarDevicesLayout->addWidget(networkRow);
    connect(networkInterfaceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &LivoxViewerWindow::onNetworkInterfaceChanged);
    connect(btnRefreshNetwork, &QPushButton::clicked, this, &LivoxViewerWindow::refreshNetworkInterfaces);

    // Linux 下默认不自动修改主机网口 IP（需要 root/sudo）；可由用户主动开启
    {
        QWidget* autoIpRow = new QWidget(lidarDevicesDockContent);
        QHBoxLayout* h = new QHBoxLayout(autoIpRow);
        h->setContentsMargins(0, 0, 0, 0);
        h->setSpacing(6);
        autoConfigHostIpCheck = new QCheckBox("自动修改主机网口IP（需管理员/Root）", autoIpRow);
        autoConfigHostIpCheck->setToolTip("开启后程序会尝试自动将所选网卡的 IPv4 配置到与雷达同网段。\nLinux 可能需要 sudo/root 权限；默认关闭以避免权限导致的失败。");
        h->addWidget(autoConfigHostIpCheck, 1);
        lidarDevicesLayout->addWidget(autoIpRow);

        connect(autoConfigHostIpCheck, &QCheckBox::toggled, this, [this](bool enabled) {
            autoConfigHostIpEnabled = enabled;
            QSettings settings("Livox", "LivoxViewerQT");
            settings.setValue("network/autoConfigHostIp", enabled);
            logMessage(QString("自动修改主机IP: %1").arg(enabled ? "已启用" : "已关闭"));
        });
    }

    QGroupBox* deviceGroup = new QGroupBox("设备管理", lidarDevicesDockContent);
    deviceGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
    QVBoxLayout* deviceLayout = new QVBoxLayout(deviceGroup);
    deviceLayout->setContentsMargins(8,8,8,8);
    deviceLayout->setSpacing(6);

    // 顶部按钮行（紧凑）
    QHBoxLayout* deviceButtonLayout = new QHBoxLayout();
    deviceButtonLayout->setContentsMargins(0,0,0,0);
    deviceButtonLayout->setSpacing(6);
    deviceButtonLayout->addStretch();
    deviceLayout->addLayout(deviceButtonLayout);

    // 设备列表（高度压缩）
    lidarDeviceList = new QListWidget(deviceGroup);
    lidarDeviceList->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    lidarDeviceList->setMinimumHeight(120);
    lidarDeviceList->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    lidarDeviceList->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    lidarDeviceList->setSelectionMode(QAbstractItemView::ExtendedSelection); // 支持多选
    deviceLayout->addWidget(lidarDeviceList);

    // 设备管理区不再放置点云控制（移至顶部工具栏）

    lidarDevicesLayout->addWidget(deviceGroup);

    // GPS模拟与串口转发输入控件
    {
        QGroupBox* gpsGroup = new QGroupBox("时间同步", lidarDevicesDockContent);
        QVBoxLayout* gpsLayout = new QVBoxLayout(gpsGroup);
        imuState.gpsSimulateCheck = new QCheckBox("启用GPS模拟输入(GPRMC)", gpsGroup);
        // 第1行：启用GPS模拟输入（左对齐）
        {
            QWidget* rowSim = new QWidget(gpsGroup);
            QHBoxLayout* hSim = new QHBoxLayout(rowSim);
            hSim->setContentsMargins(0,0,0,0);
            hSim->addWidget(imuState.gpsSimulateCheck);
            hSim->addStretch();
            gpsLayout->addWidget(rowSim);
        }
        connect(imuState.gpsSimulateCheck, &QCheckBox::toggled, this, &LivoxViewerWindow::onGpsSimulateToggled);
        // 第2行：启用串口转发输入（左对齐）
        imuState.serialEnableCheck = new QCheckBox("启用串口转发输入(GPRMC)", gpsGroup);
        {
            QWidget* rowEnable = new QWidget(gpsGroup);
            QHBoxLayout* hEn = new QHBoxLayout(rowEnable);
            hEn->setContentsMargins(0,0,0,0);
            hEn->addWidget(imuState.serialEnableCheck);
            hEn->addStretch();
            gpsLayout->addWidget(rowEnable);
        }
        connect(imuState.serialEnableCheck, &QCheckBox::toggled, this, &LivoxViewerWindow::onSerialEnableToggled);
        // 第3行：串口选择与刷新
        imuState.serialPortCombo = new QComboBox(gpsGroup);
        QPushButton* btnRefreshSerial = new QPushButton("刷新串口", gpsGroup);
        {
            QWidget* rowSer = new QWidget(gpsGroup);
            QHBoxLayout* hSer = new QHBoxLayout(rowSer);
            hSer->setContentsMargins(0,0,0,0);
            hSer->addWidget(new QLabel("串口:"));
            hSer->addWidget(imuState.serialPortCombo, 1);
            hSer->addWidget(btnRefreshSerial);
            gpsLayout->addWidget(rowSer);
        }
        gpsGroup->setLayout(gpsLayout);
        lidarDevicesLayout->addWidget(gpsGroup);
        connect(btnRefreshSerial, &QPushButton::clicked, this, &LivoxViewerWindow::refreshSerialPorts);
        // 初始化串口列表
        refreshSerialPorts();
    }

    lidarDevicesLayout->addStretch();
    lidarDevicesDockContent->setLayout(lidarDevicesLayout);

    QScrollArea* lidarDevicesScroll = new QScrollArea(lidarDevicesDock);
    lidarDevicesScroll->setWidgetResizable(true);
    lidarDevicesScroll->setWidget(lidarDevicesDockContent);
    lidarDevicesDock->setWidget(lidarDevicesScroll);
    lidarDevicesDock->setMinimumWidth(0);

    addDockWidget(Qt::LeftDockWidgetArea, lidarDevicesDock);
}

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
    btnRefreshNetwork->setIcon(QIcon(":/icons/refresh.svg"));
    btnRefreshNetwork->setIconSize(QSize(fontMetrics().height() + 4, fontMetrics().height() + 4));
    btnRefreshNetwork->setFixedWidth(fontMetrics().height() + 18);
    btnRefreshNetwork->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    btnRefreshNetwork->setToolTip("刷新网卡列表");
    networkSelectLayout->addWidget(networkLabel);
    networkSelectLayout->addWidget(networkInterfaceCombo, 1);
    networkSelectLayout->addWidget(btnRefreshNetwork);
    hostnetworkLayout->addWidget(networkRow);
    lidarDevicesLayout->addWidget(networkBlock);
    connect(networkInterfaceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &LivoxViewerWindow::onNetworkInterfaceChanged);
    connect(btnRefreshNetwork, &QPushButton::clicked, this, &LivoxViewerWindow::refreshNetworkInterfaces);

    QGroupBox* deviceGroup = new QGroupBox("设备管理", lidarDevicesDockContent);
    deviceGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    QVBoxLayout* deviceLayout = new QVBoxLayout(deviceGroup);
    deviceLayout->setContentsMargins(8,8,8,8);
    deviceLayout->setSpacing(6);

    // 设备列表（高度压缩）
    lidarDeviceList = new QListWidget(deviceGroup);
    lidarDeviceList->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    lidarDeviceList->setMinimumHeight(80);
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
        imuState.gpsSimulateCheck = new QCheckBox("GPS模拟输入", gpsGroup);
        imuState.gpsSimulateCheck->setToolTip("启用 GPS 模拟输入（GPRMC）");
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
        imuState.serialEnableCheck = new QCheckBox("串口转发输入", gpsGroup);
        imuState.serialEnableCheck->setToolTip("启用串口转发输入（GPRMC）");
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
        imuState.serialPortCombo->setMinimumWidth(0);
        QPushButton* btnRefreshSerial = new QPushButton(gpsGroup);
        btnRefreshSerial->setIcon(QIcon(":/icons/refresh.svg"));
        btnRefreshSerial->setIconSize(QSize(fontMetrics().height() + 4, fontMetrics().height() + 4));
        btnRefreshSerial->setFixedWidth(fontMetrics().height() + 18);
        btnRefreshSerial->setToolTip("刷新串口列表");
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

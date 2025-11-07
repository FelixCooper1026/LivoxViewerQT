#include "mainwindow.h"
#include <QApplication>
#include <QSplitter>
#include <QScrollArea>
#include <QColorDialog>
#include <QFrame>
#include <QVariant>
#include <QHeaderView>
#include <QInputDialog>
#include <QFileDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QNetworkInterface>
#include <QHostAddress>
#include <QAbstractSocket>
#include <QListWidget>
#include <QDesktopServices>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , sdk_initialized(false)
    , sdk_started(false)
    , currentDevice(nullptr)
    , statusLabel(nullptr)
    , pointCloudCallbackEnabled(false)
    , isNormalMode(true)
    , discoverySocket(nullptr)
    , discoveryTimer(nullptr)
    , discoveryActive(false)
    , recordParamsButton(nullptr)
    , isRecordingParams(false)
{
    setupUI();

    // 启动设备发现，SDK初始化将在设备发现完成后进行
    startDeviceDiscovery();

    // 移除状态栏自动更新逻辑

    // 设置参数查询定时器
    paramQueryTimer = new QTimer(this);
    connect(paramQueryTimer, &QTimer::timeout, this, &MainWindow::onParamQueryTimeout);
    paramQueryTimer->start(1000); // 每1秒查询一次参数

    // 恢复窗口布局与几何
    QSettings settings("Livox", "LivoxViewerQT");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

MainWindow::~MainWindow()
{
    // 保存窗口布局与几何
    QSettings settings("Livox", "LivoxViewerQT");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());

    stopDeviceDiscovery();
    cleanupLivoxSDK();
}

void MainWindow::setupUI()
{
    // 设置应用程序字体，避免DirectWrite错误
    QFont appFont = QApplication::font();
    appFont.setFamily("Microsoft YaHei"); // 使用微软雅黑字体
    appFont.setPointSize(9);
    QApplication::setFont(appFont);

    // 中央视图：点云可视化
    QWidget* centralContainer = new QWidget(this);
    QVBoxLayout* centralLayout = new QVBoxLayout(centralContainer);
    centralLayout->setContentsMargins(0,0,0,0);
    centralLayout->setSpacing(0);

    // 顶部可视化功能栏（两行）
    QWidget* viewerToolbar = new QWidget(centralContainer);
    viewerToolbar->setObjectName("ViewerToolbar");
    QVBoxLayout* viewerLayout = new QVBoxLayout(viewerToolbar);
    viewerLayout->setContentsMargins(8,4,8,4);
    viewerLayout->setSpacing(4);
    
    // 第一行工具栏
    QWidget* toolbarRow1 = new QWidget(viewerToolbar);
    QHBoxLayout* row1Layout = new QHBoxLayout(toolbarRow1);
    row1Layout->setContentsMargins(0,0,0,0);
    row1Layout->setSpacing(8);
    
    // 第二行工具栏
    QWidget* toolbarRow2 = new QWidget(viewerToolbar);
    QHBoxLayout* row2Layout = new QHBoxLayout(toolbarRow2);
    row2Layout->setContentsMargins(0,0,0,0);
    row2Layout->setSpacing(8);

    // 积分时间
    QLabel* lblFrame = new QLabel("积分时间:", toolbarRow1);
    QSpinBox* spinFrameIntervalTop = new QSpinBox(toolbarRow1);
    spinFrameIntervalTop->setRange(100, 30000);
    spinFrameIntervalTop->setSingleStep(100);
    spinFrameIntervalTop->setSuffix(" ms");
    spinFrameIntervalTop->setValue(static_cast<int>(frameIntervalMs));
    spinFrameIntervalTop->setToolTip("点云积分时间/帧间隔（渲染为滑动窗口显示）");
    connect(spinFrameIntervalTop, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::onFrameIntervalChanged);

    // 点大小
    QLabel* lblSize = new QLabel("点大小:", toolbarRow1);
    pointSizeSpin = new QSpinBox(toolbarRow1);
    pointSizeSpin->setRange(1, 10);
    pointSizeSpin->setValue(static_cast<int>(pointSizePx));
    pointSizeSpin->setToolTip("点大小（像素）");
    connect(pointSizeSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::onPointSizeChanged);

    // 着色模式
    QLabel* lblColor = new QLabel("着色:", toolbarRow1);
    colorModeCombo = new QComboBox(toolbarRow1);
    colorModeCombo->addItems({"反射率", "距离", "高度", "纯色", "平面投影"});
    colorModeCombo->setCurrentIndex(colorMode);
    colorModeCombo->setToolTip("点云着色模式");
    connect(colorModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onColorModeChanged);

    // 球坐标深度投影（移到第二行）
    QLabel* lblSpherical = new QLabel("球面投影:", toolbarRow2);
    projectionDepthCheck = new QCheckBox("启用", toolbarRow2);
    projectionDepthCheck->setChecked(projectionDepthEnabled);
    projectionDepthCheck->setToolTip("启用后按固定距离对深度进行投影，仅在球坐标点云时生效");
    connect(projectionDepthCheck, &QCheckBox::toggled, this, &MainWindow::onProjectionDepthToggled);

    QLabel* lblProj = new QLabel("投影深度(m):", toolbarRow2);
    projectionDepthSpin = new QDoubleSpinBox(toolbarRow2);
    projectionDepthSpin->setRange(0.0, 10000.0);
    projectionDepthSpin->setDecimals(1);
    projectionDepthSpin->setSingleStep(1.0);
    projectionDepthSpin->setValue(projectionDepthMeters);
    projectionDepthSpin->setToolTip("球坐标时，将depth投影到指定距离；0表示使用原始depth");
    connect(projectionDepthSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onProjectionDepthChanged);

    // 平面投影控制（移到第二行）
    QLabel* lblPlanarProj = new QLabel("平面投影:", toolbarRow2);
    planarProjectionCheck = new QCheckBox("启用", toolbarRow2);
    planarProjectionCheck->setChecked(planarProjectionEnabled);
    planarProjectionCheck->setToolTip("启用平面投影模式，将半球面展开为平面图");
    connect(planarProjectionCheck, &QCheckBox::toggled, this, &MainWindow::onPlanarProjectionToggled);
    
    QLabel* lblPlanarRadius = new QLabel("投影半径(m):", toolbarRow2);
    planarRadiusSpin = new QDoubleSpinBox(toolbarRow2);
    planarRadiusSpin->setRange(1.0, 1000.0);
    planarRadiusSpin->setDecimals(1);
    planarRadiusSpin->setSingleStep(1.0);
    planarRadiusSpin->setValue(planarProjectionRadius);
    planarRadiusSpin->setToolTip("平面投影的半径大小");
    connect(planarRadiusSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onPlanarProjectionRadiusChanged);



    // 纯色选择控件
    solidColorRow = new QWidget(toolbarRow1);
    QHBoxLayout* colorRowLayoutTop = new QHBoxLayout(solidColorRow);
    colorRowLayoutTop->setContentsMargins(0,0,0,0);
    colorRowLayoutTop->setSpacing(6);
    solidColorPreview = new QFrame(solidColorRow);
    solidColorPreview->setFixedSize(20, 20);
    solidColorPreview->setFrameShape(QFrame::Box);
    solidColorPreview->setLineWidth(1);
    solidColorPreview->setStyleSheet(QString("background-color: %1;").arg(solidColor.name()));
    solidColorButton = new QPushButton("选择颜色", solidColorRow);
    colorRowLayoutTop->addWidget(solidColorPreview);
    colorRowLayoutTop->addWidget(solidColorButton);
    connect(solidColorButton, &QPushButton::clicked, this, &MainWindow::onSolidColorClicked);
    solidColorRow->setEnabled(colorMode == ColorSolid);

    // 暂停/开启点云可视化按钮
    QPushButton* btnToggleVisualization = new QPushButton(pointCloudVisualizationEnabled ? "暂停可视化" : "开启可视化", toolbarRow1);
    btnToggleVisualization->setToolTip("暂停/开启点云可视化更新");
    connect(btnToggleVisualization, &QPushButton::clicked, [this, btnToggleVisualization]() {
        bool newState = !pointCloudVisualizationEnabled;
        onPointCloudVisualizationToggled(newState);
        btnToggleVisualization->setText(newState ? "暂停可视化" : "开启可视化");
    });

    // 操作按钮
    QPushButton* btnToggleSelection = new QPushButton("点云框选", toolbarRow1);
    QPushButton* btnMeasure = new QPushButton("点云测距", toolbarRow1);
    QPushButton* btnReset = new QPushButton("重置视图", toolbarRow1);
    connect(btnToggleSelection, &QPushButton::clicked, [this, btnToggleSelection]() {
        if (!pointCloudWidget) return;
        bool enable = !pointCloudWidget->isSelectionModeEnabled();
        pointCloudWidget->setSelectionModeEnabled(enable);
        if (!enable) {
            pointCloudWidget->clearSelectionAabb();
            // 立即记录清除日志并清空表格
            if (lastSelectionCount != -1) {
                lastSelectionCount = -1;
                logMessage("已清除框选");
            }
            if (attrTable) {
                bool sorting = attrTable->isSortingEnabled();
                attrTable->setSortingEnabled(false);
                attrTable->clearContents();
                attrTable->setRowCount(0);
                attrTable->setSortingEnabled(sorting);
            } else if (selectionTable) {
                bool sorting = selectionTable->isSortingEnabled();
                selectionTable->setSortingEnabled(false);
                selectionTable->clearContents();
                selectionTable->setRowCount(0);
                selectionTable->setSortingEnabled(sorting);
            }
            // 关闭点属性弹窗并停止日志
            if (attrDock) { attrDock->hide(); }
            selectionRealtimeEnabled = false;
            statusLabelBar->setText("已连接 - 采样中");
        } else {
            // 打开点属性弹窗
            if (attrDock) {
                attrDock->show();
                attrDock->raise();
            }
            selectionRealtimeEnabled = true;
            statusLabelBar->setText("点云框选模式：按住Ctrl+左键拖动选择区域");
        }
        btnToggleSelection->setText(enable ? "退出点云框选" : "点云框选");
    });
    connect(btnReset, &QPushButton::clicked, [this]() { pointCloudWidget->resetView(); });

    // 测距按钮逻辑
    connect(btnMeasure, &QPushButton::clicked, [this, btnMeasure]() {
        if (!pointCloudWidget) return;
        bool enable = !pointCloudWidget->isMeasurementModeEnabled();
        pointCloudWidget->setMeasurementModeEnabled(enable);
        if (enable) {
            statusLabelBar->setText("测距模式：按住Ctrl+左键选择第一点");
            logMessage("进入测距模式，已暂停点云播放");
        } else {
            statusLabelBar->setText("已连接 - 采样中");
            logMessage("退出测距模式，恢复点云播放");
        }
        btnMeasure->setText(enable ? "退出测距" : "点云测距");
    });

    // 拼装第一行工具栏
    row1Layout->addWidget(lblFrame);
    row1Layout->addWidget(spinFrameIntervalTop);
    row1Layout->addSpacing(8);
    row1Layout->addWidget(lblSize);
    row1Layout->addWidget(pointSizeSpin);
    row1Layout->addSpacing(8);
    row1Layout->addWidget(lblColor);
    row1Layout->addWidget(colorModeCombo);
    row1Layout->addWidget(solidColorRow);
    row1Layout->addSpacing(8);
    row1Layout->addWidget(btnToggleVisualization);
    row1Layout->addSpacing(8);
    row1Layout->addWidget(btnMeasure);
    row1Layout->addWidget(btnToggleSelection);
    row1Layout->addWidget(btnReset);
    row1Layout->addStretch();

    // 拼装第二行工具栏
    row2Layout->addWidget(lblSpherical);
    row2Layout->addWidget(projectionDepthCheck);
    row2Layout->addWidget(lblProj);
    row2Layout->addWidget(projectionDepthSpin);
    row2Layout->addSpacing(10);
    row2Layout->addWidget(lblPlanarProj);
    row2Layout->addWidget(planarProjectionCheck);
    row2Layout->addWidget(lblPlanarRadius);
    row2Layout->addWidget(planarRadiusSpin);
    row2Layout->addStretch();

    // 将两行添加到主工具栏
    viewerLayout->addWidget(toolbarRow1);
    viewerLayout->addWidget(toolbarRow2);

    // 可视化窗口
    pointCloudWidget = new PointCloudWidget(centralContainer);
    pointCloudWidget->setMinimumSize(800, 500);
    pointCloudWidget->setPointSize(pointSizePx);

    centralLayout->addWidget(viewerToolbar);
    centralLayout->addWidget(pointCloudWidget, 1);
    setCentralWidget(centralContainer);

    // 初始化深度投影控件状态（仅在球坐标时可用）
    if (projectionDepthCheck) {
        projectionDepthCheck->setEnabled(false);  // 默认禁用，直到选择球坐标
    }
    if (projectionDepthSpin) {
        projectionDepthSpin->setEnabled(false);  // 默认禁用，直到选择球坐标
    }
    
    // 初始化平面投影控件状态（仅在球坐标时可用）
    if (planarProjectionCheck) {
        planarProjectionCheck->setEnabled(false);  // 默认禁用，直到选择球坐标
    }
    if (planarRadiusSpin) {
        planarRadiusSpin->setEnabled(false);  // 默认禁用，直到选择球坐标
    }

    // 左侧：设备与状态 Dock
    devicesDock = new QDockWidget("设备", this);
    devicesDock->setObjectName("DevicesDock");
    devicesDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    QWidget* devicesDockContent = new QWidget(devicesDock);
    QVBoxLayout* devicesLayout = new QVBoxLayout(devicesDockContent);
    devicesLayout->setContentsMargins(8, 8, 8, 8);
    devicesLayout->setSpacing(8);

    QGroupBox* deviceGroup = new QGroupBox("设备管理", devicesDockContent);
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
    deviceList = new QListWidget(deviceGroup);
    deviceList->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    deviceList->setMinimumHeight(120);
    deviceList->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    deviceList->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    deviceLayout->addWidget(deviceList);

    // 状态行（缩小）



    // 设备管理区不再放置点云控制（移至顶部工具栏）

    devicesLayout->addWidget(deviceGroup);

    // 移除原设备视图中的点属性表格（改为右侧点属性Dock）


    // GPS模拟与串口转发输入控件
    {
        QGroupBox* gpsGroup = new QGroupBox("时间同步", devicesDockContent);
        QVBoxLayout* gpsLayout = new QVBoxLayout(gpsGroup);
        gpsSimulateCheck = new QCheckBox("启用GPS模拟输入(GPRMC)", gpsGroup);
        // 第1行：启用GPS模拟输入（左对齐）
        {
            QWidget* rowSim = new QWidget(gpsGroup);
            QHBoxLayout* hSim = new QHBoxLayout(rowSim);
            hSim->setContentsMargins(0,0,0,0);
            hSim->addWidget(gpsSimulateCheck);
            hSim->addStretch();
            gpsLayout->addWidget(rowSim);
        }
        connect(gpsSimulateCheck, &QCheckBox::toggled, this, &MainWindow::onGpsSimulateToggled);
        // 第2行：启用串口转发输入（左对齐）
        serialEnableCheck = new QCheckBox("启用串口转发输入(GPRMC)", gpsGroup);
        {
            QWidget* rowEnable = new QWidget(gpsGroup);
            QHBoxLayout* hEn = new QHBoxLayout(rowEnable);
            hEn->setContentsMargins(0,0,0,0);
            hEn->addWidget(serialEnableCheck);
            hEn->addStretch();
            gpsLayout->addWidget(rowEnable);
        }
        connect(serialEnableCheck, &QCheckBox::toggled, this, &MainWindow::onSerialEnableToggled);
        // 第3行：串口选择与刷新
        serialPortCombo = new QComboBox(gpsGroup);
        QPushButton* btnRefreshSerial = new QPushButton("刷新串口", gpsGroup);
        {
            QWidget* rowSer = new QWidget(gpsGroup);
            QHBoxLayout* hSer = new QHBoxLayout(rowSer);
            hSer->setContentsMargins(0,0,0,0);
            hSer->addWidget(new QLabel("串口:"));
            hSer->addWidget(serialPortCombo, 1);
            hSer->addWidget(btnRefreshSerial);
            gpsLayout->addWidget(rowSer);
        }
        gpsGroup->setLayout(gpsLayout);
        devicesLayout->addWidget(gpsGroup);
        connect(btnRefreshSerial, &QPushButton::clicked, this, &MainWindow::refreshSerialPorts);
        // 初始化串口列表
        refreshSerialPorts();
    }

    devicesLayout->addStretch();
    devicesDockContent->setLayout(devicesLayout);

    QScrollArea* devicesScroll = new QScrollArea(devicesDock);
    devicesScroll->setWidgetResizable(true);
    devicesScroll->setWidget(devicesDockContent);
    devicesDock->setWidget(devicesScroll);
    devicesDock->setMinimumWidth(200);

    addDockWidget(Qt::LeftDockWidgetArea, devicesDock);

    // 右侧：参数 Dock（包含标签页）
    paramsDock = new QDockWidget("参数", this);
    paramsDock->setObjectName("ParamsDock");
    paramsDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    QWidget* paramsDockContent = new QWidget(paramsDock);
    QVBoxLayout* paramsOuterLayout = new QVBoxLayout(paramsDockContent);
    paramsOuterLayout->setContentsMargins(8, 8, 8, 8);
    paramsOuterLayout->setSpacing(8);

    // 点属性 Dock（默认隐藏），尺寸与"参数"一致
    attrDock = new QDockWidget("点属性", this);
    attrDock->setObjectName("AttrDock");
    attrDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    QWidget* attrContent = new QWidget(attrDock);
    QVBoxLayout* attrLayout = new QVBoxLayout(attrContent);
    attrTable = new QTableWidget(attrContent);
    attrTable->setColumnCount(5);
    attrTable->setHorizontalHeaderLabels({"X(m)", "Y(m)", "Z(m)", "Refl", "Tag"});
    attrTable->verticalHeader()->setVisible(false);
    attrTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    attrTable->setSelectionMode(QAbstractItemView::NoSelection);
    attrTable->horizontalHeader()->setStretchLastSection(true);
    attrTable->setSortingEnabled(true);
    attrLayout->addWidget(attrTable);
    attrContent->setLayout(attrLayout);
    attrDock->setWidget(attrContent);
    addDockWidget(Qt::RightDockWidgetArea, attrDock);
    attrDock->hide();

    // 可配置参数键集合
    QVector<uint16_t> configurableKeysVec = {
        kKeyPclDataType, kKeyPatternMode, kKeyDetectMode, kKeyWorkMode, kKeyImuDataEn,
        kKeyLidarIpCfg, kKeyStateInfoHostIpCfg, kKeyLidarPointDataHostIpCfg, kKeyLidarImuHostIpCfg,
        kKeyFovCfg0, kKeyFovCfg1, kKeyFovCfgEn, kKeyInstallAttitude
    };
    // 状态参数
    QVector<uint16_t> statusKeysVec = {
        kKeySn, kKeyProductInfo, kKeyVersionApp, kKeyVersionLoader, kKeyVersionHardware, kKeyMac,
        kKeyCurWorkState, kKeyCoreTemp, kKeyPowerUpCnt, kKeyLocalTimeNow, kKeyLastSyncTime,
        kKeyTimeOffset, kKeyTimeSyncType, kKeyLidarDiagStatus, kKeyFwType, kKeyHmsCode
    };
    for (uint16_t key : configurableKeysVec) this->configurableKeys.insert(key);
    for (uint16_t key : statusKeysVec) this->statusKeys.insert(key);

    // 参数标签
    paramTabWidget = new QTabWidget(paramsDockContent);
    paramTabWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    paramTabWidget->setMinimumWidth(0);

    // 基本配置页
    QWidget* basicTab = new QWidget();
    basicTab->setMinimumWidth(0);
    basicTab->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    QFormLayout* basicLayout = new QFormLayout(basicTab);
    basicLayout->setSpacing(8);
    basicLayout->setContentsMargins(10, 10, 10, 10);
    basicLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    basicLayout->setRowWrapPolicy(QFormLayout::WrapAllRows);
    basicLayout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

    QComboBox* workModeCombo = new QComboBox();
    workModeCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    workModeCombo->addItems({"采样模式", "待机模式"});
    workModeCombo->setCurrentIndex(0);
    basicLayout->addRow("工作模式:", workModeCombo);
    paramControls[kKeyWorkMode] = workModeCombo;
    connect(workModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeyWorkMode); });

    QComboBox* patternModeCombo = new QComboBox();
    patternModeCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    patternModeCombo->addItems({"非重复扫描", "重复扫描", "低帧率重复扫描"});
    patternModeCombo->setCurrentIndex(0);
    basicLayout->addRow("扫描模式:", patternModeCombo);
    paramControls[kKeyPatternMode] = patternModeCombo;
    connect(patternModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeyPatternMode); });

    QComboBox* pclDataTypeCombo = new QComboBox();
    pclDataTypeCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    pclDataTypeCombo->addItems({"高精度笛卡尔坐标", "低精度笛卡尔坐标", "球坐标"});
    pclDataTypeCombo->setCurrentIndex(0);
    basicLayout->addRow("点云格式:", pclDataTypeCombo);
    paramControls[kKeyPclDataType] = pclDataTypeCombo;
    connect(pclDataTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeyPclDataType); });

    QComboBox* detectModeCombo = new QComboBox();
    detectModeCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    detectModeCombo->addItems({"正常模式", "敏感模式"});
    detectModeCombo->setCurrentIndex(0);
    basicLayout->addRow("探测模式:", detectModeCombo);
    paramControls[kKeyDetectMode] = detectModeCombo;
    connect(detectModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeyDetectMode); });

    QComboBox* imuDataCombo = new QComboBox();
    imuDataCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    imuDataCombo->addItems({"关闭", "开启"});
    imuDataCombo->setCurrentIndex(0);
    basicLayout->addRow("IMU数据发送:", imuDataCombo);
    paramControls[kKeyImuDataEn] = imuDataCombo;
    connect(imuDataCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeyImuDataEn); });

    QComboBox* motorSpeedCombo = new QComboBox();
    motorSpeedCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    motorSpeedCombo->addItems({"正常转速","低转速"});
    motorSpeedCombo->setCurrentIndex(0);
    basicLayout->addRow("电机转速:", motorSpeedCombo);
    paramControls[kKeySetEscMode] = motorSpeedCombo;
    connect(motorSpeedCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeySetEscMode); });

    paramTabWidget->addTab(basicTab, "基本配置");

    // 网络配置页
    QWidget* networkTab = new QWidget();
    networkTab->setMinimumWidth(0);
    networkTab->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    QFormLayout* networkLayout = new QFormLayout(networkTab);
    networkLayout->setSpacing(8);
    networkLayout->setContentsMargins(10, 10, 10, 10);
    networkLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    networkLayout->setRowWrapPolicy(QFormLayout::WrapAllRows);
    networkLayout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

    // 雷达IP 子表单（可换行）
    QLineEdit* lidarIpEdit = new QLineEdit(); lidarIpEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QLineEdit* lidarMaskEdit = new QLineEdit(); lidarMaskEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QLineEdit* lidarGatewayEdit = new QLineEdit(); lidarGatewayEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QPushButton* lidarIpButton = new QPushButton("应用");
    lidarIpButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    QWidget* lidarIpContainer = new QWidget();
    QFormLayout* lidarForm = new QFormLayout(lidarIpContainer);
    lidarForm->setRowWrapPolicy(QFormLayout::WrapAllRows);
    lidarForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    lidarForm->addRow("IP:", lidarIpEdit);
    lidarForm->addRow("掩码:", lidarMaskEdit);
    lidarForm->addRow("网关:", lidarGatewayEdit);
    // 右对齐按钮行，避免按钮拉伸
    {
        QWidget* btnRow = new QWidget();
        QHBoxLayout* btnLayout = new QHBoxLayout(btnRow);
        btnLayout->setContentsMargins(0,0,0,0);
        btnLayout->addStretch();
        btnLayout->addWidget(lidarIpButton);
        lidarForm->addRow(QString(), btnRow);
    }
    lidarIpContainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    networkLayout->addRow(new QLabel("雷达IP:"));
    networkLayout->addRow(lidarIpContainer);
    paramControls[kKeyLidarIpCfg] = lidarIpContainer;
    connect(lidarIpButton, &QPushButton::clicked, [this, lidarIpEdit, lidarMaskEdit, lidarGatewayEdit]() { applyIpConfig(kKeyLidarIpCfg, lidarIpEdit->text(), lidarMaskEdit->text(), lidarGatewayEdit->text()); });

    // 点云数据目的IP 子表单
    QLineEdit* pointDataIpEdit = new QLineEdit(); pointDataIpEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QSpinBox* pointDataPortEdit = new QSpinBox(); pointDataPortEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    pointDataPortEdit->setRange(1, 65535);
    pointDataPortEdit->setValue(57000);
    QPushButton* pointDataButton = new QPushButton("应用");
    pointDataButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    QWidget* pointDataContainer = new QWidget();
    QFormLayout* pointForm = new QFormLayout(pointDataContainer);
    pointForm->setRowWrapPolicy(QFormLayout::WrapAllRows);
    pointForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    pointForm->addRow("IP:", pointDataIpEdit);
    pointForm->addRow("端口:", pointDataPortEdit);
    {
        QWidget* btnRow = new QWidget();
        QHBoxLayout* btnLayout = new QHBoxLayout(btnRow);
        btnLayout->setContentsMargins(0,0,0,0);
        btnLayout->addStretch();
        btnLayout->addWidget(pointDataButton);
        pointForm->addRow(QString(), btnRow);
    }
    pointDataContainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    networkLayout->addRow(new QLabel("点云数据IP:"));
    networkLayout->addRow(pointDataContainer);
    paramControls[kKeyLidarPointDataHostIpCfg] = pointDataContainer;
    connect(pointDataButton, &QPushButton::clicked, [this, pointDataIpEdit, pointDataPortEdit]() { applyHostIpConfig(kKeyLidarPointDataHostIpCfg, pointDataIpEdit->text(), pointDataPortEdit->value()); });

    // IMU数据目的IP 子表单
    QLineEdit* imuDataIpEdit = new QLineEdit(); imuDataIpEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QSpinBox* imuDataPortEdit = new QSpinBox(); imuDataPortEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    imuDataPortEdit->setRange(1, 65535);
    imuDataPortEdit->setValue(57000);
    QPushButton* imuDataButton = new QPushButton("应用");
    imuDataButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    QWidget* imuDataContainer = new QWidget();
    QFormLayout* imuForm = new QFormLayout(imuDataContainer);
    imuForm->setRowWrapPolicy(QFormLayout::WrapAllRows);
    imuForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    imuForm->addRow("IP:", imuDataIpEdit);
    imuForm->addRow("端口:", imuDataPortEdit);
    {
        QWidget* btnRow = new QWidget();
        QHBoxLayout* btnLayout = new QHBoxLayout(btnRow);
        btnLayout->setContentsMargins(0,0,0,0);
        btnLayout->addStretch();
        btnLayout->addWidget(imuDataButton);
        imuForm->addRow(QString(), btnRow);
    }
    imuDataContainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    networkLayout->addRow(new QLabel("IMU数据IP:"));
    networkLayout->addRow(imuDataContainer);
    paramControls[kKeyLidarImuHostIpCfg] = imuDataContainer;
    connect(imuDataButton, &QPushButton::clicked, [this, imuDataIpEdit, imuDataPortEdit]() { applyHostIpConfig(kKeyLidarImuHostIpCfg, imuDataIpEdit->text(), imuDataPortEdit->value()); });

    // 状态信息目的IP 子表单
    QLineEdit* stateInfoIpEdit = new QLineEdit(); stateInfoIpEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QSpinBox* stateInfoPortEdit = new QSpinBox(); stateInfoPortEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    stateInfoPortEdit->setRange(1, 65535);
    stateInfoPortEdit->setValue(57000);
    QPushButton* stateInfoButton = new QPushButton("应用");
    stateInfoButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    QWidget* stateInfoContainer = new QWidget();
    QFormLayout* stateForm = new QFormLayout(stateInfoContainer);
    stateForm->setRowWrapPolicy(QFormLayout::WrapAllRows);
    stateForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    stateForm->addRow("IP:", stateInfoIpEdit);
    stateForm->addRow("端口:", stateInfoPortEdit);
    {
        QWidget* btnRow = new QWidget();
        QHBoxLayout* btnLayout = new QHBoxLayout(btnRow);
        btnLayout->setContentsMargins(0,0,0,0);
        btnLayout->addStretch();
        btnLayout->addWidget(stateInfoButton);
        stateForm->addRow(QString(), btnRow);
    }
    stateInfoContainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    networkLayout->addRow(new QLabel("状态信息IP:"));
    networkLayout->addRow(stateInfoContainer);
    paramControls[kKeyStateInfoHostIpCfg] = stateInfoContainer;
    connect(stateInfoButton, &QPushButton::clicked, [this, stateInfoIpEdit, stateInfoPortEdit]() { applyHostIpConfig(kKeyStateInfoHostIpCfg, stateInfoIpEdit->text(), stateInfoPortEdit->value()); });

    paramTabWidget->addTab(networkTab, "网络配置");
    networkTab->setLayout(networkLayout);

    // FOV 配置页
    QWidget* fovTab = new QWidget();
    fovTab->setMinimumWidth(0);
    fovTab->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    QFormLayout* fovLayout = new QFormLayout(fovTab);
    fovLayout->setSpacing(8);
    fovLayout->setContentsMargins(10, 10, 10, 10);
    fovLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    fovLayout->setRowWrapPolicy(QFormLayout::WrapAllRows);
    fovLayout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    fovLayout->setFormAlignment(Qt::AlignTop);
    fovLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    fovLayout->setRowWrapPolicy(QFormLayout::WrapAllRows);
    QCheckBox* fov0EnableCheck = new QCheckBox();
    paramControls[kKeyFovCfgEn] = fov0EnableCheck;
    QCheckBox* fov1EnableCheck = new QCheckBox();
    paramControls[0x001F] = fov1EnableCheck;
    connect(fov0EnableCheck, &QCheckBox::toggled, [this, fov0EnableCheck, fov1EnableCheck]() { updateFovEnableState(fov0EnableCheck, fov1EnableCheck); });
    connect(fov1EnableCheck, &QCheckBox::toggled, [this, fov0EnableCheck, fov1EnableCheck]() { updateFovEnableState(fov0EnableCheck, fov1EnableCheck); });
    // FOV0配置行，使用 FlowLayout 实现行内多控件自动换行
    QSpinBox* fov0YawStartEdit = new QSpinBox();
    QSpinBox* fov0YawStopEdit = new QSpinBox();
    QSpinBox* fov0PitchStartEdit = new QSpinBox();
    QSpinBox* fov0PitchStopEdit = new QSpinBox();
    QPushButton* fov0Button = new QPushButton("应用");
    fov0YawStartEdit->setRange(0, 360);
    fov0YawStopEdit->setRange(0, 360);
    fov0PitchStartEdit->setRange(-10, 60);
    fov0PitchStopEdit->setRange(-10, 60);
    QWidget* fov0Container = new QWidget();
    fov0Container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    QGridLayout* fov0Grid = new QGridLayout(fov0Container);
    fov0Grid->setContentsMargins(0, 0, 0, 0);
    fov0Grid->setHorizontalSpacing(8);
    fov0Grid->setVerticalSpacing(4);
    fov0Grid->addWidget(new QLabel("Yaw:"), 0, 0);
    fov0Grid->addWidget(fov0YawStartEdit, 0, 1);
    fov0Grid->addWidget(new QLabel("~"), 0, 2);
    fov0Grid->addWidget(fov0YawStopEdit, 0, 3);
    fov0Grid->addWidget(new QLabel("Pitch:"), 1, 0);
    fov0Grid->addWidget(fov0PitchStartEdit, 1, 1);
    fov0Grid->addWidget(new QLabel("~"), 1, 2);
    fov0Grid->addWidget(fov0PitchStopEdit, 1, 3);
    fov0Grid->setColumnStretch(1, 1);
    fov0Grid->setColumnStretch(3, 1);
    fov0Grid->addItem(new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), 1, 4);
    fov0Grid->addWidget(fov0Button, 1, 5);
    {
        QWidget* fov0Label = new QWidget();
        QHBoxLayout* fov0LabelLayout = new QHBoxLayout(fov0Label);
        fov0LabelLayout->setContentsMargins(0,0,0,0);
        fov0LabelLayout->addStretch();
        fov0LabelLayout->addWidget(new QLabel("FOV0配置"));
        fov0LabelLayout->addSpacing(6);
        fov0LabelLayout->addWidget(fov0EnableCheck);
        fovLayout->addRow(fov0Label, fov0Container);
    }
    paramControls[kKeyFovCfg0] = fov0Container;
    connect(fov0Button, &QPushButton::clicked, [this, fov0YawStartEdit, fov0YawStopEdit, fov0PitchStartEdit, fov0PitchStopEdit]() { applyFovConfig(kKeyFovCfg0, fov0YawStartEdit->value(), fov0YawStopEdit->value(), fov0PitchStartEdit->value(), fov0PitchStopEdit->value()); });

    // FOV1配置行，使用 FlowLayout
    QSpinBox* fov1YawStartEdit = new QSpinBox();
    QSpinBox* fov1YawStopEdit = new QSpinBox();
    QSpinBox* fov1PitchStartEdit = new QSpinBox();
    QSpinBox* fov1PitchStopEdit = new QSpinBox();
    QPushButton* fov1Button = new QPushButton("应用");
    fov1YawStartEdit->setRange(0, 360);
    fov1YawStopEdit->setRange(0, 360);
    fov1PitchStartEdit->setRange(-10, 60);
    fov1PitchStopEdit->setRange(-10, 60);
    QWidget* fov1Container = new QWidget();
    fov1Container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    QGridLayout* fov1Grid = new QGridLayout(fov1Container);
    fov1Grid->setContentsMargins(0, 0, 0, 0);
    fov1Grid->setHorizontalSpacing(8);
    fov1Grid->setVerticalSpacing(4);
    fov1Grid->addWidget(new QLabel("Yaw:"), 0, 0);
    fov1Grid->addWidget(fov1YawStartEdit, 0, 1);
    fov1Grid->addWidget(new QLabel("~"), 0, 2);
    fov1Grid->addWidget(fov1YawStopEdit, 0, 3);
    fov1Grid->addWidget(new QLabel("Pitch:"), 1, 0);
    fov1Grid->addWidget(fov1PitchStartEdit, 1, 1);
    fov1Grid->addWidget(new QLabel("~"), 1, 2);
    fov1Grid->addWidget(fov1PitchStopEdit, 1, 3);
    fov1Grid->setColumnStretch(1, 1);
    fov1Grid->setColumnStretch(3, 1);
    fov1Grid->addItem(new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Minimum), 1, 4);
    fov1Grid->addWidget(fov1Button, 1, 5);
    {
        QWidget* fov1Label = new QWidget();
        QHBoxLayout* fov1LabelLayout = new QHBoxLayout(fov1Label);
        fov1LabelLayout->setContentsMargins(0,0,0,0);
        fov1LabelLayout->addStretch();
        fov1LabelLayout->addWidget(new QLabel("FOV1配置"));
        fov1LabelLayout->addSpacing(6);
        fov1LabelLayout->addWidget(fov1EnableCheck);
        fovLayout->addRow(fov1Label, fov1Container);
    }
    paramControls[kKeyFovCfg1] = fov1Container;
    connect(fov1Button, &QPushButton::clicked, [this, fov1YawStartEdit, fov1YawStopEdit, fov1PitchStartEdit, fov1PitchStopEdit]() { applyFovConfig(kKeyFovCfg1, fov1YawStartEdit->value(), fov1YawStopEdit->value(), fov1PitchStartEdit->value(), fov1PitchStopEdit->value()); });

    paramTabWidget->addTab(fovTab, "FOV配置");
    fovTab->setLayout(fovLayout);

    // 外参配置页
    QWidget* attitudeTab = new QWidget();
    attitudeTab->setMinimumWidth(0);
    attitudeTab->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    QFormLayout* attitudeLayout = new QFormLayout(attitudeTab);
    attitudeLayout->setSpacing(8);
    attitudeLayout->setContentsMargins(10, 10, 10, 10);
    attitudeLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    attitudeLayout->setRowWrapPolicy(QFormLayout::WrapAllRows);
    attitudeLayout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    attitudeLayout->setFormAlignment(Qt::AlignTop);
    QDoubleSpinBox* rollEdit = new QDoubleSpinBox();
    QDoubleSpinBox* pitchEdit = new QDoubleSpinBox();
    QDoubleSpinBox* yawEdit = new QDoubleSpinBox();
    QSpinBox* xEdit = new QSpinBox();
    QSpinBox* yEdit = new QSpinBox();
    QSpinBox* zEdit = new QSpinBox();
    QPushButton* attitudeButton = new QPushButton("应用");
    rollEdit->setRange(-180.0, 180.0);
    pitchEdit->setRange(-90.0, 90.0);
    yawEdit->setRange(-180.0, 180.0);
    xEdit->setRange(-10000, 10000);
    yEdit->setRange(-10000, 10000);
    zEdit->setRange(-10000, 10000);
    // 安装姿态：每项占一行
    attitudeLayout->addRow("Roll:", rollEdit);
    attitudeLayout->addRow("Pitch:", pitchEdit);
    attitudeLayout->addRow("Yaw:", yawEdit);
    attitudeLayout->addRow("X:", xEdit);
    attitudeLayout->addRow("Y:", yEdit);
    attitudeLayout->addRow("Z:", zEdit);
    {
        QWidget* applyRow = new QWidget();
        QHBoxLayout* applyLayout = new QHBoxLayout(applyRow);
        applyLayout->setContentsMargins(0,0,0,0);
        applyLayout->addStretch();
        applyLayout->addWidget(attitudeButton);
        attitudeLayout->addRow(QString(), applyRow);
    }
    paramControls[kKeyInstallAttitude] = attitudeTab;
    connect(attitudeButton, &QPushButton::clicked, [this, rollEdit, pitchEdit, yawEdit, xEdit, yEdit, zEdit]() { applyAttitudeConfig(kKeyInstallAttitude, rollEdit->value(), pitchEdit->value(), yawEdit->value(), xEdit->value(), yEdit->value(), zEdit->value()); });
    paramTabWidget->addTab(attitudeTab, "外参配置");
    attitudeTab->setLayout(attitudeLayout);

    // 状态信息页
    QWidget* statusTab = new QWidget();
    statusTab->setMinimumWidth(0);
    statusTab->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    QFormLayout* statusLayout = new QFormLayout(statusTab);
    statusLayout->setSpacing(8);
    statusLayout->setContentsMargins(10, 10, 10, 10);
    statusLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    statusLayout->setRowWrapPolicy(QFormLayout::WrapLongRows);
    statusLayout->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    statusLayout->setFormAlignment(Qt::AlignTop);
    for (uint16_t key : statusKeysVec) {
        QLabel* nameLabel = new QLabel();
        QLabel* valueLabel = new QLabel("未查询");
            valueLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
            valueLabel->setWordWrap(true);
            valueLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
            valueLabel->setTextFormat(Qt::PlainText);
            valueLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
            valueLabel->setMinimumWidth(0);
            valueLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
            valueLabel->setStyleSheet("QLabel { background-color: #f0f0f0; padding: 2px; border: 1px solid #ccc; }");
        nameLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        nameLabel->setMinimumWidth(100);
        switch (key) {
            case kKeySn: nameLabel->setText("序列号:"); break;
            case kKeyProductInfo: nameLabel->setText("产品信息:"); break;
            case kKeyVersionApp: nameLabel->setText("固件版本:"); break;
            case kKeyVersionLoader: nameLabel->setText("LOADER版本:"); break;
            case kKeyVersionHardware: nameLabel->setText("硬件版本:"); break;
            case kKeyMac: nameLabel->setText("MAC地址:"); break;
            case kKeyCurWorkState: nameLabel->setText("当前工作状态:"); break;
            case kKeyCoreTemp: nameLabel->setText("核心温度:"); break;
            case kKeyPowerUpCnt: nameLabel->setText("上电次数:"); break;
            case kKeyLocalTimeNow: nameLabel->setText("本地时间:"); break;
            case kKeyLastSyncTime: nameLabel->setText("最后同步时间:"); break;
            case kKeyTimeOffset: nameLabel->setText("时间偏移:"); break;
            case kKeyTimeSyncType: nameLabel->setText("时间同步类型:"); break;
            case kKeyLidarDiagStatus: nameLabel->setText("雷达诊断状态:"); break;
            case kKeyFwType: nameLabel->setText("固件类型:"); break;
            case kKeyHmsCode: nameLabel->setText("HMS诊断码:"); break;
        }
        statusLayout->addRow(nameLabel, valueLabel);
        paramLabels[key] = valueLabel;
    }

    // 添加记录参数按钮
    recordParamsButton = new QPushButton("记录参数至CSV文件", statusTab);
    recordParamsButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    recordParamsButton->setStyleSheet("QPushButton { padding: 5px; }");
    statusLayout->addRow(recordParamsButton); // 这会创建一个占据整行的按钮

    // 连接按钮信号
    connect(recordParamsButton, &QPushButton::clicked, this, &MainWindow::onRecordParamsClicked);

    paramTabWidget->insertTab(0, statusTab, "状态信息");
    paramTabWidget->setCurrentIndex(0);

    paramsOuterLayout->addWidget(paramTabWidget);
    paramsDockContent->setLayout(paramsOuterLayout);

    QScrollArea* paramsScroll = new QScrollArea(paramsDock);
    paramsScroll->setWidgetResizable(true);
    paramsScroll->setWidget(paramsDockContent);
    paramsScroll->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    paramsDock->setWidget(paramsScroll);
    paramsDock->setMinimumWidth(360);

    addDockWidget(Qt::RightDockWidgetArea, paramsDock);

    // 右侧：IMU数据 Dock
    QDockWidget* imuDock = new QDockWidget("IMU数据", this);
    imuDock->setObjectName("ImuDock");
    imuDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    QWidget* imuContent = new QWidget(imuDock);
    QVBoxLayout* imuLayout = new QVBoxLayout(imuContent);
    // No per-field labels; ASCII table only
    imuAsciiLabel = new QLabel("", imuContent);
    QFont mono = imuAsciiLabel->font();
    mono.setFamily("Consolas");
    mono.setStyleHint(QFont::Monospace);
    imuAsciiLabel->setFont(mono);
    imuAsciiLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    imuAsciiLabel->setText(buildImuAscii(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    imuAsciiLabel->setStyleSheet("QLabel { background-color: #fafafa; border: 1px solid #e0e0e0; padding: 6px; }");
    imuLayout->addWidget(imuAsciiLabel);

    // Display toggle button (2 Hz text refresh)
    imuDisplayButton = new QPushButton("显示IMU数据", imuContent);
    imuLayout->addWidget(imuDisplayButton);
    connect(imuDisplayButton, &QPushButton::clicked, this, &MainWindow::onImuDisplayButtonClicked);

    imuLayout->addStretch();
    imuContent->setLayout(imuLayout);
    imuDock->setWidget(imuContent);
    addDockWidget(Qt::LeftDockWidgetArea, imuDock);

    // 底部：日志 Dock
    logDock = new QDockWidget("日志", this);
    logDock->setObjectName("LogDock");
    logDock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    QWidget* logDockContent = new QWidget(logDock);
    QVBoxLayout* logLayout = new QVBoxLayout(logDockContent);
    logText = new QTextEdit(logDockContent);
    logText->setMinimumHeight(160);
    QPushButton* clearLogButton = new QPushButton("清除日志", logDockContent);
    logLayout->addWidget(logText);
    logLayout->addWidget(clearLogButton);
    logDockContent->setLayout(logLayout);
    logDock->setWidget(logDockContent);
    logDock->setMinimumHeight(160);

    addDockWidget(Qt::BottomDockWidgetArea, logDock);

    // 初始布局尺寸（近似 CloudCompare）：左侧窄、右侧中、底部适中
    resizeDocks({devicesDock}, {240}, Qt::Horizontal);
    resizeDocks({paramsDock}, {360}, Qt::Horizontal);
    resizeDocks({logDock}, {240}, Qt::Vertical);

    // 顶部工具栏
    // mainToolBar = addToolBar("主工具栏");
    // mainToolBar->setObjectName("MainToolBar");
    // mainToolBar->setMovable(true);
    // actionStartSdk = mainToolBar->addAction("启动SDK");
    // actionStopSdk = mainToolBar->addAction("停止SDK");
    // mainToolBar->addSeparator();
    // actionRefresh = mainToolBar->addAction("刷新设备");
    // mainToolBar->addSeparator();
    actionClearCloud = new QAction("清除点云", this);
    actionResetView = new QAction("重置视图", this);

    // 菜单栏
    menuBar = new QMenuBar(this);
    setMenuBar(menuBar);
    fileMenu = menuBar->addMenu("文件");
    viewMenu = menuBar->addMenu("视图");
    deviceMenu = menuBar->addMenu("设备");
    QMenu* toolsMenu = menuBar->addMenu("工具");
    helpMenu = menuBar->addMenu("帮助");

    QAction* actionGenerateConfig = fileMenu->addAction("生成配置文件...");
    exitAction = fileMenu->addAction("退出");

    connect(actionGenerateConfig, &QAction::triggered, this, [this]() {
        runConfigGeneratorDialog();
    });

    // 数据采集子菜单
    QMenu* captureMenu = toolsMenu->addMenu("数据采集");
    QAction* actionCaptureLog = captureMenu->addAction("LOG数据采集...");
    QAction* actionCaptureDebug = captureMenu->addAction("Debug数据采集...");
    QMenu* saveMenu = toolsMenu->addMenu("保存点云");
    QAction* actionCaptureLVX2 = saveMenu->addAction("保存LVX2点云...");
    QAction* actionCapturePCD = saveMenu->addAction("保存PCD点云...");
    QAction* actionCaptureLAS = saveMenu->addAction("保存LAS点云...");
    QAction* actionSaveIMU = toolsMenu->addAction("保存IMU数据...");

    // 固件升级
    QAction* actionUpgrade = deviceMenu->addAction("固件升级...");

    // ==== 帮助菜单 ====
    // 1. Livox 官网
    QAction* actionLivoxWebsite = helpMenu->addAction("Livox 官网");
    // 2. Livox Wiki
    QAction* actionLivoxWiki = helpMenu->addAction("Livox Wiki");
    // 3. Mid-360 HMS 故障诊断码说明
    QAction* actionHmsCode = helpMenu->addAction("Mid-360 故障诊断码说明");
    // 4. 时间同步说明
    QAction* actionTimeSync = helpMenu->addAction("时间同步说明");
    // 5. 产品知识库(暂未实现)
    QAction* actionKnowledgeBase = helpMenu->addAction("产品知识库");
    // 6. 下载中心
    QAction* actionDownloadCenter = helpMenu->addAction("下载中心");

    // 关于
    aboutAction = helpMenu->addAction("关于");

    // 退出
    connect(exitAction, &QAction::triggered, this, &QWidget::close);
    // 关于
    connect(aboutAction, &QAction::triggered, [this]() {
        QMessageBox::about(this, "关于 LivoxViewerQT",
                        "<h3>LivoxViewerQT - Livox 激光雷达可视化配置软件</h3>"
                        "<p><b>版本:</b> 1.0.0</p>"
                        "<p><b>编译日期:</b> " __DATE__ " </p>"
                        "<p><b>作者:</b> FelixCooper1026</p>"
                        "<p><b>功能特性:</b></p>"
                        "<ul>"
                        "<li>Livox 激光雷达设备连接与管理</li>"
                        "<li>实时点云数据可视化</li>"
                        "<li>设备参数配置与状态监控</li>"
                        "<li>点云数据采集与保存</li>"
                        "<li>IMU 数据显示与记录</li>"
                        "<li>设备LOG数据采集与保存</li>"
                        "<li>设备固件升级</li>"
                        "</ul>"
                        "<p>基于 Qt " QT_VERSION_STR " 和 Livox SDK2 v1.3.0 开发</p>");
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



    // 采集动作：弹窗输入时长，顶部显示进度条（复用已有captureProgress，放在状态栏）
    connect(actionCaptureLog, &QAction::triggered, [this]() {
        bool ok = false;
        int sec = QInputDialog::getInt(this, "LOG数据采集", "采集时长(秒):", 10, 1, 3600, 1, &ok);
        if (!ok) return;
        if (!captureDurationSpin) {
            captureDurationSpin = new QSpinBox(this);
            captureDurationSpin->setRange(1, 3600);   // ⭐ 设置最大值为 3600
        }
        captureDurationSpin->setValue(sec);
        onStartCaptureLog();
    });
    connect(actionCaptureDebug, &QAction::triggered, [this]() {
        bool ok = false;
        int sec = QInputDialog::getInt(this, "Debug数据采集", "采集时长(秒):", 10, 1, 3600, 1, &ok);
        if (!ok) return;
        if (!captureDurationSpin) {
            captureDurationSpin = new QSpinBox(this);
            captureDurationSpin->setRange(1, 3600);   // ⭐ 设置最大值为 3600
        }
        captureDurationSpin->setValue(sec);
        onStartCaptureDebug();
    });

    connect(actionCapturePCD, &QAction::triggered, [this]() {
        if (!currentDevice || !currentDevice->is_connected) {
            QMessageBox::warning(this, "保存PCD点云", "设备未连接");
            return;
        }
        // 弹窗：保存路径 + 帧数
        QDialog dlg(this);
        dlg.setWindowTitle("保存PCD点云");
        QVBoxLayout* v = new QVBoxLayout(&dlg);
        QWidget* row1 = new QWidget(&dlg);
        QHBoxLayout* h1 = new QHBoxLayout(row1);
        h1->setContentsMargins(0,0,0,0);
        QLabel* lblPath = new QLabel("请选择保存路径:", row1);
        QLineEdit* editPath = new QLineEdit(row1);
        QPushButton* btnBrowse = new QPushButton("选择", row1);
        h1->addWidget(lblPath);
        h1->addSpacing(8);
        h1->addWidget(editPath, 1);
        h1->addSpacing(8);
        h1->addWidget(btnBrowse);
        v->addWidget(row1);

        QWidget* row2 = new QWidget(&dlg);
        QHBoxLayout* h2 = new QHBoxLayout(row2);
        h2->setContentsMargins(0,0,0,0);
        QLabel* lblCount = new QLabel("保存帧数:", row2);
        QSpinBox* spinCount = new QSpinBox(row2);
        spinCount->setRange(1, 1000000);
        spinCount->setSingleStep(1);
        spinCount->setValue(1);
        h2->addWidget(lblCount);
        h2->addSpacing(8);
        h2->addWidget(spinCount);
        h2->addStretch();
        v->addWidget(row2);

        QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        v->addWidget(box);

        connect(btnBrowse, &QPushButton::clicked, &dlg, [editPath, this]() {
            QString dir = QFileDialog::getExistingDirectory(this, "选择保存目录", QDir::homePath());
            if (!dir.isEmpty()) editPath->setText(dir);
        });
        connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

        if (dlg.exec() != QDialog::Accepted) return;
        QString baseDir = editPath->text().trimmed();
        if (baseDir.isEmpty()) {
            QMessageBox::warning(this, "保存PCD点云", "请选择保存路径");
            return;
        }
        // 创建 PCD_雷达SN 目录
        QString sn = currentDevice ? currentDevice->sn : QString("Unknown");
        QString targetDir = QDir(baseDir).filePath(QString("PCD_%1").arg(sn));
        QDir().mkpath(targetDir);
        pcdSaveDir = targetDir;
        pcdFramesRemaining = spinCount->value();
        pcdSaveActive = true;
        pcdLastSavedTimestamp = 0;
        statusLabelBar->setText(QString("开始保存PCD，共 %1 帧...").arg(pcdFramesRemaining));
        logMessage(QString("PCD保存目录: %1").arg(QDir::toNativeSeparators(pcdSaveDir)));
    });

    connect(actionCaptureLAS, &QAction::triggered, [this]() {
        if (!currentDevice || !currentDevice->is_connected) {
            QMessageBox::warning(this, "保存LAS点云", "设备未连接");
            return;
        }
        QDialog dlg(this);
        dlg.setWindowTitle("保存LAS点云");
        QVBoxLayout* v = new QVBoxLayout(&dlg);
        QWidget* row1 = new QWidget(&dlg);
        QHBoxLayout* h1 = new QHBoxLayout(row1);
        h1->setContentsMargins(0,0,0,0);
        QLabel* lblPath = new QLabel("请选择保存路径:", row1);
        QLineEdit* editPath = new QLineEdit(row1);
        QPushButton* btnBrowse = new QPushButton("选择", row1);
        h1->addWidget(lblPath);
        h1->addSpacing(8);
        h1->addWidget(editPath, 1);
        h1->addSpacing(8);
        h1->addWidget(btnBrowse);
        v->addWidget(row1);

        QWidget* row2 = new QWidget(&dlg);
        QHBoxLayout* h2 = new QHBoxLayout(row2);
        h2->setContentsMargins(0,0,0,0);
        QLabel* lblCount = new QLabel("保存帧数:", row2);
        QSpinBox* spinCount = new QSpinBox(row2);
        spinCount->setRange(1, 1000000);
        spinCount->setSingleStep(1);
        spinCount->setValue(1);
        h2->addWidget(lblCount);
        h2->addSpacing(8);
        h2->addWidget(spinCount);
        h2->addStretch();
        v->addWidget(row2);

        QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        v->addWidget(box);

        connect(btnBrowse, &QPushButton::clicked, &dlg, [editPath, this]() {
            QString dir = QFileDialog::getExistingDirectory(this, "选择保存目录", QDir::homePath());
            if (!dir.isEmpty()) editPath->setText(dir);
        });
        connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

        if (dlg.exec() != QDialog::Accepted) return;
        QString baseDir = editPath->text().trimmed();
        if (baseDir.isEmpty()) {
            QMessageBox::warning(this, "保存LAS点云", "请选择保存路径");
            return;
        }
        QString sn = currentDevice ? currentDevice->sn : QString("Unknown");
        QString targetDir = QDir(baseDir).filePath(QString("LAS_%1").arg(sn));
        QDir().mkpath(targetDir);
        lasSaveDir = targetDir;
        lasFramesRemaining = spinCount->value();
        lasSaveActive = true;
        lasLastSavedTimestamp = 0;
        statusLabelBar->setText(QString("开始保存LAS，共 %1 帧...").arg(lasFramesRemaining));
        logMessage(QString("LAS保存目录: %1").arg(QDir::toNativeSeparators(lasSaveDir)));
    });

    connect(actionCaptureLVX2, &QAction::triggered, [this]() {
        if (!currentDevice || !currentDevice->is_connected) {
            QMessageBox::warning(this, "保存LVX2点云", "设备未连接");
            return;
        }
        QDialog dlg(this);
        dlg.setWindowTitle("保存LVX2点云");
        QVBoxLayout* v = new QVBoxLayout(&dlg);
        QWidget* row1 = new QWidget(&dlg);
        QHBoxLayout* h1 = new QHBoxLayout(row1);
        h1->setContentsMargins(0,0,0,0);
        QLabel* lblPath = new QLabel("请选择保存路径:", row1);
        QLineEdit* editPath = new QLineEdit(row1);
        QPushButton* btnBrowse = new QPushButton("选择", row1);
        h1->addWidget(lblPath);
        h1->addSpacing(8);
        h1->addWidget(editPath, 1);
        h1->addSpacing(8);
        h1->addWidget(btnBrowse);
        v->addWidget(row1);

        QWidget* row2 = new QWidget(&dlg);
        QHBoxLayout* h2 = new QHBoxLayout(row2);
        h2->setContentsMargins(0,0,0,0);
        QLabel* lblSec = new QLabel("录制时长(s):", row2);
        QSpinBox* spinSec = new QSpinBox(row2);
        spinSec->setRange(1, 3600);
        spinSec->setSingleStep(1);
        spinSec->setValue(10);
        h2->addWidget(lblSec);
        h2->addSpacing(8);
        h2->addWidget(spinSec);
        h2->addStretch();
        v->addWidget(row2);

        QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        v->addWidget(box);

        connect(btnBrowse, &QPushButton::clicked, &dlg, [editPath, this]() {
            QString dir = QFileDialog::getExistingDirectory(this, "选择保存目录", QDir::homePath());
            if (!dir.isEmpty()) editPath->setText(dir);
        });
        connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

        if (dlg.exec() != QDialog::Accepted) return;
        QString baseDir = editPath->text().trimmed();
        if (baseDir.isEmpty()) {
            QMessageBox::warning(this, "保存LVX2点云", "请选择保存路径");
            return;
        }
        QString sn = currentDevice ? currentDevice->sn : QString("Unknown");
        QString targetDir = QDir(baseDir).filePath(QString("LVX2_%1").arg(sn));
        QDir().mkpath(targetDir);
        QString startTime = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
        QString filePath = QDir(targetDir).filePath(QString("%1_%2.lvx2").arg(sn, startTime));

        // 配置进度条
        if (captureProgress) {
            captureProgress->setRange(0, 100);
            captureProgress->setValue(0);
            captureProgress->setFormat("录制中 %p% (%v s)");
        }
        // 保存配置并启动录制倒计时
        captureSecondsRemaining = spinSec->value();
        captureTotalSeconds = captureSecondsRemaining;
        currentCapture = CaptureLVX2;
        statusLabelBar->setText("正在录制LVX2...");
        logMessage(QString("LVX2保存路径: %1").arg(QDir::toNativeSeparators(filePath)));
        startLvx2Recording(filePath, captureSecondsRemaining);
        captureTimer->start(1000);
    });

    // 调整状态栏进度条长度
    if (captureProgress) {
        captureProgress->setFixedWidth(260);
    }

    connect(actionUpgrade, &QAction::triggered, [this]() {
        if (!currentDevice || !currentDevice->is_connected) {
            QMessageBox::warning(this, "固件升级", "设备未连接");
            return;
        }
                 // 记住上次选择的固件路径
         QSettings settings("Livox", "LivoxViewerQT");
         QString lastDir = settings.value("upgrade/lastFirmwareDir", QDir::homePath()).toString();
         QString fw = QFileDialog::getOpenFileName(this, "选择固件文件", lastDir, "固件 (*.bin *.img);;所有文件 (*.*)");
         if (fw.isEmpty()) return;
         settings.setValue("upgrade/lastFirmwareDir", QFileInfo(fw).absolutePath());
         // 设置升级路径为固件文件完整路径（本地分隔符 + 本地8位编码）
        QFileInfo fi(fw);
        QString tryPath = fi.absoluteFilePath();
        QByteArray pathLocal = QDir::toNativeSeparators(tryPath).toLocal8Bit();
        bool okPath = SetLivoxLidarUpgradeFirmwarePath(pathLocal.constData());
        if (!okPath) {
            QMessageBox::critical(this, "固件升级", "设置固件路径失败，请确保选择单个固件文件，路径避免包含特殊字符");
            return;
        }
        SetLivoxLidarUpgradeProgressCallback([](uint32_t handle, LivoxLidarUpgradeState state, void* client){
            MainWindow* w = static_cast<MainWindow*>(client);
            if (!w || !w->captureProgress) return;
            QMetaObject::invokeMethod(w, [w, state]() {
                w->captureProgress->setValue(state.progress);
                w->captureProgress->setFormat(QString("升级进度 %1% ").arg(state.progress));
                if (state.progress >= 100) {
                    w->statusLabelBar->setText("升级完成");
                }
            });
        }, this);
        // 直接启动升级（无需切换工作模式）
        uint32_t handleArr[1] = { currentDevice->handle };
        // 在后台线程执行升级，避免阻塞UI
        std::thread([h=currentDevice->handle]() {
            uint32_t arr[1] = { h };
            UpgradeLivoxLidars(arr, 1);
        }).detach();
        captureProgress->setValue(0);
        captureProgress->setFormat("升级进度 0%");
        statusLabelBar->setText("正在升级，请勿断电...");
    });


    // 设备菜单新增：重启雷达 / 恢复出厂设置
    QAction* actionReboot = deviceMenu->addAction("重启雷达");
    QAction* actionFactoryReset = deviceMenu->addAction("恢复出厂设置");

    connect(actionReboot, &QAction::triggered, [this]() {
        if (!currentDevice || !currentDevice->is_connected) {
            logMessage("设备未连接，无法重启");
            return;
        }
        if (QMessageBox::warning(this, "重启雷达", "雷达将会重启，请确认操作", QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes) {
            livox_status st = LivoxLidarRequestReboot(currentDevice->handle, nullptr, this);
            if (st == kLivoxLidarStatusSuccess) logMessage("已发送重启命令，请等待雷达重启...");
            else logMessage(QString("发送重启命令失败: %1").arg(st));
        }
    });

    connect(actionFactoryReset, &QAction::triggered, [this]() {
        if (!currentDevice || !currentDevice->is_connected) {
            logMessage("设备未连接，无法恢复出厂设置");
            return;
        }
        if (QMessageBox::warning(this, "恢复出厂设置", "雷达将会恢复出厂设置，雷达IP将恢复为192.168.1.3，请确认操作", QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes) {
            livox_status st = LivoxLidarRequestReset(currentDevice->handle, nullptr, this);
            if (st == kLivoxLidarStatusSuccess) {
                logMessage("已发送恢复出厂设置命令，请等待雷达重启并恢复默认IP 192.168.1.3...");
                 // 清空当前设备缓存，避免显示旧IP设备
                 {
                     QMutexLocker locker(&deviceMutex);
                     devices.clear();
                 }
                 currentDevice = nullptr;
                 updateDeviceList();
                 statusLabelBar->setText("等待设备重启上线...");
                 // 分步重启SDK：先清理，稍后再初始化，避免竞态
                 QTimer::singleShot(1000, this, [this]() { cleanupLivoxSDK(); });
                 QTimer::singleShot(10000, this, [this]() { setupLivoxSDK(); });
            } else {
                logMessage(QString("发送恢复出厂设置命令失败: %1").arg(st));
            }
        }
    });

    // 视图菜单：显示/隐藏 dock
    viewMenu->addAction(devicesDock->toggleViewAction());
    viewMenu->addAction(paramsDock->toggleViewAction());
    viewMenu->addAction(imuDock->toggleViewAction());
    viewMenu->addAction(logDock->toggleViewAction());

    // 状态栏
    QStatusBar* statusBar = new QStatusBar(this);
    setStatusBar(statusBar);
    statusLabelBar = new QLabel("就绪", statusBar);
    statusLabelBar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    statusBar->addPermanentWidget(statusLabelBar, 1);
    // 在状态栏添加采集进度条
    captureProgress = new QProgressBar(statusBar);
    captureProgress->setRange(0,100);
    captureProgress->setValue(0);
    captureProgress->setFixedWidth(260);
    captureProgress->setTextVisible(true);
    statusBar->addPermanentWidget(captureProgress, 0);

    // 信号槽连接
    // 刷新按钮已移除，无需实现 onRefreshClicked
    connect(deviceList, &QListWidget::currentRowChanged, this, &MainWindow::onDeviceSelected);
    connect(clearLogButton, &QPushButton::clicked, [this]() { logText->clear(); });

    // 标签页切换
    connect(paramTabWidget, &QTabWidget::currentChanged, this, &MainWindow::onTabChanged);

    // 渲染定时器：固定刷新率（例如 30 FPS），与积分时间解耦
    renderTimer = new QTimer(this);
    renderTimer->setTimerType(Qt::PreciseTimer);
    connect(renderTimer, &QTimer::timeout, this, &MainWindow::onRenderTick);
    renderTimer->start(33);
    // 采集定时器
    captureTimer = new QTimer(this);
    connect(captureTimer, &QTimer::timeout, this, &MainWindow::onCaptureTick);
    // GPS 模拟定时器
    gpsTimer = new QTimer(this);
    connect(gpsTimer, &QTimer::timeout, this, &MainWindow::onGpsTick);

    connect(actionSaveIMU, &QAction::triggered, this, &MainWindow::onActionCaptureImuTriggered);
    actionShowImuCharts = toolsMenu->addAction("IMU数据绘图");
    connect(actionShowImuCharts, &QAction::triggered, this, &MainWindow::onActionShowImuCharts);
    
    // 点云滤波
    QAction* actionPointCloudFilter = toolsMenu->addAction("点云滤波...");

    // 点云滤波对话框
    connect(actionPointCloudFilter, &QAction::triggered, [this]() {
        if (!filterDialog) {
            filterDialog = new QDialog(this);
            filterDialog->setWindowTitle("点云滤波");
            filterDialog->setMinimumWidth(500);
            QVBoxLayout* layout = new QVBoxLayout(filterDialog);

            // Tag值滤波设置
            QGroupBox* tagGroup = new QGroupBox("Tag值滤波", filterDialog);
            QVBoxLayout* tagLayout = new QVBoxLayout(tagGroup);
            

            
            auto makeTagRow = [&](const QString& label, int& value, QSpinBox*& spin, QLabel*& desc, const QString& meaning) {
                QWidget* row = new QWidget(filterDialog);
                QHBoxLayout* h = new QHBoxLayout(row);
                h->setContentsMargins(0,0,0,0);
                QLabel* lbl = new QLabel(label + ":", row);
                spin = new QSpinBox(row);
                spin->setRange(0, 3);
                spin->setValue(value);
                spin->setToolTip("0: 置信度优; 1: 置信度中; 2: 置信度差; 3: 保留");
                desc = new QLabel(meaning, row);
                desc->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
                h->addWidget(lbl);
                h->addSpacing(8);
                h->addWidget(spin);
                h->addSpacing(12);
                h->addWidget(desc, 1);
                tagLayout->addWidget(row);
            };

            QLabel* desc76, *desc54, *desc32, *desc10;
            QString meaning76 = "保留位";
            QString meaning54 = "近处回吸噪点";
            QString meaning32 = "雨雾、灰尘等微小颗粒";
            QString meaning10 = "相近物体间的粘连点云";
            
            makeTagRow("Bit[7-6]", filterTagVal76, filterSpin76, desc76, meaning76);
            makeTagRow("Bit[5-4]", filterTagVal54, filterSpin54, desc54, meaning54);
            makeTagRow("Bit[3-2]", filterTagVal32, filterSpin32, desc32, meaning32);
            makeTagRow("Bit[1-0]", filterTagVal10, filterSpin10, desc10, meaning10);
            
            // 设置初始值
            if (filterSpin76) filterSpin76->setValue(filterTagVal76);
            if (filterSpin54) filterSpin54->setValue(filterTagVal54);
            if (filterSpin32) filterSpin32->setValue(filterTagVal32);
            if (filterSpin10) filterSpin10->setValue(filterTagVal10);

            layout->addWidget(tagGroup);



            // 滤噪列表
            QGroupBox* filterListGroup = new QGroupBox("滤噪列表", filterDialog);
            QVBoxLayout* filterListLayout = new QVBoxLayout(filterListGroup);
            
            // 添加Tag值到滤噪列表
            QWidget* addFilterRow = new QWidget(filterDialog);
            QHBoxLayout* addFilterLayout = new QHBoxLayout(addFilterRow);
            addFilterLayout->setContentsMargins(0,0,0,0);
            
            QLabel* addFilterLabel = new QLabel("当前Tag值:", addFilterRow);
            QLabel* currentTagLabel = new QLabel("0", addFilterRow);
            currentTagLabel->setStyleSheet("font-weight: bold; color: green;");
            addNoiseFilterButton = new QPushButton("添加到滤噪列表", addFilterRow);
            addNoiseFilterButton->setEnabled(false);
            
            addFilterLayout->addWidget(addFilterLabel);
            addFilterLayout->addWidget(currentTagLabel);
            addFilterLayout->addSpacing(12);
            addFilterLayout->addWidget(addNoiseFilterButton);
            addFilterLayout->addStretch();
            filterListLayout->addWidget(addFilterRow);
            
            // 滤噪列表显示
            noiseFilterList = new QListWidget(filterDialog);
            noiseFilterList->setMaximumHeight(120);
            filterListLayout->addWidget(noiseFilterList);
            
            // 移除按钮
            QHBoxLayout* removeFilterLayout = new QHBoxLayout();
            removeNoiseFilterButton = new QPushButton("移除选中项", filterDialog);
            removeNoiseFilterButton->setEnabled(false);
            removeFilterLayout->addWidget(removeNoiseFilterButton);
            removeFilterLayout->addStretch();
            filterListLayout->addLayout(removeFilterLayout);
            
            layout->addWidget(filterListGroup);

            // 噪点处理选项（全局设置）
            QGroupBox* noiseGroup = new QGroupBox("噪点处理", filterDialog);
            QVBoxLayout* noiseLayout = new QVBoxLayout(noiseGroup);
            
            showNoiseCheck = new QCheckBox("高亮显示噪点", noiseGroup);
            removeNoiseCheck = new QCheckBox("移除噪点（仅移除显示，并非真正不输出）", noiseGroup);
            
            noiseLayout->addWidget(showNoiseCheck);
            noiseLayout->addWidget(removeNoiseCheck);
            layout->addWidget(noiseGroup);

            // 控制按钮
            QWidget* ctrlRow = new QWidget(filterDialog);
            QHBoxLayout* ctrlLayout = new QHBoxLayout(ctrlRow);
            ctrlLayout->setContentsMargins(0,0,0,0);
            QPushButton* closeBtn = new QPushButton("关闭", ctrlRow);
            ctrlLayout->addStretch();
            ctrlLayout->addWidget(closeBtn);
            layout->addWidget(ctrlRow);

            // 连接信号
            auto updateTagLabel = [this]() {
                if (filterTagLabel && filterTagLabel->isVisible()) {
                    uint8_t tag = makeFilterTag();
                    filterTagLabel->setText(QString::number(tag));
                }
            };

            // 动态更新含义说明
            auto updateMeanings = [this, desc76, desc54, desc32, desc10, meaning76, meaning54, meaning32, meaning10]() {
                auto confToText = [](int v) {
                    switch(v & 3) {
                        case 0: return QString("置信度优");
                        case 1: return QString("置信度中");
                        case 2: return QString("置信度差");
                        default: return QString("保留");
                    }
                };
                
                if (desc76) desc76->setText(QString("%1（%2）").arg(meaning76, confToText(filterTagVal76)));
                if (desc54) desc54->setText(QString("%1（%2）").arg(meaning54, confToText(filterTagVal54)));
                if (desc32) desc32->setText(QString("%1（%2）").arg(meaning32, confToText(filterTagVal32)));
                if (desc10) desc10->setText(QString("%1（%2）").arg(meaning10, confToText(filterTagVal10)));
            };
            
            auto connectFilterSpin = [this, updateMeanings](QSpinBox* spin, const QString& desc) {
                connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), filterDialog, [this, spin, desc, updateMeanings]() {
                    if (desc == "Bit[7-6]") filterTagVal76 = spin->value();
                    else if (desc == "Bit[5-4]") filterTagVal54 = spin->value();
                    else if (desc == "Bit[3-2]") filterTagVal32 = spin->value();
                    else if (desc == "Bit[1-0]") filterTagVal10 = spin->value();
                    
                    // 更新含义说明和标签
                    updateMeanings();
                    
                    if (pointCloudWidget) pointCloudWidget->update();
                });
            };

            connectFilterSpin(filterSpin76, "Bit[7-6]");
            connectFilterSpin(filterSpin54, "Bit[5-4]");
            connectFilterSpin(filterSpin32, "Bit[3-2]");
            connectFilterSpin(filterSpin10, "Bit[1-0]");
            




            connect(showNoiseCheck, &QCheckBox::toggled, filterDialog, [this](bool en) { 
                showNoisePoints = en; 
                if (pointCloudWidget) pointCloudWidget->update(); 
            });
            connect(removeNoiseCheck, &QCheckBox::toggled, filterDialog, [this](bool en) { 
                removeNoisePoints = en; 
                if (pointCloudWidget) pointCloudWidget->update(); 
            });



            // 更新当前Tag值显示
            auto updateCurrentTagDisplay = [this, currentTagLabel]() {
                uint8_t tag = makeFilterTag();
                currentTagLabel->setText(QString::number(tag));
                
                // 检查是否已在列表中
                bool alreadyInList = noiseFilterTags.contains(tag);
                addNoiseFilterButton->setEnabled(!alreadyInList);
                addNoiseFilterButton->setText(alreadyInList ? "已在列表中" : "添加到滤噪列表");
            };
            
            // 连接滤噪列表相关信号
            connect(addNoiseFilterButton, &QPushButton::clicked, filterDialog, [this, currentTagLabel, updateCurrentTagDisplay]() {
                uint8_t tag = makeFilterTag();
                if (!noiseFilterTags.contains(tag)) {
                    noiseFilterTags.append(tag);
                    updateNoiseFilterList();
                    updateCurrentTagDisplay(); // 立即更新按钮状态和文字
                }
            });
            
            connect(removeNoiseFilterButton, &QPushButton::clicked, filterDialog, [this, updateCurrentTagDisplay]() {
                int currentRow = noiseFilterList->currentRow();
                if (currentRow >= 0 && currentRow < noiseFilterTags.size()) {
                    noiseFilterTags.removeAt(currentRow);
                    updateNoiseFilterList();
                    updateCurrentTagDisplay(); // 立即更新按钮状态和文字
                    if (pointCloudWidget) pointCloudWidget->update();
                }
            });
            
            connect(noiseFilterList, &QListWidget::itemSelectionChanged, filterDialog, [this]() {
                removeNoiseFilterButton->setEnabled(noiseFilterList->currentRow() >= 0);
            });
            

            
            // 重新连接spinbox信号，只更新含义说明和当前Tag值显示，不触发点云更新
            auto connectFilterSpinWithTag = [this, updateMeanings, updateCurrentTagDisplay](QSpinBox* spin, const QString& desc) {
                connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), filterDialog, [this, spin, desc, updateMeanings, updateCurrentTagDisplay]() {
                    if (desc == "Bit[7-6]") filterTagVal76 = spin->value();
                    else if (desc == "Bit[5-4]") filterTagVal54 = spin->value();
                    else if (desc == "Bit[3-2]") filterTagVal32 = spin->value();
                    else if (desc == "Bit[1-0]") filterTagVal10 = spin->value();
                    
                    // 只更新含义说明和当前Tag值显示，不触发点云更新
                    updateMeanings();
                    updateCurrentTagDisplay();
                });
            };
            
            // 重新连接所有spinbox
            connectFilterSpinWithTag(filterSpin76, "Bit[7-6]");
            connectFilterSpinWithTag(filterSpin54, "Bit[5-4]");
            connectFilterSpinWithTag(filterSpin32, "Bit[3-2]");
            connectFilterSpinWithTag(filterSpin10, "Bit[1-0]");
            
            connect(closeBtn, &QPushButton::clicked, filterDialog, &QDialog::accept);

            // 设置初始状态
            if (showNoiseCheck) showNoiseCheck->setChecked(showNoisePoints);
            if (removeNoiseCheck) removeNoiseCheck->setChecked(removeNoisePoints);
            
            // 初始化含义说明和当前Tag值显示
            updateMeanings();
            updateCurrentTagDisplay();
            
            // 初始化滤噪列表
            updateNoiseFilterList();
        }

        filterDialog->show();
        filterDialog->raise();
        filterDialog->activateWindow();
    });
}

// 刷新按钮已移除，无需实现 onRefreshClicked

void MainWindow::onDeviceSelected()
{
    int currentRow = deviceList->currentRow();
    if (currentRow >= 0 && currentRow < devices.size()) {
        QMutexLocker locker(&deviceMutex);
        auto it = devices.begin();
        std::advance(it, currentRow);
        currentDevice = &(it.value());
        if (currentDevice->is_connected) {
            if (statusLabel) statusLabel->setText("状态: 已连接");
        } else {
            if (statusLabel) statusLabel->setText("状态: 未连接");
        }
    }
}

void MainWindow::updateDeviceList()
{
    QMutexLocker locker(&deviceMutex);
    deviceList->clear();
    for (const auto& device : devices) {
        addDeviceToList(device);
    }
}

void MainWindow::addDeviceToList(const DeviceInfo& device)
{
    QString deviceText = QString("%1 (%2) - %3").arg(device.sn).arg(device.product_info).arg(device.is_streaming ? "数据流中" : "已连接");
    QListWidgetItem* item = new QListWidgetItem(deviceText);
    item->setData(Qt::UserRole, device.handle);
    deviceList->addItem(item);
}

void MainWindow::updateDeviceInfo(const DeviceInfo& device)
{
    QMutexLocker locker(&deviceMutex);
    devices[device.handle] = device;
    for (int i = 0; i < deviceList->count(); ++i) {
        QListWidgetItem* item = deviceList->item(i);
        if (item->data(Qt::UserRole).toUInt() == device.handle) {
            QString deviceText = QString("%1 (%2) - %3").arg(device.sn).arg(device.product_info).arg(device.is_streaming ? "数据流中" : "已连接");
            item->setText(deviceText);
            break;
        }
    }
}

void MainWindow::updateStatus()
{
    // 不再自动显示"已发现x个设备..."的状态
}

void MainWindow::logMessage(const QString& message)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString logEntry = QString("[%1] %2").arg(timestamp).arg(message);
    logText->append(logEntry);
    qDebug() << logEntry;
}

void MainWindow::onTabChanged(int index)
{
    if (!currentDevice || !currentDevice->is_connected) return;
    updatedConfigKeys.clear();
    livox_status status = QueryLivoxLidarInternalInfo(currentDevice->handle, onQueryInternalInfoResponse, this);
    if (status != kLivoxLidarStatusSuccess) {
        logMessage(QString("标签页切换时查询设备参数失败，错误码: %1").arg(status));
    }
}

bool MainWindow::runConfigGeneratorDialog()
{
	QDialog dlg(this);
	dlg.setWindowTitle("生成配置文件");
	QVBoxLayout* v = new QVBoxLayout(&dlg);

	// 日志配置
	QGroupBox* logGroup = new QGroupBox("日志配置", &dlg);
	QVBoxLayout* logLayout = new QVBoxLayout(logGroup);
	QCheckBox* cbLogEnable = new QCheckBox("启用雷达日志", logGroup);
	cbLogEnable->setChecked(true);
	QWidget* rowCache = new QWidget(logGroup);
	QHBoxLayout* hCache = new QHBoxLayout(rowCache);
	hCache->setContentsMargins(0,0,0,0);
	QLabel* lblCache = new QLabel("缓存大小(MB):", rowCache);
	QSpinBox* spinCache = new QSpinBox(rowCache);
	spinCache->setRange(0, 100000);
	spinCache->setValue(500);
	hCache->addWidget(lblCache);
	hCache->addSpacing(8);
	hCache->addWidget(spinCache);
	QWidget* rowPath = new QWidget(logGroup);
	QHBoxLayout* hPath = new QHBoxLayout(rowPath);
	hPath->setContentsMargins(0,0,0,0);
	QLabel* lblPath = new QLabel("日志路径:", rowPath);
	QLineEdit* editPath = new QLineEdit(rowPath);
	editPath->setText("./");
	editPath->setEnabled(false);
	hPath->addWidget(lblPath);
	hPath->addSpacing(8);
	hPath->addWidget(editPath, 1);
	logLayout->addWidget(cbLogEnable);
	logLayout->addWidget(rowCache);
	logLayout->addWidget(rowPath);
	v->addWidget(logGroup);

	// 多设备配置
	struct DeviceRow {
		QWidget* root;
		QComboBox* devType;
		QComboBox* hostIp;
		QLineEdit* mcIp;
		QSpinBox* hp1;
		QSpinBox* hp2;
		QSpinBox* hp3;
		QSpinBox* hp4;
		QSpinBox* hp5;
		QPushButton* btnRemove;
	};
	QVector<DeviceRow*> deviceRows;

	auto populateHostIpsTo = [&](QComboBox* combo) {
		combo->clear();
		for (const QNetworkInterface &iface : QNetworkInterface::allInterfaces()) {
			if (!(iface.flags() & QNetworkInterface::IsUp) || !(iface.flags() & QNetworkInterface::IsRunning)) continue;
			if (iface.flags() & QNetworkInterface::IsLoopBack) continue;
			for (const QNetworkAddressEntry &entry : iface.addressEntries()) {
				const QHostAddress &addr = entry.ip();
				if (addr.protocol() != QAbstractSocket::IPv4Protocol) continue;
				const QString ip = addr.toString();
				if (ip == "0.0.0.0" || ip.startsWith("169.254.")) continue;
				const QString label = QString("%1  -  %2 (%3)").arg(ip, iface.humanReadableName(), iface.name());
				combo->addItem(label, ip);
			}
		}
	};

	auto createLidarNetDefaults = [&](const QString& type, QJsonObject& out) {
        if (type == "MID360" || type == "Mid360s") {
			out.insert("cmd_data_port", 56100);
			out.insert("push_msg_port", 56200);
			out.insert("point_data_port", 56300);
			out.insert("imu_data_port", 56400);
			out.insert("log_data_port", 56500);
		} else { // HAP
			out.insert("cmd_data_port", 56000);
			out.insert("push_msg_port", 0);
			out.insert("point_data_port", 57000);
			out.insert("imu_data_port", 58000);
			out.insert("log_data_port", 59000);
		}
	};

	auto applyHostPortDefaults = [&](const QString& type, DeviceRow* row) {
        if (type == "MID360" || type == "Mid360s") {
			row->hp1->setValue(56101);
			row->hp2->setValue(56201);
			row->hp3->setValue(56301);
			row->hp4->setValue(56401);
			row->hp5->setValue(56501);
		} else { // HAP
			row->hp1->setValue(56000);
			row->hp2->setValue(0);
			row->hp3->setValue(57000);
			row->hp4->setValue(58000);
			row->hp5->setValue(59000);
		}
	};

	QGroupBox* devicesGroup = new QGroupBox("设备列表", &dlg);
	QVBoxLayout* devicesLayout = new QVBoxLayout(devicesGroup);
	QWidget* toolbar = new QWidget(devicesGroup);
	QHBoxLayout* hToolbar = new QHBoxLayout(toolbar);
	hToolbar->setContentsMargins(0,0,0,0);
	QPushButton* btnAddDevice = new QPushButton("添加设备", toolbar);
	hToolbar->addWidget(btnAddDevice);
	hToolbar->addStretch();
	devicesLayout->addWidget(toolbar);

	QWidget* rowsContainer = new QWidget(devicesGroup);
	QVBoxLayout* rowsLayout = new QVBoxLayout(rowsContainer);
	rowsLayout->setContentsMargins(0,0,0,0);
	rowsLayout->setSpacing(6);
	QScrollArea* rowsScroll = new QScrollArea(devicesGroup);
	rowsScroll->setWidgetResizable(true);
	rowsScroll->setFrameShape(QFrame::NoFrame);
	rowsScroll->setWidget(rowsContainer);
	rowsScroll->setMinimumHeight(260);
	rowsScroll->setMinimumWidth(660);
	devicesLayout->addWidget(rowsScroll);
	v->addWidget(devicesGroup);

	auto addDeviceRow = [&]() {
		DeviceRow* r = new DeviceRow();
		r->root = new QWidget(rowsContainer);
		QGridLayout* grid = new QGridLayout(r->root);
		grid->setContentsMargins(0,0,0,0);

		r->devType = new QComboBox(r->root);
        r->devType->addItems({"MID360", "Mid360s", "HAP"});
        r->hostIp = new QComboBox(r->root);
		populateHostIpsTo(r->hostIp);
		r->mcIp = new QLineEdit(r->root);
		r->hp1 = new QSpinBox(r->root);
		r->hp2 = new QSpinBox(r->root);
		r->hp3 = new QSpinBox(r->root);
		r->hp4 = new QSpinBox(r->root);
		r->hp5 = new QSpinBox(r->root);
		for (QSpinBox* s : {r->hp1,r->hp2,r->hp3,r->hp4,r->hp5}) { s->setRange(0, 65535); }
		r->btnRemove = new QPushButton("删除", r->root);

		int row = 0;
		grid->addWidget(new QLabel("设备类型:"), row, 0); grid->addWidget(r->devType, row, 1);
		grid->addWidget(new QLabel("host_ip:"), row, 2); grid->addWidget(r->hostIp, row, 3);
		grid->addWidget(r->btnRemove, row, 4);
		row++;
		grid->addWidget(new QLabel("multicast_ip(可选):"), row, 0); grid->addWidget(r->mcIp, row, 1, 1, 4);
		row++;
		grid->addWidget(new QLabel("cmd_data_port:"), row, 0); grid->addWidget(r->hp1, row, 1);
		grid->addWidget(new QLabel("push_msg_port:"), row, 2); grid->addWidget(r->hp2, row, 3);
		row++;
		grid->addWidget(new QLabel("point_data_port:"), row, 0); grid->addWidget(r->hp3, row, 1);
		grid->addWidget(new QLabel("imu_data_port:"), row, 2); grid->addWidget(r->hp4, row, 3);
		row++;
		grid->addWidget(new QLabel("log_data_port:"), row, 0); grid->addWidget(r->hp5, row, 1);

		applyHostPortDefaults(r->devType->currentText(), r);
		QObject::connect(r->devType, QOverload<int>::of(&QComboBox::currentIndexChanged), &dlg, [&, r](int){ applyHostPortDefaults(r->devType->currentText(), r); });
		QObject::connect(r->btnRemove, &QPushButton::clicked, &dlg, [&, r]() {
			int idx = deviceRows.indexOf(r);
			if (idx >= 0) deviceRows.removeAt(idx);
			r->root->deleteLater();
			delete r;
		});

		rowsLayout->addWidget(r->root);
		deviceRows.append(r);
	};
	QObject::connect(btnAddDevice, &QPushButton::clicked, &dlg, addDeviceRow);
	// 默认添加一行
	addDeviceRow();

	QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
	v->addWidget(box);
	connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
	connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

	if (dlg.exec() != QDialog::Accepted) return false;

	// 校验必要字段
	if (deviceRows.isEmpty()) {
		QMessageBox::warning(this, "生成配置文件", "请至少添加一台设备");
		return false;
	}
	for (DeviceRow* r : deviceRows) {
		if (!r || r->hostIp->count() == 0) {
			QMessageBox::warning(this, "生成配置文件", "未检测到主机网口IPv4地址，请检查网络连接");
			return false;
		}
	}

	// 组装 JSON
	QJsonObject root;
	root.insert("lidar_log_enable", cbLogEnable->isChecked());
	root.insert("lidar_log_cache_size_MB", spinCache->value());
	root.insert("lidar_log_path", "./");

    QJsonArray hostArrMid;
    QJsonArray hostArrMid360s;
    QJsonArray hostArrHap;
    for (DeviceRow* r : deviceRows) {
        QString type = r->devType->currentText();
        QJsonObject hostObj;
        hostObj.insert("host_ip", r->hostIp->currentData().toString());
        if (!r->mcIp->text().trimmed().isEmpty()) hostObj.insert("multicast_ip", r->mcIp->text().trimmed());
        hostObj.insert("cmd_data_port", r->hp1->value());
        hostObj.insert("point_data_port", r->hp3->value());
        hostObj.insert("imu_data_port", r->hp4->value());
        hostObj.insert("push_msg_port", r->hp2->value());
        hostObj.insert("log_data_port", r->hp5->value());

        if (type == "MID360") hostArrMid.append(hostObj);
        else if (type == "Mid360s") hostArrMid360s.append(hostObj);
        else hostArrHap.append(hostObj);
    }

    if (!hostArrMid.isEmpty()) {
        QJsonObject devObj;
        QJsonObject lidarNet; createLidarNetDefaults("MID360", lidarNet);
        devObj.insert("lidar_net_info", lidarNet);
        devObj.insert("host_net_info", hostArrMid);
        root.insert("MID360", devObj);
    }
    if (!hostArrMid360s.isEmpty()) {
        QJsonObject devObj;
        QJsonObject lidarNet; createLidarNetDefaults("Mid360s", lidarNet);
        devObj.insert("lidar_net_info", lidarNet);
        devObj.insert("host_net_info", hostArrMid360s);
        root.insert("Mid360s", devObj);
    }
    if (!hostArrHap.isEmpty()) {
        QJsonObject devObj;
        QJsonObject lidarNet; createLidarNetDefaults("HAP", lidarNet);
        devObj.insert("lidar_net_info", lidarNet);
        devObj.insert("host_net_info", hostArrHap);
        root.insert("HAP", devObj);
    }


    QString outPath = QApplication::applicationDirPath() + "/config.json";
    QFile f(outPath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        QMessageBox::critical(this, "生成配置文件", QString("无法写入: %1").arg(QDir::toNativeSeparators(outPath)));
        return false;
    }

    // 手动控制字段顺序
    QJsonObject orderedRoot;

    // 1) 日志配置（固定在最前面）
    orderedRoot.insert("lidar_log_enable", cbLogEnable->isChecked());
    orderedRoot.insert("lidar_log_cache_size_MB", spinCache->value());
    orderedRoot.insert("lidar_log_path", "./");

    // 2) 设备配置（保持 lidar_net_info 在 host_net_info 前面）
    auto makeDeviceObject = [&](const QString& type, const QJsonArray& hostArr) {
        QJsonObject devObj;
        QJsonObject lidarNet;
        createLidarNetDefaults(type, lidarNet);
        devObj.insert("lidar_net_info", lidarNet);
        devObj.insert("host_net_info", hostArr);
        return devObj;
    };

    if (!hostArrMid360s.isEmpty()) {
        orderedRoot.insert("Mid360s", makeDeviceObject("Mid360s", hostArrMid360s));
    }
    if (!hostArrMid.isEmpty()) {
        orderedRoot.insert("MID360", makeDeviceObject("MID360", hostArrMid));
    }
    if (!hostArrHap.isEmpty()) {
        orderedRoot.insert("HAP", makeDeviceObject("HAP", hostArrHap));
    }

    // 输出 JSON
    QJsonDocument doc(orderedRoot);
    f.write(doc.toJson(QJsonDocument::Indented));
    f.close();

    QMessageBox::information(this, "生成配置文件", QString("已生成: %1").arg(QDir::toNativeSeparators(outPath)));
    return true;

}

void MainWindow::updateNoiseFilterList()
{
    if (!noiseFilterList) return;
    
    noiseFilterList->clear();
    for (uint8_t tag : noiseFilterTags) {
        QString itemText = QString("Tag值: %1").arg(tag);
        QListWidgetItem* item = new QListWidgetItem(itemText);
        item->setData(Qt::UserRole, tag);
        noiseFilterList->addItem(item);
    }
    
    // 更新移除按钮状态
    if (removeNoiseFilterButton) {
        removeNoiseFilterButton->setEnabled(!noiseFilterTags.isEmpty());
    }
}

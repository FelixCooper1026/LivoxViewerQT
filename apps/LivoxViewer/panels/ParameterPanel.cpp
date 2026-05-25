#include "LivoxViewerWindow.h"
#include <QHeaderView>
#include <QSizePolicy>
#include <QTableView>

void LivoxViewerWindow::createParameterPanel()
{
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
    attrTable = new QTableView(attrContent);
    attrTable->verticalHeader()->setVisible(false);
    attrTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    attrTable->setSelectionMode(QAbstractItemView::SingleSelection);
    attrTable->setSelectionBehavior(QAbstractItemView::SelectRows);
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
    for (uint16_t key : configurableKeysVec) this->parameterState.configurableKeys.insert(key);
    for (uint16_t key : statusKeysVec) this->parameterState.statusKeys.insert(key);

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
    parameterState.controls[kKeyWorkMode] = workModeCombo;
    connect(workModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeyWorkMode); });

    QComboBox* patternModeCombo = new QComboBox();
    patternModeCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    patternModeCombo->addItems({"非重复扫描", "重复扫描", "低帧率重复扫描"});
    patternModeCombo->setCurrentIndex(0);
    basicLayout->addRow("扫描模式:", patternModeCombo);
    parameterState.controls[kKeyPatternMode] = patternModeCombo;
    connect(patternModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeyPatternMode); });

    QComboBox* pclDataTypeCombo = new QComboBox();
    pclDataTypeCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    pclDataTypeCombo->addItems({"高精度笛卡尔坐标", "低精度笛卡尔坐标", "球坐标"});
    pclDataTypeCombo->setCurrentIndex(0);
    basicLayout->addRow("点云格式:", pclDataTypeCombo);
    parameterState.controls[kKeyPclDataType] = pclDataTypeCombo;
    connect(pclDataTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeyPclDataType); });

    QComboBox* detectModeCombo = new QComboBox();
    detectModeCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    detectModeCombo->addItems({"正常模式", "敏感模式"});
    detectModeCombo->setCurrentIndex(0);
    basicLayout->addRow("探测模式:", detectModeCombo);
    parameterState.controls[kKeyDetectMode] = detectModeCombo;
    connect(detectModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeyDetectMode); });

    QComboBox* imuDataCombo = new QComboBox();
    imuDataCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    imuDataCombo->addItems({"关闭", "开启"});
    imuDataCombo->setCurrentIndex(0);
    basicLayout->addRow("IMU数据发送:", imuDataCombo);
    parameterState.controls[kKeyImuDataEn] = imuDataCombo;
    connect(imuDataCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeyImuDataEn); });

    QComboBox* motorSpeedCombo = new QComboBox();
    motorSpeedCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    motorSpeedCombo->addItems({"正常转速","低转速"});
    motorSpeedCombo->setCurrentIndex(0);
    basicLayout->addRow("电机转速:", motorSpeedCombo);
    parameterState.controls[kKeySetEscMode] = motorSpeedCombo;
    connect(motorSpeedCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeySetEscMode); });

    QComboBox* syncFilterModeCombo = new QComboBox();
    syncFilterModeCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    syncFilterModeCombo->addItems({"关闭", "开启"});
    syncFilterModeCombo->setCurrentIndex(0);
    basicLayout->addRow("异常时间过滤:", syncFilterModeCombo);
    parameterState.controls[kKeySetPpsSyncMode] = syncFilterModeCombo;
    connect(syncFilterModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeySetPpsSyncMode); });

    QComboBox* fovModeCombo = new QComboBox();
    fovModeCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    fovModeCombo->addItems({"Focus FOV", "Normal FOV"});
    fovModeCombo->setCurrentIndex(1);
    basicLayout->addRow("FOV模式:", fovModeCombo);
    parameterState.controls[kKeySetFovMode] = fovModeCombo;
    connect(fovModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeySetFovMode); });

    QComboBox* echoModeCombo = new QComboBox();
    echoModeCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    echoModeCombo->addItems({"最强回波", "第一回波"});
    echoModeCombo->setCurrentIndex(0);
    basicLayout->addRow("回波模式:", echoModeCombo);
    parameterState.controls[kKeySetEchoMode] = echoModeCombo;
    connect(echoModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this]() { onParamConfigChanged(kKeySetEchoMode); });

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
    parameterState.controls[kKeyLidarIpCfg] = lidarIpContainer;
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
    parameterState.controls[kKeyLidarPointDataHostIpCfg] = pointDataContainer;
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
    parameterState.controls[kKeyLidarImuHostIpCfg] = imuDataContainer;
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
    parameterState.controls[kKeyStateInfoHostIpCfg] = stateInfoContainer;
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
    parameterState.controls[kKeyFovCfgEn] = fov0EnableCheck;
    QCheckBox* fov1EnableCheck = new QCheckBox();
    parameterState.controls[0x001F] = fov1EnableCheck;
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
    parameterState.controls[kKeyFovCfg0] = fov0Container;
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
    parameterState.controls[kKeyFovCfg1] = fov1Container;
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
    parameterState.controls[kKeyInstallAttitude] = attitudeTab;
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
            valueLabel->setStyleSheet("QLabel { background-color: palette(base); padding: 2px; border: 1px solid palette(mid); }");
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
        parameterState.labels[key] = valueLabel;
    }

    // 添加记录参数按钮
    parameterState.recordButton = new QPushButton("记录参数至CSV文件", statusTab);
    parameterState.recordButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    parameterState.recordButton->setStyleSheet("QPushButton { padding: 5px; }");
    statusLayout->addRow(parameterState.recordButton); // 这会创建一个占据整行的按钮

    // 连接按钮信号
    connect(parameterState.recordButton, &QPushButton::clicked, this, &LivoxViewerWindow::onRecordParamsClicked);

    paramTabWidget->insertTab(0, statusTab, "状态信息");
    paramTabWidget->setCurrentIndex(0);

    paramsOuterLayout->addWidget(paramTabWidget);
    paramsDockContent->setLayout(paramsOuterLayout);

    QScrollArea* paramsScroll = new QScrollArea(paramsDock);
    paramsScroll->setWidgetResizable(true);
    paramsScroll->setWidget(paramsDockContent);
    paramsScroll->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    paramsDock->setWidget(paramsScroll);
    paramsDock->setMinimumWidth(0);

    addDockWidget(Qt::RightDockWidgetArea, paramsDock);
    tabifyDockWidget(paramsDock, attrDock);
    paramsDock->raise();
    attrDock->hide();
}

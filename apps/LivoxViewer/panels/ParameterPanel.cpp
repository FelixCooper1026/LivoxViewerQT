#include "LivoxViewerWindow.h"
#include "AppConfig/NetworkInterfaceService.h"
#include "widgets/SwitchCheckBox.h"
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
        kKeySetNTPServerIp,
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
    QVBoxLayout* networkLayout = new QVBoxLayout(networkTab);
    networkLayout->setSpacing(10);
    networkLayout->setContentsMargins(10, 10, 10, 10);

    auto configureIpEdit = [](QLineEdit* edit) {
        edit->setMinimumWidth(0);
        edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    };
    auto configureApplyButton = [this](QPushButton* button, const QString& tooltip) {
        button->setText("应用");
        button->setToolTip(tooltip);
        button->setMinimumWidth(fontMetrics().horizontalAdvance("应用") + 28);
        button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    };

    QGroupBox* lidarIpContainer = new QGroupBox("雷达设备网络", networkTab);
    lidarIpContainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QGridLayout* lidarGrid = new QGridLayout(lidarIpContainer);
    lidarGrid->setContentsMargins(10, 14, 10, 10);
    lidarGrid->setHorizontalSpacing(8);
    lidarGrid->setVerticalSpacing(7);
    lidarGrid->setColumnStretch(1, 1);

    QLineEdit* lidarIpEdit = new QLineEdit(lidarIpContainer);
    QLineEdit* lidarMaskEdit = new QLineEdit(lidarIpContainer);
    QLineEdit* lidarGatewayEdit = new QLineEdit(lidarIpContainer);
    lidarIpEdit->setObjectName("deviceIpEdit");
    lidarMaskEdit->setObjectName("deviceMaskEdit");
    lidarGatewayEdit->setObjectName("deviceGatewayEdit");
    configureIpEdit(lidarIpEdit);
    configureIpEdit(lidarMaskEdit);
    configureIpEdit(lidarGatewayEdit);
    QPushButton* lidarIpButton = new QPushButton(lidarIpContainer);
    configureApplyButton(lidarIpButton, "应用雷达设备自身 IP、子网掩码和网关");

    lidarGrid->addWidget(new QLabel("设备 IP", lidarIpContainer), 0, 0);
    lidarGrid->addWidget(lidarIpEdit, 0, 1);
    lidarGrid->addWidget(new QLabel("子网掩码", lidarIpContainer), 1, 0);
    lidarGrid->addWidget(lidarMaskEdit, 1, 1);
    lidarGrid->addWidget(new QLabel("网关", lidarIpContainer), 2, 0);
    lidarGrid->addWidget(lidarGatewayEdit, 2, 1);
    lidarGrid->addWidget(lidarIpButton, 3, 1, Qt::AlignRight);
    networkLayout->addWidget(lidarIpContainer);
    parameterState.controls[kKeyLidarIpCfg] = lidarIpContainer;
    connect(lidarIpButton, &QPushButton::clicked, [this, lidarIpEdit, lidarMaskEdit, lidarGatewayEdit]() { applyIpConfig(kKeyLidarIpCfg, lidarIpEdit->text(), lidarMaskEdit->text(), lidarGatewayEdit->text()); });

    QGroupBox* targetGroup = new QGroupBox(networkTab);
    targetGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QGridLayout* targetLayout = new QGridLayout(targetGroup);
    targetLayout->setContentsMargins(10, 10, 10, 10);
    targetLayout->setHorizontalSpacing(6);
    targetLayout->setVerticalSpacing(6);
    targetLayout->setColumnStretch(0, 1);

    QVector<QLineEdit*> targetIpEdits;
    QWidget* targetHeader = new QWidget(targetGroup);
    QHBoxLayout* targetHeaderLayout = new QHBoxLayout(targetHeader);
    targetHeaderLayout->setContentsMargins(0, 0, 0, 0);
    targetHeaderLayout->setSpacing(8);
    QLabel* targetTitle = new QLabel("数据发送目标", targetHeader);
    QFont targetTitleFont = targetTitle->font();
    targetTitleFont.setBold(true);
    targetTitle->setFont(targetTitleFont);
    QPushButton* syncTargetIpButton = new QPushButton("同步目标 IP", targetHeader);
    syncTargetIpButton->setToolTip("将点云数据、IMU数据和推送信息的目标 IP 同步为当前选择网卡的主机 IP");
    syncTargetIpButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    targetHeaderLayout->addWidget(targetTitle);
    targetHeaderLayout->addStretch();
    targetHeaderLayout->addWidget(syncTargetIpButton);
    targetLayout->addWidget(targetHeader, 0, 0, 1, 4);

    int targetRowIndex = 1;
    auto createTargetRow = [&](const QString& title, uint16_t key, int defaultPort, const QString& tooltip) {
        QLabel* titleLabel = new QLabel(title, targetGroup);
        QFont titleFont = titleLabel->font();
        titleFont.setBold(true);
        titleLabel->setFont(titleFont);
        titleLabel->setToolTip(tooltip);
        QLineEdit* ipEdit = new QLineEdit(targetGroup);
        ipEdit->setObjectName(QString("targetIpEdit_%1").arg(key));
        ipEdit->setToolTip(tooltip);
        configureIpEdit(ipEdit);
        targetIpEdits.append(ipEdit);
        QSpinBox* portEdit = new QSpinBox(targetGroup);
        portEdit->setObjectName(QString("targetPortSpin_%1").arg(key));
        portEdit->setToolTip(tooltip);
        portEdit->setRange(1, 65535);
        portEdit->setValue(defaultPort);
        portEdit->setButtonSymbols(QAbstractSpinBox::NoButtons);
        portEdit->setMinimumWidth(fontMetrics().horizontalAdvance("65535") + 24);
        portEdit->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        QLabel* separator = new QLabel(":", targetGroup);
        separator->setAlignment(Qt::AlignCenter);
        QPushButton* applyButton = new QPushButton(targetGroup);
        configureApplyButton(applyButton, tooltip);

        targetLayout->addWidget(titleLabel, targetRowIndex, 0, 1, 4);
        ++targetRowIndex;
        targetLayout->addWidget(ipEdit, targetRowIndex, 0);
        targetLayout->addWidget(separator, targetRowIndex, 1);
        targetLayout->addWidget(portEdit, targetRowIndex, 2);
        targetLayout->addWidget(applyButton, targetRowIndex, 3);
        ++targetRowIndex;
        parameterState.controls[key] = targetGroup;
        connect(applyButton, &QPushButton::clicked, [this, key, ipEdit, portEdit]() { applyHostIpConfig(key, ipEdit->text(), portEdit->value()); });
    };

    createTargetRow("点云数据", kKeyLidarPointDataHostIpCfg, 57000, "应用点云数据发送目标 IP 和端口");
    createTargetRow("IMU数据", kKeyLidarImuHostIpCfg, 57000, "应用 IMU 数据发送目标 IP 和端口");
    createTargetRow("推送信息", kKeyStateInfoHostIpCfg, 57000, "应用状态/推送信息发送目标 IP 和端口");
    connect(syncTargetIpButton, &QPushButton::clicked, this, [this, targetIpEdits]() {
        const QString selectedName = networkInterfaceCombo
            ? networkInterfaceCombo->currentData(Qt::UserRole).toString()
            : QString();
        QString hostIp;
        const auto selectedInterface = NetworkInterfaceService::findInterfaceByName(selectedName);
        if (selectedInterface.has_value()) {
            hostIp = selectedInterface->ipv4;
        } else {
            hostIp = NetworkInterfaceService::currentHostIp();
        }
        if (hostIp.isEmpty()) {
            statusLabelBar->setText("未找到可用主机 IP");
            logMessage("同步目标 IP 失败：未找到可用主机 IP");
            return;
        }
        for (QLineEdit* edit : targetIpEdits) {
            edit->setText(hostIp);
        }
        statusLabelBar->setText(QString("目标 IP 已同步为 %1").arg(hostIp));
        logMessage(QString("数据发送目标 IP 已同步为当前主机 IP: %1").arg(hostIp));
    });
    networkLayout->addWidget(targetGroup);

    QGroupBox* ntpGroup = new QGroupBox("NTP 服务器", networkTab);
    ntpGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QGridLayout* ntpLayout = new QGridLayout(ntpGroup);
    ntpLayout->setContentsMargins(10, 14, 10, 10);
    ntpLayout->setHorizontalSpacing(8);
    ntpLayout->setVerticalSpacing(7);
    ntpLayout->setColumnStretch(1, 1);

    QLineEdit* ntpIpEdit = new QLineEdit(ntpGroup);
    ntpIpEdit->setObjectName("ntpServerIpEdit");
    ntpIpEdit->setToolTip("应用 NTP 服务器 IP");
    configureIpEdit(ntpIpEdit);
    QPushButton* ntpApplyButton = new QPushButton(ntpGroup);
    configureApplyButton(ntpApplyButton, "应用 NTP 服务器 IP");

    ntpLayout->addWidget(new QLabel("服务器 IP", ntpGroup), 0, 0);
    ntpLayout->addWidget(ntpIpEdit, 0, 1);
    ntpLayout->addWidget(ntpApplyButton, 0, 2);
    networkLayout->addWidget(ntpGroup);
    parameterState.controls[kKeySetNTPServerIp] = ntpGroup;
    connect(ntpApplyButton, &QPushButton::clicked, [this, ntpIpEdit]() { applyNtpServerIpConfig(ntpIpEdit->text()); });

    networkLayout->addStretch();

    paramTabWidget->addTab(networkTab, "网络配置");
    networkTab->setLayout(networkLayout);

    // FOV 配置页
    QWidget* fovTab = new QWidget();
    fovTab->setMinimumWidth(0);
    fovTab->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    QVBoxLayout* fovLayout = new QVBoxLayout(fovTab);
    fovLayout->setSpacing(10);
    fovLayout->setContentsMargins(10, 10, 10, 10);
    QCheckBox* fov0EnableCheck = new SwitchCheckBox(fovTab);
    parameterState.controls[kKeyFovCfgEn] = fov0EnableCheck;
    QCheckBox* fov1EnableCheck = new SwitchCheckBox(fovTab);
    parameterState.controls[0x001F] = fov1EnableCheck;
    connect(fov0EnableCheck, &QCheckBox::toggled, [this, fov0EnableCheck, fov1EnableCheck]() { updateFovEnableState(fov0EnableCheck, fov1EnableCheck); });
    connect(fov1EnableCheck, &QCheckBox::toggled, [this, fov0EnableCheck, fov1EnableCheck]() { updateFovEnableState(fov0EnableCheck, fov1EnableCheck); });

    auto configureFovSpin = [](QSpinBox* spin, int minimum, int maximum) {
        spin->setRange(minimum, maximum);
        spin->setMinimumWidth(72);
        spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    };
    auto configureFovApplyButton = [this](QPushButton* button) {
        button->setText("应用");
        button->setMinimumWidth(fontMetrics().horizontalAdvance("应用") + 34);
        button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    };
    auto createFovSection = [this, fovTab](const QString& title,
                                           QCheckBox* enableSwitch,
                                           QWidget* controls,
                                           QSpinBox* yawStart,
                                           QSpinBox* yawStop,
                                           QSpinBox* pitchStart,
                                           QSpinBox* pitchStop,
                                           QPushButton* applyButton) {
        QFrame* section = new QFrame(fovTab);
        section->setObjectName("FovConfigSection");
        section->setFrameShape(QFrame::StyledPanel);
        section->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        section->setStyleSheet("QFrame#FovConfigSection { border: 1px solid palette(mid); border-radius: 6px; background: palette(base); }");

        QVBoxLayout* sectionLayout = new QVBoxLayout(section);
        sectionLayout->setContentsMargins(10, 8, 10, 10);
        sectionLayout->setSpacing(8);

        QWidget* header = new QWidget(section);
        QHBoxLayout* headerLayout = new QHBoxLayout(header);
        headerLayout->setContentsMargins(0, 0, 0, 0);
        QLabel* titleLabel = new QLabel(title, header);
        QFont titleFont = titleLabel->font();
        titleFont.setBold(true);
        titleLabel->setFont(titleFont);
        headerLayout->addWidget(titleLabel);
        headerLayout->addStretch();
        headerLayout->addWidget(enableSwitch);
        sectionLayout->addWidget(header);

        controls->setParent(section);
        controls->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        QGridLayout* grid = new QGridLayout(controls);
        grid->setContentsMargins(0, 0, 0, 0);
        grid->setHorizontalSpacing(8);
        grid->setVerticalSpacing(6);
        grid->addWidget(new QLabel("Yaw:", controls), 0, 0);
        grid->addWidget(yawStart, 0, 1);
        grid->addWidget(new QLabel("~", controls), 0, 2, Qt::AlignCenter);
        grid->addWidget(yawStop, 0, 3);
        grid->addWidget(new QLabel("Pitch:", controls), 1, 0);
        grid->addWidget(pitchStart, 1, 1);
        grid->addWidget(new QLabel("~", controls), 1, 2, Qt::AlignCenter);
        grid->addWidget(pitchStop, 1, 3);
        grid->addWidget(applyButton, 0, 4, 2, 1, Qt::AlignVCenter);
        grid->setColumnStretch(1, 1);
        grid->setColumnStretch(3, 1);
        sectionLayout->addWidget(controls);
        return section;
    };

    QSpinBox* fov0YawStartEdit = new QSpinBox();
    QSpinBox* fov0YawStopEdit = new QSpinBox();
    QSpinBox* fov0PitchStartEdit = new QSpinBox();
    QSpinBox* fov0PitchStopEdit = new QSpinBox();
    QPushButton* fov0Button = new QPushButton();
    configureFovSpin(fov0YawStartEdit, 0, 360);
    configureFovSpin(fov0YawStopEdit, 0, 360);
    configureFovSpin(fov0PitchStartEdit, -10, 60);
    configureFovSpin(fov0PitchStopEdit, -10, 60);
    configureFovApplyButton(fov0Button);
    QWidget* fov0Container = new QWidget();
    fovLayout->addWidget(createFovSection("FOV0配置", fov0EnableCheck, fov0Container, fov0YawStartEdit, fov0YawStopEdit, fov0PitchStartEdit, fov0PitchStopEdit, fov0Button));
    parameterState.controls[kKeyFovCfg0] = fov0Container;
    connect(fov0Button, &QPushButton::clicked, [this, fov0YawStartEdit, fov0YawStopEdit, fov0PitchStartEdit, fov0PitchStopEdit]() { applyFovConfig(kKeyFovCfg0, fov0YawStartEdit->value(), fov0YawStopEdit->value(), fov0PitchStartEdit->value(), fov0PitchStopEdit->value()); });

    QSpinBox* fov1YawStartEdit = new QSpinBox();
    QSpinBox* fov1YawStopEdit = new QSpinBox();
    QSpinBox* fov1PitchStartEdit = new QSpinBox();
    QSpinBox* fov1PitchStopEdit = new QSpinBox();
    QPushButton* fov1Button = new QPushButton();
    configureFovSpin(fov1YawStartEdit, 0, 360);
    configureFovSpin(fov1YawStopEdit, 0, 360);
    configureFovSpin(fov1PitchStartEdit, -10, 60);
    configureFovSpin(fov1PitchStopEdit, -10, 60);
    configureFovApplyButton(fov1Button);
    QWidget* fov1Container = new QWidget();
    fovLayout->addWidget(createFovSection("FOV1配置", fov1EnableCheck, fov1Container, fov1YawStartEdit, fov1YawStopEdit, fov1PitchStartEdit, fov1PitchStopEdit, fov1Button));
    parameterState.controls[kKeyFovCfg1] = fov1Container;
    connect(fov1Button, &QPushButton::clicked, [this, fov1YawStartEdit, fov1YawStopEdit, fov1PitchStartEdit, fov1PitchStopEdit]() { applyFovConfig(kKeyFovCfg1, fov1YawStartEdit->value(), fov1YawStopEdit->value(), fov1PitchStartEdit->value(), fov1PitchStopEdit->value()); });

    fovLayout->addStretch();
    paramTabWidget->addTab(fovTab, "FOV配置");

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
        QLabel* valueLabel = new QLabel("无信息");
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

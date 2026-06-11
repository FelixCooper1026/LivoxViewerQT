#include "LivoxViewerWindow.h"
#include "AppConfig/NetworkInterfaceService.h"
#include "widgets/ParameterOptionButtons.h"
#include "widgets/SwitchCheckBox.h"
#include <QButtonGroup>
#include <QHeaderView>
#include <QSizePolicy>
#include <QStringList>
#include <QTableView>
#include <QToolButton>

namespace {

QWidget* createParameterOptionButtons(const QStringList& options, int currentIndex, QWidget* parent)
{
    QWidget* container = new QWidget(parent);
    container->setProperty(ParameterOptionButtons::kControlProperty, true);
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QGridLayout* layout = new QGridLayout(container);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setHorizontalSpacing(4);
    layout->setVerticalSpacing(4);

    QButtonGroup* group = new QButtonGroup(container);
    group->setObjectName(ParameterOptionButtons::groupObjectName());
    group->setExclusive(true);

    const int columnCount = options.size() > 2 ? 2 : options.size();
    for (int i = 0; i < options.size(); ++i) {
        QToolButton* button = new QToolButton(container);
        button->setText(options.at(i));
        button->setToolButtonStyle(Qt::ToolButtonTextOnly);
        button->setCheckable(true);
        button->setCursor(Qt::PointingHandCursor);
        button->setMinimumHeight(30);
        button->setMinimumWidth(button->fontMetrics().horizontalAdvance(options.at(i)) + 22);
        button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        button->setStyleSheet(
            "QToolButton {"
            "  border: 1px solid palette(mid);"
            "  border-radius: 15px;"
            "  padding: 4px 10px;"
            "  background: palette(button);"
            "  color: palette(button-text);"
            "}"
            "QToolButton:hover {"
            "  background: palette(button);"
            "  border-color: palette(dark);"
            "  color: palette(button-text);"
            "}"
            "QToolButton:pressed {"
            "  background: #2f2f2f;"
            "  border-color: #2f2f2f;"
            "  color: #ffffff;"
            "}"
            "QToolButton:checked {"
            "  background: #4a4a4a;"
            "  border-color: #4a4a4a;"
            "  color: #ffffff;"
            "}"
            "QToolButton:checked:hover {"
            "  background: #4a4a4a;"
            "  border-color: #555555;"
            "  color: #ffffff;"
            "}"
            "QToolButton:checked:pressed {"
            "  background: #2f2f2f;"
            "  border-color: #2f2f2f;"
            "  color: #ffffff;"
            "}"
        );

        group->addButton(button, i);
        layout->addWidget(button, i / columnCount, i % columnCount);
    }

    for (int column = 0; column < columnCount; ++column) {
        layout->setColumnStretch(column, 0);
    }
    ParameterOptionButtons::setCurrentIndex(container, currentIndex);
    return container;
}

QFrame* createBasicConfigSection(const QString& title, QWidget* control, QWidget* parent)
{
    QFrame* section = new QFrame(parent);
    section->setObjectName("BasicConfigSection");
    section->setFrameShape(QFrame::StyledPanel);
    section->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    section->setStyleSheet("QFrame#BasicConfigSection { border: 1px solid palette(mid); border-radius: 6px; background: palette(base); }");

    QHBoxLayout* layout = new QHBoxLayout(section);
    layout->setContentsMargins(8, 4, 8, 4);
    layout->setSpacing(8);

    QLabel* titleLabel = new QLabel(title, section);
    QFont titleFont = titleLabel->font();
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);
    titleLabel->setMinimumWidth(titleLabel->fontMetrics().horizontalAdvance("异常时间过滤") + 6);
    titleLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    layout->addWidget(titleLabel, 0, Qt::AlignVCenter);
    layout->addWidget(control, 1, Qt::AlignVCenter);
    return section;
}

QFrame* createStatusInfoSection(const QString& title, QLabel* valueLabel, QWidget* parent)
{
    QFrame* section = new QFrame(parent);
    section->setObjectName("StatusInfoSection");
    section->setFrameShape(QFrame::StyledPanel);
    section->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    section->setStyleSheet("QFrame#StatusInfoSection { border: 1px solid palette(mid); border-radius: 6px; background: palette(base); }");

    QHBoxLayout* layout = new QHBoxLayout(section);
    layout->setContentsMargins(8, 3, 8, 3);
    layout->setSpacing(8);

    QLabel* titleLabel = new QLabel(title, section);
    QFont titleFont = titleLabel->font();
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);
    titleLabel->setMinimumWidth(titleLabel->fontMetrics().horizontalAdvance("异常时间过滤") + 6);
    titleLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    valueLabel->setParent(section);
    valueLabel->setWordWrap(true);
    valueLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    valueLabel->setTextFormat(Qt::PlainText);
    valueLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    valueLabel->setMinimumWidth(0);
    valueLabel->setMinimumHeight(22);
    valueLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    valueLabel->setStyleSheet("QLabel { background: transparent; color: palette(window-text); padding: 2px 0; border: none; }");

    layout->addWidget(titleLabel, 0, Qt::AlignVCenter);
    layout->addWidget(valueLabel, 1, Qt::AlignVCenter);
    return section;
}

} // namespace

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
    QVBoxLayout* basicLayout = new QVBoxLayout(basicTab);
    basicLayout->setSpacing(6);
    basicLayout->setContentsMargins(8, 8, 8, 8);

    auto addBasicOptionRow = [this, basicLayout, basicTab](const QString& title,
                                                           uint16_t key,
                                                           const QStringList& options,
                                                           int currentIndex) {
        QWidget* optionButtons = createParameterOptionButtons(options, currentIndex, basicTab);
        basicLayout->addWidget(createBasicConfigSection(title, optionButtons, basicTab));
        parameterState.controls[key] = optionButtons;
        connect(ParameterOptionButtons::buttonGroup(optionButtons), &QButtonGroup::idToggled, this, [this, key](int, bool checked) {
            if (checked) {
                onParamConfigChanged(key);
            }
        });
    };

    addBasicOptionRow("工作模式", kKeyWorkMode, {"采样模式", "待机模式"}, 0);
    addBasicOptionRow("扫描模式", kKeyPatternMode, {"非重复扫描", "重复扫描", "低帧率重复扫描"}, 0);
    addBasicOptionRow("点云格式", kKeyPclDataType, {"高精度直角", "低精度直角", "球坐标"}, 0);
    addBasicOptionRow("探测模式", kKeyDetectMode, {"正常模式", "敏感模式"}, 0);
    addBasicOptionRow("IMU数据发送", kKeyImuDataEn, {"关闭", "开启"}, 0);
    addBasicOptionRow("电机转速", kKeySetEscMode, {"正常转速", "低转速"}, 0);
    addBasicOptionRow("异常时间过滤", kKeySetPpsSyncMode, {"关闭", "开启"}, 0);
    addBasicOptionRow("FOV模式", kKeySetFovMode, {"Focus", "Normal"}, 1);
    addBasicOptionRow("回波模式", kKeySetEchoMode, {"最强回波", "第一回波"}, 0);
    basicLayout->addStretch();

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
    QVBoxLayout* statusLayout = new QVBoxLayout(statusTab);
    statusLayout->setSpacing(4);
    statusLayout->setContentsMargins(8, 8, 8, 8);
    for (uint16_t key : statusKeysVec) {
        QLabel* valueLabel = new QLabel("无信息");
        QString statusTitle;
        switch (key) {
            case kKeySn: statusTitle = "序列号"; break;
            case kKeyProductInfo: statusTitle = "产品信息"; break;
            case kKeyVersionApp: statusTitle = "固件版本"; break;
            case kKeyVersionLoader: statusTitle = "LOADER版本"; break;
            case kKeyVersionHardware: statusTitle = "硬件版本"; break;
            case kKeyMac: statusTitle = "MAC地址"; break;
            case kKeyCurWorkState: statusTitle = "当前工作状态"; break;
            case kKeyCoreTemp: statusTitle = "核心温度"; break;
            case kKeyPowerUpCnt: statusTitle = "上电次数"; break;
            case kKeyLocalTimeNow: statusTitle = "本地时间"; break;
            case kKeyLastSyncTime: statusTitle = "最后同步时间"; break;
            case kKeyTimeOffset: statusTitle = "时间偏移"; break;
            case kKeyTimeSyncType: statusTitle = "时间同步类型"; break;
            case kKeyLidarDiagStatus: statusTitle = "雷达诊断状态"; break;
            case kKeyFwType: statusTitle = "固件类型"; break;
            case kKeyHmsCode: statusTitle = "HMS诊断码"; break;
        }
        statusLayout->addWidget(createStatusInfoSection(statusTitle, valueLabel, statusTab));
        parameterState.labels[key] = valueLabel;
    }
    statusLayout->addStretch();

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

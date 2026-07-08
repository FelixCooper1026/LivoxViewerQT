#include "LivoxViewerWindow.h"
#include "NetworkInterfaceService.h"
#include "widgets/ParameterOptionButtons.h"
#include "widgets/SwitchCheckBox.h"
#include <QAbstractSpinBox>
#include <QApplication>
#include <QButtonGroup>
#include <QHeaderView>
#include <QPalette>
#include <QSizePolicy>
#include <QTableView>
#include <QToolButton>

namespace {

struct ParameterOption
{
    QString text;
    int id;
};

QString parameterOptionButtonTheme()
{
    return QApplication::palette().color(QPalette::Window).lightness() < 128
        ? QStringLiteral("dark")
        : QStringLiteral("light");
}

QWidget* createParameterOptionButtons(const QVector<ParameterOption>& options, int currentIndex, QWidget* parent)
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
        button->setText(options.at(i).text);
        button->setToolButtonStyle(Qt::ToolButtonTextOnly);
        button->setCheckable(true);
        button->setCursor(Qt::PointingHandCursor);
        button->setMinimumHeight(30);
        button->setMinimumWidth(button->fontMetrics().horizontalAdvance(options.at(i).text) + 22);
        button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        button->setProperty("parameterOptionButton", true);
        button->setProperty("parameterOptionButtonTheme", parameterOptionButtonTheme());
        button->setStyleSheet(
            "QToolButton {"
            "  border: 1px solid transparent;"
            "  border-radius: 15px;"
            "  padding: 4px 10px;"
            "  background: transparent;"
            "  color: palette(button-text);"
            "}"
            "QToolButton:hover {"
            "  background: transparent;"
            "  border-color: palette(mid);"
            "  color: palette(button-text);"
            "}"
            "QToolButton:pressed {"
            "  background: palette(alternate-base);"
            "  border-color: transparent;"
            "  color: palette(window-text);"
            "}"
            "QToolButton[parameterOptionButtonTheme=\"dark\"]:checked {"
            "  background: palette(window-text);"
            "  border-color: transparent;"
            "  color: palette(window);"
            "}"
            "QToolButton[parameterOptionButtonTheme=\"dark\"]:checked:hover {"
            "  background: palette(window-text);"
            "  border-color: palette(mid);"
            "  color: palette(window);"
            "}"
            "QToolButton[parameterOptionButtonTheme=\"dark\"]:checked:pressed {"
            "  background: palette(mid);"
            "  border-color: transparent;"
            "  color: palette(window-text);"
            "}"
            "QToolButton[parameterOptionButtonTheme=\"light\"]:checked {"
            "  background: #d8d8d8;"
            "  border-color: transparent;"
            "  color: #202020;"
            "  font-weight: 600;"
            "}"
            "QToolButton[parameterOptionButtonTheme=\"light\"]:checked:hover {"
            "  background: #d0d0d0;"
            "  border-color: palette(mid);"
            "  color: #202020;"
            "}"
            "QToolButton[parameterOptionButtonTheme=\"light\"]:checked:pressed {"
            "  background: #c6c6c6;"
            "  border-color: transparent;"
            "  color: #202020;"
            "}"
        );

        group->addButton(button, options.at(i).id);
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
    section->setFrameShape(QFrame::NoFrame);
    section->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    section->setStyleSheet(
        "QFrame#StatusInfoSection {"
        "  border: 0;"
        "  border-bottom: 1px solid palette(mid);"
        "  background: transparent;"
        "}"
    );

    QHBoxLayout* layout = new QHBoxLayout(section);
    layout->setContentsMargins(0, 7, 0, 7);
    layout->setSpacing(12);

    QLabel* titleLabel = new QLabel(title, section);
    QFont titleFont = titleLabel->font();
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);
    titleLabel->setMinimumWidth(titleLabel->fontMetrics().horizontalAdvance("时间同步类型") + 12);
    titleLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    valueLabel->setParent(section);
    valueLabel->setWordWrap(true);
    valueLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    valueLabel->setTextFormat(Qt::PlainText);
    valueLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    valueLabel->setMinimumWidth(0);
    valueLabel->setMinimumHeight(20);
    valueLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    valueLabel->setStyleSheet("QLabel { background: transparent; color: palette(window-text); padding: 0; border: none; }");

    layout->addWidget(titleLabel, 0, Qt::AlignVCenter);
    layout->addWidget(valueLabel, 1, Qt::AlignVCenter);
    return section;
}

QFrame* createConfigPanelSection(const QString& title, QWidget* content, QWidget* parent)
{
    QFrame* section = new QFrame(parent);
    section->setObjectName("ConfigPanelSection");
    section->setFrameShape(QFrame::StyledPanel);
    section->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    section->setStyleSheet("QFrame#ConfigPanelSection { border: 1px solid palette(mid); border-radius: 6px; background: palette(base); }");

    QVBoxLayout* layout = new QVBoxLayout(section);
    layout->setContentsMargins(8, 6, 8, 8);
    layout->setSpacing(6);

    QLabel* titleLabel = new QLabel(title, section);
    QFont titleFont = titleLabel->font();
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);
    layout->addWidget(titleLabel);

    content->setParent(section);
    content->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    layout->addWidget(content);
    return section;
}

} // namespace

void LivoxViewerWindow::createParameterPanel()
{
    // 右侧：参数 Dock（包含标签页）
    paramsDock = new QDockWidget("参数", this);
    paramsDock->setObjectName("ParamsDock");
    paramsDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    paramsDock->setStyleSheet(QStringLiteral(
        "QDockWidget#ParamsDock {"
        "  border: none;"
        "}"
        "QDockWidget#ParamsDock::title {"
        "  border: none;"
        "}"
    ));
    QWidget* hiddenParamsTitleBar = new QWidget(paramsDock);
    hiddenParamsTitleBar->setFixedHeight(0);
    paramsDock->setTitleBarWidget(hiddenParamsTitleBar);

    QWidget* paramsDockContent = new QWidget(paramsDock);
    QVBoxLayout* paramsOuterLayout = new QVBoxLayout(paramsDockContent);
    paramsOuterLayout->setContentsMargins(8, 8, 8, 8);
    paramsOuterLayout->setSpacing(8);

    // 点属性 Dock（默认隐藏），尺寸与"参数"一致
    attrDock = new QDockWidget("点属性", this);
    attrDock->setObjectName("AttrDock");
    attrDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    attrDock->setStyleSheet(QStringLiteral(
        "QDockWidget#AttrDock {"
        "  border: none;"
        "}"
        "QDockWidget#AttrDock::title {"
        "  border: none;"
        "}"
    ));
    QWidget* hiddenAttrTitleBar = new QWidget(attrDock);
    hiddenAttrTitleBar->setFixedHeight(0);
    attrDock->setTitleBarWidget(hiddenAttrTitleBar);

    QWidget* attrContent = new QWidget(attrDock);
    QVBoxLayout* attrLayout = new QVBoxLayout(attrContent);
    attrLayout->setContentsMargins(8, 8, 8, 8);
    attrLayout->setSpacing(8);

    QFrame* selectionSummaryCard = new QFrame(attrContent);
    selectionSummaryCard->setObjectName("SelectionSummaryCard");
    selectionSummaryCard->setFrameShape(QFrame::StyledPanel);
    selectionSummaryCard->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    selectionSummaryCard->setStyleSheet(
        "QFrame#SelectionSummaryCard {"
        "  border: 1px solid palette(mid);"
        "  border-radius: 6px;"
        "  background: palette(alternate-base);"
        "}"
    );
    QVBoxLayout* summaryLayout = new QVBoxLayout(selectionSummaryCard);
    summaryLayout->setContentsMargins(10, 8, 10, 8);
    summaryLayout->setSpacing(4);
    QLabel* summaryTitle = new QLabel("框选统计", selectionSummaryCard);
    QFont summaryTitleFont = summaryTitle->font();
    summaryTitleFont.setBold(true);
    summaryTitle->setFont(summaryTitleFont);
    selectionSummaryLabel = new QLabel("未框选点云", selectionSummaryCard);
    selectionSummaryLabel->setWordWrap(true);
    summaryLayout->addWidget(summaryTitle);
    summaryLayout->addWidget(selectionSummaryLabel);
    attrLayout->addWidget(selectionSummaryCard);

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
    paramTabWidget->setObjectName(QStringLiteral("ParameterTabs"));
    paramTabWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    paramTabWidget->setMinimumWidth(0);
    paramTabWidget->setDocumentMode(true);
    paramTabWidget->tabBar()->setDrawBase(false);
    paramTabWidget->tabBar()->setExpanding(false);
    paramTabWidget->tabBar()->setElideMode(Qt::ElideNone);
    paramTabWidget->tabBar()->setUsesScrollButtons(false);
    paramTabWidget->setStyleSheet(QStringLiteral(
        "QTabWidget#ParameterTabs::pane {"
        "  border: none;"
        "}"
        "QTabWidget#ParameterTabs QTabBar {"
        "  qproperty-drawBase: false;"
        "  border: none;"
        "  background: transparent;"
        "}"
        "QTabWidget#ParameterTabs QTabBar::base {"
        "  border: none;"
        "  background: transparent;"
        "}"
        "QTabWidget#ParameterTabs QTabBar::tab {"
        "  border: none;"
        "  border-bottom: 2px solid transparent;"
        "  background: transparent;"
        "  padding: 5px 8px;"
        "  margin-right: 2px;"
        "}"
        "QTabWidget#ParameterTabs QTabBar::tab:selected {"
        "  color: #2f8cff;"
        "  border-bottom-color: #2f8cff;"
        "  font-weight: 600;"
        "}"
        "QTabWidget#ParameterTabs QTabBar::tab:!selected {"
        "  color: palette(window-text);"
        "}"
    ));

    // 基本配置页
    QWidget* basicTab = new QWidget();
    basicTab->setMinimumWidth(0);
    basicTab->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    QVBoxLayout* basicLayout = new QVBoxLayout(basicTab);
    basicLayout->setSpacing(6);
    basicLayout->setContentsMargins(8, 8, 8, 8);

    auto addBasicOptionRow = [this, basicLayout, basicTab](const QString& title,
                                                           uint16_t key,
                                                           const QVector<ParameterOption>& options,
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

    addBasicOptionRow("工作模式", kKeyWorkMode, {{"采样模式", 0}, {"待机模式", 1}}, 0);
    addBasicOptionRow("扫描模式", kKeyPatternMode, {{"非重复扫描", 0}, {"重复扫描", 1}, {"低帧率重复扫描", 2}}, 0);
    addBasicOptionRow("点云格式", kKeyPclDataType, {{"高精度直角", 0}, {"低精度直角", 1}, {"球坐标", 2}}, 0);
    addBasicOptionRow("探测模式", kKeyDetectMode, {{"正常模式", 0}, {"敏感模式", 1}}, 0);
    addBasicOptionRow("IMU数据发送", kKeyImuDataEn, {{"开启", 1}, {"关闭", 0}}, 0);
    addBasicOptionRow("电机转速", kKeySetEscMode, {{"正常转速", 0}, {"低转速", 1}}, 0);
    addBasicOptionRow("异常时间过滤", kKeySetPpsSyncMode, {{"开启", 1}, {"关闭", 0}}, 0);
    addBasicOptionRow("FOV模式", kKeySetFovMode, {{"Normal", 1}, {"Focus", 0}}, 1);
    addBasicOptionRow("回波模式", kKeySetEchoMode, {{"最强回波", 0}, {"第一回波", 1}}, 0);
    basicLayout->addStretch();

    paramTabWidget->addTab(basicTab, "基本配置");

    // 网络配置页
    QWidget* networkTab = new QWidget();
    networkTab->setMinimumWidth(0);
    networkTab->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    QVBoxLayout* networkLayout = new QVBoxLayout(networkTab);
    networkLayout->setSpacing(6);
    networkLayout->setContentsMargins(8, 8, 8, 8);

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

    QWidget* lidarIpContainer = new QWidget(networkTab);
    lidarIpContainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QGridLayout* lidarGrid = new QGridLayout(lidarIpContainer);
    lidarGrid->setContentsMargins(0, 0, 0, 0);
    lidarGrid->setHorizontalSpacing(6);
    lidarGrid->setVerticalSpacing(5);
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
    networkLayout->addWidget(createConfigPanelSection("雷达设备网络", lidarIpContainer, networkTab));
    parameterState.controls[kKeyLidarIpCfg] = lidarIpContainer;
    connect(lidarIpButton, &QPushButton::clicked, [this, lidarIpEdit, lidarMaskEdit, lidarGatewayEdit]() { applyIpConfig(kKeyLidarIpCfg, lidarIpEdit->text(), lidarMaskEdit->text(), lidarGatewayEdit->text()); });

    QWidget* targetGroup = new QWidget(networkTab);
    targetGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QGridLayout* targetLayout = new QGridLayout(targetGroup);
    targetLayout->setContentsMargins(0, 0, 0, 0);
    targetLayout->setHorizontalSpacing(6);
    targetLayout->setVerticalSpacing(5);
    targetLayout->setColumnStretch(0, 1);

    QVector<QLineEdit*> targetIpEdits;
    QPushButton* syncTargetIpButton = new QPushButton("同步目标 IP", targetGroup);
    syncTargetIpButton->setToolTip("将点云数据、IMU数据和推送信息的目标 IP 同步为当前选择网卡的主机 IP");
    syncTargetIpButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    targetLayout->addWidget(syncTargetIpButton, 0, 0, 1, 4, Qt::AlignRight);

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
        ipEdit->setMinimumWidth(fontMetrics().horizontalAdvance("255.255.255.255") + 8);
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

    createTargetRow("点云数据", kKeyLidarPointDataHostIpCfg, 56301, "应用点云数据发送目标 IP 和端口");
    createTargetRow("IMU数据", kKeyLidarImuHostIpCfg, 56401, "应用 IMU 数据发送目标 IP 和端口");
    createTargetRow("推送信息", kKeyStateInfoHostIpCfg, 56201, "应用状态/推送信息发送目标 IP 和端口");
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
    networkLayout->addWidget(createConfigPanelSection("数据发送目标", targetGroup, networkTab));

    QWidget* ntpGroup = new QWidget(networkTab);
    ntpGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QGridLayout* ntpLayout = new QGridLayout(ntpGroup);
    ntpLayout->setContentsMargins(0, 0, 0, 0);
    ntpLayout->setHorizontalSpacing(6);
    ntpLayout->setVerticalSpacing(5);
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
    networkLayout->addWidget(createConfigPanelSection("NTP 服务器", ntpGroup, networkTab));
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
        grid->addWidget(new QLabel("水平", controls), 0, 0);
        grid->addWidget(yawStart, 0, 1);
        grid->addWidget(new QLabel("~", controls), 0, 2, Qt::AlignCenter);
        grid->addWidget(yawStop, 0, 3);
        grid->addWidget(new QLabel("垂直", controls), 1, 0);
        grid->addWidget(pitchStart, 1, 1);
        grid->addWidget(new QLabel("~", controls), 1, 2, Qt::AlignCenter);
        grid->addWidget(pitchStop, 1, 3);
        grid->addWidget(applyButton, 2, 0, 1, 4, Qt::AlignRight);
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

    auto configureAttitudeSpin = [](QAbstractSpinBox* spin) {
        spin->setMinimumWidth(76);
        spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    };
    configureAttitudeSpin(rollEdit);
    configureAttitudeSpin(pitchEdit);
    configureAttitudeSpin(yawEdit);
    configureAttitudeSpin(xEdit);
    configureAttitudeSpin(yEdit);
    configureAttitudeSpin(zEdit);

    QWidget* attitudeContent = new QWidget(fovTab);
    QGridLayout* attitudeGrid = new QGridLayout(attitudeContent);
    attitudeGrid->setContentsMargins(0, 0, 0, 0);
    attitudeGrid->setHorizontalSpacing(8);
    attitudeGrid->setVerticalSpacing(5);
    attitudeGrid->addWidget(new QLabel("Roll", attitudeContent), 0, 0);
    attitudeGrid->addWidget(new QLabel("Pitch", attitudeContent), 0, 1);
    attitudeGrid->addWidget(new QLabel("Yaw", attitudeContent), 0, 2);
    attitudeGrid->addWidget(rollEdit, 1, 0);
    attitudeGrid->addWidget(pitchEdit, 1, 1);
    attitudeGrid->addWidget(yawEdit, 1, 2);
    attitudeGrid->addWidget(new QLabel("X", attitudeContent), 2, 0);
    attitudeGrid->addWidget(new QLabel("Y", attitudeContent), 2, 1);
    attitudeGrid->addWidget(new QLabel("Z", attitudeContent), 2, 2);
    attitudeGrid->addWidget(xEdit, 3, 0);
    attitudeGrid->addWidget(yEdit, 3, 1);
    attitudeGrid->addWidget(zEdit, 3, 2);
    attitudeGrid->addWidget(attitudeButton, 4, 0, 1, 3, Qt::AlignRight);
    attitudeGrid->setColumnStretch(0, 1);
    attitudeGrid->setColumnStretch(1, 1);
    attitudeGrid->setColumnStretch(2, 1);

    QFrame* attitudeSection = createConfigPanelSection("安装姿态", attitudeContent, fovTab);
    fovLayout->addWidget(attitudeSection);
    fovLayout->addStretch();
    parameterState.controls[kKeyInstallAttitude] = attitudeSection;
    connect(attitudeButton, &QPushButton::clicked, [this, rollEdit, pitchEdit, yawEdit, xEdit, yEdit, zEdit]() { applyAttitudeConfig(kKeyInstallAttitude, rollEdit->value(), pitchEdit->value(), yawEdit->value(), xEdit->value(), yEdit->value(), zEdit->value()); });
    paramTabWidget->addTab(fovTab, "其他配置");

    // 状态信息页
    QWidget* statusTab = new QWidget();
    statusTab->setMinimumWidth(0);
    statusTab->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    QVBoxLayout* statusLayout = new QVBoxLayout(statusTab);
    statusLayout->setSpacing(0);
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

    paramTabWidget->insertTab(0, statusTab, "设备参数");
    paramTabWidget->setCurrentIndex(0);

    paramsOuterLayout->addWidget(paramTabWidget);
    paramsDockContent->setLayout(paramsOuterLayout);

    QScrollArea* paramsScroll = new QScrollArea(paramsDock);
    paramsScroll->setObjectName(QStringLiteral("ParamsDockScroll"));
    paramsScroll->setWidgetResizable(true);
    paramsScroll->setFrameShape(QFrame::NoFrame);
    paramsScroll->setStyleSheet(QStringLiteral(
        "QScrollArea#ParamsDockScroll {"
        "  border: 0;"
        "  background: palette(window);"
        "}"
    ));
    paramsScroll->setWidget(paramsDockContent);
    paramsScroll->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    paramsDock->setWidget(paramsScroll);
    paramsDock->setMinimumWidth(0);

    addDockWidget(Qt::RightDockWidgetArea, paramsDock);
    tabifyDockWidget(paramsDock, attrDock);
    paramsDock->raise();
    activeRightDock = paramsDock;
    attrDock->hide();
}

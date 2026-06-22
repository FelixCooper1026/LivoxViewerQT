#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"
#include "LivoxCore/LidarDiagnostics.h"
#include <QApplication>
#include <QGuiApplication>
#include <QSplitter>
#include <QScreen>
#include <QFrame>
#include <QVariant>
#include <QHeaderView>
#include <QFont>
#include <QListWidget>
#include <QPalette>

namespace {

constexpr int kDefaultWindowMinWidth = 1100;
constexpr int kDefaultWindowMaxWidth = 1680;
constexpr int kDefaultWindowMinHeight = 720;
constexpr int kDefaultWindowMaxHeight = 1000;
constexpr int kDevicesDockMinWidth = 220;
constexpr int kDevicesDockMaxWidth = 320;
constexpr int kParamsDockMinWidth = 320;
constexpr int kParamsDockMaxWidth = 460;
constexpr int kLogDockMinHeight = 110;
constexpr int kLogDockMaxHeight = 180;

void clearLayoutItems(QLayout* layout)
{
    if (!layout) {
        return;
    }

    while (QLayoutItem* item = layout->takeAt(0)) {
        if (QWidget* widget = item->widget()) {
            widget->deleteLater();
        }
        if (QLayout* childLayout = item->layout()) {
            clearLayoutItems(childLayout);
        }
        delete item;
    }
}

QString realtimeDeviceCardSignature(const LidarDeviceInfo& device, bool active)
{
    return QStringLiteral("%1|%2|%3|%4|%5|%6|%7")
        .arg(device.handle)
        .arg(active ? 1 : 0)
        .arg(device.product_info,
             device.sn,
             device.lidar_ip,
             device.work_state,
             device.diagnostic_summary)
        .arg(device.diagnostic_severity);
}

QString hmsDisplayColor(int severity)
{
    const bool darkTheme = QApplication::palette().color(QPalette::Window).lightness() < 128;
    switch (severity) {
    case 1: return darkTheme ? QStringLiteral("#6cb6ff") : QStringLiteral("#2d7dd2");
    case 2: return darkTheme ? QStringLiteral("#f0b44c") : QStringLiteral("#b86e00");
    case 3: return darkTheme ? QStringLiteral("#ff6b6b") : QStringLiteral("#c93434");
    case 4: return darkTheme ? QStringLiteral("#ff4d6d") : QStringLiteral("#8b0000");
    default: return QApplication::palette().color(QPalette::WindowText).name();
    }
}

QWidget* createDeviceEmptyState(QWidget* parent)
{
    QWidget* emptyState = new QWidget(parent);
    emptyState->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QVBoxLayout* layout = new QVBoxLayout(emptyState);
    layout->setContentsMargins(12, 48, 12, 48);
    layout->setSpacing(8);
    layout->setAlignment(Qt::AlignHCenter);

    QLabel* iconLabel = new QLabel(emptyState);
    const QSize emptyStateIconSize(48, 48);
    ThemeIconUtils::setThemedSvgPixmap(iconLabel, QStringLiteral(":/icons/empty_state.svg"), emptyStateIconSize);
    iconLabel->setFixedSize(emptyStateIconSize);
    iconLabel->setAlignment(Qt::AlignCenter);

    QLabel* titleLabel = new QLabel(QStringLiteral("未发现设备"), emptyState);
    QFont titleFont = titleLabel->font();
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);
    titleLabel->setAlignment(Qt::AlignCenter);

    QLabel* hintLabel = new QLabel(QStringLiteral("请刷新网卡列表\n并选择与设备连接的网卡"), emptyState);
    hintLabel->setAlignment(Qt::AlignCenter);
    hintLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));

    layout->addWidget(iconLabel, 0, Qt::AlignHCenter);
    layout->addWidget(titleLabel);
    layout->addWidget(hintLabel);
    return emptyState;
}

class RealtimeDeviceCard : public QFrame
{
public:
    explicit RealtimeDeviceCard(QWidget* parent = nullptr)
        : QFrame(parent)
    {
        setCursor(Qt::PointingHandCursor);
    }

    std::function<void()> onClicked;

protected:
    void mousePressEvent(QMouseEvent* event) override
    {
        if (event->button() == Qt::LeftButton && onClicked) {
            onClicked();
            event->accept();
            return;
        }
        QFrame::mousePressEvent(event);
    }
};

int preferredExtent(int available, int minimum, int maximum)
{
    const int upper = qMin(available, maximum);
    const int lower = qMin(minimum, upper);
    return qMin(upper, qMax(available * 4 / 5, lower));
}

QSize defaultMainWindowSize()
{
    const QSize availableSize = QGuiApplication::primaryScreen()->availableGeometry().size();
    return QSize(preferredExtent(availableSize.width(), kDefaultWindowMinWidth, kDefaultWindowMaxWidth),
                 preferredExtent(availableSize.height(), kDefaultWindowMinHeight, kDefaultWindowMaxHeight));
}

int preferredDockWidth(int windowWidth, int percent, int minimum, int maximum)
{
    return qMin(maximum, qMax(minimum, windowWidth * percent / 100));
}


} // namespace

void LivoxViewerWindow::initializeUserInterface()
{
    // 设置应用程序字体，避免DirectWrite错误
    QFont appFont = QApplication::font();
#ifdef Q_OS_WIN
    appFont.setFamily("Microsoft YaHei"); // Windows: DirectWrite 字体兼容
    appFont.setPointSize(9);
    QApplication::setFont(appFont);
#endif

    // 中央视图：点云可视化
    QWidget* centralContainer = new QWidget(this);
    QVBoxLayout* centralLayout = new QVBoxLayout(centralContainer);
    centralLayout->setContentsMargins(0,0,0,0);
    centralLayout->setSpacing(0);

    visualizationWorkspace = new VisualizationWorkspace(centralContainer);
    visualizationWorkspace->installEventFilter(this);
    realtimePointCloudView = new PointCloudView(visualizationWorkspace);
    realtimePointCloudView->installEventFilter(this);
    realtimePointCloudView->setMinimumSize(200, 200);
    realtimePointCloudView->setPointSize(pointSizePx);
    pointCloudView = realtimePointCloudView;
    connect(realtimePointCloudView, &PointCloudView::lvx2FileDropped, this, &LivoxViewerWindow::onLvx2PlaybackFileDropped);
    connect(realtimePointCloudView, &PointCloudView::selectionPointsReady, this, &LivoxViewerWindow::onSelectionPointsReady);
    realtimeVisualizationTabId = visualizationWorkspace->addTab(
        VisualizationWorkspace::TabKind::RealtimePointCloud,
        QStringLiteral("实时点云"),
        realtimePointCloudView,
        false);
    activeVisualizationTabId = realtimeVisualizationTabId;
    pointCloudViewsByTab.insert(realtimeVisualizationTabId, realtimePointCloudView);
    connect(visualizationWorkspace, &VisualizationWorkspace::focusedTabChanged,
            this, &LivoxViewerWindow::onVisualizationFocusedTabChanged);
    connect(visualizationWorkspace, &VisualizationWorkspace::tabCloseRequested,
            this, &LivoxViewerWindow::closeVisualizationTab);

    // 顶部可视化功能栏独占整行，左右 Dock 从点云区域开始对齐。
    mainToolBar = new QToolBar(QStringLiteral("工具栏"), this);
    mainToolBar->setObjectName(QStringLiteral("MainToolBar"));
    mainToolBar->setMovable(false);
    mainToolBar->setFloatable(false);
    mainToolBar->setAllowedAreas(Qt::TopToolBarArea);
    mainToolBar->setContentsMargins(0, 0, 0, 0);
    QWidget* viewerToolbar = createViewerToolbar(mainToolBar);
    mainToolBar->addWidget(viewerToolbar);
    mainToolBar->setFixedHeight(viewerToolbar->height() + 4);
    addToolBar(Qt::TopToolBarArea, mainToolBar);

    createPlaybackBar(visualizationWorkspace);

    centralLayout->addWidget(visualizationWorkspace, 1);
    setCentralWidget(centralContainer);
    resize(defaultMainWindowSize());

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

    createDevicePanel();
    createParameterPanel();
    createImuPanel();
    createFileInfoPanel();
    createLogPanel();

    setTabPosition(Qt::LeftDockWidgetArea, QTabWidget::North);
    setTabPosition(Qt::RightDockWidgetArea, QTabWidget::North);
    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

    networkDock->setMinimumWidth(kDevicesDockMinWidth);
    lidarDevicesDock->setMinimumWidth(kDevicesDockMinWidth);
    imuDock->setMinimumWidth(kDevicesDockMinWidth);
    lvx2FileDock->setMinimumWidth(kDevicesDockMinWidth);
    paramsDock->setMinimumWidth(kParamsDockMinWidth);
    attrDock->setMinimumWidth(kParamsDockMinWidth);

    // 初始布局尺寸（近似 CloudCompare）：左侧窄、右侧中、底部适中
    const int devicesDockWidth = preferredDockWidth(width(), 16, kDevicesDockMinWidth, kDevicesDockMaxWidth);
    const int paramsDockWidth = preferredDockWidth(width(), 24, kParamsDockMinWidth, kParamsDockMaxWidth);
    const int logDockHeight = qMin(kLogDockMaxHeight, qMax(kLogDockMinHeight, height() / 6));
    resizeDocks({lidarDevicesDock, paramsDock}, {devicesDockWidth, paramsDockWidth}, Qt::Horizontal);
    resizeDocks({logDock}, {logDockHeight}, Qt::Vertical);
    lidarDevicesDock->raise();
    rebuildRealtimeDeviceCards();

    createMenusAndActions();
}


// 刷新按钮已移除，无需实现 onRefreshClicked

void LivoxViewerWindow::updateLidarDeviceList()
{
    if (!realtimeDeviceListWidget) {
        return;
    }

    QVector<LidarDeviceInfo> devices;
    {
        QMutexLocker locker(&lidarDeviceMutex);
        const QList<LidarDeviceInfo> values = lidarDevices.values();
        devices.reserve(values.size());
        for (const LidarDeviceInfo& device : values) {
            if (device.is_connected) {
                devices.append(device);
            }
        }
    }

    bool currentExists = false;
    for (const LidarDeviceInfo& device : devices) {
        if (hasCurrentLidarHandle && device.handle == currentLidarHandle) {
            currentExists = true;
            break;
        }
    }
    if (devices.isEmpty()) {
        clearCurrentDevice();
        {
            QMutexLocker locker(&frameMutex);
            pendingFrames.clear();
            lastSeenTimestamp.clear();
            lastFrameTimestamp.clear();
        }
        if (realtimePointCloudView) {
            realtimePointCloudView->clearPointCloud();
        }
    } else if (!hasCurrentLidarHandle || !currentExists) {
        setCurrentDeviceHandle(devices.first().handle);
    }

    rebuildRealtimeDeviceCards();
}

void LivoxViewerWindow::rebuildRealtimeDeviceCards()
{
    if (!realtimeDeviceListWidget) {
        return;
    }

    QVector<LidarDeviceInfo> devices;
    {
        QMutexLocker locker(&lidarDeviceMutex);
        const QList<LidarDeviceInfo> values = lidarDevices.values();
        devices.reserve(values.size());
        for (const LidarDeviceInfo& device : values) {
            if (device.is_connected) {
                devices.append(device);
            }
        }
    }

    QStringList signature;
    signature.reserve(devices.size());
    if (devices.isEmpty()) {
        signature.append(QStringLiteral("__empty__"));
    }
    for (const LidarDeviceInfo& device : devices) {
        const bool active = hasCurrentLidarHandle && currentLidarHandle == device.handle;
        signature.append(realtimeDeviceCardSignature(device, active));
    }
    if (realtimeDeviceListWidget->property("realtimeDeviceCardSignature").toStringList() == signature) {
        return;
    }
    realtimeDeviceListWidget->setProperty("realtimeDeviceCardSignature", signature);

    QVBoxLayout* deviceListLayout = qobject_cast<QVBoxLayout*>(realtimeDeviceListWidget->layout());
    clearLayoutItems(deviceListLayout);
    if (devices.isEmpty()) {
        deviceListLayout->addStretch(1);
        deviceListLayout->addWidget(createDeviceEmptyState(realtimeDeviceListWidget));
        deviceListLayout->addStretch(2);
        return;
    }
    for (const LidarDeviceInfo& device : devices) {
        deviceListLayout->addWidget(createRealtimeDeviceCard(device));
    }
    deviceListLayout->addStretch();
}

void LivoxViewerWindow::updateLidarDeviceInfo(const LidarDeviceInfo& device)
{
    {
        QMutexLocker locker(&lidarDeviceMutex);
        lidarDevices[device.handle] = device;
    }
    updateLidarDeviceList();
}

QVector<LidarDeviceInfo> LivoxViewerWindow::connectedLidarDevicesSnapshot()
{
    QVector<LidarDeviceInfo> devices;
    QMutexLocker locker(&lidarDeviceMutex);
    for (const LidarDeviceInfo& device : lidarDevices) {
        if (device.is_connected) {
            devices.append(device);
        }
    }
    return devices;
}

void LivoxViewerWindow::setActiveRealtimeDevice(uint32_t handle)
{
    if (shutting_down) {
        return;
    }

    LidarDeviceInfo device;
    {
        QMutexLocker locker(&lidarDeviceMutex);
        auto it = lidarDevices.constFind(handle);
        if (it == lidarDevices.constEnd()) {
            return;
        }
        device = it.value();
    }

    setCurrentDeviceHandle(handle);
    if (statusLabel) {
        statusLabel->setText(device.is_connected ? QStringLiteral("状态: 已连接") : QStringLiteral("状态: 未连接"));
    }
    {
        QMutexLocker locker(&frameMutex);
        pendingFrames.clear();
        lastSeenTimestamp.clear();
        lastFrameTimestamp.clear();
    }
    if (realtimePointCloudView) {
        realtimePointCloudView->clearPointCloud();
    }
    updateLidarDeviceList();
    if (paramTabWidget) {
        onTabChanged(paramTabWidget->currentIndex());
    }
}

QWidget* LivoxViewerWindow::createRealtimeDeviceCard(const LidarDeviceInfo& device)
{
    const bool active = hasCurrentLidarHandle && currentLidarHandle == device.handle;
    const QString modelName = device.product_info.isEmpty() ? QStringLiteral("Unknown") : device.product_info;
    const QString statusText = device.work_state.isEmpty() ? QStringLiteral("读取中") : device.work_state;
    const QString diagnosticText = device.diagnostic_summary.isEmpty()
        ? QStringLiteral("诊断码: 未知")
        : device.diagnostic_summary;
    const QString tip = QStringLiteral("型号: %1\nSN: %2\nIP: %3\n状态: %4\n%5")
                            .arg(modelName, device.sn, device.lidar_ip, statusText, diagnosticText);

    RealtimeDeviceCard* card = new RealtimeDeviceCard(realtimeDeviceListWidget);
    card->setObjectName("RealtimeDeviceCard");
    card->setFrameShape(QFrame::StyledPanel);
    card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    card->setToolTip(tip);
    card->setStyleSheet(active
        ? "QFrame#RealtimeDeviceCard { border: 2px solid palette(highlight); border-radius: 6px; background: palette(alternate-base); }"
        : "QFrame#RealtimeDeviceCard { border: 1px solid palette(mid); border-radius: 6px; background: palette(base); }");
    card->onClicked = [this, handle = device.handle]() {
        setActiveRealtimeDevice(handle);
    };

    QVBoxLayout* cardLayout = new QVBoxLayout(card);
    cardLayout->setContentsMargins(8, 6, 8, 6);
    cardLayout->setSpacing(4);

    QHBoxLayout* headerLayout = new QHBoxLayout();
    headerLayout->setContentsMargins(0, 0, 0, 0);
    headerLayout->setSpacing(6);
    QLabel* modelLabel = new QLabel(modelName, card);
    QFont modelFont = modelLabel->font();
    modelFont.setBold(true);
    modelLabel->setFont(modelFont);
    modelLabel->setWordWrap(true);
    modelLabel->setToolTip(tip);
    modelLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    QLabel* deviceStatusLabel = new QLabel(statusText, card);
    deviceStatusLabel->setToolTip(tip);
    deviceStatusLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    headerLayout->addWidget(modelLabel, 1);
    headerLayout->addWidget(deviceStatusLabel);
    cardLayout->addLayout(headerLayout);

    QLabel* snLabel = new QLabel(QStringLiteral("SN: %1").arg(device.sn), card);
    QLabel* ipLabel = new QLabel(QStringLiteral("IP: %1").arg(device.lidar_ip), card);
    QLabel* diagnosticLabel = new QLabel(diagnosticText, card);
    diagnosticLabel->setStyleSheet(QString("color: %1; font-weight: %2;")
                                       .arg(hmsDisplayColor(device.diagnostic_severity))
                                       .arg(device.diagnostic_severity >= 3 ? "600" : "400"));
    for (QLabel* label : {snLabel, ipLabel, diagnosticLabel}) {
        label->setToolTip(tip);
        label->setWordWrap(true);
        label->setAttribute(Qt::WA_TransparentForMouseEvents);
    }
    cardLayout->addWidget(snLabel);
    cardLayout->addWidget(ipLabel);
    cardLayout->addWidget(diagnosticLabel);
    return card;
}


void LivoxViewerWindow::onTabChanged(int index)
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) ||
        !currentDevice.is_connected ||
        !currentDevice.parameter_query_ready) {
        return;
    }

    // 清除旧的配置标记，准备接受新一轮查询结果
    parameterState.updatedConfigKeys.clear();
    livox_status status = QueryLivoxLidarInternalInfo(currentDevice.handle, onQueryInternalInfoResponse, this);
    if (status != kLivoxLidarStatusSuccess) {
        logMessage(QString("切换至标签页[%1]时查询设备信息失败: %2 (错误码: %3)")
                     .arg(index)
                     .arg(getLivoxStatusString(status))
                     .arg(static_cast<int>(status)));
    }
}


void LivoxViewerWindow::updateNoiseFilterList()
{
    if (!filterState.noiseFilterList) return;

    filterState.noiseFilterList->clear();
    for (uint8_t tag : filterState.noiseFilterTags) {
        QString itemText = QString("Tag值: %1").arg(tag);
        QListWidgetItem* item = new QListWidgetItem(itemText);
        item->setData(Qt::UserRole, tag);
        filterState.noiseFilterList->addItem(item);
    }

    // 更新移除按钮状态
    if (filterState.removeNoiseFilterButton) {
        filterState.removeNoiseFilterButton->setEnabled(!filterState.noiseFilterTags.isEmpty());
    }
}

void LivoxViewerWindow::refreshNetworkInterfaces()
{
    if (!networkInterfaceCombo) return;

    constexpr qint64 kNetworkWaitLogIntervalMs = 30000;
    auto logNetworkWaitMessage = [this](const QString& message) {
        if (lastNetworkWaitLogMessage != message ||
            !networkWaitLogTimer.isValid() ||
            networkWaitLogTimer.elapsed() >= kNetworkWaitLogIntervalMs) {
            logMessage(message);
            lastNetworkWaitLogMessage = message;
            networkWaitLogTimer.restart();
        }
    };

    const QString previousName = selectedInterfaceName.isEmpty()
        ? selectedNetworkInterfaceSysName
        : selectedInterfaceName;
    networkInterfaceCombo->blockSignals(true);
    networkInterfaceCombo->clear();

    const QList<NetworkInterfaceService::NetworkInterfaceInfo> interfaces =
        NetworkInterfaceService::availableLidarInterfaces();

    int selectedIndex = -1;
    for (int i = 0; i < interfaces.size(); ++i) {
        const NetworkInterfaceService::NetworkInterfaceInfo& iface = interfaces.at(i);
        networkInterfaceCombo->addItem(QString("%1 - %2").arg(iface.displayName, iface.ipv4), iface.systemName);
        networkInterfaceCombo->setItemData(i, iface.ipv4, Qt::UserRole + 1);
        networkInterfaceCombo->setItemData(i, iface.displayName, Qt::UserRole + 2);
        networkInterfaceCombo->setItemData(i, iface.netmask, Qt::UserRole + 3);
        networkInterfaceCombo->setItemData(i, iface.broadcast, Qt::UserRole + 4);
        if (iface.systemName == previousName) {
            selectedIndex = i;
        }
    }

    if (selectedIndex < 0 && previousName.isEmpty() && !interfaces.isEmpty()) {
        selectedIndex = 0;
    }

    if (selectedIndex >= 0) {
        networkInterfaceCombo->setCurrentIndex(selectedIndex);
        selectLidarInterface(interfaces.at(selectedIndex));
        lastNetworkWaitLogMessage.clear();
        networkWaitLogTimer.invalidate();
        logMessage(QString("[Network] Selected lidar interface: %1 (%2)")
                       .arg(selectedInterfaceDisplayName, selectedHostIp));
    } else if (!previousName.isEmpty()) {
        networkInterfaceCombo->setCurrentIndex(-1);
        logNetworkWaitMessage(QString("[Network] Selected lidar interface %1 is unavailable; keep selection and wait.")
                                  .arg(previousName));
    } else {
        selectedInterfaceName.clear();
        selectedInterfaceDisplayName.clear();
        selectedHostIp.clear();
        selectedNetmask.clear();
        selectedBroadcast.clear();
        selectedNetworkIP.clear();
        selectedNetworkInterfaceHumanName.clear();
        selectedNetworkInterfaceSysName.clear();
        logNetworkWaitMessage("[Network] No valid lidar network interface found");
    }

    networkInterfaceCombo->blockSignals(false);
}

void LivoxViewerWindow::onNetworkInterfaceChanged(int index)
{
    if (!networkInterfaceCombo || index < 0) return;

    const QString systemName = networkInterfaceCombo->itemData(index, Qt::UserRole).toString();
    const auto iface = NetworkInterfaceService::findInterfaceByName(systemName);
    if (!iface.has_value()) {
        return;
    }

    const QString oldName = selectedInterfaceName;
    const QString oldIp = selectedHostIp;
    selectLidarInterface(*iface);

    if (oldName == selectedInterfaceName && oldIp == selectedHostIp) {
        return;
    }

    logMessage(QString("[Realtime] Network interface changed: %1 -> %2")
                   .arg(oldIp, selectedHostIp));

    if (sdk_started || sdk_initialized || realtimeState == RealtimeConnectionState::Running) {
        restartRealtimeConnectionForNetworkChange();
        return;
    }

    stopLidarDiscovery();
    startLidarDiscovery();
}

QString LivoxViewerWindow::getSelectedHostIP() const
{
    if (!selectedInterfaceName.isEmpty()) {
        const auto iface = NetworkInterfaceService::findInterfaceByName(selectedInterfaceName);
        if (iface.has_value()) {
            return iface->ipv4;
        }
    }

    const QList<NetworkInterfaceService::NetworkInterfaceInfo> interfaces =
        NetworkInterfaceService::availableLidarInterfaces();
    if (!interfaces.isEmpty()) {
        return interfaces.first().ipv4;
    }

    return QString();
}

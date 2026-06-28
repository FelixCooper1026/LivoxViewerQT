#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"
#include <QIcon>
#include <QSize>
#include <QSizePolicy>
#include <QToolButton>

#include <functional>

namespace {

void updateSlamLayerCardState(bool visible,
                              QToolButton* visibleButton,
                              QLabel* titleLabel,
                              QLabel* descriptionLabel)
{
    ThemeIconUtils::setThemedSvgIcon(visibleButton,
        visible ? QStringLiteral(":/icons/eye.svg") : QStringLiteral(":/icons/eye_off.svg"));
    visibleButton->setToolTip(visible ? QStringLiteral("隐藏图层") : QStringLiteral("显示图层"));
    const QString textStyle = visible ? QString() : QStringLiteral("color: palette(mid);");
    titleLabel->setStyleSheet(textStyle);
    descriptionLabel->setStyleSheet(textStyle);
}

} // namespace

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

void LivoxViewerWindow::createSlamInfoPanel()
{
    slamInfoDock = new QDockWidget(QStringLiteral("SLAM"), this);
    slamInfoDock->setObjectName(QStringLiteral("SlamInfoDock"));
    slamInfoDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    slamInfoDock->setStyleSheet(QStringLiteral(
        "QDockWidget#SlamInfoDock {"
        "  border: none;"
        "}"
        "QDockWidget#SlamInfoDock::title {"
        "  border: none;"
        "}"
    ));
    QWidget* hiddenSlamTitleBar = new QWidget(slamInfoDock);
    hiddenSlamTitleBar->setFixedHeight(0);
    slamInfoDock->setTitleBarWidget(hiddenSlamTitleBar);

    QWidget* slamInfoContent = new QWidget(slamInfoDock);
    QVBoxLayout* slamInfoLayout = new QVBoxLayout(slamInfoContent);
    slamInfoLayout->setContentsMargins(8, 8, 8, 8);
    slamInfoLayout->setSpacing(6);

    QWidget* layerListWidget = new QWidget(slamInfoContent);
    layerListWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    QVBoxLayout* layerListLayout = new QVBoxLayout(layerListWidget);
    layerListLayout->setContentsMargins(0, 0, 0, 0);
    layerListLayout->setSpacing(6);

    auto addLayerCard = [this, layerListLayout, layerListWidget](
                            const QString& title,
                            const QString& description,
                            bool visible,
                            const std::function<void(bool)>& setVisible) {
        QFrame* card = new QFrame(layerListWidget);
        card->setObjectName(QStringLiteral("SlamLayerCard"));
        card->setFrameShape(QFrame::StyledPanel);
        card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        card->setStyleSheet(QStringLiteral(
            "QFrame#SlamLayerCard {"
            "  border: 1px solid palette(mid);"
            "  border-radius: 6px;"
            "  background: palette(base);"
            "}"
        ));

        QVBoxLayout* cardLayout = new QVBoxLayout(card);
        cardLayout->setContentsMargins(8, 6, 8, 6);
        cardLayout->setSpacing(4);

        QHBoxLayout* headerLayout = new QHBoxLayout();
        headerLayout->setContentsMargins(0, 0, 0, 0);
        headerLayout->setSpacing(6);

        QToolButton* visibleButton = new QToolButton(card);
        visibleButton->setCheckable(true);
        visibleButton->setChecked(visible);
        visibleButton->setAutoRaise(true);
        visibleButton->setIconSize(QSize(20, 20));
        visibleButton->setFixedSize(28, 28);

        QLabel* titleLabel = new QLabel(title, card);
        QFont titleFont = titleLabel->font();
        titleFont.setBold(true);
        titleLabel->setFont(titleFont);
        titleLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        titleLabel->setWordWrap(true);

        headerLayout->addWidget(visibleButton);
        headerLayout->addWidget(titleLabel, 1);
        cardLayout->addLayout(headerLayout);

        QLabel* descriptionLabel = new QLabel(description, card);
        descriptionLabel->setWordWrap(true);
        cardLayout->addWidget(descriptionLabel);

        updateSlamLayerCardState(visible, visibleButton, titleLabel, descriptionLabel);
        connect(visibleButton, &QToolButton::toggled, this,
                [visibleButton, titleLabel, descriptionLabel, setVisible](bool checked) {
                    updateSlamLayerCardState(checked, visibleButton, titleLabel, descriptionLabel);
                    setVisible(checked);
                });

        layerListLayout->addWidget(card);
    };

    addLayerCard(QStringLiteral("世界系点云"),
                 QStringLiteral("SLAM tab 主点云，受积分时间、点大小、着色模式和色标控制。"),
                 slamWorldFrameVisible,
                 [this](bool visible) { setSlamWorldFrameVisible(visible); });
    addLayerCard(QStringLiteral("世界系当前帧点云"),
                 QStringLiteral("当前扫描帧世界系 overlay，使用首选项中的固定颜色。"),
                 slamWorldCurrentFrameVisible,
                 [this](bool visible) { setSlamWorldCurrentFrameVisible(visible); });
    addLayerCard(QStringLiteral("机体系点云"),
                 QStringLiteral("IMU 机体系当前帧 overlay，使用首选项中的固定颜色。"),
                 slamBodyFrameVisible,
                 [this](bool visible) { setSlamBodyFrameVisible(visible); });
    addLayerCard(QStringLiteral("轨迹"),
                 QStringLiteral("SLAM 位姿轨迹 overlay。"),
                 slamTrajectoryVisible,
                 [this](bool visible) { setSlamTrajectoryVisible(visible); });
    addLayerCard(QStringLiteral("姿态坐标轴"),
                 QStringLiteral("当前位姿坐标轴 overlay。"),
                 slamPoseAxisVisible,
                 [this](bool visible) { setSlamPoseAxisVisible(visible); });

    layerListLayout->addStretch();

    QScrollArea* layerScroll = new QScrollArea(slamInfoContent);
    layerScroll->setWidgetResizable(true);
    layerScroll->setFrameShape(QFrame::NoFrame);
    layerScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    layerScroll->setWidget(layerListWidget);
    slamInfoLayout->addWidget(layerScroll);
    slamInfoContent->setLayout(slamInfoLayout);
    slamInfoDock->setWidget(slamInfoContent);
    slamInfoDock->setMinimumWidth(0);

    addDockWidget(Qt::LeftDockWidgetArea, slamInfoDock);
    tabifyDockWidget(lidarDevicesDock, slamInfoDock);
    slamInfoDock->hide();
}

void LivoxViewerWindow::showSlamInfoPanel()
{
    if (!slamInfoDock) {
        return;
    }
    slamInfoDock->show();
    slamInfoDock->raise();
}

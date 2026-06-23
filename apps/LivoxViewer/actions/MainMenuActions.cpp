#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"

#include <QDesktopServices>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QRadioButton>
#include <QStandardPaths>
#include <QToolButton>
#include <QUrl>

#include <algorithm>

namespace {

constexpr int kPanelVisibilityButtonSize = 24;
constexpr int kPanelVisibilityIconSize = 22;

QToolButton* createPanelVisibilityButton(QWidget* parent, const QString& iconPath, const QString& tooltip)
{
    QToolButton* button = new QToolButton(parent);
    button->setCheckable(true);
    button->setAutoRaise(true);
    button->setCursor(Qt::PointingHandCursor);
    button->setToolTip(tooltip);
    button->setIconSize(QSize(kPanelVisibilityIconSize, kPanelVisibilityIconSize));
    button->setFixedSize(kPanelVisibilityButtonSize, kPanelVisibilityButtonSize);
    ThemeIconUtils::setThemedSvgIcon(button, iconPath);
    button->setStyleSheet(QStringLiteral(
        "QToolButton {"
        "  border: 1px solid transparent;"
        "  border-radius: 4px;"
        "  background: transparent;"
        "  padding: 1px;"
        "}"
        "QToolButton:hover {"
        "  background: palette(alternate-base);"
        "}"
    ));
    return button;
}

QToolButton* createMenuIconButton(QWidget* parent, const QString& iconPath, const QString& tooltip)
{
    QToolButton* button = createPanelVisibilityButton(parent, iconPath, tooltip);
    button->setCheckable(false);
    return button;
}

} // namespace

void LivoxViewerWindow::createMenusAndActions()
{
    // 顶部工具栏
    actionClearCloud = new QAction("清除点云", this);
    actionResetView = new QAction("重置视图", this);

    // 菜单栏
    menuBar = new QMenuBar(this);
    menuBar->setStyleSheet(QStringLiteral(
        "QMenuBar {"
        "  background: palette(window);"
        "  border: none;"
        "  border-bottom: 1px solid palette(mid);"
        "  padding: 3px 0 0 0;"
        "}"
        "QMenuBar::item {"
        "  padding: 4px 12px 4px 12px;"
        "  border: none;"
        "  background: transparent;"
        "}"
        "QMenuBar::item:selected,"
        "QMenuBar::item:pressed {"
        "  border: none;"
        "  background: palette(alternate-base);"
        "}"
    ));
    setMenuBar(menuBar);
    fileMenu = menuBar->addMenu("文件");
    viewMenu = menuBar->addMenu("视图");
    deviceMenu = menuBar->addMenu("设备");
    QMenu* toolsMenu = menuBar->addMenu("工具");
    helpMenu = menuBar->addMenu("帮助");

    QWidget* panelControls = new QWidget(menuBar);
    panelControls->setObjectName(QStringLiteral("PanelVisibilityControls"));
    QHBoxLayout* panelControlsLayout = new QHBoxLayout(panelControls);
    panelControlsLayout->setContentsMargins(0, 0, 8, 0);
    panelControlsLayout->setSpacing(2);

    QToolButton* leftPanelButton = createPanelVisibilityButton(
        panelControls,
        QStringLiteral(":/icons/layout_panel_left.svg"),
        QStringLiteral("显示/隐藏左侧面板"));
    QToolButton* bottomPanelButton = createPanelVisibilityButton(
        panelControls,
        QStringLiteral(":/icons/layout_panel_bottom.svg"),
        QStringLiteral("显示/隐藏底部面板"));
    QToolButton* rightPanelButton = createPanelVisibilityButton(
        panelControls,
        QStringLiteral(":/icons/layout_panel_right.svg"),
        QStringLiteral("显示/隐藏右侧面板"));

    QToolButton* settingsButton = createMenuIconButton(
        panelControls,
        QStringLiteral(":/icons/settings.svg"),
        QStringLiteral("首选项"));

    panelControlsLayout->addWidget(leftPanelButton);
    panelControlsLayout->addWidget(bottomPanelButton);
    panelControlsLayout->addWidget(rightPanelButton);
    panelControlsLayout->addWidget(settingsButton);
    menuBar->setCornerWidget(panelControls, Qt::TopRightCorner);

    auto syncPanelButtons = [this, leftPanelButton, bottomPanelButton, rightPanelButton]() {
        const bool leftVisible = (networkDock && networkDock->isVisible()) ||
                                 (lidarDevicesDock && lidarDevicesDock->isVisible()) ||
                                 (imuDock && imuDock->isVisible()) ||
                                 (lvx2FileDock && lvx2FileDock->isVisible());
        const bool bottomVisible = logDock && logDock->isVisible();
        const bool rightVisible = (paramsDock && paramsDock->isVisible()) ||
                                  (attrDock && attrDock->isVisible());
        leftPanelButton->setChecked(leftVisible);
        bottomPanelButton->setChecked(bottomVisible);
        rightPanelButton->setChecked(rightVisible);
        ThemeIconUtils::setThemedSvgIcon(leftPanelButton,
            leftVisible ? QStringLiteral(":/icons/layout_panel_left_active.svg")
                        : QStringLiteral(":/icons/layout_panel_left.svg"));
        ThemeIconUtils::setThemedSvgIcon(bottomPanelButton,
            bottomVisible ? QStringLiteral(":/icons/layout_panel_bottom_active.svg")
                          : QStringLiteral(":/icons/layout_panel_bottom.svg"));
        ThemeIconUtils::setThemedSvgIcon(rightPanelButton,
            rightVisible ? QStringLiteral(":/icons/layout_panel_right_active.svg")
                         : QStringLiteral(":/icons/layout_panel_right.svg"));
    };

    connect(leftPanelButton, &QToolButton::clicked, this, [this, syncPanelButtons]() {
        const bool visible = (networkDock && networkDock->isVisible()) ||
                             (lidarDevicesDock && lidarDevicesDock->isVisible()) ||
                             (imuDock && imuDock->isVisible()) ||
                             (lvx2FileDock && lvx2FileDock->isVisible());
        networkDock->setVisible(!visible);
        lidarDevicesDock->setVisible(!visible);
        imuDock->setVisible(!visible);
        if (lvx2FileDock) {
            lvx2FileDock->setVisible(!visible && playbackState.active);
        }
        if (!visible) {
            if (lvx2FileDock && playbackState.active) {
                lvx2FileDock->raise();
            } else {
                lidarDevicesDock->raise();
            }
        }
        syncPanelButtons();
    });
    connect(bottomPanelButton, &QToolButton::clicked, this, [this, syncPanelButtons]() {
        logDock->setVisible(!logDock->isVisible());
        syncPanelButtons();
    });
    connect(rightPanelButton, &QToolButton::clicked, this, [this, syncPanelButtons]() {
        const bool visible = (paramsDock && paramsDock->isVisible()) ||
                             (attrDock && attrDock->isVisible());
        paramsDock->setVisible(!visible);
        attrDock->setVisible(false);
        if (!visible) {
            paramsDock->raise();
        }
        syncPanelButtons();
    });
    connect(settingsButton, &QToolButton::clicked, this, &LivoxViewerWindow::showPreferencesDialog);

    for (QDockWidget* dock : {networkDock, lidarDevicesDock, imuDock, lvx2FileDock, logDock, paramsDock, attrDock}) {
        connect(dock, &QDockWidget::visibilityChanged, panelControls, syncPanelButtons);
    }
    syncPanelButtons();

    createFileActions();
    createDeviceActions();
    createHelpActions();
    // 视图菜单：显示/隐藏 dock
    viewMenu->addAction(lidarDevicesDock->toggleViewAction());
    viewMenu->addAction(lvx2FileDock->toggleViewAction());
    viewMenu->addAction(paramsDock->toggleViewAction());
    viewMenu->addAction(attrDock->toggleViewAction());
    viewMenu->addAction(imuDock->toggleViewAction());
    viewMenu->addAction(logDock->toggleViewAction());

    createStatusBarAndTimers();
    createPlaybackActions(toolsMenu);

}

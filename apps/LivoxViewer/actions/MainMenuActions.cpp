#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"

#include <QAbstractButton>
#include <QApplication>
#include <QCursor>
#include <QDesktopServices>
#include <QDialogButtonBox>
#include <QDir>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QRadioButton>
#include <QStandardPaths>
#include <QToolButton>
#include <QUrl>

#include <algorithm>

#ifdef Q_OS_WIN
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <windowsx.h>
#endif

namespace {

constexpr int kPanelVisibilityButtonSize = 24;
constexpr int kPanelVisibilityIconSize = 22;
constexpr int kTitleBarHeight = 34;
constexpr int kWindowControlButtonWidth = 46;
constexpr int kWindowControlButtonHeight = 32;
constexpr int kResizeBorderWidth = 8;
constexpr int kTitleBarTopResizeBorderWidth = 2;

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
QPoint globalMousePosition(QMouseEvent* event)
{
    return event->globalPosition().toPoint();
}
#else
QPoint globalMousePosition(QMouseEvent* event)
{
    return event->globalPos();
}
#endif

bool isInteractiveTitleBarChild(QWidget* child, QWidget* titleBar)
{
    for (QWidget* widget = child; widget && widget != titleBar; widget = widget->parentWidget()) {
        if (qobject_cast<QAbstractButton*>(widget) || qobject_cast<QMenuBar*>(widget)) {
            return true;
        }
    }
    return false;
}

void minimizeWindow(QMainWindow* window)
{
#ifdef Q_OS_WIN
    ShowWindow(HWND(window->winId()), SW_MINIMIZE);
#else
    window->showMinimized();
#endif
}

void toggleMaximizedWindow(QMainWindow* window)
{
#ifdef Q_OS_WIN
    HWND hwnd = HWND(window->winId());
    ShowWindow(hwnd, IsZoomed(hwnd) ? SW_RESTORE : SW_MAXIMIZE);
#else
    window->isMaximized() ? window->showNormal() : window->showMaximized();
#endif
}

void closeWindow(QMainWindow* window)
{
#ifdef Q_OS_WIN
    SendMessageW(HWND(window->winId()), WM_SYSCOMMAND, SC_CLOSE, 0);
#else
    window->close();
#endif
}

#ifdef Q_OS_WIN
void enableNativeWindowBehavior(QMainWindow* window)
{
    HWND hwnd = HWND(window->winId());
    const LONG_PTR style = GetWindowLongPtrW(hwnd, GWL_STYLE);
    SetWindowLongPtrW(hwnd, GWL_STYLE,
                      (style & ~WS_CAPTION) | WS_THICKFRAME | WS_SYSMENU | WS_MINIMIZEBOX | WS_MAXIMIZEBOX);
    SetWindowPos(hwnd, nullptr, 0, 0, 0, 0,
                 SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE | SWP_FRAMECHANGED);
}

bool isWindowMaximized(QMainWindow* window)
{
    return IsZoomed(HWND(window->winId()));
}
#endif

class CustomTitleBar : public QFrame
{
public:
    explicit CustomTitleBar(QMainWindow* window)
        : QFrame(window)
        , m_window(window)
    {
        setObjectName(QStringLiteral("CustomTitleBar"));
        setFixedHeight(kTitleBarHeight);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        setStyleSheet(QStringLiteral(
            "QFrame#CustomTitleBar {"
            "  background: palette(window);"
            "  border: none;"
            "  border-bottom: 1px solid palette(mid);"
            "}"
            "QLabel#WindowTitleLabel {"
            "  color: palette(window-text);"
            "  font-weight: 600;"
            "}"
            "QToolButton#WindowControlButton {"
            "  border: none;"
            "  border-radius: 0;"
            "  background: transparent;"
            "}"
            "QToolButton#WindowControlButton:hover {"
            "  background: palette(alternate-base);"
            "}"
            "QToolButton#WindowCloseButton {"
            "  border: none;"
            "  border-radius: 0;"
            "  background: transparent;"
            "}"
            "QToolButton#WindowCloseButton:hover {"
            "  background: #c42b1c;"
            "}"
        ));
    }

protected:
    void mousePressEvent(QMouseEvent* event) override
    {
        if (event->button() == Qt::LeftButton && !isInteractiveTitleBarChild(childAt(event->pos()), this)) {
#ifdef Q_OS_WIN
            POINT nativePos = {};
            GetCursorPos(&nativePos);
            ReleaseCapture();
            SendMessageW(HWND(m_window->winId()),
                         WM_NCLBUTTONDOWN,
                         HTCAPTION,
                         MAKELPARAM(nativePos.x, nativePos.y));
#else
            m_dragging = true;
            m_dragOffset = globalMousePosition(event) - m_window->frameGeometry().topLeft();
#endif
            event->accept();
            return;
        }
        QFrame::mousePressEvent(event);
    }

    void mouseMoveEvent(QMouseEvent* event) override
    {
#ifdef Q_OS_WIN
        QFrame::mouseMoveEvent(event);
#else
        if (m_dragging && (event->buttons() & Qt::LeftButton)) {
            if (m_window->isMaximized()) {
                const double xRatio = width() > 0 ? double(event->pos().x()) / double(width()) : 0.5;
                const QSize normalSize = m_window->normalGeometry().size();
                m_window->showNormal();
                const QPoint globalPos = globalMousePosition(event);
                m_dragOffset = QPoint(qBound(0, int(normalSize.width() * xRatio), normalSize.width()),
                                      qMin(event->pos().y(), kTitleBarHeight));
                m_window->move(globalPos - m_dragOffset);
            } else {
                m_window->move(globalMousePosition(event) - m_dragOffset);
            }
            event->accept();
            return;
        }
        QFrame::mouseMoveEvent(event);
#endif
    }

    void mouseReleaseEvent(QMouseEvent* event) override
    {
        if (event->button() == Qt::LeftButton && m_dragging) {
            m_dragging = false;
            event->accept();
            return;
        }
        QFrame::mouseReleaseEvent(event);
    }

    void mouseDoubleClickEvent(QMouseEvent* event) override
    {
        if (event->button() == Qt::LeftButton && !isInteractiveTitleBarChild(childAt(event->pos()), this)) {
            toggleMaximizedWindow(m_window);
            event->accept();
            return;
        }
        QFrame::mouseDoubleClickEvent(event);
    }

private:
    QMainWindow* m_window = nullptr;
    QPoint m_dragOffset;
    bool m_dragging = false;
};

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

QToolButton* createWindowControlButton(QWidget* parent, const QString& iconPath, const QString& tooltip)
{
    QToolButton* button = new QToolButton(parent);
    button->setAutoRaise(true);
    button->setCursor(Qt::PointingHandCursor);
    button->setFocusPolicy(Qt::NoFocus);
    button->setFixedSize(kWindowControlButtonWidth, kWindowControlButtonHeight);
    button->setIconSize(QSize(14, 14));
    ThemeIconUtils::setThemedSvgIcon(button, iconPath);
    button->setToolTip(tooltip);
    button->setObjectName(QStringLiteral("WindowControlButton"));
    return button;
}

} // namespace

QWidget* LivoxViewerWindow::createCustomTitleBar(QWidget* panelControls)
{
    CustomTitleBar* titleBar = new CustomTitleBar(this);
    QHBoxLayout* layout = new QHBoxLayout(titleBar);
    layout->setContentsMargins(8, 0, 0, 0);
    layout->setSpacing(0);

    menuBar->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    layout->addWidget(menuBar, 0, Qt::AlignVCenter);
    layout->addStretch(1);

    windowTitleLabel = new QLabel(titleBar);
    windowTitleLabel->setObjectName(QStringLiteral("WindowTitleLabel"));
    windowTitleLabel->setAlignment(Qt::AlignCenter);
    windowTitleLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    layout->addWidget(windowTitleLabel, 0, Qt::AlignVCenter);
    layout->addStretch(1);

    layout->addWidget(panelControls, 0, Qt::AlignVCenter);

    QToolButton* minimizeButton = createWindowControlButton(
        titleBar,
        QStringLiteral(":/icons/window_minimize.svg"),
        QStringLiteral("Minimize"));
    maximizeRestoreButton = createWindowControlButton(
        titleBar,
        QStringLiteral(":/icons/window_maximize.svg"),
        QStringLiteral("Maximize"));
    QToolButton* closeButton = createWindowControlButton(
        titleBar,
        QStringLiteral(":/icons/window_close.svg"),
        QStringLiteral("Close"));
    closeButton->setObjectName(QStringLiteral("WindowCloseButton"));

    layout->addWidget(minimizeButton);
    layout->addWidget(maximizeRestoreButton);
    layout->addWidget(closeButton);

    connect(minimizeButton, &QToolButton::clicked, this, [this]() {
        minimizeWindow(this);
    });
    connect(maximizeRestoreButton, &QToolButton::clicked, this, [this]() {
        toggleMaximizedWindow(this);
    });
    connect(closeButton, &QToolButton::clicked, this, [this]() {
        closeWindow(this);
    });

    updateWindowControlButtons();
    return titleBar;
}

void LivoxViewerWindow::updateWindowControlButtons()
{
    if (windowTitleLabel) {
        const QString title = windowTitle().isEmpty()
            ? QApplication::applicationDisplayName()
            : windowTitle();
        windowTitleLabel->setText(title);
    }
    if (maximizeRestoreButton) {
#ifdef Q_OS_WIN
        const bool maximized = isWindowMaximized(this);
#else
        const bool maximized = isMaximized();
#endif
        ThemeIconUtils::setThemedSvgIcon(maximizeRestoreButton,
            maximized ? QStringLiteral(":/icons/window_restore.svg")
                      : QStringLiteral(":/icons/window_maximize.svg"));
        maximizeRestoreButton->setToolTip(maximized ? QStringLiteral("Restore") : QStringLiteral("Maximize"));
    }
}

void LivoxViewerWindow::changeEvent(QEvent* event)
{
    QMainWindow::changeEvent(event);
    if (event->type() == QEvent::WindowStateChange ||
        event->type() == QEvent::WindowTitleChange ||
        event->type() == QEvent::StyleChange ||
        event->type() == QEvent::PaletteChange ||
        event->type() == QEvent::ApplicationPaletteChange) {
        updateWindowControlButtons();
    }
}

#ifdef Q_OS_WIN
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
bool LivoxViewerWindow::nativeEvent(const QByteArray& eventType, void* message, qintptr* result)
#else
bool LivoxViewerWindow::nativeEvent(const QByteArray& eventType, void* message, long* result)
#endif
{
    Q_UNUSED(eventType)
    MSG* msg = static_cast<MSG*>(message);
    if (msg->message == WM_GETMINMAXINFO) {
        MINMAXINFO* info = reinterpret_cast<MINMAXINFO*>(msg->lParam);
        MONITORINFO monitorInfo = {};
        monitorInfo.cbSize = sizeof(monitorInfo);
        GetMonitorInfoW(MonitorFromWindow(msg->hwnd, MONITOR_DEFAULTTONEAREST), &monitorInfo);
        const RECT work = monitorInfo.rcWork;
        const RECT monitor = monitorInfo.rcMonitor;
        info->ptMaxPosition.x = work.left - monitor.left;
        info->ptMaxPosition.y = work.top - monitor.top;
        info->ptMaxSize.x = work.right - work.left;
        info->ptMaxSize.y = work.bottom - work.top;
        *result = 0;
        return true;
    }

    if (msg->message == WM_NCCALCSIZE) {
        if (msg->wParam && IsZoomed(msg->hwnd)) {
            NCCALCSIZE_PARAMS* params = reinterpret_cast<NCCALCSIZE_PARAMS*>(msg->lParam);
            MONITORINFO monitorInfo = {};
            monitorInfo.cbSize = sizeof(monitorInfo);
            GetMonitorInfoW(MonitorFromWindow(msg->hwnd, MONITOR_DEFAULTTONEAREST), &monitorInfo);
            params->rgrc[0] = monitorInfo.rcWork;
        }
        *result = 0;
        return true;
    }

    if (msg->message == WM_NCHITTEST) {
        const QPoint nativeGlobalPos(GET_X_LPARAM(msg->lParam), GET_Y_LPARAM(msg->lParam));
        const QPoint qtGlobalPos = QCursor::pos();
        bool titleBarHit = false;
        bool interactiveTitleBarHit = false;
        if (customTitleBar) {
            const QPoint titlePos = customTitleBar->mapFromGlobal(qtGlobalPos);
            titleBarHit = customTitleBar->rect().contains(titlePos);
            interactiveTitleBarHit = titleBarHit &&
                isInteractiveTitleBarChild(customTitleBar->childAt(titlePos), customTitleBar);
        }

        RECT windowRect = {};
        GetWindowRect(msg->hwnd, &windowRect);
        const int nativeX = nativeGlobalPos.x() - windowRect.left;
        const int nativeY = nativeGlobalPos.y() - windowRect.top;
        const int nativeWidth = windowRect.right - windowRect.left;
        const int nativeHeight = windowRect.bottom - windowRect.top;
        const int topResizeWidth = titleBarHit ? kTitleBarTopResizeBorderWidth : kResizeBorderWidth;
        const bool left = nativeX >= 0 && nativeX < kResizeBorderWidth;
        const bool right = nativeX < nativeWidth && nativeX >= nativeWidth - kResizeBorderWidth;
        const bool top = nativeY >= 0 && nativeY < topResizeWidth;
        const bool bottom = nativeY < nativeHeight && nativeY >= nativeHeight - kResizeBorderWidth;

        if (!isWindowMaximized(this) && !isFullScreen()) {
            if (top && left) {
                *result = HTTOPLEFT;
                return true;
            }
            if (top && right) {
                *result = HTTOPRIGHT;
                return true;
            }
            if (bottom && left) {
                *result = HTBOTTOMLEFT;
                return true;
            }
            if (bottom && right) {
                *result = HTBOTTOMRIGHT;
                return true;
            }
            if (left) {
                *result = HTLEFT;
                return true;
            }
            if (right) {
                *result = HTRIGHT;
                return true;
            }
            if (top) {
                *result = HTTOP;
                return true;
            }
            if (bottom) {
                *result = HTBOTTOM;
                return true;
            }
        }

        if (titleBarHit && !interactiveTitleBarHit) {
            *result = HTCAPTION;
            return true;
        }
    }
    return QMainWindow::nativeEvent(eventType, message, result);
}
#endif

void LivoxViewerWindow::createMenusAndActions()
{
    // 顶部工具栏
    actionClearCloud = new QAction("清除点云", this);
    actionResetView = new QAction("重置视图", this);

    // 菜单栏
    menuBar = new QMenuBar(this);
    menuBar->setStyleSheet(QStringLiteral(
        "QMenuBar {"
        "  background: transparent;"
        "  border: none;"
        "  padding: 0;"
        "  spacing: 0px;"
        "}"
        "QMenuBar::item {"
        "  padding: 7px 12px 7px 12px;"
        "  border: none;"
        "  background: transparent;"
        "}"
        "QMenuBar::item:selected,"
        "QMenuBar::item:pressed {"
        "  border: none;"
        "  background: palette(alternate-base);"
        "}"
    ));
    fileMenu = menuBar->addMenu("文件");
    viewMenu = menuBar->addMenu("视图");
    deviceMenu = menuBar->addMenu("设备");
    QMenu* toolsMenu = menuBar->addMenu("工具");
    helpMenu = menuBar->addMenu("帮助");

    QWidget* panelControls = new QWidget(this);
    panelControls->setObjectName(QStringLiteral("PanelVisibilityControls"));
    QHBoxLayout* panelControlsLayout = new QHBoxLayout(panelControls);
    panelControlsLayout->setContentsMargins(0, 0, 6, 0);
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
    customTitleBar = createCustomTitleBar(panelControls);
    setMenuWidget(customTitleBar);
#ifdef Q_OS_WIN
    enableNativeWindowBehavior(this);
    QTimer::singleShot(0, this, [this]() {
        enableNativeWindowBehavior(this);
    });
#endif

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

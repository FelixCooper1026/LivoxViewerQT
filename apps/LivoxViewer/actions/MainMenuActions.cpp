#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"

#include <QAbstractButton>
#include <QActionEvent>
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
#include <QIcon>
#include <QInputDialog>
#include <QLabel>
#include <QPainter>
#include <QRadioButton>
#include <QResizeEvent>
#include <QShowEvent>
#include <QStandardPaths>
#include <QStyle>
#include <QStyleOption>
#include <QSvgRenderer>
#include <QTimer>
#include <QToolButton>
#include <QUrl>
#include <QWindow>

#include <algorithm>
#include <cstdlib>

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
constexpr int kTitleBarLeftMargin = 2;
constexpr int kTitleBarAppIconAreaWidth = 30;
constexpr int kTitleBarAppIconSize = 24;
constexpr int kWindowControlButtonWidth = 46;
constexpr int kWindowControlButtonHeight = 32;
constexpr int kResizeBorderWidth = 8;
constexpr int kTitleBarTopResizeBorderWidth = 2;
constexpr unsigned int kWmNcUahDrawCaption = 0x00AE;
constexpr unsigned int kWmNcUahDrawFrame = 0x00AF;

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

QPixmap svgPixmapWithColor(const QString& iconPath, const QSize& size, const QColor& color)
{
    QSvgRenderer renderer(iconPath);
    const qreal dpr = ThemeIconUtils::devicePixelRatio();
    const QSize physicalSize(qMax(1, qRound(size.width() * dpr)),
                             qMax(1, qRound(size.height() * dpr)));
    QPixmap pixmap(physicalSize);
    pixmap.setDevicePixelRatio(dpr);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    renderer.render(&painter, QRectF(QPointF(0, 0), QSizeF(size)));
    painter.setCompositionMode(QPainter::CompositionMode_SourceIn);
    painter.fillRect(QRectF(QPointF(0, 0), QSizeF(size)), color);
    return pixmap;
}

QIcon svgIconWithColor(const QString& iconPath, const QColor& color)
{
    QIcon icon;
    for (int size = 12; size <= 64; ++size) {
        icon.addPixmap(svgPixmapWithColor(iconPath, QSize(size, size), color));
    }
    return icon;
}

class CloseButtonHoverIconFilter : public QObject
{
public:
    explicit CloseButtonHoverIconFilter(QObject* parent = nullptr)
        : QObject(parent)
    {
    }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        QAbstractButton* button = qobject_cast<QAbstractButton*>(watched);
        if (!button) {
            return QObject::eventFilter(watched, event);
        }

        if (event->type() == QEvent::Enter) {
            button->setIcon(svgIconWithColor(QStringLiteral(":/icons/window_close.svg"), Qt::white));
        } else if (event->type() == QEvent::Leave) {
            ThemeIconUtils::setThemedSvgIcon(button, QStringLiteral(":/icons/window_close.svg"));
        }

        return QObject::eventFilter(watched, event);
    }
};

void positionWindowTitleLabel(QWidget* titleBar, QLabel* titleLabel)
{
    const QSize labelSize = titleLabel->sizeHint();
    const QMargins margins = titleBar->layout()->contentsMargins();
    const int contentTop = margins.top();
    const int contentHeight = titleBar->height() - contentTop;
    titleLabel->setGeometry((titleBar->width() - labelSize.width()) / 2,
                            contentTop + (contentHeight - labelSize.height()) / 2,
                            labelSize.width(),
                            labelSize.height());
    titleLabel->raise();
}

class CompactMenuBar : public QMenuBar
{
public:
    explicit CompactMenuBar(QWidget* parent = nullptr)
        : QMenuBar(parent)
    {
    }

    void refreshFixedWidth()
    {
        setFixedWidth(fullMenuWidth());
        updateGeometry();
    }

    QSize sizeHint() const override
    {
        QSize hint = QMenuBar::sizeHint();
        hint.setWidth(fullMenuWidth());
        return hint;
    }

    QSize minimumSizeHint() const override
    {
        QSize hint = QMenuBar::minimumSizeHint();
        hint.setWidth(fullMenuWidth());
        return hint;
    }

protected:
    void showEvent(QShowEvent* event) override
    {
        QMenuBar::showEvent(event);
        scheduleFixedWidthRefresh();
    }

    void actionEvent(QActionEvent* event) override
    {
        QMenuBar::actionEvent(event);
        scheduleFixedWidthRefresh();
    }

private:
    void scheduleFixedWidthRefresh()
    {
        if (m_widthRefreshPending) {
            return;
        }
        m_widthRefreshPending = true;
        QTimer::singleShot(0, this, [this]() {
            m_widthRefreshPending = false;
            refreshFixedWidth();
        });
    }

    int fullMenuWidth() const
    {
        int width = 0;
        for (QAction* action : actions()) {
            if (!action->isVisible()) {
                continue;
            }
            QString text = action->text();
            text.replace(QStringLiteral("&&"), QStringLiteral("&"));
            text.remove(QLatin1Char('&'));

            QStyleOptionMenuItem option;
            initStyleOption(&option, action);
            width += style()->sizeFromContents(
                QStyle::CT_MenuBarItem,
                &option,
                QSize(fontMetrics().horizontalAdvance(text), fontMetrics().height()),
                this).width();
        }
        return width;
    }

    bool m_widthRefreshPending = false;
};

void minimizeWindow(QMainWindow* window)
{
#ifdef Q_OS_WIN
    PostMessageW(HWND(window->winId()), WM_SYSCOMMAND, SC_MINIMIZE, 0);
#else
    window->showMinimized();
#endif
}

void toggleMaximizedWindow(QMainWindow* window)
{
#ifdef Q_OS_WIN
    HWND hwnd = HWND(window->winId());
    PostMessageW(hwnd, WM_SYSCOMMAND, IsZoomed(hwnd) ? SC_RESTORE : SC_MAXIMIZE, 0);
#else
    window->isMaximized() ? window->showNormal() : window->showMaximized();
#endif
}

void closeWindow(QMainWindow* window)
{
#ifdef Q_OS_WIN
    PostMessageW(HWND(window->winId()), WM_SYSCOMMAND, SC_CLOSE, 0);
#else
    window->close();
#endif
}

#ifdef Q_OS_WIN
void disableDwmWindowBorder(HWND hwnd)
{
    HMODULE dwmapi = LoadLibraryW(L"dwmapi.dll");
    if (!dwmapi) {
        return;
    }
    using DwmSetWindowAttributeFn = HRESULT(WINAPI*)(HWND, DWORD, LPCVOID, DWORD);
    DwmSetWindowAttributeFn setWindowAttribute =
        reinterpret_cast<DwmSetWindowAttributeFn>(GetProcAddress(dwmapi, "DwmSetWindowAttribute"));
    if (setWindowAttribute) {
        constexpr DWORD kDwmwaBorderColor = 34;
        constexpr COLORREF kDwmwaColorNone = 0xFFFFFFFE;
        setWindowAttribute(hwnd, kDwmwaBorderColor, &kDwmwaColorNone, sizeof(kDwmwaColorNone));
    }
    FreeLibrary(dwmapi);
}

bool isWindowMaximizedNative(HWND hwnd);

void enableNativeWindowBehavior(QMainWindow* window)
{
    HWND hwnd = HWND(window->winId());
    const LONG_PTR style = GetWindowLongPtrW(hwnd, GWL_STYLE);
    // 保留 WS_CAPTION 以支持 Snap、系统标题栏拖拽等原生行为
    SetWindowLongPtrW(hwnd, GWL_STYLE,
                      style | WS_CAPTION | WS_THICKFRAME | WS_SYSMENU | WS_MINIMIZEBOX | WS_MAXIMIZEBOX);
    disableDwmWindowBorder(hwnd);
    SetWindowPos(hwnd, nullptr, 0, 0, 0, 0,
                 SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE | SWP_FRAMECHANGED);
}

bool isWindowMaximized(QMainWindow* window)
{
    return isWindowMaximizedNative(HWND(window->winId()));
}

bool isWindowMaximizedNative(HWND hwnd)
{
    const LONG_PTR style = GetWindowLongPtrW(hwnd, GWL_STYLE);
    WINDOWPLACEMENT placement = {};
    placement.length = sizeof(placement);
    GetWindowPlacement(hwnd, &placement);
    return IsZoomed(hwnd) ||
           (style & WS_MAXIMIZE) ||
           placement.showCmd == SW_MAXIMIZE;
}

int systemMetricForWindow(HWND hwnd, int metric)
{
    HMODULE user32 = GetModuleHandleW(L"user32.dll");
    using GetDpiForWindowFn = UINT(WINAPI*)(HWND);
    using GetSystemMetricsForDpiFn = int(WINAPI*)(int, UINT);
    GetDpiForWindowFn getDpiForWindow =
        reinterpret_cast<GetDpiForWindowFn>(GetProcAddress(user32, "GetDpiForWindow"));
    GetSystemMetricsForDpiFn getSystemMetricsForDpi =
        reinterpret_cast<GetSystemMetricsForDpiFn>(GetProcAddress(user32, "GetSystemMetricsForDpi"));
    if (getDpiForWindow && getSystemMetricsForDpi) {
        return getSystemMetricsForDpi(metric, getDpiForWindow(hwnd));
    }
    return GetSystemMetrics(metric);
}

int nativeFrameWidth(HWND hwnd)
{
    return systemMetricForWindow(hwnd, SM_CXFRAME) + systemMetricForWindow(hwnd, SM_CXPADDEDBORDER);
}

int nativeFrameHeight(HWND hwnd)
{
    return systemMetricForWindow(hwnd, SM_CYFRAME) + systemMetricForWindow(hwnd, SM_CXPADDEDBORDER);
}

bool isMaximizedOuterFrameRect(const RECT& rect, const RECT& work, HWND hwnd)
{
    const int tolerance = 6;
    const int frameX = nativeFrameWidth(hwnd);
    const int frameY = nativeFrameHeight(hwnd);
    return std::abs(rect.left - (work.left - frameX)) <= tolerance &&
           std::abs(rect.top - (work.top - frameY)) <= tolerance &&
           std::abs(rect.right - (work.right + frameX)) <= tolerance &&
           std::abs(rect.bottom - (work.bottom + frameY)) <= tolerance;
}

bool rectCoversWorkArea(const RECT& rect, const RECT& work)
{
    const int tolerance = 6;
    return rect.left <= work.left + tolerance &&
           rect.top <= work.top + tolerance &&
           rect.right >= work.right - tolerance &&
           rect.bottom >= work.bottom - tolerance;
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
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
            if (QWindow* windowHandle = m_window->windowHandle()) {
                if (windowHandle->startSystemMove()) {
                    event->accept();
                    return;
                }
            }
#endif
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

    void resizeEvent(QResizeEvent* event) override
    {
        QFrame::resizeEvent(event);
        if (QLabel* titleLabel = findChild<QLabel*>(QStringLiteral("WindowTitleLabel"), Qt::FindDirectChildrenOnly)) {
            positionWindowTitleLabel(this, titleLabel);
        }
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
    layout->setContentsMargins(kTitleBarLeftMargin, 0, 0, 0);
    layout->setSpacing(0);

    QLabel* appIconLabel = new QLabel(titleBar);
    appIconLabel->setObjectName(QStringLiteral("TitleBarAppIcon"));
    appIconLabel->setFixedSize(kTitleBarAppIconAreaWidth, kTitleBarHeight);
    appIconLabel->setAlignment(Qt::AlignCenter);
    appIconLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    const QIcon appIcon = QApplication::windowIcon().isNull()
        ? QIcon(QStringLiteral(":/icons/app_icon.png"))
        : QApplication::windowIcon();
    appIconLabel->setPixmap(appIcon.pixmap(QSize(kTitleBarAppIconSize, kTitleBarAppIconSize)));
    layout->addWidget(appIconLabel, 0, Qt::AlignVCenter);

    menuBar->setNativeMenuBar(false);
    static_cast<CompactMenuBar*>(menuBar)->refreshFixedWidth();
    menuBar->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    layout->addWidget(menuBar, 0, Qt::AlignVCenter);
    layout->addStretch(1);

    windowTitleLabel = new QLabel(titleBar);
    windowTitleLabel->setObjectName(QStringLiteral("WindowTitleLabel"));
    windowTitleLabel->setAlignment(Qt::AlignCenter);
    windowTitleLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    windowTitleLabel->setAttribute(Qt::WA_TransparentForMouseEvents);

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
    closeButton->installEventFilter(new CloseButtonHoverIconFilter(closeButton));

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
    updateCustomTitleBarInsets();
    return titleBar;
}

void LivoxViewerWindow::updateMenuOverflow()
{
    if (!menuBar || !menuOverflowAction) {
        return;
    }

    for (QAction* action : menuBar->actions()) {
        if (action != menuOverflowAction) {
            action->setVisible(true);
        }
    }
    menuOverflowAction->setVisible(false);
    static_cast<CompactMenuBar*>(menuBar)->refreshFixedWidth();
}

void LivoxViewerWindow::rebuildMenuOverflow()
{
    if (!menuBar || !menuOverflowMenu || !menuOverflowAction) {
        return;
    }

    menuOverflowMenu->clear();
    for (QAction* action : menuBar->actions()) {
        if (action == menuOverflowAction || action->isVisible()) {
            continue;
        }

        QMenu* sourceMenu = action->menu();
        if (!sourceMenu) {
            continue;
        }

        QMenu* overflowSubMenu = menuOverflowMenu->addMenu(sourceMenu->title());
        for (QAction* sourceAction : sourceMenu->actions()) {
            overflowSubMenu->addAction(sourceAction);
        }
    }
}

QMenu* LivoxViewerWindow::createPopupMenu()
{
    QMenu* menu = new QMenu(this);
    menu->addAction(lidarDevicesDock->toggleViewAction());
    menu->addAction(paramsDock->toggleViewAction());
    menu->addAction(imuDock->toggleViewAction());
    menu->addAction(lvx2FileDock->toggleViewAction());
    menu->addAction(logDock->toggleViewAction());
    menu->addAction(attrDock->toggleViewAction());
    return menu;
}

void LivoxViewerWindow::updateWindowControlButtons()
{
    if (windowTitleLabel) {
        const QString title = windowTitle().isEmpty()
            ? QApplication::applicationDisplayName()
            : windowTitle();
        windowTitleLabel->setText(title);
        windowTitleLabel->adjustSize();
        if (customTitleBar) {
            positionWindowTitleLabel(customTitleBar, windowTitleLabel);
        }
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

void LivoxViewerWindow::updateCustomTitleBarInsets()
{
    if (!customTitleBar) {
        return;
    }

    // 始终不为标题栏添加额外内边距，保持坐标一致
    const int topInset = 0;

    customTitleBar->setFixedHeight(kTitleBarHeight + topInset);
    if (QLayout* layout = customTitleBar->layout()) {
        layout->setContentsMargins(kTitleBarLeftMargin, topInset, 0, 0);
    }
    if (windowTitleLabel) {
        positionWindowTitleLabel(customTitleBar, windowTitleLabel);
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
        updateCustomTitleBarInsets();
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
    if (msg->message == WM_NCACTIVATE) {
        *result = TRUE;
        return true;
    }

    if (msg->message == WM_NCPAINT ||
        msg->message == kWmNcUahDrawCaption ||
        msg->message == kWmNcUahDrawFrame) {
        *result = 0;
        return true;
    }

    if (msg->message == WM_SIZE ||
        msg->message == WM_WINDOWPOSCHANGED) {
        updateCustomTitleBarInsets();
    }

    if (msg->message == WM_GETMINMAXINFO) {
        MINMAXINFO* info = reinterpret_cast<MINMAXINFO*>(msg->lParam);
        MONITORINFO monitorInfo = {};
        monitorInfo.cbSize = sizeof(monitorInfo);
        GetMonitorInfoW(MonitorFromWindow(msg->hwnd, MONITOR_DEFAULTTONEAREST), &monitorInfo);
        const RECT work = monitorInfo.rcWork;
        const RECT monitor = monitorInfo.rcMonitor;
        const int frameX = nativeFrameWidth(msg->hwnd);
        const int frameY = nativeFrameHeight(msg->hwnd);
        info->ptMaxPosition.x = work.left - monitor.left - frameX;
        info->ptMaxPosition.y = work.top - monitor.top - frameY;
        info->ptMaxSize.x = work.right - work.left + frameX * 2;
        info->ptMaxSize.y = work.bottom - work.top + frameY * 2;
        *result = 0;
        return true;
    }

    if (msg->message == WM_NCCALCSIZE) {
        if (msg->wParam) {
            NCCALCSIZE_PARAMS* params = reinterpret_cast<NCCALCSIZE_PARAMS*>(msg->lParam);
            const RECT proposed = params->rgrc[0];
            MONITORINFO monitorInfo = {};
            monitorInfo.cbSize = sizeof(monitorInfo);
            GetMonitorInfoW(MonitorFromRect(&proposed, MONITOR_DEFAULTTONEAREST), &monitorInfo);
            const bool nativeMaximized = isWindowMaximizedNative(msg->hwnd);
            const bool matchOuter = isMaximizedOuterFrameRect(proposed, monitorInfo.rcWork, msg->hwnd);
            const bool coversWork = rectCoversWorkArea(proposed, monitorInfo.rcWork);
            if (nativeMaximized || matchOuter || coversWork) {
                params->rgrc[0].left = monitorInfo.rcWork.left;
                params->rgrc[0].top = monitorInfo.rcWork.top;
                params->rgrc[0].right = monitorInfo.rcWork.right;
                params->rgrc[0].bottom = monitorInfo.rcWork.bottom;
            }
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
    CompactMenuBar* compactMenuBar = new CompactMenuBar(this);
    menuBar = compactMenuBar;
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
    menuOverflowMenu = new QMenu(QStringLiteral("..."), menuBar);
    menuOverflowAction = menuBar->addMenu(menuOverflowMenu);
    menuOverflowAction->setVisible(false);
    connect(menuOverflowMenu, &QMenu::aboutToShow, this, &LivoxViewerWindow::rebuildMenuOverflow);

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
#endif

    auto syncPanelButtons = [this, leftPanelButton, bottomPanelButton, rightPanelButton]() {
        const bool leftVisible = (networkDock && networkDock->isVisible()) ||
                                 (lidarDevicesDock && lidarDevicesDock->isVisible()) ||
                                 (imuDock && imuDock->isVisible()) ||
                                 (lvx2FileDock && lvx2FileDock->isVisible()) ||
                                 (slamInfoDock && slamInfoDock->isVisible());
        const bool bottomVisible = (logDock && logDock->isVisible()) ||
                                   (slamStatusDock && slamStatusDock->isVisible());
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
                             (lvx2FileDock && lvx2FileDock->isVisible()) ||
                             (slamInfoDock && slamInfoDock->isVisible());
        networkDock->setVisible(!visible);
        lidarDevicesDock->setVisible(!visible);
        imuDock->setVisible(!visible);
        if (lvx2FileDock) {
            lvx2FileDock->setVisible(!visible && playbackState.active);
        }
        if (slamInfoDock) {
            slamInfoDock->setVisible(!visible && slamVisualizationTabId >= 0);
        }
        if (!visible) {
            if (slamInfoDock && slamVisualizationTabId >= 0) {
                slamInfoDock->raise();
            } else if (lvx2FileDock && playbackState.active) {
                lvx2FileDock->raise();
            } else {
                lidarDevicesDock->raise();
            }
        }
        syncPanelButtons();
    });
    connect(bottomPanelButton, &QToolButton::clicked, this, [this, syncPanelButtons]() {
        const bool visible = (logDock && logDock->isVisible()) ||
                             (slamStatusDock && slamStatusDock->isVisible());
        if (logDock) {
            logDock->setVisible(!visible);
        }
        if (slamStatusDock) {
            slamStatusDock->setVisible(!visible && slamUiBridge);
            if (!visible && slamStatusDock->isVisible()) {
                slamStatusDock->raise();
            }
        }
        syncPanelButtons();
    });
    connect(rightPanelButton, &QToolButton::clicked, this, [this, syncPanelButtons]() {
        const bool visible = (paramsDock && paramsDock->isVisible()) ||
                             (attrDock && attrDock->isVisible());
        if (visible) {
            restoreRightParamsDock = paramsDock->isVisible();
            restoreRightAttrDock = attrDock->isVisible();
            if (activeRightDock != paramsDock && activeRightDock != attrDock) {
                activeRightDock = restoreRightAttrDock ? attrDock : paramsDock;
            }
            paramsDock->hide();
            attrDock->hide();
        } else {
            paramsDock->setVisible(restoreRightParamsDock);
            attrDock->setVisible(restoreRightAttrDock);
            if (activeRightDock == attrDock && restoreRightAttrDock) {
                attrDock->raise();
            } else if (restoreRightParamsDock) {
                paramsDock->raise();
                activeRightDock = paramsDock;
            } else if (restoreRightAttrDock) {
                attrDock->raise();
                activeRightDock = attrDock;
            }
        }
        syncPanelButtons();
    });
    connect(settingsButton, &QToolButton::clicked, this, &LivoxViewerWindow::showPreferencesDialog);

    for (QDockWidget* dock : {networkDock, lidarDevicesDock, imuDock, lvx2FileDock, slamInfoDock, logDock, slamStatusDock, paramsDock, attrDock}) {
        connect(dock, &QDockWidget::visibilityChanged, panelControls, syncPanelButtons);
    }
    syncPanelButtons();

    ensureDataOperationActions();
    createFileActions();
    createDeviceActions();
    createHelpActions();
    toolsMenu->addAction(actionSlamOnline);
    toolsMenu->addAction(actionSlamOffline);
    // 视图菜单：显示/隐藏 dock
    viewMenu->addAction(lidarDevicesDock->toggleViewAction());
    viewMenu->addAction(lvx2FileDock->toggleViewAction());
    viewMenu->addAction(slamInfoDock->toggleViewAction());
    viewMenu->addAction(paramsDock->toggleViewAction());
    viewMenu->addAction(attrDock->toggleViewAction());
    viewMenu->addAction(imuDock->toggleViewAction());
    viewMenu->addAction(logDock->toggleViewAction());
    viewMenu->addAction(slamStatusDock->toggleViewAction());

    createStatusBarAndTimers();
    createPlaybackActions(toolsMenu);
    updateMenuOverflow();
}

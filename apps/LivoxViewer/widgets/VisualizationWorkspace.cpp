#include "widgets/VisualizationWorkspace.h"

#include "ThemeIconUtils.h"

#include <QButtonGroup>
#include <QEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QSplitter>
#include <QStyle>
#include <QTabBar>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>

namespace {

class VisualizationTabBar : public QTabBar
{
public:
    explicit VisualizationTabBar(QWidget* parent = nullptr)
        : QTabBar(parent)
    {
        setMouseTracking(true);
    }

protected:
    void paintEvent(QPaintEvent* event) override
    {
        QTabBar::paintEvent(event);

        QPainter painter(this);
        QColor separatorColor = palette().color(QPalette::Mid);
        separatorColor.setAlpha(170);
        QPen pen(separatorColor, 2);
        painter.setPen(pen);

        const int activeIndex = currentIndex();
        for (int index = 0; index < count() - 1; ++index) {
            if (index == activeIndex || index + 1 == activeIndex ||
                index == m_hoverIndex || index + 1 == m_hoverIndex) {
                continue;
            }

            const QRect rect = tabRect(index);
            if (!rect.isValid()) {
                continue;
            }
            const int x = rect.right();
            painter.drawLine(QPoint(x, rect.top() + 7), QPoint(x, rect.bottom() - 7));
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override
    {
        const int index = tabAt(event->pos());
        if (m_hoverIndex != index) {
            m_hoverIndex = index;
            update();
        }
        QTabBar::mouseMoveEvent(event);
    }

    void leaveEvent(QEvent* event) override
    {
        if (m_hoverIndex >= 0) {
            m_hoverIndex = -1;
            update();
        }
        QTabBar::leaveEvent(event);
    }

private:
    int m_hoverIndex = -1;
};

} // namespace

VisualizationWorkspace::VisualizationWorkspace(QWidget* parent)
    : QWidget(parent)
{
    QVBoxLayout* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    QWidget* tabRow = new QWidget(this);
    tabRow->setObjectName(QStringLiteral("VisualizationTabRow"));
    QHBoxLayout* tabLayout = new QHBoxLayout(tabRow);
    tabLayout->setContentsMargins(8, 4, 8, 0);
    tabLayout->setSpacing(6);

    m_tabBar = new VisualizationTabBar(tabRow);
    m_tabBar->setDocumentMode(true);
    m_tabBar->setExpanding(false);
    m_tabBar->setMovable(false);
    m_tabBar->setTabsClosable(true);
    m_tabBar->setElideMode(Qt::ElideMiddle);
    m_tabBar->setUsesScrollButtons(true);
    m_tabBar->setDrawBase(false);
    tabLayout->addWidget(m_tabBar, 1);

    m_singleButton = new QToolButton(tabRow);
    m_horizontalButton = new QToolButton(tabRow);
    m_verticalButton = new QToolButton(tabRow);
    QButtonGroup* splitButtonGroup = new QButtonGroup(tabRow);
    splitButtonGroup->setExclusive(true);
    for (QToolButton* button : {m_singleButton, m_horizontalButton, m_verticalButton}) {
        button->setCheckable(true);
        button->setAutoRaise(true);
        button->setIconSize(QSize(18, 18));
        splitButtonGroup->addButton(button);
        tabLayout->addWidget(button);
    }
    ThemeIconUtils::setThemedSvgIcon(m_singleButton, QStringLiteral(":/icons/layout_single.svg"));
    ThemeIconUtils::setThemedSvgIcon(m_horizontalButton, QStringLiteral(":/icons/layout_split_horizontal.svg"));
    ThemeIconUtils::setThemedSvgIcon(m_verticalButton, QStringLiteral(":/icons/layout_split_vertical.svg"));
    m_singleButton->setToolTip(QStringLiteral("单视图"));
    m_horizontalButton->setToolTip(QStringLiteral("左右分屏"));
    m_verticalButton->setToolTip(QStringLiteral("上下分屏"));

    root->addWidget(tabRow);

    m_splitter = new QSplitter(Qt::Horizontal, this);
    m_splitter->setChildrenCollapsible(false);
    for (int pane = 0; pane < 2; ++pane) {
        QFrame* frame = new QFrame(m_splitter);
        frame->setObjectName(QStringLiteral("VisualizationPane"));
        frame->setFrameShape(QFrame::NoFrame);
        frame->installEventFilter(this);
        QVBoxLayout* layout = new QVBoxLayout(frame);
        layout->setContentsMargins(2, 2, 2, 2);
        layout->setSpacing(0);
        QLabel* emptyLabel = new QLabel(QStringLiteral("选择一个标签页"), frame);
        emptyLabel->setAlignment(Qt::AlignCenter);
        emptyLabel->setObjectName(QStringLiteral("VisualizationPaneEmpty"));
        layout->addWidget(emptyLabel, 1);
        m_panes[pane] = frame;
        m_paneLayouts[pane] = layout;
        m_emptyLabels[pane] = emptyLabel;
        m_splitter->addWidget(frame);
    }
    m_panes[1]->setVisible(false);
    root->addWidget(m_splitter, 1);

    connect(m_tabBar, &QTabBar::currentChanged, this, [this](int index) {
        if (index >= 0) {
            activateTab(m_tabBar->tabData(index).toInt());
        }
    });
    connect(m_tabBar, &QTabBar::tabBarClicked, this, [this](int index) {
        if (index >= 0) {
            activateTab(m_tabBar->tabData(index).toInt());
        }
    });
    connect(m_tabBar, &QTabBar::tabCloseRequested, this, [this](int index) {
        emit tabCloseRequested(m_tabBar->tabData(index).toInt());
    });
    connect(m_singleButton, &QToolButton::clicked, this, [this]() { setSplitMode(SplitMode::Single); });
    connect(m_horizontalButton, &QToolButton::clicked, this, [this]() { setSplitMode(SplitMode::Horizontal); });
    connect(m_verticalButton, &QToolButton::clicked, this, [this]() { setSplitMode(SplitMode::Vertical); });

    setStyleSheet(QStringLiteral(
        "QWidget#VisualizationTabRow { background: palette(button); }"
        "QTabBar { background: transparent; }"
        "QTabBar::tab-bar { left: 0px; }"
        "QTabBar::tab {"
        "  min-width: 136px; max-width: 260px; min-height: 24px;"
        "  padding: 3px 10px 3px 14px;"
        "  margin-right: -1px;"
        "  border: 1px solid palette(mid);"
        "  border-bottom: none;"
        "  border-top-left-radius: 7px;"
        "  border-top-right-radius: 7px;"
        "  background: palette(window);"
        "  color: palette(window-text);"
        "}"
        "QTabBar::tab:!selected {"
        "  margin-top: 5px;"
        "  margin-bottom: 4px;"
        "  min-height: 18px;"
        "  background: palette(button);"
        "  border-color: transparent;"
        "  color: palette(mid);"
        "}"
        "QTabBar::tab:!selected:hover {"
        "  margin-top: 0px;"
        "  margin-bottom: 0px;"
        "  min-height: 24px;"
        "  background: palette(window);"
        "  border-color: transparent;"
        "  color: palette(window-text);"
        "}"
        "QTabBar::tab:selected {"
        "  margin-top: 0px;"
        "  background: palette(base);"
        "  border-color: palette(mid);"
        "}"
        "QFrame#VisualizationPane[focused=\"true\"] { border: 2px solid palette(highlight); }"
        "QFrame#VisualizationPane[focused=\"false\"] { border: 2px solid transparent; }"
        "QLabel#VisualizationPaneEmpty { color: palette(mid); }"));
    syncSplitButtons();
    focusPane(0);
}

int VisualizationWorkspace::addTab(TabKind kind, const QString& title, QWidget* widget, bool closable)
{
    const int tabId = m_nextTabId++;
    widget->setParent(this);
    widget->hide();
    installFocusFilter(widget, tabId);

    VisualizationTab tab;
    tab.id = tabId;
    tab.kind = kind;
    tab.title = title;
    tab.widget = widget;
    tab.closable = closable;
    m_tabs.push_back(tab);

    const int index = m_tabBar->addTab(title);
    m_tabBar->setTabData(index, tabId);
    if (closable) {
        QToolButton* closeButton = new QToolButton(m_tabBar);
        closeButton->setText(QStringLiteral("×"));
        closeButton->setAutoRaise(true);
        closeButton->setCursor(Qt::ArrowCursor);
        closeButton->setFixedSize(18, 18);
        closeButton->setStyleSheet(QStringLiteral(
            "QToolButton { border: none; border-radius: 9px; color: palette(mid); padding: 0px; }"
            "QToolButton:hover { background: rgba(0, 0, 0, 24); color: palette(window-text); }"));
        connect(closeButton, &QToolButton::clicked, this, [this, tabId]() {
            emit tabCloseRequested(tabId);
        });
        m_tabBar->setTabButton(index, QTabBar::RightSide, closeButton);
    } else {
        m_tabBar->setTabButton(index, QTabBar::RightSide, nullptr);
    }
    if (m_paneTabs[0] < 0) {
        activateTab(tabId);
    }
    syncSplitButtons();
    return tabId;
}

void VisualizationWorkspace::removeTab(int tabId)
{
    const int index = tabBarIndexForId(tabId);
    if (index >= 0) {
        m_tabBar->removeTab(index);
    }

    for (int pane = 0; pane < 2; ++pane) {
        if (m_paneTabs[pane] == tabId) {
            detachPaneWidget(pane);
            m_paneTabs[pane] = -1;
        }
    }
    QWidget* removedWidget = nullptr;
    for (auto it = m_tabs.begin(); it != m_tabs.end(); ++it) {
        if (it->id == tabId) {
            removedWidget = it->widget;
            m_tabs.erase(it);
            break;
        }
    }
    if (removedWidget) {
        removedWidget->deleteLater();
    }

    if (m_tabs.size() < 2) {
        setSplitMode(SplitMode::Single);
    }

    const int replacement = bestRecentTab(-1, -1);
    if (replacement >= 0) {
        activateTab(replacement);
    }
    syncTabBar();
    syncSplitButtons();
}

void VisualizationWorkspace::activateTab(int tabId)
{
    VisualizationTab* tab = tabById(tabId);
    if (!tab) {
        return;
    }

    tab->activationSerial = ++m_activationSerial;
    const int existingPane = paneForTab(tabId);
    if (existingPane >= 0) {
        focusPane(existingPane);
    } else {
        setPaneTab(m_focusedPane, tabId);
        focusPane(m_focusedPane);
    }
    syncTabBar();
}

void VisualizationWorkspace::setTabTitle(int tabId, const QString& title)
{
    if (VisualizationTab* tab = tabById(tabId)) {
        tab->title = title;
    }
    const int index = tabBarIndexForId(tabId);
    if (index >= 0) {
        m_tabBar->setTabText(index, title);
    }
}

void VisualizationWorkspace::setTabToolTip(int tabId, const QString& toolTip)
{
    const int index = tabBarIndexForId(tabId);
    if (index >= 0) {
        m_tabBar->setTabToolTip(index, toolTip);
    }
}

void VisualizationWorkspace::setSplitMode(SplitMode mode)
{
    if (m_splitMode == mode) {
        syncSplitButtons();
        return;
    }
    if (mode != SplitMode::Single && m_tabs.size() < 2) {
        syncSplitButtons();
        return;
    }
    m_splitMode = mode;
    m_splitter->setOrientation(mode == SplitMode::Vertical ? Qt::Vertical : Qt::Horizontal);
    m_panes[1]->setVisible(mode != SplitMode::Single);
    if (mode == SplitMode::Single) {
        if (m_paneTabs[1] >= 0) {
            detachPaneWidget(1);
            m_paneTabs[1] = -1;
        }
        focusPane(0);
    } else if (m_paneTabs[1] < 0) {
        const int secondary = bestRecentTab(m_paneTabs[0], -1);
        if (secondary >= 0) {
            setPaneTab(1, secondary);
        }
    }
    if (mode != SplitMode::Single) {
        focusPane(m_focusedPane);
    }
    syncSplitButtons();
    emit splitModeChanged(mode);
}

int VisualizationWorkspace::focusedTabId() const
{
    return m_paneTabs[m_focusedPane];
}

int VisualizationWorkspace::primaryTabId() const
{
    return m_paneTabs[0];
}

int VisualizationWorkspace::secondaryTabId() const
{
    return m_paneTabs[1];
}

QList<int> VisualizationWorkspace::tabIds() const
{
    QList<int> ids;
    for (const VisualizationTab& tab : m_tabs) {
        ids.push_back(tab.id);
    }
    return ids;
}

VisualizationWorkspace::TabKind VisualizationWorkspace::tabKind(int tabId) const
{
    return tabById(tabId)->kind;
}

QWidget* VisualizationWorkspace::tabWidget(int tabId) const
{
    return tabById(tabId)->widget;
}

bool VisualizationWorkspace::eventFilter(QObject* watched, QEvent* event)
{
    if (event->type() == QEvent::MouseButtonPress) {
        if (m_focusTabByObject.contains(watched)) {
            const int tabId = m_focusTabByObject.value(watched);
            const int pane = paneForTab(tabId);
            if (pane >= 0) {
                focusPane(pane);
                syncTabBar();
            }
        } else {
            for (int pane = 0; pane < 2; ++pane) {
                if (watched == m_panes[pane]) {
                    focusPane(pane);
                    syncTabBar();
                    break;
                }
            }
        }
    }
    return QWidget::eventFilter(watched, event);
}

VisualizationWorkspace::VisualizationTab* VisualizationWorkspace::tabById(int tabId)
{
    for (VisualizationTab& tab : m_tabs) {
        if (tab.id == tabId) {
            return &tab;
        }
    }
    return nullptr;
}

const VisualizationWorkspace::VisualizationTab* VisualizationWorkspace::tabById(int tabId) const
{
    for (const VisualizationTab& tab : m_tabs) {
        if (tab.id == tabId) {
            return &tab;
        }
    }
    return nullptr;
}

int VisualizationWorkspace::tabBarIndexForId(int tabId) const
{
    for (int index = 0; index < m_tabBar->count(); ++index) {
        if (m_tabBar->tabData(index).toInt() == tabId) {
            return index;
        }
    }
    return -1;
}

int VisualizationWorkspace::paneForTab(int tabId) const
{
    for (int pane = 0; pane < 2; ++pane) {
        if (m_paneTabs[pane] == tabId) {
            return pane;
        }
    }
    return -1;
}

void VisualizationWorkspace::focusPane(int pane)
{
    const int currentTab = m_paneTabs[pane];
    m_focusedPane = pane;
    for (int i = 0; i < 2; ++i) {
        m_panes[i]->setProperty("focused", m_splitMode != SplitMode::Single && i == pane);
        m_panes[i]->style()->unpolish(m_panes[i]);
        m_panes[i]->style()->polish(m_panes[i]);
        m_panes[i]->update();
    }
    if (currentTab != m_lastEmittedFocusedTabId) {
        m_lastEmittedFocusedTabId = currentTab;
        emit focusedTabChanged(currentTab);
    }
}

void VisualizationWorkspace::setPaneTab(int pane, int tabId)
{
    const int otherPane = pane == 0 ? 1 : 0;
    const int previousPaneTab = m_paneTabs[pane];
    if (m_paneTabs[otherPane] == tabId) {
        detachPaneWidget(otherPane);
        m_paneTabs[otherPane] = -1;
    }

    detachPaneWidget(pane);
    m_paneTabs[pane] = tabId;
    attachPaneWidget(pane, tabId);

    if (m_splitMode != SplitMode::Single && m_paneTabs[otherPane] < 0) {
        int replacement = previousPaneTab >= 0 && previousPaneTab != tabId
            ? previousPaneTab
            : bestRecentTab(tabId, -1);
        if (replacement >= 0) {
            m_paneTabs[otherPane] = replacement;
            attachPaneWidget(otherPane, replacement);
        }
    }
}

void VisualizationWorkspace::attachPaneWidget(int pane, int tabId)
{
    if (VisualizationTab* tab = tabById(tabId)) {
        m_emptyLabels[pane]->hide();
        m_paneLayouts[pane]->addWidget(tab->widget, 1);
        tab->widget->show();
    }
}

void VisualizationWorkspace::detachPaneWidget(int pane)
{
    if (m_paneTabs[pane] >= 0) {
        if (VisualizationTab* tab = tabById(m_paneTabs[pane])) {
            m_paneLayouts[pane]->removeWidget(tab->widget);
            tab->widget->hide();
            tab->widget->setParent(this);
        }
    }
    m_emptyLabels[pane]->show();
}

int VisualizationWorkspace::bestRecentTab(int excludedTabIdA, int excludedTabIdB) const
{
    const VisualizationTab* best = nullptr;
    for (const VisualizationTab& tab : m_tabs) {
        if (tab.id == excludedTabIdA || tab.id == excludedTabIdB) {
            continue;
        }
        if (!best || tab.activationSerial > best->activationSerial) {
            best = &tab;
        }
    }
    return best ? best->id : -1;
}

void VisualizationWorkspace::syncTabBar()
{
    const int index = tabBarIndexForId(focusedTabId());
    if (index >= 0 && m_tabBar->currentIndex() != index) {
        m_tabBar->blockSignals(true);
        m_tabBar->setCurrentIndex(index);
        m_tabBar->blockSignals(false);
    }
}

void VisualizationWorkspace::syncSplitButtons()
{
    const bool splitEnabled = m_tabs.size() >= 2;
    m_horizontalButton->setEnabled(splitEnabled);
    m_verticalButton->setEnabled(splitEnabled);
    m_singleButton->setChecked(m_splitMode == SplitMode::Single);
    m_horizontalButton->setChecked(m_splitMode == SplitMode::Horizontal);
    m_verticalButton->setChecked(m_splitMode == SplitMode::Vertical);
}

void VisualizationWorkspace::installFocusFilter(QWidget* widget, int tabId)
{
    m_focusTabByObject.insert(widget, tabId);
    widget->installEventFilter(this);
    for (QObject* child : widget->children()) {
        if (QWidget* childWidget = qobject_cast<QWidget*>(child)) {
            installFocusFilter(childWidget, tabId);
        }
    }
}

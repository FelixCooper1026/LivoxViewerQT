#ifndef LIVOXVIEWER_WIDGETS_VISUALIZATIONWORKSPACE_H
#define LIVOXVIEWER_WIDGETS_VISUALIZATIONWORKSPACE_H

#include <QList>
#include <QMap>
#include <QWidget>

class QHBoxLayout;
class QLabel;
class QSplitter;
class QTabBar;
class QToolButton;
class QVBoxLayout;

class VisualizationWorkspace : public QWidget
{
    Q_OBJECT

public:
    enum class TabKind {
        RealtimePointCloud,
        OfflinePointCloud
    };

    enum class SplitMode {
        Single,
        Horizontal,
        Vertical
    };

    explicit VisualizationWorkspace(QWidget* parent = nullptr);

    int addTab(TabKind kind, const QString& title, QWidget* widget, bool closable);
    void removeTab(int tabId);
    void activateTab(int tabId);
    void setTabTitle(int tabId, const QString& title);
    void setTabToolTip(int tabId, const QString& toolTip);
    void setSplitMode(SplitMode mode);

    int focusedTabId() const;
    int primaryTabId() const;
    int secondaryTabId() const;
    QList<int> tabIds() const;
    TabKind tabKind(int tabId) const;
    QWidget* tabWidget(int tabId) const;
    SplitMode splitMode() const { return m_splitMode; }

signals:
    void focusedTabChanged(int tabId);
    void tabCloseRequested(int tabId);
    void splitModeChanged(VisualizationWorkspace::SplitMode mode);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    struct VisualizationTab {
        int id = -1;
        TabKind kind = TabKind::RealtimePointCloud;
        QString title;
        QWidget* widget = nullptr;
        bool closable = true;
        int activationSerial = 0;
    };

    VisualizationTab* tabById(int tabId);
    const VisualizationTab* tabById(int tabId) const;
    int tabBarIndexForId(int tabId) const;
    int paneForTab(int tabId) const;
    void focusPane(int pane);
    void setPaneTab(int pane, int tabId);
    void attachPaneWidget(int pane, int tabId);
    void detachPaneWidget(int pane);
    int bestRecentTab(int excludedTabIdA, int excludedTabIdB) const;
    void syncTabBar();
    void syncSplitButtons();
    void installFocusFilter(QWidget* widget, int tabId);

    QTabBar* m_tabBar = nullptr;
    QToolButton* m_singleButton = nullptr;
    QToolButton* m_horizontalButton = nullptr;
    QToolButton* m_verticalButton = nullptr;
    QSplitter* m_splitter = nullptr;
    QWidget* m_panes[2] = {nullptr, nullptr};
    QVBoxLayout* m_paneLayouts[2] = {nullptr, nullptr};
    QLabel* m_emptyLabels[2] = {nullptr, nullptr};
    QList<VisualizationTab> m_tabs;
    QMap<QObject*, int> m_focusTabByObject;
    int m_nextTabId = 1;
    int m_activationSerial = 0;
    int m_focusedPane = 0;
    int m_lastEmittedFocusedTabId = -1;
    int m_paneTabs[2] = {-1, -1};
    SplitMode m_splitMode = SplitMode::Single;
};

#endif // LIVOXVIEWER_WIDGETS_VISUALIZATIONWORKSPACE_H

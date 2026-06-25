#ifndef LIVOXVIEWER_DIALOGS_IMUVISUALIZATIONDIALOG_H
#define LIVOXVIEWER_DIALOGS_IMUVISUALIZATIONDIALOG_H

#include "state/ImuRuntimeState.h"

#include <QDialog>
#include <QMap>
#include <QPoint>

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <cstdint>

class ImuOrientationView;
class QComboBox;
class QLabel;
class QEvent;
class QFrame;
class LivoxViewerWindow;
class QListWidget;
class QPushButton;
class QSlider;
class QTimer;
class QToolButton;
class SwitchCheckBox;
class QWidget;

class ImuVisualizationDialog : public QDialog
{
public:
    explicit ImuVisualizationDialog(LivoxViewerWindow* owner, bool embedded = false);
    void refreshTheme();

protected:
    void changeEvent(QEvent* event) override;
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    struct ChartPanel {
        QWidget* panel = nullptr;
        QWidget* emptyOverlay = nullptr;
        QLabel* emptyIcon = nullptr;
        QLabel* emptyLabel = nullptr;
        QChart* chart = nullptr;
        QChartView* view = nullptr;
        QLineSeries* seriesX = nullptr;
        QLineSeries* seriesY = nullptr;
        QLineSeries* seriesZ = nullptr;
        QWidget* hoverOverlay = nullptr;
        QLabel* hoverLabel = nullptr;
        QValueAxis* axisX = nullptr;
        QValueAxis* axisY = nullptr;
        QVector<ImuVisualizationSample> hoverSamples;
        double ImuVisualizationSample::*hoverXField = nullptr;
        double ImuVisualizationSample::*hoverYField = nullptr;
        double ImuVisualizationSample::*hoverZField = nullptr;
        QString hoverUnit;
        QPoint hoverMousePos;
        double defaultMin = 0.0;
        double defaultMax = 1.0;
        bool hoverActive = false;
    };

    QWidget* createToolbar();
    QWidget* createPlaybackControls();
    ChartPanel createChartPanel(const QString& title, const QString& yTitle, double defaultMin, double defaultMax);
    QWidget* createOrientationPanel();
    QWidget* createRealtimeValuesPanel();
    QWidget* createDeviceCard(const ImuVisualizationDeviceDescriptor& device);
    void refreshPlaybackControls();
    void setPinned(bool pinned);
    void refreshDeviceList();
    void updateDeviceCardSelection();
    void updateOrientationModel();
    void updateRealtimeValues(const ImuVisualizationSample* sample);
    void refreshData(bool force = false);
    void resetZoom();
    void setPaused(bool paused);
    void clearChart(ChartPanel& panel);
    void updateChart(ChartPanel& panel,
                     const QVector<ImuVisualizationSample>& samples,
                     double ImuVisualizationSample::*xField,
                     double ImuVisualizationSample::*yField,
                     double ImuVisualizationSample::*zField);
    void refreshChartTheme(ChartPanel& panel);
    ChartPanel* chartPanelForViewport(QObject* watched);
    void updateChartHover(ChartPanel& panel, const QPoint& mousePos);
    void hideChartHover(ChartPanel& panel);
    void positionChartEmptyOverlay(ChartPanel& panel);
    void positionChartEmptyOverlays();

    LivoxViewerWindow* m_owner = nullptr;
    bool m_embedded = false;
    QListWidget* m_deviceList = nullptr;
    SwitchCheckBox* m_autoScaleSwitch = nullptr;
    QPushButton* m_pauseButton = nullptr;
    QToolButton* m_pinButton = nullptr;
    QFrame* m_playbackBar = nullptr;
    QPushButton* m_playPauseButton = nullptr;
    QPushButton* m_firstFrameButton = nullptr;
    QPushButton* m_prevFrameButton = nullptr;
    QPushButton* m_nextFrameButton = nullptr;
    QPushButton* m_lastFrameButton = nullptr;
    QSlider* m_progressSlider = nullptr;
    QComboBox* m_speedCombo = nullptr;
    QComboBox* m_modeCombo = nullptr;
    QLabel* m_playbackFileLabel = nullptr;
    QLabel* m_playbackTimeLabel = nullptr;
    QLabel* m_playbackFrameLabel = nullptr;
    QLabel* m_liveValueLabels[3][3] = {};
    QTimer* m_refreshTimer = nullptr;
    ChartPanel m_accChart;
    ChartPanel m_gyroChart;
    ChartPanel m_attitudeChart;
    ImuOrientationView* m_orientationView = nullptr;
    QMap<uint32_t, ImuVisualizationDeviceDescriptor> m_devicesByHandle;
    uint32_t m_currentHandle = 0;
    uint32_t m_lastSampleHandle = 0;
    uint64_t m_lastSampleRevision = 0;
    int m_deviceRefreshElapsedMs = 0;
    bool m_haveCurrentHandle = false;
    bool m_haveLastSampleSnapshot = false;
    bool m_chartsCleared = false;
    bool m_paused = false;
    bool m_updatingPlaybackControls = false;
    bool m_progressSliderDragging = false;
};

#endif // LIVOXVIEWER_DIALOGS_IMUVISUALIZATIONDIALOG_H

#ifndef LIVOXVIEWER_DIALOGS_IMUVISUALIZATIONDIALOG_H
#define LIVOXVIEWER_DIALOGS_IMUVISUALIZATIONDIALOG_H

#include "LivoxCore/LidarDeviceInfo.h"
#include "state/ImuRuntimeState.h"

#include <QDialog>
#include <QMap>

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include <cstdint>

class ImuOrientationView;
class QLabel;
class QEvent;
class LivoxViewerWindow;
class QListWidget;
class QPushButton;
class QTimer;
class SwitchCheckBox;

class ImuVisualizationDialog : public QDialog
{
public:
    explicit ImuVisualizationDialog(LivoxViewerWindow* owner);

protected:
    void changeEvent(QEvent* event) override;

private:
    struct ChartPanel {
        QWidget* panel = nullptr;
        QLabel* emptyLabel = nullptr;
        QChart* chart = nullptr;
        QChartView* view = nullptr;
        QLineSeries* seriesX = nullptr;
        QLineSeries* seriesY = nullptr;
        QLineSeries* seriesZ = nullptr;
        QValueAxis* axisX = nullptr;
        QValueAxis* axisY = nullptr;
        double defaultMin = 0.0;
        double defaultMax = 1.0;
    };

    QWidget* createToolbar();
    ChartPanel createChartPanel(const QString& title, const QString& yTitle, double defaultMin, double defaultMax);
    QWidget* createOrientationPanel();
    QWidget* createDeviceCard(const LidarDeviceInfo& device);
    void refreshDeviceList();
    void updateDeviceCardSelection();
    void updateOrientationModel();
    void refreshData();
    void resetZoom();
    void setPaused(bool paused);
    void clearChart(ChartPanel& panel);
    void updateChart(ChartPanel& panel,
                     const QVector<ImuVisualizationSample>& samples,
                     double ImuVisualizationSample::*xField,
                     double ImuVisualizationSample::*yField,
                     double ImuVisualizationSample::*zField);
    void refreshTheme();
    void refreshChartTheme(ChartPanel& panel);

    LivoxViewerWindow* m_owner = nullptr;
    QListWidget* m_deviceList = nullptr;
    SwitchCheckBox* m_autoScaleSwitch = nullptr;
    QPushButton* m_resetButton = nullptr;
    QPushButton* m_pauseButton = nullptr;
    QTimer* m_refreshTimer = nullptr;
    ChartPanel m_accChart;
    ChartPanel m_gyroChart;
    ChartPanel m_attitudeChart;
    ImuOrientationView* m_orientationView = nullptr;
    QMap<uint32_t, LidarDeviceInfo> m_devicesByHandle;
    uint32_t m_currentHandle = 0;
    bool m_haveCurrentHandle = false;
    bool m_paused = false;
    bool m_refreshingTheme = false;
};

#endif // LIVOXVIEWER_DIALOGS_IMUVISUALIZATIONDIALOG_H

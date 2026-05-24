#ifndef LIVOXVIEWER_IMURUNTIMESTATE_H
#define LIVOXVIEWER_IMURUNTIMESTATE_H

#include <QMutex>
#include <QVector>
#include <atomic>
#include <thread>

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

class QCheckBox;
class QComboBox;
class QLabel;
class QProgressBar;
class QPushButton;
class QTableWidget;
class QTimer;
class QWidget;

struct ImuSampleState
{
    float gx = 0;
    float gy = 0;
    float gz = 0;
    float ax = 0;
    float ay = 0;
    float az = 0;
    bool have = false;
};

struct ImuChartSample
{
    double timestampSec = 0.0;
    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
};

struct ImuRuntimeState
{
    QCheckBox* gpsSimulateCheck = nullptr;
    QTimer* gpsTimer = nullptr;
    QPushButton* displayButton = nullptr;
    QTableWidget* dataTable = nullptr;

    QProgressBar* gyroBarX = nullptr;
    QProgressBar* gyroBarY = nullptr;
    QProgressBar* gyroBarZ = nullptr;
    QLabel* gyroValX = nullptr;
    QLabel* gyroValY = nullptr;
    QLabel* gyroValZ = nullptr;
    QProgressBar* accBarX = nullptr;
    QProgressBar* accBarY = nullptr;
    QProgressBar* accBarZ = nullptr;
    QLabel* accValX = nullptr;
    QLabel* accValY = nullptr;
    QLabel* accValZ = nullptr;

    QChartView* gyroChartView = nullptr;
    QChart* gyroChart = nullptr;
    QLineSeries* gyroSeriesX = nullptr;
    QLineSeries* gyroSeriesY = nullptr;
    QLineSeries* gyroSeriesZ = nullptr;
    QValueAxis* gyroAxisX = nullptr;
    QValueAxis* gyroAxisY = nullptr;

    QChartView* accChartView = nullptr;
    QChart* accChart = nullptr;
    QLineSeries* accSeriesX = nullptr;
    QLineSeries* accSeriesY = nullptr;
    QLineSeries* accSeriesZ = nullptr;
    QValueAxis* accAxisX = nullptr;
    QValueAxis* accAxisY = nullptr;

    QWidget* chartWindow = nullptr;
    QTimer* chartRefreshTimer = nullptr;
    QPushButton* chartPauseButton = nullptr;
    QPushButton* chartClearButton = nullptr;
    QPushButton* chartResetButton = nullptr;
    QLabel* chartHoverLabel = nullptr;
    bool chartPaused = false;

    std::atomic_bool displayRunning{false};
    std::thread displayThread;
    QMutex sampleMutex;
    ImuSampleState latestSample;
    QMutex chartSamplesMutex;
    QVector<ImuChartSample> chartSamples;
    double chartTimeOriginSec = -1.0;

    QComboBox* serialPortCombo = nullptr;
    QCheckBox* serialEnableCheck = nullptr;
    std::atomic_bool serialRunning{false};
    std::thread serialThread;
};

#endif // LIVOXVIEWER_IMURUNTIMESTATE_H

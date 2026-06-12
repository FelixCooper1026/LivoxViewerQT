#ifndef LIVOXVIEWER_IMURUNTIMESTATE_H
#define LIVOXVIEWER_IMURUNTIMESTATE_H

#include "imu/ImuAttitudeEstimator.h"

#include <QMap>
#include <QMutex>
#include <QPointer>
#include <QQuaternion>
#include <QVector>
#include <atomic>
#include <cstdint>
#include <thread>

class QCheckBox;
class QComboBox;
class QDialog;
class QLabel;
class QProgressBar;
class QPushButton;
class QTimer;
class ImuVisualizationDialog;

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

struct ImuVisualizationSample
{
    double timestampSec = 0.0;
    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    double rollDeg = 0.0;
    double pitchDeg = 0.0;
    double yawDeg = 0.0;
    QQuaternion orientation;
};

struct ImuVisualizationDeviceState
{
    double timeOriginSec = -1.0;
    ImuAttitudeEstimator estimator;
    QVector<ImuVisualizationSample> samples;
};

struct ImuRuntimeState
{
    QCheckBox* gpsSimulateCheck = nullptr;
    QTimer* gpsTimer = nullptr;
    QPushButton* displayButton = nullptr;
    QLabel* dataValueLabels[3][2] = {};

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

    std::atomic_bool displayRunning{false};
    std::thread displayThread;
    QMutex sampleMutex;
    ImuSampleState latestSample;
    QPointer<ImuVisualizationDialog> visualizationDialog;
    QMutex visualizationMutex;
    QMap<uint32_t, ImuVisualizationDeviceState> visualizationDevices;

    QComboBox* serialPortCombo = nullptr;
    QCheckBox* serialEnableCheck = nullptr;
    QDialog* timeSyncDialog = nullptr;
    std::atomic_bool serialRunning{false};
    std::thread serialThread;
};

#endif // LIVOXVIEWER_IMURUNTIMESTATE_H

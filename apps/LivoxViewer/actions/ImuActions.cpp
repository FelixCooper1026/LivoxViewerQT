#include "LivoxViewerWindow.h"
#include "dialogs/ImuVisualizationDialog.h"

#include <QLabel>
#include <QMetaObject>
#include <QMutexLocker>
#include <QPushButton>
#include <QTextStream>

#include <chrono>
#include <thread>

namespace {

void setImuValueLabels(ImuRuntimeState& imuState, const double values[3][2])
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 2; ++c) {
            if (imuState.dataValueLabels[r][c]) {
                imuState.dataValueLabels[r][c]->setText(QString::number(values[r][c], 'f', 3));
            }
        }
    }
}

} // namespace

void LivoxViewerWindow::onImuDisplayButtonClicked()
{
    if (imuState.displayRunning.exchange(!imuState.displayRunning.load())) {
        if (imuState.displayThread.joinable()) {
            imuState.displayThread.detach();
        }
        const double values[3][2] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
        setImuValueLabels(imuState, values);
        if (imuState.displayButton) imuState.displayButton->setText(QStringLiteral("显示IMU数据"));
        return;
    }

    if (imuState.displayButton) imuState.displayButton->setText(QStringLiteral("停止IMU显示"));

    if (imuState.displayThread.joinable()) {
        imuState.displayThread.detach();
    }
    imuState.displayThread = std::thread([this]() {
        while (imuState.displayRunning.load()) {
            float gx = 0.0f;
            float gy = 0.0f;
            float gz = 0.0f;
            float ax = 0.0f;
            float ay = 0.0f;
            float az = 0.0f;
            bool have = false;
            {
                QMutexLocker lk(&imuState.sampleMutex);
                if (imuState.latestSample.have) {
                    gx = imuState.latestSample.gx;
                    gy = imuState.latestSample.gy;
                    gz = imuState.latestSample.gz;
                    ax = imuState.latestSample.ax;
                    ay = imuState.latestSample.ay;
                    az = imuState.latestSample.az;
                    have = true;
                }
            }
            if (have) {
                QMetaObject::invokeMethod(this, [this, gx, gy, gz, ax, ay, az]() {
                    const double values[3][2] = {
                        {gx, ax}, {gy, ay}, {gz, az}
                    };
                    setImuValueLabels(imuState, values);
                });
            }
            for (int i = 0; i < 10 && imuState.displayRunning.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    });
}

void LivoxViewerWindow::onActionShowImuCharts()
{
    if (imuState.visualizationDialog) {
        imuState.visualizationDialog->show();
        imuState.visualizationDialog->raise();
        imuState.visualizationDialog->activateWindow();
        return;
    }

    auto* dialog = new ImuVisualizationDialog(this);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    imuState.visualizationDialog = dialog;
    dialog->resize(1180, 760);
    dialog->show();
}

QVector<ImuVisualizationSample> LivoxViewerWindow::imuVisualizationSamplesSnapshot(uint32_t handle)
{
    QMutexLocker lk(&imuState.visualizationMutex);
    auto it = imuState.visualizationDevices.constFind(handle);
    return it == imuState.visualizationDevices.constEnd() ? QVector<ImuVisualizationSample>() : it.value().samples;
}

void LivoxViewerWindow::onActionCaptureImuTriggered()
{
    showImuCaptureDialog();
}

void LivoxViewerWindow::appendImuCsvRow(quint64 timestamp_ns, float gx, float gy, float gz, float ax, float ay, float az)
{
    QMutexLocker lk(&captureState.imuCsvMutex);
    if (!captureState.imuSaveActive || !captureState.imuCsvFile.isOpen()) return;
    QTextStream ts(&captureState.imuCsvFile);
    ts.setRealNumberNotation(QTextStream::FixedNotation);
    ts.setRealNumberPrecision(6);
    ts << timestamp_ns << ',' << gx << ',' << gy << ',' << gz << ',' << ax << ',' << ay << ',' << az << '\n';
}

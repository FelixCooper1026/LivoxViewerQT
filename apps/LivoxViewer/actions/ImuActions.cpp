#include "LivoxViewerWindow.h"
#include "dialogs/ImuVisualizationDialog.h"
#include "Pcap/PushMsgParser.h"

#include <QLabel>
#include <QMetaObject>
#include <QMutexLocker>
#include <QPushButton>
#include <QSet>
#include <QTextStream>

#include <chrono>
#include <thread>

namespace {

constexpr double kImuChartRetentionSec = 60.0;

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

QVector<ImuVisualizationDeviceDescriptor> LivoxViewerWindow::imuVisualizationDevicesSnapshot()
{
    QVector<ImuVisualizationDeviceDescriptor> descriptors;

    const QVector<LidarDeviceInfo> realtimeDevices = connectedLidarDevicesSnapshot();
    for (const LidarDeviceInfo& device : realtimeDevices) {
        ImuVisualizationDeviceDescriptor descriptor;
        descriptor.handle = device.handle;
        descriptor.modelDisplay = device.product_info;
        descriptor.serialNumber = device.sn;
        descriptor.ipAddress = device.lidar_ip;
        descriptor.source = ImuVisualizationSource::Realtime;
        descriptors.push_back(descriptor);
    }

    if (playbackState.active && playbackState.source && playbackState.source->kind() == Playback::SourceKind::Pcap) {
        QSet<uint32_t> listedLidarIds;
        for (const PlaybackDeviceInfo& device : playbackState.devices) {
            ImuVisualizationDeviceDescriptor descriptor;
            descriptor.handle = playbackState.imuHandleByLidarId.value(device.lidarId);
            descriptor.modelDisplay = device.modelDisplay.isEmpty() ? lvx2DeviceTypeToModel(device.deviceType) : device.modelDisplay;
            descriptor.serialNumber = device.lidarSn;
            descriptor.ipAddress = PushMsgParser::lidarIdToIpString(device.lidarId);
            descriptor.source = ImuVisualizationSource::Offline;
            descriptors.push_back(descriptor);
            listedLidarIds.insert(device.lidarId);
        }

        for (auto it = playbackState.imuHandleByLidarId.constBegin(); it != playbackState.imuHandleByLidarId.constEnd(); ++it) {
            if (listedLidarIds.contains(it.key())) {
                continue;
            }
            ImuVisualizationDeviceDescriptor descriptor;
            descriptor.handle = it.value();
            descriptor.modelDisplay = QStringLiteral("未知型号");
            descriptor.ipAddress = PushMsgParser::lidarIdToIpString(it.key());
            descriptor.source = ImuVisualizationSource::Offline;
            descriptors.push_back(descriptor);
        }
    }

    return descriptors;
}

void LivoxViewerWindow::clearPlaybackImuSamples()
{
    QMutexLocker lk(&imuState.visualizationMutex);
    for (uint32_t handle : playbackState.imuHandleByLidarId) {
        imuState.visualizationDevices.remove(handle);
    }
}

void LivoxViewerWindow::appendPlaybackImuSamples(const QVector<Playback::ImuSample>& samples, bool resetSamples)
{
    if (resetSamples) {
        clearPlaybackImuSamples();
    }

    QMutexLocker lk(&imuState.visualizationMutex);
    QMap<uint32_t, double> latestRelativeSecByHandle;
    for (const Playback::ImuSample& sample : samples) {
        const uint32_t handle = playbackState.playbackImuHandleForLidarId(sample.lidarId);
        ImuVisualizationDeviceState& deviceState = imuState.visualizationDevices[handle];
        const double timestampSec = static_cast<double>(sample.timestampNs) * 1.0e-9;
        if (deviceState.timeOriginSec < 0.0) {
            deviceState.timeOriginSec = timestampSec;
        }
        const double relativeSec = timestampSec - deviceState.timeOriginSec;
        const ImuAttitudeEstimator::Result attitude = deviceState.estimator.update(
            timestampSec,
            sample.gyroX, sample.gyroY, sample.gyroZ,
            sample.accX, sample.accY, sample.accZ);
        deviceState.samples.append(ImuVisualizationSample{
            relativeSec,
            sample.gyroX, sample.gyroY, sample.gyroZ,
            sample.accX, sample.accY, sample.accZ,
            attitude.rollDeg, attitude.pitchDeg, attitude.yawDeg,
            attitude.orientation
        });
        latestRelativeSecByHandle.insert(handle, relativeSec);
    }

    for (auto it = latestRelativeSecByHandle.constBegin(); it != latestRelativeSecByHandle.constEnd(); ++it) {
        ImuVisualizationDeviceState& deviceState = imuState.visualizationDevices[it.key()];
        int removeCount = 0;
        while (removeCount < deviceState.samples.size() &&
               it.value() - deviceState.samples[removeCount].timestampSec > kImuChartRetentionSec) {
            ++removeCount;
        }
        if (removeCount > 0) {
            deviceState.samples.remove(0, removeCount);
        }
    }
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

#include "LivoxViewerWindow.h"

#include "Export/PointCloudExport.h"

#include <QDir>

#include <cstring>

bool LivoxViewerWindow::savePointCloudAsLAS(const QString& filePath, const QVector<PointCloudPoint>& points)
{
    return PointCloudExport::saveAsLAS(filePath, points);
}

bool LivoxViewerWindow::savePointCloudAsPCD(const QString& filePath, const QVector<PointCloudPoint>& points)
{
    return PointCloudExport::saveAsPCD(filePath, points);
}

void LivoxViewerWindow::handlePointCloudRecording(const PointCloudFrame& merged, uint64_t timestampNs)
{
    if (captureState.pcdSaveActive && captureState.pcdFramesRemaining > 0) {
        if (captureState.pcdLastSavedTimestamp != timestampNs) {
            QString fileName = QString::number(timestampNs) + ".pcd";
            QString filePath = QDir(captureState.pcdSaveDir).filePath(fileName);
            if (savePointCloudAsPCD(filePath, merged.points)) {
                logMessage(QString("PCD保存: %1").arg(QDir::toNativeSeparators(filePath)));
                captureState.pcdLastSavedTimestamp = timestampNs;
                captureState.pcdFramesRemaining--;
                if (captureState.pcdFramesRemaining <= 0) {
                    captureState.pcdSaveActive = false;
                    statusLabelBar->setText("PCD保存完成");
                }
            } else {
                logMessage(QString("PCD保存失败: %1").arg(QDir::toNativeSeparators(filePath)));
                captureState.pcdLastSavedTimestamp = timestampNs;
                captureState.pcdFramesRemaining--;
            }
        }
    }

    if (captureState.lasSaveActive && captureState.lasFramesRemaining > 0) {
        if (captureState.lasLastSavedTimestamp != timestampNs) {
            QString fileName = QString::number(timestampNs) + ".las";
            QString filePath = QDir(captureState.lasSaveDir).filePath(fileName);
            if (savePointCloudAsLAS(filePath, merged.points)) {
                logMessage(QString("LAS保存: %1").arg(QDir::toNativeSeparators(filePath)));
                captureState.lasLastSavedTimestamp = timestampNs;
                captureState.lasFramesRemaining--;
                if (captureState.lasFramesRemaining <= 0) {
                    captureState.lasSaveActive = false;
                    statusLabelBar->setText("LAS保存完成");
                }
            } else {
                logMessage(QString("LAS保存失败: %1").arg(QDir::toNativeSeparators(filePath)));
                captureState.lasLastSavedTimestamp = timestampNs;
                captureState.lasFramesRemaining--;
            }
        }
    }
}

void LivoxViewerWindow::startLvx2Recording(const QString& filePath, int durationSec)
{
    QMutexLocker lk(&captureState.lvx2Mutex);
    if (captureState.lvx2SaveActive) return;
    captureState.lvx2File.setFileName(filePath);
    if (!captureState.lvx2File.open(QIODevice::WriteOnly)) {
        logMessage("打开LVX2文件失败");
        captureState.current = CaptureNone;
        return;
    }
    Lvx2PublicHeader pub;
    captureState.lvx2File.write(reinterpret_cast<const char*>(&pub), sizeof(pub));
    Lvx2PrivateHeader pri;
    captureState.lvx2File.write(reinterpret_cast<const char*>(&pri), sizeof(pri));
    LidarDeviceInfo currentDevice;
    const bool hasDevice = tryGetCurrentDevice(currentDevice);
    Lvx2DeviceInfo dev{};
    QByteArray snb = hasDevice ? currentDevice.sn.left(15).toLatin1() : QByteArray("Unknown");
    std::memset(dev.lidar_sn, 0, sizeof(dev.lidar_sn));
    std::memcpy(dev.lidar_sn, snb.constData(), std::min<size_t>(size_t(snb.size()), sizeof(dev.lidar_sn)));
    dev.lidar_id = hasDevice ? currentDevice.handle : 0;
    dev.device_type = hasDevice ? currentDevice.dev_type : 0;
    captureState.lvx2File.write(reinterpret_cast<const char*>(&dev), sizeof(dev));

    captureState.lvx2SaveActive = true;
    captureState.lvx2FrameStartNs = 0;
    captureState.lvx2FrameIndex = 0;
    captureState.secondsRemaining = durationSec;
    captureState.progress->setValue(0);
    captureState.progress->setFormat("录制中 %p% (%v s)");
}

void LivoxViewerWindow::stopLvx2Recording(bool flushPending)
{
    QMutexLocker lk(&captureState.lvx2Mutex);
    if (!captureState.lvx2SaveActive) return;
    Q_UNUSED(flushPending);
    captureState.lvx2SaveActive = false;
    if (captureState.lvx2File.isOpen()) captureState.lvx2File.close();
}

static void LoggerStartCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data) {
    LivoxViewerWindow* w = static_cast<LivoxViewerWindow*>(client_data);
    if (!w) return;
    if (status != kLivoxLidarStatusSuccess || response == nullptr || response->ret_code != 0) {
        LivoxLidarStartLogger(handle, kLivoxLidarRealTimeLog, LoggerStartCallback, client_data);
        return;
    }
}

static void DebugPointCloudCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data) {
    LivoxViewerWindow* w = static_cast<LivoxViewerWindow*>(client_data);
    if (!w) return;
    Q_UNUSED(status);
    Q_UNUSED(handle);
    Q_UNUSED(response);
}

void LivoxViewerWindow::onStartCaptureLog()
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) { logMessage("设备未连接"); return; }
    if (captureState.current != CaptureNone) return;
    captureState.current = CaptureLog;
    int sec = captureState.durationSpin ? captureState.durationSpin->value() : 10;
    captureState.secondsRemaining = sec;
    captureState.totalSeconds = sec;
    logMessage(QString("开始采集日志，时长: %1s").arg(sec));
    captureState.progress->setValue(0);
    captureState.progress->setFormat("LOG采集中 %p% (%v s)");
    SaveLivoxLidarSdkLoggerFile();
    LivoxLidarStartLogger(currentDevice.handle, kLivoxLidarRealTimeLog, LoggerStartCallback, this);
    captureState.timer->start(1000);
}

void LivoxViewerWindow::onStartCaptureDebug()
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) { logMessage("设备未连接"); return; }
    if (captureState.current != CaptureNone) return;
    captureState.current = CaptureDebug;
    int sec = captureState.durationSpin ? captureState.durationSpin->value() : 10;
    captureState.secondsRemaining = sec;
    captureState.totalSeconds = sec;
    logMessage(QString("开始采集Debug点云，时长: %1s").arg(sec));
    captureState.progress->setValue(0);
    captureState.progress->setFormat("Debug采集中 %p% (%v s)");
    SetLivoxLidarDebugPointCloud(currentDevice.handle, true, DebugPointCloudCallback, this);
    captureState.timer->start(1000);
}

void LivoxViewerWindow::onCaptureTick()
{
    if (captureState.secondsRemaining <= 0) {
        captureState.timer->stop();
        if (captureState.current == CaptureLog) {
            if (hasCurrentLidarHandle) LivoxLidarStopLogger(currentLidarHandle, kLivoxLidarRealTimeLog, LoggerStartCallback, this);
            logMessage("日志采集完成");
        } else if (captureState.current == CaptureDebug) {
            if (hasCurrentLidarHandle) SetLivoxLidarDebugPointCloud(currentLidarHandle, false, DebugPointCloudCallback, this);
            logMessage("Debug点云采集完成");
        } else if (captureState.current == CaptureLVX2) {
            stopLvx2Recording(true);
            logMessage("LVX2数据保存完成");
        } else if (captureState.current == CaptureIMU) {
            {
                QMutexLocker lk(&captureState.imuCsvMutex);
                if (captureState.imuCsvFile.isOpen()) captureState.imuCsvFile.flush();
                if (captureState.imuCsvFile.isOpen()) captureState.imuCsvFile.close();
            }
            captureState.imuSaveActive = false;
            logMessage("IMU数据保存完成");
        }
        if (captureState.progress) {
            captureState.progress->setValue(100);
            captureState.progress->setFormat("采集完成");
        }
        statusLabelBar->setText("已连接 - 采样中");
        captureState.current = CaptureNone;
        return;
    }

    int total = captureState.totalSeconds > 0 ? captureState.totalSeconds : (captureState.durationSpin ? captureState.durationSpin->value() : 1);
    int done = total - captureState.secondsRemaining;
    if (done < 0) done = 0;
    if (done > total) done = total;
    int percent = total > 0 ? (done * 100 / total) : 100;
    if (captureState.progress) {
        captureState.progress->setValue(percent);
        captureState.progress->setFormat(QString("%1% (%2 s)").arg(percent).arg(captureState.secondsRemaining));
    }
    captureState.secondsRemaining--;
}

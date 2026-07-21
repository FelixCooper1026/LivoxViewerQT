#include "LivoxViewerWindow.h"

#include "PointCloudExport.h"
#include "widgets/ParameterOptionButtons.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QMutexLocker>
#include <QStringList>
#include <QTextStream>
#include <QTimer>

#include <algorithm>
#include <cstring>

namespace {

QString pointCloudFormatName(PointCloudCaptureFormat format)
{
    switch (format) {
    case PointCloudCaptureFormat::LVX2: return QStringLiteral("LVX2");
    case PointCloudCaptureFormat::PCD: return QStringLiteral("PCD");
    case PointCloudCaptureFormat::LAS: return QStringLiteral("LAS");
    case PointCloudCaptureFormat::None: return QStringLiteral("PointCloud");
    }
    return QStringLiteral("PointCloud");
}

QString pointCloudOutputDirName(PointCloudCaptureFormat format, const QString& sn)
{
    return QStringLiteral("%1_%2").arg(pointCloudFormatName(format), sn);
}

bool taskRunning(const CaptureTaskState& task)
{
    return task.status == CaptureTaskStatus::Running;
}

void appendRunningTaskMessage(QStringList& messages, const CaptureTaskState& task)
{
    if (taskRunning(task)) {
        messages.append(task.message);
    }
}

int normalizedDuration(int durationSec)
{
    return std::max(1, durationSec);
}

void startTask(CaptureTaskState& task, int durationSec, const QString& outputDir, const QString& outputPath, const QString& message)
{
    task.status = CaptureTaskStatus::Running;
    task.secondsRemaining = normalizedDuration(durationSec);
    task.totalSeconds = task.secondsRemaining;
    task.savedFiles = 0;
    task.expectedFiles = 0;
    task.integrationMs = 0;
    task.outputDir = outputDir;
    task.outputPath = outputPath;
    task.message = message;
}

void finishTask(CaptureTaskState& task, const QString& message)
{
    task.status = CaptureTaskStatus::Done;
    task.secondsRemaining = 0;
    task.message = message;
}

void failTask(CaptureTaskState& task, const QString& message)
{
    task.status = CaptureTaskStatus::Failed;
    task.secondsRemaining = 0;
    task.message = message;
}

bool shouldSavePointCloudFile(CaptureSessionState& captureState, uint64_t timestampNs)
{
    if (captureState.pointCloudTask.expectedFiles > 0 &&
        captureState.pointCloudTask.savedFiles >= captureState.pointCloudTask.expectedFiles) {
        return false;
    }
    if (captureState.pointCloudNextSaveTimestamp == 0) {
        captureState.pointCloudNextSaveTimestamp = timestampNs + captureState.pointCloudSaveIntervalNs;
        return false;
    }
    if (timestampNs < captureState.pointCloudNextSaveTimestamp) {
        return false;
    }
    return true;
}

void advancePointCloudSaveTimestamp(CaptureSessionState& captureState, uint64_t timestampNs)
{
    captureState.pointCloudNextSaveTimestamp += captureState.pointCloudSaveIntervalNs;
    while (captureState.pointCloudNextSaveTimestamp <= timestampNs) {
        captureState.pointCloudNextSaveTimestamp += captureState.pointCloudSaveIntervalNs;
    }
}

void LoggerStartCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data)
{
    LivoxViewerWindow* window = static_cast<LivoxViewerWindow*>(client_data);
    if (!window) {
        return;
    }
    if (status != kLivoxLidarStatusSuccess || response == nullptr || response->ret_code != 0) {
        LivoxLidarStartLogger(handle, kLivoxLidarRealTimeLog, LoggerStartCallback, client_data);
    }
}

void DebugPointCloudCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data)
{
    Q_UNUSED(status);
    Q_UNUSED(handle);
    Q_UNUSED(response);
    Q_UNUSED(client_data);
}

void LoggerStopCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data)
{
    Q_UNUSED(status);
    Q_UNUSED(handle);
    Q_UNUSED(response);
    Q_UNUSED(client_data);
}

} // namespace

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
    if (!captureState.pcdSaveActive && !captureState.lasSaveActive) {
        return;
    }
    if (!shouldSavePointCloudFile(captureState, timestampNs)) {
        return;
    }

    bool saved = false;
    if (captureState.pcdSaveActive && captureState.pcdLastSavedTimestamp != timestampNs) {
        const QString fileName = QString::number(timestampNs) + QStringLiteral(".pcd");
        const QString filePath = QDir(captureState.pcdSaveDir).filePath(fileName);
        if (savePointCloudAsPCD(filePath, merged.points)) {
            logMessage(QString("PCD保存: %1").arg(QDir::toNativeSeparators(filePath)));
            saved = true;
        } else {
            const QString message = QString("PCD保存失败: %1").arg(QDir::toNativeSeparators(filePath));
            logMessage(message);
            captureState.pcdSaveActive = false;
            captureState.pointCloudSaveIntervalNs = 0;
            captureState.pointCloudNextSaveTimestamp = 0;
            failTask(captureState.pointCloudTask, message);
            statusLabelBar->setText(QStringLiteral("点云数据采集失败"));
            return;
        }
        captureState.pcdLastSavedTimestamp = timestampNs;
    }

    if (captureState.lasSaveActive && captureState.lasLastSavedTimestamp != timestampNs) {
        const QString fileName = QString::number(timestampNs) + QStringLiteral(".las");
        const QString filePath = QDir(captureState.lasSaveDir).filePath(fileName);
        if (savePointCloudAsLAS(filePath, merged.points)) {
            logMessage(QString("LAS保存: %1").arg(QDir::toNativeSeparators(filePath)));
            saved = true;
        } else {
            const QString message = QString("LAS保存失败: %1").arg(QDir::toNativeSeparators(filePath));
            logMessage(message);
            captureState.lasSaveActive = false;
            captureState.pointCloudSaveIntervalNs = 0;
            captureState.pointCloudNextSaveTimestamp = 0;
            failTask(captureState.pointCloudTask, message);
            statusLabelBar->setText(QStringLiteral("点云数据采集失败"));
            return;
        }
        captureState.lasLastSavedTimestamp = timestampNs;
    }

    advancePointCloudSaveTimestamp(captureState, timestampNs);
    if (saved) {
        ++captureState.pointCloudTask.savedFiles;
    }
    if (captureState.pointCloudTask.expectedFiles > 0 &&
        captureState.pointCloudTask.savedFiles >= captureState.pointCloudTask.expectedFiles) {
        stopPointCloudCapture();
    }
}

bool LivoxViewerWindow::startLvx2Recording(const QString& filePath)
{
    QMutexLocker lk(&captureState.lvx2Mutex);
    if (captureState.lvx2SaveActive) {
        return false;
    }

    captureState.lvx2File.setFileName(filePath);
    if (!captureState.lvx2File.open(QIODevice::WriteOnly)) {
        logMessage("打开LVX2文件失败");
        return false;
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

    captureState.lvx2SaveDir = QFileInfo(filePath).absolutePath();
    captureState.lvx2SaveActive = true;
    captureState.lvx2PendingPkgs.clear();
    captureState.lvx2FrameStartNs = 0;
    captureState.lvx2FrameIndex = 0;
    return true;
}

void LivoxViewerWindow::stopLvx2Recording(bool flushPending)
{
    QMutexLocker lk(&captureState.lvx2Mutex);
    if (!captureState.lvx2SaveActive) {
        return;
    }

    if (flushPending && !captureState.lvx2PendingPkgs.isEmpty() && captureState.lvx2File.isOpen()) {
        const uint64_t frameStart = captureState.lvx2File.pos();
        Lvx2FrameHeader fh{};
        captureState.lvx2File.write(reinterpret_cast<const char*>(&fh), sizeof(fh));
        for (const QByteArray& pkg : captureState.lvx2PendingPkgs) {
            captureState.lvx2File.write(pkg);
        }
        const uint64_t nextOffset = captureState.lvx2File.pos();
        fh.current_offset = frameStart;
        fh.next_offset = nextOffset;
        fh.frame_index = captureState.lvx2FrameIndex++;
        captureState.lvx2File.seek(frameStart);
        captureState.lvx2File.write(reinterpret_cast<const char*>(&fh), sizeof(fh));
        captureState.lvx2File.seek(nextOffset);
    }

    captureState.lvx2PendingPkgs.clear();
    captureState.lvx2SaveActive = false;
    if (captureState.lvx2File.isOpen()) {
        captureState.lvx2File.close();
    }
}

QString LivoxViewerWindow::debugLogOutputDir() const
{
    return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("lidar_log/type_0"));
}

QString LivoxViewerWindow::debugPointCloudOutputDir() const
{
    return QCoreApplication::applicationDirPath();
}

bool LivoxViewerWindow::startPointCloudCapture(PointCloudCaptureFormat format,
                                               const QString& baseDir,
                                               int captureAmount,
                                               QString& errorMessage)
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
        errorMessage = QStringLiteral("设备未连接");
        return false;
    }
    if (taskRunning(captureState.pointCloudTask) || taskRunning(captureState.imuTask)) {
        errorMessage = QStringLiteral("已有点云或IMU采集任务正在运行");
        return false;
    }

    const QString sn = currentDevice.sn.isEmpty() ? QStringLiteral("Unknown") : currentDevice.sn;
    const QString targetDir = QDir(baseDir).filePath(pointCloudOutputDirName(format, sn));
    QDir().mkpath(targetDir);

    QString outputPath;
    if (format == PointCloudCaptureFormat::LVX2) {
        const QString startTime = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
        outputPath = QDir(targetDir).filePath(QStringLiteral("%1_%2.lvx2").arg(sn, startTime));
        if (!startLvx2Recording(outputPath)) {
            errorMessage = QStringLiteral("无法创建LVX2文件");
            return false;
        }
    } else if (format == PointCloudCaptureFormat::PCD) {
        captureState.pcdSaveDir = targetDir;
        captureState.pcdLastSavedTimestamp = 0;
        captureState.pcdSaveActive = true;
    } else if (format == PointCloudCaptureFormat::LAS) {
        captureState.lasSaveDir = targetDir;
        captureState.lasLastSavedTimestamp = 0;
        captureState.lasSaveActive = true;
    }

    captureState.pointCloudFormat = format;
    const bool fileCapture = format == PointCloudCaptureFormat::PCD || format == PointCloudCaptureFormat::LAS;
    startTask(captureState.pointCloudTask,
              fileCapture ? 0 : captureAmount,
              targetDir,
              outputPath,
              QStringLiteral("%1采集中").arg(pointCloudFormatName(format)));
    if (fileCapture) {
        const int frameMs = static_cast<int>(frameIntervalMs);
        captureState.pointCloudTask.secondsRemaining = 0;
        captureState.pointCloudTask.totalSeconds = 0;
        captureState.pointCloudTask.expectedFiles = captureAmount;
        captureState.pointCloudTask.integrationMs = frameMs;
        captureState.pointCloudSaveIntervalNs = frameIntervalMs * 1000000ULL;
        captureState.pointCloudNextSaveTimestamp = 0;
        QMutexLocker locker(&frameMutex);
        pendingFrames.clear();
        lastSeenTimestamp.clear();
    }
    captureState.timer->start(1000);
    statusLabelBar->setText(QStringLiteral("正在采集点云数据..."));
    logMessage(QString("%1采集开始: %2")
                   .arg(pointCloudFormatName(format), QDir::toNativeSeparators(targetDir)));
    return true;
}

bool LivoxViewerWindow::startImuCapture(const QString& baseDir, int durationSec, QString& errorMessage)
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
        errorMessage = QStringLiteral("设备未连接");
        return false;
    }
    if (taskRunning(captureState.pointCloudTask) || taskRunning(captureState.imuTask)) {
        errorMessage = QStringLiteral("已有点云或IMU采集任务正在运行");
        return false;
    }
    QWidget* ctrl = parameterState.controls.value(kKeyImuDataEn, nullptr);
    if (!ParameterOptionButtons::isOptionControl(ctrl) || ParameterOptionButtons::currentIndex(ctrl) != 1) {
        errorMessage = QStringLiteral("IMU数据发送未开启");
        return false;
    }

    const QString sn = currentDevice.sn.isEmpty() ? QStringLiteral("Unknown") : currentDevice.sn;
    const QString targetDir = QDir(baseDir).filePath(QStringLiteral("IMU_%1").arg(sn));
    QDir().mkpath(targetDir);
    const QString startTime = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    const QString filePath = QDir(targetDir).filePath(QStringLiteral("%1_%2.csv").arg(sn, startTime));

    captureState.imuCsvFile.setFileName(filePath);
    if (!captureState.imuCsvFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        errorMessage = QStringLiteral("无法创建IMU CSV文件");
        return false;
    }
    {
        QMutexLocker lk(&captureState.imuCsvMutex);
        QTextStream ts(&captureState.imuCsvFile);
        ts << "timestamp_ns,gx,gy,gz,ax,ay,az\n";
    }

    captureState.imuSaveActive = true;
    startTask(captureState.imuTask, durationSec, targetDir, filePath, QStringLiteral("IMU采集中"));
    captureState.timer->start(1000);
    statusLabelBar->setText(QStringLiteral("正在采集IMU数据..."));
    logMessage(QString("IMU采集开始: %1").arg(QDir::toNativeSeparators(filePath)));
    return true;
}

bool LivoxViewerWindow::startLogCapture(int durationSec, QString& errorMessage)
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
        errorMessage = QStringLiteral("设备未连接");
        return false;
    }
    if (taskRunning(captureState.logTask)) {
        errorMessage = QStringLiteral("LOG采集正在运行");
        return false;
    }

    const QString outputDir = debugLogOutputDir();
    QDir().mkpath(outputDir);
    SaveLivoxLidarSdkLoggerFile();
    const livox_status status = LivoxLidarStartLogger(currentDevice.handle, kLivoxLidarRealTimeLog, LoggerStartCallback, this);
    if (status != kLivoxLidarStatusSuccess) {
        errorMessage = QStringLiteral("LOG采集启动失败");
        return false;
    }

    captureState.logHandle = currentDevice.handle;
    startTask(captureState.logTask, durationSec, outputDir, QString(), QStringLiteral("LOG采集中"));
    captureState.timer->start(1000);
    statusLabelBar->setText(captureState.logTask.message);
    logMessage(QString("LOG采集开始: %1").arg(QDir::toNativeSeparators(outputDir)));
    return true;
}

bool LivoxViewerWindow::startDebugPointCloudCapture(int durationSec, QString& errorMessage)
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
        errorMessage = QStringLiteral("设备未连接");
        return false;
    }
    if (taskRunning(captureState.debugTask)) {
        errorMessage = QStringLiteral("Debug点云采集正在运行");
        return false;
    }

    const QString outputDir = debugPointCloudOutputDir();
    const livox_status status = SetLivoxLidarDebugPointCloud(currentDevice.handle, true, DebugPointCloudCallback, this);
    if (status != kLivoxLidarStatusSuccess) {
        errorMessage = QStringLiteral("Debug点云采集启动失败");
        return false;
    }

    captureState.debugHandle = currentDevice.handle;
    startTask(captureState.debugTask, durationSec, outputDir, QString(), QStringLiteral("Debug点云采集中"));
    captureState.timer->start(1000);
    statusLabelBar->setText(QStringLiteral("正在采集Debug点云..."));
    logMessage(QString("Debug点云采集开始: %1").arg(QDir::toNativeSeparators(outputDir)));
    return true;
}

void LivoxViewerWindow::stopPointCloudCapture()
{
    const int savedFiles = captureState.pointCloudTask.savedFiles;
    const int expectedFiles = captureState.pointCloudTask.expectedFiles;
    if (captureState.pointCloudFormat == PointCloudCaptureFormat::LVX2) {
        stopLvx2Recording(true);
    } else if (captureState.pointCloudFormat == PointCloudCaptureFormat::PCD) {
        captureState.pcdSaveActive = false;
    } else if (captureState.pointCloudFormat == PointCloudCaptureFormat::LAS) {
        captureState.lasSaveActive = false;
    }
    captureState.pointCloudSaveIntervalNs = 0;
    captureState.pointCloudNextSaveTimestamp = 0;

    const QString formatName = pointCloudFormatName(captureState.pointCloudFormat);
    const QString message = expectedFiles > 0
        ? QStringLiteral("%1采集完成，保存 %2/%3 个文件").arg(formatName).arg(savedFiles).arg(expectedFiles)
        : QStringLiteral("%1采集完成").arg(formatName);
    finishTask(captureState.pointCloudTask, message);
    statusLabelBar->setText(QStringLiteral("点云数据采集完成"));
    logMessage(message);
}

void LivoxViewerWindow::stopImuCapture()
{
    {
        QMutexLocker lk(&captureState.imuCsvMutex);
        if (captureState.imuCsvFile.isOpen()) {
            captureState.imuCsvFile.flush();
            captureState.imuCsvFile.close();
        }
    }
    captureState.imuSaveActive = false;
    finishTask(captureState.imuTask, QStringLiteral("IMU采集完成"));
    statusLabelBar->setText(QStringLiteral("IMU数据采集完成"));
    logMessage(QStringLiteral("IMU数据采集完成"));
}

void LivoxViewerWindow::stopLogCapture()
{
    LivoxLidarStopLogger(captureState.logHandle, kLivoxLidarRealTimeLog, LoggerStopCallback, this);
    captureState.logHandle = 0;
    finishTask(captureState.logTask, QStringLiteral("LOG采集完成"));
    statusLabelBar->setText(captureState.logTask.message);
    logMessage(QStringLiteral("LOG采集完成"));
}

void LivoxViewerWindow::stopDebugPointCloudCapture()
{
    SetLivoxLidarDebugPointCloud(captureState.debugHandle, false, DebugPointCloudCallback, this);
    captureState.debugHandle = 0;
    finishTask(captureState.debugTask, QStringLiteral("Debug点云采集完成"));
    statusLabelBar->setText(QStringLiteral("Debug点云采集完成"));
    logMessage(QStringLiteral("Debug点云采集完成"));
}

void LivoxViewerWindow::onCaptureTick()
{
    if (taskRunning(captureState.pointCloudTask) &&
        captureState.pointCloudFormat == PointCloudCaptureFormat::LVX2) {
        --captureState.pointCloudTask.secondsRemaining;
        if (captureState.pointCloudTask.secondsRemaining <= 0) {
            stopPointCloudCapture();
        }
    }
    if (taskRunning(captureState.imuTask)) {
        --captureState.imuTask.secondsRemaining;
        if (captureState.imuTask.secondsRemaining <= 0) {
            stopImuCapture();
        }
    }
    if (taskRunning(captureState.parameterTask)) {
        --captureState.parameterTask.secondsRemaining;
        if (captureState.parameterTask.secondsRemaining <= 0) {
            stopParameterCapture();
        }
    }
    if (taskRunning(captureState.logTask)) {
        --captureState.logTask.secondsRemaining;
        if (captureState.logTask.secondsRemaining <= 0) {
            stopLogCapture();
        }
    }
    if (taskRunning(captureState.debugTask)) {
        --captureState.debugTask.secondsRemaining;
        if (captureState.debugTask.secondsRemaining <= 0) {
            stopDebugPointCloudCapture();
        }
    }

    const bool anyRunning =
        taskRunning(captureState.pointCloudTask) ||
        taskRunning(captureState.imuTask) ||
        taskRunning(captureState.parameterTask) ||
        taskRunning(captureState.logTask) ||
        taskRunning(captureState.debugTask);

    if (anyRunning) {
        QStringList messages;
        appendRunningTaskMessage(messages, captureState.pointCloudTask);
        appendRunningTaskMessage(messages, captureState.imuTask);
        appendRunningTaskMessage(messages, captureState.parameterTask);
        appendRunningTaskMessage(messages, captureState.logTask);
        appendRunningTaskMessage(messages, captureState.debugTask);
        statusLabelBar->setText(messages.join(QStringLiteral(" / ")));
    } else {
        captureState.timer->stop();
    }
}

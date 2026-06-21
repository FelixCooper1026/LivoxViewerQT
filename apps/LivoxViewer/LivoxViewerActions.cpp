#include "LivoxViewerWindow.h"

#include <QDateTime>
#include <QDebug>
#include <QScrollBar>

namespace {

bool captureTaskRunning(const CaptureTaskState& task)
{
    return task.status == CaptureTaskStatus::Running;
}

} // namespace

void LivoxViewerWindow::updateStatus()
{
    const bool operationStatusActive =
        playbackState.active ||
        playbackState.loading ||
        measurementModeActive ||
        selectionRealtimeEnabled ||
        crossSectionModeActive ||
        captureTaskRunning(captureState.pointCloudTask) ||
        captureTaskRunning(captureState.imuTask) ||
        captureTaskRunning(captureState.parameterTask) ||
        captureTaskRunning(captureState.logTask) ||
        captureTaskRunning(captureState.debugTask);
    if (operationStatusActive) {
        return;
    }

    switch (realtimeState) {
    case RealtimeConnectionState::WaitingNetwork:
        statusLabelBar->setText(QStringLiteral("等待可用网卡"));
        return;
    case RealtimeConnectionState::Discovering:
        statusLabelBar->setText(QStringLiteral("正在发现设备..."));
        return;
    case RealtimeConnectionState::ReconfiguringNetwork:
        statusLabelBar->setText(QStringLiteral("正在配置主机 IP..."));
        return;
    case RealtimeConnectionState::WaitingSdkReady:
        statusLabelBar->setText(QStringLiteral("等待 SDK 就绪..."));
        return;
    case RealtimeConnectionState::InitializingSdk:
        statusLabelBar->setText(QStringLiteral("正在初始化 SDK..."));
        return;
    case RealtimeConnectionState::Stopping:
        statusLabelBar->setText(QStringLiteral("正在停止 SDK..."));
        return;
    case RealtimeConnectionState::Error:
        statusLabelBar->setText(QStringLiteral("设备连接异常"));
        return;
    case RealtimeConnectionState::Running:
    case RealtimeConnectionState::Idle:
        break;
    }

    int connectedCount = 0;
    int streamingCount = 0;
    {
        QMutexLocker locker(&lidarDeviceMutex);
        for (const LidarDeviceInfo& device : lidarDevices) {
            if (device.is_connected) {
                ++connectedCount;
                if (device.is_streaming) {
                    ++streamingCount;
                }
            }
        }
    }

    if (connectedCount > 0) {
        if (streamingCount > 0) {
            statusLabelBar->setText(connectedCount == 1
                ? QStringLiteral("已连接 - 采样中")
                : QStringLiteral("已连接 %1 台 - 采样中").arg(connectedCount));
        } else {
            statusLabelBar->setText(connectedCount == 1
                ? QStringLiteral("已连接 - 等待点云数据")
                : QStringLiteral("已连接 %1 台 - 等待点云数据").arg(connectedCount));
        }
        return;
    }

    switch (realtimeState) {
    case RealtimeConnectionState::Running:
        statusLabelBar->setText(QStringLiteral("未连接 - 等待雷达上线"));
        break;
    case RealtimeConnectionState::Idle:
        statusLabelBar->setText(sdk_started || sdk_initialized
            ? QStringLiteral("未连接 - 等待雷达上线")
            : QStringLiteral("就绪"));
        break;
    case RealtimeConnectionState::WaitingNetwork:
    case RealtimeConnectionState::Discovering:
    case RealtimeConnectionState::ReconfiguringNetwork:
    case RealtimeConnectionState::WaitingSdkReady:
    case RealtimeConnectionState::InitializingSdk:
    case RealtimeConnectionState::Stopping:
    case RealtimeConnectionState::Error:
        break;
    }
}

void LivoxViewerWindow::logMessage(const QString& message)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString logEntry = QString("[%1] %2").arg(timestamp).arg(message);
    logText->append(logEntry);
    if (logText->property("autoScroll").toBool()) {
        logText->verticalScrollBar()->setValue(logText->verticalScrollBar()->maximum());
    }
    qDebug() << logEntry;
}

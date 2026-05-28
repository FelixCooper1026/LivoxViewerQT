#include "LivoxViewerWindow.h"
#include "LivoxCore/LidarSdkService.h"

void LivoxViewerWindow::createDeviceActions()
{
    // 固件升级
    QAction* actionUpgrade = deviceMenu->addAction("固件升级...");

    // ==== 帮助菜单 ====

    connect(actionUpgrade, &QAction::triggered, this, &LivoxViewerWindow::showFirmwareUpgradeDialog);

    // 设备菜单新增：重启雷达 / 恢复出厂设置
    QAction* actionReboot = deviceMenu->addAction("重启雷达");
    QAction* actionFactoryReset = deviceMenu->addAction("恢复出厂设置");

    connect(actionReboot, &QAction::triggered, [this]() {
        LidarDeviceInfo currentDevice;
        if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
            logMessage("设备未连接，无法重启");
            return;
        }
        if (QMessageBox::warning(this, "重启雷达", "雷达将会重启，请确认操作", QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes) {
            livox_status st = LivoxLidarRequestReboot(currentDevice.handle, nullptr, this);
            if (st == kLivoxLidarStatusSuccess) logMessage("已发送重启命令，请等待雷达重启...");
            else logMessage(QString("发送重启命令失败: %1").arg(st));
        }
    });

    connect(actionFactoryReset, &QAction::triggered, [this]() {
        LidarDeviceInfo currentDevice;
        if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
            logMessage("设备未连接，无法恢复出厂设置");
            return;
        }
        if (QMessageBox::warning(this, "恢复出厂设置", "雷达将会恢复出厂设置，雷达IP将恢复为192.168.1.3，请确认操作", QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes) {
            livox_status st = LivoxLidarRequestReset(currentDevice.handle, nullptr, this);
            if (st == kLivoxLidarStatusSuccess) {
                shutting_down = true;
                pointCloudCallbackEnabled = false;
                LidarSdkService::clearCallbacks();
                logMessage("已发送恢复出厂设置命令，请等待雷达重启并恢复默认IP 192.168.1.3...");
                 // 清空当前设备缓存，避免显示旧IP设备
                 {
                     QMutexLocker locker(&lidarDeviceMutex);
                     lidarDevices.clear();
                 }
                 {
                     QMutexLocker locker(&frameMutex);
                     pendingFrames.clear();
                     lastSeenTimestamp.clear();
                     lastFrameTimestamp.clear();
                 }
                 clearCurrentDevice();
                 updateLidarDeviceList();
                 statusLabelBar->setText("等待设备重启上线...");
                 // 分步重启SDK：先清理，稍后重新发现设备，避免竞态
                 QTimer::singleShot(5000, this, [this]() {
                     shutdownLivoxSdk();
                     startLidarDiscovery();
                 });
            } else {
                logMessage(QString("发送恢复出厂设置命令失败: %1").arg(st));
            }
        }
    });
}

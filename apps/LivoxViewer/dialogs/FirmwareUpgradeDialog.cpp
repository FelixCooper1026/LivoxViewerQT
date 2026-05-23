#include "LivoxViewerWindow.h"
#include <QFileDialog>
#include <QFileInfo>
#include <QStandardPaths>

#include <thread>
void LivoxViewerWindow::showFirmwareUpgradeDialog()
{
        // 获取所有选中的设备
        QList<QListWidgetItem*> selectedItems = lidarDeviceList->selectedItems();
        if (selectedItems.isEmpty()) {
            QMessageBox::warning(this, "固件升级", "请至少选择一个设备");
            return;
        }

        // 收集选中设备的句柄并验证连接状态
        QVector<uint32_t> selectedHandles;
        QVector<QString> disconnectedDevices;
        QMutexLocker locker(&lidarDeviceMutex);

        for (QListWidgetItem* item : selectedItems) {
            uint32_t handle = item->data(Qt::UserRole).toUInt();
            if (lidarDevices.contains(handle)) {
                const LidarDeviceInfo& device = lidarDevices[handle];
                if (device.is_connected) {
                    selectedHandles.append(handle);
                } else {
                    disconnectedDevices.append(device.sn);
                }
            }
        }
        locker.unlock();

        if (selectedHandles.isEmpty()) {
            QString msg = disconnectedDevices.isEmpty()
                ? "没有可用的已连接设备"
                : QString("所选设备未连接: %1").arg(disconnectedDevices.join(", "));
            QMessageBox::warning(this, "固件升级", msg);
            return;
        }

        if (!disconnectedDevices.isEmpty()) {
            int ret = QMessageBox::warning(this, "固件升级",
                QString("以下设备未连接，将跳过: %1\n\n是否继续升级已连接的设备？").arg(disconnectedDevices.join(", ")),
                QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
            if (ret != QMessageBox::Yes) {
                return;
            }
        }

        // 记住上次选择的固件路径
        QSettings settings("Livox", "LivoxViewerQT");
        const QString docs = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
        QString lastDir = settings.value("upgrade/lastFirmwareDir", docs.isEmpty() ? QDir::homePath() : docs).toString();
        QString fw = QFileDialog::getOpenFileName(this, "选择固件文件", lastDir, "固件 (*.bin *.img);;所有文件 (*.*)");
        if (fw.isEmpty()) return;
        settings.setValue("upgrade/lastFirmwareDir", QFileInfo(fw).absolutePath());

        // 设置升级路径为固件文件完整路径（本地分隔符 + 本地8位编码）
        QFileInfo fi(fw);
        QString tryPath = fi.absoluteFilePath();
        QByteArray pathLocal = QDir::toNativeSeparators(tryPath).toLocal8Bit();
        bool okPath = SetLivoxLidarUpgradeFirmwarePath(pathLocal.constData());
        if (!okPath) {
            QMessageBox::critical(this, "固件升级", "设置固件路径失败，请确保选择单个固件文件，路径避免包含特殊字符");
            return;
        }

        // 初始化升级进度映射和总数
        {
            QMutexLocker locker(&upgradeProgressMutex);
            upgradeProgressMap.clear();
            for (uint32_t handle : selectedHandles) {
                upgradeProgressMap[handle] = 0;
            }
            upgradeTotalDevices = selectedHandles.size();
        }

        // 设置进度回调，支持多设备进度跟踪
        SetLivoxLidarUpgradeProgressCallback([](uint32_t handle, LivoxLidarUpgradeState state, void* client){
            LivoxViewerWindow* w = static_cast<LivoxViewerWindow*>(client);
            if (!w || !w->captureState.progress) return;
            QMetaObject::invokeMethod(w, [w, handle, state]() {
                // 在锁内更新进度和计算统计信息
                int avgProgress = 0;
                int completedCount = 0;
                int totalDevices = 0;
                bool allComplete = false;

                {
                    QMutexLocker locker(&w->upgradeProgressMutex);

                    // 如果该设备不在映射中，可能是之前的升级残留，忽略
                    if (!w->upgradeProgressMap.contains(handle)) {
                        return;
                    }

                    // 更新该设备的进度
                    // 检查状态事件：kLivoxLidarEventComplete = 4 表示完成
                    bool isComplete = (state.state == 4) || (state.progress >= 100);
                    w->upgradeProgressMap[handle] = isComplete ? 100 : state.progress;

                    // 计算平均进度和已完成设备数
                    totalDevices = w->upgradeTotalDevices;
                    if (totalDevices == 0) {
                        return;
                    }

                    int totalProgress = 0;
                    // 遍历所有应该升级的设备（映射中应该包含所有设备）
                    for (int progress : w->upgradeProgressMap.values()) {
                        totalProgress += progress;
                        if (progress >= 100) {
                            completedCount++;
                        }
                    }

                    // 计算平均进度：使用总设备数作为分母，确保即使某些设备未收到回调也能正确显示
                    avgProgress = totalDevices > 0 ? (totalProgress / totalDevices) : 0;

                    // 检查是否所有设备都完成
                    allComplete = (completedCount == totalDevices);
                    if (allComplete) {
                        w->upgradeProgressMap.clear();
                        w->upgradeTotalDevices = 0;
                    }
                } // 释放锁

                // 在锁外更新UI，避免长时间持有锁
                w->captureState.progress->setValue(avgProgress);
                w->captureState.progress->setFormat(QString("升级进度 %1% (%2/%3)").arg(avgProgress).arg(completedCount).arg(totalDevices));

                if (allComplete) {
                    w->statusLabelBar->setText(QString("升级完成 (%1个设备)，请等待设备重启").arg(totalDevices));
                    w->logMessage("固件升级已完成，请等待设备重启");
                }
            });
        }, this);

        // 准备设备句柄数组
        QVector<uint32_t> handleArr = selectedHandles;

        // 在后台线程执行升级，避免阻塞UI
        std::thread([handleArr]() {
            UpgradeLivoxLidars(handleArr.data(), handleArr.size());
        }).detach();

        captureState.progress->setValue(0);
        captureState.progress->setFormat(QString("升级进度 0% (0/%1)").arg(selectedHandles.size()));
        statusLabelBar->setText(QString("正在升级 %1 个设备，请勿断电或进行操作").arg(selectedHandles.size()));
        logMessage("正在进行固件升级，请勿断电或进行操作!!!");
}

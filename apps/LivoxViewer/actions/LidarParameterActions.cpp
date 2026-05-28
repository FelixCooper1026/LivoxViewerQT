#include "LivoxViewerWindow.h"
#include "LivoxCore/LidarParameterService.h"
#include "LivoxCore/LidarSdkService.h"
#include <QRegularExpression>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QDateTime>
#include <cstring>

void LivoxViewerWindow::onParamConfigChanged(uint16_t key)
{
    if (key == kKeyPclDataType) {
        updateProjectionControlsVisibility();
    }

    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) ||
        !currentDevice.is_connected ||
        !currentDevice.parameter_query_ready) {
        return;
    }
    
    // 获取控件当前值
    QWidget* control = parameterState.controls[key];
    if (!control) {
        return;
    }
    
    QString paramName;
    QString newValue;
    bool success = false;
    
    try {
        if (QComboBox* combo = qobject_cast<QComboBox*>(control)) {
            int index = combo->currentIndex();
            
            switch (key) {
                case kKeyPclDataType: {
                    paramName = "点云格式";
                    LivoxLidarPointDataType dataType = static_cast<LivoxLidarPointDataType>(index + 1); // 0->1, 1->2, 2->3
                    livox_status status = SetLivoxLidarPclDataType(currentDevice.handle, dataType, onAsyncControlResponse, this);
                    success = (status == kLivoxLidarStatusSuccess);
                    newValue = combo->currentText();
                    updateProjectionControlsVisibility();
                    break;
                }
                case kKeyPatternMode: {
                    paramName = "扫描模式";
                    LivoxLidarScanPattern pattern = static_cast<LivoxLidarScanPattern>(index);
                    livox_status status = SetLivoxLidarScanPattern(currentDevice.handle, pattern, onAsyncControlResponse, this);
                    success = (status == kLivoxLidarStatusSuccess);
                    newValue = combo->currentText();
                    break;
                }
                case kKeyDetectMode: {
                    paramName = "探测模式";
                    
                    // 检查设备类型是否支持探测模式
                    if (currentDevice.dev_type != kLivoxLidarTypeMid360) {
                        logMessage(QString("警告: 设备类型 %1 可能不支持探测模式配置").arg(currentDevice.product_info));
                    }
                    
                    LivoxLidarDetectMode mode;
                    switch (index) {
                        case 0: 
                            mode = kLivoxLidarDetectNormal; 
                            break;
                        case 1: 
                            mode = kLivoxLidarDetectSensitive; 
                            break;
                        default: 
                            logMessage(QString("探测模式索引无效: %1").arg(index));
                            newValue = "无效索引";
                            success = false;
                            break;
                    }
                    
                    if (index >= 0 && index <= 1) {
                        livox_status status = SetLivoxLidarDetectMode(currentDevice.handle, mode, onAsyncControlResponse, this);
                        success = (status == kLivoxLidarStatusSuccess);
                        newValue = combo->currentText();
                        if (!success) {
                            logMessage(QString("探测模式设置失败: %1 (错误码: %2)").arg(getLivoxStatusString(status)).arg(static_cast<int>(status)));
                        }
                    }
                    break;
                }
                case kKeyWorkMode: {
                    paramName = "工作模式";
                    LivoxLidarWorkMode workMode;
                    switch (index) {
                        case 0: workMode = kLivoxLidarNormal; break;
                        case 1: workMode = kLivoxLidarWakeUp; break;
                        default: 
                            logMessage(QString("工作模式索引无效: %1").arg(index));
                            return;
                    }
                    livox_status status = SetLivoxLidarWorkMode(currentDevice.handle, workMode, onAsyncControlResponse, this);
                    success = (status == kLivoxLidarStatusSuccess);
                    newValue = combo->currentText();
                    break;
                }
                case kKeySetEscMode: {
                    paramName = "电机转速";
                    LivoxLidarEscMode motorSpeed;
                    switch (index) {
                        case 0: motorSpeed = kLivoxEscSpeedNormal; break;
                        case 1: motorSpeed = kLivoxEscSpeedSlow; break;
                        default: 
                            logMessage(QString("电机转速索引无效: %1").arg(index));
                    }
                    livox_status status = SetLivoxLidarEscMode(currentDevice.handle, motorSpeed, onAsyncControlResponse, this);
                    success = (status == kLivoxLidarStatusSuccess);
                    newValue = combo->currentText();
                    break;
                }
                case kKeySetPpsSyncMode: {
                    paramName = "异常时间过滤";
                    LivoxLidarPpsSyncMode syncFilterMode;
                    switch (index) {
                        case 0: syncFilterMode = kLivoxPpsSyncNormal; break;
                        case 1: syncFilterMode = kLivoxPpsSyncSpec; break;
                        default: 
                            logMessage(QString("异常时间过滤索引无效: %1").arg(index));
                    }
                    livox_status status = SetLivoxLidarPpsSyncMode(currentDevice.handle, syncFilterMode, onAsyncControlResponse, this);
                    success = (status == kLivoxLidarStatusSuccess);
                    newValue = combo->currentText();
                    break;
                    
                }
                case kKeySetFovMode: {
                    paramName = "FOV模式";
                    LivoxLidarFovMode fovMode;
                    switch (index) {
                        case 0: fovMode = kLivoxSmallFovMode; break;
                        case 1: fovMode = kLivoxBigFovMode; break;
                    }
                    livox_status status = SetLivoxLidarFovMode(currentDevice.handle, fovMode, onAsyncControlResponse, this);
                    success = (status == kLivoxLidarStatusSuccess);
                    newValue = combo->currentText();
                    break;
                }
                case kKeySetEchoMode: {
                    paramName = "回波模式";
                    LivoxLidarEchoMode echoMode;
                    switch (index) {
                        case 0: echoMode = kLivoxStrongEchoMode; break;
                        case 1: echoMode = kLivoxFirstEchoMode; break;
                    }
                    livox_status status = SetLivoxLidarEchoMode(currentDevice.handle, echoMode, onAsyncControlResponse, this);
                    success = (status == kLivoxLidarStatusSuccess);
                    newValue = combo->currentText();
                    break;
                }
                case kKeyImuDataEn: {
                    paramName = "IMU数据发送";
                    livox_status status;
                    if (index == 1) { // "开启"
                        status = EnableLivoxLidarImuData(currentDevice.handle, onAsyncControlResponse, this);
                        if (status != kLivoxLidarStatusSuccess) {
                            logMessage(QString("启用IMU数据失败: %1 (错误码: %2)").arg(getLivoxStatusString(status)).arg(static_cast<int>(status)));                        }
                    } else { // "关闭"
                        status = DisableLivoxLidarImuData(currentDevice.handle, onAsyncControlResponse, this);
                        if (status != kLivoxLidarStatusSuccess) {
                            logMessage(QString("禁用IMU数据失败: %1 (错误码: %2)").arg(getLivoxStatusString(status)).arg(static_cast<int>(status)));                        }
                    }
                    success = (status == kLivoxLidarStatusSuccess);
                    newValue = combo->currentText();
                    break;
                }


                default:
                    logMessage(QString("未知的参数key: 0x%1").arg(key, 4, 16, QChar('0')));
                    return;
            }
        } else if (QCheckBox* checkBox = qobject_cast<QCheckBox*>(control)) {
            bool enabled = checkBox->isChecked();
            
            switch (key) {
                // 可以在这里添加复选框的处理逻辑
                default:
                    logMessage(QString("未知的复选框参数key: 0x%1").arg(key, 4, 16, QChar('0')));
                    return;
            }
        } else {
            logMessage(QString("未知控件类型: %1").arg(control->metaObject()->className()));
            return;
        }
        
        // 记录配置日志
        if (success) {
            logMessage(QString("发送命令: %1 -> %2").arg(paramName).arg(newValue));
            // 标记参数已更新，避免被定时器覆盖
            parameterState.updatedConfigKeys.insert(key);
        } else {
            logMessage(QString("发送命令失败: %1 -> %2").arg(paramName).arg(newValue));
        }

    } catch (const std::exception& e) {
        logMessage(QString("配置异常: %1 - %2").arg(paramName).arg(e.what()));
    } catch (...) {
        logMessage(QString("配置过程中发生未知异常: %1").arg(paramName));
    }
}

void LivoxViewerWindow::onIpConfigResponse(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void* client_data)
{
    LivoxViewerWindow* window = static_cast<LivoxViewerWindow*>(client_data);
    if (!window || window->shutting_down) return;

    const bool has_response = (response != nullptr);
    const uint8_t ret_code = has_response ? response->ret_code : 0xFF;

    QMetaObject::invokeMethod(window, [window, status, handle, ret_code, has_response]() {
        window->pendingLidarIpConfig.reset();
        // 1. 成功且设备确认执行：触发独有的重启逻辑
        if (status == kLivoxLidarStatusSuccess && has_response && ret_code == 0x00) {
            window->logMessage(QString("设备已确认IP配置，将请求重启雷达使配置生效 (handle=%1)").arg(handle));
            window->parameterState.updatedConfigKeys.insert(kKeyLidarIpCfg);

            livox_status rebootStatus = LivoxLidarRequestReboot(handle, nullptr, nullptr);
            if (rebootStatus == kLivoxLidarStatusSuccess) {
                window->shutting_down = true;
                window->pointCloudCallbackEnabled = false;
                LidarSdkService::clearCallbacks();
                window->logMessage("雷达正在重启，请等待设备以新配置上线...");
                {
                    QMutexLocker locker(&window->lidarDeviceMutex);
                    window->lidarDevices.clear();
                }
                {
                    QMutexLocker locker(&window->frameMutex);
                    window->pendingFrames.clear();
                    window->lastSeenTimestamp.clear();
                    window->lastFrameTimestamp.clear();
                }
                window->clearCurrentDevice();
                window->updateLidarDeviceList();
                if (window->statusLabelBar) window->statusLabelBar->setText("等待设备重启上线...");
                QTimer::singleShot(5000, window, [window]() {
                    window->shutdownLivoxSdk();
                    window->startLidarDiscovery();
                });
            } else {
                window->logMessage(QString("重启命令发送失败: %1").arg(getLivoxStatusString(rebootStatus)));
            }
            return;
        }

        // 2. 失败异常处理：直接调用提取的解析方法
        if (status != kLivoxLidarStatusSuccess) {
            window->logMessage(QString("设备未执行IP配置: %1 (handle=%2)")
                                 .arg(getLivoxStatusString(status)).arg(handle));
        } else if (!has_response) {
            window->logMessage(QString("设备未执行IP配置: 回调返回为空 (handle=%1)").arg(handle));
        } else {
            window->logMessage(QString("设备未执行IP配置: ret_code=0x%1 (%2) (handle=%3)")
                                 .arg(ret_code, 2, 16, QChar('0'))
                                 .arg(getRetCodeString(ret_code))
                                 .arg(handle));
        }

        if (window->statusLabelBar) window->statusLabelBar->setText("设备未执行配置");
    }, Qt::QueuedConnection);
}

void LivoxViewerWindow::applyIpConfig(uint16_t key, const QString& ip, const QString& mask, const QString& gateway)
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
        logMessage("设备未连接，无法配置");
        return;
    }
    
    const QString ipClean = ip.trimmed();
    const QString maskClean = mask.trimmed();
    const QString gwClean = gateway.trimmed();

    static const QRegularExpression ipRe(R"(^\s*((25[0-5]|2[0-4]\d|[01]?\d\d?)\.){3}(25[0-5]|2[0-4]\d|[01]?\d\d?)\s*$)");
    if (!ipRe.match(ipClean).hasMatch()) {
        logMessage("IP地址格式错误");
        return;
    }
    QString maskEff = maskClean.isEmpty() ? QString("255.255.255.0") : maskClean;
    QString gwEff = gwClean.isEmpty() ? QString("0.0.0.0") : gwClean;
    if (!ipRe.match(maskEff).hasMatch() || !ipRe.match(gwEff).hasMatch()) {
        logMessage("掩码或网关格式错误");
        return;
    }

    pendingLidarIpConfig.reset(new LivoxLidarIpInfo);
    std::memset(pendingLidarIpConfig.get(), 0, sizeof(LivoxLidarIpInfo));

    QByteArray ipBytes = ipClean.toLatin1();
    QByteArray maskBytes = maskEff.toLatin1();
    QByteArray gwBytes = gwEff.toLatin1();
    std::strncpy(pendingLidarIpConfig->ip_addr, ipBytes.constData(), sizeof(pendingLidarIpConfig->ip_addr) - 1);
    std::strncpy(pendingLidarIpConfig->net_mask, maskBytes.constData(), sizeof(pendingLidarIpConfig->net_mask) - 1);
    std::strncpy(pendingLidarIpConfig->gw_addr, gwBytes.constData(), sizeof(pendingLidarIpConfig->gw_addr) - 1);

    livox_status status = SetLivoxLidarIp(currentDevice.handle, pendingLidarIpConfig.get(), onIpConfigResponse, this);
    if (status == kLivoxLidarStatusSuccess) {
        logMessage("已发送雷达IP配置命令，等待设备确认是否执行...");
        if (statusLabelBar) statusLabelBar->setText("等待设备确认配置...");
    }
    else {
        pendingLidarIpConfig.reset();
        logMessage(QString("雷达IP配置命令发送失败: %1").arg(status));
    }
}

void LivoxViewerWindow::applyHostIpConfig(uint16_t key, const QString& ip, int port)
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
        logMessage("设备未连接，无法配置");
        return;
    }
    
    static const QRegularExpression ipRe(R"(^\s*((25[0-5]|2[0-4]\d|[01]?\d\d?)\.){3}(25[0-5]|2[0-4]\d|[01]?\d\d?)\s*$)");
    if (!ipRe.match(ip).hasMatch()) {
        logMessage("IP地址格式错误");
        return;
    }
    
    livox_status status;
    switch (key) {
        case kKeyStateInfoHostIpCfg: {
            HostStateInfoIpInfo hostConfig;
            std::memset(&hostConfig, 0, sizeof(hostConfig));
            QByteArray ipBytes = ip.toLatin1();
            std::strncpy(hostConfig.host_ip_addr, ipBytes.constData(), sizeof(hostConfig.host_ip_addr) - 1);
            hostConfig.host_state_info_port = static_cast<uint16_t>(port);
            hostConfig.lidar_state_info_port = static_cast<uint16_t>(port);
            status = SetLivoxLidarStateInfoHostIPCfg(currentDevice.handle, &hostConfig, onAsyncControlResponse, this);
            break;
        }
        case kKeyLidarPointDataHostIpCfg: {
            HostPointIPInfo hostConfig;
            std::memset(&hostConfig, 0, sizeof(hostConfig));
            QByteArray ipBytes = ip.toLatin1();
            std::strncpy(hostConfig.host_ip_addr, ipBytes.constData(), sizeof(hostConfig.host_ip_addr) - 1);
            hostConfig.host_point_data_port = static_cast<uint16_t>(port);
            hostConfig.lidar_point_data_port = static_cast<uint16_t>(port);
            status = SetLivoxLidarPointDataHostIPCfg(currentDevice.handle, &hostConfig, onAsyncControlResponse, this);
            break;
        }
        case kKeyLidarImuHostIpCfg: {
            HostImuDataIPInfo hostConfig;
            std::memset(&hostConfig, 0, sizeof(hostConfig));
            QByteArray ipBytes = ip.toLatin1();
            std::strncpy(hostConfig.host_ip_addr, ipBytes.constData(), sizeof(hostConfig.host_ip_addr) - 1);
            hostConfig.host_imu_data_port = static_cast<uint16_t>(port);
            hostConfig.lidar_imu_data_port = static_cast<uint16_t>(port);
            status = SetLivoxLidarImuDataHostIPCfg(currentDevice.handle, &hostConfig, onAsyncControlResponse, this);
            break;
        }
        default:
            logMessage("未知的IP配置类型");
            return;
    }
    
    if (status == kLivoxLidarStatusSuccess) {
        logMessage("已发送目的IP配置命令");
        // 标记参数已更新，避免被定时器覆盖
        parameterState.updatedConfigKeys.insert(key);
    } else {
        logMessage(QString("目的IP配置命令发送失败: %1").arg(status));
    }
}

void LivoxViewerWindow::applyFovConfig(uint16_t key, int yawStart, int yawStop, int pitchStart, int pitchStop)
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
        logMessage("设备未连接，无法配置");
        return;
    }
    
    FovCfg fovConfig;
    fovConfig.yaw_start = yawStart;
    fovConfig.yaw_stop = yawStop;
    fovConfig.pitch_start = pitchStart;
    fovConfig.pitch_stop = pitchStop;
    fovConfig.rsvd = 0;
    
    livox_status status;
    if (key == kKeyFovCfg0) {
        status = SetLivoxLidarFovCfg0(currentDevice.handle, &fovConfig, onAsyncControlResponse, this);
    } else {
        status = SetLivoxLidarFovCfg1(currentDevice.handle, &fovConfig, onAsyncControlResponse, this);
    }
    
    if (status == kLivoxLidarStatusSuccess) {
        logMessage(QString("已发送FOV%1配置命令").arg(key == kKeyFovCfg0 ? "0" : "1"));
        // 标记参数已更新，避免被定时器覆盖
        parameterState.updatedConfigKeys.insert(key);
    } else {
        logMessage(QString("FOV%1配置命令发送失败: %2").arg(key == kKeyFovCfg0 ? "0" : "1").arg(status));
    }
}

void LivoxViewerWindow::applyAttitudeConfig(uint16_t key, double roll, double pitch, double yaw, int x, int y, int z)
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
        logMessage("设备未连接，无法配置");
        return;
    }
    
    LivoxLidarInstallAttitude attitudeConfig;
    attitudeConfig.roll_deg = roll;
    attitudeConfig.pitch_deg = pitch;
    attitudeConfig.yaw_deg = yaw;
    attitudeConfig.x = x;
    attitudeConfig.y = y;
    attitudeConfig.z = z;
    
    livox_status status = SetLivoxLidarInstallAttitude(currentDevice.handle, &attitudeConfig, onAsyncControlResponse, this);
    if (status == kLivoxLidarStatusSuccess) {
        logMessage("已发送安装姿态配置命令");
        // 标记参数已更新，避免被定时器覆盖
        parameterState.updatedConfigKeys.insert(key);
    } else {
        logMessage(QString("安装姿态配置命令发送失败: %1").arg(status));
    }
}

void LivoxViewerWindow::updateFovEnableState(QCheckBox* fov0Check, QCheckBox* fov1Check)
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
        return;
    }
    
    // 根据两个FOV的状态计算使能值
    // 0 = 禁用所有FOV, 1 = 仅FOV0, 2 = 仅FOV1, 3 = FOV0和FOV1都启用
    uint8_t fovEnableValue = 0;
    if (fov0Check->isChecked()) {
        fovEnableValue |= 1; // 设置FOV0位
    }
    if (fov1Check->isChecked()) {
        fovEnableValue |= 2; // 设置FOV1位
    }
    
    // 发送FOV使能配置
    livox_status status = EnableLivoxLidarFov(currentDevice.handle, fovEnableValue, onAsyncControlResponse, this);
    if (status == kLivoxLidarStatusSuccess) {
        QString fovState;
        switch (fovEnableValue) {
            case 0: fovState = "禁用所有FOV"; break;
            case 1: fovState = "仅FOV0启用"; break;
            case 2: fovState = "仅FOV1启用"; break;
            case 3: fovState = "FOV0和FOV1都启用"; break;
        }
        logMessage(QString("FOV使能状态已更新: %1").arg(fovState));
        // 标记参数已更新，避免被定时器覆盖
        parameterState.updatedConfigKeys.insert(kKeyFovCfgEn);
        parameterState.updatedConfigKeys.insert(0x001F);
    } else {
        logMessage(QString("FOV使能状态更新失败: %1").arg(status));
    }
}

void LivoxViewerWindow::onParamQueryTimeout()
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) ||
        !currentDevice.is_connected ||
        !currentDevice.parameter_query_ready) {
        return;
    }
    
    // 只查询状态参数，不查询可配置参数
    // 状态参数：序列号、产品信息、版本、温度、时间等
    // 可配置参数：工作模式、扫描模式、点云格式等
    livox_status status = QueryLivoxLidarInternalInfo(currentDevice.handle, onQueryInternalInfoResponse, this);
    
    if (status != kLivoxLidarStatusSuccess) {
        logMessage(QString("查询雷达内部信息失败: %1 (错误码: %2)").arg(getLivoxStatusString(status)).arg(static_cast<int>(status)));
        
        // 如果是因为超时或未连接，可以在状态栏做简单提示
        if (status == kLivoxLidarStatusTimeout && statusLabelBar) {
            statusLabelBar->setText("查询超时，请检查网络连接");
        }
    }
}

QString LivoxViewerWindow::formatLidarParameterValue(uint16_t key, uint8_t* value, uint16_t length)
{
    return LidarParameterService::formatValue(key, value, length);
}

void LivoxViewerWindow::onRecordParamsClicked()
{
    if (!parameterState.isRecording) {
        // 生成默认文件名：记录时间_设备序列号_设备参数.csv
        QString defaultFileName;
        
        // 获取当前时间，格式为 yyyyMMdd_hhmmss
        QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
        
        // 获取设备序列号，如果没有当前设备则使用"Unknown"
        QString deviceSn = "Unknown";
        LidarDeviceInfo currentDevice;
        if (tryGetCurrentDevice(currentDevice) && !currentDevice.sn.isEmpty()) {
            deviceSn = currentDevice.sn;
        }
        
        // 生成默认文件名
        defaultFileName = QString("%1_%2_设备参数").arg(timestamp).arg(deviceSn);
        
        // 开始记录
        QString fileName = QFileDialog::getSaveFileName(
            this, 
            "选择CSV文件保存路径",
            QDir::homePath() + "/" + defaultFileName + ".csv",  // 使用生成的默认文件名
            "CSV文件 (*.csv)"
        );
        
        if (fileName.isEmpty()) {
            return;
        }
        
        // 确保文件扩展名
        if (!fileName.endsWith(".csv", Qt::CaseInsensitive)) {
            fileName += ".csv";
        }
        
        parameterState.recordFile.setFileName(fileName);
        if (!parameterState.recordFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this, "错误", "无法创建文件: " + fileName);
            return;
        }

        // 写入UTF-8 BOM头，确保Excel等软件正确识别编码
        QByteArray bom;
        bom.append(0xEF);
        bom.append(0xBB);
        bom.append(0xBF);
        parameterState.recordFile.write(bom);
        
        // 初始化参数键映射（包含所有要记录的参数）
        parameterState.recordedKeys.clear();
        parameterState.recordedOrder.clear(); // 清空顺序列表
        
        // 状态参数
        QVector<uint16_t> allKeys = {
            kKeySn, kKeyProductInfo, kKeyVersionApp, kKeyVersionLoader, kKeyVersionHardware, kKeyMac,
            kKeyCurWorkState, kKeyCoreTemp, kKeyPowerUpCnt, kKeyLocalTimeNow, kKeyLastSyncTime,
            kKeyTimeOffset, kKeyTimeSyncType, kKeyLidarDiagStatus, kKeyFwType, kKeyHmsCode,
            kKeyPclDataType, kKeyPatternMode, kKeyDetectMode, kKeyWorkMode, kKeyImuDataEn,
            kKeyLidarIpCfg, kKeyStateInfoHostIpCfg, kKeyLidarPointDataHostIpCfg, kKeyLidarImuHostIpCfg,
            kKeyFovCfg0, kKeyFovCfg1, kKeyFovCfgEn, kKeyInstallAttitude, kKeySetEscMode, kKeySetPpsSyncMode, kKeySetFovMode, kKeySetEchoMode
        };

        // 保存顺序
        parameterState.recordedOrder = allKeys;
        
        // 为每个参数键设置显示名称
        for (uint16_t key : allKeys) {
            switch (key) {
                case kKeySn: parameterState.recordedKeys[key] = "序列号"; break;
                case kKeyProductInfo: parameterState.recordedKeys[key] = "产品信息"; break;
                case kKeyVersionApp: parameterState.recordedKeys[key] = "固件版本"; break;
                case kKeyVersionLoader: parameterState.recordedKeys[key] = "LOADER版本"; break;
                case kKeyVersionHardware: parameterState.recordedKeys[key] = "硬件版本"; break;
                case kKeyMac: parameterState.recordedKeys[key] = "MAC地址"; break;
                case kKeyCurWorkState: parameterState.recordedKeys[key] = "当前工作状态"; break;
                case kKeyCoreTemp: parameterState.recordedKeys[key] = "核心温度"; break;
                case kKeyPowerUpCnt: parameterState.recordedKeys[key] = "上电次数"; break;
                case kKeyLocalTimeNow: parameterState.recordedKeys[key] = "本地时间"; break;
                case kKeyLastSyncTime: parameterState.recordedKeys[key] = "最后同步时间"; break;
                case kKeyTimeOffset: parameterState.recordedKeys[key] = "时间偏移"; break;
                case kKeyTimeSyncType: parameterState.recordedKeys[key] = "时间同步类型"; break;
                case kKeyLidarDiagStatus: parameterState.recordedKeys[key] = "雷达诊断状态"; break;
                case kKeyFwType: parameterState.recordedKeys[key] = "固件类型"; break;
                case kKeyHmsCode: parameterState.recordedKeys[key] = "HMS诊断码"; break;
                case kKeyPclDataType: parameterState.recordedKeys[key] = "点云格式"; break;
                case kKeyPatternMode: parameterState.recordedKeys[key] = "扫描模式"; break;
                case kKeyDetectMode: parameterState.recordedKeys[key] = "探测模式"; break;
                case kKeyWorkMode: parameterState.recordedKeys[key] = "工作模式"; break;
                case kKeyImuDataEn: parameterState.recordedKeys[key] = "IMU数据发送"; break;
                case kKeyLidarIpCfg: parameterState.recordedKeys[key] = "雷达IP配置"; break;
                case kKeyStateInfoHostIpCfg: parameterState.recordedKeys[key] = "状态信息目的IP"; break;
                case kKeyLidarPointDataHostIpCfg: parameterState.recordedKeys[key] = "点云数据目的IP"; break;
                case kKeyLidarImuHostIpCfg: parameterState.recordedKeys[key] = "IMU数据目的IP"; break;
                case kKeyFovCfg0: parameterState.recordedKeys[key] = "FOV0配置"; break;
                case kKeyFovCfg1: parameterState.recordedKeys[key] = "FOV1配置"; break;
                case kKeyFovCfgEn: parameterState.recordedKeys[key] = "FOV使能状态"; break;
                case kKeyInstallAttitude: parameterState.recordedKeys[key] = "安装姿态"; break;
                case kKeySetEscMode: parameterState.recordedKeys[key] = "电机转速"; break;
                case kKeySetPpsSyncMode: parameterState.recordedKeys[key] = "异常时间过滤"; break;
                case kKeySetFovMode: parameterState.recordedKeys[key] = "FOV模式"; break;
                case kKeySetEchoMode: parameterState.recordedKeys[key] = "回波模式"; break;
                default: parameterState.recordedKeys[key] = QString("参数0x%1").arg(key, 4, 16, QChar('0')); break;
            }
        }
        
        // 写入CSV表头
        QTextStream stream(&parameterState.recordFile);
        stream << "时间戳";
        for (uint16_t key : parameterState.recordedOrder) {
            stream << "," << parameterState.recordedKeys[key];
        }
        stream << "\n";

        parameterState.recordFile.flush();
        parameterState.recordFilePath = fileName;
        parameterState.isRecording = true;
        parameterState.recordButton->setText("停止参数记录");

        logMessage(QString("设备参数记录已开始"));
        
    } else {
        // 停止记录
        stopRecordParams();
    }
}


void LivoxViewerWindow::stopRecordParams()
{   
    if (parameterState.recordFile.isOpen()) {
        parameterState.recordFile.close();
    }
    
    parameterState.isRecording = false;
    parameterState.recordButton->setText("记录参数至CSV文件");
    
    QMessageBox::information(this, "记录完成", 
        QString("设备状态参数已保存至\n%1").arg(parameterState.recordFilePath));
    
    logMessage(QString("设备参数记录已停止，文件保存至: %1").arg(parameterState.recordFilePath));
}

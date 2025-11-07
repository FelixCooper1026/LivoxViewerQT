 #include "mainwindow.h"
#include <QRegularExpression>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include <QDateTime>
#include <cstring>

void MainWindow::onParamConfigChanged(uint16_t key)
{
    if (!currentDevice) {
        return;
    }

    if (!currentDevice->is_connected) {
        return;
    }
    
    // 获取控件当前值
    QWidget* control = paramControls[key];
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
                    livox_status status = SetLivoxLidarPclDataType(currentDevice->handle, dataType, onAsyncControlResponse, this);
                    success = (status == kLivoxLidarStatusSuccess);
                    newValue = combo->currentText();
                    // 启用/禁用投影深度控件：仅球坐标时可用
                    if (projectionDepthCheck) {
                        projectionDepthCheck->setEnabled(index == 2);
                    }
                    if (projectionDepthSpin) {
                        projectionDepthSpin->setEnabled(index == 2 && projectionDepthEnabled);
                    }
                    // 平面投影控件也仅在球坐标时可用
                    if (planarProjectionCheck) {
                        planarProjectionCheck->setEnabled(index == 2);
                    }
                    if (planarRadiusSpin) {
                        planarRadiusSpin->setEnabled(index == 2 && planarProjectionEnabled);
                    }
                    break;
                }
                case kKeyPatternMode: {
                    paramName = "扫描模式";
                    LivoxLidarScanPattern pattern = static_cast<LivoxLidarScanPattern>(index);
                    livox_status status = SetLivoxLidarScanPattern(currentDevice->handle, pattern, onAsyncControlResponse, this);
                    success = (status == kLivoxLidarStatusSuccess);
                    newValue = combo->currentText();
                    break;
                }
                case kKeyDetectMode: {
                    paramName = "探测模式";
                    
                    // 检查设备类型是否支持探测模式
                    if (currentDevice && currentDevice->dev_type != kLivoxLidarTypeMid360) {
                        logMessage(QString("警告: 设备类型 %1 可能不支持探测模式配置").arg(currentDevice->product_info));
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
                        livox_status status = SetLivoxLidarDetectMode(currentDevice->handle, mode, onAsyncControlResponse, this);
                        success = (status == kLivoxLidarStatusSuccess);
                        newValue = combo->currentText();
                        if (!success) {
                            logMessage(QString("探测模式设置失败，错误码: %1").arg(status));
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
                    livox_status status = SetLivoxLidarWorkMode(currentDevice->handle, workMode, onAsyncControlResponse, this);
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
                    livox_status status = SetLivoxLidarEscMode(currentDevice->handle, motorSpeed, onAsyncControlResponse, this);
                    success = (status == kLivoxLidarStatusSuccess);
                    newValue = combo->currentText();
                    break;
                }
                case kKeyImuDataEn: {
                    paramName = "IMU数据发送";
                    livox_status status;
                    if (index == 1) { // "开启"
                        status = EnableLivoxLidarImuData(currentDevice->handle, onAsyncControlResponse, this);
                        if (status != kLivoxLidarStatusSuccess) {
                            logMessage(QString("启用IMU数据失败，错误码: %1").arg(status));
                        }
                    } else { // "关闭"
                        status = DisableLivoxLidarImuData(currentDevice->handle, onAsyncControlResponse, this);
                        if (status != kLivoxLidarStatusSuccess) {
                            logMessage(QString("禁用IMU数据失败，错误码: %1").arg(status));
                        }
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
            logMessage(QString("配置成功: %1 -> %2").arg(paramName).arg(newValue));
            // 标记参数已更新，避免被定时器覆盖
            updatedConfigKeys.insert(key);
        } else {
            logMessage(QString("配置失败: %1 -> %2").arg(paramName).arg(newValue));
        }

    } catch (const std::exception& e) {
        logMessage(QString("配置异常: %1 - %2").arg(paramName).arg(e.what()));
    } catch (...) {
        logMessage(QString("配置过程中发生未知异常: %1").arg(paramName));
    }
}

void MainWindow::applyIpConfig(uint16_t key, const QString& ip, const QString& mask, const QString& gateway)
{
    if (!currentDevice || !currentDevice->is_connected) {
        logMessage("设备未连接，无法配置");
        return;
    }
    
    LivoxLidarIpInfo ipConfig;
    std::memset(&ipConfig, 0, sizeof(ipConfig));

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

    QByteArray ipBytes = ipClean.toLatin1();
    QByteArray maskBytes = maskEff.toLatin1();
    QByteArray gwBytes = gwEff.toLatin1();
    std::strncpy(ipConfig.ip_addr, ipBytes.constData(), sizeof(ipConfig.ip_addr) - 1);
    std::strncpy(ipConfig.net_mask, maskBytes.constData(), sizeof(ipConfig.net_mask) - 1);
    std::strncpy(ipConfig.gw_addr, gwBytes.constData(), sizeof(ipConfig.gw_addr) - 1);

    livox_status status = SetLivoxLidarIp(currentDevice->handle, &ipConfig, onAsyncControlResponse, this);
    if (status == kLivoxLidarStatusSuccess) {
        logMessage("雷达IP配置已发送，准备重启雷达使配置生效...");
        updatedConfigKeys.insert(key);
        livox_status rebootStatus = LivoxLidarRequestReboot(currentDevice->handle, nullptr, this);
        if (rebootStatus == kLivoxLidarStatusSuccess) {
        logMessage(QString("雷达IP已修改为[%1]，请等待雷达完成重启...").arg(ipClean));
        // 清空设备缓存，等待设备以新 IP 回来
        {
            QMutexLocker locker(&deviceMutex);
            devices.clear();
        }
        if (deviceList) updateDeviceList();
        if (statusLabelBar) statusLabelBar->setText("等待设备重启上线...");
    } else {
            logMessage(QString("发送重启命令失败: %1").arg(rebootStatus));
        }
    } else {
        logMessage(QString("雷达IP配置发送失败: %1").arg(status));
    }
}

void MainWindow::applyHostIpConfig(uint16_t key, const QString& ip, int port)
{
    if (!currentDevice || !currentDevice->is_connected) {
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
            status = SetLivoxLidarStateInfoHostIPCfg(currentDevice->handle, &hostConfig, onAsyncControlResponse, this);
            break;
        }
        case kKeyLidarPointDataHostIpCfg: {
            HostPointIPInfo hostConfig;
            std::memset(&hostConfig, 0, sizeof(hostConfig));
            QByteArray ipBytes = ip.toLatin1();
            std::strncpy(hostConfig.host_ip_addr, ipBytes.constData(), sizeof(hostConfig.host_ip_addr) - 1);
            hostConfig.host_point_data_port = static_cast<uint16_t>(port);
            hostConfig.lidar_point_data_port = static_cast<uint16_t>(port);
            status = SetLivoxLidarPointDataHostIPCfg(currentDevice->handle, &hostConfig, onAsyncControlResponse, this);
            break;
        }
        case kKeyLidarImuHostIpCfg: {
            HostImuDataIPInfo hostConfig;
            std::memset(&hostConfig, 0, sizeof(hostConfig));
            QByteArray ipBytes = ip.toLatin1();
            std::strncpy(hostConfig.host_ip_addr, ipBytes.constData(), sizeof(hostConfig.host_ip_addr) - 1);
            hostConfig.host_imu_data_port = static_cast<uint16_t>(port);
            hostConfig.lidar_imu_data_port = static_cast<uint16_t>(port);
            status = SetLivoxLidarImuDataHostIPCfg(currentDevice->handle, &hostConfig, onAsyncControlResponse, this);
            break;
        }
        default:
            logMessage("未知的IP配置类型");
            return;
    }
    
    if (status == kLivoxLidarStatusSuccess) {
        logMessage("目的IP配置已发送");
        // 标记参数已更新，避免被定时器覆盖
        updatedConfigKeys.insert(key);
    } else {
        logMessage(QString("目的IP配置发送失败: %1").arg(status));
    }
}

void MainWindow::applyFovConfig(uint16_t key, int yawStart, int yawStop, int pitchStart, int pitchStop)
{
    if (!currentDevice || !currentDevice->is_connected) {
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
        status = SetLivoxLidarFovCfg0(currentDevice->handle, &fovConfig, onAsyncControlResponse, this);
    } else {
        status = SetLivoxLidarFovCfg1(currentDevice->handle, &fovConfig, onAsyncControlResponse, this);
    }
    
    if (status == kLivoxLidarStatusSuccess) {
        logMessage(QString("FOV%1配置已发送").arg(key == kKeyFovCfg0 ? "0" : "1"));
        // 标记参数已更新，避免被定时器覆盖
        updatedConfigKeys.insert(key);
    } else {
        logMessage(QString("FOV%1配置发送失败: %2").arg(key == kKeyFovCfg0 ? "0" : "1").arg(status));
    }
}

void MainWindow::applyAttitudeConfig(uint16_t key, double roll, double pitch, double yaw, int x, int y, int z)
{
    if (!currentDevice || !currentDevice->is_connected) {
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
    
    livox_status status = SetLivoxLidarInstallAttitude(currentDevice->handle, &attitudeConfig, onAsyncControlResponse, this);
    if (status == kLivoxLidarStatusSuccess) {
        logMessage("安装姿态配置已发送");
        // 标记参数已更新，避免被定时器覆盖
        updatedConfigKeys.insert(key);
    } else {
        logMessage(QString("安装姿态配置发送失败: %1").arg(status));
    }
}

void MainWindow::updateFovEnableState(QCheckBox* fov0Check, QCheckBox* fov1Check)
{
    if (!currentDevice || !currentDevice->is_connected) {
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
    livox_status status = EnableLivoxLidarFov(currentDevice->handle, fovEnableValue, onAsyncControlResponse, this);
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
        updatedConfigKeys.insert(kKeyFovCfgEn);
        updatedConfigKeys.insert(0x001F);
    } else {
        logMessage(QString("FOV使能状态更新失败: %1").arg(status));
    }
}

void MainWindow::onParamQueryTimeout()
{
    if (!currentDevice || !currentDevice->is_connected) {
        return;
    }
    
    // 只查询状态参数，不查询可配置参数
    // 状态参数：序列号、产品信息、版本、温度、时间等
    // 可配置参数：工作模式、扫描模式、点云格式等
    livox_status status = QueryLivoxLidarInternalInfo(currentDevice->handle, onQueryInternalInfoResponse, this);
    
    if (status != kLivoxLidarStatusSuccess) {
        logMessage(QString("查询雷达内部信息失败，错误码: %1").arg(status));
    }
}

QString MainWindow::parseParamValue(uint16_t key, uint8_t* value, uint16_t length)
{
    if (!value || length == 0) {
        return "无数据";
    }
    
    switch (key) {
        case kKeyPclDataType: { // 点云格式
            if (length >= 1) {
                uint8_t dataType = value[0];
                switch (dataType) {
                    case 0x01: return "高精度笛卡尔坐标";
                    case 0x02: return "低精度笛卡尔坐标";
                    case 0x03: return "球坐标";
                    default: return QString("未知类型: %1").arg(dataType);
                }
            }
            break;
        }
        case kKeyPatternMode: { // 扫描模式
            if (length >= 1) {
                uint8_t pattern = value[0];
                switch (pattern) {
                    case 0x00: return "非重复扫描";
                    case 0x01: return "重复扫描";
                    case 0x02: return "低帧率重复扫描";
                    default: return QString("未知模式: %1").arg(pattern);
                }
            }
            break;
        }
        case kKeyLidarIpCfg: { // 雷达IP配置
            if (length >= 12) { // 4 + 4 + 4 bytes
                QString ip = QString("%1.%2.%3.%4")
                    .arg(value[0]).arg(value[1]).arg(value[2]).arg(value[3]);
                QString mask = QString("%1.%2.%3.%4")
                    .arg(value[4]).arg(value[5]).arg(value[6]).arg(value[7]);
                QString gateway = QString("%1.%2.%3.%4")
                    .arg(value[8]).arg(value[9]).arg(value[10]).arg(value[11]);
                return QString("IP:%1 Mask:%2 Gateway:%3").arg(ip).arg(mask).arg(gateway);
            }
            break;
        }
        case kKeyStateInfoHostIpCfg: { // 状态信息目的IP配置
            if (length >= 8) { // 4 + 2 + 2 bytes
                QString hostIp = QString("%1.%2.%3.%4")
                    .arg(value[0]).arg(value[1]).arg(value[2]).arg(value[3]);
                uint16_t hostPort = *reinterpret_cast<uint16_t*>(&value[4]);
                return QString("Host:%1:%2").arg(hostIp).arg(hostPort);
            }
            break;
        }
        case kKeyLidarPointDataHostIpCfg: { // 点云数据目的IP配置
            if (length >= 8) { // 4 + 2 + 2 bytes
                QString hostIp = QString("%1.%2.%3.%4")
                    .arg(value[0]).arg(value[1]).arg(value[2]).arg(value[3]);
                uint16_t hostPort = *reinterpret_cast<uint16_t*>(&value[4]);
                return QString("Host:%1:%2").arg(hostIp).arg(hostPort);
            }
            break;
        }
        case kKeyLidarImuHostIpCfg: { // IMU数据目的IP配置
            if (length >= 8) { // 4 + 2 + 2 bytes
                QString hostIp = QString("%1.%2.%3.%4")
                    .arg(value[0]).arg(value[1]).arg(value[2]).arg(value[3]);
                uint16_t hostPort = *reinterpret_cast<uint16_t*>(&value[4]);
                return QString("Host:%1:%2").arg(hostIp).arg(hostPort);
            }
            break;
        }
        case kKeyInstallAttitude: { // 安装姿态
            if (length >= 24) {
                // 前12字节是3个float (roll, pitch, yaw)
                float roll = *reinterpret_cast<float*>(&value[0]);
                float pitch = *reinterpret_cast<float*>(&value[4]);
                float yaw = *reinterpret_cast<float*>(&value[8]);
                // 后12字节是3个int32_t (x, y, z)
                int32_t x = *reinterpret_cast<int32_t*>(&value[12]);
                int32_t y = *reinterpret_cast<int32_t*>(&value[16]);
                int32_t z = *reinterpret_cast<int32_t*>(&value[20]);
                return QString("Roll:%1° Pitch:%2° Yaw:%3° X:%4mm Y:%5mm Z:%6mm")
                       .arg(roll, 0, 'f', 2).arg(pitch, 0, 'f', 2).arg(yaw, 0, 'f', 2)
                       .arg(x).arg(y).arg(z);
            }
            break;
        }
        case kKeyFovCfg0: { // FOV配置0
            if (length >= 20) { // 5 * 4 bytes (int32_t)
                int32_t yawStart = *reinterpret_cast<int32_t*>(&value[0]);
                int32_t yawStop = *reinterpret_cast<int32_t*>(&value[4]);
                int32_t pitchStart = *reinterpret_cast<int32_t*>(&value[8]);
                int32_t pitchStop = *reinterpret_cast<int32_t*>(&value[12]);
                uint32_t rsvd = *reinterpret_cast<uint32_t*>(&value[16]);
                return QString("Yaw:%1~%2° Pitch:%3~%4°").arg(yawStart).arg(yawStop).arg(pitchStart).arg(pitchStop);
            }
            break;
        }
        case kKeyFovCfg1: { // FOV配置1
            if (length >= 20) { // 5 * 4 bytes (int32_t)
                int32_t yawStart = *reinterpret_cast<int32_t*>(&value[0]);
                int32_t yawStop = *reinterpret_cast<int32_t*>(&value[4]);
                int32_t pitchStart = *reinterpret_cast<int32_t*>(&value[8]);
                int32_t pitchStop = *reinterpret_cast<int32_t*>(&value[12]);
                uint32_t rsvd = *reinterpret_cast<uint32_t*>(&value[16]);
                return QString("Yaw:%1~%2° Pitch:%3~%4°").arg(yawStart).arg(yawStop).arg(pitchStart).arg(pitchStop);
            }
            break;
        }
        case 0x0017: { // FOV使能
            if (length >= 1) {
                uint8_t fovEnableValue = value[0];
                switch (fovEnableValue) {
                    case 0: return "禁用所有FOV";
                    case 1: return "仅FOV0启用";
                    case 2: return "仅FOV1启用";
                    case 3: return "FOV0和FOV1都启用";
                    default: return QString("未知FOV状态: %1").arg(fovEnableValue);
                }
            }
            break;
        }
        case kKeyDetectMode: { // 探测模式
            if (length >= 1) {
                return value[0] ? "敏感模式" : "正常模式";
            }
            break;
        }
        case kKeyFuncIoCfg: { // 功能线IO配置
            if (length >= 4) {
                return QString("IN0:%1 IN1:%2 OUT0:%3 OUT1:%4")
                       .arg(value[0]).arg(value[1]).arg(value[2]).arg(value[3]);
            }
            break;
        }
        case kKeyWorkMode: { // 目标工作模式
            if (length >= 1) {
                uint8_t mode = value[0];
                switch (mode) {
                    case 0x01: return "采样模式";
                    case 0x02: return "待机模式";
                    case 0x03: return "睡眠模式";
                    case 0x04: return "错误状态";
                    case 0x05: return "上电自检";
                    case 0x06: return "电机启动";
                    case 0x07: return "电机停止";
                    case 0x08: return "升级中";
                    case 0x09: return "就绪";
                    default: return QString("未知模式: %1").arg(mode);
                }
            }
            break;
        }
        case kKeyImuDataEn: { // IMU数据使能
            if (length >= 1) {
                return value[0] ? "启用" : "禁用";
            }
            break;
        }
        case kKeySetEscMode: { // 电机转速
            if (length >= 1) {
                return value[0] ? "低转速" : "正常转速";
            }
            break;
        }
        case kKeySn: { // 序列号
            if (length >= 16) {
                return QString::fromLatin1(reinterpret_cast<char*>(value), 16).trimmed();
            }
            break;
        }
        case kKeyProductInfo: { // 产品信息
            if (length >= 64) {
                return QString::fromLatin1(reinterpret_cast<char*>(value), 64).trimmed();
            }
            break;
        }
        case kKeyVersionApp: { // 固件版本号
            if (length >= 4) {
                return QString("%1.%2.%3.%4").arg(value[0]).arg(value[1]).arg(value[2]).arg(value[3]);
            }
            break;
        }
        case kKeyVersionLoader: { // Loader版本号
            if (length >= 4) {
                return QString("%1.%2.%3.%4").arg(value[0]).arg(value[1]).arg(value[2]).arg(value[3]);
            }
            break;
        }
        case kKeyVersionHardware: { // 硬件版本号
            if (length >= 4) {
                return QString("%1.%2.%3.%4").arg(value[0]).arg(value[1]).arg(value[2]).arg(value[3]);
            }
            break;
        }
        case kKeyMac: { // MAC地址
            if (length >= 6) {
                return QString("%1:%2:%3:%4:%5:%6")
                       .arg(value[0], 2, 16, QChar('0'))
                       .arg(value[1], 2, 16, QChar('0'))
                       .arg(value[2], 2, 16, QChar('0'))
                       .arg(value[3], 2, 16, QChar('0'))
                       .arg(value[4], 2, 16, QChar('0'))
                       .arg(value[5], 2, 16, QChar('0'));
            }
            break;
        }
        case kKeyCurWorkState: { // 当前工作状态
            if (length >= 1) {
                uint8_t state = value[0];
                switch (state) {
                    case 0x01: return "采样";
                    case 0x02: return "待机";
                    case 0x03: return "睡眠";
                    case 0x04: return "错误";
                    case 0x05: return "自检";
                    case 0x06: return "电机启动";
                    case 0x07: return "停止";
                    case 0x08: return "升级";
                    case 0x09: return "就绪";
                    default: return QString("未知状态: %1").arg(state);
                }
            }
            break;
        }
        case kKeyCoreTemp: { // 核心温度
            if (length >= 4) {
                int32_t temp = *reinterpret_cast<int32_t*>(value);
                return QString("%1°C").arg(temp / 100.0); // Convert from 0.01°C to °C
            }
            break;
        }
        case kKeyPowerUpCnt: { // 上电次数
            if (length >= 4) {
                uint32_t count = *reinterpret_cast<uint32_t*>(value);
                return QString::number(count);
            }
            break;
        }
        case kKeyLocalTimeNow: { // 本地时间
            if (length >= 8) {
                uint64_t time = *reinterpret_cast<uint64_t*>(value);
                return QString::number(time); // 单位ns，不需要转换
            }
            break;
        }
        case kKeyLastSyncTime: { // 最后同步时间
            if (length >= 8) {
                uint64_t time = *reinterpret_cast<uint64_t*>(value);
                return QString::number(time); // 单位ns，不需要转换
            }
            break;
        }
        case kKeyTimeOffset: { // 时间偏移
            if (length >= 8) {
                int64_t offset = *reinterpret_cast<int64_t*>(value);
                return QString("%1μs").arg(offset / 1000); // Convert from ns to μs
            }
            break;
        }
        case kKeyTimeSyncType: { // 时间同步类型
            if (length >= 1) {
                uint8_t type = value[0];
                switch (type) {
                    case 0: return "无同步";
                    case 1: return "PTP同步";
                    case 2: return "GPS同步";
                    default: return QString("未知类型: %1").arg(type);
                }
            }
            break;
        }
        case kKeyFwType: { // 固件类型
            if (length >= 1) {
                uint8_t type = value[0];
                switch (type) {
                    case 0: return "Loader";
                    case 1: return "Application Image";
                    default: return QString("未知类型: %1").arg(type);
                }
            }
            break;
        }
        case kKeyHmsCode: { // HMS诊断码
            if (length >= 32) { // 8 * 4 bytes (uint32_t[8])
                QStringList faultInfo;
                bool hasError = false;
                
                for (int i = 0; i < 8; ++i) {
                    uint32_t hmsCode = *reinterpret_cast<uint32_t*>(&value[i * 4]);
                    if (hmsCode != 0) {
                        hasError = true;
                        
                        // 将32位故障码转换为8位十六进制字符串
                        QString faultCode = QString("%1").arg(hmsCode, 8, 16, QChar('0')).toUpper();
                        
                        // 解析故障码格式：AABBCCDD (32位)
                        // AA: 故障ID高字节 | BB: 故障ID低字节
                        // CC: 保留字段     | DD: 故障等级
                        QString faultId = faultCode.mid(0, 4);      // 前4位是故障ID
                        QString reserved = faultCode.mid(4, 2);     // 中间2位是保留字段
                        QString level = faultCode.mid(6, 2);        // 最后2位是故障等级
                        
                        // 获取故障级别描述
                        QString levelDesc;
                        if (level == "00") levelDesc = "无故障";
                        else if (level == "01") levelDesc = "Info消息";
                        else if (level == "02") levelDesc = "Warning警告";
                        else if (level == "03") levelDesc = "Error错误";
                        else if (level == "04") levelDesc = "Fatal严重错误";
                        else levelDesc = "未知级别";
                        
                        // 获取故障描述
                        QString faultDesc;
                        if (faultId == "0000") faultDesc = "无故障";
                        else if (faultId == "0102") faultDesc = "设备运行环境温度偏高;请检查环境温度，或排查散热措施";
                        else if (faultId == "0103") faultDesc = "设备运行环境温度较高;请检查环境温度，或排查散热措施";
                        else if (faultId == "0104") faultDesc = "设备球形光窗存在脏污,设备点云数据可信度较差;请及时清洗擦拭设备的球形光窗";
                        else if (faultId == "0105") faultDesc = "设备升级过程中出现错误;请重新进行升级";
                        else if (faultId == "0111") faultDesc = "设备内部器件温度异常;请检查环境温度，或排查散热措施";
                        else if (faultId == "0112") faultDesc = "设备内部器件温度异常;请检查环境温度，或排查散热措施";
                        else if (faultId == "0113") faultDesc = "设备内部IMU器件暂停工作;请重启设备恢复";
                        else if (faultId == "0114") faultDesc = "设备运行环境温度高;请检查环境温度，或排查散热措施";
                        else if (faultId == "0115") faultDesc = "设备运行环境温度超过承受极限，设备已停止工作;请检查环境温度，或排查散热措施";
                        else if (faultId == "0116") faultDesc = "设备外部电压异常;请检查外部电压";
                        else if (faultId == "0117") faultDesc = "设备参数异常;请尝试重启设备恢复";
                        else if (faultId == "0201") faultDesc = "扫描模块低温加热中";
                        else if (faultId == "0210") faultDesc = "扫描模块异常，请等待，若长时间未恢复，请尝试重启";
                        else if (faultId == "0211") faultDesc = "扫描模块异常，请等待，若长时间未恢复，请尝试重启";
                        else if (faultId == "0212") faultDesc = "扫描模块异常，请等待，若长时间未恢复，请尝试重启";
                        else if (faultId == "0213") faultDesc = "扫描模块异常，请等待，若长时间未恢复，请尝试重启";
                        else if (faultId == "0214") faultDesc = "扫描模块异常，请等待，若长时间未恢复，请尝试重启";
                        else if (faultId == "0215") faultDesc = "扫描模块异常，请等待，若长时间未恢复，请尝试重启";
                        else if (faultId == "0216") faultDesc = "扫描模块异常，请等待，若长时间未恢复，请尝试重启";
                        else if (faultId == "0217") faultDesc = "扫描模块异常，请等待，若长时间未恢复，请尝试重启";
                        else if (faultId == "0218") faultDesc = "扫描模块异常，请等待，若长时间未恢复，请尝试重启";
                        else if (faultId == "0219") faultDesc = "扫描模块异常，请等待，若长时间未恢复，请尝试重启";
                        else if (faultId == "0401") faultDesc = "检测到以太网连接曾断开过，请检查以太网链路是否存在异常";
                        else if (faultId == "0402") faultDesc = "ptp同步中断，或者时间跳变太大，请排查ptp时钟源是否工作正常";
                        else if (faultId == "0403") faultDesc = "PTP版本为1588-V2.1版本，设备不支持该版本，请更换1588-V2.0版本进行同步";
                        else if (faultId == "0404") faultDesc = "PPS同步异常，请检查PPS及GPS信号";
                        else if (faultId == "0405") faultDesc = "时间同步曾经发生过异常，请检查发生异常原因";
                        else if (faultId == "0406") faultDesc = "时间同步精度低，请检查同步源";
                        else if (faultId == "0407") faultDesc = "缺失GPS信号导致GPS同步失败，请检查GPS信号";
                        else if (faultId == "0408") faultDesc = "缺失PPS信号导致GPS同步失败，请检查PPS信号";
                        else if (faultId == "0409") faultDesc = "GPS信号异常，请检查GPS信号源";
                        else if (faultId == "040A") faultDesc = "PTP和gPTP信号同时存在，同步存在问题；请检查网络拓扑，单独使用PTP或gPTP同步";
                        else faultDesc = "未知故障";
                        
                        faultInfo.append(QString("[%1] 0x%2 - %3: %4").arg(i).arg(faultCode).arg(levelDesc).arg(faultDesc));
                    }
                }
                
                if (!hasError) {
                    return "无故障";
                } else {
                        // 每个HMS诊断码单独一行
                        return faultInfo.join("\n");
                    }
            }
            break;
        }
        case kKeyLidarDiagStatus: { // 雷达诊断状态
            if (length >= 2) {
                uint16_t status = *reinterpret_cast<uint16_t*>(value);
                return QString("0x%1").arg(status, 4, 16, QChar('0'));
            }
            break;
        }
        default:
            return QString("0x%1").arg(QByteArray(reinterpret_cast<char*>(value), length).toHex());
    }
    
    return "解析失败";
}

void MainWindow::onRecordParamsClicked()
{
    if (!isRecordingParams) {
        // 生成默认文件名：记录时间_设备序列号_设备参数.csv
        QString defaultFileName;
        
        // 获取当前时间，格式为 yyyyMMdd_hhmmss
        QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
        
        // 获取设备序列号，如果没有当前设备则使用"Unknown"
        QString deviceSn = "Unknown";
        if (currentDevice && !currentDevice->sn.isEmpty()) {
            deviceSn = currentDevice->sn;
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
        
        recordParamsFile.setFileName(fileName);
        if (!recordParamsFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::warning(this, "错误", "无法创建文件: " + fileName);
            return;
        }

        // 写入UTF-8 BOM头，确保Excel等软件正确识别编码
        QByteArray bom;
        bom.append(0xEF);
        bom.append(0xBB);
        bom.append(0xBF);
        recordParamsFile.write(bom);
        
        // 初始化参数键映射（包含所有要记录的参数）
        recordedParamKeys.clear();
        recordedParamOrder.clear(); // 清空顺序列表
        
        // 状态参数
        QVector<uint16_t> allKeys = {
            kKeySn, kKeyProductInfo, kKeyVersionApp, kKeyVersionLoader, kKeyVersionHardware, kKeyMac,
            kKeyCurWorkState, kKeyCoreTemp, kKeyPowerUpCnt, kKeyLocalTimeNow, kKeyLastSyncTime,
            kKeyTimeOffset, kKeyTimeSyncType, kKeyLidarDiagStatus, kKeyFwType, kKeyHmsCode,
            kKeyPclDataType, kKeyPatternMode, kKeyDetectMode, kKeyWorkMode, kKeyImuDataEn,
            kKeyLidarIpCfg, kKeyStateInfoHostIpCfg, kKeyLidarPointDataHostIpCfg, kKeyLidarImuHostIpCfg,
            kKeyFovCfg0, kKeyFovCfg1, kKeyFovCfgEn, kKeyInstallAttitude, kKeySetEscMode
        };

        // 保存顺序
        recordedParamOrder = allKeys;
        
        // 为每个参数键设置显示名称
        for (uint16_t key : allKeys) {
            switch (key) {
                case kKeySn: recordedParamKeys[key] = "序列号"; break;
                case kKeyProductInfo: recordedParamKeys[key] = "产品信息"; break;
                case kKeyVersionApp: recordedParamKeys[key] = "固件版本"; break;
                case kKeyVersionLoader: recordedParamKeys[key] = "LOADER版本"; break;
                case kKeyVersionHardware: recordedParamKeys[key] = "硬件版本"; break;
                case kKeyMac: recordedParamKeys[key] = "MAC地址"; break;
                case kKeyCurWorkState: recordedParamKeys[key] = "当前工作状态"; break;
                case kKeyCoreTemp: recordedParamKeys[key] = "核心温度"; break;
                case kKeyPowerUpCnt: recordedParamKeys[key] = "上电次数"; break;
                case kKeyLocalTimeNow: recordedParamKeys[key] = "本地时间"; break;
                case kKeyLastSyncTime: recordedParamKeys[key] = "最后同步时间"; break;
                case kKeyTimeOffset: recordedParamKeys[key] = "时间偏移"; break;
                case kKeyTimeSyncType: recordedParamKeys[key] = "时间同步类型"; break;
                case kKeyLidarDiagStatus: recordedParamKeys[key] = "雷达诊断状态"; break;
                case kKeyFwType: recordedParamKeys[key] = "固件类型"; break;
                case kKeyHmsCode: recordedParamKeys[key] = "HMS诊断码"; break;
                case kKeyPclDataType: recordedParamKeys[key] = "点云格式"; break;
                case kKeyPatternMode: recordedParamKeys[key] = "扫描模式"; break;
                case kKeyDetectMode: recordedParamKeys[key] = "探测模式"; break;
                case kKeyWorkMode: recordedParamKeys[key] = "工作模式"; break;
                case kKeyImuDataEn: recordedParamKeys[key] = "IMU数据发送"; break;
                case kKeyLidarIpCfg: recordedParamKeys[key] = "雷达IP配置"; break;
                case kKeyStateInfoHostIpCfg: recordedParamKeys[key] = "状态信息目的IP"; break;
                case kKeyLidarPointDataHostIpCfg: recordedParamKeys[key] = "点云数据目的IP"; break;
                case kKeyLidarImuHostIpCfg: recordedParamKeys[key] = "IMU数据目的IP"; break;
                case kKeyFovCfg0: recordedParamKeys[key] = "FOV0配置"; break;
                case kKeyFovCfg1: recordedParamKeys[key] = "FOV1配置"; break;
                case kKeyFovCfgEn: recordedParamKeys[key] = "FOV使能状态"; break;
                case kKeyInstallAttitude: recordedParamKeys[key] = "安装姿态"; break;
                case kKeySetEscMode: recordedParamKeys[key] = "电机转速"; break;
                default: recordedParamKeys[key] = QString("参数0x%1").arg(key, 4, 16, QChar('0')); break;
            }
        }
        
        // 写入CSV表头
        QTextStream stream(&recordParamsFile);
        stream << "时间戳";
        for (uint16_t key : recordedParamOrder) {
            stream << "," << recordedParamKeys[key];
        }
        stream << "\n";

        recordParamsFile.flush();   
        recordParamsFilePath = fileName;
        isRecordingParams = true;
        recordParamsButton->setText("停止参数记录");

        logMessage(QString("设备参数记录已开始"));
        
    } else {
        // 停止记录
        stopRecordParams();
    }
}


void MainWindow::stopRecordParams()
{   
    if (recordParamsFile.isOpen()) {
        recordParamsFile.close();
    }
    
    isRecordingParams = false;
    recordParamsButton->setText("记录参数至CSV文件");
    
    QMessageBox::information(this, "记录完成", 
        QString("设备状态参数已保存至\n%1").arg(recordParamsFilePath));
    
    logMessage(QString("设备参数记录已停止，文件保存至: %1").arg(recordParamsFilePath));
}

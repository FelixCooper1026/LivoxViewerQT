#include "mainwindow.h"
#include <QRegularExpression>
#include <QTime>
#include <QLayout>
#include <QLayoutItem>
#include <QApplication>
#include <algorithm>
#include <cstring>

void MainWindow::onDeviceInfoChange(uint32_t handle, const LivoxLidarInfo* info, void* client_data)
{
    MainWindow* window = static_cast<MainWindow*>(client_data);
    if (!window || window->shutting_down) {
        return;
    }

    if (info) {
        // 设备信息存在的处理逻辑（原有代码）
        DeviceInfo device;
        device.handle = handle;
        device.dev_type = info->dev_type;
        device.sn = QString::fromLatin1(info->sn);
        device.lidar_ip = QString::fromLatin1(info->lidar_ip);
        device.is_connected = true;
        device.is_streaming = false;

        QString deviceTypeName;
        switch (info->dev_type) {
        case kLivoxLidarTypeMid40: deviceTypeName = "Mid40"; break;
        case kLivoxLidarTypeMid70: deviceTypeName = "Mid70"; break;
        case kLivoxLidarTypeMid360: deviceTypeName = "Mid360"; break;
        case kLivoxLidarTypeMid360s: deviceTypeName = "Mid360s"; break;
        case kLivoxLidarTypeHorizon: deviceTypeName = "Horizon"; break;
        case kLivoxLidarTypeAvia: deviceTypeName = "Avia"; break;
        case kLivoxLidarTypeTele: deviceTypeName = "Tele"; break;
        case kLivoxLidarTypeHAP: deviceTypeName = "HAP"; break;
        case kLivoxLidarTypePA: deviceTypeName = "PA"; break;
        default: deviceTypeName = "Unknown"; break;
        }
        device.product_info = deviceTypeName;

        QMetaObject::invokeMethod(window, [window, device]() {
            // 去重逻辑：同 SN 只保留一条（最新一次上报的句柄）
            {
                QMutexLocker locker(&window->deviceMutex);
                QList<uint32_t> handlesToRemove;
                for (auto it = window->devices.begin(); it != window->devices.end(); ++it) {
                    if (it.value().sn == device.sn && it.key() != device.handle) {
                        handlesToRemove.append(it.key());
                    }
                }
                for (uint32_t oldHandle : handlesToRemove) {
                    window->devices.remove(oldHandle);
                }
                window->devices[device.handle] = device;
            }

            window->updateDeviceList();

            if (window->devices.size() > 0) {
                if (window->statusLabel) window->statusLabel->setText("状态: 已连接");
                window->currentDevice = &window->devices[device.handle];

                for (int i = 0; i < window->deviceList->count(); ++i) {
                    QListWidgetItem* item = window->deviceList->item(i);
                    if (item->data(Qt::UserRole).toUInt() == device.handle) {
                        window->deviceList->setCurrentRow(i);
                        break;
                    }
                }

                window->updatedConfigKeys.clear();
                if (window->currentDevice && window->currentDevice->is_connected) {
                    livox_status status = QueryLivoxLidarInternalInfo(window->currentDevice->handle, onQueryInternalInfoResponse, window);
                    if (status != kLivoxLidarStatusSuccess) {
                        window->logMessage(QString("查询设备配置参数失败，错误码: %1").arg(status));
                    }
                }
            }

            window->logMessage(QString("发现设备: %1 (%2) - IP: %3").arg(device.sn).arg(device.product_info).arg(device.lidar_ip));
        }, Qt::QueuedConnection);
    } else {
        // 设备信息不存在时的处理逻辑
        QMetaObject::invokeMethod(window, [window, handle]() {
            // 从设备列表中移除
            {
                QMutexLocker locker(&window->deviceMutex);
                if (window->devices.contains(handle)) {
                    DeviceInfo removedDevice = window->devices[handle];
                    window->logMessage(QString("设备断开连接: %1 (%2) - IP: %3")
                                           .arg(removedDevice.sn)
                                           .arg(removedDevice.product_info)
                                           .arg(removedDevice.lidar_ip));
                    window->devices.remove(handle);
                } else {
                    window->logMessage(QString("未发现设备，句柄: %1").arg(handle));
                }
            }

            window->updateDeviceList();

            if (window->devices.isEmpty()) {
                if (window->statusLabel) window->statusLabel->setText("状态: 未连接");
                window->currentDevice = nullptr;
            }
        }, Qt::QueuedConnection);
    }
}

void MainWindow::onPointCloudData(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
    MainWindow* window = static_cast<MainWindow*>(client_data);
    if (!window || window->shutting_down || !data) {
        return;
    }
    if (data) {
        // 数据验证 - 检查数据包是否有效
        if (data->dot_num > 10000 || data->data_type > 10 || data->length > 10000) {
            // 数据异常，跳过处理
            return;
        }
        
        // 计算完整数据包大小
        size_t packet_size = sizeof(LivoxLidarEthernetPacket) + data->length - 1; // -1是因为data[1]已经包含在结构体中
        
        // 深拷贝数据包
        uint8_t* data_copy = new uint8_t[packet_size];
        memcpy(data_copy, data, packet_size);
        LivoxLidarEthernetPacket* packet_copy = reinterpret_cast<LivoxLidarEthernetPacket*>(data_copy);
        
        // 使用QueuedConnection确保在主线程中执行
        QMetaObject::invokeMethod(window, [window, handle, packet_copy]() {
            // 再次验证数据
            if (packet_copy->dot_num > 10000 || packet_copy->data_type > 10) {
                window->logMessage(QString("设备%1 数据包异常，跳过处理").arg(handle));
                delete[] reinterpret_cast<uint8_t*>(packet_copy);
                return;
            }
            
            // 处理点云数据
            window->processPointCloudPacket(handle, packet_copy);

            // LVX2录制：在主线程中累积并分帧写入
            if (window->lvx2SaveActive && packet_copy->data_type == 0x01) {
                QMutexLocker lk(&window->lvx2Mutex);
                uint64_t ts = window->parseTimestamp(packet_copy->timestamp);
                if (window->lvx2FrameStartNs == 0) window->lvx2FrameStartNs = ts;
                QByteArray pkg;
                LVX2PackageHeader hdr{};
                hdr.lidar_id = handle;
                hdr.timestamp_type = packet_copy->time_type;
                memcpy(&hdr.timestamp, packet_copy->timestamp, 8);
                hdr.udp_counter = packet_copy->udp_cnt;
                hdr.data_type = packet_copy->data_type;
                hdr.data_length = packet_copy->dot_num * 14;
                hdr.frame_counter = packet_copy->frame_cnt;
                pkg.append(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
                pkg.append(reinterpret_cast<const char*>(packet_copy->data), hdr.data_length);
                window->lvx2PendingPkgs.push_back(pkg);
                if (ts - window->lvx2FrameStartNs >= 50ULL * 1000000ULL) {
                    uint64_t frameStart = window->lvx2File.pos();
                    LVX2FrameHeader fh{};
                    window->lvx2File.write(reinterpret_cast<const char*>(&fh), sizeof(fh));
                    for (const QByteArray& ba : window->lvx2PendingPkgs) window->lvx2File.write(ba);
                    uint64_t nextOff = window->lvx2File.pos();
                    fh.current_offset = frameStart;
                    fh.next_offset = nextOff;
                    fh.frame_index = window->lvx2FrameIndex++;
                    window->lvx2File.seek(frameStart);
                    window->lvx2File.write(reinterpret_cast<const char*>(&fh), sizeof(fh));
                    window->lvx2File.seek(nextOff);
                    window->lvx2PendingPkgs.clear();
                    window->lvx2FrameStartNs = ts;
                }
            }
            
            // 清理内存
            delete[] reinterpret_cast<uint8_t*>(packet_copy);
            
        }, Qt::QueuedConnection);
    }
}

void MainWindow::onImuData(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
    MainWindow* window = static_cast<MainWindow*>(client_data);
    if (!window || window->shutting_down || !data) {
        return;
    }
    if (data) {
        // 数据验证
        if (data->dot_num > 100 || data->data_type != kLivoxLidarImuData || data->length > 1000) {
            return;
        }

        // 计算完整数据包大小
        size_t packet_size = sizeof(LivoxLidarEthernetPacket) + data->length - 1;

        // 深拷贝数据包
        uint8_t* data_copy = new uint8_t[packet_size];
        memcpy(data_copy, data, packet_size);
        LivoxLidarEthernetPacket* packet_copy = reinterpret_cast<LivoxLidarEthernetPacket*>(data_copy);

        QMetaObject::invokeMethod(window, [window, handle, packet_copy]() {
            
            // 再次验证数据
            if (packet_copy->dot_num > 100 || packet_copy->data_type != kLivoxLidarImuData) {
                // logMessage(QString("设备%1 IMU数据包异常，跳过处理").arg(handle));
                delete[] reinterpret_cast<uint8_t*>(packet_copy);
                return;
            }
            
            // 限制处理的数据量
            uint32_t max_points = std::min<uint32_t>(packet_copy->dot_num, 3);
            
            // 解析IMU数据
            if (packet_copy->data_type == kLivoxLidarImuData && packet_copy->dot_num > 0) {
                LivoxLidarImuRawPoint* p_imu_data = (LivoxLidarImuRawPoint*)packet_copy->data;
                // 仅存储最新IMU样本，避免阻塞UI
                LivoxLidarImuRawPoint last = p_imu_data[packet_copy->dot_num - 1];
                {
                    QMutexLocker lk(&window->imuSampleMutex);
                    window->latestImu.gx = last.gyro_x;
                    window->latestImu.gy = last.gyro_y;
                    window->latestImu.gz = last.gyro_z;
                    window->latestImu.ax = last.acc_x;
                    window->latestImu.ay = last.acc_y;
                    window->latestImu.az = last.acc_z;
                    window->latestImu.have = true;
                }
                // 若正在保存IMU数据，将包内样本写入CSV
                if (window->imuSaveActive) {
                    quint64 ts = window->parseTimestamp(packet_copy->timestamp);
                    for (uint32_t i = 0; i < packet_copy->dot_num; ++i) {
                        const LivoxLidarImuRawPoint& s = p_imu_data[i];
                        window->appendImuCsvRow(ts, s.gyro_x, s.gyro_y, s.gyro_z, s.acc_x, s.acc_y, s.acc_z);
                    }
                }
                if (packet_copy->dot_num > max_points) {
                    // logMessage(QString("设备%1 还有 %2 个IMU点未显示...").arg(handle).arg(packet_copy->dot_num - max_points));
                }
                

            }
            
            // 清理内存
            delete[] reinterpret_cast<uint8_t*>(packet_copy);
            
        }, Qt::QueuedConnection);
    }
}

void MainWindow::onStatusInfo(uint32_t handle, uint8_t dev_type, const char* info, void* client_data)
{
    MainWindow* window = static_cast<MainWindow*>(client_data);
    if (!window || window->shutting_down || !info) {
        return;
    }
    if (info) {
        QMetaObject::invokeMethod(window, [window, handle, info]() {
            // 检查字符串是否有效，避免显示乱码
            QString statusInfo = QString::fromLatin1(info);
            if (statusInfo.isEmpty() || statusInfo.contains(QRegularExpression("[\\x00-\\x08\\x0B\\x0C\\x0E-\\x1F\\x7F-\\x9F]"))) {
                // 包含控制字符，不显示
                return;
            }
            // window->logMessage(QString("设备 %1 状态信息: %2").arg(handle).arg(statusInfo));
        }, Qt::QueuedConnection);
    }
}

void MainWindow::onAsyncControlResponse(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void* client_data)
{
    // 由于不再使用EnableLivoxLidarPointSend/DisableLivoxLidarPointSend，
    // 这个回调函数主要用于其他控制命令的响应
    MainWindow* window = static_cast<MainWindow*>(client_data);
    if (window) {
        QMetaObject::invokeMethod(window, [window, status, handle, response]() {
            if (status == kLivoxLidarStatusSuccess) {
                window->logMessage(QString("设备 %1 控制命令执行成功").arg(handle));
            } else {
                QString errorMsg;
                switch (status) {
                    case kLivoxLidarStatusFailure: errorMsg = "操作失败"; break;
                    case kLivoxLidarStatusNotConnected: errorMsg = "设备未连接"; break;
                    case kLivoxLidarStatusNotSupported: errorMsg = "设备不支持此操作"; break;
                    case kLivoxLidarStatusTimeout: errorMsg = "操作超时"; break;
                    case kLivoxLidarStatusNotEnoughMemory: errorMsg = "内存不足"; break;
                    case kLivoxLidarStatusChannelNotExist: errorMsg = "通信通道不存在"; break;
                    case kLivoxLidarStatusInvalidHandle: errorMsg = "设备句柄无效"; break;
                    case kLivoxLidarStatusHandlerImplNotExist: errorMsg = "处理器实现不存在"; break;
                    case kLivoxLidarStatusSendFailed: errorMsg = "命令发送失败"; break;
                    default: errorMsg = QString("未知错误: %1").arg(status); break;
                }
                window->logMessage(QString("设备 %1 控制命令执行失败: %2").arg(handle).arg(errorMsg));
            }
        }, Qt::QueuedConnection);
    }
}

void MainWindow::onQueryInternalInfoResponse(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void* client_data)
{
    MainWindow* window = static_cast<MainWindow*>(client_data);
    if (window && response && status == kLivoxLidarStatusSuccess) {
        
        // 在回调线程中立即深拷贝数据，避免跨线程内存生命周期问题
        uint8_t ret_code = response->ret_code;
        uint16_t param_num = response->param_num;
        
        // 计算response->data的总长度
        uint16_t total_data_length = 0;
        uint16_t off = 0;
        for (uint16_t i = 0; i < param_num; ++i) {
            if (off + 4 > 65535) { // 防止溢出
                break;
            }
            uint16_t key, length;
            memcpy(&key, &response->data[off], sizeof(uint16_t));
            memcpy(&length, &response->data[off + 2], sizeof(uint16_t));
            total_data_length = off + 4 + length;
            off += 4 + length;
        }
        
        // 深拷贝response->data
        QByteArray data_copy;
        if (total_data_length > 0 && total_data_length <= 65535) {
            data_copy = QByteArray(reinterpret_cast<const char*>(response->data), total_data_length);
        } else {
            return;
        }
        
        QMetaObject::invokeMethod(window, [window, handle, ret_code, param_num, data_copy]() {
            // 解析参数数据 - 使用深拷贝的安全数据
            uint16_t off = 0;
            uint16_t paramNum = param_num; // 使用拷贝的param_num
            
            // 使用字节级别的安全解析 - 基于拷贝的数据
            for (uint16_t i = 0; i < paramNum; ++i) {
                // 正确解析变长数组中的参数
                // 每个参数的结构：key(2字节) + length(2字节) + value(length字节)
                
                // 首先读取key和length - 从拷贝的数据中读取
                uint16_t key, length;
                memcpy(&key, data_copy.constData() + off, sizeof(uint16_t));
                memcpy(&length, data_copy.constData() + off + 2, sizeof(uint16_t));
                
                // value的起始位置 - 从拷贝的数据中获取
                const uint8_t* value = reinterpret_cast<const uint8_t*>(data_copy.constData() + off + 4);
                
                // window->logMessage(QString("处理参数 #%1: key=0x%2, length=%3").arg(i).arg(key, 0, 16).arg(length));
                
                // 检查参数长度是否合理
                if (length > 0 && length <= 1024) {
                    QString valueStr = window->parseParamValue(key, const_cast<uint8_t*>(value), length);
                    window->paramValues[key] = valueStr;
                    
                    // window->logMessage(QString("参数解析结果: key=0x%1, value='%2'").arg(key, 0, 16).arg(valueStr));
                    
                    // 更新UI显示
                    if (window->paramLabels.contains(key)) {
                        // 状态参数：实时更新
                        window->paramLabels[key]->setText(valueStr);
                    } else if (window->paramControls.contains(key)) {
                        // 可配置参数：只在设备连接时更新一次，避免与用户配置冲突
                        // 只处理非状态参数的可配置参数，且只在设备连接时更新一次
                        if (!window->updatedConfigKeys.contains(key)) {
                                QWidget* control = window->paramControls[key];
                                if (QComboBox* combo = qobject_cast<QComboBox*>(control)) {
                                    // 只更新简单的下拉框控件（基本配置）
                                    // 暂时断开信号连接，避免触发配置调用
                                    combo->blockSignals(true);
                                    
                                    // 根据值设置下拉框
                                    if (key == kKeyPclDataType) {
                                        if (valueStr.contains("高精度")) combo->setCurrentIndex(0);
                                        else if (valueStr.contains("低精度")) combo->setCurrentIndex(1);
                                        else if (valueStr.contains("球坐标")) combo->setCurrentIndex(2);
                                        // 根据当前点云格式启用/禁用投影深度控件（仅球坐标时可用）
                                        if (window->projectionDepthCheck) {
                                            window->projectionDepthCheck->setEnabled(combo->currentIndex() == 2);
                                        }
                                        if (window->projectionDepthSpin) {
                                            window->projectionDepthSpin->setEnabled(combo->currentIndex() == 2 && window->projectionDepthEnabled);
                                        }
                                        // 平面投影控件也仅在球坐标时可用
                                        if (window->planarProjectionCheck) {
                                            window->planarProjectionCheck->setEnabled(combo->currentIndex() == 2);
                                        }
                                        if (window->planarRadiusSpin) {
                                            window->planarRadiusSpin->setEnabled(combo->currentIndex() == 2 && window->planarProjectionEnabled);
                                        }
                                    } else if (key == kKeyPatternMode) {
                                            if (valueStr == "非重复扫描") combo->setCurrentIndex(0);
                                            else if (valueStr == "重复扫描") combo->setCurrentIndex(1);
                                            else if (valueStr == "低帧率重复扫描") combo->setCurrentIndex(2);
                                    } else if (key == kKeyDetectMode) {
                                        if (valueStr.contains("正常")) combo->setCurrentIndex(0);
                                        else if (valueStr.contains("敏感")) combo->setCurrentIndex(1);
                                    } else if (key == kKeyWorkMode) {
                                        if (valueStr.contains("采样")) combo->setCurrentIndex(0);
                                        else if (valueStr.contains("待机")) combo->setCurrentIndex(1);
                                        else if (valueStr.contains("睡眠")) combo->setCurrentIndex(2);
                                        else if (valueStr.contains("错误")) combo->setCurrentIndex(3);
                                        else if (valueStr.contains("自检")) combo->setCurrentIndex(4);
                                        else if (valueStr.contains("电机启动")) combo->setCurrentIndex(5);
                                        else if (valueStr.contains("停止")) combo->setCurrentIndex(6);
                                        else if (valueStr.contains("升级")) combo->setCurrentIndex(7);
                                        else if (valueStr.contains("就绪")) combo->setCurrentIndex(8);
                                    } else if (key == kKeyImuDataEn) {
                                        if (valueStr.contains("启用") || valueStr.contains("开启")) combo->setCurrentIndex(1);
                                        else combo->setCurrentIndex(0);
                                    } else if (key == kKeySetEscMode) {
                                        if (valueStr.contains("正常转速")) combo->setCurrentIndex(0);
                                        else if (valueStr.contains("低转速")) combo->setCurrentIndex(1);
                                    }
                                    
                                    // 恢复信号连接
                                    combo->blockSignals(false);
                                } else if (QCheckBox* checkBox = qobject_cast<QCheckBox*>(control)) {
                                    // 只更新复选框控件（FOV使能）
                                    // 暂时断开信号连接
                                    checkBox->blockSignals(true);
                                    
                                    // 根据值设置复选框
                                    if (key == kKeyFovCfgEn) {
                                        // FOV0使能复选框，根据位掩码设置状态
                                        // 直接使用原始数值：0=禁用所有, 1=仅FOV0, 2=仅FOV1, 3=都启用
                                        bool fov0Enabled = false;
                                        
                                        // 从原始数值解析FOV0使能状态
                                        if (length >= 1) {
                                            uint8_t fovEnableValue = value[0];
                                            fov0Enabled = (fovEnableValue & 0x01) != 0; // 第0位
                                        }
                                        
                                        // 设置FOV0复选框的状态
                                        checkBox->setChecked(fov0Enabled);
                                        
                                        // 同时更新FOV1复选框的状态
                                        QWidget* fov1Control = window->paramControls[0x001F];
                                        if (QCheckBox* fov1CheckBox = qobject_cast<QCheckBox*>(fov1Control)) {
                                            bool fov1Enabled = false;
                                            if (length >= 1) {
                                                uint8_t fovEnableValue = value[0];
                                                fov1Enabled = (fovEnableValue & 0x02) != 0; // 第1位
                                            }
                                            fov1CheckBox->blockSignals(true);
                                            fov1CheckBox->setChecked(fov1Enabled);
                                            fov1CheckBox->blockSignals(false);
                                        }
                                    } else if (key == 0x001F) {
                                        // FOV1使能复选框，根据位掩码设置状态
                                        bool fov1Enabled = false;
                                        
                                        // 从原始数值解析FOV1使能状态
                                        if (length >= 1) {
                                            uint8_t fovEnableValue = value[0];
                                            fov1Enabled = (fovEnableValue & 0x02) != 0; // 第1位
                                        }
                                        
                                        // 设置FOV1复选框的状态
                                        checkBox->setChecked(fov1Enabled);
                                        
                                        // 同时更新FOV0复选框的状态
                                        QWidget* fov0Control = window->paramControls[kKeyFovCfgEn];
                                        if (QCheckBox* fov0CheckBox = qobject_cast<QCheckBox*>(fov0Control)) {
                                            bool fov0Enabled = false;
                                            if (length >= 1) {
                                                uint8_t fovEnableValue = value[0];
                                                fov0Enabled = (fovEnableValue & 0x01) != 0; // 第0位
                                            }
                                            fov0CheckBox->blockSignals(true);
                                            fov0CheckBox->setChecked(fov0Enabled);
                                            fov0CheckBox->blockSignals(false);
                                        }
                                    }
                                    
                                    // 恢复信号连接
                                    checkBox->blockSignals(false);
                                } else if (QWidget* container = qobject_cast<QWidget*>(control)) {
                                    // 处理复杂的配置控件（网络、FOV、外参）
                                    if (key == kKeyLidarIpCfg) {
                                        // 雷达IP配置更新
                                        // 从valueStr中解析IP、掩码、网关信息
                                        // 格式: "IP:192.168.1.50 Mask:255.255.255.0 Gateway:192.168.1.1"
                                        QRegularExpression ipRegex(R"(IP:(\d+\.\d+\.\d+\.\d+)\s+Mask:(\d+\.\d+\.\d+\.\d+)\s+Gateway:(\d+\.\d+\.\d+\.\d+))");
                                        QRegularExpressionMatch match = ipRegex.match(valueStr);
                                        if (match.hasMatch()) {
                                            QString ip = match.captured(1);
                                            QString mask = match.captured(2);
                                            QString gateway = match.captured(3);
                                            
                                            // 找到对应的输入框并更新
                                             QLayout* layout = container->layout();
                                             if (layout) {
                                                 for (int i = 0; i < layout->count(); ++i) {
                                                     QLayoutItem* item = layout->itemAt(i);
                                                     QWidget* widget = item ? item->widget() : nullptr;
                                                     if (QLineEdit* edit = qobject_cast<QLineEdit*>(widget)) {
                                                         if (i == 1) edit->setText(ip);      // IP输入框
                                                         else if (i == 3) edit->setText(mask); // 掩码输入框
                                                         else if (i == 5) edit->setText(gateway); // 网关输入框
                                                     }
                                                 }
                                             }
                                        }
                                    } else if (key == kKeyLidarPointDataHostIpCfg || 
                                               key == kKeyLidarImuHostIpCfg || 
                                               key == kKeyStateInfoHostIpCfg) {
                                        // 目的IP配置更新
                                        // 格式: "Host:192.168.1.100:57000"
                                        QRegularExpression hostRegex(R"(Host:(\d+\.\d+\.\d+\.\d+):(\d+))");
                                        QRegularExpressionMatch match = hostRegex.match(valueStr);
                                        if (match.hasMatch()) {
                                            QString ip = match.captured(1);
                                            int port = match.captured(2).toInt();
                                            
                                            // 找到对应的输入框并更新
                                             QLayout* layout = container->layout();
                                             if (layout) {
                                                 for (int i = 0; i < layout->count(); ++i) {
                                                     QLayoutItem* item = layout->itemAt(i);
                                                     QWidget* widget = item ? item->widget() : nullptr;
                                                     if (QLineEdit* edit = qobject_cast<QLineEdit*>(widget)) {
                                                         edit->setText(ip);
                                                     } else if (QSpinBox* spin = qobject_cast<QSpinBox*>(widget)) {
                                                         spin->setValue(port);
                                                     }
                                                 }
                                             }
                                        }
                                    } else if (key == kKeyFovCfg0 || key == kKeyFovCfg1) {
                                        // FOV配置更新
                                        // 格式: "Yaw:0~360° Pitch:-10~60°" (单位: 1°)
                                        QRegularExpression fovRegex(R"(Yaw:(-?\d+)~(-?\d+)°\s+Pitch:(-?\d+)~(-?\d+)°)");
                                        QRegularExpressionMatch match = fovRegex.match(valueStr);
                                        if (match.hasMatch()) {
                                            int yawStart = match.captured(1).toInt();
                                            int yawStop = match.captured(2).toInt();
                                            int pitchStart = match.captured(3).toInt();
                                            int pitchStop = match.captured(4).toInt();
                                            
                                            // 找到对应的输入框并更新
                                            QLayout* layout = container->layout();
                                            if (layout) {
                                                int spinIndex = 0;
                                                for (int i = 0; i < layout->count(); ++i) {
                                                    QLayoutItem* item = layout->itemAt(i);
                                                    QWidget* widget = item ? item->widget() : nullptr;
                                                    if (QSpinBox* spin = qobject_cast<QSpinBox*>(widget)) {
                                                        switch (spinIndex) {
                                                            case 0: spin->setValue(yawStart); break;
                                                            case 1: spin->setValue(yawStop); break;
                                                            case 2: spin->setValue(pitchStart); break;
                                                            case 3: spin->setValue(pitchStop); break;
                                                        }
                                                        spinIndex++;
                                                    }
                                                }
                                            }
                                        }
                                    } else if (key == kKeyInstallAttitude) {
                                        // 安装姿态配置更新
                                        // 格式: "Roll:0.00° Pitch:0.00° Yaw:0.00° X:0mm Y:0mm Z:0mm"
                                        QRegularExpression attitudeRegex(R"(Roll:([-\d.]+)°\s+Pitch:([-\d.]+)°\s+Yaw:([-\d.]+)°\s+X:(-?\d+)mm\s+Y:(-?\d+)mm\s+Z:(-?\d+)mm)");
                                        QRegularExpressionMatch match = attitudeRegex.match(valueStr);
                                        if (match.hasMatch()) {
                                            double roll = match.captured(1).toDouble();
                                            double pitch = match.captured(2).toDouble();
                                            double yaw = match.captured(3).toDouble();
                                            int x = match.captured(4).toInt();
                                            int y = match.captured(5).toInt();
                                            int z = match.captured(6).toInt();
                                            
                                            // 找到对应的输入框并更新
                                              // 现在外参是每行一个控件，直接通过 findChild 查找
                                              if (auto* rollSpin = container->findChild<QDoubleSpinBox*>(QString(), Qt::FindDirectChildrenOnly)) {
                                                  // 保留兼容：优先找 QDoubleSpinBox，若多个则依次设置
                                              }
                                              // 通用：遍历所有子控件并按类型赋值
                                              QList<QDoubleSpinBox*> dSpins = container->findChildren<QDoubleSpinBox*>();
                                              QList<QSpinBox*> iSpins = container->findChildren<QSpinBox*>();
                                              if (dSpins.size() >= 3) {
                                                  dSpins[0]->setValue(roll);
                                                  dSpins[1]->setValue(pitch);
                                                  dSpins[2]->setValue(yaw);
                                              }
                                              if (iSpins.size() >= 3) {
                                                  iSpins[0]->setValue(x);
                                                  iSpins[1]->setValue(y);
                                                  iSpins[2]->setValue(z);
                                              }
                                        }
                                    }
                                }
                                
                                // 标记该参数已更新，防止被定时器重复更新
                                window->updatedConfigKeys.insert(key);
                            }
                        }
                }
                
                // 计算下一个参数的偏移量
                // 当前参数大小 = key(2字节) + length(2字节) + value(length字节)
                off += sizeof(uint16_t) * 2;  // key + length
                off += length;                 // value length
            }

            // 在参数解析完成后，检查是否正在记录参数
            if (window->isRecordingParams && window->recordParamsFile.isOpen()) {
                // 获取当前时间戳
                QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
                
                // 获取设备信息
                QString deviceSn = "Unknown";
                QString deviceIp = "Unknown";
                if (window->currentDevice) {
                    deviceSn = window->currentDevice->sn;
                    deviceIp = window->currentDevice->lidar_ip;
                }
                
                // 写入数据行
                QTextStream stream(&window->recordParamsFile);
                stream << timestamp;
                
                // 写入所有参数值
                for (uint16_t key : window->recordedParamOrder) {
                    QString value = window->paramValues.value(key, "N/A");
                    // 处理CSV中的特殊字符（引号和逗号）
                    if (value.contains(',') || value.contains('"') || value.contains('\n')) {
                        value = "\"" + value.replace("\"", "\"\"") + "\"";
                    }
                    stream << "," << value;
                }
                stream << "\n";
                
                window->recordParamsFile.flush();
            }
            
            // window->logMessage(QString("参数查询回调处理完成，共处理 %1 个参数").arg(paramNum));
        }, Qt::QueuedConnection);
    }
} 

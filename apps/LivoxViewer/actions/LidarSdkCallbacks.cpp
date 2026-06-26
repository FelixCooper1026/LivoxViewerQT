#include "LivoxViewerWindow.h"
#include "LivoxCore/LidarDiagnostics.h"
#include "LivoxCore/LidarPacketUtils.h"
#include "LivoxCore/LidarSdkService.h"
#include "widgets/ParameterOptionButtons.h"
#include <QRegularExpression>
#include <QStringList>
#include <QTime>
#include <QLayout>
#include <QLayoutItem>
#include <QApplication>
#include <QPalette>
#include <algorithm>
#include <cstring>

namespace {
constexpr double kImuChartRetentionSec = 60.0;

QString lidarTypeName(uint8_t devType)
{
    switch (devType) {
    case kLivoxLidarTypeMid360: return "Mid360";
    case kLivoxLidarTypeMid360s: return "Mid360s";
    case kLivoxLidarTypeMid360l: return "Mid360l";
    case kLivoxLidarTypeAvia2: return "Avia2";
    case kLivoxLidarTypeHAP: return "HAP";
    case kLivoxLidarTypePA: return "PA";
    default: return "Unknown";
    }
}

QString hmsDisplayColor(int severity);

QString hmsRichText(const QString& text, const QVector<LivoxCore::HmsCodeInfo>& codes)
{
    const QStringList lines = text.split('\n');
    QStringList richLines;
    for (int i = 0; i < lines.size(); ++i) {
        const int severity = i < codes.size() ? LivoxCore::hmsSeverity(codes.at(i).level) : 0;
        richLines.append(QString("<span style=\"color:%1; font-weight:%2;\">%3</span>")
                         .arg(hmsDisplayColor(severity))
                         .arg(severity >= 3 ? "600" : "400")
                         .arg(lines.at(i).toHtmlEscaped()));
    }
    return richLines.join("<br/>");
}

QString hmsDisplayColor(int severity)
{
    const bool darkTheme = QApplication::palette().color(QPalette::Window).lightness() < 128;
    switch (severity) {
    case 1: return darkTheme ? QStringLiteral("#6cb6ff") : QStringLiteral("#2d7dd2");
    case 2: return darkTheme ? QStringLiteral("#f0b44c") : QStringLiteral("#b86e00");
    case 3: return darkTheme ? QStringLiteral("#ff6b6b") : QStringLiteral("#c93434");
    case 4: return darkTheme ? QStringLiteral("#ff4d6d") : QStringLiteral("#8b0000");
    default: return QApplication::palette().color(QPalette::WindowText).name();
    }
}
}

void LivoxViewerWindow::activateConnectedDevice(const LidarDeviceInfo& device)
{
    if (statusLabel) {
        statusLabel->setText("状态: 已连接");
    }
    setCurrentDeviceHandle(device.handle);
    updateLidarDeviceList();
    updateStatus();
}

void LivoxViewerWindow::registerPointCloudDeviceIfNeeded(uint32_t handle, uint8_t dev_type)
{
    LidarDeviceInfo device;
    bool inserted = false;
    bool streamingChanged = false;
    {
        QMutexLocker locker(&lidarDeviceMutex);
        auto existing = lidarDevices.find(handle);
        if (existing != lidarDevices.end()) {
            if (!existing->is_streaming) {
                existing->is_streaming = true;
                device = existing.value();
                streamingChanged = true;
            }
        } else {
            LidarDiscoveryService::DiscoveryResponse discoveryDevice;
            int discoveryCandidateCount = 0;
            for (const LidarDiscoveryService::DiscoveryResponse& candidate : discoveredLidarsBySn) {
                if (dev_type != 0 && candidate.deviceType != dev_type) {
                    continue;
                }
                bool alreadyKnown = false;
                for (auto it = lidarDevices.constBegin(); it != lidarDevices.constEnd(); ++it) {
                    if (it.value().sn == candidate.serialNumber) {
                        alreadyKnown = true;
                        break;
                    }
                }
                if (!alreadyKnown) {
                    discoveryDevice = candidate;
                    ++discoveryCandidateCount;
                }
            }
            if (discoveryCandidateCount != 1) {
                return;
            }

            QList<uint32_t> handlesToRemove;
            for (auto it = lidarDevices.begin(); it != lidarDevices.end(); ++it) {
                if (it.value().sn == discoveryDevice.serialNumber) {
                    handlesToRemove.append(it.key());
                }
            }
            for (uint32_t oldHandle : handlesToRemove) {
                lidarDevices.remove(oldHandle);
            }

            const uint8_t type = dev_type != 0 ? dev_type : discoveryDevice.deviceType;
            device.handle = handle;
            device.dev_type = type;
            device.sn = discoveryDevice.serialNumber;
            device.lidar_ip = discoveryDevice.deviceIp;
            device.product_info = lidarTypeName(type);
            device.work_state = QStringLiteral("读取中");
            device.is_connected = true;
            device.is_streaming = true;
            device.parameter_query_ready = false;
            lidarDevices[handle] = device;
            inserted = true;
        }
    }

    if (!inserted && !streamingChanged) {
        return;
    }

    if (inserted) {
        activateConnectedDevice(device);
        logMessage(QString("发现设备: %1 (%2) - IP: %3").arg(device.sn).arg(device.product_info).arg(device.lidar_ip));
    } else {
        updateLidarDeviceList();
        updateStatus();
    }
}

void LivoxViewerWindow::onLidarDeviceInfoChange(uint32_t handle, const LivoxLidarInfo* info, void* client_data)
{
    LivoxViewerWindow* window = static_cast<LivoxViewerWindow*>(client_data);
    if (!window || window->shutting_down) {
        return;
    }

    if (info) {
        // 设备信息存在的处理逻辑（原有代码）
        LidarDeviceInfo device;
        device.handle = handle;
        device.dev_type = info->dev_type;
        device.sn = QString::fromLatin1(info->sn);
        device.lidar_ip = QString::fromLatin1(info->lidar_ip);
        device.work_state = QStringLiteral("读取中");
        device.is_connected = true;
        device.is_streaming = false;
        device.parameter_query_ready = true;
        device.product_info = lidarTypeName(info->dev_type);

        QMetaObject::invokeMethod(window, [window, device]() {
            if (window->shutting_down) {
                return;
            }

            // 去重逻辑：同 SN 只保留一条（最新一次上报的句柄）
            bool wasKnown = false;
            {
                QMutexLocker locker(&window->lidarDeviceMutex);
                wasKnown = window->lidarDevices.contains(device.handle);
                QList<uint32_t> handlesToRemove;
                for (auto it = window->lidarDevices.begin(); it != window->lidarDevices.end(); ++it) {
                    if (it.value().sn == device.sn && it.key() != device.handle) {
                        wasKnown = true;
                        handlesToRemove.append(it.key());
                    }
                }
                for (uint32_t oldHandle : handlesToRemove) {
                    window->lidarDevices.remove(oldHandle);
                }
                LidarDeviceInfo updatedDevice = device;
                if (window->lidarDevices.contains(device.handle)) {
                    const LidarDeviceInfo oldDevice = window->lidarDevices.value(device.handle);
                    updatedDevice.is_streaming = oldDevice.is_streaming;
                    updatedDevice.firmware_version = oldDevice.firmware_version;
                    updatedDevice.work_state = oldDevice.work_state.isEmpty() ? updatedDevice.work_state : oldDevice.work_state;
                    updatedDevice.diagnostic_summary = oldDevice.diagnostic_summary;
                    updatedDevice.diagnostic_severity = oldDevice.diagnostic_severity;
                }
                window->lidarDevices[device.handle] = updatedDevice;
            }

            if (window->statusLabel) window->statusLabel->setText("状态: 已连接");
            if (!window->hasCurrentLidarHandle) {
                window->setCurrentDeviceHandle(device.handle);
            }
            window->updateLidarDeviceList();
            window->updateStatus();

            const bool isCurrentDevice = window->hasCurrentLidarHandle && window->currentLidarHandle == device.handle;
            if (isCurrentDevice) {
                window->parameterState.updatedConfigKeys.clear();
            }
            if (device.is_connected) {
                livox_status status = QueryLivoxLidarInternalInfo(device.handle, onQueryInternalInfoResponse, window);
                if (status != kLivoxLidarStatusSuccess) {
                    window->logMessage(QString("查询设备配置参数失败: %1 (错误码: %2)").arg(getLivoxStatusString(status)).arg(static_cast<int>(status)));
                }
            }

            if (!wasKnown) {
                window->logMessage(QString("发现设备: %1 (%2) - IP: %3").arg(device.sn).arg(device.product_info).arg(device.lidar_ip));
            }
        }, Qt::QueuedConnection);
    } else {
        // 设备信息不存在时的处理逻辑
        QMetaObject::invokeMethod(window, [window, handle]() {
            if (window->shutting_down) {
                return;
            }

            const bool removedCurrent = window->hasCurrentLidarHandle && window->currentLidarHandle == handle;
            bool hasRemainingDevice = false;
            uint32_t nextHandle = 0;
            {
                QMutexLocker locker(&window->lidarDeviceMutex);
                if (window->lidarDevices.contains(handle)) {
                    LidarDeviceInfo removedDevice = window->lidarDevices[handle];
                    window->logMessage(QString("设备断开连接: %1 (%2) - IP: %3")
                                           .arg(removedDevice.sn)
                                           .arg(removedDevice.product_info)
                                           .arg(removedDevice.lidar_ip));
                    window->lidarDevices.remove(handle);
                } else {
                    window->logMessage(QString("未发现设备，句柄: %1").arg(handle));
                }
                hasRemainingDevice = !window->lidarDevices.isEmpty();
                if (hasRemainingDevice) {
                    nextHandle = window->lidarDevices.firstKey();
                }
            }

            if (!hasRemainingDevice) {
                if (window->statusLabel) window->statusLabel->setText("状态: 未连接");
                window->clearCurrentDevice();
            } else if (removedCurrent) {
                window->setCurrentDeviceHandle(nextHandle);
            }
            window->updateLidarDeviceList();
            window->updateStatus();
        }, Qt::QueuedConnection);
    }
}

void LivoxViewerWindow::onPointCloudData(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
    LivoxViewerWindow* window = static_cast<LivoxViewerWindow*>(client_data);
    if (!window || window->shutting_down || !window->pointCloudCallbackEnabled || !data) {
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
        QMetaObject::invokeMethod(window, [window, handle, dev_type, packet_copy]() {
            if (window->shutting_down || !window->pointCloudCallbackEnabled) {
                delete[] reinterpret_cast<uint8_t*>(packet_copy);
                return;
            }

            // 再次验证数据
            if (packet_copy->dot_num > 10000 || packet_copy->data_type > 10) {
                window->logMessage(QString("设备%1 数据包异常，跳过处理").arg(handle));
                delete[] reinterpret_cast<uint8_t*>(packet_copy);
                return;
            }
            
            // 处理点云数据
            window->registerPointCloudDeviceIfNeeded(handle, dev_type);
            window->liveSlamSource.appendPointPacket(handle, dev_type, packet_copy, QStringLiteral("live"));
            window->decodePointCloudPacket(handle, dev_type, packet_copy);

            // LVX2录制：在主线程中累积并分帧写入
            if (window->captureState.lvx2SaveActive && packet_copy->data_type == 0x01) {
                QMutexLocker lk(&window->captureState.lvx2Mutex);
                uint64_t ts = LivoxCore::parseLivoxTimestamp(packet_copy->timestamp);
                if (window->captureState.lvx2FrameStartNs == 0) window->captureState.lvx2FrameStartNs = ts;
                QByteArray pkg;
                Lvx2PackageHeader hdr{};
                hdr.lidar_id = handle;
                hdr.timestamp_type = packet_copy->time_type;
                memcpy(&hdr.timestamp, packet_copy->timestamp, 8);
                hdr.udp_counter = packet_copy->udp_cnt;
                hdr.data_type = packet_copy->data_type;
                hdr.data_length = packet_copy->dot_num * 14;
                hdr.frame_counter = packet_copy->frame_cnt;
                pkg.append(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
                pkg.append(reinterpret_cast<const char*>(packet_copy->data), hdr.data_length);
                window->captureState.lvx2PendingPkgs.push_back(pkg);
                if (ts - window->captureState.lvx2FrameStartNs >= 50ULL * 1000000ULL) {
                    uint64_t frameStart = window->captureState.lvx2File.pos();
                    Lvx2FrameHeader fh{};
                    window->captureState.lvx2File.write(reinterpret_cast<const char*>(&fh), sizeof(fh));
                    for (const QByteArray& ba : window->captureState.lvx2PendingPkgs) window->captureState.lvx2File.write(ba);
                    uint64_t nextOff = window->captureState.lvx2File.pos();
                    fh.current_offset = frameStart;
                    fh.next_offset = nextOff;
                    fh.frame_index = window->captureState.lvx2FrameIndex++;
                    window->captureState.lvx2File.seek(frameStart);
                    window->captureState.lvx2File.write(reinterpret_cast<const char*>(&fh), sizeof(fh));
                    window->captureState.lvx2File.seek(nextOff);
                    window->captureState.lvx2PendingPkgs.clear();
                    window->captureState.lvx2FrameStartNs = ts;
                }
            }
            
            // 清理内存
            delete[] reinterpret_cast<uint8_t*>(packet_copy);
            
        }, Qt::QueuedConnection);
    }
}

void LivoxViewerWindow::onImuData(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
{
    LivoxViewerWindow* window = static_cast<LivoxViewerWindow*>(client_data);
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
            if (window->shutting_down) {
                delete[] reinterpret_cast<uint8_t*>(packet_copy);
                return;
            }

            // 再次验证数据
            if (packet_copy->dot_num > 100 || packet_copy->data_type != kLivoxLidarImuData) {
                // logMessage(QString("设备%1 IMU数据包异常，跳过处理").arg(handle));
                delete[] reinterpret_cast<uint8_t*>(packet_copy);
                return;
            }
            // 可视化缓存按设备分组记录所有 IMU 数据，旧 dock/CSV 行为仍限定当前设备。
            
            // 解析IMU数据
            if (packet_copy->data_type == kLivoxLidarImuData && packet_copy->dot_num > 0) {
                window->liveSlamSource.appendImuPacket(handle, packet_copy);
                LivoxLidarImuRawPoint* p_imu_data = (LivoxLidarImuRawPoint*)packet_copy->data;
                const quint64 ts = LivoxCore::parseLivoxTimestamp(packet_copy->timestamp);
                const double timestampSec = static_cast<double>(ts) * 1.0e-9;
                const bool isCurrentDevice = window->hasCurrentLidarHandle && window->currentLidarHandle == handle;
                // 仅存储最新IMU样本，避免阻塞UI
                if (isCurrentDevice) {
                    LivoxLidarImuRawPoint last = p_imu_data[packet_copy->dot_num - 1];
                    {
                        QMutexLocker lk(&window->imuState.sampleMutex);
                        window->imuState.latestSample.gx = last.gyro_x;
                        window->imuState.latestSample.gy = last.gyro_y;
                        window->imuState.latestSample.gz = last.gyro_z;
                        window->imuState.latestSample.ax = last.acc_x;
                        window->imuState.latestSample.ay = last.acc_y;
                        window->imuState.latestSample.az = last.acc_z;
                        window->imuState.latestSample.have = true;
                    }
                }
                {
                    QMutexLocker lk(&window->imuState.visualizationMutex);
                    ImuVisualizationDeviceState& deviceState = window->imuState.visualizationDevices[handle];
                    if (deviceState.timeOriginSec < 0.0) {
                        deviceState.timeOriginSec = timestampSec;
                    }
                    const double relativeSec = timestampSec - deviceState.timeOriginSec;
                    for (uint32_t i = 0; i < packet_copy->dot_num; ++i) {
                        const LivoxLidarImuRawPoint& s = p_imu_data[i];
                        const ImuAttitudeEstimator::Result attitude = deviceState.estimator.update(
                            timestampSec,
                            s.gyro_x, s.gyro_y, s.gyro_z,
                            s.acc_x, s.acc_y, s.acc_z);
                        deviceState.samples.append(ImuVisualizationSample{
                            relativeSec,
                            s.gyro_x, s.gyro_y, s.gyro_z,
                            s.acc_x, s.acc_y, s.acc_z,
                            attitude.rollDeg, attitude.pitchDeg, attitude.yawDeg,
                            attitude.orientation
                        });
                    }
                    int removeCount = 0;
                    while (removeCount < deviceState.samples.size() &&
                           relativeSec - deviceState.samples[removeCount].timestampSec > kImuChartRetentionSec) {
                        ++removeCount;
                    }
                    if (removeCount > 0) {
                        deviceState.samples.remove(0, removeCount);
                    }
                    deviceState.revision = ++window->imuState.visualizationRevision;
                }
                // 若正在保存IMU数据，将包内样本写入CSV
                if (isCurrentDevice && window->captureState.imuSaveActive) {
                    for (uint32_t i = 0; i < packet_copy->dot_num; ++i) {
                        const LivoxLidarImuRawPoint& s = p_imu_data[i];
                        window->appendImuCsvRow(ts, s.gyro_x, s.gyro_y, s.gyro_z, s.acc_x, s.acc_y, s.acc_z);
                    }
                }
                

            }
            
            // 清理内存
            delete[] reinterpret_cast<uint8_t*>(packet_copy);
            
        }, Qt::QueuedConnection);
    }
}

void LivoxViewerWindow::onStatusInfo(uint32_t handle, uint8_t dev_type, const char* info, void* client_data)
{
    LivoxViewerWindow* window = static_cast<LivoxViewerWindow*>(client_data);
    if (!window || window->shutting_down || !info) {
        return;
    }
    if (info) {
        const QString statusInfo = QString::fromLatin1(info);
        QMetaObject::invokeMethod(window, [window, handle, statusInfo]() {
            // 检查字符串是否有效，避免显示乱码
            if (statusInfo.isEmpty() || statusInfo.contains(QRegularExpression("[\\x00-\\x08\\x0B\\x0C\\x0E-\\x1F\\x7F-\\x9F]"))) {
                // 包含控制字符，不显示
                return;
            }
            // window->logMessage(QString("设备 %1 状态信息: %2").arg(handle).arg(statusInfo));
        }, Qt::QueuedConnection);
    }
}

// 解析 livox_status
QString LivoxViewerWindow::getLivoxStatusString(livox_status status)
{
    return LidarSdkService::statusString(status);
}
QString LivoxViewerWindow::getRetCodeString(uint8_t ret_code)
{
    return LidarSdkService::retCodeString(ret_code);
}
void LivoxViewerWindow::onAsyncControlResponse(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void* client_data)
{
    LivoxViewerWindow* window = static_cast<LivoxViewerWindow*>(client_data);
    if (!window) return;

    const bool has_response = (response != nullptr);
    const uint8_t ret_code = has_response ? response->ret_code : 0xFF;
    const uint16_t error_key = has_response ? response->error_key : 0;

    QMetaObject::invokeMethod(window, [window, status, handle, has_response, ret_code, error_key]() {
        if (status == kLivoxLidarStatusSuccess) {
            if (!has_response) {
                window->logMessage(QString("设备 %1 控制命令返回为空（ret_code未知）").arg(handle));
                return;
            }
            if (ret_code == 0x00) {
                window->logMessage(QString("设备 %1 控制命令执行成功").arg(handle));
                return;
            }

            const QString errorKeyPart = (error_key != 0)
                ? QString(" (命令参数编号=0x%1)").arg(error_key, 4, 16, QChar('0')) : QString();

            // 复用辅助函数获取错误描述
            QString retMsg = getRetCodeString(ret_code);
            
            if (ret_code == 0x21) {
                window->logMessage(QString("设备 %1 控制命令下发成功，但参数需重启生效 (ret_code=0x%2)%3")
                                     .arg(handle).arg(ret_code, 2, 16, QChar('0')).arg(errorKeyPart));
            } else {
                window->logMessage(QString("设备 %1 控制命令执行失败: ret_code=0x%2 (%3)%4")
                                     .arg(handle).arg(ret_code, 2, 16, QChar('0')).arg(retMsg).arg(errorKeyPart));
            }
        } else {
            // 复用辅助函数获取错误描述
            QString errorMsg = getLivoxStatusString(status);
            window->logMessage(QString("设备 %1 控制命令发送失败: %2").arg(handle).arg(errorMsg));
        }
    }, Qt::QueuedConnection);
}

void LivoxViewerWindow::onQueryInternalInfoResponse(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void* client_data)
{
    LivoxViewerWindow* window = static_cast<LivoxViewerWindow*>(client_data);
    if (window && !window->shutting_down && response && status == kLivoxLidarStatusSuccess) {
        
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
            if (window->shutting_down) {
                return;
            }

            // 解析参数数据 - 使用深拷贝的安全数据
            uint16_t off = 0;
            uint16_t paramNum = param_num; // 使用拷贝的param_num
            const bool isCurrentResponse = window->hasCurrentLidarHandle && window->currentLidarHandle == handle;
            bool deviceCardNeedsRefresh = false;
            
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
                    QString valueStr = window->formatLidarParameterValue(key, const_cast<uint8_t*>(value), length);
                    const QVector<LivoxCore::HmsCodeInfo> hmsCodes =
                        key == kKeyHmsCode ? LivoxCore::parseHmsCodes(value, length) : QVector<LivoxCore::HmsCodeInfo>();
                    if (key == kKeyVersionApp || key == kKeyCurWorkState) {
                        QMutexLocker locker(&window->lidarDeviceMutex);
                        auto it = window->lidarDevices.find(handle);
                        if (it != window->lidarDevices.end()) {
                            if (key == kKeyVersionApp) {
                                it->firmware_version = valueStr;
                            } else {
                                it->work_state = valueStr;
                            }
                            deviceCardNeedsRefresh = true;
                        }
                    }
                    if (key == kKeyHmsCode) {
                        QMutexLocker locker(&window->lidarDeviceMutex);
                        auto it = window->lidarDevices.find(handle);
                        if (it != window->lidarDevices.end()) {
                            it->diagnostic_summary = LivoxCore::hmsSummary(hmsCodes);
                            it->diagnostic_severity = LivoxCore::maxHmsSeverity(hmsCodes);
                            deviceCardNeedsRefresh = true;
                        }
                    }

                    if (!isCurrentResponse) {
                        off += sizeof(uint16_t) * 2;
                        off += length;
                        continue;
                    }

                    window->parameterState.values[key] = valueStr;
                    
                    // window->logMessage(QString("参数解析结果: key=0x%1, value='%2'").arg(key, 0, 16).arg(valueStr));
                    
                    // 更新UI显示
                    if (window->parameterState.labels.contains(key)) {
                        // 状态参数：实时更新
                        QLabel* label = window->parameterState.labels[key];
                        if (key == kKeyHmsCode) {
                            label->setTextFormat(Qt::RichText);
                            label->setText(hmsRichText(valueStr, hmsCodes));
                        } else {
                            label->setTextFormat(Qt::PlainText);
                            label->setText(valueStr);
                        }
                    } else if (window->parameterState.controls.contains(key)) {
                        // 可配置参数：只在设备连接时更新一次，避免与用户配置冲突
                        // 只处理非状态参数的可配置参数，且只在设备连接时更新一次
                        if (!window->parameterState.updatedConfigKeys.contains(key)) {
                                QWidget* control = window->parameterState.controls[key];
                                if (ParameterOptionButtons::isOptionControl(control)) {
                                    ParameterOptionButtons::setSignalsBlocked(control, true);
                                    auto setOptionIndex = [control](int index) {
                                        ParameterOptionButtons::setCurrentIndex(control, index);
                                    };
                                    
                                    if (key == kKeyPclDataType) {
                                        if (valueStr.contains("高精度")) setOptionIndex(0);
                                        else if (valueStr.contains("低精度")) setOptionIndex(1);
                                        else if (valueStr.contains("球坐标")) setOptionIndex(2);
                                        window->updateProjectionControlsVisibility();
                                    } else if (key == kKeyPatternMode) {
                                            if (valueStr == "非重复扫描") setOptionIndex(0);
                                            else if (valueStr == "重复扫描") setOptionIndex(1);
                                            else if (valueStr == "低帧率重复扫描") setOptionIndex(2);
                                    } else if (key == kKeyDetectMode) {
                                        if (valueStr.contains("正常")) setOptionIndex(0);
                                        else if (valueStr.contains("敏感")) setOptionIndex(1);
                                    } else if (key == kKeyWorkMode) {
                                        if (valueStr.contains("采样")) setOptionIndex(0);
                                        else if (valueStr.contains("待机")) setOptionIndex(1);
                                        else if (valueStr.contains("睡眠")) setOptionIndex(2);
                                        else if (valueStr.contains("错误")) setOptionIndex(3);
                                        else if (valueStr.contains("自检")) setOptionIndex(4);
                                        else if (valueStr.contains("电机启动")) setOptionIndex(5);
                                        else if (valueStr.contains("停止")) setOptionIndex(6);
                                        else if (valueStr.contains("升级")) setOptionIndex(7);
                                        else if (valueStr.contains("就绪")) setOptionIndex(8);
                                    } else if (key == kKeyImuDataEn) {
                                        if (valueStr.contains("启用") || valueStr.contains("开启")) setOptionIndex(1);
                                        else setOptionIndex(0);
                                    } else if (key == kKeySetEscMode) {
                                        if (valueStr.contains("正常转速")) setOptionIndex(0);
                                        else if (valueStr.contains("低转速")) setOptionIndex(1);
                                    }else if (key == kKeySetPpsSyncMode){
                                        if (valueStr.contains("关闭异常时间过滤")) setOptionIndex(0);
                                        else if (valueStr.contains("开启异常时间过滤")) setOptionIndex(1);
                                    }else if (key == kKeySetFovMode) {
                                        if (valueStr.contains("Focus FOV")) setOptionIndex(0);
                                        else if (valueStr.contains("Normal FOV")) setOptionIndex(1);
                                    } else if (key == kKeySetEchoMode) {
                                        if (valueStr.contains("最强回波")) setOptionIndex(0);
                                        else if (valueStr.contains("第一回波")) setOptionIndex(1);
                                    }

                                    ParameterOptionButtons::setSignalsBlocked(control, false);
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
                                        QWidget* fov1Control = window->parameterState.controls[0x001F];
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
                                        QWidget* fov0Control = window->parameterState.controls[kKeyFovCfgEn];
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
                                            
                                            if (QLineEdit* edit = container->findChild<QLineEdit*>("deviceIpEdit")) {
                                                edit->setText(ip);
                                            }
                                            if (QLineEdit* edit = container->findChild<QLineEdit*>("deviceMaskEdit")) {
                                                edit->setText(mask);
                                            }
                                            if (QLineEdit* edit = container->findChild<QLineEdit*>("deviceGatewayEdit")) {
                                                edit->setText(gateway);
                                            }
                                        }
                                    } else if (key == kKeyLidarPointDataHostIpCfg || 
                                               key == kKeyLidarImuHostIpCfg || 
                                               key == kKeyStateInfoHostIpCfg) {
                                        // 目的IP配置更新
                                        // 格式: "Host:192.168.1.100:56301"
                                        QRegularExpression hostRegex(R"(Host:(\d+\.\d+\.\d+\.\d+):(\d+))");
                                        QRegularExpressionMatch match = hostRegex.match(valueStr);
                                        if (match.hasMatch()) {
                                            QString ip = match.captured(1);
                                            int port = match.captured(2).toInt();
                                            
                                            const QString ipEditName = QString("targetIpEdit_%1").arg(key);
                                            const QString portSpinName = QString("targetPortSpin_%1").arg(key);
                                            if (QLineEdit* edit = container->findChild<QLineEdit*>(ipEditName)) {
                                                edit->setText(ip);
                                            }
                                            if (QSpinBox* spin = container->findChild<QSpinBox*>(portSpinName)) {
                                                spin->setValue(port);
                                            }
                                        }
                                    } else if (key == kKeySetNTPServerIp) {
                                        QRegularExpression ntpRegex(R"(NTP:(\d+\.\d+\.\d+\.\d+))");
                                        QRegularExpressionMatch match = ntpRegex.match(valueStr);
                                        if (match.hasMatch()) {
                                            if (QLineEdit* edit = container->findChild<QLineEdit*>("ntpServerIpEdit")) {
                                                edit->setText(match.captured(1));
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
                                window->parameterState.updatedConfigKeys.insert(key);
                            }
                        }
                }
                
                // 计算下一个参数的偏移量
                // 当前参数大小 = key(2字节) + length(2字节) + value(length字节)
                off += sizeof(uint16_t) * 2;  // key + length
                off += length;                 // value length
            }

            if (deviceCardNeedsRefresh) {
                window->updateLidarDeviceList();
            }

            // 在参数解析完成后，检查是否正在记录参数
            if (isCurrentResponse && window->parameterState.isRecording && window->parameterState.recordFile.isOpen()) {
                // 获取当前时间戳
                QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
                
                // 获取设备信息
                QString deviceSn = "Unknown";
                QString deviceIp = "Unknown";
                LidarDeviceInfo currentDevice;
                if (window->tryGetCurrentDevice(currentDevice)) {
                    deviceSn = currentDevice.sn;
                    deviceIp = currentDevice.lidar_ip;
                }
                
                // 写入数据行
                QTextStream stream(&window->parameterState.recordFile);
                stream << timestamp;
                
                // 写入所有参数值
                for (uint16_t key : window->parameterState.recordedOrder) {
                    QString value = window->parameterState.values.value(key, "N/A");
                    // 处理CSV中的特殊字符（引号和逗号）
                    if (value.contains(',') || value.contains('"') || value.contains('\n')) {
                        value = "\"" + value.replace("\"", "\"\"") + "\"";
                    }
                    stream << "," << value;
                }
                stream << "\n";
                
                window->parameterState.recordFile.flush();
            }
            
            // window->logMessage(QString("参数查询回调处理完成，共处理 %1 个参数").arg(paramNum));
        }, Qt::QueuedConnection);
    }
} 

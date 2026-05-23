#include "LivoxCore/LidarParameterService.h"

#include "LivoxCore/LidarSdkTypes.h"

#include <QByteArray>
#include <QChar>
#include <QStringList>

namespace LidarParameterService {

QString formatValue(uint16_t key, uint8_t* value, uint16_t length)
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
        case kKeySetPpsSyncMode: { // 异常时间过滤
            if (length >= 1) {
                uint8_t mode = value[0];
                switch (mode) {
                    case 0x00: return "关闭异常时间过滤";
                    case 0x01: return "开启异常时间过滤";
                    default: return QString("未知模式: %1").arg(mode);
                }
            }
            break;
        }
        case kKeySetFovMode: { // FOV模式
            if (length >= 1) {
                return value[0] ? "Normal FOV" : "Focus FOV";
            }
            break;
        }
        case kKeySetEchoMode: { // 回波模式
            if (length >= 1) {
                return value[0] ? "第一回波" : "最强回波";
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
                    case 5: return "NTP同步";
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
            if (length < 32) {
                break;
            }

            QStringList faultInfo;

            for (int i = 0; i < 8; ++i) {
                uint32_t hmsCode = *reinterpret_cast<const uint32_t*>(&value[i * 4]);

                if (hmsCode == 0) {
                    continue;
                }

                // 使用位运算替代字符串截取
                // 解析 AABBCCDD -> AA BB (高16位), DD (低8位)
                uint16_t faultId = (hmsCode >> 16) & 0xFFFF;
                uint8_t level = hmsCode & 0xFF;

                // 解析故障级别
                QString levelDesc;
                switch (level) {
                    case 0x00: levelDesc = "无故障"; break;
                    case 0x01: levelDesc = "Info消息"; break;
                    case 0x02: levelDesc = "Warning警告"; break;
                    case 0x03: levelDesc = "Error错误"; break;
                    case 0x04: levelDesc = "Fatal严重错误"; break;
                    default:   levelDesc = "未知级别"; break;
                }

                // 解析故障描述
                QString faultDesc;
                switch (faultId) {
                    case 0x0000: faultDesc = "无故障"; break;
                    case 0x0102: faultDesc = "设备运行环境温度偏高;请检查环境温度，或排查散热措施"; break;
                    case 0x0103: faultDesc = "设备运行环境温度较高;请检查环境温度，或排查散热措施"; break;
                    case 0x0104: faultDesc = "设备球罩存在脏污或附近有遮挡物，请及时清洗擦拭设备球罩，或确保球罩0.1m范围内无遮挡物"; break;
                    case 0x0105: faultDesc = "设备固件升级过程中出现错误;请重新进行固件升级"; break;
                    case 0x0111:
                    case 0x0112: faultDesc = "设备内部器件温度异常;请检查环境温度，或排查散热措施"; break;
                    case 0x0113: faultDesc = "设备内部IMU器件暂停工作;请重启设备恢复"; break;
                    case 0x0114: faultDesc = "设备运行环境温度高;请检查环境温度，或排查散热措施"; break;
                    case 0x0115: faultDesc = "设备运行环境温度超过承受极限，设备已停止工作;请检查环境温度，或排查散热措施"; break;
                    case 0x0116: faultDesc = "设备外部电压异常;请检查外部电压"; break;
                    case 0x0117: faultDesc = "设备参数异常;请尝试重启设备恢复"; break;
                    case 0x0118: faultDesc = "设备内部器件损坏，无法正常工作，请联系维修人员"; break;
                    case 0x0201: faultDesc = "扫描模块低温加热中"; break;
                    // 利用 switch case 的穿透特性，将描述相同的故障码合并
                    case 0x0210: case 0x0211: case 0x0212: case 0x0213: case 0x0214:
                    case 0x0215: case 0x0216: case 0x0217: case 0x0218: case 0x0219:
                                 faultDesc = "扫描模块异常，请尝试：1.检查供电是否正常 2.重启设备 3.更新最新固件"; break;
                    case 0x0401: faultDesc = "检测到以太网连接曾经断开过，现已恢复正常，请检查以太网链路是否存在异常"; break;
                    case 0x0402: faultDesc = "PTP同步中断，或者时间跳变太大，请排查PTP时钟源是否工作正常"; break;
                    case 0x0403: faultDesc = "PTP版本为1588-V2.1版本，设备不支持该版本，请更换1588-V2.0版本进行同步"; break;
                    case 0x0404: faultDesc = "PPS同步异常，请检查PPS及GPS信号"; break;
                    case 0x0405: faultDesc = "时间同步曾经发生过异常，现已恢复正常，请检查发生异常原因"; break;
                    case 0x0406: faultDesc = "时间同步精度低，请检查同步源"; break;
                    case 0x0407: faultDesc = "缺失GPS信号导致GPS同步失败，请检查GPS信号"; break;
                    case 0x0408: faultDesc = "缺失PPS信号导致GPS同步失败，请检查PPS信号"; break;
                    case 0x0409: faultDesc = "GPS信号异常，请检查GPS信号源"; break;
                    case 0x040A: faultDesc = "PTP和gPTP信号同时存在，同步存在问题；请检查网络拓扑，单独使用PTP或gPTP同步"; break;
                    default:     faultDesc = "未知故障"; break;
                }

                QString faultCodeStr = QString("%1").arg(hmsCode, 8, 16, QChar('0')).toUpper();

                faultInfo.append(QString("[%1] 0x%2 - %3: %4")
                                 .arg(i)
                                 .arg(faultCodeStr)
                                 .arg(levelDesc)
                                 .arg(faultDesc));
            }

            return faultInfo.isEmpty() ? "无故障" : faultInfo.join("\n");
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
} // namespace LidarParameterService

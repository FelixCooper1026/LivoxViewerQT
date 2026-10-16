#ifndef LIVOXCORE_LIDARDIAGNOSTICS_H
#define LIVOXCORE_LIDARDIAGNOSTICS_H

#include "livox_lidar_def.h"

#include <QChar>
#include <QString>
#include <QStringList>
#include <QVector>

#include <algorithm>
#include <cstdint>
#include <cstring>

namespace LivoxCore {

struct HmsCodeInfo {
    int index = 0;
    uint32_t code = 0;
    uint16_t faultId = 0;
    uint8_t level = 0;
};

inline QVector<HmsCodeInfo> parseHmsCodes(const uint8_t* value, uint16_t length)
{
    QVector<HmsCodeInfo> codes;
    const int count = std::min<int>(8, int(length / sizeof(uint32_t)));
    for (int i = 0; i < count; ++i) {
        uint32_t hmsCode = 0;
        std::memcpy(&hmsCode, value + i * sizeof(uint32_t), sizeof(uint32_t));
        if (hmsCode == 0) {
            continue;
        }
        codes.append(HmsCodeInfo{i, hmsCode, uint16_t((hmsCode >> 16) & 0xFFFF), uint8_t(hmsCode & 0xFF)});
    }
    return codes;
}

inline int hmsSeverity(uint8_t level)
{
    switch (level) {
    case 0x01: return 1;
    case 0x02: return 2;
    case 0x03: return 3;
    case 0x04: return 4;
    default: return 0;
    }
}

inline int maxHmsSeverity(const QVector<HmsCodeInfo>& codes)
{
    int severity = 0;
    for (const HmsCodeInfo& code : codes) {
        severity = std::max(severity, hmsSeverity(code.level));
    }
    return severity;
}

inline QString hmsSeverityName(int severity)
{
    switch (severity) {
    case 1: return QStringLiteral("Info");
    case 2: return QStringLiteral("Warning");
    case 3: return QStringLiteral("Error");
    case 4: return QStringLiteral("Fatal");
    default: return QStringLiteral("Normal");
    }
}

inline QString hmsSeverityColor(int severity)
{
    switch (severity) {
    case 1: return QStringLiteral("#2D7DD2");
    case 2: return QStringLiteral("#D98C00");
    case 3: return QStringLiteral("#D64545");
    case 4: return QStringLiteral("#8B0000");
    default: return QStringLiteral("#000000");
    }
}

inline QString hmsSummary(const QVector<HmsCodeInfo>& codes)
{
    if (codes.isEmpty()) {
        return QStringLiteral("诊断码: 无");
    }

    QStringList codeTexts;
    for (const HmsCodeInfo& code : codes) {
        codeTexts.append(QStringLiteral("0x%1").arg(code.code, 8, 16, QChar('0')).toUpper());
    }
    return QStringLiteral("诊断码: %1 %2").arg(hmsSeverityName(maxHmsSeverity(codes)), codeTexts.join(QStringLiteral(", ")));
}

// Livox 官方 HMS 表：Mid-360 系列和 Avia2 的 0x0210 含义不同。
// https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/hms_code_mid360.html
// https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/avia2/hms_code_avia2.html
inline QString hmsFaultDescription(uint16_t faultId, uint8_t deviceType)
{
    switch (faultId) {
    case 0x0000: return QStringLiteral("无故障");
    case 0x0102: return QStringLiteral("设备运行环境温度偏高;请检查环境温度，或排查散热措施");
    case 0x0103: return QStringLiteral("设备运行环境温度较高;请检查环境温度，或排查散热措施");
    case 0x0104: return QStringLiteral("设备球罩存在脏污或附近有遮挡物，请及时清洗擦拭设备球罩，或确保球罩0.1m范围内无遮挡物");
    case 0x0105: return QStringLiteral("设备固件升级过程中出现错误;请重新进行固件升级");
    case 0x0106: return QStringLiteral("风扇转速可能异常；请检查风扇状态");
    case 0x0111:
    case 0x0112: return QStringLiteral("设备内部器件温度异常;请检查环境温度，或排查散热措施");
    case 0x0113: return QStringLiteral("设备内部IMU器件暂停工作;请重启设备恢复");
    case 0x0114: return QStringLiteral("设备运行环境温度高;请检查环境温度，或排查散热措施");
    case 0x0115: return QStringLiteral("设备运行环境温度超过承受极限，设备已停止工作;请检查环境温度，或排查散热措施");
    case 0x0116: return QStringLiteral("设备外部电压异常;请检查外部电压");
    case 0x0117: return QStringLiteral("设备参数异常;请尝试重启设备恢复");
    case 0x0118: return QStringLiteral("设备内部器件损坏，无法正常工作，请联系维修人员");
    case 0x0201: return QStringLiteral("扫描模块低温加热中");
    case 0x0210:
        if (deviceType == kLivoxLidarTypeAvia2) {
            return QStringLiteral("扫描模块正在切换转速；请等待转速稳定");
        }
        if (deviceType == 0) {
            return QStringLiteral("扫描模块状态（Avia2：转速切换；Mid-360系列：扫描模块异常）");
        }
        [[fallthrough]];
    case 0x0211: case 0x0212: case 0x0213: case 0x0214: case 0x0215:
    case 0x0216: case 0x0217: case 0x0218: case 0x0219:
        return QStringLiteral("扫描模块异常，请尝试：1.检查供电是否正常 2.重启设备 3.更新最新固件");
    case 0x021A: case 0x021B:
        return QStringLiteral("扫描模块异常；请等待，若持续过久请尝试重启设备恢复");
    case 0x021C: return QStringLiteral("扫描模块码盘脏污、转速异常；请重启设备，未恢复请返厂检查");
    case 0x0304: return QStringLiteral("TIA 直流异常，测距点云异常；请尝试重启设备恢复");
    case 0x0310: case 0x0311: case 0x0312:
        return QStringLiteral("内部模块过温；请检查设备散热情况");
    case 0x0313: return QStringLiteral("测距模块光功率裕量警告，请注意人眼安全；请等待，若持续过久请重启设备");
    case 0x0314: return QStringLiteral("测距模块光功率裕量异常，请注意人眼安全；请尝试重启设备恢复");
    case 0x0315: return QStringLiteral("测距模块可能异常；请尝试重启设备或联系 Livox 技术支持");
    case 0x0316: return QStringLiteral("测试环境检测警告；请检查并移除场景中的高反或全反射物体，否则可能损坏设备");
    case 0x0317: case 0x0318:
        return QStringLiteral("测试环境检测错误；请移除场景中的高反或全反射物体后重启设备，否则可能损坏设备");
    case 0x0401: return QStringLiteral("检测到以太网连接曾经断开过，现已恢复正常，请检查以太网链路是否存在异常");
    case 0x0402: return QStringLiteral("PTP同步中断，或者时间跳变太大，请排查PTP时钟源是否工作正常");
    case 0x0403: return QStringLiteral("PTP版本为1588-V2.1版本，设备不支持该版本，请更换1588-V2.0版本进行同步");
    case 0x0404: return QStringLiteral("PPS同步异常，请检查PPS及GPS信号");
    case 0x0405: return QStringLiteral("时间同步曾经发生过异常，现已恢复正常，请检查发生异常原因");
    case 0x0406: return QStringLiteral("时间同步精度低，请检查同步源");
    case 0x0407: return QStringLiteral("缺失GPS信号导致GPS同步失败，请检查GPS信号");
    case 0x0408: return QStringLiteral("缺失PPS信号导致GPS同步失败，请检查PPS信号");
    case 0x0409: return QStringLiteral("GPS信号异常，请检查GPS信号源");
    case 0x040A: return QStringLiteral("PTP和gPTP信号同时存在，同步存在问题；请检查网络拓扑，单独使用PTP或gPTP同步");
    default: return QStringLiteral("未知故障 ID 0x%1").arg(faultId, 4, 16, QChar('0')).toUpper();
    }
}

inline QString formatHmsCodes(const uint8_t* value, uint16_t length, uint8_t deviceType)
{
    QStringList faults;
    for (const HmsCodeInfo& code : parseHmsCodes(value, length)) {
        QString level;
        switch (code.level) {
        case 0x00: level = QStringLiteral("无故障"); break;
        case 0x01: level = QStringLiteral("Info 消息"); break;
        case 0x02: level = QStringLiteral("Warning 警告"); break;
        case 0x03: level = QStringLiteral("Error 错误"); break;
        case 0x04: level = QStringLiteral("Fatal 严重错误"); break;
        default: level = QStringLiteral("未知级别"); break;
        }
        const QString hex = QStringLiteral("0x%1").arg(code.code, 8, 16, QChar('0')).toUpper();
        faults.append(QStringLiteral("[%1] %2 - %3: %4").arg(code.index).arg(
            hex, level, hmsFaultDescription(code.faultId, deviceType)));
    }
    return faults.isEmpty() ? QStringLiteral("无故障") : faults.join(QLatin1Char('\n'));
}

} // namespace LivoxCore

#endif // LIVOXCORE_LIDARDIAGNOSTICS_H

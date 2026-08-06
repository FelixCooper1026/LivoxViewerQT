#include "PushMsgParser.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#endif

#include <QStringList>

#include <cstring>

namespace PushMsgParser {

namespace {

// 定义设备配置结构体，方便后续扩展
struct DeviceTypeConfig {
    uint8_t typeId;
    QString modelName;
};

// 集中管理固件版本与设备型号的映射关系
DeviceTypeConfig getDeviceConfig(uint8_t firmwareMajor)
{
    switch (firmwareMajor) {
        case 13: return { 9,  QStringLiteral("Mid360") };
        case 35: return { 35, QStringLiteral("Mid360S") };
        case 40: return { 40, QStringLiteral("Avia2") };
        case 41: return { 41, QStringLiteral("Mid360L") };
        default: return { 0,  QStringLiteral("未知") };
    }
}

bool isValidPushSn(const QString& sn)
{
    return !sn.isEmpty() && sn != QLatin1String("DEFAULT_LIDAR");
}

// 与 LVX2 文件头/包内 lidar_id 字段一致的字节序（见 lvx2_writer 写入时的 swap）
uint32_t toLvx2LidarId(uint32_t hostOrderIp)
{
    return ((hostOrderIp & 0xFFu) << 24) |
           ((hostOrderIp & 0xFF00u) << 8) |
           ((hostOrderIp & 0xFF0000u) >> 8) |
           ((hostOrderIp & 0xFF000000u) >> 24);
}

uint16_t readLe16(const uint8_t* data)
{
    return uint16_t(data[0]) | (uint16_t(data[1]) << 8);
}

uint32_t readLe32(const uint8_t* data)
{
    return uint32_t(data[0]) |
           (uint32_t(data[1]) << 8) |
           (uint32_t(data[2]) << 16) |
           (uint32_t(data[3]) << 24);
}

uint64_t readLe64(const uint8_t* data)
{
    return uint64_t(readLe32(data)) | (uint64_t(readLe32(data + 4)) << 32);
}

int32_t readLeInt32(const uint8_t* data)
{
    const uint32_t value = readLe32(data);
    int32_t result = 0;
    std::memcpy(&result, &value, sizeof(result));
    return result;
}

int64_t readLeInt64(const uint8_t* data)
{
    const uint64_t value = readLe64(data);
    int64_t result = 0;
    std::memcpy(&result, &value, sizeof(result));
    return result;
}

float readLeFloat(const uint8_t* data)
{
    const uint32_t value = readLe32(data);
    float result = 0.0f;
    std::memcpy(&result, &value, sizeof(result));
    return result;
}

QString hexValue(uint64_t value, int width)
{
    const QString digits = QString::number(qulonglong(value), 16).rightJustified(width, QLatin1Char('0')).toUpper();
    return QStringLiteral("0x%1").arg(digits);
}

QString hexBytes(const uint8_t* data, size_t length)
{
    return QByteArray(reinterpret_cast<const char*>(data), int(length)).toHex(' ').toUpper();
}

QString textValue(const uint8_t* data, size_t length)
{
    QString value = QString::fromLatin1(reinterpret_cast<const char*>(data), int(length));
    const int nullPosition = value.indexOf(QChar('\0'));
    if (nullPosition >= 0) {
        value.truncate(nullPosition);
    }
    return value.trimmed();
}

QString ipv4Value(const uint8_t* data)
{
    return QStringLiteral("%1.%2.%3.%4")
        .arg(data[0])
        .arg(data[1])
        .arg(data[2])
        .arg(data[3]);
}

QString dottedBytes(const uint8_t* data, size_t length)
{
    QStringList parts;
    for (size_t i = 0; i < length; ++i) {
        parts.append(QString::number(data[i]));
    }
    return parts.join(QLatin1Char('.'));
}

QString commandDescription(uint16_t commandId)
{
    switch (commandId) {
    case 0x0000: return QStringLiteral("广播发现");
    case 0x0100: return QStringLiteral("参数信息配置");
    case 0x0101: return QStringLiteral("雷达信息查询");
    case 0x0102: return QStringLiteral("雷达信息推送");
    case 0x0200: return QStringLiteral("请求设备重启");
    case 0x0201: return QStringLiteral("恢复出厂设置");
    case 0x0202: return QStringLiteral("设置雷达 GPS 时间同步时间戳");
    case 0x0300: return QStringLiteral("日志文件推送");
    case 0x0301: return QStringLiteral("日志采集配置");
    case 0x0302: return QStringLiteral("日志系统时间同步");
    case 0x0303: return QStringLiteral("Debug 点云采集配置");
    case 0x0400: return QStringLiteral("请求开始升级");
    case 0x0401: return QStringLiteral("固件数据传输");
    case 0x0402: return QStringLiteral("固件传输结束");
    case 0x0403: return QStringLiteral("获取固件升级状态");
    default: return QStringLiteral("未知命令");
    }
}

QString commandTypeDescription(uint8_t commandType)
{
    if (commandType == 0x00) {
        return QStringLiteral("REQ 请求");
    }
    if (commandType == 0x01) {
        return QStringLiteral("ACK 应答");
    }
    return QStringLiteral("未知类型 %1").arg(hexValue(commandType, 2));
}

QString senderDescription(uint8_t senderType)
{
    if (senderType == 0x00) {
        return QStringLiteral("上位机");
    }
    if (senderType == 0x01) {
        return QStringLiteral("雷达");
    }
    return QStringLiteral("未知发送端 %1").arg(hexValue(senderType, 2));
}

QString dataTypeDescription(uint8_t dataType)
{
    switch (dataType) {
    case 0: return QStringLiteral("IMU 数据");
    case 1: return QStringLiteral("直角坐标 32-bit 点云");
    case 2: return QStringLiteral("直角坐标 16-bit 点云");
    case 3: return QStringLiteral("球坐标点云");
    case 17: return QStringLiteral("双回波直角坐标点云");
    default: return QStringLiteral("未知数据类型 %1").arg(hexValue(dataType, 2));
    }
}

QString timeTypeDescription(uint8_t timeType)
{
    switch (timeType) {
    case 0: return QStringLiteral("无同步源，雷达上电后经过时间");
    case 1: return QStringLiteral("gPTP/PTP 同步，Master 时钟时间");
    case 2: return QStringLiteral("GPS 时间同步");
    case 5: return QStringLiteral("NTP 时间同步");
    default: return QStringLiteral("未知时间戳类型 %1").arg(hexValue(timeType, 2));
    }
}

QString returnCodeDescription(uint8_t returnCode)
{
    switch (returnCode) {
    case 0x00: return QStringLiteral("执行成功");
    case 0x01: return QStringLiteral("执行失败");
    case 0x02: return QStringLiteral("当前状态不支持");
    case 0x03: return QStringLiteral("设置值超出范围");
    case 0x20: return QStringLiteral("参数不支持");
    case 0x21: return QStringLiteral("参数需重启生效");
    case 0x22: return QStringLiteral("参数只读，不支持写入");
    case 0x23: return QStringLiteral("请求参数长度错误，或 ACK 数据包超过最大长度");
    case 0x24: return QStringLiteral("参数 key_num 和 key_list 不匹配");
    case 0x30: return QStringLiteral("公钥签名验证错误");
    case 0x31: return QStringLiteral("固件摘要签名验证错误");
    case 0x32: return QStringLiteral("固件类型不匹配");
    case 0x33: return QStringLiteral("固件长度超出范围");
    case 0x34: return QStringLiteral("固件擦除中");
    default: return QStringLiteral("未知返回码");
    }
}

void appendField(QStringList& lines, const QString& name, const QString& value)
{
    lines.append(QStringLiteral("%1：%2").arg(name, value));
}

void appendReturnCode(QStringList& lines, uint8_t returnCode)
{
    appendField(lines,
                QStringLiteral("返回码"),
                QStringLiteral("%1（%2）").arg(hexValue(returnCode, 2), returnCodeDescription(returnCode)));
}

QString workModeDescription(uint8_t value)
{
    switch (value) {
    case 0x01: return QStringLiteral("采样");
    case 0x02: return QStringLiteral("待机");
    case 0x04: return QStringLiteral("错误");
    case 0x05: return QStringLiteral("自检");
    case 0x06: return QStringLiteral("电机启动");
    case 0x08: return QStringLiteral("升级");
    case 0x09: return QStringLiteral("就绪");
    default: return QStringLiteral("未知状态 %1").arg(hexValue(value, 2));
    }
}

QString faultDescription(uint16_t faultId)
{
    switch (faultId) {
    case 0x0102: return QStringLiteral("设备运行环境温度偏高；请检查环境温度，或排查散热措施");
    case 0x0103: return QStringLiteral("设备运行环境温度较高；请检查环境温度，或排查散热措施");
    case 0x0104: return QStringLiteral("设备球罩存在脏污或附近有遮挡物，请清洁球罩并确保球罩 0.1m 范围内无遮挡物");
    case 0x0105: return QStringLiteral("设备固件升级过程中出现错误；请重新进行固件升级");
    case 0x0111:
    case 0x0112: return QStringLiteral("设备内部器件温度异常；请检查环境温度，或排查散热措施");
    case 0x0113: return QStringLiteral("设备内部 IMU 器件暂停工作；请重启设备恢复");
    case 0x0114: return QStringLiteral("设备运行环境温度高；请检查环境温度，或排查散热措施");
    case 0x0115: return QStringLiteral("设备运行环境温度超过承受极限，设备已停止工作；请检查环境温度或散热措施");
    case 0x0116: return QStringLiteral("设备外部电压异常；请检查外部电压");
    case 0x0117: return QStringLiteral("设备参数异常；请尝试重启设备恢复");
    case 0x0118: return QStringLiteral("设备内部器件损坏，无法正常工作，请联系维修人员");
    case 0x0201: return QStringLiteral("扫描模块低温加热中");
    case 0x0210:
    case 0x0211:
    case 0x0212:
    case 0x0213:
    case 0x0214:
    case 0x0215:
    case 0x0216:
    case 0x0217:
    case 0x0218:
    case 0x0219: return QStringLiteral("扫描模块异常，请检查供电、重启设备并更新到最新固件");
    case 0x0401: return QStringLiteral("检测到以太网连接曾经断开，现已恢复；请检查以太网链路");
    case 0x0402: return QStringLiteral("PTP 同步中断或时间跳变过大；请排查 PTP 时钟源");
    case 0x0403: return QStringLiteral("设备不支持 IEEE 1588 v2.1；请改用 IEEE 1588 v2.0");
    case 0x0404: return QStringLiteral("PPS 同步异常；请检查 PPS 及 GPS 信号");
    case 0x0405: return QStringLiteral("时间同步曾发生异常，现已恢复；请检查异常原因");
    case 0x0406: return QStringLiteral("时间同步精度低；请检查同步源");
    case 0x0407: return QStringLiteral("缺失 GPS 信号导致 GPS 同步失败；请检查 GPS 信号");
    case 0x0408: return QStringLiteral("缺失 PPS 信号导致 GPS 同步失败；请检查 PPS 信号");
    case 0x0409: return QStringLiteral("GPS 信号异常；请检查 GPS 信号源");
    case 0x040A: return QStringLiteral("PTP 和 gPTP 信号同时存在；请检查网络拓扑并仅使用一种同步源");
    default: return QStringLiteral("未知故障 ID %1").arg(hexValue(faultId, 4));
    }
}

QString faultLevelDescription(uint8_t level)
{
    switch (level) {
    case 0x01: return QStringLiteral("Info 消息");
    case 0x02: return QStringLiteral("Warning 警告");
    case 0x03: return QStringLiteral("Error 错误");
    case 0x04: return QStringLiteral("Fatal 严重错误");
    default: return QStringLiteral("未知等级");
    }
}

QString formatHmsCodes(const uint8_t* data, size_t length)
{
    QStringList faults;
    const size_t codeCount = length / 4;
    for (size_t i = 0; i < codeCount; ++i) {
        const uint32_t code = readLe32(data + i * 4);
        if (code == 0) {
            continue;
        }
        const uint16_t faultId = uint16_t(code >> 16);
        const uint8_t faultLevel = uint8_t(code & 0xFF);
        faults.append(QStringLiteral("[%1] %2 [%3] %4")
                          .arg(faults.size() + 1)
                          .arg(hexValue(code, 8))
                          .arg(faultLevelDescription(faultLevel))
                          .arg(faultDescription(faultId)));
    }
    return faults.isEmpty() ? QStringLiteral("无故障") : faults.join(QLatin1Char('\n'));
}

QString keyName(uint16_t key)
{
    switch (key) {
    case 0x0000: return QStringLiteral("点云坐标格式");
    case 0x0001: return QStringLiteral("扫描模式");
    case 0x0003: return QStringLiteral("点云发送控制");
    case 0x0004: return QStringLiteral("雷达 IP 信息");
    case 0x0005: return QStringLiteral("推送数据目标地址");
    case 0x0006: return QStringLiteral("点云数据目标地址");
    case 0x0007: return QStringLiteral("IMU 数据目标地址");
    case 0x0012: return QStringLiteral("外参配置");
    case 0x0013: return QStringLiteral("盲区范围设置");
    case 0x0015: return QStringLiteral("FOV0 配置");
    case 0x0016: return QStringLiteral("FOV1 配置");
    case 0x0017: return QStringLiteral("FOV 使能");
    case 0x0018: return QStringLiteral("探测模式");
    case 0x0019: return QStringLiteral("功能线配置");
    case 0x001A: return QStringLiteral("目标工作模式");
    case 0x001B: return QStringLiteral("窗口加热支持");
    case 0x001C: return QStringLiteral("IMU 数据输出");
    case 0x001D: return QStringLiteral("FUSA 诊断功能");
    case 0x001E: return QStringLiteral("强制加热");
    case 0x0020: return QStringLiteral("开机初始化工作模式");
    case 0x0021: return QStringLiteral("电机转速模式");
    case 0x0022: return QStringLiteral("FOV 模式");
    case 0x0024: return QStringLiteral("回波模式");
    case 0x0025: return QStringLiteral("NTP 服务器 IP");
    case 0x0026: return QStringLiteral("异常时间过滤");
    case 0x8000: return QStringLiteral("SN 号");
    case 0x8001: return QStringLiteral("产品信息");
    case 0x8002: return QStringLiteral("固件版本");
    case 0x8003: return QStringLiteral("Loader 版本");
    case 0x8004: return QStringLiteral("硬件版本");
    case 0x8005: return QStringLiteral("MAC 地址");
    case 0x8006: return QStringLiteral("当前工作状态");
    case 0x8007: return QStringLiteral("核心板温度");
    case 0x8008: return QStringLiteral("上电次数");
    case 0x8009: return QStringLiteral("雷达本地时间");
    case 0x800A: return QStringLiteral("上一次同步的 Master 时间");
    case 0x800B: return QStringLiteral("时间偏移");
    case 0x800C: return QStringLiteral("时间同步方式");
    case 0x800D: return QStringLiteral("状态码");
    case 0x800E: return QStringLiteral("异常码");
    case 0x800F: return QStringLiteral("Flash 状态");
    case 0x8010: return QStringLiteral("固件类型");
    case 0x8011: return QStringLiteral("HMS 诊断码");
    case 0x8012: return QStringLiteral("当前窗口加热状态");
    default: return QStringLiteral("未知参数");
    }
}

QString formatKeyValue(uint16_t key, const uint8_t* data, size_t length)
{
    switch (key) {
    case 0x0000:
        if (length >= 1) {
            switch (data[0]) {
            case 0x01: return QStringLiteral("直角坐标（32 bits）");
            case 0x02: return QStringLiteral("直角坐标（16 bits）");
            case 0x03: return QStringLiteral("球坐标");
            }
        }
        break;
    case 0x0001:
        if (length >= 1) {
            switch (data[0]) {
            case 0x00: return QStringLiteral("非重复扫描");
            case 0x01: return QStringLiteral("重复扫描");
            case 0x02: return QStringLiteral("低帧率重复扫描模式");
            }
        }
        break;
    case 0x0003:
        if (length >= 1) {
            return data[0] == 0 ? QStringLiteral("进入工作模式发送点云")
                                : QStringLiteral("进入工作模式不发送点云");
        }
        break;
    case 0x0004:
        if (length >= 12) {
            return QStringLiteral("IP：%1，子网掩码：%2，网关：%3")
                .arg(ipv4Value(data), ipv4Value(data + 4), ipv4Value(data + 8));
        }
        break;
    case 0x0005:
    case 0x0006:
    case 0x0007:
        if (length >= 6) {
            return QStringLiteral("%1:%2").arg(ipv4Value(data)).arg(readLe16(data + 4));
        }
        break;
    case 0x0012:
        if (length >= 24) {
            return QStringLiteral("Roll：%1°，Pitch：%2°，Yaw：%3°，X：%4mm，Y：%5mm，Z：%6mm")
                .arg(readLeFloat(data), 0, 'f', 2)
                .arg(readLeFloat(data + 4), 0, 'f', 2)
                .arg(readLeFloat(data + 8), 0, 'f', 2)
                .arg(readLeInt32(data + 12))
                .arg(readLeInt32(data + 16))
                .arg(readLeInt32(data + 20));
        }
        break;
    case 0x0013:
        if (length >= 4) {
            return QStringLiteral("%1 cm（范围 50～200 cm）").arg(readLe32(data));
        }
        break;
    case 0x0015:
    case 0x0016:
        if (length >= 16) {
            return QStringLiteral("水平：%1°～%2°，垂直：%3°～%4°")
                .arg(readLeInt32(data))
                .arg(readLeInt32(data + 4))
                .arg(readLeInt32(data + 8))
                .arg(readLeInt32(data + 12));
        }
        break;
    case 0x0017:
        if (length >= 1) {
            return QStringLiteral("FOV0：%1，FOV1：%2")
                .arg((data[0] & 0x01) != 0 ? QStringLiteral("开启") : QStringLiteral("关闭"),
                     (data[0] & 0x02) != 0 ? QStringLiteral("开启") : QStringLiteral("关闭"));
        }
        break;
    case 0x0018:
        if (length >= 1) {
            return data[0] == 0 ? QStringLiteral("正常探测模式") : QStringLiteral("敏感探测模式");
        }
        break;
    case 0x0019:
        return dottedBytes(data, length);
    case 0x001A:
    case 0x8006:
        if (length >= 1) {
            return workModeDescription(data[0]);
        }
        break;
    case 0x001B:
        if (length >= 1) {
            return data[0] == 0 ? QStringLiteral("禁止窗口加热功能") : QStringLiteral("允许窗口加热功能");
        }
        break;
    case 0x001C:
        if (length >= 1) {
            return data[0] == 0 ? QStringLiteral("关闭") : QStringLiteral("开启");
        }
        break;
    case 0x001D:
        if (length >= 1) {
            return data[0] == 0 ? QStringLiteral("关闭 FUSA 诊断功能") : QStringLiteral("开启 FUSA 诊断功能");
        }
        break;
    case 0x001E:
        if (length >= 1) {
            return data[0] == 0 ? QStringLiteral("关闭强制加热") : QStringLiteral("开启强制加热");
        }
        break;
    case 0x0020:
        if (length >= 1) {
            if (data[0] == 0) return QStringLiteral("待机状态（默认值）");
            if (data[0] == 1) return QStringLiteral("采样状态");
            if (data[0] == 2) return QStringLiteral("待机状态");
        }
        break;
    case 0x0021:
        if (length >= 1) {
            return data[0] == 0 ? QStringLiteral("默认转速") : QStringLiteral("低转速");
        }
        break;
    case 0x0022:
        if (length >= 1) {
            return data[0] == 0 ? QStringLiteral("Focus Detection Mode（小 FOV）")
                                : QStringLiteral("Normal Detection Mode（大 FOV）");
        }
        break;
    case 0x0024:
        if (length >= 1) {
            return data[0] == 0 ? QStringLiteral("最强回波") : QStringLiteral("第一回波");
        }
        break;
    case 0x0025:
        if (length >= 4) {
            return ipv4Value(data);
        }
        break;
    case 0x0026:
        if (length >= 1) {
            return data[0] == 0
                ? QStringLiteral("无异常时间过滤（时间回退可能导致点云中断）")
                : QStringLiteral("有异常时间过滤（时间回退不会导致点云中断）");
        }
        break;
    case 0x8000:
    case 0x8001:
        return textValue(data, length);
    case 0x8002:
        if (length >= 4) {
            return QStringLiteral("%1.%2.%3")
                .arg(data[0])
                .arg(data[1])
                .arg(uint16_t(data[2]) * 100 + data[3], 4, 10, QLatin1Char('0'));
        }
        break;
    case 0x8003:
    case 0x8004:
        return dottedBytes(data, length);
    case 0x8005:
        if (length >= 6) {
            return QStringLiteral("%1:%2:%3:%4:%5:%6")
                .arg(data[0], 2, 16, QLatin1Char('0'))
                .arg(data[1], 2, 16, QLatin1Char('0'))
                .arg(data[2], 2, 16, QLatin1Char('0'))
                .arg(data[3], 2, 16, QLatin1Char('0'))
                .arg(data[4], 2, 16, QLatin1Char('0'))
                .arg(data[5], 2, 16, QLatin1Char('0')).toUpper();
        }
        break;
    case 0x8007:
        if (length >= 4) {
            return QStringLiteral("%1 ℃").arg(double(readLeInt32(data)) / 100.0, 0, 'f', 2);
        }
        break;
    case 0x8008:
        if (length >= 4) {
            return QStringLiteral("%1 次").arg(readLe32(data));
        }
        break;
    case 0x8009:
    case 0x800A:
        if (length >= 8) {
            return QStringLiteral("%1 ns").arg(qulonglong(readLe64(data)));
        }
        break;
    case 0x800B:
        if (length >= 8) {
            return QStringLiteral("%1 μs").arg(double(readLeInt64(data)) / 1000.0, 0, 'f', 3);
        }
        break;
    case 0x800C:
        if (length >= 1) {
            if (data[0] == 0) return QStringLiteral("无时间同步");
            if (data[0] == 1) return QStringLiteral("PTP（IEEE 1588 v2.0）");
            if (data[0] == 2) return QStringLiteral("GPS");
        }
        break;
    case 0x800D:
        return QStringLiteral("0x%1").arg(QString::fromLatin1(QByteArray(reinterpret_cast<const char*>(data), int(length)).toHex().toUpper()));
    case 0x800E:
        if (length >= 2) {
            return hexValue(readLe16(data), 4);
        }
        break;
    case 0x800F:
        if (length >= 1) {
            return data[0] == 0 ? QStringLiteral("idle") : QStringLiteral("busy");
        }
        break;
    case 0x8010:
        if (length >= 1) {
            return data[0] == 0 ? QStringLiteral("loader") : QStringLiteral("application_image");
        }
        break;
    case 0x8011:
        return formatHmsCodes(data, length);
    case 0x8012:
        if (length >= 1) {
            return data[0] == 0 ? QStringLiteral("未加热") : QStringLiteral("正在加热");
        }
        break;
    }
    return hexBytes(data, length);
}

void appendKvList(const uint8_t* data,
                  size_t length,
                  size_t offset,
                  uint16_t count,
                  QStringList& lines,
                  QStringList& summaryParts)
{
    for (uint16_t i = 0; i < count && offset + 4 <= length; ++i) {
        const uint16_t key = readLe16(data + offset);
        const uint16_t valueLength = readLe16(data + offset + 2);
        offset += 4;
        if (offset + valueLength > length) {
            break;
        }

        const QString value = formatKeyValue(key, data + offset, valueLength);
        appendField(lines,
                    QStringLiteral("Key %1 %2").arg(hexValue(key, 4), keyName(key)),
                    value);
        if (key == 0x8000) {
            summaryParts.append(QStringLiteral("SN %1").arg(value));
        } else if (key == 0x8006) {
            summaryParts.append(QStringLiteral("状态 %1").arg(value));
        } else if (key == 0x8007) {
            summaryParts.append(QStringLiteral("温度 %1").arg(value));
        } else if (key == 0x8011 && value != QStringLiteral("无故障")) {
            summaryParts.append(QStringLiteral("HMS 有故障"));
        }
        offset += valueLength;
    }
}

void appendKeyList(const uint8_t* data, size_t length, size_t offset, uint16_t count, QStringList& lines)
{
    for (uint16_t i = 0; i < count && offset + 2 <= length; ++i) {
        const uint16_t key = readLe16(data + offset);
        appendField(lines,
                    QStringLiteral("查询参数 %1").arg(i + 1),
                    QStringLiteral("%1 %2").arg(hexValue(key, 4), keyName(key)));
        offset += 2;
    }
}

void appendControlData(uint16_t commandId,
                       uint8_t commandType,
                       const uint8_t* data,
                       size_t length,
                       bool forcePush,
                       QStringList& lines,
                       QStringList& summaryParts)
{
    if (forcePush || commandId == 0x0102) {
        if (length >= 4) {
            const uint16_t keyCount = readLe16(data);
            appendField(lines, QStringLiteral("参数数量"), QString::number(keyCount));
            appendField(lines, QStringLiteral("Data 预留位 (Reserved)"), hexValue(readLe16(data + 2), 4));
            appendKvList(data, length, 4, keyCount, lines, summaryParts);
        }
        return;
    }

    switch (commandId) {
    case 0x0000:
        if (commandType == 0x01 && length >= 24) {
            appendReturnCode(lines, data[0]);
            appendField(lines, QStringLiteral("设备类型"), QString::number(data[1]));
            const QString serialNumber = textValue(data + 2, 16);
            const QString lidarIp = ipv4Value(data + 18);
            appendField(lines, QStringLiteral("雷达 SN"), serialNumber);
            appendField(lines, QStringLiteral("雷达 IP"), lidarIp);
            appendField(lines, QStringLiteral("控制端口"), QString::number(readLe16(data + 22)));
            summaryParts.append(QStringLiteral("SN %1").arg(serialNumber));
            summaryParts.append(lidarIp);
        }
        break;
    case 0x0100:
        if (commandType == 0x00 && length >= 4) {
            const uint16_t keyCount = readLe16(data);
            appendField(lines, QStringLiteral("参数数量"), QString::number(keyCount));
            appendField(lines, QStringLiteral("Data 预留位 (Reserved)"), hexValue(readLe16(data + 2), 4));
            appendKvList(data, length, 4, keyCount, lines, summaryParts);
        } else if (commandType == 0x01 && length >= 3) {
            appendReturnCode(lines, data[0]);
            appendField(lines, QStringLiteral("错误参数"), hexValue(readLe16(data + 1), 4));
        }
        break;
    case 0x0101:
        if (commandType == 0x00 && length >= 4) {
            const uint16_t keyCount = readLe16(data);
            appendField(lines, QStringLiteral("参数数量"), QString::number(keyCount));
            appendField(lines, QStringLiteral("Data 预留位 (Reserved)"), hexValue(readLe16(data + 2), 4));
            appendKeyList(data, length, 4, keyCount, lines);
        } else if (commandType == 0x01 && length >= 3) {
            appendReturnCode(lines, data[0]);
            const uint16_t keyCount = readLe16(data + 1);
            appendField(lines, QStringLiteral("参数数量"), QString::number(keyCount));
            appendKvList(data, length, 3, keyCount, lines, summaryParts);
        }
        break;
    case 0x0200:
        if (commandType == 0x00 && length >= 2) {
            appendField(lines, QStringLiteral("重启等待时间"), QStringLiteral("%1 ms").arg(readLe16(data)));
        } else if (commandType == 0x01 && length >= 1) {
            appendReturnCode(lines, data[0]);
        }
        break;
    case 0x0201:
        if (commandType == 0x00 && length >= 16) {
            appendField(lines, QStringLiteral("SN 保留字段"), textValue(data, 16));
        } else if (commandType == 0x01 && length >= 1) {
            appendReturnCode(lines, data[0]);
        }
        break;
    case 0x0202:
        if (commandType == 0x00 && length >= 9) {
            appendField(lines, QStringLiteral("时间类型"), QString::number(data[0]));
            appendField(lines, QStringLiteral("设置时间"), QStringLiteral("%1 ns").arg(qulonglong(readLe64(data + 1))));
        } else if (commandType == 0x01 && length >= 1) {
            appendReturnCode(lines, data[0]);
        }
        break;
    case 0x0300:
        if (commandType == 0x00 && length >= 16) {
            appendField(lines, QStringLiteral("日志类型"), QString::number(data[0]));
            appendField(lines, QStringLiteral("文件索引"), QString::number(data[1]));
            appendField(lines, QStringLiteral("文件数量"), QString::number(data[2]));
            appendField(lines, QStringLiteral("标志"), hexValue(data[3], 2));
            appendField(lines, QStringLiteral("时间戳"), QString::number(readLe32(data + 4)));
            appendField(lines, QStringLiteral("传输索引"), QString::number(readLe32(data + 10)));
            appendField(lines, QStringLiteral("日志数据长度"), QString::number(readLe16(data + 14)));
        } else if (commandType == 0x01 && length >= 8) {
            appendReturnCode(lines, data[0]);
            appendField(lines, QStringLiteral("日志类型"), QString::number(data[1]));
            appendField(lines, QStringLiteral("文件索引"), QString::number(data[2]));
            appendField(lines, QStringLiteral("传输索引"), QString::number(readLe32(data + 3)));
        }
        break;
    case 0x0301:
        if (commandType == 0x00 && length >= 2) {
            appendField(lines, QStringLiteral("日志类型"), QString::number(data[0]));
            appendField(lines, QStringLiteral("采集使能"), data[1] == 0 ? QStringLiteral("关闭") : QStringLiteral("开启"));
        } else if (commandType == 0x01 && length >= 1) {
            appendReturnCode(lines, data[0]);
        }
        break;
    case 0x0302:
        if (commandType == 0x00 && length >= 4) {
            appendField(lines, QStringLiteral("日志系统时间戳"), QString::number(readLe32(data)));
        } else if (commandType == 0x01 && length >= 1) {
            appendReturnCode(lines, data[0]);
        }
        break;
    case 0x0303:
        if (commandType == 0x00 && length >= 9) {
            appendField(lines, QStringLiteral("Debug 点云采集"), data[0] == 0 ? QStringLiteral("关闭") : QStringLiteral("开启"));
            appendField(lines, QStringLiteral("主机地址"), QStringLiteral("%1:%2").arg(ipv4Value(data + 1)).arg(readLe16(data + 5)));
            appendField(lines, QStringLiteral("保留字段"), hexValue(readLe16(data + 7), 4));
        } else if (commandType == 0x01 && length >= 1) {
            appendReturnCode(lines, data[0]);
        }
        break;
    case 0x0400:
        if (commandType == 0x00 && length >= 7) {
            appendField(lines, QStringLiteral("固件类型"), QString::number(data[0]));
            appendField(lines, QStringLiteral("加密类型"), QString::number(data[1]));
            appendField(lines, QStringLiteral("固件长度"), QString::number(readLe32(data + 2)));
            appendField(lines, QStringLiteral("设备类型"), QString::number(data[6]));
        } else if (commandType == 0x01 && length >= 1) {
            appendReturnCode(lines, data[0]);
        }
        break;
    case 0x0401:
        if (commandType == 0x00 && length >= 12) {
            appendField(lines, QStringLiteral("固件偏移"), QString::number(readLe32(data)));
            appendField(lines, QStringLiteral("当前长度"), QString::number(readLe32(data + 4)));
            appendField(lines, QStringLiteral("加密类型"), QString::number(data[8]));
            appendField(lines, QStringLiteral("固件数据字节数"), QString::number(length - 12));
        } else if (commandType == 0x01 && length >= 9) {
            appendReturnCode(lines, data[0]);
            appendField(lines, QStringLiteral("当前偏移"), QString::number(readLe32(data + 1)));
            appendField(lines, QStringLiteral("已接收长度"), QString::number(readLe32(data + 5)));
        }
        break;
    case 0x0402:
        if (commandType == 0x00 && length >= 2) {
            appendField(lines, QStringLiteral("校验类型"), QString::number(data[0]));
            appendField(lines, QStringLiteral("校验数据长度"), QString::number(data[1]));
            appendField(lines, QStringLiteral("校验数据"), hexBytes(data + 2, length - 2));
        } else if (commandType == 0x01 && length >= 1) {
            appendReturnCode(lines, data[0]);
        }
        break;
    case 0x0403:
        if (commandType == 0x01 && length >= 2) {
            appendReturnCode(lines, data[0]);
            appendField(lines, QStringLiteral("升级进度"), QStringLiteral("%1%").arg(data[1]));
        }
        break;
    default:
        if (length > 0) {
            appendField(lines, QStringLiteral("原始数据"), hexBytes(data, length));
        }
        break;
    }
}

ProtocolDecodeResult decodeDataPacket(bool isPointCloud,
                                      const uint8_t* payload,
                                      size_t payloadLen)
{
    if (payload == nullptr || payloadLen < 36) {
        return {};
    }

    const uint8_t version = payload[0];
    const uint16_t declaredLength = readLe16(payload + 1);
    const uint16_t timeInterval = readLe16(payload + 3);
    const uint16_t dotCount = readLe16(payload + 5);
    const uint16_t udpCount = readLe16(payload + 7);
    const uint8_t frameCount = payload[9];
    const uint8_t dataType = payload[10];
    const uint8_t timeType = payload[11];
    const uint32_t crc32 = readLe32(payload + 24);
    const uint64_t timestamp = readLe64(payload + 28);

    ProtocolDecodeResult result;
    result.valid = true;
    result.protocol = isPointCloud ? QStringLiteral("Livox PointCloud") : QStringLiteral("Livox IMU");
    result.summary = QStringLiteral("%1 · %2 个数据单元 · UDP %3 · 帧 %4")
        .arg(dataTypeDescription(dataType))
        .arg(dotCount)
        .arg(udpCount)
        .arg(frameCount);

    QStringList lines;
    lines.append(QStringLiteral("[Livox 数据包头]"));
    appendField(lines, QStringLiteral("协议版本"), QString::number(version));
    appendField(lines, QStringLiteral("声明长度"), QStringLiteral("%1 字节").arg(declaredLength));
    appendField(lines,
                QStringLiteral("采样时间间隔"),
                QStringLiteral("%1（%2 μs）").arg(timeInterval).arg(double(timeInterval) * 0.1, 0, 'f', 1));
    appendField(lines, QStringLiteral("数据单元数量"), QString::number(dotCount));
    appendField(lines, QStringLiteral("UDP 包计数"), QString::number(udpCount));
    appendField(lines, QStringLiteral("点云帧计数"), QString::number(frameCount));
    appendField(lines,
                QStringLiteral("数据类型"),
                QStringLiteral("%1（%2）").arg(dataType).arg(dataTypeDescription(dataType)));
    appendField(lines,
                QStringLiteral("时间戳类型"),
                QStringLiteral("%1（%2）").arg(timeType).arg(timeTypeDescription(timeType)));
    appendField(lines, QStringLiteral("数据头预留位"), hexBytes(payload + 12, 12));
    appendField(lines, QStringLiteral("CRC32"), hexValue(crc32, 8));
    appendField(lines, QStringLiteral("时间戳"), QStringLiteral("%1 ns").arg(qulonglong(timestamp)));
    appendField(lines, QStringLiteral("数据区长度"), QStringLiteral("%1 字节").arg(payloadLen - 36));

    if (!isPointCloud && dataType == 0 && payloadLen >= 60) {
        appendField(lines, QStringLiteral("Gyro X"), QStringLiteral("%1 rad/s").arg(readLeFloat(payload + 36), 0, 'f', 10));
        appendField(lines, QStringLiteral("Gyro Y"), QStringLiteral("%1 rad/s").arg(readLeFloat(payload + 40), 0, 'f', 10));
        appendField(lines, QStringLiteral("Gyro Z"), QStringLiteral("%1 rad/s").arg(readLeFloat(payload + 44), 0, 'f', 10));
        appendField(lines, QStringLiteral("Acc X"), QStringLiteral("%1 g").arg(readLeFloat(payload + 48), 0, 'f', 10));
        appendField(lines, QStringLiteral("Acc Y"), QStringLiteral("%1 g").arg(readLeFloat(payload + 52), 0, 'f', 10));
        appendField(lines, QStringLiteral("Acc Z"), QStringLiteral("%1 g").arg(readLeFloat(payload + 56), 0, 'f', 10));
    }

    result.details = lines.join(QLatin1Char('\n'));
    return result;
}

} // namespace

ProtocolDecodeResult decodeProtocolPacket(uint16_t sourcePort,
                                          uint16_t destinationPort,
                                          const uint8_t* payload,
                                          size_t payloadLen)
{
    const bool isBroadcast = sourcePort == 56000 || destinationPort == 56000;
    const bool isControl = sourcePort == 56100 || destinationPort == 56100;
    const bool isPush = sourcePort == 56200 || destinationPort == 56200;
    const bool isPointCloud = sourcePort == 56300 || destinationPort == 56300;
    const bool isImu = sourcePort == 56400 || destinationPort == 56400;
    if (isPointCloud || isImu) {
        return decodeDataPacket(isPointCloud, payload, payloadLen);
    }
    if ((!isBroadcast && !isControl && !isPush) || payload == nullptr || payloadLen < 24 || payload[0] != 0xAA) {
        return {};
    }

    const uint8_t version = payload[1];
    const uint16_t declaredLength = readLe16(payload + 2);
    const uint32_t sequenceNumber = readLe32(payload + 4);
    const uint16_t commandId = readLe16(payload + 8);
    const uint8_t commandType = payload[10];
    const uint8_t senderType = payload[11];

    ProtocolDecodeResult result;
    result.valid = true;
    result.protocol = isPush
        ? QStringLiteral("Livox Push")
        : (isBroadcast ? QStringLiteral("Livox Broadcast") : QStringLiteral("Livox Control"));

    QStringList lines;
    lines.append(QStringLiteral("[Livox 控制指令帧]"));
    appendField(lines, QStringLiteral("SOF"), hexValue(payload[0], 2));
    appendField(lines, QStringLiteral("协议版本"), QString::number(version));
    appendField(lines, QStringLiteral("声明长度"), QStringLiteral("%1 字节").arg(declaredLength));
    appendField(lines, QStringLiteral("序列号"), QString::number(sequenceNumber));
    appendField(lines,
                QStringLiteral("命令 ID"),
                QStringLiteral("%1（%2）").arg(hexValue(commandId, 4), commandDescription(commandId)));
    appendField(lines, QStringLiteral("命令类型"), commandTypeDescription(commandType));
    appendField(lines, QStringLiteral("发送端"), senderDescription(senderType));
    appendField(lines, QStringLiteral("保留字段"), hexBytes(payload + 12, 6));
    appendField(lines, QStringLiteral("CRC16"), hexValue(readLe16(payload + 18), 4));
    appendField(lines, QStringLiteral("CRC32"), hexValue(readLe32(payload + 20), 8));

    QStringList summaryParts;
    summaryParts.append(commandDescription(commandId));
    summaryParts.append(commandTypeDescription(commandType));
    if (payloadLen > 24) {
        lines.append(QStringLiteral(""));
        lines.append(QStringLiteral("[Data]"));
        appendControlData(commandId,
                          commandType,
                          payload + 24,
                          payloadLen - 24,
                          isPush,
                          lines,
                          summaryParts);
    }

    result.summary = summaryParts.join(QStringLiteral(" · "));
    result.details = lines.join(QLatin1Char('\n'));
    return result;
}

uint32_t ipToLidarId(const QString& ip)
{
    uint32_t hostOrderIp = 0;
#ifdef _WIN32
    const unsigned long addr = inet_addr(ip.toLatin1().constData());
    if (addr == INADDR_NONE) {
        return 0;
    }
    hostOrderIp = ntohl(addr);
#else
    struct in_addr addr {};
    if (inet_pton(AF_INET, ip.toLatin1().constData(), &addr) != 1) {
        return 0;
    }
    hostOrderIp = ntohl(addr.s_addr);
#endif
    return toLvx2LidarId(hostOrderIp);
}

QString lidarIdToIpString(uint32_t lidarId)
{
    const uint32_t netOrder = htonl(lidarId);
    return QString("%1.%2.%3.%4")
        .arg((netOrder >> 24) & 0xFF)
        .arg((netOrder >> 16) & 0xFF)
        .arg((netOrder >> 8) & 0xFF)
        .arg(netOrder & 0xFF);
}

void parsePushPayload(const uint8_t* payload, size_t payloadLen, PushDeviceRecord& device)
{
    if (payload == nullptr || payloadLen < 32) {
        return;
    }

    size_t index = 28;
    while (index + 4 <= payloadLen) {
        const uint16_t key = uint16_t(payload[index]) | (uint16_t(payload[index + 1]) << 8);
        index += 2;
        const uint16_t length = uint16_t(payload[index]) | (uint16_t(payload[index + 1]) << 8);
        index += 2;
        if (index + length > payloadLen) {
            break;
        }

        if (key == 0x8000 && length > 0) {
            device.lidarSn = QString::fromLatin1(reinterpret_cast<const char*>(payload + index), int(length)).trimmed();
            const int nullPos = device.lidarSn.indexOf(QChar('\0'));
            if (nullPos >= 0) {
                device.lidarSn.truncate(nullPos);
            }
        } else if (key == 0x8002 && length >= 4) {
            // 解析固件版本号 app_version (aa.bb.cc.dd)，取第一个字节 aa
            const uint8_t firmwareMajor = payload[index];
            const auto config = getDeviceConfig(firmwareMajor);
            device.deviceType = config.typeId;
            device.modelDisplay = config.modelName;
        } else if (key == 0x0004 && length >= 4) {
            const QString ip = QString("%1.%2.%3.%4")
                                   .arg(payload[index + 0])
                                   .arg(payload[index + 1])
                                   .arg(payload[index + 2])
                                   .arg(payload[index + 3]);
            device.lidarId = ipToLidarId(ip);
        } else if (key == 0x0012 && length >= 24) {
            float roll = 0.0f;
            float pitch = 0.0f;
            float yaw = 0.0f;
            int32_t x = 0;
            int32_t y = 0;
            int32_t z = 0;
            std::memcpy(&roll, payload + index, 4);
            std::memcpy(&pitch, payload + index + 4, 4);
            std::memcpy(&yaw, payload + index + 8, 4);
            std::memcpy(&x, payload + index + 12, 4);
            std::memcpy(&y, payload + index + 16, 4);
            std::memcpy(&z, payload + index + 20, 4);
            device.hasExtrinsic = true;
            device.offsetRoll = roll;
            device.offsetPitch = pitch;
            device.offsetYaw = yaw;
            device.offsetX = x / 1000.0f;
            device.offsetY = y / 1000.0f;
            device.offsetZ = z / 1000.0f;
        }

        index += length;
    }
}

void mergePushPacket(const QString& srcIp,
                     const uint8_t* payload,
                     size_t payloadLen,
                     QMap<QString, PushDeviceRecord>& devicesByIp)
{
    if (srcIp.isEmpty() || payload == nullptr || payloadLen < 32) {
        return;
    }

    PushDeviceRecord parsed;
    parsed.deviceType = 0; // 默认未知
    parsePushPayload(payload, payloadLen, parsed);
    if (!isValidPushSn(parsed.lidarSn)) {
        return;
    }

    if (parsed.lidarId == 0) {
        parsed.lidarId = ipToLidarId(srcIp);
    }

    auto it = devicesByIp.find(srcIp);
    if (it == devicesByIp.end()) {
        if (parsed.modelDisplay.isEmpty()) {
            parsed.modelDisplay = QStringLiteral("未知");
        }
        devicesByIp.insert(srcIp, parsed);
        return;
    }

    PushDeviceRecord& existing = it.value();
    if (existing.lidarSn.isEmpty() || !isValidPushSn(existing.lidarSn)) {
        existing.lidarSn = parsed.lidarSn;
    }
    if (parsed.lidarId != 0) {
        existing.lidarId = parsed.lidarId;
    }
    if (parsed.deviceType != 0) {
        existing.deviceType = parsed.deviceType;
        existing.modelDisplay = parsed.modelDisplay;
    }
    if (parsed.hasExtrinsic) {
        existing.hasExtrinsic = true;
        existing.offsetRoll = parsed.offsetRoll;
        existing.offsetPitch = parsed.offsetPitch;
        existing.offsetYaw = parsed.offsetYaw;
        existing.offsetX = parsed.offsetX;
        existing.offsetY = parsed.offsetY;
        existing.offsetZ = parsed.offsetZ;
    }
}

QVector<PushDeviceRecord> finalizeDevices(const QMap<QString, PushDeviceRecord>& pushDevicesByIp,
                                          const QMap<QString, uint32_t>& dataSourceIps)
{
    QVector<PushDeviceRecord> devices;
    QMap<uint32_t, bool> seenIds;

    for (auto it = pushDevicesByIp.constBegin(); it != pushDevicesByIp.constEnd(); ++it) {
        PushDeviceRecord record = it.value();
        if (record.lidarId == 0) {
            record.lidarId = ipToLidarId(it.key());
        }
        if (record.modelDisplay.isEmpty()) {
            record.modelDisplay = QStringLiteral("未知");
        }
        if (seenIds.contains(record.lidarId)) {
            continue;
        }
        seenIds.insert(record.lidarId, true);
        devices.push_back(record);
    }

    for (auto it = dataSourceIps.constBegin(); it != dataSourceIps.constEnd(); ++it) {
        const uint32_t lidarId = it.value() != 0 ? it.value() : ipToLidarId(it.key());
        if (lidarId == 0 || seenIds.contains(lidarId)) {
            continue;
        }
        seenIds.insert(lidarId, true);

        PushDeviceRecord fallback;
        fallback.lidarId = lidarId;
        fallback.deviceType = 0; // 点云兜底数据无法感知型号，设为未知
        fallback.lidarSn = QStringLiteral("未知");
        fallback.modelDisplay = QStringLiteral("未知");
        devices.push_back(fallback);
    }

    return devices;
}

} // namespace PushMsgParser

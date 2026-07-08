#include "LidarSdkService.h"

namespace LidarSdkService {

bool initialize(const QString& configPath)
{
    return LivoxLidarSdkInit(configPath.toStdString().c_str());
}

void registerCallbacks(const CallbackSet& callbacks)
{
    SetLivoxLidarInfoChangeCallback(callbacks.infoChange, callbacks.clientData);
    SetLivoxLidarPointCloudCallBack(callbacks.pointCloud, callbacks.clientData);
    SetLivoxLidarImuDataCallback(callbacks.imu, callbacks.clientData);
    SetLivoxLidarInfoCallback(callbacks.statusInfo, callbacks.clientData);
}

void clearCallbacks()
{
    SetLivoxLidarInfoChangeCallback(nullptr, nullptr);
    SetLivoxLidarPointCloudCallBack(nullptr, nullptr);
    SetLivoxLidarImuDataCallback(nullptr, nullptr);
    SetLivoxLidarInfoCallback(nullptr, nullptr);
}

void shutdown()
{
    LivoxLidarSdkUninit();
}

QString versionString()
{
    LivoxLidarSdkVer sdkVersion;
    GetLivoxLidarSdkVer(&sdkVersion);
    return QString("v%1.%2.%3")
        .arg(sdkVersion.major)
        .arg(sdkVersion.minor)
        .arg(sdkVersion.patch);
}

QString statusString(livox_status status)
{
    switch (status) {
    case kLivoxLidarStatusSuccess: return "操作成功";
    case kLivoxLidarStatusFailure: return "操作失败";
    case kLivoxLidarStatusNotConnected: return "设备未连接";
    case kLivoxLidarStatusNotSupported: return "设备不支持此操作";
    case kLivoxLidarStatusTimeout: return "操作超时";
    case kLivoxLidarStatusNotEnoughMemory: return "内存不足";
    case kLivoxLidarStatusChannelNotExist: return "通信通道不存在";
    case kLivoxLidarStatusInvalidHandle: return "设备句柄无效";
    case kLivoxLidarStatusHandlerImplNotExist: return "处理器实现不存在";
    case kLivoxLidarStatusSendFailed: return "命令发送失败";
    default: return QString("未知错误: %1").arg(status);
    }
}

QString retCodeString(uint8_t retCode)
{
    switch (retCode) {
    case 0x00: return "执行成功";
    case 0x01: return "执行失败";
    case 0x02: return "当前状态不支持";
    case 0x03: return "设置值超出范围";
    case 0x20: return "参数不支持";
    case 0x21: return "参数需重启生效";
    case 0x22: return "参数只读，不支持写入";
    case 0x23: return "请求参数长度错误/ack数据包超出最大长度";
    case 0x24: return "参数 key_num 和 key_list 不匹配";
    case 0x30: return "公钥签名验证错误";
    case 0x31: return "固件摘要签名验证错误";
    case 0x32: return "固件类型不匹配";
    case 0x33: return "固件长度超出范围";
    case 0x34: return "固件擦除中";
    default: return QString("未知 ret_code");
    }
}

} // namespace LidarSdkService

#include "PcapExtractor.h"

#include "ImuParser.h"
#include "LidarParameterService.h"
#include "Lvx2Writer.h"
#include "PcapUdpPacket.h"
#include "PushMsgParser.h"
#include "livox_lidar_def.h"

#include <pcap.h>

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMap>
#include <QSet>
#include <QTextStream>
#include <QVector>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>

namespace PcapExtractor {

namespace {

constexpr size_t kDataHeaderSize = 36;
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

struct DeviceData {
    uint32_t lidarId = 0;
    QString sourceIp;
    PushMsgParser::PushDeviceRecord push;
    uint64_t pointPackets = 0;
    uint64_t imuPackets = 0;
    uint64_t infoPackets = 0;
};

struct ScanResult {
    uint64_t totalPackets = 0;
    uint64_t udpPackets = 0;
    uint64_t pointPortPackets = 0;
    uint64_t imuPortPackets = 0;
    uint64_t infoPortPackets = 0;
    uint64_t validPointPackets = 0;
    uint64_t validImuPackets = 0;
    uint64_t validInfoPackets = 0;
    int datalinkType = 0;
    QMap<uint32_t, DeviceData> devices;
    QMap<uint8_t, uint64_t> unsupportedPointTypes;
    QStringList warnings;
};

struct CsvOutput {
    QString finalPath;
    QString temporaryPath;
    QFile file;
    std::unique_ptr<QTextStream> stream;
};

uint16_t readLe16(const uint8_t* data)
{
    return uint16_t(data[0]) | (uint16_t(data[1]) << 8);
}

uint64_t readLe64(const uint8_t* data)
{
    uint64_t value = 0;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

size_t pointStride(uint8_t dataType)
{
    switch (dataType) {
    case 1: return 14;
    case 2: return 8;
    case 3: return 10;
    case 17: return 28;
    default: return 0;
    }
}

QByteArray sphericalToCartesian(const uint8_t* data, uint16_t pointCount)
{
    QByteArray converted(int(size_t(pointCount) * sizeof(LivoxLidarCartesianHighRawPoint)), Qt::Uninitialized);
    for (uint16_t index = 0; index < pointCount; ++index) {
        LivoxLidarSpherPoint source{};
        std::memcpy(&source, data + size_t(index) * sizeof(source), sizeof(source));

        const double theta = double(source.theta) / 100.0 * kDegreesToRadians;
        const double phi = double(source.phi) / 100.0 * kDegreesToRadians;
        LivoxLidarCartesianHighRawPoint target{};
        target.x = int32_t(double(source.depth) * std::sin(theta) * std::cos(phi));
        target.y = int32_t(double(source.depth) * std::sin(theta) * std::sin(phi));
        target.z = int32_t(double(source.depth) * std::cos(theta));
        target.reflectivity = source.reflectivity;
        target.tag = source.tag;
        std::memcpy(converted.data() + size_t(index) * sizeof(target), &target, sizeof(target));
    }
    return converted;
}

bool validPointPayload(const uint8_t* payload, size_t length)
{
    if (payload == nullptr || length < kDataHeaderSize || (payload[0] != 0 && payload[0] != 1)) {
        return false;
    }
    const size_t stride = pointStride(payload[10]);
    return stride != 0 && length >= kDataHeaderSize + size_t(readLe16(payload + 5)) * stride;
}

bool supportedDatalink(int datalinkType)
{
    if (datalinkType == DLT_EN10MB || datalinkType == DLT_LINUX_SLL || datalinkType == DLT_RAW) {
        return true;
    }
#ifdef DLT_LINUX_SLL2
    return datalinkType == DLT_LINUX_SLL2;
#else
    return false;
#endif
}

pcap_t* openPcap(const QString& path, char* errorBuffer)
{
#ifdef _WIN32
    return pcap_open_offline(path.toLocal8Bit().constData(), errorBuffer);
#else
    return pcap_open_offline(path.toUtf8().constData(), errorBuffer);
#endif
}

QString uniquePath(const QString& requested)
{
    if (!QFileInfo::exists(requested)) {
        return requested;
    }
    const QFileInfo info(requested);
    const QString base = QDir(info.absolutePath()).filePath(info.completeBaseName());
    const QString suffix = info.suffix().isEmpty() ? QString() : QStringLiteral(".") + info.suffix();
    for (int index = 1;; ++index) {
        const QString candidate = QStringLiteral("%1_%2%3").arg(base).arg(index).arg(suffix);
        if (!QFileInfo::exists(candidate)) {
            return candidate;
        }
    }
}

QString deviceSuffix(const DeviceData& device)
{
    const QString sn = device.push.lidarSn.isEmpty() ? QStringLiteral("null") : device.push.lidarSn;
    return QStringLiteral("%1_%2").arg(sn, PushMsgParser::lidarIdToIpString(device.lidarId));
}

QString csvField(QString value)
{
    if (value.contains(QLatin1Char(',')) || value.contains(QLatin1Char('"')) ||
        value.contains(QLatin1Char('\r')) || value.contains(QLatin1Char('\n'))) {
        value.replace(QStringLiteral("\""), QStringLiteral("\"\""));
        return QStringLiteral("\"") + value + QLatin1Char('"');
    }
    return value;
}

bool openCsv(CsvOutput& output, const QString& finalPath, const QString& header, QString& error)
{
    output.finalPath = uniquePath(finalPath);
    output.temporaryPath = output.finalPath + QStringLiteral(".tmp");
    output.file.setFileName(output.temporaryPath);
    if (!output.file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        error = QStringLiteral("无法创建 CSV 文件：%1").arg(output.file.errorString());
        return false;
    }
    output.file.write("\xEF\xBB\xBF", 3);
    output.stream = std::make_unique<QTextStream>(&output.file);
    *output.stream << header << "\r\n";
    return true;
}

bool commitCsv(CsvOutput& output, QString& error)
{
    output.stream->flush();
    output.file.close();
    if (!QFile::rename(output.temporaryPath, output.finalPath)) {
        QFile::remove(output.temporaryPath);
        error = QStringLiteral("CSV 临时文件重命名失败：%1").arg(output.finalPath);
        return false;
    }
    return true;
}

void discardCsv(CsvOutput& output)
{
    output.stream.reset();
    if (output.file.isOpen()) {
        output.file.close();
    }
    QFile::remove(output.temporaryPath);
}

uint64_t captureTimestampNs(const pcap_pkthdr* header)
{
    return uint64_t(header->ts.tv_sec) * 1000000000ULL + uint64_t(header->ts.tv_usec) * 1000ULL;
}

bool validPushPayload(uint16_t sourcePort, uint16_t destinationPort, const uint8_t* payload, size_t length)
{
    if (!PushMsgParser::decodeProtocolPacket(sourcePort, destinationPort, payload, length).valid) {
        return false;
    }
    return length >= 28 && payload[0] == 0xAA && readLe16(payload + 2) <= length;
}

uint32_t packetLidarId(const PcapUdp::PacketInfo& packet, const PushMsgParser::PushDeviceRecord* push = nullptr)
{
    if (push != nullptr && push->lidarId != 0) {
        return push->lidarId;
    }
    return PushMsgParser::ipToLidarId(packet.srcIp);
}

DeviceData& ensureDevice(ScanResult& scan,
                         uint32_t lidarId,
                         const QString& sourceIp,
                         const PushMsgParser::PushDeviceRecord* push = nullptr)
{
    DeviceData& device = scan.devices[lidarId];
    device.lidarId = lidarId;
    if (device.sourceIp.isEmpty()) {
        device.sourceIp = sourceIp;
    }
    if (push != nullptr) {
        if (!push->lidarSn.isEmpty() && device.push.lidarSn.isEmpty()) {
            device.push.lidarSn = push->lidarSn;
        }
        if (push->deviceType != 0) {
            device.push.deviceType = push->deviceType;
            device.push.modelDisplay = push->modelDisplay;
        }
        if (push->hasExtrinsic) {
            device.push.hasExtrinsic = true;
            device.push.offsetRoll = push->offsetRoll;
            device.push.offsetPitch = push->offsetPitch;
            device.push.offsetYaw = push->offsetYaw;
            device.push.offsetX = push->offsetX;
            device.push.offsetY = push->offsetY;
            device.push.offsetZ = push->offsetZ;
        }
    }
    device.push.lidarId = lidarId;
    return device;
}

bool scanFile(const QString& path,
              ScanResult& scan,
              QString& error,
              const ProgressCallback& progress,
              const std::atomic_bool* cancellation)
{
    char errorBuffer[PCAP_ERRBUF_SIZE] = {};
    pcap_t* handle = openPcap(path, errorBuffer);
    if (handle == nullptr) {
        error = QStringLiteral("无法打开抓包文件：%1").arg(QString::fromLocal8Bit(errorBuffer));
        return false;
    }
    scan.datalinkType = pcap_datalink(handle);
    if (!supportedDatalink(scan.datalinkType)) {
        error = QStringLiteral("当前抓包链路层类型不受支持：%1").arg(scan.datalinkType);
        pcap_close(handle);
        return false;
    }

    pcap_pkthdr* header = nullptr;
    const u_char* bytes = nullptr;
    int status = 0;
    while ((status = pcap_next_ex(handle, &header, &bytes)) >= 0) {
        if (cancellation != nullptr && cancellation->load()) {
            error = QStringLiteral("任务已取消");
            pcap_close(handle);
            return false;
        }
        if (status == 0 || header == nullptr) {
            continue;
        }
        ++scan.totalPackets;
        PcapUdp::PacketInfo packet;
        if (!PcapUdp::tryExtractUdp(bytes, header->caplen, scan.datalinkType, packet)) {
            continue;
        }
        ++scan.udpPackets;
        if (packet.srcPort == PcapUdp::kLivoxPointCloudPort) {
            ++scan.pointPortPackets;
            if (validPointPayload(packet.payload, packet.payloadLen)) {
                ++scan.validPointPackets;
                ++ensureDevice(scan, packetLidarId(packet), packet.srcIp).pointPackets;
            } else if (packet.payloadLen >= kDataHeaderSize &&
                       (packet.payload[0] == 0 || packet.payload[0] == 1) &&
                       pointStride(packet.payload[10]) == 0) {
                ++scan.unsupportedPointTypes[packet.payload[10]];
            }
        } else if (packet.srcPort == PcapUdp::kLivoxIMUPort) {
            ++scan.imuPortPackets;
            if (ImuParser::isLivoxImuPayload(packet.payload, packet.payloadLen)) {
                ++scan.validImuPackets;
                ++ensureDevice(scan, packetLidarId(packet), packet.srcIp).imuPackets;
            }
        } else if (packet.srcPort == PcapUdp::kLivoxPushDataPort) {
            ++scan.infoPortPackets;
            if (validPushPayload(packet.srcPort, packet.dstPort, packet.payload, packet.payloadLen)) {
                PushMsgParser::PushDeviceRecord push;
                PushMsgParser::parsePushPayload(packet.payload, packet.payloadLen, push);
                const uint32_t lidarId = packetLidarId(packet, &push);
                ++scan.validInfoPackets;
                ++ensureDevice(scan, lidarId, packet.srcIp, &push).infoPackets;
            }
        }
        if (progress && (scan.totalPackets % 1024 == 0)) {
            progress(20);
        }
    }
    if (status == -1) {
        const QString readError = QString::fromLocal8Bit(pcap_geterr(handle));
        if (scan.totalPackets == 0) {
            error = QStringLiteral("抓包文件读取失败：%1").arg(readError);
            pcap_close(handle);
            return false;
        }
        scan.warnings.push_back(
            QStringLiteral("抓包文件在第 %1 个数据包后存在损坏数据，已提取损坏位置之前的有效数据：%2")
                .arg(scan.totalPackets)
                .arg(readError));
    }
    pcap_close(handle);
    if (progress) {
        progress(40);
    }
    return true;
}

QString dataError(const ScanResult& scan, Kind kind)
{
    if (scan.totalPackets == 0) {
        return QStringLiteral("抓包文件为空，未发现任何数据包");
    }
    if (scan.udpPackets == 0) {
        return QStringLiteral("未发现可解析的 IPv4 UDP 数据包");
    }
    if (scan.pointPortPackets + scan.imuPortPackets + scan.infoPortPackets == 0) {
        return QStringLiteral("未发现 Livox 有效数据端口（56200/56300/56400）");
    }
    uint64_t portPackets = 0;
    uint64_t validPackets = 0;
    uint16_t port = 0;
    QString type;
    if (kind == Kind::Lvx2) {
        portPackets = scan.pointPortPackets;
        validPackets = scan.validPointPackets;
        port = PcapUdp::kLivoxPointCloudPort;
        type = QStringLiteral("点云");
    } else if (kind == Kind::ImuCsv) {
        portPackets = scan.imuPortPackets;
        validPackets = scan.validImuPackets;
        port = PcapUdp::kLivoxIMUPort;
        type = QStringLiteral("IMU");
    } else {
        portPackets = scan.infoPortPackets;
        validPackets = scan.validInfoPackets;
        port = PcapUdp::kLivoxPushDataPort;
        type = QStringLiteral("推送信息");
    }
    if (validPackets != 0) {
        return {};
    }
    if (portPackets != 0) {
        return QStringLiteral("发现 %1 个 %2 数据包，但数据均无效或已被截断").arg(portPackets).arg(port);
    }
    return QStringLiteral("未发现有效 Livox %1（源端口 %2）").arg(type).arg(port);
}

QVector<DeviceData> targetDevices(const ScanResult& scan, Kind kind)
{
    QVector<DeviceData> devices;
    for (const DeviceData& device : scan.devices) {
        const bool include = kind == Kind::Lvx2 ? device.pointPackets != 0
                           : kind == Kind::ImuCsv ? device.imuPackets != 0
                                                 : device.infoPackets != 0;
        if (include) {
            devices.push_back(device);
        }
    }
    std::sort(devices.begin(), devices.end(), [](const DeviceData& left, const DeviceData& right) {
        return left.lidarId < right.lidarId;
    });
    return devices;
}

Lvx2DeviceInfo lvxDevice(const DeviceData& source)
{
    Lvx2DeviceInfo device{};
    const QByteArray sn = (source.push.lidarSn.isEmpty() ? QByteArray("null") : source.push.lidarSn.toLatin1()).left(15);
    std::memcpy(device.lidar_sn, sn.constData(), size_t(sn.size()));
    device.lidar_id = source.lidarId;
    device.lidar_type = 8;
    device.device_type = source.push.deviceType;
    device.extrinsic_enable = source.push.hasExtrinsic ? 1 : 0;
    device.roll = source.push.offsetRoll;
    device.pitch = source.push.offsetPitch;
    device.yaw = source.push.offsetYaw;
    device.x = source.push.offsetX * 100.0f;
    device.y = source.push.offsetY * 100.0f;
    device.z = source.push.offsetZ * 100.0f;
    return device;
}

bool openSecondPass(const QString& path, std::unique_ptr<pcap_t, decltype(&pcap_close)>& handle, QString& error)
{
    char errorBuffer[PCAP_ERRBUF_SIZE] = {};
    handle.reset(openPcap(path, errorBuffer));
    if (!handle) {
        error = QStringLiteral("无法重新打开抓包文件：%1").arg(QString::fromLocal8Bit(errorBuffer));
        return false;
    }
    return true;
}

Result extractLvx2(const QString& sourcePath,
                   const QString& outputDirectory,
                   Mode mode,
                   const ScanResult& scan,
                   const ProgressCallback& progress,
                   const std::atomic_bool* cancellation)
{
    Result result;
    const QVector<DeviceData> devices = targetDevices(scan, Kind::Lvx2);
    const QString base = QFileInfo(sourcePath).completeBaseName();
    QMap<uint32_t, std::shared_ptr<Lvx2::Lvx2Writer>> writers;
    QMap<uint32_t, QString> finalPaths;
    QString error;

    if (mode == Mode::Merge) {
        QVector<Lvx2DeviceInfo> table;
        for (const DeviceData& device : devices) {
            table.push_back(lvxDevice(device));
        }
        const QString finalPath = uniquePath(QDir(outputDirectory).filePath(base + QStringLiteral(".lvx2")));
        auto writer = std::make_shared<Lvx2::Lvx2Writer>();
        if (!writer->open(finalPath + QStringLiteral(".tmp"), table, error)) {
            result.errorMessage = error;
            return result;
        }
        writers.insert(0, writer);
        finalPaths.insert(0, finalPath);
    } else {
        for (const DeviceData& device : devices) {
            const QString finalPath = uniquePath(QDir(outputDirectory).filePath(
                QStringLiteral("%1_%2.lvx2").arg(base, deviceSuffix(device))));
            auto writer = std::make_shared<Lvx2::Lvx2Writer>();
            if (!writer->open(finalPath + QStringLiteral(".tmp"), {lvxDevice(device)}, error)) {
                for (auto& opened : writers) opened->abort();
                result.errorMessage = error;
                return result;
            }
            writers.insert(device.lidarId, writer);
            finalPaths.insert(device.lidarId, finalPath);
        }
    }

    std::unique_ptr<pcap_t, decltype(&pcap_close)> handle(nullptr, &pcap_close);
    if (!openSecondPass(sourcePath, handle, error)) {
        for (auto& writer : writers) writer->abort();
        result.errorMessage = error;
        return result;
    }
    uint64_t visited = 0;
    pcap_pkthdr* packetHeader = nullptr;
    const u_char* bytes = nullptr;
    int status = 0;
    while ((status = pcap_next_ex(handle.get(), &packetHeader, &bytes)) >= 0) {
        if (cancellation != nullptr && cancellation->load()) {
            for (auto& writer : writers) writer->abort();
            result.errorMessage = QStringLiteral("任务已取消");
            return result;
        }
        if (status == 0 || packetHeader == nullptr) continue;
        ++visited;
        PcapUdp::PacketInfo packet;
        if (!PcapUdp::tryExtractUdp(bytes, packetHeader->caplen, scan.datalinkType, packet) ||
            packet.srcPort != PcapUdp::kLivoxPointCloudPort || !validPointPayload(packet.payload, packet.payloadLen)) {
            continue;
        }
        const uint32_t lidarId = packetLidarId(packet);
        Lvx2::Lvx2Writer* writer = mode == Mode::Merge ? writers.value(0).get() : writers.value(lidarId).get();
        if (writer == nullptr) continue;
        const uint16_t dotCount = readLe16(packet.payload + 5);
        const uint8_t sourceDataType = packet.payload[10];
        const uint8_t* pointData = packet.payload + kDataHeaderSize;
        QByteArray convertedData;
        if (sourceDataType == kLivoxLidarSphericalCoordinateData) {
            convertedData = sphericalToCartesian(pointData, dotCount);
            pointData = reinterpret_cast<const uint8_t*>(convertedData.constData());
        }
        const size_t dataLength = sourceDataType == kLivoxLidarSphericalCoordinateData
            ? size_t(convertedData.size())
            : size_t(dotCount) * pointStride(sourceDataType);
        Lvx2PackageHeader package{};
        package.version = packet.payload[0];
        package.lidar_id = lidarId;
        package.lidar_type = 8;
        package.timestamp_type = packet.payload[11];
        package.timestamp = readLe64(packet.payload + 28);
        package.udp_counter = readLe16(packet.payload + 7);
        package.data_type = sourceDataType == kLivoxLidarSphericalCoordinateData
            ? kLivoxLidarCartesianCoordinateHighData
            : sourceDataType;
        package.data_length = uint32_t(dataLength);
        package.frame_counter = packet.payload[9];
        if (!writer->appendPacket(captureTimestampNs(packetHeader), package,
                                  pointData, dataLength, error)) {
            for (auto& opened : writers) opened->abort();
            result.errorMessage = error;
            return result;
        }
        if (progress && visited % 512 == 0) progress(40 + int(55 * visited / scan.totalPackets));
    }
    for (auto& writer : writers) {
        if (!writer->close(error)) {
            for (auto& opened : writers) opened->abort();
            result.errorMessage = error;
            return result;
        }
    }
    QStringList committedFiles;
    for (auto it = finalPaths.constBegin(); it != finalPaths.constEnd(); ++it) {
        const QString temporary = it.value() + QStringLiteral(".tmp");
        if (!QFile::rename(temporary, it.value())) {
            QFile::remove(temporary);
            for (const QString& committed : committedFiles) QFile::remove(committed);
            result.errorMessage = QStringLiteral("LVX2 临时文件重命名失败：%1").arg(it.value());
            return result;
        }
        committedFiles.push_back(it.value());
        result.outputFiles.push_back(it.value());
    }
    for (const DeviceData& device : devices) {
        if (device.push.lidarSn.isEmpty()) {
            result.warnings.push_back(QStringLiteral("设备 %1 未发现 SN，LVX2 中已写入 null").arg(device.sourceIp));
        }
    }
    result.ok = true;
    if (progress) progress(100);
    return result;
}

const QVector<uint16_t>& infoKeys()
{
    static const QVector<uint16_t> keys = {
        kKeySn, kKeyProductInfo, kKeyVersionApp, kKeyVersionLoader, kKeyVersionHardware, kKeyMac,
        kKeyCurWorkState, kKeyCoreTemp, kKeyPowerUpCnt, kKeyLocalTimeNow, kKeyLastSyncTime,
        kKeyTimeOffset, kKeyTimeSyncType, kKeyLidarDiagStatus, kKeyFwType, kKeyHmsCode,
        kKeyPclDataType, kKeyPatternMode, kKeyDetectMode, kKeyWorkMode, kKeyImuDataEn,
        kKeyLidarIpCfg, kKeyStateInfoHostIpCfg, kKeyLidarPointDataHostIpCfg, kKeyLidarImuHostIpCfg,
        kKeyFovCfg0, kKeyFovCfg1, kKeyFovCfgEn, kKeyInstallAttitude, kKeySetEscMode,
        kKeySetPpsSyncMode, kKeySetFovMode, kKeySetEchoMode, kKeySetNTPServerIp
    };
    return keys;
}

QString infoHeader()
{
    return QStringLiteral("时间戳,序列号,产品信息,固件版本,LOADER版本,硬件版本,MAC地址,当前工作状态,核心温度 ℃,上电次数,本地时间,最后同步时间,时间偏移,时间同步类型,雷达诊断状态,固件类型,HMS诊断码,点云格式,扫描模式,探测模式,工作模式,IMU数据发送,雷达IP配置,状态信息目的IP,点云数据目的IP,IMU数据目的IP,FOV0配置,FOV1配置,FOV使能状态,安装姿态,电机转速,异常时间过滤,FOV模式,回波模式,NTP服务器IP");
}

bool updateInfoSnapshot(const uint8_t* payload, size_t length, QMap<uint16_t, QString>& snapshot)
{
    if (length < 28) return false;
    const uint16_t count = readLe16(payload + 24);
    size_t offset = 28;
    for (uint16_t index = 0; index < count; ++index) {
        if (offset + 4 > length) return false;
        const uint16_t key = readLe16(payload + offset);
        const uint16_t valueLength = readLe16(payload + offset + 2);
        offset += 4;
        if (offset + valueLength > length) return false;
        if (infoKeys().contains(key)) {
            if (key == kKeyCoreTemp && valueLength >= sizeof(int32_t)) {
                int32_t temperature = 0;
                std::memcpy(&temperature, payload + offset, sizeof(temperature));
                snapshot.insert(key, QString::number(double(temperature) / 100.0, 'f', 2));
            } else {
                snapshot.insert(key, LidarParameterService::formatValue(key,
                    const_cast<uint8_t*>(payload + offset), valueLength));
            }
        }
        offset += valueLength;
    }
    return true;
}

Result extractCsv(const QString& sourcePath,
                  const QString& outputDirectory,
                  Kind kind,
                  Mode mode,
                  const ScanResult& scan,
                  const ProgressCallback& progress,
                  const std::atomic_bool* cancellation)
{
    Result result;
    const QVector<DeviceData> devices = targetDevices(scan, kind);
    const QString base = QFileInfo(sourcePath).completeBaseName();
    const QString tag = kind == Kind::ImuCsv ? QStringLiteral("imu") : QStringLiteral("info");
    const QString header = kind == Kind::ImuCsv
        ? QStringLiteral("packet_index,udp_cnt,capture_timestamp_ns,sensor_timestamp_raw,time_type,source_ip,lidar_id,sn,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,acc_x_g,acc_y_g,acc_z_g")
        : infoHeader();
    QMap<uint32_t, std::shared_ptr<CsvOutput>> outputs;
    QString error;
    if (mode == Mode::Merge) {
        auto output = std::make_shared<CsvOutput>();
        if (!openCsv(*output, QDir(outputDirectory).filePath(QStringLiteral("%1_%2.csv").arg(base, tag)), header, error)) {
            result.errorMessage = error;
            return result;
        }
        outputs.insert(0, output);
    } else {
        for (const DeviceData& device : devices) {
            auto output = std::make_shared<CsvOutput>();
            const QString path = QDir(outputDirectory).filePath(
                QStringLiteral("%1_%2_%3.csv").arg(base, tag, deviceSuffix(device)));
            if (!openCsv(*output, path, header, error)) {
                for (auto& opened : outputs) discardCsv(*opened);
                result.errorMessage = error;
                return result;
            }
            outputs.insert(device.lidarId, output);
        }
    }

    QMap<uint32_t, QMap<uint16_t, QString>> snapshots;
    std::unique_ptr<pcap_t, decltype(&pcap_close)> handle(nullptr, &pcap_close);
    if (!openSecondPass(sourcePath, handle, error)) {
        for (auto& output : outputs) discardCsv(*output);
        result.errorMessage = error;
        return result;
    }
    uint64_t visited = 0;
    uint64_t udpPacketIndex = 0;
    pcap_pkthdr* packetHeader = nullptr;
    const u_char* bytes = nullptr;
    int status = 0;
    while ((status = pcap_next_ex(handle.get(), &packetHeader, &bytes)) >= 0) {
        if (cancellation != nullptr && cancellation->load()) {
            for (auto& output : outputs) discardCsv(*output);
            result.errorMessage = QStringLiteral("任务已取消");
            return result;
        }
        if (status == 0 || packetHeader == nullptr) continue;
        ++visited;
        PcapUdp::PacketInfo packet;
        if (!PcapUdp::tryExtractUdp(bytes, packetHeader->caplen, scan.datalinkType, packet)) continue;
        ++udpPacketIndex;

        PushMsgParser::PushDeviceRecord parsedPush;
        if (kind == Kind::InfoCsv && packet.srcPort == PcapUdp::kLivoxPushDataPort) {
            if (!validPushPayload(packet.srcPort, packet.dstPort, packet.payload, packet.payloadLen)) continue;
            PushMsgParser::parsePushPayload(packet.payload, packet.payloadLen, parsedPush);
        }
        const uint32_t lidarId = packetLidarId(packet, kind == Kind::InfoCsv ? &parsedPush : nullptr);
        CsvOutput* output = mode == Mode::Merge ? outputs.value(0).get() : outputs.value(lidarId).get();
        if (output == nullptr) continue;
        const DeviceData device = scan.devices.value(lidarId);
        const QString sn = device.push.lidarSn.isEmpty() ? QStringLiteral("null") : device.push.lidarSn;

        if (kind == Kind::ImuCsv) {
            if (packet.srcPort != PcapUdp::kLivoxIMUPort || !ImuParser::isLivoxImuPayload(packet.payload, packet.payloadLen)) continue;
            QVector<ImuParser::Sample> samples;
            ImuParser::appendImuPayload(packet.payload, packet.payloadLen, lidarId, captureTimestampNs(packetHeader), samples);
            const uint64_t sensorTimestamp = readLe64(packet.payload + 28);
            const uint16_t udpCount = readLe16(packet.payload + 7);
            const uint8_t timeType = packet.payload[11];
            for (const ImuParser::Sample& sample : samples) {
                *output->stream << udpPacketIndex << ',' << udpCount << ',' << sample.timestampNs << ','
                    << sensorTimestamp << ',' << int(timeType) << ',' << packet.srcIp << ',' << lidarId << ',' << csvField(sn) << ','
                    << QString::number(sample.gyroX, 'g', 10) << ',' << QString::number(sample.gyroY, 'g', 10) << ','
                    << QString::number(sample.gyroZ, 'g', 10) << ',' << QString::number(sample.accX, 'g', 10) << ','
                    << QString::number(sample.accY, 'g', 10) << ',' << QString::number(sample.accZ, 'g', 10) << "\r\n";
            }
        } else {
            if (packet.srcPort != PcapUdp::kLivoxPushDataPort) continue;
            QMap<uint16_t, QString>& snapshot = snapshots[lidarId];
            if (!updateInfoSnapshot(packet.payload, packet.payloadLen, snapshot)) continue;
            if (!snapshot.contains(kKeySn)) snapshot.insert(kKeySn, sn);
            if (!snapshot.contains(kKeyLidarIpCfg)) snapshot.insert(kKeyLidarIpCfg, packet.srcIp);
            const qint64 timestampMs = qint64(packetHeader->ts.tv_sec) * 1000 + packetHeader->ts.tv_usec / 1000;
            *output->stream << QDateTime::fromMSecsSinceEpoch(timestampMs).toString(QStringLiteral("yyyy-MM-dd hh:mm:ss.zzz"));
            for (uint16_t key : infoKeys()) {
                *output->stream << ',' << csvField(snapshot.value(key, QStringLiteral("N/A")));
            }
            *output->stream << "\r\n";
        }
        if (progress && visited % 512 == 0) progress(40 + int(55 * visited / scan.totalPackets));
    }

    QStringList committedFiles;
    for (auto& output : outputs) {
        if (!commitCsv(*output, error)) {
            for (auto& remaining : outputs) discardCsv(*remaining);
            for (const QString& committed : committedFiles) QFile::remove(committed);
            result.errorMessage = error;
            return result;
        }
        committedFiles.push_back(output->finalPath);
        result.outputFiles.push_back(output->finalPath);
    }
    result.ok = true;
    if (progress) progress(100);
    return result;
}

} // namespace

Result extract(const QString& sourcePath,
               const QString& outputDirectory,
               Kind kind,
               Mode mode,
               const ProgressCallback& progress,
               const std::atomic_bool* cancellationRequested)
{
    Result result;
    ScanResult scan;
    if (!scanFile(sourcePath, scan, result.errorMessage, progress, cancellationRequested)) {
        return result;
    }
    result.errorMessage = dataError(scan, kind);
    if (!result.errorMessage.isEmpty()) {
        return result;
    }
    QDir().mkpath(outputDirectory);
    Result extracted = kind == Kind::Lvx2
        ? extractLvx2(sourcePath, outputDirectory, mode, scan, progress, cancellationRequested)
        : extractCsv(sourcePath, outputDirectory, kind, mode, scan, progress, cancellationRequested);
    extracted.warnings.append(scan.warnings);
    if (extracted.ok && kind == Kind::Lvx2) {
        for (auto it = scan.unsupportedPointTypes.constBegin(); it != scan.unsupportedPointTypes.constEnd(); ++it) {
            extracted.warnings.push_back(
                QStringLiteral("发现不支持的点云 data_type：0x%1，已跳过 %2 个数据包")
                    .arg(it.key(), 2, 16, QLatin1Char('0')).arg(it.value()));
        }
    }
    return extracted;
}

} // namespace PcapExtractor

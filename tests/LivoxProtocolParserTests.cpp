#include "PushMsgParser.h"

#include <QByteArray>
#include <QString>

#include <cstdint>
#include <cstring>
#include <iostream>

namespace {

int failureCount = 0;

void expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failureCount;
    }
}

void setLe16(QByteArray& bytes, int offset, uint16_t value)
{
    bytes[offset] = char(value & 0xFF);
    bytes[offset + 1] = char(value >> 8);
}

void setLe32(QByteArray& bytes, int offset, uint32_t value)
{
    bytes[offset] = char(value & 0xFF);
    bytes[offset + 1] = char((value >> 8) & 0xFF);
    bytes[offset + 2] = char((value >> 16) & 0xFF);
    bytes[offset + 3] = char(value >> 24);
}

void setLe64(QByteArray& bytes, int offset, uint64_t value)
{
    setLe32(bytes, offset, uint32_t(value));
    setLe32(bytes, offset + 4, uint32_t(value >> 32));
}

void setLeFloat(QByteArray& bytes, int offset, float value)
{
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    setLe32(bytes, offset, bits);
}

void appendLe16(QByteArray& bytes, uint16_t value)
{
    bytes.append(char(value & 0xFF));
    bytes.append(char(value >> 8));
}

void appendLe32(QByteArray& bytes, uint32_t value)
{
    bytes.append(char(value & 0xFF));
    bytes.append(char((value >> 8) & 0xFF));
    bytes.append(char((value >> 16) & 0xFF));
    bytes.append(char(value >> 24));
}

void appendLe64(QByteArray& bytes, uint64_t value)
{
    appendLe32(bytes, uint32_t(value));
    appendLe32(bytes, uint32_t(value >> 32));
}

QByteArray makeHeader(uint16_t commandId, uint8_t commandType, uint8_t senderType)
{
    QByteArray packet(24, '\0');
    packet[0] = char(0xAA);
    packet[1] = char(1);
    setLe32(packet, 4, 42);
    setLe16(packet, 8, commandId);
    packet[10] = char(commandType);
    packet[11] = char(senderType);
    setLe16(packet, 18, 0x1234);
    setLe32(packet, 20, 0x89ABCDEF);
    return packet;
}

void appendTlv(QByteArray& bytes, uint16_t key, const QByteArray& value)
{
    appendLe16(bytes, key);
    appendLe16(bytes, uint16_t(value.size()));
    bytes.append(value);
}

PushMsgParser::ProtocolDecodeResult decode(uint16_t sourcePort, uint16_t destinationPort, QByteArray packet)
{
    setLe16(packet, 2, uint16_t(packet.size()));
    return PushMsgParser::decodeProtocolPacket(
        sourcePort,
        destinationPort,
        reinterpret_cast<const uint8_t*>(packet.constData()),
        size_t(packet.size()));
}

void testBroadcastAck()
{
    QByteArray packet = makeHeader(0x0000, 0x01, 0x01);
    packet.append(char(0x00));
    packet.append(char(0x09));
    QByteArray serial(16, '\0');
    serial.replace(0, 7, "SN12345");
    packet.append(serial);
    packet.append(char(192));
    packet.append(char(168));
    packet.append(char(1));
    packet.append(char(10));
    appendLe16(packet, 56100);

    const auto result = decode(56000, 53000, packet);
    expect(result.valid, "broadcast ACK is decoded");
    expect(result.protocol == QStringLiteral("Livox Broadcast"), "broadcast protocol label");
    expect(result.summary.contains(QStringLiteral("广播发现")), "broadcast summary contains command");
    expect(result.details.contains(QStringLiteral("雷达 SN：SN12345")), "broadcast SN field");
    expect(result.details.contains(QStringLiteral("雷达 IP：192.168.1.10")), "broadcast IP field");
    expect(result.details.contains(QStringLiteral("控制端口：56100")), "broadcast control port field");
}

void testControlAck()
{
    QByteArray packet = makeHeader(0x0200, 0x01, 0x01);
    packet.append(char(0x00));

    const auto result = decode(56100, 53000, packet);
    expect(result.valid, "control ACK is decoded");
    expect(result.protocol == QStringLiteral("Livox Control"), "control protocol label");
    expect(result.summary.contains(QStringLiteral("请求设备重启")), "control summary contains command");
    expect(result.details.contains(QStringLiteral("命令类型：ACK 应答")), "control command type");
    expect(result.details.contains(QStringLiteral("发送端：雷达")), "control sender type");
    expect(result.details.contains(QStringLiteral("返回码：0x00（执行成功）")), "control return code");
}

void testPushParametersAndHms()
{
    QByteArray packet = makeHeader(0x0102, 0x00, 0x01);
    appendLe16(packet, 8);
    appendLe16(packet, 0);

    QByteArray firmware;
    firmware.append(char(13));
    firmware.append(char(1));
    firmware.append(char(2));
    firmware.append(char(3));
    appendTlv(packet, 0x8002, firmware);

    QByteArray mac;
    mac.append(char(0x10));
    mac.append(char(0x20));
    mac.append(char(0x30));
    mac.append(char(0x40));
    mac.append(char(0x50));
    mac.append(char(0x60));
    appendTlv(packet, 0x8005, mac);

    QByteArray workMode(1, char(0x01));
    appendTlv(packet, 0x8006, workMode);

    QByteArray temperature;
    appendLe32(temperature, 2534);
    appendTlv(packet, 0x8007, temperature);

    QByteArray timeSync(1, char(0x01));
    appendTlv(packet, 0x800C, timeSync);

    QByteArray localTime;
    appendLe64(localTime, 19156903000ULL);
    appendTlv(packet, 0x8009, localTime);

    QByteArray masterTime;
    appendLe64(masterTime, 123456789ULL);
    appendTlv(packet, 0x800A, masterTime);

    QByteArray hms;
    appendLe32(hms, 0x01020003);
    hms.append(QByteArray(28, '\0'));
    appendTlv(packet, 0x8011, hms);

    const auto result = decode(56200, 53000, packet);
    expect(result.valid, "push packet is decoded");
    expect(result.protocol == QStringLiteral("Livox Push"), "push protocol label");
    expect(result.details.contains(QStringLiteral("Data 预留位 (Reserved)：0x0000")), "data reserved field label");
    expect(result.details.contains(QStringLiteral("Key 0x8002 固件版本：13.1.0203")), "firmware version field");
    expect(result.details.contains(QStringLiteral("MAC 地址：10:20:30:40:50:60")), "MAC field");
    expect(result.details.contains(QStringLiteral("当前工作状态：采样")), "work status field");
    expect(result.details.contains(QStringLiteral("核心板温度：25.34 ℃")), "temperature field");
    expect(result.details.contains(QStringLiteral("时间同步方式：PTP（IEEE 1588 v2.0）")), "time sync field");
    expect(result.details.contains(QStringLiteral("雷达本地时间：19156903000 ns")), "local time uses nanoseconds");
    expect(result.details.contains(QStringLiteral("Master 时间：123456789 ns")), "master time uses nanoseconds");
    expect(result.details.contains(QStringLiteral("Error 错误")), "HMS severity field");
    expect(result.details.contains(QStringLiteral("设备运行环境温度偏高")), "HMS fault description");
    expect(result.summary.contains(QStringLiteral("HMS 有故障")), "HMS summary warning");
}

void testPointCloudHeader()
{
    QByteArray packet(36, '\0');
    packet[0] = char(0);
    setLe16(packet, 1, 36);
    setLe16(packet, 3, 100);
    setLe16(packet, 5, 96);
    setLe16(packet, 7, 123);
    packet[9] = char(7);
    packet[10] = char(1);
    packet[11] = char(1);
    setLe32(packet, 24, 0x12345678);
    setLe64(packet, 28, 1722927600123456789ULL);

    const auto result = PushMsgParser::decodeProtocolPacket(
        56300, 53000, reinterpret_cast<const uint8_t*>(packet.constData()), size_t(packet.size()));
    expect(result.valid, "point cloud header is decoded");
    expect(result.protocol == QStringLiteral("Livox PointCloud"), "point cloud protocol label");
    expect(result.summary.contains(QStringLiteral("直角坐标 32-bit 点云")), "point cloud data type summary");
    expect(result.details.contains(QStringLiteral("数据单元数量：96")), "point cloud dot count");
    expect(result.details.contains(QStringLiteral("UDP 包计数：123")), "point cloud UDP count");
    expect(result.details.contains(QStringLiteral("点云帧计数：7")), "point cloud frame count");
    expect(result.details.contains(QStringLiteral("时间戳：1722927600123456789 ns")), "point cloud timestamp");
    expect(!result.details.contains(QStringLiteral("Point 1")), "point cloud points are not expanded");
}

void testImuHeaderAndSample()
{
    QByteArray packet(60, '\0');
    packet[0] = char(0);
    setLe16(packet, 1, 60);
    setLe16(packet, 3, 50);
    setLe16(packet, 5, 1);
    setLe16(packet, 7, 456);
    packet[9] = char(8);
    packet[10] = char(0);
    packet[11] = char(0);
    setLe32(packet, 24, 0xAABBCCDD);
    setLe64(packet, 28, 123456789ULL);
    setLeFloat(packet, 36, 1.25f);
    setLeFloat(packet, 40, -2.5f);
    setLeFloat(packet, 44, 3.75f);
    setLeFloat(packet, 48, 0.1f);
    setLeFloat(packet, 52, 0.2f);
    setLeFloat(packet, 56, 0.3f);

    const auto result = PushMsgParser::decodeProtocolPacket(
        56400, 53000, reinterpret_cast<const uint8_t*>(packet.constData()), size_t(packet.size()));
    expect(result.valid, "IMU packet is decoded");
    expect(result.protocol == QStringLiteral("Livox IMU"), "IMU protocol label");
    expect(!result.details.contains(QStringLiteral("[首个 IMU 样本]")), "IMU sample has no redundant section label");
    expect(result.details.contains(QStringLiteral("Gyro X：1.2500000000 rad/s")), "IMU gyro X");
    expect(result.details.contains(QStringLiteral("Gyro Y：-2.5000000000 rad/s")), "IMU gyro Y");
    expect(result.details.contains(QStringLiteral("Acc Z：0.3000000119 g")), "IMU acc Z");
}

} // namespace

int main()
{
    testBroadcastAck();
    testControlAck();
    testPushParametersAndHms();
    testPointCloudHeader();
    testImuHeaderAndSample();

    if (failureCount == 0) {
        std::cout << "LivoxProtocolParserTests passed\n";
    }
    return failureCount == 0 ? 0 : 1;
}

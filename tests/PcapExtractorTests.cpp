#include "Lvx2Types.h"
#include "PcapExtractor.h"

#include <QDir>
#include <QFile>
#include <QTemporaryDir>

#include <algorithm>
#include <cstring>
#include <iostream>

namespace {

void putLe16(QByteArray& bytes, int offset, uint16_t value)
{
    bytes[offset] = char(value & 0xFF);
    bytes[offset + 1] = char(value >> 8);
}

void putLe32(QByteArray& bytes, int offset, uint32_t value)
{
    for (int i = 0; i < 4; ++i) bytes[offset + i] = char(value >> (i * 8));
}

void putLe64(QByteArray& bytes, int offset, uint64_t value)
{
    for (int i = 0; i < 8; ++i) bytes[offset + i] = char(value >> (i * 8));
}

void putBe16(QByteArray& bytes, int offset, uint16_t value)
{
    bytes[offset] = char(value >> 8);
    bytes[offset + 1] = char(value & 0xFF);
}

QByteArray udpPacket(uint8_t host, uint16_t sourcePort, const QByteArray& payload)
{
    QByteArray packet(14 + 20 + 8 + payload.size(), '\0');
    putBe16(packet, 12, 0x0800);
    packet[14] = 0x45;
    putBe16(packet, 16, uint16_t(20 + 8 + payload.size()));
    packet[22] = 64;
    packet[23] = 17;
    packet[26] = char(192); packet[27] = char(168); packet[28] = 1; packet[29] = char(host);
    packet[30] = char(192); packet[31] = char(168); packet[32] = 1; packet[33] = 100;
    putBe16(packet, 34, sourcePort);
    putBe16(packet, 36, 60000);
    putBe16(packet, 38, uint16_t(8 + payload.size()));
    std::copy(payload.cbegin(), payload.cend(), packet.begin() + 42);
    return packet;
}

QByteArray pointPayload(uint64_t timestamp)
{
    QByteArray payload(50, '\0');
    payload[0] = 1;
    putLe16(payload, 1, uint16_t(payload.size()));
    putLe16(payload, 5, 1);
    payload[10] = 1;
    putLe64(payload, 28, timestamp);
    putLe32(payload, 36, 1000);
    putLe32(payload, 40, 2000);
    putLe32(payload, 44, 3000);
    payload[48] = 80;
    return payload;
}

QByteArray imuPayload(uint64_t timestamp)
{
    QByteArray payload(60, '\0');
    payload[0] = 1;
    putLe16(payload, 1, uint16_t(payload.size()));
    putLe16(payload, 5, 1);
    putLe16(payload, 7, 42);
    payload[10] = 0;
    putLe64(payload, 28, timestamp);
    const float values[] = {1, 2, 3, 4, 5, 6};
    std::memcpy(payload.data() + 36, values, sizeof(values));
    return payload;
}

QByteArray infoPayload(const QByteArray& sn)
{
    QByteArray payload(28 + 4 + sn.size() + 8 + 8, '\0');
    payload[0] = char(0xAA);
    payload[1] = 1;
    putLe16(payload, 2, uint16_t(payload.size()));
    putLe16(payload, 8, 0x0102);
    payload[11] = 1;
    putLe16(payload, 24, 3);
    putLe16(payload, 28, 0x8000);
    putLe16(payload, 30, uint16_t(sn.size()));
    std::copy(sn.cbegin(), sn.cend(), payload.begin() + 32);
    const int temperatureOffset = 32 + sn.size();
    putLe16(payload, temperatureOffset, 0x8007);
    putLe16(payload, temperatureOffset + 2, 4);
    putLe32(payload, temperatureOffset + 4, 2534);
    const int ntpOffset = temperatureOffset + 8;
    putLe16(payload, ntpOffset, 0x0025);
    putLe16(payload, ntpOffset + 2, 4);
    payload[ntpOffset + 4] = char(192);
    payload[ntpOffset + 5] = char(168);
    payload[ntpOffset + 6] = 1;
    payload[ntpOffset + 7] = 20;
    return payload;
}

void appendLe16(QByteArray& bytes, uint16_t value)
{
    bytes.append(char(value & 0xFF));
    bytes.append(char(value >> 8));
}

void appendLe32(QByteArray& bytes, uint32_t value)
{
    for (int i = 0; i < 4; ++i) bytes.append(char(value >> (i * 8)));
}

bool writePcap(const QString& path)
{
    QByteArray file;
    appendLe32(file, 0xA1B2C3D4);
    appendLe16(file, 2); appendLe16(file, 4);
    appendLe32(file, 0); appendLe32(file, 0); appendLe32(file, 65535); appendLe32(file, 1);
    const QVector<QByteArray> packets = {
        udpPacket(10, 56200, infoPayload(QByteArray("SN00000000000001", 16))),
        udpPacket(10, 56300, pointPayload(1000000000ULL)),
        udpPacket(11, 56300, pointPayload(1001000000ULL)),
        udpPacket(10, 56400, imuPayload(1002000000ULL)),
        udpPacket(11, 56400, imuPayload(1003000000ULL))
    };
    uint32_t usec = 0;
    for (const QByteArray& packet : packets) {
        appendLe32(file, 1); appendLe32(file, usec += 1000);
        appendLe32(file, uint32_t(packet.size())); appendLe32(file, uint32_t(packet.size()));
        file.append(packet);
    }
    QFile output(path);
    return output.open(QIODevice::WriteOnly) && output.write(file) == file.size();
}

bool expect(bool condition, const QString& message)
{
    if (!condition) std::cerr << message.toStdString() << '\n';
    return condition;
}

} // namespace

int main()
{
    QTemporaryDir directory;
    bool ok = expect(directory.isValid(), QStringLiteral("temporary directory failed"));
    const QString pcapPath = QDir(directory.path()).filePath(QStringLiteral("sample.pcap"));
    ok &= expect(writePcap(pcapPath), QStringLiteral("test PCAP creation failed"));

    const auto lvx = PcapExtractor::extract(pcapPath, directory.path(), PcapExtractor::Kind::Lvx2, PcapExtractor::Mode::Merge);
    ok &= expect(lvx.ok && lvx.outputFiles.size() == 1, lvx.errorMessage);
    if (lvx.ok) {
        QFile file(lvx.outputFiles.first());
        Lvx2PublicHeader publicHeader{};
        Lvx2PrivateHeader privateHeader{};
        ok &= expect(file.open(QIODevice::ReadOnly), QStringLiteral("LVX2 output open failed"));
        ok &= expect(file.read(reinterpret_cast<char*>(&publicHeader), sizeof(publicHeader)) == sizeof(publicHeader), QStringLiteral("LVX2 public header failed"));
        ok &= expect(file.read(reinterpret_cast<char*>(&privateHeader), sizeof(privateHeader)) == sizeof(privateHeader), QStringLiteral("LVX2 private header failed"));
        ok &= expect(privateHeader.device_count == 2, QStringLiteral("LVX2 must contain two devices"));
        bool foundNull = false;
        for (int i = 0; i < privateHeader.device_count; ++i) {
            Lvx2DeviceInfo device{};
            file.read(reinterpret_cast<char*>(&device), sizeof(device));
            foundNull |= QString::fromLatin1(device.lidar_sn) == QStringLiteral("null");
        }
        ok &= expect(foundNull, QStringLiteral("missing device SN must be null"));
    }

    const auto imu = PcapExtractor::extract(pcapPath, directory.path(), PcapExtractor::Kind::ImuCsv, PcapExtractor::Mode::SplitByDevice);
    ok &= expect(imu.ok && imu.outputFiles.size() == 2, imu.errorMessage);
    if (imu.ok) {
        bool foundIpFileName = false;
        for (const QString& path : imu.outputFiles) {
            foundIpFileName |= QFileInfo(path).fileName().contains(QStringLiteral("192.168.1.10"));
            QFile csv(path);
            ok &= expect(csv.open(QIODevice::ReadOnly), QStringLiteral("IMU CSV open failed"));
            const QList<QByteArray> lines = csv.readAll().split('\n');
            ok &= expect(!lines.isEmpty() && lines.first().startsWith("\xEF\xBB\xBFpacket_index,udp_cnt,"),
                         QStringLiteral("IMU CSV packet_index/udp_cnt header mismatch"));
            if (lines.size() >= 2 && !lines[1].trimmed().isEmpty()) {
                const QList<QByteArray> fields = lines[1].split(',');
                ok &= expect(fields.size() == 14 && fields[0].toULongLong() > 0,
                             QStringLiteral("IMU CSV packet_index mismatch"));
                ok &= expect(fields[1] == "42", QStringLiteral("IMU CSV udp_cnt mismatch"));
            }
        }
        ok &= expect(foundIpFileName, QStringLiteral("split filename must contain IP address"));
    }

    const auto info = PcapExtractor::extract(pcapPath, directory.path(), PcapExtractor::Kind::InfoCsv, PcapExtractor::Mode::Merge);
    ok &= expect(info.ok && info.outputFiles.size() == 1, info.errorMessage);
    if (info.ok) {
        QFile csv(info.outputFiles.first());
        ok &= expect(csv.open(QIODevice::ReadOnly), QStringLiteral("INFO CSV open failed"));
        const QList<QByteArray> lines = csv.readAll().split('\n');
        ok &= expect(!lines.isEmpty() && lines.first().count(',') == 34, QStringLiteral("INFO CSV must have 35 columns"));
        ok &= expect(lines.first().contains("核心温度 ℃"), QStringLiteral("INFO temperature header mismatch"));
        ok &= expect(lines.size() >= 2 && lines[1].contains("SN00000000000001"), QStringLiteral("INFO CSV SN missing"));
        ok &= expect(lines.size() >= 2 && lines[1].contains(",25.34,"), QStringLiteral("INFO temperature must be numeric"));
        ok &= expect(lines.size() >= 2 && lines[1].contains(",192.168.1.20\r"), QStringLiteral("INFO NTP server IP mismatch"));
    }
    return ok ? 0 : 1;
}

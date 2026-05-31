#ifndef LIVOXCORE_LIDARDIAGNOSTICS_H
#define LIVOXCORE_LIDARDIAGNOSTICS_H

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
    default: return QStringLiteral("#6B7280");
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

} // namespace LivoxCore

#endif // LIVOXCORE_LIDARDIAGNOSTICS_H

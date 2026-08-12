#ifndef LVX2_LVX2WRITER_H
#define LVX2_LVX2WRITER_H

#include "Lvx2Types.h"

#include <QByteArray>
#include <QFile>
#include <QString>
#include <QVector>

#include <cstdint>

namespace Lvx2 {

class Lvx2Writer {
public:
    bool open(const QString& filePath, const QVector<Lvx2DeviceInfo>& devices, QString& errorMessage);
    bool appendPacket(uint64_t captureTimestampNs,
                      const Lvx2PackageHeader& header,
                      const uint8_t* data,
                      size_t dataLength,
                      QString& errorMessage);
    bool close(QString& errorMessage);
    void abort();

private:
    bool flushFrame(QString& errorMessage);

    QFile file_;
    QVector<QByteArray> packages_;
    uint64_t frameStartNs_ = 0;
    uint64_t frameIndex_ = 0;
};

} // namespace Lvx2

#endif // LVX2_LVX2WRITER_H

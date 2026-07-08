#include "LivoxViewerWindow.h"

#include "PointCloudExport.h"
#include "LidarModelUtils.h"
#include "Lvx2PointParser.h"
#include "RosbagPlaybackSource.h"

#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QMap>
#include <QTextStream>
#include <QtEndian>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {

static bool readExact(QFile& file, char* data, qint64 size) {
    return file.read(data, size) == size;
}

} // namespace

bool LivoxViewerWindow::savePointCloudAsCSV(const QString& filePath, const QVector<PointCloudPoint>& points)
{
    return PointCloudExport::saveAsCSV(filePath, points);
}

bool LivoxViewerWindow::savePointCloudAsTXT(const QString& filePath, const QVector<PointCloudPoint>& points)
{
    return PointCloudExport::saveAsTXT(filePath, points);
}

bool LivoxViewerWindow::convertRosbagToPcdFile(const QString& sourcePath,
                                               const QString& outputPathNoExt,
                                               Lvx2ConvertMode mode,
                                               const std::function<void(int, int)>& progress)
{
    RosbagPlaybackSource source(static_cast<int>(frameIntervalMs));
    if (!source.load(sourcePath)) {
        return false;
    }

    QMap<uint32_t, bool> deviceVisibility;
    for (const Playback::DeviceInfo& device : source.devices()) {
        deviceVisibility.insert(device.lidarId, true);
    }

    const int total = source.frameCount();
    if (mode == Lvx2ConvertMode::SplitBy100ms) {
        for (int i = 0; i < total; ++i) {
            PointCloudFrame frame;
            if (!source.readFrame(i, deviceVisibility, frame)) {
                return false;
            }
            const QString filePath = QString("%1_%2.pcd").arg(outputPathNoExt).arg(i + 1, 5, 10, QChar('0'));
            if (!PointCloudExport::saveAsPCD(filePath, frame.points)) {
                return false;
            }
            if (progress) {
                progress(i + 1, total);
            }
            QCoreApplication::processEvents();
        }
        return true;
    }

    const QString outPath = outputPathNoExt + ".pcd";
    const QFileInfo fi(outPath);
    QFile pcdTmpFile(fi.absolutePath() + "/" + fi.completeBaseName() + ".pcd.tmp_points");
    if (!pcdTmpFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return false;
    }

    qint64 pointCount = 0;
    for (int i = 0; i < total; ++i) {
        PointCloudFrame frame;
        if (!source.readFrame(i, deviceVisibility, frame)) {
            pcdTmpFile.close();
            pcdTmpFile.remove();
            return false;
        }
        for (const PointCloudPoint& point : frame.points) {
            if (!PointCloudExport::writePcdPoint(pcdTmpFile, point)) {
                pcdTmpFile.close();
                pcdTmpFile.remove();
                return false;
            }
            ++pointCount;
        }
        if (progress) {
            progress(i + 1, total);
        }
        QCoreApplication::processEvents();
    }
    pcdTmpFile.close();

    QFile outFile(outPath);
    if (!outFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        pcdTmpFile.remove();
        return false;
    }
    if (!PointCloudExport::writePcdHeader(outFile, pointCount)) {
        outFile.close();
        pcdTmpFile.remove();
        return false;
    }
    if (!pcdTmpFile.open(QIODevice::ReadOnly)) {
        outFile.close();
        pcdTmpFile.remove();
        return false;
    }
    while (!pcdTmpFile.atEnd()) {
        const QByteArray chunk = pcdTmpFile.read(1 << 20);
        if (chunk.isEmpty() && pcdTmpFile.error() != QFile::NoError) {
            outFile.close();
            pcdTmpFile.close();
            pcdTmpFile.remove();
            return false;
        }
        if (outFile.write(chunk) != chunk.size()) {
            outFile.close();
            pcdTmpFile.close();
            pcdTmpFile.remove();
            return false;
        }
    }
    pcdTmpFile.close();
    pcdTmpFile.remove();
    return true;
}

bool LivoxViewerWindow::convertLvx2File(const QString& sourcePath,
                                 const QString& outputPathNoExt,
                                 Lvx2ConvertMode mode,
                                 Lvx2ConvertFormat format,
                                 const std::function<void(int, int)>& progress)
{
    QFile file(sourcePath);
    if (!file.open(QIODevice::ReadOnly)) {
        return false;
    }

    Lvx2PublicHeader publicHeader{};
    Lvx2PrivateHeader privateHeader{};
    if (!readExact(file, reinterpret_cast<char*>(&publicHeader), sizeof(publicHeader)) ||
        !readExact(file, reinterpret_cast<char*>(&privateHeader), sizeof(privateHeader))) {
        return false;
    }
    if (std::memcmp(publicHeader.signature, "livox_tech", 10) != 0 || publicHeader.magic_code != 0xAC0EA767) {
        return false;
    }

    QMap<uint32_t, Lvx2PlaybackExtrinsic> extrinsics;
    QMap<uint32_t, int> lineCounts;
    const int deviceCount = std::max(0, int(privateHeader.device_count));
    for (int i = 0; i < deviceCount; ++i) {
        Lvx2DeviceInfo deviceInfo{};
        if (!readExact(file, reinterpret_cast<char*>(&deviceInfo), sizeof(deviceInfo))) {
            return false;
        }
        extrinsics.insert(deviceInfo.lidar_id, Lvx2PointParser::makeExtrinsic(deviceInfo));
        lineCounts.insert(deviceInfo.lidar_id, LivoxCore::lineCountForDeviceType(deviceInfo.device_type));
    }

    QVector<Lvx2PlaybackFrameIndex> rawFrames;
    const qint64 fileSize = file.size();
    while (file.pos() + qint64(sizeof(Lvx2FrameHeader)) <= fileSize) {
        Lvx2FrameHeader frameHeader{};
        const qint64 frameOffset = file.pos();
        if (!readExact(file, reinterpret_cast<char*>(&frameHeader), sizeof(frameHeader))) {
            return false;
        }
        if (frameHeader.next_offset <= frameHeader.current_offset) {
            break;
        }

        Lvx2PlaybackFrameIndex index;
        index.offset = uint64_t(frameOffset);
        index.nextOffset = frameHeader.next_offset;
        index.frameIndex = frameHeader.frame_index;
        rawFrames.push_back(index);

        if (!file.seek(qint64(frameHeader.next_offset))) {
            return false;
        }
    }
    if (rawFrames.isEmpty()) {
        return false;
    }

    QString ext;
    switch (format) {
    case Lvx2ConvertFormat::PCD: ext = ".pcd"; break;
    case Lvx2ConvertFormat::LAS: ext = ".las"; break;
    case Lvx2ConvertFormat::CSV: ext = ".csv"; break;
    case Lvx2ConvertFormat::TXT: ext = ".txt"; break;
    }

    auto writePoints = [format](const QString& path, const QVector<PointCloudPoint>& pts) -> bool {
        switch (format) {
        case Lvx2ConvertFormat::PCD: return PointCloudExport::saveAsPCD(path, pts);
        case Lvx2ConvertFormat::LAS: return PointCloudExport::saveAsLAS(path, pts);
        case Lvx2ConvertFormat::CSV: return PointCloudExport::saveAsCSV(path, pts);
        case Lvx2ConvertFormat::TXT: return PointCloudExport::saveAsTXT(path, pts);
        }
        return false;
    };

    if (mode == Lvx2ConvertMode::MergeAllToOne) {
        const QString outPath = outputPathNoExt + ext;
        QFile outFile;
        QFile pcdTmpFile;
        QTextStream textOut;

        qint64 pointCount = 0;
        const double lasScale = 0.001;
        PointCloudExport::Bounds lasBounds;
        bool hasLasPoint = false;

        if (format == Lvx2ConvertFormat::CSV || format == Lvx2ConvertFormat::TXT) {
            outFile.setFileName(outPath);
            if (!outFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
                return false;
            }
            textOut.setDevice(&outFile);
            textOut.setRealNumberNotation(QTextStream::FixedNotation);
            textOut.setRealNumberPrecision(6);
            if (format == Lvx2ConvertFormat::CSV) {
                textOut << "x,y,z,reflectivity,tag\n";
            }
        } else if (format == Lvx2ConvertFormat::PCD) {
            const QFileInfo fi(outPath);
            const QString tmpPath = fi.absolutePath() + "/" + fi.completeBaseName() + ".pcd.tmp_points";
            pcdTmpFile.setFileName(tmpPath);
            if (!pcdTmpFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                return false;
            }
        } else if (format == Lvx2ConvertFormat::LAS) {
            outFile.setFileName(outPath);
            if (!outFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                return false;
            }
            if (!PointCloudExport::writeLasHeader(outFile, 0, {}, lasScale)) {
                outFile.close();
                return false;
            }
        }

        const int total = rawFrames.size();
        bool writeFailed = false;
        for (int i = 0; i < total; ++i) {
            const Lvx2PlaybackFrameIndex& frameIndex = rawFrames.at(i);
            qint64 cursor = qint64(frameIndex.offset) + qint64(sizeof(Lvx2FrameHeader));
            while (cursor + qint64(sizeof(Lvx2PackageHeader)) <= qint64(frameIndex.nextOffset)) {
                if (!file.seek(cursor)) {
                    return false;
                }
                Lvx2PackageHeader packageHeader{};
                if (!readExact(file, reinterpret_cast<char*>(&packageHeader), sizeof(packageHeader))) {
                    return false;
                }
                cursor += qint64(sizeof(Lvx2PackageHeader));
                const qint64 dataLength = packageHeader.data_length;
                if (dataLength < 0 || cursor + dataLength > qint64(frameIndex.nextOffset)) {
                    break;
                }
                const QByteArray payload = file.read(dataLength);
                if (payload.size() != dataLength) {
                    return false;
                }
                cursor += dataLength;
                const auto extrinsicIt = extrinsics.constFind(packageHeader.lidar_id);
                const Lvx2PlaybackExtrinsic* extrinsic =
                    (extrinsicIt == extrinsics.constEnd()) ? nullptr : &extrinsicIt.value();
                QVector<PointCloudPoint> packagePoints;
                Lvx2PointParser::appendPackagePoints(packageHeader,
                                                     payload,
                                                     extrinsic,
                                                     packagePoints,
                                                     lineCounts.value(packageHeader.lidar_id, 1));
                for (const PointCloudPoint& p : packagePoints) {
                    if (writeFailed) {
                        break;
                    }
                    if (format == Lvx2ConvertFormat::CSV) {
                        textOut << p.x << ',' << p.y << ',' << p.z << ',' << int(p.reflectivity) << ',' << int(p.tag) << "\n";
                    } else if (format == Lvx2ConvertFormat::TXT) {
                        textOut << p.x << ' ' << p.y << ' ' << p.z << ' ' << int(p.reflectivity) << ' ' << int(p.tag) << "\n";
                    } else if (format == Lvx2ConvertFormat::PCD) {
                        if (!PointCloudExport::writePcdPoint(pcdTmpFile, p)) {
                            writeFailed = true;
                            break;
                        }
                    } else if (format == Lvx2ConvertFormat::LAS) {
                        if (!hasLasPoint) {
                            lasBounds.minX = lasBounds.maxX = p.x;
                            lasBounds.minY = lasBounds.maxY = p.y;
                            lasBounds.minZ = lasBounds.maxZ = p.z;
                            hasLasPoint = true;
                        } else {
                            lasBounds.minX = std::min(lasBounds.minX, double(p.x));
                            lasBounds.minY = std::min(lasBounds.minY, double(p.y));
                            lasBounds.minZ = std::min(lasBounds.minZ, double(p.z));
                            lasBounds.maxX = std::max(lasBounds.maxX, double(p.x));
                            lasBounds.maxY = std::max(lasBounds.maxY, double(p.y));
                            lasBounds.maxZ = std::max(lasBounds.maxZ, double(p.z));
                        }
                        if (!PointCloudExport::writeLasPoint(outFile, p, lasScale)) {
                            writeFailed = true;
                            break;
                        }
                    }
                    ++pointCount;
                }
                if (writeFailed) {
                    break;
                }
            }
            if (writeFailed) {
                return false;
            }
            if (progress) {
                progress(i + 1, total);
            }
            QCoreApplication::processEvents();
        }

        if (format == Lvx2ConvertFormat::PCD) {
            pcdTmpFile.close();

            outFile.setFileName(outPath);
            if (!outFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                pcdTmpFile.remove();
                return false;
            }
            if (!PointCloudExport::writePcdHeader(outFile, pointCount)) {
                outFile.close();
                pcdTmpFile.remove();
                return false;
            }

            if (!pcdTmpFile.open(QIODevice::ReadOnly)) {
                outFile.close();
                pcdTmpFile.remove();
                return false;
            }
            while (!pcdTmpFile.atEnd()) {
                QByteArray chunk = pcdTmpFile.read(1 << 20);
                if (chunk.isEmpty() && pcdTmpFile.error() != QFile::NoError) {
                    outFile.close();
                    pcdTmpFile.close();
                    pcdTmpFile.remove();
                    return false;
                }
                outFile.write(chunk);
            }
            pcdTmpFile.close();
            pcdTmpFile.remove();
        } else if (format == Lvx2ConvertFormat::LAS) {
            const quint32 countU32 = static_cast<quint32>(std::min<qint64>(pointCount, std::numeric_limits<quint32>::max()));
            outFile.seek(0);
            if (!PointCloudExport::writeLasHeader(outFile, countU32, lasBounds, lasScale)) {
                outFile.close();
                return false;
            }
            outFile.close();
        } else {
            textOut.flush();
            outFile.close();
        }
        return true;
    }

    const int frameStep = 2;
    const int groupCount = (rawFrames.size() + frameStep - 1) / frameStep;
    for (int g = 0; g < groupCount; ++g) {
        QVector<PointCloudPoint> points;
        const int rawBegin = g * frameStep;
        const int rawEnd = std::min(rawBegin + frameStep, int(rawFrames.size()));
        for (int r = rawBegin; r < rawEnd; ++r) {
            const Lvx2PlaybackFrameIndex& frameIndex = rawFrames.at(r);
            qint64 cursor = qint64(frameIndex.offset) + qint64(sizeof(Lvx2FrameHeader));
            while (cursor + qint64(sizeof(Lvx2PackageHeader)) <= qint64(frameIndex.nextOffset)) {
                if (!file.seek(cursor)) {
                    return false;
                }
                Lvx2PackageHeader packageHeader{};
                if (!readExact(file, reinterpret_cast<char*>(&packageHeader), sizeof(packageHeader))) {
                    return false;
                }
                cursor += qint64(sizeof(Lvx2PackageHeader));
                const qint64 dataLength = packageHeader.data_length;
                if (dataLength < 0 || cursor + dataLength > qint64(frameIndex.nextOffset)) {
                    break;
                }
                const QByteArray payload = file.read(dataLength);
                if (payload.size() != dataLength) {
                    return false;
                }
                cursor += dataLength;
                const auto extrinsicIt = extrinsics.constFind(packageHeader.lidar_id);
                const Lvx2PlaybackExtrinsic* extrinsic =
                    (extrinsicIt == extrinsics.constEnd()) ? nullptr : &extrinsicIt.value();
                Lvx2PointParser::appendPackagePoints(packageHeader,
                                                     payload,
                                                     extrinsic,
                                                     points,
                                                     lineCounts.value(packageHeader.lidar_id, 1));
            }
        }
        const QString filePath = QString("%1_%2%3").arg(outputPathNoExt).arg(g + 1, 5, 10, QChar('0')).arg(ext);
        if (!writePoints(filePath, points)) {
            return false;
        }
        if (progress) {
            progress(g + 1, groupCount);
        }
        QCoreApplication::processEvents();
    }
    return true;
}

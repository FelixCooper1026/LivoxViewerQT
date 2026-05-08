#include "mainwindow.h"

#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
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

static MainWindow::Lvx2PlaybackExtrinsic makeExtrinsic(const LVX2DeviceInfo& info) {
    MainWindow::Lvx2PlaybackExtrinsic extrinsic;
    extrinsic.enabled = (info.extrinsic_enable != 0);
    extrinsic.transform.setToIdentity();
    if (!extrinsic.enabled) {
        return extrinsic;
    }

    extrinsic.transform.translate(info.x / 100.0f, info.y / 100.0f, info.z / 100.0f);
    extrinsic.transform.rotate(info.yaw, 0.0f, 0.0f, 1.0f);
    extrinsic.transform.rotate(info.pitch, 0.0f, 1.0f, 0.0f);
    extrinsic.transform.rotate(info.roll, 1.0f, 0.0f, 0.0f);
    return extrinsic;
}

static void applyExtrinsicTransform(const MainWindow::Lvx2PlaybackExtrinsic* extrinsic, Point3D& point) {
    if (!extrinsic || !extrinsic->enabled) {
        return;
    }

    const QVector4D transformed = extrinsic->transform * QVector4D(point.x, point.y, point.z, 1.0f);
    point.x = transformed.x();
    point.y = transformed.y();
    point.z = transformed.z();
}

template <typename T>
static T safeReadPoint(const char* data, int index) {
    T point;
    std::memcpy(&point, data + index * sizeof(T), sizeof(T));
    return point;
}

static void appendPackagePoints(const LVX2PackageHeader& header,
                                const QByteArray& payload,
                                const MainWindow::Lvx2PlaybackExtrinsic* extrinsic,
                                QVector<Point3D>& points) {
    if (payload.isEmpty()) {
        return;
    }

    const char* rawData = payload.constData();
    switch (header.data_type) {
    case kLivoxLidarCartesianCoordinateHighData: {
        const int pointCount = int(payload.size() / int(sizeof(LivoxLidarCartesianHighRawPoint)));
        for (int i = 0; i < pointCount; ++i) {
            auto raw = safeReadPoint<LivoxLidarCartesianHighRawPoint>(rawData, i);
            Point3D point{};
            point.x = raw.x / 1000.0f;
            point.y = raw.y / 1000.0f;
            point.z = raw.z / 1000.0f;
            point.reflectivity = raw.reflectivity;
            point.tag = raw.tag;
            applyExtrinsicTransform(extrinsic, point);
            points.push_back(point);
        }
        break;
    }
    case kLivoxLidarCartesianCoordinateLowData: {
        const int pointCount = int(payload.size() / int(sizeof(LivoxLidarCartesianLowRawPoint)));
        for (int i = 0; i < pointCount; ++i) {
            auto raw = safeReadPoint<LivoxLidarCartesianLowRawPoint>(rawData, i);
            Point3D point{};
            point.x = raw.x / 100.0f;
            point.y = raw.y / 100.0f;
            point.z = raw.z / 100.0f;
            point.reflectivity = raw.reflectivity;
            point.tag = raw.tag;
            applyExtrinsicTransform(extrinsic, point);
            points.push_back(point);
        }
        break;
    }
    case kLivoxLidarSphericalCoordinateData: {
        const int pointCount = int(payload.size() / int(sizeof(LivoxLidarSpherPoint)));
        for (int i = 0; i < pointCount; ++i) {
            auto raw = safeReadPoint<LivoxLidarSpherPoint>(rawData, i);
            Point3D point{};
            const float depth = raw.depth / 1000.0f;
            const float theta = raw.theta / 100.0f * float(M_PI) / 180.0f;
            const float phi = raw.phi / 100.0f * float(M_PI) / 180.0f;
            point.x = depth * std::sin(theta) * std::cos(phi);
            point.y = depth * std::sin(theta) * std::sin(phi);
            point.z = depth * std::cos(theta);
            point.reflectivity = raw.reflectivity;
            point.tag = raw.tag;
            applyExtrinsicTransform(extrinsic, point);
            points.push_back(point);
        }
        break;
    }
    case kLivoxLidarDoubleEchoData: {
        const int pointCount = int(payload.size() / int(sizeof(LivoxLidarDoubleEchoRawPoint)));
        for (int i = 0; i < pointCount; ++i) {
            auto raw = safeReadPoint<LivoxLidarDoubleEchoRawPoint>(rawData, i);
            Point3D p1{};
            p1.x = raw.x1 / 1000.0f;
            p1.y = raw.y1 / 1000.0f;
            p1.z = raw.z1 / 1000.0f;
            p1.reflectivity = raw.reflectivity1;
            p1.tag = raw.tag1;
            applyExtrinsicTransform(extrinsic, p1);
            points.push_back(p1);

            Point3D p2{};
            p2.x = raw.x2 / 1000.0f;
            p2.y = raw.y2 / 1000.0f;
            p2.z = raw.z2 / 1000.0f;
            p2.reflectivity = raw.reflectivity2;
            p2.tag = raw.tag2;
            applyExtrinsicTransform(extrinsic, p2);
            points.push_back(p2);
        }
        break;
    }
    default:
        break;
    }
}

template <typename PointHandler>
static void iteratePackagePoints(const LVX2PackageHeader& header,
                                 const QByteArray& payload,
                                 const MainWindow::Lvx2PlaybackExtrinsic* extrinsic,
                                 PointHandler&& onPoint) {
    if (payload.isEmpty()) {
        return;
    }

    const char* rawData = payload.constData();
    switch (header.data_type) {
    case kLivoxLidarCartesianCoordinateHighData: {
        const int pointCount = int(payload.size() / int(sizeof(LivoxLidarCartesianHighRawPoint)));
        for (int i = 0; i < pointCount; ++i) {
            auto raw = safeReadPoint<LivoxLidarCartesianHighRawPoint>(rawData, i);
            Point3D point{};
            point.x = raw.x / 1000.0f;
            point.y = raw.y / 1000.0f;
            point.z = raw.z / 1000.0f;
            point.reflectivity = raw.reflectivity;
            point.tag = raw.tag;
            applyExtrinsicTransform(extrinsic, point);
            onPoint(point);
        }
        break;
    }
    case kLivoxLidarCartesianCoordinateLowData: {
        const int pointCount = int(payload.size() / int(sizeof(LivoxLidarCartesianLowRawPoint)));
        for (int i = 0; i < pointCount; ++i) {
            auto raw = safeReadPoint<LivoxLidarCartesianLowRawPoint>(rawData, i);
            Point3D point{};
            point.x = raw.x / 100.0f;
            point.y = raw.y / 100.0f;
            point.z = raw.z / 100.0f;
            point.reflectivity = raw.reflectivity;
            point.tag = raw.tag;
            applyExtrinsicTransform(extrinsic, point);
            onPoint(point);
        }
        break;
    }
    case kLivoxLidarSphericalCoordinateData: {
        const int pointCount = int(payload.size() / int(sizeof(LivoxLidarSpherPoint)));
        for (int i = 0; i < pointCount; ++i) {
            auto raw = safeReadPoint<LivoxLidarSpherPoint>(rawData, i);
            Point3D point{};
            const float depth = raw.depth / 1000.0f;
            const float theta = raw.theta / 100.0f * float(M_PI) / 180.0f;
            const float phi = raw.phi / 100.0f * float(M_PI) / 180.0f;
            point.x = depth * std::sin(theta) * std::cos(phi);
            point.y = depth * std::sin(theta) * std::sin(phi);
            point.z = depth * std::cos(theta);
            point.reflectivity = raw.reflectivity;
            point.tag = raw.tag;
            applyExtrinsicTransform(extrinsic, point);
            onPoint(point);
        }
        break;
    }
    case kLivoxLidarDoubleEchoData: {
        const int pointCount = int(payload.size() / int(sizeof(LivoxLidarDoubleEchoRawPoint)));
        for (int i = 0; i < pointCount; ++i) {
            auto raw = safeReadPoint<LivoxLidarDoubleEchoRawPoint>(rawData, i);
            Point3D p1{};
            p1.x = raw.x1 / 1000.0f;
            p1.y = raw.y1 / 1000.0f;
            p1.z = raw.z1 / 1000.0f;
            p1.reflectivity = raw.reflectivity1;
            p1.tag = raw.tag1;
            applyExtrinsicTransform(extrinsic, p1);
            onPoint(p1);

            Point3D p2{};
            p2.x = raw.x2 / 1000.0f;
            p2.y = raw.y2 / 1000.0f;
            p2.z = raw.z2 / 1000.0f;
            p2.reflectivity = raw.reflectivity2;
            p2.tag = raw.tag2;
            applyExtrinsicTransform(extrinsic, p2);
            onPoint(p2);
        }
        break;
    }
    default:
        break;
    }
}

static inline quint16 clampU16(int v) {
    return quint16(std::max(0, std::min(65535, v)));
}

} // namespace

bool MainWindow::savePointCloudAsCSV(const QString& filePath, const QVector<Point3D>& points)
{
    QFile f(filePath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        return false;
    }
    QTextStream out(&f);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(6);
    out << "x,y,z,reflectivity,tag\n";
    for (const Point3D& p : points) {
        out << p.x << ',' << p.y << ',' << p.z << ',' << int(p.reflectivity) << ',' << int(p.tag) << "\n";
    }
    f.close();
    return true;
}

bool MainWindow::savePointCloudAsTXT(const QString& filePath, const QVector<Point3D>& points)
{
    QFile f(filePath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        return false;
    }
    QTextStream out(&f);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(6);
    for (const Point3D& p : points) {
        out << p.x << ' ' << p.y << ' ' << p.z << ' ' << int(p.reflectivity) << ' ' << int(p.tag) << "\n";
    }
    f.close();
    return true;
}

bool MainWindow::convertLvx2File(const QString& sourcePath,
                                 const QString& outputPathNoExt,
                                 Lvx2ConvertMode mode,
                                 Lvx2ConvertFormat format,
                                 const std::function<void(int, int)>& progress)
{
    QFile file(sourcePath);
    if (!file.open(QIODevice::ReadOnly)) {
        return false;
    }

    LVX2PublicHeader publicHeader{};
    LVX2PrivateHeader privateHeader{};
    if (!readExact(file, reinterpret_cast<char*>(&publicHeader), sizeof(publicHeader)) ||
        !readExact(file, reinterpret_cast<char*>(&privateHeader), sizeof(privateHeader))) {
        return false;
    }
    if (std::memcmp(publicHeader.signature, "livox_tech", 10) != 0 || publicHeader.magic_code != 0xAC0EA767) {
        return false;
    }

    QMap<uint32_t, Lvx2PlaybackExtrinsic> extrinsics;
    const int deviceCount = std::max(0, int(privateHeader.device_count));
    for (int i = 0; i < deviceCount; ++i) {
        LVX2DeviceInfo deviceInfo{};
        if (!readExact(file, reinterpret_cast<char*>(&deviceInfo), sizeof(deviceInfo))) {
            return false;
        }
        extrinsics.insert(deviceInfo.lidar_id, makeExtrinsic(deviceInfo));
    }

    QVector<Lvx2PlaybackFrameIndex> rawFrames;
    const qint64 fileSize = file.size();
    while (file.pos() + qint64(sizeof(LVX2FrameHeader)) <= fileSize) {
        LVX2FrameHeader frameHeader{};
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

    auto writePoints = [this, format](const QString& path, const QVector<Point3D>& pts) -> bool {
        switch (format) {
        case Lvx2ConvertFormat::PCD: return savePointCloudAsPCD(path, pts);
        case Lvx2ConvertFormat::LAS: return savePointCloudAsLAS(path, pts);
        case Lvx2ConvertFormat::CSV: return savePointCloudAsCSV(path, pts);
        case Lvx2ConvertFormat::TXT: return savePointCloudAsTXT(path, pts);
        }
        return false;
    };

    if (mode == Lvx2ConvertMode::MergeAllToOne) {
        const QString outPath = outputPathNoExt + ext;
        QFile outFile;
        QFile pcdTmpFile;
        QTextStream textOut;
        QTextStream pcdTmpOut;

        qint64 pointCount = 0;
        const double lasScale = 0.001;
        double lasMinX = std::numeric_limits<double>::max();
        double lasMinY = std::numeric_limits<double>::max();
        double lasMinZ = std::numeric_limits<double>::max();
        double lasMaxX = -std::numeric_limits<double>::max();
        double lasMaxY = -std::numeric_limits<double>::max();
        double lasMaxZ = -std::numeric_limits<double>::max();

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
            if (!pcdTmpFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
                return false;
            }
            pcdTmpOut.setDevice(&pcdTmpFile);
            pcdTmpOut.setRealNumberNotation(QTextStream::FixedNotation);
            pcdTmpOut.setRealNumberPrecision(6);
        } else if (format == Lvx2ConvertFormat::LAS) {
            outFile.setFileName(outPath);
            if (!outFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                return false;
            }
            QByteArray header(227, 0);
            header[0] = 'L'; header[1] = 'A'; header[2] = 'S'; header[3] = 'F';
            header[24] = 1;
            header[25] = 2;
            QByteArray sys = QByteArray("LivoxViewerQT").leftJustified(32, '\0', true);
            std::copy(sys.begin(), sys.end(), header.begin() + 26);
            QByteArray gen = QByteArray("LVX").leftJustified(32, '\0', true);
            std::copy(gen.begin(), gen.end(), header.begin() + 58);
            qToLittleEndian<quint16>(227, reinterpret_cast<uchar*>(header.data() + 94));
            qToLittleEndian<quint32>(227, reinterpret_cast<uchar*>(header.data() + 96));
            qToLittleEndian<quint32>(0, reinterpret_cast<uchar*>(header.data() + 100));
            header[104] = 0;
            qToLittleEndian<quint16>(20, reinterpret_cast<uchar*>(header.data() + 105));
            qToLittleEndian<double>(lasScale, reinterpret_cast<uchar*>(header.data() + 131));
            qToLittleEndian<double>(lasScale, reinterpret_cast<uchar*>(header.data() + 139));
            qToLittleEndian<double>(lasScale, reinterpret_cast<uchar*>(header.data() + 147));
            qToLittleEndian<double>(0.0, reinterpret_cast<uchar*>(header.data() + 155));
            qToLittleEndian<double>(0.0, reinterpret_cast<uchar*>(header.data() + 163));
            qToLittleEndian<double>(0.0, reinterpret_cast<uchar*>(header.data() + 171));
            if (outFile.write(header) != header.size()) {
                outFile.close();
                return false;
            }
        }

        const int total = rawFrames.size();
        QByteArray lasRec(20, 0);
        for (int i = 0; i < total; ++i) {
            const Lvx2PlaybackFrameIndex& frameIndex = rawFrames.at(i);
            qint64 cursor = qint64(frameIndex.offset) + qint64(sizeof(LVX2FrameHeader));
            while (cursor + qint64(sizeof(LVX2PackageHeader)) <= qint64(frameIndex.nextOffset)) {
                if (!file.seek(cursor)) {
                    return false;
                }
                LVX2PackageHeader packageHeader{};
                if (!readExact(file, reinterpret_cast<char*>(&packageHeader), sizeof(packageHeader))) {
                    return false;
                }
                cursor += qint64(sizeof(LVX2PackageHeader));
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
                iteratePackagePoints(packageHeader, payload, extrinsic, [&](const Point3D& p) {
                    if (format == Lvx2ConvertFormat::CSV) {
                        textOut << p.x << ',' << p.y << ',' << p.z << ',' << int(p.reflectivity) << ',' << int(p.tag) << "\n";
                    } else if (format == Lvx2ConvertFormat::TXT) {
                        textOut << p.x << ' ' << p.y << ' ' << p.z << ' ' << int(p.reflectivity) << ' ' << int(p.tag) << "\n";
                    } else if (format == Lvx2ConvertFormat::PCD) {
                        pcdTmpOut << p.x << ' ' << p.y << ' ' << p.z << ' ' << int(p.reflectivity) << ' ' << int(p.tag) << "\n";
                    } else if (format == Lvx2ConvertFormat::LAS) {
                        lasMinX = std::min(lasMinX, double(p.x));
                        lasMinY = std::min(lasMinY, double(p.y));
                        lasMinZ = std::min(lasMinZ, double(p.z));
                        lasMaxX = std::max(lasMaxX, double(p.x));
                        lasMaxY = std::max(lasMaxY, double(p.y));
                        lasMaxZ = std::max(lasMaxZ, double(p.z));
                        const qint32 xi = qint32(std::llround(double(p.x) / lasScale));
                        const qint32 yi = qint32(std::llround(double(p.y) / lasScale));
                        const qint32 zi = qint32(std::llround(double(p.z) / lasScale));
                        qToLittleEndian<qint32>(xi, reinterpret_cast<uchar*>(lasRec.data() + 0));
                        qToLittleEndian<qint32>(yi, reinterpret_cast<uchar*>(lasRec.data() + 4));
                        qToLittleEndian<qint32>(zi, reinterpret_cast<uchar*>(lasRec.data() + 8));
                        qToLittleEndian<quint16>(clampU16(int(p.reflectivity)), reinterpret_cast<uchar*>(lasRec.data() + 12));
                        lasRec[14] = 0;
                        lasRec[15] = 0;
                        lasRec[16] = 0;
                        lasRec[17] = static_cast<char>(p.tag);
                        qToLittleEndian<quint16>(0, reinterpret_cast<uchar*>(lasRec.data() + 18));
                        outFile.write(lasRec);
                    }
                    ++pointCount;
                });
            }
            if (progress) {
                progress(i + 1, total);
            }
            QCoreApplication::processEvents();
        }

        if (format == Lvx2ConvertFormat::PCD) {
            pcdTmpOut.flush();
            pcdTmpFile.close();

            outFile.setFileName(outPath);
            if (!outFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
                pcdTmpFile.remove();
                return false;
            }
            QTextStream finalOut(&outFile);
            finalOut.setRealNumberNotation(QTextStream::FixedNotation);
            finalOut.setRealNumberPrecision(6);
            finalOut << "# .PCD v0.7 - Point Cloud Data file\n";
            finalOut << "VERSION 0.7\n";
            finalOut << "FIELDS x y z intensity tag\n";
            finalOut << "SIZE 4 4 4 4 4\n";
            finalOut << "TYPE F F F F F\n";
            finalOut << "COUNT 1 1 1 1 1\n";
            finalOut << "WIDTH " << pointCount << "\n";
            finalOut << "HEIGHT 1\n";
            finalOut << "VIEWPOINT 0 0 0 1 0 0 0\n";
            finalOut << "POINTS " << pointCount << "\n";
            finalOut << "DATA ascii\n";
            finalOut.flush();

            if (!pcdTmpFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
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
            outFile.seek(107);
            QByteArray b4(4, 0);
            qToLittleEndian<quint32>(countU32, reinterpret_cast<uchar*>(b4.data()));
            outFile.write(b4);
            outFile.seek(111);
            outFile.write(b4);

            if (pointCount == 0) {
                lasMinX = lasMinY = lasMinZ = 0.0;
                lasMaxX = lasMaxY = lasMaxZ = 0.0;
            }
            auto writeDoubleAt = [&outFile](qint64 pos, double v) {
                QByteArray b(8, 0);
                qToLittleEndian<double>(v, reinterpret_cast<uchar*>(b.data()));
                outFile.seek(pos);
                outFile.write(b);
            };
            writeDoubleAt(179, lasMaxX);
            writeDoubleAt(187, lasMinX);
            writeDoubleAt(195, lasMaxY);
            writeDoubleAt(203, lasMinY);
            writeDoubleAt(211, lasMaxZ);
            writeDoubleAt(219, lasMinZ);
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
        QVector<Point3D> points;
        const int rawBegin = g * frameStep;
        const int rawEnd = std::min(rawBegin + frameStep, int(rawFrames.size()));
        for (int r = rawBegin; r < rawEnd; ++r) {
            const Lvx2PlaybackFrameIndex& frameIndex = rawFrames.at(r);
            qint64 cursor = qint64(frameIndex.offset) + qint64(sizeof(LVX2FrameHeader));
            while (cursor + qint64(sizeof(LVX2PackageHeader)) <= qint64(frameIndex.nextOffset)) {
                if (!file.seek(cursor)) {
                    return false;
                }
                LVX2PackageHeader packageHeader{};
                if (!readExact(file, reinterpret_cast<char*>(&packageHeader), sizeof(packageHeader))) {
                    return false;
                }
                cursor += qint64(sizeof(LVX2PackageHeader));
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
                appendPackagePoints(packageHeader, payload, extrinsic, points);
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

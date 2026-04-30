#include "mainwindow.h"

#include <QDir>
#include <QFileInfo>
#include <QMessageBox>
#include <QStandardPaths>
#include <QtEndian>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <thread>

namespace {

struct Lvx2LoadResult {
    bool ok = false;
    QString errorMessage;
    QString filePath;
    QVector<MainWindow::Lvx2PlaybackFrameIndex> rawFrames;
    QMap<uint32_t, MainWindow::Lvx2PlaybackExtrinsic> extrinsics;
};

static bool readExact(QFile& file, char* data, qint64 size) {
    return file.read(data, size) == size;
}

static uint64_t parseTimestampValue(uint64_t raw) {
    // 假设 LVX2 文件中时间戳为大端字节序，按需可改为 qFromLittleEndian
    return qFromBigEndian(raw);
}

static MainWindow::Lvx2PlaybackExtrinsic makeExtrinsic(const LVX2DeviceInfo& info) {
    MainWindow::Lvx2PlaybackExtrinsic extrinsic;
    extrinsic.enabled = (info.extrinsic_enable != 0);
    extrinsic.transform.setToIdentity();
    if (!extrinsic.enabled) {
        return extrinsic;
    }

    extrinsic.transform.translate(info.x, info.y, info.z);
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

// 安全读取点云结构体（避免非对齐访问）
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

static int rawFramesPerPlaybackFrame(uint64_t frameIntervalMs) {
    return std::max(1, int((frameIntervalMs + 49ULL) / 50ULL));
}

} // namespace

bool MainWindow::loadLvx2PlaybackFile(const QString& filePath)
{
    closeLvx2Playback(false);

    lvx2PlaybackLoading = true;
    lvx2PlaybackPath = filePath;
    lvx2PlaybackLoadToken++;
    const quint64 currentToken = lvx2PlaybackLoadToken;

    setLvx2PlaybackPlaying(false);
    updateLvx2PlaybackUi();
    if (statusLabelBar) {
        statusLabelBar->setText(QString("正在加载LVX2: %1").arg(QFileInfo(filePath).fileName()));
    }

    std::thread([this, filePath, currentToken]() {
        Lvx2LoadResult result;
        result.filePath = filePath;

        QFile file(filePath);
        if (!file.open(QIODevice::ReadOnly)) {
            result.errorMessage = "无法打开LVX2文件";
        } else {
            LVX2PublicHeader publicHeader{};
            LVX2PrivateHeader privateHeader{};
            if (!readExact(file, reinterpret_cast<char*>(&publicHeader), sizeof(publicHeader)) ||
                !readExact(file, reinterpret_cast<char*>(&privateHeader), sizeof(privateHeader))) {
                result.errorMessage = "LVX2文件头读取失败";
            } else if (std::memcmp(publicHeader.signature, "livox_tech", 10) != 0 ||
                       publicHeader.magic_code != 0xAC0EA767) {
                result.errorMessage = "不是有效的LVX2文件";
            } else {
                const int deviceCount = std::max(0, int(privateHeader.device_count));
                for (int i = 0; i < deviceCount; ++i) {
                    LVX2DeviceInfo deviceInfo{};
                    if (!readExact(file, reinterpret_cast<char*>(&deviceInfo), sizeof(deviceInfo))) {
                        result.errorMessage = "LVX2设备信息读取失败";
                        break;
                    }
                    result.extrinsics.insert(deviceInfo.lidar_id, makeExtrinsic(deviceInfo));
                }

                if (result.errorMessage.isEmpty()) {
                    const qint64 fileSize = file.size();
                    while (file.pos() + qint64(sizeof(LVX2FrameHeader)) <= fileSize) {
                        LVX2FrameHeader frameHeader{};
                        const qint64 frameOffset = file.pos();
                        if (!readExact(file, reinterpret_cast<char*>(&frameHeader), sizeof(frameHeader))) {
                            break;
                        }
                        if (frameHeader.current_offset != uint64_t(frameOffset) ||
                            frameHeader.next_offset <= frameHeader.current_offset ||
                            frameHeader.next_offset > uint64_t(fileSize)) {
                            break;
                        }

                        result.rawFrames.push_back({frameHeader.current_offset, frameHeader.next_offset, frameHeader.frame_index});
                        if (!file.seek(qint64(frameHeader.next_offset))) {
                            break;
                        }
                    }

                    if (result.rawFrames.isEmpty()) {
                        result.errorMessage = "LVX2文件中未找到可播放帧";
                    } else {
                        result.ok = true;
                    }
                }
            }
        }

        QMetaObject::invokeMethod(this, [this, currentToken, result]() {
            if (currentToken != lvx2PlaybackLoadToken) {
                return;
            }

            lvx2PlaybackLoading = false;
            if (!result.ok) {
                lvx2PlaybackPath.clear();
                updateLvx2PlaybackUi();
                if (statusLabelBar) {
                    statusLabelBar->setText(sdk_started ? "已连接 - 采样中" : "就绪");
                }
                QMessageBox::warning(this, "播放LVX2点云", result.errorMessage);
                return;
            }

            lvx2PlaybackFile.setFileName(result.filePath);
            if (!lvx2PlaybackFile.open(QIODevice::ReadOnly)) {
                lvx2PlaybackPath.clear();
                updateLvx2PlaybackUi();
                if (statusLabelBar) {
                    statusLabelBar->setText(sdk_started ? "已连接 - 采样中" : "就绪");
                }
                QMessageBox::warning(this, "播放LVX2点云", "无法打开LVX2文件进行播放");
                return;
            }

            lvx2RawFrames = result.rawFrames;
            lvx2PlaybackExtrinsics = result.extrinsics;
            lvx2PlaybackPath = result.filePath;
            lvx2PlaybackActive = true;
            lvx2PlaybackFrameCount = (lvx2RawFrames.size() + rawFramesPerPlaybackFrame(frameIntervalMs) - 1) /
                                     rawFramesPerPlaybackFrame(frameIntervalMs);
            lvx2PlaybackFrame = -1;

            {
                QMutexLocker locker(&frameMutex);
                pendingFrames.clear();
                lastSeenTimestamp.clear();
            }

            setLvx2PlaybackPlaying(false);
            updateLvx2PlaybackUi();
            showLvx2PlaybackFrame(0);

            const QString fileName = QFileInfo(result.filePath).fileName();
            if (statusLabelBar) {
                statusLabelBar->setText(QString("LVX2播放: %1").arg(fileName));
            }
            logMessage(QString("已加载LVX2文件: %1").arg(QDir::toNativeSeparators(result.filePath)));
        }, Qt::QueuedConnection);
    }).detach();

    return true;
}

void MainWindow::closeLvx2Playback(bool clearView)
{
    setLvx2PlaybackPlaying(false);

    lvx2PlaybackLoadToken++;
    lvx2PlaybackActive = false;
    lvx2PlaybackLoading = false;
    lvx2PlaybackFrame = -1;
    lvx2PlaybackFrameCount = 0;
    lvx2RawFrames.clear();
    lvx2PlaybackExtrinsics.clear();
    lvx2PlaybackPath.clear();

    if (lvx2PlaybackFile.isOpen()) {
        lvx2PlaybackFile.close();
    }

    updateLvx2PlaybackUi();
    if (clearView && pointCloudWidget) {
        pointCloudWidget->clearPointCloud();
    }

    if (statusLabelBar) {
        statusLabelBar->setText(sdk_started ? "已连接 - 采样中" : "就绪");
    }
}

void MainWindow::showLvx2PlaybackFrame(int playbackFrameIndex)
{
    if (!lvx2PlaybackActive || !lvx2PlaybackFile.isOpen() || lvx2PlaybackFrameCount <= 0) {
        return;
    }

    playbackFrameIndex = std::clamp(playbackFrameIndex, 0, lvx2PlaybackFrameCount - 1);
    const int rawFrameCount = rawFramesPerPlaybackFrame(frameIntervalMs);
    const int rawStartIndex = playbackFrameIndex * rawFrameCount;
    const int rawEndIndex = std::min(rawStartIndex + rawFrameCount, static_cast<int>(lvx2RawFrames.size()));
    if (rawStartIndex >= rawEndIndex) {
        return;
    }

    PointCloudFrame frame;
    frame.device_handle = 0;
    frame.timestamp = 0;

    for (int i = rawStartIndex; i < rawEndIndex; ++i) {
        const Lvx2PlaybackFrameIndex& frameIndex = lvx2RawFrames.at(i);
        qint64 cursor = qint64(frameIndex.offset) + qint64(sizeof(LVX2FrameHeader));
        while (cursor + qint64(sizeof(LVX2PackageHeader)) <= qint64(frameIndex.nextOffset)) {
            if (!lvx2PlaybackFile.seek(cursor)) {
                break;
            }

            LVX2PackageHeader packageHeader{};
            if (!readExact(lvx2PlaybackFile, reinterpret_cast<char*>(&packageHeader), sizeof(packageHeader))) {
                break;
            }
            cursor += qint64(sizeof(LVX2PackageHeader));

            const qint64 dataLength = qint64(packageHeader.data_length);
            if (dataLength < 0 || cursor + dataLength > qint64(frameIndex.nextOffset)) {
                break;
            }

            const QByteArray payload = lvx2PlaybackFile.read(dataLength);
            if (payload.size() != dataLength) {
                break;
            }
            cursor += dataLength;

            frame.timestamp = std::max(frame.timestamp, parseTimestampValue(packageHeader.timestamp));
            const auto extrinsicIt = lvx2PlaybackExtrinsics.constFind(packageHeader.lidar_id);
            const Lvx2PlaybackExtrinsic* extrinsic =
                (extrinsicIt == lvx2PlaybackExtrinsics.constEnd()) ? nullptr : &extrinsicIt.value();
            appendPackagePoints(packageHeader, payload, extrinsic, frame.points);
        }
    }

    applyRenderingPipeline(frame);
    publishPointCloudFrame(frame);

    lvx2PlaybackFrame = playbackFrameIndex;
    updateLvx2PlaybackUi();
}

void MainWindow::updateLvx2PlaybackUi()
{
    if (!lvx2PlaybackBar) {
        return;
    }

    lvx2PlaybackBar->setVisible(lvx2PlaybackActive || lvx2PlaybackLoading);
    if (!lvx2PlaybackActive && !lvx2PlaybackLoading) {
        return;
    }

    if (lvx2PlayPauseButton) {
        lvx2PlayPauseButton->setText(lvx2PlaybackPlaying ? "暂停" : "播放");
        lvx2PlayPauseButton->setEnabled(lvx2PlaybackActive && !lvx2PlaybackLoading);
    }
    if (lvx2PrevFrameButton) {
        lvx2PrevFrameButton->setEnabled(lvx2PlaybackActive && !lvx2PlaybackLoading && lvx2PlaybackFrame > 0);
    }
    if (lvx2NextFrameButton) {
        lvx2NextFrameButton->setEnabled(lvx2PlaybackActive && !lvx2PlaybackLoading &&
                                        lvx2PlaybackFrame >= 0 && lvx2PlaybackFrame < lvx2PlaybackFrameCount - 1);
    }
    if (lvx2ProgressSlider) {
        lvx2UpdatingSlider = true;
        lvx2ProgressSlider->setRange(0, std::max(0, lvx2PlaybackFrameCount - 1));
        lvx2ProgressSlider->setEnabled(lvx2PlaybackActive && !lvx2PlaybackLoading && lvx2PlaybackFrameCount > 0);
        lvx2ProgressSlider->setValue(std::max(0, lvx2PlaybackFrame));
        lvx2UpdatingSlider = false;
    }
    if (lvx2CloseButton) {
        lvx2CloseButton->setEnabled(!lvx2PlaybackLoading);
    }
    if (lvx2PlaybackLabel) {
        const QString name = lvx2PlaybackPath.isEmpty() ? QString() : QFileInfo(lvx2PlaybackPath).fileName();
        if (lvx2PlaybackLoading) {
            lvx2PlaybackLabel->setText(QString("%1  加载中...").arg(name));
        } else {
            lvx2PlaybackLabel->setText(QString("%1  %2/%3")
                                       .arg(name)
                                       .arg(std::max(0, lvx2PlaybackFrame) + 1)
                                       .arg(std::max(1, lvx2PlaybackFrameCount)));
        }
    }
}

void MainWindow::setLvx2PlaybackPlaying(bool playing)
{
    lvx2PlaybackPlaying = playing && lvx2PlaybackActive && !lvx2PlaybackLoading && lvx2PlaybackFrameCount > 0;
    if (lvx2PlaybackTimer) {
        if (lvx2PlaybackPlaying) {
            // 根据倍速调整定时器间隔，最小 1ms
            const int interval = std::max(1, static_cast<int>(frameIntervalMs / lvx2PlaybackSpeed));
            lvx2PlaybackTimer->start(interval);
        } else {
            lvx2PlaybackTimer->stop();
        }
    }
    updateLvx2PlaybackUi();
}

void MainWindow::onLvx2PlaybackTick()
{
    if (!lvx2PlaybackActive) {
        setLvx2PlaybackPlaying(false);
        return;
    }

    const int nextFrame = lvx2PlaybackFrame + 1;
    if (nextFrame >= lvx2PlaybackFrameCount) {
        setLvx2PlaybackPlaying(false);
        return;
    }
    showLvx2PlaybackFrame(nextFrame);
}

void MainWindow::onLvx2PlaybackSliderMoved(int value)
{
    if (lvx2UpdatingSlider || !lvx2PlaybackActive) {
        return;
    }
    setLvx2PlaybackPlaying(false);
    showLvx2PlaybackFrame(value);
}

void MainWindow::onLvx2PlaybackFileDropped(const QString& filePath)
{
    if (filePath.isEmpty()) {
        return;
    }
    loadLvx2PlaybackFile(filePath);
}
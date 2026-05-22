#include "LivoxViewerWindow.h"
#include "Pcap/PushMsgParser.h"

#include <QDir>
#include <QFileInfo>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QStandardPaths>
#include <QtEndian>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace {

struct Lvx2LoadResult {
    bool ok = false;
    QString errorMessage;
    QString filePath;
    QVector<LivoxViewerWindow::Lvx2PlaybackFrameIndex> rawFrames;
    QMap<uint32_t, LivoxViewerWindow::Lvx2PlaybackExtrinsic> extrinsics;
    QVector<Lvx2DeviceInfo> lidarDevices;
};

static bool readExact(QFile& file, char* data, qint64 size) {
    return file.read(data, size) == size;
}

static uint64_t parseTimestampValue(uint64_t raw) {
    // 假设 LVX2 文件中时间戳为大端字节序，按需可改为 qFromLittleEndian
    return qFromBigEndian(raw);
}

static LivoxViewerWindow::Lvx2PlaybackExtrinsic makeExtrinsic(const Lvx2DeviceInfo& info) {
    LivoxViewerWindow::Lvx2PlaybackExtrinsic extrinsic;
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

static void applyExtrinsicTransform(const LivoxViewerWindow::Lvx2PlaybackExtrinsic* extrinsic, PointCloudPoint& point) {
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

static void appendPackagePoints(const Lvx2PackageHeader& header,
                                const QByteArray& payload,
                                const LivoxViewerWindow::Lvx2PlaybackExtrinsic* extrinsic,
                                QVector<PointCloudPoint>& points) {
    if (payload.isEmpty()) {
        return;
    }

    const char* rawData = payload.constData();

    switch (header.data_type) {
    case kLivoxLidarCartesianCoordinateHighData: {
        const int pointCount = int(payload.size() / int(sizeof(LivoxLidarCartesianHighRawPoint)));
        for (int i = 0; i < pointCount; ++i) {
            auto raw = safeReadPoint<LivoxLidarCartesianHighRawPoint>(rawData, i);
            PointCloudPoint point{};
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
            PointCloudPoint point{};
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
            PointCloudPoint point{};
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
            PointCloudPoint p1{};
            p1.x = raw.x1 / 1000.0f;
            p1.y = raw.y1 / 1000.0f;
            p1.z = raw.z1 / 1000.0f;
            p1.reflectivity = raw.reflectivity1;
            p1.tag = raw.tag1;
            applyExtrinsicTransform(extrinsic, p1);
            points.push_back(p1);

            PointCloudPoint p2{};
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

static QVector<PointCloudPoint> collectVisiblePcapPoints(
    const PointCloudFrame& raw,
    const QMap<uint32_t, bool>& visibility,
    const QMap<uint32_t, LivoxViewerWindow::Lvx2PlaybackExtrinsic>& extrinsics)
{
    QVector<PointCloudPoint> merged;
    if (!raw.pointsByLidar.isEmpty()) {
        for (auto it = raw.pointsByLidar.constBegin(); it != raw.pointsByLidar.constEnd(); ++it) {
            if (!visibility.value(it.key(), true)) {
                continue;
            }
            QVector<PointCloudPoint> segment = it.value();
            const auto extrinsicIt = extrinsics.constFind(it.key());
            const LivoxViewerWindow::Lvx2PlaybackExtrinsic* extrinsic =
                (extrinsicIt == extrinsics.constEnd()) ? nullptr : &extrinsicIt.value();
            if (extrinsic && extrinsic->enabled) {
                for (PointCloudPoint& point : segment) {
                    applyExtrinsicTransform(extrinsic, point);
                }
            }
            merged += segment;
        }
        return merged;
    }
    return raw.points;
}

} // namespace

bool LivoxViewerWindow::loadLvx2PlaybackFile(const QString& filePath)
{
    closeLvx2Playback(false);

    pcapPlaybackFromFile = false;
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
            Lvx2PublicHeader publicHeader{};
            Lvx2PrivateHeader privateHeader{};
            if (!readExact(file, reinterpret_cast<char*>(&publicHeader), sizeof(publicHeader)) ||
                !readExact(file, reinterpret_cast<char*>(&privateHeader), sizeof(privateHeader))) {
                result.errorMessage = "LVX2文件头读取失败";
            } else if (std::memcmp(publicHeader.signature, "livox_tech", 10) != 0 ||
                       publicHeader.magic_code != 0xAC0EA767) {
                result.errorMessage = "不是有效的LVX2文件";
            } else {
                const int deviceCount = std::max(0, int(privateHeader.device_count));
                for (int i = 0; i < deviceCount; ++i) {
                    Lvx2DeviceInfo deviceInfo{};
                    if (!readExact(file, reinterpret_cast<char*>(&deviceInfo), sizeof(deviceInfo))) {
                        result.errorMessage = "LVX2设备信息读取失败";
                        break;
                    }
                    result.extrinsics.insert(deviceInfo.lidar_id, makeExtrinsic(deviceInfo));
                    result.lidarDevices.push_back(deviceInfo);
                }

                if (result.errorMessage.isEmpty()) {
                    const qint64 fileSize = file.size();
                    while (file.pos() + qint64(sizeof(Lvx2FrameHeader)) <= fileSize) {
                        Lvx2FrameHeader frameHeader{};
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
            lvx2PlaybackDevices.clear();
            lvx2PlaybackDeviceVisible.clear();
            for (const Lvx2DeviceInfo& deviceInfo : result.lidarDevices) {
                Lvx2PlaybackDeviceInfoUi uiInfo;
                uiInfo.lidarId = deviceInfo.lidar_id;
                uiInfo.deviceType = deviceInfo.device_type;
                uiInfo.lidarSn = QString::fromLatin1(deviceInfo.lidar_sn).trimmed();
                lvx2PlaybackDevices.push_back(uiInfo);
                lvx2PlaybackDeviceVisible.insert(deviceInfo.lidar_id, true);
            }
            rebuildLvx2DeviceTab();
            lvx2RawFrameCache.resize(lvx2RawFrames.size());
            lvx2RawFrameCacheValid = QVector<bool>(lvx2RawFrames.size(), false);
            lvx2SlidingWindowStart = -1;
            lvx2SlidingWindowEnd = -1;
            lvx2SlidingWindowPoints.clear();
            lvx2SlidingWindowSegmentPointCounts.clear();
            lvx2SlidingWindowTimestamp = 0;
            lvx2PlaybackPath = result.filePath;
            lvx2PlaybackActive = true;
            if (lvx2PlaybackMode == Lvx2PlaybackMode::SlidingWindow) {
                lvx2PlaybackFrameCount = static_cast<int>(lvx2RawFrames.size());
            } else {
                lvx2PlaybackFrameCount = (lvx2RawFrames.size() + rawFramesPerPlaybackFrame(frameIntervalMs) - 1) /
                                         rawFramesPerPlaybackFrame(frameIntervalMs);
            }
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

void LivoxViewerWindow::closeLvx2Playback(bool clearView)
{
    setLvx2PlaybackPlaying(false);

    lvx2PlaybackLoadToken++;
    pcapPlaybackFromFile = false;
    lvx2PlaybackActive = false;
    lvx2PlaybackLoading = false;
    lvx2PlaybackFrame = -1;
    lvx2PlaybackFrameCount = 0;
    lvx2RawFrames.clear();
    lvx2PlaybackExtrinsics.clear();
    lvx2PlaybackDevices.clear();
    lvx2PlaybackDeviceVisible.clear();
    rebuildLvx2DeviceTab();
    lvx2RawFrameCache.clear();
    lvx2RawFrameCacheValid.clear();
    lvx2SlidingWindowStart = -1;
    lvx2SlidingWindowEnd = -1;
    lvx2SlidingWindowPoints.clear();
    lvx2SlidingWindowSegmentPointCounts.clear();
    lvx2SlidingWindowTimestamp = 0;
    lvx2PlaybackPath.clear();

    if (lvx2PlaybackFile.isOpen()) {
        lvx2PlaybackFile.close();
    }

    updateLvx2PlaybackUi();
    if (clearView && pointCloudView) {
        pointCloudView->clearPointCloud();
    }

    if (statusLabelBar) {
        statusLabelBar->setText(sdk_started ? "已连接 - 采样中" : "就绪");
    }
}

void LivoxViewerWindow::showLvx2PlaybackFrame(int playbackFrameIndex)
{
    if (!lvx2PlaybackActive || lvx2PlaybackFrameCount <= 0) {
        return;
    }
    if (!pcapPlaybackFromFile && !lvx2PlaybackFile.isOpen()) {
        return;
    }

    playbackFrameIndex = std::clamp(playbackFrameIndex, 0, lvx2PlaybackFrameCount - 1);
    const int rawFrameCount = rawFramesPerPlaybackFrame(frameIntervalMs);
    int rawStartIndex = 0;
    int rawEndIndex = 0;
    if (lvx2PlaybackMode == Lvx2PlaybackMode::SlidingWindow) {
        rawEndIndex = std::min(playbackFrameIndex + 1, static_cast<int>(lvx2RawFrames.size()));
        rawStartIndex = std::max(0, rawEndIndex - rawFrameCount);
    } else {
        rawStartIndex = playbackFrameIndex * rawFrameCount;
        rawEndIndex = std::min(rawStartIndex + rawFrameCount, static_cast<int>(lvx2RawFrames.size()));
    }
    if (rawStartIndex >= rawEndIndex) {
        return;
    }

    auto parseRawFrame = [this](int rawIndex) -> const PointCloudFrame& {
        if (rawIndex < 0 || rawIndex >= lvx2RawFrames.size()) {
            throw std::out_of_range("rawIndex out of range");
        }
        if (pcapPlaybackFromFile) {
            if (rawIndex < 0 || rawIndex >= lvx2RawFrameCache.size()) {
                throw std::out_of_range("pcap rawIndex out of range");
            }
            // Pcap 帧缓存在内存中只读复用，不按 LVX2 路径做失效重解析。
            return lvx2RawFrameCache[rawIndex];
        }
        if (lvx2RawFrameCacheValid.value(rawIndex, false)) {
            return lvx2RawFrameCache[rawIndex];
        }
        PointCloudFrame parsed;
        parsed.device_handle = 0;
        parsed.timestamp = 0;
        const Lvx2PlaybackFrameIndex& frameIndex = lvx2RawFrames.at(rawIndex);
        qint64 cursor = qint64(frameIndex.offset) + qint64(sizeof(Lvx2FrameHeader));
        while (cursor + qint64(sizeof(Lvx2PackageHeader)) <= qint64(frameIndex.nextOffset)) {
            if (!lvx2PlaybackFile.seek(cursor)) {
                break;
            }
            Lvx2PackageHeader packageHeader{};
            if (!readExact(lvx2PlaybackFile, reinterpret_cast<char*>(&packageHeader), sizeof(packageHeader))) {
                break;
            }
            cursor += qint64(sizeof(Lvx2PackageHeader));
            const qint64 dataLength = qint64(packageHeader.data_length);
            if (dataLength < 0 || cursor + dataLength > qint64(frameIndex.nextOffset)) {
                break;
            }
            const QByteArray payload = lvx2PlaybackFile.read(dataLength);
            if (payload.size() != dataLength) {
                break;
            }
            cursor += dataLength;
            parsed.timestamp = std::max(parsed.timestamp, parseTimestampValue(packageHeader.timestamp));
            const auto extrinsicIt = lvx2PlaybackExtrinsics.constFind(packageHeader.lidar_id);
            const Lvx2PlaybackExtrinsic* extrinsic =
                (extrinsicIt == lvx2PlaybackExtrinsics.constEnd()) ? nullptr : &extrinsicIt.value();
            if (!lvx2PlaybackDeviceVisible.value(packageHeader.lidar_id, true)) {
                continue;
            }
            appendPackagePoints(packageHeader, payload, extrinsic, parsed.points);
        }
        lvx2RawFrameCache[rawIndex] = std::move(parsed);
        lvx2RawFrameCacheValid[rawIndex] = true;
        return lvx2RawFrameCache[rawIndex];
    };

    auto collectRawFramePoints = [this, &parseRawFrame](int rawIndex) -> QVector<PointCloudPoint> {
        const PointCloudFrame& rawFrame = parseRawFrame(rawIndex);
        if (pcapPlaybackFromFile) {
            return collectVisiblePcapPoints(rawFrame, lvx2PlaybackDeviceVisible, lvx2PlaybackExtrinsics);
        }
        return rawFrame.points;
    };

    if (lvx2PlaybackMode == Lvx2PlaybackMode::SlidingWindow) {
        const bool canIncrementalAdvance =
            (lvx2SlidingWindowStart >= 0 && lvx2SlidingWindowEnd >= 0 &&
             rawStartIndex >= lvx2SlidingWindowStart && rawEndIndex >= lvx2SlidingWindowEnd &&
             rawStartIndex - lvx2SlidingWindowStart <= 1 && rawEndIndex - lvx2SlidingWindowEnd <= 1);

        if (!canIncrementalAdvance) {
            lvx2SlidingWindowPoints.clear();
            lvx2SlidingWindowSegmentPointCounts.clear();
            for (int i = rawStartIndex; i < rawEndIndex; ++i) {
                const QVector<PointCloudPoint> rawPoints = collectRawFramePoints(i);
                lvx2SlidingWindowPoints += rawPoints;
                lvx2SlidingWindowSegmentPointCounts.push_back(rawPoints.size());
            }
        } else {
            if (rawStartIndex > lvx2SlidingWindowStart && !lvx2SlidingWindowSegmentPointCounts.isEmpty()) {
                const int removeCount = lvx2SlidingWindowSegmentPointCounts.front();
                if (removeCount > 0) {
                    lvx2SlidingWindowPoints.remove(0, removeCount);
                }
                lvx2SlidingWindowSegmentPointCounts.remove(0);
            }
            if (rawEndIndex > lvx2SlidingWindowEnd) {
                const QVector<PointCloudPoint> addedPoints = collectRawFramePoints(rawEndIndex - 1);
                lvx2SlidingWindowPoints += addedPoints;
                lvx2SlidingWindowSegmentPointCounts.push_back(addedPoints.size());
            }
        }
        lvx2SlidingWindowStart = rawStartIndex;
        lvx2SlidingWindowEnd = rawEndIndex;
        lvx2SlidingWindowTimestamp = 0;
        for (int i = lvx2SlidingWindowStart; i < lvx2SlidingWindowEnd; ++i) {
            lvx2SlidingWindowTimestamp = std::max(lvx2SlidingWindowTimestamp, parseRawFrame(i).timestamp);
        }
    } else {
        lvx2SlidingWindowStart = rawStartIndex;
        lvx2SlidingWindowEnd = rawEndIndex;
        lvx2SlidingWindowPoints.clear();
        lvx2SlidingWindowSegmentPointCounts.clear();
        lvx2SlidingWindowTimestamp = 0;
        for (int i = rawStartIndex; i < rawEndIndex; ++i) {
            const PointCloudFrame& rawFrame = parseRawFrame(i);
            lvx2SlidingWindowPoints += collectRawFramePoints(i);
            lvx2SlidingWindowTimestamp = std::max(lvx2SlidingWindowTimestamp, rawFrame.timestamp);
        }
    }

    PointCloudFrame frame;
    frame.device_handle = 0;
    frame.timestamp = lvx2SlidingWindowTimestamp;
    frame.points = lvx2SlidingWindowPoints;

    applyPointCloudPipeline(frame);
    presentPointCloudFrame(frame);

    lvx2PlaybackFrame = playbackFrameIndex;
    updateLvx2PlaybackUi();
}

QString LivoxViewerWindow::lvx2DeviceTypeToModel(uint8_t deviceType) const
{
    switch (deviceType) {
    case kLivoxLidarTypeMid40: return "Mid40";
    case kLivoxLidarTypeMid70: return "Mid70";
    case kLivoxLidarTypeMid360: return "Mid360";
    case kLivoxLidarTypeMid360s: return "Mid360s";
    case kLivoxLidarTypeHorizon: return "Horizon";
    case kLivoxLidarTypeAvia: return "Avia";
    case kLivoxLidarTypeAvia2: return "Avia2";
    case kLivoxLidarTypeTele: return "Tele";
    case kLivoxLidarTypeHAP: return "HAP";
    case kLivoxLidarTypePA: return "PA";
    default: return QString("Unknown(%1)").arg(deviceType);
    }
}

void LivoxViewerWindow::rebuildLvx2DeviceTab()
{
    if (!lvx2DeviceTable) {
        return;
    }
    QSignalBlocker blocker(lvx2DeviceTable);
    lvx2DeviceTable->clearContents();
    lvx2DeviceTable->setRowCount(lvx2PlaybackDevices.size());
    for (int row = 0; row < lvx2PlaybackDevices.size(); ++row) {
        const auto& info = lvx2PlaybackDevices[row];
        auto* visibleItem = new QTableWidgetItem();
        visibleItem->setFlags((visibleItem->flags() | Qt::ItemIsUserCheckable) & ~Qt::ItemIsEditable);
        const bool visible = lvx2PlaybackDeviceVisible.value(info.lidarId, true);
        visibleItem->setCheckState(visible ? Qt::Checked : Qt::Unchecked);
        visibleItem->setData(Qt::UserRole, static_cast<qulonglong>(info.lidarId));
        lvx2DeviceTable->setItem(row, 0, visibleItem);
        const QString modelName =
            info.modelDisplay.isEmpty() ? lvx2DeviceTypeToModel(info.deviceType) : info.modelDisplay;
        lvx2DeviceTable->setItem(row, 1, new QTableWidgetItem(modelName));
        lvx2DeviceTable->setItem(row, 2, new QTableWidgetItem(info.lidarSn));
        lvx2DeviceTable->setItem(row, 3, new QTableWidgetItem(PushMsgParser::lidarIdToIpString(info.lidarId)));
    }

    disconnect(lvx2DeviceTable, &QTableWidget::itemChanged, this, nullptr);
    connect(lvx2DeviceTable, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item) {
        if (!item || item->column() != 0) {
            return;
        }
        const uint32_t lidarId = static_cast<uint32_t>(item->data(Qt::UserRole).toULongLong());
        lvx2PlaybackDeviceVisible[lidarId] = (item->checkState() == Qt::Checked);
        if (!pcapPlaybackFromFile) {
            lvx2RawFrameCacheValid.fill(false);
        }
        lvx2SlidingWindowStart = -1;
        lvx2SlidingWindowEnd = -1;
        lvx2SlidingWindowPoints.clear();
        lvx2SlidingWindowSegmentPointCounts.clear();
        if (lvx2PlaybackActive && lvx2PlaybackFrame >= 0) {
            showLvx2PlaybackFrame(lvx2PlaybackFrame);
        }
    });
}

void LivoxViewerWindow::updateLvx2PlaybackUi()
{
    if (lvx2FileDock) {
        const bool showDock = lvx2PlaybackActive || lvx2PlaybackLoading;
        lvx2FileDock->setVisible(showDock);
        if (showDock) {
            lvx2FileDock->raise();
        }
    }

    if (!lvx2PlaybackBar) {
        return;
    }

    lvx2PlaybackBar->setVisible(lvx2PlaybackActive || lvx2PlaybackLoading);
    if (!lvx2PlaybackActive && !lvx2PlaybackLoading) {
        return;
    }

    if (lvx2PlayPauseButton) {
        QIcon icon = QIcon::fromTheme(lvx2PlaybackPlaying ? "media-playback-pause" : "media-playback-start");
        if (icon.isNull()) {
            icon = style()->standardIcon(lvx2PlaybackPlaying ? QStyle::SP_MediaPause : QStyle::SP_MediaPlay);
        }
        lvx2PlayPauseButton->setIcon(icon);
        lvx2PlayPauseButton->setToolTip(lvx2PlaybackPlaying ? "暂停" : "播放");
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
    if (lvx2PlaybackModeCombo) {
        lvx2PlaybackModeCombo->setEnabled(lvx2PlaybackActive && !lvx2PlaybackLoading);
        lvx2PlaybackModeCombo->setCurrentIndex(lvx2PlaybackMode == Lvx2PlaybackMode::SlidingWindow ? 1 : 0);
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

void LivoxViewerWindow::setLvx2PlaybackPlaying(bool playing)
{
    lvx2PlaybackPlaying = playing && lvx2PlaybackActive && !lvx2PlaybackLoading && lvx2PlaybackFrameCount > 0;
    
    if (lvx2PlaybackTimer) {
        if (lvx2PlaybackPlaying) {
            // 核心逻辑：确定基础步长
            // 离散模式下，一帧代表 frameIntervalMs 的数据
            // 滑动窗口模式下，由于步进是 1 个原始帧，其真实时间约为 50ms
            double baseStepMs = (lvx2PlaybackMode == Lvx2PlaybackMode::SlidingWindow) ? 50.0 : static_cast<double>(frameIntervalMs);
            
            // 根据倍速调整定时器触发频率：频率 = 基础步长 / 倍速
            // 例如：2倍速下，50ms 的数据需要在 25ms 内播放完
            int interval = std::max(1, static_cast<int>(baseStepMs / lvx2PlaybackSpeed));
            
            lvx2PlaybackTimer->start(interval);
        } else {
            lvx2PlaybackTimer->stop();
        }
    }
    updateLvx2PlaybackUi();
}

void LivoxViewerWindow::onLvx2PlaybackTick()
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

void LivoxViewerWindow::onLvx2PlaybackSliderMoved(int value)
{
    if (lvx2UpdatingSlider || !lvx2PlaybackActive) {
        return;
    }
    setLvx2PlaybackPlaying(false);
    showLvx2PlaybackFrame(value);
}

void LivoxViewerWindow::onLvx2PlaybackFileDropped(const QString& filePath)
{
    if (filePath.isEmpty()) {
        return;
    }
    if (filePath.endsWith(QStringLiteral(".pcap"), Qt::CaseInsensitive) ||
        filePath.endsWith(QStringLiteral(".pcapng"), Qt::CaseInsensitive) ||
        filePath.endsWith(QStringLiteral(".cap"), Qt::CaseInsensitive)) {
        loadPcapPlaybackFile(filePath);
        return;
    }
    loadLvx2PlaybackFile(filePath);
}

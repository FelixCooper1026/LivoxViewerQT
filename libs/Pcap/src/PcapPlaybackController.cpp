#include "LivoxViewerWindow.h"
#include "Pcap/PcapParser.h"
#include "Pcap/PushMsgParser.h"

#include <QDir>
#include <QFileInfo>
#include <QMessageBox>
#include <thread>

bool LivoxViewerWindow::loadPcapPlaybackFile(const QString& filePath)
{
    closeLvx2Playback(false);

    lvx2PlaybackLoading = true;
    pcapPlaybackFromFile = false;
    lvx2PlaybackPath = filePath;
    lvx2PlaybackLoadToken++;
    const quint64 currentToken = lvx2PlaybackLoadToken;

    setLvx2PlaybackPlaying(false);
    updateLvx2PlaybackUi();
    if (statusLabelBar) {
        statusLabelBar->setText(QString("正在加载Pcap: %1").arg(QFileInfo(filePath).fileName()));
    }

    std::thread([this, filePath, currentToken]() {
        const PcapParser::ParseResult parseResult = PcapParser::parseFileToFrames(filePath);

        QMetaObject::invokeMethod(this, [this, currentToken, filePath, parseResult]() {
            if (currentToken != lvx2PlaybackLoadToken) {
                return;
            }

            lvx2PlaybackLoading = false;
            if (!parseResult.ok) {
                lvx2PlaybackPath.clear();
                updateLvx2PlaybackUi();
                if (statusLabelBar) {
                    statusLabelBar->setText(sdk_started ? "已连接 - 采样中" : "就绪");
                }
                QMessageBox::warning(this, "播放Pcap文件", parseResult.errorMessage);
                return;
            }

            lvx2RawFrames.clear();
            lvx2RawFrames.reserve(parseResult.frames.size());
            for (int i = 0; i < parseResult.frames.size(); ++i) {
                Lvx2PlaybackFrameIndex index;
                index.offset = uint64_t(i);
                index.nextOffset = uint64_t(i + 1);
                index.frameIndex = uint64_t(i);
                lvx2RawFrames.push_back(index);
            }

            lvx2RawFrameCache = parseResult.frames;
            lvx2RawFrameCacheValid = QVector<bool>(lvx2RawFrameCache.size(), true);
            lvx2SlidingWindowStart = -1;
            lvx2SlidingWindowEnd = -1;
            lvx2SlidingWindowPoints.clear();
            lvx2SlidingWindowSegmentPointCounts.clear();
            lvx2SlidingWindowTimestamp = 0;

            pcapPlaybackFromFile = true;
            lvx2PlaybackPath = filePath;
            lvx2PlaybackActive = true;

            lvx2PlaybackExtrinsics.clear();
            lvx2PlaybackDevices.clear();
            lvx2PlaybackDeviceVisible.clear();
            for (const PushMsgParser::PushDeviceRecord& device : parseResult.devices) {
                Lvx2PlaybackDeviceInfoUi uiInfo;
                uiInfo.lidarId = device.lidarId;
                uiInfo.deviceType = device.deviceType;
                uiInfo.lidarSn = device.lidarSn;
                uiInfo.modelDisplay = device.modelDisplay;
                lvx2PlaybackDevices.push_back(uiInfo);
                lvx2PlaybackDeviceVisible.insert(device.lidarId, true);

                if (device.hasExtrinsic) {
                    Lvx2PlaybackExtrinsic extrinsic;
                    extrinsic.enabled = true;
                    extrinsic.transform.setToIdentity();
                    extrinsic.transform.translate(device.offsetX, device.offsetY, device.offsetZ);
                    extrinsic.transform.rotate(device.offsetYaw, 0.0f, 0.0f, 1.0f);
                    extrinsic.transform.rotate(device.offsetPitch, 0.0f, 1.0f, 0.0f);
                    extrinsic.transform.rotate(device.offsetRoll, 1.0f, 0.0f, 0.0f);
                    lvx2PlaybackExtrinsics.insert(device.lidarId, extrinsic);
                }
            }
            rebuildLvx2DeviceTab();

            if (lvx2PlaybackMode == Lvx2PlaybackMode::SlidingWindow) {
                lvx2PlaybackFrameCount = static_cast<int>(lvx2RawFrames.size());
            } else {
                const int rawFramesPerStep = std::max(1, int((frameIntervalMs + 49ULL) / 50ULL));
                lvx2PlaybackFrameCount =
                    (lvx2RawFrames.size() + rawFramesPerStep - 1) / rawFramesPerStep;
            }
            if (lvx2PlaybackFrameCount <= 0) {
                lvx2PlaybackFrameCount = 1;
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

            const QString fileName = QFileInfo(filePath).fileName();
            if (statusLabelBar) {
                statusLabelBar->setText(QString("Pcap播放: %1").arg(fileName));
            }
            logMessage(QString("已加载Pcap文件: %1 (共 %2 帧)")
                           .arg(QDir::toNativeSeparators(filePath))
                           .arg(lvx2RawFrames.size()));
        }, Qt::QueuedConnection);
    }).detach();

    return true;
}

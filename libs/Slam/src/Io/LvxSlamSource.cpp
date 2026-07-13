#include "Io/LvxSlamSource.h"

#include "Lvx2Reader.h"
#include "LvxReader.h"
#include "PlaybackSource.h"

#include <QFileInfo>
#include <QMap>
#include <QStringList>

#include <algorithm>
#include <memory>
#include <utility>

namespace {

void flushFrame(QVector<SlamInputFrame>& frames, SlamInputFrame& frame)
{
    if (frame.points.isEmpty()) {
        return;
    }
    frame.sequence = uint64_t(frames.size());
    frame.frameEndNs = frame.frameStartNs + frame.points.last().offsetNs;
    frames.push_back(std::move(frame));
    frame = SlamInputFrame();
}

SlamPoint toSlamPoint(const PointCloudPoint& source, int64_t offsetNs)
{
    SlamPoint point;
    point.x = source.x;
    point.y = source.y;
    point.z = source.z;
    point.reflectivity = source.reflectivity;
    point.tag = source.tag;
    point.line = source.line;
    point.hasLine = true;
    point.offsetNs = offsetNs;
    point.hasOffsetTime = true;
    return point;
}

} // namespace

LvxSlamSource::LvxSlamSource(int frameDurationMs)
    : frameDurationNs_(int64_t(std::max(1, frameDurationMs)) * int64_t{1000000})
{
}

bool LvxSlamSource::load(const QString& filePath, QString* error)
{
    frames_.clear();
    summaryText_.clear();

    const bool lvx2 = QFileInfo(filePath).suffix().compare(QStringLiteral("lvx2"), Qt::CaseInsensitive) == 0;
    std::unique_ptr<Playback::Source> source = lvx2
        ? std::unique_ptr<Playback::Source>(new Lvx2::Lvx2Reader())
        : std::unique_ptr<Playback::Source>(new Lvx::LvxReader());
    if (!source->load(filePath)) {
        if (error != nullptr) {
            *error = source->errorMessage();
        }
        return false;
    }

    const QVector<Playback::DeviceInfo> devices = source->devices();
    if (devices.size() != 1) {
        if (error != nullptr) {
            *error = QStringLiteral("离线 LO 仅支持单雷达 LVX/LVX2 文件。");
        }
        return false;
    }

    QMap<uint32_t, bool> visibility;
    visibility.insert(devices.first().lidarId, true);
    const int64_t sourceFrameDurationNs = int64_t(source->nominalFrameDurationNs());
    SlamInputFrame outputFrame;
    uint64_t pointCount = 0;

    for (int sourceFrameIndex = 0; sourceFrameIndex < source->frameCount(); ++sourceFrameIndex) {
        PointCloudFrame sourceFrame;
        if (!source->readFrame(sourceFrameIndex, visibility, sourceFrame)) {
            if (error != nullptr) {
                *error = source->errorMessage();
            }
            return false;
        }
        if (sourceFrame.points.isEmpty()) {
            continue;
        }

        const int64_t sourceFrameEndNs = int64_t(sourceFrame.timestamp);
        const int64_t sourceFrameStartNs = sourceFrameEndNs - sourceFrameDurationNs;
        if (outputFrame.points.isEmpty()) {
            outputFrame.frameStartNs = sourceFrameStartNs;
            outputFrame.sourceId = devices.first().lidarId;
            outputFrame.deviceType = devices.first().deviceType;
            outputFrame.timeSource = SlamTimeSource::SynthesizedFromPacketInterval;
            outputFrame.hasPointOffsetTime = true;
            outputFrame.hasCompleteImuCoverage = false;
            outputFrame.sourceName = filePath;
        } else if (sourceFrameStartNs >= outputFrame.frameStartNs + frameDurationNs_) {
            flushFrame(frames_, outputFrame);
            outputFrame.frameStartNs = sourceFrameStartNs;
            outputFrame.sourceId = devices.first().lidarId;
            outputFrame.deviceType = devices.first().deviceType;
            outputFrame.timeSource = SlamTimeSource::SynthesizedFromPacketInterval;
            outputFrame.hasPointOffsetTime = true;
            outputFrame.hasCompleteImuCoverage = false;
            outputFrame.sourceName = filePath;
        }

        const int64_t sourceOffsetNs = sourceFrameStartNs - outputFrame.frameStartNs;
        for (qsizetype pointIndex = 0; pointIndex < sourceFrame.points.size(); ++pointIndex) {
            const int64_t pointOffsetNs = sourceOffsetNs +
                int64_t(pointIndex) * sourceFrameDurationNs / int64_t(sourceFrame.points.size());
            outputFrame.points.push_back(toSlamPoint(sourceFrame.points.at(pointIndex), pointOffsetNs));
        }
        pointCount += uint64_t(sourceFrame.points.size());
    }
    flushFrame(frames_, outputFrame);

    if (frames_.isEmpty()) {
        if (error != nullptr) {
            *error = QStringLiteral("LVX/LVX2 未生成有效的离线 LO 点云帧。");
        }
        return false;
    }

    summaryText_ = QStringLiteral(
        "%1 LO 输入摘要\n- 文件: %2\n- 帧数: %3\n- 点数: %4\n- IMU 样本数: 0\n- 聚帧周期: %5 ms\n- 点内时间: 按 LVX 包帧时间和点顺序重建")
                       .arg(lvx2 ? QStringLiteral("LVX2") : QStringLiteral("LVX"),
                            filePath,
                            QString::number(frames_.size()),
                            QString::number(pointCount),
                            QString::number(frameDurationNs_ / int64_t{1000000}));
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

const QVector<SlamInputFrame>& LvxSlamSource::frames() const
{
    return frames_;
}

QString LvxSlamSource::summaryText() const
{
    return summaryText_;
}

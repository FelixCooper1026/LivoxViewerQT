#include "Io/LvxSlamSource.h"

#include "Lvx2Reader.h"
#include "LvxReader.h"
#include "PlaybackSource.h"

#include <QFileInfo>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

namespace {

void initializeOutputFrame(SlamInputFrame& frame,
                           int64_t frameStartNs,
                           uint32_t sourceId,
                           uint8_t deviceType,
                           const QString& filePath)
{
    frame = SlamInputFrame();
    frame.frameStartNs = frameStartNs;
    frame.sourceId = sourceId;
    frame.deviceType = deviceType;
    frame.timeSource = SlamTimeSource::SynthesizedFromPacketInterval;
    frame.hasPointOffsetTime = true;
    frame.hasCompleteImuCoverage = false;
    frame.sourceName = filePath;
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

LvxSlamSource::~LvxSlamSource() = default;

bool LvxSlamSource::open(const QString& filePath,
                         const std::atomic_bool* cancellationRequested,
                         QString* error)
{
    source_.reset();
    visibility_.clear();
    pendingFrame_ = SlamInputFrame();
    filePath_ = filePath;
    sourceFrameIndex_ = 0;
    estimatedFrameCount_ = 0;
    sourceId_ = 0;
    deviceType_ = 0;
    emittedFrameCount_ = 0;
    pointCount_ = 0;

    lvx2_ = QFileInfo(filePath).suffix().compare(QStringLiteral("lvx2"), Qt::CaseInsensitive) == 0;
    source_ = lvx2_
        ? std::unique_ptr<Playback::Source>(new Lvx2::Lvx2Reader(false, cancellationRequested))
        : std::unique_ptr<Playback::Source>(new Lvx::LvxReader(false, cancellationRequested));
    if (!source_->load(filePath)) {
        if (error != nullptr) {
            *error = source_->errorMessage();
        }
        return false;
    }

    const QVector<Playback::DeviceInfo> devices = source_->devices();
    if (devices.size() != 1) {
        if (error != nullptr) {
            *error = QStringLiteral("离线 LO 仅支持单雷达 LVX/LVX2 文件。");
        }
        return false;
    }

    sourceId_ = devices.first().lidarId;
    deviceType_ = devices.first().deviceType;
    visibility_.insert(sourceId_, true);
    sourceFrameDurationNs_ = int64_t(source_->nominalFrameDurationNs());
    const uint64_t sourceDurationNs = uint64_t(std::max(0, source_->frameCount())) *
                                      uint64_t(std::max<int64_t>(1, sourceFrameDurationNs_));
    const uint64_t estimated = (sourceDurationNs + uint64_t(frameDurationNs_) - 1) /
                               uint64_t(frameDurationNs_);
    estimatedFrameCount_ = int(std::min<uint64_t>(
        uint64_t(std::numeric_limits<int>::max()), std::max<uint64_t>(uint64_t{1}, estimated)));
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

bool LvxSlamSource::readNextFrame(SlamInputFrame* frame,
                                  const std::atomic_bool* cancellationRequested,
                                  QString* error)
{
    if (frame == nullptr || !source_) {
        if (error != nullptr) {
            *error = QStringLiteral("LVX/LVX2 流式输入源尚未打开。");
        }
        return false;
    }
    if (error != nullptr) {
        error->clear();
    }

    while (sourceFrameIndex_ < source_->frameCount()) {
        if (cancellationRequested && cancellationRequested->load()) {
            return false;
        }
        PointCloudFrame sourceFrame;
        if (!source_->readFrame(sourceFrameIndex_, visibility_, sourceFrame)) {
            if (error != nullptr) {
                *error = source_->errorMessage();
            }
            return false;
        }
        ++sourceFrameIndex_;
        if (sourceFrame.points.isEmpty()) {
            continue;
        }

        const int64_t sourceFrameEndNs = int64_t(sourceFrame.timestamp);
        const int64_t sourceFrameStartNs = sourceFrameEndNs - sourceFrameDurationNs_;
        SlamInputFrame completedFrame;
        const bool completesPendingFrame = !pendingFrame_.points.isEmpty() &&
            sourceFrameStartNs >= pendingFrame_.frameStartNs + frameDurationNs_;
        if (pendingFrame_.points.isEmpty()) {
            initializeOutputFrame(pendingFrame_, sourceFrameStartNs, sourceId_, deviceType_, filePath_);
        } else if (completesPendingFrame) {
            completedFrame = std::move(pendingFrame_);
            completedFrame.sequence = emittedFrameCount_++;
            completedFrame.frameEndNs = completedFrame.frameStartNs + completedFrame.points.last().offsetNs;
            initializeOutputFrame(pendingFrame_, sourceFrameStartNs, sourceId_, deviceType_, filePath_);
        }

        const int64_t sourceOffsetNs = sourceFrameStartNs - pendingFrame_.frameStartNs;
        for (qsizetype pointIndex = 0; pointIndex < sourceFrame.points.size(); ++pointIndex) {
            if ((pointIndex & qsizetype{4095}) == 0 &&
                cancellationRequested && cancellationRequested->load()) {
                return false;
            }
            const int64_t pointOffsetNs = sourceOffsetNs +
                int64_t(pointIndex) * sourceFrameDurationNs_ / int64_t(sourceFrame.points.size());
            pendingFrame_.points.push_back(toSlamPoint(sourceFrame.points.at(pointIndex), pointOffsetNs));
        }
        pointCount_ += uint64_t(sourceFrame.points.size());

        if (completesPendingFrame) {
            *frame = std::move(completedFrame);
            return true;
        }
    }

    if (!pendingFrame_.points.isEmpty()) {
        pendingFrame_.sequence = emittedFrameCount_++;
        pendingFrame_.frameEndNs = pendingFrame_.frameStartNs + pendingFrame_.points.last().offsetNs;
        *frame = std::move(pendingFrame_);
        pendingFrame_ = SlamInputFrame();
        return true;
    }
    return false;
}

int LvxSlamSource::estimatedFrameCount() const
{
    return estimatedFrameCount_;
}

QString LvxSlamSource::summaryText() const
{
    return QStringLiteral(
        "%1 LO 流式输入摘要\n- 文件: %2\n- 已生成帧数: %3（预计 %4）\n- 已读取点数: %5\n- IMU 样本数: 0\n- 聚帧周期: %6 ms\n- 点内时间: 按 LVX 包帧时间和点顺序重建")
        .arg(lvx2_ ? QStringLiteral("LVX2") : QStringLiteral("LVX"),
             filePath_,
             QString::number(emittedFrameCount_),
             QString::number(estimatedFrameCount_),
             QString::number(pointCount_),
             QString::number(frameDurationNs_ / int64_t{1000000}));
}

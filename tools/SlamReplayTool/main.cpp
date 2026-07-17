#include "Backends/FastLio/FastLioSlamBackend.h"
#include "Core/SlamRuntimeConfig.h"
#include "Io/LvxSlamSource.h"
#include "Io/PcapSlamSource.h"
#include "Io/RosbagSlamSource.h"

#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QStringList>
#include <QTextStream>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <utility>

namespace {

QString statusName(SlamStatusCode status)
{
    switch (status) {
    case SlamStatusCode::Idle:
        return QStringLiteral("Idle");
    case SlamStatusCode::Starting:
        return QStringLiteral("Starting");
    case SlamStatusCode::InitializingImu:
        return QStringLiteral("InitializingImu");
    case SlamStatusCode::Running:
        return QStringLiteral("Running");
    case SlamStatusCode::Paused:
        return QStringLiteral("Paused");
    case SlamStatusCode::Backpressure:
        return QStringLiteral("Backpressure");
    case SlamStatusCode::MissingImu:
        return QStringLiteral("MissingImu");
    case SlamStatusCode::TimeSyncError:
        return QStringLiteral("TimeSyncError");
    case SlamStatusCode::Degraded:
        return QStringLiteral("Degraded");
    case SlamStatusCode::Failed:
        return QStringLiteral("Failed");
    case SlamStatusCode::Stopped:
        return QStringLiteral("Stopped");
    }
    return QStringLiteral("Unknown");
}

QString templateName(SlamLidarTemplate lidarTemplate)
{
    return slamLidarTemplateDisplayName(lidarTemplate);
}

SlamLidarTemplate inferTemplateForPath(const QString& filePath)
{
    const QString path = QFileInfo(filePath).absoluteFilePath().toLower();
    if (path.contains(QStringLiteral("avia2"))) {
        return SlamLidarTemplate::Avia2;
    }
    if (path.contains(QStringLiteral("avia"))) {
        return SlamLidarTemplate::Avia;
    }
    if (path.contains(QStringLiteral("mid360l")) || path.contains(QStringLiteral("mid-360l"))) {
        return SlamLidarTemplate::Mid360L;
    }
    return SlamLidarTemplate::Mid360Mid360S;
}

SlamRuntimeConfig diagnosticConfigForPath(const QString& filePath)
{
    SlamRuntimeConfig config;
    applySlamLidarTemplateDefaults(config, inferTemplateForPath(filePath));
    config.publishWorldFrameCloud = false;
    config.publishDenseFrameCloud = false;
    config.publishBodyFrameCloud = false;
    config.saveMap = false;
    return config;
}

bool isPcapPath(const QString& filePath)
{
    const QString suffix = QFileInfo(filePath).suffix().toLower();
    return suffix == QStringLiteral("pcap") || suffix == QStringLiteral("pcapng");
}

bool isRosbagPath(const QString& filePath)
{
    const QString suffix = QFileInfo(filePath).suffix().toLower();
    return suffix == QStringLiteral("bag") ||
           suffix == QStringLiteral("db3") ||
           suffix == QStringLiteral("yaml") ||
           suffix == QStringLiteral("yml");
}

bool isLvxPath(const QString& filePath)
{
    const QString suffix = QFileInfo(filePath).suffix().toLower();
    return suffix == QStringLiteral("lvx") || suffix == QStringLiteral("lvx2");
}

QString resolveInputPath(const QString& inputPath)
{
    const QFileInfo info(inputPath);
    if (!info.isDir()) {
        return info.absoluteFilePath();
    }

    const QDir dir(info.absoluteFilePath());
    const QString metadataPath = dir.filePath(QStringLiteral("metadata.yaml"));
    if (QFileInfo::exists(metadataPath)) {
        return QFileInfo(metadataPath).absoluteFilePath();
    }
    const QStringList entries = dir.entryList(QStringList()
                                                  << QStringLiteral("*.bag")
                                                  << QStringLiteral("*.db3")
                                                  << QStringLiteral("*.pcap")
                                                  << QStringLiteral("*.pcapng"),
                                              QDir::Files,
                                              QDir::Name);
    if (entries.isEmpty()) {
        return info.absoluteFilePath();
    }
    return QFileInfo(dir.filePath(entries.first())).absoluteFilePath();
}

struct ReplaySummary {
    bool ok = false;
    QString error;
    int sourceFrames = 0;
    int skippedIncompleteImuFrames = 0;
    int processedFrames = 0;
    int initializingFrames = 0;
    int runningFrames = 0;
    int trajectoryPoints = 0;
    int mapFrames = 0;
    int globalMapPoints = 0;
    int mapPoints = 0;
    int keyframeCount = 0;
    int loopClosureCount = 0;
    int optimizedMapPoints = 0;
    qint64 processingMs = 0;
    qint64 finalizeMs = 0;
    qint64 stopMs = 0;
    SlamPose finalPose;
};

struct FrameStats {
    int frames = 0;
    int completeImuFrames = 0;
    int incompleteImuFrames = 0;
    uint64_t points = 0;
    uint64_t imuSamples = 0;
    int minPoints = std::numeric_limits<int>::max();
    int maxPoints = 0;
    int minImuSamples = std::numeric_limits<int>::max();
    int maxImuSamples = 0;
    int64_t minDurationNs = std::numeric_limits<int64_t>::max();
    int64_t maxDurationNs = 0;
};

void appendFrameStats(FrameStats& stats, const SlamInputFrame& frame)
{
    ++stats.frames;
    if (frame.hasCompleteImuCoverage) {
        ++stats.completeImuFrames;
    } else {
        ++stats.incompleteImuFrames;
    }
    const int pointCount = frame.points.size();
    const int imuCount = frame.imuSamples.size();
    const int64_t durationNs = std::max<int64_t>(0, frame.frameEndNs - frame.frameStartNs);
    stats.points += uint64_t(pointCount);
    stats.imuSamples += uint64_t(imuCount);
    stats.minPoints = std::min(stats.minPoints, pointCount);
    stats.maxPoints = std::max(stats.maxPoints, pointCount);
    stats.minImuSamples = std::min(stats.minImuSamples, imuCount);
    stats.maxImuSamples = std::max(stats.maxImuSamples, imuCount);
    stats.minDurationNs = std::min(stats.minDurationNs, durationNs);
    stats.maxDurationNs = std::max(stats.maxDurationNs, durationNs);
}

FrameStats collectFrameStats(const QVector<SlamInputFrame>& frames)
{
    FrameStats stats;
    for (const SlamInputFrame& frame : frames) {
        appendFrameStats(stats, frame);
    }
    if (stats.frames == 0) {
        stats.minPoints = 0;
        stats.minImuSamples = 0;
        stats.minDurationNs = 0;
    }
    return stats;
}

struct DiagnosticSource {
    QString kind;
    QString summaryText;
    QVector<SlamInputFrame> frames;
};

bool loadDiagnosticSource(const QString& filePath,
                          const SlamRuntimeConfig& config,
                          bool requireImu,
                          DiagnosticSource* source,
                          QString* error)
{
    *source = DiagnosticSource();
    if (isPcapPath(filePath)) {
        PcapSlamSource pcapSource(config.inputFrameDurationMs);
        if (!pcapSource.load(filePath, error)) {
            source->kind = QStringLiteral("pcap");
            source->summaryText = pcapSource.summaryText();
            return false;
        }
        source->kind = QStringLiteral("pcap");
        source->summaryText = pcapSource.summaryText();
        source->frames = pcapSource.frames();
        return true;
    }

    if (isRosbagPath(filePath)) {
        RosbagSlamSourceConfig rosbagConfig;
        rosbagConfig.frameDurationMs = config.inputFrameDurationMs;
        rosbagConfig.allowLivoxDriver2PointCloud2 = config.allowRosbagDriver2PointCloud2;
        rosbagConfig.allowLivoxDriverPointCloud2SynthesizedTime =
            config.allowRosbagDriverPointCloud2SynthesizedTime;
        rosbagConfig.lidarToImuTimeOffsetNs = config.lidarToImuTimeOffsetNs;
        rosbagConfig.requireImu = requireImu;
        rosbagConfig.synthesizePointOffsetTime = true;
        RosbagSlamSource rosbagSource(rosbagConfig);
        if (!rosbagSource.load(filePath, error)) {
            source->kind = QStringLiteral("rosbag");
            source->summaryText = rosbagSource.summaryText();
            return false;
        }
        source->kind = QStringLiteral("rosbag");
        source->summaryText = rosbagSource.summaryText();
        source->frames = rosbagSource.frames();
        return true;
    }

    if (isLvxPath(filePath)) {
        LvxSlamSource lvxSource(config.inputFrameDurationMs);
        std::atomic_bool cancellationRequested{false};
        if (!lvxSource.open(filePath, &cancellationRequested, error)) {
            source->kind = QStringLiteral("lvx");
            return false;
        }
        source->kind = QFileInfo(filePath).suffix().toLower();
        SlamInputFrame frame;
        while (lvxSource.readNextFrame(&frame, &cancellationRequested, error)) {
            source->frames.push_back(std::move(frame));
            frame = SlamInputFrame();
        }
        if (error != nullptr && !error->isEmpty()) {
            return false;
        }
        source->summaryText = lvxSource.summaryText();
        return true;
    }

    if (error != nullptr) {
        *error = QStringLiteral("Unsupported input suffix: %1").arg(QFileInfo(filePath).suffix());
    }
    return false;
}

ReplaySummary runFrames(const QVector<SlamInputFrame>& frames, const SlamRuntimeConfig& config)
{
    ReplaySummary result;
    result.sourceFrames = frames.size();

    QString error;
    FastLioSlamBackend backend;
    if (!backend.start(config, &error)) {
        result.error = error;
        return result;
    }

    QElapsedTimer processingTimer;
    processingTimer.start();
    for (const SlamInputFrame& frame : frames) {
        if (config.imuEnabled && !frame.hasCompleteImuCoverage) {
            ++result.skippedIncompleteImuFrames;
            continue;
        }

        SlamOutput output;
        error.clear();
        if (!backend.processFrame(frame, &output, &error)) {
            result.error = QStringLiteral("frame %1 failed: %2 (%3)")
                               .arg(frame.sequence)
                               .arg(statusName(output.status))
                               .arg(error);
            return result;
        }

        ++result.processedFrames;
        if (output.status == SlamStatusCode::InitializingImu) {
            ++result.initializingFrames;
        } else if (output.status == SlamStatusCode::Running) {
            ++result.runningFrames;
        }
        result.trajectoryPoints += output.newTrajectoryPoints.size();
        if (!output.newGlobalMapPoints.isEmpty()) {
            ++result.mapFrames;
            result.globalMapPoints += output.newGlobalMapPoints.size();
        }
        result.mapPoints = output.mapPointCount;
        result.keyframeCount = output.keyframeCount;
        result.loopClosureCount = output.loopClosureCount;
        result.finalPose = output.currentPose;
    }
    result.processingMs = processingTimer.elapsed();

    if (result.processedFrames == 0) {
        result.error = QStringLiteral("no input frame was processed");
        return result;
    }
    if (result.runningFrames == 0 || result.trajectoryPoints == 0) {
        result.error = QStringLiteral("FAST_LIO replay did not produce running pose/trajectory output");
        return result;
    }

    SlamOutput finalOutput;
    error.clear();
    QElapsedTimer finalizeTimer;
    finalizeTimer.start();
    if (!backend.finalize(&finalOutput, &error)) {
        result.error = error;
        return result;
    }
    result.finalizeMs = finalizeTimer.elapsed();
    result.keyframeCount = finalOutput.keyframeCount;
    result.loopClosureCount = finalOutput.loopClosureCount;
    result.optimizedMapPoints = finalOutput.optimizedGlobalMapPoints.size();
    QElapsedTimer stopTimer;
    stopTimer.start();
    backend.stop();
    result.stopMs = stopTimer.elapsed();
    result.ok = true;
    return result;
}

ReplaySummary runWithImuReplay(const QString& pcapPath)
{
    ReplaySummary result;
    PcapSlamSource source;
    QString error;
    if (!source.load(pcapPath, &error)) {
        result.error = error;
        return result;
    }

    result = runFrames(source.frames(), SlamRuntimeConfig());
    result.sourceFrames = source.frameCount();
    if (result.ok && result.globalMapPoints == 0) {
        result.error = QStringLiteral("FAST_LIO replay did not produce map output");
        result.ok = false;
    }
    return result;
}

bool sameFinalPose(const SlamPose& a, const SlamPose& b)
{
    constexpr double kTolerance = 1.0e-6;
    const double delta = std::fabs(a.tx - b.tx) +
                         std::fabs(a.ty - b.ty) +
                         std::fabs(a.tz - b.tz) +
                         std::fabs(a.qx - b.qx) +
                         std::fabs(a.qy - b.qy) +
                         std::fabs(a.qz - b.qz) +
                         std::fabs(a.qw - b.qw);
    return delta <= kTolerance;
}

bool verifyNoImuReplay(const QString& pcapPath, QTextStream& out)
{
    PcapSlamSource source;
    QString error;
    if (!source.load(pcapPath, &error)) {
        out << "NO_IMU_LOAD_FAILED " << error << "\n";
        return false;
    }

    out << "NO_IMU_SOURCE status=" << statusName(source.summary().status)
        << " frames=" << source.frameCount()
        << " imuSamples=" << source.summary().imuSampleCount << "\n";

    FastLioSlamBackend backend;
    SlamRuntimeConfig config;
    if (!backend.start(config, &error)) {
        out << "NO_IMU_BACKEND_START_FAILED " << error << "\n";
        return false;
    }

    SlamOutput output;
    const bool accepted = backend.processFrame(source.frameAt(0), &output, &error);
    if (accepted || output.status != SlamStatusCode::MissingImu) {
        out << "NO_IMU_REJECT_FAILED accepted=" << accepted
            << " status=" << statusName(output.status)
            << " error=" << error << "\n";
        return false;
    }

    out << "NO_IMU_REJECT_OK status=" << statusName(output.status) << "\n";
    return true;
}

int runPcapRegression(const QString& withImuPath, const QString& noImuPath, QTextStream& out)
{
    const ReplaySummary firstRun = runWithImuReplay(withImuPath);
    if (!firstRun.ok) {
        out << "WITH_IMU_RUN1_FAILED " << firstRun.error << "\n";
        return 1;
    }

    out << "WITH_IMU_RUN1_OK"
        << " sourceFrames=" << firstRun.sourceFrames
        << " skippedIncompleteImu=" << firstRun.skippedIncompleteImuFrames
        << " processed=" << firstRun.processedFrames
        << " initializing=" << firstRun.initializingFrames
        << " running=" << firstRun.runningFrames
        << " trajectory=" << firstRun.trajectoryPoints
        << " mapFrames=" << firstRun.mapFrames
        << " globalMapPoints=" << firstRun.globalMapPoints
        << " mapPoints=" << firstRun.mapPoints
        << " finalPose=[" << firstRun.finalPose.tx << "," << firstRun.finalPose.ty << "," << firstRun.finalPose.tz << "]\n";

    const ReplaySummary secondRun = runWithImuReplay(withImuPath);
    if (!secondRun.ok) {
        out << "WITH_IMU_RUN2_FAILED " << secondRun.error << "\n";
        return 1;
    }

    const bool deterministic = firstRun.processedFrames == secondRun.processedFrames &&
                               firstRun.runningFrames == secondRun.runningFrames &&
                               firstRun.trajectoryPoints == secondRun.trajectoryPoints &&
                               firstRun.mapFrames == secondRun.mapFrames &&
                               firstRun.globalMapPoints == secondRun.globalMapPoints &&
                               sameFinalPose(firstRun.finalPose, secondRun.finalPose);
    if (!deterministic) {
        out << "WITH_IMU_REPEAT_MISMATCH\n";
        return 1;
    }

    out << "WITH_IMU_REPEAT_OK"
        << " processed=" << secondRun.processedFrames
        << " running=" << secondRun.runningFrames
        << " trajectory=" << secondRun.trajectoryPoints
        << " mapFrames=" << secondRun.mapFrames
        << " globalMapPoints=" << secondRun.globalMapPoints << "\n";

    if (!verifyNoImuReplay(noImuPath, out)) {
        return 1;
    }

    out << "SLAM_REPLAY_TOOL_OK\n";
    return 0;
}

void printSourceSummary(const QString& text, QTextStream& out)
{
    const QStringList lines = text.split(QLatin1Char('\n'), Qt::SkipEmptyParts);
    for (const QString& line : lines) {
        out << "SOURCE_DETAIL " << line << "\n";
    }
}

int runDiagnostic(const QStringList& inputPaths,
                  int maxFrames,
                  bool sourceOnly,
                  bool lidarOnly,
                  bool loopClosure,
                  bool saveMap,
                  QTextStream& out)
{
    bool allOk = true;
    for (const QString& inputPath : inputPaths) {
        const QString filePath = resolveInputPath(inputPath);
        SlamRuntimeConfig config = diagnosticConfigForPath(filePath);
        config.allowPureLidar = lidarOnly;
        config.imuEnabled = !lidarOnly;
        config.backendType = lidarOnly ? QStringLiteral("FAST_LO") : QStringLiteral("FAST_LIO");
        config.loopClosureEnableFlag = loopClosure;
        config.deterministicOfflineLoopClosure = loopClosure;
        config.saveMap = saveMap;
        out << "DIAG_BEGIN path=" << filePath
            << " template=" << templateName(config.lidarTemplate)
            << " detRangeM=" << config.detRangeM
            << " blindM=" << config.blindMinRangeM
            << " frameMs=" << config.inputFrameDurationMs << "\n";
        out.flush();

        QString error;
        DiagnosticSource source;
        if (!loadDiagnosticSource(filePath, config, !sourceOnly && !lidarOnly, &source, &error)) {
            allOk = false;
            out << "SOURCE_FAILED path=" << filePath << " error=" << error << "\n";
            if (!source.summaryText.isEmpty()) {
                printSourceSummary(source.summaryText, out);
            }
            out << "DIAG_END path=" << filePath << " ok=0\n";
            out.flush();
            continue;
        }

        printSourceSummary(source.summaryText, out);
        if (maxFrames > 0 && source.frames.size() > maxFrames) {
            source.frames.resize(maxFrames);
            out << "DIAG_LIMIT maxFrames=" << maxFrames << "\n";
        }
        const FrameStats stats = collectFrameStats(source.frames);
        out << "FRAME_STATS"
            << " kind=" << source.kind
            << " frames=" << stats.frames
            << " completeImu=" << stats.completeImuFrames
            << " incompleteImu=" << stats.incompleteImuFrames
            << " points=" << stats.points
            << " imuSamples=" << stats.imuSamples
            << " minPoints=" << stats.minPoints
            << " maxPoints=" << stats.maxPoints
            << " minImu=" << stats.minImuSamples
            << " maxImu=" << stats.maxImuSamples
            << " minDurationNs=" << stats.minDurationNs
            << " maxDurationNs=" << stats.maxDurationNs << "\n";
        out.flush();

        if (sourceOnly) {
            out << "SOURCE_ONLY_OK"
                << " path=" << filePath
                << " frames=" << stats.frames
                << " points=" << stats.points << "\n";
            out << "DIAG_END path=" << filePath << " ok=1\n";
            out.flush();
            continue;
        }

        const ReplaySummary replay = runFrames(source.frames, config);
        if (!replay.ok) {
            allOk = false;
            out << "BACKEND_FAILED"
                << " sourceFrames=" << replay.sourceFrames
                << " skippedIncompleteImu=" << replay.skippedIncompleteImuFrames
                << " processed=" << replay.processedFrames
                << " initializing=" << replay.initializingFrames
                << " running=" << replay.runningFrames
                << " trajectory=" << replay.trajectoryPoints
                << " mapPoints=" << replay.mapPoints
                << " error=" << replay.error << "\n";
            out << "DIAG_END path=" << filePath << " ok=0\n";
            out.flush();
            continue;
        }

        out << "BACKEND_OK"
            << " sourceFrames=" << replay.sourceFrames
            << " skippedIncompleteImu=" << replay.skippedIncompleteImuFrames
            << " processed=" << replay.processedFrames
            << " initializing=" << replay.initializingFrames
            << " running=" << replay.runningFrames
            << " trajectory=" << replay.trajectoryPoints
            << " mapPoints=" << replay.mapPoints
            << " keyframes=" << replay.keyframeCount
            << " loops=" << replay.loopClosureCount
            << " optimizedMapPoints=" << replay.optimizedMapPoints
            << " processingMs=" << replay.processingMs
            << " finalizeMs=" << replay.finalizeMs
            << " stopMs=" << replay.stopMs
            << " finalPose=[" << replay.finalPose.tx << "," << replay.finalPose.ty << "," << replay.finalPose.tz << "]\n";
        out << "DIAG_END path=" << filePath << " ok=1\n";
        out.flush();
    }
    return allOk ? 0 : 1;
}

void printUsage(QTextStream& out)
{
    out << "Usage:\n";
    out << "  SlamReplayTool <with_imu.pcap> <no_imu.pcap>\n";
    out << "  SlamReplayTool --diagnose [--source-only] [--lidar-only] [--loop-closure] [--save-map] [--max-frames N] <pcap|bag|db3|lvx|lvx2|metadata.yaml|directory> [...]\n";
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    QTextStream out(stdout);
    const QStringList args = app.arguments();

    if (args.size() >= 3 && args.at(1) == QStringLiteral("--diagnose")) {
        int maxFrames = 0;
        bool sourceOnly = false;
        bool lidarOnly = false;
        bool loopClosure = false;
        bool saveMap = false;
        QStringList inputPaths;
        for (int i = 2; i < args.size(); ++i) {
            if (args.at(i) == QStringLiteral("--source-only")) {
                sourceOnly = true;
                continue;
            }
            if (args.at(i) == QStringLiteral("--lidar-only")) {
                lidarOnly = true;
                continue;
            }
            if (args.at(i) == QStringLiteral("--loop-closure")) {
                loopClosure = true;
                continue;
            }
            if (args.at(i) == QStringLiteral("--save-map")) {
                saveMap = true;
                continue;
            }
            if (args.at(i) == QStringLiteral("--max-frames") && i + 1 < args.size()) {
                maxFrames = args.at(++i).toInt();
                continue;
            }
            inputPaths.push_back(args.at(i));
        }
        if (inputPaths.isEmpty()) {
            printUsage(out);
            return 2;
        }
        return runDiagnostic(inputPaths, maxFrames, sourceOnly, lidarOnly, loopClosure, saveMap, out);
    }

    if (argc == 3) {
        return runPcapRegression(QString::fromLocal8Bit(argv[1]), QString::fromLocal8Bit(argv[2]), out);
    }

    printUsage(out);
    return 2;
}

#include "Slam/Backends/FastLio/FastLioSlamBackend.h"
#include "Slam/Io/PcapSlamSource.h"

#include <QCoreApplication>
#include <QTextStream>

#include <cmath>

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

struct ReplaySummary {
    bool ok = false;
    QString error;
    int sourceFrames = 0;
    int skippedIncompleteImuFrames = 0;
    int processedFrames = 0;
    int initializingFrames = 0;
    int runningFrames = 0;
    int trajectoryPoints = 0;
    int mapChunks = 0;
    int mapPoints = 0;
    SlamPose finalPose;
};

ReplaySummary runWithImuReplay(const QString& pcapPath)
{
    ReplaySummary result;
    PcapSlamSource source;
    QString error;
    if (!source.load(pcapPath, &error)) {
        result.error = error;
        return result;
    }

    result.sourceFrames = source.frameCount();
    FastLioSlamBackend backend;
    SlamRuntimeConfig config;
    if (!backend.start(config, &error)) {
        result.error = error;
        return result;
    }

    for (const SlamInputFrame& frame : source.frames()) {
        if (!frame.hasCompleteImuCoverage) {
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
        result.mapChunks += output.newMapChunks.size();
        result.mapPoints = output.mapPointCount;
        result.finalPose = output.currentPose;
    }

    if (result.runningFrames == 0 || result.trajectoryPoints == 0 || result.mapChunks == 0) {
        result.error = QStringLiteral("FAST_LIO replay did not produce running pose/trajectory/map output");
        return result;
    }

    result.ok = true;
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

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    QTextStream out(stdout);

    const QString withImuPath = argc > 1 ? QString::fromLocal8Bit(argv[1]) : QStringLiteral("E:/Livox_ws/with_imu.pcap");
    const QString noImuPath = argc > 2 ? QString::fromLocal8Bit(argv[2]) : QStringLiteral("E:/Livox_ws/no_imu.pcap");

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
        << " mapChunks=" << firstRun.mapChunks
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
                               firstRun.mapChunks == secondRun.mapChunks &&
                               sameFinalPose(firstRun.finalPose, secondRun.finalPose);
    if (!deterministic) {
        out << "WITH_IMU_REPEAT_MISMATCH\n";
        return 1;
    }

    out << "WITH_IMU_REPEAT_OK"
        << " processed=" << secondRun.processedFrames
        << " running=" << secondRun.runningFrames
        << " trajectory=" << secondRun.trajectoryPoints
        << " mapChunks=" << secondRun.mapChunks << "\n";

    if (!verifyNoImuReplay(noImuPath, out)) {
        return 1;
    }

    out << "SLAM_PHASE4_REPLAY_OK\n";
    return 0;
}

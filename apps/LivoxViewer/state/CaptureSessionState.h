#ifndef LIVOXVIEWER_CAPTURESESSIONSTATE_H
#define LIVOXVIEWER_CAPTURESESSIONSTATE_H

#include <QByteArray>
#include <QFile>
#include <QMutex>
#include <QString>
#include <QVector>
#include <cstdint>

class QTimer;

enum class CaptureTaskStatus {
    Idle,
    Running,
    Done
};

enum class PointCloudCaptureFormat {
    None,
    LVX2,
    PCD,
    LAS
};

struct CaptureTaskState
{
    CaptureTaskStatus status = CaptureTaskStatus::Idle;
    int secondsRemaining = 0;
    int totalSeconds = 0;
    int savedFiles = 0;
    int expectedFiles = 0;
    QString outputDir;
    QString outputPath;
    QString message;
};

struct CaptureSessionState
{
    QTimer* timer = nullptr;
    CaptureTaskState pointCloudTask;
    CaptureTaskState imuTask;
    CaptureTaskState logTask;
    CaptureTaskState debugTask;
    PointCloudCaptureFormat pointCloudFormat = PointCloudCaptureFormat::None;

    QString pcdSaveDir;
    bool pcdSaveActive = false;
    uint64_t pcdLastSavedTimestamp = 0;

    QString lasSaveDir;
    bool lasSaveActive = false;
    uint64_t lasLastSavedTimestamp = 0;

    uint64_t pointCloudSaveIntervalNs = 0;
    uint64_t pointCloudNextSaveTimestamp = 0;

    QString lvx2SaveDir;
    bool lvx2SaveActive = false;
    QFile lvx2File;
    QVector<QByteArray> lvx2PendingPkgs;
    uint64_t lvx2FrameStartNs = 0;
    uint64_t lvx2FrameIndex = 0;
    QMutex lvx2Mutex;

    QFile imuCsvFile;
    bool imuSaveActive = false;
    QMutex imuCsvMutex;

    uint32_t logHandle = 0;
    uint32_t debugHandle = 0;
};

#endif // LIVOXVIEWER_CAPTURESESSIONSTATE_H

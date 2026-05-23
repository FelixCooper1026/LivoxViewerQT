#ifndef LIVOXVIEWER_CAPTURESESSIONSTATE_H
#define LIVOXVIEWER_CAPTURESESSIONSTATE_H

#include <QByteArray>
#include <QFile>
#include <QMutex>
#include <QString>
#include <QVector>
#include <cstdint>

class QProgressBar;
class QPushButton;
class QSpinBox;
class QTimer;

enum CaptureType {
    CaptureNone,
    CaptureLog,
    CaptureDebug,
    CaptureLVX2,
    CaptureIMU
};

struct CaptureSessionState
{
    QSpinBox* durationSpin = nullptr;
    QPushButton* logButton = nullptr;
    QPushButton* debugButton = nullptr;
    QProgressBar* progress = nullptr;
    QTimer* timer = nullptr;
    int secondsRemaining = 0;
    int totalSeconds = 0;
    CaptureType current = CaptureNone;

    QString pcdSaveDir;
    int pcdFramesRemaining = 0;
    bool pcdSaveActive = false;
    uint64_t pcdLastSavedTimestamp = 0;

    QString lasSaveDir;
    int lasFramesRemaining = 0;
    bool lasSaveActive = false;
    uint64_t lasLastSavedTimestamp = 0;

    QString lvx2SaveDir;
    bool lvx2SaveActive = false;
    QFile lvx2File;
    QVector<QByteArray> lvx2PendingPkgs;
    uint64_t lvx2FrameStartNs = 0;
    uint64_t lvx2FrameIndex = 0;
    QMutex lvx2Mutex;

    QFile imuCsvFile;
    bool imuSaveActive = false;
    int imuSecondsRemaining = 0;
    int imuTotalSeconds = 0;
    QMutex imuCsvMutex;
};

#endif // LIVOXVIEWER_CAPTURESESSIONSTATE_H

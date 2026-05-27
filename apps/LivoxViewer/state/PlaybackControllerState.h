#ifndef LIVOXVIEWER_PLAYBACKCONTROLLERSTATE_H
#define LIVOXVIEWER_PLAYBACKCONTROLLERSTATE_H

#include <QMap>
#include <QString>
#include <QtGlobal>
#include <QVector>
#include <cstdint>
#include <memory>

#include "Lvx2/Lvx2PlaybackController.h"
#include "Playback/PlaybackSource.h"
#include "PointCloud/PointCloudFrame.h"

class QLabel;
class QPushButton;
class QComboBox;
class QSlider;
class QTimer;
class QWidget;

struct PlaybackControllerState
{
    std::shared_ptr<Playback::Source> source;
    QVector<Playback::DeviceInfo> devices;
    QMap<uint32_t, bool> deviceVisible;
    QString path;
    bool active = false;
    bool loading = false;
    bool playing = false;
    bool fileInfoDockVisible = false;
    int frame = -1;
    int frameCount = 0;
    quint64 loadToken = 0;

    QTimer* timer = nullptr;
    QWidget* bar = nullptr;
    QPushButton* playPauseButton = nullptr;
    QPushButton* firstFrameButton = nullptr;
    QPushButton* prevFrameButton = nullptr;
    QPushButton* nextFrameButton = nullptr;
    QPushButton* lastFrameButton = nullptr;
    QPushButton* closeButton = nullptr;
    QSlider* progressSlider = nullptr;
    QLabel* label = nullptr;
    QComboBox* speedCombo = nullptr;
    QComboBox* modeCombo = nullptr;

    bool updatingSlider = false;
    double speed = 1.0;
    Lvx2Playback::Mode mode = Lvx2Playback::Mode::SlidingWindow;

    int slidingWindowStart = -1;
    int slidingWindowEnd = -1;
    QVector<PointCloudPoint> slidingWindowPoints;
    QVector<int> slidingWindowSegmentPointCounts;
    uint64_t slidingWindowTimestamp = 0;

    void resetSlidingWindow()
    {
        slidingWindowStart = -1;
        slidingWindowEnd = -1;
        slidingWindowPoints.clear();
        slidingWindowSegmentPointCounts.clear();
        slidingWindowTimestamp = 0;
    }
};

#endif // LIVOXVIEWER_PLAYBACKCONTROLLERSTATE_H

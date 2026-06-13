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
    bool progressSliderDragging = false;
    double speed = 1.0;
    Lvx2Playback::Mode mode = Lvx2Playback::Mode::SlidingWindow;

    int slidingWindowStart = -1;
    int slidingWindowEnd = -1;
    QVector<PointCloudPoint> slidingWindowPoints;
    QVector<int> slidingWindowSegmentPointCounts;
    uint64_t slidingWindowTimestamp = 0;
    QMap<uint32_t, uint32_t> imuHandleByLidarId;
    uint32_t nextPlaybackImuHandle = 0x80000001u;

    void resetSlidingWindow()
    {
        slidingWindowStart = -1;
        slidingWindowEnd = -1;
        slidingWindowPoints.clear();
        slidingWindowSegmentPointCounts.clear();
        slidingWindowTimestamp = 0;
    }

    void resetPlaybackImuHandles()
    {
        imuHandleByLidarId.clear();
        nextPlaybackImuHandle = 0x80000001u;
    }

    uint32_t playbackImuHandleForLidarId(uint32_t lidarId)
    {
        auto it = imuHandleByLidarId.find(lidarId);
        if (it != imuHandleByLidarId.end()) {
            return it.value();
        }
        const uint32_t handle = nextPlaybackImuHandle++;
        imuHandleByLidarId.insert(lidarId, handle);
        return handle;
    }
};

#endif // LIVOXVIEWER_PLAYBACKCONTROLLERSTATE_H

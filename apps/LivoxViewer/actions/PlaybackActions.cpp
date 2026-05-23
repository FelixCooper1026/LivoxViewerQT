#include "LivoxViewerWindow.h"

#include <algorithm>
void LivoxViewerWindow::createPlaybackActions(QMenu* toolsMenu)
{
    // GPS 模拟定时器
    gpsTimer = new QTimer(this);
    connect(gpsTimer, &QTimer::timeout, this, &LivoxViewerWindow::onGpsTick);

    connect(lvx2PlayPauseButton, &QPushButton::clicked, [this]() {
        if (!playbackActive) {
            return;
        }
        if (playbackPlaying) {
            setLvx2PlaybackPlaying(false);
            return;
        }
        if (playbackFrameCount <= 0) {
            return;
        }
        if (playbackFrame < 0 || playbackFrame >= playbackFrameCount - 1) {
            showLvx2PlaybackFrame(0);
        }
        setLvx2PlaybackPlaying(true);
    });
    connect(lvx2FirstFrameButton, &QPushButton::clicked, [this]() {
        if (!playbackActive || playbackFrameCount <= 0) {
            return;
        }
        setLvx2PlaybackPlaying(false);
        showLvx2PlaybackFrame(0);
    });
    connect(lvx2PrevFrameButton, &QPushButton::clicked, [this]() {
        if (!playbackActive || playbackFrameCount <= 0) {
            return;
        }
        setLvx2PlaybackPlaying(false);
        showLvx2PlaybackFrame(std::max(0, playbackFrame - 1));
    });
    connect(lvx2NextFrameButton, &QPushButton::clicked, [this]() {
        if (!playbackActive || playbackFrameCount <= 0) {
            return;
        }
        setLvx2PlaybackPlaying(false);
        showLvx2PlaybackFrame(std::min(playbackFrameCount - 1, playbackFrame + 1));
    });
    connect(lvx2LastFrameButton, &QPushButton::clicked, [this]() {
        if (!playbackActive || playbackFrameCount <= 0) {
            return;
        }
        setLvx2PlaybackPlaying(false);
        showLvx2PlaybackFrame(playbackFrameCount - 1);
    });
    connect(lvx2ProgressSlider, &QSlider::valueChanged, this, &LivoxViewerWindow::onLvx2PlaybackSliderMoved);
    connect(lvx2SpeedCombo, &QComboBox::currentTextChanged, [this](const QString& text) {
        QString speedText = text;
        speedText.remove('x');
        bool ok = false;
        const double speed = speedText.toDouble(&ok);
        if (!ok || speed <= 0.0) {
            return;
        }
        playbackSpeed = speed;
        if (playbackPlaying) {
            setLvx2PlaybackPlaying(true);
        } else {
            updateLvx2PlaybackUi();
        }
    });
    connect(lvx2PlaybackModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        playbackMode = (index == 1) ? Lvx2PlaybackMode::SlidingWindow : Lvx2PlaybackMode::FrameByFrame;
        playbackSlidingWindowStart = -1;
        playbackSlidingWindowEnd = -1;
        playbackSlidingWindowPoints.clear();
        playbackSlidingWindowSegmentPointCounts.clear();
        playbackSlidingWindowTimestamp = 0;
        if (!playbackActive) {
            updateLvx2PlaybackUi();
            return;
        }
        const int sourceFrameCount = playbackSource ? playbackSource->frameCount() : 0;
        const int rawFramesPerStep = std::max(1, int((frameIntervalMs + 49ULL) / 50ULL));
        if (playbackMode == Lvx2PlaybackMode::SlidingWindow) {
            playbackFrameCount = sourceFrameCount;
        } else {
            playbackFrameCount = (sourceFrameCount + rawFramesPerStep - 1) / rawFramesPerStep;
        }
        if (playbackFrameCount <= 0) {
            playbackFrameCount = 1;
        }
        const int targetFrame = std::clamp(playbackFrame, 0, playbackFrameCount - 1);
        showLvx2PlaybackFrame(targetFrame);
        if (playbackPlaying) {
            setLvx2PlaybackPlaying(true);
        } else {
            updateLvx2PlaybackUi();
        }
    });
    connect(lvx2CloseButton, &QPushButton::clicked, [this]() {
        closeLvx2Playback(true);
    });
    actionShowImuCharts = toolsMenu->addAction("IMU数据绘图");
    connect(actionShowImuCharts, &QAction::triggered, this, &LivoxViewerWindow::onActionShowImuCharts);

    // 点云滤波
    QAction* actionPointCloudFilter = toolsMenu->addAction("点云滤波...");

    // 点云滤波对话框
    connect(actionPointCloudFilter, &QAction::triggered, this, &LivoxViewerWindow::showPointCloudFilterDialog);
}

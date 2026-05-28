#include "LivoxViewerWindow.h"

#include <algorithm>
void LivoxViewerWindow::createPlaybackActions(QMenu* toolsMenu)
{
    // GPS 模拟定时器
    imuState.gpsTimer = new QTimer(this);
    connect(imuState.gpsTimer, &QTimer::timeout, this, &LivoxViewerWindow::onGpsTick);

    connect(playbackState.playPauseButton, &QPushButton::clicked, [this]() {
        if (!playbackState.active) {
            return;
        }
        if (playbackState.playing) {
            setLvx2PlaybackPlaying(false);
            return;
        }
        if (playbackState.frameCount <= 0) {
            return;
        }
        if (playbackState.frame < 0 || playbackState.frame >= playbackState.frameCount - 1) {
            showLvx2PlaybackFrame(0);
        }
        setLvx2PlaybackPlaying(true);
    });
    connect(playbackState.firstFrameButton, &QPushButton::clicked, [this]() {
        if (!playbackState.active || playbackState.frameCount <= 0) {
            return;
        }
        setLvx2PlaybackPlaying(false);
        showLvx2PlaybackFrame(0);
    });
    connect(playbackState.prevFrameButton, &QPushButton::clicked, [this]() {
        if (!playbackState.active || playbackState.frameCount <= 0) {
            return;
        }
        setLvx2PlaybackPlaying(false);
        showLvx2PlaybackFrame(std::max(0, playbackState.frame - 1));
    });
    connect(playbackState.nextFrameButton, &QPushButton::clicked, [this]() {
        if (!playbackState.active || playbackState.frameCount <= 0) {
            return;
        }
        setLvx2PlaybackPlaying(false);
        showLvx2PlaybackFrame(std::min(playbackState.frameCount - 1, playbackState.frame + 1));
    });
    connect(playbackState.lastFrameButton, &QPushButton::clicked, [this]() {
        if (!playbackState.active || playbackState.frameCount <= 0) {
            return;
        }
        setLvx2PlaybackPlaying(false);
        showLvx2PlaybackFrame(playbackState.frameCount - 1);
    });
    connect(playbackState.progressSlider, &QSlider::valueChanged, this, &LivoxViewerWindow::onLvx2PlaybackSliderMoved);
    connect(playbackState.speedCombo, &QComboBox::currentTextChanged, [this](const QString& text) {
        QString speedText = text;
        speedText.remove('x');
        bool ok = false;
        const double speed = speedText.toDouble(&ok);
        if (!ok || speed <= 0.0) {
            return;
        }
        playbackState.speed = speed;
        if (playbackState.playing) {
            setLvx2PlaybackPlaying(true);
        } else {
            updateLvx2PlaybackUi();
        }
    });
    connect(playbackState.modeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int index) {
        playbackState.mode = (index == 1) ? Lvx2PlaybackMode::SlidingWindow : Lvx2PlaybackMode::FrameByFrame;
        playbackState.slidingWindowStart = -1;
        playbackState.slidingWindowEnd = -1;
        playbackState.slidingWindowPoints.clear();
        playbackState.slidingWindowSegmentPointCounts.clear();
        playbackState.slidingWindowTimestamp = 0;
        if (!playbackState.active) {
            updateLvx2PlaybackUi();
            return;
        }
        const int sourceFrameCount = playbackState.source ? playbackState.source->frameCount() : 0;
        const int rawFramesPerStep = std::max(1, int((frameIntervalMs + 49ULL) / 50ULL));
        if (playbackState.mode == Lvx2PlaybackMode::SlidingWindow) {
            playbackState.frameCount = sourceFrameCount;
        } else {
            playbackState.frameCount = (sourceFrameCount + rawFramesPerStep - 1) / rawFramesPerStep;
        }
        if (playbackState.frameCount <= 0) {
            playbackState.frameCount = 1;
        }
        const int targetFrame = std::clamp(playbackState.frame, 0, playbackState.frameCount - 1);
        showLvx2PlaybackFrame(targetFrame);
        if (playbackState.playing) {
            setLvx2PlaybackPlaying(true);
        } else {
            updateLvx2PlaybackUi();
        }
    });
    connect(playbackState.closeButton, &QPushButton::clicked, [this]() {
        closeLvx2Playback(true);
    });
    actionShowImuCharts = toolsMenu->addAction("IMU数据绘图");
    connect(actionShowImuCharts, &QAction::triggered, this, &LivoxViewerWindow::onActionShowImuCharts);

    // 点云滤波
    QAction* actionPointCloudFilter = toolsMenu->addAction("点云滤波...");
    QAction* actionTimeSync = toolsMenu->addAction("时间同步...");

    // 点云滤波对话框
    connect(actionPointCloudFilter, &QAction::triggered, this, &LivoxViewerWindow::showPointCloudFilterDialog);
    connect(actionTimeSync, &QAction::triggered, this, &LivoxViewerWindow::showTimeSyncDialog);
}

#include "LivoxViewerWindow.h"

#include <QShortcut>

void LivoxViewerWindow::createPlaybackActions(QMenu* toolsMenu)
{
    // GPS 模拟定时器
    imuState.gpsTimer = new QTimer(this);
    connect(imuState.gpsTimer, &QTimer::timeout, this, &LivoxViewerWindow::onGpsTick);

    connect(playbackState.playPauseButton, &QPushButton::clicked, this, &LivoxViewerWindow::playbackToggle);
    connect(playbackState.firstFrameButton, &QPushButton::clicked, this, &LivoxViewerWindow::playbackShowFirstFrame);
    connect(playbackState.prevFrameButton, &QPushButton::clicked, this, &LivoxViewerWindow::playbackShowPreviousFrame);
    connect(playbackState.nextFrameButton, &QPushButton::clicked, this, &LivoxViewerWindow::playbackShowNextFrame);
    connect(playbackState.lastFrameButton, &QPushButton::clicked, this, &LivoxViewerWindow::playbackShowLastFrame);

    auto bindPlaybackShortcut = [this](int key, const auto& callback) {
        QShortcut* shortcut = new QShortcut(QKeySequence(key), this);
        shortcut->setContext(Qt::WindowShortcut);
        connect(shortcut, &QShortcut::activated, this, callback);
    };
    bindPlaybackShortcut(Qt::Key_Space, [this]() { playbackToggle(); });
    bindPlaybackShortcut(Qt::Key_Left, [this]() { playbackShortcutPreviousFrame(); });
    bindPlaybackShortcut(Qt::Key_Right, [this]() { playbackShortcutNextFrame(); });
    bindPlaybackShortcut(Qt::CTRL | Qt::Key_Left, [this]() { playbackShowFirstFrame(); });
    bindPlaybackShortcut(Qt::CTRL | Qt::Key_Right, [this]() { playbackShowLastFrame(); });

    connect(playbackState.progressSlider, &QSlider::valueChanged, this, &LivoxViewerWindow::onLvx2PlaybackSliderMoved);
    connect(playbackState.speedCombo, &QComboBox::currentTextChanged, this, &LivoxViewerWindow::playbackSetSpeedText);
    connect(playbackState.modeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &LivoxViewerWindow::playbackSetModeIndex);
    actionShowImuCharts = toolsMenu->addAction("IMU数据可视化");
    connect(actionShowImuCharts, &QAction::triggered, this, &LivoxViewerWindow::onActionShowImuCharts);

    // 点云滤波
    QAction* actionPointCloudFilter = toolsMenu->addAction("点云滤波...");
    QAction* actionTimeSync = toolsMenu->addAction("时间同步...");

    // 点云滤波对话框
    connect(actionPointCloudFilter, &QAction::triggered, this, &LivoxViewerWindow::showPointCloudFilterDialog);
    connect(actionTimeSync, &QAction::triggered, this, &LivoxViewerWindow::showTimeSyncDialog);
}

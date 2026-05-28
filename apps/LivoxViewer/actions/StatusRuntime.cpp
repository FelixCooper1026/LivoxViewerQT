#include "LivoxViewerWindow.h"

void LivoxViewerWindow::createStatusBarAndTimers()
{
    // 状态栏
    QStatusBar* statusBar = new QStatusBar(this);
    setStatusBar(statusBar);
    statusLabelBar = new QLabel("就绪", statusBar);
    statusLabelBar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    statusBar->addPermanentWidget(statusLabelBar, 1);
    // 在状态栏添加采集进度条
    captureState.progress = new QProgressBar(statusBar);
    captureState.progress->setRange(0,100);
    captureState.progress->setValue(0);
    captureState.progress->setFixedWidth(260);
    captureState.progress->setTextVisible(true);
    statusBar->addPermanentWidget(captureState.progress, 0);

    // 标签页切换
    connect(paramTabWidget, &QTabWidget::currentChanged, this, &LivoxViewerWindow::onTabChanged);

    // 渲染定时器：固定刷新率（例如 30 FPS），与积分时间解耦
    renderTimer = new QTimer(this);
    renderTimer->setTimerType(Qt::PreciseTimer);
    connect(renderTimer, &QTimer::timeout, this, &LivoxViewerWindow::onRenderTick);
    renderTimer->start(33);
    playbackState.timer = new QTimer(this);
    playbackState.timer->setTimerType(Qt::PreciseTimer);
    connect(playbackState.timer, &QTimer::timeout, this, &LivoxViewerWindow::onLvx2PlaybackTick);
    // 采集定时器
    captureState.timer = new QTimer(this);
    connect(captureState.timer, &QTimer::timeout, this, &LivoxViewerWindow::onCaptureTick);
}

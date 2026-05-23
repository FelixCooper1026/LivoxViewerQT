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
    captureProgress = new QProgressBar(statusBar);
    captureProgress->setRange(0,100);
    captureProgress->setValue(0);
    captureProgress->setFixedWidth(260);
    captureProgress->setTextVisible(true);
    statusBar->addPermanentWidget(captureProgress, 0);

    // 信号槽连接
    // 刷新按钮已移除，无需实现 onRefreshClicked
    connect(lidarDeviceList, &QListWidget::currentRowChanged, this, &LivoxViewerWindow::onLidarDeviceSelected);

    // 标签页切换
    connect(paramTabWidget, &QTabWidget::currentChanged, this, &LivoxViewerWindow::onTabChanged);

    // 渲染定时器：固定刷新率（例如 30 FPS），与积分时间解耦
    renderTimer = new QTimer(this);
    renderTimer->setTimerType(Qt::PreciseTimer);
    connect(renderTimer, &QTimer::timeout, this, &LivoxViewerWindow::onRenderTick);
    renderTimer->start(33);
    lvx2PlaybackTimer = new QTimer(this);
    lvx2PlaybackTimer->setTimerType(Qt::PreciseTimer);
    connect(lvx2PlaybackTimer, &QTimer::timeout, this, &LivoxViewerWindow::onLvx2PlaybackTick);
    // 采集定时器
    captureTimer = new QTimer(this);
    connect(captureTimer, &QTimer::timeout, this, &LivoxViewerWindow::onCaptureTick);
}

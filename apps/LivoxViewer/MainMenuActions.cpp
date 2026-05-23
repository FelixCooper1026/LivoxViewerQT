#include "LivoxViewerWindow.h"
#include <QDesktopServices>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QInputDialog>
#include <QRadioButton>
#include <QStandardPaths>
#include <QUrl>

#include <algorithm>
void LivoxViewerWindow::createMenusAndActions()
{
    // 顶部工具栏
    actionClearCloud = new QAction("清除点云", this);
    actionResetView = new QAction("重置视图", this);

    // 菜单栏
    menuBar = new QMenuBar(this);
    setMenuBar(menuBar);
    fileMenu = menuBar->addMenu("文件");
    viewMenu = menuBar->addMenu("视图");
    deviceMenu = menuBar->addMenu("设备");
    QMenu* toolsMenu = menuBar->addMenu("工具");
    helpMenu = menuBar->addMenu("帮助");

    QAction* actionGenerateConfig = fileMenu->addAction("生成配置文件...");
    actionPlayLvx2 = fileMenu->addAction("播放LVX2点云...");
    actionPlayPcap = fileMenu->addAction("播放Pcap文件...");
    QAction* actionFormatConvert = fileMenu->addAction("格式转换...");
    QAction* actionPreferences = fileMenu->addAction("首选项...");
    fileMenu->addSeparator();
    exitAction = fileMenu->addAction("退出");

    connect(actionGenerateConfig, &QAction::triggered, this, [this]() {
        showConfigGeneratorDialog();
    });
    connect(actionPlayLvx2, &QAction::triggered, [this]() {
        QSettings settings("Livox", "LivoxViewerQT");
        QString lastDir = settings.value("playback/lastLVX2Dir",
                                         QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
        if (lastDir.isEmpty()) {
            lastDir = QDir::homePath();
        }
        const QString filePath = QFileDialog::getOpenFileName(this, "选择LVX2点云文件", lastDir, "LVX2点云 (*.lvx2)");
        if (filePath.isEmpty()) {
            return;
        }
        settings.setValue("playback/lastLVX2Dir", QFileInfo(filePath).absolutePath());
        loadLvx2PlaybackFile(filePath);
    });
    connect(actionPlayPcap, &QAction::triggered, [this]() {
        QSettings settings("Livox", "LivoxViewerQT");
        QString lastDir = settings.value("playback/lastPcapDir",
                                         QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
        if (lastDir.isEmpty()) {
            lastDir = QDir::homePath();
        }
        const QString filePath = QFileDialog::getOpenFileName(
            this,
            "选择Pcap点云文件",
            lastDir,
            "Pcap文件 (*.pcap *.pcapng);;所有文件 (*.*)");
        if (filePath.isEmpty()) {
            return;
        }
        settings.setValue("playback/lastPcapDir", QFileInfo(filePath).absolutePath());
        loadPcapPlaybackFile(filePath);
    });
    connect(actionFormatConvert, &QAction::triggered, this, &LivoxViewerWindow::showFormatConvertDialog);
    connect(actionPreferences, &QAction::triggered, this, &LivoxViewerWindow::showPreferencesDialog);

    createCaptureActions(toolsMenu);
    createDeviceActions();
    connect(exitAction, &QAction::triggered, this, &QWidget::close);
    createHelpActions();
    // 视图菜单：显示/隐藏 dock
    viewMenu->addAction(lidarDevicesDock->toggleViewAction());
    viewMenu->addAction(paramsDock->toggleViewAction());
    viewMenu->addAction(imuDock->toggleViewAction());
    if (lvx2FileDock) {
        viewMenu->addAction(lvx2FileDock->toggleViewAction());
    }
    viewMenu->addAction(logDock->toggleViewAction());

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
    createPlaybackActions(toolsMenu);

}

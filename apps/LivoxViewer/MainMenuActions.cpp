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

    // 数据采集子菜单
    QMenu* captureMenu = toolsMenu->addMenu("数据采集");
    QAction* actionCaptureLog = captureMenu->addAction("LOG数据采集...");
    QAction* actionCaptureDebug = captureMenu->addAction("Debug数据采集...");
    QMenu* saveMenu = toolsMenu->addMenu("保存点云");
    QAction* actionCaptureLVX2 = saveMenu->addAction("保存LVX2点云...");
    QAction* actionCapturePCD = saveMenu->addAction("保存PCD点云...");
    QAction* actionCaptureLAS = saveMenu->addAction("保存LAS点云...");
    QAction* actionSaveIMU = toolsMenu->addAction("保存IMU数据...");

    // 固件升级
    QAction* actionUpgrade = deviceMenu->addAction("固件升级...");

    // ==== 帮助菜单 ====
    // 1. Livox 官网
    QAction* actionLivoxWebsite = helpMenu->addAction("Livox 官网");
    // 2. Livox Wiki
    QAction* actionLivoxWiki = helpMenu->addAction("Livox Wiki");
    // 3. Mid-360 HMS 故障诊断码说明
    QAction* actionHmsCode = helpMenu->addAction("HMS 故障诊断码说明");
    // 4. 时间同步说明
    QAction* actionTimeSync = helpMenu->addAction("时间同步说明");
    // 5. 产品知识库(暂未实现)
    QAction* actionKnowledgeBase = helpMenu->addAction("产品知识库");
    // 6. 下载中心
    QAction* actionDownloadCenter = helpMenu->addAction("下载中心");

    // 关于
    aboutAction = helpMenu->addAction("关于");

    // 退出
    connect(exitAction, &QAction::triggered, this, &QWidget::close);
    // 关于
    connect(aboutAction, &QAction::triggered, [this]() {
        QMessageBox msgBox(this);
        msgBox.setWindowTitle("关于 LivoxViewerQT");
        msgBox.setTextFormat(Qt::RichText);  // 支持富文本
        msgBox.setText(
            "<h3>LivoxViewerQT - Livox 激光雷达可视化配置软件</h3>"
            "<p><b>版本:</b> 1.3.0</p>"
            "<p><b>编译日期:</b> " __DATE__ " </p>"
            "<p><b>作者:</b> FelixCooper1026</p>"
            "<p><b>功能特性:</b></p>"
            "<ul>"
            "<li>Livox 激光雷达设备连接与管理</li>"
            "<li>实时点云数据可视化</li>"
            "<li>设备参数配置与状态监控</li>"
            "<li>点云数据采集与保存</li>"
            "<li>IMU 数据显示与记录</li>"
            "<li>设备LOG数据采集与保存</li>"
            "<li>设备固件升级</li>"
            "</ul>"
            "<p><b>项目地址:</b> <a href=\"https://github.com/FelixCooper1026/LivoxViewerQT\">https://github.com/FelixCooper1026/LivoxViewerQT</a></p>"
            "<p>基于 Qt " QT_VERSION_STR " 和 Livox SDK2 开发</p>"
        );
        msgBox.exec();
    });

    // Livox 官网
    connect(actionLivoxWebsite, &QAction::triggered, []() {
        QDesktopServices::openUrl(QUrl("https://www.livoxtech.com/cn"));
    });

    // Livox Wiki
    connect(actionLivoxWiki, &QAction::triggered, []() {
        QDesktopServices::openUrl(QUrl("https://livox-wiki-cn.readthedocs.io/zh-cn/latest/tutorials/index.html"));
    });

    // Mid-360 HMS 故障诊断码说明
    connect(actionHmsCode, &QAction::triggered, []() {
        QDesktopServices::openUrl(QUrl("https://livox-wiki-cn.readthedocs.io/zh-cn/latest/tutorials/new_product/mid360/hms_code_mid360.html"));
    });

    // 时间同步说明
    connect(actionTimeSync, &QAction::triggered, []() {
        QDesktopServices::openUrl(QUrl("https://livox-wiki-cn.readthedocs.io/zh-cn/latest/tutorials/new_product/common/time_sync.html#id1"));
    });

    // 产品知识库（弹出对话框）
    connect(actionKnowledgeBase, &QAction::triggered, [this]() {
        QDialog dlg(this);
        dlg.setWindowTitle("产品知识库（暂未实现）");
        dlg.resize(600, 400);

        QVBoxLayout* layout = new QVBoxLayout(&dlg);
        QLabel* lbl = new QLabel("请选择需要查看的帮助文档：", &dlg);
        layout->addWidget(lbl);

        QListWidget* list = new QListWidget(&dlg);
        list->addItem("用户手册.pdf");
        list->addItem("快速入门.pdf");
        list->addItem("常见问题.pdf");
        layout->addWidget(list, 1);

        QPushButton* btnOpen = new QPushButton("打开文档", &dlg);
        layout->addWidget(btnOpen);

        connect(btnOpen, &QPushButton::clicked, [&]() {
            if (list->currentItem()) {
                QString fileName = list->currentItem()->text();
                QString filePath = QCoreApplication::applicationDirPath() + "/help/" + fileName;
                if (QFile::exists(filePath)) {
                    QDesktopServices::openUrl(QUrl::fromLocalFile(filePath));
                } else {
                    QMessageBox::warning(&dlg, "文件不存在", "未找到文档: " + filePath);
                }
            }
        });

        dlg.exec();
    });

    // 下载中心
    connect(actionDownloadCenter, &QAction::triggered, []() {
        QDesktopServices::openUrl(QUrl("https://www.livoxtech.com/cn/downloads"));
    });

    // 采集动作：弹窗输入时长，顶部显示进度条（复用已有captureProgress，放在状态栏）
    connect(actionCaptureLog, &QAction::triggered, [this]() {
        bool ok = false;
        int sec = QInputDialog::getInt(this, "LOG数据采集", "采集时长(秒):", 300, 10, 86400, 10, &ok);
        if (!ok) return;
        if (!captureDurationSpin) {
            captureDurationSpin = new QSpinBox(this);
            captureDurationSpin->setRange(10, 86400);   // ⭐ 设置最大值为 86400s (24 小时)
        }
        captureDurationSpin->setValue(sec);
        onStartCaptureLog();
    });
    connect(actionCaptureDebug, &QAction::triggered, [this]() {
        bool ok = false;
        int sec = QInputDialog::getInt(this, "Debug数据采集", "采集时长(秒):", 10, 1, 3600, 1, &ok);
        if (!ok) return;
        if (!captureDurationSpin) {
            captureDurationSpin = new QSpinBox(this);
            captureDurationSpin->setRange(1, 3600);   // ⭐ 设置最大值为 3600
        }
        captureDurationSpin->setValue(sec);
        onStartCaptureDebug();
    });

    //保存PCD点云
    connect(actionCapturePCD, &QAction::triggered, [this]() {
        if (!currentLidarDevice || !currentLidarDevice->is_connected) {
            QMessageBox::warning(this, "保存PCD点云", "设备未连接");
            return;
        }

        QSettings settings("Livox", "LivoxViewerQT");
        QString lastDir = settings.value("save/lastPCDDir", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
        if (lastDir.isEmpty()) lastDir = QDir::homePath();

        // 弹窗：保存路径 + 帧数
        QDialog dlg(this);
        dlg.setWindowTitle("保存PCD点云");
        QVBoxLayout* v = new QVBoxLayout(&dlg);
        QWidget* row1 = new QWidget(&dlg);
        QHBoxLayout* h1 = new QHBoxLayout(row1);
        h1->setContentsMargins(0,0,0,0);
        QLabel* lblPath = new QLabel("请选择保存路径:", row1);
        QLineEdit* editPath = new QLineEdit(row1);
        editPath->setText(lastDir);
        QPushButton* btnBrowse = new QPushButton("选择", row1);
        h1->addWidget(lblPath);
        h1->addSpacing(8);
        h1->addWidget(editPath, 1);
        h1->addSpacing(8);
        h1->addWidget(btnBrowse);
        v->addWidget(row1);

        QWidget* row2 = new QWidget(&dlg);
        QHBoxLayout* h2 = new QHBoxLayout(row2);
        h2->setContentsMargins(0,0,0,0);
        QLabel* lblCount = new QLabel("保存帧数:", row2);
        QSpinBox* spinCount = new QSpinBox(row2);
        spinCount->setRange(1, 1000000);
        spinCount->setSingleStep(1);
        spinCount->setValue(1);
        h2->addWidget(lblCount);
        h2->addSpacing(8);
        h2->addWidget(spinCount);
        h2->addStretch();
        v->addWidget(row2);

        QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        v->addWidget(box);

        connect(btnBrowse, &QPushButton::clicked, &dlg, [editPath, lastDir, this]() {
            QString dir = QFileDialog::getExistingDirectory(this, "选择保存目录", editPath->text().isEmpty() ? lastDir : editPath->text());
            if (!dir.isEmpty()) editPath->setText(dir);
        });

        connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

        if (dlg.exec() != QDialog::Accepted) return;

        QString baseDir = editPath->text().trimmed();
        if (baseDir.isEmpty()) {
            QMessageBox::warning(this, "保存PCD点云", "请选择保存路径");
            return;
        }

        // 保存本次选择的目录
        settings.setValue("save/lastPCDDir", baseDir);

        // 创建 PCD_雷达SN 目录
        QString sn = currentLidarDevice ? currentLidarDevice->sn : QString("Unknown");
        QString targetDir = QDir(baseDir).filePath(QString("PCD_%1").arg(sn));
        QDir().mkpath(targetDir);
        pcdSaveDir = targetDir;
        pcdFramesRemaining = spinCount->value();
        pcdSaveActive = true;
        pcdLastSavedTimestamp = 0;
        statusLabelBar->setText(QString("开始保存PCD，共 %1 帧...").arg(pcdFramesRemaining));
        logMessage(QString("PCD保存目录: %1").arg(QDir::toNativeSeparators(pcdSaveDir)));
    });

    //保存LAS点云
    connect(actionCaptureLAS, &QAction::triggered, [this]() {
        if (!currentLidarDevice || !currentLidarDevice->is_connected) {
            QMessageBox::warning(this, "保存LAS点云", "设备未连接");
            return;
        }

        // 读取上次目录
        QSettings settings("Livox", "LivoxViewerQT");
        QString lastDir = settings.value("save/lastLASDir", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
        if (lastDir.isEmpty()) lastDir = QDir::homePath();

        QDialog dlg(this);
        dlg.setWindowTitle("保存LAS点云");
        QVBoxLayout* v = new QVBoxLayout(&dlg);
        QWidget* row1 = new QWidget(&dlg);
        QHBoxLayout* h1 = new QHBoxLayout(row1);
        h1->setContentsMargins(0,0,0,0);
        QLabel* lblPath = new QLabel("请选择保存路径:", row1);
        QLineEdit* editPath = new QLineEdit(row1);
        editPath->setText(lastDir);
        QPushButton* btnBrowse = new QPushButton("选择", row1);
        h1->addWidget(lblPath);
        h1->addSpacing(8);
        h1->addWidget(editPath, 1);
        h1->addSpacing(8);
        h1->addWidget(btnBrowse);
        v->addWidget(row1);

        QWidget* row2 = new QWidget(&dlg);
        QHBoxLayout* h2 = new QHBoxLayout(row2);
        h2->setContentsMargins(0,0,0,0);
        QLabel* lblCount = new QLabel("保存帧数:", row2);
        QSpinBox* spinCount = new QSpinBox(row2);
        spinCount->setRange(1, 1000000);
        spinCount->setSingleStep(1);
        spinCount->setValue(1);
        h2->addWidget(lblCount);
        h2->addSpacing(8);
        h2->addWidget(spinCount);
        h2->addStretch();
        v->addWidget(row2);

        QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        v->addWidget(box);

        connect(btnBrowse, &QPushButton::clicked, &dlg, [editPath, lastDir, this]() {
            QString dir = QFileDialog::getExistingDirectory(this, "选择保存目录", editPath->text().isEmpty() ? lastDir : editPath->text());
            if (!dir.isEmpty()) editPath->setText(dir);
        });

        connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

        if (dlg.exec() != QDialog::Accepted) return;

        QString baseDir = editPath->text().trimmed();
        if (baseDir.isEmpty()) {
            QMessageBox::warning(this, "保存LAS点云", "请选择保存路径");
            return;
        }

        settings.setValue("save/lastLASDir", baseDir);
        QString sn = currentLidarDevice ? currentLidarDevice->sn : QString("Unknown");
        QString targetDir = QDir(baseDir).filePath(QString("LAS_%1").arg(sn));
        QDir().mkpath(targetDir);
        lasSaveDir = targetDir;
        lasFramesRemaining = spinCount->value();
        lasSaveActive = true;
        lasLastSavedTimestamp = 0;
        statusLabelBar->setText(QString("开始保存LAS，共 %1 帧...").arg(lasFramesRemaining));
        logMessage(QString("LAS保存目录: %1").arg(QDir::toNativeSeparators(lasSaveDir)));
    });

    //保存LVX2点云
    connect(actionCaptureLVX2, &QAction::triggered, [this]() {
        if (!currentLidarDevice || !currentLidarDevice->is_connected) {
            QMessageBox::warning(this, "保存LVX2点云", "设备未连接");
            return;
        }

        QSettings settings("Livox", "LivoxViewerQT");
        QString lastDir = settings.value("save/lastLVX2Dir", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
        if (lastDir.isEmpty()) lastDir = QDir::homePath();

        QDialog dlg(this);
        dlg.setWindowTitle("保存LVX2点云");
        QVBoxLayout* v = new QVBoxLayout(&dlg);
        QWidget* row1 = new QWidget(&dlg);
        QHBoxLayout* h1 = new QHBoxLayout(row1);
        h1->setContentsMargins(0,0,0,0);
        QLabel* lblPath = new QLabel("请选择保存路径:", row1);
        QLineEdit* editPath = new QLineEdit(row1);
        editPath->setText(lastDir);
        QPushButton* btnBrowse = new QPushButton("选择", row1);
        h1->addWidget(lblPath);
        h1->addSpacing(8);
        h1->addWidget(editPath, 1);
        h1->addSpacing(8);
        h1->addWidget(btnBrowse);
        v->addWidget(row1);

        QWidget* row2 = new QWidget(&dlg);
        QHBoxLayout* h2 = new QHBoxLayout(row2);
        h2->setContentsMargins(0,0,0,0);
        QLabel* lblSec = new QLabel("录制时长(s):", row2);
        QSpinBox* spinSec = new QSpinBox(row2);
        spinSec->setRange(1, 3600);
        spinSec->setSingleStep(1);
        spinSec->setValue(10);
        h2->addWidget(lblSec);
        h2->addSpacing(8);
        h2->addWidget(spinSec);
        h2->addStretch();
        v->addWidget(row2);

        QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
        v->addWidget(box);

        connect(btnBrowse, &QPushButton::clicked, &dlg, [editPath, lastDir, this]() {
            QString dir = QFileDialog::getExistingDirectory(this, "选择保存目录", editPath->text().isEmpty() ? lastDir : editPath->text());
            if (!dir.isEmpty()) editPath->setText(dir);
        });

        connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

        if (dlg.exec() != QDialog::Accepted) return;

        QString baseDir = editPath->text().trimmed();
        if (baseDir.isEmpty()) {
            QMessageBox::warning(this, "保存LVX2点云", "请选择保存路径");
            return;
        }

        settings.setValue("save/lastLVX2Dir", baseDir);
        QString sn = currentLidarDevice ? currentLidarDevice->sn : QString("Unknown");
        QString targetDir = QDir(baseDir).filePath(QString("LVX2_%1").arg(sn));
        QDir().mkpath(targetDir);
        QString startTime = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
        QString filePath = QDir(targetDir).filePath(QString("%1_%2.lvx2").arg(sn, startTime));

        // 配置进度条
        if (captureProgress) {
            captureProgress->setRange(0, 100);
            captureProgress->setValue(0);
            captureProgress->setFormat("录制中 %p% (%v s)");
        }
        // 保存配置并启动录制倒计时
        captureSecondsRemaining = spinSec->value();
        captureTotalSeconds = captureSecondsRemaining;
        currentCapture = CaptureLVX2;
        statusLabelBar->setText("正在录制LVX2...");
        logMessage(QString("LVX2保存路径: %1").arg(QDir::toNativeSeparators(filePath)));
        startLvx2Recording(filePath, captureSecondsRemaining);
        captureTimer->start(1000);
    });

    // 调整状态栏进度条长度
    if (captureProgress) {
        captureProgress->setFixedWidth(260);
    }

    connect(actionUpgrade, &QAction::triggered, this, &LivoxViewerWindow::showFirmwareUpgradeDialog);

    // 设备菜单新增：重启雷达 / 恢复出厂设置
    QAction* actionReboot = deviceMenu->addAction("重启雷达");
    QAction* actionFactoryReset = deviceMenu->addAction("恢复出厂设置");

    connect(actionReboot, &QAction::triggered, [this]() {
        if (!currentLidarDevice || !currentLidarDevice->is_connected) {
            logMessage("设备未连接，无法重启");
            return;
        }
        if (QMessageBox::warning(this, "重启雷达", "雷达将会重启，请确认操作", QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes) {
            livox_status st = LivoxLidarRequestReboot(currentLidarDevice->handle, nullptr, this);
            if (st == kLivoxLidarStatusSuccess) logMessage("已发送重启命令，请等待雷达重启...");
            else logMessage(QString("发送重启命令失败: %1").arg(st));
        }
    });

    connect(actionFactoryReset, &QAction::triggered, [this]() {
        if (!currentLidarDevice || !currentLidarDevice->is_connected) {
            logMessage("设备未连接，无法恢复出厂设置");
            return;
        }
        if (QMessageBox::warning(this, "恢复出厂设置", "雷达将会恢复出厂设置，雷达IP将恢复为192.168.1.3，请确认操作", QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes) {
            livox_status st = LivoxLidarRequestReset(currentLidarDevice->handle, nullptr, this);
            if (st == kLivoxLidarStatusSuccess) {
                logMessage("已发送恢复出厂设置命令，请等待雷达重启并恢复默认IP 192.168.1.3...");
                 // 清空当前设备缓存，避免显示旧IP设备
                 {
                     QMutexLocker locker(&lidarDeviceMutex);
                     lidarDevices.clear();
                 }
                 currentLidarDevice = nullptr;
                 updateLidarDeviceList();
                 statusLabelBar->setText("等待设备重启上线...");
                 // 分步重启SDK：先清理，稍后再初始化，避免竞态
                 QTimer::singleShot(1000, this, [this]() { shutdownLivoxSdk(); });
                 QTimer::singleShot(10000, this, [this]() { initializeLivoxSdk(); });
            } else {
                logMessage(QString("发送恢复出厂设置命令失败: %1").arg(st));
            }
        }
    });

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
    // GPS 模拟定时器
    gpsTimer = new QTimer(this);
    connect(gpsTimer, &QTimer::timeout, this, &LivoxViewerWindow::onGpsTick);

    connect(actionSaveIMU, &QAction::triggered, this, &LivoxViewerWindow::onActionCaptureImuTriggered);
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

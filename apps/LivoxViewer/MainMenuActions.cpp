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
    connect(actionFormatConvert, &QAction::triggered, [this]() {
        QSettings settings("Livox", "LivoxViewerQT");
        QString lastSrc = settings.value("convert/lastSource", "").toString();

        QDialog dlg(this);
        dlg.setWindowTitle("LVX2格式转换");
        dlg.resize(620, 340);
        QVBoxLayout* root = new QVBoxLayout(&dlg);

        QGroupBox* srcBox = new QGroupBox("源文件", &dlg);
        QHBoxLayout* srcLayout = new QHBoxLayout(srcBox);
        QLineEdit* srcEdit = new QLineEdit(srcBox);
        srcEdit->setText(lastSrc);
        QPushButton* srcBtn = new QPushButton("选择LVX2", srcBox);
        srcLayout->addWidget(srcEdit, 1);
        srcLayout->addWidget(srcBtn);
        root->addWidget(srcBox);

        QGroupBox* modeBox = new QGroupBox("转换模式", &dlg);
        QVBoxLayout* modeLayout = new QVBoxLayout(modeBox);
        QRadioButton* modeMerge = new QRadioButton("所有帧转换为一个文件", modeBox);
        QRadioButton* modeSplit = new QRadioButton("100ms单帧转换为多个文件", modeBox);
        modeMerge->setChecked(true);
        modeLayout->addWidget(modeMerge);
        modeLayout->addWidget(modeSplit);
        root->addWidget(modeBox);

        QGroupBox* formatBox = new QGroupBox("转换类型", &dlg);
        QHBoxLayout* formatLayout = new QHBoxLayout(formatBox);
        QRadioButton* fmtPcd = new QRadioButton("PCD", formatBox);
        QRadioButton* fmtLas = new QRadioButton("LAS", formatBox);
        QRadioButton* fmtCsv = new QRadioButton("CSV", formatBox);
        QRadioButton* fmtTxt = new QRadioButton("TXT", formatBox);
        fmtPcd->setChecked(true);
        formatLayout->addWidget(fmtPcd);
        formatLayout->addWidget(fmtLas);
        formatLayout->addWidget(fmtCsv);
        formatLayout->addWidget(fmtTxt);
        formatLayout->addStretch();
        root->addWidget(formatBox);

        QGroupBox* outBox = new QGroupBox("输出文件", &dlg);
        QGridLayout* outLayout = new QGridLayout(outBox);
        QLabel* outDirLabel = new QLabel("保存路径:", outBox);
        QLineEdit* outDirEdit = new QLineEdit(outBox);
        QPushButton* outDirBtn = new QPushButton("选择路径", outBox);
        QLabel* outNameLabel = new QLabel("文件名:", outBox);
        QLineEdit* outNameEdit = new QLineEdit(outBox);
        outLayout->addWidget(outDirLabel, 0, 0);
        outLayout->addWidget(outDirEdit, 0, 1);
        outLayout->addWidget(outDirBtn, 0, 2);
        outLayout->addWidget(outNameLabel, 1, 0);
        outLayout->addWidget(outNameEdit, 1, 1, 1, 2);
        root->addWidget(outBox);

        QProgressBar* progress = new QProgressBar(&dlg);
        progress->setRange(0, 100);
        progress->setValue(0);
        root->addWidget(progress);

        QLabel* resultLabel = new QLabel(&dlg);
        resultLabel->setText("就绪");
        root->addWidget(resultLabel);

        QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Close, &dlg);
        QPushButton* startBtn = new QPushButton("开始转换", &dlg);
        box->addButton(startBtn, QDialogButtonBox::ActionRole);
        root->addWidget(box);

        auto syncDefaultOutput = [srcEdit, outDirEdit, outNameEdit]() {
            const QFileInfo fi(srcEdit->text());
            if (fi.exists()) {
                outDirEdit->setText(fi.absolutePath());
                outNameEdit->setText(fi.completeBaseName());
            }
        };
        if (!srcEdit->text().isEmpty()) {
            syncDefaultOutput();
        }

        connect(srcBtn, &QPushButton::clicked, &dlg, [this, srcEdit, &settings, syncDefaultOutput]() {
            QString startDir = QFileInfo(srcEdit->text()).absolutePath();
            if (startDir.isEmpty()) {
                startDir = settings.value("convert/lastSourceDir", QDir::homePath()).toString();
            }
            const QString p = QFileDialog::getOpenFileName(this, "选择LVX2源文件", startDir, "LVX2点云 (*.lvx2)");
            if (p.isEmpty()) {
                return;
            }
            srcEdit->setText(p);
            settings.setValue("convert/lastSourceDir", QFileInfo(p).absolutePath());
            syncDefaultOutput();
        });
        connect(outDirBtn, &QPushButton::clicked, &dlg, [this, outDirEdit]() {
            const QString dir = QFileDialog::getExistingDirectory(this, "选择保存路径",
                outDirEdit->text().isEmpty() ? QDir::homePath() : outDirEdit->text());
            if (!dir.isEmpty()) {
                outDirEdit->setText(dir);
            }
        });
        connect(startBtn, &QPushButton::clicked, &dlg, [this, &settings, srcEdit, outDirEdit, outNameEdit,
                                                         modeSplit, fmtLas, fmtCsv, fmtTxt, progress, resultLabel, startBtn]() {
            const QString srcPath = srcEdit->text().trimmed();
            const QString outDir = outDirEdit->text().trimmed();
            const QString outName = outNameEdit->text().trimmed();
            if (srcPath.isEmpty() || outDir.isEmpty() || outName.isEmpty()) {
                resultLabel->setText("错误：请完整选择源文件、保存路径和文件名");
                return;
            }
            if (!QFileInfo::exists(srcPath)) {
                resultLabel->setText("错误：源文件不存在");
                return;
            }
            if (!QDir(outDir).exists()) {
                resultLabel->setText("错误：保存路径不存在");
                return;
            }

            settings.setValue("convert/lastSource", srcPath);
            const QString outputNoExt = QDir(outDir).filePath(outName);
            const Lvx2ConvertMode mode = modeSplit->isChecked()
                ? Lvx2ConvertMode::SplitBy100ms
                : Lvx2ConvertMode::MergeAllToOne;
            Lvx2ConvertFormat format = Lvx2ConvertFormat::PCD;
            if (fmtLas->isChecked()) {
                format = Lvx2ConvertFormat::LAS;
            } else if (fmtCsv->isChecked()) {
                format = Lvx2ConvertFormat::CSV;
            } else if (fmtTxt->isChecked()) {
                format = Lvx2ConvertFormat::TXT;
            }

            startBtn->setEnabled(false);
            progress->setValue(0);
            resultLabel->setText("正在转换...");
            bool ok = convertLvx2File(srcPath, outputNoExt, mode, format, [&](int done, int total) {
                const int value = (total > 0) ? (done * 100 / total) : 0;
                progress->setValue(value);
            });
            startBtn->setEnabled(true);
            progress->setValue(ok ? 100 : progress->value());
            resultLabel->setText(ok ? "转换完成，可继续选择其他源文件进行转换" :
                                      "转换失败，请检查LVX2文件格式或输出路径权限");
        });
        connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
        dlg.exec();
    });
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

    connect(actionUpgrade, &QAction::triggered, [this]() {
        // 获取所有选中的设备
        QList<QListWidgetItem*> selectedItems = lidarDeviceList->selectedItems();
        if (selectedItems.isEmpty()) {
            QMessageBox::warning(this, "固件升级", "请至少选择一个设备");
            return;
        }

        // 收集选中设备的句柄并验证连接状态
        QVector<uint32_t> selectedHandles;
        QVector<QString> disconnectedDevices;
        QMutexLocker locker(&lidarDeviceMutex);

        for (QListWidgetItem* item : selectedItems) {
            uint32_t handle = item->data(Qt::UserRole).toUInt();
            if (lidarDevices.contains(handle)) {
                const LidarDeviceInfo& device = lidarDevices[handle];
                if (device.is_connected) {
                    selectedHandles.append(handle);
                } else {
                    disconnectedDevices.append(device.sn);
                }
            }
        }
        locker.unlock();

        if (selectedHandles.isEmpty()) {
            QString msg = disconnectedDevices.isEmpty()
                ? "没有可用的已连接设备"
                : QString("所选设备未连接: %1").arg(disconnectedDevices.join(", "));
            QMessageBox::warning(this, "固件升级", msg);
            return;
        }

        if (!disconnectedDevices.isEmpty()) {
            int ret = QMessageBox::warning(this, "固件升级",
                QString("以下设备未连接，将跳过: %1\n\n是否继续升级已连接的设备？").arg(disconnectedDevices.join(", ")),
                QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
            if (ret != QMessageBox::Yes) {
                return;
            }
        }

        // 记住上次选择的固件路径
        QSettings settings("Livox", "LivoxViewerQT");
        const QString docs = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
        QString lastDir = settings.value("upgrade/lastFirmwareDir", docs.isEmpty() ? QDir::homePath() : docs).toString();
        QString fw = QFileDialog::getOpenFileName(this, "选择固件文件", lastDir, "固件 (*.bin *.img);;所有文件 (*.*)");
        if (fw.isEmpty()) return;
        settings.setValue("upgrade/lastFirmwareDir", QFileInfo(fw).absolutePath());

        // 设置升级路径为固件文件完整路径（本地分隔符 + 本地8位编码）
        QFileInfo fi(fw);
        QString tryPath = fi.absoluteFilePath();
        QByteArray pathLocal = QDir::toNativeSeparators(tryPath).toLocal8Bit();
        bool okPath = SetLivoxLidarUpgradeFirmwarePath(pathLocal.constData());
        if (!okPath) {
            QMessageBox::critical(this, "固件升级", "设置固件路径失败，请确保选择单个固件文件，路径避免包含特殊字符");
            return;
        }

        // 初始化升级进度映射和总数
        {
            QMutexLocker locker(&upgradeProgressMutex);
            upgradeProgressMap.clear();
            for (uint32_t handle : selectedHandles) {
                upgradeProgressMap[handle] = 0;
            }
            upgradeTotalDevices = selectedHandles.size();
        }

        // 设置进度回调，支持多设备进度跟踪
        SetLivoxLidarUpgradeProgressCallback([](uint32_t handle, LivoxLidarUpgradeState state, void* client){
            LivoxViewerWindow* w = static_cast<LivoxViewerWindow*>(client);
            if (!w || !w->captureProgress) return;
            QMetaObject::invokeMethod(w, [w, handle, state]() {
                // 在锁内更新进度和计算统计信息
                int avgProgress = 0;
                int completedCount = 0;
                int totalDevices = 0;
                bool allComplete = false;

                {
                    QMutexLocker locker(&w->upgradeProgressMutex);

                    // 如果该设备不在映射中，可能是之前的升级残留，忽略
                    if (!w->upgradeProgressMap.contains(handle)) {
                        return;
                    }

                    // 更新该设备的进度
                    // 检查状态事件：kLivoxLidarEventComplete = 4 表示完成
                    bool isComplete = (state.state == 4) || (state.progress >= 100);
                    w->upgradeProgressMap[handle] = isComplete ? 100 : state.progress;

                    // 计算平均进度和已完成设备数
                    totalDevices = w->upgradeTotalDevices;
                    if (totalDevices == 0) {
                        return;
                    }

                    int totalProgress = 0;
                    // 遍历所有应该升级的设备（映射中应该包含所有设备）
                    for (int progress : w->upgradeProgressMap.values()) {
                        totalProgress += progress;
                        if (progress >= 100) {
                            completedCount++;
                        }
                    }

                    // 计算平均进度：使用总设备数作为分母，确保即使某些设备未收到回调也能正确显示
                    avgProgress = totalDevices > 0 ? (totalProgress / totalDevices) : 0;

                    // 检查是否所有设备都完成
                    allComplete = (completedCount == totalDevices);
                    if (allComplete) {
                        w->upgradeProgressMap.clear();
                        w->upgradeTotalDevices = 0;
                    }
                } // 释放锁

                // 在锁外更新UI，避免长时间持有锁
                w->captureProgress->setValue(avgProgress);
                w->captureProgress->setFormat(QString("升级进度 %1% (%2/%3)").arg(avgProgress).arg(completedCount).arg(totalDevices));

                if (allComplete) {
                    w->statusLabelBar->setText(QString("升级完成 (%1个设备)，请等待设备重启").arg(totalDevices));
                    w->logMessage("固件升级已完成，请等待设备重启");
                }
            });
        }, this);

        // 准备设备句柄数组
        QVector<uint32_t> handleArr = selectedHandles;

        // 在后台线程执行升级，避免阻塞UI
        std::thread([handleArr]() {
            UpgradeLivoxLidars(handleArr.data(), handleArr.size());
        }).detach();

        captureProgress->setValue(0);
        captureProgress->setFormat(QString("升级进度 0% (0/%1)").arg(selectedHandles.size()));
        statusLabelBar->setText(QString("正在升级 %1 个设备，请勿断电或进行操作").arg(selectedHandles.size()));
        logMessage("正在进行固件升级，请勿断电或进行操作!!!");
    });


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
    connect(actionPointCloudFilter, &QAction::triggered, [this]() {
        if (!filterDialog) {
            filterDialog = new QDialog(this);
            filterDialog->setWindowTitle("点云滤波");
            filterDialog->setMinimumWidth(500);
            QVBoxLayout* layout = new QVBoxLayout(filterDialog);

            // Tag值滤波设置
            QGroupBox* tagGroup = new QGroupBox("Tag值滤波", filterDialog);
            QVBoxLayout* tagLayout = new QVBoxLayout(tagGroup);



            auto makeTagRow = [&](const QString& label, int& value, QSpinBox*& spin, QLabel*& desc, const QString& meaning) {
                QWidget* row = new QWidget(filterDialog);
                QHBoxLayout* h = new QHBoxLayout(row);
                h->setContentsMargins(0,0,0,0);
                QLabel* lbl = new QLabel(label + ":", row);
                spin = new QSpinBox(row);
                spin->setRange(0, 3);
                spin->setValue(value);
                spin->setToolTip("0: 置信度优; 1: 置信度中; 2: 置信度差; 3: 保留");
                desc = new QLabel(meaning, row);
                desc->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
                h->addWidget(lbl);
                h->addSpacing(8);
                h->addWidget(spin);
                h->addSpacing(12);
                h->addWidget(desc, 1);
                tagLayout->addWidget(row);
            };

            QLabel* desc76, *desc54, *desc32, *desc10;
            QString meaning76 = "保留位";
            QString meaning54 = "近处回吸噪点";
            QString meaning32 = "雨雾、灰尘等微小颗粒";
            QString meaning10 = "相近物体间的粘连点云";

            makeTagRow("Bit[7-6]", filterTagVal76, filterSpin76, desc76, meaning76);
            makeTagRow("Bit[5-4]", filterTagVal54, filterSpin54, desc54, meaning54);
            makeTagRow("Bit[3-2]", filterTagVal32, filterSpin32, desc32, meaning32);
            makeTagRow("Bit[1-0]", filterTagVal10, filterSpin10, desc10, meaning10);

            // 设置初始值
            if (filterSpin76) filterSpin76->setValue(filterTagVal76);
            if (filterSpin54) filterSpin54->setValue(filterTagVal54);
            if (filterSpin32) filterSpin32->setValue(filterTagVal32);
            if (filterSpin10) filterSpin10->setValue(filterTagVal10);

            layout->addWidget(tagGroup);



            // 滤噪列表
            QGroupBox* filterListGroup = new QGroupBox("滤噪列表", filterDialog);
            QVBoxLayout* filterListLayout = new QVBoxLayout(filterListGroup);

            // 添加Tag值到滤噪列表
            QWidget* addFilterRow = new QWidget(filterDialog);
            QHBoxLayout* addFilterLayout = new QHBoxLayout(addFilterRow);
            addFilterLayout->setContentsMargins(0,0,0,0);

            QLabel* addFilterLabel = new QLabel("当前Tag值:", addFilterRow);
            QLabel* currentTagLabel = new QLabel("0", addFilterRow);
            currentTagLabel->setStyleSheet("font-weight: bold; color: green;");
            addNoiseFilterButton = new QPushButton("添加到滤噪列表", addFilterRow);
            addNoiseFilterButton->setEnabled(false);

            addFilterLayout->addWidget(addFilterLabel);
            addFilterLayout->addWidget(currentTagLabel);
            addFilterLayout->addSpacing(12);
            addFilterLayout->addWidget(addNoiseFilterButton);
            addFilterLayout->addStretch();
            filterListLayout->addWidget(addFilterRow);

            // 滤噪列表显示
            noiseFilterList = new QListWidget(filterDialog);
            noiseFilterList->setMaximumHeight(120);
            filterListLayout->addWidget(noiseFilterList);

            // 移除按钮
            QHBoxLayout* removeFilterLayout = new QHBoxLayout();
            removeNoiseFilterButton = new QPushButton("移除选中项", filterDialog);
            removeNoiseFilterButton->setEnabled(false);
            removeFilterLayout->addWidget(removeNoiseFilterButton);
            removeFilterLayout->addStretch();
            filterListLayout->addLayout(removeFilterLayout);

            layout->addWidget(filterListGroup);

            // 噪点处理选项（全局设置）
            QGroupBox* noiseGroup = new QGroupBox("噪点处理", filterDialog);
            QVBoxLayout* noiseLayout = new QVBoxLayout(noiseGroup);

            showNoiseCheck = new QCheckBox("高亮显示噪点", noiseGroup);
            removeNoiseCheck = new QCheckBox("移除噪点（仅移除显示，并非真正不输出）", noiseGroup);

            noiseLayout->addWidget(showNoiseCheck);
            noiseLayout->addWidget(removeNoiseCheck);
            layout->addWidget(noiseGroup);

            // 控制按钮
            QWidget* ctrlRow = new QWidget(filterDialog);
            QHBoxLayout* ctrlLayout = new QHBoxLayout(ctrlRow);
            ctrlLayout->setContentsMargins(0,0,0,0);
            QPushButton* closeBtn = new QPushButton("关闭", ctrlRow);
            ctrlLayout->addStretch();
            ctrlLayout->addWidget(closeBtn);
            layout->addWidget(ctrlRow);

            // 连接信号
            auto updateTagLabel = [this]() {
                if (filterTagLabel && filterTagLabel->isVisible()) {
                    uint8_t tag = makeFilterTag();
                    filterTagLabel->setText(QString::number(tag));
                }
            };

            // 动态更新含义说明
            auto updateMeanings = [this, desc76, desc54, desc32, desc10, meaning76, meaning54, meaning32, meaning10]() {
                auto confToText = [](int v) {
                    switch(v & 3) {
                        case 0: return QString("置信度优");
                        case 1: return QString("置信度中");
                        case 2: return QString("置信度差");
                        default: return QString("保留");
                    }
                };

                if (desc76) desc76->setText(QString("%1（%2）").arg(meaning76, confToText(filterTagVal76)));
                if (desc54) desc54->setText(QString("%1（%2）").arg(meaning54, confToText(filterTagVal54)));
                if (desc32) desc32->setText(QString("%1（%2）").arg(meaning32, confToText(filterTagVal32)));
                if (desc10) desc10->setText(QString("%1（%2）").arg(meaning10, confToText(filterTagVal10)));
            };

            auto connectFilterSpin = [this, updateMeanings](QSpinBox* spin, const QString& desc) {
                connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), filterDialog, [this, spin, desc, updateMeanings]() {
                    if (desc == "Bit[7-6]") filterTagVal76 = spin->value();
                    else if (desc == "Bit[5-4]") filterTagVal54 = spin->value();
                    else if (desc == "Bit[3-2]") filterTagVal32 = spin->value();
                    else if (desc == "Bit[1-0]") filterTagVal10 = spin->value();

                    // 更新含义说明和标签
                    updateMeanings();

                    if (pointCloudView) pointCloudView->update();
                });
            };

            connectFilterSpin(filterSpin76, "Bit[7-6]");
            connectFilterSpin(filterSpin54, "Bit[5-4]");
            connectFilterSpin(filterSpin32, "Bit[3-2]");
            connectFilterSpin(filterSpin10, "Bit[1-0]");





            connect(showNoiseCheck, &QCheckBox::toggled, filterDialog, [this](bool en) {
                showNoisePoints = en;
                if (pointCloudView) pointCloudView->update();
            });
            connect(removeNoiseCheck, &QCheckBox::toggled, filterDialog, [this](bool en) {
                removeNoisePoints = en;
                if (pointCloudView) pointCloudView->update();
            });



            // 更新当前Tag值显示
            auto updateCurrentTagDisplay = [this, currentTagLabel]() {
                uint8_t tag = makeFilterTag();
                currentTagLabel->setText(QString::number(tag));

                // 检查是否已在列表中
                bool alreadyInList = noiseFilterTags.contains(tag);
                addNoiseFilterButton->setEnabled(!alreadyInList);
                addNoiseFilterButton->setText(alreadyInList ? "已在列表中" : "添加到滤噪列表");
            };

            // 连接滤噪列表相关信号
            connect(addNoiseFilterButton, &QPushButton::clicked, filterDialog, [this, currentTagLabel, updateCurrentTagDisplay]() {
                uint8_t tag = makeFilterTag();
                if (!noiseFilterTags.contains(tag)) {
                    noiseFilterTags.append(tag);
                    updateNoiseFilterList();
                    updateCurrentTagDisplay(); // 立即更新按钮状态和文字
                }
            });

            connect(removeNoiseFilterButton, &QPushButton::clicked, filterDialog, [this, updateCurrentTagDisplay]() {
                int currentRow = noiseFilterList->currentRow();
                if (currentRow >= 0 && currentRow < noiseFilterTags.size()) {
                    noiseFilterTags.removeAt(currentRow);
                    updateNoiseFilterList();
                    updateCurrentTagDisplay(); // 立即更新按钮状态和文字
                    if (pointCloudView) pointCloudView->update();
                }
            });

            connect(noiseFilterList, &QListWidget::itemSelectionChanged, filterDialog, [this]() {
                removeNoiseFilterButton->setEnabled(noiseFilterList->currentRow() >= 0);
            });



            // 重新连接spinbox信号，只更新含义说明和当前Tag值显示，不触发点云更新
            auto connectFilterSpinWithTag = [this, updateMeanings, updateCurrentTagDisplay](QSpinBox* spin, const QString& desc) {
                connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), filterDialog, [this, spin, desc, updateMeanings, updateCurrentTagDisplay]() {
                    if (desc == "Bit[7-6]") filterTagVal76 = spin->value();
                    else if (desc == "Bit[5-4]") filterTagVal54 = spin->value();
                    else if (desc == "Bit[3-2]") filterTagVal32 = spin->value();
                    else if (desc == "Bit[1-0]") filterTagVal10 = spin->value();

                    // 只更新含义说明和当前Tag值显示，不触发点云更新
                    updateMeanings();
                    updateCurrentTagDisplay();
                });
            };

            // 重新连接所有spinbox
            connectFilterSpinWithTag(filterSpin76, "Bit[7-6]");
            connectFilterSpinWithTag(filterSpin54, "Bit[5-4]");
            connectFilterSpinWithTag(filterSpin32, "Bit[3-2]");
            connectFilterSpinWithTag(filterSpin10, "Bit[1-0]");

            connect(closeBtn, &QPushButton::clicked, filterDialog, &QDialog::accept);

            // 设置初始状态
            if (showNoiseCheck) showNoiseCheck->setChecked(showNoisePoints);
            if (removeNoiseCheck) removeNoiseCheck->setChecked(removeNoisePoints);

            // 初始化含义说明和当前Tag值显示
            updateMeanings();
            updateCurrentTagDisplay();

            // 初始化滤噪列表
            updateNoiseFilterList();
        }

        filterDialog->show();
        filterDialog->raise();
        filterDialog->activateWindow();
    });
}

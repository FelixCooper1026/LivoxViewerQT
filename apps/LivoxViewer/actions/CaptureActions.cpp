#include "LivoxViewerWindow.h"
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QInputDialog>
#include <QStandardPaths>
void LivoxViewerWindow::createCaptureActions(QMenu* toolsMenu)
{
    // 数据采集子菜单
    QMenu* captureMenu = toolsMenu->addMenu("数据采集");
    QAction* actionCaptureLog = captureMenu->addAction("LOG数据采集...");
    QAction* actionCaptureDebug = captureMenu->addAction("Debug数据采集...");
    QMenu* saveMenu = toolsMenu->addMenu("保存点云");
    QAction* actionCaptureLVX2 = saveMenu->addAction("保存LVX2点云...");
    QAction* actionCapturePCD = saveMenu->addAction("保存PCD点云...");
    QAction* actionCaptureLAS = saveMenu->addAction("保存LAS点云...");
    QAction* actionSaveIMU = toolsMenu->addAction("保存IMU数据...");

    // 采集动作：弹窗输入时长，顶部显示进度条（复用已有captureState.progress，放在状态栏）
    connect(actionCaptureLog, &QAction::triggered, [this]() {
        bool ok = false;
        int sec = QInputDialog::getInt(this, "LOG数据采集", "采集时长(秒):", 300, 10, 86400, 10, &ok);
        if (!ok) return;
        if (!captureState.durationSpin) {
            captureState.durationSpin = new QSpinBox(this);
            captureState.durationSpin->setRange(10, 86400);   // ⭐ 设置最大值为 86400s (24 小时)
        }
        captureState.durationSpin->setValue(sec);
        onStartCaptureLog();
    });
    connect(actionCaptureDebug, &QAction::triggered, [this]() {
        bool ok = false;
        int sec = QInputDialog::getInt(this, "Debug数据采集", "采集时长(秒):", 10, 1, 3600, 1, &ok);
        if (!ok) return;
        if (!captureState.durationSpin) {
            captureState.durationSpin = new QSpinBox(this);
            captureState.durationSpin->setRange(1, 3600);   // ⭐ 设置最大值为 3600
        }
        captureState.durationSpin->setValue(sec);
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
        captureState.pcdSaveDir = targetDir;
        captureState.pcdFramesRemaining = spinCount->value();
        captureState.pcdSaveActive = true;
        captureState.pcdLastSavedTimestamp = 0;
        statusLabelBar->setText(QString("开始保存PCD，共 %1 帧...").arg(captureState.pcdFramesRemaining));
        logMessage(QString("PCD保存目录: %1").arg(QDir::toNativeSeparators(captureState.pcdSaveDir)));
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
        captureState.lasSaveDir = targetDir;
        captureState.lasFramesRemaining = spinCount->value();
        captureState.lasSaveActive = true;
        captureState.lasLastSavedTimestamp = 0;
        statusLabelBar->setText(QString("开始保存LAS，共 %1 帧...").arg(captureState.lasFramesRemaining));
        logMessage(QString("LAS保存目录: %1").arg(QDir::toNativeSeparators(captureState.lasSaveDir)));
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
        if (captureState.progress) {
            captureState.progress->setRange(0, 100);
            captureState.progress->setValue(0);
            captureState.progress->setFormat("录制中 %p% (%v s)");
        }
        // 保存配置并启动录制倒计时
        captureState.secondsRemaining = spinSec->value();
        captureState.totalSeconds = captureState.secondsRemaining;
        captureState.current = CaptureLVX2;
        statusLabelBar->setText("正在录制LVX2...");
        logMessage(QString("LVX2保存路径: %1").arg(QDir::toNativeSeparators(filePath)));
        startLvx2Recording(filePath, captureState.secondsRemaining);
        captureState.timer->start(1000);
    });

    // 调整状态栏进度条长度
    if (captureState.progress) {
        captureState.progress->setFixedWidth(260);
    }

    connect(actionSaveIMU, &QAction::triggered, this, &LivoxViewerWindow::onActionCaptureImuTriggered);
}

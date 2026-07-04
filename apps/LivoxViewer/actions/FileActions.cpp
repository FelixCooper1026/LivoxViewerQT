#include "LivoxViewerWindow.h"
#include "Pcap/PcapPlaybackController.h"
#include "Rosbag/RosbagPlaybackController.h"
#include "ThemeIconUtils.h"

#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>
#include <QStandardPaths>

namespace {

QString selectPlaybackFile(QWidget* parent, const QString& title, const QString& startDir, const QString& nameFilter)
{
    QFileDialog dialog(parent);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setWindowTitle(title);
    dialog.setDirectory(startDir);
    dialog.setNameFilter(nameFilter);
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setAcceptMode(QFileDialog::AcceptOpen);

    if (dialog.exec() != QDialog::Accepted) {
        return QString();
    }

    const QStringList selectedFiles = dialog.selectedFiles();
    return selectedFiles.isEmpty() ? QString() : selectedFiles.first();
}

} // namespace

void LivoxViewerWindow::ensureDataOperationActions()
{
    if (!actionPlayPointCloud) {
        actionPlayPointCloud = new QAction(QStringLiteral("播放点云文件..."), this);
        ThemeIconUtils::setThemedSvgIcon(actionPlayPointCloud, QStringLiteral(":/icons/data_operation_playback.svg"));
        actionPlayPointCloud->setToolTip(QStringLiteral("播放点云文件"));
        connect(actionPlayPointCloud, &QAction::triggered, this, &LivoxViewerWindow::openPointCloudPlaybackFileDialog);
    }

    if (!actionSlamOnline) {
        actionSlamOnline = new QAction(QStringLiteral("SLAM 在线"), this);
        ThemeIconUtils::setThemedSvgIcon(actionSlamOnline, QStringLiteral(":/icons/data_operation_slam_online.svg"));
        actionSlamOnline->setToolTip(QStringLiteral("进入在线 SLAM 模式"));
        connect(actionSlamOnline, &QAction::triggered, this, &LivoxViewerWindow::startOnlineSlamFromMenu);
    }

    if (!actionSlamOffline) {
        actionSlamOffline = new QAction(QStringLiteral("SLAM 离线"), this);
        ThemeIconUtils::setThemedSvgIcon(actionSlamOffline, QStringLiteral(":/icons/data_operation_slam_offline.svg"));
        actionSlamOffline->setToolTip(QStringLiteral("进入离线 SLAM 模式"));
        connect(actionSlamOffline, &QAction::triggered, this, &LivoxViewerWindow::startOfflineSlamFromMenu);
    }

    if (!actionShowImuCharts) {
        actionShowImuCharts = new QAction(QStringLiteral("IMU 数据可视化"), this);
        ThemeIconUtils::setThemedSvgIcon(actionShowImuCharts, QStringLiteral(":/icons/data_operation_imu_visualization.svg"));
        actionShowImuCharts->setToolTip(QStringLiteral("打开 IMU 数据可视化窗口"));
        connect(actionShowImuCharts, &QAction::triggered, this, &LivoxViewerWindow::onActionShowImuCharts);
    }
}

void LivoxViewerWindow::openPointCloudPlaybackFileDialog()
{
    QSettings settings("Livox", "LivoxViewerQT");
    QString lastDir = settings.value("playback/lastDir",
                                     QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
    if (lastDir.isEmpty()) {
        lastDir = QDir::homePath();
    }

    const QString filePath = selectPlaybackFile(
        this,
        "选择点云回放文件",
        lastDir,
        "点云回放文件 (*.lvx *.lvx2 *.pcap *.pcapng *.cap *.bag *.db3 *.yaml *.yml);;"
        "LVX文件 (*.lvx);;"
        "LVX2文件 (*.lvx2);;"
        "Pcap文件 (*.pcap *.pcapng *.cap);;"
        "ROS1 Bag文件 (*.bag);;"
        "ROS2 Bag文件 (*.db3 *.yaml *.yml);;"
        "所有文件 (*.*)");
    if (filePath.isEmpty()) {
        return;
    }

    settings.setValue("playback/lastDir", QFileInfo(filePath).absolutePath());
    if (RosbagPlayback::isSupportedFile(filePath)) {
        loadRosbagPlaybackFile(filePath);
    } else if (PcapPlayback::isSupportedFile(filePath)) {
        loadPcapPlaybackFile(filePath);
    } else if (filePath.endsWith(QStringLiteral(".lvx"), Qt::CaseInsensitive)) {
        loadLvxPlaybackFile(filePath);
    } else {
        loadLvx2PlaybackFile(filePath);
    }
}

void LivoxViewerWindow::createFileActions()
{
    ensureDataOperationActions();

    QAction* actionGenerateConfig = fileMenu->addAction("生成配置文件...");
    fileMenu->addAction(actionPlayPointCloud);
    QAction* actionFormatConvert = fileMenu->addAction("格式转换...");
    QAction* actionPreferences = fileMenu->addAction("首选项...");
    fileMenu->addSeparator();
    exitAction = fileMenu->addAction("退出");

    connect(actionGenerateConfig, &QAction::triggered, this, [this]() {
        showConfigGeneratorDialog();
    });
    connect(actionFormatConvert, &QAction::triggered, this, &LivoxViewerWindow::showFormatConvertDialog);
    connect(actionPreferences, &QAction::triggered, this, &LivoxViewerWindow::showPreferencesDialog);
    connect(exitAction, &QAction::triggered, this, &QWidget::close);
}

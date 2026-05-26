#include "LivoxViewerWindow.h"
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
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

void LivoxViewerWindow::createFileActions()
{
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
    connect(actionPlayLvx2, &QAction::triggered, this, [this]() {
        QSettings settings("Livox", "LivoxViewerQT");
        QString lastDir = settings.value("playback/lastLVX2Dir",
                                         QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
        if (lastDir.isEmpty()) {
            lastDir = QDir::homePath();
        }
        const QString filePath = selectPlaybackFile(this, "选择LVX2点云文件", lastDir, "LVX2点云 (*.lvx2)");
        if (filePath.isEmpty()) {
            return;
        }
        settings.setValue("playback/lastLVX2Dir", QFileInfo(filePath).absolutePath());
        loadLvx2PlaybackFile(filePath);
    });
    connect(actionPlayPcap, &QAction::triggered, this, [this]() {
        QSettings settings("Livox", "LivoxViewerQT");
        QString lastDir = settings.value("playback/lastPcapDir",
                                         QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
        if (lastDir.isEmpty()) {
            lastDir = QDir::homePath();
        }
        const QString filePath = selectPlaybackFile(
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
    connect(exitAction, &QAction::triggered, this, &QWidget::close);
}

#include "LivoxViewerWindow.h"

#include "Pcap/PcapReader.h"

#include <QFileInfo>
#include <QMessageBox>

#include <memory>
#include <thread>

bool LivoxViewerWindow::loadPcapPlaybackFile(const QString& filePath)
{
    closeLvx2Playback(false);

    playbackLoading = true;
    playbackPath = filePath;
    playbackLoadToken++;
    const quint64 currentToken = playbackLoadToken;

    setLvx2PlaybackPlaying(false);
    updateLvx2PlaybackUi();
    if (statusLabelBar) {
        statusLabelBar->setText(QString("正在加载Pcap: %1").arg(QFileInfo(filePath).fileName()));
    }

    std::thread([this, filePath, currentToken]() {
        auto source = std::make_shared<Pcap::PcapReader>();
        const bool ok = source->load(filePath);
        const QString errorMessage = source->errorMessage();

        QMetaObject::invokeMethod(this, [this, currentToken, source, ok, errorMessage]() {
            if (currentToken != playbackLoadToken) {
                return;
            }

            playbackLoading = false;
            if (!ok) {
                playbackPath.clear();
                updateLvx2PlaybackUi();
                if (statusLabelBar) {
                    statusLabelBar->setText(sdk_started ? "已连接 - 采样中" : "就绪");
                }
                QMessageBox::warning(this, "播放Pcap文件", errorMessage);
                return;
            }

            finishPlaybackSourceLoad(source);
        }, Qt::QueuedConnection);
    }).detach();

    return true;
}

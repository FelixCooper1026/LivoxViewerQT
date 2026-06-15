#include "LivoxViewerWindow.h"

#include "Pcap/PcapReader.h"

#include <QFileInfo>
#include <QMessageBox>

#include <memory>
#include <thread>

bool LivoxViewerWindow::loadPcapPlaybackFile(const QString& filePath)
{
    closeLvx2Playback(false);

    playbackState.loading = true;
    playbackState.path = filePath;
    playbackState.loadToken++;
    const quint64 currentToken = playbackState.loadToken;

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
            if (currentToken != playbackState.loadToken) {
                return;
            }

            playbackState.loading = false;
            if (!ok) {
                playbackState.path.clear();
                updateLvx2PlaybackUi();
                updateStatus();
                QMessageBox::warning(this, "播放Pcap文件", errorMessage);
                return;
            }

            finishPlaybackSourceLoad(source);
        }, Qt::QueuedConnection);
    }).detach();

    return true;
}

#include "LivoxViewerWindow.h"

#include "Pcap/PcapReader.h"

#include <QFileInfo>
#include <QMessageBox>

#include <memory>
#include <thread>

bool LivoxViewerWindow::loadPcapPlaybackFile(const QString& filePath)
{
    const int tabId = createOfflinePointCloudTab(filePath);

    playbackState.loading = true;
    playbackState.path = filePath;
    playbackState.loadToken++;
    const quint64 currentToken = playbackState.loadToken;

    setLvx2PlaybackPlaying(false);
    updateLvx2PlaybackUi();
    if (statusLabelBar) {
        statusLabelBar->setText(QString("正在加载Pcap: %1").arg(QFileInfo(filePath).fileName()));
    }
    saveBoundPlaybackState();

    std::thread([this, filePath, tabId, currentToken]() {
        auto source = std::make_shared<Pcap::PcapReader>();
        const bool ok = source->load(filePath);
        const QString errorMessage = source->errorMessage();

        QMetaObject::invokeMethod(this, [this, tabId, currentToken, source, ok, errorMessage]() {
            PlaybackControllerState* state = playbackStateForTab(tabId);
            if (!state || currentToken != state->loadToken) {
                return;
            }

            state->loading = false;
            if (!ok) {
                state->path.clear();
                updateLvx2PlaybackUi();
                updateStatus();
                QMessageBox::warning(this, "播放Pcap文件", errorMessage);
                closeVisualizationTab(tabId);
                return;
            }

            finishPlaybackSourceLoad(tabId, source);
        }, Qt::QueuedConnection);
    }).detach();

    return true;
}
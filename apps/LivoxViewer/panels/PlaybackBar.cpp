#include "LivoxViewerWindow.h"

#include <QIcon>
#include <QStyle>

QWidget* LivoxViewerWindow::createPlaybackBar(QWidget* parent)
{
    playbackState.bar = new QWidget(parent);
        QHBoxLayout* playbackLayout = new QHBoxLayout(playbackState.bar);
        playbackLayout->setContentsMargins(8, 4, 8, 4);
        playbackLayout->setSpacing(6);
        playbackState.playPauseButton = new QPushButton(playbackState.bar);
        playbackState.firstFrameButton = new QPushButton(playbackState.bar);
        playbackState.prevFrameButton = new QPushButton(playbackState.bar);
        playbackState.nextFrameButton = new QPushButton(playbackState.bar);
        playbackState.lastFrameButton = new QPushButton(playbackState.bar);
        playbackState.progressSlider = new QSlider(Qt::Horizontal, playbackState.bar);
        playbackState.progressSlider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        playbackState.speedCombo = new QComboBox(playbackState.bar);
        playbackState.speedCombo->addItems({"x0.1", "x0.5", "x1.0", "x2.0", "x4.0", "x8.0"});
        playbackState.speedCombo->setCurrentText("x1.0");
        playbackState.modeCombo = new QComboBox(playbackState.bar);
        playbackState.modeCombo->addItems({"逐帧播放", "滑窗播放"});
        playbackState.modeCombo->setCurrentIndex(1);
        auto themedSvgIcon = [this](const QString& themeName, QStyle::StandardPixmap fallback) {
            QIcon icon = QIcon::fromTheme(themeName);
            if (icon.isNull()) {
                icon = style()->standardIcon(fallback);
            }
            return icon;
        };
        const int playbackIconSize = fontMetrics().height() + 4;
        playbackState.playPauseButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
        playbackState.firstFrameButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
        playbackState.prevFrameButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
        playbackState.nextFrameButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
        playbackState.lastFrameButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
        playbackState.playPauseButton->setIcon(themedSvgIcon("media-playback-start", QStyle::SP_MediaPlay));
        playbackState.firstFrameButton->setIcon(themedSvgIcon("go-first-view-page", QStyle::SP_MediaSkipBackward));
        playbackState.prevFrameButton->setIcon(themedSvgIcon("media-seek-backward", QStyle::SP_MediaSeekBackward));
        playbackState.nextFrameButton->setIcon(themedSvgIcon("media-seek-forward", QStyle::SP_MediaSeekForward));
        playbackState.lastFrameButton->setIcon(themedSvgIcon("go-last-view-page", QStyle::SP_MediaSkipForward));
        playbackState.playPauseButton->setToolTip("播放/暂停");
        playbackState.firstFrameButton->setToolTip("首帧");
        playbackState.prevFrameButton->setToolTip("上一帧");
        playbackState.nextFrameButton->setToolTip("下一帧");
        playbackState.lastFrameButton->setToolTip("尾帧");
        playbackState.label = new QLabel(playbackState.bar);
        playbackState.closeButton = new QPushButton("关闭文件", playbackState.bar);
        playbackLayout->addWidget(playbackState.firstFrameButton);
        playbackLayout->addWidget(playbackState.prevFrameButton);
        playbackLayout->addWidget(playbackState.playPauseButton);
        playbackLayout->addWidget(playbackState.nextFrameButton);
        playbackLayout->addWidget(playbackState.lastFrameButton);
        playbackLayout->addWidget(playbackState.progressSlider, 1);
        playbackLayout->addWidget(playbackState.speedCombo);
        playbackLayout->addWidget(playbackState.modeCombo);
        playbackLayout->addWidget(playbackState.label);
        playbackLayout->addWidget(playbackState.closeButton);
        playbackState.bar->setVisible(false);
    return playbackState.bar;
}

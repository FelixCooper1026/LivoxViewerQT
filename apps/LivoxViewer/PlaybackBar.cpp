#include "LivoxViewerWindow.h"

#include <QIcon>
#include <QStyle>

QWidget* LivoxViewerWindow::createPlaybackBar(QWidget* parent)
{
    lvx2PlaybackBar = new QWidget(parent);
        QHBoxLayout* playbackLayout = new QHBoxLayout(lvx2PlaybackBar);
        playbackLayout->setContentsMargins(8, 4, 8, 4);
        playbackLayout->setSpacing(6);
        lvx2PlayPauseButton = new QPushButton(lvx2PlaybackBar);
        lvx2FirstFrameButton = new QPushButton(lvx2PlaybackBar);
        lvx2PrevFrameButton = new QPushButton(lvx2PlaybackBar);
        lvx2NextFrameButton = new QPushButton(lvx2PlaybackBar);
        lvx2LastFrameButton = new QPushButton(lvx2PlaybackBar);
        lvx2ProgressSlider = new QSlider(Qt::Horizontal, lvx2PlaybackBar);
        lvx2ProgressSlider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        lvx2SpeedCombo = new QComboBox(lvx2PlaybackBar);
        lvx2SpeedCombo->addItems({"x0.1", "x0.5", "x1.0", "x2.0", "x4.0", "x8.0"});
        lvx2SpeedCombo->setCurrentText("x1.0");
        lvx2PlaybackModeCombo = new QComboBox(lvx2PlaybackBar);
        lvx2PlaybackModeCombo->addItems({"逐帧播放", "滑窗播放"});
        lvx2PlaybackModeCombo->setCurrentIndex(1);
        auto themedSvgIcon = [this](const QString& themeName, QStyle::StandardPixmap fallback) {
            QIcon icon = QIcon::fromTheme(themeName);
            if (icon.isNull()) {
                icon = style()->standardIcon(fallback);
            }
            return icon;
        };
        const int playbackIconSize = fontMetrics().height() + 4;
        lvx2PlayPauseButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
        lvx2FirstFrameButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
        lvx2PrevFrameButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
        lvx2NextFrameButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
        lvx2LastFrameButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
        lvx2PlayPauseButton->setIcon(themedSvgIcon("media-playback-start", QStyle::SP_MediaPlay));
        lvx2FirstFrameButton->setIcon(themedSvgIcon("go-first-view-page", QStyle::SP_MediaSkipBackward));
        lvx2PrevFrameButton->setIcon(themedSvgIcon("media-seek-backward", QStyle::SP_MediaSeekBackward));
        lvx2NextFrameButton->setIcon(themedSvgIcon("media-seek-forward", QStyle::SP_MediaSeekForward));
        lvx2LastFrameButton->setIcon(themedSvgIcon("go-last-view-page", QStyle::SP_MediaSkipForward));
        lvx2PlayPauseButton->setToolTip("播放/暂停");
        lvx2FirstFrameButton->setToolTip("首帧");
        lvx2PrevFrameButton->setToolTip("上一帧");
        lvx2NextFrameButton->setToolTip("下一帧");
        lvx2LastFrameButton->setToolTip("尾帧");
        lvx2PlaybackLabel = new QLabel(lvx2PlaybackBar);
        lvx2CloseButton = new QPushButton("关闭文件", lvx2PlaybackBar);
        playbackLayout->addWidget(lvx2FirstFrameButton);
        playbackLayout->addWidget(lvx2PrevFrameButton);
        playbackLayout->addWidget(lvx2PlayPauseButton);
        playbackLayout->addWidget(lvx2NextFrameButton);
        playbackLayout->addWidget(lvx2LastFrameButton);
        playbackLayout->addWidget(lvx2ProgressSlider, 1);
        playbackLayout->addWidget(lvx2SpeedCombo);
        playbackLayout->addWidget(lvx2PlaybackModeCombo);
        playbackLayout->addWidget(lvx2PlaybackLabel);
        playbackLayout->addWidget(lvx2CloseButton);
        lvx2PlaybackBar->setVisible(false);
    return lvx2PlaybackBar;
}

#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"

QWidget* LivoxViewerWindow::createPlaybackBar(QWidget* parent)
{
    playbackState.bar = new QWidget(parent);
    playbackState.bar->setObjectName(QStringLiteral("PlaybackOverlayBar"));
    playbackState.bar->setAttribute(Qt::WA_StyledBackground, true);
    playbackState.bar->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    playbackState.bar->setStyleSheet(QStringLiteral(
        "QWidget#PlaybackOverlayBar {"
        "  background: palette(window);"
        "  border: 1px solid palette(mid);"
        "  border-radius: 6px;"
        "}"
        "QWidget#PlaybackOverlayBar QLabel {"
        "  color: palette(window-text);"
        "  background: transparent;"
        "}"));

    QVBoxLayout* playbackRootLayout = new QVBoxLayout(playbackState.bar);
    playbackRootLayout->setContentsMargins(8, 4, 8, 4);
    playbackRootLayout->setSpacing(2);

    QHBoxLayout* playbackLayout = new QHBoxLayout();
    playbackLayout->setContentsMargins(0, 0, 0, 0);
    playbackLayout->setSpacing(6);

    playbackState.playPauseButton = new QPushButton(playbackState.bar);
    playbackState.firstFrameButton = new QPushButton(playbackState.bar);
    playbackState.prevFrameButton = new QPushButton(playbackState.bar);
    playbackState.nextFrameButton = new QPushButton(playbackState.bar);
    playbackState.lastFrameButton = new QPushButton(playbackState.bar);
    playbackState.progressSlider = new QSlider(Qt::Horizontal, playbackState.bar);
    playbackState.progressSlider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    playbackState.progressSlider->installEventFilter(this);

    playbackState.speedCombo = new QComboBox(playbackState.bar);
    playbackState.speedCombo->addItems({"x0.1", "x0.5", "x1.0", "x2.0", "x4.0", "x8.0", "x16.0"});
    playbackState.speedCombo->setCurrentText("x1.0");

    playbackState.modeCombo = new QComboBox(playbackState.bar);
    playbackState.modeCombo->addItems({"逐帧播放", "滑窗播放"});
    playbackState.modeCombo->setCurrentIndex(1);

    const int playbackIconSize = fontMetrics().height() + 4;
    playbackState.playPauseButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
    playbackState.firstFrameButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
    playbackState.prevFrameButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
    playbackState.nextFrameButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
    playbackState.lastFrameButton->setIconSize(QSize(playbackIconSize, playbackIconSize));
    ThemeIconUtils::setThemedSvgIcon(playbackState.playPauseButton, QStringLiteral(":/icons/playback_play.svg"));
    ThemeIconUtils::setThemedSvgIcon(playbackState.firstFrameButton, QStringLiteral(":/icons/playback_first.svg"));
    ThemeIconUtils::setThemedSvgIcon(playbackState.prevFrameButton, QStringLiteral(":/icons/playback_previous.svg"));
    ThemeIconUtils::setThemedSvgIcon(playbackState.nextFrameButton, QStringLiteral(":/icons/playback_next.svg"));
    ThemeIconUtils::setThemedSvgIcon(playbackState.lastFrameButton, QStringLiteral(":/icons/playback_last.svg"));
    playbackState.playPauseButton->setToolTip("播放/暂停");
    playbackState.firstFrameButton->setToolTip("首帧");
    playbackState.prevFrameButton->setToolTip("上一帧");
    playbackState.nextFrameButton->setToolTip("下一帧");
    playbackState.lastFrameButton->setToolTip("尾帧");

    playbackState.label = new QLabel(playbackState.bar);
    playbackState.label->setMinimumWidth(0);
    playbackState.label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    playbackState.label->setWordWrap(false);
    playbackState.label->installEventFilter(this);

    playbackLayout->addWidget(playbackState.firstFrameButton);
    playbackLayout->addWidget(playbackState.prevFrameButton);
    playbackLayout->addWidget(playbackState.playPauseButton);
    playbackLayout->addWidget(playbackState.nextFrameButton);
    playbackLayout->addWidget(playbackState.lastFrameButton);
    playbackLayout->addWidget(playbackState.progressSlider, 1);
    playbackLayout->addWidget(playbackState.speedCombo);
    playbackLayout->addWidget(playbackState.modeCombo);
    playbackRootLayout->addLayout(playbackLayout);
    playbackRootLayout->addWidget(playbackState.label);

    playbackState.bar->setVisible(false);
    return playbackState.bar;
}

QWidget* LivoxViewerWindow::createSlamControlBar(QWidget* parent)
{
    slamControlBar = new QWidget(parent);
    slamControlBar->setObjectName(QStringLiteral("SlamOverlayBar"));
    slamControlBar->setAttribute(Qt::WA_StyledBackground, true);
    slamControlBar->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    slamControlBar->setStyleSheet(QStringLiteral(
        "QWidget#SlamOverlayBar {"
        "  background: palette(window);"
        "  border: 1px solid palette(mid);"
        "  border-radius: 6px;"
        "}"
        "QWidget#SlamOverlayBar QLabel {"
        "  color: palette(window-text);"
        "  background: transparent;"
        "}"));

    QHBoxLayout* layout = new QHBoxLayout(slamControlBar);
    layout->setContentsMargins(8, 4, 8, 4);
    layout->setSpacing(6);

    slamStartButton = new QPushButton(QStringLiteral("启动"), slamControlBar);
    slamPauseButton = new QPushButton(QStringLiteral("暂停"), slamControlBar);
    slamStopButton = new QPushButton(QStringLiteral("停止"), slamControlBar);
    slamResetButton = new QPushButton(QStringLiteral("重置"), slamControlBar);
    slamClearButton = new QPushButton(QStringLiteral("清空显示"), slamControlBar);
    slamExportTrajectoryButton = new QPushButton(QStringLiteral("保存轨迹..."), slamControlBar);
    slamExportMapButton = new QPushButton(QStringLiteral("保存全局点云地图..."), slamControlBar);
    slamControlLabel = new QLabel(QStringLiteral("SLAM"), slamControlBar);
    slamControlLabel->setMinimumWidth(0);
    slamControlLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    slamControlLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);

    layout->addWidget(slamStartButton);
    layout->addWidget(slamPauseButton);
    layout->addWidget(slamStopButton);
    layout->addWidget(slamResetButton);
    layout->addWidget(slamClearButton);
    layout->addWidget(slamExportTrajectoryButton);
    layout->addWidget(slamExportMapButton);
    layout->addWidget(slamControlLabel, 1);

    connect(slamStartButton, &QPushButton::clicked, this, &LivoxViewerWindow::startSlamProcessing);
    connect(slamPauseButton, &QPushButton::clicked, this, &LivoxViewerWindow::pauseSlamProcessing);
    connect(slamStopButton, &QPushButton::clicked, this, &LivoxViewerWindow::stopSlamProcessing);
    connect(slamResetButton, &QPushButton::clicked, this, &LivoxViewerWindow::resetSlamProcessing);
    connect(slamClearButton, &QPushButton::clicked, this, &LivoxViewerWindow::clearSlamDisplay);
    connect(slamExportTrajectoryButton, &QPushButton::clicked, this, &LivoxViewerWindow::exportSlamTrajectoryFromDialog);
    connect(slamExportMapButton, &QPushButton::clicked, this, &LivoxViewerWindow::exportSlamGlobalMapFromDialog);

    slamControlBar->setVisible(false);
    return slamControlBar;
}

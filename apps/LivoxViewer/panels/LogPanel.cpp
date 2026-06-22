#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"

void LivoxViewerWindow::createLogPanel()
{
    // 底部：日志 Dock
    logDock = new QDockWidget("日志", this);
    logDock->setObjectName("LogDock");
    logDock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    QWidget* hiddenLogTitleBar = new QWidget(logDock);
    hiddenLogTitleBar->setFixedHeight(0);
    logDock->setTitleBarWidget(hiddenLogTitleBar);

    QWidget* logDockContent = new QWidget(logDock);
    QVBoxLayout* logLayout = new QVBoxLayout(logDockContent);
    logLayout->setContentsMargins(6, 2, 6, 6);
    logLayout->setSpacing(4);

    QWidget* logHeader = new QWidget(logDockContent);
    QHBoxLayout* logHeaderLayout = new QHBoxLayout(logHeader);
    logHeaderLayout->setContentsMargins(0, 0, 0, 0);
    logHeaderLayout->setSpacing(0);
    QLabel* logTitle = new QLabel(QStringLiteral("日志"), logHeader);
    QFont logTitleFont = logTitle->font();
    logTitleFont.setBold(true);
    logTitle->setFont(logTitleFont);
    QToolButton* clearLogButton = new QToolButton(logHeader);
    QToolButton* copyLogButton = new QToolButton(logHeader);
    QCheckBox* autoScrollCheck = new QCheckBox(QStringLiteral("自动滚动"), logHeader);
    clearLogButton->setText(QStringLiteral("清空"));
    copyLogButton->setText(QStringLiteral("复制"));
    autoScrollCheck->setChecked(true);
    ThemeIconUtils::setThemedSvgIcon(clearLogButton, QStringLiteral(":/icons/convert_clear.svg"));
    ThemeIconUtils::setThemedSvgIcon(copyLogButton, QStringLiteral(":/icons/copy.svg"));
    for (QToolButton* button : {clearLogButton, copyLogButton}) {
        button->setAutoRaise(true);
        button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        button->setCursor(Qt::PointingHandCursor);
        button->setIconSize(QSize(fontMetrics().height() + 2, fontMetrics().height() + 2));
        button->setStyleSheet(QStringLiteral(
            "QToolButton {"
            "  border: none;"
            "  background: transparent;"
            "  color: palette(window-text);"
            "  padding: 0;"
            "}"
            "QToolButton:hover {"
            "  color: palette(highlight);"
            "}"
        ));
    }
    autoScrollCheck->setCursor(Qt::PointingHandCursor);

    QWidget* logActions = new QWidget(logHeader);
    QHBoxLayout* logActionsLayout = new QHBoxLayout(logActions);
    logActionsLayout->setContentsMargins(0, 0, 0, 0);
    logActionsLayout->setSpacing(20);
    logActionsLayout->addWidget(clearLogButton);
    logActionsLayout->addWidget(copyLogButton);
    logActionsLayout->addWidget(autoScrollCheck);

    logHeaderLayout->addWidget(logTitle);
    logHeaderLayout->addStretch();
    logHeaderLayout->addWidget(logActions);
    logLayout->addWidget(logHeader);

    QFrame* logFrame = new QFrame(logDockContent);
    logFrame->setObjectName(QStringLiteral("LogFrame"));
    logFrame->setFrameShape(QFrame::NoFrame);
    logFrame->setStyleSheet(QStringLiteral(
        "QFrame#LogFrame {"
        "  border: 0;"
        "  border-top: 1px solid palette(mid);"
        "  background: palette(base);"
        "}"
    ));
    QVBoxLayout* logFrameLayout = new QVBoxLayout(logFrame);
    logFrameLayout->setContentsMargins(6, 4, 6, 4);
    logFrameLayout->setSpacing(0);

    logText = new QTextEdit(logFrame);
    logText->setObjectName(QStringLiteral("LogText"));
    logText->setReadOnly(true);
    logText->setFrameShape(QFrame::NoFrame);
    logText->setMinimumHeight(50);
    logText->setProperty("autoScroll", true);
    logText->setStyleSheet(QStringLiteral(
        "QTextEdit#LogText {"
        "  border: none;"
        "  background: palette(base);"
        "}"
    ));
    connect(clearLogButton, &QPushButton::clicked, logText, &QTextEdit::clear);
    connect(copyLogButton, &QPushButton::clicked, this, [this]() {
        logText->selectAll();
        logText->copy();
    });
    connect(autoScrollCheck, &QCheckBox::toggled, logText, [this](bool checked) {
        logText->setProperty("autoScroll", checked);
    });
    logFrameLayout->addWidget(logText);
    logLayout->addWidget(logFrame);
    logDockContent->setLayout(logLayout);
    logDock->setWidget(logDockContent);
    logDock->setMinimumHeight(50);

    addDockWidget(Qt::BottomDockWidgetArea, logDock);
}

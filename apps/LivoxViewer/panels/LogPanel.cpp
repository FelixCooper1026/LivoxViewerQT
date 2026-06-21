#include "LivoxViewerWindow.h"

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
    logHeaderLayout->setSpacing(8);
    QLabel* logTitle = new QLabel(QStringLiteral("日志"), logHeader);
    QFont logTitleFont = logTitle->font();
    logTitleFont.setBold(true);
    logTitle->setFont(logTitleFont);
    QPushButton* clearLogButton = new QPushButton(QStringLiteral("清空"), logHeader);
    QPushButton* copyLogButton = new QPushButton(QStringLiteral("复制"), logHeader);
    QCheckBox* autoScrollCheck = new QCheckBox(QStringLiteral("自动滚动"), logHeader);
    autoScrollCheck->setChecked(true);
    clearLogButton->setIcon(QIcon(QStringLiteral(":/icons/convert_clear.svg")));
    copyLogButton->setIcon(QIcon(QStringLiteral(":/icons/copy.svg")));
    for (QPushButton* button : {clearLogButton, copyLogButton}) {
        button->setFlat(true);
        button->setCursor(Qt::PointingHandCursor);
        button->setIconSize(QSize(fontMetrics().height() + 2, fontMetrics().height() + 2));
    }
    logHeaderLayout->addWidget(logTitle);
    logHeaderLayout->addStretch();
    logHeaderLayout->addWidget(clearLogButton);
    logHeaderLayout->addWidget(copyLogButton);
    logHeaderLayout->addWidget(autoScrollCheck);
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

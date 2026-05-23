#include "LivoxViewerWindow.h"

void LivoxViewerWindow::createLogPanel()
{
    // 底部：日志 Dock
    logDock = new QDockWidget("日志", this);
    logDock->setObjectName("LogDock");
    logDock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    QWidget* logDockContent = new QWidget(logDock);
    QVBoxLayout* logLayout = new QVBoxLayout(logDockContent);
    logText = new QTextEdit(logDockContent);
    logText->setMinimumHeight(50);
    QPushButton* clearLogButton = new QPushButton("清除日志", logDockContent);
    logLayout->addWidget(logText);
    logLayout->addWidget(clearLogButton);
    logDockContent->setLayout(logLayout);
    logDock->setWidget(logDockContent);
    logDock->setMinimumHeight(50);

    addDockWidget(Qt::BottomDockWidgetArea, logDock);

    connect(clearLogButton, &QPushButton::clicked, [this]() { logText->clear(); });
}

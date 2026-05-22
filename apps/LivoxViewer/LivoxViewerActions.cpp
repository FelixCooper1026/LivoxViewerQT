#include "LivoxViewerWindow.h"

#include <QDateTime>
#include <QDebug>

void LivoxViewerWindow::updateStatus()
{
    // 不再自动显示"已发现x个设备..."的状态
}

void LivoxViewerWindow::logMessage(const QString& message)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString logEntry = QString("[%1] %2").arg(timestamp).arg(message);
    logText->append(logEntry);
    qDebug() << logEntry;
}

#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QPushButton>
#include <QSize>
#include <QVBoxLayout>

static QString nmeaChecksum(const QString& payload)
{
    quint8 cs = 0;
    for (QChar c : payload) cs ^= c.toLatin1();
    return QString("*%1").arg(cs, 2, 16, QChar('0')).toUpper();
}

void LivoxViewerWindow::showTimeSyncDialog()
{
    if (!imuState.timeSyncDialog) {
        imuState.timeSyncDialog = new QDialog(this);
        imuState.timeSyncDialog->setWindowTitle("时间同步");
        imuState.timeSyncDialog->resize(360, 180);

        QVBoxLayout* rootLayout = new QVBoxLayout(imuState.timeSyncDialog);
        rootLayout->setContentsMargins(12, 12, 12, 12);
        rootLayout->setSpacing(10);

        QGroupBox* gpsGroup = new QGroupBox("GPS模拟", imuState.timeSyncDialog);
        QVBoxLayout* gpsLayout = new QVBoxLayout(gpsGroup);
        imuState.gpsSimulateCheck = new QCheckBox("GPS模拟输入", gpsGroup);
        imuState.gpsSimulateCheck->setToolTip("启用 GPS 模拟输入（GPRMC）");
        gpsLayout->addWidget(imuState.gpsSimulateCheck);
        connect(imuState.gpsSimulateCheck, &QCheckBox::toggled, this, &LivoxViewerWindow::onGpsSimulateToggled);
        rootLayout->addWidget(gpsGroup);

        QGroupBox* serialGroup = new QGroupBox("串口转发", imuState.timeSyncDialog);
        QVBoxLayout* serialLayout = new QVBoxLayout(serialGroup);
        imuState.serialEnableCheck = new QCheckBox("串口转发输入", serialGroup);
        imuState.serialEnableCheck->setToolTip("启用串口转发输入（GPRMC）");
        serialLayout->addWidget(imuState.serialEnableCheck);
        connect(imuState.serialEnableCheck, &QCheckBox::toggled, this, &LivoxViewerWindow::onSerialEnableToggled);

        QWidget* serialRow = new QWidget(serialGroup);
        QHBoxLayout* serialRowLayout = new QHBoxLayout(serialRow);
        serialRowLayout->setContentsMargins(0, 0, 0, 0);
        serialRowLayout->setSpacing(6);
        imuState.serialPortCombo = new QComboBox(serialRow);
        imuState.serialPortCombo->setMinimumWidth(0);
        QPushButton* refreshSerialButton = new QPushButton(serialRow);
        ThemeIconUtils::setThemedSvgIcon(refreshSerialButton, QStringLiteral(":/icons/refresh.svg"));
        refreshSerialButton->setIconSize(QSize(fontMetrics().height() + 4, fontMetrics().height() + 4));
        refreshSerialButton->setFixedWidth(fontMetrics().height() + 18);
        refreshSerialButton->setToolTip("刷新串口列表");
        serialRowLayout->addWidget(new QLabel("串口:", serialRow));
        serialRowLayout->addWidget(imuState.serialPortCombo, 1);
        serialRowLayout->addWidget(refreshSerialButton);
        serialLayout->addWidget(serialRow);
        connect(refreshSerialButton, &QPushButton::clicked, this, &LivoxViewerWindow::refreshSerialPorts);
        rootLayout->addWidget(serialGroup);

        QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, imuState.timeSyncDialog);
        connect(buttonBox, &QDialogButtonBox::rejected, imuState.timeSyncDialog, &QDialog::hide);
        rootLayout->addWidget(buttonBox);

        refreshSerialPorts();
    } else {
        refreshSerialPorts();
    }

    imuState.timeSyncDialog->show();
    imuState.timeSyncDialog->raise();
    imuState.timeSyncDialog->activateWindow();
}

void LivoxViewerWindow::onGpsSimulateToggled(bool enabled)
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
        imuState.gpsSimulateCheck->setChecked(false);
        return;
    }

    if (enabled) {
        imuState.gpsTimer->start(1000);
        statusLabelBar->setText("GPS模拟输入已启用");
        logMessage("GPS模拟输入已启用");
    } else {
        imuState.gpsTimer->stop();
        statusLabelBar->setText("GPS模拟输入已关闭");
        logMessage("GPS模拟输入已关闭");
    }
}

void LivoxViewerWindow::onGpsTick()
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) return;
    QDateTime now = QDateTime::currentDateTimeUtc();
    QString timeStr = now.toString("hhmmss");
    QString dateStr = now.toString("ddMMyy");
    QString lat = "3959.000";
    QString lon = "11623.000";
    QString payload = QString("GPRMC,%1,A,%2,N,%3,E,0.0,0.0,%4,,,").arg(timeStr).arg(lat).arg(lon).arg(dateStr);
    QString sentence = "$" + payload + nmeaChecksum(payload) + "\r\n";
    logMessage(QString("GPS模拟报文: %1").arg(sentence.trimmed()));
    QByteArray rmc = sentence.toLatin1();
    SetLivoxLidarRmcSyncTime(currentDevice.handle, rmc.constData(), static_cast<uint16_t>(rmc.size()), nullptr, nullptr);
}

void LivoxViewerWindow::refreshSerialPorts()
{
    imuState.serialPortCombo->clear();
    QList<QSerialPortInfo> infos = QSerialPortInfo::availablePorts();
    if (infos.isEmpty()) {
        imuState.serialPortCombo->addItem("未连接");
        imuState.serialPortCombo->setEnabled(false);
        imuState.serialEnableCheck->setEnabled(false);
    } else {
        for (const QSerialPortInfo& info : infos) {
            imuState.serialPortCombo->addItem(info.portName());
        }
        imuState.serialPortCombo->setEnabled(true);
        imuState.serialEnableCheck->setEnabled(true);
    }
}

void LivoxViewerWindow::onSerialEnableToggled(bool enabled)
{
    if (!enabled) {
        imuState.serialRunning.store(false);
        if (imuState.serialThread.joinable()) imuState.serialThread.join();
        logMessage("串口转发GPS已关闭");
        statusLabelBar->setText("串口转发GPS已关闭");
        return;
    }
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
        imuState.serialEnableCheck->setChecked(false);
        return;
    }
    QString portName = imuState.serialPortCombo ? imuState.serialPortCombo->currentText() : QString();
    if (portName.isEmpty() || portName == "未连接") {
        imuState.serialEnableCheck->setChecked(false);
        return;
    }
    imuState.serialRunning.store(true);
    QMetaObject::invokeMethod(this, [this, portName]() {
        logMessage(QString("串口转发GPS已启用，端口: %1").arg(portName));
        statusLabelBar->setText(QString("串口转发GPS已启用，端口: %1").arg(portName));
    }, Qt::QueuedConnection);

    imuState.serialThread = std::thread([this, portName]() {
        QSerialPort serial;
        serial.setPortName(portName);
        serial.setBaudRate(QSerialPort::Baud9600);
        serial.setDataBits(QSerialPort::Data8);
        serial.setParity(QSerialPort::NoParity);
        serial.setStopBits(QSerialPort::OneStop);
        if (!serial.open(QIODevice::ReadOnly)) {
            QMetaObject::invokeMethod(this, [this, portName]() {
                imuState.serialEnableCheck->setChecked(false);
                logMessage(QString("串口转发GPS启动失败，无法打开端口: %1").arg(portName));
                statusLabelBar->setText(QString("串口转发GPS启动失败，端口: %1").arg(portName));
            }, Qt::QueuedConnection);
            imuState.serialRunning.store(false);
            return;
        }
        QByteArray buffer;
        while (imuState.serialRunning.load()) {
            if (!serial.waitForReadyRead(200)) {
                continue;
            }
            buffer += serial.readAll();
            int idx;
            while ((idx = buffer.indexOf('\n')) >= 0) {
                QByteArray line = buffer.left(idx + 1);
                buffer.remove(0, idx + 1);
                if (line.startsWith("$GP") || line.startsWith("$GN")) {
                    LidarDeviceInfo currentDevice;
                    if (tryGetCurrentDevice(currentDevice) && currentDevice.is_connected) {
                        QString gpsMessage = QString::fromLatin1(line.trimmed());

                        if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC")) {
                            QMetaObject::invokeMethod(this, [this, gpsMessage, portName]() {
                                logMessage(QString("串口转发GPS同步: %1").arg(gpsMessage));
                                statusLabelBar->setText(QString("串口转发GPS同步中... 端口: %1").arg(portName));
                            }, Qt::QueuedConnection);

                            SetLivoxLidarRmcSyncTime(currentDevice.handle, line.constData(), static_cast<uint16_t>(line.size()), nullptr, nullptr);
                        } else {
                            QMetaObject::invokeMethod(this, [this, gpsMessage]() {
                                logMessage(QString("串口转发GPS报文: %1").arg(gpsMessage));
                            }, Qt::QueuedConnection);
                        }
                    }
                }
            }
        }
        serial.close();
    });
}

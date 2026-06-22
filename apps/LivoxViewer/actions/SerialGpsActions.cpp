#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDateTimeEdit>
#include <QDialog>
#include <QDialogButtonBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
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
        imuState.timeSyncDialog->resize(520, 320);
        imuState.timeSyncDialog->setStyleSheet(QStringLiteral(
            "QComboBox:disabled {"
            "  background: palette(alternate-base);"
            "  color: palette(mid);"
            "  border: 1px solid palette(mid);"
            "}"
            "QCheckBox:disabled {"
            "  color: palette(mid);"
            "}"
        ));

        QVBoxLayout* rootLayout = new QVBoxLayout(imuState.timeSyncDialog);
        rootLayout->setContentsMargins(16, 16, 16, 16);
        rootLayout->setSpacing(12);

        QGroupBox* gpsGroup = new QGroupBox("GPS模拟", imuState.timeSyncDialog);
        QVBoxLayout* gpsLayout = new QVBoxLayout(gpsGroup);
        gpsLayout->setSpacing(8);

        QWidget* gpsHeaderRow = new QWidget(gpsGroup);
        QHBoxLayout* gpsHeaderLayout = new QHBoxLayout(gpsHeaderRow);
        gpsHeaderLayout->setContentsMargins(0, 0, 0, 0);
        imuState.gpsSimulateCheck = new QCheckBox("启用 GPS 模拟输入（GPRMC）", gpsHeaderRow);
        gpsHeaderLayout->addWidget(imuState.gpsSimulateCheck);
        gpsHeaderLayout->addStretch();
        gpsLayout->addWidget(gpsHeaderRow);

        QWidget* gpsTimeRow = new QWidget(gpsGroup);
        QHBoxLayout* gpsTimeLayout = new QHBoxLayout(gpsTimeRow);
        gpsTimeLayout->setContentsMargins(0, 0, 0, 0);
        gpsTimeLayout->setSpacing(8);
        QLabel* gpsTimeLabel = new QLabel("起始UTC时间:", gpsTimeRow);
        imuState.gpsDateTimeEdit = new QDateTimeEdit(gpsTimeRow);
        imuState.gpsDateTimeEdit->setDisplayFormat("yyyy-MM-dd HH:mm:ss");
        imuState.gpsDateTimeEdit->setTimeSpec(Qt::UTC);
        imuState.gpsDateTimeEdit->setCalendarPopup(true);
        imuState.gpsDateTimeEdit->setMinimumWidth(190);
        imuState.gpsDateTimeEdit->setDateTime(QDateTime::currentDateTimeUtc());
        imuState.gpsUseCurrentTimeButton = new QPushButton("当前UTC", gpsTimeRow);
        gpsTimeLayout->addWidget(gpsTimeLabel);
        gpsTimeLayout->addWidget(imuState.gpsDateTimeEdit, 1);
        gpsTimeLayout->addWidget(imuState.gpsUseCurrentTimeButton);
        gpsLayout->addWidget(gpsTimeRow);

        QLabel* gpsHintLabel = new QLabel("启动后按所选起始时间每秒递增，并向当前设备发送GPRMC。", gpsGroup);
        gpsHintLabel->setWordWrap(true);
        gpsHintLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));
        gpsLayout->addWidget(gpsHintLabel);

        connect(imuState.gpsSimulateCheck, &QCheckBox::toggled, this, &LivoxViewerWindow::onGpsSimulateToggled);
        connect(imuState.gpsUseCurrentTimeButton, &QPushButton::clicked, imuState.timeSyncDialog, [this]() {
            imuState.gpsDateTimeEdit->setDateTime(QDateTime::currentDateTimeUtc());
        });
        rootLayout->addWidget(gpsGroup);

        QGroupBox* serialGroup = new QGroupBox("串口转发", imuState.timeSyncDialog);
        QVBoxLayout* serialLayout = new QVBoxLayout(serialGroup);
        serialLayout->setSpacing(8);
        imuState.serialEnableCheck = new QCheckBox("启用串口转发输入（GPRMC）", serialGroup);
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
        if (QPushButton* closeButton = buttonBox->button(QDialogButtonBox::Close)) {
            closeButton->setText(QStringLiteral("关闭"));
        }
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
    if (!enabled) {
        imuState.gpsTimer->stop();
        imuState.gpsDateTimeEdit->setEnabled(true);
        imuState.gpsUseCurrentTimeButton->setEnabled(true);
        statusLabelBar->setText("GPS模拟输入已关闭");
        logMessage("GPS模拟输入已关闭");
        return;
    }

    imuState.gpsDateTimeEdit->setEnabled(false);
    imuState.gpsUseCurrentTimeButton->setEnabled(false);

    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) {
        QSignalBlocker blocker(imuState.gpsSimulateCheck);
        imuState.gpsSimulateCheck->setChecked(false);
        imuState.gpsDateTimeEdit->setEnabled(true);
        imuState.gpsUseCurrentTimeButton->setEnabled(true);
        statusLabelBar->setText("GPS模拟启动失败：未选择已连接设备");
        logMessage("GPS模拟启动失败：未选择已连接设备");
        return;
    }

    imuState.gpsSimulationBaseUtc = imuState.gpsDateTimeEdit->dateTime().toUTC();
    imuState.gpsSimulationElapsedSeconds = 0;
    onGpsTick();
    imuState.gpsTimer->start(1000);
    const QString startTime = imuState.gpsSimulationBaseUtc.toString("yyyy-MM-dd HH:mm:ss 'UTC'");
    statusLabelBar->setText(QString("GPS模拟输入已启用: %1").arg(startTime));
    logMessage(QString("GPS模拟输入已启用，起始时间: %1").arg(startTime));
}

void LivoxViewerWindow::onGpsTick()
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) || !currentDevice.is_connected) return;
    const QDateTime simulatedUtc = imuState.gpsSimulationBaseUtc.addSecs(imuState.gpsSimulationElapsedSeconds);
    ++imuState.gpsSimulationElapsedSeconds;
    QString timeStr = simulatedUtc.toString("hhmmss");
    QString dateStr = simulatedUtc.toString("ddMMyy");
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
        imuState.serialPortCombo->addItem("未检测到可用串口，无法启用串口转发输入");
        imuState.serialPortCombo->setEnabled(false);
        imuState.serialEnableCheck->setEnabled(false);
        imuState.serialEnableCheck->setToolTip("未检测到可用串口，无法启用串口转发输入");
    } else {
        for (const QSerialPortInfo& info : infos) {
            imuState.serialPortCombo->addItem(info.portName());
        }
        imuState.serialPortCombo->setEnabled(true);
        imuState.serialEnableCheck->setEnabled(true);
        imuState.serialEnableCheck->setToolTip("启用串口转发输入（GPRMC）");
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
    if (portName.isEmpty() || portName == "未检测到可用串口，无法启用串口转发输入") {
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

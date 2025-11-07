#include "mainwindow.h"
#include <algorithm>
#include <limits>
#include <QColorDialog>
#include <cmath>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QFileDialog>
#include <QDialogButtonBox>
#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QMessageBox>
#include <QDateTime>
#include <QtEndian>
#include <cstring>

// 数值排序项（整行排序时按数值比较当前列）
class NumberItem : public QTableWidgetItem {
public:
    explicit NumberItem(double v, int decimals = 3)
        : QTableWidgetItem(QString::number(v, 'f', decimals)), value(v) {}
    explicit NumberItem(int v)
        : QTableWidgetItem(QString::number(v)), value(static_cast<double>(v)) {}
    bool operator<(const QTableWidgetItem& other) const override {
        const NumberItem* o = dynamic_cast<const NumberItem*>(&other);
        if (o) return value < o->value;
        return QTableWidgetItem::operator<(other);
    }
private:
    double value;
};

// Logger callbacks
static void LoggerStartCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data) {
    MainWindow* w = static_cast<MainWindow*>(client_data);
    if (!w) return;
    if (status != kLivoxLidarStatusSuccess || response == nullptr || response->ret_code != 0) {
        LivoxLidarStartLogger(handle, kLivoxLidarRealTimeLog, LoggerStartCallback, client_data);
        return;
    }
}

static void DebugPointCloudCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data) {
    MainWindow* w = static_cast<MainWindow*>(client_data);
    if (!w) return;
}

void MainWindow::onStartCaptureLog()
{
    if (!currentDevice || !currentDevice->is_connected) { logMessage("设备未连接"); return; }
    if (currentCapture != CaptureNone) return;
    currentCapture = CaptureLog;
    int sec = captureDurationSpin ? captureDurationSpin->value() : 10;
    captureSecondsRemaining = sec;
    captureTotalSeconds = sec;
    logMessage(QString("开始采集日志，时长: %1s").arg(sec));
    captureProgress->setValue(0);
    captureProgress->setFormat("LOG采集中 %p% (%v s)");
    // 启动日志
    SaveLivoxLidarSdkLoggerFile();
    LivoxLidarStartLogger(currentDevice->handle, kLivoxLidarRealTimeLog, LoggerStartCallback, this);
    captureTimer->start(1000);
}

void MainWindow::onStartCaptureDebug()
{
    if (!currentDevice || !currentDevice->is_connected) { logMessage("设备未连接"); return; }
    if (currentCapture != CaptureNone) return;
    currentCapture = CaptureDebug;
    int sec = captureDurationSpin ? captureDurationSpin->value() : 10;
    captureSecondsRemaining = sec;
    captureTotalSeconds = sec;
    captureProgress->setValue(0);
    captureProgress->setFormat("Debug采集中 %p% (%v s)");
    // 开启Debug点云
    SetLivoxLidarDebugPointCloud(currentDevice->handle, true, DebugPointCloudCallback, this);
    captureTimer->start(1000);
}

void MainWindow::onCaptureTick()
{
    if (captureSecondsRemaining <= 0) {
        captureTimer->stop();
        // 停止采集/录制
        if (currentCapture == CaptureLog) {
            LivoxLidarStopLogger(currentDevice->handle, kLivoxLidarRealTimeLog, LoggerStartCallback, this);
        } else if (currentCapture == CaptureDebug) {
            SetLivoxLidarDebugPointCloud(currentDevice->handle, false, DebugPointCloudCallback, this);
        } else if (currentCapture == CaptureLVX2) {
            stopLvx2Recording(true);
        } else if (currentCapture == CaptureIMU) {
            {
                QMutexLocker lk(&imuCsvMutex);
                if (imuCsvFile.isOpen()) imuCsvFile.flush();
                if (imuCsvFile.isOpen()) imuCsvFile.close();
            }
            imuSaveActive = false;
            logMessage("IMU保存完成");
        }
        if (captureProgress) {
            captureProgress->setValue(100);
            captureProgress->setFormat("采集完成");
        }
        statusLabelBar->setText("已连接 - 采样中");
        currentCapture = CaptureNone;
        return;
    }
    //statusLabelBar->setText("数据采集中");

    int total = captureTotalSeconds > 0 ? captureTotalSeconds : (captureDurationSpin ? captureDurationSpin->value() : 1);
    int done = total - captureSecondsRemaining;
    if (done < 0) done = 0;
    if (done > total) done = total;
    int percent = total > 0 ? (done * 100 / total) : 100;
    if (captureProgress) {
        captureProgress->setValue(percent);
        captureProgress->setFormat(QString("%1% (%2 s)").arg(percent).arg(captureSecondsRemaining));
    }
    captureSecondsRemaining--;
}

static QString nmeaChecksum(const QString& payload)
{
    quint8 cs = 0;
    for (QChar c : payload) cs ^= c.toLatin1();
    return QString("*%1").arg(cs, 2, 16, QChar('0')).toUpper();
}

void MainWindow::onGpsSimulateToggled(bool enabled)
{
    if (enabled) {
        gpsTimer->start(1000); // 1 Hz
        statusLabelBar->setText("GPS模拟输入已启用");
        logMessage("GPS模拟输入已启用");
    } else {
        gpsTimer->stop();
        statusLabelBar->setText("GPS模拟输入已关闭");
        logMessage("GPS模拟输入已关闭");
    }
}

void MainWindow::onGpsTick()
{
    if (!currentDevice || !currentDevice->is_connected) return;
    // 生成简化 RMC：$GPRMC,hhmmss,A,lat,N,lon,E,0.0,0.0,ddmmyy,,,*CS\r\n
    QDateTime now = QDateTime::currentDateTimeUtc();
    QString timeStr = now.toString("hhmmss");
    QString dateStr = now.toString("ddMMyy");
    // 使用固定坐标（北京附近），速度/航向为0
    QString lat = "3959.000"; // 39°59.000'
    QString lon = "11623.000"; // 116°23.000'
    QString payload = QString("GPRMC,%1,A,%2,N,%3,E,0.0,0.0,%4,,,").arg(timeStr).arg(lat).arg(lon).arg(dateStr);
    QString sentence = "$" + payload + nmeaChecksum(payload) + "\r\n";
    // 打印到日志区域
    logMessage(QString("GPS模拟报文: %1").arg(sentence.trimmed()));
    QByteArray rmc = sentence.toLatin1();
    SetLivoxLidarRmcSyncTime(currentDevice->handle, rmc.constData(), static_cast<uint16_t>(rmc.size()), nullptr, nullptr);
}

QString MainWindow::buildImuAscii(double gx, double gy, double gz, double ax, double ay, double az) const
{
    auto fw = [](double v){ return QString::number(v, 'f', 3).rightJustified(7, ' '); };
    QString s;
    s += "+----------------------------------+\n";
    s += "|   Gyro(rad/s)   |     Acc(g)     |\n";
    s += "+----------------------------------+\n";
    s += QString("| X:%1       | X:%2      |\n").arg(fw(gx)).arg(fw(ax));
    s += QString("| Y:%1       | Y:%2      |\n").arg(fw(gy)).arg(fw(ay));
    s += QString("| Z:%1       | Z:%2      |\n").arg(fw(gz)).arg(fw(az));
    s += "+----------------------------------+";
    return s;
}

void MainWindow::onImuDisplayButtonClicked()
{
    // Toggle 2 Hz text-only updater
    if (imuDisplayRunning.exchange(!imuDisplayRunning.load())) {
        // turned off
        if (imuDisplayThread.joinable()) {
            imuDisplayThread.detach(); // avoid blocking GUI
        }
        if (imuAsciiLabel) {
            imuAsciiLabel->setText(buildImuAscii(0.0,0.0,0.0,0.0,0.0,0.0));
        }
        if (imuDisplayButton) imuDisplayButton->setText("显示IMU数据");
        return;
    }

    if (imuDisplayButton) imuDisplayButton->setText("停止IMU显示");

    // Start or restart text updater thread (2 Hz)
    if (imuDisplayThread.joinable()) {
        imuDisplayThread.detach();
    }
    imuDisplayThread = std::thread([this]() {
        while (imuDisplayRunning.load()) {
            float gx=0, gy=0, gz=0, ax=0, ay=0, az=0; bool have=false;
            {
                QMutexLocker lk(&imuSampleMutex);
                if (latestImu.have) {
                    gx = latestImu.gx; gy = latestImu.gy; gz = latestImu.gz;
                    ax = latestImu.ax; ay = latestImu.ay; az = latestImu.az;
                    have = true;
                }
            }
            if (have) {
                QString text = buildImuAscii(gx, gy, gz, ax, ay, az);
                QMetaObject::invokeMethod(this, [this, text]() {
                    if (imuAsciiLabel) imuAsciiLabel->setText(text);
                });
            }
            for (int i=0; i<10 && imuDisplayRunning.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    });
}

void MainWindow::onActionShowImuCharts()
{
    if (imuChartWindow && imuChartWindow->isVisible()) {
        imuChartWindow->raise();
        imuChartWindow->activateWindow();
        return;
    }
    // Build chart window
    imuChartWindow = new QWidget(this, Qt::Window);
    imuChartWindow->setAttribute(Qt::WA_DeleteOnClose);
    imuChartWindow->setWindowTitle("IMU数据曲线");
    QVBoxLayout* layout = new QVBoxLayout(imuChartWindow);

    // Gyro chart
    gyroChart = new QChart();
    gyroSeriesX = new QLineSeries(); gyroSeriesX->setName("gx");
    gyroSeriesY = new QLineSeries(); gyroSeriesY->setName("gy");
    gyroSeriesZ = new QLineSeries(); gyroSeriesZ->setName("gz");
    gyroChart->addSeries(gyroSeriesX);
    gyroChart->addSeries(gyroSeriesY);
    gyroChart->addSeries(gyroSeriesZ);
    gyroAxisX = new QValueAxis(); gyroAxisX->setTitleText("时间 (s)");
    gyroAxisY = new QValueAxis(); gyroAxisY->setTitleText("角速度 (rad/s)"); gyroAxisY->setRange(-50, 50);
    gyroChart->addAxis(gyroAxisX, Qt::AlignBottom);
    gyroChart->addAxis(gyroAxisY, Qt::AlignLeft);
    for (auto s : {gyroSeriesX, gyroSeriesY, gyroSeriesZ}) { s->attachAxis(gyroAxisX); s->attachAxis(gyroAxisY); }
    gyroChart->legend()->setVisible(true);
    gyroChartView = new QChartView(gyroChart, imuChartWindow);
    gyroChartView->setRenderHint(QPainter::Antialiasing);

    // Acc chart
    accChart = new QChart();
    accSeriesX = new QLineSeries(); accSeriesX->setName("ax");
    accSeriesY = new QLineSeries(); accSeriesY->setName("ay");
    accSeriesZ = new QLineSeries(); accSeriesZ->setName("az");
    accChart->addSeries(accSeriesX);
    accChart->addSeries(accSeriesY);
    accChart->addSeries(accSeriesZ);
    accAxisX = new QValueAxis(); accAxisX->setTitleText("时间 (s)");
    accAxisY = new QValueAxis(); accAxisY->setTitleText("加速度 (g)"); accAxisY->setRange(-4, 4);
    accChart->addAxis(accAxisX, Qt::AlignBottom);
    accChart->addAxis(accAxisY, Qt::AlignLeft);
    for (auto s : {accSeriesX, accSeriesY, accSeriesZ}) { s->attachAxis(accAxisX); s->attachAxis(accAxisY); }
    accChart->legend()->setVisible(true);
    accChartView = new QChartView(accChart, imuChartWindow);
    accChartView->setRenderHint(QPainter::Antialiasing);

    layout->addWidget(gyroChartView);
    layout->addWidget(accChartView);
    imuChartWindow->setLayout(layout);

    // Start or restart chart updater (20 Hz) without blocking GUI
    if (imuChartRunning.exchange(true)) {
        if (imuChartThread.joinable()) imuChartThread.detach();
    }
    imuChartThread = std::thread([this]() {
        double t = 0.0;
        const double dt = 0.05; // 20 Hz
        const double windowSec = 10.0;
        const int maxPoints = static_cast<int>(windowSec / dt);
        while (imuChartRunning.load()) {
            float gx=0, gy=0, gz=0, ax=0, ay=0, az=0; bool have=false;
            {
                QMutexLocker lk(&imuSampleMutex);
                if (latestImu.have) {
                    gx = latestImu.gx; gy = latestImu.gy; gz = latestImu.gz;
                    ax = latestImu.ax; ay = latestImu.ay; az = latestImu.az;
                    have = true;
                }
            }
            if (have) {
                QMetaObject::invokeMethod(this, [this, gx, gy, gz, ax, ay, az, t, maxPoints, dt]() {
                    auto pushPoint = [&](QLineSeries* s, double x, double y){ if (!s) return; s->append(x, y); if (s->count() > maxPoints) s->removePoints(0, s->count() - maxPoints); };
                    pushPoint(gyroSeriesX, t, gx); pushPoint(gyroSeriesY, t, gy); pushPoint(gyroSeriesZ, t, gz);
                    pushPoint(accSeriesX, t, ax); pushPoint(accSeriesY, t, ay); pushPoint(accSeriesZ, t, az);
                    if (gyroAxisX) gyroAxisX->setRange(std::max(0.0, t - (maxPoints - 1) * dt), t);
                    if (accAxisX) accAxisX->setRange(std::max(0.0, t - (maxPoints - 1) * dt), t);
                });
            }
            t += dt;
            for (int i=0; i<5 && imuChartRunning.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    });

    // Stop chart when window closed
    connect(imuChartWindow, &QObject::destroyed, this, [this]() {
        imuChartRunning.store(false);
        if (imuChartThread.joinable()) imuChartThread.detach();
        gyroChart = nullptr; gyroChartView = nullptr; gyroSeriesX = gyroSeriesY = gyroSeriesZ = nullptr; gyroAxisX = gyroAxisY = nullptr;
        accChart = nullptr; accChartView = nullptr; accSeriesX = accSeriesY = accSeriesZ = nullptr; accAxisX = accAxisY = nullptr;
        imuChartWindow = nullptr;
    });

    imuChartWindow->resize(900, 600);
    imuChartWindow->show();
}

void MainWindow::onActionCaptureImuTriggered()
{
    if (!currentDevice || !currentDevice->is_connected) {
        QMessageBox::warning(this, "保存IMU数据", "设备未连接");
        return;
    }
    if (currentCapture != CaptureNone) {
        QMessageBox::warning(this, "保存IMU数据", "当前已有采集任务在进行中");
        return;
    }
    // 检查IMU数据发送是否开启
    QWidget* ctrl = paramControls.value(kKeyImuDataEn, nullptr);
    QComboBox* imuCombo = qobject_cast<QComboBox*>(ctrl);
    if (!imuCombo || imuCombo->currentIndex() != 1) {
        QMessageBox::warning(this, "保存IMU数据", "IMU数据发送未开启！");
        return;
    }
    QDialog dlg(this);
    dlg.setWindowTitle("保存IMU数据");
    QVBoxLayout* v = new QVBoxLayout(&dlg);
    // 路径
    QWidget* row1 = new QWidget(&dlg);
    QHBoxLayout* h1 = new QHBoxLayout(row1);
    h1->setContentsMargins(0,0,0,0);
    QLabel* lblPath = new QLabel("请选择保存路径:", row1);
    QLineEdit* editPath = new QLineEdit(row1);
    QPushButton* btnBrowse = new QPushButton("选择", row1);
    h1->addWidget(lblPath);
    h1->addSpacing(8);
    h1->addWidget(editPath, 1);
    h1->addSpacing(8);
    h1->addWidget(btnBrowse);
    v->addWidget(row1);
    // 时长
    QWidget* row2 = new QWidget(&dlg);
    QHBoxLayout* h2 = new QHBoxLayout(row2);
    h2->setContentsMargins(0,0,0,0);
    QLabel* lblSec = new QLabel("保存时长(s):", row2);
    QSpinBox* spinSec = new QSpinBox(row2);
    spinSec->setRange(10, 3600);
    spinSec->setSingleStep(10);
    spinSec->setValue(30);
    h2->addWidget(lblSec);
    h2->addSpacing(8);
    h2->addWidget(spinSec);
    h2->addStretch();
    v->addWidget(row2);
    // 按钮
    QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    v->addWidget(box);
    connect(btnBrowse, &QPushButton::clicked, &dlg, [editPath, this]() {
        QString dir = QFileDialog::getExistingDirectory(this, "选择保存目录", QDir::homePath());
        if (!dir.isEmpty()) editPath->setText(dir);
    });
    connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    if (dlg.exec() != QDialog::Accepted) return;
    QString baseDir = editPath->text().trimmed();
    if (baseDir.isEmpty()) {
        QMessageBox::warning(this, "保存IMU数据", "请选择保存路径");
        return;
    }
    QString sn = currentDevice ? currentDevice->sn : QString("Unknown");
    QString targetDir = QDir(baseDir).filePath(QString("IMU_%1").arg(sn));
    QDir().mkpath(targetDir);
    QString startTime = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString filePath = QDir(targetDir).filePath(QString("%1_%2.csv").arg(sn, startTime));
    // 打开CSV
    imuCsvFile.setFileName(filePath);
    if (!imuCsvFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        QMessageBox::warning(this, "保存IMU数据", "无法创建CSV文件");
        return;
    }
    {
        QMutexLocker lk(&imuCsvMutex);
        QTextStream ts(&imuCsvFile);
        ts << "timestamp_ns,gx,gy,gz,ax,ay,az\n";
    }
    // 配置进度条
    if (captureProgress) {
        captureProgress->setRange(0, 100);
        captureProgress->setValue(0);
        captureProgress->setFormat("IMU采集中 %p% (%v s)");
    }
    // 启动计时
    captureSecondsRemaining = spinSec->value();
    captureTotalSeconds = captureSecondsRemaining;
    currentCapture = CaptureIMU;
    imuSaveActive = true;
    statusLabelBar->setText("正在保存IMU数据...");
    logMessage(QString("IMU保存路径: %1").arg(QDir::toNativeSeparators(filePath)));
    captureTimer->start(1000);
}

void MainWindow::appendImuCsvRow(quint64 timestamp_ns, float gx, float gy, float gz, float ax, float ay, float az)
{
    QMutexLocker lk(&imuCsvMutex);
    if (!imuSaveActive || !imuCsvFile.isOpen()) return;
    QTextStream ts(&imuCsvFile);
    ts.setRealNumberNotation(QTextStream::FixedNotation);
    ts.setRealNumberPrecision(6);
    ts << timestamp_ns << ',' << gx << ',' << gy << ',' << gz << ',' << ax << ',' << ay << ',' << az << '\n';
}

void MainWindow::refreshSerialPorts()
{
    serialPortCombo->clear();
    QList<QSerialPortInfo> infos = QSerialPortInfo::availablePorts();
    if (infos.isEmpty()) {
        serialPortCombo->addItem("未连接");
        serialPortCombo->setEnabled(false);
        serialEnableCheck->setEnabled(false);
    } else {
        for (const QSerialPortInfo& info : infos) {
            serialPortCombo->addItem(info.portName());
        }
        serialPortCombo->setEnabled(true);
        serialEnableCheck->setEnabled(true);
    }
}

void MainWindow::onSerialEnableToggled(bool enabled)
{
    if (!enabled) {
        serialRunning.store(false);
        if (serialThread.joinable()) serialThread.join();
        // 记录串口停止日志
        logMessage("串口转发GPS已关闭");
        // 更新状态栏
        statusLabelBar->setText("串口转发GPS已关闭");
        return;
    }
    if (!currentDevice || !currentDevice->is_connected) {
        serialEnableCheck->setChecked(false);
        return;
    }
    QString portName = serialPortCombo ? serialPortCombo->currentText() : QString();
    if (portName.isEmpty() || portName == "未连接") {
        serialEnableCheck->setChecked(false);
        return;
    }
    serialRunning.store(true);
    // 记录串口启动日志并更新状态栏
    QMetaObject::invokeMethod(this, [this, portName]() {
        logMessage(QString("串口转发GPS已启用，端口: %1").arg(portName));
        statusLabelBar->setText(QString("串口转发GPS已启用，端口: %1").arg(portName));
    }, Qt::QueuedConnection);
    
    serialThread = std::thread([this, portName]() {
        QSerialPort serial;
        serial.setPortName(portName);
        serial.setBaudRate(QSerialPort::Baud9600);
        serial.setDataBits(QSerialPort::Data8);
        serial.setParity(QSerialPort::NoParity);
        serial.setStopBits(QSerialPort::OneStop);
        if (!serial.open(QIODevice::ReadOnly)) {
            QMetaObject::invokeMethod(this, [this, portName]() { 
                serialEnableCheck->setChecked(false); 
                logMessage(QString("串口转发GPS启动失败，无法打开端口: %1").arg(portName));
                statusLabelBar->setText(QString("串口转发GPS启动失败，端口: %1").arg(portName));
            }, Qt::QueuedConnection);
            serialRunning.store(false);
            return;
        }
        QByteArray buffer;
        while (serialRunning.load()) {
            if (!serial.waitForReadyRead(200)) {
                continue;
            }
            buffer += serial.readAll();
            int idx;
            while ((idx = buffer.indexOf('\n')) >= 0) {
                QByteArray line = buffer.left(idx + 1);
                buffer.remove(0, idx + 1);
                // 查找GPS报文（RMC、GGA、GSA、GSV等）
                if (line.startsWith("$GP") || line.startsWith("$GN")) {
                    if (currentDevice && currentDevice->is_connected) {
                        QString gpsMessage = QString::fromLatin1(line.trimmed());
                        
                        // 对于RMC报文，进行时间同步
                        if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC")) {
                            // 打印GPS同步报文到日志并更新状态栏
                            QMetaObject::invokeMethod(this, [this, gpsMessage, portName]() {
                                logMessage(QString("串口转发GPS同步: %1").arg(gpsMessage));
                                statusLabelBar->setText(QString("串口转发GPS同步中... 端口: %1").arg(portName));
                            }, Qt::QueuedConnection);
                            
                            SetLivoxLidarRmcSyncTime(currentDevice->handle, line.constData(), static_cast<uint16_t>(line.size()), nullptr, nullptr);
                        } else {
                            // 打印其他GPS报文到日志
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

void MainWindow::onFrameIntervalChanged(int ms)
{
    if (ms < 50) ms = 50;
    frameIntervalMs = static_cast<uint64_t>(ms);
    logMessage(QString("点云积分时间已设置为 %1 ms").arg(ms));
}

void MainWindow::processPointCloudPacket(uint32_t handle, const LivoxLidarEthernetPacket* packet)
{
    if (!packet || packet->dot_num == 0) {
        return;
    }
    
    // 解析时间戳
    uint64_t timestamp = parseTimestamp(packet->timestamp);
    
    // 创建点云帧
    PointCloudFrame frame;
    frame.timestamp = timestamp;
    frame.device_handle = handle;
    
    // 根据数据类型解析点云数据
    if (packet->data_type == kLivoxLidarCartesianCoordinateHighData) {
        LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)packet->data;
        for (uint32_t i = 0; i < packet->dot_num; i++) {
            Point3D point;
            point.x = p_point_data[i].x / 1000.0f; // 转换为米
            point.y = p_point_data[i].y / 1000.0f;
            point.z = p_point_data[i].z / 1000.0f;
            point.reflectivity = p_point_data[i].reflectivity;
            point.tag = p_point_data[i].tag;
            
            frame.points.append(point);
        }
    }
    else if (packet->data_type == kLivoxLidarCartesianCoordinateLowData) {
        LivoxLidarCartesianLowRawPoint *p_point_data = (LivoxLidarCartesianLowRawPoint *)packet->data;
        for (uint32_t i = 0; i < packet->dot_num; i++) {
            Point3D point;
            point.x = p_point_data[i].x / 100.0f; // 转换为米
            point.y = p_point_data[i].y / 100.0f;
            point.z = p_point_data[i].z / 100.0f;
            point.reflectivity = p_point_data[i].reflectivity;
            point.tag = p_point_data[i].tag;
            
            frame.points.append(point);
        }
    }
    else if (packet->data_type == kLivoxLidarSphericalCoordinateData) {
        LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint *)packet->data;
        for (uint32_t i = 0; i < packet->dot_num; i++) {
            Point3D point;
            
            // 球坐标转笛卡尔坐标
            float depth = p_point_data[i].depth / 1000.0f; // 转换为米
            float theta = p_point_data[i].theta / 100.0f * M_PI / 180.0f; // 转换为弧度
            float phi = p_point_data[i].phi / 100.0f * M_PI / 180.0f; // 转换为弧度
            
            // 深度投影：启用且设置了投影距离（>0）时，使用该距离替换 depth
            if (projectionDepthEnabled && projectionDepthMeters > 0.0f) {
                depth = projectionDepthMeters;
            }
            
            // 平面投影模式时，设置投影深度为平面投影半径
            if (planarProjectionEnabled && projectionDepthMeters <= 0.0f) {
                depth = planarProjectionRadius;
            }
            
            // 平面投影：如果启用平面投影，将球坐标转换为平面坐标
            if (planarProjectionEnabled) {
                // 等距圆柱投影：将球面展开为平面
                // phi (方位角) 映射到 X 轴，theta (仰角) 映射到 Y 轴
                float phi_deg = phi * 180.0f / M_PI;  // 转换为度
                float theta_deg = theta * 180.0f / M_PI;  // 转换为度
                
                // 将方位角映射到 [-180, 180] 度范围
                if (phi_deg > 180.0f) phi_deg -= 360.0f;
                
                // 将仰角映射到 [-90, 90] 度范围（完整球面）
                theta_deg = 90.0f - theta_deg; // 转换坐标系，使0°为水平，正值为上方，负值为下方
                
                // 计算平面坐标
                point.x = planarProjectionRadius * phi_deg / 180.0f;  // X轴：方位角
                point.y = planarProjectionRadius * theta_deg / 90.0f;  // Y轴：仰角
                point.z = 0.0f;  // 平面投影时Z设为0
            } else {
                // 原始球坐标转笛卡尔坐标
                point.x = depth * sin(theta) * cos(phi);
                point.y = depth * sin(theta) * sin(phi);
                point.z = depth * cos(theta);
            }
            point.reflectivity = p_point_data[i].reflectivity;
            point.tag = p_point_data[i].tag;
            
            frame.points.append(point);
        }
    }
    
    // 推入待处理队列，记录最新时间戳
    {
        QMutexLocker locker(&frameMutex);
        pendingFrames[handle].enqueue(frame);
        lastSeenTimestamp[handle] = timestamp;
    }
}

uint64_t MainWindow::parseTimestamp(const uint8_t* timestamp)
{
    // 按小端序解析时间戳
    uint64_t result = 0;
    for (int i = 7; i >= 0; --i) {
        result = (result << 8) | timestamp[i];
    }
    return result;
}

void MainWindow::publishPointCloudFrame(const PointCloudFrame& frame)
{
    // 在主线程中更新点云显示
    QMetaObject::invokeMethod(this, [this, frame]() {
        pointCloudWidget->updatePointCloud(frame);
    }, Qt::QueuedConnection);
}

void MainWindow::calculatePointColor(uint8_t reflectivity, uint8_t tag, float& r, float& g, float& b)
{
    // 参考Livox Viewer的着色逻辑（Livox color-coding strategy）
    uint8_t cur_reflectivity = reflectivity;
    uint8_t tag_value = tag;
    
    if (cur_reflectivity < 30) {
        r = 0;
        g = static_cast<float>(cur_reflectivity * 255 / 30) / 255.0f;
        b = 1.0f;
    }
    else if (cur_reflectivity < 90) {
        r = 0;
        g = 1.0f;
        b = static_cast<float>((90 - cur_reflectivity) * 255 / 60) / 255.0f;
    }
    else if (cur_reflectivity < 150) {
        r = static_cast<float>((cur_reflectivity - 90) * 255 / 60) / 255.0f;
        g = 1.0f;
        b = 0;
    }
    else {
        r = 1.0f;
        g = static_cast<float>((255 - cur_reflectivity) * 255 / (256 - 150)) / 255.0f;
        b = 0;
    }
}

static inline void calcColorDistance(const Point3D& p, float minD, float maxD, float& r, float& g, float& b)
{
    const float dx = p.x, dy = p.y, dz = p.z;
    float d = std::sqrt(dx*dx + dy*dy + dz*dz);
    float t = 0.0f;
    if (maxD > minD) t = (d - minD) / (maxD - minD);
    t = std::clamp(t, 0.0f, 1.0f);
    // 蓝->青->绿->黄->红
    if (t < 0.25f)      { r = 0.0f;           g = t/0.25f;     b = 1.0f; }
    else if (t < 0.5f)  { r = 0.0f;           g = 1.0f;        b = 1.0f - (t-0.25f)/0.25f; }
    else if (t < 0.75f) { r = (t-0.5f)/0.25f; g = 1.0f;        b = 0.0f; }
    else                { r = 1.0f;           g = 1.0f-(t-0.75f)/0.25f; b = 0.0f; }
}

static inline void calcColorElevation(const Point3D& p, float minZ, float maxZ, float& r, float& g, float& b)
{
    float t = 0.0f;
    if (maxZ > minZ) t = (p.z - minZ) / (maxZ - minZ);
    t = std::clamp(t, 0.0f, 1.0f);
    // 低->高: 蓝->红
    r = t;
    g = 0.0f;
    b = 1.0f - t;
}

static inline void calcColorSolid(const QColor& color, float& r, float& g, float& b)
{
    r = color.redF();
    g = color.greenF();
    b = color.blueF();
}

static inline quint16 clampU16(int v) { return quint16(std::max(0, std::min(65535, v))); }

bool MainWindow::savePointCloudAsLAS(const QString& filePath, const QVector<Point3D>& points)
{
    QFile f(filePath);
    if (!f.open(QIODevice::WriteOnly)) return false;

    // LAS 1.2 header (little-endian), Point Data Record Format 0
    // We will use scale (0.001) and offset 0 for simplicity; compute bbox
    double scaleX = 0.001, scaleY = 0.001, scaleZ = 0.001;
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    double maxZ = -std::numeric_limits<double>::max();
    for (const Point3D& p : points) {
        if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
        if (p.z < minZ) minZ = p.z; if (p.z > maxZ) maxZ = p.z;
    }
    double offX = 0.0, offY = 0.0, offZ = 0.0; // offsets set to 0

    QByteArray header(227, 0); // LAS 1.2 header size
    // File Signature "LASF"
    header[0] = 'L'; header[1] = 'A'; header[2] = 'S'; header[3] = 'F';
    // File Source ID, Global Encoding -> leave 0
    // Project ID GUIDs -> zeros
    // Version Major/Minor
    header[24] = 1; // version major
    header[25] = 2; // version minor
    // System Identifier (32 bytes)
    QByteArray sys = QByteArray("LivoxViewerQT"); sys = sys.leftJustified(32, '\0', true);
    std::copy(sys.begin(), sys.end(), header.begin() + 26);
    // Generating Software (32 bytes)
    QByteArray gen = QByteArray("LVX"); gen = gen.leftJustified(32, '\0', true);
    std::copy(gen.begin(), gen.end(), header.begin() + 58);
    // File Creation Day/Year -> leave 0
    // Header Size
    qToLittleEndian<quint16>(227, reinterpret_cast<uchar*>(header.data() + 94));
    // Offset to point data: header (227) + no VLRs (0)
    qToLittleEndian<quint32>(227, reinterpret_cast<uchar*>(header.data() + 96));
    // Number of Variable Length Records
    qToLittleEndian<quint32>(0, reinterpret_cast<uchar*>(header.data() + 100));
    // Point Data Format
    header[104] = 0; // format 0
    // Point Data Record Length (bytes) -> 20 for format 0
    qToLittleEndian<quint16>(20, reinterpret_cast<uchar*>(header.data() + 105));
    // Legacy number of point records
    qToLittleEndian<quint32>(static_cast<quint32>(points.size()), reinterpret_cast<uchar*>(header.data() + 107));
    // Legacy number of points by return (5 x uint32) -> set first = count
    qToLittleEndian<quint32>(static_cast<quint32>(points.size()), reinterpret_cast<uchar*>(header.data() + 111));
    // Scale Factors (X,Y,Z) at offsets 131,139,147 as doubles
    qToLittleEndian<double>(scaleX, reinterpret_cast<uchar*>(header.data() + 131));
    qToLittleEndian<double>(scaleY, reinterpret_cast<uchar*>(header.data() + 139));
    qToLittleEndian<double>(scaleZ, reinterpret_cast<uchar*>(header.data() + 147));
    // Offsets (X,Y,Z) as doubles
    qToLittleEndian<double>(offX, reinterpret_cast<uchar*>(header.data() + 155));
    qToLittleEndian<double>(offY, reinterpret_cast<uchar*>(header.data() + 163));
    qToLittleEndian<double>(offZ, reinterpret_cast<uchar*>(header.data() + 171));
    // Max/Min (X,Y,Z) as doubles
    qToLittleEndian<double>(maxX, reinterpret_cast<uchar*>(header.data() + 179));
    qToLittleEndian<double>(minX, reinterpret_cast<uchar*>(header.data() + 187));
    qToLittleEndian<double>(maxY, reinterpret_cast<uchar*>(header.data() + 195));
    qToLittleEndian<double>(minY, reinterpret_cast<uchar*>(header.data() + 203));
    qToLittleEndian<double>(maxZ, reinterpret_cast<uchar*>(header.data() + 211));
    qToLittleEndian<double>(minZ, reinterpret_cast<uchar*>(header.data() + 219));

    if (f.write(header) != header.size()) { f.close(); return false; }

    // Write points (Format 0): X,Y,Z as int32, Intensity uint16, flags+classification etc.
    QByteArray rec(20, 0);
    for (const Point3D& p : points) {
        // Convert to scaled integer: integer = round((coord - offset)/scale)
        qint32 xi = qint32(std::llround((p.x - offX) / scaleX));
        qint32 yi = qint32(std::llround((p.y - offY) / scaleY));
        qint32 zi = qint32(std::llround((p.z - offZ) / scaleZ));
        qToLittleEndian<qint32>(xi, reinterpret_cast<uchar*>(rec.data() + 0));
        qToLittleEndian<qint32>(yi, reinterpret_cast<uchar*>(rec.data() + 4));
        qToLittleEndian<qint32>(zi, reinterpret_cast<uchar*>(rec.data() + 8));
        // Intensity
        qToLittleEndian<quint16>(clampU16(int(p.reflectivity)), reinterpret_cast<uchar*>(rec.data() + 12));
        // Return flags (1), classification (1), scan angle (1), user data (1) -> put tag into user data
        rec[14] = 1;             // return number bits -> 1
        rec[15] = 1;             // classification -> unclassified
        rec[16] = 0;             // scan angle rank
        rec[17] = static_cast<char>(p.tag); // user data stores tag
        // Point source ID (uint16)
        qToLittleEndian<quint16>(0, reinterpret_cast<uchar*>(rec.data() + 18));
        if (f.write(rec) != rec.size()) { f.close(); return false; }
    }

    f.close();
    return true;
}

bool MainWindow::savePointCloudAsPCD(const QString& filePath, const QVector<Point3D>& points)
{
    QFile f(filePath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        return false;
    }
    QTextStream out(&f);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(6);
    // ASCII PCD header with reflectivity(intensity) and tag
    out << "# .PCD v0.7 - Point Cloud Data file\n";
    out << "VERSION 0.7\n";
    out << "FIELDS x y z intensity tag\n";
    out << "SIZE 4 4 4 4 4\n";
    out << "TYPE F F F F F\n";
    out << "COUNT 1 1 1 1 1\n";
    out << "WIDTH " << points.size() << "\n";
    out << "HEIGHT 1\n";
    out << "VIEWPOINT 0 0 0 1 0 0 0\n";
    out << "POINTS " << points.size() << "\n";
    out << "DATA ascii\n";
    for (const Point3D& p : points) {
        out << p.x << ' ' << p.y << ' ' << p.z << ' ' << int(p.reflectivity) << ' ' << int(p.tag) << "\n";
    }
    f.close();
    return true;
}

void MainWindow::onRenderTick()
{
	// 暂停可视化模式：停止更新点云缓冲，但仍按固定刷新率重绘以跟随相机/叠加层
	if (!pointCloudVisualizationEnabled) {
		{
			QMutexLocker locker(&frameMutex);
			for (auto it = pendingFrames.begin(); it != pendingFrames.end(); ++it) {
				it.value().clear();
			}
		}
		if (pointCloudWidget) {
			pointCloudWidget->update();
		}
		return;
	}
	
	// 测距模式：暂停点云可视化播放（停止更新点云缓冲），但仍按固定刷新率重绘以跟随相机/叠加层
	if (pointCloudWidget && pointCloudWidget->isMeasurementModeEnabled()) {
		{
			QMutexLocker locker(&frameMutex);
			for (auto it = pendingFrames.begin(); it != pendingFrames.end(); ++it) {
				it.value().clear();
			}
		}
		pointCloudWidget->update();
		return;
	}
	// 以固定刷新率合并滑动窗口内的点并渲染
	uint64_t now_ns = 0;
	{
		QMutexLocker locker(&frameMutex);
		for (auto it = lastSeenTimestamp.begin(); it != lastSeenTimestamp.end(); ++it) {
			if (it.value() > now_ns) now_ns = it.value();
		}
	}
	if (now_ns == 0) return;

	uint64_t window_ns = frameIntervalMs * 1000000ULL;
	uint64_t window_begin = (now_ns > window_ns) ? (now_ns - window_ns) : 0ULL;

	PointCloudFrame merged;
	merged.timestamp = now_ns;
	merged.device_handle = 0;

	bool hasAnyPoint = false;
	{
		QMutexLocker locker(&frameMutex);
		for (auto it = pendingFrames.begin(); it != pendingFrames.end(); ++it) {
			QQueue<PointCloudFrame>& q = it.value();
			while (!q.isEmpty() && q.head().timestamp < window_begin) {
				q.dequeue();
			}
			for (int i = 0; i < q.size(); ++i) {
				const PointCloudFrame& f = q.at(i);
				if (f.timestamp >= window_begin && f.timestamp <= now_ns) {
					merged.points += f.points;
					hasAnyPoint = true;
				}
			}
		}
	}

	if (hasAnyPoint) {
		if (colorMode == ColorByReflectivity) {
			for (Point3D& p : merged.points) {
				calculatePointColor(p.reflectivity, p.tag, p.r, p.g, p.b);
			}
			if (pointCloudWidget) pointCloudWidget->setLegend(ColorByReflectivity, 0.0f, 255.0f, true);
		} else if (colorMode == ColorByDistance) {
			float minD = std::numeric_limits<float>::max();
			float maxD = 0.0f;
			for (const Point3D& p : merged.points) {
				float d = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
				if (d < minD) minD = d;
				if (d > maxD) maxD = d;
			}
			if (!(maxD > minD)) { minD = 0.0f; maxD = 1.0f; }
			for (Point3D& p : merged.points) {
				float r, g, b; (void)r; (void)g; (void)b;
				const float dx = p.x, dy = p.y, dz = p.z;
				float d = std::sqrt(dx*dx + dy*dy + dz*dz);
				float t = 0.0f;
				if (maxD > minD) t = (d - minD) / (maxD - minD);
				t = std::clamp(t, 0.0f, 1.0f);
				if (t < 0.25f)      { p.r = 0.0f;           p.g = t/0.25f;     p.b = 1.0f; }
				else if (t < 0.5f)  { p.r = 0.0f;           p.g = 1.0f;        p.b = 1.0f - (t-0.25f)/0.25f; }
				else if (t < 0.75f) { p.r = (t-0.5f)/0.25f; p.g = 1.0f;        p.b = 0.0f; }
				else                { p.r = 1.0f;           p.g = 1.0f-(t-0.75f)/0.25f; p.b = 0.0f; }
			}
			if (pointCloudWidget) pointCloudWidget->setLegend(ColorByDistance, minD, maxD, true);
		} else if (colorMode == ColorByElevation) {
			float minZ = std::numeric_limits<float>::max();
			float maxZ = std::numeric_limits<float>::lowest();
			for (const Point3D& p : merged.points) {
				if (p.z < minZ) minZ = p.z;
				if (p.z > maxZ) maxZ = p.z;
			}
			if (!(maxZ > minZ)) { minZ = -1.0f; maxZ = 1.0f; }
			for (Point3D& p : merged.points) {
				float t = 0.0f;
				if (maxZ > minZ) t = (p.z - minZ) / (maxZ - minZ);
				t = std::clamp(t, 0.0f, 1.0f);
				p.r = t; p.g = 0.0f; p.b = 1.0f - t;
			}
			if (pointCloudWidget) pointCloudWidget->setLegend(ColorByElevation, minZ, maxZ, true);
		        } else if (colorMode == ColorSolid) {
            for (Point3D& p : merged.points) {
                p.r = solidColor.redF();
                p.g = solidColor.greenF();
                p.b = solidColor.blueF();
            }
            if (pointCloudWidget) pointCloudWidget->setLegend(ColorSolid, 0.0f, 1.0f, false);
        } else if (colorMode == ColorByPlanarProjection) {
            // 平面投影模式：根据点在平面上的位置着色
            float minX = std::numeric_limits<float>::max();
            float maxX = std::numeric_limits<float>::lowest();
            float minY = std::numeric_limits<float>::max();
            float maxY = std::numeric_limits<float>::lowest();
            
            // 计算平面坐标范围
            for (const Point3D& p : merged.points) {
                if (p.x < minX) minX = p.x;
                if (p.x > maxX) maxX = p.x;
                if (p.y < minY) minY = p.y;
                if (p.y > maxY) maxY = p.y;
            }
            
            if (!(maxX > minX)) { minX = -planarProjectionRadius; maxX = planarProjectionRadius; }
            if (!(maxY > minY)) { minY = 0.0f; maxY = planarProjectionRadius; }
            
            for (Point3D& p : merged.points) {
                // 根据平面位置着色
                float tx = (p.x - minX) / (maxX - minX);
                float ty = (p.y - minY) / (maxY - minY);
                tx = std::clamp(tx, 0.0f, 1.0f);
                ty = std::clamp(ty, 0.0f, 1.0f);
                
                // 使用HSV颜色空间创建渐变效果
                float hue = tx * 360.0f;  // X轴对应色相
                float saturation = 0.8f;  // 固定饱和度
                float value = 0.5f + ty * 0.5f;  // Y轴对应明度
                
                // HSV转RGB
                float c = value * saturation;
                float x = c * (1.0f - std::abs(std::fmod(hue / 60.0f, 2.0f) - 1.0f));
                float m = value - c;
                
                if (hue < 60.0f) {
                    p.r = c + m; p.g = x + m; p.b = m;
                } else if (hue < 120.0f) {
                    p.r = x + m; p.g = c + m; p.b = m;
                } else if (hue < 180.0f) {
                    p.r = m; p.g = c + m; p.b = x + m;
                } else if (hue < 240.0f) {
                    p.r = m; p.g = x + m; p.b = c + m;
                } else if (hue < 300.0f) {
                    p.r = x + m; p.g = m; p.b = c + m;
                } else {
                    p.r = c + m; p.g = m; p.b = x + m;
                }
                
                // 确保RGB值在[0,1]范围内
                p.r = std::clamp(p.r, 0.0f, 1.0f);
                p.g = std::clamp(p.g, 0.0f, 1.0f);
                p.b = std::clamp(p.b, 0.0f, 1.0f);
            }
            if (pointCloudWidget) pointCloudWidget->setLegend(ColorByPlanarProjection, 0.0f, 1.0f, true);
        }
		        // 点云滤波处理
        merged.points = applyPointCloudFilters(merged.points);
        

		// 保存PCD：在渲染循环中，当开启保存任务时按帧保存
		if (pcdSaveActive && pcdFramesRemaining > 0) {
			// 用合并窗口末尾时间戳作为文件名（纳秒）
			if (pcdLastSavedTimestamp != now_ns) {
				QString fileName = QString::number(now_ns) + ".pcd";
				QString filePath = QDir(pcdSaveDir).filePath(fileName);
				if (savePointCloudAsPCD(filePath, merged.points)) {
					logMessage(QString("PCD保存: %1").arg(QDir::toNativeSeparators(filePath)));
					pcdLastSavedTimestamp = now_ns;
					pcdFramesRemaining--;
					if (pcdFramesRemaining <= 0) {
						pcdSaveActive = false;
						statusLabelBar->setText("PCD保存完成");
					}
				} else {
					logMessage(QString("PCD保存失败: %1").arg(QDir::toNativeSeparators(filePath)));
					// 即使失败也避免卡住
					pcdLastSavedTimestamp = now_ns;
					pcdFramesRemaining--;
				}
			}
		}
		// 保存LAS：与PCD一致的触发策略
		if (lasSaveActive && lasFramesRemaining > 0) {
			if (lasLastSavedTimestamp != now_ns) {
				QString fileName = QString::number(now_ns) + ".las";
				QString filePath = QDir(lasSaveDir).filePath(fileName);
				if (savePointCloudAsLAS(filePath, merged.points)) {
					logMessage(QString("LAS保存: %1").arg(QDir::toNativeSeparators(filePath)));
					lasLastSavedTimestamp = now_ns;
					lasFramesRemaining--;
					if (lasFramesRemaining <= 0) {
						lasSaveActive = false;
						statusLabelBar->setText("LAS保存完成");
					}
				} else {
					logMessage(QString("LAS保存失败: %1").arg(QDir::toNativeSeparators(filePath)));
					lasLastSavedTimestamp = now_ns;
					lasFramesRemaining--;
				}
			}
		}
		publishPointCloudFrame(merged);
	}

	if (selectionRealtimeEnabled && pointCloudWidget && (attrTable || selectionTable)) {
		updateSelectionTableAndLog();
	}
}

void MainWindow::onMeasurementUpdated()
{
	if (!pointCloudWidget) return;
	if (pointCloudWidget->hasMeasureP1() && !pointCloudWidget->hasMeasureP2()) {
		statusLabelBar->setText("测距：已选择第一点，按住Ctrl+左键选择第二点");
	} else if (pointCloudWidget->hasMeasureP1() && pointCloudWidget->hasMeasureP2()) {
		double d = pointCloudWidget->getMeasureDistance();
		statusLabelBar->setText(QString("测距结果：%1 m").arg(d, 0, 'f', 3));
		logMessage(QString("测距完成：%1 m").arg(d, 0, 'f', 3));
	} else {
		statusLabelBar->setText("测距模式：按住Ctrl+左键选择第一点");
	}
}

void MainWindow::onPointSizeChanged(int px)
{
    pointSizePx = static_cast<float>(px);
    if (pointCloudWidget) pointCloudWidget->setPointSize(pointSizePx);
}

void MainWindow::onColorModeChanged(int index)
{
    colorMode = index;
    if (solidColorRow) {
        solidColorRow->setEnabled(colorMode == ColorSolid);
    }
    if (pointCloudWidget) {
        if (colorMode == ColorByReflectivity) {
            pointCloudWidget->setLegend(ColorByReflectivity, 0.0f, 255.0f, true);
        } else if (colorMode == ColorByDistance) {
            pointCloudWidget->setLegend(ColorByDistance, 0.0f, 1.0f, true);
        } else if (colorMode == ColorByElevation) {
            pointCloudWidget->setLegend(ColorByElevation, -1.0f, 1.0f, true);
        } else if (colorMode == ColorSolid) {
            pointCloudWidget->setLegend(ColorSolid, 0.0f, 1.0f, false);
        } else if (colorMode == ColorByPlanarProjection) {
            pointCloudWidget->setLegend(ColorByPlanarProjection, 0.0f, 1.0f, true);
        }
    }
}

void MainWindow::onSolidColorClicked()
{
    QColor c = QColorDialog::getColor(solidColor, this, "选择点云颜色");
    if (!c.isValid()) return;
    solidColor = c;
    if (solidColorPreview) {
        solidColorPreview->setStyleSheet(QString("background-color: %1;").arg(solidColor.name()));
    }
}

void MainWindow::onProjectionDepthChanged(double meters)
{
    if (meters < 0.0) meters = 0.0;
    projectionDepthMeters = static_cast<float>(meters);
}

void MainWindow::onProjectionDepthToggled(bool enabled)
{
    projectionDepthEnabled = enabled;
    if (projectionDepthSpin) {
        projectionDepthSpin->setEnabled(enabled);
    }
    logMessage(enabled ? "深度投影已启用" : "深度投影已关闭");
}

void MainWindow::onPlanarProjectionToggled(bool enabled)
{
    planarProjectionEnabled = enabled;
    if (enabled) {
        logMessage("平面投影模式已启用");
        statusLabelBar->setText("平面投影模式已启用");
        // 自动调整视角以便观察平面投影
        if (pointCloudWidget) {
            pointCloudWidget->resetView();
            // 设置平面投影视角以便观察
            pointCloudWidget->setTopDownView();
        }
    } else {
        logMessage("平面投影模式已关闭");
        statusLabelBar->setText("平面投影模式已关闭");
        // 恢复默认视角
        if (pointCloudWidget) {
            pointCloudWidget->resetView();
        }
    }
    if (planarRadiusSpin) {
        planarRadiusSpin->setEnabled(enabled);
    }
}

void MainWindow::onPlanarProjectionRadiusChanged(double radius)
{
    if (radius < 1.0) radius = 1.0;
    planarProjectionRadius = static_cast<float>(radius);
    logMessage(QString("平面投影半径已设置为 %1 m").arg(radius));
}

void MainWindow::onPointCloudVisualizationToggled(bool enabled)
{
    pointCloudVisualizationEnabled = enabled;
    if (enabled) {
        logMessage("点云可视化已开启");
    } else {
        logMessage("点云可视化已暂停");
    }
} 

void MainWindow::updateSelectionTableAndLog()
{
    QVector<Point3D> pts;
    if (pointCloudWidget->hasSelectionAabb()) {
        pts = pointCloudWidget->pointsInPersistSelection(200000);
    } else {
        QRect sel = pointCloudWidget->currentSelectionRect();
        if (!sel.isEmpty()) pts = pointCloudWidget->pointsInRect(sel, 200000);
    }

    QTableWidget* table = attrTable ? attrTable : selectionTable;
    if (!table) return;

    if (!pts.isEmpty()) {
        int count = pts.size();
        if (count != lastSelectionCount) {
            lastSelectionCount = count;
            logMessage(QString("框选点个数: %1").arg(count));
        }
        bool sorting = table->isSortingEnabled();
        table->setSortingEnabled(false);
        table->clearContents();
        table->setRowCount(0);
        const int maxRows = 500;
        int rows = 0;
        for (const Point3D& p : pts) {
            if (rows >= maxRows) break;
            int row = table->rowCount();
            table->insertRow(row);
            table->setItem(row, 0, new NumberItem(p.x, 3));
            table->setItem(row, 1, new NumberItem(p.y, 3));
            table->setItem(row, 2, new NumberItem(p.z, 3));
            table->setItem(row, 3, new NumberItem(static_cast<int>(p.reflectivity)));
            table->setItem(row, 4, new NumberItem(static_cast<int>(p.tag)));
            rows++;
        }
        table->setSortingEnabled(sorting);
    } else {
        if (lastSelectionCount != -1) {
            lastSelectionCount = -1;
            logMessage("已清除框选");
        }
        if (table) {
            table->setSortingEnabled(false);
            table->clearContents();
            table->setRowCount(0);
            table->setSortingEnabled(true);
        }
    }
}

void MainWindow::onSelectionFinished()
{
    if (!pointCloudWidget || !selectionTable) return;
    updateSelectionTableAndLog();
} 

void MainWindow::startLvx2Recording(const QString& filePath, int durationSec)
{
    QMutexLocker lk(&lvx2Mutex);
    if (lvx2SaveActive) return;
    lvx2File.setFileName(filePath);
    if (!lvx2File.open(QIODevice::WriteOnly)) {
        logMessage("打开LVX2文件失败");
        currentCapture = CaptureNone;
        return;
    }
    // 写头
    LVX2PublicHeader pub;
    lvx2File.write(reinterpret_cast<const char*>(&pub), sizeof(pub));
    LVX2PrivateHeader pri;
    lvx2File.write(reinterpret_cast<const char*>(&pri), sizeof(pri));
    LVX2DeviceInfo dev{};
    QByteArray snb = currentDevice ? currentDevice->sn.left(15).toLatin1() : QByteArray("Unknown");
    std::memset(dev.lidar_sn, 0, sizeof(dev.lidar_sn));
    std::memcpy(dev.lidar_sn, snb.constData(), std::min<size_t>(size_t(snb.size()), sizeof(dev.lidar_sn)));
    dev.lidar_id = currentDevice ? currentDevice->handle : 0;
    lvx2File.write(reinterpret_cast<const char*>(&dev), sizeof(dev));

    lvx2SaveActive = true;
    lvx2FrameStartNs = 0;
    lvx2FrameIndex = 0;
    // 借用采集计时器
    captureSecondsRemaining = durationSec;
    captureProgress->setValue(0);
    captureProgress->setFormat("录制中 %p% (%v s)");
}

void MainWindow::stopLvx2Recording(bool flushPending)
{
    QMutexLocker lk(&lvx2Mutex);
    if (!lvx2SaveActive) return;
    Q_UNUSED(flushPending);
    lvx2SaveActive = false;
    if (lvx2File.isOpen()) lvx2File.close();
}

QVector<Point3D> MainWindow::applyPointCloudFilters(const QVector<Point3D>& inputPoints)
{
    if (inputPoints.isEmpty()) {
        return inputPoints;
    }

    QVector<Point3D> filteredPoints = inputPoints;

    // 噪点处理（基于tag值识别）
    if (showNoisePoints || removeNoisePoints) {
        QVector<Point3D> processedPoints;
        for (const Point3D& p : filteredPoints) {
            bool isNoise = filterTagMatches(p.tag);
            Point3D processedPoint = p;
            
            if (showNoisePoints && isNoise) {
                // 高亮噪点（红色）
                processedPoint.r = 1.0f;
                processedPoint.g = 0.0f;
                processedPoint.b = 0.0f;
            }
            
            if (!removeNoisePoints || !isNoise) {
                // 根据设置决定是否保留噪点
                processedPoints.append(processedPoint);
            }
        }
        filteredPoints = processedPoints;
    }



    return filteredPoints;
}

#include "LivoxViewerWindow.h"

#include "Export/PointCloudExport.h"
#include "PointCloud/PointCloudColorizer.h"
#include "PointCloud/PointCloudDecoder.h"
#include "PointCloud/PointCloudFilter.h"
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
#include <QChartView>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QValueAxis>
#include <QStandardPaths>

class YAxisZoomChartView : public QChartView {
public:
    explicit YAxisZoomChartView(QChart *chart, QWidget *parent = nullptr) 
        : QChartView(chart, parent), m_isDragging(false) {
        setRenderHint(QPainter::Antialiasing);
        setMouseTracking(true); // 开启追踪，确保不按键移动时也能触发 MoveEvent 切换光标
        
        // 初始状态：张开的手
        viewport()->setCursor(Qt::OpenHandCursor);
    }

protected:
    // --- 缩放逻辑保持不变 ---
    void wheelEvent(QWheelEvent *event) override {
        QValueAxis *axisY = getYAxis();
        if (axisY && chart()) {
            QPointF mousePos = event->position();
            if (!chart()->plotArea().contains(mousePos)) return;

            double axisYHeight = chart()->plotArea().height();
            double yMin = axisY->min();
            double yMax = axisY->max();
            double mouseValue = yMax - (mousePos.y() - chart()->plotArea().top()) * (yMax - yMin) / axisYHeight;
            
            double factor = (event->angleDelta().y() > 0) ? 0.8 : 1.25;
            double currentRange = yMax - yMin;
            double newRange = currentRange * factor;

            if (newRange < 0.001 || newRange > 1000.0) return;

            double relativePos = (mouseValue - yMin) / currentRange;
            double newMin = mouseValue - (relativePos * newRange);
            double newMax = newMin + newRange;

            axisY->setRange(newMin, newMax);
            event->accept();
        }
    }

    // --- 鼠标按下：切换为闭合手势 ---
    void mousePressEvent(QMouseEvent *event) override {
        if (event->button() == Qt::LeftButton) {
            if (chart() && chart()->plotArea().contains(event->position())) {
                m_isDragging = true;
                m_lastMousePos = event->position();
                
                // 核心修改：切换为 ClosedHandCursor
                viewport()->setCursor(Qt::ClosedHandCursor);
                
                event->accept();
                return; 
            }
        }
        QChartView::mousePressEvent(event);
    }

    // --- 鼠标移动：维持手势或处理平移 ---
    void mouseMoveEvent(QMouseEvent *event) override {
        if (m_isDragging) {
            // 确保拖拽过程中一直是闭合手势
            viewport()->setCursor(Qt::ClosedHandCursor);

            QValueAxis *axisY = getYAxis();
            if (axisY && chart()) {
                QRectF plotArea = chart()->plotArea();
                double axisYHeight = plotArea.height();
                double yRange = axisY->max() - axisY->min();

                double pixelDiff = event->position().y() - m_lastMousePos.y();
                double valueDiff = (pixelDiff / axisYHeight) * yRange;

                axisY->setRange(axisY->min() + valueDiff, axisY->max() + valueDiff);
                m_lastMousePos = event->position();
                event->accept();
                return;
            }
        } else {
            // 没有拖拽时，在绘图区显示张开的手
            if (chart() && chart()->plotArea().contains(event->position())) {
                viewport()->setCursor(Qt::OpenHandCursor);
            } else {
                viewport()->setCursor(Qt::ArrowCursor);
            }
        }
        QChartView::mouseMoveEvent(event);
    }

    // --- 鼠标释放：恢复张开手势 ---
    void mouseReleaseEvent(QMouseEvent *event) override {
        if (event->button() == Qt::LeftButton) {
            m_isDragging = false;
            
            // 核心修改：恢复为 OpenHandCursor
            if (chart() && chart()->plotArea().contains(event->position())) {
                viewport()->setCursor(Qt::OpenHandCursor);
            }
            
            event->accept();
        }
        QChartView::mouseReleaseEvent(event);
    }

    // 当鼠标离开控件时，恢复标准箭头（防止光标样式污染其他 UI）
    void leaveEvent(QEvent *event) override {
        viewport()->setCursor(Qt::ArrowCursor);
        QChartView::leaveEvent(event);
    }

private:
    QValueAxis* getYAxis() {
        if (!chart()) return nullptr;
        auto axes = chart()->axes(Qt::Vertical);
        if (axes.isEmpty()) return nullptr;
        return qobject_cast<QValueAxis*>(axes.first());
    }

    bool m_isDragging;
    QPointF m_lastMousePos;
};

// 数值排序项（整行排序时按数值比较当前列）
class NumberItem : public QTableWidgetItem {
public:
    explicit NumberItem(double v, int decimals = 3)
        : QTableWidgetItem(QString::number(v, 'f', decimals)), value(v) {}
    explicit NumberItem(int v)
        : QTableWidgetItem(QString::number(v)), value(static_cast<double>(v)) {}
    explicit NumberItem(qlonglong v)
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
    LivoxViewerWindow* w = static_cast<LivoxViewerWindow*>(client_data);
    if (!w) return;
    if (status != kLivoxLidarStatusSuccess || response == nullptr || response->ret_code != 0) {
        LivoxLidarStartLogger(handle, kLivoxLidarRealTimeLog, LoggerStartCallback, client_data);
        return;
    }
}

static void DebugPointCloudCallback(livox_status status, uint32_t handle, LivoxLidarLoggerResponse* response, void* client_data) {
    LivoxViewerWindow* w = static_cast<LivoxViewerWindow*>(client_data);
    if (!w) return;
}

void LivoxViewerWindow::onStartCaptureLog()
{
    if (!currentLidarDevice || !currentLidarDevice->is_connected) { logMessage("设备未连接"); return; }
    if (captureState.current != CaptureNone) return;
    captureState.current = CaptureLog;
    int sec = captureState.durationSpin ? captureState.durationSpin->value() : 10;
    captureState.secondsRemaining = sec;
    captureState.totalSeconds = sec;
    logMessage(QString("开始采集日志，时长: %1s").arg(sec));
    captureState.progress->setValue(0);
    captureState.progress->setFormat("LOG采集中 %p% (%v s)");
    // 启动日志
    SaveLivoxLidarSdkLoggerFile();
    LivoxLidarStartLogger(currentLidarDevice->handle, kLivoxLidarRealTimeLog, LoggerStartCallback, this);
    captureState.timer->start(1000);
}

void LivoxViewerWindow::onStartCaptureDebug()
{
    if (!currentLidarDevice || !currentLidarDevice->is_connected) { logMessage("设备未连接"); return; }
    if (captureState.current != CaptureNone) return;
    captureState.current = CaptureDebug;
    int sec = captureState.durationSpin ? captureState.durationSpin->value() : 10;
    captureState.secondsRemaining = sec;
    captureState.totalSeconds = sec;
    logMessage(QString("开始采集Debug点云，时长: %1s").arg(sec));
    captureState.progress->setValue(0);
    captureState.progress->setFormat("Debug采集中 %p% (%v s)");
    // 开启Debug点云
    SetLivoxLidarDebugPointCloud(currentLidarDevice->handle, true, DebugPointCloudCallback, this);
    captureState.timer->start(1000);
}

void LivoxViewerWindow::onCaptureTick()
{
    if (captureState.secondsRemaining <= 0) {
        captureState.timer->stop();
        // 停止采集/录制
        if (captureState.current == CaptureLog) {
            LivoxLidarStopLogger(currentLidarDevice->handle, kLivoxLidarRealTimeLog, LoggerStartCallback, this);
            logMessage("日志采集完成");
        } else if (captureState.current == CaptureDebug) {
            SetLivoxLidarDebugPointCloud(currentLidarDevice->handle, false, DebugPointCloudCallback, this);
            logMessage("Debug点云采集完成");
        } else if (captureState.current == CaptureLVX2) {
            stopLvx2Recording(true);
            logMessage("LVX2数据保存完成");
        } else if (captureState.current == CaptureIMU) {
            {
                QMutexLocker lk(&captureState.imuCsvMutex);
                if (captureState.imuCsvFile.isOpen()) captureState.imuCsvFile.flush();
                if (captureState.imuCsvFile.isOpen()) captureState.imuCsvFile.close();
            }
            captureState.imuSaveActive = false;
            logMessage("IMU数据保存完成");
        }
        if (captureState.progress) {
            captureState.progress->setValue(100);
            captureState.progress->setFormat("采集完成");
        }
        statusLabelBar->setText("已连接 - 采样中");
        captureState.current = CaptureNone;
        return;
    }
    //statusLabelBar->setText("数据采集中");

    int total = captureState.totalSeconds > 0 ? captureState.totalSeconds : (captureState.durationSpin ? captureState.durationSpin->value() : 1);
    int done = total - captureState.secondsRemaining;
    if (done < 0) done = 0;
    if (done > total) done = total;
    int percent = total > 0 ? (done * 100 / total) : 100;
    if (captureState.progress) {
        captureState.progress->setValue(percent);
        captureState.progress->setFormat(QString("%1% (%2 s)").arg(percent).arg(captureState.secondsRemaining));
    }
    captureState.secondsRemaining--;
}

static QString nmeaChecksum(const QString& payload)
{
    quint8 cs = 0;
    for (QChar c : payload) cs ^= c.toLatin1();
    return QString("*%1").arg(cs, 2, 16, QChar('0')).toUpper();
}

void LivoxViewerWindow::onGpsSimulateToggled(bool enabled)
{
    if (!currentLidarDevice || !currentLidarDevice->is_connected) {
        imuState.gpsSimulateCheck->setChecked(false);
        return;
    }

    if (enabled) {
        imuState.gpsTimer->start(1000); // 1 Hz
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
    if (!currentLidarDevice || !currentLidarDevice->is_connected) return;
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
    SetLivoxLidarRmcSyncTime(currentLidarDevice->handle, rmc.constData(), static_cast<uint16_t>(rmc.size()), nullptr, nullptr);
}

QString LivoxViewerWindow::buildImuAscii(double gx, double gy, double gz, double ax, double ay, double az) const
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

void LivoxViewerWindow::onImuDisplayButtonClicked()
{
    // Toggle 2 Hz text-only updater
    if (imuState.displayRunning.exchange(!imuState.displayRunning.load())) {
        // turned off
        if (imuState.displayThread.joinable()) {
            imuState.displayThread.detach(); // avoid blocking GUI
        }
        if (imuState.dataTable) {
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 2; ++c) {
                    QTableWidgetItem* item = imuState.dataTable->item(r, c);
                    if (!item) {
                        item = new QTableWidgetItem();
                        item->setTextAlignment(Qt::AlignCenter);
                        imuState.dataTable->setItem(r, c, item);
                    }
                    item->setText("0.000");
                }
            }
        }
        if (imuState.asciiLabel) imuState.asciiLabel->setText("状态: 已停止");
        if (imuState.displayButton) imuState.displayButton->setText("显示IMU数据");
        return;
    }

    if (imuState.displayButton) imuState.displayButton->setText("停止IMU显示");

    // Start or restart text updater thread (2 Hz)
    if (imuState.displayThread.joinable()) {
        imuState.displayThread.detach();
    }
    imuState.displayThread = std::thread([this]() {
        while (imuState.displayRunning.load()) {
            float gx=0, gy=0, gz=0, ax=0, ay=0, az=0; bool have=false;
            {
                QMutexLocker lk(&imuState.sampleMutex);
                if (imuState.latestSample.have) {
                    gx = imuState.latestSample.gx; gy = imuState.latestSample.gy; gz = imuState.latestSample.gz;
                    ax = imuState.latestSample.ax; ay = imuState.latestSample.ay; az = imuState.latestSample.az;
                    have = true;
                }
            }
            if (have) {
                QMetaObject::invokeMethod(this, [this, gx, gy, gz, ax, ay, az]() {
                    if (imuState.dataTable) {
                        const double values[3][2] = {
                            {gx, ax}, {gy, ay}, {gz, az}
                        };
                        for (int r = 0; r < 3; ++r) {
                            for (int c = 0; c < 2; ++c) {
                                QTableWidgetItem* item = imuState.dataTable->item(r, c);
                                if (!item) {
                                    item = new QTableWidgetItem();
                                    item->setTextAlignment(Qt::AlignCenter);
                                    imuState.dataTable->setItem(r, c, item);
                                }
                                item->setText(QString::number(values[r][c], 'f', 3));
                            }
                        }
                    }
                    if (imuState.asciiLabel) imuState.asciiLabel->setText("状态: 接收中");
                });
            }
            for (int i=0; i<10 && imuState.displayRunning.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    });
}

void LivoxViewerWindow::onActionShowImuCharts()
{
    if (imuState.chartWindow && imuState.chartWindow->isVisible()) {
        imuState.chartWindow->raise();
        imuState.chartWindow->activateWindow();
        return;
    }
    // Build chart window
    imuState.chartWindow = new QWidget(this, Qt::Window);
    imuState.chartWindow->setAttribute(Qt::WA_DeleteOnClose);
    imuState.chartWindow->setWindowTitle("IMU数据曲线");
    QVBoxLayout* layout = new QVBoxLayout(imuState.chartWindow);

    // Gyro chart
    imuState.gyroChart = new QChart();
    imuState.gyroSeriesX = new QLineSeries(); imuState.gyroSeriesX->setName("gyro_x");
    imuState.gyroSeriesY = new QLineSeries(); imuState.gyroSeriesY->setName("gyro_y");
    imuState.gyroSeriesZ = new QLineSeries(); imuState.gyroSeriesZ->setName("gyro_z");
    imuState.gyroChart->addSeries(imuState.gyroSeriesX);
    imuState.gyroChart->addSeries(imuState.gyroSeriesY);
    imuState.gyroChart->addSeries(imuState.gyroSeriesZ);
    imuState.gyroAxisX = new QValueAxis(); imuState.gyroAxisX->setTitleText("时间 (s)");
    imuState.gyroAxisY = new QValueAxis(); imuState.gyroAxisY->setTitleText("角速度 (rad/s)"); imuState.gyroAxisY->setRange(-50, 50);
    imuState.gyroChart->addAxis(imuState.gyroAxisX, Qt::AlignBottom);
    imuState.gyroChart->addAxis(imuState.gyroAxisY, Qt::AlignLeft);
    for (auto s : {imuState.gyroSeriesX, imuState.gyroSeriesY, imuState.gyroSeriesZ}) { s->attachAxis(imuState.gyroAxisX); s->attachAxis(imuState.gyroAxisY); }
    imuState.gyroChart->legend()->setVisible(true);
    imuState.gyroChartView = new YAxisZoomChartView(imuState.gyroChart, imuState.chartWindow);
    
    // Acc chart
    imuState.accChart = new QChart();
    imuState.accSeriesX = new QLineSeries(); imuState.accSeriesX->setName("acc_x");
    imuState.accSeriesY = new QLineSeries(); imuState.accSeriesY->setName("acc_y");
    imuState.accSeriesZ = new QLineSeries(); imuState.accSeriesZ->setName("acc_z");
    imuState.accChart->addSeries(imuState.accSeriesX);
    imuState.accChart->addSeries(imuState.accSeriesY);
    imuState.accChart->addSeries(imuState.accSeriesZ);
    imuState.accAxisX = new QValueAxis(); imuState.accAxisX->setTitleText("时间 (s)");
    imuState.accAxisY = new QValueAxis(); imuState.accAxisY->setTitleText("加速度 (g)"); imuState.accAxisY->setRange(-4, 4);
    imuState.accChart->addAxis(imuState.accAxisX, Qt::AlignBottom);
    imuState.accChart->addAxis(imuState.accAxisY, Qt::AlignLeft);
    for (auto s : {imuState.accSeriesX, imuState.accSeriesY, imuState.accSeriesZ}) { s->attachAxis(imuState.accAxisX); s->attachAxis(imuState.accAxisY); }
    imuState.accChart->legend()->setVisible(true);
    imuState.accChartView = new YAxisZoomChartView(imuState.accChart, imuState.chartWindow);

    layout->addWidget(imuState.gyroChartView);
    layout->addWidget(imuState.accChartView);
    imuState.chartWindow->setLayout(layout);

    // Start or restart chart updater (20 Hz) without blocking GUI
    if (imuState.chartRunning.exchange(true)) {
        if (imuState.chartThread.joinable()) imuState.chartThread.detach();
    }
    imuState.chartThread = std::thread([this]() {
        double t = 0.0;
        const double dt = 0.05; // 20 Hz
        const double windowSec = 5.0;
        const int maxPoints = static_cast<int>(windowSec / dt);
        while (imuState.chartRunning.load()) {
            float gx=0, gy=0, gz=0, ax=0, ay=0, az=0; bool have=false;
            {
                QMutexLocker lk(&imuState.sampleMutex);
                if (imuState.latestSample.have) {
                    gx = imuState.latestSample.gx; gy = imuState.latestSample.gy; gz = imuState.latestSample.gz;
                    ax = imuState.latestSample.ax; ay = imuState.latestSample.ay; az = imuState.latestSample.az;
                    have = true;
                }
            }
            if (have) {
                QMetaObject::invokeMethod(this, [this, gx, gy, gz, ax, ay, az, t, maxPoints, dt]() {
                    auto pushPoint = [&](QLineSeries* s, double x, double y){ if (!s) return; s->append(x, y); if (s->count() > maxPoints) s->removePoints(0, s->count() - maxPoints); };
                    pushPoint(imuState.gyroSeriesX, t, gx); pushPoint(imuState.gyroSeriesY, t, gy); pushPoint(imuState.gyroSeriesZ, t, gz);
                    pushPoint(imuState.accSeriesX, t, ax); pushPoint(imuState.accSeriesY, t, ay); pushPoint(imuState.accSeriesZ, t, az);
                    if (imuState.gyroAxisX) imuState.gyroAxisX->setRange(std::max(0.0, t - (maxPoints - 1) * dt), t);
                    if (imuState.accAxisX) imuState.accAxisX->setRange(std::max(0.0, t - (maxPoints - 1) * dt), t);
                });
            }
            t += dt;
            for (int i=0; i<5 && imuState.chartRunning.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    });

    // Stop chart when window closed
    connect(imuState.chartWindow, &QObject::destroyed, this, [this]() {
        imuState.chartRunning.store(false);
        if (imuState.chartThread.joinable()) imuState.chartThread.detach();
        imuState.gyroChart = nullptr; imuState.gyroChartView = nullptr; imuState.gyroSeriesX = imuState.gyroSeriesY = imuState.gyroSeriesZ = nullptr; imuState.gyroAxisX = imuState.gyroAxisY = nullptr;
        imuState.accChart = nullptr; imuState.accChartView = nullptr; imuState.accSeriesX = imuState.accSeriesY = imuState.accSeriesZ = nullptr; imuState.accAxisX = imuState.accAxisY = nullptr;
        imuState.chartWindow = nullptr;
    });

    imuState.chartWindow->resize(900, 600);
    imuState.chartWindow->show();
}

void LivoxViewerWindow::onActionCaptureImuTriggered()
{
    if (!currentLidarDevice || !currentLidarDevice->is_connected) {
        QMessageBox::warning(this, "保存IMU数据", "设备未连接");
        return;
    }
    if (captureState.current != CaptureNone) {
        QMessageBox::warning(this, "保存IMU数据", "当前已有采集任务在进行中");
        return;
    }
    // 检查IMU数据发送是否开启
    QWidget* ctrl = parameterState.controls.value(kKeyImuDataEn, nullptr);
    QComboBox* imuCombo = qobject_cast<QComboBox*>(ctrl);
    if (!imuCombo || imuCombo->currentIndex() != 1) {
        QMessageBox::warning(this, "保存IMU数据", "IMU数据发送未开启！");
        return;
    }

    QSettings settings("Livox", "LivoxViewerQT");
    QString lastDir = settings.value("save/lastIMUDir", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
    if (lastDir.isEmpty()) lastDir = QDir::homePath();

    QDialog dlg(this);
    dlg.setWindowTitle("保存IMU数据");
    QVBoxLayout* v = new QVBoxLayout(&dlg);
    // 路径
    QWidget* row1 = new QWidget(&dlg);
    QHBoxLayout* h1 = new QHBoxLayout(row1);
    h1->setContentsMargins(0,0,0,0);
    QLabel* lblPath = new QLabel("请选择保存路径:", row1);
    QLineEdit* editPath = new QLineEdit(row1);
    editPath->setText(lastDir);   // 设置上次目录为默认
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
    spinSec->setRange(10, 86400);
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

    // 浏览按钮：使用当前 editPath 文本或上次目录作为起始目录
    connect(btnBrowse, &QPushButton::clicked, &dlg, [editPath, lastDir, this]() {
        QString startDir = editPath->text().isEmpty() ? lastDir : editPath->text();
        QString dir = QFileDialog::getExistingDirectory(this, "选择保存目录", startDir);
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

    // 保存本次选择的目录
    settings.setValue("save/lastIMUDir", baseDir);

    QString sn = currentLidarDevice ? currentLidarDevice->sn : QString("Unknown");
    QString targetDir = QDir(baseDir).filePath(QString("IMU_%1").arg(sn));
    QDir().mkpath(targetDir);
    QString startTime = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString filePath = QDir(targetDir).filePath(QString("%1_%2.csv").arg(sn, startTime));

    // 打开CSV
    captureState.imuCsvFile.setFileName(filePath);
    if (!captureState.imuCsvFile.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        QMessageBox::warning(this, "保存IMU数据", "无法创建CSV文件");
        return;
    }
    {
        QMutexLocker lk(&captureState.imuCsvMutex);
        QTextStream ts(&captureState.imuCsvFile);
        ts << "timestamp_ns,gx,gy,gz,ax,ay,az\n";
    }
    // 配置进度条
    if (captureState.progress) {
        captureState.progress->setRange(0, 100);
        captureState.progress->setValue(0);
        captureState.progress->setFormat("IMU采集中 %p% (%v s)");
    }
    // 启动计时
    captureState.secondsRemaining = spinSec->value();
    captureState.totalSeconds = captureState.secondsRemaining;
    captureState.current = CaptureIMU;
    captureState.imuSaveActive = true;
    statusLabelBar->setText("正在保存IMU数据...");
    logMessage(QString("IMU保存路径: %1").arg(QDir::toNativeSeparators(filePath)));
    captureState.timer->start(1000);
}

void LivoxViewerWindow::appendImuCsvRow(quint64 timestamp_ns, float gx, float gy, float gz, float ax, float ay, float az)
{
    QMutexLocker lk(&captureState.imuCsvMutex);
    if (!captureState.imuSaveActive || !captureState.imuCsvFile.isOpen()) return;
    QTextStream ts(&captureState.imuCsvFile);
    ts.setRealNumberNotation(QTextStream::FixedNotation);
    ts.setRealNumberPrecision(6);
    ts << timestamp_ns << ',' << gx << ',' << gy << ',' << gz << ',' << ax << ',' << ay << ',' << az << '\n';
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
        // 记录串口停止日志
        logMessage("串口转发GPS已关闭");
        // 更新状态栏
        statusLabelBar->setText("串口转发GPS已关闭");
        return;
    }
    if (!currentLidarDevice || !currentLidarDevice->is_connected) {
        imuState.serialEnableCheck->setChecked(false);
        return;
    }
    QString portName = imuState.serialPortCombo ? imuState.serialPortCombo->currentText() : QString();
    if (portName.isEmpty() || portName == "未连接") {
        imuState.serialEnableCheck->setChecked(false);
        return;
    }
    imuState.serialRunning.store(true);
    // 记录串口启动日志并更新状态栏
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
                // 查找GPS报文（RMC、GGA、GSA、GSV等）
                if (line.startsWith("$GP") || line.startsWith("$GN")) {
                    if (currentLidarDevice && currentLidarDevice->is_connected) {
                        QString gpsMessage = QString::fromLatin1(line.trimmed());
                        
                        // 对于RMC报文，进行时间同步
                        if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC")) {
                            // 打印GPS同步报文到日志并更新状态栏
                            QMetaObject::invokeMethod(this, [this, gpsMessage, portName]() {
                                logMessage(QString("串口转发GPS同步: %1").arg(gpsMessage));
                                statusLabelBar->setText(QString("串口转发GPS同步中... 端口: %1").arg(portName));
                            }, Qt::QueuedConnection);
                            
                            SetLivoxLidarRmcSyncTime(currentLidarDevice->handle, line.constData(), static_cast<uint16_t>(line.size()), nullptr, nullptr);
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

void LivoxViewerWindow::onFrameIntervalChanged(int ms)
{
    if (ms < 50) ms = 50;
    frameIntervalMs = static_cast<uint64_t>(ms);
    logMessage(QString("点云积分时间已设置为 %1 ms").arg(ms));
    if (playbackState.active) {
        playbackState.slidingWindowStart = -1;
        playbackState.slidingWindowEnd = -1;
        playbackState.slidingWindowPoints.clear();
        playbackState.slidingWindowSegmentPointCounts.clear();
        playbackState.slidingWindowTimestamp = 0;
        const int sourceFrameCount = playbackState.source ? playbackState.source->frameCount() : 0;
        const int rawFramesPerStep = std::max(1, int((frameIntervalMs + 49ULL) / 50ULL));
        if (playbackState.mode == Lvx2PlaybackMode::SlidingWindow) {
            playbackState.frameCount = sourceFrameCount;
        } else {
            playbackState.frameCount = (sourceFrameCount + rawFramesPerStep - 1) / rawFramesPerStep;
        }
        if (playbackState.frameCount <= 0) {
            playbackState.frameCount = 1;
        }
        const int targetFrame = std::clamp(playbackState.frame, 0, playbackState.frameCount - 1);
        showLvx2PlaybackFrame(targetFrame);
        if (playbackState.playing) {
            setLvx2PlaybackPlaying(true);
        } else {
            updateLvx2PlaybackUi();
        }
    }
}

void LivoxViewerWindow::decodePointCloudPacket(uint32_t handle, const LivoxLidarEthernetPacket* packet)
{
    PointCloudDecoder::DecodeOptions options;
    options.depthProjectionEnabled = projectionDepthEnabled;
    options.depthMeters = projectionDepthMeters;
    options.planarProjectionEnabled = planarProjectionEnabled;
    options.planarRadius = planarProjectionRadius;

    PointCloudFrame frame;
    if (!PointCloudDecoder::decodeLivoxPacket(handle, packet, options, frame)) {
        return;
    }

    {
        QMutexLocker locker(&frameMutex);
        pendingFrames[handle].enqueue(frame);
        lastSeenTimestamp[handle] = frame.timestamp;
    }
}

void LivoxViewerWindow::presentPointCloudFrame(const PointCloudFrame& frame)
{
    // 在主线程中更新点云显示
    QMetaObject::invokeMethod(this, [this, frame]() mutable {
        pointCloudView->updatePointCloud(std::move(frame));
        if (selectionRealtimeEnabled && pointCloudView && (attrTable || selectionTable)) {
            updateSelectionTableAndLog();
        }
    }, Qt::QueuedConnection);
}

void LivoxViewerWindow::applyPointCloudPipeline(PointCloudFrame& frame)
{
    PointCloudColorizer::Config colorConfig;
    colorConfig.mode = colorMode;
    colorConfig.solidColor = solidColor;
    colorConfig.planarProjectionRadius = planarProjectionRadius;

    const PointCloudPipelineLegend legend = PointCloudColorizer::apply(frame.points, colorConfig);
    if (pointCloudView) {
        pointCloudView->setLegend(legend.mode, legend.minValue, legend.maxValue, legend.visible);
    }

    PointCloudFilter::Config filterConfig;
    filterConfig.showNoisePoints = showNoisePoints;
    filterConfig.removeNoisePoints = removeNoisePoints;
    filterConfig.noiseTags = noiseFilterTags;
    frame.points = PointCloudFilter::apply(frame.points, filterConfig);
}

bool LivoxViewerWindow::savePointCloudAsLAS(const QString& filePath, const QVector<PointCloudPoint>& points)
{
    return PointCloudExport::saveAsLAS(filePath, points);
}

bool LivoxViewerWindow::savePointCloudAsPCD(const QString& filePath, const QVector<PointCloudPoint>& points)
{
    return PointCloudExport::saveAsPCD(filePath, points);
}

void LivoxViewerWindow::onRenderTick()
{
    if (playbackState.active) {
        {
            QMutexLocker locker(&frameMutex);
            pendingFrames.clear();
            lastSeenTimestamp.clear();
        }
        if (pointCloudView) {
            pointCloudView->update();
        }
        return;
    }

	// 暂停可视化模式：停止更新点云缓冲，但仍按固定刷新率重绘以跟随相机/叠加层
	if (!pointCloudVisualizationEnabled) {
		{
			QMutexLocker locker(&frameMutex);
			for (auto it = pendingFrames.begin(); it != pendingFrames.end(); ++it) {
				it.value().clear();
			}
		}
		if (pointCloudView) {
			pointCloudView->update();
		}
		return;
	}
	
	// 测距模式：暂停点云可视化播放（停止更新点云缓冲），但仍按固定刷新率重绘以跟随相机/叠加层
	if (pointCloudView && pointCloudView->isMeasurementModeEnabled()) {
		{
			QMutexLocker locker(&frameMutex);
			for (auto it = pendingFrames.begin(); it != pendingFrames.end(); ++it) {
				it.value().clear();
			}
		}
		pointCloudView->update();
		return;
	}

    // 获取当前选中的设备句柄
    uint32_t targetHandle = 0;
    bool hasTarget = false;
    {
        // 加锁以安全访问 currentLidarDevice
        QMutexLocker devLocker(&lidarDeviceMutex);
        if (currentLidarDevice) {
            targetHandle = currentLidarDevice->handle;
            hasTarget = true;
        }
    }

// 计算当前显示时间戳（建议也改为只基于当前设备的时间戳，避免多设备时间不同步导致的滑动窗口异常）
    uint64_t now_ns = 0;
    {
        QMutexLocker locker(&frameMutex);
        // 修改：只获取目标设备的时间戳，如果没有目标设备则保持原有逻辑或跳过
        if (hasTarget && lastSeenTimestamp.contains(targetHandle)) {
            now_ns = lastSeenTimestamp[targetHandle];
        } else if (!hasTarget) {
            // 如果没有选中设备，保持原有逻辑寻找最大时间戳（或者直接 return）
            for (auto it = lastSeenTimestamp.begin(); it != lastSeenTimestamp.end(); ++it) {
                if (it.value() > now_ns) now_ns = it.value();
            }
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
            // 过滤：如果有点中的设备，且当前迭代的设备句柄不等于选中设备的句柄，则跳过
            if (hasTarget && it.key() != targetHandle) {
                continue;
            }
            
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
        applyPointCloudPipeline(merged);

		// 保存PCD：在渲染循环中，当开启保存任务时按帧保存
		if (captureState.pcdSaveActive && captureState.pcdFramesRemaining > 0) {
			// 用合并窗口末尾时间戳作为文件名（纳秒）
			if (captureState.pcdLastSavedTimestamp != now_ns) {
				QString fileName = QString::number(now_ns) + ".pcd";
				QString filePath = QDir(captureState.pcdSaveDir).filePath(fileName);
				if (savePointCloudAsPCD(filePath, merged.points)) {
					logMessage(QString("PCD保存: %1").arg(QDir::toNativeSeparators(filePath)));
					captureState.pcdLastSavedTimestamp = now_ns;
					captureState.pcdFramesRemaining--;
					if (captureState.pcdFramesRemaining <= 0) {
						captureState.pcdSaveActive = false;
						statusLabelBar->setText("PCD保存完成");
					}
				} else {
					logMessage(QString("PCD保存失败: %1").arg(QDir::toNativeSeparators(filePath)));
					// 即使失败也避免卡住
					captureState.pcdLastSavedTimestamp = now_ns;
					captureState.pcdFramesRemaining--;
				}
			}
		}
		// 保存LAS：与PCD一致的触发策略
		if (captureState.lasSaveActive && captureState.lasFramesRemaining > 0) {
			if (captureState.lasLastSavedTimestamp != now_ns) {
				QString fileName = QString::number(now_ns) + ".las";
				QString filePath = QDir(captureState.lasSaveDir).filePath(fileName);
				if (savePointCloudAsLAS(filePath, merged.points)) {
					logMessage(QString("LAS保存: %1").arg(QDir::toNativeSeparators(filePath)));
					captureState.lasLastSavedTimestamp = now_ns;
					captureState.lasFramesRemaining--;
					if (captureState.lasFramesRemaining <= 0) {
						captureState.lasSaveActive = false;
						statusLabelBar->setText("LAS保存完成");
					}
				} else {
					logMessage(QString("LAS保存失败: %1").arg(QDir::toNativeSeparators(filePath)));
					captureState.lasLastSavedTimestamp = now_ns;
					captureState.lasFramesRemaining--;
				}
			}
		}
		presentPointCloudFrame(merged);
	}

	if (selectionRealtimeEnabled && pointCloudView && (attrTable || selectionTable)) {
		updateSelectionTableAndLog();
	}
}

void LivoxViewerWindow::onMeasurementUpdated()
{
	if (!pointCloudView) return;
	if (pointCloudView->hasMeasureP1() && !pointCloudView->hasMeasureP2()) {
		statusLabelBar->setText("测距：已选择第一点，按住Ctrl+左键选择第二点");
	} else if (pointCloudView->hasMeasureP1() && pointCloudView->hasMeasureP2()) {
		double d = pointCloudView->getMeasureDistance();
		statusLabelBar->setText(QString("测距结果：%1 m").arg(d, 0, 'f', 3));
		logMessage(QString("测距完成：%1 m").arg(d, 0, 'f', 3));
	} else {
		statusLabelBar->setText("测距模式：按住Ctrl+左键选择第一点");
	}
}

void LivoxViewerWindow::onPointSizeChanged(int px)
{
    pointSizePx = static_cast<float>(px);
    if (pointCloudView) pointCloudView->setPointSize(pointSizePx);
}

void LivoxViewerWindow::onColorModeChanged(int index)
{
    colorMode = index;
    if (solidColorRow) {
        solidColorRow->setEnabled(colorMode == ColorSolid);
    }
    if (pointCloudView) {
        if (colorMode == ColorByReflectivity) {
            pointCloudView->setLegend(ColorByReflectivity, 0.0f, 255.0f, true);
        } else if (colorMode == ColorByDistance) {
            pointCloudView->setLegend(ColorByDistance, 0.0f, 1.0f, true);
        } else if (colorMode == ColorByElevation) {
            pointCloudView->setLegend(ColorByElevation, -1.0f, 1.0f, true);
        } else if (colorMode == ColorSolid) {
            pointCloudView->setLegend(ColorSolid, 0.0f, 1.0f, false);
        } else if (colorMode == ColorByPlanarProjection) {
            pointCloudView->setLegend(ColorByPlanarProjection, 0.0f, 1.0f, true);
        }
    }
}

void LivoxViewerWindow::onSolidColorClicked()
{
    QColor c = QColorDialog::getColor(solidColor, this, "选择点云颜色");
    if (!c.isValid()) return;
    solidColor = c;
    if (solidColorPreview) {
        solidColorPreview->setStyleSheet(QString("background-color: %1;").arg(solidColor.name()));
    }
}

void LivoxViewerWindow::onProjectionDepthChanged(double meters)
{
    if (meters < 0.0) meters = 0.0;
    projectionDepthMeters = static_cast<float>(meters);
}

void LivoxViewerWindow::onProjectionDepthToggled(bool enabled)
{
    projectionDepthEnabled = enabled;
    if (projectionDepthSpin) {
        projectionDepthSpin->setEnabled(enabled);
    }
    logMessage(enabled ? "深度投影已启用" : "深度投影已关闭");
}

void LivoxViewerWindow::onPlanarProjectionToggled(bool enabled)
{
    planarProjectionEnabled = enabled;
    if (enabled) {
        logMessage("平面投影模式已启用");
        statusLabelBar->setText("平面投影模式已启用");
        // 自动调整视角以便观察平面投影
        if (pointCloudView) {
            pointCloudView->resetView();
            // 设置平面投影视角以便观察
            pointCloudView->setTopDownView();
        }
    } else {
        logMessage("平面投影模式已关闭");
        statusLabelBar->setText("平面投影模式已关闭");
        // 恢复默认视角
        if (pointCloudView) {
            pointCloudView->resetView();
        }
    }
    if (planarRadiusSpin) {
        planarRadiusSpin->setEnabled(enabled);
    }
}

void LivoxViewerWindow::onPlanarProjectionRadiusChanged(double radius)
{
    if (radius < 1.0) radius = 1.0;
    planarProjectionRadius = static_cast<float>(radius);
    logMessage(QString("平面投影半径已设置为 %1 m").arg(radius));
}

void LivoxViewerWindow::onPointCloudVisualizationToggled(bool enabled)
{
    pointCloudVisualizationEnabled = enabled;
    if (enabled) {
        logMessage("点云可视化已开启");
    } else {
        logMessage("点云可视化已暂停");
    }
} 

void LivoxViewerWindow::updateSelectionTableAndLog()
{
    QVector<PointCloudPoint> pts;
    if (pointCloudView->hasSelectionAabb()) {
        pts = pointCloudView->pointsInPersistSelection(20000000);
    } else {
        QRect sel = pointCloudView->currentSelectionRect();
        if (!sel.isEmpty()) pts = pointCloudView->pointsInRect(sel, 20000000);
    }

    QTableWidget* table = attrTable ? attrTable : selectionTable;
    if (!table) return;

    if (!pts.isEmpty()) {
        int totalCount = pts.size();
        int zeroCount = 0;
        for (const PointCloudPoint& p : pts) {
            if (p.x == 0.0f && p.y == 0.0f && p.z == 0.0f) {
                ++zeroCount;
            }
        }
        int validCount = totalCount - zeroCount;

        if (totalCount != lastSelectionCount) {
            lastSelectionCount = totalCount;
            logMessage(QString("框选点云个数: %1, 零点个数: %2, 总数: %3")
                       .arg(validCount).arg(zeroCount).arg(totalCount));
        }

        bool sorting = table->isSortingEnabled();
        table->setSortingEnabled(false);
        table->clearContents();
        table->setRowCount(0);
        const int maxRows = 500;
        int rows = 0;
        for (const PointCloudPoint& p : pts) {
            if (rows >= maxRows) break;
            int row = table->rowCount();
            table->insertRow(row);
            auto* i0 = new NumberItem(row + 1);
            auto* i1 = new NumberItem(p.x, 3);
            auto* i2 = new NumberItem(p.y, 3);
            auto* i3 = new NumberItem(p.z, 3);
            auto* i4 = new NumberItem(static_cast<int>(p.reflectivity));
            auto* i5 = new NumberItem(static_cast<int>(p.tag));
            i0->setTextAlignment(Qt::AlignCenter);
            i1->setTextAlignment(Qt::AlignCenter);
            i2->setTextAlignment(Qt::AlignCenter);
            i3->setTextAlignment(Qt::AlignCenter);
            i4->setTextAlignment(Qt::AlignCenter);
            i5->setTextAlignment(Qt::AlignCenter);
            table->setItem(row, 0, i0);
            table->setItem(row, 1, i1);
            table->setItem(row, 2, i2);
            table->setItem(row, 3, i3);
            table->setItem(row, 4, i4);
            table->setItem(row, 5, i5);
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

void LivoxViewerWindow::onSelectionFinished()
{
    if (!pointCloudView || !(attrTable || selectionTable)) return;
    updateSelectionTableAndLog();
} 

void LivoxViewerWindow::startLvx2Recording(const QString& filePath, int durationSec)
{
    QMutexLocker lk(&captureState.lvx2Mutex);
    if (captureState.lvx2SaveActive) return;
    captureState.lvx2File.setFileName(filePath);
    if (!captureState.lvx2File.open(QIODevice::WriteOnly)) {
        logMessage("打开LVX2文件失败");
        captureState.current = CaptureNone;
        return;
    }
    // 写头
    Lvx2PublicHeader pub;
    captureState.lvx2File.write(reinterpret_cast<const char*>(&pub), sizeof(pub));
    Lvx2PrivateHeader pri;
    captureState.lvx2File.write(reinterpret_cast<const char*>(&pri), sizeof(pri));
    Lvx2DeviceInfo dev{};
    QByteArray snb = currentLidarDevice ? currentLidarDevice->sn.left(15).toLatin1() : QByteArray("Unknown");
    std::memset(dev.lidar_sn, 0, sizeof(dev.lidar_sn));
    std::memcpy(dev.lidar_sn, snb.constData(), std::min<size_t>(size_t(snb.size()), sizeof(dev.lidar_sn)));
    dev.lidar_id = currentLidarDevice ? currentLidarDevice->handle : 0;
    dev.device_type = currentLidarDevice ? currentLidarDevice->dev_type : 0;
    captureState.lvx2File.write(reinterpret_cast<const char*>(&dev), sizeof(dev));

    captureState.lvx2SaveActive = true;
    captureState.lvx2FrameStartNs = 0;
    captureState.lvx2FrameIndex = 0;
    // 借用采集计时器
    captureState.secondsRemaining = durationSec;
    captureState.progress->setValue(0);
    captureState.progress->setFormat("录制中 %p% (%v s)");
}

void LivoxViewerWindow::stopLvx2Recording(bool flushPending)
{
    QMutexLocker lk(&captureState.lvx2Mutex);
    if (!captureState.lvx2SaveActive) return;
    Q_UNUSED(flushPending);
    captureState.lvx2SaveActive = false;
    if (captureState.lvx2File.isOpen()) captureState.lvx2File.close();
}

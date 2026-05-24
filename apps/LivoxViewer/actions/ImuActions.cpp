#include "LivoxViewerWindow.h"

#include <QChartView>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTextStream>
#include <QVBoxLayout>
#include <QValueAxis>

#include <algorithm>

class YAxisZoomChartView : public QChartView {
public:
    explicit YAxisZoomChartView(QChart *chart, QWidget *parent = nullptr)
        : QChartView(chart, parent), m_isDragging(false) {
        setRenderHint(QPainter::Antialiasing);
        setMouseTracking(true);
        viewport()->setCursor(Qt::OpenHandCursor);
    }

protected:
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

    void mousePressEvent(QMouseEvent *event) override {
        if (event->button() == Qt::LeftButton) {
            if (chart() && chart()->plotArea().contains(event->position())) {
                m_isDragging = true;
                m_lastMousePos = event->position();
                viewport()->setCursor(Qt::ClosedHandCursor);
                event->accept();
                return;
            }
        }
        QChartView::mousePressEvent(event);
    }

    void mouseMoveEvent(QMouseEvent *event) override {
        if (m_isDragging) {
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
            if (chart() && chart()->plotArea().contains(event->position())) {
                viewport()->setCursor(Qt::OpenHandCursor);
            } else {
                viewport()->setCursor(Qt::ArrowCursor);
            }
        }
        QChartView::mouseMoveEvent(event);
    }

    void mouseReleaseEvent(QMouseEvent *event) override {
        if (event->button() == Qt::LeftButton) {
            m_isDragging = false;
            if (chart() && chart()->plotArea().contains(event->position())) {
                viewport()->setCursor(Qt::OpenHandCursor);
            }
            event->accept();
        }
        QChartView::mouseReleaseEvent(event);
    }

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
    if (imuState.displayRunning.exchange(!imuState.displayRunning.load())) {
        if (imuState.displayThread.joinable()) {
            imuState.displayThread.detach();
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
            for (int i = 0; i < 10 && imuState.displayRunning.load(); ++i) {
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
    imuState.chartWindow = new QWidget(this, Qt::Window);
    imuState.chartWindow->setAttribute(Qt::WA_DeleteOnClose);
    imuState.chartWindow->setWindowTitle("IMU数据曲线");
    QVBoxLayout* layout = new QVBoxLayout(imuState.chartWindow);

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

    if (imuState.chartRunning.exchange(true)) {
        if (imuState.chartThread.joinable()) imuState.chartThread.detach();
    }
    imuState.chartThread = std::thread([this]() {
        double t = 0.0;
        const double dt = 0.05;
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
            for (int i = 0; i < 5 && imuState.chartRunning.load(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    });

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

    QWidget* row1 = new QWidget(&dlg);
    QHBoxLayout* h1 = new QHBoxLayout(row1);
    h1->setContentsMargins(0,0,0,0);
    QLabel* lblPath = new QLabel("请选择保存路径:", row1);
    QLineEdit* editPath = new QLineEdit(row1);
    editPath->setText(lastDir);
    QPushButton* btnBrowse = new QPushButton("选择", row1);
    h1->addWidget(lblPath);
    h1->addSpacing(8);
    h1->addWidget(editPath, 1);
    h1->addSpacing(8);
    h1->addWidget(btnBrowse);
    v->addWidget(row1);

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

    QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    v->addWidget(box);

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

    settings.setValue("save/lastIMUDir", baseDir);

    QString sn = currentLidarDevice ? currentLidarDevice->sn : QString("Unknown");
    QString targetDir = QDir(baseDir).filePath(QString("IMU_%1").arg(sn));
    QDir().mkpath(targetDir);
    QString startTime = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString filePath = QDir(targetDir).filePath(QString("%1_%2.csv").arg(sn, startTime));

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

    if (captureState.progress) {
        captureState.progress->setRange(0, 100);
        captureState.progress->setValue(0);
        captureState.progress->setFormat("IMU采集中 %p% (%v s)");
    }

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

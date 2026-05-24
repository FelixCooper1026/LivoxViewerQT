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
#include <QMargins>
#include <QMouseEvent>
#include <QMessageBox>
#include <QPen>
#include <QPushButton>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>
#include <QValueAxis>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace {

constexpr double kImuChartWindowSec = 5.0;
constexpr int kImuChartRefreshMs = 50;
constexpr double kGyroMin = -50.0;
constexpr double kGyroMax = 50.0;
constexpr double kAccMin = -4.0;
constexpr double kAccMax = 4.0;

const QColor kAxisXColor(214, 64, 69);
const QColor kAxisYColor(45, 156, 99);
const QColor kAxisZColor(47, 111, 196);

class YAxisZoomChartView : public QChartView {
public:
    explicit YAxisZoomChartView(QChart *chart, QWidget *parent = nullptr)
        : QChartView(chart, parent), m_isDragging(false) {
        setRenderHint(QPainter::Antialiasing);
        setMouseTracking(true);
        viewport()->setCursor(Qt::OpenHandCursor);
    }

    void setHoverCallback(std::function<void(double)> callback) {
        m_hoverCallback = std::move(callback);
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
                emitHoverValue(event->position());
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
    void emitHoverValue(const QPointF& mousePos) {
        if (!m_hoverCallback || !chart()) {
            return;
        }
        QValueAxis* axisX = getXAxis();
        if (!axisX) {
            return;
        }
        const QRectF plotArea = chart()->plotArea();
        if (plotArea.width() <= 0.0) {
            return;
        }
        const double relative = (mousePos.x() - plotArea.left()) / plotArea.width();
        const double xValue = axisX->min() + relative * (axisX->max() - axisX->min());
        m_hoverCallback(xValue);
    }

    QValueAxis* getXAxis() {
        if (!chart()) return nullptr;
        auto axes = chart()->axes(Qt::Horizontal);
        if (axes.isEmpty()) return nullptr;
        return qobject_cast<QValueAxis*>(axes.first());
    }

    QValueAxis* getYAxis() {
        if (!chart()) return nullptr;
        auto axes = chart()->axes(Qt::Vertical);
        if (axes.isEmpty()) return nullptr;
        return qobject_cast<QValueAxis*>(axes.first());
    }

    bool m_isDragging;
    QPointF m_lastMousePos;
    std::function<void(double)> m_hoverCallback;
};

void configureImuSeries(QLineSeries* series, const QString& name, const QColor& color)
{
    series->setName(name);
    QPen pen(color);
    pen.setWidthF(1.8);
    series->setPen(pen);
    series->setUseOpenGL(false);
}

void configureImuChart(QChart* chart, const QString& title)
{
    chart->setTitle(title);
    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignTop);
    chart->setMargins(QMargins(8, 8, 8, 8));
}

void configureImuAxis(QValueAxis* axis, const QString& title, double minValue, double maxValue)
{
    axis->setTitleText(title);
    axis->setRange(minValue, maxValue);
    axis->setGridLineVisible(true);
    axis->setMinorGridLineVisible(true);
    axis->setLabelFormat("%.2f");
}

QList<QPointF> makeImuPoints(const QVector<ImuChartSample>& samples, double startSec, double endSec, double ImuChartSample::*field)
{
    QList<QPointF> points;
    points.reserve(samples.size());
    for (const ImuChartSample& sample : samples) {
        if (sample.timestampSec >= startSec && sample.timestampSec <= endSec) {
            points.append(QPointF(sample.timestampSec, sample.*field));
        }
    }
    return points;
}

bool findNearestImuSample(const QVector<ImuChartSample>& samples, double timestampSec, ImuChartSample* nearest)
{
    if (samples.isEmpty() || !nearest) {
        return false;
    }

    const ImuChartSample* best = nullptr;
    double bestDistance = std::numeric_limits<double>::max();
    for (const ImuChartSample& sample : samples) {
        const double distance = std::abs(sample.timestampSec - timestampSec);
        if (distance < bestDistance) {
            bestDistance = distance;
            best = &sample;
        }
    }

    if (!best || bestDistance > 0.25) {
        return false;
    }

    *nearest = *best;
    return true;
}

} // namespace

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
                    if (imuState.asciiLabel) imuState.asciiLabel->setText(buildImuAscii(gx, gy, gz, ax, ay, az));
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
    layout->setContentsMargins(12, 12, 12, 12);
    layout->setSpacing(8);

    QWidget* controlBar = new QWidget(imuState.chartWindow);
    QHBoxLayout* controls = new QHBoxLayout(controlBar);
    controls->setContentsMargins(0, 0, 0, 0);
    controls->setSpacing(8);
    imuState.chartResetButton = new QPushButton("重置缩放", controlBar);
    imuState.chartClearButton = new QPushButton("清空曲线", controlBar);
    imuState.chartPauseButton = new QPushButton("暂停", controlBar);
    imuState.chartHoverLabel = new QLabel("悬停查看 X/Y/Z 数值", controlBar);
    imuState.chartHoverLabel->setMinimumWidth(460);
    controls->addWidget(imuState.chartResetButton);
    controls->addWidget(imuState.chartClearButton);
    controls->addWidget(imuState.chartPauseButton);
    controls->addStretch();
    controls->addWidget(imuState.chartHoverLabel);
    layout->addWidget(controlBar);

    imuState.gyroChart = new QChart();
    configureImuChart(imuState.gyroChart, "Gyro(rad/s)");
    imuState.gyroSeriesX = new QLineSeries(); configureImuSeries(imuState.gyroSeriesX, "X", kAxisXColor);
    imuState.gyroSeriesY = new QLineSeries(); configureImuSeries(imuState.gyroSeriesY, "Y", kAxisYColor);
    imuState.gyroSeriesZ = new QLineSeries(); configureImuSeries(imuState.gyroSeriesZ, "Z", kAxisZColor);
    imuState.gyroChart->addSeries(imuState.gyroSeriesX);
    imuState.gyroChart->addSeries(imuState.gyroSeriesY);
    imuState.gyroChart->addSeries(imuState.gyroSeriesZ);
    imuState.gyroAxisX = new QValueAxis(); configureImuAxis(imuState.gyroAxisX, "时间 (s)", 0.0, kImuChartWindowSec);
    imuState.gyroAxisY = new QValueAxis(); configureImuAxis(imuState.gyroAxisY, "Gyro(rad/s)", kGyroMin, kGyroMax);
    imuState.gyroChart->addAxis(imuState.gyroAxisX, Qt::AlignBottom);
    imuState.gyroChart->addAxis(imuState.gyroAxisY, Qt::AlignLeft);
    for (auto s : {imuState.gyroSeriesX, imuState.gyroSeriesY, imuState.gyroSeriesZ}) { s->attachAxis(imuState.gyroAxisX); s->attachAxis(imuState.gyroAxisY); }
    auto* gyroView = new YAxisZoomChartView(imuState.gyroChart, imuState.chartWindow);
    imuState.gyroChartView = gyroView;

    imuState.accChart = new QChart();
    configureImuChart(imuState.accChart, "Acc(g)");
    imuState.accSeriesX = new QLineSeries(); configureImuSeries(imuState.accSeriesX, "X", kAxisXColor);
    imuState.accSeriesY = new QLineSeries(); configureImuSeries(imuState.accSeriesY, "Y", kAxisYColor);
    imuState.accSeriesZ = new QLineSeries(); configureImuSeries(imuState.accSeriesZ, "Z", kAxisZColor);
    imuState.accChart->addSeries(imuState.accSeriesX);
    imuState.accChart->addSeries(imuState.accSeriesY);
    imuState.accChart->addSeries(imuState.accSeriesZ);
    imuState.accAxisX = new QValueAxis(); configureImuAxis(imuState.accAxisX, "时间 (s)", 0.0, kImuChartWindowSec);
    imuState.accAxisY = new QValueAxis(); configureImuAxis(imuState.accAxisY, "Acc(g)", kAccMin, kAccMax);
    imuState.accChart->addAxis(imuState.accAxisX, Qt::AlignBottom);
    imuState.accChart->addAxis(imuState.accAxisY, Qt::AlignLeft);
    for (auto s : {imuState.accSeriesX, imuState.accSeriesY, imuState.accSeriesZ}) { s->attachAxis(imuState.accAxisX); s->attachAxis(imuState.accAxisY); }
    auto* accView = new YAxisZoomChartView(imuState.accChart, imuState.chartWindow);
    imuState.accChartView = accView;

    layout->addWidget(imuState.gyroChartView);
    layout->addWidget(imuState.accChartView);
    imuState.chartWindow->setLayout(layout);

    auto refreshCharts = [this]() {
        if (!imuState.chartWindow || imuState.chartPaused) {
            return;
        }

        QVector<ImuChartSample> samples;
        {
            QMutexLocker lk(&imuState.chartSamplesMutex);
            samples = imuState.chartSamples;
        }

        const double latestSec = samples.isEmpty() ? 0.0 : samples.last().timestampSec;
        const double endSec = std::max(kImuChartWindowSec, latestSec);
        const double startSec = std::max(0.0, endSec - kImuChartWindowSec);

        if (imuState.gyroSeriesX) imuState.gyroSeriesX->replace(makeImuPoints(samples, startSec, endSec, &ImuChartSample::gx));
        if (imuState.gyroSeriesY) imuState.gyroSeriesY->replace(makeImuPoints(samples, startSec, endSec, &ImuChartSample::gy));
        if (imuState.gyroSeriesZ) imuState.gyroSeriesZ->replace(makeImuPoints(samples, startSec, endSec, &ImuChartSample::gz));
        if (imuState.accSeriesX) imuState.accSeriesX->replace(makeImuPoints(samples, startSec, endSec, &ImuChartSample::ax));
        if (imuState.accSeriesY) imuState.accSeriesY->replace(makeImuPoints(samples, startSec, endSec, &ImuChartSample::ay));
        if (imuState.accSeriesZ) imuState.accSeriesZ->replace(makeImuPoints(samples, startSec, endSec, &ImuChartSample::az));
        if (imuState.gyroAxisX) imuState.gyroAxisX->setRange(startSec, endSec);
        if (imuState.accAxisX) imuState.accAxisX->setRange(startSec, endSec);
    };

    auto updateHoverLabel = [this](double timestampSec) {
        QVector<ImuChartSample> samples;
        {
            QMutexLocker lk(&imuState.chartSamplesMutex);
            samples = imuState.chartSamples;
        }

        ImuChartSample sample;
        if (!findNearestImuSample(samples, timestampSec, &sample)) {
            if (imuState.chartHoverLabel) {
                imuState.chartHoverLabel->setText("悬停查看 X/Y/Z 数值");
            }
            return;
        }

        if (imuState.chartHoverLabel) {
            imuState.chartHoverLabel->setText(
                QString("t=%1s  Gyro X:%2 Y:%3 Z:%4  Acc X:%5 Y:%6 Z:%7")
                    .arg(sample.timestampSec, 0, 'f', 2)
                    .arg(sample.gx, 0, 'f', 3)
                    .arg(sample.gy, 0, 'f', 3)
                    .arg(sample.gz, 0, 'f', 3)
                    .arg(sample.ax, 0, 'f', 3)
                    .arg(sample.ay, 0, 'f', 3)
                    .arg(sample.az, 0, 'f', 3));
        }
    };

    gyroView->setHoverCallback(updateHoverLabel);
    accView->setHoverCallback(updateHoverLabel);

    connect(imuState.chartResetButton, &QPushButton::clicked, this, [this]() {
        if (imuState.gyroAxisY) imuState.gyroAxisY->setRange(kGyroMin, kGyroMax);
        if (imuState.accAxisY) imuState.accAxisY->setRange(kAccMin, kAccMax);
    });

    connect(imuState.chartClearButton, &QPushButton::clicked, this, [this]() {
        {
            QMutexLocker lk(&imuState.chartSamplesMutex);
            imuState.chartSamples.clear();
            imuState.chartTimeOriginSec = -1.0;
        }
        for (QLineSeries* series : {imuState.gyroSeriesX, imuState.gyroSeriesY, imuState.gyroSeriesZ,
                                    imuState.accSeriesX, imuState.accSeriesY, imuState.accSeriesZ}) {
            if (series) series->clear();
        }
        if (imuState.gyroAxisX) imuState.gyroAxisX->setRange(0.0, kImuChartWindowSec);
        if (imuState.accAxisX) imuState.accAxisX->setRange(0.0, kImuChartWindowSec);
        if (imuState.chartHoverLabel) imuState.chartHoverLabel->setText("悬停查看 X/Y/Z 数值");
    });

    connect(imuState.chartPauseButton, &QPushButton::clicked, this, [this]() {
        imuState.chartPaused = !imuState.chartPaused;
        if (imuState.chartPauseButton) {
            imuState.chartPauseButton->setText(imuState.chartPaused ? "继续" : "暂停");
        }
    });

    imuState.chartPaused = false;
    imuState.chartRefreshTimer = new QTimer(imuState.chartWindow);
    imuState.chartRefreshTimer->setInterval(kImuChartRefreshMs);
    connect(imuState.chartRefreshTimer, &QTimer::timeout, this, refreshCharts);
    imuState.chartRefreshTimer->start();

    connect(imuState.chartWindow, &QObject::destroyed, this, [this]() {
        if (imuState.chartRefreshTimer) {
            imuState.chartRefreshTimer->stop();
        }
        imuState.gyroChart = nullptr; imuState.gyroChartView = nullptr; imuState.gyroSeriesX = imuState.gyroSeriesY = imuState.gyroSeriesZ = nullptr; imuState.gyroAxisX = imuState.gyroAxisY = nullptr;
        imuState.accChart = nullptr; imuState.accChartView = nullptr; imuState.accSeriesX = imuState.accSeriesY = imuState.accSeriesZ = nullptr; imuState.accAxisX = imuState.accAxisY = nullptr;
        imuState.chartRefreshTimer = nullptr;
        imuState.chartPauseButton = nullptr;
        imuState.chartClearButton = nullptr;
        imuState.chartResetButton = nullptr;
        imuState.chartHoverLabel = nullptr;
        imuState.chartPaused = false;
        imuState.chartWindow = nullptr;
    });

    refreshCharts();
    imuState.chartWindow->resize(980, 700);
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

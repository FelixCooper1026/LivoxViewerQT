#include "dialogs/ImuVisualizationDialog.h"

#include "LivoxViewerWindow.h"
#include "widgets/ImuOrientationView.h"
#include "widgets/SwitchCheckBox.h"

#include <QApplication>
#include <QAbstractItemView>
#include <QEvent>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPalette>
#include <QPainter>
#include <QPen>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStyle>
#include <QTimer>
#include <QVariant>
#include <QVBoxLayout>

#include <QtCharts/QLegend>

#include <algorithm>
#include <limits>

namespace {

constexpr double kVisibleWindowSec = 5.0;
constexpr int kRefreshIntervalMs = 50;

const QColor kAxisXColor(214, 64, 69);
const QColor kAxisYColor(45, 156, 99);
const QColor kAxisZColor(47, 111, 196);

bool isDarkPalette(const QPalette& palette)
{
    return palette.color(QPalette::Window).lightness() < 128;
}

QString colorName(const QColor& color)
{
    return color.name(QColor::HexRgb);
}

void configureSeries(QLineSeries* series, const QString& name, const QColor& color)
{
    series->setName(name);
    QPen pen(color);
    pen.setWidthF(1.8);
    series->setPen(pen);
    series->setUseOpenGL(false);
}

void configureAxis(QValueAxis* axis, const QString& title, double minValue, double maxValue)
{
    axis->setTitleText(title);
    axis->setRange(minValue, maxValue);
    axis->setGridLineVisible(true);
    axis->setMinorGridLineVisible(true);
    axis->setLabelFormat("%.2f");
}

QList<QPointF> makePoints(const QVector<ImuVisualizationSample>& samples,
                          double startSec,
                          double endSec,
                          double ImuVisualizationSample::*field)
{
    QList<QPointF> points;
    points.reserve(samples.size());
    for (const ImuVisualizationSample& sample : samples) {
        if (sample.timestampSec >= startSec && sample.timestampSec <= endSec) {
            points.append(QPointF(sample.timestampSec, sample.*field));
        }
    }
    return points;
}

} // namespace

ImuVisualizationDialog::ImuVisualizationDialog(LivoxViewerWindow* owner)
    : QDialog(owner)
    , m_owner(owner)
{
    setObjectName(QStringLiteral("ImuVisualizationDialog"));
    setWindowFlags(windowFlags() | Qt::WindowMinMaxButtonsHint);
    setWindowTitle(QStringLiteral("IMU数据可视化"));
    setMinimumSize(980, 640);

    QVBoxLayout* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 10, 10, 10);
    root->setSpacing(8);
    root->addWidget(createToolbar());

    QWidget* content = new QWidget(this);
    QHBoxLayout* contentLayout = new QHBoxLayout(content);
    contentLayout->setContentsMargins(0, 0, 0, 0);
    contentLayout->setSpacing(8);

    m_deviceList = new QListWidget(content);
    m_deviceList->setObjectName(QStringLiteral("ImuDeviceList"));
    m_deviceList->setFixedWidth(220);
    m_deviceList->setSpacing(6);
    m_deviceList->setFrameShape(QFrame::NoFrame);
    m_deviceList->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_deviceList->setSelectionMode(QAbstractItemView::SingleSelection);
    connect(m_deviceList, &QListWidget::currentRowChanged, this, [this](int row) {
        QListWidgetItem* item = m_deviceList->item(row);
        if (!item) {
            return;
        }
        m_currentHandle = item->data(Qt::UserRole).toUInt();
        m_haveCurrentHandle = true;
        updateDeviceCardSelection();
        updateOrientationModel();
        refreshData();
    });
    contentLayout->addWidget(m_deviceList);

    QWidget* visualizationArea = new QWidget(content);
    QGridLayout* grid = new QGridLayout(visualizationArea);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setSpacing(8);
    m_accChart = createChartPanel(QStringLiteral("加速度数据"), QStringLiteral("Acc (g)"), -4.0, 4.0);
    m_gyroChart = createChartPanel(QStringLiteral("角速度数据"), QStringLiteral("Gyro (rad/s)"), -50.0, 50.0);
    m_attitudeChart = createChartPanel(QStringLiteral("姿态角（软件解算）"), QStringLiteral("Angle (deg)"), -180.0, 180.0);
    m_attitudeChart.seriesX->setName(QStringLiteral("Roll"));
    m_attitudeChart.seriesY->setName(QStringLiteral("Pitch"));
    m_attitudeChart.seriesZ->setName(QStringLiteral("Yaw"));
    grid->addWidget(m_accChart.panel, 0, 0);
    grid->addWidget(m_gyroChart.panel, 0, 1);
    grid->addWidget(m_attitudeChart.panel, 1, 0);
    grid->addWidget(createOrientationPanel(), 1, 1);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);
    grid->setRowStretch(0, 1);
    grid->setRowStretch(1, 1);
    contentLayout->addWidget(visualizationArea, 1);
    root->addWidget(content, 1);

    m_refreshTimer = new QTimer(this);
    m_refreshTimer->setInterval(kRefreshIntervalMs);
    connect(m_refreshTimer, &QTimer::timeout, this, [this]() {
        refreshDeviceList();
        if (!m_paused) {
            refreshData();
        }
    });
    m_refreshTimer->start();

    refreshTheme();
    refreshDeviceList();
    refreshData();
}

void ImuVisualizationDialog::changeEvent(QEvent* event)
{
    QDialog::changeEvent(event);
    if (event->type() == QEvent::PaletteChange ||
        event->type() == QEvent::ApplicationPaletteChange) {
        refreshTheme();
    }
}

QWidget* ImuVisualizationDialog::createToolbar()
{
    QFrame* toolbar = new QFrame(this);
    toolbar->setObjectName(QStringLiteral("ImuToolbar"));
    QHBoxLayout* layout = new QHBoxLayout(toolbar);
    layout->setContentsMargins(8, 6, 8, 6);
    layout->setSpacing(8);

    QLabel* autoScaleLabel = new QLabel(QStringLiteral("自动缩放"), toolbar);
    m_autoScaleSwitch = new SwitchCheckBox(toolbar);
    m_autoScaleSwitch->setChecked(true);
    layout->addWidget(autoScaleLabel);
    layout->addWidget(m_autoScaleSwitch);
    layout->addSpacing(8);

    m_resetButton = new QPushButton(QStringLiteral("重置缩放"), toolbar);
    m_pauseButton = new QPushButton(QStringLiteral("暂停"), toolbar);
    connect(m_resetButton, &QPushButton::clicked, this, [this]() { resetZoom(); });
    connect(m_pauseButton, &QPushButton::clicked, this, [this]() { setPaused(!m_paused); });
    layout->addWidget(m_resetButton);
    layout->addWidget(m_pauseButton);
    layout->addStretch();

    return toolbar;
}

ImuVisualizationDialog::ChartPanel ImuVisualizationDialog::createChartPanel(const QString& title,
                                                                            const QString& yTitle,
                                                                            double defaultMin,
                                                                            double defaultMax)
{
    ChartPanel panel;
    panel.defaultMin = defaultMin;
    panel.defaultMax = defaultMax;

    QFrame* frame = new QFrame(this);
    frame->setObjectName(QStringLiteral("ImuVisualizationPanel"));
    QVBoxLayout* layout = new QVBoxLayout(frame);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(6);

    QWidget* titleRow = new QWidget(frame);
    QHBoxLayout* titleLayout = new QHBoxLayout(titleRow);
    titleLayout->setContentsMargins(0, 0, 0, 0);
    titleLayout->setSpacing(8);
    QLabel* titleLabel = new QLabel(title, titleRow);
    titleLabel->setObjectName(QStringLiteral("ImuPanelTitle"));
    panel.emptyLabel = new QLabel(QStringLiteral("等待 IMU 数据"), titleRow);
    panel.emptyLabel->setObjectName(QStringLiteral("ImuPanelHint"));
    titleLayout->addWidget(titleLabel);
    titleLayout->addStretch();
    titleLayout->addWidget(panel.emptyLabel);
    layout->addWidget(titleRow);

    panel.chart = new QChart();
    panel.chart->legend()->setVisible(true);
    panel.chart->legend()->setAlignment(Qt::AlignTop);
    panel.chart->setMargins(QMargins(4, 0, 4, 4));
    panel.seriesX = new QLineSeries();
    panel.seriesY = new QLineSeries();
    panel.seriesZ = new QLineSeries();
    configureSeries(panel.seriesX, QStringLiteral("X"), kAxisXColor);
    configureSeries(panel.seriesY, QStringLiteral("Y"), kAxisYColor);
    configureSeries(panel.seriesZ, QStringLiteral("Z"), kAxisZColor);
    panel.chart->addSeries(panel.seriesX);
    panel.chart->addSeries(panel.seriesY);
    panel.chart->addSeries(panel.seriesZ);
    panel.axisX = new QValueAxis();
    configureAxis(panel.axisX, QStringLiteral("时间 (s)"), 0.0, kVisibleWindowSec);
    panel.axisY = new QValueAxis();
    configureAxis(panel.axisY, yTitle, defaultMin, defaultMax);
    panel.chart->addAxis(panel.axisX, Qt::AlignBottom);
    panel.chart->addAxis(panel.axisY, Qt::AlignLeft);
    for (QLineSeries* series : {panel.seriesX, panel.seriesY, panel.seriesZ}) {
        series->attachAxis(panel.axisX);
        series->attachAxis(panel.axisY);
    }

    panel.view = new QChartView(panel.chart, frame);
    panel.view->setRenderHint(QPainter::Antialiasing);
    layout->addWidget(panel.view, 1);

    panel.panel = frame;
    return panel;
}

QWidget* ImuVisualizationDialog::createOrientationPanel()
{
    QFrame* frame = new QFrame(this);
    frame->setObjectName(QStringLiteral("ImuVisualizationPanel"));
    QVBoxLayout* layout = new QVBoxLayout(frame);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(6);

    QLabel* titleLabel = new QLabel(QStringLiteral("3D 模型姿态"), frame);
    titleLabel->setObjectName(QStringLiteral("ImuPanelTitle"));
    layout->addWidget(titleLabel);

    m_orientationView = new ImuOrientationView(frame);
    layout->addWidget(m_orientationView, 1);
    return frame;
}

QWidget* ImuVisualizationDialog::createDeviceCard(const ImuVisualizationDeviceDescriptor& device)
{
    QWidget* card = new QWidget(m_deviceList);
    card->setObjectName(QStringLiteral("ImuDeviceCard"));
    card->setProperty("handle", QVariant::fromValue(static_cast<quint32>(device.handle)));
    card->setProperty("selected", false);
    QVBoxLayout* layout = new QVBoxLayout(card);
    layout->setContentsMargins(10, 8, 10, 8);
    layout->setSpacing(3);

    QLabel* modelLabel = new QLabel(device.modelDisplay.isEmpty() ? QStringLiteral("未知型号") : device.modelDisplay, card);
    modelLabel->setObjectName(QStringLiteral("ImuDeviceModel"));
    QLabel* sourceLabel = new QLabel(
        device.source == ImuVisualizationSource::Realtime ? QStringLiteral("实时数据流") : QStringLiteral("离线数据流"),
        card);
    sourceLabel->setObjectName(QStringLiteral("ImuDeviceSourceTag"));
    QLabel* snLabel = new QLabel(device.serialNumber.isEmpty() ? QStringLiteral("SN: --") : QStringLiteral("SN: %1").arg(device.serialNumber), card);
    snLabel->setObjectName(QStringLiteral("ImuDeviceSn"));
    QLabel* ipLabel = new QLabel(device.ipAddress.isEmpty() ? QStringLiteral("IP: --") : QStringLiteral("IP: %1").arg(device.ipAddress), card);
    ipLabel->setObjectName(QStringLiteral("ImuDeviceIp"));
    layout->addWidget(modelLabel);
    layout->addWidget(sourceLabel);
    layout->addWidget(snLabel);
    layout->addWidget(ipLabel);
    return card;
}

void ImuVisualizationDialog::refreshDeviceList()
{
    const QVector<ImuVisualizationDeviceDescriptor> devices = m_owner->imuVisualizationDevicesSnapshot();
    bool rebuild = devices.size() != m_devicesByHandle.size();
    for (const ImuVisualizationDeviceDescriptor& device : devices) {
        const auto it = m_devicesByHandle.constFind(device.handle);
        if (it == m_devicesByHandle.constEnd() ||
            it->modelDisplay != device.modelDisplay ||
            it->serialNumber != device.serialNumber ||
            it->ipAddress != device.ipAddress ||
            it->source != device.source) {
            rebuild = true;
            break;
        }
    }

    bool currentStillExists = false;
    for (const ImuVisualizationDeviceDescriptor& device : devices) {
        if (m_haveCurrentHandle && device.handle == m_currentHandle) {
            currentStillExists = true;
            break;
        }
    }
    if (!devices.isEmpty() && (!m_haveCurrentHandle || !currentStillExists)) {
        m_currentHandle = devices.first().handle;
        m_haveCurrentHandle = true;
        rebuild = true;
    } else if (devices.isEmpty()) {
        m_haveCurrentHandle = false;
        m_currentHandle = 0;
    }

    if (rebuild) {
        QSignalBlocker blocker(m_deviceList);
        m_deviceList->clear();
        m_devicesByHandle.clear();
        int selectedRow = -1;
        for (int i = 0; i < devices.size(); ++i) {
            const ImuVisualizationDeviceDescriptor& device = devices.at(i);
            m_devicesByHandle.insert(device.handle, device);
            QListWidgetItem* item = new QListWidgetItem();
            item->setData(Qt::UserRole, QVariant::fromValue(static_cast<quint32>(device.handle)));
            item->setSizeHint(QSize(196, 92));
            m_deviceList->addItem(item);
            m_deviceList->setItemWidget(item, createDeviceCard(device));
            if (m_haveCurrentHandle && device.handle == m_currentHandle) {
                selectedRow = i;
            }
        }
        if (selectedRow >= 0) {
            m_deviceList->setCurrentRow(selectedRow);
        }
        updateOrientationModel();
    }

    updateDeviceCardSelection();
}

void ImuVisualizationDialog::updateDeviceCardSelection()
{
    for (int i = 0; i < m_deviceList->count(); ++i) {
        QListWidgetItem* item = m_deviceList->item(i);
        QWidget* card = m_deviceList->itemWidget(item);
        if (!item || !card) {
            continue;
        }
        const bool selected = m_haveCurrentHandle && item->data(Qt::UserRole).toUInt() == m_currentHandle;
        card->setProperty("selected", selected);
        card->style()->unpolish(card);
        card->style()->polish(card);
        card->update();
    }
}

void ImuVisualizationDialog::updateOrientationModel()
{
    if (!m_orientationView || !m_haveCurrentHandle || !m_devicesByHandle.contains(m_currentHandle)) {
        return;
    }
    const ImuVisualizationDeviceDescriptor device = m_devicesByHandle.value(m_currentHandle);
    m_orientationView->setDeviceModelName(device.modelDisplay);
}

void ImuVisualizationDialog::refreshData()
{
    if (!m_haveCurrentHandle) {
        clearChart(m_accChart);
        clearChart(m_gyroChart);
        clearChart(m_attitudeChart);
        if (m_orientationView) {
            m_orientationView->setHasData(false);
        }
        return;
    }

    const QVector<ImuVisualizationSample> samples = m_owner->imuVisualizationSamplesSnapshot(m_currentHandle);
    const bool hasSamples = !samples.isEmpty();
    updateChart(m_accChart, samples, &ImuVisualizationSample::ax, &ImuVisualizationSample::ay, &ImuVisualizationSample::az);
    updateChart(m_gyroChart, samples, &ImuVisualizationSample::gx, &ImuVisualizationSample::gy, &ImuVisualizationSample::gz);
    updateChart(m_attitudeChart, samples, &ImuVisualizationSample::rollDeg, &ImuVisualizationSample::pitchDeg, &ImuVisualizationSample::yawDeg);

    if (m_orientationView) {
        m_orientationView->setHasData(hasSamples);
        if (hasSamples) {
            m_orientationView->setOrientation(samples.last().orientation);
        }
    }
}

void ImuVisualizationDialog::resetZoom()
{
    for (ChartPanel* panel : {&m_accChart, &m_gyroChart, &m_attitudeChart}) {
        panel->axisX->setRange(0.0, kVisibleWindowSec);
        panel->axisY->setRange(panel->defaultMin, panel->defaultMax);
    }
}

void ImuVisualizationDialog::setPaused(bool paused)
{
    m_paused = paused;
    m_pauseButton->setText(paused ? QStringLiteral("继续") : QStringLiteral("暂停"));
}

void ImuVisualizationDialog::clearChart(ChartPanel& panel)
{
    panel.seriesX->clear();
    panel.seriesY->clear();
    panel.seriesZ->clear();
    panel.axisX->setRange(0.0, kVisibleWindowSec);
    panel.axisY->setRange(panel.defaultMin, panel.defaultMax);
    panel.emptyLabel->setVisible(true);
}

void ImuVisualizationDialog::updateChart(ChartPanel& panel,
                                         const QVector<ImuVisualizationSample>& samples,
                                         double ImuVisualizationSample::*xField,
                                         double ImuVisualizationSample::*yField,
                                         double ImuVisualizationSample::*zField)
{
    if (samples.isEmpty()) {
        clearChart(panel);
        return;
    }

    const double latestSec = samples.last().timestampSec;
    const double endSec = std::max(kVisibleWindowSec, latestSec);
    const double startSec = std::max(0.0, endSec - kVisibleWindowSec);
    panel.seriesX->replace(makePoints(samples, startSec, endSec, xField));
    panel.seriesY->replace(makePoints(samples, startSec, endSec, yField));
    panel.seriesZ->replace(makePoints(samples, startSec, endSec, zField));
    panel.axisX->setRange(startSec, endSec);

    if (m_autoScaleSwitch->isChecked()) {
        double minValue = std::numeric_limits<double>::max();
        double maxValue = std::numeric_limits<double>::lowest();
        for (const ImuVisualizationSample& sample : samples) {
            if (sample.timestampSec < startSec || sample.timestampSec > endSec) {
                continue;
            }
            for (double value : {sample.*xField, sample.*yField, sample.*zField}) {
                minValue = std::min(minValue, value);
                maxValue = std::max(maxValue, value);
            }
        }
        if (minValue <= maxValue) {
            const double span = std::max(0.001, maxValue - minValue);
            const double margin = std::max(span * 0.12, 0.05);
            panel.axisY->setRange(minValue - margin, maxValue + margin);
        }
    }

    panel.emptyLabel->setVisible(false);
}

void ImuVisualizationDialog::refreshTheme()
{
    if (m_refreshingTheme) {
        return;
    }
    m_refreshingTheme = true;

    const QPalette pal = palette();
    const bool dark = isDarkPalette(pal);
    const QColor window = pal.color(QPalette::Window);
    const QColor base = pal.color(QPalette::Base);
    const QColor text = pal.color(QPalette::WindowText);
    const QColor hint = pal.color(QPalette::Mid);
    const QColor border = dark ? QColor(82, 86, 92) : QColor(210, 214, 220);
    const QColor selectedBg = dark ? QColor(47, 51, 57) : QColor(238, 241, 245);
    const QColor hoverBorder = dark ? QColor(142, 148, 156) : QColor(118, 126, 136);

    const QString styleSheet = QStringLiteral(
        "QDialog#ImuVisualizationDialog { background: %1; color: %2; }"
        "QFrame#ImuToolbar, QFrame#ImuVisualizationPanel { background: %3; border: 1px solid %4; border-radius: 6px; }"
        "QLabel#ImuPanelTitle, QLabel#ImuDeviceModel { color: %2; font-weight: 600; }"
        "QLabel#ImuPanelHint { color: %5; }"
        "QLabel#ImuDeviceSourceTag { color: %2; }"
        "QListWidget#ImuDeviceList { background: transparent; border: none; outline: 0; }"
        "QListWidget#ImuDeviceList::item { border: none; padding: 0; background: transparent; }"
        "QWidget#ImuDeviceCard { background: %3; border: 1px solid %4; border-radius: 6px; }"
        "QWidget#ImuDeviceCard:hover { border-color: %6; }"
        "QWidget#ImuDeviceCard[selected=\"true\"] { background: %7; border-color: %6; }")
        .arg(colorName(window),
             colorName(text),
             colorName(base),
             colorName(border),
             colorName(hint),
             colorName(hoverBorder),
             colorName(selectedBg));
    if (this->styleSheet() != styleSheet) {
        setStyleSheet(styleSheet);
    }

    refreshChartTheme(m_accChart);
    refreshChartTheme(m_gyroChart);
    refreshChartTheme(m_attitudeChart);
    if (m_orientationView) {
        m_orientationView->refreshTheme();
    }

    m_refreshingTheme = false;
}

void ImuVisualizationDialog::refreshChartTheme(ChartPanel& panel)
{
    if (!panel.chart) {
        return;
    }

    const QPalette pal = palette();
    const bool dark = isDarkPalette(pal);
    const QColor base = pal.color(QPalette::Base);
    const QColor text = pal.color(QPalette::WindowText);
    const QColor axis = dark ? QColor(160, 166, 174) : QColor(92, 96, 104);
    const QColor grid = dark ? QColor(58, 62, 68) : QColor(224, 226, 230);

    panel.chart->setBackgroundBrush(base);
    panel.chart->setPlotAreaBackgroundBrush(base);
    panel.chart->setPlotAreaBackgroundVisible(true);
    panel.chart->legend()->setLabelColor(text);

    QPen axisPen(axis);
    QPen gridPen(grid);
    gridPen.setWidthF(0.8);
    for (QValueAxis* valueAxis : {panel.axisX, panel.axisY}) {
        valueAxis->setLabelsColor(text);
        valueAxis->setTitleBrush(text);
        valueAxis->setLinePen(axisPen);
        valueAxis->setGridLinePen(gridPen);
        valueAxis->setMinorGridLinePen(gridPen);
    }
    panel.view->setBackgroundBrush(base);
}

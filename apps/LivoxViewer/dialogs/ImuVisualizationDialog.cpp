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
#include <QIcon>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMouseEvent>
#include <QPalette>
#include <QPainter>
#include <QPaintEvent>
#include <QPen>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStyle>
#include <QTimer>
#include <QVariant>
#include <QVBoxLayout>
#include <QWidget>

#include <QtCharts/QLegend>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {

constexpr double kVisibleWindowSec = 5.0;
constexpr int kRefreshIntervalMs = 100;
constexpr int kDeviceRefreshIntervalMs = 500;

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

class ChartHoverOverlay : public QWidget
{
public:
    explicit ChartHoverOverlay(QWidget* parent)
        : QWidget(parent)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_TranslucentBackground);
        hide();
    }

    void setHover(const QRect& plotRect,
                  int hoverX,
                  const std::array<QPointF, 3>& points,
                  const QColor& lineColor,
                  const QColor& pointBorderColor)
    {
        m_plotRect = plotRect;
        m_hoverX = hoverX;
        m_points = points;
        m_lineColor = lineColor;
        m_pointBorderColor = pointBorderColor;
        show();
        update();
    }

    void clearHover()
    {
        hide();
    }

protected:
    void paintEvent(QPaintEvent* event) override
    {
        QWidget::paintEvent(event);
        if (!m_plotRect.isValid()) {
            return;
        }

        QPainter painter(this);
        painter.setClipRect(m_plotRect);
        painter.setRenderHint(QPainter::Antialiasing);

        QPen linePen(m_lineColor);
        linePen.setWidthF(1.0);
        linePen.setStyle(Qt::DashLine);
        painter.setPen(linePen);
        painter.drawLine(QPointF(m_hoverX, m_plotRect.top()), QPointF(m_hoverX, m_plotRect.bottom()));

        constexpr double kRadius = 4.5;
        const std::array<QColor, 3> pointColors = {kAxisXColor, kAxisYColor, kAxisZColor};
        painter.setPen(QPen(m_pointBorderColor, 1.4));
        for (int i = 0; i < static_cast<int>(m_points.size()); ++i) {
            painter.setBrush(pointColors[static_cast<size_t>(i)]);
            painter.drawEllipse(m_points[static_cast<size_t>(i)], kRadius, kRadius);
        }
    }

private:
    QRect m_plotRect;
    int m_hoverX = 0;
    std::array<QPointF, 3> m_points;
    QColor m_lineColor;
    QColor m_pointBorderColor;
};

void configureSeries(QLineSeries* series, const QString& name, const QColor& color)
{
    series->setName(name);
    QPen pen(color);
    pen.setWidthF(1.8);
    series->setPen(pen);
    series->setUseOpenGL(false);
}

QString unitFromAxisTitle(const QString& title)
{
    const int left = title.indexOf(QLatin1Char('('));
    const int right = title.indexOf(QLatin1Char(')'), left + 1);
    if (left >= 0 && right > left) {
        return title.mid(left + 1, right - left - 1);
    }
    return QString();
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

QRect plotAreaInView(QChartView* view, QChart* chart)
{
    const QRectF plotArea = chart->plotArea();
    const QPoint topLeft = view->mapFromScene(chart->mapToScene(plotArea.topLeft()));
    const QPoint bottomRight = view->mapFromScene(chart->mapToScene(plotArea.bottomRight()));
    return QRect(topLeft, bottomRight).normalized();
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
        refreshData(true);
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
        m_deviceRefreshElapsedMs += kRefreshIntervalMs;
        if (m_deviceRefreshElapsedMs >= kDeviceRefreshIntervalMs) {
            m_deviceRefreshElapsedMs = 0;
            refreshDeviceList();
        }
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

bool ImuVisualizationDialog::eventFilter(QObject* watched, QEvent* event)
{
    if (ChartPanel* panel = chartPanelForViewport(watched)) {
        if (event->type() == QEvent::MouseMove) {
            const QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
            panel->hoverActive = true;
            panel->hoverMousePos = mouseEvent->pos();
            updateChartHover(*panel, panel->hoverMousePos);
        } else if (event->type() == QEvent::Resize) {
            if (panel->hoverOverlay) {
                panel->hoverOverlay->setGeometry(panel->view->viewport()->rect());
            }
            positionChartEmptyOverlay(*panel);
            hideChartHover(*panel);
        } else if (event->type() == QEvent::Leave) {
            hideChartHover(*panel);
        }
        return false;
    }
    return QDialog::eventFilter(watched, event);
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
    connect(m_autoScaleSwitch, &QCheckBox::toggled, this, [this]() { refreshData(true); });
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
    titleLayout->addWidget(titleLabel);
    titleLayout->addStretch();
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
    panel.view->setMouseTracking(true);
    panel.view->viewport()->setMouseTracking(true);
    panel.view->viewport()->installEventFilter(this);
    layout->addWidget(panel.view, 1);

    panel.hoverUnit = unitFromAxisTitle(yTitle);
    panel.hoverOverlay = new ChartHoverOverlay(panel.view->viewport());
    panel.hoverOverlay->setGeometry(panel.view->viewport()->rect());

    panel.hoverLabel = new QLabel(panel.view->viewport());
    panel.hoverLabel->setObjectName(QStringLiteral("ImuChartHoverLabel"));
    panel.hoverLabel->setTextFormat(Qt::RichText);
    panel.hoverLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    panel.hoverLabel->hide();

    panel.emptyOverlay = new QWidget(panel.view->viewport());
    panel.emptyOverlay->setObjectName(QStringLiteral("ImuChartEmptyOverlay"));
    panel.emptyOverlay->setAttribute(Qt::WA_TransparentForMouseEvents);
    QVBoxLayout* emptyLayout = new QVBoxLayout(panel.emptyOverlay);
    emptyLayout->setContentsMargins(0, 0, 0, 0);
    emptyLayout->setSpacing(6);
    panel.emptyIcon = new QLabel(panel.emptyOverlay);
    panel.emptyIcon->setAlignment(Qt::AlignCenter);
    panel.emptyIcon->setPixmap(QIcon(QStringLiteral(":/icons/status_pending.svg")).pixmap(QSize(32, 32)));
    panel.emptyLabel = new QLabel(QStringLiteral("等待 IMU 数据"), panel.emptyOverlay);
    panel.emptyLabel->setObjectName(QStringLiteral("ImuPanelHint"));
    panel.emptyLabel->setAlignment(Qt::AlignCenter);
    emptyLayout->addWidget(panel.emptyIcon, 0, Qt::AlignHCenter);
    emptyLayout->addWidget(panel.emptyLabel, 0, Qt::AlignHCenter);
    panel.emptyOverlay->adjustSize();
    panel.emptyOverlay->hide();

    panel.panel = frame;
    QTimer::singleShot(0, this, [this, chart = panel.chart]() {
        for (ChartPanel* candidate : {&m_accChart, &m_gyroChart, &m_attitudeChart}) {
            if (candidate->chart == chart) {
                positionChartEmptyOverlay(*candidate);
                break;
            }
        }
    });
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
    const bool hadCurrentHandle = m_haveCurrentHandle;
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
        if (hadCurrentHandle && m_orientationView) {
            m_orientationView->clearScene();
            refreshData(true);
        }
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
        updateDeviceCardSelection();
    }
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

ImuVisualizationDialog::ChartPanel* ImuVisualizationDialog::chartPanelForViewport(QObject* watched)
{
    for (ChartPanel* panel : {&m_accChart, &m_gyroChart, &m_attitudeChart}) {
        if (panel->view && panel->view->viewport() == watched) {
            return panel;
        }
    }
    return nullptr;
}

void ImuVisualizationDialog::hideChartHover(ChartPanel& panel)
{
    panel.hoverActive = false;
    if (panel.hoverOverlay) {
        static_cast<ChartHoverOverlay*>(panel.hoverOverlay)->clearHover();
    }
    if (panel.hoverLabel) {
        panel.hoverLabel->hide();
    }
}

void ImuVisualizationDialog::positionChartEmptyOverlay(ChartPanel& panel)
{
    if (!panel.emptyOverlay || !panel.view || !panel.chart) {
        return;
    }

    panel.emptyOverlay->adjustSize();
    QRect targetRect = plotAreaInView(panel.view, panel.chart);
    if (targetRect.width() <= 0 || targetRect.height() <= 0) {
        targetRect = panel.view->viewport()->rect();
    }
    const int x = targetRect.left() + (targetRect.width() - panel.emptyOverlay->width()) / 2;
    const int y = targetRect.top() + (targetRect.height() - panel.emptyOverlay->height()) / 2;
    panel.emptyOverlay->move(std::max(targetRect.left(), x), std::max(targetRect.top(), y));
}

void ImuVisualizationDialog::updateChartHover(ChartPanel& panel, const QPoint& mousePos)
{
    if (panel.hoverSamples.isEmpty() ||
        !panel.view ||
        !panel.chart ||
        !panel.axisX ||
        !panel.axisY ||
        !panel.hoverXField ||
        !panel.hoverYField ||
        !panel.hoverZField) {
        hideChartHover(panel);
        return;
    }

    const QRect plotRect = plotAreaInView(panel.view, panel.chart);
    if (!plotRect.contains(mousePos)) {
        hideChartHover(panel);
        return;
    }

    const QPointF chartPos = panel.chart->mapFromScene(panel.view->mapToScene(mousePos));
    const double hoverTime = panel.chart->mapToValue(chartPos, panel.seriesX).x();
    const double axisMin = panel.axisX->min();
    const double axisMax = panel.axisX->max();
    const auto visibleBegin = std::lower_bound(
        panel.hoverSamples.constBegin(),
        panel.hoverSamples.constEnd(),
        axisMin,
        [](const ImuVisualizationSample& sample, double timestampSec) {
            return sample.timestampSec < timestampSec;
        });
    const auto visibleEnd = std::upper_bound(
        visibleBegin,
        panel.hoverSamples.constEnd(),
        axisMax,
        [](double timestampSec, const ImuVisualizationSample& sample) {
            return timestampSec < sample.timestampSec;
        });
    if (visibleBegin == visibleEnd) {
        hideChartHover(panel);
        return;
    }

    auto it = std::lower_bound(
        visibleBegin,
        visibleEnd,
        hoverTime,
        [](const ImuVisualizationSample& sample, double timestampSec) {
            return sample.timestampSec < timestampSec;
        });
    if (it == visibleEnd) {
        --it;
    } else if (it != visibleBegin) {
        auto prev = it;
        --prev;
        if (std::abs(prev->timestampSec - hoverTime) <= std::abs(it->timestampSec - hoverTime)) {
            it = prev;
        }
    }

    const ImuVisualizationSample& sample = *it;
    const double timeSec = sample.timestampSec;
    const double xValue = sample.*panel.hoverXField;
    const double yValue = sample.*panel.hoverYField;
    const double zValue = sample.*panel.hoverZField;

    const QPointF xPosition = panel.chart->mapToPosition(QPointF(timeSec, xValue), panel.seriesX);
    auto chartPointToView = [&panel](const QPointF& chartPoint) {
        return QPointF(panel.view->mapFromScene(panel.chart->mapToScene(chartPoint)));
    };
    const std::array<QPointF, 3> hoverPoints = {
        chartPointToView(panel.chart->mapToPosition(QPointF(timeSec, xValue), panel.seriesX)),
        chartPointToView(panel.chart->mapToPosition(QPointF(timeSec, yValue), panel.seriesY)),
        chartPointToView(panel.chart->mapToPosition(QPointF(timeSec, zValue), panel.seriesZ))
    };
    const QPalette pal = palette();
    const QColor hoverLineColor = isDarkPalette(pal) ? QColor(160, 166, 174) : QColor(92, 96, 104);
    if (panel.hoverOverlay) {
        panel.hoverOverlay->setGeometry(panel.view->viewport()->rect());
        static_cast<ChartHoverOverlay*>(panel.hoverOverlay)
            ->setHover(plotRect, qRound(chartPointToView(xPosition).x()), hoverPoints, hoverLineColor, pal.color(QPalette::Base));
        panel.hoverOverlay->raise();
    }

    const QString unitSuffix = panel.hoverUnit.isEmpty() ? QString() : QStringLiteral(" %1").arg(panel.hoverUnit);
    auto valueLine = [&unitSuffix](const QColor& color, const QString& name, double value) {
        return QStringLiteral("<span style=\"color:%1;\">●</span> %2：%3%4")
            .arg(colorName(color),
                 name.toHtmlEscaped(),
                 QString::number(value, 'f', 3),
                 unitSuffix.toHtmlEscaped());
    };
    panel.hoverLabel->setText(QStringLiteral("时间：%1 s<br/>%2<br/>%3<br/>%4")
                                  .arg(QString::number(timeSec, 'f', 3),
                                       valueLine(kAxisXColor, panel.seriesX->name(), xValue),
                                       valueLine(kAxisYColor, panel.seriesY->name(), yValue),
                                       valueLine(kAxisZColor, panel.seriesZ->name(), zValue)));
    panel.hoverLabel->adjustSize();

    constexpr int kOffset = 14;
    int labelX = mousePos.x() + kOffset;
    if (labelX + panel.hoverLabel->width() > plotRect.right()) {
        labelX = mousePos.x() - panel.hoverLabel->width() - kOffset;
    }
    int labelY = mousePos.y() - panel.hoverLabel->height() - kOffset;
    if (labelY < plotRect.top()) {
        labelY = mousePos.y() + kOffset;
    }
    labelX = std::clamp(labelX, plotRect.left(), std::max(plotRect.left(), plotRect.right() - panel.hoverLabel->width()));
    labelY = std::clamp(labelY, plotRect.top(), std::max(plotRect.top(), plotRect.bottom() - panel.hoverLabel->height()));
    panel.hoverLabel->move(labelX, labelY);
    panel.hoverLabel->show();
    panel.hoverLabel->raise();
}

void ImuVisualizationDialog::refreshData(bool force)
{
    if (!m_haveCurrentHandle) {
        if (force || !m_chartsCleared || m_haveLastSampleSnapshot) {
            clearChart(m_accChart);
            clearChart(m_gyroChart);
            clearChart(m_attitudeChart);
            if (m_orientationView) {
                m_orientationView->setHasData(false);
            }
            m_chartsCleared = true;
            m_haveLastSampleSnapshot = false;
            m_lastSampleHandle = 0;
            m_lastSampleRevision = 0;
        }
        return;
    }

    const ImuVisualizationSamplesSnapshot snapshot = m_owner->imuVisualizationSamplesSnapshot(m_currentHandle, kVisibleWindowSec);
    if (!force &&
        m_haveLastSampleSnapshot &&
        m_lastSampleHandle == m_currentHandle &&
        m_lastSampleRevision == snapshot.revision) {
        return;
    }

    m_haveLastSampleSnapshot = true;
    m_lastSampleHandle = m_currentHandle;
    m_lastSampleRevision = snapshot.revision;

    const QVector<ImuVisualizationSample>& samples = snapshot.samples;
    const bool hasSamples = !samples.isEmpty();
    if (!hasSamples) {
        clearChart(m_accChart);
        clearChart(m_gyroChart);
        clearChart(m_attitudeChart);
        if (m_orientationView) {
            m_orientationView->setHasData(false);
        }
        m_chartsCleared = true;
        return;
    }

    updateChart(m_accChart, samples, &ImuVisualizationSample::ax, &ImuVisualizationSample::ay, &ImuVisualizationSample::az);
    updateChart(m_gyroChart, samples, &ImuVisualizationSample::gx, &ImuVisualizationSample::gy, &ImuVisualizationSample::gz);
    updateChart(m_attitudeChart, samples, &ImuVisualizationSample::rollDeg, &ImuVisualizationSample::pitchDeg, &ImuVisualizationSample::yawDeg);
    m_chartsCleared = false;

    if (m_orientationView) {
        m_orientationView->setHasData(true);
        m_orientationView->setOrientation(samples.last().orientation);
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
    panel.hoverSamples.clear();
    hideChartHover(panel);
    panel.seriesX->clear();
    panel.seriesY->clear();
    panel.seriesZ->clear();
    panel.axisX->setRange(0.0, kVisibleWindowSec);
    panel.axisY->setRange(panel.defaultMin, panel.defaultMax);
    if (panel.emptyOverlay) {
        positionChartEmptyOverlay(panel);
        panel.emptyOverlay->show();
        panel.emptyOverlay->raise();
    }
}

void ImuVisualizationDialog::updateChart(ChartPanel& panel,
                                         const QVector<ImuVisualizationSample>& samples,
                                         double ImuVisualizationSample::*xField,
                                         double ImuVisualizationSample::*yField,
                                         double ImuVisualizationSample::*zField)
{
    panel.hoverSamples = samples;
    panel.hoverXField = xField;
    panel.hoverYField = yField;
    panel.hoverZField = zField;
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

    if (panel.emptyOverlay) {
        panel.emptyOverlay->hide();
    }
    if (panel.hoverActive) {
        updateChartHover(panel, panel.hoverMousePos);
    }
}

void ImuVisualizationDialog::refreshTheme()
{
    const QPalette pal = QApplication::palette();
    setPalette(pal);
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
        "QWidget#ImuChartEmptyOverlay { background: transparent; }"
        "QLabel#ImuPanelTitle, QLabel#ImuDeviceModel { color: %2; font-weight: 600; }"
        "QLabel#ImuPanelHint { color: %5; }"
        "QLabel#ImuDeviceSourceTag { color: %2; }"
        "QLabel#ImuChartHoverLabel { color: %2; background: %3; border: 1px solid %4; border-radius: 4px; padding: 6px 8px; }"
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
    updateDeviceCardSelection();

    refreshChartTheme(m_accChart);
    refreshChartTheme(m_gyroChart);
    refreshChartTheme(m_attitudeChart);
    if (m_orientationView) {
        m_orientationView->refreshTheme();
    }
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

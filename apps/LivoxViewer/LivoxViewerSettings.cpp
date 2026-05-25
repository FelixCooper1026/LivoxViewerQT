#include "LivoxViewerWindow.h"

#include <QColorDialog>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QPushButton>
#include <QRadioButton>
#include <QSettings>
#include <QTabWidget>
#include <QVBoxLayout>

LivoxViewerWindow::LivoxViewerWindow(QWidget *parent)
    : QMainWindow(parent)
    , sdk_initialized(false)
    , sdk_started(false)
    , currentLidarDevice(nullptr)
    , statusLabel(nullptr)
    , pointCloudCallbackEnabled(false)
    , isNormalMode(true)
    , lidarDiscoverySocket(nullptr)
    , lidarDiscoveryTimer(nullptr)
    , lidarDiscoveryActive(false)
{
    initializeUserInterface();
    loadViewPreferences();

    // 网络自动配置开关（Linux 默认关闭）
    {
        QSettings settings("Livox", "LivoxViewerQT");
#ifdef Q_OS_LINUX
        const bool defaultAutoConfig = false;
#else
        const bool defaultAutoConfig = true;
#endif
        autoConfigHostIpEnabled = settings.value("network/autoConfigHostIp", defaultAutoConfig).toBool();
        if (autoConfigHostIpCheck) {
            autoConfigHostIpCheck->setChecked(autoConfigHostIpEnabled);
        }
    }

    // 初始化网络接口列表
    refreshNetworkInterfaces();

    // 启动设备发现，SDK初始化将在设备发现完成后进行
    startLidarDiscovery();

    // 移除状态栏自动更新逻辑

    // 设置参数查询定时器
    paramQueryTimer = new QTimer(this);
    connect(paramQueryTimer, &QTimer::timeout, this, &LivoxViewerWindow::onParamQueryTimeout);
    paramQueryTimer->start(1000); // 每1秒查询一次参数

    // 恢复窗口布局与几何
    QSettings settings("Livox", "LivoxViewerQT");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

LivoxViewerWindow::~LivoxViewerWindow()
{
    // 保存窗口布局与几何
    QSettings settings("Livox", "LivoxViewerQT");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    saveViewPreferences();

    stopLidarDiscovery();
    shutdownLivoxSdk();
}

void LivoxViewerWindow::loadViewPreferences()
{
    if (!pointCloudView) {
        return;
    }

    QSettings settings("Livox", "LivoxViewerQT");
    distanceLegendMin = settings.value("legend/distanceMin", distanceLegendMin).toFloat();
    distanceLegendMax = settings.value("legend/distanceMax", distanceLegendMax).toFloat();
    elevationLegendMin = settings.value("legend/elevationMin", elevationLegendMin).toFloat();
    elevationLegendMax = settings.value("legend/elevationMax", elevationLegendMax).toFloat();
    if (distanceLegendMax <= distanceLegendMin) {
        distanceLegendMax = distanceLegendMin + 1.0f;
    }
    if (elevationLegendMax <= elevationLegendMin) {
        elevationLegendMax = elevationLegendMin + 1.0f;
    }
    PointCloudView::GridConfig config = pointCloudView->gridConfig();
    config.range = settings.value("grid/range", config.range).toFloat();
    config.step = settings.value("grid/step", config.step).toFloat();
    config.color = settings.value("grid/color", config.color).value<QColor>();
    config.type = static_cast<PointCloudView::GridConfig::Type>(
        settings.value("grid/type", int(config.type)).toInt());

    if (config.type != PointCloudView::GridConfig::Square &&
        config.type != PointCloudView::GridConfig::ConcentricCircles) {
        config.type = PointCloudView::GridConfig::Square;
    }

    pointCloudView->setGridConfig(config);
    updatePointCloudLegend();
}

void LivoxViewerWindow::saveViewPreferences()
{
    if (!pointCloudView) {
        return;
    }

    const PointCloudView::GridConfig config = pointCloudView->gridConfig();
    QSettings settings("Livox", "LivoxViewerQT");
    settings.setValue("grid/range", config.range);
    settings.setValue("grid/step", config.step);
    settings.setValue("grid/color", config.color);
    settings.setValue("grid/type", int(config.type));
    settings.setValue("legend/distanceMin", distanceLegendMin);
    settings.setValue("legend/distanceMax", distanceLegendMax);
    settings.setValue("legend/elevationMin", elevationLegendMin);
    settings.setValue("legend/elevationMax", elevationLegendMax);
}

void LivoxViewerWindow::showPreferencesDialog()
{
    if (!pointCloudView) {
        return;
    }

    QDialog dlg(this);
    dlg.setWindowTitle("首选项");
    dlg.resize(520, 360);

    PointCloudView::GridConfig config = pointCloudView->gridConfig();
    QColor selectedColor = config.color;

    QVBoxLayout* layout = new QVBoxLayout(&dlg);
    QTabWidget* tabs = new QTabWidget(&dlg);
    QWidget* gridTab = new QWidget(tabs);
    QVBoxLayout* gridLayout = new QVBoxLayout(gridTab);
    QFormLayout* form = new QFormLayout();
    QWidget* legendTab = new QWidget(tabs);
    QFormLayout* legendForm = new QFormLayout(legendTab);

    QDoubleSpinBox* rangeSpin = new QDoubleSpinBox(&dlg);
    rangeSpin->setRange(1.0, 10000.0);
    rangeSpin->setDecimals(1);
    rangeSpin->setSingleStep(10.0);
    rangeSpin->setSuffix(" m");
    rangeSpin->setValue(config.range);

    QDoubleSpinBox* stepSpin = new QDoubleSpinBox(&dlg);
    stepSpin->setRange(0.1, 1000.0);
    stepSpin->setDecimals(1);
    stepSpin->setSingleStep(0.5);
    stepSpin->setSuffix(" m");
    stepSpin->setValue(config.step);

    QWidget* colorRow = new QWidget(&dlg);
    QHBoxLayout* colorLayout = new QHBoxLayout(colorRow);
    colorLayout->setContentsMargins(0, 0, 0, 0);
    colorLayout->setSpacing(8);
    QFrame* colorPreview = new QFrame(colorRow);
    colorPreview->setFixedSize(28, 20);
    colorPreview->setFrameShape(QFrame::Box);
    colorPreview->setLineWidth(1);
    colorPreview->setStyleSheet(QString("background-color: %1;").arg(selectedColor.name()));
    QPushButton* colorButton = new QPushButton("选择颜色", colorRow);
    colorLayout->addWidget(colorPreview);
    colorLayout->addWidget(colorButton);
    colorLayout->addStretch();
    connect(colorButton, &QPushButton::clicked, &dlg, [&dlg, &selectedColor, colorPreview]() {
        QColor color = QColorDialog::getColor(selectedColor, &dlg, "选择网格颜色");
        if (!color.isValid()) {
            return;
        }
        selectedColor = color;
        colorPreview->setStyleSheet(QString("background-color: %1;").arg(selectedColor.name()));
    });

    QWidget* typeRow = new QWidget(&dlg);
    QHBoxLayout* typeLayout = new QHBoxLayout(typeRow);
    typeLayout->setContentsMargins(0, 0, 0, 0);
    typeLayout->setSpacing(12);
    QRadioButton* squareRadio = new QRadioButton("方形", typeRow);
    QRadioButton* circleRadio = new QRadioButton("同心圆", typeRow);
    if (config.type == PointCloudView::GridConfig::ConcentricCircles) {
        circleRadio->setChecked(true);
    } else {
        squareRadio->setChecked(true);
    }
    typeLayout->addWidget(squareRadio);
    typeLayout->addWidget(circleRadio);
    typeLayout->addStretch();

    auto createLegendSpin = [&dlg](double minValue, double maxValue, double value) {
        QDoubleSpinBox* spin = new QDoubleSpinBox(&dlg);
        spin->setRange(minValue, maxValue);
        spin->setDecimals(2);
        spin->setSingleStep(0.5);
        spin->setSuffix(" m");
        spin->setValue(value);
        return spin;
    };
    QDoubleSpinBox* distanceMinSpin = createLegendSpin(0.0, 100000.0, distanceLegendMin);
    QDoubleSpinBox* distanceMaxSpin = createLegendSpin(distanceLegendMin + 0.01, 100000.0, distanceLegendMax);
    QDoubleSpinBox* elevationMinSpin = createLegendSpin(-100000.0, elevationLegendMax - 0.01, elevationLegendMin);
    QDoubleSpinBox* elevationMaxSpin = createLegendSpin(elevationLegendMin + 0.01, 100000.0, elevationLegendMax);

    connect(distanceMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [distanceMaxSpin](double value) { distanceMaxSpin->setMinimum(value + 0.01); });
    connect(distanceMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [distanceMinSpin](double value) { distanceMinSpin->setMaximum(value - 0.01); });
    connect(elevationMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [elevationMaxSpin](double value) { elevationMaxSpin->setMinimum(value + 0.01); });
    connect(elevationMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [elevationMinSpin](double value) { elevationMinSpin->setMaximum(value - 0.01); });

    form->addRow("网格范围:", rangeSpin);
    form->addRow("网格间距:", stepSpin);
    form->addRow("网格颜色:", colorRow);
    form->addRow("网格类型:", typeRow);
    legendForm->addRow("距离低值:", distanceMinSpin);
    legendForm->addRow("距离高值:", distanceMaxSpin);
    legendForm->addRow("高度低值:", elevationMinSpin);
    legendForm->addRow("高度高值:", elevationMaxSpin);

    QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    gridLayout->addLayout(form);
    gridLayout->addStretch();
    tabs->addTab(gridTab, "网格");
    tabs->addTab(legendTab, "图例");
    layout->addWidget(tabs);
    layout->addWidget(box);

    if (dlg.exec() != QDialog::Accepted) {
        return;
    }

    config.range = float(rangeSpin->value());
    config.step = float(stepSpin->value());
    config.color = selectedColor;
    config.type = circleRadio->isChecked()
        ? PointCloudView::GridConfig::ConcentricCircles
        : PointCloudView::GridConfig::Square;
    distanceLegendMin = float(distanceMinSpin->value());
    distanceLegendMax = float(distanceMaxSpin->value());
    elevationLegendMin = float(elevationMinSpin->value());
    elevationLegendMax = float(elevationMaxSpin->value());

    pointCloudView->setGridConfig(config);
    updatePointCloudLegend();
    saveViewPreferences();
}

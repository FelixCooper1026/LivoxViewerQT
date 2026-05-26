#include "LivoxViewerWindow.h"

#include <QApplication>
#include <QColorDialog>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QPalette>
#include <QPushButton>
#include <QRadioButton>
#include <QSettings>
#include <QStyle>
#include <QStyleFactory>
#include <QStyleHints>
#include <QTabWidget>
#include <QtGlobal>
#include <QVBoxLayout>
#include <QWidget>

namespace {

void refreshWidgetStyle(QWidget* widget)
{
    if (!widget) {
        return;
    }

    if (!widget->styleSheet().isEmpty()) {
        const QString styleSheet = widget->styleSheet();
        widget->setStyleSheet(QString());
        widget->setStyleSheet(styleSheet);
    }

    if (QStyle* style = widget->style()) {
        style->unpolish(widget);
        style->polish(widget);
    }
    widget->update();

    const QObjectList children = widget->children();
    for (QObject* child : children) {
        if (QWidget* childWidget = qobject_cast<QWidget*>(child)) {
            refreshWidgetStyle(childWidget);
        }
    }
}

void refreshApplicationStyles()
{
    const QWidgetList topLevelWidgets = QApplication::topLevelWidgets();
    for (QWidget* widget : topLevelWidgets) {
        refreshWidgetStyle(widget);
    }
}

} // namespace

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

#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
    connect(QGuiApplication::styleHints(), &QStyleHints::colorSchemeChanged, this, [this](Qt::ColorScheme) {
        if (themeMode == ThemeFollowSystem) {
            applyUiTheme();
        }
    });
#endif

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

bool LivoxViewerWindow::shouldUseDarkTheme() const
{
    if (themeMode == ThemeDark) {
        return true;
    }
    if (themeMode == ThemeLight) {
        return false;
    }
#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
    return QGuiApplication::styleHints()->colorScheme() == Qt::ColorScheme::Dark;
#else
    return QApplication::style()->standardPalette().color(QPalette::Window).lightness() < 128;
#endif
}

void LivoxViewerWindow::applyUiTheme()
{
    QApplication* app = qobject_cast<QApplication*>(QCoreApplication::instance());
    if (!app) {
        return;
    }

    app->setStyle(QStyleFactory::create("Fusion"));
    QPalette palette;

    if (shouldUseDarkTheme()) {
        palette.setColor(QPalette::Window, QColor(45, 45, 48));
        palette.setColor(QPalette::WindowText, QColor(240, 240, 240));
        palette.setColor(QPalette::Base, QColor(30, 30, 30));
        palette.setColor(QPalette::AlternateBase, QColor(50, 50, 52));
        palette.setColor(QPalette::ToolTipBase, QColor(45, 45, 48));
        palette.setColor(QPalette::ToolTipText, QColor(240, 240, 240));
        palette.setColor(QPalette::Text, QColor(240, 240, 240));
        palette.setColor(QPalette::Button, QColor(53, 53, 56));
        palette.setColor(QPalette::ButtonText, QColor(240, 240, 240));
        palette.setColor(QPalette::BrightText, Qt::red);
        palette.setColor(QPalette::Highlight, QColor(42, 130, 218));
        palette.setColor(QPalette::HighlightedText, Qt::white);
        palette.setColor(QPalette::Link, QColor(90, 160, 230));
        palette.setColor(QPalette::Mid, QColor(85, 85, 88));
    } else {
        palette.setColor(QPalette::Window, QColor(255, 255, 255));
        palette.setColor(QPalette::WindowText, QColor(20, 20, 20));
        palette.setColor(QPalette::Base, QColor(255, 255, 255));
        palette.setColor(QPalette::AlternateBase, QColor(245, 245, 245));
        palette.setColor(QPalette::ToolTipBase, QColor(255, 255, 255));
        palette.setColor(QPalette::ToolTipText, QColor(20, 20, 20));
        palette.setColor(QPalette::Text, QColor(20, 20, 20));
        palette.setColor(QPalette::Button, QColor(245, 245, 245));
        palette.setColor(QPalette::ButtonText, QColor(20, 20, 20));
        palette.setColor(QPalette::BrightText, Qt::red);
        palette.setColor(QPalette::Highlight, QColor(0, 120, 215));
        palette.setColor(QPalette::HighlightedText, Qt::white);
        palette.setColor(QPalette::Link, QColor(0, 102, 204));
        palette.setColor(QPalette::Mid, QColor(190, 190, 190));
    }

    app->setPalette(palette);
    refreshApplicationStyles();
}

void LivoxViewerWindow::loadViewPreferences()
{
    if (!pointCloudView) {
        return;
    }

    QSettings settings("Livox", "LivoxViewerQT");
    themeMode = settings.value("theme/mode", themeMode).toInt();
    if (themeMode < ThemeFollowSystem || themeMode > ThemeDark) {
        themeMode = ThemeFollowSystem;
    }
    applyUiTheme();

    distanceLegendMin = settings.value("legend/distanceMin", distanceLegendMin).toFloat();
    distanceLegendMax = settings.value("legend/distanceMax", distanceLegendMax).toFloat();
    elevationLegendMin = settings.value("legend/elevationMin", elevationLegendMin).toFloat();
    elevationLegendMax = settings.value("legend/elevationMax", elevationLegendMax).toFloat();
    solidColor = settings.value("color/solidColor", solidColor).value<QColor>();
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
    settings.setValue("color/solidColor", solidColor);
    settings.setValue("theme/mode", themeMode);
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
    QWidget* colorTab = new QWidget(tabs);
    QFormLayout* colorForm = new QFormLayout(colorTab);
    QColor selectedSolidColor = solidColor;
    QWidget* themeTab = new QWidget(tabs);
    QFormLayout* themeForm = new QFormLayout(themeTab);
    QComboBox* themeCombo = new QComboBox(themeTab);
    themeCombo->addItems({"跟随系统", "浅色", "深色"});
    themeCombo->setCurrentIndex(std::clamp(themeMode, int(ThemeFollowSystem), int(ThemeDark)));

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

    QWidget* solidColorRow = new QWidget(&dlg);
    QHBoxLayout* solidColorLayout = new QHBoxLayout(solidColorRow);
    solidColorLayout->setContentsMargins(0, 0, 0, 0);
    solidColorLayout->setSpacing(8);
    QFrame* solidColorPreview = new QFrame(solidColorRow);
    solidColorPreview->setFixedSize(28, 20);
    solidColorPreview->setFrameShape(QFrame::Box);
    solidColorPreview->setLineWidth(1);
    solidColorPreview->setStyleSheet(QString("background-color: %1;").arg(selectedSolidColor.name()));
    QPushButton* solidColorButton = new QPushButton("选择颜色", solidColorRow);
    solidColorLayout->addWidget(solidColorPreview);
    solidColorLayout->addWidget(solidColorButton);
    solidColorLayout->addStretch();
    connect(solidColorButton, &QPushButton::clicked, &dlg, [&dlg, &selectedSolidColor, solidColorPreview]() {
        QColor color = QColorDialog::getColor(selectedSolidColor, &dlg, "选择纯色点云颜色");
        if (!color.isValid()) {
            return;
        }
        selectedSolidColor = color;
        solidColorPreview->setStyleSheet(QString("background-color: %1;").arg(selectedSolidColor.name()));
    });

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
    colorForm->addRow("纯色模式颜色:", solidColorRow);
    themeForm->addRow("应用主题:", themeCombo);

    QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    gridLayout->addLayout(form);
    gridLayout->addStretch();
    tabs->addTab(gridTab, "网格");
    tabs->addTab(legendTab, "图例");
    tabs->addTab(colorTab, "着色");
    tabs->addTab(themeTab, "主题");
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
    solidColor = selectedSolidColor;
    themeMode = themeCombo->currentIndex();

    pointCloudView->setGridConfig(config);
    applyUiTheme();
    updatePointCloudLegend();
    saveViewPreferences();
}

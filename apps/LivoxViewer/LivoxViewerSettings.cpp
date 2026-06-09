#include "LivoxViewerWindow.h"
#include "widgets/SwitchCheckBox.h"

#include <QAbstractButton>
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFile>
#include <QFont>
#include <QFrame>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QIcon>
#include <QPalette>
#include <QProcess>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QRadioButton>
#include <QSettings>
#include <QSize>
#include <QSizePolicy>
#include <QScrollArea>
#include <QStyle>
#include <QStyleFactory>
#include <QStyleHints>
#include <QStackedWidget>
#include <QStringList>
#include <QTextStream>
#include <QToolButton>
#include <QtGlobal>
#include <QVBoxLayout>
#include <QWidget>

namespace {

constexpr int kDockStateVersion = 2;
constexpr int kPreferenceControlColumnWidth = 115;
constexpr int kPreferenceSpinBoxWidth = 100;

void setPreferenceFont(QWidget* widget, int pointSize, QFont::Weight weight)
{
    QFont font = widget->font();
#ifdef Q_OS_WIN
    font.setFamily(QStringLiteral("Microsoft YaHei UI"));
#endif
    font.setPointSize(pointSize);
    font.setWeight(weight);
    font.setStyleStrategy(QFont::PreferAntialias);
    widget->setFont(font);
}

void usePreferenceControlColumn(QWidget* widget)
{
    widget->setProperty("preferenceControlColumnWidth", kPreferenceControlColumnWidth);
}

void preparePreferenceSpinBox(QDoubleSpinBox* spin)
{
    spin->setFixedWidth(kPreferenceSpinBoxWidth);
    usePreferenceControlColumn(spin);
}

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

bool defaultAutoConfigHostIp()
{
#ifdef Q_OS_LINUX
    return false;
#else
    return true;
#endif
}

#ifdef Q_OS_LINUX
QString linuxCommandOutput(const QString& program, const QStringList& arguments)
{
    QProcess process;
    process.start(program, arguments, QIODevice::ReadOnly);
    if (!process.waitForStarted(300) || !process.waitForFinished(500)) {
        return {};
    }
    if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) {
        return {};
    }
    return QString::fromLocal8Bit(process.readAllStandardOutput()).trimmed().toLower();
}

int linuxSystemDarkThemeState()
{
    const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    const QStringList envKeys = {
        "COLOR_SCHEME",
        "GTK_THEME",
        "XDG_CURRENT_DESKTOP",
        "DESKTOP_SESSION"
    };
    for (const QString& key : envKeys) {
        const QString value = env.value(key).toLower();
        if (value.contains("dark")) {
            return 1;
        }
        if (value.contains("light")) {
            return 0;
        }
    }

    const QString gnomeColorScheme = linuxCommandOutput(
        QStringLiteral("gsettings"),
        {QStringLiteral("get"), QStringLiteral("org.gnome.desktop.interface"), QStringLiteral("color-scheme")});
    if (gnomeColorScheme.contains(QStringLiteral("prefer-dark"))) {
        return 1;
    }
    if (gnomeColorScheme.contains(QStringLiteral("prefer-light"))) {
        return 0;
    }

    const QString gnomeGtkTheme = linuxCommandOutput(
        QStringLiteral("gsettings"),
        {QStringLiteral("get"), QStringLiteral("org.gnome.desktop.interface"), QStringLiteral("gtk-theme")});
    if (gnomeGtkTheme.contains(QStringLiteral("dark"))) {
        return 1;
    }
    if (gnomeGtkTheme.contains(QStringLiteral("light"))) {
        return 0;
    }

    const QStringList kdeConfigCommands = {QStringLiteral("kreadconfig6"), QStringLiteral("kreadconfig5")};
    for (const QString& command : kdeConfigCommands) {
        const QString kdeColorScheme = linuxCommandOutput(
            command,
            {QStringLiteral("--group"), QStringLiteral("General"), QStringLiteral("--key"), QStringLiteral("ColorScheme")});
        if (kdeColorScheme.contains(QStringLiteral("dark"))) {
            return 1;
        }
        if (kdeColorScheme.contains(QStringLiteral("light"))) {
            return 0;
        }
    }

    auto readTextFile = [](const QString& path) {
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            return QString();
        }
        QTextStream stream(&file);
        return stream.readAll().toLower();
    };

    const QString gtkSettings = readTextFile(QDir::homePath() + "/.config/gtk-3.0/settings.ini");
    if (gtkSettings.contains("gtk-application-prefer-dark-theme=true") ||
        (gtkSettings.contains("gtk-theme-name") && gtkSettings.contains("dark"))) {
        return 1;
    }
    if (gtkSettings.contains("gtk-application-prefer-dark-theme=false") ||
        (gtkSettings.contains("gtk-theme-name") && gtkSettings.contains("light"))) {
        return 0;
    }

    const QString kdeGlobals = readTextFile(QDir::homePath() + "/.config/kdeglobals");
    if (kdeGlobals.contains("colorscheme") && kdeGlobals.contains("dark")) {
        return 1;
    }
    if (kdeGlobals.contains("colorscheme") && kdeGlobals.contains("light")) {
        return 0;
    }

    return -1;
}
#endif

QLabel* createPreferenceDescription(const QString& text, QWidget* parent)
{
    QLabel* label = new QLabel(text, parent);
    label->setWordWrap(true);
    label->setStyleSheet("color: palette(mid);");
    return label;
}

QFrame* createPreferenceSection(QWidget* parent)
{
    QFrame* section = new QFrame(parent);
    section->setObjectName("PreferenceSection");
    section->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);

    QVBoxLayout* layout = new QVBoxLayout(section);
    layout->setContentsMargins(16, 10, 16, 10);
    layout->setSpacing(0);

    section->setStyleSheet(
        "#PreferenceSection {"
        "  background: palette(base);"
        "  border: 1px solid palette(mid);"
        "  border-radius: 6px;"
        "}"
    );
    return section;
}

void addPreferenceSectionTitle(QVBoxLayout* pageLayout, const QString& title)
{
    QLabel* label = new QLabel(title, pageLayout->parentWidget());
    setPreferenceFont(label, 10, QFont::DemiBold);
    pageLayout->addWidget(label);
}

void addPreferenceRow(QFrame* section, const QString& title, const QString& description, QWidget* control)
{
    QVBoxLayout* sectionLayout = qobject_cast<QVBoxLayout*>(section->layout());
    const int rowCount = section->property("rowCount").toInt();
    if (rowCount > 0) {
        QFrame* separator = new QFrame(section);
        separator->setFrameShape(QFrame::HLine);
        separator->setFrameShadow(QFrame::Plain);
        separator->setStyleSheet("color: palette(mid);");
        sectionLayout->addWidget(separator);
    }

    QWidget* row = new QWidget(section);
    QHBoxLayout* rowLayout = new QHBoxLayout(row);
    rowLayout->setContentsMargins(0, 10, 0, 10);
    rowLayout->setSpacing(18);

    QVBoxLayout* textLayout = new QVBoxLayout();
    textLayout->setContentsMargins(0, 0, 0, 0);
    textLayout->setSpacing(4);

    QLabel* titleLabel = new QLabel(title, row);
    setPreferenceFont(titleLabel, 9, QFont::DemiBold);
    textLayout->addWidget(titleLabel);
    if (!description.isEmpty()) {
        textLayout->addWidget(createPreferenceDescription(description, row));
    }

    rowLayout->addLayout(textLayout, 1);
    const int controlColumnWidth = control->property("preferenceControlColumnWidth").toInt();
    if (controlColumnWidth > 0) {
        QWidget* controlCell = new QWidget(row);
        controlCell->setFixedWidth(controlColumnWidth);
        QHBoxLayout* controlLayout = new QHBoxLayout(controlCell);
        controlLayout->setContentsMargins(0, 0, 0, 0);
        controlLayout->setSpacing(0);
        controlLayout->addWidget(control, 0, Qt::AlignLeft | Qt::AlignVCenter);
        controlLayout->addStretch();
        rowLayout->addWidget(controlCell, 0, Qt::AlignRight | Qt::AlignVCenter);
    } else {
        rowLayout->addWidget(control, 0, Qt::AlignRight | Qt::AlignVCenter);
    }
    sectionLayout->addWidget(row);
    section->setProperty("rowCount", rowCount + 1);
}

QPushButton* createPreferenceNavButton(const QString& text, const QString& iconPath, QWidget* parent)
{
    QPushButton* button = new QPushButton(text, parent);
    button->setCheckable(true);
    button->setMinimumHeight(44);
    button->setCursor(Qt::PointingHandCursor);
    button->setIcon(QIcon(iconPath));
    button->setIconSize(QSize(22, 22));
    setPreferenceFont(button, 10, QFont::Medium);
    button->setStyleSheet(
        "QPushButton {"
        "  text-align: left;"
        "  padding: 8px 14px;"
        "  border: none;"
        "  border-radius: 6px;"
        "}"
        "QPushButton:hover {"
        "  background: palette(base);"
        "}"
        "QPushButton:checked {"
        "  background: palette(highlight);"
        "  color: palette(highlighted-text);"
        "}"
    );
    return button;
}

QToolButton* createThemeModeButton(const QString& text, const QString& iconPath, QWidget* parent)
{
    QToolButton* button = new QToolButton(parent);
    button->setCheckable(true);
    button->setCursor(Qt::PointingHandCursor);
    button->setIcon(QIcon(iconPath));
    button->setIconSize(QSize(24, 24));
    button->setText(text);
    button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    button->setMinimumHeight(36);
    setPreferenceFont(button, 10, QFont::Medium);
    button->setStyleSheet(
        "QToolButton {"
        "  border: none;"
        "  border-radius: 18px;"
        "  padding: 4px 14px;"
        "  background: transparent;"
        "}"
        "QToolButton:hover {"
        "  background: palette(alternate-base);"
        "}"
        "QToolButton:checked {"
        "  background: palette(mid);"
        "}"
    );
    return button;
}

} // namespace

LivoxViewerWindow::LivoxViewerWindow(QWidget *parent)
    : QMainWindow(parent)
    , sdk_initialized(false)
    , sdk_started(false)
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
    if (settings.value("layout/version", 0).toInt() == kDockStateVersion) {
        restoreGeometry(settings.value("geometry").toByteArray());
        restoreState(settings.value("windowState").toByteArray(), kDockStateVersion);
    }
    if (lvx2FileDock) {
        lvx2FileDock->hide();
    }
    if (attrDock) {
        attrDock->hide();
    }
}

LivoxViewerWindow::~LivoxViewerWindow()
{
    // 保存窗口布局与几何
    QSettings settings("Livox", "LivoxViewerQT");
    if (lvx2FileDock) {
        lvx2FileDock->hide();
    }
    if (attrDock) {
        attrDock->hide();
    }
    settings.setValue("layout/version", kDockStateVersion);
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState(kDockStateVersion));
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
#ifdef Q_OS_LINUX
    const int linuxDarkState = linuxSystemDarkThemeState();
    if (linuxDarkState >= 0) {
        return linuxDarkState == 1;
    }
#endif
#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
    const Qt::ColorScheme colorScheme = QGuiApplication::styleHints()->colorScheme();
    if (colorScheme == Qt::ColorScheme::Dark) {
        return true;
    }
    if (colorScheme == Qt::ColorScheme::Light) {
        return false;
    }
#endif
    return QApplication::style()->standardPalette().color(QPalette::Window).lightness() < 128;
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
    autoConfigHostIpEnabled = settings.value("network/autoConfigHostIp", defaultAutoConfigHostIp()).toBool();
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
    const QStringList storedLineColors = settings.value("color/lineColors").toStringList();
    for (int i = 0; i < storedLineColors.size() && i < lineColors.size(); ++i) {
        const QColor color(storedLineColors.at(i));
        if (color.isValid()) {
            lineColors[i] = color;
        }
    }
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
    QStringList storedLineColors;
    for (const QColor& color : lineColors) {
        storedLineColors.append(color.name(QColor::HexRgb));
    }
    settings.setValue("color/lineColors", storedLineColors);
    settings.setValue("theme/mode", themeMode);
    settings.setValue("network/autoConfigHostIp", autoConfigHostIpEnabled);
}

void LivoxViewerWindow::showPreferencesDialog()
{
    if (!pointCloudView) {
        return;
    }

    QDialog dlg(this);
    dlg.setWindowTitle("首选项");
    dlg.resize(760, 520);
    const int originalThemeMode = themeMode;

    PointCloudView::GridConfig config = pointCloudView->gridConfig();
    QColor selectedColor = config.color;

    QVBoxLayout* layout = new QVBoxLayout(&dlg);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    QHBoxLayout* contentLayout = new QHBoxLayout();
    contentLayout->setContentsMargins(0, 0, 0, 0);
    contentLayout->setSpacing(0);

    QWidget* navigation = new QWidget(&dlg);
    navigation->setFixedWidth(190);
    navigation->setObjectName("PreferencesNavPanel");
    QVBoxLayout* navigationLayout = new QVBoxLayout(navigation);
    navigationLayout->setContentsMargins(14, 18, 14, 18);
    navigationLayout->setSpacing(8);

    QButtonGroup* navigationGroup = new QButtonGroup(navigation);
    navigationGroup->setExclusive(true);
    navigation->setStyleSheet(
        "#PreferencesNavPanel {"
        "  background: palette(alternate-base);"
        "  border-right: 1px solid palette(mid);"
        "}"
    );

    QWidget* settingsContent = new QWidget(&dlg);
    QVBoxLayout* settingsContentLayout = new QVBoxLayout(settingsContent);
    settingsContentLayout->setContentsMargins(0, 0, 18, 14);
    settingsContentLayout->setSpacing(0);

    QStackedWidget* pages = new QStackedWidget(settingsContent);
    auto createSettingsPage = [pages](const QString& title, const QString& description) {
        QWidget* pageContainer = new QWidget(pages);
        QVBoxLayout* containerLayout = new QVBoxLayout(pageContainer);
        containerLayout->setContentsMargins(0, 0, 0, 0);
        containerLayout->setSpacing(0);

        QScrollArea* scrollArea = new QScrollArea(pageContainer);
        scrollArea->setWidgetResizable(true);
        scrollArea->setFrameShape(QFrame::NoFrame);
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

        QWidget* page = new QWidget(scrollArea);
        QVBoxLayout* pageLayout = new QVBoxLayout(page);
        pageLayout->setContentsMargins(24, 18, 24, 18);
        pageLayout->setSpacing(12);

        QLabel* titleLabel = new QLabel(title, page);
        setPreferenceFont(titleLabel, 13, QFont::Bold);
        pageLayout->addWidget(titleLabel);
        pageLayout->addWidget(createPreferenceDescription(description, page));
        scrollArea->setWidget(page);
        containerLayout->addWidget(scrollArea);
        pages->addWidget(pageContainer);
        return page;
    };

    QWidget* themeTab = createSettingsPage("主题", "设置应用界面的颜色模式。");
    QWidget* connectionTab = createSettingsPage("连接", "配置与雷达连接时的主机网络行为。");
    QWidget* gridTab = createSettingsPage("网格", "调整点云视图中的世界坐标网格。");
    QWidget* legendTab = createSettingsPage("图例", "设置距离和高度着色图例的数值范围。");
    QWidget* colorTab = createSettingsPage("着色", "设置纯色着色模式使用的颜色。");

    QVBoxLayout* themeLayout = qobject_cast<QVBoxLayout*>(themeTab->layout());
    QVBoxLayout* connectionLayout = qobject_cast<QVBoxLayout*>(connectionTab->layout());
    QVBoxLayout* gridLayout = qobject_cast<QVBoxLayout*>(gridTab->layout());
    QVBoxLayout* legendLayout = qobject_cast<QVBoxLayout*>(legendTab->layout());
    QVBoxLayout* colorPageLayout = qobject_cast<QVBoxLayout*>(colorTab->layout());
    QColor selectedSolidColor = solidColor;
    QVector<QColor> selectedLineColors = lineColors;
    QButtonGroup* themeGroup = new QButtonGroup(&dlg);
    QWidget* themeOptions = new QWidget(themeTab);
    QHBoxLayout* themeOptionsLayout = new QHBoxLayout(themeOptions);
    themeOptionsLayout->setContentsMargins(0, 0, 0, 0);
    themeOptionsLayout->setSpacing(4);
    QToolButton* lightThemeButton = createThemeModeButton("浅色", ":/icons/settings_theme_light.svg", themeOptions);
    QToolButton* darkThemeButton = createThemeModeButton("深色", ":/icons/settings_theme_dark.svg", themeOptions);
    QToolButton* followThemeButton = createThemeModeButton("系统", ":/icons/settings_theme_system.svg", themeOptions);
    themeGroup->addButton(lightThemeButton, ThemeLight);
    themeGroup->addButton(darkThemeButton, ThemeDark);
    themeGroup->addButton(followThemeButton, ThemeFollowSystem);
    if (QAbstractButton* checkedThemeButton = themeGroup->button(std::clamp(themeMode, int(ThemeFollowSystem), int(ThemeDark)))) {
        checkedThemeButton->setChecked(true);
    }
    themeOptionsLayout->addWidget(lightThemeButton);
    themeOptionsLayout->addWidget(darkThemeButton);
    themeOptionsLayout->addWidget(followThemeButton);
    themeOptionsLayout->addStretch();
    connect(themeGroup, &QButtonGroup::idToggled, &dlg, [this](int id, bool checked) {
        if (checked) {
            themeMode = id;
            applyUiTheme();
        }
    });

    SwitchCheckBox* autoConfigHostIpPreferenceCheck = new SwitchCheckBox(connectionTab);
    autoConfigHostIpPreferenceCheck->setChecked(autoConfigHostIpEnabled);
    autoConfigHostIpPreferenceCheck->setToolTip("开启后程序会尝试自动将所选网卡的 IPv4 配置到与雷达同网段。\nLinux 可能需要 sudo/root 权限；默认关闭以避免权限导致的失败。");

    QDoubleSpinBox* rangeSpin = new QDoubleSpinBox(&dlg);
    rangeSpin->setRange(1.0, 10000.0);
    rangeSpin->setDecimals(1);
    rangeSpin->setSingleStep(10.0);
    rangeSpin->setSuffix(" m");
    rangeSpin->setValue(config.range);
    preparePreferenceSpinBox(rangeSpin);

    QDoubleSpinBox* stepSpin = new QDoubleSpinBox(&dlg);
    stepSpin->setRange(0.1, 1000.0);
    stepSpin->setDecimals(1);
    stepSpin->setSingleStep(0.5);
    stepSpin->setSuffix(" m");
    stepSpin->setValue(config.step);
    preparePreferenceSpinBox(stepSpin);

    QWidget* colorRow = new QWidget(&dlg);
    usePreferenceControlColumn(colorRow);
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
    usePreferenceControlColumn(typeRow);
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
        preparePreferenceSpinBox(spin);
        return spin;
    };
    QDoubleSpinBox* distanceMinSpin = createLegendSpin(0.0, 100000.0, distanceLegendMin);
    QDoubleSpinBox* distanceMaxSpin = createLegendSpin(distanceLegendMin + 0.01, 100000.0, distanceLegendMax);
    QDoubleSpinBox* elevationMinSpin = createLegendSpin(-100000.0, elevationLegendMax - 0.01, elevationLegendMin);
    QDoubleSpinBox* elevationMaxSpin = createLegendSpin(elevationLegendMin + 0.01, 100000.0, elevationLegendMax);

    QWidget* solidColorRow = new QWidget(&dlg);
    usePreferenceControlColumn(solidColorRow);
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

    QVector<QWidget*> lineColorRows;
    QVector<QWidget*> lineLegendRows;
    for (int i = 0; i < selectedLineColors.size(); ++i) {
        QWidget* row = new QWidget(&dlg);
        usePreferenceControlColumn(row);
        QHBoxLayout* rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->setSpacing(8);
        QFrame* preview = new QFrame(row);
        preview->setFixedSize(28, 20);
        preview->setFrameShape(QFrame::Box);
        preview->setLineWidth(1);
        preview->setStyleSheet(QString("background-color: %1;").arg(selectedLineColors.at(i).name()));
        QPushButton* button = new QPushButton("选择颜色", row);
        rowLayout->addWidget(preview);
        rowLayout->addWidget(button);
        rowLayout->addStretch();
        QWidget* legendRow = new QWidget(&dlg);
        QHBoxLayout* legendRowLayout = new QHBoxLayout(legendRow);
        legendRowLayout->setContentsMargins(0, 0, 0, 0);
        legendRowLayout->setSpacing(8);
        QFrame* legendPreview = new QFrame(legendRow);
        legendPreview->setFixedSize(28, 20);
        legendPreview->setFrameShape(QFrame::Box);
        legendPreview->setLineWidth(1);
        legendPreview->setStyleSheet(QString("background-color: %1;").arg(selectedLineColors.at(i).name()));
        QLabel* legendColorLabel = new QLabel(selectedLineColors.at(i).name(QColor::HexRgb).toUpper(), legendRow);
        legendRowLayout->addWidget(legendPreview);
        legendRowLayout->addWidget(legendColorLabel);
        legendRowLayout->addStretch();
        connect(button, &QPushButton::clicked, &dlg, [&dlg, &selectedLineColors, preview, legendPreview, legendColorLabel, i]() {
            QColor color = QColorDialog::getColor(selectedLineColors.at(i), &dlg, QString("选择 Line %1 点云颜色").arg(i));
            if (!color.isValid()) {
                return;
            }
            selectedLineColors[i] = color;
            preview->setStyleSheet(QString("background-color: %1;").arg(color.name()));
            legendPreview->setStyleSheet(QString("background-color: %1;").arg(color.name()));
            legendColorLabel->setText(color.name(QColor::HexRgb).toUpper());
        });
        lineColorRows.append(row);
        usePreferenceControlColumn(legendRow);
        lineLegendRows.append(legendRow);
    }

    connect(distanceMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [distanceMaxSpin](double value) { distanceMaxSpin->setMinimum(value + 0.01); });
    connect(distanceMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [distanceMinSpin](double value) { distanceMinSpin->setMaximum(value - 0.01); });
    connect(elevationMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [elevationMaxSpin](double value) { elevationMaxSpin->setMinimum(value + 0.01); });
    connect(elevationMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [elevationMinSpin](double value) { elevationMinSpin->setMaximum(value - 0.01); });

    QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, settingsContent);
    connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    addPreferenceSectionTitle(themeLayout, "外观");
    QFrame* themeSection = createPreferenceSection(themeTab);
    addPreferenceRow(themeSection, "应用主题", "选择应用界面的明暗模式。", themeOptions);
    themeLayout->addWidget(themeSection);
    themeLayout->addStretch();

    addPreferenceSectionTitle(connectionLayout, "主机网络");
    QFrame* connectionSection = createPreferenceSection(connectionTab);
    addPreferenceRow(connectionSection, "自动配置主机 IP", "启用后，程序会在连接前尝试把所选网卡 IP 调整到与雷达同网段。", autoConfigHostIpPreferenceCheck);
    connectionLayout->addWidget(connectionSection);
    connectionLayout->addStretch();

    addPreferenceSectionTitle(gridLayout, "世界坐标网格");
    QFrame* gridSection = createPreferenceSection(gridTab);
    addPreferenceRow(gridSection, "范围", "控制网格从中心向外显示的最大距离。", rangeSpin);
    addPreferenceRow(gridSection, "间距", "控制相邻网格线或圆环之间的距离。", stepSpin);
    addPreferenceRow(gridSection, "颜色", "设置网格线的显示颜色。", colorRow);
    addPreferenceRow(gridSection, "类型", "选择方形网格或同心圆网格。", typeRow);
    gridLayout->addWidget(gridSection);
    gridLayout->addStretch();

    addPreferenceSectionTitle(legendLayout, "距离图例");
    QFrame* distanceLegendSection = createPreferenceSection(legendTab);
    addPreferenceRow(distanceLegendSection, "低值", "距离颜色映射的下限。", distanceMinSpin);
    addPreferenceRow(distanceLegendSection, "高值", "距离颜色映射的上限。", distanceMaxSpin);
    legendLayout->addWidget(distanceLegendSection);
    addPreferenceSectionTitle(legendLayout, "高度图例");
    QFrame* elevationLegendSection = createPreferenceSection(legendTab);
    addPreferenceRow(elevationLegendSection, "低值", "高度颜色映射的下限。", elevationMinSpin);
    addPreferenceRow(elevationLegendSection, "高值", "高度颜色映射的上限。", elevationMaxSpin);
    legendLayout->addWidget(elevationLegendSection);
    addPreferenceSectionTitle(legendLayout, "线号图例");
    QFrame* lineLegendSection = createPreferenceSection(legendTab);
    for (int i = 0; i < lineLegendRows.size(); ++i) {
        addPreferenceRow(lineLegendSection, QString("Line %1").arg(i), QString(), lineLegendRows.at(i));
    }
    legendLayout->addWidget(lineLegendSection);
    legendLayout->addStretch();

    addPreferenceSectionTitle(colorPageLayout, "纯色模式");
    QFrame* colorSection = createPreferenceSection(colorTab);
    addPreferenceRow(colorSection, "点云颜色", "设置纯色着色模式下所有点的显示颜色。", solidColorRow);
    colorPageLayout->addWidget(colorSection);
    addPreferenceSectionTitle(colorPageLayout, "线号模式");
    QFrame* lineColorSection = createPreferenceSection(colorTab);
    for (int i = 0; i < lineColorRows.size(); ++i) {
        addPreferenceRow(lineColorSection,
                         QString("Line %1").arg(i),
                         QString("设置线号 %1 的点云颜色。").arg(i),
                         lineColorRows.at(i));
    }
    colorPageLayout->addWidget(lineColorSection);
    colorPageLayout->addStretch();

    const QStringList navigationNames = {"主题", "连接", "网格", "图例", "着色"};
    const QStringList navigationIcons = {
        ":/icons/settings_theme.svg",
        ":/icons/settings_connection.svg",
        ":/icons/settings_grid.svg",
        ":/icons/settings_legend.svg",
        ":/icons/settings_color.svg"
    };
    for (int i = 0; i < navigationNames.size(); ++i) {
        QPushButton* button = createPreferenceNavButton(navigationNames.at(i), navigationIcons.at(i), navigation);
        navigationGroup->addButton(button, i);
        navigationLayout->addWidget(button);
    }
    navigationLayout->addStretch();
    connect(navigationGroup, &QButtonGroup::idClicked, pages, [pages](int id) {
        pages->setCurrentIndex(id);
    });
    if (QAbstractButton* firstButton = navigationGroup->button(0)) {
        firstButton->setChecked(true);
    }

    settingsContentLayout->addWidget(pages, 1);
    settingsContentLayout->addWidget(box, 0, Qt::AlignRight);

    contentLayout->addWidget(navigation);
    contentLayout->addWidget(settingsContent, 1);
    layout->addLayout(contentLayout, 1);

    if (dlg.exec() != QDialog::Accepted) {
        themeMode = originalThemeMode;
        applyUiTheme();
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
    lineColors = selectedLineColors;
    themeMode = themeGroup->checkedId();
    const bool previousAutoConfigHostIpEnabled = autoConfigHostIpEnabled;
    autoConfigHostIpEnabled = autoConfigHostIpPreferenceCheck->isChecked();

    pointCloudView->setGridConfig(config);
    applyUiTheme();
    updatePointCloudLegend();
    if (playbackState.active && playbackState.frame >= 0) {
        showLvx2PlaybackFrame(playbackState.frame);
    }
    saveViewPreferences();
    if (autoConfigHostIpEnabled != previousAutoConfigHostIpEnabled) {
        logMessage(QString("自动修改主机IP: %1").arg(autoConfigHostIpEnabled ? "已启用" : "已关闭"));
    }
}

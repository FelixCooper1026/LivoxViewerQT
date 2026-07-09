#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"
#include "dialogs/ImuVisualizationDialog.h"
#include "slam/SlamUiBridge.h"
#include "widgets/SwitchCheckBox.h"

#include "PointCloudColorizer.h"

#include <QAbstractButton>
#include <QAbstractItemView>
#include <QApplication>
#include <QButtonGroup>
#include <QChildEvent>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFile>
#include <QFont>
#include <QFrame>
#include <QGridLayout>
#include <QGuiApplication>
#include <QHash>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QPalette>
#include <QPair>
#include <QPointer>
#include <QProcess>
#include <QProcessEnvironment>
#include <QProxyStyle>
#include <QPushButton>
#include <QRadioButton>
#include <QSettings>
#include <QSize>
#include <QSizePolicy>
#include <QScrollArea>
#include <QSpinBox>
#include <QStyle>
#include <QStyleFactory>
#include <QStyleHints>
#include <QStackedWidget>
#include <QTabBar>
#include <QTabWidget>
#include <QStringList>
#include <QTextStream>
#include <QTimer>
#include <QToolButton>
#include <QtGlobal>
#include <QVariant>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <array>
#include <cmath>

namespace {

constexpr int kDockStateVersion = 6;
constexpr int kPreferenceControlColumnWidth = 104;
constexpr int kPreferenceComboBoxWidth = 104;
constexpr int kPreferenceSpinBoxWidth = 104;
constexpr int kPreferenceFontPointIncrease = 1;
constexpr int kPreferenceNavIconTextSpacingChars = 2;

class ComboBoxPopupStyle : public QProxyStyle
{
public:
    explicit ComboBoxPopupStyle(QStyle* baseStyle)
        : QProxyStyle(baseStyle)
    {}

    int styleHint(StyleHint hint,
                  const QStyleOption* option = nullptr,
                  const QWidget* widget = nullptr,
                  QStyleHintReturn* returnData = nullptr) const override
    {
        if (hint == QStyle::SH_ComboBox_Popup) {
            return 0;
        }
        return QProxyStyle::styleHint(hint, option, widget, returnData);
    }
};

class ComboBoxPopupBehaviorFilter : public QObject
{
public:
    explicit ComboBoxPopupBehaviorFilter(QObject* parent = nullptr)
        : QObject(parent)
    {}

    void installRecursively(QObject* object)
    {
        if (!object) {
            return;
        }
        if (QComboBox* combo = qobject_cast<QComboBox*>(object)) {
            installComboBox(combo);
        }
        const QObjectList children = object->children();
        for (QObject* child : children) {
            installRecursively(child);
        }
    }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (event->type() == QEvent::ChildAdded) {
            QPointer<QObject> child(static_cast<QChildEvent*>(event)->child());
            QTimer::singleShot(0, this, [this, child]() {
                installRecursively(child);
            });
        } else if (event->type() == QEvent::Polish || event->type() == QEvent::Show) {
            if (QComboBox* combo = qobject_cast<QComboBox*>(watched)) {
                installComboBox(combo);
            } else if (QAbstractItemView* view = qobject_cast<QAbstractItemView*>(watched)) {
                keepPopupAtTop(view);
            }
        }
        return QObject::eventFilter(watched, event);
    }

private:
    void installComboBox(QComboBox* combo)
    {
        if (!combo || !combo->view()) {
            return;
        }

        QAbstractItemView* view = combo->view();
        if (!view->property("livoxComboPopupBehaviorInstalled").toBool()) {
            view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
            view->setProperty("livoxComboPopupBehaviorInstalled", true);
            view->installEventFilter(this);
        }
        view->setProperty("livoxComboPopupOwner", QVariant::fromValue<QObject*>(combo));
    }

    void keepPopupAtTop(QAbstractItemView* view)
    {
        QObject* owner = view->property("livoxComboPopupOwner").value<QObject*>();
        QComboBox* combo = qobject_cast<QComboBox*>(owner);
        if (!combo || combo->view() != view || combo->count() <= 0) {
            return;
        }
        QTimer::singleShot(0, view, [combo, view]() {
            if (!combo || combo->view() != view || combo->count() <= 0) {
                return;
            }
            const QModelIndex currentIndex =
                combo->model()->index(combo->currentIndex(), combo->modelColumn(), combo->rootModelIndex());
            view->setCurrentIndex(currentIndex);
            const QModelIndex firstIndex = combo->model()->index(0, combo->modelColumn(), combo->rootModelIndex());
            view->scrollTo(firstIndex, QAbstractItemView::PositionAtTop);
            view->viewport()->update();
        });
    }
};

class TrimmedDoubleSpinBox : public QDoubleSpinBox
{
public:
    explicit TrimmedDoubleSpinBox(QWidget* parent = nullptr)
        : QDoubleSpinBox(parent)
    {}

protected:
    QString textFromValue(double value) const override
    {
        QString text = locale().toString(value, 'f', decimals());
        const QString decimalPoint = locale().decimalPoint();
        if (text.contains(decimalPoint)) {
            while (text.endsWith(QLatin1Char('0'))) {
                text.chop(1);
            }
            if (text.endsWith(decimalPoint)) {
                text.chop(1);
            }
        }
        return text == QStringLiteral("-0") ? QStringLiteral("0") : text;
    }
};

void installComboBoxPopupBehavior(QApplication* app)
{
    static QPointer<ComboBoxPopupBehaviorFilter> behaviorFilter;
    if (!app) {
        return;
    }
    if (!behaviorFilter) {
        behaviorFilter = new ComboBoxPopupBehaviorFilter(app);
        app->installEventFilter(behaviorFilter);
    }

    const QWidgetList topLevelWidgets = QApplication::topLevelWidgets();
    for (QWidget* widget : topLevelWidgets) {
        behaviorFilter->installRecursively(widget);
    }
}

void setPreferenceFont(QWidget* widget, int pointSize, QFont::Weight weight)
{
    QFont font = widget->font();
#ifdef Q_OS_WIN
    font.setFamily(QStringLiteral("Microsoft YaHei UI"));
#endif
    font.setPointSize(pointSize + kPreferenceFontPointIncrease);
    font.setWeight(weight);
    widget->setFont(font);
}

void increasePreferenceBaseFont(QWidget* widget)
{
    QFont font = widget->font();
    font.setPointSize(font.pointSize() + kPreferenceFontPointIncrease);
    widget->setFont(font);
}

void usePreferenceControlColumn(QWidget* widget)
{
    widget->setProperty("preferenceControlColumnWidth", kPreferenceControlColumnWidth);
}

void usePreferenceControlColumn(QWidget* widget, int width)
{
    widget->setProperty("preferenceControlColumnWidth", width);
}

void preparePreferenceSpinBox(QDoubleSpinBox* spin)
{
    spin->setFixedWidth(kPreferenceSpinBoxWidth);
    usePreferenceControlColumn(spin);
}

QString colorSwatchStyleSheet(const QColor& color)
{
    return QStringLiteral(
               "QPushButton {"
               "  background-color: %1;"
               "  border: 1px solid palette(mid);"
               "  border-radius: 3px;"
               "  padding: 0;"
               "}"
               "QPushButton:hover {"
               "  border-color: palette(highlight);"
               "}")
        .arg(color.name(QColor::HexRgb));
}

void updateColorSwatchButton(QPushButton* button, const QColor& color)
{
    if (!button) {
        return;
    }
    button->setStyleSheet(colorSwatchStyleSheet(color));
    button->setToolTip(QStringLiteral("点击选择颜色：%1").arg(color.name(QColor::HexRgb).toUpper()));
}

QPushButton* createColorSwatchButton(QWidget* parent, const QColor& color)
{
    QPushButton* button = new QPushButton(parent);
    button->setFixedSize(28, 20);
    button->setCursor(Qt::PointingHandCursor);
    button->setFocusPolicy(Qt::StrongFocus);
    updateColorSwatchButton(button, color);
    return button;
}

void refreshWidgetStyle(QWidget* widget)
{
    if (!widget) {
        return;
    }

    if (widget->property("parameterOptionButton").toBool()) {
        const bool darkTheme = QApplication::palette().color(QPalette::Window).lightness() < 128;
        widget->setProperty("parameterOptionButtonTheme", darkTheme ? QStringLiteral("dark") : QStringLiteral("light"));
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

QString darkThemeControlStyleSheet()
{
    const QString comboArrowIcon = QStringLiteral(":/icons/combo_arrow_dark_theme.svg");
    const QString borderColor = QStringLiteral("#8a8a8d");
    const QString hoverBorderColor = QStringLiteral("#c8c8c8");
    const QString focusBorderColor = QStringLiteral("#f0f0f0");

    return QStringLiteral(
        "QComboBox, QLineEdit, QAbstractSpinBox {"
        "  min-height: 20px;"
        "  border: 1px solid %1;"
        "  border-radius: 4px;"
        "  background: palette(base);"
        "  color: palette(text);"
        "  selection-background-color: palette(highlight);"
        "  selection-color: palette(highlighted-text);"
        "}"
        "QComboBox {"
        "  padding: 2px 20px 2px 6px;"
        "}"
        "QLineEdit, QAbstractSpinBox {"
        "  padding: 2px 6px;"
        "}"
        "QComboBox:hover, QLineEdit:hover, QAbstractSpinBox:hover {"
        "  border-color: %2;"
        "}"
        "QComboBox:focus, QLineEdit:focus, QAbstractSpinBox:focus {"
        "  border-color: %3;"
        "}"
        "QComboBox:disabled, QLineEdit:disabled, QAbstractSpinBox:disabled {"
        "  background: palette(alternate-base);"
        "  color: palette(mid);"
        "  border-color: %1;"
        "}"
        "QComboBox::drop-down {"
        "  subcontrol-origin: padding;"
        "  subcontrol-position: top right;"
        "  width: 18px;"
        "  border-left: 1px solid %1;"
        "  border-top-right-radius: 4px;"
        "  border-bottom-right-radius: 4px;"
        "  background: palette(button);"
        "}"
        "QComboBox::drop-down:hover {"
        "  background: palette(alternate-base);"
        "}"
        "QComboBox::down-arrow {"
        "  image: url(%4);"
        "  width: 8px;"
        "  height: 8px;"
        "}"
        "QComboBox QAbstractItemView {"
        "  border: 1px solid %1;"
        "  background: palette(base);"
        "  color: palette(text);"
        "  selection-background-color: palette(highlight);"
        "  selection-color: palette(highlighted-text);"
        "  outline: 0;"
        "}"
        "QCheckBox {"
        "  color: palette(window-text);"
        "  spacing: 6px;"
        "}"
        "QCheckBox:disabled {"
        "  color: palette(mid);"
        "}"
        "QCheckBox::indicator {"
        "  width: 15px;"
        "  height: 15px;"
        "  border: 1px solid %1;"
        "  border-radius: 3px;"
        "  background: palette(base);"
        "}"
        "QCheckBox::indicator:hover {"
        "  border-color: %2;"
        "}"
        "QCheckBox::indicator:focus {"
        "  border-color: %3;"
        "}"
        "QCheckBox::indicator:checked {"
        "  image: url(:/icons/checkmark_control.svg);"
        "  background: palette(highlight);"
        "  border-color: palette(highlight);"
        "}"
        "QCheckBox::indicator:unchecked:disabled {"
        "  background: palette(alternate-base);"
        "  border-color: %1;"
        "}"
        "QCheckBox::indicator:checked:disabled {"
        "  image: url(:/icons/checkmark_control.svg);"
        "  background: palette(mid);"
        "  border-color: palette(mid);"
        "}"
    ).arg(borderColor, hoverBorderColor, focusBorderColor, comboArrowIcon);
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
    setPreferenceFont(label, 10, QFont::Medium);
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
    setPreferenceFont(titleLabel, 9, QFont::Medium);
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
    QPushButton* button = new QPushButton(QString(kPreferenceNavIconTextSpacingChars, QLatin1Char(' ')) + text, parent);
    button->setCheckable(true);
    button->setMinimumHeight(44);
    button->setCursor(Qt::PointingHandCursor);
    ThemeIconUtils::setThemedSvgIcon(button, iconPath);
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
    ThemeIconUtils::setThemedSvgIcon(button, iconPath);
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
    setWindowFlag(Qt::FramelessWindowHint, true);
    setWindowTitle(QApplication::applicationDisplayName());
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
    restoreGeometry(settings.value("geometry").toByteArray());
    if (settings.value("layout/version", 0).toInt() == kDockStateVersion) {
        const bool restored = restoreState(settings.value("windowState").toByteArray(), kDockStateVersion);
        if (!restored) {
            settings.remove("windowState");
        }
    }
    mainToolBar->show();
    if (lvx2FileDock) {
        lvx2FileDock->hide();
    }
    if (slamInfoDock && slamVisualizationTabId < 0) {
        slamInfoDock->hide();
        if (lidarDevicesDock) {
            lidarDevicesDock->raise();
        }
    }
    if (attrDock) {
        attrDock->hide();
    }
    if (slamStatusDock && !slamUiBridge) {
        slamStatusDock->hide();
    }

    QTimer::singleShot(0, this, [this]() {
        for (QTabBar* tabBar : findChildren<QTabBar*>()) {
            bool isDeviceDockTabs = false;
            bool isParameterDockTabs = false;
            bool isBottomDockTabs = false;
            for (int index = 0; index < tabBar->count(); ++index) {
                const QString text = tabBar->tabText(index);
                if (text == QStringLiteral("设备") ||
                    text == QStringLiteral("IMU数据") ||
                    text == QStringLiteral("文件信息")) {
                    isDeviceDockTabs = true;
                } else if (text == QStringLiteral("参数") ||
                           text == QStringLiteral("点属性")) {
                    isParameterDockTabs = true;
                } else if (text == QStringLiteral("日志") ||
                           text == QStringLiteral("SLAM状态")) {
                    isBottomDockTabs = true;
                }
            }
            if (!isDeviceDockTabs && !isParameterDockTabs && !isBottomDockTabs) {
                continue;
            }

            tabBar->setObjectName(isDeviceDockTabs
                ? QStringLiteral("DeviceDockTabBar")
                : (isParameterDockTabs
                    ? QStringLiteral("ParameterDockTabBar")
                    : QStringLiteral("BottomDockTabBar")));
            tabBar->setDocumentMode(true);
            tabBar->setDrawBase(false);
            tabBar->setExpanding(false);
            tabBar->setElideMode(Qt::ElideNone);
            tabBar->setUsesScrollButtons(false);
            const QString objectSelector = isDeviceDockTabs
                ? QStringLiteral("QTabBar#DeviceDockTabBar")
                : (isParameterDockTabs
                    ? QStringLiteral("QTabBar#ParameterDockTabBar")
                    : QStringLiteral("QTabBar#BottomDockTabBar"));
            tabBar->setStyleSheet(QStringLiteral(
                "%1 {"
                "  qproperty-drawBase: false;"
                "  border: none;"
                "  background: transparent;"
                "}"
                "%1::base {"
                "  border: none;"
                "  background: transparent;"
                "}"
                "%1::tab {"
                "  border: none;"
                "  border-bottom: 2px solid transparent;"
                "  background: transparent;"
                "  padding: 5px 8px;"
                "  margin-right: 2px;"
                "}"
                "%1::tab:selected {"
                "  color: #2f8cff;"
                "  border-bottom-color: #2f8cff;"
                "  font-weight: 600;"
                "}"
                "%1::tab:!selected {"
                "  color: palette(window-text);"
                "}"
            ).arg(objectSelector));
        }
    });
}

LivoxViewerWindow::~LivoxViewerWindow()
{
    if (slamUiBridge || slamWorker.joinable()) {
        stopSlamProcessing();
    }
    if (slamMapExportWorker.joinable()) {
        slamMapExportWorker.join();
    }

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

    app->setStyle(new ComboBoxPopupStyle(QStyleFactory::create("Fusion")));
    QPalette palette;

    const bool darkTheme = shouldUseDarkTheme();
    if (darkTheme) {
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
        palette.setColor(QPalette::Mid, QColor(138, 138, 141));
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
    app->setStyleSheet(darkTheme ? darkThemeControlStyleSheet() : QString());
    installComboBoxPopupBehavior(app);
    ThemeIconUtils::refreshObject(this);
    refreshApplicationStyles();
    if (imuState.visualizationDialog) {
        imuState.visualizationDialog->refreshTheme();
    }
}

void LivoxViewerWindow::loadViewPreferences()
{
    if (!pointCloudView) {
        return;
    }

    QSettings settings("Livox", "LivoxViewerQT");
    slamRuntimeConfig = loadSlamRuntimeConfig(settings, QStringLiteral("slam/runtime"));
    liveSlamSource.setFrameDurationMs(slamRuntimeConfig.inputFrameDurationMs);
    slamWorldCurrentFrameColor = settings.value(QStringLiteral("slam/worldCurrentFrameColor"), slamWorldCurrentFrameColor).value<QColor>();
    if (!slamWorldCurrentFrameColor.isValid()) {
        slamWorldCurrentFrameColor = QColor(255, 255, 255);
    }
    slamWorldCurrentFramePointSizePx = std::clamp(
        settings.value(QStringLiteral("slam/worldCurrentFramePointSizePx"), slamWorldCurrentFramePointSizePx).toFloat(),
        1.0f,
        10.0f);
    const bool hasSlamBodyFrameColor = settings.contains(QStringLiteral("slam/bodyFrameColor"));
    slamBodyFrameColor = settings.value(QStringLiteral("slam/bodyFrameColor"), slamBodyFrameColor).value<QColor>();
    if (!slamBodyFrameColor.isValid() ||
        (hasSlamBodyFrameColor && slamBodyFrameColor == QColor(0, 255, 0))) {
        slamBodyFrameColor = QColor(255, 140, 26);
    }
    slamBodyFramePointSizePx = std::clamp(
        settings.value(QStringLiteral("slam/bodyFramePointSizePx"), slamBodyFramePointSizePx).toFloat(),
        1.0f,
        10.0f);
    slamTrajectoryColor = settings.value(QStringLiteral("slam/trajectoryColor"), slamTrajectoryColor).value<QColor>();
    if (!slamTrajectoryColor.isValid()) {
        slamTrajectoryColor = QColor(26, 191, 255);
    }
    slamTrajectoryLineWidthPx = std::clamp(
        settings.value(QStringLiteral("slam/trajectoryLineWidthPx"), slamTrajectoryLineWidthPx).toFloat(),
        1.0f,
        10.0f);
    slamPoseAxisLengthM = std::clamp(
        settings.value(QStringLiteral("slam/poseAxisLengthM"), slamPoseAxisLengthM).toFloat(),
        0.1f,
        10.0f);
    slamPoseAxisLineWidthPx = std::clamp(
        settings.value(QStringLiteral("slam/poseAxisLineWidthPx"), slamPoseAxisLineWidthPx).toFloat(),
        1.0f,
        10.0f);
    rebuildSlamInfoPanel();
    syncSlamRenderLayerVisibility();
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
    reflectivityColorScale = settings.value("color/reflectivityScale", reflectivityColorScale).toInt();
    if (reflectivityColorScale < 0 || reflectivityColorScale > PointCloudColorizer::ReflectivityGISTEarth) {
        reflectivityColorScale = 0;
    }
    solidColor = settings.value("color/solidColor", solidColor).value<QColor>();
    pointCloudBackgroundPreset = settings.value("background/preset", pointCloudBackgroundPreset).toInt();
    if (pointCloudBackgroundPreset < BackgroundDeepBlack || pointCloudBackgroundPreset > BackgroundPureWhite) {
        pointCloudBackgroundPreset = BackgroundGraphite;
    }
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
        config.type != PointCloudView::GridConfig::ConcentricCircles &&
        config.type != PointCloudView::GridConfig::SquareAndConcentricCircles) {
        config.type = PointCloudView::GridConfig::Square;
    }

    pointCloudView->setGridConfig(config);
    applyPointCloudBackground();
    updatePointCloudLegend();
    syncReflectivityColorScaleControls();
}

QString colorBarStyleSheet(const QVector<QColor>& colors)
{
    QStringList stops;
    const int colorCount = int(colors.size());
    const int last = colorCount - 1;
    for (int i = 0; i < colorCount; ++i) {
        stops.append(QString("stop:%1 %2").arg(double(i) / double(last), 0, 'f', 3).arg(colors.at(i).name()));
    }
    return QString("QFrame { border: 1px solid palette(mid); border-radius: 3px; background: qlineargradient(x1:0, y1:0, x2:1, y2:0, %1); }")
        .arg(stops.join(", "));
}

QString verticalColorBarStyleSheet(const QColor& topColor, const QColor& bottomColor)
{
    return QString("QFrame { border: 1px solid palette(mid); border-radius: 3px; background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 %1, stop:1 %2); }")
        .arg(topColor.name(), bottomColor.name());
}

QPair<QColor, QColor> backgroundPresetColors(int preset)
{
    switch (preset) {
    case 0: return {QColor("#050608"), QColor("#050608")};
    case 2: return {QColor("#071426"), QColor("#0F2942")};
    case 3: return {QColor("#263442"), QColor("#101820")};
    case 4: return {QColor("#DCE8F2"), QColor("#405A72")};
    case 5: return {QColor("#656B72"), QColor("#30343A")};
    case 6: return {QColor("#F1F3F5"), QColor("#D9DEE3")};
    case 7: return {QColor("#FFFFFF"), QColor("#FFFFFF")};
    case 1:
    default: return {QColor("#181B20"), QColor("#181B20")};
    }
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
    settings.setValue("color/reflectivityScale", reflectivityColorScale);
    settings.setValue("color/solidColor", solidColor);
    settings.setValue("background/preset", pointCloudBackgroundPreset);
    QStringList storedLineColors;
    for (const QColor& color : lineColors) {
        storedLineColors.append(color.name(QColor::HexRgb));
    }
    settings.setValue("color/lineColors", storedLineColors);
    settings.setValue("theme/mode", themeMode);
    settings.setValue("network/autoConfigHostIp", autoConfigHostIpEnabled);
    settings.setValue(QStringLiteral("slam/worldCurrentFrameColor"), slamWorldCurrentFrameColor);
    settings.setValue(QStringLiteral("slam/worldCurrentFramePointSizePx"), slamWorldCurrentFramePointSizePx);
    settings.setValue(QStringLiteral("slam/bodyFrameColor"), slamBodyFrameColor);
    settings.setValue(QStringLiteral("slam/bodyFramePointSizePx"), slamBodyFramePointSizePx);
    settings.setValue(QStringLiteral("slam/trajectoryColor"), slamTrajectoryColor);
    settings.setValue(QStringLiteral("slam/trajectoryLineWidthPx"), slamTrajectoryLineWidthPx);
    settings.setValue(QStringLiteral("slam/poseAxisLengthM"), slamPoseAxisLengthM);
    settings.setValue(QStringLiteral("slam/poseAxisLineWidthPx"), slamPoseAxisLineWidthPx);
    saveSlamRuntimeConfig(settings, slamRuntimeConfig, QStringLiteral("slam/runtime"));
}

void LivoxViewerWindow::showPreferencesDialog()
{
    if (!pointCloudView) {
        return;
    }

    QDialog dlg(this);
    dlg.setWindowTitle("首选项");
    dlg.resize(900, 640);
    increasePreferenceBaseFont(&dlg);
    const int originalThemeMode = themeMode;

    PointCloudView::GridConfig config = pointCloudView->gridConfig();
    QColor selectedColor = config.color;
    QColor selectedSlamWorldCurrentFrameColor = slamWorldCurrentFrameColor;
    QColor selectedSlamBodyFrameColor = slamBodyFrameColor;
    QColor selectedSlamTrajectoryColor = slamTrajectoryColor;

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
        setPreferenceFont(titleLabel, 13, QFont::DemiBold);
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
    QWidget* legendTab = createSettingsPage("图例", "设置距离、高度和线号着色图例。");
    QWidget* colorTab = createSettingsPage("着色", "设置纯色着色模式使用的颜色。");
    QWidget* backgroundTab = createSettingsPage("背景", "设置 OpenGL 点云视图的背景颜色。");
    QWidget* slamTab = createSettingsPage("SLAM", "设置旁路 SLAM 后端和轨迹显示参数。");

    QVBoxLayout* themeLayout = qobject_cast<QVBoxLayout*>(themeTab->layout());
    QVBoxLayout* connectionLayout = qobject_cast<QVBoxLayout*>(connectionTab->layout());
    QVBoxLayout* gridLayout = qobject_cast<QVBoxLayout*>(gridTab->layout());
    QVBoxLayout* legendLayout = qobject_cast<QVBoxLayout*>(legendTab->layout());
    QVBoxLayout* colorPageLayout = qobject_cast<QVBoxLayout*>(colorTab->layout());
    QVBoxLayout* backgroundLayout = qobject_cast<QVBoxLayout*>(backgroundTab->layout());
    QVBoxLayout* slamLayout = qobject_cast<QVBoxLayout*>(slamTab->layout());
    QColor selectedSolidColor = solidColor;
    QVector<QColor> selectedLineColors = lineColors;
    int selectedReflectivityColorScale = reflectivityColorScale;
    int selectedBackgroundPreset = pointCloudBackgroundPreset;
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
    QPushButton* colorPreview = createColorSwatchButton(colorRow, selectedColor);
    colorLayout->addWidget(colorPreview);
    colorLayout->addStretch();
    connect(colorPreview, &QPushButton::clicked, &dlg, [&dlg, &selectedColor, colorPreview]() {
        QColor color = QColorDialog::getColor(selectedColor, &dlg, "选择网格颜色");
        if (!color.isValid()) {
            return;
        }
        selectedColor = color;
        updateColorSwatchButton(colorPreview, selectedColor);
    });

    QComboBox* gridTypeCombo = new QComboBox(&dlg);
    gridTypeCombo->addItem(QStringLiteral("方形"), int(PointCloudView::GridConfig::Square));
    gridTypeCombo->addItem(QStringLiteral("同心圆"), int(PointCloudView::GridConfig::ConcentricCircles));
    gridTypeCombo->addItem(QStringLiteral("方形 + 同心圆"), int(PointCloudView::GridConfig::SquareAndConcentricCircles));
    const int gridTypeIndex = gridTypeCombo->findData(int(config.type));
    gridTypeCombo->setCurrentIndex(gridTypeIndex >= 0 ? gridTypeIndex : 0);
    gridTypeCombo->setFixedWidth(kPreferenceComboBoxWidth);
    gridTypeCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    usePreferenceControlColumn(gridTypeCombo, kPreferenceComboBoxWidth);

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

    QWidget* reflectivityScaleRow = new QWidget(&dlg);
    usePreferenceControlColumn(reflectivityScaleRow, 300);
    QVBoxLayout* reflectivityScaleLayout = new QVBoxLayout(reflectivityScaleRow);
    reflectivityScaleLayout->setContentsMargins(0, 0, 0, 0);
    reflectivityScaleLayout->setSpacing(6);
    QComboBox* reflectivityScaleCombo = new QComboBox(reflectivityScaleRow);
    reflectivityScaleCombo->addItems({
        QStringLiteral("BGYR (Blue → Green → Yellow → Red)"),
        QStringLiteral("Rainbow"),
        QStringLiteral("Viridis"),
        QStringLiteral("Turbo"),
        QStringLiteral("Cividis"),
        QStringLiteral("High contrast"),
        QStringLiteral("Grayscale"),
        QStringLiteral("Plasma"),
        QStringLiteral("Spectral"),
        QStringLiteral("Terrain"),
        QStringLiteral("GISTEarth")
    });
    reflectivityScaleCombo->setCurrentIndex(selectedReflectivityColorScale);
    reflectivityScaleCombo->setFixedWidth(280);
    QFrame* reflectivityScalePreview = new QFrame(reflectivityScaleRow);
    reflectivityScalePreview->setFixedSize(280, 20);
    reflectivityScalePreview->setStyleSheet(colorBarStyleSheet(
        PointCloudColorizer::reflectivityColorScaleStops(selectedReflectivityColorScale)));
    reflectivityScaleLayout->addWidget(reflectivityScaleCombo);
    reflectivityScaleLayout->addWidget(reflectivityScalePreview);
    connect(reflectivityScaleCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dlg,
            [&selectedReflectivityColorScale, reflectivityScalePreview](int index) {
                selectedReflectivityColorScale = index;
                reflectivityScalePreview->setStyleSheet(colorBarStyleSheet(
                    PointCloudColorizer::reflectivityColorScaleStops(selectedReflectivityColorScale)));
            });

    QComboBox* backgroundPresetCombo = new QComboBox(backgroundTab);
    backgroundPresetCombo->addItem(QStringLiteral("深黑 (Deep Black)"), BackgroundDeepBlack);
    backgroundPresetCombo->addItem(QStringLiteral("石墨黑 (Graphite)"), BackgroundGraphite);
    backgroundPresetCombo->addItem(QStringLiteral("午夜蓝 (Midnight Blue)"), BackgroundMidnightBlue);
    backgroundPresetCombo->addItem(QStringLiteral("深灰蓝 (Slate)"), BackgroundSlate);
    backgroundPresetCombo->addItem(QStringLiteral("CloudCompare 经典 (CloudCompare Classic)"), BackgroundCloudCompareClassic);
    backgroundPresetCombo->addItem(QStringLiteral("中性灰 (Neutral Gray)"), BackgroundNeutralGray);
    backgroundPresetCombo->addItem(QStringLiteral("浅灰 (Light Gray)"), BackgroundLightGray);
    backgroundPresetCombo->addItem(QStringLiteral("纯白 (Pure White)"), BackgroundPureWhite);
    const int backgroundIndex = backgroundPresetCombo->findData(selectedBackgroundPreset);
    backgroundPresetCombo->setCurrentIndex(backgroundIndex >= 0 ? backgroundIndex : 1);
    backgroundPresetCombo->setFixedWidth(360);
    usePreferenceControlColumn(backgroundPresetCombo, 360);
    const QPair<QColor, QColor> initialBackgroundColors = backgroundPresetColors(selectedBackgroundPreset);
    QFrame* backgroundPresetPreview = new QFrame(backgroundTab);
    backgroundPresetPreview->setFixedHeight(300);
    backgroundPresetPreview->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    backgroundPresetPreview->setStyleSheet(verticalColorBarStyleSheet(initialBackgroundColors.first,
                                                                      initialBackgroundColors.second));
    connect(backgroundPresetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dlg,
            [&selectedBackgroundPreset, backgroundPresetCombo, backgroundPresetPreview](int index) {
                selectedBackgroundPreset = backgroundPresetCombo->itemData(index).toInt();
                const QPair<QColor, QColor> colors = backgroundPresetColors(selectedBackgroundPreset);
                backgroundPresetPreview->setStyleSheet(verticalColorBarStyleSheet(colors.first, colors.second));
            });

    auto createSlamVoxelSpin = [&dlg](double value) {
        QDoubleSpinBox* spin = new QDoubleSpinBox(&dlg);
        spin->setRange(0.01, 5.0);
        spin->setDecimals(2);
        spin->setSingleStep(0.01);
        spin->setSuffix(QStringLiteral(" m"));
        spin->setValue(value);
        preparePreferenceSpinBox(spin);
        return spin;
    };
    QDoubleSpinBox* slamFilterSurfSpin = createSlamVoxelSpin(slamRuntimeConfig.filterSizeSurfM);
    QDoubleSpinBox* slamFilterMapSpin = createSlamVoxelSpin(slamRuntimeConfig.filterSizeMapM);
    QDoubleSpinBox* slamScanRateSpin = new QDoubleSpinBox(&dlg);
    slamScanRateSpin->setRange(1.0, 100.0);
    slamScanRateSpin->setDecimals(1);
    slamScanRateSpin->setSingleStep(1.0);
    slamScanRateSpin->setSuffix(QStringLiteral(" Hz"));
    slamScanRateSpin->setValue(slamRuntimeConfig.preprocessScanRateHz);
    preparePreferenceSpinBox(slamScanRateSpin);
    QSpinBox* slamFrameDurationSpin = new QSpinBox(&dlg);
    slamFrameDurationSpin->setRange(10, 1000);
    slamFrameDurationSpin->setSingleStep(10);
    slamFrameDurationSpin->setSuffix(QStringLiteral(" ms"));
    slamFrameDurationSpin->setValue(slamRuntimeConfig.inputFrameDurationMs);
    slamFrameDurationSpin->setFixedWidth(kPreferenceSpinBoxWidth);
    usePreferenceControlColumn(slamFrameDurationSpin);
    bool syncingSlamFrameControls = false;
    connect(slamScanRateSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [&syncingSlamFrameControls, slamFrameDurationSpin](double value) {
                if (syncingSlamFrameControls) {
                    return;
                }
                syncingSlamFrameControls = true;
                slamFrameDurationSpin->setValue(std::clamp(static_cast<int>(std::lround(1000.0 / value)), 10, 1000));
                syncingSlamFrameControls = false;
            });
    connect(slamFrameDurationSpin, QOverload<int>::of(&QSpinBox::valueChanged), &dlg,
            [&syncingSlamFrameControls, slamScanRateSpin](int value) {
                if (syncingSlamFrameControls) {
                    return;
                }
                syncingSlamFrameControls = true;
                slamScanRateSpin->setValue(1000.0 / double(value));
                syncingSlamFrameControls = false;
            });

    auto createSlamDoubleSpin = [&dlg](double value,
                                       double minValue,
                                       double maxValue,
                                       int decimals,
                                       double step,
                                       const QString& suffix) {
        QDoubleSpinBox* spin = new QDoubleSpinBox(&dlg);
        spin->setRange(minValue, maxValue);
        spin->setDecimals(decimals);
        spin->setSingleStep(step);
        spin->setSuffix(suffix);
        spin->setValue(value);
        preparePreferenceSpinBox(spin);
        return spin;
    };
    QDoubleSpinBox* slamGravityNormSpin =
        createSlamDoubleSpin(slamRuntimeConfig.gravityNorm, 1.0, 20.0, 4, 0.001, QStringLiteral(" m/s²"));
    QDoubleSpinBox* slamCubeSideLengthSpin =
        createSlamDoubleSpin(slamRuntimeConfig.cubeSideLengthM, 10.0, 10000.0, 1, 10.0, QStringLiteral(" m"));
    QDoubleSpinBox* slamDetRangeSpin =
        createSlamDoubleSpin(slamRuntimeConfig.detRangeM, 1.0, 2000.0, 1, 10.0, QStringLiteral(" m"));
    QDoubleSpinBox* slamFovDegreeSpin =
        createSlamDoubleSpin(slamRuntimeConfig.fovDegree, 1.0, 360.0, 1, 1.0, QStringLiteral(" °"));
    QDoubleSpinBox* slamBlindMinRangeSpin =
        createSlamDoubleSpin(slamRuntimeConfig.blindMinRangeM, 0.0, 100.0, 2, 0.1, QStringLiteral(" m"));
    QSpinBox* slamPointFilterNumSpin = new QSpinBox(&dlg);
    slamPointFilterNumSpin->setRange(1, 100);
    slamPointFilterNumSpin->setSingleStep(1);
    slamPointFilterNumSpin->setValue(slamRuntimeConfig.pointFilterNum);
    slamPointFilterNumSpin->setFixedWidth(kPreferenceSpinBoxWidth);
    usePreferenceControlColumn(slamPointFilterNumSpin);
    QSpinBox* slamMaxIterationsSpin = new QSpinBox(&dlg);
    slamMaxIterationsSpin->setRange(1, 100);
    slamMaxIterationsSpin->setSingleStep(1);
    slamMaxIterationsSpin->setValue(slamRuntimeConfig.maxIterations);
    slamMaxIterationsSpin->setFixedWidth(kPreferenceSpinBoxWidth);
    usePreferenceControlColumn(slamMaxIterationsSpin);
    QDoubleSpinBox* slamGyrCovSpin =
        createSlamDoubleSpin(slamRuntimeConfig.gyrCov, 0.000001, 100.0, 6, 0.01, QString());
    QDoubleSpinBox* slamAccCovSpin =
        createSlamDoubleSpin(slamRuntimeConfig.accCov, 0.000001, 100.0, 6, 0.01, QString());
    QDoubleSpinBox* slamBGyrCovSpin =
        createSlamDoubleSpin(slamRuntimeConfig.bGyrCov, 0.000001, 100.0, 6, 0.0001, QString());
    QDoubleSpinBox* slamBAccCovSpin =
        createSlamDoubleSpin(slamRuntimeConfig.bAccCov, 0.000001, 100.0, 6, 0.0001, QString());

    QWidget* slamTemplateRow = new QWidget(&dlg);
    usePreferenceControlColumn(slamTemplateRow, 360);
    QHBoxLayout* slamTemplateLayout = new QHBoxLayout(slamTemplateRow);
    slamTemplateLayout->setContentsMargins(0, 0, 0, 0);
    slamTemplateLayout->setSpacing(8);
    QComboBox* slamTemplateCombo = new QComboBox(slamTemplateRow);
    slamTemplateCombo->addItem(slamLidarTemplateDisplayName(SlamLidarTemplate::Mid360Mid360S),
                               int(SlamLidarTemplate::Mid360Mid360S));
    slamTemplateCombo->addItem(slamLidarTemplateDisplayName(SlamLidarTemplate::Avia),
                               int(SlamLidarTemplate::Avia));
    const int slamTemplateIndex = slamTemplateCombo->findData(int(slamRuntimeConfig.lidarTemplate));
    slamTemplateCombo->setCurrentIndex(slamTemplateIndex >= 0 ? slamTemplateIndex : 0);
    slamTemplateCombo->setFixedWidth(168);
    QPushButton* slamRestoreTemplateDefaultsButton = new QPushButton(QStringLiteral("恢复默认"), slamTemplateRow);
    slamTemplateLayout->addWidget(slamTemplateCombo);
    slamTemplateLayout->addWidget(slamRestoreTemplateDefaultsButton);
    slamTemplateLayout->addStretch();

    auto createSlamSwitch = [&dlg](bool checked) {
        SwitchCheckBox* check = new SwitchCheckBox(&dlg);
        check->setChecked(checked);
        usePreferenceControlColumn(check);
        return check;
    };
    SwitchCheckBox* slamExtrinsicEstimationCheck = createSlamSwitch(slamRuntimeConfig.extrinsicEstimationEnabled);
    auto createSlamExtrinsicSpin = [&dlg](double value,
                                          double minValue,
                                          double maxValue,
                                          int decimals,
                                          double step,
                                          const QString& suffix,
                                          bool trimTrailingZeros = false) {
        QDoubleSpinBox* spin = trimTrailingZeros
            ? static_cast<QDoubleSpinBox*>(new TrimmedDoubleSpinBox(&dlg))
            : new QDoubleSpinBox(&dlg);
        spin->setRange(minValue, maxValue);
        spin->setDecimals(decimals);
        spin->setSingleStep(step);
        spin->setValue(value);
        spin->setFixedWidth(92);
        if (!suffix.isEmpty()) {
            spin->setSuffix(suffix);
        }
        return spin;
    };
    std::array<QDoubleSpinBox*, 3> slamExtrinsicTSpins = {};
    QWidget* slamExtrinsicTRow = new QWidget(&dlg);
    usePreferenceControlColumn(slamExtrinsicTRow, 360);
    QHBoxLayout* slamExtrinsicTLayout = new QHBoxLayout(slamExtrinsicTRow);
    slamExtrinsicTLayout->setContentsMargins(0, 0, 0, 0);
    slamExtrinsicTLayout->setSpacing(6);
    const QStringList slamExtrinsicTLabels = {QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")};
    for (int i = 0; i < 3; ++i) {
        QLabel* label = new QLabel(slamExtrinsicTLabels.at(i), slamExtrinsicTRow);
        slamExtrinsicTSpins[static_cast<std::size_t>(i)] =
            createSlamExtrinsicSpin(slamRuntimeConfig.extrinsicT_L_I[i], -10.0, 10.0, 5, 0.001, QStringLiteral(" m"));
        slamExtrinsicTLayout->addWidget(label);
        slamExtrinsicTLayout->addWidget(slamExtrinsicTSpins[static_cast<std::size_t>(i)]);
    }
    slamExtrinsicTLayout->addStretch();
    std::array<QDoubleSpinBox*, 9> slamExtrinsicRSpins = {};
    QWidget* slamExtrinsicRGrid = new QWidget(&dlg);
    usePreferenceControlColumn(slamExtrinsicRGrid, 360);
    QGridLayout* slamExtrinsicRLayout = new QGridLayout(slamExtrinsicRGrid);
    slamExtrinsicRLayout->setContentsMargins(0, 0, 0, 0);
    slamExtrinsicRLayout->setHorizontalSpacing(6);
    slamExtrinsicRLayout->setVerticalSpacing(6);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const int index = row * 3 + col;
            slamExtrinsicRSpins[static_cast<std::size_t>(index)] =
                createSlamExtrinsicSpin(slamRuntimeConfig.extrinsicR_L_I[index], -1.0, 1.0, 6, 0.001, QString(), true);
            slamExtrinsicRLayout->addWidget(slamExtrinsicRSpins[static_cast<std::size_t>(index)], row, col);
        }
    }
    SwitchCheckBox* slamPublishWorldCheck = createSlamSwitch(slamRuntimeConfig.publishWorldFrameCloud);
    SwitchCheckBox* slamPublishDenseCheck = createSlamSwitch(slamRuntimeConfig.publishDenseFrameCloud);
    SwitchCheckBox* slamPublishBodyCheck = createSlamSwitch(slamRuntimeConfig.publishBodyFrameCloud);
    SwitchCheckBox* slamSaveMapCheck = createSlamSwitch(slamRuntimeConfig.saveMap);
    SwitchCheckBox* slamDynamicDetectionCheck = createSlamSwitch(slamRuntimeConfig.dynamicObjectDetectionEnabled);
    QWidget* slamWorldCurrentFrameColorRow = new QWidget(&dlg);
    usePreferenceControlColumn(slamWorldCurrentFrameColorRow);
    QHBoxLayout* slamWorldCurrentFrameColorLayout = new QHBoxLayout(slamWorldCurrentFrameColorRow);
    slamWorldCurrentFrameColorLayout->setContentsMargins(0, 0, 0, 0);
    slamWorldCurrentFrameColorLayout->setSpacing(8);
    QPushButton* slamWorldCurrentFrameColorPreview =
        createColorSwatchButton(slamWorldCurrentFrameColorRow, selectedSlamWorldCurrentFrameColor);
    slamWorldCurrentFrameColorLayout->addWidget(slamWorldCurrentFrameColorPreview);
    slamWorldCurrentFrameColorLayout->addStretch();
    connect(slamWorldCurrentFrameColorPreview,
            &QPushButton::clicked,
            &dlg,
            [&dlg, &selectedSlamWorldCurrentFrameColor, slamWorldCurrentFrameColorPreview]() {
                QColor color = QColorDialog::getColor(selectedSlamWorldCurrentFrameColor,
                                                       &dlg,
                                                       QStringLiteral("选择世界系当前帧点云颜色"));
                if (!color.isValid()) {
                    return;
                }
                selectedSlamWorldCurrentFrameColor = color;
                updateColorSwatchButton(slamWorldCurrentFrameColorPreview, selectedSlamWorldCurrentFrameColor);
            });
    QDoubleSpinBox* slamWorldCurrentFramePointSizeSpin =
        createSlamDoubleSpin(slamWorldCurrentFramePointSizePx, 1.0, 10.0, 1, 0.5, QStringLiteral(" px"));
    QWidget* slamBodyColorRow = new QWidget(&dlg);
    usePreferenceControlColumn(slamBodyColorRow);
    QHBoxLayout* slamBodyColorLayout = new QHBoxLayout(slamBodyColorRow);
    slamBodyColorLayout->setContentsMargins(0, 0, 0, 0);
    slamBodyColorLayout->setSpacing(8);
    QPushButton* slamBodyColorPreview = createColorSwatchButton(slamBodyColorRow, selectedSlamBodyFrameColor);
    slamBodyColorLayout->addWidget(slamBodyColorPreview);
    slamBodyColorLayout->addStretch();
    connect(slamBodyColorPreview, &QPushButton::clicked, &dlg, [&dlg, &selectedSlamBodyFrameColor, slamBodyColorPreview]() {
        QColor color = QColorDialog::getColor(selectedSlamBodyFrameColor, &dlg, QStringLiteral("选择 IMU 机体系点云颜色"));
        if (!color.isValid()) {
            return;
        }
        selectedSlamBodyFrameColor = color;
        updateColorSwatchButton(slamBodyColorPreview, selectedSlamBodyFrameColor);
    });
    QDoubleSpinBox* slamBodyFramePointSizeSpin =
        createSlamDoubleSpin(slamBodyFramePointSizePx, 1.0, 10.0, 1, 0.5, QStringLiteral(" px"));
    QWidget* slamTrajectoryColorRow = new QWidget(&dlg);
    usePreferenceControlColumn(slamTrajectoryColorRow);
    QHBoxLayout* slamTrajectoryColorLayout = new QHBoxLayout(slamTrajectoryColorRow);
    slamTrajectoryColorLayout->setContentsMargins(0, 0, 0, 0);
    slamTrajectoryColorLayout->setSpacing(8);
    QPushButton* slamTrajectoryColorPreview = createColorSwatchButton(slamTrajectoryColorRow, selectedSlamTrajectoryColor);
    slamTrajectoryColorLayout->addWidget(slamTrajectoryColorPreview);
    slamTrajectoryColorLayout->addStretch();
    connect(slamTrajectoryColorPreview,
            &QPushButton::clicked,
            &dlg,
            [&dlg, &selectedSlamTrajectoryColor, slamTrajectoryColorPreview]() {
                QColor color = QColorDialog::getColor(selectedSlamTrajectoryColor, &dlg, QStringLiteral("选择 SLAM 轨迹颜色"));
                if (!color.isValid()) {
                    return;
                }
                selectedSlamTrajectoryColor = color;
                updateColorSwatchButton(slamTrajectoryColorPreview, selectedSlamTrajectoryColor);
            });
    QDoubleSpinBox* slamTrajectoryLineWidthSpin =
        createSlamDoubleSpin(slamTrajectoryLineWidthPx, 1.0, 10.0, 1, 0.5, QStringLiteral(" px"));
    QDoubleSpinBox* slamPoseAxisLengthSpin =
        createSlamDoubleSpin(slamPoseAxisLengthM, 0.1, 10.0, 2, 0.1, QStringLiteral(" m"));
    QDoubleSpinBox* slamPoseAxisLineWidthSpin =
        createSlamDoubleSpin(slamPoseAxisLineWidthPx, 1.0, 10.0, 1, 0.5, QStringLiteral(" px"));
    auto syncSlamPublishControls = [slamPublishWorldCheck, slamPublishDenseCheck, slamPublishBodyCheck]() {
        const bool enabled = slamPublishWorldCheck->isChecked();
        slamPublishDenseCheck->setEnabled(enabled);
        slamPublishBodyCheck->setEnabled(enabled);
    };
    connect(slamPublishWorldCheck, &QCheckBox::toggled, &dlg,
            [syncSlamPublishControls](bool) { syncSlamPublishControls(); });
    syncSlamPublishControls();

    auto currentSlamTemplate = [slamTemplateCombo]() {
        return slamLidarTemplateFromInt(slamTemplateCombo->currentData().toInt());
    };
    auto setSlamRuntimeControls = [&](const SlamRuntimeConfig& runtimeDefaults) {
        syncingSlamFrameControls = true;
        slamScanRateSpin->setValue(runtimeDefaults.preprocessScanRateHz);
        slamFrameDurationSpin->setValue(runtimeDefaults.inputFrameDurationMs);
        syncingSlamFrameControls = false;
        slamFilterSurfSpin->setValue(runtimeDefaults.filterSizeSurfM);
        slamFilterMapSpin->setValue(runtimeDefaults.filterSizeMapM);
        slamGravityNormSpin->setValue(runtimeDefaults.gravityNorm);
        slamCubeSideLengthSpin->setValue(runtimeDefaults.cubeSideLengthM);
        slamDetRangeSpin->setValue(runtimeDefaults.detRangeM);
        slamFovDegreeSpin->setValue(runtimeDefaults.fovDegree);
        slamBlindMinRangeSpin->setValue(runtimeDefaults.blindMinRangeM);
        slamPointFilterNumSpin->setValue(runtimeDefaults.pointFilterNum);
        slamMaxIterationsSpin->setValue(runtimeDefaults.maxIterations);
        slamGyrCovSpin->setValue(runtimeDefaults.gyrCov);
        slamAccCovSpin->setValue(runtimeDefaults.accCov);
        slamBGyrCovSpin->setValue(runtimeDefaults.bGyrCov);
        slamBAccCovSpin->setValue(runtimeDefaults.bAccCov);
        slamExtrinsicEstimationCheck->setChecked(runtimeDefaults.extrinsicEstimationEnabled);
        for (int i = 0; i < 3; ++i) {
            slamExtrinsicTSpins[static_cast<std::size_t>(i)]->setValue(runtimeDefaults.extrinsicT_L_I[i]);
        }
        for (int i = 0; i < 9; ++i) {
            slamExtrinsicRSpins[static_cast<std::size_t>(i)]->setValue(runtimeDefaults.extrinsicR_L_I[i]);
        }
        slamPublishWorldCheck->setChecked(runtimeDefaults.publishWorldFrameCloud);
        slamPublishDenseCheck->setChecked(runtimeDefaults.publishDenseFrameCloud);
        slamPublishBodyCheck->setChecked(runtimeDefaults.publishBodyFrameCloud);
        slamSaveMapCheck->setChecked(runtimeDefaults.saveMap);
        slamDynamicDetectionCheck->setChecked(runtimeDefaults.dynamicObjectDetectionEnabled);
        syncSlamPublishControls();
    };
    auto slamRuntimeConfigFromControls = [&](SlamLidarTemplate lidarTemplate) {
        SlamRuntimeConfig config;
        applySlamLidarTemplateDefaults(config, lidarTemplate);
        config.filterSizeSurfM = slamFilterSurfSpin->value();
        config.filterSizeMapM = slamFilterMapSpin->value();
        config.gravityNorm = slamGravityNormSpin->value();
        config.preprocessScanRateHz = slamScanRateSpin->value();
        config.inputFrameDurationMs = slamFrameDurationSpin->value();
        config.cubeSideLengthM = slamCubeSideLengthSpin->value();
        config.detRangeM = slamDetRangeSpin->value();
        config.fovDegree = slamFovDegreeSpin->value();
        config.blindMinRangeM = slamBlindMinRangeSpin->value();
        config.pointFilterNum = slamPointFilterNumSpin->value();
        config.maxIterations = slamMaxIterationsSpin->value();
        config.gyrCov = slamGyrCovSpin->value();
        config.accCov = slamAccCovSpin->value();
        config.bGyrCov = slamBGyrCovSpin->value();
        config.bAccCov = slamBAccCovSpin->value();
        config.allowRosbagDriver2PointCloud2 = true;
        config.allowRosbagDriverPointCloud2SynthesizedTime = true;
        config.extrinsicEstimationEnabled = slamExtrinsicEstimationCheck->isChecked();
        for (int i = 0; i < 3; ++i) {
            config.extrinsicT_L_I[i] = slamExtrinsicTSpins[static_cast<std::size_t>(i)]->value();
        }
        for (int i = 0; i < 9; ++i) {
            config.extrinsicR_L_I[i] = slamExtrinsicRSpins[static_cast<std::size_t>(i)]->value();
        }
        config.publishWorldFrameCloud = slamPublishWorldCheck->isChecked();
        config.publishDenseFrameCloud = slamPublishDenseCheck->isChecked();
        config.publishBodyFrameCloud = slamPublishBodyCheck->isChecked();
        config.saveMap = slamSaveMapCheck->isChecked();
        config.dynamicObjectDetectionEnabled = slamDynamicDetectionCheck->isChecked();
        config.dynamicObjectClusterEnabled = false;
        return config;
    };
    QHash<int, SlamRuntimeConfig> editedSlamTemplateConfigs;
    auto loadSlamTemplateConfigForDialog = [&](SlamLidarTemplate lidarTemplate) {
        const int templateKey = int(lidarTemplate);
        if (editedSlamTemplateConfigs.contains(templateKey)) {
            return editedSlamTemplateConfigs.value(templateKey);
        }
        QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
        return loadSlamRuntimeConfigForTemplate(settings, QStringLiteral("slam/runtime"), lidarTemplate);
    };
    SlamLidarTemplate activeSlamTemplate = currentSlamTemplate();
    connect(slamTemplateCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dlg,
            [&editedSlamTemplateConfigs,
             &activeSlamTemplate,
             slamRuntimeConfigFromControls,
             loadSlamTemplateConfigForDialog,
             setSlamRuntimeControls,
             currentSlamTemplate](int) {
                editedSlamTemplateConfigs.insert(int(activeSlamTemplate),
                                                 slamRuntimeConfigFromControls(activeSlamTemplate));
                activeSlamTemplate = currentSlamTemplate();
                setSlamRuntimeControls(loadSlamTemplateConfigForDialog(activeSlamTemplate));
            });
    connect(slamRestoreTemplateDefaultsButton, &QPushButton::clicked, &dlg,
            [&editedSlamTemplateConfigs, setSlamRuntimeControls, currentSlamTemplate]() {
                SlamRuntimeConfig runtimeDefaults;
                applySlamLidarTemplateDefaults(runtimeDefaults, currentSlamTemplate());
                editedSlamTemplateConfigs.insert(int(runtimeDefaults.lidarTemplate), runtimeDefaults);
                setSlamRuntimeControls(runtimeDefaults);
            });

    QWidget* solidColorRow = new QWidget(&dlg);
    usePreferenceControlColumn(solidColorRow);
    QHBoxLayout* solidColorLayout = new QHBoxLayout(solidColorRow);
    solidColorLayout->setContentsMargins(0, 0, 0, 0);
    solidColorLayout->setSpacing(8);
    QPushButton* solidColorPreview = createColorSwatchButton(solidColorRow, selectedSolidColor);
    solidColorLayout->addWidget(solidColorPreview);
    solidColorLayout->addStretch();
    connect(solidColorPreview, &QPushButton::clicked, &dlg, [&dlg, &selectedSolidColor, solidColorPreview]() {
        QColor color = QColorDialog::getColor(selectedSolidColor, &dlg, "选择纯色点云颜色");
        if (!color.isValid()) {
            return;
        }
        selectedSolidColor = color;
        updateColorSwatchButton(solidColorPreview, selectedSolidColor);
    });

    QVector<QWidget*> lineColorRows;
    QVector<QWidget*> lineLegendRows;
    for (int i = 0; i < selectedLineColors.size(); ++i) {
        QWidget* row = new QWidget(&dlg);
        usePreferenceControlColumn(row);
        QHBoxLayout* rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->setSpacing(8);
        QPushButton* preview = createColorSwatchButton(row, selectedLineColors.at(i));
        rowLayout->addWidget(preview);
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
        connect(preview, &QPushButton::clicked, &dlg, [&dlg, &selectedLineColors, preview, legendPreview, legendColorLabel, i]() {
            QColor color = QColorDialog::getColor(selectedLineColors.at(i), &dlg, QString("选择 Line %1 点云颜色").arg(i));
            if (!color.isValid()) {
                return;
            }
            selectedLineColors[i] = color;
            updateColorSwatchButton(preview, color);
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

    QWidget* buttonRow = new QWidget(settingsContent);
    QHBoxLayout* buttonRowLayout = new QHBoxLayout(buttonRow);
    buttonRowLayout->setContentsMargins(0, 0, 0, 0);
    buttonRowLayout->setSpacing(8);
    QPushButton* okButton = new QPushButton(QStringLiteral("确定"), buttonRow);
    QPushButton* cancelButton = new QPushButton(QStringLiteral("取消"), buttonRow);
    buttonRowLayout->addStretch();
    buttonRowLayout->addWidget(okButton);
    buttonRowLayout->addWidget(cancelButton);
    connect(okButton, &QPushButton::clicked, &dlg, &QDialog::accept);
    connect(cancelButton, &QPushButton::clicked, &dlg, &QDialog::reject);

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
    addPreferenceRow(gridSection, "类型", "选择方形网格、同心圆网格或两者同时显示。", gridTypeCombo);
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

    addPreferenceSectionTitle(colorPageLayout, "反射率模式");
    QFrame* reflectivityColorSection = createPreferenceSection(colorTab);
    addPreferenceRow(reflectivityColorSection,
                     "色标",
                     "设置反射率着色模式使用的颜色映射。",
                     reflectivityScaleRow);
    colorPageLayout->addWidget(reflectivityColorSection);
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

    addPreferenceSectionTitle(backgroundLayout, "点云背景");
    QFrame* backgroundSection = createPreferenceSection(backgroundTab);
    addPreferenceRow(backgroundSection, "背景方案", "设置 OpenGL 点云可视化区域的顶部和底部背景颜色。", backgroundPresetCombo);
    backgroundLayout->addWidget(backgroundSection);
    backgroundLayout->addSpacing(12);
    backgroundLayout->addWidget(backgroundPresetPreview);
    backgroundLayout->addStretch();

    addPreferenceSectionTitle(slamLayout, "LiDAR 模板");
    QFrame* slamTemplateSection = createPreferenceSection(slamTab);
    addPreferenceRow(slamTemplateSection,
                     "模板",
                     "选择 LiDAR 类型预设，用于快速填充探测距离、视场角、近距离盲区和 LiDAR-IMU 外参等参数",
                     slamTemplateRow);
    slamLayout->addWidget(slamTemplateSection);

    QTabWidget* slamSettingsTabs = new QTabWidget(slamTab);
    slamSettingsTabs->setObjectName(QStringLiteral("SlamSettingsTabs"));
    slamSettingsTabs->setDocumentMode(true);
    slamSettingsTabs->tabBar()->setDrawBase(false);
    slamSettingsTabs->tabBar()->setExpanding(false);
    slamSettingsTabs->tabBar()->setElideMode(Qt::ElideNone);
    slamSettingsTabs->tabBar()->setUsesScrollButtons(false);
    slamSettingsTabs->setStyleSheet(QStringLiteral(
        "QTabWidget#SlamSettingsTabs::pane { border: none; }"
        "QTabWidget#SlamSettingsTabs QTabBar { border: none; background: transparent; }"
        "QTabWidget#SlamSettingsTabs QTabBar::base { border: none; background: transparent; }"
        "QTabWidget#SlamSettingsTabs QTabBar::tab {"
        " border: none;"
        " border-bottom: 2px solid transparent;"
        " background: transparent;"
        " padding: 5px 8px;"
        " margin-right: 10px;"
        "}"
        "QTabWidget#SlamSettingsTabs QTabBar::tab:selected {"
        " color: #2f8cff;"
        " border-bottom-color: #2f8cff;"
        " font-weight: 600;"
        "}"
        "QTabWidget#SlamSettingsTabs QTabBar::tab:!selected {"
        " color: palette(window-text);"
        "}"));
    auto createSlamSettingsTab = [slamSettingsTabs](const QString& title) {
        QWidget* tab = new QWidget(slamSettingsTabs);
        QVBoxLayout* tabLayout = new QVBoxLayout(tab);
        tabLayout->setContentsMargins(0, 12, 0, 0);
        tabLayout->setSpacing(12);
        slamSettingsTabs->addTab(tab, title);
        return tab;
    };

    QWidget* slamBackendTab = createSlamSettingsTab(QStringLiteral("后端参数"));
    QWidget* slamImuNoiseTab = createSlamSettingsTab(QStringLiteral("IMU 噪声"));
    QWidget* slamExtrinsicTab = createSlamSettingsTab(QStringLiteral("LiDAR-IMU 外参"));
    QWidget* slamPublishTab = createSlamSettingsTab(QStringLiteral("点云输出"));
    QWidget* slamDynamicTab = createSlamSettingsTab(QStringLiteral("动态检测"));
    QWidget* slamVisualTab = createSlamSettingsTab(QStringLiteral("显示样式"));

    QVBoxLayout* slamBackendTabLayout = qobject_cast<QVBoxLayout*>(slamBackendTab->layout());
    QVBoxLayout* slamImuNoiseTabLayout = qobject_cast<QVBoxLayout*>(slamImuNoiseTab->layout());
    QVBoxLayout* slamExtrinsicTabLayout = qobject_cast<QVBoxLayout*>(slamExtrinsicTab->layout());
    QVBoxLayout* slamPublishTabLayout = qobject_cast<QVBoxLayout*>(slamPublishTab->layout());
    QVBoxLayout* slamDynamicTabLayout = qobject_cast<QVBoxLayout*>(slamDynamicTab->layout());
    QVBoxLayout* slamVisualTabLayout = qobject_cast<QVBoxLayout*>(slamVisualTab->layout());

    QFrame* slamBackendSection = createPreferenceSection(slamBackendTab);
    addPreferenceRow(slamBackendSection,
                     "扫描频率",
                     "preprocessScanRateHz，输入点云的帧率，用于推导聚帧周期并影响时间间隔计算",
                     slamScanRateSpin);
    addPreferenceRow(slamBackendSection,
                     "聚帧周期",
                     "inputFrameDurationMs，单个 SLAM 输入帧的时间长度，用于实时数据和 PCAP 数据按时间切分点云帧",
                     slamFrameDurationSpin);
    addPreferenceRow(slamBackendSection,
                     "近距离盲区",
                     "blindMinRangeM，滤除 LiDAR 坐标系下距离过近的点，减小近场噪点对匹配的影响",
                     slamBlindMinRangeSpin);
    addPreferenceRow(slamBackendSection,
                     "点过滤步长",
                     "pointFilterNum，按点序间隔抽样输入点云，值越大输入点越少、速度更快但约束变弱",
                     slamPointFilterNumSpin);
    addPreferenceRow(slamBackendSection,
                     "重力加速度",
                     "gravityNorm，IMU 初始化使用的重力模长，影响姿态初始化和加速度归一化",
                     slamGravityNormSpin);
    addPreferenceRow(slamBackendSection,
                     "局部地图边长",
                     "cubeSideLengthM，局部地图立方体边长，决定滑动局部地图的空间范围和内存占用",
                     slamCubeSideLengthSpin);
    addPreferenceRow(slamBackendSection,
                     "探测距离",
                     "detRangeM，参与局部地图维护和匹配的有效探测距离，过小会减少远处稳定结构约束",
                     slamDetRangeSpin);
    addPreferenceRow(slamBackendSection,
                     "水平视场角",
                     "fovDegree，LiDAR 水平有效视场角，用于限制参与匹配的地图点范围",
                     slamFovDegreeSpin);
    addPreferenceRow(slamBackendSection,
                     "最大迭代次数",
                     "maxIterations，每帧状态更新的最大迭代次数，值越大收敛更充分但耗时增加",
                     slamMaxIterationsSpin);
    addPreferenceRow(slamBackendSection,
                     "表面滤波体素",
                     "filterSizeSurfM，输入点云降采样体素尺寸，值越大点数越少、速度更快但细节减少",
                     slamFilterSurfSpin);
    addPreferenceRow(slamBackendSection,
                     "地图滤波体素",
                     "filterSizeMapM，地图点云降采样体素尺寸，值越大地图越稀疏、内存更低但匹配精度可能下降",
                     slamFilterMapSpin);
    slamBackendTabLayout->addWidget(slamBackendSection);
    slamBackendTabLayout->addStretch();

    QFrame* slamImuNoiseSection = createPreferenceSection(slamImuNoiseTab);
    addPreferenceRow(slamImuNoiseSection,
                     "陀螺仪测量噪声",
                     "gyrCov，陀螺仪测量噪声协方差，值越大表示越不信任角速度测量",
                     slamGyrCovSpin);
    addPreferenceRow(slamImuNoiseSection,
                     "加速度计测量噪声",
                     "accCov，加速度计测量噪声协方差，值越大表示越不信任线加速度测量",
                     slamAccCovSpin);
    addPreferenceRow(slamImuNoiseSection,
                     "陀螺仪零偏噪声",
                     "bGyrCov，陀螺仪零偏随机游走噪声，值越大表示零偏变化越快",
                     slamBGyrCovSpin);
    addPreferenceRow(slamImuNoiseSection,
                     "加速度计零偏噪声",
                     "bAccCov，加速度计零偏随机游走噪声，值越大表示零偏变化越快",
                     slamBAccCovSpin);
    slamImuNoiseTabLayout->addWidget(slamImuNoiseSection);
    slamImuNoiseTabLayout->addStretch();

    QFrame* slamExtrinsicSection = createPreferenceSection(slamExtrinsicTab);
    addPreferenceRow(slamExtrinsicSection,
                     "在线估计外参",
                     "extrinsicEstimationEnabled，允许运行时估计 LiDAR-IMU 外参，开启后可补偿外参误差但稳定性更依赖数据质量",
                     slamExtrinsicEstimationCheck);
    addPreferenceRow(slamExtrinsicSection,
                     "外参平移 T",
                     "extrinsicT_L_I，LiDAR 到 IMU 的平移向量，单位 m，影响点云去畸变和位姿估计坐标关系",
                     slamExtrinsicTRow);
    addPreferenceRow(slamExtrinsicSection,
                     "外参旋转 R",
                     "extrinsicR_L_I，LiDAR 到 IMU 的旋转矩阵，按行优先填写 3x3，影响点云与 IMU 姿态对齐",
                     slamExtrinsicRGrid);
    slamExtrinsicTabLayout->addWidget(slamExtrinsicSection);
    slamExtrinsicTabLayout->addStretch();

    QFrame* slamPublishSection = createPreferenceSection(slamPublishTab);
    addPreferenceRow(slamPublishSection,
                     "输出世界系点云",
                     "publishWorldFrameCloud，输出世界坐标系点云，用于 SLAM 视图显示历史建图结果",
                     slamPublishWorldCheck);
    addPreferenceRow(slamPublishSection,
                     "稠密世界系点云",
                     "publishDenseFrameCloud，世界系点云使用去畸变后的稠密点云，开启后细节更多但点数和内存占用增加",
                     slamPublishDenseCheck);
    addPreferenceRow(slamPublishSection,
                     "输出机体系点云",
                     "publishBodyFrameCloud，输出 IMU 机体系下的当前帧点云，用于检查去畸变前后的当前帧形态",
                     slamPublishBodyCheck);
    addPreferenceRow(slamPublishSection,
                     "保存完整全局地图",
                     "saveMap，累计完整世界系点云地图用于导出，开启后可保存更完整地图但内存占用增加",
                     slamSaveMapCheck);
    slamPublishTabLayout->addWidget(slamPublishSection);
    slamPublishTabLayout->addStretch();

    QFrame* slamDynamicSection = createPreferenceSection(slamDynamicTab);
    addPreferenceRow(slamDynamicSection,
                     "启用动态检测",
                     "dynamicObjectDetectionEnabled，启用 M-detector 无聚类核心版，对去畸变当前帧生成动态点 overlay 和统计信息",
                     slamDynamicDetectionCheck);
    slamDynamicTabLayout->addWidget(slamDynamicSection);
    slamDynamicTabLayout->addStretch();

    QFrame* slamVisualSection = createPreferenceSection(slamVisualTab);
    addPreferenceRow(slamVisualSection,
                     "世界系当前帧颜色",
                     "slamWorldCurrentFrameColor，世界坐标系当前帧点云颜色，用于区分最新输入帧和历史地图",
                     slamWorldCurrentFrameColorRow);
    addPreferenceRow(slamVisualSection,
                     "世界系当前帧点大小",
                     "slamWorldCurrentFramePointSizePx，世界坐标系当前帧点大小，影响当前帧点云的显示醒目程度",
                     slamWorldCurrentFramePointSizeSpin);
    addPreferenceRow(slamVisualSection,
                     "机体系当前帧颜色",
                     "slamBodyFrameColor，IMU 机体系当前帧点云颜色，用于区分机体系点云和世界系点云",
                     slamBodyColorRow);
    addPreferenceRow(slamVisualSection,
                     "机体系当前帧点大小",
                     "slamBodyFramePointSizePx，IMU 机体系当前帧点大小，影响机体系当前帧点云显示醒目程度",
                     slamBodyFramePointSizeSpin);
    addPreferenceRow(slamVisualSection,
                     "轨迹颜色",
                     "slamTrajectoryColor，位姿轨迹线颜色，用于区分轨迹和点云",
                     slamTrajectoryColorRow);
    addPreferenceRow(slamVisualSection,
                     "轨迹线宽",
                     "slamTrajectoryLineWidthPx，位姿轨迹线宽，值越大轨迹显示越醒目",
                     slamTrajectoryLineWidthSpin);
    addPreferenceRow(slamVisualSection,
                     "位姿坐标轴长度",
                     "slamPoseAxisLengthM，当前位置姿态坐标轴长度，影响姿态方向显示范围",
                     slamPoseAxisLengthSpin);
    addPreferenceRow(slamVisualSection,
                     "位姿坐标轴线宽",
                     "slamPoseAxisLineWidthPx，当前位置姿态坐标轴线宽，值越大坐标轴显示越醒目",
                     slamPoseAxisLineWidthSpin);
    slamVisualTabLayout->addWidget(slamVisualSection);
    slamVisualTabLayout->addStretch();

    slamLayout->addWidget(slamSettingsTabs);
    slamLayout->addStretch();

    const QStringList navigationNames = {"主题", "连接", "网格", "图例", "着色", "背景", "SLAM"};
    const QStringList navigationIcons = {
        ":/icons/settings_theme.svg",
        ":/icons/settings_connection.svg",
        ":/icons/settings_grid.svg",
        ":/icons/settings_legend.svg",
        ":/icons/settings_color.svg",
        ":/icons/settings_background.svg",
        ":/icons/settings.svg"
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
    settingsContentLayout->addWidget(buttonRow);

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
    config.type = static_cast<PointCloudView::GridConfig::Type>(gridTypeCombo->currentData().toInt());
    distanceLegendMin = float(distanceMinSpin->value());
    distanceLegendMax = float(distanceMaxSpin->value());
    elevationLegendMin = float(elevationMinSpin->value());
    elevationLegendMax = float(elevationMaxSpin->value());
    reflectivityColorScale = selectedReflectivityColorScale;
    solidColor = selectedSolidColor;
    lineColors = selectedLineColors;
    pointCloudBackgroundPreset = selectedBackgroundPreset;
    themeMode = themeGroup->checkedId();
    const bool previousAutoConfigHostIpEnabled = autoConfigHostIpEnabled;
    const double previousSlamFilterSurfM = slamRuntimeConfig.filterSizeSurfM;
    const double previousSlamFilterMapM = slamRuntimeConfig.filterSizeMapM;
    const SlamLidarTemplate previousSlamLidarTemplate = slamRuntimeConfig.lidarTemplate;
    const double previousSlamGravityNorm = slamRuntimeConfig.gravityNorm;
    const double previousSlamScanRateHz = slamRuntimeConfig.preprocessScanRateHz;
    const int previousSlamFrameDurationMs = slamRuntimeConfig.inputFrameDurationMs;
    const double previousSlamCubeSideLengthM = slamRuntimeConfig.cubeSideLengthM;
    const double previousSlamDetRangeM = slamRuntimeConfig.detRangeM;
    const double previousSlamFovDegree = slamRuntimeConfig.fovDegree;
    const double previousSlamBlindMinRangeM = slamRuntimeConfig.blindMinRangeM;
    const int previousSlamPointFilterNum = slamRuntimeConfig.pointFilterNum;
    const int previousSlamMaxIterations = slamRuntimeConfig.maxIterations;
    const double previousSlamGyrCov = slamRuntimeConfig.gyrCov;
    const double previousSlamAccCov = slamRuntimeConfig.accCov;
    const double previousSlamBGyrCov = slamRuntimeConfig.bGyrCov;
    const double previousSlamBAccCov = slamRuntimeConfig.bAccCov;
    const bool previousSlamPublishWorld = slamRuntimeConfig.publishWorldFrameCloud;
    const bool previousSlamPublishDense = slamRuntimeConfig.publishDenseFrameCloud;
    const bool previousSlamPublishBody = slamRuntimeConfig.publishBodyFrameCloud;
    const bool previousSlamSaveMap = slamRuntimeConfig.saveMap;
    const bool previousSlamDynamicDetection = slamRuntimeConfig.dynamicObjectDetectionEnabled;
    const bool previousSlamExtrinsicEstimationEnabled = slamRuntimeConfig.extrinsicEstimationEnabled;
    const std::array<double, 3> previousSlamExtrinsicT = {
        slamRuntimeConfig.extrinsicT_L_I[0],
        slamRuntimeConfig.extrinsicT_L_I[1],
        slamRuntimeConfig.extrinsicT_L_I[2]
    };
    const std::array<double, 9> previousSlamExtrinsicR = {
        slamRuntimeConfig.extrinsicR_L_I[0],
        slamRuntimeConfig.extrinsicR_L_I[1],
        slamRuntimeConfig.extrinsicR_L_I[2],
        slamRuntimeConfig.extrinsicR_L_I[3],
        slamRuntimeConfig.extrinsicR_L_I[4],
        slamRuntimeConfig.extrinsicR_L_I[5],
        slamRuntimeConfig.extrinsicR_L_I[6],
        slamRuntimeConfig.extrinsicR_L_I[7],
        slamRuntimeConfig.extrinsicR_L_I[8]
    };
    const QColor previousSlamWorldCurrentFrameColor = slamWorldCurrentFrameColor;
    const QColor previousSlamBodyFrameColor = slamBodyFrameColor;
    const QColor previousSlamTrajectoryColor = slamTrajectoryColor;
    const float previousSlamWorldCurrentFramePointSizePx = slamWorldCurrentFramePointSizePx;
    const float previousSlamBodyFramePointSizePx = slamBodyFramePointSizePx;
    const float previousSlamTrajectoryLineWidthPx = slamTrajectoryLineWidthPx;
    const float previousSlamPoseAxisLengthM = slamPoseAxisLengthM;
    const float previousSlamPoseAxisLineWidthPx = slamPoseAxisLineWidthPx;
    autoConfigHostIpEnabled = autoConfigHostIpPreferenceCheck->isChecked();
    slamRuntimeConfig.filterSizeSurfM = slamFilterSurfSpin->value();
    slamRuntimeConfig.filterSizeMapM = slamFilterMapSpin->value();
    slamRuntimeConfig.lidarTemplate = currentSlamTemplate();
    slamRuntimeConfig.gravityNorm = slamGravityNormSpin->value();
    slamRuntimeConfig.preprocessScanRateHz = slamScanRateSpin->value();
    slamRuntimeConfig.inputFrameDurationMs = slamFrameDurationSpin->value();
    slamRuntimeConfig.cubeSideLengthM = slamCubeSideLengthSpin->value();
    slamRuntimeConfig.detRangeM = slamDetRangeSpin->value();
    slamRuntimeConfig.fovDegree = slamFovDegreeSpin->value();
    slamRuntimeConfig.blindMinRangeM = slamBlindMinRangeSpin->value();
    slamRuntimeConfig.pointFilterNum = slamPointFilterNumSpin->value();
    slamRuntimeConfig.maxIterations = slamMaxIterationsSpin->value();
    slamRuntimeConfig.gyrCov = slamGyrCovSpin->value();
    slamRuntimeConfig.accCov = slamAccCovSpin->value();
    slamRuntimeConfig.bGyrCov = slamBGyrCovSpin->value();
    slamRuntimeConfig.bAccCov = slamBAccCovSpin->value();
    slamRuntimeConfig.allowRosbagDriver2PointCloud2 = true;
    slamRuntimeConfig.allowRosbagDriverPointCloud2SynthesizedTime = true;
    slamRuntimeConfig.extrinsicEstimationEnabled = slamExtrinsicEstimationCheck->isChecked();
    for (int i = 0; i < 3; ++i) {
        slamRuntimeConfig.extrinsicT_L_I[i] = slamExtrinsicTSpins[static_cast<std::size_t>(i)]->value();
    }
    for (int i = 0; i < 9; ++i) {
        slamRuntimeConfig.extrinsicR_L_I[i] = slamExtrinsicRSpins[static_cast<std::size_t>(i)]->value();
    }
    slamRuntimeConfig.publishWorldFrameCloud = slamPublishWorldCheck->isChecked();
    slamRuntimeConfig.publishDenseFrameCloud = slamPublishDenseCheck->isChecked();
    slamRuntimeConfig.publishBodyFrameCloud = slamPublishBodyCheck->isChecked();
    slamRuntimeConfig.saveMap = slamSaveMapCheck->isChecked();
    slamRuntimeConfig.dynamicObjectDetectionEnabled = slamDynamicDetectionCheck->isChecked();
    slamRuntimeConfig.dynamicObjectClusterEnabled = false;
    slamWorldCurrentFrameColor = selectedSlamWorldCurrentFrameColor;
    slamBodyFrameColor = selectedSlamBodyFrameColor;
    slamTrajectoryColor = selectedSlamTrajectoryColor;
    slamWorldCurrentFramePointSizePx = static_cast<float>(slamWorldCurrentFramePointSizeSpin->value());
    slamBodyFramePointSizePx = static_cast<float>(slamBodyFramePointSizeSpin->value());
    slamTrajectoryLineWidthPx = static_cast<float>(slamTrajectoryLineWidthSpin->value());
    slamPoseAxisLengthM = static_cast<float>(slamPoseAxisLengthSpin->value());
    slamPoseAxisLineWidthPx = static_cast<float>(slamPoseAxisLineWidthSpin->value());
    editedSlamTemplateConfigs.insert(int(slamRuntimeConfig.lidarTemplate), slamRuntimeConfig);
    {
        QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
        for (auto it = editedSlamTemplateConfigs.cbegin(); it != editedSlamTemplateConfigs.cend(); ++it) {
            saveSlamRuntimeConfigForTemplate(settings, it.value(), QStringLiteral("slam/runtime"));
        }
    }
    if (slamUiBridge) {
        slamUiBridge->setWorldFrameColor(slamWorldCurrentFrameColor);
        slamUiBridge->setBodyFrameColor(slamBodyFrameColor);
        slamUiBridge->setTrajectoryColor(slamTrajectoryColor);
        slamUiBridge->setWorldFramePointSize(slamWorldCurrentFramePointSizePx);
        slamUiBridge->setBodyFramePointSize(slamBodyFramePointSizePx);
        slamUiBridge->setTrajectoryLineWidth(slamTrajectoryLineWidthPx);
        slamUiBridge->setPoseAxisLength(slamPoseAxisLengthM);
        slamUiBridge->setPoseAxisLineWidth(slamPoseAxisLineWidthPx);
    }
    rebuildSlamInfoPanel();
    syncSlamRenderLayerVisibility();
    syncSlamTemplateControl();
    refreshSlamWorldPointCloud();

    syncReflectivityColorScaleControls();
    pointCloudView->setGridConfig(config);
    applyPointCloudBackground();
    applyUiTheme();
    recolorPointCloudViews();
    if (playbackState.active && playbackState.frame >= 0) {
        showLvx2PlaybackFrame(playbackState.frame);
    }
    saveViewPreferences();
    if (autoConfigHostIpEnabled != previousAutoConfigHostIpEnabled) {
        logMessage(QString("自动修改主机IP: %1").arg(autoConfigHostIpEnabled ? "已启用" : "已关闭"));
    }
    const bool slamExtrinsicChanged = [&]() {
        if (slamRuntimeConfig.extrinsicEstimationEnabled != previousSlamExtrinsicEstimationEnabled) {
            return true;
        }
        for (int i = 0; i < 3; ++i) {
            if (slamRuntimeConfig.extrinsicT_L_I[i] != previousSlamExtrinsicT[static_cast<std::size_t>(i)]) {
                return true;
            }
        }
        for (int i = 0; i < 9; ++i) {
            if (slamRuntimeConfig.extrinsicR_L_I[i] != previousSlamExtrinsicR[static_cast<std::size_t>(i)]) {
                return true;
            }
        }
        return false;
    }();
    if (slamRuntimeConfig.filterSizeSurfM != previousSlamFilterSurfM ||
        slamRuntimeConfig.filterSizeMapM != previousSlamFilterMapM ||
        slamRuntimeConfig.lidarTemplate != previousSlamLidarTemplate ||
        slamRuntimeConfig.gravityNorm != previousSlamGravityNorm ||
        slamRuntimeConfig.preprocessScanRateHz != previousSlamScanRateHz ||
        slamRuntimeConfig.inputFrameDurationMs != previousSlamFrameDurationMs ||
        slamRuntimeConfig.cubeSideLengthM != previousSlamCubeSideLengthM ||
        slamRuntimeConfig.detRangeM != previousSlamDetRangeM ||
        slamRuntimeConfig.fovDegree != previousSlamFovDegree ||
        slamRuntimeConfig.blindMinRangeM != previousSlamBlindMinRangeM ||
        slamRuntimeConfig.pointFilterNum != previousSlamPointFilterNum ||
        slamRuntimeConfig.maxIterations != previousSlamMaxIterations ||
        slamRuntimeConfig.gyrCov != previousSlamGyrCov ||
        slamRuntimeConfig.accCov != previousSlamAccCov ||
        slamRuntimeConfig.bGyrCov != previousSlamBGyrCov ||
        slamRuntimeConfig.bAccCov != previousSlamBAccCov ||
        slamExtrinsicChanged ||
        slamRuntimeConfig.publishWorldFrameCloud != previousSlamPublishWorld ||
        slamRuntimeConfig.publishDenseFrameCloud != previousSlamPublishDense ||
        slamRuntimeConfig.publishBodyFrameCloud != previousSlamPublishBody ||
        slamRuntimeConfig.saveMap != previousSlamSaveMap ||
        slamRuntimeConfig.dynamicObjectDetectionEnabled != previousSlamDynamicDetection ||
        slamWorldCurrentFrameColor != previousSlamWorldCurrentFrameColor ||
        slamBodyFrameColor != previousSlamBodyFrameColor ||
        slamTrajectoryColor != previousSlamTrajectoryColor ||
        slamWorldCurrentFramePointSizePx != previousSlamWorldCurrentFramePointSizePx ||
        slamBodyFramePointSizePx != previousSlamBodyFramePointSizePx ||
        slamTrajectoryLineWidthPx != previousSlamTrajectoryLineWidthPx ||
        slamPoseAxisLengthM != previousSlamPoseAxisLengthM ||
        slamPoseAxisLineWidthPx != previousSlamPoseAxisLineWidthPx) {
        liveSlamSource.setFrameDurationMs(slamRuntimeConfig.inputFrameDurationMs);
        logMessage(QStringLiteral("[SLAM] 设置已更新"));
    }
}

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
#include <QCloseEvent>
#include <QColorDialog>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFont>
#include <QFrame>
#include <QGridLayout>
#include <QGuiApplication>
#include <QHash>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
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
#include <QStyleOption>
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

class CurrentPageHeightTabWidget : public QTabWidget
{
public:
    explicit CurrentPageHeightTabWidget(QWidget* parent = nullptr)
        : QTabWidget(parent)
    {
        connect(this, &QTabWidget::currentChanged, this, [this]() {
            QTimer::singleShot(0, this, [this]() {
                syncCurrentPageHeight();
            });
        });
    }

    void syncCurrentPageHeight()
    {
        setMaximumHeight(QWIDGETSIZE_MAX);
        updateGeometry();
        setMaximumHeight(sizeHint().height());
        updateGeometry();
    }

    QSize sizeHint() const override
    {
        QWidget* page = currentWidget();
        if (!page) {
            return QTabWidget::sizeHint();
        }

        QStyleOptionTabWidgetFrame option;
        initStyleOption(&option);
        const QSize contentSize = page->sizeHint().expandedTo(tabBar()->sizeHint());
        return style()->sizeFromContents(QStyle::CT_TabWidget, &option, contentSize, this);
    }

    QSize minimumSizeHint() const override
    {
        QWidget* page = currentWidget();
        if (!page) {
            return QTabWidget::minimumSizeHint();
        }

        QStyleOptionTabWidgetFrame option;
        initStyleOption(&option);
        const QSize contentSize = page->minimumSizeHint().expandedTo(tabBar()->minimumSizeHint());
        return style()->sizeFromContents(QStyle::CT_TabWidget, &option, contentSize, this);
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

void refreshParameterOptionButtonThemes()
{
    const bool darkTheme = QApplication::palette().color(QPalette::Window).lightness() < 128;
    for (QWidget* widget : QApplication::allWidgets()) {
        if (!widget->property("parameterOptionButton").toBool()) {
            continue;
        }
        widget->setProperty("parameterOptionButtonTheme", darkTheme ? QStringLiteral("dark") : QStringLiteral("light"));
        widget->style()->unpolish(widget);
        widget->style()->polish(widget);
        widget->update();
    }
}

void refreshApplicationWidgetThemes()
{
    for (QWidget* widget : QApplication::allWidgets()) {
        widget->style()->unpolish(widget);
        widget->style()->polish(widget);
        widget->update();
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

void addPreferenceRow(QFrame* section,
                      const QString& title,
                      const QString& description,
                      QWidget* control,
                      const QString& toolTip = QString())
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
    if (!toolTip.isEmpty()) {
        row->setToolTip(toolTip);
    }
    QHBoxLayout* rowLayout = new QHBoxLayout(row);
    rowLayout->setContentsMargins(0, 10, 0, 10);
    rowLayout->setSpacing(18);

    QVBoxLayout* textLayout = new QVBoxLayout();
    textLayout->setContentsMargins(0, 0, 0, 0);
    textLayout->setSpacing(4);

    QLabel* titleLabel = new QLabel(title, row);
    if (!toolTip.isEmpty()) {
        titleLabel->setToolTip(toolTip);
    }
    setPreferenceFont(titleLabel, 9, QFont::Medium);
    textLayout->addWidget(titleLabel);
    if (!description.isEmpty()) {
        QLabel* descriptionLabel = createPreferenceDescription(description, row);
        if (!toolTip.isEmpty()) {
            descriptionLabel->setToolTip(toolTip);
        }
        textLayout->addWidget(descriptionLabel);
    }

    if (!toolTip.isEmpty()) {
        control->setToolTip(toolTip);
        const QList<QWidget*> controlChildren = control->findChildren<QWidget*>();
        for (QWidget* child : controlChildren) {
            child->setToolTip(toolTip);
        }
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
    QSettings themeSettings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    themeMode = themeSettings.value(QStringLiteral("theme/mode"), themeMode).toInt();
    if (themeMode < ThemeFollowSystem || themeMode > ThemeDark) {
        themeMode = ThemeFollowSystem;
    }
    applyUiTheme();
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
    slamWorkerCancel.store(true);
    slamWorkerPaused.store(false);
    if (slamWorker.joinable()) {
        slamWorker.join();
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

void LivoxViewerWindow::closeEvent(QCloseEvent* event)
{
    if (!shutting_down) {
        shutting_down = true;
        pointCloudCallbackEnabled = false;
        if (renderTimer) {
            renderTimer->stop();
        }
        if (paramQueryTimer) {
            paramQueryTimer->stop();
        }
        if (captureState.timer) {
            captureState.timer->stop();
        }
        if (playbackState.timer) {
            playbackState.timer->stop();
        }
        stopLidarDiscovery();
        shutdownLivoxSdk();
        shutting_down = true;
    }
    QMainWindow::closeEvent(event);
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

    static bool comboBoxPopupStyleInstalled = false;
    if (!comboBoxPopupStyleInstalled) {
        app->setStyle(new ComboBoxPopupStyle(QStyleFactory::create("Fusion")));
        comboBoxPopupStyleInstalled = true;
    }
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
    refreshApplicationWidgetThemes();
    ThemeIconUtils::refreshObject(this);
    refreshParameterOptionButtonThemes();
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
    syncSlamTemplateControl();
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
    slamDynamicObjectColor = settings.value(
        QStringLiteral("slam/dynamicObjectColor"), slamDynamicObjectColor).value<QColor>();
    if (!slamDynamicObjectColor.isValid()) {
        slamDynamicObjectColor = QColor(239, 41, 41);
    }
    slamDynamicObjectPointSizePx = std::clamp(
        settings.value(QStringLiteral("slam/dynamicObjectPointSizePx"),
                       slamDynamicObjectPointSizePx).toFloat(),
        1.0f,
        10.0f);
    auto loadSlamColor = [&settings](const QString& key, const QColor& defaultColor) {
        const QColor color = settings.value(key, defaultColor).value<QColor>();
        return color.isValid() ? color : defaultColor;
    };
    auto loadSlamPointSize = [&settings](const QString& key, float defaultSize) {
        return std::clamp(settings.value(key, defaultSize).toFloat(), 1.0f, 10.0f);
    };
    slamDynamicAggressiveColor = loadSlamColor(
        QStringLiteral("slam/dynamicAggressiveColor"), slamDynamicAggressiveColor);
    slamDynamicModerateColor = loadSlamColor(
        QStringLiteral("slam/dynamicModerateColor"), slamDynamicModerateColor);
    slamDynamicConservativeColor = loadSlamColor(
        QStringLiteral("slam/dynamicConservativeColor"), slamDynamicConservativeColor);
    slamFreeDomScanVoxelColor = loadSlamColor(
        QStringLiteral("slam/freeDomScanVoxelColor"), slamFreeDomScanVoxelColor);
    slamFreeDomDynamicVoxelColor = loadSlamColor(
        QStringLiteral("slam/freeDomDynamicVoxelColor"), slamFreeDomDynamicVoxelColor);
    slamFreeDomRaycastedVoxelColor = loadSlamColor(
        QStringLiteral("slam/freeDomRaycastedVoxelColor"), slamFreeDomRaycastedVoxelColor);
    slamFreeDomFreeVoxelColor = loadSlamColor(
        QStringLiteral("slam/freeDomFreeVoxelColor"), slamFreeDomFreeVoxelColor);
    slamFreeDomStaticVoxelColor = loadSlamColor(
        QStringLiteral("slam/freeDomStaticVoxelColor"), slamFreeDomStaticVoxelColor);
    slamFreeDomEnhancedColor = loadSlamColor(
        QStringLiteral("slam/freeDomEnhancedColor"), slamFreeDomEnhancedColor);
    slamFreeDomScanVoxelPointSizePx = loadSlamPointSize(
        QStringLiteral("slam/freeDomScanVoxelPointSizePx"), slamFreeDomScanVoxelPointSizePx);
    slamFreeDomDynamicVoxelPointSizePx = loadSlamPointSize(
        QStringLiteral("slam/freeDomDynamicVoxelPointSizePx"), slamFreeDomDynamicVoxelPointSizePx);
    slamFreeDomRaycastedVoxelPointSizePx = loadSlamPointSize(
        QStringLiteral("slam/freeDomRaycastedVoxelPointSizePx"), slamFreeDomRaycastedVoxelPointSizePx);
    slamFreeDomFreeVoxelPointSizePx = loadSlamPointSize(
        QStringLiteral("slam/freeDomFreeVoxelPointSizePx"), slamFreeDomFreeVoxelPointSizePx);
    slamFreeDomStaticVoxelPointSizePx = loadSlamPointSize(
        QStringLiteral("slam/freeDomStaticVoxelPointSizePx"), slamFreeDomStaticVoxelPointSizePx);
    slamFreeDomEnhancedPointSizePx = loadSlamPointSize(
        QStringLiteral("slam/freeDomEnhancedPointSizePx"), slamFreeDomEnhancedPointSizePx);
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
    pointCloudEdlConfig.enabled = settings.value(
        QStringLiteral("visualization/edl/enabled"), pointCloudEdlConfig.enabled).toBool();
    pointCloudEdlConfig.strength = std::clamp(
        settings.value(QStringLiteral("visualization/edl/qedlStrength"), pointCloudEdlConfig.strength).toFloat(),
        0.0f,
        300.0f);
    pointCloudEdlConfig.radiusPx = std::clamp(
        settings.value(QStringLiteral("visualization/edl/qedlRadiusPx"), pointCloudEdlConfig.radiusPx).toFloat(),
        0.5f,
        8.0f);
    const float storedEdlRenderScale = settings.value(
        QStringLiteral("visualization/edl/renderScale"), pointCloudEdlConfig.renderScale).toFloat();
    pointCloudEdlConfig.renderScale = storedEdlRenderScale < 0.625f
        ? 0.5f
        : (storedEdlRenderScale < 0.875f ? 0.75f : 1.0f);
    pointCloudEdlConfig.roundPointSplat = settings.value(
        QStringLiteral("visualization/edl/roundPointSplat"),
        pointCloudEdlConfig.roundPointSplat).toBool();
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
    forEachPointCloudView([this](PointCloudView* view) {
        if (view) {
            view->setEdlConfig(pointCloudEdlConfig);
        }
    });
    syncPointCloudEdlAction();
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
    settings.setValue(QStringLiteral("visualization/edl/enabled"), pointCloudEdlConfig.enabled);
    settings.setValue(QStringLiteral("visualization/edl/qedlStrength"), pointCloudEdlConfig.strength);
    settings.setValue(QStringLiteral("visualization/edl/qedlRadiusPx"), pointCloudEdlConfig.radiusPx);
    settings.setValue(QStringLiteral("visualization/edl/renderScale"), pointCloudEdlConfig.renderScale);
    settings.setValue(QStringLiteral("visualization/edl/roundPointSplat"),
                      pointCloudEdlConfig.roundPointSplat);
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
    settings.setValue(QStringLiteral("slam/dynamicObjectColor"), slamDynamicObjectColor);
    settings.setValue(QStringLiteral("slam/dynamicObjectPointSizePx"), slamDynamicObjectPointSizePx);
    settings.setValue(QStringLiteral("slam/dynamicAggressiveColor"), slamDynamicAggressiveColor);
    settings.setValue(QStringLiteral("slam/dynamicModerateColor"), slamDynamicModerateColor);
    settings.setValue(QStringLiteral("slam/dynamicConservativeColor"), slamDynamicConservativeColor);
    settings.setValue(QStringLiteral("slam/freeDomScanVoxelColor"), slamFreeDomScanVoxelColor);
    settings.setValue(QStringLiteral("slam/freeDomDynamicVoxelColor"), slamFreeDomDynamicVoxelColor);
    settings.setValue(QStringLiteral("slam/freeDomRaycastedVoxelColor"), slamFreeDomRaycastedVoxelColor);
    settings.setValue(QStringLiteral("slam/freeDomFreeVoxelColor"), slamFreeDomFreeVoxelColor);
    settings.setValue(QStringLiteral("slam/freeDomStaticVoxelColor"), slamFreeDomStaticVoxelColor);
    settings.setValue(QStringLiteral("slam/freeDomEnhancedColor"), slamFreeDomEnhancedColor);
    settings.setValue(QStringLiteral("slam/freeDomScanVoxelPointSizePx"), slamFreeDomScanVoxelPointSizePx);
    settings.setValue(QStringLiteral("slam/freeDomDynamicVoxelPointSizePx"), slamFreeDomDynamicVoxelPointSizePx);
    settings.setValue(QStringLiteral("slam/freeDomRaycastedVoxelPointSizePx"), slamFreeDomRaycastedVoxelPointSizePx);
    settings.setValue(QStringLiteral("slam/freeDomFreeVoxelPointSizePx"), slamFreeDomFreeVoxelPointSizePx);
    settings.setValue(QStringLiteral("slam/freeDomStaticVoxelPointSizePx"), slamFreeDomStaticVoxelPointSizePx);
    settings.setValue(QStringLiteral("slam/freeDomEnhancedPointSizePx"), slamFreeDomEnhancedPointSizePx);
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
    QColor selectedSlamDynamicObjectColor = slamDynamicObjectColor;
    QColor selectedSlamDynamicAggressiveColor = slamDynamicAggressiveColor;
    QColor selectedSlamDynamicModerateColor = slamDynamicModerateColor;
    QColor selectedSlamDynamicConservativeColor = slamDynamicConservativeColor;
    QColor selectedSlamFreeDomScanVoxelColor = slamFreeDomScanVoxelColor;
    QColor selectedSlamFreeDomDynamicVoxelColor = slamFreeDomDynamicVoxelColor;
    QColor selectedSlamFreeDomRaycastedVoxelColor = slamFreeDomRaycastedVoxelColor;
    QColor selectedSlamFreeDomFreeVoxelColor = slamFreeDomFreeVoxelColor;
    QColor selectedSlamFreeDomStaticVoxelColor = slamFreeDomStaticVoxelColor;
    QColor selectedSlamFreeDomEnhancedColor = slamFreeDomEnhancedColor;
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
    QWidget* colorTab = createSettingsPage("着色", "设置点云颜色映射与屏幕空间深度增强效果。");
    QWidget* backgroundTab = createSettingsPage("背景", "设置 OpenGL 点云视图的背景颜色。");
    QWidget* slamTab = createSettingsPage("后端", "设置 SLAM 后端和轨迹显示样式等参数。");

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

    SwitchCheckBox* edlEnabledCheck = new SwitchCheckBox(colorTab);
    edlEnabledCheck->setChecked(pointCloudEdlConfig.enabled);
    usePreferenceControlColumn(edlEnabledCheck);
    auto createEdlSpin = [&dlg](double minimum,
                                double maximum,
                                double step,
                                double value,
                                const QString& suffix = QString()) {
        QDoubleSpinBox* spin = new QDoubleSpinBox(&dlg);
        spin->setRange(minimum, maximum);
        spin->setDecimals(2);
        spin->setSingleStep(step);
        spin->setValue(value);
        spin->setSuffix(suffix);
        preparePreferenceSpinBox(spin);
        return spin;
    };
    QDoubleSpinBox* edlStrengthSpin = createEdlSpin(0.0, 300.0, 5.0, pointCloudEdlConfig.strength);
    QDoubleSpinBox* edlRadiusSpin = createEdlSpin(0.5, 8.0, 0.1, pointCloudEdlConfig.radiusPx,
                                                   QStringLiteral(" px"));
    QComboBox* edlQualityCombo = new QComboBox(colorTab);
    edlQualityCombo->addItem(QStringLiteral("低（50% 分辨率）"), 0);
    edlQualityCombo->addItem(QStringLiteral("中（75% 分辨率）"), 1);
    edlQualityCombo->addItem(QStringLiteral("高（100% 分辨率）"), 2);
    int edlQuality = 2;
    if (pointCloudEdlConfig.renderScale == 0.5f) {
        edlQuality = 0;
    } else if (pointCloudEdlConfig.renderScale == 0.75f) {
        edlQuality = 1;
    }
    edlQualityCombo->setCurrentIndex(edlQuality);
    edlQualityCombo->setFixedWidth(220);
    usePreferenceControlColumn(edlQualityCombo, 220);
    SwitchCheckBox* edlRoundPointCheck = new SwitchCheckBox(colorTab);
    edlRoundPointCheck->setChecked(pointCloudEdlConfig.roundPointSplat);
    usePreferenceControlColumn(edlRoundPointCheck);

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
    slamTemplateCombo->addItem(slamLidarTemplateDisplayName(SlamLidarTemplate::Mid360L),
                               int(SlamLidarTemplate::Mid360L));
    slamTemplateCombo->addItem(slamLidarTemplateDisplayName(SlamLidarTemplate::Avia),
                               int(SlamLidarTemplate::Avia));
    slamTemplateCombo->addItem(slamLidarTemplateDisplayName(SlamLidarTemplate::Avia2),
                               int(SlamLidarTemplate::Avia2));
    slamTemplateCombo->addItem(slamLidarTemplateDisplayName(SlamLidarTemplate::Custom),
                               int(SlamLidarTemplate::Custom));
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
    SwitchCheckBox* slamLidarOnlyCheck = createSlamSwitch(slamRuntimeConfig.allowPureLidar &&
                                                          !slamRuntimeConfig.imuEnabled);
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
            createSlamExtrinsicSpin(slamRuntimeConfig.extrinsicT_L_I[i], -10.0, 10.0, 7, 0.0001, QStringLiteral(" m"));
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
    SwitchCheckBox* slamDynamicDetectionCheck = createSlamSwitch(slamRuntimeConfig.dynamicFilterEnabled);
    SwitchCheckBox* slamDynamicRemovalCheck = createSlamSwitch(slamRuntimeConfig.dynamicPointRemovalEnabled);
    SwitchCheckBox* slamDynamicDebugCheck = createSlamSwitch(
        slamRuntimeConfig.dynamicDebugVisualizationEnabled);
    QComboBox* slamDynamicBackendCombo = new QComboBox(&dlg);
    slamDynamicBackendCombo->addItem(
        QStringLiteral("M-detector"), int(DynamicFilterBackend::MDetector));
    slamDynamicBackendCombo->addItem(
        QStringLiteral("FreeDOM"), int(DynamicFilterBackend::FreeDOM));
    slamDynamicBackendCombo->setCurrentIndex(std::max(
        0,
        slamDynamicBackendCombo->findData(
            int(slamRuntimeConfig.dynamicFilterBackend))));
    slamDynamicBackendCombo->setFixedWidth(kPreferenceComboBoxWidth);
    usePreferenceControlColumn(slamDynamicBackendCombo);
    SwitchCheckBox* slamDynamicClusterCheck = createSlamSwitch(slamRuntimeConfig.dynamicObjectClusterEnabled);
    QDoubleSpinBox* slamDynamicBufferDelaySpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectBufferDelaySec, 0.0, 5.0, 3, 0.01, QStringLiteral(" s"));
    QDoubleSpinBox* slamDynamicDepthMapDurationSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectDepthMapDurationSec, 0.01, 10.0, 3, 0.05, QStringLiteral(" s"));
    auto createSlamIntSpin = [&dlg](int value, int minValue, int maxValue) {
        QSpinBox* spin = new QSpinBox(&dlg);
        spin->setRange(minValue, maxValue);
        spin->setValue(value);
        spin->setFixedWidth(kPreferenceSpinBoxWidth);
        usePreferenceControlColumn(spin);
        return spin;
    };
    QDoubleSpinBox* freeDomSensorMinRangeSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.sensorMinRangeM, 0.0, 1999.9, 2, 0.1, QStringLiteral(" m"));
    QDoubleSpinBox* freeDomSensorMaxRangeSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.sensorMaxRangeM, 0.1, 2000.0, 1, 5.0, QStringLiteral(" m"));
    QDoubleSpinBox* freeDomSensorMinZSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.sensorMinZM, -1000.0, 999.9, 1, 1.0, QStringLiteral(" m"));
    QDoubleSpinBox* freeDomSensorMaxZSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.sensorMaxZM, -999.9, 1000.0, 1, 1.0, QStringLiteral(" m"));
    QDoubleSpinBox* freeDomSubVoxelSizeSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.subVoxelSizeM, 0.01, 10.0, 3, 0.01, QStringLiteral(" m"));
    QSpinBox* freeDomVoxelDepthSpin = createSlamIntSpin(
        int(slamRuntimeConfig.freeDom.voxelDepth), 0, 12);
    QSpinBox* freeDomBlockDepthSpin = createSlamIntSpin(
        int(slamRuntimeConfig.freeDom.blockDepth), 0, 16);
    SwitchCheckBox* freeDomLocalMapCheck = createSlamSwitch(
        slamRuntimeConfig.freeDom.localMapEnabled);
    QDoubleSpinBox* freeDomLocalMapRangeSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.localMapRangeM, 0.1, 2000.0, 1, 5.0, QStringLiteral(" m"));
    QDoubleSpinBox* freeDomLocalMapMinZSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.localMapMinZM, -1000.0, 999.9, 1, 1.0, QStringLiteral(" m"));
    QDoubleSpinBox* freeDomLocalMapMaxZSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.localMapMaxZM, -999.9, 1000.0, 1, 1.0, QStringLiteral(" m"));
    QDoubleSpinBox* freeDomRaycastMaxRangeSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.raycastMaxRangeM, 0.1, 2000.0, 1, 5.0, QStringLiteral(" m"));
    QDoubleSpinBox* freeDomRaycastMinZSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.raycastMinZM, -1000.0, 999.9, 1, 1.0, QStringLiteral(" m"));
    QDoubleSpinBox* freeDomRaycastMaxZSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.raycastMaxZM, -999.9, 1000.0, 1, 1.0, QStringLiteral(" m"));
    QSpinBox* freeDomCountsToFreeSpin = createSlamIntSpin(
        int(slamRuntimeConfig.freeDom.countsToFree), 1, 10000);
    QSpinBox* freeDomCountsToRevertSpin = createSlamIntSpin(
        int(slamRuntimeConfig.freeDom.countsToRevert), 1, 10000);
    QComboBox* freeDomConservativeConnectivityCombo = new QComboBox(&dlg);
    for(int value : {6, 18, 26})
        freeDomConservativeConnectivityCombo->addItem(QString::number(value), value);
    freeDomConservativeConnectivityCombo->setCurrentIndex(
        freeDomConservativeConnectivityCombo->findData(
            int(slamRuntimeConfig.freeDom.conservativeConnectivity)));
    freeDomConservativeConnectivityCombo->setFixedWidth(kPreferenceComboBoxWidth);
    usePreferenceControlColumn(freeDomConservativeConnectivityCombo);
    QComboBox* freeDomAggressiveConnectivityCombo = new QComboBox(&dlg);
    for(int value : {6, 18, 26, 80, 124})
        freeDomAggressiveConnectivityCombo->addItem(QString::number(value), value);
    freeDomAggressiveConnectivityCombo->setCurrentIndex(
        freeDomAggressiveConnectivityCombo->findData(
            int(slamRuntimeConfig.freeDom.aggressiveConnectivity)));
    freeDomAggressiveConnectivityCombo->setFixedWidth(kPreferenceComboBoxWidth);
    usePreferenceControlColumn(freeDomAggressiveConnectivityCombo);
    SwitchCheckBox* freeDomRaycastEnhancementCheck = createSlamSwitch(
        slamRuntimeConfig.freeDom.raycastEnhancementEnabled);
    QDoubleSpinBox* freeDomHorizontalFovSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.lidarHorizontalFovDeg, 1.0, 360.0, 1, 1.0, QStringLiteral(" °"));
    QDoubleSpinBox* freeDomVerticalFovUpperSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.lidarVerticalFovUpperDeg, -89.9, 90.0, 1, 1.0, QStringLiteral(" °"));
    QDoubleSpinBox* freeDomVerticalFovLowerSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.lidarVerticalFovLowerDeg, -90.0, 89.9, 1, 1.0, QStringLiteral(" °"));
    QSpinBox* freeDomDepthLinesSpin = createSlamIntSpin(
        int(slamRuntimeConfig.freeDom.depthImageVerticalLines), 2, 4096);
    QDoubleSpinBox* freeDomDepthMinRangeSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.depthImageMinRangeM, 0.0, 2000.0, 2, 0.1, QStringLiteral(" m"));
    QDoubleSpinBox* freeDomEnhancementMaxRangeSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.maxRaycastEnhancementRangeM, 0.1, 2000.0, 1, 5.0, QStringLiteral(" m"));
    QDoubleSpinBox* freeDomDepthMarginSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.raycastEnhancementDepthMarginM, 0.0, 10.0, 2, 0.05, QStringLiteral(" m"));
    QSpinBox* freeDomInpaintSizeSpin = createSlamIntSpin(
        int(slamRuntimeConfig.freeDom.inpaintSize), 1, 99);
    QSpinBox* freeDomErosionSizeSpin = createSlamIntSpin(
        int(slamRuntimeConfig.freeDom.erosionSize), 0, 99);
    QDoubleSpinBox* freeDomMinEnhancementAreaSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.minRaycastEnhancementArea, 0.0, 1.0, 5, 0.001, QString());
    QDoubleSpinBox* freeDomTopMarginSpin = createSlamDoubleSpin(
        slamRuntimeConfig.freeDom.depthImageTopMargin, 0.0, 1.0, 3, 0.01, QString());
    SwitchCheckBox* freeDomLearnFovCheck = createSlamSwitch(
        slamRuntimeConfig.freeDom.learnFov);
    SwitchCheckBox* freeDomFovMaskCheck = createSlamSwitch(
        slamRuntimeConfig.freeDom.fovMaskEnabled);
    QLineEdit* freeDomFovMaskPathEdit = new QLineEdit(
        QString::fromStdString(slamRuntimeConfig.freeDom.fovMaskPath), &dlg);
    freeDomFovMaskPathEdit->setMinimumWidth(260);
    QSpinBox* freeDomThreadsSpin = createSlamIntSpin(
        int(slamRuntimeConfig.freeDom.numThreads), 1, 64);
    QSpinBox* freeDomDebugIntervalSpin = createSlamIntSpin(
        int(slamRuntimeConfig.dynamicDebugSnapshotIntervalFrames), 1, 10000);
    QSpinBox* freeDomMapIntervalSpin = createSlamIntSpin(
        int(slamRuntimeConfig.freeDomMapSnapshotIntervalFrames), 1, 10000);
    connect(freeDomVoxelDepthSpin, QOverload<int>::of(&QSpinBox::valueChanged), &dlg,
            [freeDomBlockDepthSpin](int value) {
                freeDomBlockDepthSpin->setMinimum(value);
            });
    freeDomBlockDepthSpin->setMinimum(freeDomVoxelDepthSpin->value());
    connect(freeDomSensorMinRangeSpin,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            &dlg,
            [freeDomSensorMaxRangeSpin](double value) {
                freeDomSensorMaxRangeSpin->setMinimum(value + 0.1);
            });
    connect(freeDomSensorMinZSpin,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            &dlg,
            [freeDomSensorMaxZSpin](double value) {
                freeDomSensorMaxZSpin->setMinimum(value + 0.1);
            });
    connect(freeDomLocalMapMinZSpin,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            &dlg,
            [freeDomLocalMapMaxZSpin](double value) {
                freeDomLocalMapMaxZSpin->setMinimum(value + 0.1);
            });
    connect(freeDomRaycastMinZSpin,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            &dlg,
            [freeDomRaycastMaxZSpin](double value) {
                freeDomRaycastMaxZSpin->setMinimum(value + 0.1);
            });
    connect(freeDomVerticalFovLowerSpin,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            &dlg,
            [freeDomVerticalFovUpperSpin](double value) {
                freeDomVerticalFovUpperSpin->setMinimum(value + 0.1);
            });
    auto syncFreeDomFovMaskPath = [freeDomLearnFovCheck,
                                   freeDomFovMaskCheck,
                                   freeDomFovMaskPathEdit]() {
        freeDomFovMaskPathEdit->setEnabled(
            freeDomLearnFovCheck->isChecked() || freeDomFovMaskCheck->isChecked());
    };
    connect(freeDomLearnFovCheck, &QCheckBox::toggled, &dlg,
            [syncFreeDomFovMaskPath](bool) { syncFreeDomFovMaskPath(); });
    connect(freeDomFovMaskCheck, &QCheckBox::toggled, &dlg,
            [syncFreeDomFovMaskPath](bool) { syncFreeDomFovMaskPath(); });
    syncFreeDomFovMaskPath();
    SwitchCheckBox* slamLoopClosureCheck = createSlamSwitch(slamRuntimeConfig.loopClosureEnableFlag);
    QDoubleSpinBox* slamKeyframeDistanceSpin = createSlamDoubleSpin(
        slamRuntimeConfig.surroundingKeyframeAddingDistThreshold, 0.1, 1000.0, 1, 0.5, QStringLiteral(" m"));
    QDoubleSpinBox* slamKeyframeAngleSpin = createSlamDoubleSpin(
        slamRuntimeConfig.surroundingKeyframeAddingAngleThreshold, 0.001, 3.142, 3, 0.01, QStringLiteral(" rad"));
    QDoubleSpinBox* slamLoopClosureFrequencySpin = createSlamDoubleSpin(
        slamRuntimeConfig.loopClosureFrequency, 0.1, 100.0, 1, 0.1, QStringLiteral(" Hz"));
    QDoubleSpinBox* slamHistoryKeyframeSearchRadiusSpin = createSlamDoubleSpin(
        slamRuntimeConfig.historyKeyframeSearchRadius, 0.1, 1000.0, 1, 0.5, QStringLiteral(" m"));
    QDoubleSpinBox* slamHistoryKeyframeSearchTimeDiffSpin = createSlamDoubleSpin(
        slamRuntimeConfig.historyKeyframeSearchTimeDiff, 0.1, 3600.0, 1, 1.0, QStringLiteral(" s"));
    QSpinBox* slamHistoryKeyframeSearchNumSpin = createSlamIntSpin(
        slamRuntimeConfig.historyKeyframeSearchNum, 1, 1000);
    QDoubleSpinBox* slamHistoryKeyframeFitnessScoreSpin = createSlamDoubleSpin(
        slamRuntimeConfig.historyKeyframeFitnessScore, 0.001, 10.0, 3, 0.01, QString());
    SwitchCheckBox* slamReconstructKdTreeCheck = createSlamSwitch(slamRuntimeConfig.reconstructKdTree);
    const QVector<QWidget*> slamLoopClosureControls = {
        slamLoopClosureFrequencySpin,
        slamHistoryKeyframeSearchRadiusSpin,
        slamHistoryKeyframeSearchTimeDiffSpin,
        slamHistoryKeyframeSearchNumSpin,
        slamHistoryKeyframeFitnessScoreSpin,
        slamReconstructKdTreeCheck
    };
    auto syncSlamLoopClosureControls = [slamLoopClosureCheck, slamLoopClosureControls]() {
        const bool enabled = slamLoopClosureCheck->isChecked();
        for (QWidget* control : slamLoopClosureControls) {
            control->setEnabled(enabled);
        }
    };
    connect(slamLoopClosureCheck, &QCheckBox::toggled, &dlg,
            [syncSlamLoopClosureControls](bool) { syncSlamLoopClosureControls(); });
    syncSlamLoopClosureControls();
    QDoubleSpinBox* slamDynamicClusterVoxelSizeSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectClusterVoxelSizeM, 0.05, 2.0, 2, 0.05, QStringLiteral(" m"));
    QSpinBox* slamDynamicClusterExtendVoxelSpin = createSlamIntSpin(
        slamRuntimeConfig.dynamicObjectClusterExtendVoxel, 1, 10);
    QSpinBox* slamDynamicClusterMinVoxelCountSpin = createSlamIntSpin(
        slamRuntimeConfig.dynamicObjectClusterMinVoxelCount, 1, 100);
    QDoubleSpinBox* slamDynamicClusterTrustThresholdSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectClusterTrustThreshold, 0.01, 1.0, 2, 0.05, QString());
    QDoubleSpinBox* slamDynamicClusterGroundDistanceSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectClusterGroundDistanceThresholdM,
        0.02, 1.0, 2, 0.01, QStringLiteral(" m"));
    QDoubleSpinBox* slamDynamicClusterGroundMaxAngleSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectClusterGroundMaxAngleDeg,
        1.0, 89.0, 1, 1.0, QStringLiteral(" °"));
    QSpinBox* slamDynamicMaxDepthMapsSpin = createSlamIntSpin(
        slamRuntimeConfig.dynamicObjectMaxDepthMaps, 1, 50);
    QSpinBox* slamDynamicMinHistoryMapsSpin = createSlamIntSpin(
        slamRuntimeConfig.dynamicObjectMinHistoryMaps, 1, slamRuntimeConfig.dynamicObjectMaxDepthMaps);
    connect(slamDynamicMaxDepthMapsSpin, QOverload<int>::of(&QSpinBox::valueChanged), &dlg,
            [slamDynamicMinHistoryMapsSpin](int value) {
                slamDynamicMinHistoryMapsSpin->setMaximum(value);
            });
    QDoubleSpinBox* slamDynamicHorizontalResolutionSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectHorizontalResolutionRad, 0.0025, 0.2, 4, 0.0025, QStringLiteral(" rad"));
    QDoubleSpinBox* slamDynamicVerticalResolutionSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectVerticalResolutionRad, 0.0025, 0.2, 4, 0.0025, QStringLiteral(" rad"));
    QDoubleSpinBox* slamDynamicVerticalFovDownSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectVerticalFovDownDeg, -90.0, 89.9, 1, 1.0, QStringLiteral(" °"));
    QDoubleSpinBox* slamDynamicVerticalFovUpSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectVerticalFovUpDeg, -89.9, 90.0, 1, 1.0, QStringLiteral(" °"));
    connect(slamDynamicVerticalFovDownSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [slamDynamicVerticalFovUpSpin](double value) {
                slamDynamicVerticalFovUpSpin->setMinimum(value + 0.1);
            });
    connect(slamDynamicVerticalFovUpSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [slamDynamicVerticalFovDownSpin](double value) {
                slamDynamicVerticalFovDownSpin->setMaximum(value - 0.1);
            });
    QDoubleSpinBox* slamDynamicHorizontalFovRightSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectHorizontalFovRightDeg, -180.0, 179.9, 1, 1.0, QStringLiteral(" °"));
    QDoubleSpinBox* slamDynamicHorizontalFovLeftSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectHorizontalFovLeftDeg, -179.9, 180.0, 1, 1.0, QStringLiteral(" °"));
    connect(slamDynamicHorizontalFovRightSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [slamDynamicHorizontalFovLeftSpin](double value) {
                slamDynamicHorizontalFovLeftSpin->setMinimum(value + 0.1);
            });
    connect(slamDynamicHorizontalFovLeftSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [slamDynamicHorizontalFovRightSpin](double value) {
                slamDynamicHorizontalFovRightSpin->setMaximum(value - 0.1);
            });
    QDoubleSpinBox* slamDynamicMinRangeSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectMinRangeM, 0.0, 1999.9, 2, 0.1, QStringLiteral(" m"));
    QDoubleSpinBox* slamDynamicMaxRangeSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectMaxRangeM, 0.1, 2000.0, 1, 10.0, QStringLiteral(" m"));
    connect(slamDynamicMinRangeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [slamDynamicMaxRangeSpin](double value) {
                slamDynamicMaxRangeSpin->setMinimum(value + 0.1);
            });
    connect(slamDynamicMaxRangeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), &dlg,
            [slamDynamicMinRangeSpin](double value) {
                slamDynamicMinRangeSpin->setMaximum(value - 0.1);
            });
    QSpinBox* slamDynamicNeighborPixelRadiusSpin = createSlamIntSpin(
        slamRuntimeConfig.dynamicObjectNeighborPixelRadius, 0, 10);
    QDoubleSpinBox* slamDynamicCase1DepthMarginSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectCase1DepthMarginM, 0.01, 10.0, 2, 0.05, QStringLiteral(" m"));
    QDoubleSpinBox* slamDynamicCase2DepthMarginSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectCase2DepthMarginM, 0.01, 10.0, 2, 0.05, QStringLiteral(" m"));
    QDoubleSpinBox* slamDynamicCase3DepthMarginSpin = createSlamDoubleSpin(
        slamRuntimeConfig.dynamicObjectCase3DepthMarginM, 0.01, 10.0, 2, 0.05, QStringLiteral(" m"));
    QSpinBox* slamDynamicCase1VoteThresholdSpin = createSlamIntSpin(
        slamRuntimeConfig.dynamicObjectCase1VoteThreshold, 1, 50);
    QSpinBox* slamDynamicCase2OcclusionChainLengthSpin = createSlamIntSpin(
        slamRuntimeConfig.dynamicObjectCase2OcclusionChainLength, 1, 50);
    QSpinBox* slamDynamicCase3OcclusionChainLengthSpin = createSlamIntSpin(
        slamRuntimeConfig.dynamicObjectCase3OcclusionChainLength, 1, 50);
    const QVector<QWidget*> slamDynamicClusterControls = {
        slamDynamicClusterVoxelSizeSpin,
        slamDynamicClusterExtendVoxelSpin,
        slamDynamicClusterMinVoxelCountSpin,
        slamDynamicClusterTrustThresholdSpin,
        slamDynamicClusterGroundDistanceSpin,
        slamDynamicClusterGroundMaxAngleSpin
    };
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
    QWidget* slamDynamicObjectColorRow = new QWidget(&dlg);
    usePreferenceControlColumn(slamDynamicObjectColorRow);
    QHBoxLayout* slamDynamicObjectColorLayout = new QHBoxLayout(slamDynamicObjectColorRow);
    slamDynamicObjectColorLayout->setContentsMargins(0, 0, 0, 0);
    slamDynamicObjectColorLayout->setSpacing(8);
    QPushButton* slamDynamicObjectColorPreview =
        createColorSwatchButton(slamDynamicObjectColorRow, selectedSlamDynamicObjectColor);
    slamDynamicObjectColorLayout->addWidget(slamDynamicObjectColorPreview);
    slamDynamicObjectColorLayout->addStretch();
    connect(slamDynamicObjectColorPreview,
            &QPushButton::clicked,
            &dlg,
            [&dlg, &selectedSlamDynamicObjectColor, slamDynamicObjectColorPreview]() {
                QColor color = QColorDialog::getColor(selectedSlamDynamicObjectColor,
                                                       &dlg,
                                                       QStringLiteral("选择动态物体点云颜色"));
                if (!color.isValid()) {
                    return;
                }
                selectedSlamDynamicObjectColor = color;
                updateColorSwatchButton(slamDynamicObjectColorPreview,
                                        selectedSlamDynamicObjectColor);
            });
    QDoubleSpinBox* slamDynamicObjectPointSizeSpin =
        createSlamDoubleSpin(slamDynamicObjectPointSizePx,
                             1.0,
                             10.0,
                             1,
                             0.5,
                             QStringLiteral(" px"));
    auto createSlamLayerColorRow = [&dlg](QColor* selectedColor,
                                          const QString& dialogTitle) {
        QWidget* row = new QWidget(&dlg);
        usePreferenceControlColumn(row);
        QHBoxLayout* rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->setSpacing(8);
        QPushButton* preview = createColorSwatchButton(row, *selectedColor);
        rowLayout->addWidget(preview);
        rowLayout->addStretch();
        connect(preview,
                &QPushButton::clicked,
                &dlg,
                [&dlg, selectedColor, preview, dialogTitle]() {
                    const QColor color = QColorDialog::getColor(*selectedColor,
                                                                 &dlg,
                                                                 dialogTitle);
                    if (!color.isValid()) {
                        return;
                    }
                    *selectedColor = color;
                    updateColorSwatchButton(preview, *selectedColor);
                });
        return row;
    };
    QWidget* slamDynamicAggressiveColorRow = createSlamLayerColorRow(
        &selectedSlamDynamicAggressiveColor, QStringLiteral("选择 Aggressive 动态点颜色"));
    QWidget* slamDynamicModerateColorRow = createSlamLayerColorRow(
        &selectedSlamDynamicModerateColor, QStringLiteral("选择 Moderate 动态点颜色"));
    QWidget* slamDynamicConservativeColorRow = createSlamLayerColorRow(
        &selectedSlamDynamicConservativeColor, QStringLiteral("选择 Conservative 动态点颜色"));
    QWidget* slamFreeDomScanVoxelColorRow = createSlamLayerColorRow(
        &selectedSlamFreeDomScanVoxelColor, QStringLiteral("选择 Scan Voxel 颜色"));
    QWidget* slamFreeDomDynamicVoxelColorRow = createSlamLayerColorRow(
        &selectedSlamFreeDomDynamicVoxelColor, QStringLiteral("选择 Dynamic Voxel 颜色"));
    QWidget* slamFreeDomRaycastedVoxelColorRow = createSlamLayerColorRow(
        &selectedSlamFreeDomRaycastedVoxelColor, QStringLiteral("选择 Raycasted Voxel 颜色"));
    QWidget* slamFreeDomFreeVoxelColorRow = createSlamLayerColorRow(
        &selectedSlamFreeDomFreeVoxelColor, QStringLiteral("选择 Free Voxel 颜色"));
    QWidget* slamFreeDomStaticVoxelColorRow = createSlamLayerColorRow(
        &selectedSlamFreeDomStaticVoxelColor, QStringLiteral("选择 Static Voxel 颜色"));
    QWidget* slamFreeDomEnhancedColorRow = createSlamLayerColorRow(
        &selectedSlamFreeDomEnhancedColor, QStringLiteral("选择 Enhanced Point 颜色"));
    QDoubleSpinBox* slamFreeDomScanVoxelPointSizeSpin =
        createSlamDoubleSpin(slamFreeDomScanVoxelPointSizePx, 1.0, 10.0, 1, 0.5, QStringLiteral(" px"));
    QDoubleSpinBox* slamFreeDomDynamicVoxelPointSizeSpin =
        createSlamDoubleSpin(slamFreeDomDynamicVoxelPointSizePx, 1.0, 10.0, 1, 0.5, QStringLiteral(" px"));
    QDoubleSpinBox* slamFreeDomRaycastedVoxelPointSizeSpin =
        createSlamDoubleSpin(slamFreeDomRaycastedVoxelPointSizePx, 1.0, 10.0, 1, 0.5, QStringLiteral(" px"));
    QDoubleSpinBox* slamFreeDomFreeVoxelPointSizeSpin =
        createSlamDoubleSpin(slamFreeDomFreeVoxelPointSizePx, 1.0, 10.0, 1, 0.5, QStringLiteral(" px"));
    QDoubleSpinBox* slamFreeDomStaticVoxelPointSizeSpin =
        createSlamDoubleSpin(slamFreeDomStaticVoxelPointSizePx, 1.0, 10.0, 1, 0.5, QStringLiteral(" px"));
    QDoubleSpinBox* slamFreeDomEnhancedPointSizeSpin =
        createSlamDoubleSpin(slamFreeDomEnhancedPointSizePx, 1.0, 10.0, 1, 0.5, QStringLiteral(" px"));
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
    auto applyDynamicFilterControls = [&](SlamRuntimeConfig& config) {
        config.dynamicFilterBackend = static_cast<DynamicFilterBackend>(
            slamDynamicBackendCombo->currentData().toInt());
        config.dynamicFilterEnabled = slamDynamicDetectionCheck->isChecked();
        config.dynamicPointRemovalEnabled =
            config.dynamicFilterEnabled && slamDynamicRemovalCheck->isChecked();
        config.dynamicDebugVisualizationEnabled = slamDynamicDebugCheck->isChecked();
        config.dynamicDebugSnapshotIntervalFrames =
            unsigned(freeDomDebugIntervalSpin->value());
        config.freeDomMapSnapshotIntervalFrames =
            unsigned(freeDomMapIntervalSpin->value());

        config.dynamicObjectDetectionEnabled = config.dynamicFilterEnabled;
        config.dynamicObjectRemovalEnabled = config.dynamicPointRemovalEnabled;
        config.dynamicObjectClusterEnabled =
            config.dynamicFilterBackend == DynamicFilterBackend::MDetector &&
            config.dynamicFilterEnabled && slamDynamicClusterCheck->isChecked();

        FreeDomRuntimeConfig& freeDom = config.freeDom;
        freeDom.sensorMinRangeM = freeDomSensorMinRangeSpin->value();
        freeDom.sensorMaxRangeM = freeDomSensorMaxRangeSpin->value();
        freeDom.sensorMinZM = freeDomSensorMinZSpin->value();
        freeDom.sensorMaxZM = freeDomSensorMaxZSpin->value();
        freeDom.subVoxelSizeM = freeDomSubVoxelSizeSpin->value();
        freeDom.voxelDepth = unsigned(freeDomVoxelDepthSpin->value());
        freeDom.blockDepth = unsigned(freeDomBlockDepthSpin->value());
        freeDom.localMapEnabled = freeDomLocalMapCheck->isChecked();
        freeDom.localMapRangeM = freeDomLocalMapRangeSpin->value();
        freeDom.localMapMinZM = freeDomLocalMapMinZSpin->value();
        freeDom.localMapMaxZM = freeDomLocalMapMaxZSpin->value();
        freeDom.raycastMaxRangeM = freeDomRaycastMaxRangeSpin->value();
        freeDom.raycastMinZM = freeDomRaycastMinZSpin->value();
        freeDom.raycastMaxZM = freeDomRaycastMaxZSpin->value();
        freeDom.countsToFree = unsigned(freeDomCountsToFreeSpin->value());
        freeDom.countsToRevert = unsigned(freeDomCountsToRevertSpin->value());
        freeDom.conservativeConnectivity = unsigned(
            freeDomConservativeConnectivityCombo->currentData().toInt());
        freeDom.aggressiveConnectivity = unsigned(
            freeDomAggressiveConnectivityCombo->currentData().toInt());
        freeDom.raycastEnhancementEnabled = freeDomRaycastEnhancementCheck->isChecked();
        freeDom.lidarHorizontalFovDeg = freeDomHorizontalFovSpin->value();
        freeDom.lidarVerticalFovUpperDeg = freeDomVerticalFovUpperSpin->value();
        freeDom.lidarVerticalFovLowerDeg = freeDomVerticalFovLowerSpin->value();
        freeDom.depthImageVerticalLines = unsigned(freeDomDepthLinesSpin->value());
        freeDom.depthImageMinRangeM = freeDomDepthMinRangeSpin->value();
        freeDom.maxRaycastEnhancementRangeM = freeDomEnhancementMaxRangeSpin->value();
        freeDom.raycastEnhancementDepthMarginM = freeDomDepthMarginSpin->value();
        freeDom.inpaintSize = unsigned(freeDomInpaintSizeSpin->value());
        freeDom.erosionSize = unsigned(freeDomErosionSizeSpin->value());
        freeDom.minRaycastEnhancementArea = freeDomMinEnhancementAreaSpin->value();
        freeDom.depthImageTopMargin = freeDomTopMarginSpin->value();
        freeDom.learnFov = freeDomLearnFovCheck->isChecked();
        freeDom.fovMaskEnabled = freeDomFovMaskCheck->isChecked();
        freeDom.fovMaskPath = freeDomFovMaskPathEdit->text().toStdString();
        freeDom.numThreads = unsigned(freeDomThreadsSpin->value());
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
        slamLidarOnlyCheck->setChecked(runtimeDefaults.allowPureLidar && !runtimeDefaults.imuEnabled);
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
        slamLoopClosureCheck->setChecked(runtimeDefaults.loopClosureEnableFlag);
        slamKeyframeDistanceSpin->setValue(runtimeDefaults.surroundingKeyframeAddingDistThreshold);
        slamKeyframeAngleSpin->setValue(runtimeDefaults.surroundingKeyframeAddingAngleThreshold);
        slamLoopClosureFrequencySpin->setValue(runtimeDefaults.loopClosureFrequency);
        slamHistoryKeyframeSearchRadiusSpin->setValue(runtimeDefaults.historyKeyframeSearchRadius);
        slamHistoryKeyframeSearchTimeDiffSpin->setValue(runtimeDefaults.historyKeyframeSearchTimeDiff);
        slamHistoryKeyframeSearchNumSpin->setValue(runtimeDefaults.historyKeyframeSearchNum);
        slamHistoryKeyframeFitnessScoreSpin->setValue(runtimeDefaults.historyKeyframeFitnessScore);
        slamReconstructKdTreeCheck->setChecked(runtimeDefaults.reconstructKdTree);
        slamDynamicBackendCombo->setCurrentIndex(std::max(
            0,
            slamDynamicBackendCombo->findData(int(runtimeDefaults.dynamicFilterBackend))));
        slamDynamicDetectionCheck->setChecked(runtimeDefaults.dynamicFilterEnabled);
        slamDynamicRemovalCheck->setChecked(runtimeDefaults.dynamicPointRemovalEnabled);
        slamDynamicDebugCheck->setChecked(runtimeDefaults.dynamicDebugVisualizationEnabled);
        freeDomDebugIntervalSpin->setValue(
            int(runtimeDefaults.dynamicDebugSnapshotIntervalFrames));
        freeDomMapIntervalSpin->setValue(
            int(runtimeDefaults.freeDomMapSnapshotIntervalFrames));
        const FreeDomRuntimeConfig& freeDom = runtimeDefaults.freeDom;
        freeDomSensorMinRangeSpin->setValue(freeDom.sensorMinRangeM);
        freeDomSensorMaxRangeSpin->setValue(freeDom.sensorMaxRangeM);
        freeDomSensorMinZSpin->setValue(freeDom.sensorMinZM);
        freeDomSensorMaxZSpin->setValue(freeDom.sensorMaxZM);
        freeDomSubVoxelSizeSpin->setValue(freeDom.subVoxelSizeM);
        freeDomVoxelDepthSpin->setValue(int(freeDom.voxelDepth));
        freeDomBlockDepthSpin->setValue(int(freeDom.blockDepth));
        freeDomLocalMapCheck->setChecked(freeDom.localMapEnabled);
        freeDomLocalMapRangeSpin->setValue(freeDom.localMapRangeM);
        freeDomLocalMapMinZSpin->setValue(freeDom.localMapMinZM);
        freeDomLocalMapMaxZSpin->setValue(freeDom.localMapMaxZM);
        freeDomRaycastMaxRangeSpin->setValue(freeDom.raycastMaxRangeM);
        freeDomRaycastMinZSpin->setValue(freeDom.raycastMinZM);
        freeDomRaycastMaxZSpin->setValue(freeDom.raycastMaxZM);
        freeDomCountsToFreeSpin->setValue(int(freeDom.countsToFree));
        freeDomCountsToRevertSpin->setValue(int(freeDom.countsToRevert));
        freeDomConservativeConnectivityCombo->setCurrentIndex(
            freeDomConservativeConnectivityCombo->findData(
                int(freeDom.conservativeConnectivity)));
        freeDomAggressiveConnectivityCombo->setCurrentIndex(
            freeDomAggressiveConnectivityCombo->findData(
                int(freeDom.aggressiveConnectivity)));
        freeDomRaycastEnhancementCheck->setChecked(freeDom.raycastEnhancementEnabled);
        freeDomHorizontalFovSpin->setValue(freeDom.lidarHorizontalFovDeg);
        freeDomVerticalFovLowerSpin->setValue(freeDom.lidarVerticalFovLowerDeg);
        freeDomVerticalFovUpperSpin->setValue(freeDom.lidarVerticalFovUpperDeg);
        freeDomDepthLinesSpin->setValue(int(freeDom.depthImageVerticalLines));
        freeDomDepthMinRangeSpin->setValue(freeDom.depthImageMinRangeM);
        freeDomEnhancementMaxRangeSpin->setValue(freeDom.maxRaycastEnhancementRangeM);
        freeDomDepthMarginSpin->setValue(freeDom.raycastEnhancementDepthMarginM);
        freeDomInpaintSizeSpin->setValue(int(freeDom.inpaintSize));
        freeDomErosionSizeSpin->setValue(int(freeDom.erosionSize));
        freeDomMinEnhancementAreaSpin->setValue(freeDom.minRaycastEnhancementArea);
        freeDomTopMarginSpin->setValue(freeDom.depthImageTopMargin);
        freeDomLearnFovCheck->setChecked(freeDom.learnFov);
        freeDomFovMaskCheck->setChecked(freeDom.fovMaskEnabled);
        freeDomFovMaskPathEdit->setText(QString::fromStdString(freeDom.fovMaskPath));
        freeDomThreadsSpin->setValue(int(freeDom.numThreads));
        slamDynamicClusterCheck->setChecked(runtimeDefaults.dynamicObjectClusterEnabled);
        slamDynamicClusterVoxelSizeSpin->setValue(runtimeDefaults.dynamicObjectClusterVoxelSizeM);
        slamDynamicClusterExtendVoxelSpin->setValue(runtimeDefaults.dynamicObjectClusterExtendVoxel);
        slamDynamicClusterMinVoxelCountSpin->setValue(runtimeDefaults.dynamicObjectClusterMinVoxelCount);
        slamDynamicClusterTrustThresholdSpin->setValue(runtimeDefaults.dynamicObjectClusterTrustThreshold);
        slamDynamicClusterGroundDistanceSpin->setValue(
            runtimeDefaults.dynamicObjectClusterGroundDistanceThresholdM);
        slamDynamicClusterGroundMaxAngleSpin->setValue(runtimeDefaults.dynamicObjectClusterGroundMaxAngleDeg);
        slamDynamicBufferDelaySpin->setValue(runtimeDefaults.dynamicObjectBufferDelaySec);
        slamDynamicDepthMapDurationSpin->setValue(runtimeDefaults.dynamicObjectDepthMapDurationSec);
        slamDynamicMaxDepthMapsSpin->setValue(runtimeDefaults.dynamicObjectMaxDepthMaps);
        slamDynamicMinHistoryMapsSpin->setValue(runtimeDefaults.dynamicObjectMinHistoryMaps);
        slamDynamicHorizontalResolutionSpin->setValue(runtimeDefaults.dynamicObjectHorizontalResolutionRad);
        slamDynamicVerticalResolutionSpin->setValue(runtimeDefaults.dynamicObjectVerticalResolutionRad);
        slamDynamicVerticalFovDownSpin->setValue(runtimeDefaults.dynamicObjectVerticalFovDownDeg);
        slamDynamicVerticalFovUpSpin->setValue(runtimeDefaults.dynamicObjectVerticalFovUpDeg);
        slamDynamicHorizontalFovRightSpin->setValue(runtimeDefaults.dynamicObjectHorizontalFovRightDeg);
        slamDynamicHorizontalFovLeftSpin->setValue(runtimeDefaults.dynamicObjectHorizontalFovLeftDeg);
        slamDynamicMinRangeSpin->setValue(runtimeDefaults.dynamicObjectMinRangeM);
        slamDynamicMaxRangeSpin->setValue(runtimeDefaults.dynamicObjectMaxRangeM);
        slamDynamicNeighborPixelRadiusSpin->setValue(runtimeDefaults.dynamicObjectNeighborPixelRadius);
        slamDynamicCase1DepthMarginSpin->setValue(runtimeDefaults.dynamicObjectCase1DepthMarginM);
        slamDynamicCase2DepthMarginSpin->setValue(runtimeDefaults.dynamicObjectCase2DepthMarginM);
        slamDynamicCase3DepthMarginSpin->setValue(runtimeDefaults.dynamicObjectCase3DepthMarginM);
        slamDynamicCase1VoteThresholdSpin->setValue(runtimeDefaults.dynamicObjectCase1VoteThreshold);
        slamDynamicCase2OcclusionChainLengthSpin->setValue(
            runtimeDefaults.dynamicObjectCase2OcclusionChainLength);
        slamDynamicCase3OcclusionChainLengthSpin->setValue(
            runtimeDefaults.dynamicObjectCase3OcclusionChainLength);
        syncSlamPublishControls();
        syncSlamLoopClosureControls();
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
        config.allowPureLidar = slamLidarOnlyCheck->isChecked();
        config.imuEnabled = !config.allowPureLidar;
        config.backendType = config.allowPureLidar ? QStringLiteral("FAST_LO") : QStringLiteral("FAST_LIO");
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
        config.loopClosureEnableFlag = slamLoopClosureCheck->isChecked();
        config.surroundingKeyframeAddingDistThreshold = slamKeyframeDistanceSpin->value();
        config.surroundingKeyframeAddingAngleThreshold = slamKeyframeAngleSpin->value();
        config.loopClosureFrequency = slamLoopClosureFrequencySpin->value();
        config.historyKeyframeSearchRadius = slamHistoryKeyframeSearchRadiusSpin->value();
        config.historyKeyframeSearchTimeDiff = slamHistoryKeyframeSearchTimeDiffSpin->value();
        config.historyKeyframeSearchNum = slamHistoryKeyframeSearchNumSpin->value();
        config.historyKeyframeFitnessScore = slamHistoryKeyframeFitnessScoreSpin->value();
        config.reconstructKdTree = slamReconstructKdTreeCheck->isChecked();
        applyDynamicFilterControls(config);
        config.dynamicObjectClusterVoxelSizeM = slamDynamicClusterVoxelSizeSpin->value();
        config.dynamicObjectClusterExtendVoxel = slamDynamicClusterExtendVoxelSpin->value();
        config.dynamicObjectClusterMinVoxelCount = slamDynamicClusterMinVoxelCountSpin->value();
        config.dynamicObjectClusterTrustThreshold = slamDynamicClusterTrustThresholdSpin->value();
        config.dynamicObjectClusterGroundDistanceThresholdM = slamDynamicClusterGroundDistanceSpin->value();
        config.dynamicObjectClusterGroundMaxAngleDeg = slamDynamicClusterGroundMaxAngleSpin->value();
        config.dynamicObjectBufferDelaySec = slamDynamicBufferDelaySpin->value();
        config.dynamicObjectDepthMapDurationSec = slamDynamicDepthMapDurationSpin->value();
        config.dynamicObjectMaxDepthMaps = slamDynamicMaxDepthMapsSpin->value();
        config.dynamicObjectMinHistoryMaps = slamDynamicMinHistoryMapsSpin->value();
        config.dynamicObjectHorizontalResolutionRad = slamDynamicHorizontalResolutionSpin->value();
        config.dynamicObjectVerticalResolutionRad = slamDynamicVerticalResolutionSpin->value();
        config.dynamicObjectVerticalFovDownDeg = slamDynamicVerticalFovDownSpin->value();
        config.dynamicObjectVerticalFovUpDeg = slamDynamicVerticalFovUpSpin->value();
        config.dynamicObjectHorizontalFovRightDeg = slamDynamicHorizontalFovRightSpin->value();
        config.dynamicObjectHorizontalFovLeftDeg = slamDynamicHorizontalFovLeftSpin->value();
        config.dynamicObjectMinRangeM = slamDynamicMinRangeSpin->value();
        config.dynamicObjectMaxRangeM = slamDynamicMaxRangeSpin->value();
        config.dynamicObjectNeighborPixelRadius = slamDynamicNeighborPixelRadiusSpin->value();
        config.dynamicObjectCase1DepthMarginM = slamDynamicCase1DepthMarginSpin->value();
        config.dynamicObjectCase2DepthMarginM = slamDynamicCase2DepthMarginSpin->value();
        config.dynamicObjectCase3DepthMarginM = slamDynamicCase3DepthMarginSpin->value();
        config.dynamicObjectCase1VoteThreshold = slamDynamicCase1VoteThresholdSpin->value();
        config.dynamicObjectCase2OcclusionChainLength =
            slamDynamicCase2OcclusionChainLengthSpin->value();
        config.dynamicObjectCase3OcclusionChainLength =
            slamDynamicCase3OcclusionChainLengthSpin->value();
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
    connect(okButton, &QPushButton::clicked, &dlg,
            [&dlg, freeDomLearnFovCheck, freeDomFovMaskCheck, freeDomFovMaskPathEdit]() {
                if ((freeDomLearnFovCheck->isChecked() || freeDomFovMaskCheck->isChecked()) &&
                    freeDomFovMaskPathEdit->text().trimmed().isEmpty()) {
                    QMessageBox::warning(
                        &dlg,
                        QStringLiteral("FreeDOM 配置无效"),
                        freeDomLearnFovCheck->isChecked()
                            ? QStringLiteral("学习 FOV 需要指定 FOV Mask 输出路径。")
                            : QStringLiteral("启用 FOV Mask 需要指定已有的 Mask 文件。"));
                    return;
                }
                dlg.accept();
            });
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

    addPreferenceSectionTitle(colorPageLayout, "Eye-Dome Lighting");
    QFrame* edlSection = createPreferenceSection(colorTab);
    addPreferenceRow(edlSection, "启用", "使用 CloudCompare 风格的三尺度 Eye-Dome Lighting。", edlEnabledCheck);
    addPreferenceRow(edlSection, "强度", "控制三尺度深度遮蔽的明暗对比。", edlStrengthSpin);
    addPreferenceRow(edlSection, "邻域距离", "设置透视投影的邻域采样距离；正交投影自动使用其 40%。", edlRadiusSpin);
    addPreferenceRow(edlSection, "质量", "调整场景离屏分辨率；每档均使用三尺度与双边滤波。", edlQualityCombo);
    addPreferenceRow(edlSection, "圆形点", "将方形 point sprite 裁剪为圆形，减少块状黑边。", edlRoundPointCheck);
    colorPageLayout->addWidget(edlSection);
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

    CurrentPageHeightTabWidget* slamSettingsTabs = new CurrentPageHeightTabWidget(slamTab);
    slamSettingsTabs->setObjectName(QStringLiteral("SlamSettingsTabs"));
    slamSettingsTabs->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
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
    QWidget* slamLoopClosureTab = createSlamSettingsTab(QStringLiteral("回环优化"));
    QWidget* slamDynamicTab = createSlamSettingsTab(QStringLiteral("动态检测"));
    QWidget* slamVisualTab = createSlamSettingsTab(QStringLiteral("显示样式"));

    QVBoxLayout* slamBackendTabLayout = qobject_cast<QVBoxLayout*>(slamBackendTab->layout());
    QVBoxLayout* slamImuNoiseTabLayout = qobject_cast<QVBoxLayout*>(slamImuNoiseTab->layout());
    QVBoxLayout* slamExtrinsicTabLayout = qobject_cast<QVBoxLayout*>(slamExtrinsicTab->layout());
    QVBoxLayout* slamPublishTabLayout = qobject_cast<QVBoxLayout*>(slamPublishTab->layout());
    QVBoxLayout* slamLoopClosureTabLayout = qobject_cast<QVBoxLayout*>(slamLoopClosureTab->layout());
    QVBoxLayout* slamDynamicTabLayout = qobject_cast<QVBoxLayout*>(slamDynamicTab->layout());
    QVBoxLayout* slamVisualTabLayout = qobject_cast<QVBoxLayout*>(slamVisualTab->layout());

    QFrame* slamBackendSection = createPreferenceSection(slamBackendTab);
    addPreferenceRow(slamBackendSection,
                     "纯激光里程计",
                     "allowPureLidar，无 IMU 离线数据使用 FAST_LO；关闭时保持 FAST_LIO 严格 IMU 校验",
                     slamLidarOnlyCheck);
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
                     "加速度计白噪声协方差",
                     "accCov，IMU 加速度测量值本身的噪声大小；值越大，表示加速度计读数越不可信，滤波器会更依赖激光雷达匹配结果修正状态",
                     slamAccCovSpin);
    addPreferenceRow(slamImuNoiseSection,
                     "陀螺仪白噪声协方差",
                     "gyrCov，IMU 角速度测量值本身的噪声大小；值越大，表示陀螺仪读数越不可信，滤波器会更依赖激光雷达匹配结果修正状态",
                     slamGyrCovSpin);
    addPreferenceRow(slamImuNoiseSection,
                     "加速度计零偏随机游走协方差",
                     "bAccCov，加速度计零偏随时间变化的快慢程度；值越大，表示零偏变化越快，滤波器会更频繁地估计和更新零偏",
                     slamBAccCovSpin);
    addPreferenceRow(slamImuNoiseSection,
                     "陀螺仪零偏随机游走协方差",
                     "bGyrCov，陀螺仪零偏随时间变化的快慢程度；值越大，表示零偏变化越快，滤波器会更频繁地估计和更新零偏",
                     slamBGyrCovSpin);
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

    QFrame* slamKeyframeSection = createPreferenceSection(slamLoopClosureTab);
    addPreferenceRow(slamKeyframeSection,
                     "关键帧距离阈值",
                     "surroundingKeyframeAddingDistThreshold，当前位置相对上一关键帧的平移达到该值时保存新关键帧；小场景需要适当减小",
                     slamKeyframeDistanceSpin);
    addPreferenceRow(slamKeyframeSection,
                     "关键帧角度阈值",
                     "surroundingKeyframeAddingAngleThreshold，当前位置相对上一关键帧的任一欧拉角变化达到该值时保存新关键帧",
                     slamKeyframeAngleSpin);
    slamLoopClosureTabLayout->addWidget(slamKeyframeSection);

    QFrame* slamLoopClosureSection = createPreferenceSection(slamLoopClosureTab);
    addPreferenceRow(slamLoopClosureSection,
                     "启用回环检测",
                     "loopClosureEnableFlag，启动独立回环检测线程；新设置在下一次开始 SLAM 建图时生效",
                     slamLoopClosureCheck);
    addPreferenceRow(slamLoopClosureSection,
                     "检测频率",
                     "loopClosureFrequency，每秒执行回环候选搜索和 ICP 匹配的次数",
                     slamLoopClosureFrequencySpin);
    addPreferenceRow(slamLoopClosureSection,
                     "历史搜索半径",
                     "historyKeyframeSearchRadius，在优化关键帧轨迹中搜索历史回环候选的空间半径",
                     slamHistoryKeyframeSearchRadiusSpin);
    addPreferenceRow(slamLoopClosureSection,
                     "最小时间间隔",
                     "historyKeyframeSearchTimeDiff，当前关键帧与历史候选必须相隔的最短时间；回到旧位置前需超过此时间",
                     slamHistoryKeyframeSearchTimeDiffSpin);
    addPreferenceRow(slamLoopClosureSection,
                     "历史子地图帧数",
                     "historyKeyframeSearchNum，历史候选前后参与 ICP 子地图拼接的关键帧数量",
                     slamHistoryKeyframeSearchNumSpin);
    addPreferenceRow(slamLoopClosureSection,
                     "ICP 适配度阈值",
                     "historyKeyframeFitnessScore，ICP 适配度分数必须小于该值才接受回环约束，值越小越严格",
                     slamHistoryKeyframeFitnessScoreSpin);
    addPreferenceRow(slamLoopClosureSection,
                     "回环后重建 ikd-tree",
                     "reconstructKdTree，回环优化历史位姿后使用优化关键帧重新构建 FAST-LIO 局部地图",
                     slamReconstructKdTreeCheck);
    slamLoopClosureTabLayout->addWidget(slamLoopClosureSection);
    slamLoopClosureTabLayout->addStretch();

    auto addDynamicPreferenceRow = [](QFrame* section,
                                      const QString& title,
                                      const QString& description,
                                      QWidget* control,
                                      const QString& tuningGuide) {
        addPreferenceRow(section,
                         title,
                         description,
                         control,
                         QStringLiteral("<div style=\"width: 480px;\"><b>%1</b><br/>%2<br/><br/>"
                                        "<b>调参建议：</b>%3</div>")
                             .arg(title.toHtmlEscaped(),
                                  description.toHtmlEscaped(),
                                  tuningGuide.toHtmlEscaped()));
    };

    QFrame* slamDynamicEnableSection = createPreferenceSection(slamDynamicTab);
    addDynamicPreferenceRow(slamDynamicEnableSection,
                     "启用动态检测",
                     "dynamicFilterEnabled，启用所选后端并对去畸变 LiDAR 当前帧生成逐点标签",
                     slamDynamicDetectionCheck,
                     "建议先关闭聚类增强，仅观察 Case1/2/3 原始点并校准投影、历史窗口和深度阈值。启用后会自动显示不累计的世界系当前帧背景，不需要开启世界系或机体系点云输出。");
    slamDynamicTabLayout->addWidget(slamDynamicEnableSection);

    QFrame* slamDynamicSettingsSection = createPreferenceSection(slamDynamicTab);
    addDynamicPreferenceRow(slamDynamicSettingsSection,
                     "算法",
                     "dynamicFilterBackend，同一 SLAM 会话只运行一个动态滤除后端",
                     slamDynamicBackendCombo,
                     "M-detector 与 FreeDOM 的历史状态相互隔离。切换后需重启或重置 SLAM 会话。");
    addDynamicPreferenceRow(slamDynamicSettingsSection,
                     "去除动态点云",
                     "dynamicPointRemovalEnabled，根据统一 dynamic 标签从当前帧输出和 SLAM 增量地图中剔除动态点",
                     slamDynamicRemovalCheck,
                     "启用动态检测并确认动态目标识别稳定后再开启。误检会删除真实静态结构，因此应先校准深度阈值和历史窗口；开启聚类增强可让被剔除的目标轮廓更完整。");
    slamDynamicTabLayout->addWidget(slamDynamicSettingsSection);

    QStackedWidget* slamDynamicParameterStack = new QStackedWidget(slamDynamicTab);
    QWidget* slamMDetectorPage = new QWidget(slamDynamicParameterStack);
    QVBoxLayout* slamMDetectorLayout = new QVBoxLayout(slamMDetectorPage);
    slamMDetectorLayout->setContentsMargins(0, 0, 0, 0);
    slamMDetectorLayout->setSpacing(10);
    QWidget* slamFreeDomPage = new QWidget(slamDynamicParameterStack);
    QVBoxLayout* slamFreeDomLayout = new QVBoxLayout(slamFreeDomPage);
    slamFreeDomLayout->setContentsMargins(0, 0, 0, 0);
    slamFreeDomLayout->setSpacing(10);
    slamDynamicParameterStack->addWidget(slamMDetectorPage);
    slamDynamicParameterStack->addWidget(slamFreeDomPage);

    QFrame* slamDynamicClusterSection = createPreferenceSection(slamMDetectorPage);
    addDynamicPreferenceRow(slamDynamicClusterSection,
                     "启用聚类增强",
                     "dynamicObjectClusterEnabled，以 Case1/2/3 事件点为种子回填对象区域，并执行地面和孤立点过滤",
                     slamDynamicClusterCheck,
                     "先确保无聚类模式的事件点稳定，再开启聚类。开启后目标轮廓更完整，但会增加聚类耗时；若相邻目标被合并，优先减小体素尺寸或连接半径。");
    addDynamicPreferenceRow(slamDynamicClusterSection,
                     "聚类体素尺寸",
                     "dynamicObjectClusterVoxelSizeM，事件点和原始点的稀疏体素边长",
                     slamDynamicClusterVoxelSizeSpin,
                     "减小可保留小目标和边缘细节，但体素数量与耗时会上升且区域更易碎裂；增大可抑制稀疏噪声，但可能合并相邻目标。建议从局部点间距的 1~2 倍开始。");
    addDynamicPreferenceRow(slamDynamicClusterSection,
                     "体素连接半径",
                     "dynamicObjectClusterExtendVoxel，体素连通域搜索的整数半径",
                     slamDynamicClusterExtendVoxelSpin,
                     "增大可跨越遮挡和稀疏间隙，目标回填更完整，但更容易把邻近物体连成一体；减小可分离近距离目标，但可能切碎单个目标。通常在 2~5 之间微调。");
    addDynamicPreferenceRow(slamDynamicClusterSection,
                     "最小事件体素数",
                     "dynamicObjectClusterMinVoxelCount，小于该事件体素数量的连通域会被过滤",
                     slamDynamicClusterMinVoxelCountSpin,
                     "误检呈零散小块时增大；小目标或远距离目标被过滤时减小。应结合体素尺寸调节，体素越大，同一目标包含的事件体素通常越少。");
    addDynamicPreferenceRow(slamDynamicClusterSection,
                     "聚类可信度",
                     "dynamicObjectClusterTrustThreshold，事件种子点数与最终回填点数的最小比例",
                     slamDynamicClusterTrustThresholdSpin,
                     "增大可拒绝只有少量事件种子却回填很大区域的可疑聚类，误检更少但召回下降；减小可保留部分遮挡目标。建议先在 0.05~0.20 范围内调整。");
    addDynamicPreferenceRow(slamDynamicClusterSection,
                     "地面距离阈值",
                     "dynamicObjectClusterGroundDistanceThresholdM，点到估计地面小于该距离时从动态区域删除",
                     slamDynamicClusterGroundDistanceSpin,
                     "地面残留较多时增大；低矮目标、轮胎或脚部被削掉时减小。该值应略大于地面拟合噪声，不宜用来补偿明显错误的外参或姿态。");
    addDynamicPreferenceRow(slamDynamicClusterSection,
                     "地面最大倾角",
                     "dynamicObjectClusterGroundMaxAngleDeg，候选地面法向相对 IMU 世界竖直轴允许的最大夹角",
                     slamDynamicClusterGroundMaxAngleSpin,
                     "坡道地面无法剔除时增大；墙面或目标表面被误当作地面时减小。平整场景可用较小角度，坡地场景再逐步放宽。");
    slamMDetectorLayout->addWidget(slamDynamicClusterSection);

    QFrame* slamDynamicHistorySection = createPreferenceSection(slamDynamicTab);
    addDynamicPreferenceRow(slamDynamicHistorySection,
                     "缓冲延迟",
                     "dynamicObjectBufferDelaySec，当前帧经过该延迟后才进入历史深度图，避免过近时间片参与比较",
                     slamDynamicBufferDelaySpin,
                     "增大可避免相邻帧过于相似造成的自比较，并扩大可观察位移，但检测预热更慢、历史更新滞后；减小响应更快。建议至少覆盖 1 个输入帧周期，再按目标速度调整。");
    addDynamicPreferenceRow(slamDynamicHistorySection,
                     "深度图时间片",
                     "dynamicObjectDepthMapDurationSec，每张历史深度图聚合的时间跨度",
                     slamDynamicDepthMapDurationSpin,
                     "增大可提高单张图的覆盖密度、降低空洞，但运动目标会在同一图内拖影；减小时间分辨率更高，但历史图更稀疏。高速目标优先减小，稀疏点云可适当增大。");
    addDynamicPreferenceRow(slamDynamicHistorySection,
                     "最大历史图数",
                     "dynamicObjectMaxDepthMaps，保留的历史深度图时间片数量",
                     slamDynamicMaxDepthMapsSpin,
                     "增大可获得更长时间基线和更多 Case1 投票样本，但内存、比较耗时及旧场景残留都会增加；减小响应场景变化更快。历史跨度约为该值乘以深度图时间片。");
    addDynamicPreferenceRow(slamDynamicHistorySection,
                     "最少历史图数",
                     "dynamicObjectMinHistoryMaps，达到该历史深度图数量后才开始输出动态判定",
                     slamDynamicMinHistoryMapsSpin,
                     "增大可让背景建立更充分、降低启动阶段误检，但首次输出更晚；减小可更快开始检测。该值不能超过最大历史图数，通常取 2~3。");
    slamMDetectorLayout->addWidget(slamDynamicHistorySection);

    QFrame* slamDynamicProjectionSection = createPreferenceSection(slamDynamicTab);
    addDynamicPreferenceRow(slamDynamicProjectionSection,
                     "水平角分辨率",
                     "dynamicObjectHorizontalResolutionRad，球面深度图水平像素角度，越小越精细且内存占用越高",
                     slamDynamicHorizontalResolutionSpin,
                     "减小可区分角度接近的目标和背景，但深度图更大、耗时和内存上升；增大可降耗并容忍轻微姿态误差，但细小目标可能落入同一像素。优先使用模板默认值。");
    addDynamicPreferenceRow(slamDynamicProjectionSection,
                     "垂直角分辨率",
                     "dynamicObjectVerticalResolutionRad，球面深度图垂直像素角度，越小越精细且内存占用越高",
                     slamDynamicVerticalResolutionSpin,
                     "减小可保留更多垂直结构细节，但增加深度图高度和计算量；增大可抑制稀疏扫描线空洞，但可能混合地面与目标。通常不应明显小于雷达实际垂直角间隔。");
    addDynamicPreferenceRow(slamDynamicProjectionSection,
                     "垂直视场下界",
                     "dynamicObjectVerticalFovDownDeg，低于该仰角的点不参与动态检测",
                     slamDynamicVerticalFovDownSpin,
                     "按雷达安装姿态和有效视场设置。提高下界可排除近场地面并降低计算量，但会漏掉低矮目标；降低下界可保留更多地面附近目标。必须小于上界。");
    addDynamicPreferenceRow(slamDynamicProjectionSection,
                     "垂直视场上界",
                     "dynamicObjectVerticalFovUpDeg，高于该仰角的点不参与动态检测",
                     slamDynamicVerticalFovUpSpin,
                     "按雷达实际有效视场设置。降低上界可去除天空、顶棚等无关区域并节省内存，但可能漏掉高处目标；提高上界会扩大检测区域。必须大于下界。");
    addDynamicPreferenceRow(slamDynamicProjectionSection,
                     "水平视场右界",
                     "dynamicObjectHorizontalFovRightDeg，水平视场的负角度边界，Avia 默认 -34°",
                     slamDynamicHorizontalFovRightSpin,
                     "与雷达实际水平视场和安装朝向一致。向 0° 收窄可排除无效侧后方点，向 -180° 放宽可覆盖更多区域；设置过窄会直接漏检边缘目标。必须小于左界。");
    addDynamicPreferenceRow(slamDynamicProjectionSection,
                     "水平视场左界",
                     "dynamicObjectHorizontalFovLeftDeg，水平视场的正角度边界，Avia 默认 34°",
                     slamDynamicHorizontalFovLeftSpin,
                     "与右界共同定义有效水平区域。向 0° 收窄可减少无效点，向 180° 放宽可覆盖全周；边缘目标被截断时应放宽。必须大于右界。");
    addDynamicPreferenceRow(slamDynamicProjectionSection,
                     "检测最近距离",
                     "dynamicObjectMinRangeM，动态检测独立近距离盲区，不复用 FAST_LIO 匹配盲区",
                     slamDynamicMinRangeSpin,
                     "近场噪声、机身或自反射产生误检时增大；需要检测贴近雷达的目标时减小。不要低于雷达可靠测距下限，并保持小于最远距离。");
    addDynamicPreferenceRow(slamDynamicProjectionSection,
                     "检测最远距离",
                     "dynamicObjectMaxRangeM，超出该距离的点不写入动态检测深度图",
                     slamDynamicMaxRangeSpin,
                     "减小可显著降低远距离稀疏噪声和计算量；增大可检测更远目标，但深度误差与配准误差更容易触发误检。建议按业务所需最远距离设置，而非直接使用雷达量程上限。");
    addDynamicPreferenceRow(slamDynamicProjectionSection,
                     "邻域像素半径",
                     "dynamicObjectNeighborPixelRadius，深度比较时在投影像素周围搜索的像素半径",
                     slamDynamicNeighborPixelRadiusSpin,
                     "增大可容忍姿态误差、扫描线错位和投影抖动，减少历史空洞，但会跨物体边界比较并增加耗时；减小边界更锐利但更易漏匹配。通常从 1 开始。");
    slamMDetectorLayout->addWidget(slamDynamicProjectionSection);

    QFrame* slamDynamicCaseSection = createPreferenceSection(slamDynamicTab);
    addDynamicPreferenceRow(slamDynamicCaseSection,
                     "Case1 深度阈值",
                     "dynamicObjectCase1DepthMarginM，当前点比历史静态背景更近时所需的最小深度差",
                     slamDynamicCase1DepthMarginSpin,
                     "误把配准抖动或表面噪声判为新前景时增大；慢速、小尺寸或远距离目标漏检时减小。应高于该距离段的测距噪声与位姿误差总量。");
    addDynamicPreferenceRow(slamDynamicCaseSection,
                     "Case1 投票阈值",
                     "dynamicObjectCase1VoteThreshold，判定 Case1 所需的历史深度图命中数",
                     slamDynamicCase1VoteThresholdSpin,
                     "增大要求更多历史图一致支持，误检更少但启动更慢、短时目标可能漏检；减小响应更快但对历史噪声更敏感。建议不高于常态下可用历史图数。");
    addDynamicPreferenceRow(slamDynamicCaseSection,
                     "Case2 深度阈值",
                     "dynamicObjectCase2DepthMarginM，当前点比历史最远深度更远时所需的最小深度差",
                     slamDynamicCase2DepthMarginSpin,
                     "Case2 远侧显露误检较多时增大；离开视线的目标尾部漏检时减小。阈值还受时间差与最小速度约束，宜与帧率和目标速度一起评估。");
    addDynamicPreferenceRow(slamDynamicCaseSection,
                     "Case2 遮挡链长度",
                     "dynamicObjectCase2OcclusionChainLength，Case2 需连续通过一致性、速度和加速度检查的遮挡段数",
                     slamDynamicCase2OcclusionChainLengthSpin,
                     "增大可强化时序一致性并降低偶发误检，但检测延迟增加且短轨迹目标易漏；减小响应更快。高速或短时目标可适当减小，静态场景误检多时增大。");
    addDynamicPreferenceRow(slamDynamicCaseSection,
                     "Case3 深度阈值",
                     "dynamicObjectCase3DepthMarginM，当前点比历史最近深度更近时所需的最小深度差",
                     slamDynamicCase3DepthMarginSpin,
                     "Case3 新遮挡误检较多时增大；前景目标刚进入视线时漏检则减小。近距离点密集区域可略小，远距离或位姿噪声较大时应提高。");
    addDynamicPreferenceRow(slamDynamicCaseSection,
                     "Case3 遮挡链长度",
                     "dynamicObjectCase3OcclusionChainLength，Case3 需连续通过一致性、速度和加速度检查的遮挡段数",
                     slamDynamicCase3OcclusionChainLengthSpin,
                     "增大可减少单帧遮挡变化造成的误检，但会增加确认延迟；减小可更早发现进入视线的目标。建议先用 3，再根据误检和响应时间成对权衡。");
    slamMDetectorLayout->addWidget(slamDynamicCaseSection);
    slamMDetectorLayout->addStretch();

    QFrame* freeDomGeometrySection = createPreferenceSection(slamFreeDomPage);
    addDynamicPreferenceRow(freeDomGeometrySection, "传感器最近距离", "freeDom/sensorMinRangeM", freeDomSensorMinRangeSpin, "小于该距离的输入点标记为 Invalid，不进入 FreeDOM ScanMap。");
    addDynamicPreferenceRow(freeDomGeometrySection, "传感器最远距离", "freeDom/sensorMaxRangeM", freeDomSensorMaxRangeSpin, "应覆盖业务有效范围，并与设备模板和 Raycast 范围一致。");
    addDynamicPreferenceRow(freeDomGeometrySection, "传感器 Z 下界", "freeDom/sensorMinZM，相对传感器高度", freeDomSensorMinZSpin, "限制参与 ScanMap 的垂直范围。");
    addDynamicPreferenceRow(freeDomGeometrySection, "传感器 Z 上界", "freeDom/sensorMaxZM，相对传感器高度", freeDomSensorMaxZSpin, "必须大于 Z 下界。");
    addDynamicPreferenceRow(freeDomGeometrySection, "Sub-voxel 尺寸", "freeDom/subVoxelSizeM", freeDomSubVoxelSizeSpin, "决定静态地图最小保存分辨率；修改后必须重置 SLAM 会话。");
    addDynamicPreferenceRow(freeDomGeometrySection, "Voxel 深度", "freeDom/voxelDepth", freeDomVoxelDepthSpin, "voxelSize = subVoxelSize × 2^voxelDepth。");
    addDynamicPreferenceRow(freeDomGeometrySection, "Block 深度", "freeDom/blockDepth", freeDomBlockDepthSpin, "必须不小于 Voxel 深度；修改后必须重置会话。");
    slamFreeDomLayout->addWidget(freeDomGeometrySection);

    QFrame* freeDomMapSection = createPreferenceSection(slamFreeDomPage);
    addDynamicPreferenceRow(freeDomMapSection, "启用局部地图", "freeDom/localMapEnabled", freeDomLocalMapCheck, "使用 FreeDOM 原生 remove_map_out_of_bound 控制内存，不会随机丢弃地图。");
    addDynamicPreferenceRow(freeDomMapSection, "局部地图范围", "freeDom/localMapRangeM", freeDomLocalMapRangeSpin, "关闭局部地图时 FreeDOM 的持久地图会随路程增长。");
    addDynamicPreferenceRow(freeDomMapSection, "局部地图 Z 下界", "freeDom/localMapMinZM", freeDomLocalMapMinZSpin, "相对移动中心的局部地图垂直下界。");
    addDynamicPreferenceRow(freeDomMapSection, "局部地图 Z 上界", "freeDom/localMapMaxZM", freeDomLocalMapMaxZSpin, "必须大于局部地图 Z 下界。");
    addDynamicPreferenceRow(freeDomMapSection, "Raycast 最远距离", "freeDom/raycastMaxRangeM", freeDomRaycastMaxRangeSpin, "限制自由空间射线长度。");
    addDynamicPreferenceRow(freeDomMapSection, "Raycast Z 下界", "freeDom/raycastMinZM", freeDomRaycastMinZSpin, "相对传感器高度的射线下界。");
    addDynamicPreferenceRow(freeDomMapSection, "Raycast Z 上界", "freeDom/raycastMaxZM", freeDomRaycastMaxZSpin, "必须大于 Raycast Z 下界。");
    addDynamicPreferenceRow(freeDomMapSection, "确认自由次数", "freeDom/countsToFree", freeDomCountsToFreeSpin, "连续自由观测达到该次数并满足邻域条件后确认自由空间。");
    addDynamicPreferenceRow(freeDomMapSection, "撤销自由次数", "freeDom/countsToRevert", freeDomCountsToRevertSpin, "持续重新占据达到该次数后撤销自由状态。");
    addDynamicPreferenceRow(freeDomMapSection, "保守连通性", "freeDom/conservativeConnectivity", freeDomConservativeConnectivityCombo, "支持 6、18、26 邻域。");
    addDynamicPreferenceRow(freeDomMapSection, "激进连通性", "freeDom/aggressiveConnectivity", freeDomAggressiveConnectivityCombo, "支持 6、18、26、80、124；124 对应上游两次 26 邻域传播。");
    slamFreeDomLayout->addWidget(freeDomMapSection);

    QFrame* freeDomEnhancementSection = createPreferenceSection(slamFreeDomPage);
    addDynamicPreferenceRow(freeDomEnhancementSection, "Raycast Enhancement", "freeDom/raycastEnhancementEnabled", freeDomRaycastEnhancementCheck, "完整运行球面深度图、腐蚀、Telea Inpaint、区域过滤和增强射线。");
    addDynamicPreferenceRow(freeDomEnhancementSection, "水平 FOV", "freeDom/lidarHorizontalFovDeg", freeDomHorizontalFovSpin, "Mid-360 模板为 360°，Avia 模板使用设备有效水平视场。");
    addDynamicPreferenceRow(freeDomEnhancementSection, "垂直 FOV 下界", "freeDom/lidarVerticalFovLowerDeg", freeDomVerticalFovLowerSpin, "Mid-360 模板为 -7°。");
    addDynamicPreferenceRow(freeDomEnhancementSection, "垂直 FOV 上界", "freeDom/lidarVerticalFovUpperDeg", freeDomVerticalFovUpperSpin, "Mid-360 模板为 52°。");
    addDynamicPreferenceRow(freeDomEnhancementSection, "深度图行数", "freeDom/depthImageVerticalLines", freeDomDepthLinesSpin, "这是球面深度图离散行数，不是 Mid-360 物理扫描线数。");
    addDynamicPreferenceRow(freeDomEnhancementSection, "深度图最近距离", "freeDom/depthImageMinRangeM", freeDomDepthMinRangeSpin, "过滤近距离鬼影。");
    addDynamicPreferenceRow(freeDomEnhancementSection, "增强最远距离", "freeDom/maxRaycastEnhancementRangeM", freeDomEnhancementMaxRangeSpin, "决定 8 位深度量化范围和增强射线最大长度。");
    addDynamicPreferenceRow(freeDomEnhancementSection, "深度安全边距", "freeDom/raycastEnhancementDepthMarginM", freeDomDepthMarginSpin, "对应论文 safety margin。");
    addDynamicPreferenceRow(freeDomEnhancementSection, "Inpaint 尺寸", "freeDom/inpaintSize", freeDomInpaintSizeSpin, "OpenCV Telea 补全半径。");
    addDynamicPreferenceRow(freeDomEnhancementSection, "腐蚀尺寸", "freeDom/erosionSize", freeDomErosionSizeSpin, "用于深度安全腐蚀和增强区域腐蚀。");
    addDynamicPreferenceRow(freeDomEnhancementSection, "最小增强区域", "freeDom/minRaycastEnhancementArea", freeDomMinEnhancementAreaSpin, "以深度图总面积比例过滤小连通域。");
    addDynamicPreferenceRow(freeDomEnhancementSection, "顶部边距", "freeDom/depthImageTopMargin", freeDomTopMarginSpin, "限制用于增强的顶部区域比例。");
    addDynamicPreferenceRow(freeDomEnhancementSection, "学习 FOV", "freeDom/learnFov", freeDomLearnFovCheck, "学习模式不输出增强点；停止或重置 SLAM 时写入 FOV Mask。");
    addDynamicPreferenceRow(freeDomEnhancementSection, "启用 FOV Mask", "freeDom/fovMaskEnabled", freeDomFovMaskCheck, "处理非矩形 FOV 或固定遮挡。");
    QWidget* freeDomFovMaskPathRow = new QWidget(slamFreeDomPage);
    QHBoxLayout* freeDomFovMaskPathLayout = new QHBoxLayout(freeDomFovMaskPathRow);
    freeDomFovMaskPathLayout->setContentsMargins(0, 0, 0, 0);
    QPushButton* freeDomFovMaskBrowseButton = new QPushButton(QStringLiteral("浏览..."), freeDomFovMaskPathRow);
    freeDomFovMaskPathLayout->addWidget(freeDomFovMaskPathEdit, 1);
    freeDomFovMaskPathLayout->addWidget(freeDomFovMaskBrowseButton);
    auto syncFreeDomFovBrowseButtonText = [freeDomLearnFovCheck,
                                           freeDomFovMaskBrowseButton]() {
        freeDomFovMaskBrowseButton->setText(
            freeDomLearnFovCheck->isChecked()
                ? QStringLiteral("选择输出位置...")
                : QStringLiteral("浏览..."));
    };
    connect(freeDomLearnFovCheck, &QCheckBox::toggled, &dlg,
            [syncFreeDomFovBrowseButtonText](bool) {
                syncFreeDomFovBrowseButtonText();
            });
    syncFreeDomFovBrowseButtonText();
    connect(freeDomFovMaskBrowseButton, &QPushButton::clicked, &dlg,
            [&dlg, freeDomLearnFovCheck, freeDomFovMaskPathEdit]() {
        const QString currentPath = freeDomFovMaskPathEdit->text().trimmed();
        const QString path = freeDomLearnFovCheck->isChecked()
            ? QFileDialog::getSaveFileName(
                  &dlg,
                  QStringLiteral("保存 FreeDOM FOV Mask"),
                  currentPath.isEmpty()
                      ? QDir(QDir::homePath()).filePath(QStringLiteral("freedom_fov_mask.png"))
                      : currentPath,
                  QStringLiteral("PNG 图像 (*.png)"))
            : QFileDialog::getOpenFileName(
                  &dlg,
                  QStringLiteral("选择 FreeDOM FOV Mask"),
                  currentPath,
                  QStringLiteral("PNG 图像 (*.png)"));
        if(!path.isEmpty())
            freeDomFovMaskPathEdit->setText(path);
    });
    addDynamicPreferenceRow(freeDomEnhancementSection, "FOV Mask 路径", "freeDom/fovMaskPath", freeDomFovMaskPathRow, "Mask 尺寸必须与不含全景 margin 的深度图一致。");
    slamFreeDomLayout->addWidget(freeDomEnhancementSection);

    QFrame* freeDomRuntimeSection = createPreferenceSection(slamFreeDomPage);
    addDynamicPreferenceRow(freeDomRuntimeSection, "显示算法调试图层", "dynamicDebugVisualizationEnabled，按节流频率复制 FreeDOM Scan/Free/Static/Raycast 快照", slamDynamicDebugCheck, "调试快照会复制体素和深度图数据。只在调参时开启，并用下方快照间隔控制开销；关闭后 SLAM Dock 不显示相关调试卡片。");
    addDynamicPreferenceRow(freeDomRuntimeSection, "算法线程数", "freeDom/numThreads", freeDomThreadsSpin, "独立于 FAST_LIO/OpenMP 线程数，避免 CPU 过度订阅。");
    addDynamicPreferenceRow(freeDomRuntimeSection, "调试快照间隔", "dynamicDebugSnapshotIntervalFrames", freeDomDebugIntervalSpin, "按帧节流 Scan/Free/Static/Raycast 和深度图快照。");
    addDynamicPreferenceRow(freeDomRuntimeSection, "地图快照间隔", "freeDomMapSnapshotIntervalFrames", freeDomMapIntervalSpin, "按帧节流完整静态点/体素地图快照，用于显示和导出。");
    slamFreeDomLayout->addWidget(freeDomRuntimeSection);
    slamFreeDomLayout->addStretch();

    auto syncSlamDynamicControls = [slamDynamicDetectionCheck,
                                    slamDynamicBackendCombo,
                                    slamDynamicSettingsSection,
                                    slamDynamicParameterStack,
                                    slamDynamicClusterCheck,
                                    slamDynamicClusterControls,
                                    slamSettingsTabs]() {
        const bool detectionEnabled = slamDynamicDetectionCheck->isChecked();
        const bool freeDomSelected =
            slamDynamicBackendCombo->currentData().toInt() ==
            int(DynamicFilterBackend::FreeDOM);
        slamDynamicSettingsSection->setVisible(detectionEnabled);
        slamDynamicParameterStack->setVisible(detectionEnabled);
        slamDynamicParameterStack->setCurrentIndex(freeDomSelected ? 1 : 0);
        const bool clusterEnabled =
            detectionEnabled && !freeDomSelected &&
            slamDynamicClusterCheck->isChecked();
        for (QWidget* control : slamDynamicClusterControls) {
            control->setEnabled(clusterEnabled);
        }
        QTimer::singleShot(0, slamSettingsTabs, [slamSettingsTabs]() {
            slamSettingsTabs->syncCurrentPageHeight();
        });
    };
    connect(slamDynamicDetectionCheck, &QCheckBox::toggled, &dlg,
            [syncSlamDynamicControls](bool) { syncSlamDynamicControls(); });
    connect(slamDynamicBackendCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            &dlg,
            [syncSlamDynamicControls](int) { syncSlamDynamicControls(); });
    connect(slamDynamicClusterCheck, &QCheckBox::toggled, &dlg,
            [syncSlamDynamicControls](bool) { syncSlamDynamicControls(); });
    syncSlamDynamicControls();
    slamDynamicTabLayout->addWidget(slamDynamicParameterStack);
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
                     "M-detector 动态点颜色",
                     "slamDynamicObjectColor，M-detector Case1/2/3 检出的动态点颜色",
                     slamDynamicObjectColorRow);
    addPreferenceRow(slamVisualSection,
                     "动态点大小",
                     "slamDynamicObjectPointSizePx，M-detector 动态点和 FreeDOM 三级动态点共用的显示点大小",
                     slamDynamicObjectPointSizeSpin);
    addPreferenceRow(slamVisualSection,
                     "Aggressive 动态点颜色",
                     "FreeDOM Aggressive 等级动态点，默认红色",
                     slamDynamicAggressiveColorRow);
    addPreferenceRow(slamVisualSection,
                     "Moderate 动态点颜色",
                     "FreeDOM Moderate 等级动态点，默认橙色",
                     slamDynamicModerateColorRow);
    addPreferenceRow(slamVisualSection,
                     "Conservative 动态点颜色",
                     "FreeDOM Conservative 等级动态点，默认紫色",
                     slamDynamicConservativeColorRow);
    addPreferenceRow(slamVisualSection,
                     "Scan Voxel 颜色",
                     "当前帧扫描体素颜色",
                     slamFreeDomScanVoxelColorRow);
    addPreferenceRow(slamVisualSection,
                     "Scan Voxel 点大小",
                     "当前帧扫描体素点大小",
                     slamFreeDomScanVoxelPointSizeSpin);
    addPreferenceRow(slamVisualSection,
                     "Dynamic Voxel 颜色",
                     "当前帧动态体素颜色",
                     slamFreeDomDynamicVoxelColorRow);
    addPreferenceRow(slamVisualSection,
                     "Dynamic Voxel 点大小",
                     "当前帧动态体素点大小",
                     slamFreeDomDynamicVoxelPointSizeSpin);
    addPreferenceRow(slamVisualSection,
                     "Raycasted Voxel 颜色",
                     "射线遍历体素颜色",
                     slamFreeDomRaycastedVoxelColorRow);
    addPreferenceRow(slamVisualSection,
                     "Raycasted Voxel 点大小",
                     "射线遍历体素点大小",
                     slamFreeDomRaycastedVoxelPointSizeSpin);
    addPreferenceRow(slamVisualSection,
                     "Free Voxel 颜色",
                     "持久化自由空间体素颜色",
                     slamFreeDomFreeVoxelColorRow);
    addPreferenceRow(slamVisualSection,
                     "Free Voxel 点大小",
                     "持久化自由空间体素点大小",
                     slamFreeDomFreeVoxelPointSizeSpin);
    addPreferenceRow(slamVisualSection,
                     "Static Voxel 颜色",
                     "FreeDOM 静态体素地图颜色",
                     slamFreeDomStaticVoxelColorRow);
    addPreferenceRow(slamVisualSection,
                     "Static Voxel 点大小",
                     "FreeDOM 静态体素地图点大小",
                     slamFreeDomStaticVoxelPointSizeSpin);
    addPreferenceRow(slamVisualSection,
                     "Enhanced Point 颜色",
                     "Raycast Enhancement 生成点颜色",
                     slamFreeDomEnhancedColorRow);
    addPreferenceRow(slamVisualSection,
                     "Enhanced Point 点大小",
                     "Raycast Enhancement 生成点大小",
                     slamFreeDomEnhancedPointSizeSpin);
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
    slamSettingsTabs->syncCurrentPageHeight();
    slamLayout->addStretch();

    const QStringList navigationNames = {"主题", "连接", "网格", "图例", "着色", "背景", "后端"};
    const QStringList navigationIcons = {
        ":/icons/settings_theme.svg",
        ":/icons/settings_connection.svg",
        ":/icons/settings_grid.svg",
        ":/icons/settings_legend.svg",
        ":/icons/settings_color.svg",
        ":/icons/settings_background.svg",
        ":/icons/settings_slam.svg"
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
        if (themeMode != originalThemeMode) {
            themeMode = originalThemeMode;
            applyUiTheme();
        }
        return;
    }

    const bool previousAutoConfigHostIpEnabled = autoConfigHostIpEnabled;
    const PointCloudView::GridConfig previousGridConfig = pointCloudView->gridConfig();
    const float previousDistanceLegendMin = distanceLegendMin;
    const float previousDistanceLegendMax = distanceLegendMax;
    const float previousElevationLegendMin = elevationLegendMin;
    const float previousElevationLegendMax = elevationLegendMax;
    const int previousReflectivityColorScale = reflectivityColorScale;
    const QColor previousSolidColor = solidColor;
    const QVector<QColor> previousLineColors = lineColors;
    const int previousPointCloudBackgroundPreset = pointCloudBackgroundPreset;

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
    pointCloudEdlConfig.enabled = edlEnabledCheck->isChecked();
    pointCloudEdlConfig.strength = float(edlStrengthSpin->value());
    pointCloudEdlConfig.radiusPx = float(edlRadiusSpin->value());
    pointCloudEdlConfig.roundPointSplat = edlRoundPointCheck->isChecked();
    switch (edlQualityCombo->currentData().toInt()) {
    case 0:
        pointCloudEdlConfig.renderScale = 0.5f;
        break;
    case 1:
        pointCloudEdlConfig.renderScale = 0.75f;
        break;
    default:
        pointCloudEdlConfig.renderScale = 1.0f;
        break;
    }
    themeMode = themeGroup->checkedId();
    const SlamRuntimeConfig previousSlamRuntimeConfig = slamRuntimeConfig;
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
    const bool previousSlamDynamicDetection = slamRuntimeConfig.dynamicFilterEnabled;
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
    const QColor previousSlamDynamicObjectColor = slamDynamicObjectColor;
    const QColor previousSlamDynamicAggressiveColor = slamDynamicAggressiveColor;
    const QColor previousSlamDynamicModerateColor = slamDynamicModerateColor;
    const QColor previousSlamDynamicConservativeColor = slamDynamicConservativeColor;
    const QColor previousSlamFreeDomScanVoxelColor = slamFreeDomScanVoxelColor;
    const QColor previousSlamFreeDomDynamicVoxelColor = slamFreeDomDynamicVoxelColor;
    const QColor previousSlamFreeDomRaycastedVoxelColor = slamFreeDomRaycastedVoxelColor;
    const QColor previousSlamFreeDomFreeVoxelColor = slamFreeDomFreeVoxelColor;
    const QColor previousSlamFreeDomStaticVoxelColor = slamFreeDomStaticVoxelColor;
    const QColor previousSlamFreeDomEnhancedColor = slamFreeDomEnhancedColor;
    const QColor previousSlamTrajectoryColor = slamTrajectoryColor;
    const float previousSlamWorldCurrentFramePointSizePx = slamWorldCurrentFramePointSizePx;
    const float previousSlamBodyFramePointSizePx = slamBodyFramePointSizePx;
    const float previousSlamDynamicObjectPointSizePx = slamDynamicObjectPointSizePx;
    const float previousSlamFreeDomScanVoxelPointSizePx = slamFreeDomScanVoxelPointSizePx;
    const float previousSlamFreeDomDynamicVoxelPointSizePx = slamFreeDomDynamicVoxelPointSizePx;
    const float previousSlamFreeDomRaycastedVoxelPointSizePx = slamFreeDomRaycastedVoxelPointSizePx;
    const float previousSlamFreeDomFreeVoxelPointSizePx = slamFreeDomFreeVoxelPointSizePx;
    const float previousSlamFreeDomStaticVoxelPointSizePx = slamFreeDomStaticVoxelPointSizePx;
    const float previousSlamFreeDomEnhancedPointSizePx = slamFreeDomEnhancedPointSizePx;
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
    slamRuntimeConfig.allowPureLidar = slamLidarOnlyCheck->isChecked();
    slamRuntimeConfig.imuEnabled = !slamRuntimeConfig.allowPureLidar;
    slamRuntimeConfig.backendType = slamRuntimeConfig.allowPureLidar
        ? QStringLiteral("FAST_LO")
        : QStringLiteral("FAST_LIO");
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
    slamRuntimeConfig.loopClosureEnableFlag = slamLoopClosureCheck->isChecked();
    slamRuntimeConfig.surroundingKeyframeAddingDistThreshold = slamKeyframeDistanceSpin->value();
    slamRuntimeConfig.surroundingKeyframeAddingAngleThreshold = slamKeyframeAngleSpin->value();
    slamRuntimeConfig.loopClosureFrequency = slamLoopClosureFrequencySpin->value();
    slamRuntimeConfig.historyKeyframeSearchRadius = slamHistoryKeyframeSearchRadiusSpin->value();
    slamRuntimeConfig.historyKeyframeSearchTimeDiff = slamHistoryKeyframeSearchTimeDiffSpin->value();
    slamRuntimeConfig.historyKeyframeSearchNum = slamHistoryKeyframeSearchNumSpin->value();
    slamRuntimeConfig.historyKeyframeFitnessScore = slamHistoryKeyframeFitnessScoreSpin->value();
    slamRuntimeConfig.reconstructKdTree = slamReconstructKdTreeCheck->isChecked();
    applyDynamicFilterControls(slamRuntimeConfig);
    slamRuntimeConfig.dynamicObjectClusterVoxelSizeM = slamDynamicClusterVoxelSizeSpin->value();
    slamRuntimeConfig.dynamicObjectClusterExtendVoxel = slamDynamicClusterExtendVoxelSpin->value();
    slamRuntimeConfig.dynamicObjectClusterMinVoxelCount = slamDynamicClusterMinVoxelCountSpin->value();
    slamRuntimeConfig.dynamicObjectClusterTrustThreshold = slamDynamicClusterTrustThresholdSpin->value();
    slamRuntimeConfig.dynamicObjectClusterGroundDistanceThresholdM = slamDynamicClusterGroundDistanceSpin->value();
    slamRuntimeConfig.dynamicObjectClusterGroundMaxAngleDeg = slamDynamicClusterGroundMaxAngleSpin->value();
    slamRuntimeConfig.dynamicObjectBufferDelaySec = slamDynamicBufferDelaySpin->value();
    slamRuntimeConfig.dynamicObjectDepthMapDurationSec = slamDynamicDepthMapDurationSpin->value();
    slamRuntimeConfig.dynamicObjectMaxDepthMaps = slamDynamicMaxDepthMapsSpin->value();
    slamRuntimeConfig.dynamicObjectMinHistoryMaps = slamDynamicMinHistoryMapsSpin->value();
    slamRuntimeConfig.dynamicObjectHorizontalResolutionRad = slamDynamicHorizontalResolutionSpin->value();
    slamRuntimeConfig.dynamicObjectVerticalResolutionRad = slamDynamicVerticalResolutionSpin->value();
    slamRuntimeConfig.dynamicObjectVerticalFovDownDeg = slamDynamicVerticalFovDownSpin->value();
    slamRuntimeConfig.dynamicObjectVerticalFovUpDeg = slamDynamicVerticalFovUpSpin->value();
    slamRuntimeConfig.dynamicObjectHorizontalFovRightDeg = slamDynamicHorizontalFovRightSpin->value();
    slamRuntimeConfig.dynamicObjectHorizontalFovLeftDeg = slamDynamicHorizontalFovLeftSpin->value();
    slamRuntimeConfig.dynamicObjectMinRangeM = slamDynamicMinRangeSpin->value();
    slamRuntimeConfig.dynamicObjectMaxRangeM = slamDynamicMaxRangeSpin->value();
    slamRuntimeConfig.dynamicObjectNeighborPixelRadius = slamDynamicNeighborPixelRadiusSpin->value();
    slamRuntimeConfig.dynamicObjectCase1DepthMarginM = slamDynamicCase1DepthMarginSpin->value();
    slamRuntimeConfig.dynamicObjectCase2DepthMarginM = slamDynamicCase2DepthMarginSpin->value();
    slamRuntimeConfig.dynamicObjectCase3DepthMarginM = slamDynamicCase3DepthMarginSpin->value();
    slamRuntimeConfig.dynamicObjectCase1VoteThreshold = slamDynamicCase1VoteThresholdSpin->value();
    slamRuntimeConfig.dynamicObjectCase2OcclusionChainLength =
        slamDynamicCase2OcclusionChainLengthSpin->value();
    slamRuntimeConfig.dynamicObjectCase3OcclusionChainLength =
        slamDynamicCase3OcclusionChainLengthSpin->value();
    slamWorldCurrentFrameColor = selectedSlamWorldCurrentFrameColor;
    slamBodyFrameColor = selectedSlamBodyFrameColor;
    slamDynamicObjectColor = selectedSlamDynamicObjectColor;
    slamDynamicAggressiveColor = selectedSlamDynamicAggressiveColor;
    slamDynamicModerateColor = selectedSlamDynamicModerateColor;
    slamDynamicConservativeColor = selectedSlamDynamicConservativeColor;
    slamFreeDomScanVoxelColor = selectedSlamFreeDomScanVoxelColor;
    slamFreeDomDynamicVoxelColor = selectedSlamFreeDomDynamicVoxelColor;
    slamFreeDomRaycastedVoxelColor = selectedSlamFreeDomRaycastedVoxelColor;
    slamFreeDomFreeVoxelColor = selectedSlamFreeDomFreeVoxelColor;
    slamFreeDomStaticVoxelColor = selectedSlamFreeDomStaticVoxelColor;
    slamFreeDomEnhancedColor = selectedSlamFreeDomEnhancedColor;
    slamTrajectoryColor = selectedSlamTrajectoryColor;
    slamWorldCurrentFramePointSizePx = static_cast<float>(slamWorldCurrentFramePointSizeSpin->value());
    slamBodyFramePointSizePx = static_cast<float>(slamBodyFramePointSizeSpin->value());
    slamDynamicObjectPointSizePx =
        static_cast<float>(slamDynamicObjectPointSizeSpin->value());
    slamFreeDomScanVoxelPointSizePx =
        static_cast<float>(slamFreeDomScanVoxelPointSizeSpin->value());
    slamFreeDomDynamicVoxelPointSizePx =
        static_cast<float>(slamFreeDomDynamicVoxelPointSizeSpin->value());
    slamFreeDomRaycastedVoxelPointSizePx =
        static_cast<float>(slamFreeDomRaycastedVoxelPointSizeSpin->value());
    slamFreeDomFreeVoxelPointSizePx =
        static_cast<float>(slamFreeDomFreeVoxelPointSizeSpin->value());
    slamFreeDomStaticVoxelPointSizePx =
        static_cast<float>(slamFreeDomStaticVoxelPointSizeSpin->value());
    slamFreeDomEnhancedPointSizePx =
        static_cast<float>(slamFreeDomEnhancedPointSizeSpin->value());
    slamTrajectoryLineWidthPx = static_cast<float>(slamTrajectoryLineWidthSpin->value());
    slamPoseAxisLengthM = static_cast<float>(slamPoseAxisLengthSpin->value());
    slamPoseAxisLineWidthPx = static_cast<float>(slamPoseAxisLineWidthSpin->value());
    const bool gridChanged =
        config.range != previousGridConfig.range ||
        config.step != previousGridConfig.step ||
        config.color != previousGridConfig.color ||
        config.type != previousGridConfig.type;
    const bool pointCloudColorChanged =
        distanceLegendMin != previousDistanceLegendMin ||
        distanceLegendMax != previousDistanceLegendMax ||
        elevationLegendMin != previousElevationLegendMin ||
        elevationLegendMax != previousElevationLegendMax ||
        reflectivityColorScale != previousReflectivityColorScale ||
        solidColor != previousSolidColor ||
        lineColors != previousLineColors;
    const bool backgroundChanged = pointCloudBackgroundPreset != previousPointCloudBackgroundPreset;
    const bool slamLayerAvailabilityChanged =
        slamRuntimeConfig.publishWorldFrameCloud != previousSlamPublishWorld ||
        slamRuntimeConfig.publishBodyFrameCloud != previousSlamPublishBody ||
        slamRuntimeConfig.dynamicFilterEnabled != previousSlamDynamicDetection ||
        slamRuntimeConfig.dynamicFilterBackend != previousSlamRuntimeConfig.dynamicFilterBackend ||
        slamRuntimeConfig.dynamicDebugVisualizationEnabled !=
            previousSlamRuntimeConfig.dynamicDebugVisualizationEnabled ||
        slamRuntimeConfig.freeDom.raycastEnhancementEnabled !=
            previousSlamRuntimeConfig.freeDom.raycastEnhancementEnabled;
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
        slamUiBridge->setDynamicObjectColor(slamDynamicObjectColor);
        slamUiBridge->setDynamicAggressiveColor(slamDynamicAggressiveColor);
        slamUiBridge->setDynamicModerateColor(slamDynamicModerateColor);
        slamUiBridge->setDynamicConservativeColor(slamDynamicConservativeColor);
        slamUiBridge->setFreeDomScanVoxelColor(slamFreeDomScanVoxelColor);
        slamUiBridge->setFreeDomDynamicVoxelColor(slamFreeDomDynamicVoxelColor);
        slamUiBridge->setFreeDomRaycastedVoxelColor(slamFreeDomRaycastedVoxelColor);
        slamUiBridge->setFreeDomFreeVoxelColor(slamFreeDomFreeVoxelColor);
        slamUiBridge->setFreeDomStaticVoxelColor(slamFreeDomStaticVoxelColor);
        slamUiBridge->setFreeDomEnhancedColor(slamFreeDomEnhancedColor);
        slamUiBridge->setTrajectoryColor(slamTrajectoryColor);
        slamUiBridge->setWorldFramePointSize(slamWorldCurrentFramePointSizePx);
        slamUiBridge->setBodyFramePointSize(slamBodyFramePointSizePx);
        slamUiBridge->setDynamicObjectPointSize(slamDynamicObjectPointSizePx);
        slamUiBridge->setFreeDomScanVoxelPointSize(slamFreeDomScanVoxelPointSizePx);
        slamUiBridge->setFreeDomDynamicVoxelPointSize(slamFreeDomDynamicVoxelPointSizePx);
        slamUiBridge->setFreeDomRaycastedVoxelPointSize(slamFreeDomRaycastedVoxelPointSizePx);
        slamUiBridge->setFreeDomFreeVoxelPointSize(slamFreeDomFreeVoxelPointSizePx);
        slamUiBridge->setFreeDomStaticVoxelPointSize(slamFreeDomStaticVoxelPointSizePx);
        slamUiBridge->setFreeDomEnhancedPointSize(slamFreeDomEnhancedPointSizePx);
        slamUiBridge->setTrajectoryLineWidth(slamTrajectoryLineWidthPx);
        slamUiBridge->setPoseAxisLength(slamPoseAxisLengthM);
        slamUiBridge->setPoseAxisLineWidth(slamPoseAxisLineWidthPx);
    }
    if (previousSlamPublishWorld && !slamRuntimeConfig.publishWorldFrameCloud) {
        clearSlamWorldPointCloud();
    }
    if (slamLayerAvailabilityChanged) {
        rebuildSlamInfoPanel();
        syncSlamRenderLayerVisibility();
    }
    syncSlamTemplateControl();
    if (slamRuntimeConfig.publishWorldFrameCloud != previousSlamPublishWorld) {
        refreshSlamWorldPointCloud();
    }

    if (pointCloudColorChanged) {
        syncReflectivityColorScaleControls();
        recolorPointCloudViews();
    }
    if (gridChanged) {
        pointCloudView->setGridConfig(config);
    }
    if (backgroundChanged) {
        applyPointCloudBackground();
    }
    forEachPointCloudView([this](PointCloudView* view) {
        if (view) {
            view->setEdlConfig(pointCloudEdlConfig);
        }
    });
    syncPointCloudEdlAction();
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
    const bool freeDomConfigChanged = [&]() {
        const FreeDomRuntimeConfig& current = slamRuntimeConfig.freeDom;
        const FreeDomRuntimeConfig& previous = previousSlamRuntimeConfig.freeDom;
        return current.sensorMinRangeM != previous.sensorMinRangeM ||
            current.sensorMaxRangeM != previous.sensorMaxRangeM ||
            current.sensorMinZM != previous.sensorMinZM ||
            current.sensorMaxZM != previous.sensorMaxZM ||
            current.subVoxelSizeM != previous.subVoxelSizeM ||
            current.voxelDepth != previous.voxelDepth ||
            current.blockDepth != previous.blockDepth ||
            current.localMapEnabled != previous.localMapEnabled ||
            current.localMapRangeM != previous.localMapRangeM ||
            current.localMapMinZM != previous.localMapMinZM ||
            current.localMapMaxZM != previous.localMapMaxZM ||
            current.raycastMaxRangeM != previous.raycastMaxRangeM ||
            current.raycastMinZM != previous.raycastMinZM ||
            current.raycastMaxZM != previous.raycastMaxZM ||
            current.countsToFree != previous.countsToFree ||
            current.countsToRevert != previous.countsToRevert ||
            current.conservativeConnectivity != previous.conservativeConnectivity ||
            current.aggressiveConnectivity != previous.aggressiveConnectivity ||
            current.raycastEnhancementEnabled != previous.raycastEnhancementEnabled ||
            current.lidarHorizontalFovDeg != previous.lidarHorizontalFovDeg ||
            current.lidarVerticalFovUpperDeg != previous.lidarVerticalFovUpperDeg ||
            current.lidarVerticalFovLowerDeg != previous.lidarVerticalFovLowerDeg ||
            current.depthImageVerticalLines != previous.depthImageVerticalLines ||
            current.depthImageMinRangeM != previous.depthImageMinRangeM ||
            current.maxRaycastEnhancementRangeM != previous.maxRaycastEnhancementRangeM ||
            current.raycastEnhancementDepthMarginM != previous.raycastEnhancementDepthMarginM ||
            current.inpaintSize != previous.inpaintSize ||
            current.erosionSize != previous.erosionSize ||
            current.minRaycastEnhancementArea != previous.minRaycastEnhancementArea ||
            current.depthImageTopMargin != previous.depthImageTopMargin ||
            current.learnFov != previous.learnFov ||
            current.fovMaskEnabled != previous.fovMaskEnabled ||
            current.fovMaskPath != previous.fovMaskPath ||
            current.numThreads != previous.numThreads;
    }();
    const bool slamDynamicConfigChanged =
        slamRuntimeConfig.dynamicFilterBackend != previousSlamRuntimeConfig.dynamicFilterBackend ||
        slamRuntimeConfig.dynamicFilterEnabled != previousSlamRuntimeConfig.dynamicFilterEnabled ||
        slamRuntimeConfig.dynamicPointRemovalEnabled != previousSlamRuntimeConfig.dynamicPointRemovalEnabled ||
        slamRuntimeConfig.dynamicDebugVisualizationEnabled !=
            previousSlamRuntimeConfig.dynamicDebugVisualizationEnabled ||
        slamRuntimeConfig.dynamicDebugSnapshotIntervalFrames !=
            previousSlamRuntimeConfig.dynamicDebugSnapshotIntervalFrames ||
        slamRuntimeConfig.freeDomMapSnapshotIntervalFrames !=
            previousSlamRuntimeConfig.freeDomMapSnapshotIntervalFrames ||
        freeDomConfigChanged ||
        slamRuntimeConfig.dynamicObjectRemovalEnabled != previousSlamRuntimeConfig.dynamicObjectRemovalEnabled ||
        slamRuntimeConfig.dynamicObjectClusterEnabled != previousSlamRuntimeConfig.dynamicObjectClusterEnabled ||
        slamRuntimeConfig.dynamicObjectClusterVoxelSizeM != previousSlamRuntimeConfig.dynamicObjectClusterVoxelSizeM ||
        slamRuntimeConfig.dynamicObjectClusterExtendVoxel != previousSlamRuntimeConfig.dynamicObjectClusterExtendVoxel ||
        slamRuntimeConfig.dynamicObjectClusterMinVoxelCount != previousSlamRuntimeConfig.dynamicObjectClusterMinVoxelCount ||
        slamRuntimeConfig.dynamicObjectClusterTrustThreshold != previousSlamRuntimeConfig.dynamicObjectClusterTrustThreshold ||
        slamRuntimeConfig.dynamicObjectClusterGroundDistanceThresholdM != previousSlamRuntimeConfig.dynamicObjectClusterGroundDistanceThresholdM ||
        slamRuntimeConfig.dynamicObjectClusterGroundMaxAngleDeg != previousSlamRuntimeConfig.dynamicObjectClusterGroundMaxAngleDeg ||
        slamRuntimeConfig.dynamicObjectBufferDelaySec != previousSlamRuntimeConfig.dynamicObjectBufferDelaySec ||
        slamRuntimeConfig.dynamicObjectDepthMapDurationSec != previousSlamRuntimeConfig.dynamicObjectDepthMapDurationSec ||
        slamRuntimeConfig.dynamicObjectMaxDepthMaps != previousSlamRuntimeConfig.dynamicObjectMaxDepthMaps ||
        slamRuntimeConfig.dynamicObjectMinHistoryMaps != previousSlamRuntimeConfig.dynamicObjectMinHistoryMaps ||
        slamRuntimeConfig.dynamicObjectHorizontalResolutionRad != previousSlamRuntimeConfig.dynamicObjectHorizontalResolutionRad ||
        slamRuntimeConfig.dynamicObjectVerticalResolutionRad != previousSlamRuntimeConfig.dynamicObjectVerticalResolutionRad ||
        slamRuntimeConfig.dynamicObjectVerticalFovDownDeg != previousSlamRuntimeConfig.dynamicObjectVerticalFovDownDeg ||
        slamRuntimeConfig.dynamicObjectVerticalFovUpDeg != previousSlamRuntimeConfig.dynamicObjectVerticalFovUpDeg ||
        slamRuntimeConfig.dynamicObjectHorizontalFovRightDeg != previousSlamRuntimeConfig.dynamicObjectHorizontalFovRightDeg ||
        slamRuntimeConfig.dynamicObjectHorizontalFovLeftDeg != previousSlamRuntimeConfig.dynamicObjectHorizontalFovLeftDeg ||
        slamRuntimeConfig.dynamicObjectMinRangeM != previousSlamRuntimeConfig.dynamicObjectMinRangeM ||
        slamRuntimeConfig.dynamicObjectMaxRangeM != previousSlamRuntimeConfig.dynamicObjectMaxRangeM ||
        slamRuntimeConfig.dynamicObjectNeighborPixelRadius != previousSlamRuntimeConfig.dynamicObjectNeighborPixelRadius ||
        slamRuntimeConfig.dynamicObjectCase1DepthMarginM != previousSlamRuntimeConfig.dynamicObjectCase1DepthMarginM ||
        slamRuntimeConfig.dynamicObjectCase2DepthMarginM != previousSlamRuntimeConfig.dynamicObjectCase2DepthMarginM ||
        slamRuntimeConfig.dynamicObjectCase3DepthMarginM != previousSlamRuntimeConfig.dynamicObjectCase3DepthMarginM ||
        slamRuntimeConfig.dynamicObjectCase1VoteThreshold != previousSlamRuntimeConfig.dynamicObjectCase1VoteThreshold ||
        slamRuntimeConfig.dynamicObjectCase2OcclusionChainLength !=
            previousSlamRuntimeConfig.dynamicObjectCase2OcclusionChainLength ||
        slamRuntimeConfig.dynamicObjectCase3OcclusionChainLength !=
            previousSlamRuntimeConfig.dynamicObjectCase3OcclusionChainLength;
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
        slamRuntimeConfig.allowPureLidar != previousSlamRuntimeConfig.allowPureLidar ||
        slamRuntimeConfig.imuEnabled != previousSlamRuntimeConfig.imuEnabled ||
        slamExtrinsicChanged ||
        slamRuntimeConfig.publishWorldFrameCloud != previousSlamPublishWorld ||
        slamRuntimeConfig.publishDenseFrameCloud != previousSlamPublishDense ||
        slamRuntimeConfig.publishBodyFrameCloud != previousSlamPublishBody ||
        slamRuntimeConfig.saveMap != previousSlamSaveMap ||
        slamRuntimeConfig.loopClosureEnableFlag != previousSlamRuntimeConfig.loopClosureEnableFlag ||
        slamRuntimeConfig.surroundingKeyframeAddingDistThreshold !=
            previousSlamRuntimeConfig.surroundingKeyframeAddingDistThreshold ||
        slamRuntimeConfig.surroundingKeyframeAddingAngleThreshold !=
            previousSlamRuntimeConfig.surroundingKeyframeAddingAngleThreshold ||
        slamRuntimeConfig.loopClosureFrequency != previousSlamRuntimeConfig.loopClosureFrequency ||
        slamRuntimeConfig.historyKeyframeSearchRadius != previousSlamRuntimeConfig.historyKeyframeSearchRadius ||
        slamRuntimeConfig.historyKeyframeSearchTimeDiff != previousSlamRuntimeConfig.historyKeyframeSearchTimeDiff ||
        slamRuntimeConfig.historyKeyframeSearchNum != previousSlamRuntimeConfig.historyKeyframeSearchNum ||
        slamRuntimeConfig.historyKeyframeFitnessScore != previousSlamRuntimeConfig.historyKeyframeFitnessScore ||
        slamRuntimeConfig.reconstructKdTree != previousSlamRuntimeConfig.reconstructKdTree ||
        slamRuntimeConfig.dynamicFilterEnabled != previousSlamDynamicDetection ||
        slamDynamicConfigChanged ||
        slamWorldCurrentFrameColor != previousSlamWorldCurrentFrameColor ||
        slamBodyFrameColor != previousSlamBodyFrameColor ||
        slamDynamicObjectColor != previousSlamDynamicObjectColor ||
        slamDynamicAggressiveColor != previousSlamDynamicAggressiveColor ||
        slamDynamicModerateColor != previousSlamDynamicModerateColor ||
        slamDynamicConservativeColor != previousSlamDynamicConservativeColor ||
        slamFreeDomScanVoxelColor != previousSlamFreeDomScanVoxelColor ||
        slamFreeDomDynamicVoxelColor != previousSlamFreeDomDynamicVoxelColor ||
        slamFreeDomRaycastedVoxelColor != previousSlamFreeDomRaycastedVoxelColor ||
        slamFreeDomFreeVoxelColor != previousSlamFreeDomFreeVoxelColor ||
        slamFreeDomStaticVoxelColor != previousSlamFreeDomStaticVoxelColor ||
        slamFreeDomEnhancedColor != previousSlamFreeDomEnhancedColor ||
        slamTrajectoryColor != previousSlamTrajectoryColor ||
        slamWorldCurrentFramePointSizePx != previousSlamWorldCurrentFramePointSizePx ||
        slamBodyFramePointSizePx != previousSlamBodyFramePointSizePx ||
        slamDynamicObjectPointSizePx != previousSlamDynamicObjectPointSizePx ||
        slamFreeDomScanVoxelPointSizePx != previousSlamFreeDomScanVoxelPointSizePx ||
        slamFreeDomDynamicVoxelPointSizePx != previousSlamFreeDomDynamicVoxelPointSizePx ||
        slamFreeDomRaycastedVoxelPointSizePx != previousSlamFreeDomRaycastedVoxelPointSizePx ||
        slamFreeDomFreeVoxelPointSizePx != previousSlamFreeDomFreeVoxelPointSizePx ||
        slamFreeDomStaticVoxelPointSizePx != previousSlamFreeDomStaticVoxelPointSizePx ||
        slamFreeDomEnhancedPointSizePx != previousSlamFreeDomEnhancedPointSizePx ||
        slamTrajectoryLineWidthPx != previousSlamTrajectoryLineWidthPx ||
        slamPoseAxisLengthM != previousSlamPoseAxisLengthM ||
        slamPoseAxisLineWidthPx != previousSlamPoseAxisLineWidthPx) {
        liveSlamSource.setFrameDurationMs(slamRuntimeConfig.inputFrameDurationMs);
        if (slamDynamicConfigChanged && slamWorkerActive.load()) {
            const QString restartMessage = QStringLiteral(
                "[SLAM] 动态滤除设置已保存；当前会话继续使用原后端，请重置或重启 SLAM 后生效。");
            logMessage(restartMessage);
            if (statusBar()) {
                statusBar()->showMessage(restartMessage, 6000);
            }
        } else {
            logMessage(QStringLiteral("[SLAM] 设置已更新"));
        }
    }
}

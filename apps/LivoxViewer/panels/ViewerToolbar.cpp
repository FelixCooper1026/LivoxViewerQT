#include "LivoxViewerWindow.h"

#include "Export/PointCloudExport.h"

#include <QActionGroup>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QIcon>
#include <QLayout>
#include <QMenu>
#include <QMessageBox>
#include <QSettings>
#include <QSignalBlocker>
#include <QStandardPaths>
#include <QStyle>
#include <QToolButton>
#include <QWidgetAction>

#include <algorithm>

namespace {

class FlowLayout : public QLayout
{
public:
    explicit FlowLayout(QWidget* parent = nullptr, int margin = 0, int hSpacing = 8, int vSpacing = 6)
        : QLayout(parent)
        , m_hSpacing(hSpacing)
        , m_vSpacing(vSpacing)
    {
        setContentsMargins(margin, margin, margin, margin);
    }

    ~FlowLayout() override
    {
        QLayoutItem* item = nullptr;
        while ((item = takeAt(0)) != nullptr) {
            delete item;
        }
    }

    void addItem(QLayoutItem* item) override { m_items.append(item); }
    int count() const override { return m_items.size(); }
    QLayoutItem* itemAt(int index) const override { return m_items.value(index); }

    QLayoutItem* takeAt(int index) override
    {
        if (index < 0 || index >= m_items.size()) {
            return nullptr;
        }
        return m_items.takeAt(index);
    }

    Qt::Orientations expandingDirections() const override { return Qt::Horizontal; }
    bool hasHeightForWidth() const override { return true; }
    int heightForWidth(int width) const override { return doLayout(QRect(0, 0, width, 0), true); }

    QSize sizeHint() const override
    {
        QSize size;
        for (QLayoutItem* item : m_items) {
            QWidget* widget = item->widget();
            if (widget && widget->isHidden()) {
                continue;
            }
            size = size.expandedTo(item->sizeHint());
        }
        QMargins margins = contentsMargins();
        size += QSize(margins.left() + margins.right(), margins.top() + margins.bottom());
        return size;
    }

    QSize minimumSize() const override
    {
        QSize size;
        for (QLayoutItem* item : m_items) {
            QSize itemSize = item->minimumSize();
            itemSize.setHeight(std::max(itemSize.height(), item->sizeHint().height()));
            size = size.expandedTo(itemSize);
        }
        QMargins margins = contentsMargins();
        size += QSize(margins.left() + margins.right(), margins.top() + margins.bottom());
        return size;
    }

    void setGeometry(const QRect& rect) override
    {
        QLayout::setGeometry(rect);
        doLayout(rect, false);
    }

private:
    int doLayout(const QRect& rect, bool testOnly) const
    {
        QMargins margins = contentsMargins();
        QRect effectiveRect = rect.adjusted(margins.left(), margins.top(), -margins.right(), -margins.bottom());
        int x = effectiveRect.x();
        int y = effectiveRect.y();
        int lineHeight = 0;
        const int lineWidth = std::max(1, effectiveRect.width());

        for (QLayoutItem* item : m_items) {
            QWidget* widget = item->widget();
            if (widget && widget->isHidden()) {
                continue;
            }

            QSize hint = item->sizeHint();
            const int minWidth = std::min(item->minimumSize().width(), lineWidth);
            int itemWidth = std::min(hint.width(), lineWidth);
            itemWidth = std::max(itemWidth, minWidth);
            const QSize itemSize(itemWidth, hint.height());
            const int nextX = x + itemSize.width() + m_hSpacing;

            if (nextX - m_hSpacing > effectiveRect.right() && lineHeight > 0) {
                x = effectiveRect.x();
                y += lineHeight + m_vSpacing;
                lineHeight = 0;
            }

            if (!testOnly) {
                item->setGeometry(QRect(QPoint(x, y), itemSize));
            }

            x += itemSize.width() + m_hSpacing;
            lineHeight = std::max(lineHeight, itemSize.height());
        }

        return y + lineHeight - rect.y() + margins.bottom();
    }

    QVector<QLayoutItem*> m_items;
    int m_hSpacing = 8;
    int m_vSpacing = 6;
};

class ToolbarGroup : public QWidget
{
public:
    explicit ToolbarGroup(const QString& title, QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setObjectName("ViewerToolbarGroup");
        setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

        QVBoxLayout* root = new QVBoxLayout(this);
        root->setContentsMargins(8, 5, 8, 3);
        root->setSpacing(3);

        m_controls = new QWidget(this);
        m_controlsLayout = new QHBoxLayout(m_controls);
        m_controlsLayout->setContentsMargins(0, 0, 0, 0);
        m_controlsLayout->setSpacing(6);
        root->addWidget(m_controls, 1);

        QWidget* titleRow = new QWidget(this);
        QHBoxLayout* titleLayout = new QHBoxLayout(titleRow);
        titleLayout->setContentsMargins(0, 0, 0, 0);
        titleLayout->setSpacing(3);
        titleLayout->addStretch();

        m_title = new QLabel(title, titleRow);
        QFont titleFont = m_title->font();
        titleFont.setPointSizeF(std::max(7.0, titleFont.pointSizeF() * 0.9));
        m_title->setFont(titleFont);
        m_title->setAlignment(Qt::AlignCenter);
        titleLayout->addWidget(m_title);

        m_moreMenu = new QMenu(this);
        m_moreButton = new QToolButton(titleRow);
        m_moreButton->setText("...");
        m_moreButton->setToolTip("更多");
        m_moreButton->setPopupMode(QToolButton::InstantPopup);
        m_moreButton->setMenu(m_moreMenu);
        m_moreButton->setAutoRaise(true);
        m_moreButton->setVisible(false);
        titleLayout->addWidget(m_moreButton);
        titleLayout->addStretch();
        root->addWidget(titleRow);

        setStyleSheet(
            "#ViewerToolbarGroup {"
            "  background: palette(window);"
            "  border-right: 1px solid palette(mid);"
            "}"
            "#ViewerToolbarGroup[leadingSeparator=\"true\"] {"
            "  border-left: 1px solid palette(mid);"
            "}"
            "#ViewerToolbarGroup QLabel { color: palette(window-text); }"
        );
    }

    QHBoxLayout* controlsLayout() const { return m_controlsLayout; }
    QMenu* moreMenu() const { return m_moreMenu; }

    void setLeadingSeparatorVisible(bool visible)
    {
        setProperty("leadingSeparator", visible);
        style()->unpolish(this);
        style()->polish(this);
        update();
    }

    void addPrimaryWidget(QWidget* widget)
    {
        m_primaryWidgets.append(widget);
        m_controlsLayout->addWidget(widget);
    }

    void addSecondaryWidget(QWidget* widget)
    {
        m_secondaryWidgets.append(widget);
        m_controlsLayout->addWidget(widget);
    }

    QSize sizeHint() const override
    {
        return QSize(estimatedWidth(true, true), QWidget::sizeHint().height());
    }

    QSize minimumSizeHint() const override
    {
        return QSize(estimatedWidth(true, false), QWidget::sizeHint().height());
    }

protected:
    void resizeEvent(QResizeEvent* event) override
    {
        QWidget::resizeEvent(event);
        updateOverflow();
    }

private:
    int estimatedWidth(bool includePrimary, bool includeSecondary) const
    {
        const QMargins margins = layout()->contentsMargins();
        int controlsWidth = 0;

        int visibleControlCount = 0;
        auto addWidgetWidth = [&](QWidget* widget) {
            controlsWidth += widget->sizeHint().width();
            ++visibleControlCount;
        };

        if (includePrimary) {
            for (QWidget* widget : m_primaryWidgets) {
                addWidgetWidth(widget);
            }
        }
        if (includeSecondary) {
            for (QWidget* widget : m_secondaryWidgets) {
                addWidgetWidth(widget);
            }
        }

        if (visibleControlCount > 1) {
            controlsWidth += 6 * (visibleControlCount - 1);
        }

        int titleWidth = m_title->sizeHint().width();
        if (!m_moreMenu->actions().isEmpty()) {
            titleWidth += m_moreButton->sizeHint().width() + 3;
        }
        return margins.left() + margins.right() + std::max(controlsWidth, titleWidth);
    }

    void updateOverflow()
    {
        const int widthNow = width();
        if (widthNow <= 0) {
            return;
        }

        const int fullWidth = estimatedWidth(true, true);
        const bool compact = widthNow < fullWidth;

        for (QWidget* widget : m_primaryWidgets) {
            widget->setVisible(!widget->property("toolbarOptionalHidden").toBool());
        }
        for (QWidget* widget : m_secondaryWidgets) {
            widget->setVisible(!compact && !widget->property("toolbarOptionalHidden").toBool());
        }

        m_moreButton->setVisible(compact && !m_moreMenu->actions().isEmpty());
    }

    QLabel* m_title = nullptr;
    QWidget* m_controls = nullptr;
    QHBoxLayout* m_controlsLayout = nullptr;
    QToolButton* m_moreButton = nullptr;
    QMenu* m_moreMenu = nullptr;
    QVector<QWidget*> m_primaryWidgets;
    QVector<QWidget*> m_secondaryWidgets;
};

QToolButton* createIconButton(QAction* action, QWidget* parent, const QSize& iconSize)
{
    QToolButton* button = new QToolButton(parent);
    button->setDefaultAction(action);
    button->setToolButtonStyle(Qt::ToolButtonIconOnly);
    button->setIconSize(iconSize);
    button->setAutoRaise(true);
    return button;
}

QWidget* createLabeledWidget(const QString& label, QWidget* control, QWidget* parent)
{
    QWidget* row = new QWidget(parent);
    QHBoxLayout* layout = new QHBoxLayout(row);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(4);
    QLabel* text = new QLabel(label, row);
    layout->addWidget(text);
    layout->addWidget(control);
    return row;
}

QWidget* createIconLabeledWidget(const QString& iconPath, const QString& toolTip, QWidget* control, QWidget* parent, const QSize& iconSize)
{
    QWidget* row = new QWidget(parent);
    QHBoxLayout* layout = new QHBoxLayout(row);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(4);

    QLabel* icon = new QLabel(row);
    icon->setPixmap(QIcon(iconPath).pixmap(iconSize));
    icon->setFixedSize(iconSize);
    icon->setToolTip(toolTip);
    icon->setAttribute(Qt::WA_TransparentForMouseEvents);
    layout->addWidget(icon);
    layout->addWidget(control);
    return row;
}

QSpinBox* cloneSpinBox(QSpinBox* source, QWidget* parent)
{
    QSpinBox* spin = new QSpinBox(parent);
    spin->setRange(source->minimum(), source->maximum());
    spin->setSingleStep(source->singleStep());
    spin->setSuffix(source->suffix());
    spin->setValue(source->value());
    QObject::connect(source, QOverload<int>::of(&QSpinBox::valueChanged), spin, [spin](int value) {
        QSignalBlocker blocker(spin);
        spin->setValue(value);
    });
    QObject::connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), source, [source](int value) {
        source->setValue(value);
    });
    return spin;
}

QDoubleSpinBox* cloneDoubleSpinBox(QDoubleSpinBox* source, QWidget* parent)
{
    QDoubleSpinBox* spin = new QDoubleSpinBox(parent);
    spin->setRange(source->minimum(), source->maximum());
    spin->setDecimals(source->decimals());
    spin->setSingleStep(source->singleStep());
    spin->setSuffix(source->suffix());
    spin->setValue(source->value());
    QObject::connect(source, QOverload<double>::of(&QDoubleSpinBox::valueChanged), spin, [spin](double value) {
        QSignalBlocker blocker(spin);
        spin->setValue(value);
    });
    QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), source, [source](double value) {
        source->setValue(value);
    });
    return spin;
}

void addWidgetAction(QMenu* menu, const QString& label, QWidget* widget)
{
    QWidget* row = new QWidget(menu);
    QHBoxLayout* layout = new QHBoxLayout(row);
    layout->setContentsMargins(8, 4, 8, 4);
    layout->setSpacing(6);
    layout->addWidget(new QLabel(label, row));
    layout->addWidget(widget);

    QWidgetAction* action = new QWidgetAction(menu);
    action->setDefaultWidget(row);
    menu->addAction(action);
}

QString extensionForSelectedFilter(const QString& selectedFilter)
{
    if (selectedFilter.contains("LAS")) {
        return QStringLiteral("las");
    }
    if (selectedFilter.contains("CSV")) {
        return QStringLiteral("csv");
    }
    if (selectedFilter.contains("TXT")) {
        return QStringLiteral("txt");
    }
    return QStringLiteral("pcd");
}

bool saveCrossSectionPoints(QWidget* parent, const QVector<PointCloudPoint>& points, QString& savedPath)
{
    QSettings settings("Livox", "LivoxViewerQT");
    QString lastDir = settings.value("save/lastCrossSectionDir", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).toString();
    if (lastDir.isEmpty()) {
        lastDir = QDir::homePath();
    }

    QString selectedFilter = QStringLiteral("PCD (*.pcd)");
    QString filePath = QFileDialog::getSaveFileName(parent,
                                                    QStringLiteral("导出当前切片"),
                                                    lastDir,
                                                    QStringLiteral("PCD (*.pcd);;LAS (*.las);;CSV (*.csv);;TXT (*.txt)"),
                                                    &selectedFilter);
    if (filePath.isEmpty()) {
        return false;
    }

    QFileInfo fileInfo(filePath);
    QString suffix = fileInfo.suffix().toLower();
    if (suffix.isEmpty()) {
        suffix = extensionForSelectedFilter(selectedFilter);
        filePath += QStringLiteral(".") + suffix;
        fileInfo.setFile(filePath);
    }

    bool ok = false;
    if (suffix == QStringLiteral("pcd")) {
        ok = PointCloudExport::saveAsPCD(filePath, points);
    } else if (suffix == QStringLiteral("las")) {
        ok = PointCloudExport::saveAsLAS(filePath, points);
    } else if (suffix == QStringLiteral("csv")) {
        ok = PointCloudExport::saveAsCSV(filePath, points);
    } else if (suffix == QStringLiteral("txt")) {
        ok = PointCloudExport::saveAsTXT(filePath, points);
    } else {
        QMessageBox::warning(parent, QStringLiteral("导出当前切片"), QStringLiteral("不支持的导出格式"));
        return false;
    }

    if (!ok) {
        QMessageBox::warning(parent, QStringLiteral("导出当前切片"), QStringLiteral("切片导出失败"));
        return false;
    }

    settings.setValue("save/lastCrossSectionDir", fileInfo.absolutePath());
    savedPath = filePath;
    return true;
}

} // namespace

QWidget* LivoxViewerWindow::createViewerToolbar(QWidget* parent)
{
    QWidget* viewerToolbar = new QWidget(parent);
    viewerToolbar->setObjectName("ViewerToolbar");
    viewerToolbar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    viewerToolbar->setMinimumHeight(fontMetrics().height() * 4);
    QVBoxLayout* viewerLayout = new QVBoxLayout(viewerToolbar);
    viewerLayout->setContentsMargins(8, 4, 8, 4);
    viewerLayout->setSpacing(0);

    QWidget* toolbarContent = new QWidget(viewerToolbar);
    toolbarContent->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    FlowLayout* toolbarLayout = new FlowLayout(toolbarContent, 0, 8, 6);
    toolbarContent->setLayout(toolbarLayout);
    viewerLayout->addWidget(toolbarContent);

    const QSize toolbarIconSize(fontMetrics().height() + 8, fontMetrics().height() + 8);

    ToolbarGroup* displayGroup = new ToolbarGroup("显示控制", viewerToolbar);
    displayGroup->setLeadingSeparatorVisible(true);
    QAction* gridAction = new QAction(QIcon(":/icons/grid.svg"), "世界坐标网格", this);
    gridAction->setCheckable(true);
    gridAction->setChecked(true);
    gridAction->setToolTip("显示/隐藏世界坐标网格");
    connect(gridAction, &QAction::toggled, this, [this](bool checked) {
        if (pointCloudView) {
            pointCloudView->setGridVisible(checked);
        }
    });
    displayGroup->addPrimaryWidget(createIconButton(gridAction, displayGroup, toolbarIconSize));

    actionPointCloudVisualization = new QAction(QIcon(":/icons/point_cloud_live.svg"), "冻结实时点云", this);
    actionPointCloudVisualization->setCheckable(true);
    actionPointCloudVisualization->setChecked(pointCloudVisualizationEnabled);
    connect(actionPointCloudVisualization, &QAction::triggered, this, [this](bool checked) {
        onPointCloudVisualizationToggled(checked);
    });
    syncPointCloudVisualizationAction();
    displayGroup->addPrimaryWidget(createIconButton(actionPointCloudVisualization, displayGroup, toolbarIconSize));

    QSpinBox* spinFrameIntervalTop = new QSpinBox(displayGroup);
    spinFrameIntervalTop->setRange(100, 30000);
    spinFrameIntervalTop->setSingleStep(100);
    spinFrameIntervalTop->setSuffix(" ms");
    spinFrameIntervalTop->setValue(static_cast<int>(frameIntervalMs));
    spinFrameIntervalTop->setToolTip("点云积分时间/帧间隔");
    connect(spinFrameIntervalTop, QOverload<int>::of(&QSpinBox::valueChanged), this, &LivoxViewerWindow::onFrameIntervalChanged);
    QWidget* frameIntervalRow = createIconLabeledWidget(":/icons/integration_time.svg", "积分时间", spinFrameIntervalTop, displayGroup, toolbarIconSize);
    displayGroup->addPrimaryWidget(frameIntervalRow);

    pointSizeSpin = new QSpinBox(displayGroup);
    pointSizeSpin->setRange(1, 10);
    pointSizeSpin->setValue(static_cast<int>(pointSizePx));
    pointSizeSpin->setToolTip("点大小（像素）");
    connect(pointSizeSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &LivoxViewerWindow::onPointSizeChanged);
    QWidget* pointSizeRow = createLabeledWidget("点大小:", pointSizeSpin, displayGroup);
    displayGroup->addPrimaryWidget(pointSizeRow);

    colorModeCombo = new QComboBox(displayGroup);
    colorModeCombo->addItems({"反射率", "距离", "高度", "纯色", "线号"});
    colorModeCombo->setCurrentIndex(colorMode);
    colorModeCombo->setToolTip("点云着色模式");
    QWidget* colorModeRow = createLabeledWidget("着色:", colorModeCombo, displayGroup);
    displayGroup->addPrimaryWidget(colorModeRow);

    displayGroup->moreMenu()->addAction(gridAction);
    displayGroup->moreMenu()->addAction(actionPointCloudVisualization);
    addWidgetAction(displayGroup->moreMenu(), "积分时间", cloneSpinBox(spinFrameIntervalTop, displayGroup->moreMenu()));
    addWidgetAction(displayGroup->moreMenu(), "点大小", cloneSpinBox(pointSizeSpin, displayGroup->moreMenu()));
    QComboBox* overflowColorMode = new QComboBox(displayGroup->moreMenu());
    overflowColorMode->addItems({"反射率", "距离", "高度", "纯色", "线号"});
    overflowColorMode->setCurrentIndex(colorModeCombo->currentIndex());
    connect(colorModeCombo, QOverload<int>::of(&QComboBox::activated), this, [this, overflowColorMode](int index) {
        QSignalBlocker blocker(overflowColorMode);
        overflowColorMode->setCurrentIndex(index);
        onColorModeClicked(index);
    });
    connect(overflowColorMode, QOverload<int>::of(&QComboBox::activated), this, [this](int index) {
        if (colorModeCombo) {
            QSignalBlocker blocker(colorModeCombo);
            colorModeCombo->setCurrentIndex(index);
        }
        onColorModeClicked(index);
    });
    addWidgetAction(displayGroup->moreMenu(), "着色", overflowColorMode);
    toolbarLayout->addWidget(displayGroup);

    ToolbarGroup* transformGroup = new ToolbarGroup("点云变换", viewerToolbar);
    projectionControlsGroup = transformGroup;
    projectionDepthCheck = new QCheckBox("球面投影", transformGroup);
    projectionDepthCheck->setChecked(projectionDepthEnabled);
    projectionDepthCheck->setToolTip("启用后按固定距离对深度进行投影，仅在球坐标点云时生效");
    connect(projectionDepthCheck, &QCheckBox::toggled, this, &LivoxViewerWindow::onProjectionDepthToggled);
    transformGroup->addPrimaryWidget(projectionDepthCheck);

    projectionDepthSpin = new QDoubleSpinBox(transformGroup);
    projectionDepthSpin->setRange(0.0, 10000.0);
    projectionDepthSpin->setDecimals(1);
    projectionDepthSpin->setSingleStep(1.0);
    projectionDepthSpin->setValue(projectionDepthMeters);
    projectionDepthSpin->setSuffix(" m");
    projectionDepthSpin->setToolTip("球坐标时，将 depth 投影到指定距离；0 表示使用原始 depth");
    connect(projectionDepthSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &LivoxViewerWindow::onProjectionDepthChanged);
    QWidget* depthRow = createLabeledWidget("深度:", projectionDepthSpin, transformGroup);
    transformGroup->addSecondaryWidget(depthRow);

    planarProjectionCheck = new QCheckBox("平面投影", transformGroup);
    planarProjectionCheck->setChecked(planarProjectionEnabled);
    planarProjectionCheck->setToolTip("启用平面投影模式，将半球面展开为平面图");
    connect(planarProjectionCheck, &QCheckBox::toggled, this, &LivoxViewerWindow::onPlanarProjectionToggled);
    transformGroup->addPrimaryWidget(planarProjectionCheck);

    planarRadiusSpin = new QDoubleSpinBox(transformGroup);
    planarRadiusSpin->setRange(1.0, 1000.0);
    planarRadiusSpin->setDecimals(1);
    planarRadiusSpin->setSingleStep(1.0);
    planarRadiusSpin->setValue(planarProjectionRadius);
    planarRadiusSpin->setSuffix(" m");
    planarRadiusSpin->setToolTip("平面投影的半径大小");
    connect(planarRadiusSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &LivoxViewerWindow::onPlanarProjectionRadiusChanged);
    QWidget* radiusRow = createLabeledWidget("半径:", planarRadiusSpin, transformGroup);
    transformGroup->addSecondaryWidget(radiusRow);

    QAction* projectionDepthAction = transformGroup->moreMenu()->addAction("球面投影");
    projectionDepthAction->setCheckable(true);
    projectionDepthAction->setChecked(projectionDepthEnabled);
    connect(projectionDepthAction, &QAction::toggled, projectionDepthCheck, &QCheckBox::setChecked);
    connect(projectionDepthCheck, &QCheckBox::toggled, projectionDepthAction, [projectionDepthAction](bool checked) {
        QSignalBlocker blocker(projectionDepthAction);
        projectionDepthAction->setChecked(checked);
    });
    addWidgetAction(transformGroup->moreMenu(), "投影深度", cloneDoubleSpinBox(projectionDepthSpin, transformGroup->moreMenu()));
    QAction* planarProjectionAction = transformGroup->moreMenu()->addAction("平面投影");
    planarProjectionAction->setCheckable(true);
    planarProjectionAction->setChecked(planarProjectionEnabled);
    connect(planarProjectionAction, &QAction::toggled, planarProjectionCheck, &QCheckBox::setChecked);
    connect(planarProjectionCheck, &QCheckBox::toggled, planarProjectionAction, [planarProjectionAction](bool checked) {
        QSignalBlocker blocker(planarProjectionAction);
        planarProjectionAction->setChecked(checked);
    });
    addWidgetAction(transformGroup->moreMenu(), "投影半径", cloneDoubleSpinBox(planarRadiusSpin, transformGroup->moreMenu()));
    transformGroup->setVisible(false);

    ToolbarGroup* toolsGroup = new ToolbarGroup("点云工具", viewerToolbar);
    QAction* measureAction = new QAction(QIcon(":/icons/measure.svg"), "点云测距", this);
    measureAction->setCheckable(true);
    measureAction->setToolTip("点云测距");
    connect(measureAction, &QAction::triggered, this, [this, measureAction]() {
        if (!pointCloudView) {
            QSignalBlocker blocker(measureAction);
            measureAction->setChecked(false);
            return;
        }
        const bool enable = !pointCloudView->isMeasurementModeEnabled();
        pointCloudView->setMeasurementModeEnabled(enable);
        measurementModeActive = enable;
        QSignalBlocker blocker(measureAction);
        measureAction->setChecked(enable);
        if (enable) {
            pointCloudVisualizationBeforeMeasurement = pointCloudVisualizationEnabled;
            onPointCloudVisualizationToggled(false);
            statusLabelBar->setText("测距模式：按住Ctrl+左键选择第一点");
            logMessage("进入测距模式，已暂停点云播放");
        } else {
            onPointCloudVisualizationToggled(pointCloudVisualizationBeforeMeasurement);
            statusLabelBar->setText("已连接 - 采样中");
            logMessage("退出测距模式，恢复点云播放");
        }
    });
    toolsGroup->addPrimaryWidget(createIconButton(measureAction, toolsGroup, toolbarIconSize));
    toolsGroup->moreMenu()->addAction(measureAction);

    QAction* selectionAction = new QAction(QIcon(":/icons/select_box.svg"), "点云框选", this);
    selectionAction->setCheckable(true);
    selectionAction->setToolTip("点云框选");
    connect(selectionAction, &QAction::triggered, this, [this, selectionAction]() {
        if (!pointCloudView) {
            QSignalBlocker blocker(selectionAction);
            selectionAction->setChecked(false);
            return;
        }
        const bool enable = !pointCloudView->isSelectionModeEnabled();
        pointCloudView->setSelectionModeEnabled(enable);
        if (!enable) {
            pointCloudView->clearSelectionAabb();
            if (lastSelectionCount != -1) {
                lastSelectionCount = -1;
                logMessage("已清除框选");
            }
            updateSelectionTableAndLog();
            if (attrDock) {
                attrDock->hide();
            }
            selectionRealtimeEnabled = false;
            statusLabelBar->setText("已连接 - 采样中");
        } else {
            if (attrDock) {
                attrDock->show();
                attrDock->raise();
            }
            selectionRealtimeEnabled = true;
            updateSelectionTableAndLog();
            statusLabelBar->setText("点云框选模式：按住Ctrl+左键拖动选择区域");
        }
        QSignalBlocker blocker(selectionAction);
        selectionAction->setChecked(enable);
    });
    toolsGroup->addPrimaryWidget(createIconButton(selectionAction, toolsGroup, toolbarIconSize));
    toolsGroup->moreMenu()->addAction(selectionAction);

    QAction* crossSectionAction = new QAction(QIcon(":/icons/select_box.svg"), "Cross Section", this);
    crossSectionAction->setCheckable(true);
    crossSectionAction->setToolTip("Cross Section");
    QAction* crossSectionControlsAction = new QAction("显示交互控件", this);
    crossSectionControlsAction->setCheckable(true);
    crossSectionControlsAction->setChecked(true);
    crossSectionControlsAction->setEnabled(false);
    QCheckBox* crossSectionControlsCheck = new QCheckBox("控件", toolsGroup);
    crossSectionControlsCheck->setChecked(true);
    crossSectionControlsCheck->setEnabled(false);
    crossSectionControlsCheck->setToolTip("显示/隐藏Cross Section交互控件");
    connect(crossSectionControlsAction, &QAction::toggled, crossSectionControlsCheck, [crossSectionControlsCheck](bool checked) {
        QSignalBlocker blocker(crossSectionControlsCheck);
        crossSectionControlsCheck->setChecked(checked);
    });
    connect(crossSectionControlsCheck, &QCheckBox::toggled, crossSectionControlsAction, [crossSectionControlsAction](bool checked) {
        crossSectionControlsAction->setChecked(checked);
    });
    connect(crossSectionAction, &QAction::triggered, this, [this, crossSectionAction, crossSectionControlsAction, crossSectionControlsCheck, measureAction, selectionAction]() {
        if (!pointCloudView) {
            QSignalBlocker blocker(crossSectionAction);
            crossSectionAction->setChecked(false);
            return;
        }

        const bool enable = !pointCloudView->isCrossSectionModeEnabled();
        if (enable) {
            if (pointCloudView->isMeasurementModeEnabled()) {
                pointCloudView->setMeasurementModeEnabled(false);
                measurementModeActive = false;
                QSignalBlocker blocker(measureAction);
                measureAction->setChecked(false);
            }
            if (pointCloudView->isSelectionModeEnabled()) {
                pointCloudView->setSelectionModeEnabled(false);
                selectionRealtimeEnabled = false;
                QSignalBlocker blocker(selectionAction);
                selectionAction->setChecked(false);
            }

            measureAction->setEnabled(false);
            selectionAction->setEnabled(false);
            pointCloudVisualizationBeforeCrossSection = pointCloudVisualizationEnabled;
            playbackPlayingBeforeCrossSection = playbackState.playing;
            crossSectionModeActive = true;
            pointCloudView->setCrossSectionModeEnabled(true);
            pointCloudView->setCrossSectionControlsVisible(true);
            if (!pointCloudView->isCrossSectionModeEnabled()) {
                crossSectionModeActive = false;
                measureAction->setEnabled(true);
                selectionAction->setEnabled(true);
                QSignalBlocker blocker(crossSectionAction);
                crossSectionAction->setChecked(false);
                statusLabelBar->setText("Cross Section: 当前点云为空");
                logMessage("Cross Section未启动：当前点云为空");
                return;
            }
            crossSectionControlsAction->setEnabled(true);
            crossSectionControlsCheck->setEnabled(true);
            {
                QSignalBlocker blocker(crossSectionControlsAction);
                crossSectionControlsAction->setChecked(true);
            }
            {
                QSignalBlocker blocker(crossSectionControlsCheck);
                crossSectionControlsCheck->setChecked(true);
            }
            statusLabelBar->setText("Cross Section: 拖动箭头/面/圆环调整裁剪盒");
            logMessage("进入Cross Section");
        } else {
            pointCloudView->setCrossSectionModeEnabled(false);
            pointCloudView->setCrossSectionControlsVisible(true);
            crossSectionModeActive = false;
            measureAction->setEnabled(true);
            selectionAction->setEnabled(true);
            updateLvx2PlaybackUi();
            statusLabelBar->setText(sdk_started ? "已连接 - 采样中" : "就绪");
            logMessage("退出Cross Section，已恢复点云显示");
            crossSectionControlsAction->setEnabled(false);
            crossSectionControlsCheck->setEnabled(false);
            {
                QSignalBlocker blocker(crossSectionControlsAction);
                crossSectionControlsAction->setChecked(true);
            }
            {
                QSignalBlocker blocker(crossSectionControlsCheck);
                crossSectionControlsCheck->setChecked(true);
            }
        }

        QSignalBlocker blocker(crossSectionAction);
        crossSectionAction->setChecked(enable);
    });
    connect(pointCloudView, &PointCloudView::crossSectionChanged, this, [this](int clippedPointCount, int sourcePointCount) {
        if (crossSectionModeActive && statusLabelBar) {
            statusLabelBar->setText(QString("Cross Section: %1 / %2 点").arg(clippedPointCount).arg(sourcePointCount));
        }
    });
    connect(crossSectionControlsAction, &QAction::toggled, this, [this](bool visible) {
        if (pointCloudView) {
            pointCloudView->setCrossSectionControlsVisible(visible);
        }
    });

    QAction* exportCrossSectionAction = new QAction("导出当前切片...", this);
    connect(exportCrossSectionAction, &QAction::triggered, this, [this]() {
        if (!pointCloudView || !pointCloudView->isCrossSectionModeEnabled()) {
            QMessageBox::warning(this, "导出当前切片", "请先进入Cross Section模式");
            return;
        }
        const QVector<PointCloudPoint> points = pointCloudView->currentCrossSectionPoints();
        if (points.isEmpty()) {
            QMessageBox::warning(this, "导出当前切片", "当前切片为空");
            return;
        }
        QString savedPath;
        if (saveCrossSectionPoints(this, points, savedPath)) {
            logMessage(QString("当前切片已导出: %1").arg(QDir::toNativeSeparators(savedPath)));
        }
    });

    toolsGroup->addPrimaryWidget(createIconButton(crossSectionAction, toolsGroup, toolbarIconSize));
    toolsGroup->addPrimaryWidget(crossSectionControlsCheck);
    toolsGroup->moreMenu()->addAction(crossSectionAction);
    toolsGroup->moreMenu()->addAction(crossSectionControlsAction);
    toolsGroup->moreMenu()->addAction(exportCrossSectionAction);
    toolbarLayout->addWidget(toolsGroup);

    ToolbarGroup* projectionGroup = new ToolbarGroup("投影控制", viewerToolbar);
    QActionGroup* projectionModeGroup = new QActionGroup(projectionGroup);
    projectionModeGroup->setExclusive(true);

    QAction* perspectiveProjectionAction = new QAction(QIcon(":/icons/projection_perspective.svg"), "透视投影", this);
    perspectiveProjectionAction->setCheckable(true);
    perspectiveProjectionAction->setChecked(true);
    perspectiveProjectionAction->setToolTip("透视投影");
    projectionModeGroup->addAction(perspectiveProjectionAction);
    connect(perspectiveProjectionAction, &QAction::triggered, this, [this]() {
        pointCloudView->setProjectionMode(PointCloudView::ProjectionMode::Perspective);
    });
    projectionGroup->addPrimaryWidget(createIconButton(perspectiveProjectionAction, projectionGroup, toolbarIconSize));

    QAction* orthographicProjectionAction = new QAction(QIcon(":/icons/projection_orthographic.svg"), "正交投影", this);
    orthographicProjectionAction->setCheckable(true);
    orthographicProjectionAction->setToolTip("正交投影");
    projectionModeGroup->addAction(orthographicProjectionAction);
    connect(orthographicProjectionAction, &QAction::triggered, this, [this]() {
        pointCloudView->setProjectionMode(PointCloudView::ProjectionMode::Orthographic);
    });
    projectionGroup->addPrimaryWidget(createIconButton(orthographicProjectionAction, projectionGroup, toolbarIconSize));
    projectionGroup->moreMenu()->addAction(perspectiveProjectionAction);
    projectionGroup->moreMenu()->addAction(orthographicProjectionAction);
    toolbarLayout->addWidget(projectionGroup);

    ToolbarGroup* viewGroup = new ToolbarGroup("视角控制", viewerToolbar);
    auto applyViewPreset = [this](int index) {
        if (!pointCloudView) {
            return;
        }
        switch (index) {
        case 0:
            pointCloudView->setViewPreset(PointCloudView::ViewPreset::Top);
            break;
        case 1:
            pointCloudView->setViewPreset(PointCloudView::ViewPreset::Front);
            break;
        case 2:
            pointCloudView->setViewPreset(PointCloudView::ViewPreset::Left);
            break;
        case 3:
            pointCloudView->setViewPreset(PointCloudView::ViewPreset::Right);
            break;
        case 4:
            pointCloudView->setViewPreset(PointCloudView::ViewPreset::Back);
            break;
        default:
            pointCloudView->setViewPreset(PointCloudView::ViewPreset::Top);
            break;
        }
    };

    QActionGroup* viewActionGroup = new QActionGroup(viewGroup);
    viewActionGroup->setExclusive(true);
    const QStringList viewNames = {"俯视图", "前视图", "左视图", "右视图", "后视图"};
    const QStringList viewIcons = {
        ":/icons/view_top.svg",
        ":/icons/view_front.svg",
        ":/icons/view_left.svg",
        ":/icons/view_right.svg",
        ":/icons/view_back.svg"
    };
    for (int i = 0; i < viewNames.size(); ++i) {
        QAction* action = new QAction(QIcon(viewIcons.at(i)), viewNames.at(i), this);
        action->setCheckable(true);
        action->setChecked(i == 0);
        action->setToolTip(viewNames.at(i));
        viewActionGroup->addAction(action);
        connect(action, &QAction::triggered, this, [applyViewPreset, i]() {
            applyViewPreset(i);
        });
        viewGroup->addPrimaryWidget(createIconButton(action, viewGroup, toolbarIconSize));
        viewGroup->moreMenu()->addAction(action);
    }

    QAction* resetViewAction = new QAction(QIcon(":/icons/reset_view.svg"), "重置视图", this);
    resetViewAction->setToolTip("重置视图");
    connect(resetViewAction, &QAction::triggered, this, [this]() {
        if (pointCloudView) {
            pointCloudView->resetView();
        }
    });
    viewGroup->addSecondaryWidget(createIconButton(resetViewAction, viewGroup, toolbarIconSize));
    viewGroup->moreMenu()->addSeparator();
    viewGroup->moreMenu()->addAction(resetViewAction);
    toolbarLayout->addWidget(viewGroup);
    toolbarLayout->addWidget(transformGroup);

    return viewerToolbar;
}

void LivoxViewerWindow::updateProjectionControlsVisibility()
{
    bool isSpherical = false;
    if (QWidget* control = parameterState.controls.value(kKeyPclDataType, nullptr)) {
        if (QComboBox* combo = qobject_cast<QComboBox*>(control)) {
            isSpherical = (combo->currentIndex() == 2);
        }
    }

    if (projectionControlsGroup) {
        projectionControlsGroup->setVisible(isSpherical);
    }
    if (projectionDepthCheck) {
        projectionDepthCheck->setEnabled(isSpherical);
    }
    if (projectionDepthSpin) {
        projectionDepthSpin->setEnabled(isSpherical && projectionDepthEnabled);
    }
    if (planarProjectionCheck) {
        planarProjectionCheck->setEnabled(isSpherical);
    }
    if (planarRadiusSpin) {
        planarRadiusSpin->setEnabled(isSpherical && planarProjectionEnabled);
    }
}

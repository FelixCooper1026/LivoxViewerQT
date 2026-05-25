#include "LivoxViewerWindow.h"

#include <QActionGroup>
#include <QIcon>
#include <QLayout>
#include <QMenu>
#include <QSignalBlocker>
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
            "  background: #dedede;"
            "  border-left: 1px solid #b9b9b9;"
            "  border-right: 1px solid #b9b9b9;"
            "}"
            "#ViewerToolbarGroup QLabel { color: #202020; }"
        );
    }

    QHBoxLayout* controlsLayout() const { return m_controlsLayout; }
    QMenu* moreMenu() const { return m_moreMenu; }

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

    const QSize toolbarIconSize(fontMetrics().height() + 4, fontMetrics().height() + 4);

    ToolbarGroup* displayGroup = new ToolbarGroup("显示控制", viewerToolbar);
    QSpinBox* spinFrameIntervalTop = new QSpinBox(displayGroup);
    spinFrameIntervalTop->setRange(100, 30000);
    spinFrameIntervalTop->setSingleStep(100);
    spinFrameIntervalTop->setSuffix(" ms");
    spinFrameIntervalTop->setValue(static_cast<int>(frameIntervalMs));
    spinFrameIntervalTop->setToolTip("点云积分时间/帧间隔");
    connect(spinFrameIntervalTop, QOverload<int>::of(&QSpinBox::valueChanged), this, &LivoxViewerWindow::onFrameIntervalChanged);
    QWidget* frameIntervalRow = createLabeledWidget("积分:", spinFrameIntervalTop, displayGroup);
    displayGroup->addPrimaryWidget(frameIntervalRow);

    pointSizeSpin = new QSpinBox(displayGroup);
    pointSizeSpin->setRange(1, 10);
    pointSizeSpin->setValue(static_cast<int>(pointSizePx));
    pointSizeSpin->setToolTip("点大小（像素）");
    connect(pointSizeSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &LivoxViewerWindow::onPointSizeChanged);
    QWidget* pointSizeRow = createLabeledWidget("点:", pointSizeSpin, displayGroup);
    displayGroup->addPrimaryWidget(pointSizeRow);

    QAction* gridAction = new QAction(QIcon(":/icons/grid.svg"), "世界坐标网格", this);
    gridAction->setCheckable(true);
    gridAction->setChecked(true);
    gridAction->setToolTip("显示/隐藏世界坐标网格");
    connect(gridAction, &QAction::toggled, this, [this](bool checked) {
        if (pointCloudView) {
            pointCloudView->setGridVisible(checked);
        }
    });
    displayGroup->addSecondaryWidget(createIconButton(gridAction, displayGroup, toolbarIconSize));

    colorModeCombo = new QComboBox(displayGroup);
    colorModeCombo->addItems({"反射率", "距离", "高度", "纯色"});
    colorModeCombo->setCurrentIndex(colorMode);
    colorModeCombo->setToolTip("点云着色模式");
    connect(colorModeCombo, QOverload<int>::of(&QComboBox::activated), this, &LivoxViewerWindow::onColorModeChanged);
    QWidget* colorModeRow = createLabeledWidget("着色:", colorModeCombo, displayGroup);
    displayGroup->addPrimaryWidget(colorModeRow);

    solidColorRow = new QWidget(displayGroup);
    QHBoxLayout* colorRowLayoutTop = new QHBoxLayout(solidColorRow);
    colorRowLayoutTop->setContentsMargins(0, 0, 0, 0);
    colorRowLayoutTop->setSpacing(6);
    solidColorPreview = new QFrame(solidColorRow);
    solidColorPreview->setFixedSize(20, 20);
    solidColorPreview->setFrameShape(QFrame::Box);
    solidColorPreview->setLineWidth(1);
    solidColorPreview->setStyleSheet(QString("background-color: %1;").arg(solidColor.name()));
    solidColorButton = new QPushButton("颜色", solidColorRow);
    colorRowLayoutTop->addWidget(solidColorPreview);
    colorRowLayoutTop->addWidget(solidColorButton);
    connect(solidColorButton, &QPushButton::clicked, this, &LivoxViewerWindow::onSolidColorClicked);
    solidColorRow->setProperty("toolbarOptionalHidden", colorMode != ColorSolid);
    solidColorRow->setVisible(colorMode == ColorSolid);
    solidColorRow->setEnabled(colorMode == ColorSolid);
    displayGroup->addSecondaryWidget(solidColorRow);

    QAction* visualizationAction = new QAction(style()->standardIcon(QStyle::SP_MediaPause), "点云可视化", this);
    visualizationAction->setCheckable(true);
    visualizationAction->setChecked(pointCloudVisualizationEnabled);
    auto syncVisualizationAction = [this, visualizationAction]() {
        visualizationAction->setText(pointCloudVisualizationEnabled ? "暂停点云可视化" : "开启点云可视化");
        visualizationAction->setIcon(style()->standardIcon(pointCloudVisualizationEnabled ? QStyle::SP_MediaPause : QStyle::SP_MediaPlay));
        QSignalBlocker blocker(visualizationAction);
        visualizationAction->setChecked(pointCloudVisualizationEnabled);
    };
    connect(visualizationAction, &QAction::triggered, this, [this, syncVisualizationAction](bool checked) {
        onPointCloudVisualizationToggled(checked);
        syncVisualizationAction();
    });
    syncVisualizationAction();
    displayGroup->addSecondaryWidget(createIconButton(visualizationAction, displayGroup, toolbarIconSize));

    addWidgetAction(displayGroup->moreMenu(), "积分时间", cloneSpinBox(spinFrameIntervalTop, displayGroup->moreMenu()));
    addWidgetAction(displayGroup->moreMenu(), "点大小", cloneSpinBox(pointSizeSpin, displayGroup->moreMenu()));
    displayGroup->moreMenu()->addAction(gridAction);
    QComboBox* overflowColorMode = new QComboBox(displayGroup->moreMenu());
    overflowColorMode->addItems({"反射率", "距离", "高度", "纯色"});
    overflowColorMode->setCurrentIndex(colorModeCombo->currentIndex());
    connect(colorModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), overflowColorMode, [overflowColorMode](int index) {
        QSignalBlocker blocker(overflowColorMode);
        overflowColorMode->setCurrentIndex(index);
    });
    connect(overflowColorMode, QOverload<int>::of(&QComboBox::activated), this, [this](int index) {
        if (colorModeCombo) {
            colorModeCombo->setCurrentIndex(index);
        }
        onColorModeChanged(index);
    });
    addWidgetAction(displayGroup->moreMenu(), "着色", overflowColorMode);
    displayGroup->moreMenu()->addAction("选择纯色...", this, &LivoxViewerWindow::onSolidColorClicked);
    displayGroup->moreMenu()->addAction(visualizationAction);
    toolbarLayout->addWidget(displayGroup);

    ToolbarGroup* projectionGroup = new ToolbarGroup("投影控制", viewerToolbar);
    projectionControlsGroup = projectionGroup;
    projectionDepthCheck = new QCheckBox("球面投影", projectionGroup);
    projectionDepthCheck->setChecked(projectionDepthEnabled);
    projectionDepthCheck->setToolTip("启用后按固定距离对深度进行投影，仅在球坐标点云时生效");
    connect(projectionDepthCheck, &QCheckBox::toggled, this, &LivoxViewerWindow::onProjectionDepthToggled);
    projectionGroup->addPrimaryWidget(projectionDepthCheck);

    projectionDepthSpin = new QDoubleSpinBox(projectionGroup);
    projectionDepthSpin->setRange(0.0, 10000.0);
    projectionDepthSpin->setDecimals(1);
    projectionDepthSpin->setSingleStep(1.0);
    projectionDepthSpin->setValue(projectionDepthMeters);
    projectionDepthSpin->setSuffix(" m");
    projectionDepthSpin->setToolTip("球坐标时，将 depth 投影到指定距离；0 表示使用原始 depth");
    connect(projectionDepthSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &LivoxViewerWindow::onProjectionDepthChanged);
    QWidget* depthRow = createLabeledWidget("深度:", projectionDepthSpin, projectionGroup);
    projectionGroup->addSecondaryWidget(depthRow);

    planarProjectionCheck = new QCheckBox("平面投影", projectionGroup);
    planarProjectionCheck->setChecked(planarProjectionEnabled);
    planarProjectionCheck->setToolTip("启用平面投影模式，将半球面展开为平面图");
    connect(planarProjectionCheck, &QCheckBox::toggled, this, &LivoxViewerWindow::onPlanarProjectionToggled);
    projectionGroup->addPrimaryWidget(planarProjectionCheck);

    planarRadiusSpin = new QDoubleSpinBox(projectionGroup);
    planarRadiusSpin->setRange(1.0, 1000.0);
    planarRadiusSpin->setDecimals(1);
    planarRadiusSpin->setSingleStep(1.0);
    planarRadiusSpin->setValue(planarProjectionRadius);
    planarRadiusSpin->setSuffix(" m");
    planarRadiusSpin->setToolTip("平面投影的半径大小");
    connect(planarRadiusSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &LivoxViewerWindow::onPlanarProjectionRadiusChanged);
    QWidget* radiusRow = createLabeledWidget("半径:", planarRadiusSpin, projectionGroup);
    projectionGroup->addSecondaryWidget(radiusRow);

    QAction* projectionDepthAction = projectionGroup->moreMenu()->addAction("球面投影");
    projectionDepthAction->setCheckable(true);
    projectionDepthAction->setChecked(projectionDepthEnabled);
    connect(projectionDepthAction, &QAction::toggled, projectionDepthCheck, &QCheckBox::setChecked);
    connect(projectionDepthCheck, &QCheckBox::toggled, projectionDepthAction, [projectionDepthAction](bool checked) {
        QSignalBlocker blocker(projectionDepthAction);
        projectionDepthAction->setChecked(checked);
    });
    addWidgetAction(projectionGroup->moreMenu(), "投影深度", cloneDoubleSpinBox(projectionDepthSpin, projectionGroup->moreMenu()));
    QAction* planarProjectionAction = projectionGroup->moreMenu()->addAction("平面投影");
    planarProjectionAction->setCheckable(true);
    planarProjectionAction->setChecked(planarProjectionEnabled);
    connect(planarProjectionAction, &QAction::toggled, planarProjectionCheck, &QCheckBox::setChecked);
    connect(planarProjectionCheck, &QCheckBox::toggled, planarProjectionAction, [planarProjectionAction](bool checked) {
        QSignalBlocker blocker(planarProjectionAction);
        planarProjectionAction->setChecked(checked);
    });
    addWidgetAction(projectionGroup->moreMenu(), "投影半径", cloneDoubleSpinBox(planarRadiusSpin, projectionGroup->moreMenu()));
    projectionGroup->setVisible(false);
    toolbarLayout->addWidget(projectionGroup);

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
        QSignalBlocker blocker(measureAction);
        measureAction->setChecked(enable);
        if (enable) {
            statusLabelBar->setText("测距模式：按住Ctrl+左键选择第一点");
            logMessage("进入测距模式，已暂停点云播放");
        } else {
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
    toolbarLayout->addWidget(toolsGroup);

    ToolbarGroup* viewGroup = new ToolbarGroup("视角控制", viewerToolbar);
    QComboBox* viewPresetCombo = new QComboBox(viewGroup);
    viewPresetCombo->addItems({"俯视图", "前视图", "左视图", "右视图", "后视图"});
    auto applyViewPreset = [this, viewPresetCombo](int index) {
        if (!pointCloudView) {
            return;
        }
        viewPresetCombo->setCurrentIndex(index);
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
    connect(viewPresetCombo, QOverload<int>::of(&QComboBox::activated), this, applyViewPreset);
    QWidget* viewPresetRow = createLabeledWidget("视角:", viewPresetCombo, viewGroup);
    viewGroup->addPrimaryWidget(viewPresetRow);

    QAction* resetViewAction = new QAction(QIcon(":/icons/reset_view.svg"), "重置视图", this);
    resetViewAction->setToolTip("重置视图");
    connect(resetViewAction, &QAction::triggered, this, [this]() {
        if (pointCloudView) {
            pointCloudView->resetView();
        }
    });
    viewGroup->addSecondaryWidget(createIconButton(resetViewAction, viewGroup, toolbarIconSize));

    QActionGroup* viewActionGroup = new QActionGroup(viewGroup);
    const QStringList viewNames = {"俯视图", "前视图", "左视图", "右视图", "后视图"};
    for (int i = 0; i < viewNames.size(); ++i) {
        QAction* action = viewGroup->moreMenu()->addAction(viewNames.at(i));
        action->setCheckable(true);
        viewActionGroup->addAction(action);
        action->setChecked(i == 0);
        connect(action, &QAction::triggered, this, [applyViewPreset, i]() {
            applyViewPreset(i);
        });
    }
    connect(viewPresetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), viewActionGroup, [viewActionGroup](int index) {
        QList<QAction*> actions = viewActionGroup->actions();
        if (index >= 0 && index < actions.size()) {
            QSignalBlocker blocker(actions.at(index));
            actions.at(index)->setChecked(true);
        }
    });
    viewGroup->moreMenu()->addSeparator();
    viewGroup->moreMenu()->addAction(resetViewAction);
    toolbarLayout->addWidget(viewGroup);

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

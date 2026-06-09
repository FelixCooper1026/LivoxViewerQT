#include "LivoxViewerWindow.h"

#include "ThemeIconUtils.h"
#include "widgets/SwitchCheckBox.h"
#include "plugins/StlModel/StlModelLoader.h"

#include <QActionGroup>
#include <QDir>
#include <QFileInfo>
#include <QIcon>
#include <QLayout>
#include <QMenu>
#include <QMessageBox>
#include <QSettings>
#include <QSignalBlocker>
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
    int compactPriority(QLayoutItem* item) const
    {
        QWidget* widget = item->widget();
        return widget ? widget->property("toolbarCompactPriority").toInt() : 0;
    }

    int itemFullWidth(QLayoutItem* item, int lineWidth) const
    {
        const int minWidth = itemMinWidth(item, lineWidth);
        return std::max(std::min(item->sizeHint().width(), lineWidth), minWidth);
    }

    int itemMinWidth(QLayoutItem* item, int lineWidth) const
    {
        return std::min(item->minimumSize().width(), lineWidth);
    }

    int totalWidth(const QVector<int>& widths) const
    {
        int total = 0;
        for (int width : widths) {
            total += width;
        }
        if (widths.size() > 1) {
            total += m_hSpacing * (widths.size() - 1);
        }
        return total;
    }

    QVector<int> rowWidths(const QVector<QLayoutItem*>& rowItems, int lineWidth) const
    {
        QVector<int> widths;
        QVector<int> minimums;
        QVector<int> compactIndexes;
        widths.reserve(rowItems.size());
        minimums.reserve(rowItems.size());

        for (int i = 0; i < rowItems.size(); ++i) {
            QLayoutItem* item = rowItems.at(i);
            const int fullWidth = itemFullWidth(item, lineWidth);
            const int minWidth = itemMinWidth(item, lineWidth);
            widths.append(fullWidth);
            minimums.append(minWidth);
            if (compactPriority(item) > 0 && minWidth < fullWidth) {
                compactIndexes.append(i);
            }
        }

        int usedWidth = totalWidth(widths);
        if (usedWidth <= lineWidth) {
            return widths;
        }

        std::sort(compactIndexes.begin(), compactIndexes.end(), [this, &rowItems](int lhs, int rhs) {
            const int lhsPriority = compactPriority(rowItems.at(lhs));
            const int rhsPriority = compactPriority(rowItems.at(rhs));
            if (lhsPriority == rhsPriority) {
                return lhs > rhs;
            }
            return lhsPriority < rhsPriority;
        });

        for (int index : compactIndexes) {
            usedWidth -= widths.at(index) - minimums.at(index);
            widths[index] = minimums.at(index);
            if (usedWidth <= lineWidth) {
                break;
            }
        }

        return widths;
    }

    int layoutRow(const QVector<QLayoutItem*>& rowItems, const QRect& effectiveRect, int y, int lineWidth, bool testOnly) const
    {
        if (rowItems.isEmpty()) {
            return 0;
        }

        const QVector<int> widths = rowWidths(rowItems, lineWidth);
        int x = effectiveRect.x();
        int rowHeight = 0;
        for (int i = 0; i < rowItems.size(); ++i) {
            QLayoutItem* item = rowItems.at(i);
            const QSize itemSize(widths.at(i), item->sizeHint().height());
            if (!testOnly) {
                item->setGeometry(QRect(QPoint(x, y), itemSize));
            }
            x += itemSize.width() + m_hSpacing;
            rowHeight = std::max(rowHeight, itemSize.height());
        }
        return rowHeight;
    }

    int doLayout(const QRect& rect, bool testOnly) const
    {
        QMargins margins = contentsMargins();
        QRect effectiveRect = rect.adjusted(margins.left(), margins.top(), -margins.right(), -margins.bottom());
        int y = effectiveRect.y();
        const int lineWidth = std::max(1, effectiveRect.width());
        QVector<QLayoutItem*> rowItems;

        for (QLayoutItem* item : m_items) {
            QWidget* widget = item->widget();
            if (widget && widget->isHidden()) {
                continue;
            }

            QVector<QLayoutItem*> candidate = rowItems;
            candidate.append(item);
            const QVector<int> candidateWidths = rowWidths(candidate, lineWidth);
            if (!rowItems.isEmpty() && totalWidth(candidateWidths) > lineWidth) {
                y += layoutRow(rowItems, effectiveRect, y, lineWidth, testOnly) + m_vSpacing;
                rowItems.clear();
            }

            rowItems.append(item);
        }

        y += layoutRow(rowItems, effectiveRect, y, lineWidth, testOnly);
        return y - rect.y() + margins.bottom();
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
        setFixedHeight(fontMetrics().height() * 4 + 6);

        QVBoxLayout* root = new QVBoxLayout(this);
        root->setContentsMargins(8, 5, 8, 0);
        root->setSpacing(5);

        m_controls = new QWidget(this);
        m_controlsLayout = new QHBoxLayout(m_controls);
        m_controlsLayout->setContentsMargins(0, 0, 0, 0);
        m_controlsLayout->setSpacing(6);
        m_controlsLayout->setAlignment(Qt::AlignVCenter);
        m_moreMenu = new QMenu(this);
        m_moreButton = new QToolButton(m_controls);
        m_moreButton->setArrowType(Qt::DownArrow);
        m_moreButton->setToolTip("更多");
        m_moreButton->setPopupMode(QToolButton::InstantPopup);
        m_moreButton->setMenu(m_moreMenu);
        m_moreButton->setAutoRaise(true);
        m_moreButton->setStyleSheet("QToolButton::menu-indicator { image: none; }");
        m_moreButton->setVisible(false);
        m_controlsLayout->addWidget(m_moreButton);
        root->addWidget(m_controls, 1);

        QWidget* titleRow = new QWidget(this);
        QHBoxLayout* titleLayout = new QHBoxLayout(titleRow);
        titleLayout->setContentsMargins(0, 2, 0, 0);
        titleLayout->setSpacing(3);
        titleLayout->addStretch();

        m_title = new QLabel(title, titleRow);
        QFont titleFont = m_title->font();
        titleFont.setPointSizeF(std::max(7.0, titleFont.pointSizeF() * 0.9));
        m_title->setFont(titleFont);
        m_title->setAlignment(Qt::AlignCenter);
        titleRow->setFixedHeight(m_title->sizeHint().height() + 5);
        titleLayout->addWidget(m_title);

        titleLayout->addStretch();
        root->addWidget(titleRow, 0, Qt::AlignBottom);

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

    void setCompactPriority(int priority)
    {
        m_compactPriority = priority;
        setProperty("toolbarCompactPriority", priority);
    }

    void addPrimaryWidget(QWidget* widget)
    {
        m_primaryWidgets.append(widget);
        m_controlsLayout->insertWidget(m_controlsLayout->count() - 1, widget);
    }

    void addSecondaryWidget(QWidget* widget)
    {
        m_secondaryWidgets.append(widget);
        m_controlsLayout->insertWidget(m_controlsLayout->count() - 1, widget);
    }

    QSize sizeHint() const override
    {
        return QSize(estimatedWidth(-1, true, false), height());
    }

    QSize minimumSizeHint() const override
    {
        return QSize(estimatedWidth(m_compactPriority > 0 ? 1 : -1, false, m_compactPriority > 0), height());
    }

protected:
    void resizeEvent(QResizeEvent* event) override
    {
        QWidget::resizeEvent(event);
        updateOverflow();
    }

private:
    int estimatedWidth(int primaryCount, bool includeSecondary, bool includeMore) const
    {
        const QMargins margins = layout()->contentsMargins();
        int controlsWidth = 0;

        int visibleControlCount = 0;
        auto addWidgetWidth = [&](QWidget* widget) {
            if (widget->property("toolbarOptionalHidden").toBool()) {
                return;
            }
            controlsWidth += widget->sizeHint().width();
            ++visibleControlCount;
        };

        if (primaryCount != 0) {
            const int primarySize = static_cast<int>(m_primaryWidgets.size());
            const int count = primaryCount < 0 ? primarySize : std::min(primaryCount, primarySize);
            for (int i = 0; i < count; ++i) {
                addWidgetWidth(m_primaryWidgets.at(i));
            }
        }
        if (includeSecondary) {
            for (QWidget* widget : m_secondaryWidgets) {
                addWidgetWidth(widget);
            }
        }
        if (includeMore && !m_moreMenu->actions().isEmpty()) {
            controlsWidth += m_moreButton->sizeHint().width();
            ++visibleControlCount;
        }

        if (visibleControlCount > 1) {
            controlsWidth += 6 * (visibleControlCount - 1);
        }

        const int titleWidth = m_title->sizeHint().width();
        return margins.left() + margins.right() + std::max(controlsWidth, titleWidth);
    }

    void updateOverflow()
    {
        const int widthNow = width();
        if (widthNow <= 0) {
            return;
        }

        const int fullWidth = estimatedWidth(-1, true, false);
        const bool compact = widthNow < fullWidth;
        const bool compactGroup = compact && m_compactPriority > 0;

        for (int i = 0; i < m_primaryWidgets.size(); ++i) {
            QWidget* widget = m_primaryWidgets.at(i);
            widget->setVisible((!compactGroup || i == 0) && !widget->property("toolbarOptionalHidden").toBool());
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
    int m_compactPriority = 0;
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
    ThemeIconUtils::setThemedSvgPixmap(icon, iconPath, iconSize);
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

QString stlModelKey(QString modelName)
{
    modelName = modelName.trimmed();
    const QString lower = modelName.toLower();
    if (lower.contains(QStringLiteral("mid360s"))) {
        return QStringLiteral("Mid360s");
    }
    if (lower.contains(QStringLiteral("mid360l"))) {
        return QStringLiteral("Mid360l");
    }
    if (lower.contains(QStringLiteral("mid360"))) {
        return QStringLiteral("Mid360");
    }
    if (lower.contains(QStringLiteral("avia2"))) {
        return QStringLiteral("Avia2");
    }
    if (lower.contains(QStringLiteral("hap"))) {
        return QStringLiteral("HAP");
    }
    if (lower == QStringLiteral("pa") || lower.contains(QStringLiteral("livox pa"))) {
        return QStringLiteral("PA");
    }
    return {};
}

QString stlModelPathForKey(const QString& modelKey)
{
    return QDir(QStringLiteral(LIVOX_VIEWER_SOURCE_DIR))
        .filePath(QStringLiteral("plugins/StlModel/models/%1.glb").arg(modelKey));
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
    ThemeIconUtils::setThemedSvgIcon(gridAction, QStringLiteral(":/icons/grid.svg"));
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
    ThemeIconUtils::setThemedSvgIcon(actionPointCloudVisualization, QStringLiteral(":/icons/point_cloud_live.svg"));
    actionPointCloudVisualization->setCheckable(true);
    actionPointCloudVisualization->setChecked(pointCloudVisualizationEnabled);
    connect(actionPointCloudVisualization, &QAction::triggered, this, [this](bool checked) {
        onPointCloudVisualizationToggled(checked);
    });
    syncPointCloudVisualizationAction();
    displayGroup->addPrimaryWidget(createIconButton(actionPointCloudVisualization, displayGroup, toolbarIconSize));

    QAction* stlModelAction = new QAction(QIcon(":/icons/3d_model_off.svg"), "GLB模型", this);
    ThemeIconUtils::setThemedSvgIcon(stlModelAction, QStringLiteral(":/icons/3d_model_off.svg"));
    stlModelAction->setCheckable(true);
    stlModelAction->setToolTip("显示设备GLB模型");
    connect(stlModelAction, &QAction::toggled, this, [stlModelAction](bool checked) {
        ThemeIconUtils::setThemedSvgIcon(stlModelAction,
            checked ? QStringLiteral(":/icons/3d_model_on.svg") : QStringLiteral(":/icons/3d_model_off.svg"));
    });
    QString loadedStlModelKey;
    connect(stlModelAction, &QAction::triggered, this, [this, stlModelAction, loadedStlModelKey](bool checked) mutable {
        if (!pointCloudView) {
            QSignalBlocker blocker(stlModelAction);
            stlModelAction->setChecked(false);
            ThemeIconUtils::setThemedSvgIcon(stlModelAction, QStringLiteral(":/icons/3d_model_off.svg"));
            return;
        }

        if (!checked) {
            pointCloudView->setStlModelVisible(false);
            statusLabelBar->setText("GLB模型已隐藏");
            logMessage("GLB模型已隐藏");
            return;
        }

        QString modelName;
        if (playbackState.active && !playbackState.devices.isEmpty()) {
            const Playback::DeviceInfo* selectedDevice = nullptr;
            for (const Playback::DeviceInfo& device : playbackState.devices) {
                if (playbackState.deviceVisible.value(device.lidarId, true)) {
                    selectedDevice = &device;
                    break;
                }
            }
            if (!selectedDevice) {
                selectedDevice = &playbackState.devices.first();
            }
            modelName = selectedDevice->modelDisplay.isEmpty()
                ? lvx2DeviceTypeToModel(selectedDevice->deviceType)
                : selectedDevice->modelDisplay;
        } else {
            LidarDeviceInfo currentDevice;
            if (tryGetCurrentDevice(currentDevice)) {
                modelName = currentDevice.product_info.isEmpty()
                    ? lvx2DeviceTypeToModel(currentDevice.dev_type)
                    : currentDevice.product_info;
            }
        }

        const QString modelKey = stlModelKey(modelName);
        if (modelKey.isEmpty()) {
            QMessageBox::warning(this, "显示GLB模型", QString("没有匹配的设备模型: %1").arg(modelName));
            QSignalBlocker blocker(stlModelAction);
            stlModelAction->setChecked(false);
            ThemeIconUtils::setThemedSvgIcon(stlModelAction, QStringLiteral(":/icons/3d_model_off.svg"));
            return;
        }

        const QString filePath = stlModelPathForKey(modelKey);
        if (!QFileInfo::exists(filePath)) {
            QMessageBox::warning(this, "显示GLB模型", QString("未找到GLB模型文件: %1").arg(QDir::toNativeSeparators(filePath)));
            QSignalBlocker blocker(stlModelAction);
            stlModelAction->setChecked(false);
            ThemeIconUtils::setThemedSvgIcon(stlModelAction, QStringLiteral(":/icons/3d_model_off.svg"));
            return;
        }

        if (loadedStlModelKey != modelKey || !pointCloudView->hasStlModel()) {
            StlModel::Mesh mesh;
            QString errorMessage;
            if (!StlModel::load(filePath, mesh, errorMessage)) {
                QMessageBox::warning(this, "显示GLB模型", errorMessage);
                QSignalBlocker blocker(stlModelAction);
                stlModelAction->setChecked(false);
                ThemeIconUtils::setThemedSvgIcon(stlModelAction, QStringLiteral(":/icons/3d_model_off.svg"));
                return;
            }

            const bool sourceXReversed = modelKey == QStringLiteral("Avia2");
            pointCloudView->setStlModelMesh(mesh, sourceXReversed);
            loadedStlModelKey = modelKey;
            statusLabelBar->setText(QString("GLB模型: %1 %2 面").arg(modelKey).arg(mesh.triangles.size() / 3));
            logMessage(QString("GLB模型已加载: %1").arg(QDir::toNativeSeparators(filePath)));
            return;
        }

        pointCloudView->setStlModelVisible(true);
        statusLabelBar->setText(QString("GLB模型已显示: %1").arg(modelKey));
        logMessage(QString("GLB模型已显示: %1").arg(modelKey));
    });
    displayGroup->addPrimaryWidget(createIconButton(stlModelAction, displayGroup, toolbarIconSize));

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
    QWidget* pointSizeRow = createIconLabeledWidget(":/icons/pixel_size.svg", "点大小（像素）", pointSizeSpin, displayGroup, toolbarIconSize);
    displayGroup->addPrimaryWidget(pointSizeRow);

    colorModeCombo = new QComboBox(displayGroup);
    colorModeCombo->addItems({"反射率", "距离", "高度", "纯色", "线号"});
    colorModeCombo->setCurrentIndex(colorMode);
    colorModeCombo->setToolTip("点云着色模式");
    QWidget* colorModeRow = createIconLabeledWidget(":/icons/paint_bucket.svg", "点云着色模式", colorModeCombo, displayGroup, toolbarIconSize);
    displayGroup->addPrimaryWidget(colorModeRow);

    displayGroup->moreMenu()->addAction(gridAction);
    displayGroup->moreMenu()->addAction(actionPointCloudVisualization);
    displayGroup->moreMenu()->addAction(stlModelAction);
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
    toolsGroup->setCompactPriority(2);
    QAction* measureAction = new QAction(QIcon(":/icons/measure.svg"), "点云测距", this);
    ThemeIconUtils::setThemedSvgIcon(measureAction, QStringLiteral(":/icons/measure.svg"));
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
    ThemeIconUtils::setThemedSvgIcon(selectionAction, QStringLiteral(":/icons/select_box.svg"));
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

    QAction* crossSectionAction = new QAction(QIcon(":/icons/cross_section.svg"), "Cross Section", this);
    ThemeIconUtils::setThemedSvgIcon(crossSectionAction, QStringLiteral(":/icons/cross_section.svg"));
    crossSectionAction->setCheckable(true);
    crossSectionAction->setToolTip("Cross Section");
    QAction* crossSectionControlsAction = new QAction("显示交互控件", this);
    crossSectionControlsAction->setCheckable(true);
    crossSectionControlsAction->setChecked(true);
    crossSectionControlsAction->setEnabled(false);
    crossSectionControlsAction->setVisible(false);
    SwitchCheckBox* crossSectionControlsSwitch = new SwitchCheckBox(toolsGroup);
    crossSectionControlsSwitch->setChecked(true);
    crossSectionControlsSwitch->setEnabled(false);
    crossSectionControlsSwitch->setVisible(false);
    crossSectionControlsSwitch->setProperty("toolbarOptionalHidden", true);
    crossSectionControlsSwitch->setToolTip("显示/隐藏Cross Section交互控件");
    connect(crossSectionControlsAction, &QAction::toggled, crossSectionControlsSwitch, [crossSectionControlsSwitch](bool checked) {
        QSignalBlocker blocker(crossSectionControlsSwitch);
        crossSectionControlsSwitch->setChecked(checked);
    });
    connect(crossSectionControlsSwitch, &QCheckBox::toggled, crossSectionControlsAction, [crossSectionControlsAction](bool checked) {
        crossSectionControlsAction->setChecked(checked);
    });
    connect(crossSectionAction, &QAction::triggered, this, [this, crossSectionAction, crossSectionControlsAction, crossSectionControlsSwitch, measureAction, selectionAction]() {
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
            crossSectionControlsSwitch->setEnabled(true);
            crossSectionControlsAction->setVisible(true);
            crossSectionControlsSwitch->setProperty("toolbarOptionalHidden", false);
            crossSectionControlsSwitch->setVisible(true);
            {
                QSignalBlocker blocker(crossSectionControlsAction);
                crossSectionControlsAction->setChecked(true);
            }
            {
                QSignalBlocker blocker(crossSectionControlsSwitch);
                crossSectionControlsSwitch->setChecked(true);
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
            crossSectionControlsSwitch->setEnabled(false);
            crossSectionControlsAction->setVisible(false);
            crossSectionControlsSwitch->setProperty("toolbarOptionalHidden", true);
            crossSectionControlsSwitch->setVisible(false);
            {
                QSignalBlocker blocker(crossSectionControlsAction);
                crossSectionControlsAction->setChecked(true);
            }
            {
                QSignalBlocker blocker(crossSectionControlsSwitch);
                crossSectionControlsSwitch->setChecked(true);
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

    toolsGroup->addPrimaryWidget(createIconButton(crossSectionAction, toolsGroup, toolbarIconSize));
    toolsGroup->addPrimaryWidget(crossSectionControlsSwitch);
    toolsGroup->moreMenu()->addAction(crossSectionAction);
    toolsGroup->moreMenu()->addAction(crossSectionControlsAction);
    toolbarLayout->addWidget(toolsGroup);

    ToolbarGroup* projectionGroup = new ToolbarGroup("投影控制", viewerToolbar);
    QActionGroup* projectionModeGroup = new QActionGroup(projectionGroup);
    projectionModeGroup->setExclusive(true);

    QAction* perspectiveProjectionAction = new QAction(QIcon(":/icons/projection_perspective.svg"), "透视投影", this);
    ThemeIconUtils::setThemedSvgIcon(perspectiveProjectionAction, QStringLiteral(":/icons/projection_perspective.svg"));
    perspectiveProjectionAction->setCheckable(true);
    perspectiveProjectionAction->setChecked(true);
    perspectiveProjectionAction->setToolTip("透视投影");
    projectionModeGroup->addAction(perspectiveProjectionAction);
    connect(perspectiveProjectionAction, &QAction::triggered, this, [this]() {
        pointCloudView->setProjectionMode(PointCloudView::ProjectionMode::Perspective);
    });
    projectionGroup->addPrimaryWidget(createIconButton(perspectiveProjectionAction, projectionGroup, toolbarIconSize));

    QAction* orthographicProjectionAction = new QAction(QIcon(":/icons/projection_orthographic.svg"), "正交投影", this);
    ThemeIconUtils::setThemedSvgIcon(orthographicProjectionAction, QStringLiteral(":/icons/projection_orthographic.svg"));
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
    viewGroup->setCompactPriority(1);
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
        ThemeIconUtils::setThemedSvgIcon(action, viewIcons.at(i));
        action->setToolTip(viewNames.at(i));
        connect(action, &QAction::triggered, this, [applyViewPreset, i]() {
            applyViewPreset(i);
        });
        viewGroup->addPrimaryWidget(createIconButton(action, viewGroup, toolbarIconSize));
        viewGroup->moreMenu()->addAction(action);
    }

    QAction* resetViewAction = new QAction(QIcon(":/icons/reset_view.svg"), "重置视图", this);
    ThemeIconUtils::setThemedSvgIcon(resetViewAction, QStringLiteral(":/icons/reset_view.svg"));
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

    ToolbarGroup* captureGroup = new ToolbarGroup("数据采集", viewerToolbar);

    QAction* pointCloudCaptureAction = new QAction(QIcon(":/icons/capture_camera.svg"), "点云录制", this);
    ThemeIconUtils::setThemedSvgIcon(pointCloudCaptureAction, QStringLiteral(":/icons/capture_camera.svg"));
    pointCloudCaptureAction->setToolTip("点云录制");
    connect(pointCloudCaptureAction, &QAction::triggered, this, &LivoxViewerWindow::showPointCloudCaptureDialog);
    captureGroup->addPrimaryWidget(createIconButton(pointCloudCaptureAction, captureGroup, toolbarIconSize));
    captureGroup->moreMenu()->addAction(pointCloudCaptureAction);

    QAction* imuCaptureAction = new QAction(QIcon(":/icons/capture_imu.svg"), "IMU数据采集", this);
    ThemeIconUtils::setThemedSvgIcon(imuCaptureAction, QStringLiteral(":/icons/capture_imu.svg"));
    imuCaptureAction->setToolTip("IMU数据采集");
    connect(imuCaptureAction, &QAction::triggered, this, &LivoxViewerWindow::showImuCaptureDialog);
    captureGroup->addPrimaryWidget(createIconButton(imuCaptureAction, captureGroup, toolbarIconSize));
    captureGroup->moreMenu()->addAction(imuCaptureAction);

    QAction* debugCaptureAction = new QAction(QIcon(":/icons/capture_debug.svg"), "Debug数据采集", this);
    ThemeIconUtils::setThemedSvgIcon(debugCaptureAction, QStringLiteral(":/icons/capture_debug.svg"));
    debugCaptureAction->setToolTip("Debug数据采集");
    connect(debugCaptureAction, &QAction::triggered, this, &LivoxViewerWindow::showDebugCaptureDialog);
    captureGroup->addPrimaryWidget(createIconButton(debugCaptureAction, captureGroup, toolbarIconSize));
    captureGroup->moreMenu()->addAction(debugCaptureAction);

    toolbarLayout->addWidget(captureGroup);
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

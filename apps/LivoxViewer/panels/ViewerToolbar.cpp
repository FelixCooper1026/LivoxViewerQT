#include "LivoxViewerWindow.h"

#include <QIcon>
#include <QStyle>

QWidget* LivoxViewerWindow::createViewerToolbar(QWidget* parent)
{
    QWidget* viewerToolbar = new QWidget(parent);
        viewerToolbar->setObjectName("ViewerToolbar");
        QVBoxLayout* viewerLayout = new QVBoxLayout(viewerToolbar);
        viewerLayout->setContentsMargins(8,8,8,8);
        viewerLayout->setSpacing(4);

        // 第一行工具栏
        QWidget* toolbarRow1 = new QWidget(viewerToolbar);
        QHBoxLayout* row1Layout = new QHBoxLayout(toolbarRow1);
        row1Layout->setContentsMargins(0,0,0,0);
        row1Layout->setSpacing(8);

        // 第二行工具栏
        QWidget* toolbarRow2 = new QWidget(viewerToolbar);
        QHBoxLayout* row2Layout = new QHBoxLayout(toolbarRow2);
        row2Layout->setContentsMargins(0,0,0,0);
        row2Layout->setSpacing(8);

        // 积分时间
        QLabel* lblFrame = new QLabel("积分时间:", toolbarRow1);
        QSpinBox* spinFrameIntervalTop = new QSpinBox(toolbarRow1);
        spinFrameIntervalTop->setRange(100, 30000);
        spinFrameIntervalTop->setSingleStep(100);
        spinFrameIntervalTop->setSuffix(" ms");
        spinFrameIntervalTop->setValue(static_cast<int>(frameIntervalMs));
        spinFrameIntervalTop->setToolTip("点云积分时间/帧间隔（渲染为滑动窗口显示）");
        connect(spinFrameIntervalTop, QOverload<int>::of(&QSpinBox::valueChanged), this, &LivoxViewerWindow::onFrameIntervalChanged);

        // 点大小
        QLabel* lblSize = new QLabel("点大小:", toolbarRow1);
        pointSizeSpin = new QSpinBox(toolbarRow1);
        pointSizeSpin->setRange(1, 10);
        pointSizeSpin->setValue(static_cast<int>(pointSizePx));
        pointSizeSpin->setToolTip("点大小（像素）");
        connect(pointSizeSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &LivoxViewerWindow::onPointSizeChanged);

        // 着色模式
        QLabel* lblColor = new QLabel("着色:", toolbarRow1);
        colorModeCombo = new QComboBox(toolbarRow1);
        colorModeCombo->addItems({"反射率", "距离", "高度", "纯色"});
        colorModeCombo->setCurrentIndex(colorMode);
        colorModeCombo->setToolTip("点云着色模式");
        connect(colorModeCombo, QOverload<int>::of(&QComboBox::activated), this, &LivoxViewerWindow::onColorModeChanged);

        // 网格显示控制
        QCheckBox* gridCheck = new QCheckBox("显示网格", toolbarRow2);
        gridCheck->setChecked(true); // 默认勾选
        gridCheck->setToolTip("显示/隐藏世界坐标网格 (1m间距)");
        connect(gridCheck, &QCheckBox::toggled, this, [this](bool checked) {
            if (pointCloudView) {
                pointCloudView->setGridVisible(checked); // 调用 PointCloudView 的控制函数
            }
        });

        // 球坐标深度投影
        projectionDepthCheck = new QCheckBox("球面投影", toolbarRow2);
        projectionDepthCheck->setChecked(projectionDepthEnabled);
        projectionDepthCheck->setToolTip("启用后按固定距离对深度进行投影，仅在球坐标点云时生效");
        connect(projectionDepthCheck, &QCheckBox::toggled, this, &LivoxViewerWindow::onProjectionDepthToggled);

        QLabel* lblProj = new QLabel("投影深度(m):", toolbarRow2);
        projectionDepthSpin = new QDoubleSpinBox(toolbarRow2);
        projectionDepthSpin->setRange(0.0, 10000.0);
        projectionDepthSpin->setDecimals(1);
        projectionDepthSpin->setSingleStep(1.0);
        projectionDepthSpin->setValue(projectionDepthMeters);
        projectionDepthSpin->setToolTip("球坐标时，将depth投影到指定距离；0表示使用原始depth");
        connect(projectionDepthSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &LivoxViewerWindow::onProjectionDepthChanged);

        // 平面投影控制
        planarProjectionCheck = new QCheckBox("平面投影", toolbarRow2);
        planarProjectionCheck->setChecked(planarProjectionEnabled);
        planarProjectionCheck->setToolTip("启用平面投影模式，将半球面展开为平面图");
        connect(planarProjectionCheck, &QCheckBox::toggled, this, &LivoxViewerWindow::onPlanarProjectionToggled);

        QLabel* lblPlanarRadius = new QLabel("投影半径(m):", toolbarRow2);
        planarRadiusSpin = new QDoubleSpinBox(toolbarRow2);
        planarRadiusSpin->setRange(1.0, 1000.0);
        planarRadiusSpin->setDecimals(1);
        planarRadiusSpin->setSingleStep(1.0);
        planarRadiusSpin->setValue(planarProjectionRadius);
        planarRadiusSpin->setToolTip("平面投影的半径大小");
        connect(planarRadiusSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &LivoxViewerWindow::onPlanarProjectionRadiusChanged);

        // 纯色选择控件
        solidColorRow = new QWidget(toolbarRow1);
        QHBoxLayout* colorRowLayoutTop = new QHBoxLayout(solidColorRow);
        colorRowLayoutTop->setContentsMargins(0,0,0,0);
        colorRowLayoutTop->setSpacing(6);
        solidColorPreview = new QFrame(solidColorRow);
        solidColorPreview->setFixedSize(20, 20);
        solidColorPreview->setFrameShape(QFrame::Box);
        solidColorPreview->setLineWidth(1);
        solidColorPreview->setStyleSheet(QString("background-color: %1;").arg(solidColor.name()));
        solidColorButton = new QPushButton("选择颜色", solidColorRow);
        colorRowLayoutTop->addWidget(solidColorPreview);
        colorRowLayoutTop->addWidget(solidColorButton);
        connect(solidColorButton, &QPushButton::clicked, this, &LivoxViewerWindow::onSolidColorClicked);
        solidColorRow->setEnabled(colorMode == ColorSolid);

        // 暂停/开启点云可视化按钮
        QPushButton* btnToggleVisualization = new QPushButton(pointCloudVisualizationEnabled ? "暂停可视化" : "开启可视化", toolbarRow1);
        btnToggleVisualization->setToolTip("暂停/开启点云可视化更新");
        connect(btnToggleVisualization, &QPushButton::clicked, [this, btnToggleVisualization]() {
            bool newState = !pointCloudVisualizationEnabled;
            onPointCloudVisualizationToggled(newState);
            btnToggleVisualization->setText(newState ? "暂停可视化" : "开启可视化");
        });

        // 操作按钮
        QPushButton* btnToggleSelection = new QPushButton("点云框选", toolbarRow1);
        QPushButton* btnMeasure = new QPushButton("点云测距", toolbarRow1);
        QPushButton* btnReset = new QPushButton("重置视图", toolbarRow1);
        const QSize toolbarIconSize(fontMetrics().height() + 4, fontMetrics().height() + 4);
        btnToggleSelection->setIcon(QIcon(":/icons/select_box.svg"));
        btnMeasure->setIcon(QIcon(":/icons/measure.svg"));
        btnReset->setIcon(QIcon(":/icons/reset_view.svg"));
        btnToggleSelection->setIconSize(toolbarIconSize);
        btnMeasure->setIconSize(toolbarIconSize);
        btnReset->setIconSize(toolbarIconSize);
        QLabel* lblViewPreset = new QLabel("视角:", toolbarRow1);
        QComboBox* viewPresetCombo = new QComboBox(toolbarRow1);
        viewPresetCombo->addItems({"俯视图", "前视图", "左视图", "右视图", "后视图"});
        connect(viewPresetCombo, QOverload<int>::of(&QComboBox::activated), [this](int index) {
            if (!pointCloudView) return;
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
            case 5:
            default:
                pointCloudView->setViewPreset(PointCloudView::ViewPreset::Top);
                break;
            }
        });
        connect(btnToggleSelection, &QPushButton::clicked, [this, btnToggleSelection]() {
            if (!pointCloudView) return;
            bool enable = !pointCloudView->isSelectionModeEnabled();
            pointCloudView->setSelectionModeEnabled(enable);
            if (!enable) {
                pointCloudView->clearSelectionAabb();
                // 立即记录清除日志并清空表格
                if (lastSelectionCount != -1) {
                    lastSelectionCount = -1;
                    logMessage("已清除框选");
                }
                updateSelectionTableAndLog();
                // 关闭点属性弹窗并停止日志
                if (attrDock) { attrDock->hide(); }
                selectionRealtimeEnabled = false;
                statusLabelBar->setText("已连接 - 采样中");
            } else {
                // 打开点属性弹窗
                if (attrDock) {
                    attrDock->show();
                    attrDock->raise();
                }
                selectionRealtimeEnabled = true;
                updateSelectionTableAndLog();
                statusLabelBar->setText("点云框选模式：按住Ctrl+左键拖动选择区域");
            }
            btnToggleSelection->setText(enable ? "退出框选" : "点云框选");
        });
        connect(btnReset, &QPushButton::clicked, [this]() { pointCloudView->resetView(); });

        // 测距按钮逻辑
        connect(btnMeasure, &QPushButton::clicked, [this, btnMeasure]() {
            if (!pointCloudView) return;
            bool enable = !pointCloudView->isMeasurementModeEnabled();
            pointCloudView->setMeasurementModeEnabled(enable);
            if (enable) {
                statusLabelBar->setText("测距模式：按住Ctrl+左键选择第一点");
                logMessage("进入测距模式，已暂停点云播放");
            } else {
                statusLabelBar->setText("已连接 - 采样中");
                logMessage("退出测距模式，恢复点云播放");
            }
            btnMeasure->setText(enable ? "退出测距" : "点云测距");
        });

        // 拼装第一行工具栏
        row1Layout->addWidget(lblFrame);
        row1Layout->addWidget(spinFrameIntervalTop);
        row1Layout->addSpacing(8);
        row1Layout->addWidget(lblSize);
        row1Layout->addWidget(pointSizeSpin);
        row1Layout->addSpacing(8);
        row1Layout->addWidget(lblColor);
        row1Layout->addWidget(colorModeCombo);
        row1Layout->addWidget(solidColorRow);
        row1Layout->addSpacing(8);
        row1Layout->addWidget(btnToggleVisualization);
        row1Layout->addSpacing(8);
        row1Layout->addWidget(btnMeasure);
        row1Layout->addWidget(btnToggleSelection);
        row1Layout->addWidget(btnReset);
        row1Layout->addWidget(lblViewPreset);
        row1Layout->addWidget(viewPresetCombo);
        row1Layout->addStretch();

        // 拼装第二行工具栏
        row2Layout->addWidget(gridCheck);
        row2Layout->addSpacing(8); // 增加一点间距
        row2Layout->addWidget(projectionDepthCheck);
        row2Layout->addWidget(lblProj);
        row2Layout->addWidget(projectionDepthSpin);
        row2Layout->addSpacing(8);
        row2Layout->addWidget(planarProjectionCheck);
        row2Layout->addWidget(lblPlanarRadius);
        row2Layout->addWidget(planarRadiusSpin);
        row2Layout->addStretch();

        // 将两行添加到主工具栏
        viewerLayout->addWidget(toolbarRow1);
        viewerLayout->addWidget(toolbarRow2);

        // 可视化窗口
    return viewerToolbar;
}

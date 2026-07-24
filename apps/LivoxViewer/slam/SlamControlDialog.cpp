#include "slam/SlamControlDialog.h"

#include "LivoxViewerWindow.h"
#include "slam/SlamUiBridge.h"

#include <QComboBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPixmap>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

SlamControlDialog::SlamControlDialog(LivoxViewerWindow* window, SlamUiBridge* bridge, QWidget* parent)
    : QDialog(parent)
    , m_window(window)
    , m_bridge(bridge)
{
    setWindowTitle(QStringLiteral("SLAM"));
    setModal(false);
    setAttribute(Qt::WA_DeleteOnClose, false);
    resize(640, 560);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(16, 14, 16, 14);
    layout->setSpacing(12);

    QFormLayout* inputForm = new QFormLayout();
    inputForm->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    inputForm->setHorizontalSpacing(14);
    inputForm->setVerticalSpacing(8);
    m_inputModeCombo = new QComboBox(this);
    m_inputModeCombo->addItem(QStringLiteral("离线 SLAM"), 0);
    m_inputModeCombo->addItem(QStringLiteral("在线 SLAM"), 1);
    m_inputModeCombo->setCurrentIndex(m_window && m_window->isOfflineSlamMode() ? 0 : 1);
    inputForm->addRow(QStringLiteral("输入模式:"), m_inputModeCombo);

    m_offlineSourceWidget = new QWidget(this);
    QHBoxLayout* offlineSourceLayout = new QHBoxLayout(m_offlineSourceWidget);
    offlineSourceLayout->setContentsMargins(0, 0, 0, 0);
    offlineSourceLayout->setSpacing(8);
    m_offlinePathLabel = new QLabel(QStringLiteral("未加载离线数据源"), m_offlineSourceWidget);
    m_offlinePathLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_offlinePathLabel->setWordWrap(true);
    QPushButton* loadPcapButton = new QPushButton(QStringLiteral("加载数据源..."), m_offlineSourceWidget);
    offlineSourceLayout->addWidget(m_offlinePathLabel, 1);
    offlineSourceLayout->addWidget(loadPcapButton);
    inputForm->addRow(QStringLiteral("离线数据:"), m_offlineSourceWidget);
    layout->addLayout(inputForm);

    QFormLayout* form = new QFormLayout();
    form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    form->setFormAlignment(Qt::AlignTop);
    form->setHorizontalSpacing(14);
    form->setVerticalSpacing(8);

    addField(form, QStringLiteral("状态"));
    addField(form, QStringLiteral("模式"));
    addField(form, QStringLiteral("后端"));
    addField(form, QStringLiteral("IMU 状态"));
    addField(form, QStringLiteral("输入 FPS"));
    addField(form, QStringLiteral("后端耗时"));
    addField(form, QStringLiteral("动态检测耗时"));
    addField(form, QStringLiteral("动态聚类耗时"));
    addField(form, QStringLiteral("丢帧数"));
    addField(form, QStringLiteral("当前位姿"));
    addField(form, QStringLiteral("轨迹点数"));
    addField(form, QStringLiteral("关键帧数"));
    addField(form, QStringLiteral("回环约束数"));
    addField(form, QStringLiteral("局部 ikd-tree 有效点数"));
    addField(form, QStringLiteral("世界系点云总数"));
    addField(form, QStringLiteral("机体系当前帧点数"));
    addField(form, QStringLiteral("完整全局地图点数"));
    addField(form, QStringLiteral("动态检测模式"));
    addField(form, QStringLiteral("动态目标点"));
    addField(form, QStringLiteral("FreeDOM 地图"));
    addField(form, QStringLiteral("FreeDOM 分阶段耗时"));
    addField(form, QStringLiteral("错误信息"));
    layout->addLayout(form, 1);

    QHBoxLayout* controlButtonLayout = new QHBoxLayout();
    controlButtonLayout->setContentsMargins(0, 0, 0, 0);
    controlButtonLayout->setSpacing(8);
    QPushButton* startButton = new QPushButton(QStringLiteral("启动"), this);
    QPushButton* pauseButton = new QPushButton(QStringLiteral("暂停"), this);
    QPushButton* stopButton = new QPushButton(QStringLiteral("停止"), this);
    QPushButton* resetButton = new QPushButton(QStringLiteral("重置"), this);
    QPushButton* clearButton = new QPushButton(QStringLiteral("清空显示"), this);
    controlButtonLayout->addWidget(startButton);
    controlButtonLayout->addWidget(pauseButton);
    controlButtonLayout->addWidget(stopButton);
    controlButtonLayout->addWidget(resetButton);
    controlButtonLayout->addWidget(clearButton);
    controlButtonLayout->addStretch();
    layout->addLayout(controlButtonLayout);

    QHBoxLayout* exportButtonLayout = new QHBoxLayout();
    exportButtonLayout->setContentsMargins(0, 0, 0, 0);
    exportButtonLayout->setSpacing(8);
    QPushButton* exportTrajectoryButton = new QPushButton(QStringLiteral("保存轨迹..."), this);
    QPushButton* exportMapButton = new QPushButton(QStringLiteral("保存完整全局地图..."), this);
    QPushButton* exportFreeDomPointMapButton =
        new QPushButton(QStringLiteral("保存 FreeDOM 静态点地图..."), this);
    QPushButton* exportFreeDomVoxelMapButton =
        new QPushButton(QStringLiteral("保存 FreeDOM 静态 Voxel 地图..."), this);
    QPushButton* previewFreeDomDepthButton =
        new QPushButton(QStringLiteral("预览 FreeDOM 深度图..."), this);
    exportTrajectoryButton->setObjectName(QStringLiteral("exportSlamTrajectoryButton"));
    exportMapButton->setObjectName(QStringLiteral("exportSlamMapButton"));
    exportButtonLayout->addWidget(exportTrajectoryButton);
    exportButtonLayout->addWidget(exportMapButton);
    exportButtonLayout->addWidget(exportFreeDomPointMapButton);
    exportButtonLayout->addWidget(exportFreeDomVoxelMapButton);
    exportButtonLayout->addWidget(previewFreeDomDepthButton);
    exportButtonLayout->addStretch();
    layout->addLayout(exportButtonLayout);

    connect(m_inputModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (!m_window) {
            return;
        }
        if (index == 0) {
            m_window->setSlamInputModeOffline();
        } else {
            m_window->setSlamInputModeOnline();
        }
        refreshInputControls();
    });
    connect(loadPcapButton, &QPushButton::clicked, this, [this]() {
        if (m_window) {
            m_window->loadOfflineSlamPcap();
        }
        refreshInputControls();
    });
    connect(startButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::startSlamProcessing);
    connect(pauseButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::pauseSlamProcessing);
    connect(stopButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::stopSlamProcessing);
    connect(resetButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::resetSlamProcessing);
    connect(clearButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::clearSlamDisplay);
    connect(exportTrajectoryButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::exportSlamTrajectoryFromDialog);
    connect(exportMapButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::exportSlamGlobalMapFromDialog);
    connect(exportFreeDomPointMapButton,
            &QPushButton::clicked,
            m_window,
            &LivoxViewerWindow::exportFreeDomStaticPointMapFromDialog);
    connect(exportFreeDomVoxelMapButton,
            &QPushButton::clicked,
            m_window,
            &LivoxViewerWindow::exportFreeDomStaticVoxelMapFromDialog);
    connect(previewFreeDomDepthButton, &QPushButton::clicked, this, [this]() {
        if (!m_bridge || m_bridge->freeDomDepthImage().isNull()) {
            QMessageBox::information(this,
                                     QStringLiteral("FreeDOM 深度图"),
                                     QStringLiteral("当前没有 FreeDOM 深度图快照。请启用 FreeDOM 调试可视化并运行 SLAM。"));
            return;
        }
        QDialog* preview = new QDialog(this);
        preview->setAttribute(Qt::WA_DeleteOnClose);
        preview->setWindowTitle(QStringLiteral("FreeDOM DepthImage / EnhancedDepthImage"));
        QHBoxLayout* previewLayout = new QHBoxLayout(preview);
        auto addImage = [preview, previewLayout](const QString& title, const QImage& image) {
            QVBoxLayout* column = new QVBoxLayout();
            QLabel* titleLabel = new QLabel(title, preview);
            QLabel* imageLabel = new QLabel(preview);
            imageLabel->setAlignment(Qt::AlignCenter);
            imageLabel->setPixmap(QPixmap::fromImage(image).scaled(
                560, 420, Qt::KeepAspectRatio, Qt::SmoothTransformation));
            column->addWidget(titleLabel);
            column->addWidget(imageLabel, 1);
            previewLayout->addLayout(column);
        };
        addImage(QStringLiteral("DepthImage"), m_bridge->freeDomDepthImage());
        addImage(QStringLiteral("EnhancedDepthImage"),
                 m_bridge->freeDomEnhancedDepthImage());
        preview->resize(1180, 520);
        preview->show();
    });
    if (m_bridge) {
        connect(m_bridge, &SlamUiBridge::displayStateChanged, this, &SlamControlDialog::refreshFields);
        connect(m_bridge, &SlamUiBridge::displayStateChanged, this, &SlamControlDialog::refreshInputControls);
    }

    refreshFields();
    refreshInputControls();
}

void SlamControlDialog::refreshFields()
{
    if (!m_bridge) {
        return;
    }

    const SlamUiBridge::DisplayState state = m_bridge->displayState();
    m_fields.value(QStringLiteral("状态"))->setText(state.status);
    m_fields.value(QStringLiteral("模式"))->setText(state.mode);
    m_fields.value(QStringLiteral("后端"))->setText(state.backend);
    m_fields.value(QStringLiteral("IMU 状态"))->setText(state.imuState);
    m_fields.value(QStringLiteral("输入 FPS"))->setText(state.inputFps);
    m_fields.value(QStringLiteral("后端耗时"))->setText(QStringLiteral("%1 ms").arg(state.backendMs));
    m_fields.value(QStringLiteral("动态检测耗时"))->setText(QStringLiteral("%1 ms").arg(state.dynamicDetectorMs));
    m_fields.value(QStringLiteral("动态聚类耗时"))->setText(QStringLiteral("%1 ms").arg(state.dynamicClusterMs));
    m_fields.value(QStringLiteral("丢帧数"))->setText(state.droppedFrames);
    m_fields.value(QStringLiteral("当前位姿"))->setText(state.currentPose);
    m_fields.value(QStringLiteral("轨迹点数"))->setText(state.trajectoryPoints);
    m_fields.value(QStringLiteral("关键帧数"))->setText(state.keyframeCount);
    m_fields.value(QStringLiteral("回环约束数"))->setText(state.loopClosureCount);
    m_fields.value(QStringLiteral("局部 ikd-tree 有效点数"))->setText(state.mapPoints);
    m_fields.value(QStringLiteral("世界系点云总数"))->setText(state.worldFramePoints);
    m_fields.value(QStringLiteral("机体系当前帧点数"))->setText(state.bodyFramePoints);
    m_fields.value(QStringLiteral("完整全局地图点数"))->setText(state.globalMapPoints);
    m_fields.value(QStringLiteral("动态检测模式"))->setText(state.dynamicMode);
    m_fields.value(QStringLiteral("动态目标点"))->setText(state.dynamicPoints);
    m_fields.value(QStringLiteral("FreeDOM 地图"))->setText(state.freeDomMap);
    m_fields.value(QStringLiteral("FreeDOM 分阶段耗时"))->setText(state.freeDomStages);
    m_fields.value(QStringLiteral("错误信息"))->setText(state.error);
}

void SlamControlDialog::refreshInputControls()
{
    if (!m_window) {
        return;
    }
    const bool offline = m_window->isOfflineSlamMode();
    if (m_inputModeCombo) {
        const QSignalBlocker blocker(m_inputModeCombo);
        m_inputModeCombo->setCurrentIndex(offline ? 0 : 1);
    }
    if (m_offlineSourceWidget) {
        m_offlineSourceWidget->setVisible(offline);
    }
    if (m_offlinePathLabel) {
        const QString path = m_window->offlineSlamPcapPath();
        m_offlinePathLabel->setText(path.isEmpty()
                                        ? QStringLiteral("未加载离线数据源")
                                        : QFileInfo(path).fileName());
        m_offlinePathLabel->setToolTip(path);
    }
}

QLabel* SlamControlDialog::addField(QFormLayout* form, const QString& name)
{
    QLabel* value = new QLabel(QStringLiteral("-"), this);
    value->setTextInteractionFlags(Qt::TextSelectableByMouse);
    value->setWordWrap(true);
    form->addRow(name + QStringLiteral(":"), value);
    m_fields.insert(name, value);
    return value;
}

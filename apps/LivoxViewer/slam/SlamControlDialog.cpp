#include "slam/SlamControlDialog.h"

#include "LivoxViewerWindow.h"
#include "slam/SlamUiBridge.h"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
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
    addField(form, QStringLiteral("丢帧数"));
    addField(form, QStringLiteral("当前位姿"));
    addField(form, QStringLiteral("轨迹点数"));
    addField(form, QStringLiteral("后端地图点数"));
    addField(form, QStringLiteral("世界系当前帧点数"));
    addField(form, QStringLiteral("机体系当前帧点数"));
    addField(form, QStringLiteral("完整全局地图点数"));
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
    QPushButton* exportCsvButton = new QPushButton(QStringLiteral("导出 CSV"), this);
    QPushButton* exportTumButton = new QPushButton(QStringLiteral("导出 TUM"), this);
    QPushButton* exportPcdButton = new QPushButton(QStringLiteral("导出完整全局地图 PCD"), this);
    QPushButton* exportLasButton = new QPushButton(QStringLiteral("导出完整全局地图 LAS"), this);
    exportCsvButton->setObjectName(QStringLiteral("exportSlamCsvButton"));
    exportTumButton->setObjectName(QStringLiteral("exportSlamTumButton"));
    exportPcdButton->setObjectName(QStringLiteral("exportSlamMapPcdButton"));
    exportLasButton->setObjectName(QStringLiteral("exportSlamMapLasButton"));
    exportButtonLayout->addWidget(exportCsvButton);
    exportButtonLayout->addWidget(exportTumButton);
    exportButtonLayout->addWidget(exportPcdButton);
    exportButtonLayout->addWidget(exportLasButton);
    exportButtonLayout->addStretch();
    layout->addLayout(exportButtonLayout);

    connect(startButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::startSlamProcessing);
    connect(pauseButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::pauseSlamProcessing);
    connect(stopButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::stopSlamProcessing);
    connect(resetButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::resetSlamProcessing);
    connect(clearButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::clearSlamDisplay);
    connect(exportCsvButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::exportSlamTrajectoryCsv);
    connect(exportTumButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::exportSlamTrajectoryTum);
    connect(exportPcdButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::exportSlamMapPcd);
    connect(exportLasButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::exportSlamMapLas);
    if (m_bridge) {
        connect(m_bridge, &SlamUiBridge::displayStateChanged, this, &SlamControlDialog::refreshFields);
    }

    refreshFields();
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
    m_fields.value(QStringLiteral("丢帧数"))->setText(state.droppedFrames);
    m_fields.value(QStringLiteral("当前位姿"))->setText(state.currentPose);
    m_fields.value(QStringLiteral("轨迹点数"))->setText(state.trajectoryPoints);
    m_fields.value(QStringLiteral("后端地图点数"))->setText(state.mapPoints);
    m_fields.value(QStringLiteral("世界系当前帧点数"))->setText(state.worldFramePoints);
    m_fields.value(QStringLiteral("机体系当前帧点数"))->setText(state.bodyFramePoints);
    m_fields.value(QStringLiteral("完整全局地图点数"))->setText(state.globalMapPoints);
    m_fields.value(QStringLiteral("错误信息"))->setText(state.error);
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

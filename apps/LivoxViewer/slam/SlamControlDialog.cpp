#include "slam/SlamControlDialog.h"

#include "LivoxViewerWindow.h"
#include "slam/SlamUiBridge.h"

#include <QCheckBox>
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
    resize(520, 420);

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
    addField(form, QStringLiteral("地图点数"));
    addField(form, QStringLiteral("错误信息"));
    layout->addLayout(form, 1);

    m_mapPreviewCheck = new QCheckBox(QStringLiteral("稀疏地图预览"), this);
    m_mapPreviewCheck->setChecked(false);
    layout->addWidget(m_mapPreviewCheck);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->setContentsMargins(0, 0, 0, 0);
    buttonLayout->setSpacing(8);
    QPushButton* startButton = new QPushButton(QStringLiteral("启动"), this);
    QPushButton* pauseButton = new QPushButton(QStringLiteral("暂停"), this);
    QPushButton* stopButton = new QPushButton(QStringLiteral("停止"), this);
    QPushButton* resetButton = new QPushButton(QStringLiteral("重置"), this);
    QPushButton* clearButton = new QPushButton(QStringLiteral("清空显示"), this);
    buttonLayout->addWidget(startButton);
    buttonLayout->addWidget(pauseButton);
    buttonLayout->addWidget(stopButton);
    buttonLayout->addWidget(resetButton);
    buttonLayout->addWidget(clearButton);
    buttonLayout->addStretch();
    layout->addLayout(buttonLayout);

    connect(startButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::startSlamProcessing);
    connect(pauseButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::pauseSlamProcessing);
    connect(stopButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::stopSlamProcessing);
    connect(resetButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::resetSlamProcessing);
    connect(clearButton, &QPushButton::clicked, m_window, &LivoxViewerWindow::clearSlamDisplay);
    connect(m_mapPreviewCheck, &QCheckBox::toggled, m_window, &LivoxViewerWindow::setSlamMapPreviewEnabled);
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
    m_fields.value(QStringLiteral("地图点数"))->setText(state.mapPoints);
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

#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"
#include "slam/SlamUiBridge.h"

namespace {

QFrame* createSlamStatusGroup(const QString& title, QWidget* parent, QVBoxLayout** bodyLayoutOut)
{
    QFrame* group = new QFrame(parent);
    group->setObjectName(QStringLiteral("SlamStatusGroup"));
    group->setFrameShape(QFrame::NoFrame);
    group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    QVBoxLayout* layout = new QVBoxLayout(group);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(6);

    QLabel* titleLabel = new QLabel(title, group);
    QFont titleFont = titleLabel->font();
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);
    layout->addWidget(titleLabel);

    QFrame* separator = new QFrame(group);
    separator->setFrameShape(QFrame::HLine);
    separator->setFrameShadow(QFrame::Plain);
    separator->setStyleSheet(QStringLiteral("color: palette(mid);"));
    layout->addWidget(separator);

    QVBoxLayout* bodyLayout = new QVBoxLayout();
    bodyLayout->setContentsMargins(0, 0, 0, 0);
    bodyLayout->setSpacing(0);
    layout->addLayout(bodyLayout, 1);
    *bodyLayoutOut = bodyLayout;
    return group;
}

QWidget* createSlamStatusField(const QString& title, QLabel** valueOut, QWidget* parent)
{
    QWidget* field = new QWidget(parent);
    field->setObjectName(QStringLiteral("SlamStatusField"));
    field->setMinimumWidth(0);
    field->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    field->setStyleSheet(QStringLiteral(
        "QWidget#SlamStatusField {"
        "  border-bottom: 1px solid palette(mid);"
        "}"));

    QHBoxLayout* layout = new QHBoxLayout(field);
    layout->setContentsMargins(0, 4, 0, 4);
    layout->setSpacing(10);

    QLabel* titleLabel = new QLabel(title, field);
    titleLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));
    titleLabel->setMinimumWidth(0);
    titleLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    QLabel* valueLabel = new QLabel(QStringLiteral("-"), field);
    valueLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    valueLabel->setWordWrap(false);
    valueLabel->setMinimumWidth(0);
    valueLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    valueLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    layout->addWidget(titleLabel, 1);
    layout->addWidget(valueLabel, 1);
    *valueOut = valueLabel;
    return field;
}

QLabel* createSlamStatusTextValue(QWidget* parent)
{
    QLabel* valueLabel = new QLabel(QStringLiteral("-"), parent);
    valueLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    valueLabel->setWordWrap(true);
    valueLabel->setMinimumWidth(0);
    valueLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    valueLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    return valueLabel;
}

} // namespace

void LivoxViewerWindow::createLogPanel()
{
    // 底部：日志 Dock
    logDock = new QDockWidget("日志", this);
    logDock->setObjectName("LogDock");
    logDock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    QWidget* hiddenLogTitleBar = new QWidget(logDock);
    hiddenLogTitleBar->setFixedHeight(0);
    logDock->setTitleBarWidget(hiddenLogTitleBar);

    QWidget* logDockContent = new QWidget(logDock);
    QVBoxLayout* logLayout = new QVBoxLayout(logDockContent);
    logLayout->setContentsMargins(6, 2, 6, 6);
    logLayout->setSpacing(4);

    QWidget* logHeader = new QWidget(logDockContent);
    QHBoxLayout* logHeaderLayout = new QHBoxLayout(logHeader);
    logHeaderLayout->setContentsMargins(0, 0, 0, 0);
    logHeaderLayout->setSpacing(0);
    QLabel* logTitle = new QLabel(QStringLiteral("日志"), logHeader);
    QFont logTitleFont = logTitle->font();
    logTitleFont.setBold(true);
    logTitle->setFont(logTitleFont);
    QToolButton* clearLogButton = new QToolButton(logHeader);
    QToolButton* copyLogButton = new QToolButton(logHeader);
    QCheckBox* autoScrollCheck = new QCheckBox(QStringLiteral("自动滚动"), logHeader);
    clearLogButton->setText(QStringLiteral("清空"));
    copyLogButton->setText(QStringLiteral("复制"));
    autoScrollCheck->setChecked(true);
    ThemeIconUtils::setThemedSvgIcon(clearLogButton, QStringLiteral(":/icons/convert_clear.svg"));
    ThemeIconUtils::setThemedSvgIcon(copyLogButton, QStringLiteral(":/icons/copy.svg"));
    for (QToolButton* button : {clearLogButton, copyLogButton}) {
        button->setAutoRaise(true);
        button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        button->setCursor(Qt::PointingHandCursor);
        button->setIconSize(QSize(fontMetrics().height() + 2, fontMetrics().height() + 2));
        button->setStyleSheet(QStringLiteral(
            "QToolButton {"
            "  border: none;"
            "  background: transparent;"
            "  color: palette(window-text);"
            "  padding: 0;"
            "}"
            "QToolButton:hover {"
            "  color: palette(highlight);"
            "}"
        ));
    }
    autoScrollCheck->setCursor(Qt::PointingHandCursor);

    QWidget* logActions = new QWidget(logHeader);
    QHBoxLayout* logActionsLayout = new QHBoxLayout(logActions);
    logActionsLayout->setContentsMargins(0, 0, 0, 0);
    logActionsLayout->setSpacing(20);
    logActionsLayout->addWidget(clearLogButton);
    logActionsLayout->addWidget(copyLogButton);
    logActionsLayout->addWidget(autoScrollCheck);

    logHeaderLayout->addWidget(logTitle);
    logHeaderLayout->addStretch();
    logHeaderLayout->addWidget(logActions);
    logLayout->addWidget(logHeader);

    QFrame* logFrame = new QFrame(logDockContent);
    logFrame->setObjectName(QStringLiteral("LogFrame"));
    logFrame->setFrameShape(QFrame::NoFrame);
    logFrame->setStyleSheet(QStringLiteral(
        "QFrame#LogFrame {"
        "  border: 0;"
        "  border-top: 1px solid palette(mid);"
        "  background: palette(base);"
        "}"
    ));
    QVBoxLayout* logFrameLayout = new QVBoxLayout(logFrame);
    logFrameLayout->setContentsMargins(6, 4, 6, 4);
    logFrameLayout->setSpacing(0);

    logText = new QTextEdit(logFrame);
    logText->setObjectName(QStringLiteral("LogText"));
    logText->setReadOnly(true);
    logText->setFrameShape(QFrame::NoFrame);
    logText->setMinimumHeight(50);
    logText->setProperty("autoScroll", true);
    logText->setStyleSheet(QStringLiteral(
        "QTextEdit#LogText {"
        "  border: none;"
        "  background: palette(base);"
        "}"
    ));
    connect(clearLogButton, &QPushButton::clicked, logText, &QTextEdit::clear);
    connect(copyLogButton, &QPushButton::clicked, this, [this]() {
        logText->selectAll();
        logText->copy();
    });
    connect(autoScrollCheck, &QCheckBox::toggled, logText, [this](bool checked) {
        logText->setProperty("autoScroll", checked);
    });
    logFrameLayout->addWidget(logText);
    logLayout->addWidget(logFrame);
    logDockContent->setLayout(logLayout);
    logDock->setWidget(logDockContent);
    logDock->setMinimumHeight(50);

    addDockWidget(Qt::BottomDockWidgetArea, logDock);
}

void LivoxViewerWindow::createSlamStatusPanel()
{
    slamStatusDock = new QDockWidget(QStringLiteral("SLAM状态"), this);
    slamStatusDock->setObjectName(QStringLiteral("SlamStatusDock"));
    slamStatusDock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    QWidget* hiddenTitleBar = new QWidget(slamStatusDock);
    hiddenTitleBar->setFixedHeight(0);
    slamStatusDock->setTitleBarWidget(hiddenTitleBar);

    QWidget* content = new QWidget(slamStatusDock);
    QVBoxLayout* root = new QVBoxLayout(content);
    root->setContentsMargins(6, 6, 6, 6);
    root->setSpacing(0);

    QFrame* fieldFrame = new QFrame(content);
    fieldFrame->setObjectName(QStringLiteral("SlamStatusFrame"));
    fieldFrame->setFrameShape(QFrame::NoFrame);
    fieldFrame->setMinimumHeight(0);
    fieldFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    fieldFrame->setStyleSheet(QStringLiteral(
        "QFrame#SlamStatusFrame {"
        "  border: 0;"
        "  border-top: 1px solid palette(mid);"
        "  background: palette(base);"
        "}"));
    QGridLayout* fieldLayout = new QGridLayout(fieldFrame);
    fieldLayout->setContentsMargins(0, 0, 0, 0);
    fieldLayout->setHorizontalSpacing(8);
    fieldLayout->setVerticalSpacing(8);
    fieldLayout->setAlignment(Qt::AlignTop);

    slamStatusFields.clear();

    auto addField = [this](QVBoxLayout* layout, QFrame* parent, const QString& name) {
        QLabel* value = nullptr;
        QWidget* field = createSlamStatusField(name, &value, parent);
        slamStatusFields.insert(name, value);
        layout->addWidget(field);
    };

    QVBoxLayout* statusBody = nullptr;
    QFrame* statusGroup = createSlamStatusGroup(QStringLiteral("状态摘要"), fieldFrame, &statusBody);
    addField(statusBody, statusGroup, QStringLiteral("状态"));
    addField(statusBody, statusGroup, QStringLiteral("模式"));
    addField(statusBody, statusGroup, QStringLiteral("后端"));
    addField(statusBody, statusGroup, QStringLiteral("IMU 状态"));
    statusBody->addStretch(1);

    QVBoxLayout* performanceBody = nullptr;
    QFrame* performanceGroup = createSlamStatusGroup(QStringLiteral("性能统计"), fieldFrame, &performanceBody);
    addField(performanceBody, performanceGroup, QStringLiteral("输入 FPS"));
    addField(performanceBody, performanceGroup, QStringLiteral("后端耗时"));
    addField(performanceBody, performanceGroup, QStringLiteral("丢帧数"));
    addField(performanceBody, performanceGroup, QStringLiteral("轨迹点数"));
    performanceBody->addStretch(1);

    QVBoxLayout* mapBody = nullptr;
    QFrame* mapGroup = createSlamStatusGroup(QStringLiteral("地图与点云信息"), fieldFrame, &mapBody);
    addField(mapBody, mapGroup, QStringLiteral("局部 ikd-tree 有效点数"));
    addField(mapBody, mapGroup, QStringLiteral("世界系点云总数"));
    addField(mapBody, mapGroup, QStringLiteral("机体系当前帧点数"));
    addField(mapBody, mapGroup, QStringLiteral("完整全局地图点数"));
    mapBody->addStretch(1);

    QVBoxLayout* poseBody = nullptr;
    QFrame* poseGroup = createSlamStatusGroup(QStringLiteral("当前位姿"), fieldFrame, &poseBody);
    QLabel* poseValue = createSlamStatusTextValue(poseGroup);
    slamStatusFields.insert(QStringLiteral("当前位姿"), poseValue);
    poseBody->addWidget(poseValue);
    poseBody->addStretch(1);

    QVBoxLayout* errorBody = nullptr;
    QFrame* errorGroup = createSlamStatusGroup(QStringLiteral("错误信息"), fieldFrame, &errorBody);
    QLabel* errorValue = createSlamStatusTextValue(errorGroup);
    slamStatusFields.insert(QStringLiteral("错误信息"), errorValue);
    errorBody->addWidget(errorValue);

    fieldLayout->addWidget(statusGroup, 0, 0);
    fieldLayout->addWidget(performanceGroup, 0, 1);
    fieldLayout->addWidget(mapGroup, 0, 2);
    fieldLayout->addWidget(poseGroup, 0, 3);
    fieldLayout->addWidget(errorGroup, 1, 0, 1, 4);
    for (int column = 0; column < 4; ++column) {
        fieldLayout->setColumnStretch(column, 1);
    }
    fieldLayout->setColumnStretch(2, 1);
    fieldLayout->setRowStretch(2, 1);

    QScrollArea* statusScroll = new QScrollArea(content);
    statusScroll->setWidgetResizable(true);
    statusScroll->setFrameShape(QFrame::NoFrame);
    statusScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    statusScroll->setMinimumHeight(0);
    statusScroll->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Ignored);
    statusScroll->setWidget(fieldFrame);

    root->addWidget(statusScroll, 1);
    content->setLayout(root);
    content->setMinimumHeight(0);
    slamStatusDock->setWidget(content);
    slamStatusDock->setMinimumHeight(50);

    addDockWidget(Qt::BottomDockWidgetArea, slamStatusDock);
    tabifyDockWidget(logDock, slamStatusDock);
    slamStatusDock->hide();
}

void LivoxViewerWindow::tabifySlamStatusPanel()
{
    if (!logDock || !slamStatusDock) {
        return;
    }
    if (tabifiedDockWidgets(logDock).contains(slamStatusDock)) {
        return;
    }
    if (dockWidgetArea(logDock) == Qt::NoDockWidgetArea) {
        addDockWidget(Qt::BottomDockWidgetArea, logDock);
    }
    if (dockWidgetArea(slamStatusDock) == Qt::NoDockWidgetArea) {
        addDockWidget(Qt::BottomDockWidgetArea, slamStatusDock);
    }
    tabifyDockWidget(logDock, slamStatusDock);
}

void LivoxViewerWindow::showSlamStatusPanel()
{
    if (!slamStatusDock) {
        return;
    }
    tabifySlamStatusPanel();
    slamStatusDock->show();
    slamStatusDock->raise();
}

void LivoxViewerWindow::updateSlamStatusPanel()
{
    if (!slamUiBridge || slamStatusFields.isEmpty()) {
        return;
    }

    const SlamUiBridge::DisplayState state = slamUiBridge->displayState();
    auto setField = [this](const QString& name, const QString& text) {
        QLabel* label = slamStatusFields.value(name, nullptr);
        if (!label) {
            return;
        }
        const QString displayText = text.isEmpty() ? QStringLiteral("-") : text;
        label->setText(label->wordWrap()
                           ? displayText
                           : label->fontMetrics().elidedText(displayText, Qt::ElideMiddle, label->width()));
        label->setToolTip(text);
    };

    setField(QStringLiteral("状态"), state.status);
    setField(QStringLiteral("模式"), state.mode);
    setField(QStringLiteral("后端"), state.backend);
    setField(QStringLiteral("IMU 状态"), state.imuState);
    setField(QStringLiteral("输入 FPS"), state.inputFps);
    setField(QStringLiteral("后端耗时"), QStringLiteral("%1 ms").arg(state.backendMs));
    setField(QStringLiteral("丢帧数"), state.droppedFrames);
    setField(QStringLiteral("当前位姿"), state.currentPose);
    setField(QStringLiteral("轨迹点数"), state.trajectoryPoints);
    setField(QStringLiteral("局部 ikd-tree 有效点数"), state.mapPoints);
    setField(QStringLiteral("世界系点云总数"), state.worldFramePoints);
    setField(QStringLiteral("机体系当前帧点数"), state.bodyFramePoints);
    setField(QStringLiteral("完整全局地图点数"), state.globalMapPoints);
    setField(QStringLiteral("错误信息"), state.error);
}

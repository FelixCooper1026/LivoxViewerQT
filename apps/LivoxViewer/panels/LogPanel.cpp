#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"
#include "slam/SlamUiBridge.h"

namespace {

QWidget* createSlamStatusField(const QString& title, QLabel** valueOut, QWidget* parent)
{
    QWidget* field = new QWidget(parent);
    field->setObjectName(QStringLiteral("SlamStatusField"));
    field->setMinimumWidth(0);
    field->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QHBoxLayout* layout = new QHBoxLayout(field);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(4);

    QLabel* titleLabel = new QLabel(title + QStringLiteral(":"), field);
    titleLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));
    titleLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    QLabel* valueLabel = new QLabel(QStringLiteral("-"), field);
    valueLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    valueLabel->setWordWrap(false);
    valueLabel->setMinimumWidth(0);
    valueLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    layout->addWidget(titleLabel);
    layout->addWidget(valueLabel, 1);
    *valueOut = valueLabel;
    return field;
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
    root->setContentsMargins(6, 2, 6, 6);
    root->setSpacing(4);

    QWidget* header = new QWidget(content);
    header->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QHBoxLayout* headerLayout = new QHBoxLayout(header);
    headerLayout->setContentsMargins(0, 0, 0, 0);
    headerLayout->setSpacing(0);
    QLabel* title = new QLabel(QStringLiteral("SLAM状态"), header);
    QFont titleFont = title->font();
    titleFont.setBold(true);
    title->setFont(titleFont);
    headerLayout->addWidget(title);
    headerLayout->addStretch();
    root->addWidget(header, 0, Qt::AlignTop);

    QFrame* fieldFrame = new QFrame(content);
    fieldFrame->setObjectName(QStringLiteral("SlamStatusFrame"));
    fieldFrame->setFrameShape(QFrame::NoFrame);
    fieldFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    fieldFrame->setStyleSheet(QStringLiteral(
        "QFrame#SlamStatusFrame {"
        "  border: 0;"
        "  border-top: 1px solid palette(mid);"
        "  background: palette(base);"
        "}"));
    QGridLayout* fieldLayout = new QGridLayout(fieldFrame);
    fieldLayout->setContentsMargins(6, 4, 6, 4);
    fieldLayout->setHorizontalSpacing(12);
    fieldLayout->setVerticalSpacing(3);
    fieldLayout->setAlignment(Qt::AlignTop);

    auto addField = [this, fieldFrame, fieldLayout](const QString& name, int row, int column, int columnSpan = 1) {
        QLabel* value = nullptr;
        QWidget* field = createSlamStatusField(name, &value, fieldFrame);
        slamStatusFields.insert(name, value);
        fieldLayout->addWidget(field, row, column, 1, columnSpan);
    };

    addField(QStringLiteral("状态"), 0, 0);
    addField(QStringLiteral("模式"), 0, 1);
    addField(QStringLiteral("后端"), 0, 2);
    addField(QStringLiteral("IMU 状态"), 0, 3);
    addField(QStringLiteral("输入 FPS"), 0, 4);
    addField(QStringLiteral("后端耗时"), 0, 5);
    addField(QStringLiteral("丢帧数"), 0, 6);
    addField(QStringLiteral("轨迹点数"), 1, 0);
    addField(QStringLiteral("局部 ikd-tree 有效点数"), 1, 1, 2);
    addField(QStringLiteral("世界系点云总数"), 1, 3);
    addField(QStringLiteral("机体系当前帧点数"), 1, 4);
    addField(QStringLiteral("完整全局地图点数"), 1, 5, 2);
    addField(QStringLiteral("当前位姿"), 2, 0, 4);
    addField(QStringLiteral("错误信息"), 2, 4, 3);
    for (int column = 0; column < 7; ++column) {
        fieldLayout->setColumnStretch(column, 1);
    }
    fieldLayout->setRowStretch(3, 1);

    root->addWidget(fieldFrame, 1);
    content->setLayout(root);
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
        label->setText(label->fontMetrics().elidedText(text.isEmpty() ? QStringLiteral("-") : text,
                                                       Qt::ElideMiddle,
                                                       label->width()));
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

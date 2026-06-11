#include "LivoxViewerWindow.h"
#include "ThemeIconUtils.h"

#include <QAbstractItemView>
#include <QButtonGroup>
#include <QDesktopServices>
#include <QDir>
#include <QDialog>
#include <QFileDialog>
#include <QFont>
#include <QFrame>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QToolButton>
#include <QUrl>
#include <QSize>
#include <QSizePolicy>
#include <QVBoxLayout>

#include <algorithm>

namespace {

constexpr int kTaskColumnType = 0;
constexpr int kTaskColumnPath = 1;
constexpr int kTaskColumnStatus = 2;
constexpr int kTaskColumnRemaining = 3;
constexpr int kTaskColumnAction = 4;

constexpr int kDebugColumnType = 0;
constexpr int kDebugColumnPath = 1;
constexpr int kDebugColumnDuration = 2;
constexpr int kDebugColumnStatus = 3;
constexpr int kDebugColumnRemaining = 4;
constexpr int kDebugColumnAction = 5;
constexpr int kTaskTableRowHeight = 44;
constexpr int kTaskTableFramePadding = 1;

QString defaultDocumentsDir()
{
    QString dir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    return dir.isEmpty() ? QDir::homePath() : dir;
}

QString selectFolder(QWidget* parent, const QString& startDir, const QString& title)
{
    QFileDialog dialog(parent, title, startDir);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setOption(QFileDialog::ShowDirsOnly, true);
    dialog.setFileMode(QFileDialog::Directory);
    if (dialog.exec() != QDialog::Accepted) {
        return {};
    }
    const QStringList selected = dialog.selectedFiles();
    return selected.isEmpty() ? QString() : selected.first();
}

QPushButton* createNavButton(const QString& text, QWidget* parent)
{
    QPushButton* button = new QPushButton(text, parent);
    button->setCheckable(true);
    button->setMinimumHeight(44);
    button->setCursor(Qt::PointingHandCursor);
    button->setStyleSheet(
        "QPushButton {"
        "  text-align: left;"
        "  padding: 8px 14px;"
        "  border: none;"
        "  border-radius: 6px;"
        "}"
        "QPushButton:hover { background: palette(base); }"
        "QPushButton:checked {"
        "  background: palette(highlight);"
        "  color: palette(highlighted-text);"
        "}"
    );
    return button;
}

QToolButton* createSmallIconButton(const QString& iconPath, const QString& tooltip, QWidget* parent)
{
    QToolButton* button = new QToolButton(parent);
    ThemeIconUtils::setThemedSvgIcon(button, iconPath);
    button->setIconSize(QSize(20, 20));
    button->setToolTip(tooltip);
    button->setAccessibleName(tooltip);
    button->setAutoRaise(true);
    button->setCursor(Qt::PointingHandCursor);
    button->setFixedSize(34, 34);
    return button;
}

QToolButton* createBrowseButton(QWidget* parent)
{
    return createSmallIconButton(QStringLiteral(":/icons/convert_browse_folder.svg"), QStringLiteral("浏览"), parent);
}

QPushButton* createStartButton(const QString& text, QWidget* parent)
{
    QPushButton* button = new QPushButton(text, parent);
    button->setMinimumSize(140, 44);
    button->setStyleSheet(
        "QPushButton {"
        "  background: #18bff0;"
        "  color: white;"
        "  border: none;"
        "  border-radius: 6px;"
        "  font-weight: 600;"
        "}"
        "QPushButton:disabled {"
        "  background: palette(mid);"
        "  color: palette(button-text);"
        "}"
    );
    return button;
}

QTableWidget* createTaskTable(QWidget* parent, const QStringList& headers)
{
    QTableWidget* table = new QTableWidget(parent);
    table->setObjectName(QStringLiteral("CaptureTaskTable"));
    table->setColumnCount(headers.size());
    table->setHorizontalHeaderLabels(headers);
    table->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    for (int column = 0; column < table->columnCount(); ++column) {
        table->horizontalHeaderItem(column)->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    }
    table->verticalHeader()->setVisible(false);
    table->verticalHeader()->setDefaultSectionSize(kTaskTableRowHeight);
    table->setSelectionMode(QAbstractItemView::NoSelection);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setAlternatingRowColors(true);
    table->setShowGrid(false);
    table->setFocusPolicy(Qt::NoFocus);
    table->setFrameShape(QFrame::NoFrame);
    table->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    table->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    table->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    table->horizontalHeader()->setStretchLastSection(false);
    table->horizontalHeader()->setSectionsClickable(false);
    table->horizontalHeader()->setHighlightSections(false);
    table->setStyleSheet(
        "QTableWidget#CaptureTaskTable { border: none; gridline-color: transparent; background: palette(base); }"
        "QTableWidget#CaptureTaskTable::viewport { background: palette(base); }"
        "QTableWidget::item { border: none; padding-left: 8px; padding-right: 18px; }"
        "QHeaderView::section { border: none; padding: 6px 18px 6px 8px; }"
    );
    return table;
}

int taskTableContentHeight(QTableWidget* table)
{
    return table->horizontalHeader()->sizeHint().height()
        + table->rowCount() * table->verticalHeader()->defaultSectionSize();
}

QFrame* createTaskTableFrame(QTableWidget* table, QWidget* parent)
{
    QFrame* frame = new QFrame(parent);
    frame->setObjectName(QStringLiteral("CaptureTaskTableFrame"));
    frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    frame->setStyleSheet(
        "#CaptureTaskTableFrame {"
        "  background: palette(base);"
        "  border: 1px solid palette(mid);"
        "  border-radius: 6px;"
        "}"
    );

    QVBoxLayout* layout = new QVBoxLayout(frame);
    layout->setContentsMargins(kTaskTableFramePadding, kTaskTableFramePadding, kTaskTableFramePadding, kTaskTableFramePadding);
    layout->setSpacing(0);
    layout->addWidget(table);

    const int contentHeight = taskTableContentHeight(table);
    table->setMinimumHeight(contentHeight);
    frame->setMinimumHeight(contentHeight + kTaskTableFramePadding * 2);
    return frame;
}

QWidget* createProgressCell(QProgressBar** progressOut, QLabel** labelOut, QLabel** iconOut)
{
    QWidget* cell = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(cell);
    layout->setContentsMargins(0, 4, 0, 4);
    layout->setSpacing(4);

    QWidget* statusRow = new QWidget(cell);
    QHBoxLayout* statusLayout = new QHBoxLayout(statusRow);
    statusLayout->setContentsMargins(0, 0, 0, 0);
    statusLayout->setSpacing(6);

    QLabel* icon = new QLabel(statusRow);
    icon->setFixedSize(16, 16);
    icon->setVisible(false);
    QLabel* label = new QLabel(QStringLiteral("待开始"), statusRow);
    QProgressBar* progress = new QProgressBar(cell);
    progress->setRange(0, 100);
    progress->setValue(0);
    progress->setTextVisible(false);
    progress->setFixedHeight(8);

    statusLayout->addWidget(icon);
    statusLayout->addWidget(label);
    statusLayout->addStretch();
    layout->addWidget(statusRow);
    layout->addWidget(progress);
    *progressOut = progress;
    *labelOut = label;
    *iconOut = icon;
    return cell;
}

QWidget* createSingleOpenActionCell(QToolButton** openButtonOut)
{
    QWidget* cell = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(cell);
    layout->setContentsMargins(0, 0, 18, 0);
    layout->setSpacing(6);
    QToolButton* openButton = createSmallIconButton(QStringLiteral(":/icons/convert_open_folder.svg"), QStringLiteral("打开文件所在目录"), cell);
    layout->addWidget(openButton);
    layout->addStretch();
    *openButtonOut = openButton;
    return cell;
}

QWidget* createDebugActionCell(QPushButton** startButtonOut, QToolButton** openButtonOut)
{
    QWidget* cell = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(cell);
    layout->setContentsMargins(0, 0, 18, 0);
    layout->setSpacing(8);

    QPushButton* startButton = new QPushButton(QStringLiteral("开始"), cell);
    startButton->setMinimumWidth(68);
    QToolButton* openButton = createSmallIconButton(QStringLiteral(":/icons/convert_open_folder.svg"), QStringLiteral("打开文件所在目录"), cell);
    layout->addWidget(startButton);
    layout->addWidget(openButton);
    layout->addStretch();

    *startButtonOut = startButton;
    *openButtonOut = openButton;
    return cell;
}

void setTextItem(QTableWidget* table, int row, int column, const QString& text)
{
    QTableWidgetItem* item = table->item(row, column);
    if (!item) {
        item = new QTableWidgetItem();
        table->setItem(row, column, item);
    }
    item->setText(text);
    item->setToolTip(text);
}

bool taskRunning(const CaptureTaskState& task)
{
    return task.status == CaptureTaskStatus::Running;
}

int taskProgressPercent(const CaptureTaskState& task)
{
    if (task.status == CaptureTaskStatus::Done) {
        return 100;
    }
    if (task.expectedFiles > 0) {
        return std::clamp(task.savedFiles * 100 / task.expectedFiles, 0, 100);
    }
    if (task.status != CaptureTaskStatus::Running || task.totalSeconds <= 0) {
        return 0;
    }
    const int done = std::clamp(task.totalSeconds - task.secondsRemaining, 0, task.totalSeconds);
    return done * 100 / task.totalSeconds;
}

QString remainingText(const CaptureTaskState& task)
{
    if (task.expectedFiles > 0) {
        if (task.status == CaptureTaskStatus::Running) {
            return QStringLiteral("%1 帧").arg(std::max(0, task.expectedFiles - task.savedFiles));
        }
        if (task.status == CaptureTaskStatus::Done) {
            return QStringLiteral("0 帧");
        }
        return QStringLiteral("-");
    }
    if (task.status == CaptureTaskStatus::Running) {
        return QStringLiteral("%1 s").arg(std::max(0, task.secondsRemaining));
    }
    if (task.status == CaptureTaskStatus::Done) {
        return QStringLiteral("0 s");
    }
    return QStringLiteral("-");
}

QString progressLabelText(const CaptureTaskState& task)
{
    if (task.status == CaptureTaskStatus::Running) {
        if (task.expectedFiles > 0) {
            return QStringLiteral("%1% (%2/%3)")
                .arg(taskProgressPercent(task))
                .arg(task.savedFiles)
                .arg(task.expectedFiles);
        }
        return QStringLiteral("%1%").arg(taskProgressPercent(task));
    }
    if (task.status == CaptureTaskStatus::Done) {
        if (task.expectedFiles > 0) {
            return QStringLiteral("已完成 (%1/%2)")
                .arg(task.savedFiles)
                .arg(task.expectedFiles);
        }
        return QStringLiteral("已完成");
    }
    if (task.status == CaptureTaskStatus::Failed) {
        if (task.expectedFiles > 0) {
            return QStringLiteral("失败 (%1/%2)")
                .arg(task.savedFiles)
                .arg(task.expectedFiles);
        }
        return QStringLiteral("失败");
    }
    return QStringLiteral("待开始");
}

void updateProgress(QProgressBar* progress, QLabel* label, QLabel* icon, const CaptureTaskState& task)
{
    progress->setValue(taskProgressPercent(task));
    label->setText(progressLabelText(task));
    if (task.status == CaptureTaskStatus::Done) {
        icon->setPixmap(QIcon(QStringLiteral(":/icons/status_done.svg")).pixmap(QSize(16, 16)));
        icon->setVisible(true);
        label->setStyleSheet(QStringLiteral("color: #2ecc71;"));
    } else if (task.status == CaptureTaskStatus::Failed) {
        icon->setPixmap(QIcon(QStringLiteral(":/icons/status_failed.svg")).pixmap(QSize(16, 16)));
        icon->setVisible(true);
        label->setStyleSheet(QStringLiteral("color: #dc5050;"));
    } else {
        icon->setVisible(false);
        label->setStyleSheet(QString());
    }
}

void openOutputDir(const QString& dir)
{
    if (!dir.isEmpty()) {
        QDesktopServices::openUrl(QUrl::fromLocalFile(dir));
    }
}

QString pointCloudFormatText(PointCloudCaptureFormat format)
{
    switch (format) {
    case PointCloudCaptureFormat::LVX2: return QStringLiteral("LVX2");
    case PointCloudCaptureFormat::PCD: return QStringLiteral("PCD");
    case PointCloudCaptureFormat::LAS: return QStringLiteral("LAS");
    case PointCloudCaptureFormat::None: return QStringLiteral("-");
    }
    return QStringLiteral("-");
}

bool isPointCloudFileFormat(PointCloudCaptureFormat format)
{
    return format == PointCloudCaptureFormat::PCD || format == PointCloudCaptureFormat::LAS;
}

void configurePointCloudAmountControl(QLabel* label, QSpinBox* spin, PointCloudCaptureFormat format)
{
    const int mode = isPointCloudFileFormat(format) ? 1 : 0;
    if (spin->property("pointCloudAmountModeSet").toBool() &&
        spin->property("pointCloudAmountMode").toInt() == mode) {
        return;
    }

    if (mode == 1) {
        label->setText(QStringLiteral("保存帧数"));
        spin->setRange(1, 1000000);
        spin->setSingleStep(1);
        spin->setSuffix(QStringLiteral(" 帧"));
    } else {
        label->setText(QStringLiteral("采集时长"));
        spin->setRange(1, 86400);
        spin->setSingleStep(1);
        spin->setSuffix(QStringLiteral(" s"));
    }
    spin->setProperty("pointCloudAmountMode", mode);
    spin->setProperty("pointCloudAmountModeSet", true);
}

} // namespace

void LivoxViewerWindow::showPointCloudCaptureDialog()
{
    if (pointCloudCaptureDialog) {
        pointCloudCaptureDialog->show();
        pointCloudCaptureDialog->raise();
        pointCloudCaptureDialog->activateWindow();
        return;
    }

    QDialog* dlg = new QDialog(this);
    pointCloudCaptureDialog = dlg;
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowTitle(QStringLiteral("点云数据采集"));
    dlg->setWindowModality(Qt::NonModal);
    dlg->setWindowFlags(dlg->windowFlags()
        | Qt::Window
        | Qt::WindowMinimizeButtonHint
        | Qt::WindowMaximizeButtonHint
        | Qt::WindowCloseButtonHint);
    dlg->resize(1040, 560);

    QHBoxLayout* root = new QHBoxLayout(dlg);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    QWidget* navPanel = new QWidget(dlg);
    navPanel->setFixedWidth(170);
    navPanel->setObjectName("CaptureNavPanel");
    QVBoxLayout* navLayout = new QVBoxLayout(navPanel);
    navLayout->setContentsMargins(14, 18, 14, 18);
    navLayout->setSpacing(8);
    QLabel* navTitle = new QLabel(QStringLiteral("采集格式"), navPanel);
    QFont navTitleFont = navTitle->font();
    navTitleFont.setBold(true);
    navTitle->setFont(navTitleFont);
    navLayout->addWidget(navTitle);

    QButtonGroup* formatGroup = new QButtonGroup(navPanel);
    formatGroup->setExclusive(true);
    QPushButton* lvx2Button = createNavButton(QStringLiteral("LVX2"), navPanel);
    QPushButton* pcdButton = createNavButton(QStringLiteral("PCD"), navPanel);
    QPushButton* lasButton = createNavButton(QStringLiteral("LAS"), navPanel);
    formatGroup->addButton(lvx2Button, int(PointCloudCaptureFormat::LVX2));
    formatGroup->addButton(pcdButton, int(PointCloudCaptureFormat::PCD));
    formatGroup->addButton(lasButton, int(PointCloudCaptureFormat::LAS));
    lvx2Button->setChecked(true);
    if (captureState.pointCloudFormat == PointCloudCaptureFormat::PCD) {
        pcdButton->setChecked(true);
    } else if (captureState.pointCloudFormat == PointCloudCaptureFormat::LAS) {
        lasButton->setChecked(true);
    }
    navLayout->addWidget(lvx2Button);
    navLayout->addWidget(pcdButton);
    navLayout->addWidget(lasButton);
    navLayout->addStretch();
    navPanel->setStyleSheet(
        "#CaptureNavPanel {"
        "  background: palette(alternate-base);"
        "  border-right: 1px solid palette(mid);"
        "}"
    );

    QWidget* mainPanel = new QWidget(dlg);
    QVBoxLayout* mainLayout = new QVBoxLayout(mainPanel);
    mainLayout->setContentsMargins(18, 16, 18, 14);
    mainLayout->setSpacing(12);

    QLabel* dialogStatusLabel = new QLabel(QStringLiteral("请选择采集格式、时长和保存路径"), mainPanel);
    dialogStatusLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));
    mainLayout->addWidget(dialogStatusLabel);

    QTableWidget* table = createTaskTable(mainPanel, {QStringLiteral("格式"), QStringLiteral("保存位置"), QStringLiteral("状态"), QStringLiteral("剩余时间"), QStringLiteral("操作")});
    table->setRowCount(1);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnType, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnPath, QHeaderView::Stretch);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnStatus, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnRemaining, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnAction, QHeaderView::Fixed);
    table->setColumnWidth(kTaskColumnType, 100);
    table->setColumnWidth(kTaskColumnStatus, 220);
    table->setColumnWidth(kTaskColumnRemaining, 110);
    table->setColumnWidth(kTaskColumnAction, 120);
    QProgressBar* progress = nullptr;
    QLabel* progressLabel = nullptr;
    QLabel* progressIcon = nullptr;
    table->setCellWidget(0, kTaskColumnStatus, createProgressCell(&progress, &progressLabel, &progressIcon));
    QToolButton* openButton = nullptr;
    table->setCellWidget(0, kTaskColumnAction, createSingleOpenActionCell(&openButton));
    mainLayout->addWidget(createTaskTableFrame(table, mainPanel), 1);

    QWidget* footer = new QWidget(mainPanel);
    QHBoxLayout* footerLayout = new QHBoxLayout(footer);
    footerLayout->setContentsMargins(0, 0, 0, 0);
    footerLayout->setSpacing(14);

    QWidget* fields = new QWidget(footer);
    QVBoxLayout* fieldsLayout = new QVBoxLayout(fields);
    fieldsLayout->setContentsMargins(0, 0, 0, 0);
    fieldsLayout->setSpacing(8);

    QWidget* durationRow = new QWidget(fields);
    QHBoxLayout* durationLayout = new QHBoxLayout(durationRow);
    durationLayout->setContentsMargins(0, 0, 0, 0);
    QLabel* durationLabel = new QLabel(QStringLiteral("采集时长"), durationRow);
    QSpinBox* durationSpin = new QSpinBox(durationRow);
    durationSpin->setRange(1, 86400);
    durationSpin->setValue(10);
    durationSpin->setSuffix(QStringLiteral(" s"));
    durationLayout->addWidget(durationLabel);
    durationLayout->addWidget(durationSpin);
    durationLayout->addStretch();
    fieldsLayout->addWidget(durationRow);

    QLabel* integrationLabel = new QLabel(fields);
    QLabel* relationshipLabel = new QLabel(fields);
    relationshipLabel->setWordWrap(true);
    relationshipLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));
    fieldsLayout->addWidget(integrationLabel);
    fieldsLayout->addWidget(relationshipLabel);

    QWidget* pathRow = new QWidget(fields);
    QHBoxLayout* pathLayout = new QHBoxLayout(pathRow);
    pathLayout->setContentsMargins(0, 0, 0, 0);
    QLabel* pathLabel = new QLabel(QStringLiteral("保存路径"), pathRow);
    QLineEdit* pathEdit = new QLineEdit(pathRow);
    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    pathEdit->setText(settings.value(QStringLiteral("save/lastPointCloudCaptureDir"), defaultDocumentsDir()).toString());
    QToolButton* browseButton = createBrowseButton(pathRow);
    pathLayout->addWidget(pathLabel);
    pathLayout->addWidget(pathEdit, 1);
    pathLayout->addWidget(browseButton);
    fieldsLayout->addWidget(pathRow);

    QPushButton* startButton = createStartButton(QStringLiteral("开始采集"), footer);
    footerLayout->addWidget(fields, 1);
    footerLayout->addWidget(startButton, 0, Qt::AlignBottom);
    mainLayout->addWidget(footer);

    root->addWidget(navPanel);
    root->addWidget(mainPanel, 1);

    auto currentFormat = [formatGroup]() {
        return static_cast<PointCloudCaptureFormat>(formatGroup->checkedId());
    };
    auto selectedTaskShown = [this, currentFormat]() {
        return captureState.pointCloudTask.status != CaptureTaskStatus::Idle &&
               captureState.pointCloudFormat == currentFormat();
    };
    auto rowOutputDir = [this, selectedTaskShown, pathEdit]() {
        return selectedTaskShown() ? captureState.pointCloudTask.outputDir : pathEdit->text().trimmed();
    };
    auto updateUi = [this,
                     table,
                     progress,
                     progressLabel,
                     progressIcon,
                     openButton,
                     startButton,
                     durationLabel,
                     durationSpin,
                     integrationLabel,
                     relationshipLabel,
                     pathEdit,
                     browseButton,
                     lvx2Button,
                     pcdButton,
                     lasButton,
                     currentFormat,
                     selectedTaskShown,
                     rowOutputDir]() {
        const PointCloudCaptureFormat format = currentFormat();
        const bool fileFormat = isPointCloudFileFormat(format);
        const bool showingTask = selectedTaskShown();
        const CaptureTaskState displayTask = showingTask ? captureState.pointCloudTask : CaptureTaskState{};
        configurePointCloudAmountControl(durationLabel, durationSpin, format);
        table->horizontalHeaderItem(kTaskColumnRemaining)->setText(fileFormat ? QStringLiteral("剩余帧数") : QStringLiteral("剩余时间"));
        setTextItem(table, 0, kTaskColumnType, pointCloudFormatText(format));
        setTextItem(table, 0, kTaskColumnPath, QDir::toNativeSeparators(rowOutputDir()));
        updateProgress(progress, progressLabel, progressIcon, displayTask);
        setTextItem(table, 0, kTaskColumnRemaining, remainingText(displayTask));
        const int integrationMs = showingTask && displayTask.integrationMs > 0
            ? displayTask.integrationMs
            : static_cast<int>(frameIntervalMs);
        integrationLabel->setVisible(fileFormat);
        relationshipLabel->setVisible(fileFormat);
        integrationLabel->setText(QStringLiteral("当前积分时间：%1 ms（来自工具栏“显示控制”）").arg(integrationMs));
        relationshipLabel->setText(QStringLiteral("每保存 1 帧会按当前积分时间合并点云并生成 1 个 %1 文件；积分时间决定每个文件的点云累积窗口和保存间隔，保存帧数决定最终文件数量。")
            .arg(pointCloudFormatText(format)));
        const bool running = taskRunning(captureState.pointCloudTask);
        openButton->setEnabled(showingTask && !captureState.pointCloudTask.outputDir.isEmpty());
        startButton->setEnabled(!running && !taskRunning(captureState.imuTask));
        durationSpin->setEnabled(!running && !taskRunning(captureState.imuTask));
        pathEdit->setEnabled(!running && !taskRunning(captureState.imuTask));
        browseButton->setEnabled(!running && !taskRunning(captureState.imuTask));
        lvx2Button->setEnabled(!running);
        pcdButton->setEnabled(!running);
        lasButton->setEnabled(!running);
    };

    QObject::connect(formatGroup, &QButtonGroup::idClicked, dlg, [updateUi](int) {
        updateUi();
    });
    QObject::connect(browseButton, &QToolButton::clicked, dlg, [dlg, pathEdit]() {
        const QString dir = selectFolder(dlg, pathEdit->text().trimmed(), QStringLiteral("选择保存目录"));
        if (!dir.isEmpty()) {
            pathEdit->setText(dir);
        }
    });
    QObject::connect(openButton, &QToolButton::clicked, dlg, [this]() {
        openOutputDir(captureState.pointCloudTask.outputDir);
    });
    QObject::connect(startButton, &QPushButton::clicked, dlg, [this, dlg, dialogStatusLabel, pathEdit, durationSpin, currentFormat, updateUi]() {
        const QString baseDir = pathEdit->text().trimmed();
        if (baseDir.isEmpty()) {
            dialogStatusLabel->setText(QStringLiteral("请选择保存路径"));
            return;
        }
        QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
        settings.setValue(QStringLiteral("save/lastPointCloudCaptureDir"), baseDir);
        QString errorMessage;
        if (!startPointCloudCapture(currentFormat(), baseDir, durationSpin->value(), errorMessage)) {
            dialogStatusLabel->setText(errorMessage);
            QMessageBox::warning(dlg, QStringLiteral("点云数据采集"), errorMessage);
            return;
        }
        dialogStatusLabel->setText(QStringLiteral("采集任务已启动"));
        updateUi();
    });

    QTimer* refreshTimer = new QTimer(dlg);
    QObject::connect(refreshTimer, &QTimer::timeout, dlg, updateUi);
    refreshTimer->start(250);
    updateUi();
    dlg->show();
}

void LivoxViewerWindow::showImuCaptureDialog()
{
    if (imuCaptureDialog) {
        imuCaptureDialog->show();
        imuCaptureDialog->raise();
        imuCaptureDialog->activateWindow();
        return;
    }

    QDialog* dlg = new QDialog(this);
    imuCaptureDialog = dlg;
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowTitle(QStringLiteral("IMU数据采集"));
    dlg->setWindowModality(Qt::NonModal);
    dlg->setWindowFlags(dlg->windowFlags()
        | Qt::Window
        | Qt::WindowMinimizeButtonHint
        | Qt::WindowMaximizeButtonHint
        | Qt::WindowCloseButtonHint);
    dlg->resize(900, 460);

    QVBoxLayout* root = new QVBoxLayout(dlg);
    root->setContentsMargins(18, 16, 18, 14);
    root->setSpacing(12);

    QLabel* dialogStatusLabel = new QLabel(QStringLiteral("设置采集时长和保存路径"), dlg);
    dialogStatusLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));
    root->addWidget(dialogStatusLabel);

    QTableWidget* table = createTaskTable(dlg, {QStringLiteral("格式"), QStringLiteral("保存位置"), QStringLiteral("状态"), QStringLiteral("剩余时间"), QStringLiteral("操作")});
    table->setRowCount(1);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnType, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnPath, QHeaderView::Stretch);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnStatus, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnRemaining, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnAction, QHeaderView::Fixed);
    table->setColumnWidth(kTaskColumnType, 100);
    table->setColumnWidth(kTaskColumnStatus, 220);
    table->setColumnWidth(kTaskColumnRemaining, 110);
    table->setColumnWidth(kTaskColumnAction, 120);
    setTextItem(table, 0, kTaskColumnType, QStringLiteral("CSV"));
    QProgressBar* progress = nullptr;
    QLabel* progressLabel = nullptr;
    QLabel* progressIcon = nullptr;
    table->setCellWidget(0, kTaskColumnStatus, createProgressCell(&progress, &progressLabel, &progressIcon));
    QToolButton* openButton = nullptr;
    table->setCellWidget(0, kTaskColumnAction, createSingleOpenActionCell(&openButton));
    root->addWidget(createTaskTableFrame(table, dlg), 1);

    QWidget* footer = new QWidget(dlg);
    QHBoxLayout* footerLayout = new QHBoxLayout(footer);
    footerLayout->setContentsMargins(0, 0, 0, 0);
    footerLayout->setSpacing(14);
    QWidget* fields = new QWidget(footer);
    QVBoxLayout* fieldsLayout = new QVBoxLayout(fields);
    fieldsLayout->setContentsMargins(0, 0, 0, 0);
    fieldsLayout->setSpacing(8);

    QWidget* durationRow = new QWidget(fields);
    QHBoxLayout* durationLayout = new QHBoxLayout(durationRow);
    durationLayout->setContentsMargins(0, 0, 0, 0);
    QLabel* durationLabel = new QLabel(QStringLiteral("采集时长"), durationRow);
    QSpinBox* durationSpin = new QSpinBox(durationRow);
    durationSpin->setRange(10, 86400);
    durationSpin->setValue(30);
    durationSpin->setSingleStep(10);
    durationSpin->setSuffix(QStringLiteral(" s"));
    durationLayout->addWidget(durationLabel);
    durationLayout->addWidget(durationSpin);
    durationLayout->addStretch();
    fieldsLayout->addWidget(durationRow);

    QWidget* pathRow = new QWidget(fields);
    QHBoxLayout* pathLayout = new QHBoxLayout(pathRow);
    pathLayout->setContentsMargins(0, 0, 0, 0);
    QLabel* pathLabel = new QLabel(QStringLiteral("保存路径"), pathRow);
    QLineEdit* pathEdit = new QLineEdit(pathRow);
    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    pathEdit->setText(settings.value(QStringLiteral("save/lastIMUDir"), defaultDocumentsDir()).toString());
    QToolButton* browseButton = createBrowseButton(pathRow);
    pathLayout->addWidget(pathLabel);
    pathLayout->addWidget(pathEdit, 1);
    pathLayout->addWidget(browseButton);
    fieldsLayout->addWidget(pathRow);

    QPushButton* startButton = createStartButton(QStringLiteral("开始采集"), footer);
    footerLayout->addWidget(fields, 1);
    footerLayout->addWidget(startButton, 0, Qt::AlignBottom);
    root->addWidget(footer);

    auto updateUi = [this, table, progress, progressLabel, progressIcon, openButton, startButton, durationSpin, pathEdit, browseButton]() {
        const bool running = taskRunning(captureState.imuTask);
        const QString outputDir = captureState.imuTask.outputDir.isEmpty()
            ? pathEdit->text().trimmed()
            : captureState.imuTask.outputDir;
        setTextItem(table, 0, kTaskColumnPath, QDir::toNativeSeparators(outputDir));
        updateProgress(progress, progressLabel, progressIcon, captureState.imuTask);
        setTextItem(table, 0, kTaskColumnRemaining, remainingText(captureState.imuTask));
        openButton->setEnabled(!captureState.imuTask.outputDir.isEmpty());
        startButton->setEnabled(!running && !taskRunning(captureState.pointCloudTask));
        durationSpin->setEnabled(!running && !taskRunning(captureState.pointCloudTask));
        pathEdit->setEnabled(!running && !taskRunning(captureState.pointCloudTask));
        browseButton->setEnabled(!running && !taskRunning(captureState.pointCloudTask));
    };
    QObject::connect(browseButton, &QToolButton::clicked, dlg, [dlg, pathEdit]() {
        const QString dir = selectFolder(dlg, pathEdit->text().trimmed(), QStringLiteral("选择保存目录"));
        if (!dir.isEmpty()) {
            pathEdit->setText(dir);
        }
    });
    QObject::connect(openButton, &QToolButton::clicked, dlg, [this]() {
        openOutputDir(captureState.imuTask.outputDir);
    });
    QObject::connect(startButton, &QPushButton::clicked, dlg, [this, dlg, dialogStatusLabel, pathEdit, durationSpin, updateUi]() {
        const QString baseDir = pathEdit->text().trimmed();
        if (baseDir.isEmpty()) {
            dialogStatusLabel->setText(QStringLiteral("请选择保存路径"));
            return;
        }
        QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
        settings.setValue(QStringLiteral("save/lastIMUDir"), baseDir);
        QString errorMessage;
        if (!startImuCapture(baseDir, durationSpin->value(), errorMessage)) {
            dialogStatusLabel->setText(errorMessage);
            QMessageBox::warning(dlg, QStringLiteral("IMU数据采集"), errorMessage);
            return;
        }
        dialogStatusLabel->setText(QStringLiteral("采集任务已启动"));
        updateUi();
    });

    QTimer* refreshTimer = new QTimer(dlg);
    QObject::connect(refreshTimer, &QTimer::timeout, dlg, updateUi);
    refreshTimer->start(250);
    updateUi();
    dlg->show();
}

void LivoxViewerWindow::showParameterCaptureDialog()
{
    if (parameterCaptureDialog) {
        parameterCaptureDialog->show();
        parameterCaptureDialog->raise();
        parameterCaptureDialog->activateWindow();
        return;
    }

    QDialog* dlg = new QDialog(this);
    parameterCaptureDialog = dlg;
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowTitle(QStringLiteral("设备参数信息采集"));
    dlg->setWindowModality(Qt::NonModal);
    dlg->setWindowFlags(dlg->windowFlags()
        | Qt::Window
        | Qt::WindowMinimizeButtonHint
        | Qt::WindowMaximizeButtonHint
        | Qt::WindowCloseButtonHint);
    dlg->resize(900, 460);

    QVBoxLayout* root = new QVBoxLayout(dlg);
    root->setContentsMargins(18, 16, 18, 14);
    root->setSpacing(12);

    QLabel* dialogStatusLabel = new QLabel(QStringLiteral("设置采集时长和保存路径"), dlg);
    dialogStatusLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));
    root->addWidget(dialogStatusLabel);

    QTableWidget* table = createTaskTable(dlg, {QStringLiteral("格式"), QStringLiteral("保存位置"), QStringLiteral("状态"), QStringLiteral("剩余时间"), QStringLiteral("操作")});
    table->setRowCount(1);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnType, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnPath, QHeaderView::Stretch);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnStatus, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnRemaining, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kTaskColumnAction, QHeaderView::Fixed);
    table->setColumnWidth(kTaskColumnType, 100);
    table->setColumnWidth(kTaskColumnStatus, 220);
    table->setColumnWidth(kTaskColumnRemaining, 110);
    table->setColumnWidth(kTaskColumnAction, 120);
    setTextItem(table, 0, kTaskColumnType, QStringLiteral("CSV"));
    QProgressBar* progress = nullptr;
    QLabel* progressLabel = nullptr;
    QLabel* progressIcon = nullptr;
    table->setCellWidget(0, kTaskColumnStatus, createProgressCell(&progress, &progressLabel, &progressIcon));
    QToolButton* openButton = nullptr;
    table->setCellWidget(0, kTaskColumnAction, createSingleOpenActionCell(&openButton));
    root->addWidget(createTaskTableFrame(table, dlg), 1);

    QWidget* footer = new QWidget(dlg);
    QHBoxLayout* footerLayout = new QHBoxLayout(footer);
    footerLayout->setContentsMargins(0, 0, 0, 0);
    footerLayout->setSpacing(14);
    QWidget* fields = new QWidget(footer);
    QVBoxLayout* fieldsLayout = new QVBoxLayout(fields);
    fieldsLayout->setContentsMargins(0, 0, 0, 0);
    fieldsLayout->setSpacing(8);

    QWidget* durationRow = new QWidget(fields);
    QHBoxLayout* durationLayout = new QHBoxLayout(durationRow);
    durationLayout->setContentsMargins(0, 0, 0, 0);
    QLabel* durationLabel = new QLabel(QStringLiteral("采集时长"), durationRow);
    QSpinBox* durationSpin = new QSpinBox(durationRow);
    durationSpin->setRange(10, 86400);
    durationSpin->setValue(30);
    durationSpin->setSingleStep(10);
    durationSpin->setSuffix(QStringLiteral(" s"));
    durationLayout->addWidget(durationLabel);
    durationLayout->addWidget(durationSpin);
    durationLayout->addStretch();
    fieldsLayout->addWidget(durationRow);

    QWidget* pathRow = new QWidget(fields);
    QHBoxLayout* pathLayout = new QHBoxLayout(pathRow);
    pathLayout->setContentsMargins(0, 0, 0, 0);
    QLabel* pathLabel = new QLabel(QStringLiteral("保存路径"), pathRow);
    QLineEdit* pathEdit = new QLineEdit(pathRow);
    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    pathEdit->setText(settings.value(QStringLiteral("save/lastParameterDir"), defaultDocumentsDir()).toString());
    QToolButton* browseButton = createBrowseButton(pathRow);
    pathLayout->addWidget(pathLabel);
    pathLayout->addWidget(pathEdit, 1);
    pathLayout->addWidget(browseButton);
    fieldsLayout->addWidget(pathRow);

    QPushButton* startButton = createStartButton(QStringLiteral("开始采集"), footer);
    footerLayout->addWidget(fields, 1);
    footerLayout->addWidget(startButton, 0, Qt::AlignBottom);
    root->addWidget(footer);

    auto updateUi = [this, table, progress, progressLabel, progressIcon, openButton, startButton, durationSpin, pathEdit, browseButton]() {
        const bool running = taskRunning(captureState.parameterTask);
        const QString outputDir = captureState.parameterTask.outputDir.isEmpty()
            ? pathEdit->text().trimmed()
            : captureState.parameterTask.outputDir;
        setTextItem(table, 0, kTaskColumnPath, QDir::toNativeSeparators(outputDir));
        updateProgress(progress, progressLabel, progressIcon, captureState.parameterTask);
        setTextItem(table, 0, kTaskColumnRemaining, remainingText(captureState.parameterTask));
        openButton->setEnabled(!captureState.parameterTask.outputDir.isEmpty());
        startButton->setEnabled(!running);
        durationSpin->setEnabled(!running);
        pathEdit->setEnabled(!running);
        browseButton->setEnabled(!running);
    };
    QObject::connect(browseButton, &QToolButton::clicked, dlg, [dlg, pathEdit]() {
        const QString dir = selectFolder(dlg, pathEdit->text().trimmed(), QStringLiteral("选择保存目录"));
        if (!dir.isEmpty()) {
            pathEdit->setText(dir);
        }
    });
    QObject::connect(openButton, &QToolButton::clicked, dlg, [this]() {
        openOutputDir(captureState.parameterTask.outputDir);
    });
    QObject::connect(startButton, &QPushButton::clicked, dlg, [this, dlg, dialogStatusLabel, pathEdit, durationSpin, updateUi]() {
        const QString baseDir = pathEdit->text().trimmed();
        if (baseDir.isEmpty()) {
            dialogStatusLabel->setText(QStringLiteral("请选择保存路径"));
            return;
        }
        QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
        settings.setValue(QStringLiteral("save/lastParameterDir"), baseDir);
        QString errorMessage;
        if (!startParameterCapture(baseDir, durationSpin->value(), errorMessage)) {
            dialogStatusLabel->setText(errorMessage);
            QMessageBox::warning(dlg, QStringLiteral("设备参数信息采集"), errorMessage);
            return;
        }
        dialogStatusLabel->setText(QStringLiteral("采集任务已启动"));
        updateUi();
    });

    QTimer* refreshTimer = new QTimer(dlg);
    QObject::connect(refreshTimer, &QTimer::timeout, dlg, updateUi);
    refreshTimer->start(250);
    updateUi();
    dlg->show();
}

void LivoxViewerWindow::showDebugCaptureDialog()
{
    if (debugCaptureDialog) {
        debugCaptureDialog->show();
        debugCaptureDialog->raise();
        debugCaptureDialog->activateWindow();
        return;
    }

    QDialog* dlg = new QDialog(this);
    debugCaptureDialog = dlg;
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowTitle(QStringLiteral("Debug数据采集"));
    dlg->setWindowModality(Qt::NonModal);
    dlg->setWindowFlags(dlg->windowFlags()
        | Qt::Window
        | Qt::WindowMinimizeButtonHint
        | Qt::WindowMaximizeButtonHint
        | Qt::WindowCloseButtonHint);
    dlg->resize(1080, 460);

    QVBoxLayout* root = new QVBoxLayout(dlg);
    root->setContentsMargins(18, 16, 18, 14);
    root->setSpacing(12);

    QLabel* dialogStatusLabel = new QLabel(QStringLiteral("LOG和Debug点云可同时采集"), dlg);
    dialogStatusLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));
    root->addWidget(dialogStatusLabel);

    QTableWidget* table = createTaskTable(dlg, {
        QStringLiteral("类型"),
        QStringLiteral("保存位置"),
        QStringLiteral("时长"),
        QStringLiteral("状态"),
        QStringLiteral("剩余时间"),
        QStringLiteral("操作")
    });
    table->setRowCount(2);
    table->horizontalHeader()->setSectionResizeMode(kDebugColumnType, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kDebugColumnPath, QHeaderView::Stretch);
    table->horizontalHeader()->setSectionResizeMode(kDebugColumnDuration, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kDebugColumnStatus, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kDebugColumnRemaining, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kDebugColumnAction, QHeaderView::Fixed);
    table->setColumnWidth(kDebugColumnType, 140);
    table->setColumnWidth(kDebugColumnDuration, 110);
    table->setColumnWidth(kDebugColumnStatus, 220);
    table->setColumnWidth(kDebugColumnRemaining, 110);
    table->setColumnWidth(kDebugColumnAction, 150);

    setTextItem(table, 0, kDebugColumnType, QStringLiteral("LOG数据"));
    setTextItem(table, 1, kDebugColumnType, QStringLiteral("Debug点云"));
    setTextItem(table, 0, kDebugColumnPath, QDir::toNativeSeparators(debugLogOutputDir()));
    setTextItem(table, 1, kDebugColumnPath, QDir::toNativeSeparators(debugPointCloudOutputDir()));

    QSpinBox* logDurationSpin = new QSpinBox(table);
    logDurationSpin->setRange(10, 86400);
    logDurationSpin->setSingleStep(10);
    logDurationSpin->setValue(300);
    logDurationSpin->setSuffix(QStringLiteral(" s"));
    QSpinBox* debugDurationSpin = new QSpinBox(table);
    debugDurationSpin->setRange(10, 3600);
    debugDurationSpin->setSingleStep(1);
    debugDurationSpin->setValue(10);
    debugDurationSpin->setSuffix(QStringLiteral(" s"));
    table->setCellWidget(0, kDebugColumnDuration, logDurationSpin);
    table->setCellWidget(1, kDebugColumnDuration, debugDurationSpin);

    QProgressBar* logProgress = nullptr;
    QLabel* logProgressLabel = nullptr;
    QLabel* logProgressIcon = nullptr;
    QProgressBar* debugProgress = nullptr;
    QLabel* debugProgressLabel = nullptr;
    QLabel* debugProgressIcon = nullptr;
    table->setCellWidget(0, kDebugColumnStatus, createProgressCell(&logProgress, &logProgressLabel, &logProgressIcon));
    table->setCellWidget(1, kDebugColumnStatus, createProgressCell(&debugProgress, &debugProgressLabel, &debugProgressIcon));

    QPushButton* logStartButton = nullptr;
    QToolButton* logOpenButton = nullptr;
    QPushButton* debugStartButton = nullptr;
    QToolButton* debugOpenButton = nullptr;
    table->setCellWidget(0, kDebugColumnAction, createDebugActionCell(&logStartButton, &logOpenButton));
    table->setCellWidget(1, kDebugColumnAction, createDebugActionCell(&debugStartButton, &debugOpenButton));
    root->addWidget(createTaskTableFrame(table, dlg), 1);

    auto updateUi = [this,
                     table,
                     logProgress,
                     logProgressLabel,
                     logProgressIcon,
                     debugProgress,
                     debugProgressLabel,
                     debugProgressIcon,
                     logDurationSpin,
                     debugDurationSpin,
                     logStartButton,
                     debugStartButton,
                     logOpenButton,
                     debugOpenButton]() {
        updateProgress(logProgress, logProgressLabel, logProgressIcon, captureState.logTask);
        updateProgress(debugProgress, debugProgressLabel, debugProgressIcon, captureState.debugTask);
        setTextItem(table, 0, kDebugColumnRemaining, remainingText(captureState.logTask));
        setTextItem(table, 1, kDebugColumnRemaining, remainingText(captureState.debugTask));
        logDurationSpin->setEnabled(!taskRunning(captureState.logTask));
        debugDurationSpin->setEnabled(!taskRunning(captureState.debugTask));
        logStartButton->setEnabled(!taskRunning(captureState.logTask));
        debugStartButton->setEnabled(!taskRunning(captureState.debugTask));
        logOpenButton->setEnabled(!debugLogOutputDir().isEmpty());
        debugOpenButton->setEnabled(!debugPointCloudOutputDir().isEmpty());
    };

    QObject::connect(logStartButton, &QPushButton::clicked, dlg, [this, dlg, dialogStatusLabel, logDurationSpin, updateUi]() {
        QString errorMessage;
        if (!startLogCapture(logDurationSpin->value(), errorMessage)) {
            dialogStatusLabel->setText(errorMessage);
            QMessageBox::warning(dlg, QStringLiteral("Debug数据采集"), errorMessage);
            return;
        }
        dialogStatusLabel->setText(QStringLiteral("LOG数据采集已启动"));
        updateUi();
    });
    QObject::connect(debugStartButton, &QPushButton::clicked, dlg, [this, dlg, dialogStatusLabel, debugDurationSpin, updateUi]() {
        QString errorMessage;
        if (!startDebugPointCloudCapture(debugDurationSpin->value(), errorMessage)) {
            dialogStatusLabel->setText(errorMessage);
            QMessageBox::warning(dlg, QStringLiteral("Debug数据采集"), errorMessage);
            return;
        }
        dialogStatusLabel->setText(QStringLiteral("Debug点云采集已启动"));
        updateUi();
    });
    QObject::connect(logOpenButton, &QToolButton::clicked, dlg, [this]() {
        openOutputDir(debugLogOutputDir());
    });
    QObject::connect(debugOpenButton, &QToolButton::clicked, dlg, [this]() {
        openOutputDir(debugPointCloudOutputDir());
    });

    QTimer* refreshTimer = new QTimer(dlg);
    QObject::connect(refreshTimer, &QTimer::timeout, dlg, updateUi);
    refreshTimer->start(250);
    updateUi();
    dlg->show();
}

#include "LivoxViewerWindow.h"

#include <QAbstractItemModel>
#include <QAbstractItemView>
#include <QApplication>
#include <QButtonGroup>
#include <QColor>
#include <QDesktopServices>
#include <QDialog>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QObject>
#include <QProgressBar>
#include <QPushButton>
#include <QRadioButton>
#include <QSettings>
#include <QSet>
#include <QSize>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QToolButton>
#include <QUrl>
#include <QVBoxLayout>

namespace {

constexpr int kColumnFileName = 0;
constexpr int kColumnFileSize = 1;
constexpr int kColumnMode = 2;
constexpr int kColumnStatus = 3;
constexpr int kColumnAction = 4;
constexpr int kPathRole = Qt::UserRole;
constexpr int kStateRole = Qt::UserRole + 1;

enum class JobState {
    Pending = 0,
    Running,
    Done,
    Failed
};

struct ConvertDialogState : QObject {
    explicit ConvertDialogState(QObject* parent)
        : QObject(parent)
        , settings("Livox", "LivoxViewerQT")
    {
    }

    QSettings settings;
    QSet<QString> addedPaths;
    Lvx2Convert::Format currentFormat = Lvx2Convert::Format::PCD;
    Lvx2Convert::Mode currentMode = Lvx2Convert::Mode::MergeAllToOne;
};

QString normalizedPathKey(const QString& path)
{
    QString normalized = QFileInfo(path).canonicalFilePath();
    if (normalized.isEmpty()) {
        normalized = QFileInfo(path).absoluteFilePath();
    }
    normalized = QDir::cleanPath(normalized);
#ifdef Q_OS_WIN
    normalized = normalized.toLower();
#endif
    return normalized;
}

QString formatFileSize(qint64 bytes)
{
    const char* units[] = {"B", "KB", "MB", "GB"};
    double value = double(bytes);
    int unitIndex = 0;
    while (value >= 1024.0 && unitIndex < 3) {
        value /= 1024.0;
        ++unitIndex;
    }
    if (unitIndex == 0) {
        return QString("%1 %2").arg(bytes).arg(units[unitIndex]);
    }
    return QString("%1 %2").arg(value, 0, 'f', 2).arg(units[unitIndex]);
}

QString modeText(Lvx2Convert::Mode mode)
{
    return mode == Lvx2Convert::Mode::SplitBy100ms
        ? QString("按 100ms 拆分")
        : QString("合并为单个文件");
}

QStringList selectLvx2Files(QWidget* parent, const QString& startDir)
{
    QFileDialog dialog(parent, "添加 LVX2 文件", startDir);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setNameFilter("LVX2 点云 (*.lvx2)");
    if (dialog.exec() != QDialog::Accepted) {
        return {};
    }
    return dialog.selectedFiles();
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
        "QPushButton:hover {"
        "  background: palette(base);"
        "}"
        "QPushButton:checked {"
        "  background: palette(highlight);"
        "  color: palette(highlighted-text);"
        "}"
    );
    return button;
}

QToolButton* createIconButton(const QString& iconPath, const QString& tooltip, QWidget* parent)
{
    QToolButton* button = new QToolButton(parent);
    button->setIcon(QIcon(iconPath));
    button->setIconSize(QSize(22, 22));
    button->setToolTip(tooltip);
    button->setAccessibleName(tooltip);
    button->setAutoRaise(true);
    button->setCursor(Qt::PointingHandCursor);
    button->setFixedSize(34, 34);
    return button;
}

QToolButton* createToolbarButton(const QString& iconPath, const QString& text, QWidget* parent)
{
    QToolButton* button = new QToolButton(parent);
    button->setIcon(QIcon(iconPath));
    button->setIconSize(QSize(20, 20));
    button->setText(text);
    button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    button->setToolTip(text);
    button->setAccessibleName(text);
    button->setCursor(Qt::PointingHandCursor);
    button->setMinimumSize(104, 34);
    button->setStyleSheet(
        "QToolButton {"
        "  padding: 0 12px;"
        "  border: 1px solid palette(mid);"
        "  border-radius: 6px;"
        "  background: palette(button);"
        "}"
        "QToolButton:hover {"
        "  background: palette(alternate-base);"
        "}"
    );
    return button;
}

QWidget* createStatusCell(const QString& text, int value, const QString& iconPath = QString(), const QColor& color = QColor())
{
    QWidget* cell = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(cell);
    layout->setContentsMargins(0, 4, 0, 4);
    layout->setSpacing(4);

    QWidget* statusRow = new QWidget(cell);
    QHBoxLayout* statusLayout = new QHBoxLayout(statusRow);
    statusLayout->setContentsMargins(0, 0, 0, 0);
    statusLayout->setSpacing(6);
    if (!iconPath.isEmpty()) {
        QLabel* icon = new QLabel(statusRow);
        icon->setPixmap(QIcon(iconPath).pixmap(QSize(16, 16)));
        icon->setFixedSize(16, 16);
        statusLayout->addWidget(icon);
    }
    QLabel* label = new QLabel(text, statusRow);
    label->setObjectName("statusLabel");
    if (color.isValid()) {
        label->setStyleSheet(QString("color: %1;").arg(color.name()));
    }
    statusLayout->addWidget(label);
    statusLayout->addStretch();

    QProgressBar* progress = new QProgressBar(cell);
    progress->setObjectName("statusProgress");
    progress->setRange(0, 100);
    progress->setValue(value);
    progress->setTextVisible(false);
    progress->setFixedHeight(8);

    layout->addWidget(statusRow);
    layout->addWidget(progress);
    return cell;
}

QTableWidgetItem* fileNameItem(QTableWidget* table, int row)
{
    return table->item(row, kColumnFileName);
}

JobState jobState(QTableWidget* table, int row)
{
    if (QTableWidgetItem* item = fileNameItem(table, row)) {
        return static_cast<JobState>(item->data(kStateRole).toInt());
    }
    return JobState::Pending;
}

void setJobState(QTableWidget* table, int row, JobState state)
{
    if (QTableWidgetItem* item = fileNameItem(table, row)) {
        item->setData(kStateRole, int(state));
    }
}

QString rowPath(QTableWidget* table, int row)
{
    if (QTableWidgetItem* item = fileNameItem(table, row)) {
        return item->data(kPathRole).toString();
    }
    return {};
}

void setStatusPending(QTableWidget* table, int row)
{
    setJobState(table, row, JobState::Pending);
    table->setCellWidget(row, kColumnStatus, createStatusCell("待转换", 0));
    table->resizeRowToContents(row);
}

void setStatusProgress(QTableWidget* table, int row, int value)
{
    QWidget* cell = table->cellWidget(row, kColumnStatus);
    QProgressBar* progress = cell ? cell->findChild<QProgressBar*>("statusProgress") : nullptr;
    QLabel* label = cell ? cell->findChild<QLabel*>("statusLabel") : nullptr;
    if (!progress || !label) {
        cell = createStatusCell("0%", 0);
        table->setCellWidget(row, kColumnStatus, cell);
        progress = cell->findChild<QProgressBar*>("statusProgress");
        label = cell->findChild<QLabel*>("statusLabel");
    }
    setJobState(table, row, JobState::Running);
    progress->setValue(value);
    label->setText(QString("%1%").arg(value));
    table->resizeRowToContents(row);
}

void setStatusDone(QTableWidget* table, int row)
{
    setJobState(table, row, JobState::Done);
    table->setCellWidget(row, kColumnStatus, createStatusCell("已完成", 100, ":/icons/status_done.svg", QColor(46, 204, 113)));
    table->resizeRowToContents(row);
}

void setStatusFailed(QTableWidget* table, int row)
{
    setJobState(table, row, JobState::Failed);
    table->setCellWidget(row, kColumnStatus, createStatusCell("失败", 0, ":/icons/status_failed.svg", QColor(220, 80, 80)));
    table->resizeRowToContents(row);
}

int rowForWidget(QTableWidget* table, QWidget* widget)
{
    for (int row = 0; row < table->rowCount(); ++row) {
        QWidget* actionCell = table->cellWidget(row, kColumnAction);
        if (actionCell == widget || (actionCell && actionCell->isAncestorOf(widget))) {
            return row;
        }
    }
    return -1;
}

QToolButton* actionButton(QTableWidget* table, int row, const char* name)
{
    QWidget* cell = table->cellWidget(row, kColumnAction);
    return cell ? cell->findChild<QToolButton*>(name) : nullptr;
}

void updateActionButtons(QTableWidget* table, int row, bool converting)
{
    if (QToolButton* openButton = actionButton(table, row, "openFolderButton")) {
        openButton->setVisible(jobState(table, row) == JobState::Done);
        openButton->setEnabled(!converting && jobState(table, row) == JobState::Done);
    }
    if (QToolButton* deleteButton = actionButton(table, row, "deleteJobButton")) {
        deleteButton->setEnabled(!converting);
    }
}

void updateAllActionButtons(QTableWidget* table, bool converting)
{
    for (int row = 0; row < table->rowCount(); ++row) {
        updateActionButtons(table, row, converting);
    }
}

QWidget* createActionCell(QTableWidget* table, QLineEdit* outputDirEdit, QSet<QString>* addedPaths)
{
    QWidget* cell = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(cell);
    layout->setContentsMargins(0, 0, 30, 0);
    layout->setSpacing(6);

    QToolButton* openButton = createIconButton(":/icons/convert_open_folder.svg", "打开输出目录", cell);
    openButton->setObjectName("openFolderButton");
    openButton->setVisible(false);
    QToolButton* deleteButton = createIconButton(":/icons/convert_delete.svg", "删除任务", cell);
    deleteButton->setObjectName("deleteJobButton");

    layout->addWidget(openButton);
    layout->addWidget(deleteButton);
    layout->addStretch();

    QObject::connect(openButton, &QToolButton::clicked, cell, [outputDirEdit]() {
        const QString outputDir = outputDirEdit->text().trimmed();
        if (!outputDir.isEmpty()) {
            QDesktopServices::openUrl(QUrl::fromLocalFile(outputDir));
        }
    });
    QObject::connect(deleteButton, &QToolButton::clicked, cell, [table, cell, addedPaths]() {
        const int row = rowForWidget(table, cell);
        if (row < 0) {
            return;
        }
        addedPaths->remove(normalizedPathKey(rowPath(table, row)));
        table->removeRow(row);
    });

    return cell;
}

void addJobRow(QTableWidget* table,
               const QString& path,
               Lvx2Convert::Mode mode,
               QLineEdit* outputDirEdit,
               QSet<QString>* addedPaths)
{
    const QFileInfo fileInfo(path);
    const QString normalized = normalizedPathKey(path);
    if (addedPaths->contains(normalized)) {
        return;
    }
    addedPaths->insert(normalized);

    const int row = table->rowCount();
    table->insertRow(row);

    QTableWidgetItem* nameItem = new QTableWidgetItem(fileInfo.fileName());
    nameItem->setData(kPathRole, fileInfo.absoluteFilePath());
    nameItem->setData(kStateRole, int(JobState::Pending));
    table->setItem(row, kColumnFileName, nameItem);
    table->setItem(row, kColumnFileSize, new QTableWidgetItem(formatFileSize(fileInfo.size())));
    table->setItem(row, kColumnMode, new QTableWidgetItem(modeText(mode)));
    setStatusPending(table, row);
    table->setCellWidget(row, kColumnAction, createActionCell(table, outputDirEdit, addedPaths));
    table->resizeRowToContents(row);
}

void updateModeText(QTableWidget* table, Lvx2Convert::Mode mode)
{
    const QString text = modeText(mode);
    for (int row = 0; row < table->rowCount(); ++row) {
        if (QTableWidgetItem* item = table->item(row, kColumnMode)) {
            item->setText(text);
        }
    }
}

void resetJobStatuses(QTableWidget* table)
{
    for (int row = 0; row < table->rowCount(); ++row) {
        setStatusPending(table, row);
    }
    updateAllActionButtons(table, false);
}

void setControlsEnabled(const QList<QWidget*>& controls, bool enabled)
{
    for (QWidget* control : controls) {
        control->setEnabled(enabled);
    }
}

} // namespace

void LivoxViewerWindow::showFormatConvertDialog()
{
    QDialog* dlg = new QDialog(this);
    ConvertDialogState* state = new ConvertDialogState(dlg);
    dlg->setWindowTitle("LVX2 格式转换");
    dlg->setWindowModality(Qt::NonModal);
    dlg->setWindowFlags(dlg->windowFlags()
        | Qt::Window
        | Qt::WindowMinimizeButtonHint
        | Qt::WindowMaximizeButtonHint
        | Qt::WindowCloseButtonHint);
    dlg->resize(1140, 680);

    QHBoxLayout* root = new QHBoxLayout(dlg);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    QWidget* navPanel = new QWidget(dlg);
    navPanel->setFixedWidth(190);
    navPanel->setObjectName("ConvertNavPanel");
    QVBoxLayout* navLayout = new QVBoxLayout(navPanel);
    navLayout->setContentsMargins(14, 18, 14, 18);
    navLayout->setSpacing(8);

    QLabel* navTitle = new QLabel("转换类型", navPanel);
    QFont navTitleFont = navTitle->font();
    navTitleFont.setBold(true);
    navTitle->setFont(navTitleFont);
    navLayout->addWidget(navTitle);

    QButtonGroup* formatGroup = new QButtonGroup(navPanel);
    formatGroup->setExclusive(true);
    QPushButton* pcdButton = createNavButton("LVX2 转 PCD", navPanel);
    QPushButton* lasButton = createNavButton("LVX2 转 LAS", navPanel);
    QPushButton* csvButton = createNavButton("LVX2 转 CSV", navPanel);
    QPushButton* txtButton = createNavButton("LVX2 转 TXT", navPanel);
    formatGroup->addButton(pcdButton, int(Lvx2Convert::Format::PCD));
    formatGroup->addButton(lasButton, int(Lvx2Convert::Format::LAS));
    formatGroup->addButton(csvButton, int(Lvx2Convert::Format::CSV));
    formatGroup->addButton(txtButton, int(Lvx2Convert::Format::TXT));
    pcdButton->setChecked(true);
    navLayout->addWidget(pcdButton);
    navLayout->addWidget(lasButton);
    navLayout->addWidget(csvButton);
    navLayout->addWidget(txtButton);
    navLayout->addStretch();
    navPanel->setStyleSheet(
        "#ConvertNavPanel {"
        "  background: palette(alternate-base);"
        "  border-right: 1px solid palette(mid);"
        "}"
    );

    QWidget* mainPanel = new QWidget(dlg);
    QVBoxLayout* mainLayout = new QVBoxLayout(mainPanel);
    mainLayout->setContentsMargins(18, 16, 18, 14);
    mainLayout->setSpacing(12);

    QWidget* toolbar = new QWidget(mainPanel);
    QHBoxLayout* toolbarLayout = new QHBoxLayout(toolbar);
    toolbarLayout->setContentsMargins(0, 0, 0, 0);
    toolbarLayout->setSpacing(10);
    QToolButton* addFilesButton = createToolbarButton(":/icons/convert_add_file.svg", "添加文件", toolbar);
    QToolButton* addFolderButton = createToolbarButton(":/icons/convert_add_folder.svg", "添加文件夹", toolbar);
    QToolButton* clearButton = createToolbarButton(":/icons/convert_clear.svg", "清空", toolbar);
    toolbarLayout->addWidget(addFilesButton);
    toolbarLayout->addWidget(addFolderButton);
    toolbarLayout->addStretch();
    toolbarLayout->addWidget(clearButton);
    mainLayout->addWidget(toolbar);

    QLabel* dialogStatusLabel = new QLabel("请添加 LVX2 文件", mainPanel);
    dialogStatusLabel->setStyleSheet("color: palette(mid);");
    mainLayout->addWidget(dialogStatusLabel);

    QTableWidget* table = new QTableWidget(mainPanel);
    table->setColumnCount(5);
    table->setHorizontalHeaderLabels({"文件名", "文件大小", "转换模式", "状态", "操作"});
    table->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    for (int column = 0; column < table->columnCount(); ++column) {
        table->horizontalHeaderItem(column)->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    }
    table->verticalHeader()->setVisible(false);
    table->setSelectionMode(QAbstractItemView::NoSelection);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setAlternatingRowColors(true);
    table->setShowGrid(false);
    table->setFocusPolicy(Qt::NoFocus);
    table->horizontalHeader()->setStretchLastSection(false);
    table->horizontalHeader()->setSectionsClickable(false);
    table->horizontalHeader()->setHighlightSections(false);
    table->horizontalHeader()->setSectionResizeMode(kColumnFileName, QHeaderView::Stretch);
    table->horizontalHeader()->setSectionResizeMode(kColumnFileSize, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kColumnMode, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kColumnStatus, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(kColumnAction, QHeaderView::Fixed);
    table->setColumnWidth(kColumnFileSize, 132);
    table->setColumnWidth(kColumnMode, 176);
    table->setColumnWidth(kColumnStatus, 250);
    table->setColumnWidth(kColumnAction, 140);
    table->setMinimumHeight(310);
    table->setStyleSheet(
        "QTableWidget { gridline-color: transparent; }"
        "QTableWidget::item { border: none; padding-left: 8px; padding-right: 30px; }"
        "QHeaderView::section { border: none; padding: 6px 30px 6px 8px; }"
    );
    mainLayout->addWidget(table, 1);

    QWidget* footer = new QWidget(mainPanel);
    QHBoxLayout* footerLayout = new QHBoxLayout(footer);
    footerLayout->setContentsMargins(0, 0, 0, 0);
    footerLayout->setSpacing(14);

    QWidget* footerFields = new QWidget(footer);
    QVBoxLayout* footerFieldsLayout = new QVBoxLayout(footerFields);
    footerFieldsLayout->setContentsMargins(0, 0, 0, 0);
    footerFieldsLayout->setSpacing(8);

    QWidget* modeRow = new QWidget(footerFields);
    QHBoxLayout* modeLayout = new QHBoxLayout(modeRow);
    modeLayout->setContentsMargins(0, 0, 0, 0);
    QLabel* modeLabel = new QLabel("转换模式", modeRow);
    modeLabel->setFixedWidth(92);
    QRadioButton* mergeModeButton = new QRadioButton("合并为单个文件", modeRow);
    QRadioButton* splitModeButton = new QRadioButton("按 100ms 拆分", modeRow);
    mergeModeButton->setChecked(true);
    modeLayout->addWidget(modeLabel);
    modeLayout->addWidget(mergeModeButton);
    modeLayout->addWidget(splitModeButton);
    modeLayout->addStretch();
    footerFieldsLayout->addWidget(modeRow);

    QWidget* outputRow = new QWidget(footerFields);
    QHBoxLayout* outputLayout = new QHBoxLayout(outputRow);
    outputLayout->setContentsMargins(0, 0, 0, 0);
    QLabel* outputLabel = new QLabel("输出目录", outputRow);
    outputLabel->setFixedWidth(92);
    QLineEdit* outputDirEdit = new QLineEdit(outputRow);
    outputDirEdit->setPlaceholderText("选择输出目录");
    outputDirEdit->setText(state->settings.value("convert/lastSourceDir", "").toString());
    QToolButton* outputFolderButton = createIconButton(":/icons/convert_browse_folder.svg", "浏览", outputRow);
    outputLayout->addWidget(outputLabel);
    outputLayout->addWidget(outputDirEdit, 1);
    outputLayout->addWidget(outputFolderButton);
    footerFieldsLayout->addWidget(outputRow);

    QPushButton* startButton = new QPushButton("开始转换", footer);
    startButton->setMinimumSize(140, 44);
    startButton->setEnabled(false);
    startButton->setStyleSheet(
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

    footerLayout->addWidget(footerFields, 1);
    footerLayout->addWidget(startButton, 0, Qt::AlignBottom);
    mainLayout->addWidget(footer);

    root->addWidget(navPanel);
    root->addWidget(mainPanel, 1);

    auto selectedMode = [mergeModeButton]() {
        return mergeModeButton->isChecked()
            ? Lvx2Convert::Mode::MergeAllToOne
            : Lvx2Convert::Mode::SplitBy100ms;
    };

    auto updateStartEnabled = [table, startButton]() {
        startButton->setEnabled(table->rowCount() > 0);
    };

    auto addPaths = [table, outputDirEdit, dialogStatusLabel, state, selectedMode, updateStartEnabled](const QStringList& paths) {
        bool addedAny = false;
        for (const QString& path : paths) {
            const QFileInfo info(path);
            if (!info.exists() || info.suffix().compare("lvx2", Qt::CaseInsensitive) != 0) {
                continue;
            }
            const int before = table->rowCount();
            addJobRow(table, info.absoluteFilePath(), selectedMode(), outputDirEdit, &state->addedPaths);
            if (table->rowCount() > before) {
                addedAny = true;
                state->settings.setValue("convert/lastSource", info.absoluteFilePath());
                state->settings.setValue("convert/lastSourceDir", info.absolutePath());
                if (outputDirEdit->text().trimmed().isEmpty()) {
                    outputDirEdit->setText(info.absolutePath());
                }
            }
        }
        if (addedAny) {
                dialogStatusLabel->setText("已添加任务");
        }
        updateStartEnabled();
    };

    QObject::connect(formatGroup, &QButtonGroup::idClicked, dlg, [table, dialogStatusLabel, state](int id) {
        const auto nextFormat = static_cast<Lvx2Convert::Format>(id);
        if (state->currentFormat != nextFormat) {
            state->currentFormat = nextFormat;
            resetJobStatuses(table);
            dialogStatusLabel->setText(table->rowCount() > 0 ? "已添加任务" : "请添加 LVX2 文件");
        }
    });
    QObject::connect(mergeModeButton, &QRadioButton::toggled, dlg, [table, dialogStatusLabel, state](bool checked) {
        if (checked) {
            state->currentMode = Lvx2Convert::Mode::MergeAllToOne;
            updateModeText(table, state->currentMode);
            resetJobStatuses(table);
            dialogStatusLabel->setText(table->rowCount() > 0 ? "已添加任务" : "请添加 LVX2 文件");
        }
    });
    QObject::connect(splitModeButton, &QRadioButton::toggled, dlg, [table, dialogStatusLabel, state](bool checked) {
        if (checked) {
            state->currentMode = Lvx2Convert::Mode::SplitBy100ms;
            updateModeText(table, state->currentMode);
            resetJobStatuses(table);
            dialogStatusLabel->setText(table->rowCount() > 0 ? "已添加任务" : "请添加 LVX2 文件");
        }
    });
    QObject::connect(addFilesButton, &QToolButton::clicked, dlg, [dlg, state, addPaths]() {
        const QString startDir = state->settings.value("convert/lastSourceDir", QDir::homePath()).toString();
        addPaths(selectLvx2Files(dlg, startDir));
    });
    QObject::connect(addFolderButton, &QToolButton::clicked, dlg, [dlg, state, addPaths]() {
        const QString startDir = state->settings.value("convert/lastSourceDir", QDir::homePath()).toString();
        const QString folder = selectFolder(dlg, startDir, "添加 LVX2 文件夹");
        if (folder.isEmpty()) {
            return;
        }
        state->settings.setValue("convert/lastSourceDir", folder);
        QDir dir(folder);
        const QFileInfoList files = dir.entryInfoList({"*.lvx2"}, QDir::Files | QDir::Readable, QDir::Name);
        QStringList paths;
        paths.reserve(files.size());
        for (const QFileInfo& file : files) {
            paths.append(file.absoluteFilePath());
        }
        addPaths(paths);
    });
    QObject::connect(clearButton, &QToolButton::clicked, dlg, [table, dialogStatusLabel, state, updateStartEnabled]() {
        table->setRowCount(0);
        state->addedPaths.clear();
        dialogStatusLabel->setText("任务已清空");
        updateStartEnabled();
    });
    QObject::connect(outputFolderButton, &QToolButton::clicked, dlg, [dlg, outputDirEdit, state]() {
        const QString startDir = outputDirEdit->text().trimmed().isEmpty()
            ? state->settings.value("convert/lastSourceDir", QDir::homePath()).toString()
            : outputDirEdit->text().trimmed();
        const QString folder = selectFolder(dlg, startDir, "选择输出目录");
        if (!folder.isEmpty()) {
            outputDirEdit->setText(folder);
            state->settings.setValue("convert/lastSourceDir", folder);
        }
    });
    QObject::connect(table->model(), &QAbstractItemModel::rowsRemoved, dlg, [table, dialogStatusLabel, updateStartEnabled]() {
        updateStartEnabled();
        if (table->rowCount() == 0) {
            dialogStatusLabel->setText("请添加 LVX2 文件");
        }
    });

    const QList<QWidget*> lockedControls = {
        addFilesButton,
        addFolderButton,
        clearButton,
        pcdButton,
        lasButton,
        csvButton,
        txtButton,
        mergeModeButton,
        splitModeButton,
        outputDirEdit,
        outputFolderButton,
        startButton
    };

    QObject::connect(startButton, &QPushButton::clicked, dlg, [this, dlg, table, outputDirEdit, dialogStatusLabel, state, lockedControls, updateStartEnabled]() {
        if (table->rowCount() == 0) {
            dialogStatusLabel->setText("请先添加 LVX2 文件");
            return;
        }
        const QString outputDir = outputDirEdit->text().trimmed();
        if (outputDir.isEmpty()) {
            dialogStatusLabel->setText("请选择输出目录");
            return;
        }
        if (!QDir(outputDir).exists()) {
            dialogStatusLabel->setText("输出目录不存在");
            return;
        }

        setControlsEnabled(lockedControls, false);
        updateAllActionButtons(table, true);
        dialogStatusLabel->setText("正在转换...");
        dlg->setCursor(Qt::BusyCursor);

        for (int row = 0; row < table->rowCount(); ++row) {
            const QString srcPath = rowPath(table, row);
            const QString outputNoExt = QDir(outputDir).filePath(QFileInfo(srcPath).completeBaseName());
            if (QTableWidgetItem* item = table->item(row, kColumnMode)) {
                item->setText(modeText(state->currentMode));
            }

            setStatusProgress(table, row, 0);
            QApplication::processEvents();
            const bool ok = convertLvx2File(srcPath, outputNoExt, state->currentMode, state->currentFormat, [table, row](int done, int total) {
                const int value = total > 0 ? done * 100 / total : 0;
                setStatusProgress(table, row, value);
                QApplication::processEvents();
            });
            if (ok) {
                setStatusDone(table, row);
            } else {
                setStatusFailed(table, row);
            }
            updateActionButtons(table, row, true);
            state->settings.setValue("convert/lastSource", srcPath);
        }

        dlg->unsetCursor();
        setControlsEnabled(lockedControls, true);
        updateAllActionButtons(table, false);
        updateStartEnabled();
        dialogStatusLabel->setText("转换完成");
    });

    dlg->show();
    dlg->raise();
    dlg->activateWindow();
}

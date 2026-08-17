#include "LivoxViewerWindow.h"
#include "dialogs/DialogWindowUtils.h"
#include "ThemeIconUtils.h"
#include "PcapExtractor.h"

#include <QAbstractItemModel>
#include <QAbstractItemView>
#include <QAbstractButton>
#include <QApplication>
#include <QButtonGroup>
#include <QColor>
#include <QDesktopServices>
#include <QDialog>
#include <QDir>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QFutureWatcher>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMimeData>
#include <QObject>
#include <QProgressBar>
#include <QPointer>
#include <QPushButton>
#include <QRadioButton>
#include <QSettings>
#include <QSet>
#include <QSignalBlocker>
#include <QSize>
#include <QSizePolicy>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QToolButton>
#include <QUrl>
#include <QVBoxLayout>
#include <QtConcurrent>

#include <algorithm>
#include <atomic>
#include <functional>
#include <memory>
#include <utility>

namespace {

constexpr int kColumnFileName = 0;
constexpr int kColumnFileSize = 1;
constexpr int kColumnMode = 2;
constexpr int kColumnStatus = 3;
constexpr int kColumnAction = 4;
constexpr int kPathRole = Qt::UserRole;
constexpr int kStateRole = Qt::UserRole + 1;
constexpr int kOutputFilesRole = Qt::UserRole + 2;
constexpr int kOptionRowHeight = 36;
constexpr int kOptionLabelSpacing = 6;
constexpr int kOptionRadioSpacing = 14;
constexpr int kTableFramePadding = 1;
constexpr int kRosbagToPcdFormatId = 100;
constexpr int kPcapToLvx2FormatId = 200;
constexpr int kPcapToImuFormatId = 201;
constexpr int kPcapToInfoFormatId = 202;

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
    bool currentRosbagToPcd = false;
    int currentToolId = int(Lvx2Convert::Format::PCD);
    bool extractionRunning = false;
    std::shared_ptr<std::atomic_bool> extractionCancellation;
    QMap<int, QStringList> pathsBySourceType;
    QMap<int, QMap<QString, JobState>> statesByTool;
    QMap<int, QMap<QString, QStringList>> outputsByTool;
    QMap<int, Lvx2Convert::Mode> modeByTool;
};

bool isPcapTool(int toolId)
{
    return toolId >= kPcapToLvx2FormatId && toolId <= kPcapToInfoFormatId;
}

int sourceTypeForTool(int toolId)
{
    if (isPcapTool(toolId)) return 2;
    if (toolId == kRosbagToPcdFormatId) return 1;
    return 0;
}

PcapExtractor::Kind extractionKind(int toolId)
{
    if (toolId == kPcapToLvx2FormatId) return PcapExtractor::Kind::Lvx2;
    if (toolId == kPcapToImuFormatId) return PcapExtractor::Kind::ImuCsv;
    return PcapExtractor::Kind::InfoCsv;
}

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
        ? QString("按单帧拆分")
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

QStringList selectRosbagFiles(QWidget* parent, const QString& startDir)
{
    QFileDialog dialog(parent, QStringLiteral("添加 ROSbag 文件"), startDir);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setNameFilter(QStringLiteral("ROSbag (*.bag *.db3 *.mcap *.yaml *.yml)"));
    if (dialog.exec() != QDialog::Accepted) {
        return {};
    }
    return dialog.selectedFiles();
}

QStringList selectPcapFiles(QWidget* parent, const QString& startDir)
{
    QFileDialog dialog(parent, QStringLiteral("添加 PCAP 文件"), startDir);
    dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setNameFilter(QStringLiteral("抓包文件 (*.pcap *.pcapng *.cap)"));
    if (dialog.exec() != QDialog::Accepted) return {};
    return dialog.selectedFiles();
}

bool isRosbagConvertFile(const QFileInfo& info)
{
    const QString suffix = info.suffix();
    return suffix.compare(QStringLiteral("bag"), Qt::CaseInsensitive) == 0 ||
           suffix.compare(QStringLiteral("db3"), Qt::CaseInsensitive) == 0 ||
           suffix.compare(QStringLiteral("mcap"), Qt::CaseInsensitive) == 0 ||
           suffix.compare(QStringLiteral("yaml"), Qt::CaseInsensitive) == 0 ||
           suffix.compare(QStringLiteral("yml"), Qt::CaseInsensitive) == 0;
}

bool isPcapFile(const QFileInfo& info)
{
    const QString suffix = info.suffix();
    return suffix.compare(QStringLiteral("pcap"), Qt::CaseInsensitive) == 0 ||
           suffix.compare(QStringLiteral("pcapng"), Qt::CaseInsensitive) == 0 ||
           suffix.compare(QStringLiteral("cap"), Qt::CaseInsensitive) == 0;
}

bool isSupportedSourceFile(const QFileInfo& info, bool rosbagToPcd, bool pcapTool)
{
    if (!info.exists() || !info.isFile()) return false;
    if (pcapTool) return isPcapFile(info);
    if (rosbagToPcd) return isRosbagConvertFile(info);
    return info.suffix().compare(QStringLiteral("lvx2"), Qt::CaseInsensitive) == 0;
}

QStringList folderFilters(bool rosbagToPcd, bool pcapTool)
{
    if (pcapTool) return {QStringLiteral("*.pcap"), QStringLiteral("*.pcapng"), QStringLiteral("*.cap")};
    return rosbagToPcd
        ? QStringList{QStringLiteral("*.bag"), QStringLiteral("*.db3"), QStringLiteral("*.mcap"), QStringLiteral("*.yaml"), QStringLiteral("*.yml")}
        : QStringList{QStringLiteral("*.lvx2")};
}

QString addFilePrompt(bool rosbagToPcd, bool pcapTool)
{
    if (pcapTool) return QStringLiteral("请添加 PCAP、PCAPNG 或 CAP 文件，或将文件拖放至下方窗口内");
    return rosbagToPcd
        ? QStringLiteral("请添加 ROSbag 文件，或将文件拖放至下方窗口内")
        : QStringLiteral("请添加 LVX2 文件，或将文件拖放至下方窗口内");
}

class ConvertDropEventFilter : public QObject
{
public:
    using FileValidator = std::function<bool(const QFileInfo&)>;
    using DropHandler = std::function<void(const QStringList&)>;

    ConvertDropEventFilter(FileValidator validator, DropHandler handler, QObject* parent)
        : QObject(parent)
        , validator_(std::move(validator))
        , handler_(std::move(handler))
    {
    }

    void installOn(QWidget* widget)
    {
        widget->setAcceptDrops(true);
        widget->installEventFilter(this);
    }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (event->type() != QEvent::DragEnter &&
            event->type() != QEvent::DragMove &&
            event->type() != QEvent::Drop) {
            return QObject::eventFilter(watched, event);
        }

        QDropEvent* dropEvent = static_cast<QDropEvent*>(event);
        const QStringList paths = supportedPaths(dropEvent->mimeData());
        if (paths.isEmpty()) {
            dropEvent->ignore();
            return true;
        }

        dropEvent->acceptProposedAction();
        if (event->type() == QEvent::Drop) handler_(paths);
        return true;
    }

private:
    QStringList supportedPaths(const QMimeData* mimeData) const
    {
        QStringList paths;
        if (!mimeData || !mimeData->hasUrls()) return paths;

        for (const QUrl& url : mimeData->urls()) {
            if (!url.isLocalFile()) continue;
            const QFileInfo info(url.toLocalFile());
            if (validator_(info)) paths.append(info.absoluteFilePath());
        }
        return paths;
    }

    FileValidator validator_;
    DropHandler handler_;
};

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
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    button->setCheckable(true);
    button->setMinimumHeight(38);
    button->setCursor(Qt::PointingHandCursor);
    button->setStyleSheet(
        "QPushButton {"
        "  text-align: left;"
        "  padding: 7px 12px 7px 30px;"
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

QToolButton* createNavGroupButton(const QString& text, QWidget* parent)
{
    QToolButton* button = new QToolButton(parent);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    button->setText(text);
    button->setCheckable(true);
    button->setChecked(true);
    button->setArrowType(Qt::DownArrow);
    button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    button->setCursor(Qt::PointingHandCursor);
    button->setMinimumHeight(38);
    button->setStyleSheet(QStringLiteral(
        "QToolButton { text-align: left; padding: 7px 8px; border: none; border-radius: 6px; font-weight: 600; }"
        "QToolButton:hover { background: palette(base); }"));
    return button;
}

QToolButton* createIconButton(const QString& iconPath, const QString& tooltip, QWidget* parent)
{
    QToolButton* button = new QToolButton(parent);
    ThemeIconUtils::setThemedSvgIcon(button, iconPath);
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
    const QString displayText = QStringLiteral("  ") + text;
    ThemeIconUtils::setThemedSvgIcon(button, iconPath);
    button->setIconSize(QSize(20, 20));
    button->setText(displayText);
    button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    button->setToolTip(text);
    button->setAccessibleName(text);
    button->setCursor(Qt::PointingHandCursor);
    const int width = 20 + button->fontMetrics().horizontalAdvance(displayText) + 24;
    button->setMinimumSize(width, 34);
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

QWidget* createStatusCell(const QString& text,
                          int value,
                          const QString& iconPath = QString(),
                          const QColor& color = QColor(),
                          bool showProgress = true)
{
    QWidget* cell = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(cell);
    layout->setContentsMargins(0, 4, 0, 4);
    layout->setSpacing(4);

    QWidget* statusRow = new QWidget(cell);
    statusRow->setFixedHeight(20);
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
    label->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    if (color.isValid()) {
        label->setStyleSheet(QString("color: %1;").arg(color.name()));
    }
    statusLayout->addWidget(label);
    statusLayout->addStretch();

    layout->addWidget(statusRow);
    if (showProgress) {
        QProgressBar* progress = new QProgressBar(cell);
        progress->setObjectName("statusProgress");
        progress->setRange(0, 100);
        progress->setValue(value);
        progress->setTextVisible(false);
        progress->setFixedHeight(8);
        layout->addWidget(progress);
    }
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

void replaceStatusCell(QTableWidget* table, int row, QWidget* replacement)
{
    if (QWidget* oldCell = table->cellWidget(row, kColumnStatus)) {
        oldCell->hide();
        table->removeCellWidget(row, kColumnStatus);
        delete oldCell;
    }
    table->setCellWidget(row, kColumnStatus, replacement);
}

QStringList rowOutputFiles(QTableWidget* table, int row)
{
    if (QTableWidgetItem* item = fileNameItem(table, row)) {
        return item->data(kOutputFilesRole).toStringList();
    }
    return {};
}

void setRowOutputFiles(QTableWidget* table, int row, const QStringList& files)
{
    if (QTableWidgetItem* item = fileNameItem(table, row)) {
        item->setData(kOutputFilesRole, files);
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
    replaceStatusCell(table, row, createStatusCell("待转换", 0));
    table->resizeRowToContents(row);
}

void setStatusProgress(QTableWidget* table, int row, int value)
{
    if (row < 0 || row >= table->rowCount()) {
        return;
    }
    const JobState state = jobState(table, row);
    if (state == JobState::Done || state == JobState::Failed) {
        return;
    }
    QWidget* cell = table->cellWidget(row, kColumnStatus);
    QProgressBar* progress = cell ? cell->findChild<QProgressBar*>("statusProgress") : nullptr;
    QLabel* label = cell ? cell->findChild<QLabel*>("statusLabel") : nullptr;
    if (!progress || !label) {
        cell = createStatusCell("0%", 0);
        replaceStatusCell(table, row, cell);
        progress = cell->findChild<QProgressBar*>("statusProgress");
        label = cell->findChild<QLabel*>("statusLabel");
    }
    setJobState(table, row, JobState::Running);
    const int boundedValue = std::clamp(value, 0, 100);
    progress->setValue(boundedValue);
    label->setText(QString("%1%").arg(boundedValue));
    table->resizeRowToContents(row);
}

void setStatusDone(QTableWidget* table, int row)
{
    setJobState(table, row, JobState::Done);
    replaceStatusCell(table, row, createStatusCell("已完成", 100, ":/icons/status_done.svg", QColor(46, 204, 113), false));
    table->resizeRowToContents(row);
}

void setStatusFailed(QTableWidget* table, int row)
{
    setJobState(table, row, JobState::Failed);
    replaceStatusCell(table, row, createStatusCell("失败", 0, ":/icons/status_failed.svg", QColor(220, 80, 80), false));
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
    if (QToolButton* openFileButton = actionButton(table, row, "openFileButton")) {
        const bool available = jobState(table, row) == JobState::Done && !rowOutputFiles(table, row).isEmpty();
        openFileButton->setVisible(available);
        openFileButton->setEnabled(!converting && available);
    }
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

QString outputDirForSource(const QString& srcPath, bool useSourceDir, const QString& customOutputDir)
{
    return useSourceDir ? QFileInfo(srcPath).absolutePath() : customOutputDir;
}

QWidget* createActionCell(QTableWidget* table,
                          QRadioButton* sameSourceRadio,
                          QLineEdit* outputDirEdit,
                          QSet<QString>* addedPaths)
{
    QWidget* cell = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(cell);
    layout->setContentsMargins(0, 0, 14, 0);
    layout->setSpacing(6);

    QToolButton* openFileButton = createIconButton(":/icons/convert_open_file.svg", "打开输出文件", cell);
    openFileButton->setObjectName("openFileButton");
    openFileButton->setVisible(false);
    QToolButton* openButton = createIconButton(":/icons/convert_open_folder.svg", "打开输出目录", cell);
    openButton->setObjectName("openFolderButton");
    openButton->setVisible(false);
    QToolButton* deleteButton = createIconButton(":/icons/convert_delete.svg", "删除任务", cell);
    deleteButton->setObjectName("deleteJobButton");

    layout->addWidget(openFileButton);
    layout->addWidget(openButton);
    layout->addWidget(deleteButton);
    layout->addStretch();

    QObject::connect(openFileButton, &QToolButton::clicked, cell, [table, cell]() {
        const int row = rowForWidget(table, cell);
        if (row < 0) return;
        const QStringList files = rowOutputFiles(table, row);
        if (!files.isEmpty()) {
            QDesktopServices::openUrl(QUrl::fromLocalFile(files.first()));
        }
    });
    QObject::connect(openButton, &QToolButton::clicked, cell, [table, cell, sameSourceRadio, outputDirEdit]() {
        const int row = rowForWidget(table, cell);
        if (row < 0) {
            return;
        }
        const QString outputDir = outputDirForSource(rowPath(table, row),
                                                     sameSourceRadio->isChecked(),
                                                     outputDirEdit->text().trimmed());
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
               const QString& mode,
               QRadioButton* sameSourceRadio,
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
    table->setItem(row, kColumnMode, new QTableWidgetItem(mode));
    setStatusPending(table, row);
    table->setCellWidget(row, kColumnAction, createActionCell(table, sameSourceRadio, outputDirEdit, addedPaths));
    table->resizeRowToContents(row);
}

void updateModeText(QTableWidget* table, const QString& text)
{
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

QFrame* createConvertTableFrame(QTableWidget* table, QWidget* parent)
{
    QFrame* frame = new QFrame(parent);
    frame->setObjectName(QStringLiteral("ConvertTaskTableFrame"));
    frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    frame->setStyleSheet(
        "#ConvertTaskTableFrame {"
        "  background: palette(base);"
        "  border: 1px solid palette(mid);"
        "  border-radius: 6px;"
        "}"
    );

    QVBoxLayout* layout = new QVBoxLayout(frame);
    layout->setContentsMargins(kTableFramePadding, kTableFramePadding, kTableFramePadding, kTableFramePadding);
    layout->setSpacing(0);
    layout->addWidget(table);
    return frame;
}

} // namespace

void LivoxViewerWindow::showFormatConvertDialog()
{
    QDialog* dlg = new QDialog(this);
    ConvertDialogState* state = new ConvertDialogState(dlg);
    dlg->setWindowTitle(QStringLiteral("格式转换"));
    dlg->setWindowModality(Qt::NonModal);
    DialogWindowUtils::enableTopLevelWindowControls(dlg);
    dlg->resize(1140, 680);

    QHBoxLayout* root = new QHBoxLayout(dlg);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    QWidget* navPanel = new QWidget(dlg);
    navPanel->setFixedWidth(240);
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
    QToolButton* pointGroupButton = createNavGroupButton(QStringLiteral("点云转换"), navPanel);
    pointGroupButton->setChecked(state->settings.value(QStringLiteral("convert/pointGroupExpanded"), true).toBool());
    pointGroupButton->setArrowType(pointGroupButton->isChecked() ? Qt::DownArrow : Qt::RightArrow);
    QWidget* pointGroupContent = new QWidget(navPanel);
    QVBoxLayout* pointGroupLayout = new QVBoxLayout(pointGroupContent);
    pointGroupLayout->setContentsMargins(0, 0, 0, 0);
    pointGroupLayout->setSpacing(4);
    QPushButton* pcdButton = createNavButton("LVX2 转 PCD", navPanel);
    QPushButton* lasButton = createNavButton("LVX2 转 LAS", navPanel);
    QPushButton* csvButton = createNavButton("LVX2 转 CSV", navPanel);
    QPushButton* txtButton = createNavButton("LVX2 转 TXT", navPanel);
    QPushButton* rosbagPcdButton = createNavButton(QStringLiteral("ROSbag 转 PCD"), navPanel);
    QToolButton* extractGroupButton = createNavGroupButton(QStringLiteral("数据提取"), navPanel);
    extractGroupButton->setChecked(state->settings.value(QStringLiteral("convert/extractGroupExpanded"), true).toBool());
    extractGroupButton->setArrowType(extractGroupButton->isChecked() ? Qt::DownArrow : Qt::RightArrow);
    QWidget* extractGroupContent = new QWidget(navPanel);
    QVBoxLayout* extractGroupLayout = new QVBoxLayout(extractGroupContent);
    extractGroupLayout->setContentsMargins(0, 0, 0, 0);
    extractGroupLayout->setSpacing(4);
    QPushButton* pcapLvx2Button = createNavButton(QStringLiteral("PCAP 提取 LVX2"), navPanel);
    QPushButton* pcapImuButton = createNavButton(QStringLiteral("PCAP 提取 IMU"), navPanel);
    QPushButton* pcapInfoButton = createNavButton(QStringLiteral("PCAP 提取 INFO"), navPanel);
    formatGroup->addButton(pcdButton, int(Lvx2Convert::Format::PCD));
    formatGroup->addButton(lasButton, int(Lvx2Convert::Format::LAS));
    formatGroup->addButton(csvButton, int(Lvx2Convert::Format::CSV));
    formatGroup->addButton(txtButton, int(Lvx2Convert::Format::TXT));
    formatGroup->addButton(rosbagPcdButton, kRosbagToPcdFormatId);
    formatGroup->addButton(pcapLvx2Button, kPcapToLvx2FormatId);
    formatGroup->addButton(pcapImuButton, kPcapToImuFormatId);
    formatGroup->addButton(pcapInfoButton, kPcapToInfoFormatId);
    pcdButton->setChecked(true);
    pointGroupLayout->addWidget(pcdButton);
    pointGroupLayout->addWidget(lasButton);
    pointGroupLayout->addWidget(csvButton);
    pointGroupLayout->addWidget(txtButton);
    pointGroupLayout->addWidget(rosbagPcdButton);
    extractGroupLayout->addWidget(pcapLvx2Button);
    extractGroupLayout->addWidget(pcapImuButton);
    extractGroupLayout->addWidget(pcapInfoButton);
    navLayout->addWidget(pointGroupButton);
    navLayout->addWidget(pointGroupContent);
    navLayout->addWidget(extractGroupButton);
    navLayout->addWidget(extractGroupContent);
    pointGroupContent->setVisible(pointGroupButton->isChecked());
    extractGroupContent->setVisible(extractGroupButton->isChecked());
    navLayout->addStretch();
    QObject::connect(pointGroupButton, &QToolButton::toggled, pointGroupContent, [pointGroupButton, pointGroupContent](bool expanded) {
        pointGroupContent->setVisible(expanded);
        pointGroupButton->setArrowType(expanded ? Qt::DownArrow : Qt::RightArrow);
    });
    QObject::connect(pointGroupButton, &QToolButton::toggled, state, [state](bool expanded) {
        state->settings.setValue(QStringLiteral("convert/pointGroupExpanded"), expanded);
    });
    QObject::connect(extractGroupButton, &QToolButton::toggled, extractGroupContent, [extractGroupButton, extractGroupContent](bool expanded) {
        extractGroupContent->setVisible(expanded);
        extractGroupButton->setArrowType(expanded ? Qt::DownArrow : Qt::RightArrow);
    });
    QObject::connect(extractGroupButton, &QToolButton::toggled, state, [state](bool expanded) {
        state->settings.setValue(QStringLiteral("convert/extractGroupExpanded"), expanded);
    });
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

    QLabel* dialogStatusLabel = new QLabel(addFilePrompt(false, false), mainPanel);
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
    table->setFrameShape(QFrame::NoFrame);
    table->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
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
    table->setColumnWidth(kColumnAction, 172);
    table->setMinimumHeight(310);
    table->setStyleSheet(
        "QTableWidget { border: none; gridline-color: transparent; background: palette(base); }"
        "QTableWidget::viewport { background: palette(base); }"
        "QTableWidget::item { border: none; padding-left: 8px; padding-right: 30px; }"
        "QHeaderView::section { border: none; padding: 6px 30px 6px 8px; }"
    );
    mainLayout->addWidget(createConvertTableFrame(table, mainPanel), 1);

    QWidget* footer = new QWidget(mainPanel);
    QHBoxLayout* footerLayout = new QHBoxLayout(footer);
    footerLayout->setContentsMargins(0, 0, 0, 0);
    footerLayout->setSpacing(14);

    QWidget* footerFields = new QWidget(footer);
    QVBoxLayout* footerFieldsLayout = new QVBoxLayout(footerFields);
    footerFieldsLayout->setContentsMargins(0, 0, 0, 0);
    footerFieldsLayout->setSpacing(8);

    QWidget* modeBlock = new QWidget(footerFields);
    QVBoxLayout* modeBlockLayout = new QVBoxLayout(modeBlock);
    modeBlockLayout->setContentsMargins(0, 0, 0, 0);
    modeBlockLayout->setSpacing(kOptionLabelSpacing);
    QLabel* modeLabel = new QLabel("转换模式", modeBlock);
    modeBlockLayout->addWidget(modeLabel);

    QWidget* modeRow = new QWidget(modeBlock);
    modeRow->setFixedHeight(kOptionRowHeight);
    QHBoxLayout* modeLayout = new QHBoxLayout(modeRow);
    modeLayout->setContentsMargins(0, 0, 0, 0);
    modeLayout->setSpacing(kOptionRadioSpacing);
    QRadioButton* mergeModeButton = new QRadioButton("合并为单个文件", modeRow);
    QRadioButton* splitModeButton = new QRadioButton("按单帧拆分", modeRow);
    mergeModeButton->setChecked(true);
    modeLayout->addWidget(mergeModeButton, 0, Qt::AlignVCenter);
    modeLayout->addWidget(splitModeButton, 0, Qt::AlignVCenter);
    modeLayout->addStretch();
    modeBlockLayout->addWidget(modeRow);
    footerFieldsLayout->addWidget(modeBlock);

    QWidget* outputBlock = new QWidget(footerFields);
    QVBoxLayout* outputBlockLayout = new QVBoxLayout(outputBlock);
    outputBlockLayout->setContentsMargins(0, 0, 0, 0);
    outputBlockLayout->setSpacing(kOptionLabelSpacing);
    QLabel* outputLabel = new QLabel("输出", outputBlock);
    outputBlockLayout->addWidget(outputLabel);

    QWidget* outputRow = new QWidget(outputBlock);
    outputRow->setFixedHeight(kOptionRowHeight);
    QHBoxLayout* outputLayout = new QHBoxLayout(outputRow);
    outputLayout->setContentsMargins(0, 0, 0, 0);
    outputLayout->setSpacing(kOptionRadioSpacing);
    QRadioButton* sameSourceOutputRadio = new QRadioButton("与源文件相同", outputRow);
    QRadioButton* customOutputRadio = new QRadioButton("自定义", outputRow);
    const bool useSourceOutputDir = state->settings.value("convert/useSourceDir", true).toBool();
    sameSourceOutputRadio->setChecked(useSourceOutputDir);
    customOutputRadio->setChecked(!useSourceOutputDir);

    QFrame* outputPathFrame = new QFrame(outputRow);
    outputPathFrame->setObjectName("ConvertOutputPathFrame");
    outputPathFrame->setFixedHeight(kOptionRowHeight - 2);
    outputPathFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    outputPathFrame->setStyleSheet(
        "QFrame#ConvertOutputPathFrame {"
        "  border: 1px solid palette(mid);"
        "  border-radius: 6px;"
        "  background: palette(base);"
        "}"
    );
    QHBoxLayout* outputPathLayout = new QHBoxLayout(outputPathFrame);
    outputPathLayout->setContentsMargins(10, 0, 4, 0);
    outputPathLayout->setSpacing(4);
    QLineEdit* outputDirEdit = new QLineEdit(outputPathFrame);
    outputDirEdit->setPlaceholderText("选择输出目录");
    outputDirEdit->setText(state->settings.value("convert/lastOutputDir", "").toString());
    outputDirEdit->setFrame(false);
    outputDirEdit->setStyleSheet(
        "QLineEdit { border: none; background: transparent; }"
        "QLineEdit:disabled { color: #9aa0a6; }"
    );
    QToolButton* outputFolderButton = createIconButton(":/icons/convert_browse_folder.svg", "浏览", outputPathFrame);
    outputFolderButton->setObjectName(QStringLiteral("InlineBrowseButton"));
    outputFolderButton->setAutoRaise(false);
    outputFolderButton->setIconSize(QSize(18, 18));
    outputFolderButton->setFixedSize(28, 28);
    outputFolderButton->setStyleSheet(QStringLiteral(
        "QToolButton#InlineBrowseButton { border: none; border-radius: 3px; background: transparent; }"
        "QToolButton#InlineBrowseButton:hover { background: palette(button); }"
        "QToolButton#InlineBrowseButton:pressed { background: palette(midlight); }"));
    outputPathLayout->addWidget(outputDirEdit, 1);
    outputPathLayout->addWidget(outputFolderButton);

    const int firstRadioWidth = std::max(mergeModeButton->sizeHint().width(),
                                         sameSourceOutputRadio->sizeHint().width());
    mergeModeButton->setFixedWidth(firstRadioWidth);
    sameSourceOutputRadio->setFixedWidth(firstRadioWidth);

    outputLayout->addWidget(sameSourceOutputRadio, 0, Qt::AlignVCenter);
    outputLayout->addWidget(customOutputRadio, 0, Qt::AlignVCenter);
    outputLayout->addWidget(outputPathFrame, 1, Qt::AlignVCenter);
    outputBlockLayout->addWidget(outputRow);
    footerFieldsLayout->addWidget(outputBlock);

    auto updateOutputPathEnabled = [customOutputRadio, outputPathFrame, outputDirEdit, outputFolderButton]() {
        const bool enabled = customOutputRadio->isChecked();
        outputPathFrame->setStyleSheet(enabled
            ? "QFrame#ConvertOutputPathFrame {"
              "  border: 1px solid palette(mid);"
              "  border-radius: 6px;"
              "  background: palette(base);"
              "}"
            : "QFrame#ConvertOutputPathFrame {"
              "  border: 1px solid #d4d8dd;"
              "  border-radius: 6px;"
              "  background: #f3f5f7;"
              "}"
        );
        outputDirEdit->setEnabled(enabled);
        outputFolderButton->setEnabled(enabled);
    };
    updateOutputPathEnabled();
    QObject::connect(customOutputRadio, &QRadioButton::toggled, dlg, [updateOutputPathEnabled](bool) {
        updateOutputPathEnabled();
    });

    QPushButton* startButton = new QPushButton("开始转换", footer);
    startButton->setMinimumSize(140, 44);
    startButton->setEnabled(false);
    startButton->setStyleSheet(
        "QPushButton {"
        "  background: palette(highlight);"
        "  color: palette(highlighted-text);"
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

    auto selectedModeText = [state, mergeModeButton]() {
        if (isPcapTool(state->currentToolId)) {
            return mergeModeButton->isChecked()
                ? QStringLiteral("合并为单个文件")
                : QStringLiteral("按设备拆分");
        }
        return modeText(mergeModeButton->isChecked() ? Lvx2Convert::Mode::MergeAllToOne
                                                      : Lvx2Convert::Mode::SplitBy100ms);
    };

    auto updateStartEnabled = [table, startButton]() {
        startButton->setEnabled(table->rowCount() > 0);
    };

    auto saveVisiblePaths = [table, state]() {
        QStringList paths;
        QMap<QString, JobState> states;
        QMap<QString, QStringList> outputs;
        paths.reserve(table->rowCount());
        for (int row = 0; row < table->rowCount(); ++row) {
            const QString path = rowPath(table, row);
            paths.push_back(path);
            states.insert(normalizedPathKey(path), jobState(table, row));
            outputs.insert(normalizedPathKey(path), rowOutputFiles(table, row));
        }
        state->pathsBySourceType.insert(sourceTypeForTool(state->currentToolId), paths);
        state->statesByTool.insert(state->currentToolId, states);
        state->outputsByTool.insert(state->currentToolId, outputs);
        state->modeByTool.insert(state->currentToolId, state->currentMode);
    };

    auto restoreVisiblePaths = [table, sameSourceOutputRadio, outputDirEdit, state, selectedModeText, updateStartEnabled]() {
        table->setRowCount(0);
        state->addedPaths.clear();
        const QStringList paths = state->pathsBySourceType.value(sourceTypeForTool(state->currentToolId));
        for (const QString& path : paths) {
            addJobRow(table,
                      path,
                      selectedModeText(),
                      sameSourceOutputRadio,
                      outputDirEdit,
                      &state->addedPaths);
            const int row = table->rowCount() - 1;
            const JobState savedState = state->statesByTool.value(state->currentToolId)
                                            .value(normalizedPathKey(path), JobState::Pending);
            setRowOutputFiles(table,
                              row,
                              state->outputsByTool.value(state->currentToolId)
                                  .value(normalizedPathKey(path)));
            if (savedState == JobState::Done) {
                setStatusDone(table, row);
            } else if (savedState == JobState::Failed) {
                setStatusFailed(table, row);
            }
        }
        updateAllActionButtons(table, false);
        updateStartEnabled();
    };

    auto restoreVisibleStates = [table, state]() {
        const QMap<QString, JobState> states = state->statesByTool.value(state->currentToolId);
        const QMap<QString, QStringList> outputs = state->outputsByTool.value(state->currentToolId);
        for (int row = 0; row < table->rowCount(); ++row) {
            const QString pathKey = normalizedPathKey(rowPath(table, row));
            const JobState savedState = states.value(pathKey, JobState::Pending);
            setRowOutputFiles(table, row, outputs.value(pathKey));
            if (savedState == JobState::Done) {
                setStatusDone(table, row);
            } else if (savedState == JobState::Failed) {
                setStatusFailed(table, row);
            } else {
                setStatusPending(table, row);
            }
        }
        updateAllActionButtons(table, false);
    };

    auto addPaths = [table, sameSourceOutputRadio, outputDirEdit, dialogStatusLabel, state, selectedModeText, updateStartEnabled](const QStringList& paths) {
        bool addedAny = false;
        const bool rosbagToPcd = state->currentRosbagToPcd;
        const bool pcapTool = isPcapTool(state->currentToolId);
        for (const QString& path : paths) {
            const QFileInfo info(path);
            if (!isSupportedSourceFile(info, rosbagToPcd, pcapTool)) continue;
            const int before = table->rowCount();
            addJobRow(table,
                      info.absoluteFilePath(),
                      selectedModeText(),
                      sameSourceOutputRadio,
                      outputDirEdit,
                      &state->addedPaths);
            if (table->rowCount() > before) {
                addedAny = true;
                state->settings.setValue("convert/lastSource", info.absoluteFilePath());
                state->settings.setValue("convert/lastSourceDir", info.absolutePath());
            }
        }
        if (addedAny) {
                dialogStatusLabel->setText("已添加任务");
        }
        updateStartEnabled();
    };

    ConvertDropEventFilter* dropFilter = new ConvertDropEventFilter(
        [state](const QFileInfo& info) {
            return isSupportedSourceFile(info, state->currentRosbagToPcd, isPcapTool(state->currentToolId));
        },
        addPaths,
        dlg);
    dropFilter->installOn(dlg);
    dropFilter->installOn(mainPanel);
    dropFilter->installOn(table->viewport());

    QObject::connect(formatGroup, &QButtonGroup::idClicked, dlg, [table, dialogStatusLabel, state, mergeModeButton, splitModeButton, startButton, saveVisiblePaths, restoreVisiblePaths, restoreVisibleStates, updateStartEnabled](int id) {
        state->settings.setValue(QStringLiteral("convert/lastToolId"), id);
        const bool nextRosbagToPcd = id == kRosbagToPcdFormatId;
        const bool nextPcapTool = isPcapTool(id);
        const auto nextFormat = (nextRosbagToPcd || nextPcapTool)
            ? Lvx2Convert::Format::PCD
            : static_cast<Lvx2Convert::Format>(id);
        if (state->currentToolId != id) {
            const int previousSourceType = sourceTypeForTool(state->currentToolId);
            saveVisiblePaths();
            state->currentFormat = nextFormat;
            state->currentRosbagToPcd = nextRosbagToPcd;
            state->currentToolId = id;
            state->currentMode = state->modeByTool.value(id, Lvx2Convert::Mode::MergeAllToOne);
            {
                const QSignalBlocker mergeBlocker(mergeModeButton);
                const QSignalBlocker splitBlocker(splitModeButton);
                mergeModeButton->setChecked(state->currentMode == Lvx2Convert::Mode::MergeAllToOne);
                splitModeButton->setChecked(state->currentMode == Lvx2Convert::Mode::SplitBy100ms);
            }
            mergeModeButton->setText(QStringLiteral("合并为单个文件"));
            splitModeButton->setText(nextPcapTool ? QStringLiteral("按设备拆分") : QStringLiteral("按单帧拆分"));
            startButton->setText(nextPcapTool ? QStringLiteral("开始提取") : QStringLiteral("开始转换"));
            if (previousSourceType != sourceTypeForTool(id)) {
                restoreVisiblePaths();
            } else {
                restoreVisibleStates();
                updateModeText(table, mergeModeButton->isChecked()
                    ? QStringLiteral("合并为单个文件")
                    : (nextPcapTool ? QStringLiteral("按设备拆分") : QStringLiteral("按单帧拆分")));
                updateStartEnabled();
            }
            dialogStatusLabel->setText(table->rowCount() > 0 ? QStringLiteral("已添加任务") : addFilePrompt(state->currentRosbagToPcd, nextPcapTool));
        }
    });
    QObject::connect(mergeModeButton, &QRadioButton::toggled, dlg, [table, dialogStatusLabel, state, selectedModeText](bool checked) {
        if (checked) {
            state->currentMode = Lvx2Convert::Mode::MergeAllToOne;
            updateModeText(table, selectedModeText());
            for (int row = 0; row < table->rowCount(); ++row) setRowOutputFiles(table, row, {});
            resetJobStatuses(table);
            dialogStatusLabel->setText(table->rowCount() > 0 ? QStringLiteral("已添加任务") : addFilePrompt(state->currentRosbagToPcd, isPcapTool(state->currentToolId)));
        }
    });
    QObject::connect(splitModeButton, &QRadioButton::toggled, dlg, [table, dialogStatusLabel, state, selectedModeText](bool checked) {
        if (checked) {
            state->currentMode = Lvx2Convert::Mode::SplitBy100ms;
            updateModeText(table, selectedModeText());
            for (int row = 0; row < table->rowCount(); ++row) setRowOutputFiles(table, row, {});
            resetJobStatuses(table);
            dialogStatusLabel->setText(table->rowCount() > 0 ? QStringLiteral("已添加任务") : addFilePrompt(state->currentRosbagToPcd, isPcapTool(state->currentToolId)));
        }
    });
    QObject::connect(addFilesButton, &QToolButton::clicked, dlg, [dlg, state, addPaths]() {
        const QString startDir = state->settings.value("convert/lastSourceDir", QDir::homePath()).toString();
        addPaths(state->currentRosbagToPcd ? selectRosbagFiles(dlg, startDir)
                                          : (isPcapTool(state->currentToolId) ? selectPcapFiles(dlg, startDir) : selectLvx2Files(dlg, startDir)));
    });
    QObject::connect(addFolderButton, &QToolButton::clicked, dlg, [dlg, state, addPaths]() {
        const QString startDir = state->settings.value("convert/lastSourceDir", QDir::homePath()).toString();
        const QString folder = selectFolder(dlg,
                                            startDir,
                                            isPcapTool(state->currentToolId) ? QStringLiteral("添加 PCAP 文件夹") : state->currentRosbagToPcd
                                                ? QStringLiteral("添加 ROSbag 文件夹")
                                                : QStringLiteral("添加 LVX2 文件夹"));
        if (folder.isEmpty()) {
            return;
        }
        state->settings.setValue("convert/lastSourceDir", folder);
        QDir dir(folder);
        const QFileInfoList files = dir.entryInfoList(folderFilters(state->currentRosbagToPcd, isPcapTool(state->currentToolId)), QDir::Files | QDir::Readable, QDir::Name);
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
        dialogStatusLabel->setText(addFilePrompt(state->currentRosbagToPcd, isPcapTool(state->currentToolId)));
        updateStartEnabled();
    });
    QObject::connect(outputFolderButton, &QToolButton::clicked, dlg, [dlg, customOutputRadio, outputDirEdit, state]() {
        customOutputRadio->setChecked(true);
        const QString startDir = outputDirEdit->text().trimmed().isEmpty()
            ? state->settings.value("convert/lastSourceDir", QDir::homePath()).toString()
            : outputDirEdit->text().trimmed();
        const QString folder = selectFolder(dlg, startDir, "选择输出目录");
        if (!folder.isEmpty()) {
            outputDirEdit->setText(folder);
            state->settings.setValue("convert/lastOutputDir", folder);
        }
    });
    QObject::connect(table->model(), &QAbstractItemModel::rowsRemoved, dlg, [table, dialogStatusLabel, state, updateStartEnabled]() {
        updateStartEnabled();
        if (table->rowCount() == 0) {
            dialogStatusLabel->setText(addFilePrompt(state->currentRosbagToPcd, isPcapTool(state->currentToolId)));
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
        rosbagPcdButton,
        pcapLvx2Button,
        pcapImuButton,
        pcapInfoButton,
        pointGroupButton,
        extractGroupButton,
        mergeModeButton,
        splitModeButton,
        sameSourceOutputRadio,
        customOutputRadio,
        outputDirEdit,
        outputFolderButton
    };

    QObject::connect(startButton, &QPushButton::clicked, dlg, [this, dlg, table, sameSourceOutputRadio, outputDirEdit, dialogStatusLabel, state, startButton, lockedControls, selectedModeText, updateStartEnabled]() {
        if (state->extractionRunning) {
            state->extractionCancellation->store(true);
            startButton->setEnabled(false);
            startButton->setText(QStringLiteral("正在取消..."));
            dialogStatusLabel->setText(QStringLiteral("正在取消当前任务..."));
            return;
        }
        if (table->rowCount() == 0) {
            dialogStatusLabel->setText(state->currentRosbagToPcd ? QStringLiteral("请先添加 ROSbag 文件")
                : (isPcapTool(state->currentToolId) ? QStringLiteral("请先添加 PCAP 文件") : QStringLiteral("请先添加 LVX2 文件")));
            return;
        }
        const bool useSourceDir = sameSourceOutputRadio->isChecked();
        const QString customOutputDir = outputDirEdit->text().trimmed();
        if (!useSourceDir && customOutputDir.isEmpty()) {
            dialogStatusLabel->setText("请选择输出目录");
            return;
        }
        if (!useSourceDir && !QDir(customOutputDir).exists()) {
            dialogStatusLabel->setText("输出目录不存在");
            return;
        }
        state->settings.setValue("convert/useSourceDir", useSourceDir);
        if (!customOutputDir.isEmpty()) {
            state->settings.setValue("convert/lastOutputDir", customOutputDir);
        }

        if (isPcapTool(state->currentToolId)) {
            QStringList sources;
            QStringList outputDirectories;
            for (int row = 0; row < table->rowCount(); ++row) {
                const QString source = rowPath(table, row);
                sources.push_back(source);
                outputDirectories.push_back(outputDirForSource(source, useSourceDir, customOutputDir));
                if (QTableWidgetItem* item = table->item(row, kColumnMode)) item->setText(selectedModeText());
                setStatusProgress(table, row, 0);
            }
            setControlsEnabled(lockedControls, false);
            state->extractionRunning = true;
            state->extractionCancellation = std::make_shared<std::atomic_bool>(false);
            startButton->setText(QStringLiteral("取消"));
            startButton->setEnabled(true);
            updateAllActionButtons(table, true);
            dialogStatusLabel->setText(QStringLiteral("正在提取..."));
            const PcapExtractor::Kind kind = extractionKind(state->currentToolId);
            const PcapExtractor::Mode mode = state->currentMode == Lvx2Convert::Mode::MergeAllToOne
                ? PcapExtractor::Mode::Merge : PcapExtractor::Mode::SplitByDevice;
            QPointer<QTableWidget> safeTable(table);
            auto* watcher = new QFutureWatcher<QVector<PcapExtractor::Result>>(dlg);
            QObject::connect(watcher, &QFutureWatcher<QVector<PcapExtractor::Result>>::finished, dlg,
                [dlg, watcher, table, dialogStatusLabel, state, startButton, lockedControls, updateStartEnabled]() {
                    const QVector<PcapExtractor::Result> results = watcher->result();
                    QStringList failures;
                    QStringList warnings;
                    for (int row = 0; row < results.size(); ++row) {
                        if (results[row].ok) {
                            setRowOutputFiles(table, row, results[row].outputFiles);
                            setStatusDone(table, row);
                            warnings.append(results[row].warnings);
                        } else {
                            setRowOutputFiles(table, row, {});
                            setStatusFailed(table, row);
                            failures.push_back(QStringLiteral("%1：%2").arg(QFileInfo(rowPath(table, row)).fileName(), results[row].errorMessage));
                        }
                        updateActionButtons(table, row, false);
                    }
                    setControlsEnabled(lockedControls, true);
                    state->extractionRunning = false;
                    state->extractionCancellation.reset();
                    startButton->setText(QStringLiteral("开始提取"));
                    updateAllActionButtons(table, false);
                    updateStartEnabled();
                    dialogStatusLabel->setText(failures.isEmpty() ? QStringLiteral("提取完成") : QStringLiteral("部分任务提取失败"));
                    if (!failures.isEmpty()) {
                        QMessageBox::warning(dlg, QStringLiteral("PCAP 数据提取"), failures.join(QLatin1Char('\n')));
                    } else if (!warnings.isEmpty()) {
                        QMessageBox::information(dlg, QStringLiteral("PCAP 数据提取"), warnings.join(QLatin1Char('\n')));
                    }
                    watcher->deleteLater();
                });
            const std::shared_ptr<std::atomic_bool> cancellation = state->extractionCancellation;
            QObject::connect(dlg, &QObject::destroyed, [cancellation]() { cancellation->store(true); });
            watcher->setFuture(QtConcurrent::run([sources, outputDirectories, kind, mode, safeTable, cancellation]() {
                QVector<PcapExtractor::Result> results;
                results.reserve(sources.size());
                for (int row = 0; row < sources.size(); ++row) {
                    if (cancellation->load()) {
                        PcapExtractor::Result cancelled;
                        cancelled.errorMessage = QStringLiteral("任务已取消");
                        results.push_back(cancelled);
                        continue;
                    }
                    results.push_back(PcapExtractor::extract(sources[row], outputDirectories[row], kind, mode,
                        [safeTable, row](int value) {
                            if (!safeTable) return;
                            QMetaObject::invokeMethod(safeTable, [safeTable, row, value]() {
                                if (safeTable) setStatusProgress(safeTable, row, value);
                            }, Qt::QueuedConnection);
                        }, cancellation.get()));
                }
                return results;
            }));
            return;
        }

        setControlsEnabled(lockedControls, false);
        startButton->setEnabled(false);
        updateAllActionButtons(table, true);
        dialogStatusLabel->setText("正在转换...");
        dlg->setCursor(Qt::BusyCursor);

        for (int row = 0; row < table->rowCount(); ++row) {
            const QString srcPath = rowPath(table, row);
            const QString outputDir = outputDirForSource(srcPath, useSourceDir, customOutputDir);
            const QString outputNoExt = QDir(outputDir).filePath(QFileInfo(srcPath).completeBaseName());
            if (QTableWidgetItem* item = table->item(row, kColumnMode)) {
                item->setText(selectedModeText());
            }

            setStatusProgress(table, row, 0);
            setRowOutputFiles(table, row, {});
            QApplication::processEvents();
            QStringList outputFiles;
            const bool ok = state->currentRosbagToPcd
                ? convertRosbagToPcdFile(srcPath, outputNoExt, state->currentMode, [table, row](int done, int total) {
                    const int value = total > 0 ? done * 100 / total : 0;
                    setStatusProgress(table, row, value);
                    QApplication::processEvents();
                }, &outputFiles)
                : convertLvx2File(srcPath, outputNoExt, state->currentMode, state->currentFormat, [table, row](int done, int total) {
                const int value = total > 0 ? done * 100 / total : 0;
                setStatusProgress(table, row, value);
                QApplication::processEvents();
            }, &outputFiles);
            if (ok) {
                setRowOutputFiles(table, row, outputFiles);
                setStatusDone(table, row);
            } else {
                setRowOutputFiles(table, row, {});
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

    const int savedToolId = state->settings.value(QStringLiteral("convert/lastToolId"), int(Lvx2Convert::Format::PCD)).toInt();
    if (QAbstractButton* savedButton = formatGroup->button(savedToolId)) {
        savedButton->click();
    }
    dlg->show();
    dlg->raise();
    dlg->activateWindow();
}

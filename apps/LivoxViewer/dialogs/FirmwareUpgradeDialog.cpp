#include "LivoxViewerWindow.h"
#include "widgets/SwitchCheckBox.h"

#include <QCloseEvent>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSet>
#include <QSizePolicy>
#include <QStandardPaths>
#include <QTimer>
#include <QVBoxLayout>

#include <thread>

namespace {

class FirmwareUpgradeDialog final : public QDialog {
public:
    using QDialog::QDialog;

    bool upgradeRunning = false;

    void reject() override
    {
        if (!upgradeRunning) {
            QDialog::reject();
        }
    }

protected:
    void closeEvent(QCloseEvent* event) override
    {
        if (upgradeRunning) {
            event->ignore();
            return;
        }
        QDialog::closeEvent(event);
    }
};

struct UpgradeDeviceRow {
    uint32_t handle = 0;
    int deviceType = 0;
    SwitchCheckBox* switchBox = nullptr;
    QLabel* workStateLabel = nullptr;
    QLabel* progressLabel = nullptr;
    QProgressBar* progressBar = nullptr;
};

struct FirmwareGroupRow {
    int deviceType = 0;
    QString modelName;
    QLineEdit* firmwareEdit = nullptr;
    QPushButton* browseButton = nullptr;
};

struct FirmwareUpgradeBatch {
    int deviceType = 0;
    QString modelName;
    QString firmwarePath;
    QVector<uint32_t> handles;
};

struct FirmwareUpgradeRuntime final : QObject {
    using QObject::QObject;

    QVector<FirmwareUpgradeBatch> batches;
    QMap<uint32_t, QProgressBar*> progressBars;
    QMap<uint32_t, QLabel*> progressLabels;
    QMap<uint32_t, int> progressByHandle;
    QSet<uint32_t> finishedHandles;
    QSet<uint32_t> failedHandles;
    QLabel* summaryLabel = nullptr;
    QPushButton* startButton = nullptr;
    QPushButton* closeButton = nullptr;
    FirmwareUpgradeDialog* dialog = nullptr;
    int activeBatchIndex = -1;
    bool upgradeRunning = false;
    bool advancingBatch = false;
    std::function<void()> startNextBatch;
};

QString firmwarePathSettingKey(int deviceType)
{
    return QString("upgrade/firmwarePath/%1").arg(deviceType);
}

QString modelNameForDevice(const LidarDeviceInfo& device)
{
    return device.product_info.isEmpty() ? QStringLiteral("Unknown") : device.product_info;
}

QString workStateText(const LidarDeviceInfo& device)
{
    return device.work_state.isEmpty() ? QStringLiteral("读取中") : device.work_state;
}

QString upgradeStateText(LivoxLidarFsmEvent state)
{
    switch (state) {
    case kLivoxLidarEventRequestUpgrade: return QStringLiteral("请求升级");
    case kLivoxLidarEventXferFirmware: return QStringLiteral("传输固件");
    case kLivoxLidarEventCompleteXferFirmware: return QStringLiteral("传输完成");
    case kLivoxLidarEventGetUpgradeProgress: return QStringLiteral("写入中");
    case kLivoxLidarEventComplete: return QStringLiteral("完成");
    case kLivoxLidarEventReinit: return QStringLiteral("等待重启");
    case kLivoxLidarEventTimeout: return QStringLiteral("超时");
    case kLivoxLidarEventErr: return QStringLiteral("失败");
    case kLivoxLidarEventUndef: return QStringLiteral("未知");
    }
    return QStringLiteral("未知");
}

bool isUpgradeFinishedState(LivoxLidarFsmEvent state, uint8_t progress)
{
    return state == kLivoxLidarEventComplete ||
           state == kLivoxLidarEventTimeout ||
           state == kLivoxLidarEventErr ||
           progress >= 100;
}

bool isUpgradeFailedState(LivoxLidarFsmEvent state)
{
    return state == kLivoxLidarEventTimeout || state == kLivoxLidarEventErr;
}

bool batchContainsHandle(const FirmwareUpgradeBatch& batch, uint32_t handle)
{
    for (uint32_t batchHandle : batch.handles) {
        if (batchHandle == handle) {
            return true;
        }
    }
    return false;
}

void setProgressWidgets(FirmwareUpgradeRuntime* runtime, uint32_t handle, int value, const QString& text)
{
    if (QProgressBar* progressBar = runtime->progressBars.value(handle, nullptr)) {
        progressBar->setValue(value);
        progressBar->setFormat(QString("%1%").arg(value));
    }
    if (QLabel* progressLabel = runtime->progressLabels.value(handle, nullptr)) {
        progressLabel->setText(text);
    }
}

} // namespace

void LivoxViewerWindow::showFirmwareUpgradeDialog()
{
    const QVector<LidarDeviceInfo> devices = connectedLidarDevicesSnapshot();
    if (devices.isEmpty()) {
        QMessageBox::warning(this, "固件升级", "没有可用的已连接设备");
        return;
    }

    QSettings settings("Livox", "LivoxViewerQT");
    const QString docs = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString defaultDir = settings.value("upgrade/lastFirmwareDir",
                                              docs.isEmpty() ? QDir::homePath() : docs).toString();

    FirmwareUpgradeDialog dialog(this);
    FirmwareUpgradeRuntime runtime;
    runtime.dialog = &dialog;
    dialog.setWindowTitle("固件升级");
    dialog.resize(760, 620);

    QVBoxLayout* rootLayout = new QVBoxLayout(&dialog);
    rootLayout->setContentsMargins(14, 14, 14, 14);
    rootLayout->setSpacing(10);

    QLabel* introLabel = new QLabel("按雷达型号分别选择固件，并勾选需要升级的设备。", &dialog);
    introLabel->setStyleSheet("color: palette(mid);");
    introLabel->setWordWrap(true);
    rootLayout->addWidget(introLabel);

    QMap<int, QVector<LidarDeviceInfo>> devicesByType;
    for (const LidarDeviceInfo& device : devices) {
        devicesByType[int(device.dev_type)].append(device);
    }

    QVector<UpgradeDeviceRow> upgradeRows;
    QVector<FirmwareGroupRow> groupRows;

    QWidget* groupListWidget = new QWidget(&dialog);
    QVBoxLayout* groupListLayout = new QVBoxLayout(groupListWidget);
    groupListLayout->setContentsMargins(0, 0, 0, 0);
    groupListLayout->setSpacing(10);

    for (auto it = devicesByType.constBegin(); it != devicesByType.constEnd(); ++it) {
        const int deviceType = it.key();
        const QVector<LidarDeviceInfo>& groupDevices = it.value();
        const QString modelName = modelNameForDevice(groupDevices.first());

        QFrame* groupFrame = new QFrame(groupListWidget);
        groupFrame->setObjectName("FirmwareUpgradeGroup");
        groupFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
        groupFrame->setStyleSheet(
            "QFrame#FirmwareUpgradeGroup {"
            "  border: 1px solid palette(mid);"
            "  border-radius: 6px;"
            "  background: palette(base);"
            "}"
        );

        QVBoxLayout* groupLayout = new QVBoxLayout(groupFrame);
        groupLayout->setContentsMargins(12, 10, 12, 10);
        groupLayout->setSpacing(8);

        QLabel* groupTitle = new QLabel(QString("%1  (%2 台)").arg(modelName).arg(groupDevices.size()), groupFrame);
        QFont groupTitleFont = groupTitle->font();
        groupTitleFont.setBold(true);
        groupTitle->setFont(groupTitleFont);
        groupLayout->addWidget(groupTitle);

        QWidget* firmwareRow = new QWidget(groupFrame);
        QHBoxLayout* firmwareLayout = new QHBoxLayout(firmwareRow);
        firmwareLayout->setContentsMargins(0, 0, 0, 0);
        firmwareLayout->setSpacing(8);
        QLabel* firmwareLabel = new QLabel("固件:", firmwareRow);
        QLineEdit* firmwareEdit = new QLineEdit(firmwareRow);
        firmwareEdit->setPlaceholderText(QString("选择 %1 固件文件").arg(modelName));
        firmwareEdit->setText(settings.value(firmwarePathSettingKey(deviceType)).toString());
        QPushButton* browseButton = new QPushButton("浏览", firmwareRow);
        firmwareLayout->addWidget(firmwareLabel);
        firmwareLayout->addWidget(firmwareEdit, 1);
        firmwareLayout->addWidget(browseButton);
        groupLayout->addWidget(firmwareRow);
        groupRows.append({deviceType, modelName, firmwareEdit, browseButton});

        connect(browseButton, &QPushButton::clicked, &dialog, [&, firmwareEdit, modelName]() {
            const QString currentPath = firmwareEdit->text().trimmed();
            const QString initialDir = currentPath.isEmpty() ? defaultDir : QFileInfo(currentPath).absolutePath();
            QFileDialog fileDialog(&dialog,
                                   QString("选择 %1 固件文件").arg(modelName),
                                   initialDir,
                                   "固件 (*.bin *.img);;所有文件 (*.*)");
            fileDialog.setFileMode(QFileDialog::ExistingFile);
            fileDialog.setAcceptMode(QFileDialog::AcceptOpen);
            fileDialog.setOption(QFileDialog::DontUseNativeDialog, true);
            if (fileDialog.exec() == QDialog::Accepted && !fileDialog.selectedFiles().isEmpty()) {
                firmwareEdit->setText(QDir::toNativeSeparators(fileDialog.selectedFiles().first()));
            }
        });

        for (const LidarDeviceInfo& device : groupDevices) {
            QFrame* card = new QFrame(groupFrame);
            card->setObjectName("FirmwareUpgradeDeviceCard");
            card->setFrameShape(QFrame::StyledPanel);
            card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            card->setStyleSheet("QFrame#FirmwareUpgradeDeviceCard { border: 1px solid palette(mid); border-radius: 6px; background: palette(alternate-base); }");

            QVBoxLayout* cardLayout = new QVBoxLayout(card);
            cardLayout->setContentsMargins(8, 6, 8, 6);
            cardLayout->setSpacing(6);

            QHBoxLayout* infoLayout = new QHBoxLayout();
            infoLayout->setContentsMargins(0, 0, 0, 0);
            infoLayout->setSpacing(8);

            SwitchCheckBox* switchBox = new SwitchCheckBox(card);
            switchBox->setChecked(hasCurrentLidarHandle && device.handle == currentLidarHandle);
            infoLayout->addWidget(switchBox);

            QVBoxLayout* textLayout = new QVBoxLayout();
            textLayout->setContentsMargins(0, 0, 0, 0);
            textLayout->setSpacing(3);

            QLabel* titleLabel = new QLabel(QString("%1 / %2").arg(modelName, device.sn), card);
            QFont titleFont = titleLabel->font();
            titleFont.setBold(true);
            titleLabel->setFont(titleFont);
            titleLabel->setWordWrap(true);
            QLabel* ipLabel = new QLabel(QString("IP: %1").arg(device.lidar_ip), card);
            QLabel* firmwareVersionLabel = new QLabel(QString("当前固件: %1").arg(device.firmware_version.isEmpty() ? QStringLiteral("读取中") : device.firmware_version), card);
            QLabel* workStateLabel = new QLabel(QString("状态: %1").arg(workStateText(device)), card);
            for (QLabel* label : {ipLabel, firmwareVersionLabel, workStateLabel}) {
                label->setWordWrap(true);
            }
            textLayout->addWidget(titleLabel);
            textLayout->addWidget(ipLabel);
            textLayout->addWidget(firmwareVersionLabel);
            textLayout->addWidget(workStateLabel);
            infoLayout->addLayout(textLayout, 1);
            cardLayout->addLayout(infoLayout);

            QHBoxLayout* progressLayout = new QHBoxLayout();
            progressLayout->setContentsMargins(0, 0, 0, 0);
            progressLayout->setSpacing(8);
            QLabel* progressLabel = new QLabel("等待开始", card);
            progressLabel->setMinimumWidth(74);
            QProgressBar* progressBar = new QProgressBar(card);
            progressBar->setRange(0, 100);
            progressBar->setValue(0);
            progressBar->setTextVisible(true);
            progressBar->setFormat("0%");
            progressLayout->addWidget(progressLabel);
            progressLayout->addWidget(progressBar, 1);
            cardLayout->addLayout(progressLayout);

            groupLayout->addWidget(card);
            upgradeRows.append({device.handle, deviceType, switchBox, workStateLabel, progressLabel, progressBar});
        }

        groupListLayout->addWidget(groupFrame);
    }
    groupListLayout->addStretch();

    QScrollArea* groupScroll = new QScrollArea(&dialog);
    groupScroll->setWidgetResizable(true);
    groupScroll->setFrameShape(QFrame::NoFrame);
    groupScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    groupScroll->setWidget(groupListWidget);
    rootLayout->addWidget(groupScroll, 1);

    QLabel* summaryLabel = new QLabel("准备升级", &dialog);
    summaryLabel->setStyleSheet("color: palette(mid);");
    runtime.summaryLabel = summaryLabel;
    rootLayout->addWidget(summaryLabel);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->setContentsMargins(0, 0, 0, 0);
    buttonLayout->addStretch();
    QPushButton* startButton = new QPushButton("开始升级", &dialog);
    QPushButton* closeButton = new QPushButton("关闭", &dialog);
    startButton->setMinimumWidth(110);
    closeButton->setMinimumWidth(90);
    runtime.startButton = startButton;
    runtime.closeButton = closeButton;
    buttonLayout->addWidget(startButton);
    buttonLayout->addWidget(closeButton);
    rootLayout->addLayout(buttonLayout);

    QTimer statusRefreshTimer(&dialog);
    connect(&statusRefreshTimer, &QTimer::timeout, &dialog, [&]() {
        QMap<uint32_t, LidarDeviceInfo> snapshot;
        {
            QMutexLocker locker(&lidarDeviceMutex);
            snapshot = lidarDevices;
        }
        for (UpgradeDeviceRow& row : upgradeRows) {
            const auto it = snapshot.constFind(row.handle);
            if (it != snapshot.constEnd()) {
                row.workStateLabel->setText(QString("状态: %1").arg(workStateText(it.value())));
            }
        }
    });
    statusRefreshTimer.start(1000);

    runtime.startNextBatch = [&]() {
        runtime.advancingBatch = false;
        ++runtime.activeBatchIndex;
        if (runtime.activeBatchIndex >= runtime.batches.size()) {
            const bool hasFailure = !runtime.failedHandles.isEmpty();
            runtime.upgradeRunning = false;
            dialog.upgradeRunning = false;
            startButton->setEnabled(false);
            closeButton->setEnabled(true);
            summaryLabel->setText(hasFailure
                ? "升级流程结束，部分设备失败，请查看设备进度。"
                : "升级流程完成，请等待设备重启。");
            logMessage(hasFailure ? "固件升级流程结束，部分设备失败" : "固件升级已完成，请等待设备重启");
            return;
        }

        const FirmwareUpgradeBatch batch = runtime.batches.at(runtime.activeBatchIndex);
        summaryLabel->setText(QString("正在升级 %1 (%2/%3)")
                                  .arg(batch.modelName)
                                  .arg(runtime.activeBatchIndex + 1)
                                  .arg(runtime.batches.size()));

        const QByteArray pathLocal = QDir::toNativeSeparators(batch.firmwarePath).toLocal8Bit();
        if (!SetLivoxLidarUpgradeFirmwarePath(pathLocal.constData())) {
            for (uint32_t handle : batch.handles) {
                runtime.failedHandles.insert(handle);
                runtime.finishedHandles.insert(handle);
                setProgressWidgets(&runtime, handle, runtime.progressByHandle.value(handle, 0), "路径失败");
            }
            QTimer::singleShot(300, &runtime, runtime.startNextBatch);
            return;
        }

        for (uint32_t handle : batch.handles) {
            setProgressWidgets(&runtime, handle, 0, "等待设备");
        }

        const QVector<uint32_t> handles = batch.handles;
        std::thread([handles]() {
            UpgradeLivoxLidars(handles.data(), uint8_t(handles.size()));
        }).detach();
    };

    SetLivoxLidarUpgradeProgressCallback([](uint32_t handle, LivoxLidarUpgradeState state, void* clientData) {
        FirmwareUpgradeRuntime* runtime = static_cast<FirmwareUpgradeRuntime*>(clientData);
        if (!runtime) {
            return;
        }

        QMetaObject::invokeMethod(runtime, [runtime, handle, state]() {
            if (!runtime->upgradeRunning ||
                runtime->activeBatchIndex < 0 ||
                runtime->activeBatchIndex >= runtime->batches.size()) {
                return;
            }

            const FirmwareUpgradeBatch& batch = runtime->batches.at(runtime->activeBatchIndex);
            if (!batchContainsHandle(batch, handle)) {
                return;
            }

            const bool failed = isUpgradeFailedState(state.state);
            const bool finished = isUpgradeFinishedState(state.state, state.progress);
            const int progress = failed
                ? runtime->progressByHandle.value(handle, 0)
                : (finished ? 100 : int(state.progress));
            runtime->progressByHandle[handle] = progress;

            if (finished) {
                runtime->finishedHandles.insert(handle);
            }
            if (failed) {
                runtime->failedHandles.insert(handle);
            }

            setProgressWidgets(runtime, handle, progress, upgradeStateText(state.state));

            bool batchComplete = true;
            for (uint32_t batchHandle : batch.handles) {
                if (!runtime->finishedHandles.contains(batchHandle)) {
                    batchComplete = false;
                    break;
                }
            }

            if (batchComplete && !runtime->advancingBatch) {
                runtime->advancingBatch = true;
                QTimer::singleShot(500, runtime, runtime->startNextBatch);
            }
        }, Qt::QueuedConnection);
    }, &runtime);

    connect(startButton, &QPushButton::clicked, &dialog, [&]() {
        runtime.batches.clear();
        runtime.progressBars.clear();
        runtime.progressLabels.clear();
        runtime.progressByHandle.clear();
        runtime.finishedHandles.clear();
        runtime.failedHandles.clear();
        runtime.activeBatchIndex = -1;
        runtime.advancingBatch = false;

        for (const FirmwareGroupRow& group : groupRows) {
            QVector<uint32_t> selectedHandles;
            for (UpgradeDeviceRow& row : upgradeRows) {
                if (row.deviceType == group.deviceType && row.switchBox->isChecked()) {
                    selectedHandles.append(row.handle);
                    runtime.progressBars.insert(row.handle, row.progressBar);
                    runtime.progressLabels.insert(row.handle, row.progressLabel);
                    runtime.progressByHandle.insert(row.handle, 0);
                    row.progressBar->setValue(0);
                    row.progressBar->setFormat("0%");
                    row.progressLabel->setText("等待开始");
                }
            }
            if (selectedHandles.isEmpty()) {
                continue;
            }

            const QString firmwarePath = group.firmwareEdit->text().trimmed();
            const QFileInfo firmwareInfo(firmwarePath);
            if (firmwarePath.isEmpty() || !firmwareInfo.exists() || !firmwareInfo.isFile()) {
                QMessageBox::warning(&dialog,
                                     "固件升级",
                                     QString("请选择有效的 %1 固件文件").arg(group.modelName));
                return;
            }

            settings.setValue(firmwarePathSettingKey(group.deviceType), firmwareInfo.absoluteFilePath());
            settings.setValue("upgrade/lastFirmwareDir", firmwareInfo.absolutePath());
            runtime.batches.append({group.deviceType, group.modelName, firmwareInfo.absoluteFilePath(), selectedHandles});
        }

        if (runtime.batches.isEmpty()) {
            QMessageBox::warning(&dialog, "固件升级", "请至少开启一个设备的升级开关");
            return;
        }

        for (UpgradeDeviceRow& row : upgradeRows) {
            row.switchBox->setEnabled(false);
        }
        for (const FirmwareGroupRow& group : groupRows) {
            group.firmwareEdit->setEnabled(false);
            group.browseButton->setEnabled(false);
        }

        runtime.upgradeRunning = true;
        dialog.upgradeRunning = true;
        startButton->setEnabled(false);
        closeButton->setEnabled(false);
        logMessage("正在进行固件升级，请勿断电或进行操作!!!");
        runtime.startNextBatch();
    });

    connect(closeButton, &QPushButton::clicked, &dialog, [&dialog]() {
        dialog.reject();
    });

    dialog.exec();
    SetLivoxLidarUpgradeProgressCallback(nullptr, nullptr);
}

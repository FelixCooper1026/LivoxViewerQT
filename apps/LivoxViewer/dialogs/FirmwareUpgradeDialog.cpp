#include "LivoxViewerWindow.h"
#include "widgets/SwitchCheckBox.h"

#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSizePolicy>
#include <QStandardPaths>
#include <QVBoxLayout>

#include <thread>

void LivoxViewerWindow::showFirmwareUpgradeDialog()
{
    const QVector<LidarDeviceInfo> devices = connectedLidarDevicesSnapshot();
    if (devices.isEmpty()) {
        QMessageBox::warning(this, "固件升级", "没有可用的已连接设备");
        return;
    }

    QSettings settings("Livox", "LivoxViewerQT");
    const QString docs = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString lastDir = settings.value("upgrade/lastFirmwareDir",
                                           docs.isEmpty() ? QDir::homePath() : docs).toString();

    QDialog dialog(this);
    dialog.setWindowTitle("固件升级");
    dialog.resize(460, 420);

    QVBoxLayout* rootLayout = new QVBoxLayout(&dialog);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(10);

    QHBoxLayout* fileLayout = new QHBoxLayout();
    fileLayout->setContentsMargins(0, 0, 0, 0);
    fileLayout->setSpacing(6);
    QLabel* fileLabel = new QLabel("固件:", &dialog);
    QLineEdit* firmwareEdit = new QLineEdit(&dialog);
    firmwareEdit->setPlaceholderText("选择固件文件");
    QPushButton* browseButton = new QPushButton("浏览", &dialog);
    fileLayout->addWidget(fileLabel);
    fileLayout->addWidget(firmwareEdit, 1);
    fileLayout->addWidget(browseButton);
    rootLayout->addLayout(fileLayout);

    struct UpgradeDeviceRow {
        uint32_t handle = 0;
        SwitchCheckBox* switchBox = nullptr;
    };
    QVector<UpgradeDeviceRow> upgradeRows;

    QWidget* listWidget = new QWidget(&dialog);
    QVBoxLayout* listLayout = new QVBoxLayout(listWidget);
    listLayout->setContentsMargins(0, 0, 0, 0);
    listLayout->setSpacing(6);

    for (const LidarDeviceInfo& device : devices) {
        QFrame* card = new QFrame(listWidget);
        card->setObjectName("FirmwareUpgradeDeviceCard");
        card->setFrameShape(QFrame::StyledPanel);
        card->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        card->setStyleSheet("QFrame#FirmwareUpgradeDeviceCard { border: 1px solid palette(mid); border-radius: 6px; background: palette(base); }");

        QHBoxLayout* cardLayout = new QHBoxLayout(card);
        cardLayout->setContentsMargins(8, 6, 8, 6);
        cardLayout->setSpacing(8);

        SwitchCheckBox* switchBox = new SwitchCheckBox(card);
        switchBox->setChecked(hasCurrentLidarHandle && device.handle == currentLidarHandle);
        cardLayout->addWidget(switchBox);

        QVBoxLayout* textLayout = new QVBoxLayout();
        textLayout->setContentsMargins(0, 0, 0, 0);
        textLayout->setSpacing(3);

        const QString modelName = device.product_info.isEmpty() ? QStringLiteral("Unknown") : device.product_info;
        const QString statusText = device.is_streaming ? QStringLiteral("数据流中") : QStringLiteral("已连接");
        QLabel* titleLabel = new QLabel(QString("%1 / %2").arg(modelName, statusText), card);
        QFont titleFont = titleLabel->font();
        titleFont.setBold(true);
        titleLabel->setFont(titleFont);
        titleLabel->setWordWrap(true);
        QLabel* snLabel = new QLabel(QString("SN: %1").arg(device.sn), card);
        QLabel* ipLabel = new QLabel(QString("IP: %1").arg(device.lidar_ip), card);
        const QString firmwareVersion = device.firmware_version.isEmpty() ? QStringLiteral("读取中") : device.firmware_version;
        QLabel* firmwareLabel = new QLabel(QString("固件: %1").arg(firmwareVersion), card);
        snLabel->setWordWrap(true);
        ipLabel->setWordWrap(true);
        firmwareLabel->setWordWrap(true);
        textLayout->addWidget(titleLabel);
        textLayout->addWidget(snLabel);
        textLayout->addWidget(ipLabel);
        textLayout->addWidget(firmwareLabel);
        cardLayout->addLayout(textLayout, 1);

        listLayout->addWidget(card);
        upgradeRows.append({device.handle, switchBox});
    }
    listLayout->addStretch();

    QScrollArea* listScroll = new QScrollArea(&dialog);
    listScroll->setWidgetResizable(true);
    listScroll->setFrameShape(QFrame::NoFrame);
    listScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    listScroll->setMinimumHeight(220);
    listScroll->setWidget(listWidget);
    rootLayout->addWidget(listScroll, 1);

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    buttonBox->button(QDialogButtonBox::Ok)->setText("开始升级");
    rootLayout->addWidget(buttonBox);

    connect(browseButton, &QPushButton::clicked, &dialog, [&]() {
        const QString currentPath = firmwareEdit->text().trimmed();
        const QString initialDir = currentPath.isEmpty() ? lastDir : QFileInfo(currentPath).absolutePath();
        QFileDialog fileDialog(&dialog, "选择固件文件", initialDir, "固件 (*.bin *.img);;所有文件 (*.*)");
        fileDialog.setFileMode(QFileDialog::ExistingFile);
        fileDialog.setAcceptMode(QFileDialog::AcceptOpen);
        fileDialog.setOption(QFileDialog::DontUseNativeDialog, true);
        if (fileDialog.exec() == QDialog::Accepted && !fileDialog.selectedFiles().isEmpty()) {
            firmwareEdit->setText(QDir::toNativeSeparators(fileDialog.selectedFiles().first()));
        }
    });
    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    QVector<uint32_t> selectedHandles;
    connect(buttonBox, &QDialogButtonBox::accepted, &dialog, [&]() {
        const QString firmwarePath = firmwareEdit->text().trimmed();
        QFileInfo firmwareInfo(firmwarePath);
        if (firmwarePath.isEmpty() || !firmwareInfo.exists() || !firmwareInfo.isFile()) {
            QMessageBox::warning(&dialog, "固件升级", "请选择有效的固件文件");
            return;
        }

        selectedHandles.clear();
        for (const UpgradeDeviceRow& row : upgradeRows) {
            if (row.switchBox->isChecked()) {
                selectedHandles.append(row.handle);
            }
        }
        if (selectedHandles.isEmpty()) {
            QMessageBox::warning(&dialog, "固件升级", "请至少开启一个设备的升级开关");
            return;
        }

        settings.setValue("upgrade/lastFirmwareDir", firmwareInfo.absolutePath());

        const QByteArray pathLocal = QDir::toNativeSeparators(firmwareInfo.absoluteFilePath()).toLocal8Bit();
        const bool okPath = SetLivoxLidarUpgradeFirmwarePath(pathLocal.constData());
        if (!okPath) {
            QMessageBox::critical(&dialog, "固件升级", "设置固件路径失败，请确保选择单个固件文件，路径避免包含特殊字符");
            return;
        }

        dialog.accept();
    });

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    {
        QMutexLocker locker(&upgradeProgressMutex);
        upgradeProgressMap.clear();
        for (uint32_t handle : selectedHandles) {
            upgradeProgressMap[handle] = 0;
        }
        upgradeTotalDevices = selectedHandles.size();
    }

    SetLivoxLidarUpgradeProgressCallback([](uint32_t handle, LivoxLidarUpgradeState state, void* client){
        LivoxViewerWindow* w = static_cast<LivoxViewerWindow*>(client);
        if (!w || !w->captureState.progress) return;
        QMetaObject::invokeMethod(w, [w, handle, state]() {
            int avgProgress = 0;
            int completedCount = 0;
            int totalDevices = 0;
            bool allComplete = false;

            {
                QMutexLocker locker(&w->upgradeProgressMutex);

                if (!w->upgradeProgressMap.contains(handle)) {
                    return;
                }

                const bool isComplete = (state.state == 4) || (state.progress >= 100);
                w->upgradeProgressMap[handle] = isComplete ? 100 : state.progress;

                totalDevices = w->upgradeTotalDevices;
                if (totalDevices == 0) {
                    return;
                }

                int totalProgress = 0;
                for (int progress : w->upgradeProgressMap.values()) {
                    totalProgress += progress;
                    if (progress >= 100) {
                        completedCount++;
                    }
                }

                avgProgress = totalProgress / totalDevices;
                allComplete = completedCount == totalDevices;
                if (allComplete) {
                    w->upgradeProgressMap.clear();
                    w->upgradeTotalDevices = 0;
                }
            }

            w->captureState.progress->setValue(avgProgress);
            w->captureState.progress->setFormat(QString("升级进度 %1% (%2/%3)").arg(avgProgress).arg(completedCount).arg(totalDevices));

            if (allComplete) {
                w->statusLabelBar->setText(QString("升级完成 (%1个设备)，请等待设备重启").arg(totalDevices));
                w->logMessage("固件升级已完成，请等待设备重启");
            }
        });
    }, this);

    const QVector<uint32_t> handleArr = selectedHandles;
    std::thread([handleArr]() {
        UpgradeLivoxLidars(handleArr.data(), handleArr.size());
    }).detach();

    captureState.progress->setValue(0);
    captureState.progress->setFormat(QString("升级进度 0% (0/%1)").arg(selectedHandles.size()));
    statusLabelBar->setText(QString("正在升级 %1 个设备，请勿断电或进行操作").arg(selectedHandles.size()));
    logMessage("正在进行固件升级，请勿断电或进行操作!!!");
}

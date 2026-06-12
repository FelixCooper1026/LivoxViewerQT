#include "LivoxViewerWindow.h"
#include "widgets/SwitchCheckBox.h"

#include <QAbstractItemView>
#include <QAbstractSocket>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFont>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHostAddress>
#include <QIODevice>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMessageBox>
#include <QNetworkInterface>
#include <QPushButton>
#include <QSizePolicy>
#include <QSpinBox>
#include <QStackedWidget>
#include <QStandardPaths>
#include <QVBoxLayout>
#include <QVector>

namespace {

struct DeviceRow {
    QWidget* detailPage = nullptr;
    QWidget* listWidget = nullptr;
    QListWidgetItem* listItem = nullptr;
    QLabel* listDotLabel = nullptr;
    QLabel* listNameLabel = nullptr;
    QLabel* listTypeLabel = nullptr;
    QLineEdit* nameEdit = nullptr;
    QComboBox* devType = nullptr;
    QComboBox* hostIp = nullptr;
    QLineEdit* mcIp = nullptr;
    QLineEdit* fixedCmdPort = nullptr;
    QLineEdit* fixedPushPort = nullptr;
    QLineEdit* fixedPointPort = nullptr;
    QLineEdit* fixedImuPort = nullptr;
    QLineEdit* fixedLogPort = nullptr;
    QSpinBox* hostCmdPort = nullptr;
    QSpinBox* hostPushPort = nullptr;
    QSpinBox* hostPointPort = nullptr;
    QSpinBox* hostImuPort = nullptr;
    QSpinBox* hostLogPort = nullptr;
    bool autoName = true;
};

QLabel* createConfigGeneratorFieldLabel(const QString& text, QWidget* parent)
{
    QLabel* label = new QLabel(text, parent);
    label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    label->setStyleSheet("color: palette(window-text);");
    return label;
}

QLabel* createConfigGeneratorSectionTitle(const QString& text, QWidget* parent)
{
    QLabel* titleLabel = new QLabel(text, parent);
    titleLabel->setObjectName("ConfigGeneratorSectionTitle");
    QFont titleFont = titleLabel->font();
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);
    return titleLabel;
}

QFrame* createConfigGeneratorSection(QWidget* parent)
{
    QFrame* section = new QFrame(parent);
    section->setObjectName("ConfigGeneratorSection");
    section->setFrameShape(QFrame::StyledPanel);
    section->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    return section;
}

QLineEdit* createFixedPortEdit(QWidget* parent)
{
    QLineEdit* edit = new QLineEdit(parent);
    edit->setEnabled(false);
    edit->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    edit->setMinimumWidth(150);
    edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    return edit;
}

QSpinBox* createHostPortSpin(QWidget* parent)
{
    QSpinBox* spin = new QSpinBox(parent);
    spin->setRange(0, 65535);
    spin->setMinimumWidth(150);
    spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    return spin;
}

QString deviceTypeDisplayName(const QString& key)
{
    if (key == "MID360") return QStringLiteral("Mid-360");
    if (key == "Mid360s") return QStringLiteral("Mid-360s");
    if (key == "Mid360l") return QStringLiteral("Mid-360l");
    if (key == "Avia2") return QStringLiteral("Avia-2");
    return QStringLiteral("HAP");
}

QString deviceTypeKey(const DeviceRow* row)
{
    const QString key = row->devType->currentData().toString();
    return key.isEmpty() ? row->devType->currentText() : key;
}

void addDeviceTypeItems(QComboBox* combo)
{
    combo->addItem("Mid-360", "MID360");
    combo->addItem("Mid-360s", "Mid360s");
    combo->addItem("Mid-360l", "Mid360l");
    combo->addItem("Avia-2", "Avia2");
    combo->addItem("HAP", "HAP");
}

void setFixedPortText(QLineEdit* edit, int value)
{
    edit->setText(QString::number(value));
}

} // namespace

bool LivoxViewerWindow::showConfigGeneratorDialog()
{
    QDialog dlg(this);
    dlg.setWindowTitle("生成配置文件");
    dlg.resize(1000, 760);
    dlg.setMinimumSize(860, 640);
    dlg.setStyleSheet(
        "QFrame#ConfigGeneratorSection {"
        "  border: 1px solid palette(mid);"
        "  border-radius: 6px;"
        "  background: palette(base);"
        "}"
        "QLabel#ConfigGeneratorSectionTitle {"
        "  color: palette(window-text);"
        "}"
        "QPushButton#ConfigGeneratorAddButton, QPushButton#ConfigGeneratorRemoveButton, QPushButton#ConfigGeneratorSecondaryButton {"
        "  border: 1px solid palette(mid);"
        "  border-radius: 5px;"
        "  padding: 6px 16px;"
        "  background: palette(button);"
        "  color: palette(button-text);"
        "}"
        "QPushButton#ConfigGeneratorAddButton:hover, QPushButton#ConfigGeneratorRemoveButton:hover, QPushButton#ConfigGeneratorSecondaryButton:hover {"
        "  background: palette(alternate-base);"
        "}"
        "QPushButton#ConfigGeneratorAddButton:pressed, QPushButton#ConfigGeneratorRemoveButton:pressed, QPushButton#ConfigGeneratorSecondaryButton:pressed {"
        "  background: palette(mid);"
        "}"
        "QPushButton#ConfigGeneratorRemoveButton:disabled {"
        "  background: palette(window);"
        "  color: palette(mid);"
        "  border-color: palette(mid);"
        "}"
        "QPushButton#ConfigGeneratorPrimaryButton {"
        "  border: 1px solid palette(highlight);"
        "  border-radius: 5px;"
        "  padding: 7px 22px;"
        "  background: palette(highlight);"
        "  color: palette(highlighted-text);"
        "}"
        "QPushButton#ConfigGeneratorPrimaryButton:pressed {"
        "  background: palette(dark);"
        "}"
        "QComboBox, QSpinBox, QLineEdit {"
        "  min-height: 28px;"
        "}"
        "QListWidget#ConfigGeneratorDeviceList {"
        "  border: none;"
        "  background: palette(base);"
        "  outline: none;"
        "}"
        "QListWidget#ConfigGeneratorDeviceList::item {"
        "  border-bottom: 1px solid palette(mid);"
        "}"
        "QListWidget#ConfigGeneratorDeviceList::item:selected {"
        "  background: palette(alternate-base);"
        "}"
        "QWidget#ConfigGeneratorDeviceListItem {"
        "  background: transparent;"
        "}"
    );

    QVBoxLayout* mainLayout = new QVBoxLayout(&dlg);
    mainLayout->setContentsMargins(10, 10, 10, 10);
    mainLayout->setSpacing(10);

    QFrame* logSection = createConfigGeneratorSection(&dlg);
    QVBoxLayout* logLayout = new QVBoxLayout(logSection);
    logLayout->setContentsMargins(14, 10, 14, 12);
    logLayout->setSpacing(10);
    logLayout->addWidget(createConfigGeneratorSectionTitle("日志配置", logSection));

    QWidget* logTopRow = new QWidget(logSection);
    QHBoxLayout* logTopLayout = new QHBoxLayout(logTopRow);
    logTopLayout->setContentsMargins(0, 0, 0, 0);
    logTopLayout->setSpacing(10);

    QLabel* lblLogEnable = createConfigGeneratorFieldLabel("启用 SDK 日志", logTopRow);
    QCheckBox* cbLogEnable = new SwitchCheckBox(logTopRow);
    cbLogEnable->setChecked(true);
    logTopLayout->addWidget(lblLogEnable);
    logTopLayout->addWidget(cbLogEnable);
    logTopLayout->addStretch();

    QLabel* lblCache = createConfigGeneratorFieldLabel("缓存大小", logTopRow);
    QSpinBox* spinCache = new QSpinBox(logTopRow);
    spinCache->setRange(0, 100000);
    spinCache->setValue(500);
    spinCache->setMinimumWidth(90);
    QLabel* cacheUnitLabel = createConfigGeneratorFieldLabel("MB", logTopRow);
    logTopLayout->addWidget(lblCache);
    logTopLayout->addWidget(spinCache);
    logTopLayout->addWidget(cacheUnitLabel);
    logTopLayout->addStretch();
    logLayout->addWidget(logTopRow);

    QWidget* logPathRow = new QWidget(logSection);
    QHBoxLayout* logPathLayout = new QHBoxLayout(logPathRow);
    logPathLayout->setContentsMargins(0, 0, 0, 0);
    logPathLayout->setSpacing(10);
    QLabel* lblPath = createConfigGeneratorFieldLabel("日志路径", logPathRow);
    QLineEdit* editPath = new QLineEdit(logPathRow);
    editPath->setText("./logs");
    QPushButton* browseButton = new QPushButton("浏览...", logPathRow);
    browseButton->setObjectName("ConfigGeneratorSecondaryButton");
    logPathLayout->addWidget(lblPath);
    logPathLayout->addWidget(editPath, 1);
    logPathLayout->addWidget(browseButton);
    logLayout->addWidget(logPathRow);
    mainLayout->addWidget(logSection);

    QObject::connect(browseButton, &QPushButton::clicked, &dlg, [&]() {
        const QString startDir = editPath->text().trimmed().isEmpty() ? QDir::currentPath() : editPath->text().trimmed();
        const QString selectedDir = QFileDialog::getExistingDirectory(&dlg, "选择日志路径", startDir);
        if (!selectedDir.isEmpty()) {
            editPath->setText(QDir::toNativeSeparators(selectedDir));
        }
    });

    struct DeviceListUi {
        QListWidget* list = nullptr;
        QLabel* countLabel = nullptr;
        QStackedWidget* detailStack = nullptr;
        QPushButton* removeButton = nullptr;
    } deviceListUi;
    QVector<DeviceRow*> deviceRows;

    auto populateHostIpsTo = [&](QComboBox* combo) {
        combo->clear();
        for (const QNetworkInterface& iface : QNetworkInterface::allInterfaces()) {
            if (!(iface.flags() & QNetworkInterface::IsUp) || !(iface.flags() & QNetworkInterface::IsRunning)) continue;
            if (iface.flags() & QNetworkInterface::IsLoopBack) continue;
            for (const QNetworkAddressEntry& entry : iface.addressEntries()) {
                const QHostAddress& addr = entry.ip();
                if (addr.protocol() != QAbstractSocket::IPv4Protocol) continue;
                const QString ip = addr.toString();
                if (ip == "0.0.0.0" || ip.startsWith("169.254.")) continue;
                const QString label = QString("%1  -  %2 (%3)").arg(ip, iface.humanReadableName(), iface.name());
                combo->addItem(label, ip);
            }
        }
    };

    auto createLidarNetDefaults = [&](const QString& type, QJsonObject& out) {
        if (type == "MID360" || type == "Mid360s" || type == "Mid360l" || type == "Avia2") {
            out.insert("cmd_data_port", 56100);
            out.insert("push_msg_port", 56200);
            out.insert("point_data_port", 56300);
            out.insert("imu_data_port", 56400);
            out.insert("log_data_port", 56500);
        } else {
            out.insert("cmd_data_port", 56000);
            out.insert("push_msg_port", 0);
            out.insert("point_data_port", 57000);
            out.insert("imu_data_port", 58000);
            out.insert("log_data_port", 59000);
        }
    };

    auto applyFixedPortDefaults = [&](DeviceRow* row, const QString& type) {
        QJsonObject lidarNet;
        createLidarNetDefaults(type, lidarNet);
        setFixedPortText(row->fixedCmdPort, lidarNet.value("cmd_data_port").toInt());
        setFixedPortText(row->fixedPushPort, lidarNet.value("push_msg_port").toInt());
        setFixedPortText(row->fixedPointPort, lidarNet.value("point_data_port").toInt());
        setFixedPortText(row->fixedImuPort, lidarNet.value("imu_data_port").toInt());
        setFixedPortText(row->fixedLogPort, lidarNet.value("log_data_port").toInt());
    };

    auto applyHostPortDefaults = [&](const QString& type, DeviceRow* row) {
        if (type == "MID360" || type == "Mid360s" || type == "Mid360l" || type == "Avia2") {
            row->hostCmdPort->setValue(56101);
            row->hostPushPort->setValue(56201);
            row->hostPointPort->setValue(56301);
            row->hostImuPort->setValue(56401);
            row->hostLogPort->setValue(56501);
        } else {
            row->hostCmdPort->setValue(56000);
            row->hostPushPort->setValue(0);
            row->hostPointPort->setValue(57000);
            row->hostImuPort->setValue(58000);
            row->hostLogPort->setValue(59000);
        }
    };

    auto updateDeviceListItem = [&](DeviceRow* row) {
        const bool selected = deviceListUi.list && deviceListUi.list->currentItem() == row->listItem;
        const QString nameText = row->nameEdit->text().trimmed().isEmpty()
            ? QStringLiteral("设备")
            : row->nameEdit->text().trimmed();
        const QString typeText = deviceTypeDisplayName(deviceTypeKey(row));
        row->listNameLabel->setText(nameText);
        row->listTypeLabel->setText(typeText);
        row->listDotLabel->setStyleSheet(selected ? "color: palette(highlight);" : "color: palette(mid);");
        row->listWidget->setStyleSheet(selected
            ? "QWidget#ConfigGeneratorDeviceListItem { background: palette(alternate-base); }"
            : "QWidget#ConfigGeneratorDeviceListItem { background: transparent; }");
    };

    auto refreshDeviceList = [&]() {
        for (DeviceRow* row : deviceRows) {
            updateDeviceListItem(row);
        }
        deviceListUi.countLabel->setText(QString("共 %1 台设备").arg(deviceRows.size()));
        deviceListUi.removeButton->setEnabled(deviceRows.size() > 1);
    };

    auto renumberAutoDeviceNames = [&]() {
        for (int i = 0; i < deviceRows.size(); ++i) {
            DeviceRow* row = deviceRows.at(i);
            if (row->autoName) {
                row->nameEdit->setText(QString("设备 %1").arg(i + 1));
            }
        }
    };

    QHBoxLayout* deviceAreaLayout = new QHBoxLayout();
    deviceAreaLayout->setContentsMargins(0, 0, 0, 0);
    deviceAreaLayout->setSpacing(10);

    QFrame* deviceListPanel = createConfigGeneratorSection(&dlg);
    deviceListPanel->setFixedWidth(290);
    QVBoxLayout* deviceListLayout = new QVBoxLayout(deviceListPanel);
    deviceListLayout->setContentsMargins(0, 0, 0, 0);
    deviceListLayout->setSpacing(0);

    QWidget* deviceListHeader = new QWidget(deviceListPanel);
    QHBoxLayout* deviceListHeaderLayout = new QHBoxLayout(deviceListHeader);
    deviceListHeaderLayout->setContentsMargins(14, 10, 14, 10);
    deviceListHeaderLayout->setSpacing(8);
    deviceListHeaderLayout->addWidget(createConfigGeneratorSectionTitle("设备列表", deviceListHeader));
    deviceListHeaderLayout->addStretch();
    QPushButton* btnAddDevice = new QPushButton("添加", deviceListHeader);
    btnAddDevice->setObjectName("ConfigGeneratorAddButton");
    QPushButton* btnRemoveDevice = new QPushButton("删除", deviceListHeader);
    btnRemoveDevice->setObjectName("ConfigGeneratorRemoveButton");
    deviceListUi.removeButton = btnRemoveDevice;
    deviceListHeaderLayout->addWidget(btnAddDevice);
    deviceListHeaderLayout->addWidget(btnRemoveDevice);
    deviceListLayout->addWidget(deviceListHeader);

    QListWidget* deviceList = new QListWidget(deviceListPanel);
    deviceList->setObjectName("ConfigGeneratorDeviceList");
    deviceList->setSelectionMode(QAbstractItemView::SingleSelection);
    deviceList->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    deviceList->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    deviceListUi.list = deviceList;
    deviceListLayout->addWidget(deviceList, 1);

    QLabel* deviceCountLabel = new QLabel(deviceListPanel);
    deviceCountLabel->setContentsMargins(14, 8, 14, 8);
    deviceCountLabel->setStyleSheet("border-top: 1px solid palette(mid); color: palette(window-text);");
    deviceListUi.countLabel = deviceCountLabel;
    deviceListLayout->addWidget(deviceCountLabel);
    deviceAreaLayout->addWidget(deviceListPanel);

    QFrame* detailPanel = createConfigGeneratorSection(&dlg);
    QVBoxLayout* detailLayout = new QVBoxLayout(detailPanel);
    detailLayout->setContentsMargins(20, 14, 20, 14);
    detailLayout->setSpacing(12);
    detailLayout->addWidget(createConfigGeneratorSectionTitle("设备详情", detailPanel));

    QStackedWidget* detailStack = new QStackedWidget(detailPanel);
    deviceListUi.detailStack = detailStack;
    detailLayout->addWidget(detailStack, 1);
    deviceAreaLayout->addWidget(detailPanel, 1);
    mainLayout->addLayout(deviceAreaLayout, 1);

    auto addDeviceRow = [&]() {
        DeviceRow* row = new DeviceRow();
        row->detailPage = new QWidget(detailStack);
        QVBoxLayout* detailPageLayout = new QVBoxLayout(row->detailPage);
        detailPageLayout->setContentsMargins(0, 0, 0, 0);
        detailPageLayout->setSpacing(12);

        QGridLayout* formGrid = new QGridLayout();
        formGrid->setContentsMargins(0, 0, 0, 0);
        formGrid->setHorizontalSpacing(12);
        formGrid->setVerticalSpacing(8);
        formGrid->setColumnMinimumWidth(0, 88);

        row->nameEdit = new QLineEdit(QString("设备 %1").arg(deviceRows.size() + 1), row->detailPage);
        row->devType = new QComboBox(row->detailPage);
        addDeviceTypeItems(row->devType);
        row->hostIp = new QComboBox(row->detailPage);
        populateHostIpsTo(row->hostIp);
        row->mcIp = new QLineEdit(row->detailPage);
        row->mcIp->setPlaceholderText("可选，例如 224.1.1.5，留空表示不启用");

        int formRow = 0;
        formGrid->addWidget(createConfigGeneratorFieldLabel("设备名称", row->detailPage), formRow, 0);
        formGrid->addWidget(row->nameEdit, formRow, 1);
        ++formRow;
        formGrid->addWidget(createConfigGeneratorFieldLabel("设备类型", row->detailPage), formRow, 0);
        formGrid->addWidget(row->devType, formRow, 1);
        ++formRow;
        formGrid->addWidget(createConfigGeneratorFieldLabel("主机 IP", row->detailPage), formRow, 0);
        formGrid->addWidget(row->hostIp, formRow, 1);
        ++formRow;
        formGrid->addWidget(createConfigGeneratorFieldLabel("组播 IP", row->detailPage), formRow, 0);
        formGrid->addWidget(row->mcIp, formRow, 1);
        detailPageLayout->addLayout(formGrid);

        QLabel* portTitle = createConfigGeneratorSectionTitle("端口映射配置", row->detailPage);
        detailPageLayout->addWidget(portTitle);

        QGridLayout* portGrid = new QGridLayout();
        portGrid->setContentsMargins(0, 0, 0, 0);
        portGrid->setHorizontalSpacing(16);
        portGrid->setVerticalSpacing(8);
        portGrid->setColumnMinimumWidth(0, 88);
        portGrid->setColumnStretch(1, 1);
        portGrid->setColumnStretch(2, 1);
        portGrid->addWidget(createConfigGeneratorFieldLabel("端口类型", row->detailPage), 0, 0);
        portGrid->addWidget(createConfigGeneratorFieldLabel("设备固定端口", row->detailPage), 0, 1);
        portGrid->addWidget(createConfigGeneratorFieldLabel("主机目标端口", row->detailPage), 0, 2);

        row->fixedCmdPort = createFixedPortEdit(row->detailPage);
        row->fixedPushPort = createFixedPortEdit(row->detailPage);
        row->fixedPointPort = createFixedPortEdit(row->detailPage);
        row->fixedImuPort = createFixedPortEdit(row->detailPage);
        row->fixedLogPort = createFixedPortEdit(row->detailPage);
        row->hostCmdPort = createHostPortSpin(row->detailPage);
        row->hostPushPort = createHostPortSpin(row->detailPage);
        row->hostPointPort = createHostPortSpin(row->detailPage);
        row->hostImuPort = createHostPortSpin(row->detailPage);
        row->hostLogPort = createHostPortSpin(row->detailPage);

        auto addPortRow = [&](int tableRow, const QString& label, QLineEdit* fixedPort, QSpinBox* hostPort) {
            portGrid->addWidget(createConfigGeneratorFieldLabel(label, row->detailPage), tableRow, 0);
            portGrid->addWidget(fixedPort, tableRow, 1);
            portGrid->addWidget(hostPort, tableRow, 2);
        };
        addPortRow(1, "命令端口", row->fixedCmdPort, row->hostCmdPort);
        addPortRow(2, "推送端口", row->fixedPushPort, row->hostPushPort);
        addPortRow(3, "点云端口", row->fixedPointPort, row->hostPointPort);
        addPortRow(4, "IMU端口", row->fixedImuPort, row->hostImuPort);
        addPortRow(5, "日志端口", row->fixedLogPort, row->hostLogPort);
        detailPageLayout->addLayout(portGrid);

        QWidget* portFooter = new QWidget(row->detailPage);
        QHBoxLayout* portFooterLayout = new QHBoxLayout(portFooter);
        portFooterLayout->setContentsMargins(0, 8, 0, 0);
        QPushButton* resetPortsButton = new QPushButton("恢复默认目标端口", portFooter);
        resetPortsButton->setObjectName("ConfigGeneratorSecondaryButton");
        QLabel* noteLabel = new QLabel("说明：左侧为设备固定端口，不可修改；右侧为本机接收端口。", portFooter);
        noteLabel->setStyleSheet("color: palette(window-text);");
        portFooterLayout->addWidget(resetPortsButton);
        portFooterLayout->addStretch();
        portFooterLayout->addWidget(noteLabel);
        detailPageLayout->addWidget(portFooter);
        detailPageLayout->addStretch();

        applyFixedPortDefaults(row, deviceTypeKey(row));
        applyHostPortDefaults(deviceTypeKey(row), row);

        QWidget* itemWidget = new QWidget(deviceList);
        itemWidget->setObjectName("ConfigGeneratorDeviceListItem");
        itemWidget->setAttribute(Qt::WA_TransparentForMouseEvents);
        QHBoxLayout* itemLayout = new QHBoxLayout(itemWidget);
        itemLayout->setContentsMargins(14, 8, 14, 8);
        itemLayout->setSpacing(10);
        row->listDotLabel = new QLabel("●", itemWidget);
        row->listDotLabel->setFixedWidth(14);
        itemLayout->addWidget(row->listDotLabel, 0, Qt::AlignTop);
        QVBoxLayout* itemTextLayout = new QVBoxLayout();
        itemTextLayout->setContentsMargins(0, 0, 0, 0);
        itemTextLayout->setSpacing(2);
        row->listNameLabel = new QLabel(itemWidget);
        row->listTypeLabel = new QLabel(itemWidget);
        row->listTypeLabel->setStyleSheet("color: palette(mid);");
        itemTextLayout->addWidget(row->listNameLabel);
        itemTextLayout->addWidget(row->listTypeLabel);
        itemLayout->addLayout(itemTextLayout, 1);
        row->listWidget = itemWidget;

        row->listItem = new QListWidgetItem();
        row->listItem->setSizeHint(QSize(0, 64));
        deviceList->addItem(row->listItem);
        deviceList->setItemWidget(row->listItem, itemWidget);
        detailStack->addWidget(row->detailPage);
        deviceRows.append(row);

        QObject::connect(row->nameEdit, &QLineEdit::textChanged, &dlg, [&, row]() {
            updateDeviceListItem(row);
        });
        QObject::connect(row->nameEdit, &QLineEdit::textEdited, &dlg, [row]() {
            row->autoName = false;
        });
        QObject::connect(row->devType, QOverload<int>::of(&QComboBox::currentIndexChanged), &dlg, [&, row](int) {
            const QString type = deviceTypeKey(row);
            applyFixedPortDefaults(row, type);
            applyHostPortDefaults(type, row);
            updateDeviceListItem(row);
        });
        QObject::connect(resetPortsButton, &QPushButton::clicked, &dlg, [&, row]() {
            applyHostPortDefaults(deviceTypeKey(row), row);
        });

        deviceList->setCurrentItem(row->listItem);
        renumberAutoDeviceNames();
        refreshDeviceList();
    };

    QObject::connect(deviceList, &QListWidget::currentRowChanged, &dlg, [&](int row) {
        if (row >= 0) {
            detailStack->setCurrentIndex(row);
        }
        refreshDeviceList();
    });

    QObject::connect(btnAddDevice, &QPushButton::clicked, &dlg, addDeviceRow);
    QObject::connect(btnRemoveDevice, &QPushButton::clicked, &dlg, [&]() {
        if (deviceRows.size() <= 1) {
            return;
        }
        const int rowIndex = deviceList->currentRow();
        if (rowIndex < 0 || rowIndex >= deviceRows.size()) {
            return;
        }
        DeviceRow* row = deviceRows.takeAt(rowIndex);
        QWidget* itemWidget = deviceList->itemWidget(row->listItem);
        deviceList->removeItemWidget(row->listItem);
        delete deviceList->takeItem(rowIndex);
        if (itemWidget) {
            itemWidget->deleteLater();
        }
        detailStack->removeWidget(row->detailPage);
        row->detailPage->deleteLater();
        delete row;
        renumberAutoDeviceNames();
        const int newRow = qMin(rowIndex, deviceRows.size() - 1);
        deviceList->setCurrentRow(newRow);
        refreshDeviceList();
    });

    addDeviceRow();

    QWidget* actionRow = new QWidget(&dlg);
    QHBoxLayout* actionLayout = new QHBoxLayout(actionRow);
    actionLayout->setContentsMargins(0, 8, 0, 0);
    actionLayout->addStretch();
    QPushButton* cancelButton = new QPushButton("取消", actionRow);
    cancelButton->setObjectName("ConfigGeneratorSecondaryButton");
    QPushButton* generateButton = new QPushButton("生成配置文件", actionRow);
    generateButton->setObjectName("ConfigGeneratorPrimaryButton");
    actionLayout->addWidget(cancelButton);
    actionLayout->addWidget(generateButton);
    mainLayout->addWidget(actionRow);
    QObject::connect(cancelButton, &QPushButton::clicked, &dlg, &QDialog::reject);
    QObject::connect(generateButton, &QPushButton::clicked, &dlg, &QDialog::accept);

    if (dlg.exec() != QDialog::Accepted) return false;

    if (deviceRows.isEmpty()) {
        QMessageBox::warning(this, "生成配置文件", "请至少添加一台设备");
        return false;
    }
    for (DeviceRow* row : deviceRows) {
        if (!row || row->hostIp->count() == 0) {
            QMessageBox::warning(this, "生成配置文件", "未检测到主机网口IPv4地址，请检查网络连接");
            return false;
        }
    }

    const QString logPath = editPath->text().trimmed().isEmpty()
        ? QStringLiteral("./logs")
        : editPath->text().trimmed();
    QJsonObject root;
    root.insert("lidar_log_enable", cbLogEnable->isChecked());
    root.insert("lidar_log_cache_size_MB", spinCache->value());
    root.insert("lidar_log_path", logPath);

    QJsonArray hostArrMID360;
    QJsonArray hostArrMid360s;
    QJsonArray hostArrMid360l;
    QJsonArray hostArrAvia2;
    QJsonArray hostArrHap;
    for (DeviceRow* row : deviceRows) {
        const QString type = deviceTypeKey(row);
        QJsonObject hostObj;
        hostObj.insert("host_ip", row->hostIp->currentData().toString());
        if (!row->mcIp->text().trimmed().isEmpty()) hostObj.insert("multicast_ip", row->mcIp->text().trimmed());
        hostObj.insert("cmd_data_port", row->hostCmdPort->value());
        hostObj.insert("point_data_port", row->hostPointPort->value());
        hostObj.insert("imu_data_port", row->hostImuPort->value());
        hostObj.insert("push_msg_port", row->hostPushPort->value());
        hostObj.insert("log_data_port", row->hostLogPort->value());

        if (type == "MID360") hostArrMID360.append(hostObj);
        else if (type == "Mid360s") hostArrMid360s.append(hostObj);
        else if (type == "Mid360l") hostArrMid360l.append(hostObj);
        else if (type == "Avia2") hostArrAvia2.append(hostObj);
        else hostArrHap.append(hostObj);
    }

    if (!hostArrMID360.isEmpty()) {
        QJsonObject devObj;
        QJsonObject lidarNet; createLidarNetDefaults("MID360", lidarNet);
        devObj.insert("lidar_net_info", lidarNet);
        devObj.insert("host_net_info", hostArrMID360);
        root.insert("MID360", devObj);
    }
    if (!hostArrMid360s.isEmpty()) {
        QJsonObject devObj;
        QJsonObject lidarNet; createLidarNetDefaults("Mid360s", lidarNet);
        devObj.insert("lidar_net_info", lidarNet);
        devObj.insert("host_net_info", hostArrMid360s);
        root.insert("Mid360s", devObj);
    }
    if (!hostArrMid360l.isEmpty()) {
        QJsonObject devObj;
        QJsonObject lidarNet; createLidarNetDefaults("Mid360l", lidarNet);
        devObj.insert("lidar_net_info", lidarNet);
        devObj.insert("host_net_info", hostArrMid360l);
        root.insert("Mid360l", devObj);
    }
    if (!hostArrAvia2.isEmpty()) {
        QJsonObject devObj;
        QJsonObject lidarNet; createLidarNetDefaults("Avia2", lidarNet);
        devObj.insert("lidar_net_info", lidarNet);
        devObj.insert("host_net_info", hostArrAvia2);
        root.insert("Avia2", devObj);
    }
    if (!hostArrHap.isEmpty()) {
        QJsonObject devObj;
        QJsonObject lidarNet; createLidarNetDefaults("HAP", lidarNet);
        devObj.insert("lidar_net_info", lidarNet);
        devObj.insert("host_net_info", hostArrHap);
        root.insert("HAP", devObj);
    }

    QString outPath;
    {
        QString cfgDir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
        if (cfgDir.isEmpty()) {
            cfgDir = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
        }
        if (!cfgDir.isEmpty()) {
            QDir().mkpath(cfgDir);
            outPath = QDir(cfgDir).filePath("config.json");
        } else {
            outPath = QApplication::applicationDirPath() + "/config.json";
        }
    }
    QFile f(outPath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        QMessageBox::critical(this, "生成配置文件", QString("无法写入: %1").arg(QDir::toNativeSeparators(outPath)));
        return false;
    }

    QJsonObject orderedRoot;
    orderedRoot.insert("lidar_log_enable", cbLogEnable->isChecked());
    orderedRoot.insert("lidar_log_cache_size_MB", spinCache->value());
    orderedRoot.insert("lidar_log_path", logPath);

    auto makeDeviceObject = [&](const QString& type, const QJsonArray& hostArr) {
        QJsonObject devObj;
        QJsonObject lidarNet;
        createLidarNetDefaults(type, lidarNet);
        devObj.insert("lidar_net_info", lidarNet);
        devObj.insert("host_net_info", hostArr);
        return devObj;
    };

    if (!hostArrMid360s.isEmpty()) {
        orderedRoot.insert("Mid360s", makeDeviceObject("Mid360s", hostArrMid360s));
    }
    if (!hostArrMID360.isEmpty()) {
        orderedRoot.insert("MID360", makeDeviceObject("MID360", hostArrMID360));
    }
    if (!hostArrMid360l.isEmpty()) {
        orderedRoot.insert("Mid360l", makeDeviceObject("Mid360l", hostArrMid360l));
    }
    if (!hostArrAvia2.isEmpty()) {
        orderedRoot.insert("Avia2", makeDeviceObject("Avia2", hostArrAvia2));
    }
    if (!hostArrHap.isEmpty()) {
        orderedRoot.insert("HAP", makeDeviceObject("HAP", hostArrHap));
    }

    QJsonDocument doc(orderedRoot);
    f.write(doc.toJson(QJsonDocument::Indented));
    f.close();

    QMessageBox::information(this, "生成配置文件", QString("已生成: %1").arg(QDir::toNativeSeparators(outPath)));
    return true;
}

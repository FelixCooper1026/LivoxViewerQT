#include "LivoxViewerWindow.h"
#include <QApplication>
#include <QSplitter>
#include <QScrollArea>
#include <QColorDialog>
#include <QFrame>
#include <QVariant>
#include <QHeaderView>
#include <QInputDialog>
#include <QFileDialog>
#include <QFileInfo>
#include <QDialogButtonBox>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QNetworkInterface>
#include <QHostAddress>
#include <QAbstractSocket>
#include <QListWidget>
#include <QDesktopServices>
#include <QStandardPaths>
#include <QRadioButton>
#include <QSignalBlocker>

#include <algorithm>

void LivoxViewerWindow::initializeUserInterface()
{
    // 设置应用程序字体，避免DirectWrite错误
    QFont appFont = QApplication::font();
#ifdef Q_OS_WIN
    appFont.setFamily("Microsoft YaHei"); // Windows: DirectWrite 字体兼容
    appFont.setPointSize(9);
    QApplication::setFont(appFont);
#endif

    // 中央视图：点云可视化
    QWidget* centralContainer = new QWidget(this);
    QVBoxLayout* centralLayout = new QVBoxLayout(centralContainer);
    centralLayout->setContentsMargins(0,0,0,0);
    centralLayout->setSpacing(0);

    // 顶部可视化功能栏（两行）
    QWidget* viewerToolbar = createViewerToolbar(centralContainer);

    pointCloudView = new PointCloudView(centralContainer);
    pointCloudView->setMinimumSize(200, 200);
    pointCloudView->setPointSize(pointSizePx);
    connect(pointCloudView, &PointCloudView::lvx2FileDropped, this, &LivoxViewerWindow::onLvx2PlaybackFileDropped);

    createPlaybackBar(centralContainer);

    centralLayout->addWidget(viewerToolbar);
    centralLayout->addWidget(playbackState.bar);
    centralLayout->addWidget(pointCloudView, 1);
    setCentralWidget(centralContainer);

    // 初始化深度投影控件状态（仅在球坐标时可用）
    if (projectionDepthCheck) {
        projectionDepthCheck->setEnabled(false);  // 默认禁用，直到选择球坐标
    }
    if (projectionDepthSpin) {
        projectionDepthSpin->setEnabled(false);  // 默认禁用，直到选择球坐标
    }

    // 初始化平面投影控件状态（仅在球坐标时可用）
    if (planarProjectionCheck) {
        planarProjectionCheck->setEnabled(false);  // 默认禁用，直到选择球坐标
    }
    if (planarRadiusSpin) {
        planarRadiusSpin->setEnabled(false);  // 默认禁用，直到选择球坐标
    }

    createDevicePanel();
    createParameterPanel();
    createImuPanel();
    createFileInfoPanel();
    createLogPanel();

    // 初始布局尺寸（近似 CloudCompare）：左侧窄、右侧中、底部适中
    resizeDocks({lidarDevicesDock}, {240}, Qt::Horizontal);
    resizeDocks({paramsDock}, {360}, Qt::Horizontal);
    resizeDocks({logDock}, {240}, Qt::Vertical);

    createMenusAndActions();
}


// 刷新按钮已移除，无需实现 onRefreshClicked

void LivoxViewerWindow::onLidarDeviceSelected()
{
    if (shutting_down) {
        return;
    }

    int currentRow = lidarDeviceList->currentRow();
    if (currentRow >= 0 && currentRow < lidarDevices.size()) {
        QMutexLocker locker(&lidarDeviceMutex);
        auto it = lidarDevices.begin();
        std::advance(it, currentRow);
        setCurrentDeviceHandle(it.key());
        if (it.value().is_connected) {
            if (statusLabel) statusLabel->setText("状态: 已连接");
        } else {
            if (statusLabel) statusLabel->setText("状态: 未连接");
        }
    }
    // 切换设备后，立即触发当前标签页的刷新逻辑
    // 这会清空 parameterState.updatedConfigKeys 并发送 QueryInternalInfo 命令，从而刷新所有 Tab 的参数显示
    if (paramTabWidget) {
        onTabChanged(paramTabWidget->currentIndex());
    }
}

void LivoxViewerWindow::updateLidarDeviceList()
{
    QList<LidarDeviceInfo> devices;
    {
        QMutexLocker locker(&lidarDeviceMutex);
        devices = lidarDevices.values();
    }

    const QSignalBlocker blocker(lidarDeviceList);
    lidarDeviceList->clear();
    for (const auto& device : devices) {
        addLidarDeviceToList(device);
    }
}

void LivoxViewerWindow::addLidarDeviceToList(const LidarDeviceInfo& device)
{
    QString deviceText = QString("%1 (%2) - %3").arg(device.sn).arg(device.product_info).arg(device.is_streaming ? "数据流中" : "已连接");
    QListWidgetItem* item = new QListWidgetItem(deviceText);
    item->setData(Qt::UserRole, device.handle);
    lidarDeviceList->addItem(item);
}

void LivoxViewerWindow::updateLidarDeviceInfo(const LidarDeviceInfo& device)
{
    QMutexLocker locker(&lidarDeviceMutex);
    lidarDevices[device.handle] = device;
    for (int i = 0; i < lidarDeviceList->count(); ++i) {
        QListWidgetItem* item = lidarDeviceList->item(i);
        if (item->data(Qt::UserRole).toUInt() == device.handle) {
            QString deviceText = QString("%1 (%2) - %3").arg(device.sn).arg(device.product_info).arg(device.is_streaming ? "数据流中" : "已连接");
            item->setText(deviceText);
            break;
        }
    }
}


void LivoxViewerWindow::onTabChanged(int index)
{
    LidarDeviceInfo currentDevice;
    if (!tryGetCurrentDevice(currentDevice) ||
        !currentDevice.is_connected ||
        !currentDevice.parameter_query_ready) {
        return;
    }

    // 清除旧的配置标记，准备接受新一轮查询结果
    parameterState.updatedConfigKeys.clear();
    livox_status status = QueryLivoxLidarInternalInfo(currentDevice.handle, onQueryInternalInfoResponse, this);
    if (status != kLivoxLidarStatusSuccess) {
        logMessage(QString("切换至标签页[%1]时查询设备信息失败: %2 (错误码: %3)")
                     .arg(index)
                     .arg(getLivoxStatusString(status))
                     .arg(static_cast<int>(status)));
    }
}

bool LivoxViewerWindow::showConfigGeneratorDialog()
{
	QDialog dlg(this);
	dlg.setWindowTitle("生成配置文件");
	QVBoxLayout* v = new QVBoxLayout(&dlg);

	// 日志配置
	QGroupBox* logGroup = new QGroupBox("日志配置", &dlg);
	QVBoxLayout* logLayout = new QVBoxLayout(logGroup);
	QCheckBox* cbLogEnable = new QCheckBox("启用雷达日志", logGroup);
	cbLogEnable->setChecked(true);
	QWidget* rowCache = new QWidget(logGroup);
	QHBoxLayout* hCache = new QHBoxLayout(rowCache);
	hCache->setContentsMargins(0,0,0,0);
	QLabel* lblCache = new QLabel("缓存大小(MB):", rowCache);
	QSpinBox* spinCache = new QSpinBox(rowCache);
	spinCache->setRange(0, 100000);
	spinCache->setValue(500);
	hCache->addWidget(lblCache);
	hCache->addSpacing(8);
	hCache->addWidget(spinCache);
	QWidget* rowPath = new QWidget(logGroup);
	QHBoxLayout* hPath = new QHBoxLayout(rowPath);
	hPath->setContentsMargins(0,0,0,0);
	QLabel* lblPath = new QLabel("日志路径:", rowPath);
	QLineEdit* editPath = new QLineEdit(rowPath);
	editPath->setText("./");
	editPath->setEnabled(false);
	hPath->addWidget(lblPath);
	hPath->addSpacing(8);
	hPath->addWidget(editPath, 1);
	logLayout->addWidget(cbLogEnable);
	logLayout->addWidget(rowCache);
	logLayout->addWidget(rowPath);
	v->addWidget(logGroup);

	// 多设备配置
	struct DeviceRow {
		QWidget* root;
		QComboBox* devType;
		QComboBox* hostIp;
		QLineEdit* mcIp;
		QSpinBox* hp1;
		QSpinBox* hp2;
		QSpinBox* hp3;
		QSpinBox* hp4;
		QSpinBox* hp5;
		QPushButton* btnRemove;
	};
	QVector<DeviceRow*> deviceRows;

	auto populateHostIpsTo = [&](QComboBox* combo) {
		combo->clear();
		for (const QNetworkInterface &iface : QNetworkInterface::allInterfaces()) {
			if (!(iface.flags() & QNetworkInterface::IsUp) || !(iface.flags() & QNetworkInterface::IsRunning)) continue;
			if (iface.flags() & QNetworkInterface::IsLoopBack) continue;
			for (const QNetworkAddressEntry &entry : iface.addressEntries()) {
				const QHostAddress &addr = entry.ip();
				if (addr.protocol() != QAbstractSocket::IPv4Protocol) continue;
				const QString ip = addr.toString();
				if (ip == "0.0.0.0" || ip.startsWith("169.254.")) continue;
				const QString label = QString("%1  -  %2 (%3)").arg(ip, iface.humanReadableName(), iface.name());
				combo->addItem(label, ip);
			}
		}
	};

	auto createLidarNetDefaults = [&](const QString& type, QJsonObject& out) {
        if (type == "MID360" || type == "Mid360s" || type == "Avia2") {
			out.insert("cmd_data_port", 56100);
			out.insert("push_msg_port", 56200);
			out.insert("point_data_port", 56300);
			out.insert("imu_data_port", 56400);
			out.insert("log_data_port", 56500);
		} else { // HAP
			out.insert("cmd_data_port", 56000);
			out.insert("push_msg_port", 0);
			out.insert("point_data_port", 57000);
			out.insert("imu_data_port", 58000);
			out.insert("log_data_port", 59000);
		}
	};

	auto applyHostPortDefaults = [&](const QString& type, DeviceRow* row) {
        if (type == "MID360" || type == "Mid360s" || type == "Avia2") {
			row->hp1->setValue(56101);
			row->hp2->setValue(56201);
			row->hp3->setValue(56301);
			row->hp4->setValue(56401);
			row->hp5->setValue(56501);
		} else { // HAP
			row->hp1->setValue(56000);
			row->hp2->setValue(0);
			row->hp3->setValue(57000);
			row->hp4->setValue(58000);
			row->hp5->setValue(59000);
		}
	};

	QGroupBox* lidarDevicesGroup = new QGroupBox("设备列表", &dlg);
	QVBoxLayout* lidarDevicesLayout = new QVBoxLayout(lidarDevicesGroup);
	QWidget* toolbar = new QWidget(lidarDevicesGroup);
	QHBoxLayout* hToolbar = new QHBoxLayout(toolbar);
	hToolbar->setContentsMargins(0,0,0,0);
	QPushButton* btnAddDevice = new QPushButton("添加设备", toolbar);
	hToolbar->addWidget(btnAddDevice);
	hToolbar->addStretch();
	lidarDevicesLayout->addWidget(toolbar);

	QWidget* rowsContainer = new QWidget(lidarDevicesGroup);
	QVBoxLayout* rowsLayout = new QVBoxLayout(rowsContainer);
	rowsLayout->setContentsMargins(0,0,0,0);
	rowsLayout->setSpacing(6);
	QScrollArea* rowsScroll = new QScrollArea(lidarDevicesGroup);
	rowsScroll->setWidgetResizable(true);
	rowsScroll->setFrameShape(QFrame::NoFrame);
	rowsScroll->setWidget(rowsContainer);
	rowsScroll->setMinimumHeight(260);
	rowsScroll->setMinimumWidth(660);
	lidarDevicesLayout->addWidget(rowsScroll);
	v->addWidget(lidarDevicesGroup);

	auto addDeviceRow = [&]() {
		DeviceRow* r = new DeviceRow();
		r->root = new QWidget(rowsContainer);
		QGridLayout* grid = new QGridLayout(r->root);
		grid->setContentsMargins(0,0,0,0);

		r->devType = new QComboBox(r->root);
        r->devType->addItems({"MID360", "Mid360s", "Avia2", "HAP"});
        r->hostIp = new QComboBox(r->root);
		populateHostIpsTo(r->hostIp);
		r->mcIp = new QLineEdit(r->root);
		r->hp1 = new QSpinBox(r->root);
		r->hp2 = new QSpinBox(r->root);
		r->hp3 = new QSpinBox(r->root);
		r->hp4 = new QSpinBox(r->root);
		r->hp5 = new QSpinBox(r->root);
		for (QSpinBox* s : {r->hp1,r->hp2,r->hp3,r->hp4,r->hp5}) { s->setRange(0, 65535); }
		r->btnRemove = new QPushButton("删除", r->root);

		int row = 0;
		grid->addWidget(new QLabel("设备类型:"), row, 0); grid->addWidget(r->devType, row, 1);
		grid->addWidget(new QLabel("host_ip:"), row, 2); grid->addWidget(r->hostIp, row, 3);
		grid->addWidget(r->btnRemove, row, 4);
		row++;
		grid->addWidget(new QLabel("multicast_ip(可选):"), row, 0); grid->addWidget(r->mcIp, row, 1, 1, 4);
		row++;
		grid->addWidget(new QLabel("cmd_data_port:"), row, 0); grid->addWidget(r->hp1, row, 1);
		grid->addWidget(new QLabel("push_msg_port:"), row, 2); grid->addWidget(r->hp2, row, 3);
		row++;
		grid->addWidget(new QLabel("point_data_port:"), row, 0); grid->addWidget(r->hp3, row, 1);
		grid->addWidget(new QLabel("imu_data_port:"), row, 2); grid->addWidget(r->hp4, row, 3);
		row++;
		grid->addWidget(new QLabel("log_data_port:"), row, 0); grid->addWidget(r->hp5, row, 1);

		applyHostPortDefaults(r->devType->currentText(), r);
		QObject::connect(r->devType, QOverload<int>::of(&QComboBox::currentIndexChanged), &dlg, [&, r](int){ applyHostPortDefaults(r->devType->currentText(), r); });
		QObject::connect(r->btnRemove, &QPushButton::clicked, &dlg, [&, r]() {
			int idx = deviceRows.indexOf(r);
			if (idx >= 0) deviceRows.removeAt(idx);
			r->root->deleteLater();
			delete r;
		});

		rowsLayout->addWidget(r->root);
		deviceRows.append(r);
	};
	QObject::connect(btnAddDevice, &QPushButton::clicked, &dlg, addDeviceRow);
	// 默认添加一行
	addDeviceRow();

	QDialogButtonBox* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
	v->addWidget(box);
	connect(box, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
	connect(box, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

	if (dlg.exec() != QDialog::Accepted) return false;

	// 校验必要字段
	if (deviceRows.isEmpty()) {
		QMessageBox::warning(this, "生成配置文件", "请至少添加一台设备");
		return false;
	}
	for (DeviceRow* r : deviceRows) {
		if (!r || r->hostIp->count() == 0) {
			QMessageBox::warning(this, "生成配置文件", "未检测到主机网口IPv4地址，请检查网络连接");
			return false;
		}
	}

	// 组装 JSON
	QJsonObject root;
	root.insert("lidar_log_enable", cbLogEnable->isChecked());
	root.insert("lidar_log_cache_size_MB", spinCache->value());
	root.insert("lidar_log_path", "./");

    QJsonArray hostArrMID360;
    QJsonArray hostArrMid360s;
    QJsonArray hostArrAvia2;
    QJsonArray hostArrHap;
    for (DeviceRow* r : deviceRows) {
        QString type = r->devType->currentText();
        QJsonObject hostObj;
        hostObj.insert("host_ip", r->hostIp->currentData().toString());
        if (!r->mcIp->text().trimmed().isEmpty()) hostObj.insert("multicast_ip", r->mcIp->text().trimmed());
        hostObj.insert("cmd_data_port", r->hp1->value());
        hostObj.insert("point_data_port", r->hp3->value());
        hostObj.insert("imu_data_port", r->hp4->value());
        hostObj.insert("push_msg_port", r->hp2->value());
        hostObj.insert("log_data_port", r->hp5->value());

        if (type == "MID360") hostArrMID360.append(hostObj);
        else if (type == "Mid360s") hostArrMid360s.append(hostObj);
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
            // 回退：仍写入程序目录（便于便携运行）
            outPath = QApplication::applicationDirPath() + "/config.json";
        }
    }
    QFile f(outPath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        QMessageBox::critical(this, "生成配置文件", QString("无法写入: %1").arg(QDir::toNativeSeparators(outPath)));
        return false;
    }

    // 手动控制字段顺序
    QJsonObject orderedRoot;

    // 1) 日志配置（固定在最前面）
    orderedRoot.insert("lidar_log_enable", cbLogEnable->isChecked());
    orderedRoot.insert("lidar_log_cache_size_MB", spinCache->value());
    orderedRoot.insert("lidar_log_path", "./");

    // 2) 设备配置（保持 lidar_net_info 在 host_net_info 前面）
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
    if (!hostArrAvia2.isEmpty()) {
        orderedRoot.insert("Avia2", makeDeviceObject("Avia2", hostArrAvia2));
    }
    if (!hostArrHap.isEmpty()) {
        orderedRoot.insert("HAP", makeDeviceObject("HAP", hostArrHap));
    }

    // 输出 JSON
    QJsonDocument doc(orderedRoot);
    f.write(doc.toJson(QJsonDocument::Indented));
    f.close();

    QMessageBox::information(this, "生成配置文件", QString("已生成: %1").arg(QDir::toNativeSeparators(outPath)));
    return true;

}

void LivoxViewerWindow::updateNoiseFilterList()
{
    if (!filterState.noiseFilterList) return;

    filterState.noiseFilterList->clear();
    for (uint8_t tag : filterState.noiseFilterTags) {
        QString itemText = QString("Tag值: %1").arg(tag);
        QListWidgetItem* item = new QListWidgetItem(itemText);
        item->setData(Qt::UserRole, tag);
        filterState.noiseFilterList->addItem(item);
    }

    // 更新移除按钮状态
    if (filterState.removeNoiseFilterButton) {
        filterState.removeNoiseFilterButton->setEnabled(!filterState.noiseFilterTags.isEmpty());
    }
}

void LivoxViewerWindow::refreshNetworkInterfaces()
{
    if (!networkInterfaceCombo) return;

    const QString previousName = selectedInterfaceName;
    networkInterfaceCombo->blockSignals(true);
    networkInterfaceCombo->clear();

    const QList<NetworkInterfaceService::NetworkInterfaceInfo> interfaces =
        NetworkInterfaceService::availableLidarInterfaces();

    int selectedIndex = -1;
    for (int i = 0; i < interfaces.size(); ++i) {
        const NetworkInterfaceService::NetworkInterfaceInfo& iface = interfaces.at(i);
        networkInterfaceCombo->addItem(QString("%1 - %2").arg(iface.displayName, iface.ipv4), iface.systemName);
        networkInterfaceCombo->setItemData(i, iface.ipv4, Qt::UserRole + 1);
        networkInterfaceCombo->setItemData(i, iface.displayName, Qt::UserRole + 2);
        networkInterfaceCombo->setItemData(i, iface.netmask, Qt::UserRole + 3);
        networkInterfaceCombo->setItemData(i, iface.broadcast, Qt::UserRole + 4);
        if (iface.systemName == previousName) {
            selectedIndex = i;
        }
    }

    if (selectedIndex < 0 && !interfaces.isEmpty()) {
        selectedIndex = 0;
    }

    if (selectedIndex >= 0) {
        networkInterfaceCombo->setCurrentIndex(selectedIndex);
        selectLidarInterface(interfaces.at(selectedIndex));
        logMessage(QString("[Network] Selected lidar interface: %1 (%2)")
                       .arg(selectedInterfaceDisplayName, selectedHostIp));
    } else {
        selectedInterfaceName.clear();
        selectedInterfaceDisplayName.clear();
        selectedHostIp.clear();
        selectedNetmask.clear();
        selectedBroadcast.clear();
        selectedNetworkIP.clear();
        selectedNetworkInterfaceHumanName.clear();
        selectedNetworkInterfaceSysName.clear();
        logMessage("[Network] No valid lidar network interface found");
    }

    networkInterfaceCombo->blockSignals(false);
}

void LivoxViewerWindow::onNetworkInterfaceChanged(int index)
{
    if (!networkInterfaceCombo || index < 0) return;

    const QString systemName = networkInterfaceCombo->itemData(index, Qt::UserRole).toString();
    const auto iface = NetworkInterfaceService::findInterfaceByName(systemName);
    if (!iface.has_value()) {
        return;
    }

    const QString oldName = selectedInterfaceName;
    const QString oldIp = selectedHostIp;
    selectLidarInterface(*iface);

    if (oldName == selectedInterfaceName && oldIp == selectedHostIp) {
        return;
    }

    logMessage(QString("[Realtime] Network interface changed: %1 -> %2")
                   .arg(oldIp, selectedHostIp));

    if (sdk_started || sdk_initialized || realtimeState == RealtimeConnectionState::Running) {
        restartRealtimeConnectionForNetworkChange();
        return;
    }

    stopLidarDiscovery();
    startLidarDiscovery();
}

QString LivoxViewerWindow::getSelectedHostIP() const
{
    if (!selectedInterfaceName.isEmpty()) {
        const auto iface = NetworkInterfaceService::findInterfaceByName(selectedInterfaceName);
        if (iface.has_value()) {
            return iface->ipv4;
        }
    }

    const QList<NetworkInterfaceService::NetworkInterfaceInfo> interfaces =
        NetworkInterfaceService::availableLidarInterfaces();
    if (!interfaces.isEmpty()) {
        return interfaces.first().ipv4;
    }

    return QString();
}

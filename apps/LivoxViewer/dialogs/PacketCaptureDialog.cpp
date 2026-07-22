#include "dialogs/PacketCaptureDialog.h"

#include "dialogs/DialogWindowUtils.h"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QHostAddress>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPushButton>
#include <QRadioButton>
#include <QRegularExpression>
#include <QScrollBar>
#include <QSettings>
#include <QSignalBlocker>
#include <QStandardPaths>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QUrl>
#include <QVBoxLayout>

#include <pcap.h>

#include <chrono>
#include <cstring>

namespace {

constexpr int kPacketBatchSize = 64;
constexpr quint16 kBroadcastPort = 56000;
constexpr quint16 kControlPort = 56100;
constexpr quint16 kPointCloudPortNew = 56300;
constexpr quint16 kPointCloudPortOld = 57000;
constexpr quint16 kImuPortNew = 56400;
constexpr quint16 kImuPortOld = 58000;
constexpr quint16 kPushPort = 56200;
constexpr quint16 kPtpEventPort = 319;
constexpr quint16 kPtpGeneralPort = 320;

quint16 readBigEndian16(const unsigned char* data)
{
    return (quint16(data[0]) << 8) | quint16(data[1]);
}

quint32 readBigEndian32(const unsigned char* data)
{
    return (quint32(data[0]) << 24) |
           (quint32(data[1]) << 16) |
           (quint32(data[2]) << 8) |
           quint32(data[3]);
}

QString ipv4Address(const unsigned char* data)
{
    return QHostAddress(readBigEndian32(data)).toString();
}

QString ipv6Address(const unsigned char* data)
{
    Q_IPV6ADDR address{};
    std::memcpy(address.c, data, sizeof(address.c));
    return QHostAddress(address).toString();
}

QString macAddress(const unsigned char* data)
{
    return QStringLiteral("%1:%2:%3:%4:%5:%6")
        .arg(data[0], 2, 16, QLatin1Char('0'))
        .arg(data[1], 2, 16, QLatin1Char('0'))
        .arg(data[2], 2, 16, QLatin1Char('0'))
        .arg(data[3], 2, 16, QLatin1Char('0'))
        .arg(data[4], 2, 16, QLatin1Char('0'))
        .arg(data[5], 2, 16, QLatin1Char('0'))
        .toUpper();
}

QString payloadHex(const unsigned char* data, int capturedLength, int payloadOffset)
{
    if (payloadOffset >= capturedLength) {
        return {};
    }
    return QByteArray(reinterpret_cast<const char*>(data + payloadOffset), capturedLength - payloadOffset)
        .toHex(' ')
        .toUpper();
}

bool usesPort(quint16 sourcePort, quint16 destinationPort, quint16 port)
{
    return sourcePort == port || destinationPort == port;
}

QString initialSaveDirectory()
{
    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    const QString savedDirectory = settings.value(QStringLiteral("save/lastPcapCaptureDir")).toString();
    return savedDirectory.isEmpty()
        ? QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)
        : savedDirectory;
}

void setPacketCell(QTableWidget* table, int row, int column, const QString& text)
{
    QTableWidgetItem* item = new QTableWidgetItem(text);
    item->setTextAlignment(Qt::AlignCenter);
    item->setToolTip(text);
    table->setItem(row, column, item);
}

} // namespace

PacketCaptureDialog::PacketCaptureDialog(QWidget* parent)
    : QDialog(parent)
{
    DialogWindowUtils::enableTopLevelWindowControls(this);
    setAttribute(Qt::WA_DeleteOnClose);
    setWindowTitle(QStringLiteral("网络数据抓包"));
    resize(1320, 760);
    setMinimumSize(980, 620);

    QVBoxLayout* root = new QVBoxLayout(this);
    root->setContentsMargins(14, 14, 14, 14);
    root->setSpacing(10);

    QWidget* captureRow = new QWidget(this);
    QHBoxLayout* captureLayout = new QHBoxLayout(captureRow);
    captureLayout->setContentsMargins(0, 0, 0, 0);
    captureLayout->setSpacing(8);
    captureLayout->addWidget(new QLabel(QStringLiteral("网卡"), captureRow));
    m_interfaceCombo = new QComboBox(captureRow);
    m_interfaceCombo->setMinimumWidth(300);
    captureLayout->addWidget(m_interfaceCombo, 1);
    m_startButton = new QPushButton(QStringLiteral("开始抓包"), captureRow);
    m_stopButton = new QPushButton(QStringLiteral("停止"), captureRow);
    m_clearButton = new QPushButton(QStringLiteral("清空"), captureRow);
    captureLayout->addWidget(m_startButton);
    captureLayout->addWidget(m_stopButton);
    captureLayout->addWidget(m_clearButton);
    root->addWidget(captureRow);

    QGroupBox* filterGroup = new QGroupBox(QStringLiteral("显示筛选（可多选）"), this);
    QGridLayout* filterLayout = new QGridLayout(filterGroup);
    filterLayout->setContentsMargins(10, 8, 10, 8);
    filterLayout->setHorizontalSpacing(14);
    filterLayout->setVerticalSpacing(8);
    m_broadcastFilter = new QCheckBox(QStringLiteral("广播（56000）"), filterGroup);
    m_controlFilter = new QCheckBox(QStringLiteral("控制指令（56100）"), filterGroup);
    m_pointCloudFilter = new QCheckBox(QStringLiteral("点云数据"), filterGroup);
    m_imuFilter = new QCheckBox(QStringLiteral("IMU 数据"), filterGroup);
    m_pushFilter = new QCheckBox(QStringLiteral("推送信息"), filterGroup);
    m_ptpFilter = new QCheckBox(QStringLiteral("PTP 报文"), filterGroup);
    m_arpFilter = new QCheckBox(QStringLiteral("ARP 握手包"), filterGroup);
    filterLayout->addWidget(m_broadcastFilter, 0, 0);
    filterLayout->addWidget(m_controlFilter, 0, 1);
    filterLayout->addWidget(m_pointCloudFilter, 0, 2);
    filterLayout->addWidget(m_imuFilter, 0, 3);
    filterLayout->addWidget(m_pushFilter, 0, 4);
    filterLayout->addWidget(m_ptpFilter, 0, 5);
    filterLayout->addWidget(m_arpFilter, 0, 6);

    m_sourceIpFilter = new QCheckBox(QStringLiteral("源 IP"), filterGroup);
    m_sourceIpEdit = new QLineEdit(filterGroup);
    m_sourceIpEdit->setPlaceholderText(QStringLiteral("输入源 IP"));
    m_sourceIpEdit->setEnabled(false);
    m_destinationIpFilter = new QCheckBox(QStringLiteral("目标 IP"), filterGroup);
    m_destinationIpEdit = new QLineEdit(filterGroup);
    m_destinationIpEdit->setPlaceholderText(QStringLiteral("输入目标 IP"));
    m_destinationIpEdit->setEnabled(false);
    filterLayout->addWidget(m_sourceIpFilter, 1, 0);
    filterLayout->addWidget(m_sourceIpEdit, 1, 1, 1, 2);
    filterLayout->addWidget(m_destinationIpFilter, 1, 3);
    filterLayout->addWidget(m_destinationIpEdit, 1, 4, 1, 3);
    filterLayout->setColumnStretch(2, 1);
    filterLayout->setColumnStretch(5, 1);
    root->addWidget(filterGroup);

    QGroupBox* saveGroup = new QGroupBox(QStringLiteral("PCAP 保存"), this);
    QGridLayout* saveLayout = new QGridLayout(saveGroup);
    saveLayout->setContentsMargins(10, 8, 10, 8);
    saveLayout->setHorizontalSpacing(8);
    saveLayout->setVerticalSpacing(8);
    saveLayout->addWidget(new QLabel(QStringLiteral("保存目录"), saveGroup), 0, 0);
    m_saveDirectoryEdit = new QLineEdit(initialSaveDirectory(), saveGroup);
    QPushButton* browseButton = new QPushButton(QStringLiteral("浏览..."), saveGroup);
    saveLayout->addWidget(m_saveDirectoryEdit, 0, 1, 1, 5);
    saveLayout->addWidget(browseButton, 0, 6);
    saveLayout->addWidget(new QLabel(QStringLiteral("文件名"), saveGroup), 1, 0);
    m_fileNameEdit = new QLineEdit(saveGroup);
    m_saveAllRadio = new QRadioButton(QStringLiteral("保存所有包"), saveGroup);
    m_saveFilteredRadio = new QRadioButton(QStringLiteral("只保存筛选的包"), saveGroup);
    m_saveAllRadio->setChecked(true);
    m_saveButton = new QPushButton(QStringLiteral("保存 PCAP"), saveGroup);
    QPushButton* openDirectoryButton = new QPushButton(QStringLiteral("打开保存目录"), saveGroup);
    saveLayout->addWidget(m_fileNameEdit, 1, 1, 1, 2);
    saveLayout->addWidget(m_saveAllRadio, 1, 3);
    saveLayout->addWidget(m_saveFilteredRadio, 1, 4, 1, 2);
    saveLayout->addWidget(m_saveButton, 1, 6);
    saveLayout->addWidget(openDirectoryButton, 1, 7);
    saveLayout->setColumnStretch(2, 1);
    root->addWidget(saveGroup);

    m_statusLabel = new QLabel(QStringLiteral("就绪。抓包停止后可手动保存全部或当前筛选的数据包。"), this);
    m_statusLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));
    root->addWidget(m_statusLabel);

    m_packetTable = new QTableWidget(this);
    m_packetTable->setColumnCount(8);
    m_packetTable->setHorizontalHeaderLabels({
        QStringLiteral("序号"),
        QStringLiteral("时间"),
        QStringLiteral("源地址"),
        QStringLiteral("目标地址"),
        QStringLiteral("协议"),
        QStringLiteral("长度"),
        QStringLiteral("端口"),
        QStringLiteral("内容")
    });
    m_packetTable->verticalHeader()->setVisible(false);
    m_packetTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_packetTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_packetTable->setAlternatingRowColors(true);
    m_packetTable->setShowGrid(false);
    QHeaderView* packetHeader = m_packetTable->horizontalHeader();
    packetHeader->setDefaultAlignment(Qt::AlignCenter);
    packetHeader->setSectionResizeMode(QHeaderView::Interactive);
    packetHeader->setStretchLastSection(false);
    const int columnWidths[] = {70, 110, 165, 165, 85, 75, 125, 460};
    for (int column = 0; column < 8; ++column) {
        m_packetTable->setColumnWidth(column, columnWidths[column]);
    }
    root->addWidget(m_packetTable, 1);

    connect(m_interfaceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (index >= 0 && m_interfaceChangedHandler) {
            m_interfaceChangedHandler(m_interfaceCombo->itemData(index).toString());
        }
    });

    const QList<QCheckBox*> typeFilters = {
        m_broadcastFilter, m_controlFilter, m_pointCloudFilter, m_imuFilter,
        m_pushFilter, m_ptpFilter, m_arpFilter
    };
    for (QCheckBox* filter : typeFilters) {
        connect(filter, &QCheckBox::toggled, this, [this]() { rebuildTable(); });
    }
    connect(m_sourceIpFilter, &QCheckBox::toggled, this, [this](bool checked) {
        m_sourceIpEdit->setEnabled(checked);
        rebuildTable();
    });
    connect(m_destinationIpFilter, &QCheckBox::toggled, this, [this](bool checked) {
        m_destinationIpEdit->setEnabled(checked);
        rebuildTable();
    });
    connect(m_sourceIpEdit, &QLineEdit::textChanged, this, [this]() { rebuildTable(); });
    connect(m_destinationIpEdit, &QLineEdit::textChanged, this, [this]() { rebuildTable(); });
    connect(m_fileNameEdit, &QLineEdit::textEdited, this, [this]() { m_fileNameCustomized = true; });
    connect(m_saveAllRadio, &QRadioButton::toggled, this, [this]() {
        if (!m_fileNameCustomized) {
            updateDefaultFileName();
        }
    });
    connect(m_startButton, &QPushButton::clicked, this, [this]() { startCapture(); });
    connect(m_stopButton, &QPushButton::clicked, this, [this]() { stopCapture(); });
    connect(m_clearButton, &QPushButton::clicked, this, [this]() {
        m_packets.clear();
        m_packetTable->setRowCount(0);
        updateCaptureControls();
        m_statusLabel->setText(m_captureActive
            ? QStringLiteral("抓包中 · 已清空当前数据")
            : QStringLiteral("数据已清空"));
    });
    connect(browseButton, &QPushButton::clicked, this, [this]() {
        QFileDialog* directoryDialog = new QFileDialog(
            this, QStringLiteral("选择 PCAP 保存目录"), m_saveDirectoryEdit->text());
        directoryDialog->setFileMode(QFileDialog::Directory);
        directoryDialog->setOption(QFileDialog::ShowDirsOnly);
        directoryDialog->setOption(QFileDialog::DontUseNativeDialog);
        directoryDialog->setModal(false);
        directoryDialog->setAttribute(Qt::WA_DeleteOnClose);
        connect(directoryDialog, &QFileDialog::fileSelected, this, [this](const QString& directory) {
            m_saveDirectoryEdit->setText(directory);
            QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
            settings.setValue(QStringLiteral("save/lastPcapCaptureDir"), directory);
        });
        directoryDialog->open();
    });
    connect(m_saveButton, &QPushButton::clicked, this, [this]() { saveCapture(); });
    connect(openDirectoryButton, &QPushButton::clicked, this, [this]() {
        QDesktopServices::openUrl(QUrl::fromLocalFile(m_saveDirectoryEdit->text().trimmed()));
    });

    m_captureTimestamp = QDateTime::currentDateTime();
    updateDefaultFileName();
    updateCaptureControls();
}

PacketCaptureDialog::~PacketCaptureDialog()
{
    m_stopRequested.store(true);
    if (m_captureThread.joinable()) {
        m_captureThread.join();
    }
}

void PacketCaptureDialog::setInterfaces(
    const QList<NetworkInterfaceService::NetworkInterfaceInfo>& interfaces,
    const QString& selectedSystemName)
{
    if (m_captureActive) {
        return;
    }

    QSignalBlocker blocker(m_interfaceCombo);
    m_interfaceCombo->clear();
    int selectedIndex = -1;
    for (int i = 0; i < interfaces.size(); ++i) {
        const NetworkInterfaceService::NetworkInterfaceInfo& interface = interfaces.at(i);
        m_interfaceCombo->addItem(
            QStringLiteral("%1 - %2").arg(interface.displayName, interface.ipv4),
            interface.systemName);
        m_interfaceCombo->setItemData(i, interface.ipv4, Qt::UserRole + 1);
        if (interface.systemName == selectedSystemName) {
            selectedIndex = i;
        }
    }
    m_interfaceCombo->setCurrentIndex(selectedIndex >= 0 ? selectedIndex : (interfaces.isEmpty() ? -1 : 0));
    updateCaptureControls();
}

void PacketCaptureDialog::selectInterface(const QString& systemName)
{
    if (m_captureActive) {
        return;
    }
    const int index = m_interfaceCombo->findData(systemName);
    if (index >= 0 && index != m_interfaceCombo->currentIndex()) {
        QSignalBlocker blocker(m_interfaceCombo);
        m_interfaceCombo->setCurrentIndex(index);
    }
}

void PacketCaptureDialog::setInterfaceChangedHandler(std::function<void(const QString&)> handler)
{
    m_interfaceChangedHandler = std::move(handler);
}

void PacketCaptureDialog::setDeviceSerialNumber(const QString& serialNumber)
{
    m_deviceSerialNumber = serialNumber.isEmpty() ? QStringLiteral("Unknown") : serialNumber;
    if (!m_captureActive && m_packets.isEmpty()) {
        updateDefaultFileName();
    }
}

void PacketCaptureDialog::closeEvent(QCloseEvent* event)
{
    stopCapture();
    QDialog::closeEvent(event);
}

void PacketCaptureDialog::startCapture()
{
    const int interfaceIndex = m_interfaceCombo->currentIndex();
    if (interfaceIndex < 0) {
        QMessageBox::warning(this, QStringLiteral("网络数据抓包"), QStringLiteral("请选择抓包网卡。"));
        return;
    }

    m_packets.clear();
    m_packetTable->setRowCount(0);
    m_dataLinkType = 0;
    m_captureTimestamp = QDateTime::currentDateTime();
    m_fileNameCustomized = false;
    updateDefaultFileName();
    m_stopRequested.store(false);
    m_captureActive = true;
    updateCaptureControls();
    m_statusLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));
    m_statusLabel->setText(QStringLiteral("正在打开网卡并开始抓包..."));

    const QString systemName = m_interfaceCombo->itemData(interfaceIndex).toString();
    const QString ipv4 = m_interfaceCombo->itemData(interfaceIndex, Qt::UserRole + 1).toString();
    m_captureThread = std::thread([this, systemName, ipv4]() {
        captureLoop(systemName, ipv4);
    });
}

void PacketCaptureDialog::stopCapture()
{
    if (!m_captureActive) {
        return;
    }
    m_stopRequested.store(true);
    if (m_captureThread.joinable()) {
        m_captureThread.join();
    }
    m_captureActive = false;
    updateCaptureControls();
    m_statusLabel->setText(QStringLiteral("抓包已停止 · 已捕获 %1 个数据包，可手动保存 PCAP")
                               .arg(m_packets.size()));
}

QString PacketCaptureDialog::captureDeviceName(const QString& systemName,
                                               const QString& ipv4,
                                               QString* errorMessage)
{
    char errorBuffer[PCAP_ERRBUF_SIZE]{};
    pcap_if_t* devices = nullptr;
    if (pcap_findalldevs(&devices, errorBuffer) != 0) {
        *errorMessage = QString::fromLocal8Bit(errorBuffer);
        return {};
    }

    QString matchedName;
    QString nameCandidate;
    for (pcap_if_t* device = devices; device; device = device->next) {
        const QString deviceName = QString::fromLocal8Bit(device->name);
        if (deviceName.contains(systemName, Qt::CaseInsensitive)) {
            nameCandidate = deviceName;
        }
        for (pcap_addr_t* address = device->addresses; address; address = address->next) {
            if (!address->addr || address->addr->sa_family != AF_INET) {
                continue;
            }
            const auto* socketAddress = reinterpret_cast<const sockaddr_in*>(address->addr);
            const QString addressText = QHostAddress(ntohl(socketAddress->sin_addr.s_addr)).toString();
            if (addressText == ipv4) {
                matchedName = deviceName;
                break;
            }
        }
        if (!matchedName.isEmpty()) {
            break;
        }
    }
    pcap_freealldevs(devices);

    if (matchedName.isEmpty()) {
        matchedName = nameCandidate;
    }
    if (matchedName.isEmpty()) {
        *errorMessage = QStringLiteral("Npcap 中未找到网卡 %1 (%2)").arg(systemName, ipv4);
    }
    return matchedName;
}

void PacketCaptureDialog::captureLoop(QString systemName, QString ipv4)
{
    auto postFinished = [this](QString message, bool failed) {
        QMetaObject::invokeMethod(this, [this, message = std::move(message), failed]() {
            finishCapture(message, failed);
        }, Qt::QueuedConnection);
    };
    auto postPackets = [this](QVector<PacketRow> packets) {
        QMetaObject::invokeMethod(this, [this, packets = std::move(packets)]() mutable {
            appendPackets(std::move(packets));
        }, Qt::QueuedConnection);
    };

    QString deviceError;
    const QString deviceName = captureDeviceName(systemName, ipv4, &deviceError);
    if (deviceName.isEmpty()) {
        postFinished(deviceError, true);
        return;
    }

    char errorBuffer[PCAP_ERRBUF_SIZE]{};
    const QByteArray encodedDeviceName = deviceName.toLocal8Bit();
    pcap_t* capture = pcap_open_live(encodedDeviceName.constData(), 65535, 1, 250, errorBuffer);
    if (!capture) {
        postFinished(QStringLiteral("打开抓包网卡失败：%1").arg(QString::fromLocal8Bit(errorBuffer)), true);
        return;
    }

    const int dataLinkType = pcap_datalink(capture);
    QVector<PacketRow> batch;
    batch.reserve(kPacketBatchSize);
    auto lastBatchPost = std::chrono::steady_clock::now();
    quint64 packetNumber = 0;
    double firstTimestamp = -1.0;
    QString captureError;

    while (!m_stopRequested.load()) {
        pcap_pkthdr* header = nullptr;
        const unsigned char* data = nullptr;
        const int result = pcap_next_ex(capture, &header, &data);
        if (result == 0) {
            if (!batch.isEmpty() &&
                std::chrono::steady_clock::now() - lastBatchPost >= std::chrono::milliseconds(100)) {
                postPackets(std::move(batch));
                batch.clear();
                batch.reserve(kPacketBatchSize);
                lastBatchPost = std::chrono::steady_clock::now();
            }
            continue;
        }
        if (result < 0) {
            if (result == -1) {
                captureError = QString::fromLocal8Bit(pcap_geterr(capture));
            }
            break;
        }

        const double timestamp = double(header->ts.tv_sec) + double(header->ts.tv_usec) / 1000000.0;
        if (firstTimestamp < 0.0) {
            firstTimestamp = timestamp;
        }
        ++packetNumber;
        batch.append(decodePacket(data,
                                  int(header->caplen),
                                  int(header->len),
                                  dataLinkType,
                                  packetNumber,
                                  timestamp - firstTimestamp,
                                  qint64(header->ts.tv_sec),
                                  qint64(header->ts.tv_usec)));
        if (batch.size() >= kPacketBatchSize ||
            std::chrono::steady_clock::now() - lastBatchPost >= std::chrono::milliseconds(100)) {
            postPackets(std::move(batch));
            batch.clear();
            batch.reserve(kPacketBatchSize);
            lastBatchPost = std::chrono::steady_clock::now();
        }
    }

    if (!batch.isEmpty()) {
        postPackets(std::move(batch));
    }
    pcap_close(capture);

    if (!captureError.isEmpty()) {
        postFinished(QStringLiteral("抓包失败：%1").arg(captureError), true);
    } else {
        postFinished(QStringLiteral("抓包已停止，可手动保存 PCAP"), false);
    }
}

PacketCaptureDialog::PacketRow PacketCaptureDialog::decodePacket(const unsigned char* data,
                                                                 int capturedLength,
                                                                 int originalLength,
                                                                 int dataLinkType,
                                                                 quint64 number,
                                                                 double relativeTimeSec,
                                                                 qint64 timestampSec,
                                                                 qint64 timestampUsec)
{
    PacketRow packet;
    packet.number = number;
    packet.relativeTimeSec = relativeTimeSec;
    packet.length = originalLength;
    packet.source = QStringLiteral("-");
    packet.destination = QStringLiteral("-");
    packet.protocol = QStringLiteral("链路");
    packet.ports = QStringLiteral("-");
    packet.rawData = QByteArray(reinterpret_cast<const char*>(data), capturedLength);
    packet.timestampSec = timestampSec;
    packet.timestampUsec = timestampUsec;
    packet.capturedLength = capturedLength;
    packet.originalLength = originalLength;
    packet.dataLinkType = dataLinkType;

    if (dataLinkType != DLT_EN10MB || capturedLength < 14) {
        packet.payloadHex = payloadHex(data, capturedLength, 0);
        return packet;
    }

    packet.destination = macAddress(data);
    packet.source = macAddress(data + 6);
    int networkOffset = 14;
    quint16 etherType = readBigEndian16(data + 12);
    while ((etherType == 0x8100 || etherType == 0x88a8) && capturedLength >= networkOffset + 4) {
        etherType = readBigEndian16(data + networkOffset + 2);
        networkOffset += 4;
    }

    if (etherType == 0x0800 && capturedLength >= networkOffset + 20) {
        const int headerLength = int(data[networkOffset] & 0x0f) * 4;
        if (headerLength < 20 || capturedLength < networkOffset + headerLength) {
            return packet;
        }
        packet.source = ipv4Address(data + networkOffset + 12);
        packet.destination = ipv4Address(data + networkOffset + 16);
        const quint8 protocol = data[networkOffset + 9];
        const int transportOffset = networkOffset + headerLength;
        int dataOffset = transportOffset;
        if (protocol == 17 && capturedLength >= transportOffset + 8) {
            const quint16 sourcePort = readBigEndian16(data + transportOffset);
            const quint16 destinationPort = readBigEndian16(data + transportOffset + 2);
            packet.protocol = QStringLiteral("UDP");
            packet.ports = QStringLiteral("%1 → %2").arg(sourcePort).arg(destinationPort);
            packet.broadcast = usesPort(sourcePort, destinationPort, kBroadcastPort);
            packet.control = usesPort(sourcePort, destinationPort, kControlPort);
            packet.pointCloud = usesPort(sourcePort, destinationPort, kPointCloudPortNew) ||
                                usesPort(sourcePort, destinationPort, kPointCloudPortOld);
            packet.imu = usesPort(sourcePort, destinationPort, kImuPortNew) ||
                         usesPort(sourcePort, destinationPort, kImuPortOld);
            packet.push = usesPort(sourcePort, destinationPort, kPushPort);
            packet.ptp = usesPort(sourcePort, destinationPort, kPtpEventPort) ||
                         usesPort(sourcePort, destinationPort, kPtpGeneralPort);
            if (packet.ptp) {
                packet.protocol = QStringLiteral("PTP");
            }
            dataOffset = transportOffset + 8;
        } else if (protocol == 6 && capturedLength >= transportOffset + 20) {
            const quint16 sourcePort = readBigEndian16(data + transportOffset);
            const quint16 destinationPort = readBigEndian16(data + transportOffset + 2);
            const int tcpHeaderLength = int(data[transportOffset + 12] >> 4) * 4;
            packet.protocol = QStringLiteral("TCP");
            packet.ports = QStringLiteral("%1 → %2").arg(sourcePort).arg(destinationPort);
            dataOffset = tcpHeaderLength >= 20 && capturedLength >= transportOffset + tcpHeaderLength
                ? transportOffset + tcpHeaderLength
                : transportOffset;
        } else if (protocol == 1) {
            packet.protocol = QStringLiteral("ICMP");
            dataOffset = capturedLength >= transportOffset + 8 ? transportOffset + 8 : transportOffset;
        } else {
            packet.protocol = QStringLiteral("IPv4");
        }
        packet.payloadHex = payloadHex(data, capturedLength, dataOffset);
    } else if (etherType == 0x86dd && capturedLength >= networkOffset + 40) {
        packet.source = ipv6Address(data + networkOffset + 8);
        packet.destination = ipv6Address(data + networkOffset + 24);
        const quint8 nextHeader = data[networkOffset + 6];
        const int transportOffset = networkOffset + 40;
        int dataOffset = transportOffset;
        if (nextHeader == 17 && capturedLength >= transportOffset + 8) {
            const quint16 sourcePort = readBigEndian16(data + transportOffset);
            const quint16 destinationPort = readBigEndian16(data + transportOffset + 2);
            packet.protocol = QStringLiteral("UDPv6");
            packet.ports = QStringLiteral("%1 → %2").arg(sourcePort).arg(destinationPort);
            packet.broadcast = usesPort(sourcePort, destinationPort, kBroadcastPort);
            packet.control = usesPort(sourcePort, destinationPort, kControlPort);
            packet.ptp = usesPort(sourcePort, destinationPort, kPtpEventPort) ||
                         usesPort(sourcePort, destinationPort, kPtpGeneralPort);
            if (packet.ptp) {
                packet.protocol = QStringLiteral("PTP");
            }
            dataOffset = transportOffset + 8;
        } else if (nextHeader == 6 && capturedLength >= transportOffset + 20) {
            const quint16 sourcePort = readBigEndian16(data + transportOffset);
            const quint16 destinationPort = readBigEndian16(data + transportOffset + 2);
            const int tcpHeaderLength = int(data[transportOffset + 12] >> 4) * 4;
            packet.protocol = QStringLiteral("TCPv6");
            packet.ports = QStringLiteral("%1 → %2").arg(sourcePort).arg(destinationPort);
            dataOffset = tcpHeaderLength >= 20 && capturedLength >= transportOffset + tcpHeaderLength
                ? transportOffset + tcpHeaderLength
                : transportOffset;
        } else {
            packet.protocol = QStringLiteral("IPv6");
        }
        packet.payloadHex = payloadHex(data, capturedLength, dataOffset);
    } else if (etherType == 0x0806) {
        packet.protocol = QStringLiteral("ARP");
        packet.arp = true;
        if (capturedLength >= networkOffset + 28) {
            packet.source = ipv4Address(data + networkOffset + 14);
            packet.destination = ipv4Address(data + networkOffset + 24);
        }
        packet.payloadHex = payloadHex(data, capturedLength, networkOffset);
    } else if (etherType == 0x88f7) {
        packet.protocol = QStringLiteral("PTP");
        packet.ptp = true;
        packet.payloadHex = payloadHex(data, capturedLength, networkOffset);
    } else {
        packet.protocol = QStringLiteral("Ethernet 0x%1")
                              .arg(etherType, 4, 16, QLatin1Char('0'))
                              .toUpper();
        packet.payloadHex = payloadHex(data, capturedLength, networkOffset);
    }

    return packet;
}

void PacketCaptureDialog::appendPackets(QVector<PacketRow> packets)
{
    const bool scrollToBottom = m_packetTable->verticalScrollBar()->value() >=
        m_packetTable->verticalScrollBar()->maximum() - 1;
    for (PacketRow& packet : packets) {
        if (m_dataLinkType == 0) {
            m_dataLinkType = packet.dataLinkType;
        }
        m_packets.append(std::move(packet));
        const PacketRow& storedPacket = m_packets.constLast();
        if (matchesFilter(storedPacket)) {
            appendPacketToTable(storedPacket);
        }
    }
    if (scrollToBottom) {
        m_packetTable->scrollToBottom();
    }
    updateCaptureControls();
    if (m_captureActive) {
        m_statusLabel->setText(QStringLiteral("抓包中 · 已捕获 %1 个数据包 · 当前显示 %2 个")
                                   .arg(m_packets.size())
                                   .arg(m_packetTable->rowCount()));
    }
}

void PacketCaptureDialog::appendPacketToTable(const PacketRow& packet)
{
    const int row = m_packetTable->rowCount();
    m_packetTable->insertRow(row);
    setPacketCell(m_packetTable, row, 0, QString::number(packet.number));
    setPacketCell(m_packetTable, row, 1, QString::number(packet.relativeTimeSec, 'f', 6));
    setPacketCell(m_packetTable, row, 2, packet.source);
    setPacketCell(m_packetTable, row, 3, packet.destination);
    setPacketCell(m_packetTable, row, 4, packet.protocol);
    setPacketCell(m_packetTable, row, 5, QString::number(packet.length));
    setPacketCell(m_packetTable, row, 6, packet.ports);
    setPacketCell(m_packetTable, row, 7, packet.payloadHex);
}

void PacketCaptureDialog::rebuildTable()
{
    m_packetTable->setUpdatesEnabled(false);
    m_packetTable->setRowCount(0);
    for (const PacketRow& packet : m_packets) {
        if (matchesFilter(packet)) {
            appendPacketToTable(packet);
        }
    }
    m_packetTable->setUpdatesEnabled(true);
    m_packetTable->viewport()->update();
    if (!m_fileNameCustomized) {
        updateDefaultFileName();
    }
    m_statusLabel->setText(QStringLiteral("已捕获 %1 个数据包 · 当前显示 %2 个")
                               .arg(m_packets.size())
                               .arg(m_packetTable->rowCount()));
}

bool PacketCaptureDialog::matchesFilter(const PacketRow& packet) const
{
    const bool hasTypeFilter = m_broadcastFilter->isChecked() ||
                               m_controlFilter->isChecked() ||
                               m_pointCloudFilter->isChecked() ||
                               m_imuFilter->isChecked() ||
                               m_pushFilter->isChecked() ||
                               m_ptpFilter->isChecked() ||
                               m_arpFilter->isChecked();
    const bool matchesType = !hasTypeFilter ||
                             (m_broadcastFilter->isChecked() && packet.broadcast) ||
                             (m_controlFilter->isChecked() && packet.control) ||
                             (m_pointCloudFilter->isChecked() && packet.pointCloud) ||
                             (m_imuFilter->isChecked() && packet.imu) ||
                             (m_pushFilter->isChecked() && packet.push) ||
                             (m_ptpFilter->isChecked() && packet.ptp) ||
                             (m_arpFilter->isChecked() && packet.arp);
    if (!matchesType) {
        return false;
    }
    if (m_sourceIpFilter->isChecked() &&
        packet.source.compare(m_sourceIpEdit->text().trimmed(), Qt::CaseInsensitive) != 0) {
        return false;
    }
    if (m_destinationIpFilter->isChecked() &&
        packet.destination.compare(m_destinationIpEdit->text().trimmed(), Qt::CaseInsensitive) != 0) {
        return false;
    }
    return true;
}

QString PacketCaptureDialog::selectedFilterName() const
{
    QStringList filters;
    if (m_broadcastFilter->isChecked()) {
        filters.append(QStringLiteral("broadcast"));
    }
    if (m_controlFilter->isChecked()) {
        filters.append(QStringLiteral("control"));
    }
    if (m_pointCloudFilter->isChecked()) {
        filters.append(QStringLiteral("pointcloud"));
    }
    if (m_imuFilter->isChecked()) {
        filters.append(QStringLiteral("imu"));
    }
    if (m_pushFilter->isChecked()) {
        filters.append(QStringLiteral("push"));
    }
    if (m_ptpFilter->isChecked()) {
        filters.append(QStringLiteral("ptp"));
    }
    if (m_arpFilter->isChecked()) {
        filters.append(QStringLiteral("arp"));
    }
    if (m_sourceIpFilter->isChecked()) {
        filters.append(QStringLiteral("sourceip"));
    }
    if (m_destinationIpFilter->isChecked()) {
        filters.append(QStringLiteral("destinationip"));
    }
    return filters.isEmpty() ? QStringLiteral("all") : filters.join(QLatin1Char('_'));
}

void PacketCaptureDialog::updateDefaultFileName()
{
    QString serialNumber = m_deviceSerialNumber;
    serialNumber.replace(QRegularExpression(QStringLiteral("[\\\\/:*?\"<>|]")), QStringLiteral("_"));
    m_fileNameEdit->setText(QStringLiteral("%1_%2_%3.pcap")
                                .arg(serialNumber,
                                     m_captureTimestamp.toString(QStringLiteral("yyyyMMdd_HHmmss_zzz")),
                                     m_saveFilteredRadio->isChecked()
                                         ? selectedFilterName()
                                         : QStringLiteral("all")));
}

void PacketCaptureDialog::saveCapture()
{
    QString directory = m_saveDirectoryEdit->text().trimmed();
    if (directory.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("保存 PCAP"), QStringLiteral("请选择保存目录。"));
        return;
    }
    QString fileName = m_fileNameEdit->text().trimmed();
    if (fileName.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("保存 PCAP"), QStringLiteral("请输入文件名。"));
        return;
    }
    if (!fileName.endsWith(QStringLiteral(".pcap"), Qt::CaseInsensitive)) {
        fileName += QStringLiteral(".pcap");
        m_fileNameEdit->setText(fileName);
    }

    const QString outputPath = QDir(directory).filePath(fileName);
    if (QFileInfo::exists(outputPath) &&
        QMessageBox::question(this,
                              QStringLiteral("覆盖 PCAP 文件"),
                              QStringLiteral("文件已存在，是否覆盖？\n%1")
                                  .arg(QDir::toNativeSeparators(outputPath))) != QMessageBox::Yes) {
        return;
    }

    pcap_t* deadCapture = pcap_open_dead(m_dataLinkType, 65535);
    const QByteArray encodedOutputPath = outputPath.toLocal8Bit();
    pcap_dumper_t* dumper = pcap_dump_open(deadCapture, encodedOutputPath.constData());
    if (!dumper) {
        const QString error = QString::fromLocal8Bit(pcap_geterr(deadCapture));
        pcap_close(deadCapture);
        QMessageBox::warning(this, QStringLiteral("保存 PCAP"), QStringLiteral("创建 PCAP 文件失败：%1").arg(error));
        return;
    }

    int savedCount = 0;
    for (const PacketRow& packet : m_packets) {
        if (m_saveFilteredRadio->isChecked() && !matchesFilter(packet)) {
            continue;
        }
        pcap_pkthdr header{};
        header.ts.tv_sec = long(packet.timestampSec);
        header.ts.tv_usec = long(packet.timestampUsec);
        header.caplen = bpf_u_int32(packet.capturedLength);
        header.len = bpf_u_int32(packet.originalLength);
        pcap_dump(reinterpret_cast<unsigned char*>(dumper),
                  &header,
                  reinterpret_cast<const unsigned char*>(packet.rawData.constData()));
        ++savedCount;
    }
    pcap_dump_close(dumper);
    pcap_close(deadCapture);

    QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
    settings.setValue(QStringLiteral("save/lastPcapCaptureDir"), directory);
    m_statusLabel->setStyleSheet(QStringLiteral("color: palette(mid);"));
    m_statusLabel->setText(QStringLiteral("已保存 %1 个数据包至 %2")
                               .arg(savedCount)
                               .arg(QDir::toNativeSeparators(outputPath)));
    QMessageBox::information(this,
                             QStringLiteral("保存成功"),
                             QStringLiteral("已成功保存 %1 个数据包。\n%2")
                                 .arg(savedCount)
                                 .arg(QDir::toNativeSeparators(outputPath)));
}

void PacketCaptureDialog::finishCapture(const QString& message, bool failed)
{
    if (m_captureThread.joinable()) {
        m_captureThread.join();
    }
    m_captureActive = false;
    updateCaptureControls();
    m_statusLabel->setStyleSheet(failed
        ? QStringLiteral("color: #dc5050;")
        : QStringLiteral("color: palette(mid);"));
    m_statusLabel->setText(message);
}

void PacketCaptureDialog::updateCaptureControls()
{
    m_interfaceCombo->setEnabled(!m_captureActive);
    m_startButton->setEnabled(!m_captureActive && m_interfaceCombo->currentIndex() >= 0);
    m_stopButton->setEnabled(m_captureActive);
    m_saveButton->setEnabled(!m_captureActive && !m_packets.isEmpty());
    m_saveDirectoryEdit->setEnabled(!m_captureActive);
    m_fileNameEdit->setEnabled(!m_captureActive);
    m_saveAllRadio->setEnabled(!m_captureActive);
    m_saveFilteredRadio->setEnabled(!m_captureActive);
}

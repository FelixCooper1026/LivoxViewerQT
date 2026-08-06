#include "dialogs/PacketCaptureDialog.h"

#include "ThemeIconUtils.h"
#include "PushMsgParser.h"
#include "dialogs/DialogWindowUtils.h"
#ifdef Q_OS_LINUX
#include "helpers/PacketCaptureProtocol.h"
#endif

#include <QAbstractItemView>
#include <QAbstractTableModel>
#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDesktopServices>
#include <QDialogButtonBox>
#include <QDir>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QGridLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QHostAddress>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QRadioButton>
#include <QRegularExpression>
#include <QScrollBar>
#include <QScopedValueRollback>
#include <QSettings>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QStandardPaths>
#include <QTableView>
#include <QTimer>
#include <QToolButton>
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

#ifdef Q_OS_LINUX
bool readProcessBytes(QProcess& process,
                      char* destination,
                      qint64 size,
                      const std::atomic_bool& stopRequested,
                      bool& stopSent)
{
    qint64 bytesRead = 0;
    while (bytesRead < size) {
        if (stopRequested.load() && !stopSent) {
            process.write("stop\n");
            process.waitForBytesWritten(25);
            stopSent = true;
        }
        const QByteArray data = process.read(size - bytesRead);
        if (!data.isEmpty()) {
            std::memcpy(destination + bytesRead, data.constData(), std::size_t(data.size()));
            bytesRead += data.size();
            continue;
        }
        if (!process.waitForReadyRead(25) && process.state() == QProcess::NotRunning) {
            break;
        }
    }
    return bytesRead == size;
}
#endif

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

QString formatByteCount(quint64 bytes)
{
    constexpr double kKilobyte = 1024.0;
    constexpr double kMegabyte = kKilobyte * 1024.0;
    constexpr double kGigabyte = kMegabyte * 1024.0;
    if (bytes >= quint64(kGigabyte)) {
        return QStringLiteral("%1 GB").arg(double(bytes) / kGigabyte, 0, 'f', 1);
    }
    if (bytes >= quint64(kMegabyte)) {
        return QStringLiteral("%1 MB").arg(double(bytes) / kMegabyte, 0, 'f', 1);
    }
    if (bytes >= quint64(kKilobyte)) {
        return QStringLiteral("%1 KB").arg(double(bytes) / kKilobyte, 0, 'f', 1);
    }
    return QStringLiteral("%1 B").arg(bytes);
}

QString formatElapsedTime(qint64 milliseconds)
{
    const qint64 totalSeconds = milliseconds / 1000;
    const qint64 hours = totalSeconds / 3600;
    const qint64 minutes = (totalSeconds % 3600) / 60;
    const qint64 seconds = totalSeconds % 60;
    return QStringLiteral("%1:%2:%3")
        .arg(hours, 2, 10, QLatin1Char('0'))
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 2, 10, QLatin1Char('0'));
}

} // namespace

class PacketTableModel : public QAbstractTableModel
{
public:
    PacketTableModel(PacketCaptureDialog* dialog,
                     QVector<PacketCaptureDialog::PacketRow>* packets)
        : QAbstractTableModel(dialog)
        , m_dialog(dialog)
        , m_packets(packets)
    {
    }

    int rowCount(const QModelIndex& parent = QModelIndex()) const override
    {
        return parent.isValid() ? 0 : m_visiblePacketIndexes.size();
    }

    int columnCount(const QModelIndex& parent = QModelIndex()) const override
    {
        return parent.isValid() ? 0 : 8;
    }

    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override
    {
        if (!index.isValid()) {
            return {};
        }

        const PacketCaptureDialog::PacketRow& packet =
            m_packets->at(m_visiblePacketIndexes.at(index.row()));
        if (role == Qt::TextAlignmentRole) {
            return index.column() == 0 || index.column() == 1 ||
                       index.column() == 2 || index.column() == 3 ||
                       index.column() == 4 || index.column() == 5
                ? QVariant::fromValue(Qt::AlignCenter)
                : QVariant::fromValue(Qt::AlignLeft | Qt::AlignVCenter);
        }
        if (role == Qt::ForegroundRole && index.column() == 4) {
            if (packet.protocol == QStringLiteral("PTP")) {
                return m_dialog->palette().color(QPalette::Highlight);
            }
            if (packet.protocol == QStringLiteral("ARP")) {
                return QColor(QStringLiteral("#d18b28"));
            }
            if (packet.protocol.startsWith(QStringLiteral("TCP"))) {
                return m_dialog->palette().color(QPalette::Link);
            }
        }
        if (role == Qt::UserRole && index.column() == 0) {
            return QVariant::fromValue<qulonglong>(packet.number);
        }
        if (role != Qt::DisplayRole && role != Qt::ToolTipRole) {
            return {};
        }
        if (role == Qt::ToolTipRole && index.column() == 7 && !packet.decodedDetails.isEmpty()) {
            return packet.decodedDetails;
        }

        switch (index.column()) {
        case 0: return QString::number(packet.number);
        case 1: return QString::number(packet.relativeTimeSec, 'f', 6);
        case 2: return packet.source;
        case 3: return packet.destination;
        case 4: return packet.protocol;
        case 5: return QString::number(packet.length);
        case 6: return packet.ports;
        case 7: return packet.decodedSummary.isEmpty() ? packet.payloadHex : packet.decodedSummary;
        default: return {};
        }
    }

    QVariant headerData(int section,
                        Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override
    {
        if (orientation != Qt::Horizontal || role != Qt::DisplayRole) {
            return QAbstractTableModel::headerData(section, orientation, role);
        }
        static const QStringList headers = {
            QStringLiteral("序号"), QStringLiteral("时间"), QStringLiteral("源地址"),
            QStringLiteral("目标地址"), QStringLiteral("协议"), QStringLiteral("长度"),
            QStringLiteral("端口"), QStringLiteral("内容")
        };
        return headers.at(section);
    }

    void clearPackets()
    {
        beginResetModel();
        m_packets->clear();
        m_visiblePacketIndexes.clear();
        endResetModel();
    }

    void appendVisiblePackets(const QVector<int>& packetIndexes)
    {
        if (packetIndexes.isEmpty()) {
            return;
        }
        const int firstRow = m_visiblePacketIndexes.size();
        beginInsertRows(QModelIndex(), firstRow, firstRow + packetIndexes.size() - 1);
        m_visiblePacketIndexes.append(packetIndexes);
        endInsertRows();
    }

    template<typename Predicate>
    void rebuild(Predicate matches)
    {
        QVector<int> visiblePacketIndexes;
        visiblePacketIndexes.reserve(m_packets->size());
        for (int i = 0; i < m_packets->size(); ++i) {
            if (matches(m_packets->at(i))) {
                visiblePacketIndexes.append(i);
            }
        }
        beginResetModel();
        m_visiblePacketIndexes.swap(visiblePacketIndexes);
        endResetModel();
    }

    const PacketCaptureDialog::PacketRow& packetAt(int tableRow) const
    {
        return m_packets->at(m_visiblePacketIndexes.at(tableRow));
    }

    void refreshProtocolColors()
    {
        if (!m_visiblePacketIndexes.isEmpty()) {
            emit dataChanged(index(0, 4), index(rowCount() - 1, 4), {Qt::ForegroundRole});
        }
    }

private:
    PacketCaptureDialog* m_dialog;
    QVector<PacketCaptureDialog::PacketRow>* m_packets;
    QVector<int> m_visiblePacketIndexes;
};

PacketCaptureDialog::PacketCaptureDialog(QWidget* parent)
    : QDialog(parent)
{
    DialogWindowUtils::enableTopLevelWindowControls(this);
    setAttribute(Qt::WA_DeleteOnClose);
    setObjectName(QStringLiteral("packetCaptureDialog"));
    setWindowTitle(QStringLiteral("网络数据抓包 - LivoxViewerQT"));
    resize(1360, 820);
    setMinimumSize(1080, 680);

    QVBoxLayout* root = new QVBoxLayout(this);
    root->setContentsMargins(16, 14, 16, 16);
    root->setSpacing(10);

    QFrame* captureToolbar = new QFrame(this);
    captureToolbar->setObjectName(QStringLiteral("captureToolbar"));
    QHBoxLayout* captureLayout = new QHBoxLayout(captureToolbar);
    captureLayout->setContentsMargins(0, 0, 0, 0);
    captureLayout->setSpacing(8);
    QLabel* interfaceLabel = new QLabel(QStringLiteral("网卡"), captureToolbar);
    interfaceLabel->setFixedWidth(40);
    captureLayout->addWidget(interfaceLabel);
    m_interfaceCombo = new QComboBox(captureToolbar);
    m_interfaceCombo->setMinimumWidth(0);
    m_interfaceCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_interfaceCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    m_interfaceCombo->setToolTip(QStringLiteral("选择用于网络抓包的网络接口"));
    captureLayout->addWidget(m_interfaceCombo, 1);
    m_startButton = new QPushButton(QStringLiteral("开始抓包"), captureToolbar);
    m_startButton->setObjectName(QStringLiteral("primaryButton"));
    m_startButton->setMinimumWidth(110);
    m_stopButton = new QPushButton(QStringLiteral("停止"), captureToolbar);
    m_stopButton->setObjectName(QStringLiteral("dangerButton"));
    m_stopButton->setMinimumWidth(88);
    m_clearButton = new QPushButton(QStringLiteral("清空"), captureToolbar);
    m_clearButton->setObjectName(QStringLiteral("secondaryButton"));
    m_clearButton->setMinimumWidth(88);
    captureLayout->addWidget(m_startButton);
    captureLayout->addWidget(m_stopButton);
    captureLayout->addWidget(m_clearButton);
    root->addWidget(captureToolbar);

    QFrame* filterCard = new QFrame(this);
    filterCard->setObjectName(QStringLiteral("filterCard"));
    QVBoxLayout* filterCardLayout = new QVBoxLayout(filterCard);
    filterCardLayout->setContentsMargins(10, 4, 0, 4);
    filterCardLayout->setSpacing(8);
    QHBoxLayout* filterChipLayout = new QHBoxLayout();
    filterChipLayout->setSpacing(8);
    auto createFilterChip = [filterCard](const QString& text) {
        QToolButton* button = new QToolButton(filterCard);
        button->setObjectName(QStringLiteral("filterChip"));
        button->setText(text);
        button->setCheckable(true);
        button->setCursor(Qt::PointingHandCursor);
        return button;
    };
    m_broadcastFilter = createFilterChip(QStringLiteral("广播 56000"));
    m_controlFilter = createFilterChip(QStringLiteral("控制 56100"));
    m_pointCloudFilter = createFilterChip(QStringLiteral("点云 56300"));
    m_imuFilter = createFilterChip(QStringLiteral("IMU 56400"));
    m_pushFilter = createFilterChip(QStringLiteral("推送 56200"));
    m_ptpFilter = createFilterChip(QStringLiteral("PTP"));
    m_arpFilter = createFilterChip(QStringLiteral("ARP"));
    for (QToolButton* filter : {m_broadcastFilter, m_controlFilter, m_pointCloudFilter,
                                m_imuFilter, m_pushFilter, m_ptpFilter, m_arpFilter}) {
        filterChipLayout->addWidget(filter);
    }
    m_sourceIpFilter = createFilterChip(QStringLiteral("源 IP"));
    m_sourceIpCombo = new QComboBox(filterCard);
    m_sourceIpCombo->setFixedHeight(32);
    m_sourceIpCombo->setFixedWidth(190);
    m_sourceIpCombo->setVisible(false);
    m_destinationIpFilter = createFilterChip(QStringLiteral("目标 IP"));
    m_destinationIpCombo = new QComboBox(filterCard);
    m_destinationIpCombo->setFixedHeight(32);
    m_destinationIpCombo->setFixedWidth(190);
    m_destinationIpCombo->setVisible(false);
    filterChipLayout->addWidget(m_sourceIpFilter);
    filterChipLayout->addWidget(m_sourceIpCombo);
    filterChipLayout->addWidget(m_destinationIpFilter);
    filterChipLayout->addWidget(m_destinationIpCombo);
    filterChipLayout->addStretch();
    filterCardLayout->addLayout(filterChipLayout);
    root->addWidget(filterCard);

    QFrame* statisticsBar = new QFrame(this);
    statisticsBar->setObjectName(QStringLiteral("statisticsBar"));
    QHBoxLayout* statisticsLayout = new QHBoxLayout(statisticsBar);
    statisticsLayout->setContentsMargins(12, 8, 12, 8);
    auto addMetric = [statisticsBar, statisticsLayout](const QString& iconPath, QLabel*& valueLabel) {
        QWidget* metric = new QWidget(statisticsBar);
        QHBoxLayout* metricLayout = new QHBoxLayout(metric);
        metricLayout->setContentsMargins(0, 0, 0, 0);
        metricLayout->setSpacing(8);
        QLabel* iconLabel = new QLabel(metric);
        ThemeIconUtils::setThemedSvgPixmap(iconLabel, iconPath, QSize(18, 18));
        valueLabel = new QLabel(metric);
        metricLayout->addStretch();
        metricLayout->addWidget(iconLabel);
        metricLayout->addWidget(valueLabel);
        metricLayout->addStretch();
        statisticsLayout->addWidget(metric, 1);
    };
    auto addMetricSeparator = [statisticsBar, statisticsLayout]() {
        QFrame* separator = new QFrame(statisticsBar);
        separator->setObjectName(QStringLiteral("metricSeparator"));
        separator->setFixedSize(1, 24);
        statisticsLayout->addWidget(separator);
    };
    addMetric(QStringLiteral(":/icons/packet_stat_captured.svg"), m_totalPacketsLabel);
    addMetricSeparator();
    addMetric(QStringLiteral(":/icons/eye.svg"), m_visiblePacketsLabel);
    addMetricSeparator();
    addMetric(QStringLiteral(":/icons/packet_stat_bytes.svg"), m_totalBytesLabel);
    addMetricSeparator();
    addMetric(QStringLiteral(":/icons/packet_stat_elapsed.svg"), m_elapsedTimeLabel);
    root->addWidget(statisticsBar);

    m_packetTable = new QTableView(this);
    m_packetTable->setObjectName(QStringLiteral("packetTable"));
    m_packetModel = new PacketTableModel(this, &m_packets);
    m_packetTable->setModel(m_packetModel);
    m_packetTable->verticalHeader()->setVisible(false);
    m_packetTable->verticalHeader()->setDefaultSectionSize(30);
    m_packetTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_packetTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_packetTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_packetTable->setVerticalScrollMode(QAbstractItemView::ScrollPerItem);
    m_packetTable->setAlternatingRowColors(true);
    m_packetTable->setShowGrid(false);
    QHeaderView* packetHeader = m_packetTable->horizontalHeader();
    packetHeader->setDefaultAlignment(Qt::AlignCenter);
    packetHeader->setSectionResizeMode(QHeaderView::Interactive);
    packetHeader->setSectionResizeMode(7, QHeaderView::Stretch);
    const int columnWidths[] = {72, 110, 170, 170, 150, 76, 130};
    for (int column = 0; column < 7; ++column) {
        m_packetTable->setColumnWidth(column, columnWidths[column]);
    }
    m_emptyStateLabel = new QLabel(
        QStringLiteral("暂无数据，点击“开始抓包”后显示数据包"), m_packetTable->viewport());
    m_emptyStateLabel->setAlignment(Qt::AlignCenter);
    m_emptyStateLabel->setStyleSheet(QStringLiteral("color: palette(mid); font-size: 14px;"));
    m_emptyStateLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    QVBoxLayout* emptyStateLayout = new QVBoxLayout(m_packetTable->viewport());
    emptyStateLayout->addStretch();
    emptyStateLayout->addWidget(m_emptyStateLabel, 0, Qt::AlignCenter);
    emptyStateLayout->addStretch();
    root->addWidget(m_packetTable, 1);
    root->addSpacing(12);

    QFrame* saveCard = new QFrame(this);
    saveCard->setObjectName(QStringLiteral("saveCard"));
    QGridLayout* saveLayout = new QGridLayout(saveCard);
    saveLayout->setContentsMargins(10, 4, 0, 0);
    saveLayout->setHorizontalSpacing(8);
    saveLayout->setVerticalSpacing(8);
    saveLayout->addWidget(new QLabel(QStringLiteral("保存到"), saveCard), 0, 0);
    m_saveDirectoryEdit = new QLineEdit(initialSaveDirectory(), saveCard);
    m_saveDirectoryEdit->setObjectName(QStringLiteral("packetCaptureLineEdit"));
    m_saveDirectoryEdit->setFixedHeight(32);
    m_saveDirectoryEdit->setTextMargins(0, 0, 32, 0);
    QToolButton* browseButton = new QToolButton(m_saveDirectoryEdit);
    browseButton->setObjectName(QStringLiteral("inlineBrowseButton"));
    ThemeIconUtils::setThemedSvgIcon(browseButton, QStringLiteral(":/icons/convert_browse_folder.svg"));
    browseButton->setIconSize(QSize(18, 18));
    browseButton->setToolTip(QStringLiteral("浏览"));
    browseButton->setCursor(Qt::PointingHandCursor);
    browseButton->setFixedSize(28, 28);
    QHBoxLayout* browseLayout = new QHBoxLayout(m_saveDirectoryEdit);
    browseLayout->setContentsMargins(0, 2, 2, 2);
    browseLayout->addStretch();
    browseLayout->addWidget(browseButton);
    QPushButton* openDirectoryButton = new QPushButton(QStringLiteral("打开目录"), saveCard);
    openDirectoryButton->setObjectName(QStringLiteral("secondaryButton"));
    openDirectoryButton->setFixedHeight(32);
    saveLayout->addWidget(m_saveDirectoryEdit, 0, 1, 1, 6);
    saveLayout->addWidget(openDirectoryButton, 0, 7);
    saveLayout->addWidget(new QLabel(QStringLiteral("文件名"), saveCard), 1, 0);
    m_fileNameEdit = new QLineEdit(saveCard);
    m_fileNameEdit->setObjectName(QStringLiteral("packetCaptureLineEdit"));
    m_fileNameEdit->setFixedHeight(32);
    saveLayout->addWidget(m_fileNameEdit, 1, 1, 1, 7);
    m_saveAllRadio = new QRadioButton(QStringLiteral("保存所有包"), saveCard);
    m_saveFilteredRadio = new QRadioButton(QStringLiteral("仅保存当前筛选"), saveCard);
    m_saveAllRadio->setChecked(true);
    m_saveButton = new QPushButton(QStringLiteral("保存 PCAP"), saveCard);
    m_saveButton->setObjectName(QStringLiteral("primaryButton"));
    m_saveButton->setMinimumWidth(150);
    saveLayout->addWidget(m_saveAllRadio, 2, 1);
    saveLayout->addWidget(m_saveFilteredRadio, 2, 2);
    saveLayout->addWidget(m_saveButton, 2, 7);
    saveLayout->setColumnStretch(5, 1);
    root->addWidget(saveCard);

    connect(m_interfaceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (index >= 0 && m_interfaceChangedHandler) {
            m_interfaceChangedHandler(m_interfaceCombo->itemData(index).toString());
        }
    });

    const QList<QToolButton*> typeFilters = {
        m_broadcastFilter, m_controlFilter, m_pointCloudFilter, m_imuFilter,
        m_pushFilter, m_ptpFilter, m_arpFilter
    };
    for (QToolButton* filter : typeFilters) {
        connect(filter, &QToolButton::toggled, this, [this]() { rebuildTable(); });
    }
    connect(m_sourceIpFilter, &QToolButton::toggled, this, [this](bool checked) {
        m_sourceIpCombo->setVisible(checked);
        rebuildTable();
    });
    connect(m_destinationIpFilter, &QToolButton::toggled, this, [this](bool checked) {
        m_destinationIpCombo->setVisible(checked);
        rebuildTable();
    });
    connect(m_sourceIpCombo, &QComboBox::currentTextChanged, this, [this]() { rebuildTable(); });
    connect(m_destinationIpCombo, &QComboBox::currentTextChanged, this, [this]() { rebuildTable(); });
    connect(m_fileNameEdit, &QLineEdit::textEdited, this, [this]() { m_fileNameCustomized = true; });
    connect(m_saveAllRadio, &QRadioButton::toggled, this, [this]() {
        if (!m_fileNameCustomized) {
            updateDefaultFileName();
        }
    });
    connect(m_startButton, &QPushButton::clicked, this, [this]() { startCapture(); });
    connect(m_stopButton, &QPushButton::clicked, this, [this]() { stopCapture(); });
    connect(m_clearButton, &QPushButton::clicked, this, [this]() {
        m_packetModel->clearPackets();
        clearCapturedAddresses();
        m_totalBytes = 0;
        updateStatistics();
        updateEmptyState();
        updateCaptureControls();
    });
    connect(browseButton, &QToolButton::clicked, this, [this]() {
        const QString directory = selectFolder(
            this, m_saveDirectoryEdit->text().trimmed(), QStringLiteral("选择保存目录"));
        if (!directory.isEmpty()) {
            m_saveDirectoryEdit->setText(directory);
            QSettings settings(QStringLiteral("Livox"), QStringLiteral("LivoxViewerQT"));
            settings.setValue(QStringLiteral("save/lastPcapCaptureDir"), directory);
        }
    });
    connect(m_saveButton, &QPushButton::clicked, this, [this]() { saveCapture(); });
    connect(openDirectoryButton, &QPushButton::clicked, this, [this]() {
        QDesktopServices::openUrl(QUrl::fromLocalFile(m_saveDirectoryEdit->text().trimmed()));
    });
    connect(m_packetTable, &QTableView::doubleClicked, this, [this](const QModelIndex& index) {
        showPacketDetails(index.row());
    });

    m_elapsedTimer = new QTimer(this);
    connect(m_elapsedTimer, &QTimer::timeout, this, [this]() { updateStatistics(); });

    m_captureTimestamp = QDateTime::currentDateTime();
    updateDefaultFileName();
    refreshTheme();
    updateStatistics();
    updateEmptyState();
    updateCaptureControls();
}

PacketCaptureDialog::~PacketCaptureDialog()
{
    m_stopRequested.store(true);
#ifdef Q_OS_LINUX
    {
        std::lock_guard<std::mutex> lock(m_privilegedCaptureMutex);
        m_privilegedWorkerShutdown = true;
    }
    m_privilegedCaptureCondition.notify_one();
#endif
    if (m_captureThread.joinable()) {
        m_captureThread.join();
    }
}

void PacketCaptureDialog::changeEvent(QEvent* event)
{
    QDialog::changeEvent(event);
    if (event->type() == QEvent::PaletteChange ||
        event->type() == QEvent::ApplicationPaletteChange) {
        refreshTheme();
    }
}

void PacketCaptureDialog::refreshTheme()
{
    if (m_refreshingTheme) {
        return;
    }
    QScopedValueRollback<bool> refreshingTheme(m_refreshingTheme, true);

    const bool darkTheme = palette().color(QPalette::Window).lightness() < 128;
    const QString borderColor = darkTheme ? QStringLiteral("#5f6268") : QStringLiteral("#dfe4ec");
    const QString surfaceColor = darkTheme ? QStringLiteral("#292b2f") : QStringLiteral("#fafbfd");
    const QString hoverColor = darkTheme ? QStringLiteral("#35383d") : QStringLiteral("#f3f5f8");
    const QString pressedColor = darkTheme ? QStringLiteral("#45484e") : QStringLiteral("#e8ebf0");
    const QString disabledColor = darkTheme ? QStringLiteral("#44474c") : QStringLiteral("#d8dde5");
    const QColor highlight = palette().color(QPalette::Highlight);
    const QString highlightHover = (darkTheme ? highlight.lighter(112) : highlight.darker(105)).name();
    const QString highlightPressed = (darkTheme ? highlight.darker(112) : highlight.darker(115)).name();

    setStyleSheet(QStringLiteral(R"(
        QDialog#packetCaptureDialog {
            background: palette(window);
        }
        QFrame#statisticsBar {
            border: 1px solid %1;
            border-radius: 4px;
            background: %2;
        }
        QFrame#metricSeparator {
            border: none;
            background: %1;
        }
        QLineEdit#packetCaptureLineEdit {
            min-height: 32px;
            border: 1px solid %1;
            border-radius: 4px;
            background: palette(base);
            color: palette(text);
            padding: 0 10px;
        }
        QLineEdit#packetCaptureLineEdit:hover {
            border-color: palette(mid);
        }
        QLineEdit#packetCaptureLineEdit:focus {
            border-color: palette(highlight);
        }
        QToolButton#inlineBrowseButton {
            border: none;
            border-radius: 3px;
            background: transparent;
        }
        QToolButton#inlineBrowseButton:hover {
            background: %3;
        }
        QToolButton#inlineBrowseButton:pressed {
            background: %4;
        }
        QPushButton#secondaryButton {
            min-height: 32px;
            border: 1px solid %1;
            border-radius: 4px;
            background: palette(button);
            color: palette(button-text);
            padding: 0 16px;
        }
        QPushButton#secondaryButton:hover {
            border-color: palette(mid);
            background: %3;
        }
        QPushButton#secondaryButton:pressed {
            border-color: palette(mid);
            background: %4;
        }
        QPushButton#secondaryButton:disabled {
            border-color: %1;
            background: %5;
            color: palette(mid);
        }
        QPushButton#primaryButton {
            min-height: 32px;
            padding: 0 18px;
            border: 1px solid palette(highlight);
            border-radius: 4px;
            background: palette(highlight);
            color: palette(highlighted-text);
        }
        QPushButton#primaryButton:hover {
            border-color: %6;
            background: %6;
        }
        QPushButton#primaryButton:pressed {
            border-color: %7;
            background: %7;
        }
        QPushButton#primaryButton:disabled {
            border-color: %5;
            background: %5;
            color: palette(mid);
        }
        QPushButton#dangerButton {
            min-height: 32px;
            padding: 0 16px;
            border: 1px solid #b95855;
            border-radius: 4px;
            background: palette(button);
            color: #d86a66;
        }
        QPushButton#dangerButton:hover {
            background: %3;
            border-color: #d86a66;
        }
        QPushButton#dangerButton:pressed {
            background: %4;
            border-color: #a94442;
        }
        QPushButton#dangerButton:disabled {
            border-color: %1;
            background: %5;
            color: palette(mid);
        }
        QToolButton#filterChip {
            min-height: 28px;
            margin-left: 1px;
            margin-right: 1px;
            padding: 0 12px;
            border: 1px solid %1;
            border-radius: 15px;
            background: palette(base);
            color: palette(button-text);
        }
        QToolButton#filterChip:hover {
            border-color: palette(mid);
            background: %3;
        }
        QToolButton#filterChip:pressed {
            border-color: palette(mid);
            background: %4;
        }
        QToolButton#filterChip:checked {
            border-color: palette(highlight);
            background: palette(highlight);
            color: palette(highlighted-text);
        }
        QToolButton#filterChip:checked:hover {
            border-color: %6;
            background: %6;
        }
        QToolButton#filterChip:checked:pressed {
            border-color: %7;
            background: %7;
        }
        QTableView#packetTable {
            border: 1px solid %1;
            border-radius: 4px;
            gridline-color: %1;
            background: palette(base);
            color: palette(text);
            alternate-background-color: palette(alternate-base);
        }
        QTableView#packetTable QHeaderView::section {
            min-height: 34px;
            border: none;
            border-right: 1px solid %1;
            border-bottom: 1px solid %1;
            background: %2;
            color: palette(window-text);
            padding: 0 8px;
        }
    )").arg(borderColor,
             surfaceColor,
             hoverColor,
             pressedColor,
             disabledColor,
             highlightHover,
             highlightPressed));

    ThemeIconUtils::refreshObject(this);
    for (QWidget* child : findChildren<QWidget*>()) {
        ThemeIconUtils::refreshObject(child);
    }
    m_packetModel->refreshProtocolColors();
}

void PacketCaptureDialog::setInterfaces(
    const QList<NetworkInterfaceService::NetworkInterfaceInfo>& interfaces,
    const QString& selectedSystemName)
{
    if (m_captureActive) {
        return;
    }

    const QString currentSystemName = m_interfaceCombo->currentData().toString();
    const QString targetSystemName = selectedSystemName.isEmpty()
        ? currentSystemName
        : selectedSystemName;
    QSignalBlocker blocker(m_interfaceCombo);
    m_interfaceCombo->clear();
    int selectedIndex = -1;
    for (int i = 0; i < interfaces.size(); ++i) {
        const NetworkInterfaceService::NetworkInterfaceInfo& interface = interfaces.at(i);
        const QString address = interface.ipv4.isEmpty()
            ? QStringLiteral("未活动 / 无 IPv4")
            : interface.ipv4;
        m_interfaceCombo->addItem(
            QStringLiteral("%1 - %2").arg(interface.displayName, address),
            interface.systemName);
        m_interfaceCombo->setItemData(i, interface.ipv4, Qt::UserRole + 1);
        if (interface.systemName == targetSystemName) {
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

    m_packetModel->clearPackets();
    clearCapturedAddresses();
    m_totalBytes = 0;
    m_dataLinkType = 0;
    m_captureTimestamp = QDateTime::currentDateTime();
    m_fileNameCustomized = false;
    updateDefaultFileName();
    m_stopRequested.store(false);
    m_captureActive = true;
    m_captureElapsed.start();
    m_elapsedTimer->start(250);
    updateStatistics();
    updateEmptyState();
    updateCaptureControls();

    const QString systemName = m_interfaceCombo->itemData(interfaceIndex).toString();
    const QString ipv4 = m_interfaceCombo->itemData(interfaceIndex, Qt::UserRole + 1).toString();
#ifdef Q_OS_LINUX
    if (m_usePrivilegedCaptureHelper) {
        {
            std::lock_guard<std::mutex> lock(m_privilegedCaptureMutex);
            m_privilegedSystemName = systemName;
            m_privilegedIpv4 = ipv4;
            m_privilegedCapturePending = true;
        }
        if (m_captureThread.joinable() && !m_privilegedWorkerRunning.load()) {
            m_captureThread.join();
        }
        if (!m_captureThread.joinable()) {
            m_captureThread = std::thread([this]() { privilegedCaptureWorker(); });
        }
        m_privilegedCaptureCondition.notify_one();
        return;
    }
#endif
    m_captureThread = std::thread([this, systemName, ipv4]() { captureLoop(systemName, ipv4); });
}

void PacketCaptureDialog::stopCapture()
{
    if (!m_captureActive) {
        return;
    }
    m_stopRequested.store(true);
    m_stopButton->setEnabled(false);
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
    auto postFinished = [this](QString message, bool failed, bool permissionDenied = false) {
        QMetaObject::invokeMethod(this, [this, message = std::move(message), failed, permissionDenied]() {
            finishCapture(message, failed, permissionDenied);
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
    pcap_t* capture = pcap_create(encodedDeviceName.constData(), errorBuffer);
    if (!capture) {
        postFinished(QStringLiteral("打开抓包网卡失败：%1").arg(QString::fromLocal8Bit(errorBuffer)), true);
        return;
    }
    pcap_set_snaplen(capture, 65535);
    pcap_set_promisc(capture, 1);
    pcap_set_timeout(capture, 250);
    const int activateResult = pcap_activate(capture);
    if (activateResult < 0) {
        const QString error = QString::fromLocal8Bit(pcap_geterr(capture));
        const bool permissionDenied = activateResult == PCAP_ERROR_PERM_DENIED ||
                                      activateResult == PCAP_ERROR_PROMISC_PERM_DENIED;
        pcap_close(capture);
        postFinished(QStringLiteral("打开抓包网卡失败：%1").arg(error), true, permissionDenied);
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

#ifdef Q_OS_LINUX
void PacketCaptureDialog::privilegedCaptureWorker()
{
    m_privilegedWorkerRunning.store(true);
    auto postFinished = [this](QString message, bool failed) {
        QMetaObject::invokeMethod(this, [this, message = std::move(message), failed]() {
            finishCapture(message, failed);
        }, Qt::QueuedConnection);
    };

    const QString pkexecPath = QStandardPaths::findExecutable(
        QStringLiteral("pkexec"), {QStringLiteral("/usr/bin"), QStringLiteral("/bin")});
    const QString helperPath = QCoreApplication::applicationDirPath() +
        QStringLiteral("/LivoxPacketCaptureHelper");
    QStringList helperArguments{helperPath};
    const QString appImagePath = qEnvironmentVariable("APPIMAGE");
    if (!appImagePath.isEmpty()) {
        helperArguments = {
            QStringLiteral("/usr/bin/env"),
            QStringLiteral("APPIMAGE_EXTRACT_AND_RUN=1"),
            appImagePath,
            QStringLiteral("--livox-packet-capture-helper")
        };
    }
    QProcess helper;
    helper.start(pkexecPath, helperArguments);
    if (!helper.waitForStarted()) {
        postFinished(QStringLiteral("无法启动抓包权限辅助程序。"), true);
        m_privilegedWorkerRunning.store(false);
        return;
    }

    while (true) {
        QString systemName;
        QString ipv4;
        {
            std::unique_lock<std::mutex> lock(m_privilegedCaptureMutex);
            m_privilegedCaptureCondition.wait(lock, [this]() {
                return m_privilegedCapturePending || m_privilegedWorkerShutdown;
            });
            if (m_privilegedWorkerShutdown) {
                break;
            }
            systemName = m_privilegedSystemName;
            ipv4 = m_privilegedIpv4;
            m_privilegedCapturePending = false;
        }

        QString deviceError;
        const QString deviceName = captureDeviceName(systemName, ipv4, &deviceError);
        if (deviceName.isEmpty()) {
            postFinished(deviceError, true);
            continue;
        }
        capturePrivilegedSession(helper, deviceName);
        if (helper.state() == QProcess::NotRunning) {
            break;
        }
    }

    helper.write("quit\n");
    helper.waitForBytesWritten(250);
    helper.closeWriteChannel();
    helper.waitForFinished(1000);
    m_privilegedWorkerRunning.store(false);
}

void PacketCaptureDialog::capturePrivilegedSession(QProcess& helper, const QString& deviceName)
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

    helper.write("start " + deviceName.toLocal8Bit() + '\n');
    helper.waitForBytesWritten(250);
    bool stopSent = false;
    PacketCaptureProtocol::StreamHeader streamHeader;
    if (!readProcessBytes(helper,
                          reinterpret_cast<char*>(&streamHeader),
                          sizeof(streamHeader),
                          m_stopRequested,
                          stopSent)) {
        if (m_stopRequested.load()) {
            postFinished(QStringLiteral("抓包已停止，可手动保存 PCAP"), false);
        } else {
            const QString error = QString::fromLocal8Bit(helper.readAllStandardError()).trimmed();
            postFinished(error.isEmpty() ? QStringLiteral("管理员验证未完成。") : error, true);
        }
        return;
    }
    if (streamHeader.magic != PacketCaptureProtocol::kStreamMagic) {
        postFinished(QStringLiteral("抓包权限辅助程序协议不匹配。"), true);
        return;
    }
    if (streamHeader.dataLinkType < 0) {
        const QString error = QString::fromLocal8Bit(helper.readAllStandardError()).trimmed();
        postFinished(QStringLiteral("打开抓包网卡失败：%1").arg(error), true);
        return;
    }

    QVector<PacketRow> batch;
    batch.reserve(kPacketBatchSize);
    auto lastBatchPost = std::chrono::steady_clock::now();
    quint64 packetNumber = 0;
    double firstTimestamp = -1.0;
    QString captureError;

    while (true) {
        PacketCaptureProtocol::PacketHeader packetHeader;
        if (!readProcessBytes(helper,
                              reinterpret_cast<char*>(&packetHeader),
                              sizeof(packetHeader),
                              m_stopRequested,
                              stopSent)) {
            break;
        }
        if (packetHeader.capturedLength == PacketCaptureProtocol::kCaptureStopped) {
            break;
        }
        QByteArray data(int(packetHeader.capturedLength), '\0');
        if (!readProcessBytes(helper, data.data(), data.size(), m_stopRequested, stopSent)) {
            break;
        }

        const double timestamp = double(packetHeader.timestampSec) +
                                 double(packetHeader.timestampUsec) / 1000000.0;
        if (firstTimestamp < 0.0) {
            firstTimestamp = timestamp;
        }
        ++packetNumber;
        batch.append(decodePacket(
            reinterpret_cast<const unsigned char*>(data.constData()),
            int(packetHeader.capturedLength),
            int(packetHeader.originalLength),
            streamHeader.dataLinkType,
            packetNumber,
            timestamp - firstTimestamp,
            packetHeader.timestampSec,
            packetHeader.timestampUsec));
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
    if (!m_stopRequested.load()) {
        captureError = QString::fromLocal8Bit(helper.readAllStandardError()).trimmed();
    }

    if (!captureError.isEmpty()) {
        postFinished(QStringLiteral("抓包失败：%1").arg(captureError), true);
    } else {
        postFinished(QStringLiteral("抓包已停止，可手动保存 PCAP"), false);
    }
}
#endif

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
            const PushMsgParser::ProtocolDecodeResult decoded =
                PushMsgParser::decodeProtocolPacket(sourcePort,
                                                    destinationPort,
                                                    data + dataOffset,
                                                    size_t(capturedLength - dataOffset));
            if (decoded.valid) {
                packet.protocol = decoded.protocol;
                packet.decodedSummary = decoded.summary;
                packet.decodedDetails = decoded.details;
            }
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
    {
        QSignalBlocker sourceBlocker(m_sourceIpCombo);
        QSignalBlocker destinationBlocker(m_destinationIpCombo);
        for (const PacketRow& packet : packets) {
            const QHostAddress sourceAddress(packet.source);
            if (!sourceAddress.isNull() && !m_sourceAddresses.contains(packet.source)) {
                m_sourceAddresses.insert(packet.source);
                m_sourceIpCombo->addItem(packet.source);
            }
            const QHostAddress destinationAddress(packet.destination);
            if (!destinationAddress.isNull() && !m_destinationAddresses.contains(packet.destination)) {
                m_destinationAddresses.insert(packet.destination);
                m_destinationIpCombo->addItem(packet.destination);
            }
        }
    }
    const FilterState filter = currentFilterState();
    QVector<int> visiblePacketIndexes;
    visiblePacketIndexes.reserve(packets.size());
    for (PacketRow& packet : packets) {
        if (m_dataLinkType == 0) {
            m_dataLinkType = packet.dataLinkType;
        }
        m_totalBytes += quint64(packet.originalLength);
        m_packets.append(std::move(packet));
        const int packetIndex = m_packets.size() - 1;
        if (matchesFilter(m_packets.at(packetIndex), filter)) {
            visiblePacketIndexes.append(packetIndex);
        }
    }
    m_packetModel->appendVisiblePackets(visiblePacketIndexes);
    if (scrollToBottom) {
        m_packetTable->scrollToBottom();
    }
    updateStatistics();
    updateEmptyState();
    updateCaptureControls();
}

void PacketCaptureDialog::rebuildTable()
{
    const FilterState filter = currentFilterState();
    m_packetModel->rebuild([filter](const PacketRow& packet) {
        return matchesFilter(packet, filter);
    });
    updateStatistics();
    updateEmptyState();
    if (!m_fileNameCustomized) {
        updateDefaultFileName();
    }
}

void PacketCaptureDialog::updateStatistics()
{
    m_totalPacketsLabel->setText(QStringLiteral("已捕获  %L1").arg(m_packets.size()));
    m_visiblePacketsLabel->setText(QStringLiteral("当前显示  %L1").arg(m_packetModel->rowCount()));
    m_totalBytesLabel->setText(QStringLiteral("数据量  %1").arg(formatByteCount(m_totalBytes)));
    const qint64 elapsedMilliseconds = m_captureElapsed.isValid() ? m_captureElapsed.elapsed() : 0;
    m_elapsedTimeLabel->setText(
        QStringLiteral("运行时间  %1").arg(formatElapsedTime(elapsedMilliseconds)));
}

void PacketCaptureDialog::updateEmptyState()
{
    const bool empty = m_packetModel->rowCount() == 0;
    m_emptyStateLabel->setVisible(empty);
    if (empty) {
        m_emptyStateLabel->setText(m_packets.isEmpty()
            ? QStringLiteral("暂无数据，点击“开始抓包”后显示数据包")
            : QStringLiteral("当前筛选条件下暂无数据"));
    }
}

void PacketCaptureDialog::showPacketDetails(int tableRow)
{
    QDialog detailsDialog(this);
    detailsDialog.resize(820, 680);
    QVBoxLayout* detailsLayout = new QVBoxLayout(&detailsDialog);
    QLabel* summaryLabel = new QLabel(&detailsDialog);
    summaryLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    QToolButton* previousButton = new QToolButton(&detailsDialog);
    QToolButton* nextButton = new QToolButton(&detailsDialog);
    previousButton->setFixedSize(36, 32);
    nextButton->setFixedSize(36, 32);
    previousButton->setIconSize(QSize(18, 18));
    nextButton->setIconSize(QSize(18, 18));
    previousButton->setToolTip(QStringLiteral("上一包"));
    nextButton->setToolTip(QStringLiteral("下一包"));
    previousButton->setCursor(Qt::PointingHandCursor);
    nextButton->setCursor(Qt::PointingHandCursor);
    ThemeIconUtils::setThemedSvgIcon(previousButton, QStringLiteral(":/icons/playback_previous.svg"));
    ThemeIconUtils::setThemedSvgIcon(nextButton, QStringLiteral(":/icons/playback_next.svg"));
    QLabel* positionLabel = new QLabel(&detailsDialog);
    positionLabel->setAlignment(Qt::AlignCenter);

    QHBoxLayout* navigationLayout = new QHBoxLayout();
    navigationLayout->addStretch();
    navigationLayout->addWidget(previousButton);
    navigationLayout->addWidget(positionLabel);
    navigationLayout->addWidget(nextButton);
    navigationLayout->addStretch();

    QLabel* decodedLabel = new QLabel(QStringLiteral("Livox 协议解析"), &detailsDialog);
    QPlainTextEdit* decodedEdit = new QPlainTextEdit(&detailsDialog);
    decodedEdit->setReadOnly(true);
    decodedEdit->setLineWrapMode(QPlainTextEdit::WidgetWidth);
    QPlainTextEdit* payloadEdit = new QPlainTextEdit(&detailsDialog);
    payloadEdit->setReadOnly(true);
    payloadEdit->setLineWrapMode(QPlainTextEdit::WidgetWidth);
    QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Close, &detailsDialog);
    connect(buttons, &QDialogButtonBox::rejected, &detailsDialog, &QDialog::reject);
    detailsLayout->addWidget(summaryLabel);
    detailsLayout->addLayout(navigationLayout);
    detailsLayout->addWidget(decodedLabel);
    detailsLayout->addWidget(decodedEdit, 2);
    detailsLayout->addWidget(new QLabel(QStringLiteral("内容（Payload Hex）"), &detailsDialog));
    detailsLayout->addWidget(payloadEdit, 1);
    detailsLayout->addWidget(buttons);

    int currentTableRow = tableRow;
    auto updateDetails = [&]() {
        const PacketRow& packet = m_packetModel->packetAt(currentTableRow);
        detailsDialog.setWindowTitle(QStringLiteral("数据包详情 #%1").arg(packet.number));
        summaryLabel->setText(
            QStringLiteral("时间：%1 s    源地址：%2    目标地址：%3\n协议：%4    长度：%5    端口：%6")
                .arg(QString::number(packet.relativeTimeSec, 'f', 6),
                     packet.source,
                     packet.destination,
                     packet.protocol,
                     QString::number(packet.length),
                     packet.ports));
        const bool hasDecodedDetails = !packet.decodedDetails.isEmpty();
        decodedLabel->setVisible(hasDecodedDetails);
        decodedEdit->setVisible(hasDecodedDetails);
        decodedEdit->setPlainText(packet.decodedDetails);
        payloadEdit->setPlainText(packet.payloadHex);
        previousButton->setEnabled(currentTableRow > 0);
        nextButton->setEnabled(currentTableRow + 1 < m_packetModel->rowCount());
        positionLabel->setText(QStringLiteral("%1 / %2")
                                   .arg(currentTableRow + 1)
                                   .arg(m_packetModel->rowCount()));
        m_packetTable->selectRow(currentTableRow);
        m_packetTable->scrollTo(m_packetModel->index(currentTableRow, 0),
                                QAbstractItemView::PositionAtCenter);
    };
    connect(previousButton, &QToolButton::clicked, &detailsDialog, [&]() {
        --currentTableRow;
        updateDetails();
    });
    connect(nextButton, &QToolButton::clicked, &detailsDialog, [&]() {
        ++currentTableRow;
        updateDetails();
    });
    connect(m_packetModel, &QAbstractItemModel::rowsInserted, &detailsDialog, [&]() {
        nextButton->setEnabled(currentTableRow + 1 < m_packetModel->rowCount());
        positionLabel->setText(QStringLiteral("%1 / %2")
                                   .arg(currentTableRow + 1)
                                   .arg(m_packetModel->rowCount()));
    });
    updateDetails();
    detailsDialog.exec();
}

PacketCaptureDialog::FilterState PacketCaptureDialog::currentFilterState() const
{
    return {
        m_broadcastFilter->isChecked(),
        m_controlFilter->isChecked(),
        m_pointCloudFilter->isChecked(),
        m_imuFilter->isChecked(),
        m_pushFilter->isChecked(),
        m_ptpFilter->isChecked(),
        m_arpFilter->isChecked(),
        m_sourceIpFilter->isChecked(),
        m_destinationIpFilter->isChecked(),
        m_sourceIpCombo->currentText(),
        m_destinationIpCombo->currentText()
    };
}

void PacketCaptureDialog::clearCapturedAddresses()
{
    QSignalBlocker sourceBlocker(m_sourceIpCombo);
    QSignalBlocker destinationBlocker(m_destinationIpCombo);
    m_sourceAddresses.clear();
    m_destinationAddresses.clear();
    m_sourceIpCombo->clear();
    m_destinationIpCombo->clear();
}

bool PacketCaptureDialog::matchesFilter(const PacketRow& packet, const FilterState& filter)
{
    const bool hasTypeFilter = filter.broadcast || filter.control || filter.pointCloud ||
                               filter.imu || filter.push || filter.ptp || filter.arp;
    const bool matchesType = !hasTypeFilter ||
                             (filter.broadcast && packet.broadcast) ||
                             (filter.control && packet.control) ||
                             (filter.pointCloud && packet.pointCloud) ||
                             (filter.imu && packet.imu) ||
                             (filter.push && packet.push) ||
                             (filter.ptp && packet.ptp) ||
                             (filter.arp && packet.arp);
    if (!matchesType) {
        return false;
    }
    if (filter.sourceIpEnabled &&
        packet.source.compare(filter.sourceIp, Qt::CaseInsensitive) != 0) {
        return false;
    }
    if (filter.destinationIpEnabled &&
        packet.destination.compare(filter.destinationIp, Qt::CaseInsensitive) != 0) {
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
    const FilterState filter = currentFilterState();
    for (const PacketRow& packet : m_packets) {
        if (m_saveFilteredRadio->isChecked() && !matchesFilter(packet, filter)) {
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
    QMessageBox::information(this,
                             QStringLiteral("保存成功"),
                             QStringLiteral("已成功保存 %1 个数据包。\n%2")
                                 .arg(savedCount)
                                 .arg(QDir::toNativeSeparators(outputPath)));
}

void PacketCaptureDialog::finishCapture(const QString& message, bool failed, bool permissionDenied)
{
#ifndef Q_OS_LINUX
    Q_UNUSED(permissionDenied);
#endif
    if (m_captureThread.joinable()
#ifdef Q_OS_LINUX
        && !m_usePrivilegedCaptureHelper
#endif
    ) {
        m_captureThread.join();
    }
    m_captureActive = false;
    m_elapsedTimer->stop();
    updateStatistics();
    updateCaptureControls();
    if (failed) {
#ifdef Q_OS_LINUX
        if (permissionDenied) {
            requestCapturePermission();
            return;
        }
#endif
        QMessageBox::warning(this, QStringLiteral("抓包失败"), message);
    }
}

#ifdef Q_OS_LINUX
void PacketCaptureDialog::requestCapturePermission()
{
    const QString pkexecPath = QStandardPaths::findExecutable(
        QStringLiteral("pkexec"), {QStringLiteral("/usr/bin"), QStringLiteral("/bin")});
    const QString helperPath = QCoreApplication::applicationDirPath() +
        QStringLiteral("/LivoxPacketCaptureHelper");
    if (pkexecPath.isEmpty() || !QFileInfo::exists(helperPath)) {
        QMessageBox::warning(
            this,
            QStringLiteral("无法获取权限"),
            pkexecPath.isEmpty()
                ? QStringLiteral("系统未安装 pkexec，请先安装 polkit。")
                : QStringLiteral("未找到抓包权限辅助程序。"));
        return;
    }
    m_usePrivilegedCaptureHelper = true;
    startCapture();
}
#endif

void PacketCaptureDialog::updateCaptureControls()
{
    m_interfaceCombo->setEnabled(!m_captureActive);
    m_startButton->setEnabled(!m_captureActive && m_interfaceCombo->currentIndex() >= 0);
    m_stopButton->setEnabled(m_captureActive);
    m_clearButton->setEnabled(!m_packets.isEmpty());
    m_saveButton->setEnabled(!m_captureActive && !m_packets.isEmpty());
    m_saveDirectoryEdit->setEnabled(!m_captureActive);
    m_fileNameEdit->setEnabled(!m_captureActive);
    m_saveAllRadio->setEnabled(!m_captureActive);
    m_saveFilteredRadio->setEnabled(!m_captureActive);
}

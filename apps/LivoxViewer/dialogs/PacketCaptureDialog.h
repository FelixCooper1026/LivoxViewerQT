#ifndef LIVOXVIEWER_DIALOGS_PACKETCAPTUREDIALOG_H
#define LIVOXVIEWER_DIALOGS_PACKETCAPTUREDIALOG_H

#include "NetworkInterfaceService.h"

#include <QByteArray>
#include <QDateTime>
#include <QDialog>
#include <QElapsedTimer>
#include <QSet>
#include <QString>
#include <QVector>

#include <atomic>
#include <functional>
#include <thread>

class QCloseEvent;
class QComboBox;
class QEvent;
class QLabel;
class QLineEdit;
class QPushButton;
class QRadioButton;
class QTableView;
class QTimer;
class QToolButton;
class PacketTableModel;

class PacketCaptureDialog : public QDialog
{
public:
    explicit PacketCaptureDialog(QWidget* parent = nullptr);
    ~PacketCaptureDialog() override;

    void setInterfaces(const QList<NetworkInterfaceService::NetworkInterfaceInfo>& interfaces,
                       const QString& selectedSystemName);
    void selectInterface(const QString& systemName);
    void setInterfaceChangedHandler(std::function<void(const QString&)> handler);
    void setDeviceSerialNumber(const QString& serialNumber);

protected:
    void changeEvent(QEvent* event) override;
    void closeEvent(QCloseEvent* event) override;

private:
    friend class PacketTableModel;

    struct PacketRow {
        quint64 number = 0;
        double relativeTimeSec = 0.0;
        QString source;
        QString destination;
        QString protocol;
        int length = 0;
        QString ports;
        QString payloadHex;
        bool broadcast = false;
        bool control = false;
        bool pointCloud = false;
        bool imu = false;
        bool push = false;
        bool ptp = false;
        bool arp = false;
        QByteArray rawData;
        qint64 timestampSec = 0;
        qint64 timestampUsec = 0;
        int capturedLength = 0;
        int originalLength = 0;
        int dataLinkType = 0;
    };

    struct FilterState {
        bool broadcast = false;
        bool control = false;
        bool pointCloud = false;
        bool imu = false;
        bool push = false;
        bool ptp = false;
        bool arp = false;
        bool sourceIpEnabled = false;
        bool destinationIpEnabled = false;
        QString sourceIp;
        QString destinationIp;
    };

    void startCapture();
    void stopCapture();
    void captureLoop(QString systemName, QString ipv4);
    void appendPackets(QVector<PacketRow> packets);
    void rebuildTable();
    void updateStatistics();
    void updateEmptyState();
    void refreshTheme();
    void showPacketDetails(int tableRow);
    FilterState currentFilterState() const;
    static bool matchesFilter(const PacketRow& packet, const FilterState& filter);
    void saveCapture();
    void updateDefaultFileName();
    QString selectedFilterName() const;
    void finishCapture(const QString& message, bool failed);
    void updateCaptureControls();
    void clearCapturedAddresses();
    static QString captureDeviceName(const QString& systemName, const QString& ipv4, QString* errorMessage);
    static PacketRow decodePacket(const unsigned char* data,
                                  int capturedLength,
                                  int originalLength,
                                  int dataLinkType,
                                  quint64 number,
                                  double relativeTimeSec,
                                  qint64 timestampSec,
                                  qint64 timestampUsec);

    QComboBox* m_interfaceCombo = nullptr;
    QToolButton* m_broadcastFilter = nullptr;
    QToolButton* m_controlFilter = nullptr;
    QToolButton* m_pointCloudFilter = nullptr;
    QToolButton* m_imuFilter = nullptr;
    QToolButton* m_pushFilter = nullptr;
    QToolButton* m_ptpFilter = nullptr;
    QToolButton* m_arpFilter = nullptr;
    QToolButton* m_sourceIpFilter = nullptr;
    QToolButton* m_destinationIpFilter = nullptr;
    QComboBox* m_sourceIpCombo = nullptr;
    QComboBox* m_destinationIpCombo = nullptr;
    QLineEdit* m_saveDirectoryEdit = nullptr;
    QLineEdit* m_fileNameEdit = nullptr;
    QRadioButton* m_saveAllRadio = nullptr;
    QRadioButton* m_saveFilteredRadio = nullptr;
    QPushButton* m_startButton = nullptr;
    QPushButton* m_stopButton = nullptr;
    QPushButton* m_clearButton = nullptr;
    QPushButton* m_saveButton = nullptr;
    QLabel* m_totalPacketsLabel = nullptr;
    QLabel* m_visiblePacketsLabel = nullptr;
    QLabel* m_totalBytesLabel = nullptr;
    QLabel* m_elapsedTimeLabel = nullptr;
    QLabel* m_emptyStateLabel = nullptr;
    QTableView* m_packetTable = nullptr;
    PacketTableModel* m_packetModel = nullptr;
    QTimer* m_elapsedTimer = nullptr;
    QElapsedTimer m_captureElapsed;
    QVector<PacketRow> m_packets;
    QSet<QString> m_sourceAddresses;
    QSet<QString> m_destinationAddresses;
    quint64 m_totalBytes = 0;
    QString m_deviceSerialNumber = QStringLiteral("Unknown");
    QDateTime m_captureTimestamp;
    int m_dataLinkType = 0;
    std::function<void(const QString&)> m_interfaceChangedHandler;
    std::thread m_captureThread;
    std::atomic_bool m_stopRequested{false};
    bool m_captureActive = false;
    bool m_fileNameCustomized = false;
    bool m_refreshingTheme = false;
};

#endif // LIVOXVIEWER_DIALOGS_PACKETCAPTUREDIALOG_H

#ifndef LIVOXVIEWER_DIALOGS_PACKETCAPTUREDIALOG_H
#define LIVOXVIEWER_DIALOGS_PACKETCAPTUREDIALOG_H

#include "NetworkInterfaceService.h"

#include <QByteArray>
#include <QDateTime>
#include <QDialog>
#include <QString>
#include <QVector>

#include <atomic>
#include <functional>
#include <thread>

class QCheckBox;
class QCloseEvent;
class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QRadioButton;
class QTableWidget;

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
    void closeEvent(QCloseEvent* event) override;

private:
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

    void startCapture();
    void stopCapture();
    void captureLoop(QString systemName, QString ipv4);
    void appendPackets(QVector<PacketRow> packets);
    void appendPacketToTable(const PacketRow& packet);
    void rebuildTable();
    bool matchesFilter(const PacketRow& packet) const;
    void saveCapture();
    void updateDefaultFileName();
    QString selectedFilterName() const;
    void finishCapture(const QString& message, bool failed);
    void updateCaptureControls();
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
    QCheckBox* m_broadcastFilter = nullptr;
    QCheckBox* m_controlFilter = nullptr;
    QCheckBox* m_pointCloudFilter = nullptr;
    QCheckBox* m_imuFilter = nullptr;
    QCheckBox* m_pushFilter = nullptr;
    QCheckBox* m_ptpFilter = nullptr;
    QCheckBox* m_arpFilter = nullptr;
    QCheckBox* m_sourceIpFilter = nullptr;
    QCheckBox* m_destinationIpFilter = nullptr;
    QLineEdit* m_sourceIpEdit = nullptr;
    QLineEdit* m_destinationIpEdit = nullptr;
    QLineEdit* m_saveDirectoryEdit = nullptr;
    QLineEdit* m_fileNameEdit = nullptr;
    QRadioButton* m_saveAllRadio = nullptr;
    QRadioButton* m_saveFilteredRadio = nullptr;
    QPushButton* m_startButton = nullptr;
    QPushButton* m_stopButton = nullptr;
    QPushButton* m_clearButton = nullptr;
    QPushButton* m_saveButton = nullptr;
    QLabel* m_statusLabel = nullptr;
    QTableWidget* m_packetTable = nullptr;
    QVector<PacketRow> m_packets;
    QString m_deviceSerialNumber = QStringLiteral("Unknown");
    QDateTime m_captureTimestamp;
    int m_dataLinkType = 0;
    std::function<void(const QString&)> m_interfaceChangedHandler;
    std::thread m_captureThread;
    std::atomic_bool m_stopRequested{false};
    bool m_captureActive = false;
    bool m_fileNameCustomized = false;
};

#endif // LIVOXVIEWER_DIALOGS_PACKETCAPTUREDIALOG_H

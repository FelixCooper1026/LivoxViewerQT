#ifndef PCAP_PCAPPARSER_H
#define PCAP_PCAPPARSER_H

#include "Pcap/PushMsgParser.h"
#include "PointCloud/PointCloudFrame.h"

#include <QString>
#include <QVector>

namespace PcapParser {

struct ParseResult {
    bool ok = false;
    QString errorMessage;
    QVector<PointCloudFrame> frames;
    QVector<PushMsgParser::PushDeviceRecord> devices;
    uint64_t totalPacketsScanned = 0;
    int datalinkType = 0;
};

ParseResult parseFileToFrames(const QString& filePath);

} // namespace PcapParser

#endif // PCAP_PCAPPARSER_H

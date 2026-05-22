#ifndef PCAP_PCAPPLAYBACKCONTROLLER_H
#define PCAP_PCAPPLAYBACKCONTROLLER_H

#include <QString>

namespace PcapPlayback {

inline bool isSupportedFile(const QString& filePath)
{
    return filePath.endsWith(QStringLiteral(".pcap"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".pcapng"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".cap"), Qt::CaseInsensitive);
}

} // namespace PcapPlayback

// Playback UI and frame stepping reuse LVX2 playback in LivoxViewerWindow.

#endif // PCAP_PCAPPLAYBACKCONTROLLER_H

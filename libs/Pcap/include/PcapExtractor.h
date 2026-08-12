#ifndef PCAP_PCAPEXTRACTOR_H
#define PCAP_PCAPEXTRACTOR_H

#include <QString>
#include <QStringList>

#include <atomic>
#include <functional>

namespace PcapExtractor {

enum class Kind {
    Lvx2,
    ImuCsv,
    InfoCsv
};

enum class Mode {
    Merge,
    SplitByDevice
};

struct Result {
    bool ok = false;
    QString errorMessage;
    QStringList warnings;
    QStringList outputFiles;
};

using ProgressCallback = std::function<void(int)>;

Result extract(const QString& sourcePath,
               const QString& outputDirectory,
               Kind kind,
               Mode mode,
               const ProgressCallback& progress = {},
               const std::atomic_bool* cancellationRequested = nullptr);

} // namespace PcapExtractor

#endif // PCAP_PCAPEXTRACTOR_H

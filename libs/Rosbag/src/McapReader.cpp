#include "McapReader.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QStringList>

#include <mcap/reader.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>

namespace {

QVector<QString> parseMetadataRelativeFilePaths(const QString& metadataPath)
{
    QVector<QString> result;
    QFile file(metadataPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return result;
    }

    bool inRelativeFilePaths = false;
    while (!file.atEnd()) {
        QString line = QString::fromUtf8(file.readLine()).trimmed();
        if (line.startsWith(QLatin1Char('#')) || line.isEmpty()) {
            continue;
        }
        if (line.startsWith(QStringLiteral("relative_file_paths:"))) {
            inRelativeFilePaths = true;
            const int bracketStart = line.indexOf(QLatin1Char('['));
            const int bracketEnd = line.indexOf(QLatin1Char(']'));
            if (bracketStart >= 0 && bracketEnd > bracketStart) {
                const QString inside = line.mid(bracketStart + 1, bracketEnd - bracketStart - 1);
                const QStringList items = inside.split(QLatin1Char(','), Qt::SkipEmptyParts);
                for (QString item : items) {
                    item = item.trimmed();
                    item.remove(QLatin1Char('"'));
                    item.remove(QLatin1Char('\''));
                    if (!item.isEmpty()) {
                        result.push_back(item);
                    }
                }
                inRelativeFilePaths = false;
            }
            continue;
        }
        if (inRelativeFilePaths) {
            if (line.startsWith(QLatin1String("- "))) {
                line = line.mid(2).trimmed();
                line.remove(QLatin1Char('"'));
                line.remove(QLatin1Char('\''));
                if (!line.isEmpty()) {
                    result.push_back(line);
                }
                continue;
            }
            if (!line.startsWith(QLatin1Char('-'))) {
                inRelativeFilePaths = false;
            }
        }
    }
    return result;
}

QVector<QString> resolveMcapFiles(const QString& filePath, QString* error)
{
    const QFileInfo info(filePath);
    if (!info.exists()) {
        if (error) {
            *error = QStringLiteral("ROS2 MCAP 加载失败：文件不存在 %1。").arg(filePath);
        }
        return {};
    }
    if (info.suffix().compare(QStringLiteral("mcap"), Qt::CaseInsensitive) == 0) {
        return {info.absoluteFilePath()};
    }
    if (info.suffix().compare(QStringLiteral("yaml"), Qt::CaseInsensitive) != 0 &&
        info.suffix().compare(QStringLiteral("yml"), Qt::CaseInsensitive) != 0) {
        if (error) {
            *error = QStringLiteral("ROS2 MCAP 加载失败：仅支持选择 .mcap 或 metadata.yaml。");
        }
        return {};
    }

    QVector<QString> files;
    const QDir baseDir = info.absoluteDir();
    for (const QString& relativePath : parseMetadataRelativeFilePaths(info.absoluteFilePath())) {
        if (QFileInfo(relativePath).suffix().compare(QStringLiteral("mcap"), Qt::CaseInsensitive) != 0) {
            continue;
        }
        const QFileInfo mcapInfo(baseDir.filePath(relativePath));
        if (mcapInfo.exists()) {
            files.push_back(mcapInfo.absoluteFilePath());
        }
    }
    if (files.isEmpty() && error) {
        *error = QStringLiteral("ROS2 MCAP 加载失败：metadata.yaml 未引用有效的 MCAP 文件。");
    }
    return files;
}

QString connectionKey(const QString& topic, const QString& type, const QString& messageEncoding)
{
    return topic + QChar(0x1f) + type + QChar(0x1f) + messageEncoding;
}

QString statusText(const mcap::Status& status)
{
    return QString::fromStdString(status.message);
}

bool openMcap(const QString& path, std::ifstream& stream, mcap::McapReader& reader, QString* error)
{
#ifdef _WIN32
    stream.open(std::filesystem::path(path.toStdWString()), std::ios::binary);
#else
    stream.open(path.toStdString(), std::ios::binary);
#endif
    if (!stream.is_open()) {
        if (error) {
            *error = QStringLiteral("ROS2 MCAP 加载失败：无法打开 %1。").arg(path);
        }
        return false;
    }
    const mcap::Status status = reader.open(stream);
    if (!status.ok()) {
        if (error) {
            *error = QStringLiteral("ROS2 MCAP 加载失败：%1：%2").arg(path, statusText(status));
        }
        return false;
    }
    return true;
}

int connectionIndexById(const QVector<Rosbag::Connection>& connections, int id)
{
    for (int i = 0; i < connections.size(); ++i) {
        if (connections.at(i).id == id) {
            return i;
        }
    }
    return -1;
}

} // namespace

namespace Rosbag {

bool isMcapPath(const QString& filePath)
{
    const QFileInfo info(filePath);
    if (info.suffix().compare(QStringLiteral("mcap"), Qt::CaseInsensitive) == 0) {
        return true;
    }
    if (info.suffix().compare(QStringLiteral("yaml"), Qt::CaseInsensitive) != 0 &&
        info.suffix().compare(QStringLiteral("yml"), Qt::CaseInsensitive) != 0) {
        return false;
    }
    for (const QString& relativePath : parseMetadataRelativeFilePaths(info.absoluteFilePath())) {
        if (QFileInfo(relativePath).suffix().compare(QStringLiteral("mcap"), Qt::CaseInsensitive) == 0) {
            return true;
        }
    }
    return false;
}

bool McapReader::readConnections(const QString& filePath, QString* error)
{
    clear();
    summary_.filePath = filePath;
    const QVector<QString> files = resolveMcapFiles(filePath, error);
    if (files.isEmpty()) {
        return false;
    }

    QHash<QString, int> idsByKey;
    for (const QString& path : files) {
        std::ifstream stream;
        mcap::McapReader reader;
        if (!openMcap(path, stream, reader, error)) {
            return false;
        }
        QString problem;
        const mcap::Status summaryStatus = reader.readSummary(
            mcap::ReadSummaryMethod::AllowFallbackScan,
            [&problem](const mcap::Status& status) { problem = statusText(status); });
        if (!summaryStatus.ok()) {
            if (error) {
                *error = QStringLiteral("ROS2 MCAP 加载失败：读取摘要失败：%1").arg(statusText(summaryStatus));
            }
            return false;
        }
        if (!problem.isEmpty()) {
            if (error) {
                *error = QStringLiteral("ROS2 MCAP 加载失败：%1").arg(problem);
            }
            return false;
        }

        QVector<mcap::ChannelPtr> channels;
        for (const auto& entry : reader.channels()) {
            channels.push_back(entry.second);
        }
        std::sort(channels.begin(), channels.end(), [](const mcap::ChannelPtr& lhs, const mcap::ChannelPtr& rhs) {
            return lhs->id < rhs->id;
        });

        QHash<int, int> stableIdByChannelId;
        for (const mcap::ChannelPtr& channel : channels) {
            const mcap::SchemaPtr schema = reader.schema(channel->schemaId);
            const QString topic = QString::fromStdString(channel->topic);
            const QString type = schema ? QString::fromStdString(schema->name) : QString();
            const QString messageEncoding = QString::fromStdString(channel->messageEncoding);
            const QString key = connectionKey(topic, type, messageEncoding);
            int stableId = idsByKey.value(key, -1);
            if (stableId < 0) {
                stableId = connections_.size() + 1;
                idsByKey.insert(key, stableId);
                Connection connection;
                connection.id = stableId;
                connection.topic = topic;
                connection.type = type;
                connection.messageEncoding = messageEncoding;
                connection.fields.insert(QStringLiteral("serialization_format"), messageEncoding.toUtf8());
                if (schema) {
                    connection.schemaEncoding = QString::fromStdString(schema->encoding);
                    connection.messageDefinition = QString::fromUtf8(
                        reinterpret_cast<const char*>(schema->data.data()), int(schema->data.size()));
                }
                connections_.push_back(std::move(connection));
            }
            stableIdByChannelId.insert(channel->id, stableId);
        }

        const std::optional<mcap::Statistics>& statistics = reader.statistics();
        if (statistics) {
            const bool firstMessages = summary_.messageCount == 0;
            summary_.messageCount += int64_t(statistics->messageCount);
            if (statistics->messageCount > 0) {
                if (firstMessages || int64_t(statistics->messageStartTime) < summary_.startTimestampNs) {
                    summary_.startTimestampNs = int64_t(statistics->messageStartTime);
                }
                if (firstMessages || int64_t(statistics->messageEndTime) > summary_.endTimestampNs) {
                    summary_.endTimestampNs = int64_t(statistics->messageEndTime);
                }
            }
            summary_.chunkCount += int(statistics->chunkCount);
            for (const auto& count : statistics->channelMessageCounts) {
                const int stableId = stableIdByChannelId.value(count.first, -1);
                const int index = connectionIndexById(connections_, stableId);
                if (index >= 0) {
                    connections_[index].messageCount += int64_t(count.second);
                }
            }
        } else {
            summary_.chunkCount += int(reader.chunkIndexes().size());
        }
        reader.close();
    }

    summary_.connectionCount = connections_.size();
    if (connections_.isEmpty()) {
        if (error) {
            *error = QStringLiteral("ROS2 MCAP 加载失败：未找到任何 channel。");
        }
        return false;
    }
    if (error) {
        error->clear();
    }
    return true;
}

bool McapReader::streamMessages(const QString& filePath,
                                const QSet<int>& connectionIds,
                                const std::atomic_bool* cancellationRequested,
                                const std::function<bool(const SerializedMessage&)>& consumer,
                                const std::function<void(int64_t, int64_t)>& progress,
                                QString* error)
{
    const QVector<QString> files = resolveMcapFiles(filePath, error);
    if (files.isEmpty()) {
        return false;
    }

    int64_t totalSize = 0;
    for (const QString& path : files) {
        totalSize += QFileInfo(path).size();
    }
    int64_t completedSize = 0;
    summary_.messageCount = 0;
    summary_.startTimestampNs = 0;
    summary_.endTimestampNs = 0;
    for (Connection& connection : connections_) {
        connection.messageCount = 0;
    }

    for (const QString& path : files) {
        if (cancellationRequested && cancellationRequested->load()) {
            return false;
        }
        std::ifstream stream;
        mcap::McapReader reader;
        if (!openMcap(path, stream, reader, error)) {
            return false;
        }
        QString problem;
        const mcap::Status summaryStatus = reader.readSummary(
            mcap::ReadSummaryMethod::AllowFallbackScan,
            [&problem](const mcap::Status& status) { problem = statusText(status); });
        if (!summaryStatus.ok()) {
            if (error) {
                *error = QStringLiteral("ROS2 MCAP 加载失败：读取摘要失败：%1").arg(statusText(summaryStatus));
            }
            return false;
        }

        QHash<int, int> stableIdByChannelId;
        for (const auto& entry : reader.channels()) {
            const mcap::ChannelPtr& channel = entry.second;
            const mcap::SchemaPtr schema = reader.schema(channel->schemaId);
            const QString key = connectionKey(QString::fromStdString(channel->topic),
                                              schema ? QString::fromStdString(schema->name) : QString(),
                                              QString::fromStdString(channel->messageEncoding));
            for (const Connection& connection : connections_) {
                if (connectionKey(connection.topic, connection.type, connection.messageEncoding) == key) {
                    stableIdByChannelId.insert(channel->id, connection.id);
                    break;
                }
            }
        }

        mcap::ReadMessageOptions options;
        options.readOrder = mcap::ReadMessageOptions::ReadOrder::FileOrder;
        options.topicFilter = [&stableIdByChannelId, &connectionIds, &reader](std::string_view topic) {
            for (const auto& entry : reader.channels()) {
                if (entry.second->topic == topic) {
                    const int stableId = stableIdByChannelId.value(entry.first, -1);
                    if (stableId >= 0 && (connectionIds.isEmpty() || connectionIds.contains(stableId))) {
                        return true;
                    }
                }
            }
            return false;
        };

        const int64_t fileSize = QFileInfo(path).size();
        bool consumerStopped = false;
        auto view = reader.readMessages(
            [&problem](const mcap::Status& status) { problem = statusText(status); }, options);
        for (const mcap::MessageView& item : view) {
            if (cancellationRequested && cancellationRequested->load()) {
                return false;
            }
            const int stableId = stableIdByChannelId.value(item.channel->id, -1);
            if (stableId < 0 || (!connectionIds.isEmpty() && !connectionIds.contains(stableId))) {
                continue;
            }
            const int index = connectionIndexById(connections_, stableId);
            if (index < 0) {
                continue;
            }

            SerializedMessage message;
            message.connectionId = stableId;
            message.topic = connections_.at(index).topic;
            message.type = connections_.at(index).type;
            message.timestampNs = int64_t(item.message.logTime);
            message.publishTimestampNs = int64_t(item.message.publishTime);
            message.data = QByteArray(reinterpret_cast<const char*>(item.message.data), int(item.message.dataSize));
            connections_[index].messageCount++;
            summary_.messageCount++;
            if (summary_.messageCount == 1 || message.timestampNs < summary_.startTimestampNs) {
                summary_.startTimestampNs = message.timestampNs;
            }
            if (summary_.messageCount == 1 || message.timestampNs > summary_.endTimestampNs) {
                summary_.endTimestampNs = message.timestampNs;
            }

            const uint64_t rawOffset = item.messageOffset.chunkOffset.value_or(item.messageOffset.offset);
            progress(completedSize + std::min<int64_t>(fileSize, int64_t(rawOffset)), totalSize);
            if (!consumer(message)) {
                consumerStopped = true;
                break;
            }
        }
        if (!problem.isEmpty()) {
            if (error) {
                *error = QStringLiteral("ROS2 MCAP 加载失败：%1").arg(problem);
            }
            return false;
        }
        if (consumerStopped) {
            return false;
        }
        reader.close();
        completedSize += fileSize;
        progress(completedSize, totalSize);
    }

    if (error) {
        error->clear();
    }
    return true;
}

bool McapReader::read(const QString& filePath, QString* error)
{
    if (!readConnections(filePath, error)) {
        return false;
    }
    messages_.clear();
    const bool ok = streamMessages(
        filePath,
        {},
        nullptr,
        [this](const SerializedMessage& message) {
            messages_.push_back(message);
            return true;
        },
        [](int64_t, int64_t) {},
        error);
    if (!ok) {
        return false;
    }
    if (messages_.isEmpty()) {
        if (error) {
            *error = QStringLiteral("ROS2 MCAP 加载失败：未找到 message data。");
        }
        return false;
    }
    return true;
}

void McapReader::clear()
{
    connections_.clear();
    messages_.clear();
    summary_ = Summary();
}

const QVector<Connection>& McapReader::connections() const
{
    return connections_;
}

const QVector<SerializedMessage>& McapReader::messages() const
{
    return messages_;
}

const Summary& McapReader::summary() const
{
    return summary_;
}

} // namespace Rosbag

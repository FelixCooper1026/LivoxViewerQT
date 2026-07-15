#include "Ros2BagReader.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSet>
#include <QStringList>
#include <QUuid>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlError>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlRecord>

#include <algorithm>

namespace {

QString topicListText(const QVector<Rosbag::Connection>& connections)
{
    QStringList topics;
    for (const Rosbag::Connection& connection : connections) {
        topics << QStringLiteral("%1(%2)").arg(connection.topic, connection.type);
    }
    return topics.join(QStringLiteral(", "));
}

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

QVector<QString> resolveDb3Files(const QString& filePath, QString* error)
{
    QFileInfo info(filePath);
    if (!info.exists()) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 db3 加载失败：文件不存在 %1。").arg(filePath);
        }
        return {};
    }

    if (info.suffix().compare(QStringLiteral("db3"), Qt::CaseInsensitive) == 0) {
        return {info.absoluteFilePath()};
    }

    if (info.fileName().compare(QStringLiteral("metadata.yaml"), Qt::CaseInsensitive) != 0 &&
        info.suffix().compare(QStringLiteral("yaml"), Qt::CaseInsensitive) != 0 &&
        info.suffix().compare(QStringLiteral("yml"), Qt::CaseInsensitive) != 0) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 db3 加载失败：仅支持选择 .db3 或 metadata.yaml。");
        }
        return {};
    }

    const QVector<QString> relativePaths = parseMetadataRelativeFilePaths(info.absoluteFilePath());
    if (relativePaths.isEmpty()) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 db3 加载失败：metadata.yaml 未包含 relative_file_paths。");
        }
        return {};
    }

    QVector<QString> db3Files;
    const QDir baseDir = info.absoluteDir();
    for (const QString& relativePath : relativePaths) {
        const QString dbPath = baseDir.filePath(relativePath);
        if (QFileInfo::exists(dbPath)) {
            db3Files.push_back(QFileInfo(dbPath).absoluteFilePath());
        }
    }
    if (db3Files.isEmpty() && error != nullptr) {
        *error = QStringLiteral("ROS2 db3 加载失败：metadata.yaml 引用的 db3 文件不存在。");
    }
    return db3Files;
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

QString sqlErrorText(const QSqlQuery& query)
{
    return query.lastError().text();
}

} // namespace

namespace Rosbag {

bool Ros2BagReader::readConnections(const QString& filePath, QString* error)
{
    clear();
    summary_.filePath = filePath;
    const QVector<QString> db3Files = resolveDb3Files(filePath, error);
    if (db3Files.isEmpty()) {
        return false;
    }

    for (const QString& db3Path : db3Files) {
        const QString connectionName = QStringLiteral("livoxviewer_ros2bag_topics_%1")
                                           .arg(QUuid::createUuid().toString(QUuid::Id128));
        bool ok = true;
        {
            QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), connectionName);
            db.setDatabaseName(db3Path);
            if (!db.open()) {
                if (error != nullptr) {
                    *error = QStringLiteral("ROS2 db3 加载失败：无法打开 %1：%2")
                                 .arg(db3Path, db.lastError().text());
                }
                ok = false;
            } else {
                QSqlQuery topicQuery(db);
                if (!topicQuery.exec(QStringLiteral(
                        "SELECT id, name, type, serialization_format FROM topics ORDER BY id"))) {
                    if (error != nullptr) {
                        *error = QStringLiteral("ROS2 db3 加载失败：读取 topics 表失败：%1")
                                     .arg(sqlErrorText(topicQuery));
                    }
                    ok = false;
                } else {
                    while (topicQuery.next()) {
                        Connection connection;
                        connection.id = topicQuery.value(0).toInt();
                        connection.topic = topicQuery.value(1).toString();
                        connection.type = topicQuery.value(2).toString();
                        connection.fields.insert(QStringLiteral("serialization_format"),
                                                 topicQuery.value(3).toString().toUtf8());
                        const int existing = connectionIndexById(connections_, connection.id);
                        if (existing < 0) {
                            connections_.push_back(std::move(connection));
                        }
                    }
                }
                db.close();
            }
        }
        QSqlDatabase::removeDatabase(connectionName);
        if (!ok) {
            return false;
        }
    }

    summary_.connectionCount = connections_.size();
    summary_.chunkCount = db3Files.size();
    if (connections_.isEmpty()) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 db3 加载失败：未找到任何 topic。");
        }
        return false;
    }
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

bool Ros2BagReader::streamMessages(const QString& filePath,
                                   const QSet<int>& connectionIds,
                                   const std::atomic_bool* cancellationRequested,
                                   const std::function<bool(const SerializedMessage&)>& consumer,
                                   QString* error)
{
    const QVector<QString> db3Files = resolveDb3Files(filePath, error);
    if (db3Files.isEmpty()) {
        return false;
    }

    summary_.messageCount = 0;
    summary_.startTimestampNs = 0;
    summary_.endTimestampNs = 0;
    for (Connection& connection : connections_) {
        connection.messageCount = 0;
    }

    QString queryText = QStringLiteral("SELECT topic_id, timestamp, data FROM messages");
    if (!connectionIds.isEmpty()) {
        QStringList ids;
        ids.reserve(connectionIds.size());
        for (int id : connectionIds) {
            ids.push_back(QString::number(id));
        }
        queryText += QStringLiteral(" WHERE topic_id IN (%1)").arg(ids.join(QLatin1Char(',')));
    }
    queryText += QStringLiteral(" ORDER BY timestamp, id");

    for (const QString& db3Path : db3Files) {
        if (cancellationRequested && cancellationRequested->load()) {
            return false;
        }
        const QString connectionName = QStringLiteral("livoxviewer_ros2bag_stream_%1")
                                           .arg(QUuid::createUuid().toString(QUuid::Id128));
        bool ok = true;
        bool consumerStopped = false;
        {
            QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), connectionName);
            db.setDatabaseName(db3Path);
            if (!db.open()) {
                if (error != nullptr) {
                    *error = QStringLiteral("ROS2 db3 加载失败：无法打开 %1：%2")
                                 .arg(db3Path, db.lastError().text());
                }
                ok = false;
            } else {
                QSqlQuery messageQuery(db);
                messageQuery.setForwardOnly(true);
                if (!messageQuery.exec(queryText)) {
                    if (error != nullptr) {
                        *error = QStringLiteral("ROS2 db3 加载失败：读取 messages 表失败：%1")
                                     .arg(sqlErrorText(messageQuery));
                    }
                    ok = false;
                } else {
                    while (messageQuery.next()) {
                        if (cancellationRequested && cancellationRequested->load()) {
                            ok = false;
                            break;
                        }
                        SerializedMessage message;
                        message.connectionId = messageQuery.value(0).toInt();
                        message.timestampNs = messageQuery.value(1).toLongLong();
                        message.data = messageQuery.value(2).toByteArray();
                        const int index = connectionIndexById(connections_, message.connectionId);
                        if (index >= 0) {
                            connections_[index].messageCount++;
                            message.topic = connections_.at(index).topic;
                            message.type = connections_.at(index).type;
                        }
                        summary_.messageCount++;
                        if (summary_.messageCount == 1 || message.timestampNs < summary_.startTimestampNs) {
                            summary_.startTimestampNs = message.timestampNs;
                        }
                        if (summary_.messageCount == 1 || message.timestampNs > summary_.endTimestampNs) {
                            summary_.endTimestampNs = message.timestampNs;
                        }
                        if (!consumer(message)) {
                            consumerStopped = true;
                            ok = false;
                            break;
                        }
                    }
                }
                db.close();
            }
        }
        QSqlDatabase::removeDatabase(connectionName);
        if (!ok) {
            return false;
        }
        if (consumerStopped) {
            return false;
        }
    }
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

bool Ros2BagReader::read(const QString& filePath, QString* error)
{
    clear();
    summary_.filePath = filePath;

    const QVector<QString> db3Files = resolveDb3Files(filePath, error);
    if (db3Files.isEmpty()) {
        return false;
    }

    for (const QString& db3Path : db3Files) {
        const QString connectionName = QStringLiteral("livoxviewer_ros2bag_%1").arg(QUuid::createUuid().toString(QUuid::Id128));
        {
            QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), connectionName);
            db.setDatabaseName(db3Path);
            if (!db.open()) {
                if (error != nullptr) {
                    *error = QStringLiteral("ROS2 db3 加载失败：无法打开 %1：%2")
                                 .arg(db3Path, db.lastError().text());
                }
                QSqlDatabase::removeDatabase(connectionName);
                return false;
            }

            QSqlQuery topicQuery(db);
            if (!topicQuery.exec(QStringLiteral("SELECT id, name, type, serialization_format FROM topics ORDER BY id"))) {
                if (error != nullptr) {
                    *error = QStringLiteral("ROS2 db3 加载失败：读取 topics 表失败：%1").arg(sqlErrorText(topicQuery));
                }
                db.close();
                QSqlDatabase::removeDatabase(connectionName);
                return false;
            }

            QSet<int> dbTopicIds;
            while (topicQuery.next()) {
                Connection connection;
                connection.id = topicQuery.value(0).toInt();
                connection.topic = topicQuery.value(1).toString();
                connection.type = topicQuery.value(2).toString();
                connection.fields.insert(QStringLiteral("serialization_format"),
                                         topicQuery.value(3).toString().toUtf8());
                dbTopicIds.insert(connection.id);

                const int existing = connectionIndexById(connections_, connection.id);
                if (existing >= 0) {
                    connections_[existing].topic = connection.topic;
                    connections_[existing].type = connection.type;
                    connections_[existing].fields = connection.fields;
                } else {
                    connections_.push_back(std::move(connection));
                }
            }

            QSqlQuery messageQuery(db);
            if (!messageQuery.exec(QStringLiteral("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp, id"))) {
                if (error != nullptr) {
                    *error = QStringLiteral("ROS2 db3 加载失败：读取 messages 表失败：%1").arg(sqlErrorText(messageQuery));
                }
                db.close();
                QSqlDatabase::removeDatabase(connectionName);
                return false;
            }

            while (messageQuery.next()) {
                SerializedMessage message;
                message.connectionId = messageQuery.value(0).toInt();
                message.timestampNs = messageQuery.value(1).toLongLong();
                message.data = messageQuery.value(2).toByteArray();

                const int index = connectionIndexById(connections_, message.connectionId);
                if (index >= 0) {
                    connections_[index].messageCount++;
                    message.topic = connections_.at(index).topic;
                    message.type = connections_.at(index).type;
                }
                messages_.push_back(std::move(message));
                summary_.messageCount++;
                if (summary_.messageCount == 1 || message.timestampNs < summary_.startTimestampNs) {
                    summary_.startTimestampNs = message.timestampNs;
                }
                if (summary_.messageCount == 1 || message.timestampNs > summary_.endTimestampNs) {
                    summary_.endTimestampNs = message.timestampNs;
                }
            }
            summary_.chunkCount++;
            db.close();
        }
        QSqlDatabase::removeDatabase(connectionName);
    }

    summary_.connectionCount = connections_.size();
    if (connections_.isEmpty()) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 db3 加载失败：未找到任何 topic。");
        }
        return false;
    }
    if (messages_.isEmpty()) {
        if (error != nullptr) {
            *error = QStringLiteral("ROS2 db3 加载失败：未找到 message data。已发现 topic: %1。").arg(topicListText(connections_));
        }
        return false;
    }

    std::sort(messages_.begin(), messages_.end(), [](const SerializedMessage& lhs, const SerializedMessage& rhs) {
        if (lhs.timestampNs == rhs.timestampNs) {
            return lhs.connectionId < rhs.connectionId;
        }
        return lhs.timestampNs < rhs.timestampNs;
    });
    return true;
}

void Ros2BagReader::clear()
{
    connections_.clear();
    messages_.clear();
    summary_ = Summary();
}

const QVector<Connection>& Ros2BagReader::connections() const
{
    return connections_;
}

const QVector<SerializedMessage>& Ros2BagReader::messages() const
{
    return messages_;
}

const Summary& Ros2BagReader::summary() const
{
    return summary_;
}

} // namespace Rosbag

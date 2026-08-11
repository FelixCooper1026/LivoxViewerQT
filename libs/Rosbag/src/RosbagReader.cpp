#include "RosbagReader.h"

#include <QBuffer>
#include <QFile>
#include <QStringList>

#include <lz4frame.h>

#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>

namespace {

constexpr uint8_t kOpMsgData = 0x02;
constexpr uint8_t kOpFileHeader = 0x03;
constexpr uint8_t kOpChunk = 0x05;
constexpr uint8_t kOpConnection = 0x07;

struct RawRecord {
    QHash<QString, QByteArray> header;
    QByteArray data;
};

uint32_t readLe32(const char* data)
{
    return uint32_t(uint8_t(data[0])) |
           (uint32_t(uint8_t(data[1])) << 8) |
           (uint32_t(uint8_t(data[2])) << 16) |
           (uint32_t(uint8_t(data[3])) << 24);
}

uint64_t readLe64(const char* data)
{
    uint64_t value = 0;
    for (int i = 7; i >= 0; --i) {
        value = (value << 8) | uint8_t(data[i]);
    }
    return value;
}

bool readExact(QIODevice& device, qint64 size, QByteArray* out, QString* error)
{
    QByteArray data = device.read(size);
    if (data.size() != size) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 记录读取失败：文件提前结束。");
        }
        return false;
    }
    if (out != nullptr) {
        *out = std::move(data);
    }
    return true;
}

bool readU32(QIODevice& device, uint32_t* value, bool* eof, QString* error)
{
    QByteArray bytes = device.read(4);
    if (bytes.isEmpty() && device.atEnd()) {
        if (eof != nullptr) {
            *eof = true;
        }
        return false;
    }
    if (bytes.size() != 4) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 记录读取失败：长度字段不完整。");
        }
        return false;
    }
    if (eof != nullptr) {
        *eof = false;
    }
    *value = readLe32(bytes.constData());
    return true;
}

bool parseHeaderFields(const QByteArray& data, QHash<QString, QByteArray>* fields, QString* error)
{
    fields->clear();
    int offset = 0;
    while (offset < data.size()) {
        if (data.size() - offset < 4) {
            if (error != nullptr) {
                *error = QStringLiteral("ROSbag header field 长度不完整。");
            }
            return false;
        }
        const uint32_t fieldLen = readLe32(data.constData() + offset);
        offset += 4;
        if (fieldLen > uint32_t(data.size() - offset)) {
            if (error != nullptr) {
                *error = QStringLiteral("ROSbag header field 超出记录边界。");
            }
            return false;
        }
        const QByteArray field = data.mid(offset, int(fieldLen));
        offset += int(fieldLen);
        const int equalIndex = field.indexOf('=');
        if (equalIndex <= 0) {
            if (error != nullptr) {
                *error = QStringLiteral("ROSbag header field 缺少 '='。");
            }
            return false;
        }
        const QString name = QString::fromLatin1(field.constData(), equalIndex);
        fields->insert(name, field.mid(equalIndex + 1));
    }
    return true;
}

bool readRecord(QIODevice& device, RawRecord* record, bool* eof, QString* error)
{
    uint32_t headerLen = 0;
    if (!readU32(device, &headerLen, eof, error)) {
        return false;
    }

    QByteArray headerBytes;
    if (!readExact(device, headerLen, &headerBytes, error)) {
        return false;
    }
    if (!parseHeaderFields(headerBytes, &record->header, error)) {
        return false;
    }

    uint32_t dataLen = 0;
    bool dataLenEof = false;
    if (!readU32(device, &dataLen, &dataLenEof, error)) {
        if (dataLenEof && error != nullptr) {
            *error = QStringLiteral("ROSbag 记录读取失败：缺少 data length。");
        }
        return false;
    }
    if (!readExact(device, dataLen, &record->data, error)) {
        return false;
    }
    if (eof != nullptr) {
        *eof = false;
    }
    return true;
}

uint8_t opCode(const QHash<QString, QByteArray>& fields)
{
    const QByteArray op = fields.value(QStringLiteral("op"));
    return op.isEmpty() ? 0 : uint8_t(op.at(0));
}

bool readFieldU32(const QHash<QString, QByteArray>& fields, const QString& name, uint32_t* value)
{
    const QByteArray bytes = fields.value(name);
    if (bytes.size() != 4) {
        return false;
    }
    *value = readLe32(bytes.constData());
    return true;
}

bool readFieldU64(const QHash<QString, QByteArray>& fields, const QString& name, uint64_t* value)
{
    const QByteArray bytes = fields.value(name);
    if (bytes.size() != 8) {
        return false;
    }
    *value = readLe64(bytes.constData());
    return true;
}

bool readFieldTimeNs(const QHash<QString, QByteArray>& fields, const QString& name, int64_t* timestampNs)
{
    const QByteArray bytes = fields.value(name);
    if (bytes.size() != 8) {
        return false;
    }
    const uint32_t sec = readLe32(bytes.constData());
    const uint32_t nsec = readLe32(bytes.constData() + 4);
    *timestampNs = int64_t(sec) * 1000000000LL + int64_t(nsec);
    return true;
}

QString readFieldString(const QHash<QString, QByteArray>& fields, const QString& name)
{
    return QString::fromUtf8(fields.value(name));
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

QString topicListText(const QVector<Rosbag::Connection>& connections)
{
    QStringList topics;
    for (const Rosbag::Connection& connection : connections) {
        topics << QStringLiteral("%1(%2)").arg(connection.topic, connection.type);
    }
    return topics.join(QStringLiteral(", "));
}

bool mergeConnectionRecord(const RawRecord& record,
                           QVector<Rosbag::Connection>& connections,
                           QString* error)
{
    uint32_t conn = 0;
    if (!readFieldU32(record.header, QStringLiteral("conn"), &conn)) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：connection record 缺少 conn 字段。");
        }
        return false;
    }

    QHash<QString, QByteArray> dataFields;
    if (!parseHeaderFields(record.data, &dataFields, error)) {
        return false;
    }

    Rosbag::Connection connection;
    connection.id = int(conn);
    connection.topic = readFieldString(record.header, QStringLiteral("topic"));
    connection.type = readFieldString(dataFields, QStringLiteral("type"));
    connection.md5sum = readFieldString(dataFields, QStringLiteral("md5sum"));
    connection.messageDefinition = readFieldString(dataFields, QStringLiteral("message_definition"));
    connection.fields = dataFields;

    const int existing = connectionIndexById(connections, connection.id);
    if (existing >= 0) {
        connections[existing] = std::move(connection);
    } else {
        connections.push_back(std::move(connection));
    }
    return true;
}

struct Lz4DecompressionContextDeleter {
    void operator()(LZ4F_dctx* context) const
    {
        if (context != nullptr) {
            LZ4F_freeDecompressionContext(context);
        }
    }
};

bool decompressLz4Frame(const QByteArray& compressed,
                        uint32_t uncompressedSize,
                        QByteArray* decompressed,
                        QString* error)
{
    if (uncompressedSize > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：lz4 chunk 解压后大小超过当前进程可分配范围。");
        }
        return false;
    }

    LZ4F_decompressionContext_t rawContext = nullptr;
    size_t result = LZ4F_createDecompressionContext(&rawContext, LZ4F_VERSION);
    if (LZ4F_isError(result)) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：创建 lz4 解压上下文失败：%1。")
                         .arg(QString::fromLatin1(LZ4F_getErrorName(result)));
        }
        return false;
    }
    std::unique_ptr<LZ4F_dctx, Lz4DecompressionContextDeleter> context(rawContext);

    QByteArray output;
    output.resize(static_cast<int>(uncompressedSize));
    const char* source = compressed.constData();
    char* destination = output.data();
    const size_t sourceSize = static_cast<size_t>(compressed.size());
    const size_t destinationSize = static_cast<size_t>(output.size());
    size_t sourceOffset = 0;
    size_t destinationOffset = 0;

    while (sourceOffset < sourceSize) {
        size_t sourceChunkSize = sourceSize - sourceOffset;
        size_t destinationChunkSize = destinationSize - destinationOffset;
        result = LZ4F_decompress(context.get(),
                                 destination + destinationOffset,
                                 &destinationChunkSize,
                                 source + sourceOffset,
                                 &sourceChunkSize,
                                 nullptr);
        if (LZ4F_isError(result)) {
            if (error != nullptr) {
                *error = QStringLiteral("ROSbag 加载失败：lz4 chunk 解压失败：%1。")
                             .arg(QString::fromLatin1(LZ4F_getErrorName(result)));
            }
            return false;
        }

        sourceOffset += sourceChunkSize;
        destinationOffset += destinationChunkSize;
        if (result == 0) {
            break;
        }
        if (sourceChunkSize == 0 && destinationChunkSize == 0) {
            if (error != nullptr) {
                *error = QStringLiteral("ROSbag 加载失败：lz4 chunk 解压未能继续推进。");
            }
            return false;
        }
    }

    if (result != 0 || sourceOffset != sourceSize || destinationOffset != destinationSize) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：lz4 chunk 解压大小不匹配，期望 %1 字节，实际 %2 字节。")
                         .arg(uncompressedSize)
                         .arg(static_cast<qulonglong>(destinationOffset));
        }
        return false;
    }

    *decompressed = std::move(output);
    return true;
}

} // namespace

namespace Rosbag {

bool Reader::readConnections(const QString& filePath, QString* error)
{
    clear();
    summary_.filePath = filePath;

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：无法打开文件 %1。").arg(filePath);
        }
        return false;
    }

    if (file.readLine() != QByteArray("#ROSBAG V2.0\n")) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：文件头不是 ROS1 bag v2.0。");
        }
        return false;
    }
    RawRecord fileHeader;
    bool eof = false;
    if (!readRecord(file, &fileHeader, &eof, error) || opCode(fileHeader.header) != kOpFileHeader) {
        if (error != nullptr && error->isEmpty()) {
            *error = QStringLiteral("ROSbag 加载失败：缺少 file header record。");
        }
        return false;
    }

    uint32_t connCount = 0;
    uint32_t chunkCount = 0;
    uint64_t indexPosition = 0;
    readFieldU32(fileHeader.header, QStringLiteral("conn_count"), &connCount);
    readFieldU32(fileHeader.header, QStringLiteral("chunk_count"), &chunkCount);
    if (!readFieldU64(fileHeader.header, QStringLiteral("index_pos"), &indexPosition) ||
        indexPosition >= uint64_t(file.size()) || !file.seek(qint64(indexPosition))) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：file header 中的 index_pos 无效。");
        }
        return false;
    }
    summary_.connectionCount = int(connCount);
    summary_.chunkCount = int(chunkCount);

    while (!file.atEnd()) {
        RawRecord record;
        QString readError;
        if (!readRecord(file, &record, &eof, &readError)) {
            if (eof) {
                break;
            }
            if (error != nullptr) {
                *error = readError;
            }
            return false;
        }
        if (opCode(record.header) == kOpConnection &&
            !mergeConnectionRecord(record, connections_, error)) {
            return false;
        }
    }

    summary_.connectionCount = connections_.size();
    if (connections_.isEmpty()) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：索引区未找到 topic connection。");
        }
        return false;
    }
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

bool Reader::streamMessages(const QString& filePath,
                            const QSet<int>& connectionIds,
                            const std::atomic_bool* cancellationRequested,
                            const std::function<bool(const SerializedMessage&)>& consumer,
                            const std::function<void(int64_t, int64_t)>& progress,
                            QString* error)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：无法打开文件 %1。").arg(filePath);
        }
        return false;
    }
    if (file.readLine() != QByteArray("#ROSBAG V2.0\n")) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：文件头不是 ROS1 bag v2.0。");
        }
        return false;
    }
    const int64_t fileSize = file.size();

    summary_.messageCount = 0;
    summary_.startTimestampNs = 0;
    summary_.endTimestampNs = 0;
    for (Connection& connection : connections_) {
        connection.messageCount = 0;
    }

    bool consumerStopped = false;
    auto processRecord = [this,
                          &connectionIds,
                          cancellationRequested,
                          &consumer,
                          error,
                          &consumerStopped](const RawRecord& record,
                                            auto&& processRecordRef) -> bool {
        if ((cancellationRequested && cancellationRequested->load()) || consumerStopped) {
            return false;
        }

        const uint8_t op = opCode(record.header);
        if (op == kOpChunk) {
            const QString compression = readFieldString(record.header, QStringLiteral("compression"));
            uint32_t uncompressedSize = 0;
            if (!readFieldU32(record.header, QStringLiteral("size"), &uncompressedSize)) {
                if (error != nullptr) {
                    *error = QStringLiteral("ROSbag 加载失败：chunk record 缺少 size 字段。");
                }
                return false;
            }

            QByteArray chunkData;
            if (compression == QStringLiteral("none")) {
                chunkData = record.data;
            } else if (compression == QStringLiteral("lz4")) {
                if (!decompressLz4Frame(record.data, uncompressedSize, &chunkData, error)) {
                    return false;
                }
            } else {
                if (error != nullptr) {
                    *error = QStringLiteral("ROSbag 加载失败：不支持压缩 chunk '%1'。").arg(compression);
                }
                return false;
            }

            QBuffer buffer(&chunkData);
            if (!buffer.open(QIODevice::ReadOnly)) {
                if (error != nullptr) {
                    *error = QStringLiteral("ROSbag 加载失败：无法读取 chunk。");
                }
                return false;
            }
            while (!buffer.atEnd()) {
                RawRecord inner;
                bool eof = false;
                QString innerError;
                if (!readRecord(buffer, &inner, &eof, &innerError)) {
                    if (eof) {
                        break;
                    }
                    if (error != nullptr) {
                        *error = innerError;
                    }
                    return false;
                }
                if (!processRecordRef(inner, processRecordRef)) {
                    return false;
                }
            }
            return true;
        }

        if (op != kOpMsgData) {
            return true;
        }

        uint32_t conn = 0;
        int64_t timestampNs = 0;
        if (!readFieldU32(record.header, QStringLiteral("conn"), &conn) ||
            !readFieldTimeNs(record.header, QStringLiteral("time"), &timestampNs)) {
            if (error != nullptr) {
                *error = QStringLiteral("ROSbag 加载失败：message data record 缺少 conn 或 time 字段。");
            }
            return false;
        }
        if (!connectionIds.isEmpty() && !connectionIds.contains(int(conn))) {
            return true;
        }

        SerializedMessage message;
        message.connectionId = int(conn);
        message.timestampNs = timestampNs;
        message.data = record.data;
        const int index = connectionIndexById(connections_, message.connectionId);
        if (index >= 0) {
            connections_[index].messageCount++;
            message.topic = connections_.at(index).topic;
            message.type = connections_.at(index).type;
        }
        summary_.messageCount++;
        if (summary_.messageCount == 1 || timestampNs < summary_.startTimestampNs) {
            summary_.startTimestampNs = timestampNs;
        }
        if (summary_.messageCount == 1 || timestampNs > summary_.endTimestampNs) {
            summary_.endTimestampNs = timestampNs;
        }
        consumerStopped = !consumer(message);
        return !consumerStopped;
    };

    while (!file.atEnd()) {
        if (cancellationRequested && cancellationRequested->load()) {
            return false;
        }
        RawRecord record;
        bool eof = false;
        QString readError;
        if (!readRecord(file, &record, &eof, &readError)) {
            if (eof) {
                break;
            }
            if (error != nullptr) {
                *error = readError;
            }
            return false;
        }
        progress(file.pos(), fileSize);
        if (!processRecord(record, processRecord)) {
            return false;
        }
    }
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

bool Reader::read(const QString& filePath, QString* error)
{
    clear();
    summary_.filePath = filePath;

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：无法打开文件 %1。").arg(filePath);
        }
        return false;
    }

    const QByteArray magic = file.readLine();
    if (magic != QByteArray("#ROSBAG V2.0\n")) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：文件头不是 ROS1 bag v2.0。");
        }
        return false;
    }

    auto processRecord = [this, error](const RawRecord& record, bool* stop, auto&& processRecordRef) -> bool {
        const uint8_t op = opCode(record.header);
        switch (op) {
        case kOpFileHeader: {
            uint32_t connCount = 0;
            uint32_t chunkCount = 0;
            readFieldU32(record.header, QStringLiteral("conn_count"), &connCount);
            readFieldU32(record.header, QStringLiteral("chunk_count"), &chunkCount);
            summary_.connectionCount = int(connCount);
            summary_.chunkCount = int(chunkCount);
            return true;
        }
        case kOpConnection: {
            return mergeConnectionRecord(record, connections_, error);
        }
        case kOpChunk: {
            const QString compression = readFieldString(record.header, QStringLiteral("compression"));
            uint32_t uncompressedSize = 0;
            if (!readFieldU32(record.header, QStringLiteral("size"), &uncompressedSize)) {
                if (error != nullptr) {
                    *error = QStringLiteral("ROSbag 加载失败：chunk record 缺少 size 字段。");
                }
                return false;
            }

            QByteArray chunkData;
            if (compression == QStringLiteral("none")) {
                chunkData = record.data;
            } else if (compression == QStringLiteral("lz4")) {
                if (!decompressLz4Frame(record.data, uncompressedSize, &chunkData, error)) {
                    return false;
                }
            } else {
                if (error != nullptr) {
                    *error = QStringLiteral("ROSbag 加载失败：不支持压缩 chunk '%1'。当前支持 uncompressed 和 lz4 ROS1 bag。")
                                 .arg(compression);
                }
                return false;
            }

            QBuffer buffer;
            buffer.setData(chunkData);
            if (!buffer.open(QIODevice::ReadOnly)) {
                if (error != nullptr) {
                    *error = QStringLiteral("ROSbag 加载失败：无法读取 chunk。");
                }
                return false;
            }
            while (!buffer.atEnd()) {
                RawRecord inner;
                bool eof = false;
                QString innerError;
                if (!readRecord(buffer, &inner, &eof, &innerError)) {
                    if (eof) {
                        break;
                    }
                    if (error != nullptr) {
                        *error = innerError;
                    }
                    return false;
                }
                if (!processRecordRef(inner, stop, processRecordRef)) {
                    return false;
                }
                if (*stop) {
                    return true;
                }
            }
            return true;
        }
        case kOpMsgData: {
            uint32_t conn = 0;
            int64_t timestampNs = 0;
            if (!readFieldU32(record.header, QStringLiteral("conn"), &conn) ||
                !readFieldTimeNs(record.header, QStringLiteral("time"), &timestampNs)) {
                if (error != nullptr) {
                    *error = QStringLiteral("ROSbag 加载失败：message data record 缺少 conn 或 time 字段。");
                }
                return false;
            }

            SerializedMessage message;
            message.connectionId = int(conn);
            message.timestampNs = timestampNs;
            message.data = record.data;

            const int index = connectionIndexById(connections_, int(conn));
            if (index >= 0) {
                connections_[index].messageCount++;
                message.topic = connections_.at(index).topic;
                message.type = connections_.at(index).type;
            }
            messages_.push_back(std::move(message));
            summary_.messageCount++;
            if (summary_.messageCount == 1 || timestampNs < summary_.startTimestampNs) {
                summary_.startTimestampNs = timestampNs;
            }
            if (summary_.messageCount == 1 || timestampNs > summary_.endTimestampNs) {
                summary_.endTimestampNs = timestampNs;
            }
            return true;
        }
        default:
            return true;
        }
    };

    bool stop = false;
    while (!file.atEnd() && !stop) {
        RawRecord record;
        bool eof = false;
        QString readError;
        if (!readRecord(file, &record, &eof, &readError)) {
            if (eof) {
                break;
            }
            if (error != nullptr) {
                *error = readError;
            }
            return false;
        }
        if (!processRecord(record, &stop, processRecord)) {
            return false;
        }
    }

    summary_.connectionCount = connections_.size();
    if (connections_.isEmpty()) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：未找到任何 topic connection。");
        }
        return false;
    }
    if (messages_.isEmpty()) {
        if (error != nullptr) {
            *error = QStringLiteral("ROSbag 加载失败：未找到 message data。已发现 topic: %1。").arg(topicListText(connections_));
        }
        return false;
    }

    std::sort(messages_.begin(), messages_.end(), [](const SerializedMessage& lhs, const SerializedMessage& rhs) {
        return lhs.timestampNs < rhs.timestampNs;
    });
    return true;
}

void Reader::clear()
{
    connections_.clear();
    messages_.clear();
    summary_ = Summary();
}

const QVector<Connection>& Reader::connections() const
{
    return connections_;
}

const QVector<SerializedMessage>& Reader::messages() const
{
    return messages_;
}

const Summary& Reader::summary() const
{
    return summary_;
}

} // namespace Rosbag

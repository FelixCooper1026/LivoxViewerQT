# ROS1 / ROS2 Bag 接入离线点云播放开发方案

## 1. 当前实现状态

### 1.1 2026-06-30 分支状态

`codex/slam` 分支目前已经实现 ROS1 / ROS2 bag 的读取、反序列化，并同时接入了离线 SLAM 与普通离线点云播放。

离线 SLAM 链路为：

```text
ROS1 / ROS2 bag → RosbagSlamSource → SlamInputFrame → FastLioSlamBackend
```

普通离线点云播放链路为：

```text
ROS1 / ROS2 bag → RosbagPlaybackSource → Playback::Source → PlaybackBar → PointCloudView
```

当前状态应定义为：

> ROS1 / ROS2 bag 已经接入离线 SLAM 和普通离线点云播放；本文前半部分保留原开发方案，底部“实施记录”记录已完成项和仍有限制。

---

## 2. 已实现能力确认

### 2.1 ROS1 Bag 读取

当前已经实现 ROS1 bag v2.0 文件读取。

支持内容：

- 读取 `#ROSBAG V2.0` 文件头
- 解析 record header
- 解析 connection record
- 解析 message data record
- 按时间戳排序 message
- 提取 topic、type、message data

当前限制：

- 仅支持 `compression=none`
- 暂不支持 `bz2`
- 暂不支持 `lz4`

因此第一版普通播放也应明确限制为：

```text
仅支持未压缩 ROS1 bag。
```

---

### 2.2 ROS2 Bag 读取

当前已经实现 ROS2 SQLite `.db3` bag 读取。

支持入口：

- 直接选择 `.db3`
- 选择 `metadata.yaml`
- 选择 `.yaml / .yml`

支持内容：

- 解析 `metadata.yaml` 中的 `relative_file_paths`
- 读取 ROS2 bag 的 `topics` 表
- 读取 ROS2 bag 的 `messages` 表
- 提取 topic id、topic name、type、serialization_format
- 提取 message timestamp 和 serialized data

当前限制：

- 仅支持 SQLite3 `.db3`
- 暂不支持 MCAP
- 暂不支持其他 ROS2 storage backend

---

### 2.3 消息反序列化

当前已经实现以下消息反序列化：

#### ROS1

- `livox_ros_driver/CustomMsg`
- `sensor_msgs/Imu`
- `sensor_msgs/PointCloud2`

#### ROS2

- `livox_ros_driver2/msg/CustomMsg`
- `sensor_msgs/msg/Imu`
- `sensor_msgs/msg/PointCloud2`

ROS2 反序列化基于 CDR reader，目前只支持可识别的小端 CDR 封装。

---

### 2.4 ROSbag 接入 SLAM

当前 `RosbagSlamSource` 已经完成以下工作：

- 根据后缀判断 ROS1 / ROS2：
  - `.bag` → ROS1 bag
  - `.db3 / .yaml / .yml` → ROS2 bag
- 自动选择 LiDAR topic
- 自动选择 IMU topic
- 解析 LiDAR 消息
- 解析 IMU 消息
- 生成 `SlamInputFrame`
- 附加 IMU 样本
- 检查 IMU 是否覆盖 LiDAR 帧
- 输出可供 FAST-LIO 使用的离线 SLAM 输入帧

当前离线 SLAM 支持的文件选择类型已经包括：

```text
*.pcap
*.pcapng
*.bag
*.db3
*.yaml
*.yml
```

---

## 3. 普通播放接入状态

当前 ROS1 / ROS2 bag 已经接入普通离线点云播放；本节保留原方案背景，并标记当前已经完成的接口状态。

现有播放抽象为：

```cpp
namespace Playback {

enum class SourceKind {
    Lvx2,
    Pcap,
    Rosbag
};

class Source {
public:
    virtual ~Source() = default;

    virtual bool load(const QString& filePath) = 0;
    virtual SourceKind kind() const = 0;
    virtual QString path() const = 0;
    virtual QString errorMessage() const = 0;
    virtual int frameCount() const = 0;
    virtual QVector<DeviceInfo> devices() const = 0;

    virtual bool readFrame(int frameIndex,
                           const QMap<uint32_t, bool>& deviceVisibility,
                           PointCloudFrame& frame) = 0;

    virtual QVector<ImuSample> readImuSamples(uint64_t startTimestampNs,
                                              uint64_t endTimestampNs) const;
};

}
```

当前 `SourceKind` 已包含 `Rosbag`，`RosbagPlaybackSource` 已经实现 `Playback::Source`，bag 可以复用现有播放条、播放按钮、滑动窗口、离线点云 tab、普通点云显示流程和离线 IMU 曲线。

---

## 4. 开发目标

将 ROS1 / ROS2 bag 接入普通离线点云播放系统，实现以下能力：

1. 通过菜单打开 ROS1 / ROS2 bag。
2. 支持拖拽 `.bag / .db3 / metadata.yaml` 到点云窗口。
3. 加载后创建普通离线点云 tab。
4. 支持播放 / 暂停。
5. 支持上一帧 / 下一帧 / 第一帧 / 最后一帧。
6. 支持进度条跳转。
7. 支持播放速度控制。
8. 支持普通帧播放模式。
9. 支持滑动窗口累积显示。
10. 支持设备显隐控制。
11. 支持反射率、距离、高程、线号、纯色等现有着色模式。
12. 支持 IMU 样本读取和 IMU 曲线显示。
13. 不启动 SLAM 后端。
14. 不依赖 FAST-LIO 处理结果。
15. 保留现有 ROSbag 离线 SLAM 功能。

最终目标结构：

```text
RosbagReader / Ros2BagReader
        ↓
RosMessageParsers
        ↓
RosbagFrameSource 或 RosbagSlamSource
        ↓
 ┌──────────────────────────┬──────────────────────────┐
 │ RosbagPlaybackSource      │ RosbagSlamSource          │
 │ 普通离线点云播放           │ FAST-LIO 离线建图          │
 └──────────────────────────┴──────────────────────────┘
        ↓                              ↓
Playback::Source / PlaybackBar         FastLioSlamBackend
```

---

## 5. 总体技术路线

采用最小侵入方式：

```text
ROS1 / ROS2 bag
    ↓
RosbagReader / Ros2BagReader
    ↓
RosMessageParsers
    ↓
RosbagSlamSource 或抽象后的 RosbagFrameSource
    ↓
RosbagPlaybackSource
    ↓
Playback::Source
    ↓
现有离线播放 UI
    ↓
PointCloudView
```

核心思想：

> 复用当前 `codex/slam` 分支已经完成的 bag 读取、反序列化和帧生成逻辑，只新增一个面向普通播放的 `Playback::Source` 适配层。

---

## 6. Phase 1：扩展 Playback 抽象

### 6.1 修改 `Playback::SourceKind`

文件：

```text
libs/Playback/include/Playback/PlaybackSource.h
```

当前：

```cpp
enum class SourceKind {
    Lvx2,
    Pcap
};
```

修改为：

```cpp
enum class SourceKind {
    Lvx2,
    Pcap,
    Rosbag
};
```

这样现有播放系统可以识别三类数据源：

```text
LVX2
PCAP
ROSbag
```

---

### 6.2 设备信息约定

ROSbag 中通常没有真实的设备 SN、IP、deviceType。

建议约定：

```cpp
Playback::DeviceInfo device;
device.lidarId = 1;
device.deviceType = 0;
device.lidarSn = lidarTopic;
device.modelDisplay = "ROS1 Livox CustomMsg";
```

或者：

```cpp
device.modelDisplay = "ROS2 Livox CustomMsg";
device.lidarSn = "/livox/lidar";
```

如果是 PointCloud2：

```cpp
device.modelDisplay = "ROS2 Livox PointCloud2";
device.lidarSn = "/livox/lidar";
```

这样可以复用现有设备显隐 UI，不需要为 ROSbag 单独重做设备面板。

---

## 7. Phase 2：新增 `RosbagPlaybackSource`

### 7.1 新增文件

建议新增：

```text
libs/Rosbag/include/Rosbag/RosbagPlaybackSource.h
libs/Rosbag/src/RosbagPlaybackSource.cpp
```

原因：

- bag 读取相关实现已经在 `libs/Rosbag`
- 普通播放只是 bag 的一种输出适配
- 避免把 ROSbag 相关代码散落到 `libs/Playback`

---

### 7.2 类定义建议

```cpp
#ifndef ROSBAG_ROSBAGPLAYBACKSOURCE_H
#define ROSBAG_ROSBAGPLAYBACKSOURCE_H

#include "Playback/PlaybackSource.h"
#include "Slam/Io/RosbagSlamSource.h"

class RosbagPlaybackSource final : public Playback::Source
{
public:
    explicit RosbagPlaybackSource(int frameDurationMs = 100);

    bool load(const QString& filePath) override;
    Playback::SourceKind kind() const override;
    QString path() const override;
    QString errorMessage() const override;
    int frameCount() const override;
    QVector<Playback::DeviceInfo> devices() const override;

    bool readFrame(int frameIndex,
                   const QMap<uint32_t, bool>& deviceVisibility,
                   PointCloudFrame& frame) override;

    QVector<Playback::ImuSample> readImuSamples(uint64_t startTimestampNs,
                                                uint64_t endTimestampNs) const override;

    void invalidateCache() override;

private:
    QString path_;
    QString errorMessage_;
    QVector<SlamInputFrame> frames_;
    QVector<SlamImuSample> imuSamples_;
    QVector<Playback::DeviceInfo> devices_;
    QString summaryText_;
    int frameDurationMs_ = 100;
};

#endif
```

---

## 8. Phase 3：支持普通播放不强制要求 IMU

当前 `RosbagSlamSource` 是为 SLAM 服务的，因此要求 IMU 存在，并且要求 IMU 覆盖 LiDAR 帧。

普通点云播放不应该有这个限制。

### 8.1 修改 `RosbagSlamSourceConfig`

文件：

```text
libs/Slam/include/Slam/Io/RosbagSlamSource.h
```

当前配置：

```cpp
struct RosbagSlamSourceConfig {
    QString lidarTopic;
    QString imuTopic;
    bool autoDetectTopics = true;
    bool useLivoxCustomTimebase = false;
    bool useHeaderStamp = true;
    bool requirePointOffsetTime = true;
    bool synthesizePointOffsetTime = false;
    bool allowLivoxDriver2PointCloud2 = false;
    bool allowLivoxDriverPointCloud2SynthesizedTime = false;
    int64_t lidarToImuTimeOffsetNs = 0;
    int frameDurationMs = 100;
};
```

建议新增：

```cpp
bool requireImu = true;
```

修改后：

```cpp
struct RosbagSlamSourceConfig {
    QString lidarTopic;
    QString imuTopic;
    bool autoDetectTopics = true;
    bool useLivoxCustomTimebase = false;
    bool useHeaderStamp = true;
    bool requirePointOffsetTime = true;
    bool synthesizePointOffsetTime = false;
    bool allowLivoxDriver2PointCloud2 = false;
    bool allowLivoxDriverPointCloud2SynthesizedTime = false;
    bool requireImu = true;
    int64_t lidarToImuTimeOffsetNs = 0;
    int frameDurationMs = 100;
};
```

### 8.2 修改 IMU topic 检查逻辑

当前逻辑是找不到 IMU topic 就失败。

应改为：

```cpp
const Rosbag::Connection* imuConnection = config_.imuTopic.isEmpty()
    ? autoSelectImuConnection(connections)
    : findConnectionByTopic(connections, config_.imuTopic);

if (imuConnection == nullptr && config_.requireImu) {
    errorMessage_ = QStringLiteral("ROSbag 加载失败：未找到 IMU topic。已发现 topic: %1。")
        .arg(topicListText(summary_.topics));
    if (error != nullptr) {
        *error = errorMessage_;
    }
    return false;
}
```

普通播放时：

```cpp
config.requireImu = false;
```

离线 SLAM 时：

```cpp
config.requireImu = true;
```

### 8.3 修改 IMU 覆盖检查逻辑

当前逻辑在无 IMU 或 IMU 不覆盖时会失败。

应改为：

```cpp
if (config_.requireImu) {
    if (!summary_.hasImu) {
        // SLAM 模式失败
        return false;
    }

    attachImuSamples(frames_, imuSamples, summary_);

    if (summary_.framesWithCompleteImuCoverage == 0) {
        // SLAM 模式失败
        return false;
    }
} else {
    if (!imuSamples.isEmpty()) {
        attachImuSamples(frames_, imuSamples, summary_);
    } else {
        summary_.messages.push_back(QStringLiteral("ROSbag 未包含 IMU topic，普通点云播放将仅显示点云。"));
    }
}
```

---

## 9. Phase 4：暴露 IMU 样本给普通播放

当前 `RosbagSlamSource::load()` 内部有局部变量：

```cpp
QVector<SlamImuSample> imuSamples;
```

普通播放需要复用这些 IMU 样本。

### 9.1 增加成员变量

文件：

```text
libs/Slam/include/Slam/Io/RosbagSlamSource.h
```

新增：

```cpp
const QVector<SlamImuSample>& imuSamples() const;
```

private 增加：

```cpp
QVector<SlamImuSample> imuSamples_;
```

### 9.2 保存 IMU 样本

文件：

```text
libs/Slam/src/Io/RosbagSlamSource.cpp
```

在解析完成后：

```cpp
imuSamples_ = imuSamples;
```

### 9.3 实现接口

```cpp
const QVector<SlamImuSample>& RosbagSlamSource::imuSamples() const
{
    return imuSamples_;
}
```

---

## 10. Phase 5：实现 `RosbagPlaybackSource::load()`

示例：

```cpp
bool RosbagPlaybackSource::load(const QString& filePath)
{
    path_ = filePath;
    errorMessage_.clear();
    frames_.clear();
    imuSamples_.clear();
    devices_.clear();

    RosbagSlamSourceConfig config;
    config.frameDurationMs = frameDurationMs_;
    config.requireImu = false;
    config.allowLivoxDriver2PointCloud2 = true;
    config.allowLivoxDriverPointCloud2SynthesizedTime = true;
    config.synthesizePointOffsetTime = true;

    RosbagSlamSource source(config);
    QString error;
    if (!source.load(filePath, &error)) {
        errorMessage_ = error;
        return false;
    }

    frames_ = source.frames();
    imuSamples_ = source.imuSamples();
    summaryText_ = source.summaryText();

    const RosbagSlamSourceSummary& summary = source.summary();

    Playback::DeviceInfo device;
    device.lidarId = 1;
    device.deviceType = 0;
    device.lidarSn = summary.lidarTopic;

    if (summary.format.contains(QStringLiteral("ROS2"), Qt::CaseInsensitive)) {
        device.modelDisplay = QStringLiteral("ROS2 %1").arg(summary.lidarFormat);
    } else {
        device.modelDisplay = QStringLiteral("ROS1 %1").arg(summary.lidarFormat);
    }

    devices_.push_back(device);

    return !frames_.isEmpty();
}
```

---

## 11. Phase 6：实现 `SlamInputFrame` 到 `PointCloudFrame` 转换

### 11.1 转换函数

```cpp
static PointCloudPoint toPointCloudPoint(const SlamPoint& point)
{
    PointCloudPoint result;
    result.x = point.x;
    result.y = point.y;
    result.z = point.z;
    result.r = 1.0f;
    result.g = 1.0f;
    result.b = 1.0f;
    result.reflectivity = point.reflectivity;
    result.tag = point.tag;
    result.line = point.line;
    result.spherical = false;
    result.theta = 0.0f;
    result.phi = 0.0f;
    result.depth = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    return result;
}
```

### 11.2 `readFrame()` 实现

```cpp
bool RosbagPlaybackSource::readFrame(int frameIndex,
                                     const QMap<uint32_t, bool>& deviceVisibility,
                                     PointCloudFrame& frame)
{
    frame = PointCloudFrame();

    if (frameIndex < 0 || frameIndex >= frames_.size()) {
        return false;
    }

    const SlamInputFrame& sourceFrame = frames_.at(frameIndex);
    const uint32_t lidarId = sourceFrame.sourceId == 0 ? 1 : sourceFrame.sourceId;

    frame.timestamp = uint64_t(std::max<int64_t>(0, sourceFrame.frameStartNs));

    if (deviceVisibility.contains(lidarId) && !deviceVisibility.value(lidarId)) {
        return true;
    }

    frame.points.reserve(sourceFrame.points.size());

    for (const SlamPoint& point : sourceFrame.points) {
        frame.points.push_back(toPointCloudPoint(point));
    }

    return true;
}
```

注意：实际字段名应以当前 `PointCloudFrame` 定义为准。如果 `timestamp` 字段名称不同，需要同步调整。

---

## 12. Phase 7：实现 `readImuSamples()`

```cpp
QVector<Playback::ImuSample> RosbagPlaybackSource::readImuSamples(uint64_t startTimestampNs,
                                                                  uint64_t endTimestampNs) const
{
    QVector<Playback::ImuSample> result;

    for (const SlamImuSample& sample : imuSamples_) {
        if (sample.timestampNs < 0) {
            continue;
        }

        const uint64_t timestamp = uint64_t(sample.timestampNs);
        if (timestamp < startTimestampNs || timestamp > endTimestampNs) {
            continue;
        }

        Playback::ImuSample out;
        out.lidarId = sample.lidarId == 0 ? 1 : sample.lidarId;
        out.timestampNs = timestamp;
        out.gyroX = float(sample.gyroRadPerSec[0]);
        out.gyroY = float(sample.gyroRadPerSec[1]);
        out.gyroZ = float(sample.gyroRadPerSec[2]);
        out.accX = float(sample.accelMps2[0]);
        out.accY = float(sample.accelMps2[1]);
        out.accZ = float(sample.accelMps2[2]);

        result.push_back(out);
    }

    return result;
}
```

---

## 13. Phase 8：新增 ROSbag 文件识别工具

新增文件：

```text
libs/Rosbag/include/Rosbag/RosbagPlaybackController.h
```

内容：

```cpp
#ifndef ROSBAG_ROSBAGPLAYBACKCONTROLLER_H
#define ROSBAG_ROSBAGPLAYBACKCONTROLLER_H

#include <QString>

namespace RosbagPlayback {

inline bool isSupportedFile(const QString& filePath)
{
    return filePath.endsWith(QStringLiteral(".bag"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".db3"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".yaml"), Qt::CaseInsensitive) ||
           filePath.endsWith(QStringLiteral(".yml"), Qt::CaseInsensitive);
}

}

#endif
```

---

## 14. Phase 9：新增 `loadRosbagPlaybackFile()`

### 14.1 头文件声明

文件：

```text
apps/LivoxViewer/LivoxViewerWindow.h
```

新增：

```cpp
bool loadRosbagPlaybackFile(const QString& filePath);
```

### 14.2 实现函数

文件：

```text
apps/LivoxViewer/actions/PlaybackControllerActions.cpp
```

新增：

```cpp
bool LivoxViewerWindow::loadRosbagPlaybackFile(const QString& filePath)
{
    const int tabId = createOfflinePointCloudTab(filePath);

    PlaybackControllerState* state = playbackStateForTab(tabId);
    if (!state) {
        return false;
    }

    state->loading = true;
    state->path = filePath;
    state->loadToken++;
    const quint64 currentToken = state->loadToken;

    setLvx2PlaybackPlaying(false);
    updateLvx2PlaybackUi();

    if (statusLabelBar) {
        statusLabelBar->setText(QString("正在加载ROSbag: %1").arg(QFileInfo(filePath).fileName()));
    }

    saveBoundPlaybackState();

    std::thread([this, filePath, tabId, currentToken]() {
        auto source = std::make_shared<RosbagPlaybackSource>(int(frameIntervalMs));
        const bool ok = source->load(filePath);
        const QString errorMessage = source->errorMessage();

        QMetaObject::invokeMethod(this, [this, tabId, currentToken, source, ok, errorMessage]() {
            PlaybackControllerState* state = playbackStateForTab(tabId);
            if (!state || currentToken != state->loadToken) {
                return;
            }

            state->loading = false;

            if (!ok) {
                state->path.clear();
                updateLvx2PlaybackUi();
                updateStatus();
                QMessageBox::warning(this, "播放ROSbag点云", errorMessage);
                closeVisualizationTab(tabId);
                return;
            }

            finishPlaybackSourceLoad(tabId, source);
        }, Qt::QueuedConnection);
    }).detach();

    return true;
}
```

需要 include：

```cpp
#include "Rosbag/RosbagPlaybackSource.h"
#include "Rosbag/RosbagPlaybackController.h"
```

---

## 15. Phase 10：修改文件打开入口

需要在现有文件打开逻辑中增加 ROSbag 判断。

伪代码：

```cpp
if (RosbagPlayback::isSupportedFile(filePath)) {
    return loadRosbagPlaybackFile(filePath);
}

if (PcapPlayback::isSupportedFile(filePath)) {
    return loadPcapPlaybackFile(filePath);
}

if (Lvx2::isSupportedFile(filePath)) {
    return loadLvx2PlaybackFile(filePath);
}
```

需要检查并修改的文件：

```text
apps/LivoxViewer/actions/FileActions.cpp
apps/LivoxViewer/actions/PlaybackControllerActions.cpp
apps/LivoxViewer/actions/PcapPlaybackActions.cpp
apps/LivoxViewer/actions/VisualizationWorkspaceActions.cpp
PointCloudView::dropEvent 相关处理
```

---

## 16. Phase 11：修改文件选择过滤器

普通离线点云播放打开文件时，过滤器建议改为：

```cpp
QStringLiteral("点云回放文件 (*.lvx2 *.pcap *.pcapng *.cap *.bag *.db3 *.yaml *.yml)")
QStringLiteral("LVX2 文件 (*.lvx2)")
QStringLiteral("PCAP 文件 (*.pcap *.pcapng *.cap)")
QStringLiteral("ROS1 Bag 文件 (*.bag)")
QStringLiteral("ROS2 Bag 文件 (*.db3 *.yaml *.yml)")
QStringLiteral("所有文件 (*.*)")
```

---

## 17. Phase 12：修改播放 UI 文案

当前 `finishPlaybackSourceLoad()` 文案只区分 PCAP 和 LVX2。

建议新增 helper：

```cpp
static QString playbackSourceDisplayName(Playback::SourceKind kind)
{
    switch (kind) {
    case Playback::SourceKind::Pcap:
        return QStringLiteral("Pcap");
    case Playback::SourceKind::Rosbag:
        return QStringLiteral("ROSbag");
    case Playback::SourceKind::Lvx2:
    default:
        return QStringLiteral("LVX2");
    }
}
```

状态栏：

```cpp
statusLabelBar->setText(QString("%1播放: %2").arg(sourceName, fileName));
```

日志：

```cpp
logMessage(QString("已加载%1文件: %2 (共%3帧)")
    .arg(sourceName)
    .arg(QDir::toNativeSeparators(playbackState.path))
    .arg(playbackState.source->frameCount()));
```

---

## 18. Phase 13：播放时间策略

当前播放条显示时间大概率基于：

```text
frameIndex × 100 ms
```

第一版可以继续沿用现有逻辑，确保改动小。

第二版再扩展真实时间轴。

### 18.1 第一版策略

- `frameCount()` 返回 bag 中有效 LiDAR 帧数
- `readFrame(i)` 返回第 i 个 LiDAR message 转换出的点云
- 播放条时间继续沿用现有 100 ms 逻辑
- 滑动窗口仍按现有 `frameIntervalMs` 合并若干 raw frame

### 18.2 第二版策略

扩展 `Playback::Source`：

```cpp
virtual uint64_t frameTimestampNs(int frameIndex) const;
virtual uint64_t durationNs() const;
```

然后播放条显示真实 bag 时间：

```text
当前时间 = frameTimestampNs(i) - frameTimestampNs(0)
总时长 = frameTimestampNs(last) - frameTimestampNs(0)
```

---

## 19. Phase 14：CMake 修改

文件：

```text
CMakeLists.txt
```

新增 source：

```cmake
libs/Rosbag/src/RosbagPlaybackSource.cpp
```

新增 header：

```cmake
libs/Rosbag/include/Rosbag/RosbagPlaybackSource.h
libs/Rosbag/include/Rosbag/RosbagPlaybackController.h
```

确保 include path 已包含：

```cmake
${CMAKE_CURRENT_SOURCE_DIR}/libs/Rosbag/include
```

当前 `codex/slam` 分支已包含 `libs/Rosbag/include`，因此通常只需要增加新文件到 `SOURCES` 和 `HEADERS`。

---

## 20. Phase 15：兼容性与限制

### 20.1 第一版支持范围

| 类型 | 支持情况 |
|---|---|
| ROS1 uncompressed `.bag` | 支持 |
| ROS1 compressed `.bag` | 暂不支持 |
| ROS2 `.db3` | 支持 |
| ROS2 `metadata.yaml` | 支持 |
| ROS2 MCAP | 暂不支持 |
| Livox CustomMsg | 支持 |
| Livox driver2 PointCloud2 | 支持 |
| Livox driver1 PointCloud2 | 支持，但点内时间可能合成 |
| sensor_msgs/Imu | 支持 |
| 只有点云无 IMU | 普通播放支持，SLAM 不支持 |
| 多 LiDAR topic | 第一版自动选择一个 topic |
| 多 IMU topic | 第一版自动选择一个 topic |

---

## 21. Phase 16：测试矩阵

### 21.1 编译测试

| 平台 | 测试项 |
|---|---|
| Windows + MSVC + Qt6 | CMake configure / build |
| Ubuntu + GCC + Qt6 | CMake configure / build |
| Windows | Qt SQL / SQLite plugin 是否能随程序发布 |
| Linux | Qt SQL / SQLite plugin 是否能随 AppImage / deb 发布 |
| 无 SQLite plugin | ROS2 db3 加载失败提示是否明确 |
| 无 Eigen / OpenMP | 不应影响普通播放；但当前分支 SLAM CMake 会强依赖，需要单独评估 |

### 21.2 数据测试

| 数据类型 | 预期 |
|---|---|
| ROS1 CustomMsg + Imu | 可播放点云，可读取 IMU |
| ROS1 CustomMsg 无 Imu | 普通播放可播放，SLAM 不允许 |
| ROS1 compressed bag | 明确提示暂不支持 |
| ROS2 metadata.yaml + db3 | 可播放 |
| 直接选择 ROS2 `.db3` | 可播放 |
| ROS2 driver2 PointCloud2 + timestamp | 可播放 |
| ROS1 / ROS2 driver1 PointCloud2 无 timestamp | 普通播放可播放，SLAM 提示精度风险 |
| 多 topic bag | 自动选择 `/livox/lidar` 优先 |
| 空 bag | 加载失败，提示未找到 message data |
| 非 Livox PointCloud2 | 加载失败，提示字段不匹配 |

---

## 22. Phase 17：建议实施顺序

1. 修改 `Playback::SourceKind`，增加 `Rosbag`。
2. 给 `RosbagSlamSourceConfig` 增加 `requireImu`。
3. 修改 `RosbagSlamSource`，支持普通播放模式下不强制 IMU。
4. 暴露 `RosbagSlamSource::imuSamples()`。
5. 新增 `RosbagPlaybackSource`。
6. 新增 `RosbagPlaybackController::isSupportedFile()`。
7. 新增 `LivoxViewerWindow::loadRosbagPlaybackFile()`。
8. 修改文件打开入口和拖拽入口。
9. 修改播放 UI 文案。
10. 修改 CMake。
11. Windows / Linux 编译验证。
12. 用 ROS1 / ROS2 样例 bag 验证播放。
13. 再考虑真实时间轴、多 topic 选择、压缩 bag 支持。

---

## 23. Codex 提示词建议

可以直接给 Codex 的开发提示词如下：

```text
请在当前 LivoxViewerQT 的 codex/slam 分支上实现 ROS1/ROS2 bag 接入普通离线点云播放功能。

当前分支已经有 RosbagReader、Ros2BagReader、RosMessageParsers、RosbagSlamSource，并且 ROS1/ROS2 bag 已经可以接入离线 SLAM。现在需要把 bag 也接入现有 Playback::Source 普通点云播放链路，复用现有播放条、离线点云 tab、播放/暂停/上一帧/下一帧/进度条/滑动窗口/着色管线。

要求：
1. 在 Playback::SourceKind 中新增 Rosbag。
2. 新增 RosbagPlaybackSource，继承 Playback::Source。
3. RosbagPlaybackSource 复用现有 RosbagSlamSource 或抽象出的公共 bag frame source，将 SlamInputFrame 转换为 PointCloudFrame。
4. 普通点云播放不应强制要求 IMU，因此需要给 RosbagSlamSourceConfig 增加 requireImu，SLAM 模式为 true，普通播放为 false。
5. RosbagPlaybackSource::readFrame 需要返回 PointCloudFrame。
6. RosbagPlaybackSource::readImuSamples 需要返回 Playback::ImuSample，用于复用现有 IMU 曲线显示。
7. 新增 RosbagPlaybackController::isSupportedFile，支持 .bag/.db3/.yaml/.yml。
8. 新增 LivoxViewerWindow::loadRosbagPlaybackFile。
9. 修改文件打开和拖拽入口，使 .bag/.db3/.yaml/.yml 进入普通离线点云播放，而不是只能进离线 SLAM。
10. 修改 finishPlaybackSourceLoad 的状态栏和日志文案，支持 LVX2/PCAP/ROSbag 三类源。
11. 更新 CMakeLists.txt，加入新增源文件和头文件。
12. 第一版只要求支持未压缩 ROS1 bag、ROS2 sqlite db3、metadata.yaml；暂不支持 ROS1 bz2/lz4 压缩和 ROS2 MCAP。
13. 保持现有离线 SLAM 功能不破坏。
14. 保持 Windows + Linux 可编译。

请先分析现有 Playback 和 RosbagSlamSource 代码结构，再分步骤修改。完成后给出变更文件列表、关键实现说明、编译注意事项和测试建议。
```

---

## 24. 最终建议

当前最优方案不是重新写一套 ROSbag 播放器，而是：

```text
复用 RosbagReader / Ros2BagReader / RosMessageParsers / RosbagSlamSource
新增 RosbagPlaybackSource 适配 Playback::Source
```

这样可以最小化重复解析逻辑，并保持：

```text
ROSbag → 普通播放
ROSbag → 离线 SLAM
```

两条链路并存。

---

## 25. 2026-06-29 实施记录

已按本文档第一版路线完成普通离线点云播放接入：

- `Playback::SourceKind` 已新增 `Rosbag`。
- `RosbagSlamSourceConfig` 已新增 `requireImu`，默认 `true`，保持离线 SLAM 仍强制要求 IMU。
- `RosbagSlamSource` 已支持 `requireImu=false`，普通播放可加载无 IMU 或 IMU 覆盖不完整的 ROSbag。
- `RosbagSlamSource::imuSamples()` 已暴露完整 IMU 样本，供普通播放 IMU 曲线复用。
- 已新增 `RosbagPlaybackSource`，将 `SlamInputFrame` 转换为 `PointCloudFrame`，并实现 `readImuSamples()`。
- 已新增 `RosbagPlaybackController::isSupportedFile()`，支持 `.bag / .db3 / .yaml / .yml`。
- 文件菜单“播放点云文件...”已支持 LVX2、PCAP、ROS1 bag、ROS2 db3/metadata.yaml 统一选择并自动分发。
- 点云窗口拖拽 `.bag / .db3 / .yaml / .yml` 已进入普通离线播放链路。
- 播放状态栏、日志和离线 IMU 设备列表已识别 ROSbag 数据源。
- CMake 已加入新增 ROSbag 普通播放源文件和头文件。

当前仍保持第一版限制：

- ROS1 bag 仅支持未压缩 chunk。
- ROS2 仅支持 sqlite `.db3` 与 `metadata.yaml`。
- 多 LiDAR topic 仍沿用自动选择一个 LiDAR topic 的策略。
- 播放时间轴沿用现有普通离线播放 100 ms 显示策略，尚未切换为 bag 真实时间轴。

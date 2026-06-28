# ROSbag 作为 SLAM 数据源开发方案

本文根据 `C:/Users/FelixCooper/Desktop/rosbag_implement.txt` 中与 GPT 沟通的方案，并结合当前 LivoxViewerQT SLAM 架构，整理“接收 ROSbag 文件作为 SLAM 数据源”的导入开发文档。

目标不是把 ROS 节点嵌入程序，而是把 ROSbag 解析为项目已有的 `SlamInputFrame`，继续复用现有 `FastLioSlamBackend`、`SlamUiBridge`、`SLAM` 点云 tab、状态 dock、导出功能和浮动控制条。

## 1. 当前项目状态

当前 SLAM 输入链路已经具备以下基础：

- 实时 SLAM：
  - `libs/Slam/include/Slam/Io/LiveLidarSlamSource.h`
  - `libs/Slam/src/Io/LiveLidarSlamSource.cpp`
  - 由 Livox SDK 点云/IMU callback 组装 `SlamInputFrame` 并推入 `SlamInputQueue`。
- 离线 PCAP SLAM：
  - `libs/Slam/include/Slam/Io/PcapSlamSource.h`
  - `libs/Slam/src/Io/PcapSlamSource.cpp`
  - 当前实现是 `PcapSlamSource::load()` 先解析完整文件为 `QVector<SlamInputFrame>`，然后 `apps/LivoxViewer/slam/SlamWindowActions.cpp` 中的离线 worker 按帧驱动 `FastLioSlamBackend`。
- 后端统一入口：
  - `libs/Slam/include/Slam/Core/SlamTypes.h`
  - `libs/Slam/include/Slam/Core/SlamInputQueue.h`
  - `libs/Slam/include/Slam/Backends/FastLio/FastLioSlamBackend.h`
  - `FastLioSlamBackend::processFrame(const SlamInputFrame&, SlamOutput*, QString*)`
- UI 已完成：
  - `工具 -> SLAM（在线）`
  - `工具 -> SLAM（离线）`
  - 独立 `SLAM` 点云 tab
  - 左侧 `SLAM` 图层 dock
  - 底部 `SLAM状态` dock
  - SLAM tab 浮动控制条
  - 轨迹导出 CSV/TUM
  - 完整全局点云地图导出 PCD/LAS

重要现实约束：

- 当前项目没有统一的 `ISlamSource` 抽象给 PCAP/Live 共用；`ISlamBackend` 只抽象了后端。
- 因此 ROSbag 第一版不应假设已有 `ISlamSource`，而应先按现有 `PcapSlamSource` 模式新增 `RosbagSlamSource::load()`，再在 Phase 2 抽象统一离线 source。
- 根 `CMakeLists.txt` 直接列出 SLAM 源文件，没有 `libs/Slam/CMakeLists.txt` 子工程；新增文件需要加入根 `SOURCES` 和 `HEADERS` 列表。

## 2. 产品目标

新增能力：

- 用户可以选择 ROSbag 文件作为离线 SLAM 数据源。
- ROSbag 内的 LiDAR/IMU topic 被解析为 `SlamInputFrame`。
- SLAM 后端仍使用现有 `FastLioSlamBackend`。
- 现有 SLAM UI、轨迹 overlay、世界系点云、机体系点云、完整地图保存、轨迹保存全部复用。

第一版推荐支持：

- ROS1 `.bag`
- 未压缩 bag
- 单 LiDAR topic + 单 IMU topic
- LiDAR topic 类型：
  - `livox_ros_driver2/CustomMsg`
  - `livox_ros_driver/CustomMsg` 可在同一 parser 框架中兼容，但本项目优先按本地 `B:/Workspace/livox_ros_driver2` 的实际定义落地
- IMU topic 类型：
  - `sensor_msgs/Imu`
- 一个 LiDAR message 对应一个 `SlamInputFrame`
- 离线最快速度建图，后续再考虑按 bag 时间回放

第一版不做：

- 不支持 ROS2 `.db3` / `.mcap`
- 不支持压缩 chunk
- 不支持多雷达 topic 同时融合
- 不支持无 IMU 数据启动 FAST_LIO
- 不支持缺少点内时间的 `PointCloud2` 直接启动 FAST_LIO
- 不启动 ROS master
- 不依赖用户安装 ROS
- 不链接 `roscpp`、`rosbag`、`sensor_msgs`、`catkin`、`ament`

## 3. 架构原则

### 3.1 不引入 ROS runtime

不推荐：

- 在 LivoxViewerQT 内启动 ROS node
- `find_package(rosbag)`
- `find_package(roscpp)`
- 直接链接 `sensor_msgs`
- 要求用户电脑装 ROS

原因：

- Windows 打包困难。
- Linux 发行版与 ROS 版本强绑定。
- Qt 程序会依赖 catkin/ament 环境。
- 与当前项目“无 ROS runtime、Windows/Linux 可交付”的方向冲突。

推荐：

- 新增轻量 ROSbag 读取层。
- 只解析项目需要的 ROS1 bag record、connection、chunk、message data。
- 对支持的 message type 做手写反序列化。
- 最终统一转换为当前项目已有结构：
  - `SlamPoint`
  - `SlamImuSample`
  - `SlamInputFrame`

### 3.2 不修改 FAST_LIO 后端输入接口

ROSbag 支持必须收敛到：

```cpp
bool FastLioSlamBackend::processFrame(const SlamInputFrame& frame,
                                      SlamOutput* output,
                                      QString* error);
```

不要把 ROS message 类型传进后端。

### 3.3 不在 UI 线程读取大 bag

ROSbag 解析和 SLAM 后端都必须在 worker 线程执行。

UI 线程只负责：

- 文件选择
- 显示摘要
- 更新状态
- 接收 `SlamOutput`
- 更新 OpenGL view
- 导出结果时启动后台导出线程

## 4. 推荐模块设计

### 4.1 新增 Rosbag 轻量读取层

新增目录：

```text
libs/Rosbag/include/Rosbag/RosbagTypes.h
libs/Rosbag/include/Rosbag/RosbagReader.h
libs/Rosbag/include/Rosbag/RosMessageParsers.h
libs/Rosbag/src/RosbagReader.cpp
libs/Rosbag/src/RosMessageParsers.cpp
```

职责划分：

- `RosbagTypes`
  - 定义 bag format、topic connection、serialized message、summary、错误枚举。
- `RosbagReader`
  - 读取 ROS1 `.bag`。
  - 扫描 connection/topic/type。
  - 读取 message record。
  - 按 bag 时间顺序输出 serialized message。
- `RosMessageParsers`
  - 解析 `std_msgs/Header`。
  - 解析 `sensor_msgs/Imu`。
  - 解析 `livox_ros_driver/CustomMsg`。
  - 解析 `livox_ros_driver2/CustomMsg`。

第一版 `RosbagReader` 只支持：

- ROS1 bag v2.0。
- uncompressed chunk。
- 必要 record：
  - bag header
  - connection
  - chunk
  - message data

后续扩展：

- bz2 chunk
- lz4 chunk
- index data 快速 seek
- ROS2 db3
- MCAP

### 4.2 新增 RosbagSlamSource

新增文件：

```text
libs/Slam/include/Slam/Io/RosbagSlamSource.h
libs/Slam/src/Io/RosbagSlamSource.cpp
```

第一版接口建议贴近 `PcapSlamSource`，便于低风险接入：

```cpp
struct RosbagSlamTopicInfo {
    QString topic;
    QString type;
    int64_t messageCount = 0;
};

struct RosbagSlamSourceConfig {
    QString lidarTopic;
    QString imuTopic;

    bool autoDetectTopics = true;
    bool useLivoxCustomTimebase = true;
    bool useHeaderStamp = true;
    bool requirePointOffsetTime = true;
    bool synthesizePointOffsetTime = false;
    bool allowLivoxDriver2PointCloud2 = false; // MVP 默认关，后续增强可打开

    int64_t lidarToImuTimeOffsetNs = 0;
    int frameDurationMs = 100;
};

struct RosbagSlamSourceSummary {
    QString filePath;
    QString format; // ros1_bag
    QVector<RosbagSlamTopicInfo> topics;

    QString lidarTopic;
    QString lidarType;
    QString lidarFormat; // LivoxCustomMsg / LivoxDriver2PointCloud2 / UnsupportedPclPointXyzi
    QString imuTopic;
    QString imuType;
    QString pointTimeMode; // CustomMsgOffsetNs / PointCloud2AbsoluteTimestampNs / Missing

    int frameCount = 0;
    uint64_t pointCount = 0;
    uint64_t imuSampleCount = 0;
    uint64_t lidarMessageCount = 0;
    uint64_t imuMessageCount = 0;
    uint64_t emptyLidarMessageCount = 0;
    int framesWithCompleteImuCoverage = 0;

    int64_t startTimestampNs = 0;
    int64_t endTimestampNs = 0;
    bool hasImu = false;
    bool hasPointOffsetTime = false;
    bool hasCompleteImuCoverage = false;
    QStringList messages;
};

class RosbagSlamSource {
public:
    explicit RosbagSlamSource(const RosbagSlamSourceConfig& config = {});

    void setConfig(const RosbagSlamSourceConfig& config);
    const RosbagSlamSourceConfig& config() const;

    bool load(const QString& filePath, QString* error);
    void clear();

    int frameCount() const;
    const SlamInputFrame& frameAt(int index) const;
    const QVector<SlamInputFrame>& frames() const;
    const RosbagSlamSourceSummary& summary() const;
    QString errorMessage() const;

private:
    RosbagSlamSourceConfig config_;
    QVector<SlamInputFrame> frames_;
    RosbagSlamSourceSummary summary_;
    QString errorMessage_;
};
```

说明：

- MVP 不需要立即实现 start/pause/resume/seek。
- 因为当前 PCAP 离线源也是 `load() -> frames()` 模式。
- 后续若抽象统一离线 source，可再引入 `ISlamOfflineSource`。

### 4.3 后续统一 source 抽象

当 ROSbag 与 PCAP 都稳定后，可以新增：

```cpp
class ISlamOfflineSource {
public:
    virtual ~ISlamOfflineSource() = default;
    virtual bool load(const QString& path, QString* error) = 0;
    virtual void clear() = 0;
    virtual int frameCount() const = 0;
    virtual const SlamInputFrame& frameAt(int index) const = 0;
    virtual QString summaryText() const = 0;
};
```

`PcapSlamSource` 和 `RosbagSlamSource` 再统一实现它。

不建议第一版同时做此重构，避免扩大改动面。

## 5. ROS message 解析策略

### 5.1 livox_ros_driver2 实际点云数据格式

本节基于本地 `B:/Workspace/livox_ros_driver2` 源码确认，版本定义位于 `src/include/livox_ros_driver2.h`，当前为 `1.2.6`。

关键源码位置：

- `msg/CustomMsg.msg`
- `msg/CustomPoint.msg`
- `src/lddc.h`
- `src/lddc.cpp`
- `src/comm/comm.h`
- `src/comm/pub_handler.cpp`
- `launch_ROS1/msg_MID360.launch`
- `launch_ROS1/rviz_MID360.launch`

#### 5.1.1 发布格式选择

`src/lddc.h` 中 `TransferType` 定义：

```cpp
typedef enum {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg = 1,
  kPclPxyziMsg = 2,
  kLivoxImuMsg = 3,
} TransferType;
```

ROS1 launch 默认值：

- `launch_ROS1/msg_MID360.launch`
  - `xfer_format=1`
  - 点云 topic 默认 `/livox/lidar`
  - IMU topic 默认 `/livox/imu`
  - `publish_freq=10.0`
  - `enable_lidar_bag=true`
  - `enable_imu_bag=true`
- `launch_ROS1/rviz_MID360.launch`
  - `xfer_format=0`
  - 点云 topic 仍是 `/livox/lidar`
  - 用于 RViz 显示的默认格式是 `sensor_msgs/PointCloud2`

因此 ROSbag SLAM 的 MVP 应优先支持 `xfer_format=1` 录制出的 `livox_ros_driver2/CustomMsg`。`PointCloud2` 可作为后续增强，不应先于 CustomMsg。

topic 命名：

- `multi_topic=0` 时：
  - 点云：`/livox/lidar`
  - IMU：`/livox/imu`
- `multi_topic=1` 时：
  - 点云：`/livox/lidar_<ip_with_underscore>`
  - IMU：`/livox/imu_<ip_with_underscore>`

自动 topic 识别必须同时支持全局 topic 和 multi-topic 名称。

#### 5.1.2 Livox CustomMsg 精确定义

`msg/CustomMsg.msg`：

```text
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
```

`msg/CustomPoint.msg`：

```text
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
```

`src/lddc.cpp` 的实际赋值语义：

- `timebase = pkg.base_time`
- `header.stamp = timebase`
- `point_num = pkg.points_num`
- `lidar_id = lds_->lidars_[index].handle`
- `point.x/y/z = points[i].x/y/z`
- `point.reflectivity = points[i].intensity`
- `point.tag = points[i].tag`
- `point.line = points[i].line`
- `point.offset_time = uint32(points[i].offset_time - pkg.base_time)`

`src/comm/pub_handler.cpp` 中 `points[i].offset_time` 来自：

```cpp
point.offset_time = pkt.time_stamp + i * pkt.point_interval;
```

结论：

- `timebase` 是当前发布帧首点绝对时间，单位 ns。
- `offset_time` 是相对 `timebase` 的点内时间，单位 ns。
- `header.stamp` 与 `timebase` 表达同一个帧起始时间。
- `offset_time` 不需要猜单位，按 ns 解析。
- `point_num` 应与 `points.size()` 一致；不一致时 `load()` 失败。

#### 5.1.3 PointCloud2 精确定义

`xfer_format=0` 时，`src/lddc.cpp::InitPointcloud2MsgHeader()` 创建 7 个字段：

| 字段 | offset | datatype | 含义 |
|---|---:|---|---|
| `x` | 0 | `FLOAT32` | X，m |
| `y` | 4 | `FLOAT32` | Y，m |
| `z` | 8 | `FLOAT32` | Z，m |
| `intensity` | 12 | `FLOAT32` | 反射率 |
| `tag` | 16 | `UINT8` | Livox tag |
| `line` | 17 | `UINT8` | laser line |
| `timestamp` | 18 | `FLOAT64` | 每点绝对时间，ns |

`point_step = sizeof(LivoxPointXyzrtlt)`。该结构体在 `src/comm/comm.h` 中受 `#pragma pack(1)` 影响，布局为 26 字节：

```cpp
typedef struct {
  float x;
  float y;
  float z;
  float reflectivity;
  uint8_t tag;
  uint8_t line;
  double timestamp;
} LivoxPointXyzrtlt;
```

`timestamp` 字段不是相对时间，而是 `points[i].offset_time` 的绝对 ns 时间。若后续支持 driver2 的 `PointCloud2`，转换规则应为：

```cpp
frameStartNs = header.stamp;
pointAbsoluteNs = int64_t(point.timestamp);
point.offsetNs = pointAbsoluteNs - frameStartNs;
```

必须校验：

- `height == 1`
- `point_step >= 26`
- 存在上述 7 个字段
- `timestamp` 字段为 `FLOAT64`
- `timestamp - header.stamp` 非负且落在合理帧周期内

如果 `PointCloud2` 缺少 `timestamp`，或者字段不是 Livox driver2 的布局，第一版必须拒绝启动，而不是用点序静默合成。

#### 5.1.4 PCL PointXYZI 格式

`xfer_format=2` 时发布 `pcl::PointCloud<pcl::PointXYZI>`，`src/lddc.cpp::FillPointsToPclMsg()` 只保留：

- `x`
- `y`
- `z`
- `intensity`

该格式丢失：

- 点内时间
- `line`
- `tag`

因此不能作为 FAST_LIO 默认输入。遇到这类 bag 应返回 `Failed`：

```text
ROSbag 加载失败：topic /livox/lidar 使用 pcl::PointCloud<pcl::PointXYZI>，缺少点内时间，无法作为 FAST_LIO 输入。
```

#### 5.1.5 IMU 数据格式

`src/lddc.cpp::InitImuMsg()` 发布 `sensor_msgs/Imu`：

- `header.frame_id = "livox_frame"`
- `header.stamp = imu_data.time_stamp`
- `angular_velocity.x/y/z = gyro_x/y/z`
- `linear_acceleration.x/y/z = acc_x/acc_y/acc_z`

协方差没有写入有效值，第一版不进入 `SlamImuSample`。

#### 5.1.6 ROS1 反序列化注意事项

第一版手写 parser 需要按 ROS1 serialization 顺序读取：

- `std_msgs/Header`
  - `uint32 seq`
  - `time stamp`：`uint32 sec` + `uint32 nsec`
  - `string frame_id`：`uint32 length` + bytes
- `CustomMsg`
  - `Header`
  - `uint64 timebase`
  - `uint32 point_num`
  - `uint8 lidar_id`
  - `uint8[3] rsvd`
  - `uint32 points_array_length`
  - repeated `CustomPoint`
- `CustomPoint`
  - `uint32 offset_time`
  - `float32 x`
  - `float32 y`
  - `float32 z`
  - `uint8 reflectivity`
  - `uint8 tag`
  - `uint8 line`

需要集中做边界检查，任何越界或数组长度不一致都应 `load()` 失败。

### 5.2 时间戳统一

内部统一使用纳秒：

```cpp
int64_t timestampNs;
```

ROS1 time：

- `uint32 sec`
- `uint32 nsec`

转换：

```cpp
int64_t rosTimeToNs(uint32_t sec, uint32_t nsec)
{
    return int64_t(sec) * 1000000000LL + int64_t(nsec);
}
```

LiDAR 帧时间优先级：

1. Livox CustomMsg `timebase`
2. driver2 PointCloud2 `header.stamp`
3. `header.stamp`
4. bag record receive time

MVP 建议：

- Livox CustomMsg 优先用 `timebase`。
- `timebase == 0` 时 fallback 到 `header.stamp`。
- 记录 summary message 说明实际使用的时间源。

### 5.3 Livox CustomMsg

优先支持：

- `livox_ros_driver2/CustomMsg`
- `livox_ros_driver/CustomMsg`，字段兼容时可走同一个 parser

字段按本地 `livox_ros_driver2` 实际定义解析：

- `header`
- `timebase`
- `point_num`
- `lidar_id`
- `rsvd[3]`
- `points[]`
  - `offset_time`
  - `x`
  - `y`
  - `z`
  - `reflectivity`
  - `tag`
  - `line`

转换为 `SlamPoint`：

```cpp
SlamPoint point;
point.x = source.x;
point.y = source.y;
point.z = source.z;
point.reflectivity = source.reflectivity;
point.tag = source.tag;
point.line = source.line;
point.hasLine = true;
point.offsetNs = source.offset_time;
point.hasOffsetTime = true;
```

解析规则：

- `offset_time` 单位固定按 ns 处理。
- `timebase` 单位固定按 ns 处理。
- `frame.frameStartNs = timebase`。
- `frame.frameEndNs = timebase + max(offset_time)`。
- `point_num != points_array_length` 时失败。
- `points_array_length == 0` 时丢弃该 LiDAR message，并在 summary 计数。

帧时间：

```cpp
frame.frameStartNs = timebase;
frame.frameEndNs = frame.frameStartNs + max(point.offsetNs);
```

### 5.4 sensor_msgs/Imu

解析字段：

- `header.stamp`
- `angular_velocity.x/y/z`
- `linear_acceleration.x/y/z`

转换为 `SlamImuSample`：

```cpp
SlamImuSample sample;
sample.timestampNs = headerStampNs;
sample.gyroRadPerSec[0] = angular_velocity.x;
sample.gyroRadPerSec[1] = angular_velocity.y;
sample.gyroRadPerSec[2] = angular_velocity.z;
sample.accelMps2[0] = linear_acceleration.x;
sample.accelMps2[1] = linear_acceleration.y;
sample.accelMps2[2] = linear_acceleration.z;
```

单位要求：

- gyro: rad/s
- accel: m/s²

不处理：

- covariance 暂不进入 `SlamImuSample`。

### 5.5 sensor_msgs/PointCloud2

第一版不作为必交付，但本地 `livox_ros_driver2` 的 `xfer_format=0` 具备可确定解析的字段布局，可作为第二优先级增强。

通用字段映射仍需保留：

- `x`
- `y`
- `z`
- `intensity` / `reflectivity`
- `time` / `timestamp` / `offset_time` / `t` / `curvature`
- `ring` / `line`

driver2 专用规则：

- 若 topic type 为 `sensor_msgs/PointCloud2`
- 且字段精确包含 `x/y/z/intensity/tag/line/timestamp`
- 且 `timestamp` 为 `FLOAT64`
- 则 `timestamp` 按每点绝对 ns 时间处理
- `offsetNs = timestamp - header.stamp`

通用规则：

- 没有点内时间字段时，默认拒绝启动 FAST_LIO。
- 可以后续提供“按点序合成点内时间”的高级选项。
- 合成时必须标记 `SlamTimeSource::SynthesizedFromPacketInterval` 或 summary warning。

合成公式：

```cpp
point.offsetNs = i * frameDurationNs / pointCount;
```

但这对 Livox 非重复扫描并不严格可靠，所以不能默认启用。

## 6. SlamInputFrame 组装

ROSbag 第一版采用：

- 一个 LiDAR message = 一个 `SlamInputFrame`

流程：

1. 按时间读取所有 IMU message，形成 IMU buffer。
2. 按时间读取 LiDAR CustomMsg。
3. 解析点云为 `QVector<SlamPoint>`。
4. 计算 `frameStartNs` 和 `frameEndNs`。
5. 从 IMU buffer 中截取覆盖 `[frameStartNs, frameEndNs]` 的样本。
6. 生成 `SlamInputFrame`。
7. 设置：
   - `hasPointOffsetTime`
   - `hasCompleteImuCoverage`
   - `timeSource`
   - `sourceName`
8. 存入 `frames_`。

IMU 覆盖判定沿用当前实时/PCAP 语义：

```cpp
frame.hasCompleteImuCoverage =
    !frame.imuSamples.isEmpty() &&
    frame.imuSamples.first().timestampNs <= frame.frameStartNs &&
    frame.imuSamples.last().timestampNs >= frame.frameEndNs;
```

边缘帧处理：

- bag 开头/结尾 IMU 不完整时可以丢弃边缘帧。
- 如果全部帧都没有完整 IMU 覆盖，`load()` 返回失败。
- 错误状态统一为 `Failed`，错误信息必须说明原因。

## 7. UI 接入方案

### 7.1 文件选择入口

当前入口：

- `工具 -> SLAM（离线）`
- `LivoxViewerWindow::startOfflineSlamFromMenu()`
- `LivoxViewerWindow::loadOfflineSlamPcap()`

建议改造：

- 将 `loadOfflineSlamPcap()` 重命名为更通用的 `loadOfflineSlamSource()`。
- 保留旧函数作为私有兼容 wrapper，降低改动风险：

```cpp
bool LivoxViewerWindow::loadOfflineSlamSource();
bool LivoxViewerWindow::loadOfflineSlamPcap(); // 调用 loadOfflineSlamSource()
```

文件过滤器：

```text
SLAM 数据源 (*.pcap *.bag)
PCAP 文件 (*.pcap)
ROS1 Bag 文件 (*.bag)
所有文件 (*.*)
```

内部状态建议从：

```cpp
QString slamOfflinePcapPath;
```

扩展为：

```cpp
enum class SlamOfflineSourceKind {
    None,
    Pcap,
    Rosbag
};

SlamOfflineSourceKind slamOfflineSourceKind = SlamOfflineSourceKind::None;
QString slamOfflineSourcePath;
```

为了降低一次性改动，也可以第一步保留 `slamOfflinePcapPath`，新增：

```cpp
QString slamOfflineSourcePath;
QString slamOfflineSourceDisplayName;
```

但最终应统一命名，避免后续 UI 显示 “PCAP” 但实际加载 bag。

### 7.2 SLAM tab 标题和控制条

SLAM tab 仍使用 `SLAM`。

浮动控制条 source 文案建议：

```text
离线 SLAM | 未运行 | F:\xxx.bag
```

如果来源是 ROSbag，可显示：

```text
离线 SLAM | ROSbag | 未运行 | F:\xxx.bag
```

### 7.3 ROSbag 摘要 UI

加载 ROSbag 后应在日志和状态栏输出摘要。

建议新增一个简洁 dialog 或复用状态日志：

```text
[SLAM] 已加载 ROSbag:
格式: ROS1 bag
LiDAR topic: /livox/lidar
LiDAR type: livox_ros_driver2/CustomMsg
LiDAR format: Livox CustomMsg
IMU topic: /livox/imu
IMU type: sensor_msgs/Imu
时间范围: 123.4 s
点云帧数: 1234
IMU 样本数: 123456
点内时间: CustomMsg offset_time(ns)
IMU 覆盖: 完整 1200 / 1234
```

若自动识别 topic 有歧义，需要弹出选择 dialog。

第一版可以先自动选择：

- LiDAR topic 优先：
  - `/livox/lidar`
  - `/livox/lidar_<ip_with_underscore>`
  - `/livox/lidar_msg`
  - 第一个 type 为 `livox_ros_driver2/CustomMsg` 或 `livox_ros_driver/CustomMsg` 的 topic
  - 后续增强允许时，再选择字段布局匹配 driver2 的 `sensor_msgs/PointCloud2`
- IMU topic 优先：
  - `/livox/imu`
  - `/livox/imu_<ip_with_underscore>`
  - `/livox/imu_data`
  - 第一个 type 为 `sensor_msgs/Imu` 的 topic

若找不到：

- `load()` 失败。
- 状态显示 `Failed`。
- 错误信息明确列出已发现 topic/type。

### 7.4 首选项 SLAM 页

建议新增 “ROSbag 输入” 小节：

- LiDAR topic：默认自动
- IMU topic：默认自动
- 优先使用 Livox CustomMsg timebase：默认开
- 要求点内时间：默认开
- 允许解析 Livox driver2 PointCloud2：默认关，建议等 CustomMsg 跑通后再开放
- 允许 PointCloud2 合成点内时间：默认关，MVP 可先不做
- LiDAR/IMU 时间偏移 ns：默认 0

MVP 如果不想扩 UI，可以先只做自动识别和日志摘要；但文档要求进入产品化时补设置项。

## 8. 离线 worker 接入

当前 `SlamWindowActions.cpp` 离线 worker 逻辑：

```cpp
PcapSlamSource source(config.inputFrameDurationMs);
if (!source.load(pcapPath, &error)) {
    postOutput(statusOutput(SlamStatusCode::Failed, error));
    ...
}

for (const SlamInputFrame& frame : source.frames()) {
    backend.processFrame(frame, &output, &error);
}
```

建议抽出 helper：

```cpp
bool loadOfflineFramesForSlam(const QString& path,
                              SlamOfflineSourceKind kind,
                              const SlamRuntimeConfig& runtimeConfig,
                              QVector<SlamInputFrame>* frames,
                              QString* summaryText,
                              QString* error);
```

内部：

```cpp
switch (kind) {
case SlamOfflineSourceKind::Pcap:
    PcapSlamSource pcap(runtimeConfig.inputFrameDurationMs);
    ...
case SlamOfflineSourceKind::Rosbag:
    RosbagSlamSource rosbag(configFromSettings);
    ...
}
```

这样不需要第一版就引入多态接口。

启动失败语义：

- ROSbag 不存在：`Failed`
- 格式不支持：`Failed`
- 未找到 LiDAR topic：`Failed`
- 未找到 IMU topic：`Failed`
- 不支持的 message type：`Failed`
- 点内时间缺失：`Failed`
- `PointCloud2` 字段不匹配 driver2 布局：`Failed`
- `pcl::PointCloud<pcl::PointXYZI>` 缺少点内时间：`Failed`
- IMU 覆盖全不完整：`Failed`

状态面板的“错误信息”应显示具体原因。

## 9. CMake 接入

当前项目根 `CMakeLists.txt` 直接管理源文件。

需要新增到 `SOURCES`：

```cmake
libs/Rosbag/src/RosbagReader.cpp
libs/Rosbag/src/RosMessageParsers.cpp
libs/Slam/src/Io/RosbagSlamSource.cpp
```

需要新增到 header 列表：

```cmake
libs/Rosbag/include/Rosbag/RosbagTypes.h
libs/Rosbag/include/Rosbag/RosbagReader.h
libs/Rosbag/include/Rosbag/RosMessageParsers.h
libs/Slam/include/Slam/Io/RosbagSlamSource.h
```

需要新增 include path：

```cmake
${CMAKE_CURRENT_SOURCE_DIR}/libs/Rosbag/include
```

不新增 ROS 相关 `find_package`。

压缩支持后续再加：

- bz2：可使用系统 zlib/bzip2 或 `third-party` 下 vendored source。
- lz4：若引入第三方库，按项目现有规则放在 `third-party`，并更新 `scripts/setup_third_party.ps1`。

MVP uncompressed ROS1 bag 不需要新增第三方依赖。

## 10. 开发阶段拆分

### Phase R1：文档与接口骨架

目标：

- 新增 `RosbagSlamSource` 头/源文件。
- 新增 `libs/Rosbag` 基础类型。
- CMake 可编译。
- 暂不解析真实 bag。

验收：

- Windows 编译通过。
- 现有 PCAP/在线 SLAM 行为不变。

### Phase R2：ROS1 bag metadata 扫描

目标：

- 打开 `.bag`。
- 校验 magic header。
- 扫描 connection。
- 输出 topic/type/message count/time range。

验收：

- 对不支持/损坏 bag 给出明确错误。
- 对含 `/livox/lidar` 和 `/livox/imu` 的 bag 能输出摘要。

### Phase R3：反序列化 Livox CustomMsg + sensor_msgs/Imu

目标：

- 按本地 `livox_ros_driver2` 1.2.6 的 `.msg` 精确定义解析 `livox_ros_driver2/CustomMsg`。
- 字段兼容时解析 `livox_ros_driver/CustomMsg`。
- 解析 `sensor_msgs/Imu`。
- 转成 `SlamPoint` 和 `SlamImuSample`。

验收：

- 能统计点数、IMU 样本数。
- `CustomMsg.timebase` 正确作为 `frameStartNs`。
- `CustomPoint.offset_time` 按 ns 正确写入 `SlamPoint::offsetNs`。
- `point_num` 与实际 points 数组长度不一致时失败。
- header/timebase 时间戳正确转换为 ns。

### Phase R4：组装 SlamInputFrame

目标：

- 一个 LiDAR message 生成一个 `SlamInputFrame`。
- 附加 IMU coverage。
- 生成 `frames_` 和 summary。

验收：

- `frameCount > 0`
- `hasPointOffsetTime == true`
- 对完整 bag，`framesWithCompleteImuCoverage` 高于可接受阈值。
- 对无 IMU bag，`load()` 失败并给出错误。

### Phase R5：UI 接入离线 SLAM

目标：

- `工具 -> SLAM（离线）` 文件选择支持 `.bag`。
- 根据扩展名选择 PCAP 或 ROSbag source。
- 浮动控制条显示实际 source path。
- 点击 `启动` 后运行 ROSbag 离线 SLAM。

验收：

- 选择 `.pcap` 行为不变。
- 选择 `.bag` 后不自动启动，等待浮动控制条 `启动`。
- 轨迹/点云/status dock 正常刷新。
- 错误信息能显示 ROSbag 解析失败原因。

### Phase R6：导出与回归验证

目标：

- ROSbag SLAM 输出复用现有轨迹 CSV/TUM、地图 PCD/LAS 保存。
- 完整全局地图仍来自后端 `newGlobalMapPoints` 缓存，不从 OpenGL 读取。

验收：

- CSV/TUM 能保存。
- PCD/LAS 能保存。
- 关闭 SLAM tab 后，左侧 SLAM dock 和底部 SLAM状态 dock 隐藏。
- 再次加载 PCAP 不受影响。

## 11. 测试计划

### 11.1 单元测试建议

新增测试目标或开发期 CLI 工具：

```text
tools/RosbagInspect
```

功能：

- 打印 bag topic/type。
- 打印消息数量。
- 打印 LiDAR/IMU 时间范围。
- 打印前 3 帧 `SlamInputFrame` 摘要。

测试用例：

- `launch_ROS1/msg_MID360.launch` 默认 `xfer_format=1` 录制的 `livox_ros_driver2/CustomMsg + sensor_msgs/Imu` bag。
- multi-topic 模式下的 `/livox/lidar_<ip>` + `/livox/imu_<ip>` bag。
- 缺 IMU bag。
- 缺 LiDAR bag。
- 不支持 message type bag。
- 损坏 bag。
- 空 bag。
- `xfer_format=0` 的 driver2 `PointCloud2` bag：MVP 阶段应明确提示暂不支持；后续增强开启后验证 `timestamp` 绝对 ns 转 `offsetNs`。
- `PointCloud2` 无 `timestamp` 字段 bag。
- `xfer_format=2` 的 `pcl::PointCloud<pcl::PointXYZI>` bag：必须失败并提示缺少点内时间。

### 11.2 UI 手动验收

1. `工具 -> SLAM（离线）`
2. 选择 ROS1 `.bag`
3. 程序创建/切换 `SLAM` 点云 tab
4. 左侧显示 SLAM 图层 dock
5. 底部显示 SLAM 状态 dock
6. 不自动启动
7. 点击浮动控制条 `启动`
8. 状态从 `Starting` 进入 `Running` 或明确 `Failed`
9. 轨迹 overlay 更新
10. 世界系点云显示
11. 保存轨迹 CSV/TUM
12. 保存完整全局地图 PCD/LAS
13. 关闭 SLAM tab 后 SLAM dock 自动隐藏

### 11.3 回归测试

必须确认不破坏：

- 在线 SLAM 无数据流时显示 `Failed` 和错误信息。
- PCAP 离线 SLAM。
- PCAP 普通播放。
- LVX2 播放。
- 日志/SLAM状态 dock tabify。
- 左侧设备 dock 默认启动不 raise SLAM tab。

## 12. 风险与应对

| 风险 | 影响 | 应对 |
|---|---|---|
| ROS1 bag 格式复杂 | 解析失败或兼容性不足 | MVP 只支持 uncompressed bag；错误信息明确 |
| Livox CustomMsg 存在版本差异 | 字段布局不同 | 根据 `connection.type` 分派 parser；为 driver1/driver2 分开 parser |
| offset_time 单位不一致 | 去畸变时间错误 | 对本地 driver2 固定按 ns；driver1 若兼容也必须在 summary 中显示时间源 |
| IMU 覆盖不足 | FAST_LIO 初始化失败 | load 阶段 coverage 检查；失败用 `Failed` |
| PointCloud2 字段不统一 | 无法解析点内时间/line | MVP 默认不支持；后续只对 driver2 精确字段布局先开放 |
| 大 bag 内存占用高 | 全量 `frames_` 占内存 | MVP 可接受；Phase 2 改流式读取或 `.slamcache` |
| Windows 打包依赖 | 交付困难 | 不引入 ROS runtime；第三方依赖放 `third-party` |
| UI 线程卡顿 | 体验差 | 所有 load/parse/SLAM 在 worker 线程 |

## 13. 建议的数据源错误信息

错误文案应具体、可操作。

示例：

```text
ROSbag 加载失败：不支持压缩 chunk。当前第一版仅支持 uncompressed ROS1 bag。
```

```text
ROSbag 加载失败：未找到 LiDAR topic。已发现 topic: /camera/image(sensor_msgs/Image), /livox/imu(sensor_msgs/Imu)。
```

```text
ROSbag 加载失败：LiDAR topic /points 类型 sensor_msgs/PointCloud2 缺少 time/offset_time 字段，无法为 FAST_LIO 提供点内时间。
```

```text
ROSbag 加载失败：LiDAR topic /livox/lidar 是 sensor_msgs/PointCloud2，但字段不匹配 livox_ros_driver2 布局。需要 x/y/z/intensity/tag/line/timestamp，且 timestamp 为 FLOAT64。
```

```text
ROSbag 加载失败：LiDAR topic /livox/lidar 使用 pcl::PointCloud<pcl::PointXYZI>，缺少点内时间，无法作为 FAST_LIO 输入。
```

```text
ROSbag 加载失败：IMU 样本未覆盖任何 LiDAR 帧。请检查 /livox/imu topic、时间戳和 LiDAR/IMU 时间偏移配置。
```

## 14. 后续扩展路线

### 14.1 PointCloud2 支持

新增：

- 第一阶段只支持 `livox_ros_driver2` 的精确布局：`x/y/z/intensity/tag/line/timestamp`。
- `timestamp` 按每点绝对 ns 处理，转换为 `offsetNs = timestamp - header.stamp`。
- 字段扫描。
- 字段映射配置。
- endian / point_step / row_step 处理。
- time 字段单位推断。

默认策略：

- 有 time 字段才允许启动。
- 无 time 字段默认 `Failed`。
- 不对缺少时间字段的 PointCloud2 默认按点序合成点内时间。

### 14.2 ROS2 db3 支持

新增：

- 读取 `metadata.yaml`。
- SQLite 读取 topics/messages。
- CDR 反序列化。

可选第三方：

- SQLite：Qt 自带 SQL 模块或直接 sqlite amalgamation。
- YAML：可以手写 metadata 最小解析，或 third-party yaml-cpp。

### 14.3 MCAP 支持

新增：

- MCAP reader。
- schema/channel/message 解析。
- compression support。

建议作为独立 Phase，不与 ROS1 MVP 混合。

### 14.4 SlamCache

为大 bag 或外部转换器准备内部缓存格式：

```text
Header
Source metadata
Frame block repeated:
  frameStartNs
  frameEndNs
  points[]
  imuSamples[]
```

用途：

- 加速重复运行。
- 隔离 ROSbag 格式复杂度。
- 支持外部转换器过渡。

## 15. 当前实施状态

2026-06-28 已按 R1-R6 完成第一版导入：

- R1/R2：
  - 新增 `libs/Rosbag/include/Rosbag/RosbagTypes.h`。
  - 新增 `libs/Rosbag/include/Rosbag/RosbagReader.h`。
  - 新增 `libs/Rosbag/src/RosbagReader.cpp`。
  - 支持 ROS1 bag v2.0 magic header 校验。
  - 支持 `connection`、`chunk`、`message data` record 读取。
  - 只支持 `compression=none` 的 uncompressed chunk；遇到压缩 chunk 直接 `Failed`。
- R3：
  - 新增 `libs/Rosbag/include/Rosbag/RosMessageParsers.h`。
  - 新增 `libs/Rosbag/src/RosMessageParsers.cpp`。
  - 手写解析 `std_msgs/Header`、`livox_ros_driver2/CustomMsg`、字段兼容的 `livox_ros_driver/CustomMsg`、`sensor_msgs/Imu`。
  - `CustomMsg.timebase` 和 `CustomPoint.offset_time` 均按 ns 处理。
- R4：
  - 新增 `libs/Slam/include/Slam/Io/RosbagSlamSource.h`。
  - 新增 `libs/Slam/src/Io/RosbagSlamSource.cpp`。
  - 自动识别 `/livox/lidar`、`/livox/lidar_<ip>`、`/livox/imu`、`/livox/imu_<ip>`。
  - 一个 Livox CustomMsg 生成一个 `SlamInputFrame`。
  - 自动附加 IMU 样本并计算 `hasCompleteImuCoverage`。
  - 无 LiDAR、无 IMU、PointCloud2、unsupported type、IMU 不覆盖任何帧均返回 `Failed`。
- R5：
  - `工具 -> SLAM（离线）` 文件选择支持 `*.pcap`、`*.pcapng`、`*.bag`。
  - 内部新增 `SlamOfflineSourceKind`，按扩展名选择 `PcapSlamSource` 或 `RosbagSlamSource`。
  - 保留旧 `loadOfflineSlamPcap()` / `offlineSlamPcapPath()` 作为兼容 wrapper。
  - 浮动控制条显示实际离线数据源路径，菜单入口仍只加载数据源，不自动启动 worker。
- R6：
  - 根 `CMakeLists.txt` 已加入 `libs/Rosbag` 源文件、头文件和 include path。
  - `git diff --check` 通过。
  - `C:/Users/FelixCooper/Desktop/compile.bat` 编译通过。

当前第一版仍未实现：

- 压缩 ROS1 bag chunk。
- ROS2 `.db3` / MCAP。
- driver2 PointCloud2 解析。
- 大 bag 流式读取或 `.slamcache`。

## 16. 给后续 Codex 的实施提示词

```text
当前 LivoxViewerQT 已有 PcapSlamSource、LiveLidarSlamSource、SlamInputFrame、SlamInputQueue、FastLioSlamBackend 和完整 SLAM UI。现在新增 ROSbag 文件作为离线 SLAM 数据源。

第一版目标：
1. 支持 ROS1 uncompressed .bag。
2. 优先支持 livox_ros_driver2/CustomMsg 和 sensor_msgs/Imu；livox_ros_driver2/CustomMsg 字段以 B:/Workspace/livox_ros_driver2/msg/CustomMsg.msg 与 CustomPoint.msg 为准。
3. ROSbag 最终转换为现有 SlamInputFrame，不修改 FastLioSlamBackend 输入接口。
4. 不引入 ROS runtime、roscpp、rosbag、catkin、ament。
5. 不在 UI 线程读取大 bag 或运行 SLAM。
6. 遇到无 IMU、缺点内 offset time、topic 类型不支持时统一返回 Failed，并在 SLAM状态 dock 的错误信息中显示原因。
7. CustomMsg.timebase 和 CustomPoint.offset_time 均按 ns 处理；offset_time 是相对 timebase 的点内时间。
8. driver2 的 PointCloud2 是后续增强：只有字段精确匹配 x/y/z/intensity/tag/line/timestamp 且 timestamp 为 FLOAT64 时才可解析，timestamp 是每点绝对 ns。

实现顺序：
1. 新增 libs/Rosbag 轻量 ROS1 bag reader 和 message parser。
2. 新增 libs/Slam/Io/RosbagSlamSource，接口先贴近 PcapSlamSource。
3. 解析 driver2 CustomMsg + Imu，组装 SlamInputFrame。
4. 离线 SLAM 文件选择支持 .bag，并根据扩展名选择 PcapSlamSource 或 RosbagSlamSource。
5. 复用现有 SLAM tab、状态 dock、浮动控制条和导出功能。
6. 更新 CMakeLists.txt，新增源文件和 include path。
7. 运行 git diff --check 和 C:\Users\FelixCooper\Desktop\compile.bat。

严格禁止：
- 不启动 ROS master。
- 不要求用户安装 ROS。
- 不把 ROS message 类型传给 FastLioSlamBackend。
- 不把 pcl::PointCloud<pcl::PointXYZI> 当作可用 FAST_LIO 输入。
- 不对缺失点内时间的 PointCloud2 默认合成时间。
- 不从 OpenGL VBO 读取点云用于导出。
- 不影响现有 PCAP 离线 SLAM、在线 SLAM、PCAP 普通播放。
```

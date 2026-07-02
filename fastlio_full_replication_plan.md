# FAST-LIO 输入同步完整复刻方案

## 目标

当前工程已经复用了 FAST-LIO 的 `ImuProcess`、ESKF、地图更新和点云去畸变逻辑，但实时、PCAP、ROSbag 三条数据源仍在各自提前构造 `SlamInputFrame::imuSamples`。这会让 IMU 覆盖语义、首帧处理、跨帧 `last_imu_` 衔接和原版 `sync_packages()` 不完全一致。

本次目标是把输入同步层完整收敛到原版 FAST-LIO 语义：所有数据源只按时间顺序投递 LiDAR 帧和 IMU 样本，统一同步器负责等待 IMU 水位线、计算帧尾时间、消费 IMU buffer、输出当前帧的 `MeasureGroup` 等价输入。

## 原版 FAST-LIO 语义

原版在线主循环的关键约束如下：

- LiDAR 和 IMU 分别进入 `lidar_buffer`、`imu_buffer`。
- `sync_packages()` 在 LiDAR 或 IMU 缺失时返回 false。
- 首次取出一帧 LiDAR 时，计算 `lidar_end_time`，设置 `lidar_pushed=true`。
- 当 `last_timestamp_imu < lidar_end_time` 时继续等待，不输出包。
- 当 IMU 水位线覆盖帧尾后，清空当前 `meas.imu`，从 `imu_buffer` 顺序弹出 `<= lidar_end_time` 的 IMU。
- 不复制 IMU 样本，不向当前帧加入帧尾后的第一个真实 IMU。
- 输出包后弹出 LiDAR 帧，并把未消费的未来 IMU 保留在 buffer 中。
- 主循环拿到第一个有效包后只设置 `first_lidar_time` 并跳过处理，从第二个包开始调用 `ImuProcess::Process()`。
- `ImuProcess::UndistortPcl()` 内部用 `last_imu_` 衔接上一帧尾部 IMU。

## 当前不等价点

- 实时源、PCAP 源、ROSbag 源各自 attach IMU，存在多套同步语义。
- 旧版曾把帧首前 IMU 和帧尾后 IMU 附到每帧；新版只消费 `<= frameEnd`，但仍由数据源提前拼包。
- `SlamInputFrame::hasCompleteImuCoverage` 同时承担数据源诊断和后端输入判定，语义混杂。
- 离线源预先给所有帧写入 `imuSamples`，不是按原版 buffer 顺序逐包同步。
- 后端 `FastLioSlamBackend` 的首帧跳过逻辑存在，但输入同步状态不在后端统一管理。

## 设计

### 1. 新增统一同步器

新增 `FastLioInputSynchronizer`，内部维护：

- LiDAR 队列：按原版 `lidar_buffer` 保存待处理点云帧。
- LiDAR 起始时间队列：等价原版 `time_buffer`。
- IMU 队列：按 timestamp 排序保存待处理 IMU。
- `lastTimestampImuNs`：等价原版 `last_timestamp_imu`。
- `lidarPushed`、`pendingLidarEndNs`、`lidarMeanScanTimeNs`、`scanNum`：等价原版 `lidar_pushed`、`lidar_end_time`、`lidar_mean_scantime`、`scan_num`。

同步器输出 `SlamInputFrame`，但该 frame 的 `imuSamples` 必须完全来自原版消费规则：

- 只消费 `timestampNs <= lidarEndNs` 的 IMU。
- 消费后从 IMU 队列移除。
- 不加入帧尾后的真实 IMU。
- 不做 start/tail padding。

### 2. 数据源职责收敛

实时源：

- 继续负责 packet 时间归一化、切点云帧、IMU 时间归一化和诊断统计。
- 不再自行 attach IMU。
- 点云帧完成后 push 到同步器。
- IMU packet 解析后 push 到同步器。
- worker 从同步器 trySync 出可处理 frame。

PCAP/ROSbag：

- 解析阶段只生成点云帧和全量 IMU 样本。
- 按 timestamp 顺序 replay 到同一个同步器。
- 离线 worker 或诊断工具从同步器取同步后的 frames。
- 不再在 `PcapSlamSource::attachImuSamples()` / `RosbagSlamSource::attachImuSamples()` 中预先写死每帧 IMU。

### 3. 后端职责保持接近原版主循环

`FastLioSlamBackend::processFrame()` 继续接受同步器输出的 `SlamInputFrame`：

- 第一帧只设置 `first_lidar_time` 并返回 `InitializingImu`。
- 后续帧调用 `ImuProcess::Process()`。
- 后端不再根据数据源自定义覆盖规则判断完整性，只要求同步器输出的 frame 有 IMU 且点内 offset 可用。

### 4. 诊断字段拆分

新增或调整统计字段区分：

- IMU 水位线不足：`lastTimestampImuNs < lidarEndNs`。
- 同步器输出空 IMU 包：原版语义下不应进入后端。
- 数据源时间异常：乱序、时间跳变、time_type 不一致。
- 后端跳过首帧：原版主循环行为，不计为错误。

## 实现步骤

1. 新增同步器头/源文件，并覆盖原版 `sync_packages()` 的状态机。
2. 修改 PCAP/ROSbag 离线源：解析后通过同步器生成同步 frames，删除各自独立 attach IMU 逻辑。
3. 修改实时源：点云帧和 IMU 样本进入同步器，由同步器决定何时输出 frame。
4. 修改在线 worker、离线 worker、`SlamPhase4Replay`，统一使用同步器输出的 frame 统计。
5. 保留现有时间归一化和数据源异常诊断，不改变 FAST-LIO 算法参数。
6. 使用 `scripts/dev_build_run.bat Release` 编译验证。

## 验证

- `git diff --check`。
- `scripts/dev_build_run.bat Release`。
- 对同一 PCAP/ROSbag 重跑 `SlamPhase4Replay --diagnose --max-frames 300`。
- 对 `new.csv` / `old.csv` 对应数据重跑轨迹导出，重点观察：
  - 低速段轨迹二阶差分。
  - 同一时间戳相对旧版累计偏移。
  - 剧烈运动段漂移是否保持改善。
  - 同步器等待 IMU 的帧数和空 IMU 输出是否为 0。

## 2026-07-02 实现记录

- 新增 `FastLioInputSynchronizer`，统一维护 LiDAR buffer、IMU buffer、`lidar_pushed`、`lidar_end_time`、`last_timestamp_imu` 和平均扫描时间。
- 实时源 `LiveLidarSlamSource` 保留时间归一化和 packet 切帧，完成的 LiDAR 帧与 IMU 样本进入统一同步器，不再单独 attach IMU。
- PCAP 离线源解析完成后通过统一同步器生成 FAST-LIO 输入帧。
- ROSbag SLAM 加载路径在 `requireImu=true` 时通过统一同步器生成 FAST-LIO 输入帧；普通 ROSbag 播放路径继续保留原始点云帧。
- 验证已通过：`git diff --check`；`scripts/dev_build_run.bat Release`；`SlamPhase4Replay --diagnose --max-frames 300 E:\Livox_ws\with_imu.pcap`；`SlamPhase4Replay --diagnose --max-frames 300 E:\Livox_ws\sandbox\test_data\avia\hku_campus_custommsg.bag`。

# FAST_LIO 迁移流程与参数审计

日期：2026-06-27

本文梳理原版 FAST_LIO 在本仓库 `libs/Slam/third_party/fast_lio` 中的流程和参数配置，并对照当前 LivoxViewerQT SLAM 实现，记录已经迁移、部分迁移和未迁移内容。本文聚焦当前代码事实，不把 UI 预览缓存等同于后端完整全局地图。

## 1. 参考源码范围

原版 FAST_LIO：

- `libs/Slam/third_party/fast_lio/config/mid360.yaml`
- `libs/Slam/third_party/fast_lio/src/laserMapping.cpp`
- `libs/Slam/third_party/fast_lio/src/IMU_Processing.hpp`
- `libs/Slam/third_party/fast_lio/src/preprocess.cpp`
- `libs/Slam/third_party/fast_lio/src/ikd-Tree/ikd_Tree.h`
- `libs/Slam/third_party/fast_lio/src/use-ikfom.hpp`

当前 LivoxViewerQT SLAM：

- `libs/Slam/include/Slam/Core/SlamTypes.h`
- `libs/Slam/include/Slam/Core/SlamRuntimeConfig.h`
- `libs/Slam/include/Slam/Core/SlamMapPreviewConfig.h`
- `libs/Slam/src/Core/SlamRuntimeConfig.cpp`
- `libs/Slam/src/Core/SlamMapPreviewConfig.cpp`
- `libs/Slam/src/Backends/FastLio/FastLioSlamBackend.cpp`
- `libs/Slam/src/Io/PcapSlamSource.cpp`
- `libs/Slam/src/Io/LiveLidarSlamSource.cpp`
- `libs/Slam/src/Visualization/GlobalMapPreviewStore.cpp`
- `apps/LivoxViewer/slam/SlamWindowActions.cpp`
- `apps/LivoxViewer/slam/SlamUiBridge.cpp`
- `apps/LivoxViewer/slam/SlamControlDialog.cpp`
- `apps/LivoxViewer/LivoxViewerSettings.cpp`
- `libs/PointCloud/src/PointCloudView.cpp`

## 2. 原版 FAST_LIO 主流程

原版入口在 `laserMapping.cpp`。整体运行模式是 ROS node，LiDAR/IMU 通过 topic 回调进入缓存，主线程高频轮询同步后的 `MeasureGroup`，再执行 IMU 去畸变、EKF 更新、IKD-tree 增量建图和 ROS 发布。

### 2.1 输入与缓存

1. `standard_pcl_cbk()` 或 `livox_pcl_cbk()` 接收点云 topic。
2. `p_pre->process()` 执行预处理，把 ROS 点云或 Livox custom message 转为 FAST_LIO 使用的 `PointCloudXYZI`。
3. 点云被推入 `lidar_buffer`，起始时间被推入 `time_buffer`。
4. `imu_cbk()` 接收 IMU topic，应用 `common/time_offset_lidar_to_imu`，必要时使用 `common/time_sync_en` 自同步偏移，然后推入 `imu_buffer`。
5. 点云或 IMU 时间回退时，原版会清空对应缓存。

### 2.2 时间同步

`sync_packages(MeasureGroup& meas)` 从两个缓存中拼出一次处理所需的 LiDAR scan 和 IMU 序列：

1. 取队首 LiDAR scan 和 scan 起始时间。
2. 根据末点 `curvature` 中保存的相对时间估算 `lidar_end_time`。
3. 如果 IMU 最新时间还没覆盖到 `lidar_end_time`，本轮等待。
4. 收集所有早于 `lidar_end_time` 的 IMU 样本。
5. 弹出已消费的 LiDAR scan、时间戳和 IMU 样本。

### 2.3 后端处理

主循环在 `while (ros::ok())` 内执行：

1. 首帧只记录 `first_lidar_time`，供 IMU 初始化使用。
2. `p_imu->Process(Measures, kf, feats_undistort)` 完成 IMU 初始化、点云去畸变和状态预测。
3. 根据当前位姿更新 `state_point`、`pos_lid`。
4. `lasermap_fov_segment()` 维护局部地图立方体窗口；位姿靠近窗口边缘时移动窗口，并从 IKD-tree 删除离开窗口的 box。
5. `downSizeFilterSurf` 对去畸变点云做表面特征降采样。
6. 首次建图时，把降采样点转到世界系并 `ikdtree.Build()`。
7. 正常帧执行最近邻搜索、平面约束构建和 `kf.update_iterated_dyn_share_modified()`。
8. `map_incremental()` 根据 `filter_size_map` 体素规则筛选新点并调用 `ikdtree.Add_Points()`。
9. 发布里程计、轨迹、世界系点云、机体系点云，必要时累计 PCD 保存缓存。

### 2.4 输出

原版输出是 ROS 发布和本地文件：

- `/cloud_registered`：世界系当前帧点云。
- `/cloud_registered_body`：机体系当前帧点云。
- `/cloud_effected`：有效特征点，默认调用被注释。
- `/Laser_map`：局部地图，默认调用被注释。
- `/Odometry`：当前里程计。
- `/path`：轨迹，取决于 `publish/path_en`。
- `PCD/scans.pcd` 或分片 PCD：取决于 `pcd_save/pcd_save_en` 和 `pcd_save/interval`。
- `Log/pos_log.txt`、`Log/fast_lio_time_log.csv`：取决于 `runtime_pos_log_enable`。

## 3. 原版 FAST_LIO 参数配置

`mid360.yaml` 是本仓库当前携带的原版配置样例；`laserMapping.cpp` 同时为这些参数提供默认值。下表的“当前迁移状态”以 LivoxViewerQT 当前代码为准。

| 原版参数 | `mid360.yaml` 值 | `laserMapping.cpp` 默认值 | 原版作用 | 当前迁移状态 |
| --- | ---: | ---: | --- | --- |
| `common/lid_topic` | `/livox/lidar` | `/livox/lidar` | ROS 点云订阅 topic | 未迁移；当前不通过 ROS topic 输入 |
| `common/imu_topic` | `/livox/imu` | `/livox/imu` | ROS IMU 订阅 topic | 未迁移；当前从 PCAP/SDK payload 解析 IMU |
| `common/time_sync_en` | `false` | `false` | 原版软件自同步开关 | 未完整迁移；当前要求原始时间戳与 IMU 覆盖，错误时返回 `TimeSyncError` |
| `common/time_offset_lidar_to_imu` | `0.0` | `0.0` | LiDAR 到 IMU 时间偏移 | 结构已迁移为 `lidarToImuTimeOffsetNs`，但当前 PCAP 后端未实际应用 |
| `preprocess/lidar_type` | `1` | `AVIA` | 选择 Livox/Velodyne/Ouster 预处理分支 | 部分迁移；当前直接解析 Livox payload，后端固定 `AVIA` |
| `preprocess/scan_line` | `4` | `16` | 线数，用于 Livox 预处理 | 部分迁移；当前从设备类型推导 lineCount，不作为后端参数 |
| `preprocess/blind` | `0.5` | `0.01` | 近距离盲区滤除 | 未迁移为可配置项；当前 PCAP/Live 解析路径未看到等价 blind 过滤 |
| `preprocess/timestamp_unit` | 未配置 | `US` | 点内时间单位 | 部分迁移；当前 Livox payload 使用 `time_interval * 100ns` 直接计算 |
| `preprocess/scan_rate` | 未配置 | `10` | 扫描频率估计 | 未迁移；当前固定 50 ms 聚帧 |
| `point_filter_num` | 未配置 | `2` | 点抽样过滤步长 | 未迁移；当前不走原版 `Preprocess::point_filter_num` |
| `feature_extract_enable` | 未配置 | `false` | 是否提取角/面特征 | 未迁移；当前沿用 FAST_LIO 直接 surf 下采样路线 |
| `filter_size_corner` | 未配置 | `0.5` | 角点降采样体素 | 未迁移；当前未使用角点特征分支 |
| `filter_size_surf` | 未配置 | `0.5` | 输入 surf 特征降采样体素 | 已迁移为 `SlamRuntimeConfig::filterSizeSurfM`，设置页可修改 |
| `filter_size_map` | 未配置 | `0.5` | 后端 IKD-tree 地图增量体素 | 已迁移为 `SlamRuntimeConfig::filterSizeMapM`，设置页可修改 |
| `cube_side_length` | 未配置 | `200` | 局部地图立方体边长 | 部分迁移；当前后端硬编码 `kDefaultCubeLen = 200.0` |
| `mapping/det_range` | `100.0` | `300.0` | FOV/局部地图移动阈值相关检测距离 | 部分迁移；当前后端硬编码 `kDefaultDetRange = 300.0`，与 `mid360.yaml` 不一致 |
| `mapping/fov_degree` | `360` | `180` | 视场角，内部上限裁到 179.9 | 部分迁移；当前后端硬编码 `kDefaultFovDeg = 180.0`，与 `mid360.yaml` 不一致 |
| `mapping/gyr_cov` | `0.1` | `0.1` | 陀螺仪噪声协方差 | 部分迁移；当前后端硬编码 `0.1` |
| `mapping/acc_cov` | `0.1` | `0.1` | 加速度计噪声协方差 | 部分迁移；当前后端硬编码 `0.1` |
| `mapping/b_gyr_cov` | `0.0001` | `0.0001` | 陀螺仪 bias 协方差 | 部分迁移；当前后端硬编码 `0.0001` |
| `mapping/b_acc_cov` | `0.0001` | `0.0001` | 加速度计 bias 协方差 | 部分迁移；当前后端硬编码 `0.0001` |
| `mapping/extrinsic_est_en` | `false` | `true` | 是否在线估计外参 | 未迁移为功能开关；当前使用静态外参并做合法性校验 |
| `mapping/extrinsic_T` | `[-0.011, -0.02329, 0.04412]` | 空数组 | LiDAR 相对 IMU 平移 | 已迁移为 `extrinsicT_L_I[3]`，但设置页暂未暴露 |
| `mapping/extrinsic_R` | identity | 空数组 | LiDAR 相对 IMU 旋转 | 已迁移为 `extrinsicR_L_I[9]`，但设置页暂未暴露 |
| `max_iteration` | 未配置 | `4` | EKF 迭代次数 | 部分迁移；当前后端硬编码 `kDefaultMaxIterations = 4` |
| `map_file_path` | 未配置 | 空 | 原版地图文件路径占位 | 未迁移 |
| `publish/path_en` | `false` | `true` | ROS path 发布 | UI 侧以轨迹 overlay/导出替代，不再用 ROS |
| `publish/scan_publish_en` | `true` | `true` | 是否发布世界系点云 | UI 侧以原有点云显示和 SLAM overlay 替代 |
| `publish/dense_publish_en` | `true` | `true` | 发布当前帧 dense/降采样点云 | 未按原版发布语义迁移；当前地图预览是后端新增点 chunk 的 UI 预览 |
| `publish/scan_bodyframe_pub_en` | `true` | `true` | 是否发布机体系点云 | 未迁移 |
| `pcd_save/pcd_save_en` | `true` | `false` | 是否累计保存 PCD | 未迁移为完整后端地图保存；当前只有轨迹导出和当前预览地图导出 |
| `pcd_save/interval` | `-1` | `-1` | PCD 分片间隔 | 未迁移 |
| `runtime_pos_log_enable` | 未配置 | `false` | 保存调试位置/耗时日志 | 未迁移为原版日志文件；当前 UI 显示部分运行状态 |

## 4. 当前 LivoxViewerQT SLAM 主流程

当前实现把 FAST_LIO 从 ROS node 改造成应用内后端。主流程分为输入源、后端、UI 桥接、OpenGL overlay 四层。

### 4.1 数据结构

`SlamTypes.h` 定义当前跨模块数据契约：

- `SlamPoint`：点坐标、反射率、tag、line、点内 offset 时间。
- `SlamImuSample`：IMU 时间戳、角速度、加速度。
- `SlamInputFrame`：一帧点云、帧起止时间、IMU 样本、是否有点内 offset、是否有完整 IMU 覆盖。
- `SlamOutput`：状态、当前位姿、新增轨迹点、新增地图 chunk、输入 FPS、后端耗时、丢帧数、后端地图点数、轨迹点数、IMU 健康状态。

### 4.2 PCAP 输入

当前 UI worker 使用 `PcapSlamSource`：

1. 使用 libpcap 离线读取文件。
2. 扫描 push message 和数据源 IP，推导设备信息、设备类型和线数。
3. 解析 Livox 点云 payload，支持 Cartesian high、Cartesian low、Spherical、Double echo。
4. 以 Livox packet timestamp 和 `time_interval * 100ns` 计算点内时间。
5. 固定按 50 ms 聚合为 `SlamInputFrame`。
6. 解析 Livox IMU payload，并按帧起止时间附加前后覆盖样本。
7. 如果没有 IMU、点内 offset 缺失、点云乱序或 IMU 覆盖不完整，摘要状态标为 `MissingImu` 或 `TimeSyncError`。
8. 当前 MVP 拒绝多点云源 PCAP。

### 4.3 Live 输入

`LiveLidarSlamSource` 已存在，用于把实时 SDK packet 转为 `SlamInputFrame` 并推入 `SlamInputQueue`：

- 点云解析逻辑与 PCAP 输入类似。
- IMU 样本缓存在内存中，帧 flush 时附加覆盖样本。
- 队列有容量和丢帧计数。
- 当前 UI 的 `startSlamProcessing()` 仍使用 PCAP source；实时 SLAM worker 尚未完整接入主 UI 启动路径。

### 4.4 后端处理

`FastLioSlamBackend::processFrame()` 是当前后端入口。它保留了原版 FAST_LIO 的核心算法顺序，但输入输出改为应用内结构：

1. 校验后端已启动。
2. 要求每帧有 IMU 样本。
3. 要求 IMU 覆盖到 LiDAR frame end。
4. 要求每个点有 offset time。
5. 把 `SlamInputFrame` 转为 FAST_LIO `MeasureGroup`。
6. 首帧记录 `first_lidar_time` 并进入 IMU 初始化。
7. `ImuProcess::Process()` 执行 IMU 处理和点云去畸变。
8. `laserMapFovSegment()` 维护局部地图窗口。
9. `downSizeFilterSurf` 用 `filterSizeSurfM` 对去畸变点云降采样。
10. 首次建图时 `ikdtree.Build()`。
11. 正常帧执行 IKFOM 迭代更新。
12. `mapIncremental()` 用 `filterSizeMapM` 控制 IKD-tree 增量添加。
13. 输出当前位姿、轨迹点、后端地图点数和新增地图 chunk。

当前 `SlamOutput::mapPointCount` 来自 `ikdtree.validnum()`，是后端局部 IKD-tree 有效点数量，不是 UI 预览点数量。

### 4.5 UI worker 与旁路 UI

`LivoxViewerWindow::startSlamProcessing()` 当前流程：

1. 通过 `ensureSlamUiBridge()` 懒创建 `SlamUiBridge`。
2. 要求当前激活 source 是 PCAP 播放源。
3. 复制当前 `SlamRuntimeConfig`。
4. 启动独立 `std::thread`。
5. worker 加载 `PcapSlamSource`，启动 `FastLioSlamBackend`。
6. 按 PCAP 原始帧时间 replay；暂停时累积 pause duration。
7. IMU 覆盖不完整的帧直接计入 dropped frame。
8. 每帧调用 `backend.processFrame()`。
9. 通过 `QMetaObject::invokeMethod(..., Qt::QueuedConnection)` 把 `SlamOutput` 投递回 UI 线程。
10. 如果地图预览模式为 `Off`，worker 在投递前清空 `newMapChunks`，避免 UI 继续累计预览地图。

`SlamUiBridge` 是 QObject，不继承 QWidget，10 Hz `QTimer` 刷新状态文本，并缓存：

- 最新 `SlamOutput`。
- 轨迹点。
- 地图预览 store。
- 当前错误提示。

`SlamControlDialog` 是非模态浮动窗口，通过“工具 -> SLAM...”菜单懒创建，不改 dock/tab 布局。

### 4.6 Overlay 与地图预览

`SlamRenderSnapshot` 是 UI 线程到 `PointCloudView` 的渲染快照：

- 轨迹线顶点。
- 当前位姿坐标轴顶点。
- 地图预览 append points。
- 地图预览 indexed updates。
- reset、enabled、点数上限和当前点数。

`PointCloudView` 只暴露：

- `setSlamRenderSnapshot(const SlamRenderSnapshot&)`
- `clearSlamRenderOverlay()`

这两个接口都断言在 UI 主线程调用。OpenGL VBO/VAO 的创建、写入和销毁在当前 context 有效时执行。地图预览 VBO 使用增量 append/update，不从 OpenGL VBO 读取数据用于导出。

`GlobalMapPreviewStore` 是 UI 预览缓存：

- `Off`：不累计。
- `GlobalSparse`：全局稀疏预览，默认上限 2,000,000 点，体素 0.10 m。
- `GlobalDense`：全局稠密预览，默认上限 20,000,000 点，体素 0.05 m。
- 两种模式都覆盖完整建图范围，不按最近时间或最近 chunk 淘汰。
- 使用 voxel hash 做全局去重。
- 达到上限后用 hash 概率替换维持全局覆盖。
- 每次 UI tick 按 `uploadPointsPerTick` 分批上传。
- 切换模式或配置会清空旧预览缓存。

当前“导出当前预览地图 PCD/LAS”从 `SlamUiBridge::mapPreviewSnapshot()` 获取 UI 预览顶点并导出。它不是完整原始全局地图导出。

## 5. 当前项目参数配置

### 5.1 `SlamRuntimeConfig`

定义位置：`libs/Slam/include/Slam/Core/SlamRuntimeConfig.h`

保存/加载位置：`libs/Slam/src/Core/SlamRuntimeConfig.cpp`

QSettings 前缀：`slam/runtime`

| 参数 | 默认值 | 当前是否 UI 暴露 | 当前使用位置 | 修改后的影响 |
| --- | ---: | --- | --- | --- |
| `backendType` | `FAST_LIO` | 否 | 配置持久化占位 | 当前实际 worker 固定创建 `FastLioSlamBackend` |
| `lidarModel` | 空 | 否 | 配置持久化占位 | 当前未参与 PCAP 后端处理 |
| `imuEnabled` | `true` | 否 | `FastLioSlamBackend::start()` | false 会导致 FAST_LIO 拒绝启动 |
| `allowPureLidar` | `false` | 否 | 配置持久化占位 | 当前 FAST_LIO 仍强制要求 IMU |
| `lidarToImuTimeOffsetNs` | `0` | 否 | 配置持久化占位 | 当前 PCAP 输入/后端未实际应用该偏移 |
| `gravityNorm` | `9.81` | 否 | 配置持久化占位 | 当前未传入 `ImuProcess` |
| `extrinsicT_L_I[3]` | `[0,0,0]` | 否 | 后端初始化 `ImuProcess::set_extrinsic()` | 改变 LiDAR-IMU 外参平移，直接影响去畸变、位姿和建图 |
| `extrinsicR_L_I[9]` | identity | 否 | 后端初始化 `ImuProcess::set_extrinsic()` | 改变 LiDAR-IMU 外参旋转，直接影响去畸变、位姿和建图 |
| `filterSizeSurfM` | `0.5` | 是 | `downSizeFilterSurf.setLeafSize()` | 控制输入 surf 点降采样；越大点越少、速度更快、细节更少 |
| `filterSizeMapM` | `0.5` | 是 | `downSizeFilterMap`、`ikdtree.set_downsample_param()`、`mapIncremental()` | 控制后端地图增量体素；影响后端地图点数、地图 chunk 点数和预览输入规模 |
| `mapVoxelSizeM` | `0.1` | 否 | 配置持久化占位 | 当前未参与后端或 UI 预览逻辑 |
| `maxMapPoints` | `2,000,000` | 否 | 配置持久化占位 | 当前不限制后端 IKD-tree 或 UI 预览 |
| `maxTrajectoryPoints` | `200,000` | 否 | 配置持久化占位 | 当前 `SlamUiBridge` 内部另有 `kMaxTrajectoryPoints = 200000` |
| `maxInputQueueFrames` | `8` | 否 | Live source 可用于 queue 容量 | 当前 PCAP worker 不走 `SlamInputQueue` |
| `saveTrajectory` | `false` | 否 | 配置持久化占位 | 当前轨迹由按钮手动导出 CSV/TUM |
| `saveMap` | `false` | 否 | 配置持久化占位 | 当前地图由按钮手动导出当前 UI 预览缓存 |
| `logLevel` | `info` | 否 | 配置持久化占位 | 当前未驱动 FAST_LIO 日志级别 |

### 5.2 `SlamMapPreviewConfig`

定义位置：`libs/Slam/include/Slam/Core/SlamMapPreviewConfig.h`

保存/加载位置：`libs/Slam/src/Core/SlamMapPreviewConfig.cpp`

QSettings 前缀：`slam/mapPreview`

| 参数 | 默认值 | 当前是否 UI 暴露 | 当前使用位置 | 修改后的影响 |
| --- | ---: | --- | --- | --- |
| `mode` | `Off` | 是 | `SlamWindowActions`、`SlamUiBridge`、`GlobalMapPreviewStore` | 控制是否累计和显示地图预览 |
| `globalSparseMaxPoints` | `2,000,000` | 是 | `GlobalMapPreviewStore::maxPreviewPoints()` | 稀疏预览最多保留点数 |
| `globalSparseVoxelSizeM` | `0.10` | 是 | `GlobalMapPreviewStore::voxelKey()` | 稀疏预览全局去重体素；越大点越少 |
| `globalSparseUploadPointsPerTick` | `20,000` | 是 | `SlamUiBridge::buildRenderSnapshot()` | 每次 10 Hz UI 刷新最多上传的稀疏预览点 |
| `globalDenseMaxPoints` | `20,000,000` | 是 | `GlobalMapPreviewStore::maxPreviewPoints()` | 稠密预览最多保留点数；内存和 OpenGL 压力明显更高 |
| `globalDenseVoxelSizeM` | `0.05` | 是 | `GlobalMapPreviewStore::voxelKey()` | 稠密预览全局去重体素；越小点越多 |
| `globalDenseUploadPointsPerTick` | `20,000` | 是 | `SlamUiBridge::buildRenderSnapshot()` | 每次 10 Hz UI 刷新最多上传的稠密预览点 |

地图预览配置只影响 UI 预览 store 和 overlay 上传，不影响 FAST_LIO 后端 IKD-tree 建图质量。它的输入仍来自后端输出的 `newMapChunks`，所以后端 `filterSizeMapM` 会影响预览可见的源点数量上限。

### 5.3 当前后端硬编码参数

定义位置：`libs/Slam/src/Backends/FastLio/FastLioSlamBackend.cpp`

| 当前常量 | 值 | 对应原版参数 | 影响 |
| --- | ---: | --- | --- |
| `kInitTime` | `0.1` | `INIT_TIME` | IMU 初始化后多久允许 EKF 正式进入更新 |
| `kLaserPointCov` | `0.001` | `LASER_POINT_COV` | 激光点观测噪声 |
| `kDefaultCubeLen` | `200.0` | `cube_side_length` | 局部地图窗口大小 |
| `kDefaultDetRange` | `300.0` | `mapping/det_range` | 局部地图移动阈值/FOV 相关判断 |
| `kDefaultFovDeg` | `180.0` | `mapping/fov_degree` | FOV 角度，内部上限 179.9 |
| `kDefaultMaxIterations` | `4` | `max_iteration` | IKFOM 迭代次数 |
| `kMoveThreshold` | `1.5` | `MOV_THRESHOLD` | 局部地图窗口移动阈值 |
| `kMinMapInitPoints` | `5` | 原版同类判断 | 建图/更新所需最小降采样点数 |
| IMU gyr cov | `0.1` | `mapping/gyr_cov` | IMU 噪声模型 |
| IMU acc cov | `0.1` | `mapping/acc_cov` | IMU 噪声模型 |
| IMU gyr bias cov | `0.0001` | `mapping/b_gyr_cov` | IMU bias 随机游走 |
| IMU acc bias cov | `0.0001` | `mapping/b_acc_cov` | IMU bias 随机游走 |
| `imuProcessor->lidar_type` | `AVIA` | `preprocess/lidar_type` | 后端 IMU 处理内的 LiDAR 类型标记 |

这些常量目前没有进入设置页或 QSettings。尤其 `det_range=300`、`fov_degree=180` 与 `mid360.yaml` 中 `det_range=100`、`fov_degree=360` 存在语义差异，后续如果追求与原版 MID360 配置一致，需要决定以 yaml 为准还是以原版代码默认值为准。

## 6. 流程迁移对照

| 原版环节 | 当前项目对应实现 | 迁移状态 | 备注 |
| --- | --- | --- | --- |
| ROS node 初始化 | Qt 应用内 SLAM 模块 | 已替换 | 不再需要 ROS runtime |
| ROS LiDAR topic 订阅 | `PcapSlamSource` / `LiveLidarSlamSource` | 部分完成 | PCAP UI worker 已接入；Live source 尚未接入 UI worker |
| ROS IMU topic 订阅 | PCAP/SDK IMU payload 解析 | 部分完成 | 依赖 Livox 原始 payload 时间 |
| `Preprocess::process()` | 项目自有 Livox payload 解析 | 部分完成 | 未迁移 blind、point_filter_num、feature extraction 等可配置预处理 |
| `sync_packages()` | `SlamInputFrame` 帧组装和 IMU 覆盖校验 | 部分完成 | 当前更严格：无完整 IMU 覆盖直接丢帧/报错 |
| `ImuProcess::Process()` | 直接复用 | 已迁移 | 仍使用原版 IMU 处理代码 |
| IKFOM 状态与更新 | 直接复用 `use-ikfom.hpp` 相关逻辑 | 已迁移 | 当前封装在后端对象状态中 |
| `lasermap_fov_segment()` | `laserMapFovSegment()` | 已迁移 | 参数大多仍硬编码 |
| `map_incremental()` | `mapIncremental()` | 已迁移 | `filterSizeMapM` 可配置 |
| `ikdtree.Build/Add/Delete` | 直接复用 IKD-tree | 已迁移 | 后端地图点数来自 `ikdtree.validnum()` |
| ROS odom/path 发布 | `SlamOutput` 当前位姿/轨迹 + overlay/export | 已替换 | CSV/TUM 手动导出已实现 |
| ROS 当前帧点云发布 | 原有点云播放显示 + SLAM overlay | 部分替换 | 没有完全复刻 `/cloud_registered` 语义 |
| ROS `/Laser_map` 发布 | UI 地图预览 store | 部分替换 | 是预览缓存，不是完整后端地图 |
| 原版 PCD 保存 | 当前预览地图 PCD/LAS 导出 | 未等价迁移 | 当前导出数据源不是完整全局地图 |
| 原版 debug log | UI 状态字段和应用日志 | 部分替换 | 原版 CSV debug log 未迁移 |
| 原版 ROS 参数服务器 | `QSettings` + 设置页 + 后端常量 | 部分完成 | 只有部分参数可配置 |

## 7. 当前迁移完成情况

### 7.1 已完成

- FAST_LIO 核心算法已脱离 ROS node，以 `FastLioSlamBackend` 形式嵌入项目。
- `ImuProcess`、IKFOM、IKD-tree 增量建图核心路径已复用。
- PCAP 离线 SLAM worker 已接入 UI，支持按原始时间 replay、暂停、停止、重置。
- `SlamInputFrame`/`SlamOutput` 数据契约已建立。
- 缺 IMU、IMU 覆盖不完整、点内时间缺失等情况已有状态反馈。
- 旁路 UI 已完成：`SlamUiBridge`、非模态 `SlamControlDialog`、不改 dock/tab 创建流程。
- 当前位姿和轨迹 overlay 已接入 `PointCloudView`。
- 地图预览已支持关闭、全局稀疏、全局稠密三种模式。
- 地图预览关闭时 worker 不投递 map chunks，UI store 不继续累计。
- 地图预览使用 UI 线程 store 和分批 OpenGL 上传，后端线程不访问 `PointCloudView`。
- 轨迹 CSV/TUM 导出已实现。
- 当前预览地图 PCD/LAS 导出已实现，并在日志中标注预览模式和点数。
- 第三方依赖已按项目目录 `third-party` 管理，并由脚本/CMake 提示处理。

### 7.2 部分完成

- Live SLAM 输入源已实现，但实时 SLAM worker 尚未成为主 UI 可用流程。
- FAST_LIO 参数只迁移了一部分到 `SlamRuntimeConfig` 和设置页。
- `extrinsicT_L_I`、`extrinsicR_L_I` 已进入配置结构和持久化，但设置页未暴露。
- `lidarToImuTimeOffsetNs`、`gravityNorm`、`mapVoxelSizeM`、`maxMapPoints` 等已进入配置结构，但当前后端没有实际使用或只作为占位。
- `filterSizeSurfM` 和 `filterSizeMapM` 已可配置，但原版 `filter_size_corner`、`blind`、`point_filter_num` 未迁移。
- 地图预览覆盖完整建图范围，但其点源来自后端增量 chunk；后端 chunk 仍受 `filterSizeMapM` 和局部 IKD-tree 增量逻辑影响。
- 后端地图点数和预览点数已经在 UI 中分开显示，但二者可能在某些参数组合下接近或相同。

### 7.3 未迁移或明确未完成

- 完整全局地图后端数据源与完整全局地图 PCD/LAS 导出尚未实现。
- 不能用 UI 预览缓存冒充完整全局地图。
- 原版 `pcd_save/pcd_save_en` 和 `pcd_save/interval` 的全量保存语义未迁移。
- 原版 ROS topic 输出未迁移，也不是当前 Qt 应用的目标形态。
- 原版 `time_sync_en` 自同步逻辑未等价迁移。
- 原版 `extrinsic_est_en` 在线外参估计未迁移。
- 原版 runtime debug 文件输出未迁移。
- 多 LiDAR PCAP SLAM 未支持。
- Velodyne/Ouster 等非 Livox 预处理路径未迁移。
- Linux/macOS 构建和运行未在当前记录中验证。

## 8. 关键差异与风险

### 8.1 参数默认值差异

`mid360.yaml` 中 `mapping/det_range=100.0`、`mapping/fov_degree=360`，当前后端使用原版代码默认值 `detRange=300.0`、`fovDeg=180.0`。这会影响局部地图窗口移动和 FOV 判断。当前测试通过并不代表参数语义已经完全等价于 `mid360.yaml`。

### 8.2 时间同步策略差异

原版可以在 `time_sync_en=true` 时尝试 LiDAR/IMU 自同步；当前实现更依赖录制数据的原始时间戳、点内 offset 和 IMU 覆盖。优点是行为更可控，缺点是对数据质量更敏感。

### 8.3 预处理差异

原版 `Preprocess` 中的 `blind`、`point_filter_num`、`scan_line`、`feature_extract_enable` 未以同样方式迁移。当前直接解析 Livox payload 并交给后端 surf voxel filter，这可能让输入点数量、近距离点处理和异常点过滤与原版不同。

### 8.4 地图点数语义

当前至少有三类点数：

1. 后端地图点数：`SlamOutput::mapPointCount = ikdtree.validnum()`。
2. 新增地图 chunk 点数：每帧从 `mapIncremental()` 的 added points 生成。
3. UI 预览点数：`GlobalMapPreviewStore::previewPointCount()`，受预览模式体素和上限影响。

如果预览体素小于或接近后端 `filterSizeMapM`，并且未达到上限，预览点数可能接近后端新增点累计结果。它仍不是完整原始全局地图点数。

### 8.5 完整地图导出风险

当前预览地图导出读取 UI 预览缓存，不读取 OpenGL VBO，也不读取后端完整全局地图。后续实现完整全局地图导出时，必须新增后端完整地图数据源，并放在 worker/background 路径中执行，不能在 UI 线程执行 KD-tree flatten 或大规模文件导出。

### 8.6 稠密预览资源风险

全局稠密默认上限 20,000,000 点。单个 `SlamRenderVertex` 约 24 字节，仅顶点数组约 480 MB；再加 QVector、hash、索引、pending queue 和 OpenGL buffer，实际峰值内存会显著更高。当前通过体素、上限和每 tick 上传量控制卡顿风险，但长轨迹、低体素和高反射更新仍可能造成内存压力。

## 9. 推荐后续迁移顺序

1. 明确 FAST_LIO 参数基线：以 `mid360.yaml` 为准，还是以原版代码默认值为准；尤其是 `det_range`、`fov_degree`、外参和 blind。
2. 将真正影响后端行为且需要调参的硬编码项迁入 `SlamRuntimeConfig`：`detRange`、`fovDeg`、`cubeLen`、IMU cov、`maxIterations`、`blind`。
3. 决定 `lidarToImuTimeOffsetNs` 的应用位置：输入源时间戳修正，或构造 `MeasureGroup` 时修正。
4. 把外参配置暴露到设置页，至少支持 MID360 默认外参一键恢复和矩阵合法性校验。
5. 接入 Live SLAM worker，让 `LiveLidarSlamSource` 的 queue 成为 UI 可启动的数据源。
6. 设计完整全局地图后端数据源：在后端线程维护或按需导出，避免 UI 线程 flatten IKD-tree。
7. 保留当前 UI 预览导出文案，新增独立“导出完整全局地图 PCD/LAS”功能，避免数据语义混淆。
8. 补充回归测试：PCAP 输入摘要、后端参数有效性、预览模式切换清空、Off 模式不累计、轨迹导出、预览地图导出。

## 10. 当前结论

迁移已经完成了“FAST_LIO 核心算法可在 LivoxViewerQT 内通过 PCAP worker 跑通，并以旁路 UI 显示状态、位姿、轨迹和受限地图预览”的目标。

尚未完成的是“与原版 FAST_LIO 参数语义完全一致”和“完整全局地图后端数据源/导出”。当前最重要的技术债是后端参数仍有多项硬编码，且 UI 地图预览与完整地图导出的数据语义必须继续保持分离。

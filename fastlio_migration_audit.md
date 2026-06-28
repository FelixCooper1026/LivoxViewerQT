# FAST_LIO 迁移流程与参数审计

日期：2026-06-27

本文梳理原版 FAST_LIO 流程/参数，以及当前 LivoxViewerQT SLAM 迁移状态。当前版本已移除旧地图预览缓存逻辑，改为按原版发布语义输出当前帧世界系/机体系点云，并按原版 `pcd_save` 语义累计完整全局地图用于 PCD/LAS 导出。

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
- `libs/Slam/src/Core/SlamRuntimeConfig.cpp`
- `libs/Slam/src/Backends/FastLio/FastLioSlamBackend.cpp`
- `libs/Slam/src/Export/SlamMapExport.cpp`
- `libs/Slam/src/Export/SlamTrajectoryExport.cpp`
- `libs/Slam/src/Io/PcapSlamSource.cpp`
- `libs/Slam/src/Io/LiveLidarSlamSource.cpp`
- `libs/Slam/include/Slam/Visualization/SlamRenderSnapshot.h`
- `apps/LivoxViewer/slam/SlamWindowActions.cpp`
- `apps/LivoxViewer/slam/SlamUiBridge.cpp`
- `apps/LivoxViewer/slam/SlamControlDialog.cpp`
- `apps/LivoxViewer/LivoxViewerSettings.cpp`
- `libs/PointCloud/src/PointCloudView.cpp`

## 2. 原版 FAST_LIO 主流程

原版入口在 `laserMapping.cpp`。整体运行模式是 ROS node，LiDAR/IMU 通过 topic 回调进入缓存，主线程高频轮询同步后的 `MeasureGroup`，再执行 IMU 去畸变、EKF 更新、IKD-tree 增量建图和 ROS 发布。

流程摘要：

1. `standard_pcl_cbk()` 或 `livox_pcl_cbk()` 接收点云 topic。
2. `p_pre->process()` 把 ROS 点云或 Livox custom message 转为 FAST_LIO 使用的 `PointCloudXYZI`。
3. `imu_cbk()` 接收 IMU topic，应用 `common/time_offset_lidar_to_imu`，必要时使用 `common/time_sync_en` 自同步偏移。
4. `sync_packages()` 根据 LiDAR scan 起止时间收集 IMU 样本，拼出 `MeasureGroup`。
5. 首帧记录 `first_lidar_time`，后续帧调用 `p_imu->Process()` 做 IMU 初始化、预测和点云去畸变。
6. `lasermap_fov_segment()` 维护局部地图立方体窗口。
7. `downSizeFilterSurf` 对去畸变点云做输入降采样。
8. 首次建图时 `ikdtree.Build()`；正常帧执行最近邻搜索、平面约束和 IKFOM 迭代更新。
9. `map_incremental()` 使用 `filter_size_map` 体素规则向 IKD-tree 增量加点。
10. 按配置发布 odom/path/world cloud/body cloud，并按 `pcd_save` 语义累计世界系 dense 帧用于 PCD 保存。

## 3. 原版发布与保存语义

原版发布相关代码位于 `laserMapping.cpp`：

- `publish_frame_world()`：
  - 当 `scan_pub_en=true` 时发布 `/cloud_registered`。
  - 发布点源由 `dense_pub_en` 决定：true 使用 `feats_undistort`，false 使用 `feats_down_body`。
  - 发布前把点从 LiDAR body frame 变换到 world frame。
- `publish_frame_body()`：
  - 当 `scan_pub_en && scan_body_pub_en` 时发布 `/cloud_registered_body`。
  - 点源固定使用 `feats_undistort`。
  - 发布前只做 LiDAR body 到 IMU body 的外参变换。
- `pcd_save`：
  - 当 `pcd_save_en=true` 时，即使 `scan_pub_en=false`，仍在 `publish_frame_world()` 中把 `feats_undistort` 转到 world frame 并追加到 `pcl_wait_save`。
  - `interval=-1` 时最终写一个 `PCD/scans.pcd`。

当前项目没有 ROS topic，因此“发布”迁移为 `SlamOutput` 中的点云输出。世界系点云进入 SLAM 专属 `PointCloudView` 主点云流，按工具栏积分时间显示；IMU 机体系点云仍作为固定颜色 overlay 显示。

## 4. 原版参数配置与当前迁移状态

| 原版参数 | `mid360.yaml` 值 | 原版作用 | 当前状态 |
| --- | ---: | --- | --- |
| `common/lid_topic` | `/livox/lidar` | ROS 点云订阅 topic | 未迁移；当前不走 ROS |
| `common/imu_topic` | `/livox/imu` | ROS IMU 订阅 topic | 未迁移；当前从 PCAP/SDK payload 解析 |
| `common/time_sync_en` | `false` | 软件自同步开关 | 未等价迁移；当前要求原始时间戳和 IMU 覆盖 |
| `common/time_offset_lidar_to_imu` | `0.0` | LiDAR 到 IMU 时间偏移 | 已进入 `SlamRuntimeConfig::lidarToImuTimeOffsetNs`，当前未实际应用 |
| `common_lib.h/G_m_s2` | `9.81` | 重力加速度常量 | 已迁移为 `gravityNorm`，设置页可配置；默认 9.81 m/s² |
| `preprocess/lidar_type` | `1` | 选择预处理分支 | 部分迁移；当前直接解析 Livox payload，后端固定 `AVIA` |
| `preprocess/scan_line` | `4` | 线数 | 部分迁移；当前从设备类型推导 lineCount |
| `preprocess/blind` | `0.5` | 近距离盲区滤除 | 已迁移为 `blindMinRangeM`，设置页可配置；Mid360/Mid360S 默认 0.5 m，Avia 默认 4.0 m |
| `preprocess/timestamp_unit` | 未配置 | 点内时间单位 | 部分迁移；Livox payload 使用 `time_interval * 100ns` |
| `preprocess/scan_rate` | 未配置 | 扫描频率估计 | 已迁移为 `preprocessScanRateHz`，设置页可配置；默认 10 Hz，并同步推导 `inputFrameDurationMs=100` |
| `point_filter_num` | 未配置 | 点抽样过滤 | 未迁移 |
| `feature_extract_enable` | 未配置 | 特征提取开关 | 未迁移 |
| `filter_size_corner` | 未配置 | 角点降采样 | 未迁移 |
| `filter_size_surf` | 未配置 | 输入 surf 降采样 | 已迁移为 `filterSizeSurfM`，设置页可配置 |
| `filter_size_map` | 未配置 | IKD-tree 地图体素 | 已迁移为 `filterSizeMapM`，设置页可配置 |
| `cube_side_length` | 未配置 | 局部地图窗口大小 | 已迁移为 `cubeSideLengthM`，设置页可配置；默认 200.0 m |
| `mapping/det_range` | `100.0` | 检测距离/FOV 判断 | 已迁移为 `detRangeM`，设置页可配置；Mid360/Mid360S 默认 100.0 m，Avia 默认 450.0 m |
| `mapping/fov_degree` | `360` | FOV 角度 | 已迁移为 `fovDegree`，设置页可配置；内部仍按原版加 10° 余量并裁到 179.9° |
| `mapping/gyr_cov` | `0.1` | 陀螺仪噪声 | 已迁移为 `gyrCov`，设置页可配置 |
| `mapping/acc_cov` | `0.1` | 加速度计噪声 | 已迁移为 `accCov`，设置页可配置 |
| `mapping/b_gyr_cov` | `0.0001` | 陀螺 bias 协方差 | 已迁移为 `bGyrCov`，设置页可配置 |
| `mapping/b_acc_cov` | `0.0001` | 加计 bias 协方差 | 已迁移为 `bAccCov`，设置页可配置 |
| `mapping/extrinsic_est_en` | `false` | 在线外参估计 | 已迁移为 `extrinsicEstimationEnabled`，设置页可配置；关闭时按原版将外参雅可比列置零 |
| `mapping/extrinsic_T` | `[-0.011,-0.02329,0.04412]` | LiDAR-IMU 平移 | 已进入 `extrinsicT_L_I`，设置页可配置 |
| `mapping/extrinsic_R` | identity | LiDAR-IMU 旋转 | 已进入 `extrinsicR_L_I`，设置页可配置 |
| `max_iteration` | 未配置 | EKF 迭代次数 | 已迁移为 `maxIterations`，设置页可配置；默认 4 |
| `publish/path_en` | `false` | 发布 path | 以轨迹 overlay/CSV/TUM 导出替代 |
| `publish/scan_publish_en` | `true` | 发布世界系当前帧点云 | 已迁移为 `publishWorldFrameCloud`，设置页可配置 |
| `publish/dense_publish_en` | `true` | 世界系当前帧 dense/降采样选择 | 已迁移为 `publishDenseFrameCloud`，设置页可配置 |
| `publish/scan_bodyframe_pub_en` | `true` | 发布机体系当前帧点云 | 已迁移为 `publishBodyFrameCloud`，设置页可配置 |
| `pcd_save/pcd_save_en` | `true` | 累计保存完整地图 | 已迁移为 `saveMap`，设置页可配置 |
| `pcd_save/interval` | `-1` | PCD 分片间隔 | 未迁移；当前导出为手动触发完整 PCD/LAS |
| `runtime_pos_log_enable` | 未配置 | 调试日志 | 未迁移为原版文件日志 |

## 5. 当前项目流程

### 5.1 输入源

PCAP 路径由 `PcapSlamSource` 实现：

- 使用 libpcap 读取 Livox UDP payload。
- 支持 Cartesian high、Cartesian low、Spherical、Double echo 点云格式。
- 使用 Livox packet timestamp 和 `time_interval * 100ns` 计算点内 offset。
- 按 `inputFrameDurationMs` 聚合为 `SlamInputFrame`，默认由 `preprocessScanRateHz=10` 推导为 100 ms；`frameEndNs` 仍来自帧内实际最大点时间。
- 解析 IMU payload，并为每帧附加覆盖帧起止时间的 IMU 样本。
- 缺 IMU、点内 offset 缺失、点云乱序或 IMU 覆盖不完整时给出 `MissingImu` 或 `TimeSyncError`。

Live 路径由 `LiveLidarSlamSource` 实现，已能把实时 SDK packet 转为 `SlamInputFrame` 并推入 `SlamInputQueue`。当前主 UI 的 SLAM 启动路径仍以 PCAP worker 为主。

### 5.2 后端

`FastLioSlamBackend::processFrame()` 保留原版 FAST_LIO 核心算法顺序：

1. 校验后端已启动、IMU 样本存在、IMU 覆盖完整、点内 offset 存在。
2. 把 `SlamInputFrame` 转为 FAST_LIO `MeasureGroup`。
3. 调用 `ImuProcess::Process()`。
4. 调用 `laserMapFovSegment()`。
5. 用 `filterSizeSurfM` 执行 surf 降采样。
6. 首次建图时 `ikdtree.Build()`。
7. 正常帧执行 IKFOM 迭代更新和 `mapIncremental()`。
8. 输出当前位姿、轨迹点、后端 IKD-tree 地图点数。
9. 按配置输出世界系当前帧点云、机体系当前帧点云。
10. 按 `saveMap` 配置输出完整全局地图增量点。

`SlamOutput::mapPointCount` 仍来自 `ikdtree.validnum()`，表示后端局部 IKD-tree 有效点数。

### 5.3 UI 与渲染

旁路 UI 原则保持不变：

- `SlamUiBridge` 是 QObject，不是 QWidget。
- 工具菜单提供 `SLAM（在线）` / `SLAM（离线）` 两个入口；在线入口启动实时 SLAM worker，离线入口自动弹出 PCAP 文件选择框并使用当前 PCAP SLAM source。
- `SlamControlDialog` 类仍保留为过渡实现，但不再作为工具菜单主入口。
- SLAM 状态字段迁移到底部 `SLAM状态` dock，并与日志 dock tabify；状态布局为横向字段条，适配底部 dock。
- SLAM 控制按钮迁移到 `SLAM` 点云可视化 tab 上方浮动控制条。
- 离线 PCAP 加载后创建独立的 `SLAM` OpenGL tab，不复用离线播放 tab，也不显示离线播放控制条。
- 不修改现有 dock/tab 创建流程。
- 后端线程不直接访问 `PointCloudView`。
- worker 通过 `QMetaObject::invokeMethod(..., Qt::QueuedConnection)` 把 `SlamOutput` 投递到 UI 线程。

`SlamRenderSnapshot` 当前包含：

- 轨迹线顶点。
- 当前位姿坐标轴顶点。
- 世界系当前扫描帧点云顶点。
- 机体系当前帧点云顶点。

世界系点云主显示不走 SLAM overlay，而是转换为 `PointCloudFrame` 后进入 SLAM tab 主点云缓存。工具栏的积分时间、点大小、着色模式和反射率色标直接作用于该主点云，对应原版 RViz 中 `currPoints` 通过客户端长 Decay 叠加 `/cloud_registered` 的显示语义。

世界系当前帧点云使用同一份 `publishedWorldFramePoints` 数据源生成固定颜色 overlay，只保留最新一帧，对应原版 RViz 中 `surround` 订阅 `/cloud_registered` 且 Decay Time = 0 的显示语义。

状态字段中的 `局部 ikd-tree 有效点数` 对应 `SlamOutput::mapPointCount`，来源为 FAST_LIO 后端 `ikdtree.validnum()`；`世界系点云总数` 是 UI bridge 累计收到的世界系发布点云总数，不等同于局部 ikd-tree 点数。

`PointCloudView::setSlamRenderSnapshot()` 和 `clearSlamRenderOverlay()` 仍断言必须在 UI 主线程调用。OpenGL VBO/VAO 创建、写入和销毁只在当前 context 有效时执行。

### 5.4 完整地图导出

完整全局地图数据源来自 FAST_LIO 后端输出的 `newGlobalMapPoints`：

- 点源匹配原版 `pcd_save`：使用 `feats_undistort` dense 当前帧。
- 每帧点被变换到 world frame 后追加。
- `SlamUiBridge` 只缓存后端输出的完整地图点，不从 OpenGL VBO 读取。
- `SlamMapExport` 直接流式写 `QVector<SlamPoint>` 到 PCD/LAS，不先复制成 UI 点云数组。
- 文件写入在后台线程执行，不在 UI 线程执行完整地图导出。

## 6. 当前配置

`SlamRuntimeConfig` 定义在 `libs/Slam/include/Slam/Core/SlamRuntimeConfig.h`，通过 `QSettings` 的 `slam/runtime` 前缀保存。

| 参数 | 默认值 | 设置页 | 当前影响 |
| --- | ---: | --- | --- |
| `backendType` | `FAST_LIO` | 否 | 当前 worker 固定创建 `FastLioSlamBackend` |
| `lidarModel` | 空 | 否 | 占位 |
| `lidarTemplate` | `Mid360/Mid360S` | 是 | 控制 SLAM 设置页模板默认值；当前模板差异项为 `detRangeM`、`fovDegree`、`extrinsicT_L_I` 和 `blindMinRangeM` |
| `imuEnabled` | `true` | 否 | false 会导致 FAST_LIO 拒绝启动 |
| `allowPureLidar` | `false` | 否 | 占位；FAST_LIO 当前仍强制要求 IMU |
| `lidarToImuTimeOffsetNs` | `0` | 否 | 已持久化，当前未实际应用 |
| `gravityNorm` | `9.81` | 是 | 对应原版 `G_m_s2`；传入 `ImuProcess`，用于 IMU 初始化重力向量长度、加速度归一化和初始化协方差缩放 |
| `extrinsicT_L_I[3]` | `[-0.011,-0.02329,0.04412]` | 是 | 后端初始化 LiDAR 到 IMU 外参平移；旧版本保存的 `[0,0,0]+identity` 默认值会迁移为当前模板默认；Avia 模板默认 `[0.04165,0.02326,-0.0284]` |
| `extrinsicR_L_I[9]` | identity | 是 | 后端初始化 LiDAR 到 IMU 外参旋转 |
| `extrinsicEstimationEnabled` | `false` | 是 | 对应原版 `mapping/extrinsic_est_en`；false 时外参固定，EKF 量测雅可比的外参 6 列置零 |
| `cubeSideLengthM` | `200.0` | 是 | 对应原版 `cube_side_length`；控制局部 ikd-tree 滑动地图立方体边长 |
| `detRangeM` | `100.0` | 是 | 对应原版 `mapping/det_range`；控制局部地图滑动触发距离；Avia 模板默认 450.0 m |
| `fovDegree` | `360.0` | 是 | 对应原版 `mapping/fov_degree`；用于视野内地图点判断，内部按原版裁到 179.9°；Avia 模板默认 90° |
| `blindMinRangeM` | `0.5` | 是 | 对应原版 `preprocess/blind`；在 `SlamInputFrame -> MeasureGroup` 转换时滤除 LiDAR 坐标系近场点；Avia 模板默认 4.0 m |
| `maxIterations` | `4` | 是 | 对应原版 `max_iteration`；控制每帧 iEKF 最大迭代次数 |
| `gyrCov` | `0.1` | 是 | 对应原版 `mapping/gyr_cov`；传入 `ImuProcess::set_gyr_cov()` |
| `accCov` | `0.1` | 是 | 对应原版 `mapping/acc_cov`；传入 `ImuProcess::set_acc_cov()` |
| `bGyrCov` | `0.0001` | 是 | 对应原版 `mapping/b_gyr_cov`；传入 `ImuProcess::set_gyr_bias_cov()` |
| `bAccCov` | `0.0001` | 是 | 对应原版 `mapping/b_acc_cov`；传入 `ImuProcess::set_acc_bias_cov()` |
| `filterSizeSurfM` | `0.5` | 是 | 输入 surf 点降采样；越大点越少 |
| `filterSizeMapM` | `0.5` | 是 | IKD-tree 地图增量体素；影响后端地图点数 |
| `preprocessScanRateHz` | `10.0` | 是 | 对齐原版 `preprocess/scan_rate` 默认语义；用于推导 SLAM 输入聚帧周期 |
| `inputFrameDurationMs` | `100` | 是 | PCAP/Live SLAM 输入源切分当前帧的周期；实际帧结束时间仍取帧内最后点时间 |
| `publishWorldFrameCloud` | `true` | 是 | 输出世界系 `/cloud_registered` 语义点云；同时供 SLAM tab 积分窗口和世界系当前帧 overlay 使用 |
| `publishDenseFrameCloud` | `true` | 是 | 世界系点云使用 dense 去畸变点云；false 使用降采样点云 |
| `publishBodyFrameCloud` | `true` | 是 | 在世界系发布开启时输出/显示 IMU 机体系当前帧点云 |
| `mapVoxelSizeM` | `0.1` | 否 | 占位 |
| `maxMapPoints` | `2,000,000` | 否 | 占位；当前不限制完整地图导出点数 |
| `maxTrajectoryPoints` | `200,000` | 否 | 占位；UI bridge 内部仍限制轨迹缓存 |
| `maxInputQueueFrames` | `8` | 否 | Live source queue 容量相关 |
| `saveTrajectory` | `false` | 否 | 占位；当前轨迹手动导出 |
| `saveMap` | `true` | 是 | 累计完整全局地图点，用于 PCD/LAS 导出 |
| `logLevel` | `info` | 否 | 占位 |

## 7. 迁移完成情况

已完成：

- FAST_LIO 核心算法以 `FastLioSlamBackend` 嵌入项目。
- PCAP 离线 SLAM worker 已接入 UI，支持按原始时间 replay、暂停、停止、重置。
- 当前位姿、轨迹 overlay 已接入 `PointCloudView`。
- 原版 `scan_publish_en`、`dense_publish_en`、`scan_bodyframe_pub_en` 已迁移为用户可配置项。
- 原版 RViz `surround`/`currPoints` 对同一 `/cloud_registered` 的两种显示语义已迁移：`世界系当前帧点云` 固定颜色 overlay 只显示最新帧，`世界系点云` 主点云按积分时间窗口累计显示。
- 原版 `preprocess/scan_rate` 已迁移为用户可配置扫描频率，并同步控制 PCAP/Live SLAM 输入聚帧周期。
- 原版 `mapping/extrinsic_est_en` 已迁移为 `extrinsicEstimationEnabled`，默认 false；关闭时外参雅可比列按原版置零。
- MID360 原版外参 `[-0.011,-0.02329,0.04412] + identity` 已作为默认值，并在 SLAM 设置页暴露平移/旋转配置。
- 原版 `cube_side_length`、`mapping/det_range`、`mapping/fov_degree`、`max_iteration` 已迁移为用户可配置项。
- 原版 `preprocess/blind` 已迁移为 `blindMinRangeM`，在进入 FAST_LIO IMU 去畸变和建图前过滤近距离点。
- SLAM 设置页新增 LiDAR 模板：`Mid360/Mid360S` 和 `Avia`。模板只覆盖已迁移的原版差异项：探测距离、水平视场角、外参平移 T 和近距离盲区；恢复默认按钮会按当前模板恢复 SLAM 运行参数默认值。
- 原版重力常量 `G_m_s2` 已迁移为 `gravityNorm`，默认 9.81 m/s²，并传入 `ImuProcess`。
- 原版 IMU 噪声参数 `gyr_cov`、`acc_cov`、`b_gyr_cov`、`b_acc_cov` 已迁移为用户可配置项，并传入 `ImuProcess`。
- 世界系 dense/降采样点云已作为 SLAM tab 主点云输出，可复用工具栏积分时间、点大小、着色模式和色标。
- 世界系当前帧点云作为 SLAM overlay 输出，颜色由首选项 SLAM 页配置，默认白色。
- 机体系当前帧 dense 点云仍作为 SLAM overlay 输出，颜色由首选项 SLAM 页配置，默认绿色。
- 旧地图预览配置、store、增量预览 VBO 和预览导出已移除。
- 原版 `pcd_save` 的 dense world-frame 累计语义已迁移为完整全局地图缓存。
- 新增完整全局地图 PCD/LAS 导出，数据源不是 UI 预览缓存，也不是 OpenGL VBO。
- 轨迹 CSV/TUM 导出保留。

部分完成：

- Live SLAM 输入源已实现，但实时 SLAM worker 尚未成为主 UI 可用流程。
- `lidarToImuTimeOffsetNs`、`mapVoxelSizeM`、`maxMapPoints` 等仍是占位或未完全使用。
- 原版 PCD `interval` 分片保存未迁移；当前是手动导出一个完整 PCD/LAS。

未完成：

- 原版 `time_sync_en` 自同步逻辑未等价迁移。
- 原版 runtime debug 文件输出未迁移。
- 多 LiDAR PCAP SLAM 未支持。
- Velodyne/Ouster 等非 Livox 预处理路径未迁移。
- Linux/macOS 构建和运行未验证。

## 8. 风险与后续建议

- 完整地图保存默认开启会增加内存占用；长时间 dense 轨迹可能占用大量内存。后续可以增加点数上限、分片落盘或后台 streaming writer。
- 当前完整地图点源是原版 `pcd_save` 语义的 dense scan accumulation，不是 IKD-tree flatten。若后续需要导出 IKD-tree 全局地图，应新增独立功能和文案。
- `detRangeM` 默认已改为 Mid360/Mid360S 模板的 100 m；旧版本已保存的用户配置不会被强制覆盖，需要用户在首选项 SLAM 页点击“恢复默认”才会套用模板默认值。
- `publishBodyFrameCloud` 在同一个 3D 视图中显示 IMU body frame 点云，坐标语义不同于 world frame；它用于迁移原版发布输出，不应被理解为全局地图。
- 完整地图导出已放到后台线程，但导出前从 `SlamUiBridge` 获取 `QVector<SlamPoint>` 快照仍可能带来短时内存压力。

## 9. 当前结论

当前迁移已经从“UI 稀疏/稠密地图预览”切换到“原版发布语义 + 完整后端地图保存/导出”。后续重点是继续减少硬编码参数、补齐时间偏移实际应用，以及为长轨迹完整地图保存增加更稳的分片或流式落盘机制。

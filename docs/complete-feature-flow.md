# LivoxViewerQT 完整功能流程文档

更新时间：2026-05-24
当前代码基线：`81c35d8 refactor: encapsulate point cloud filter state` 之后的工作区状态

本文档按当前项目文件结构和代码职责梳理 LivoxViewerQT 的完整功能流程。当前项目仍构建为单一可执行程序 `LivoxViewerQT`，但代码已经拆分为应用层 `apps/LivoxViewer/` 与多个内部库目录 `libs/`。

## 1. 项目定位

LivoxViewerQT 是基于 Qt、CMake、Livox SDK2、Npcap/libpcap 的 Livox 激光雷达可视化、控制、采集和离线分析工具。

主要能力：

- 实时发现 Livox 雷达并初始化 Livox SDK。
- 实时接收点云、IMU、状态和参数回调。
- OpenGL 渲染点云，支持视图控制、着色、投影、框选和测距。
- 查询、显示、修改和记录雷达参数。
- 录制 LVX2、保存 PCD/LAS、保存 IMU CSV、采集 LOG/Debug 数据。
- 离线加载并播放 LVX2、PCAP、PCAPNG、CAP 文件。
- 将 LVX2 转换为 PCD、LAS、CSV、TXT。

## 2. 当前目录职责

```text
LivoxViewerQT/
  apps/LivoxViewer/                 Qt 应用层、主窗口、菜单、面板、动作入口
    actions/                        主窗口业务动作实现
    dialogs/                        对话框创建与交互
    panels/                         Dock 面板和工具栏创建
    state/                          应用层状态聚合结构
  libs/AppConfig/                   配置文件、网卡和应用设置服务
  libs/Export/                      PCD/LAS/CSV/TXT 点云导出
  libs/LivoxCore/                   Livox SDK、发现、参数服务和共享类型
  libs/Lvx2/                        LVX2 读取、帧索引、点解析
  libs/Pcap/                        PCAP 读取、UDP 解析、Push/点云解析
  libs/Playback/                    离线播放统一 Source 抽象
  libs/PointCloud/                  点云类型、解码、滤波、着色、投影、OpenGL 视图
  livox_sdk_qt/                     Livox SDK2 头文件和静态库
  third-party/npcap-sdk-1.16/       Npcap SDK 头文件和 Windows 链接库
  testdata/manual/                  本地手工 smoke 样例，真实样例不入库
  docs/                             架构、流程和重构文档
```

## 3. 启动流程

入口位于 `apps/LivoxViewer/main.cpp`。

1. Windows 下调用 `SetDllDirectoryA("C:\\Windows\\System32\\Npcap")`，让运行时优先从 Npcap 安装目录加载 DLL。
2. 创建 `QApplication`，设置应用名 `LivoxViewerQT`、版本 `1.3.0`、组织名和图标。
3. 创建并显示 `LivoxViewerWindow`。
4. `LivoxViewerWindow` 构造函数执行：
   - `initializeUserInterface()` 创建主界面。
   - `loadViewPreferences()` 恢复点云网格显示偏好。
   - 从 `QSettings` 读取自动配置主机 IP 开关。
   - `refreshNetworkInterfaces()` 刷新本机网卡列表。
   - `startLidarDiscovery()` 启动雷达发现流程。
   - 创建 `paramQueryTimer`，每 1 秒查询参数。
   - 恢复窗口几何和 dock 布局。
5. Qt 事件循环处理 UI、定时器、UDP、SDK 回调转发和 OpenGL 渲染刷新。
6. 析构时保存窗口布局和视图偏好，停止发现流程并关闭 Livox SDK。

## 4. UI 组织流程

UI 由 `apps/LivoxViewer/LivoxViewerUi.cpp` 组织，具体控件创建已经拆到 `panels/`、`dialogs/` 和 `actions/`。

主界面组成：

- 中央点云视图：`PointCloudView`，基于 `QOpenGLWidget`。
- 视图工具栏：`panels/ViewerToolbar.cpp`。
- 回放控制条：`panels/PlaybackBar.cpp`。
- 设备面板：`panels/DevicePanel.cpp`。
- 参数面板：`panels/ParameterPanel.cpp`。
- 文件信息面板：`panels/FileInfoPanel.cpp`。
- IMU 面板：`panels/ImuPanel.cpp`。
- 日志面板：`panels/LogPanel.cpp`。
- 菜单和动作：`actions/MainMenuActions.cpp` 只负责菜单骨架，具体动作分散到 `FileActions`、`CaptureActions`、`DeviceActions`、`PlaybackActions`、`HelpActions`、`StatusRuntime` 等文件。

### 4.1 工具栏功能

工具栏负责点云显示相关控制：

- 积分时间。
- 点大小。
- 着色模式：反射率、距离、高程、纯色、平面投影。
- 纯色选择。
- 球面深度投影。
- 平面投影和投影半径。
- 暂停/恢复点云可视化。
- 框选、测距、视角重置和视角预设。

### 4.2 Dock 面板功能

设备面板：

- 网卡选择。
- 自动配置主机 IP 开关。
- 雷达设备列表。
- GPS 模拟。
- 串口列表刷新和串口转发。

参数面板：

- 基本参数。
- 网络参数。
- FOV 参数。
- 外参参数。
- 状态参数。
- 参数 CSV 记录。

文件信息面板：

- 显示离线 LVX2/PCAP 设备信息。
- 控制回放中各设备点云显示/隐藏。

IMU 面板：

- 显示最新陀螺仪和加速度值。
- 显示 ASCII 状态。
- 打开 IMU 曲线窗口。

日志面板：

- 显示运行日志。
- 支持清空日志。

## 5. 网络与设备发现流程

主要实现位于 `apps/LivoxViewer/actions/LidarSdkActions.cpp`，纯解析能力由 `libs/LivoxCore/LidarDiscoveryService` 提供。

1. 启动时刷新本机网络接口，过滤不可用、回环、虚拟和无效 IPv4 接口。
2. 用户可在设备面板选择用于雷达通信的有线网卡。
3. 如果没有检测到有线网口连接，程序打印一次等待提示，并保持 2 秒一次的定时检测；定时检测不会持续刷屏。
4. 检测到有线网口后，创建 UDP socket 并监听发现端口。
5. 周期性广播 Livox 发现命令。
6. 收到雷达响应后解析设备 IP、型号、序列号等信息。
7. 若设备与主机 IP 不在兼容网段：
   - Windows 默认允许自动配置主机 IP。
   - Linux 默认关闭自动配置，需要用户确认或手动配置。
8. 发现到的设备写入 `lidarDevices`，刷新设备列表，并用于后续配置文件和 SDK 初始化。

## 6. 配置文件与 SDK 初始化流程

配置能力分散在应用层 `LidarSdkActions.cpp` 和库层 `libs/AppConfig`、`libs/LivoxCore`。

1. 初始化 SDK 前先检查是否存在有线设备连接。
2. 解析标准配置目录中的 `config.json`；不存在时检查兼容旧路径。
3. 未找到配置文件时打开配置生成对话框。
4. 找到配置文件后校验 `host_net_info[*].host_ip` 是否与当前选择的主机 IP 一致。
5. 不一致时自动更新配置文件中的 host IP。
6. 若发现流程拿到了设备类型，尝试修正配置文件中的 device type。
7. 调用 Livox SDK 初始化接口。
8. 注册设备信息、点云、IMU、状态、控制响应和参数响应回调。
9. 初始化成功后进入可采样/可显示状态。
10. 关闭时先停止发现，再注销 SDK，避免回调访问已销毁状态。

## 7. SDK 回调流程

主要实现位于 `apps/LivoxViewer/actions/LidarSdkCallbacks.cpp`。

### 7.1 设备信息回调

1. SDK 回调收到设备上线、离线或状态变化。
2. 深拷贝必要字段。
3. 通过 Qt 主线程更新 `lidarDevices`。
4. 刷新设备列表和当前设备信息。

### 7.2 点云回调

1. SDK 回调收到 `LivoxLidarEthernetPacket`。
2. 校验点数、数据类型和 payload 长度。
3. 深拷贝包数据，避免 SDK 回调返回后内存失效。
4. 通过 `QMetaObject::invokeMethod` 回到主线程。
5. 调用 `decodePointCloudPacket()` 解码为 `PointCloudFrame`。

### 7.3 IMU 回调

1. 只处理当前设备或可匹配设备的 IMU 数据。
2. 解析 gyro 和 acceleration。
3. 更新 `imuState.latestSample`。
4. 若 IMU CSV 保存开启，追加 CSV 行。
5. IMU 面板和曲线线程读取最新样本显示。

### 7.4 状态与参数回调

1. 状态回调更新日志和设备状态。
2. 参数响应回调解析内部参数 key、length、value。
3. 状态参数直接更新 label。
4. 可配置参数只在首次或必要时同步到控件，避免覆盖用户正在编辑的值。

## 8. 参数查询、配置和记录流程

主要实现位于 `apps/LivoxViewer/actions/LidarParameterActions.cpp`，状态集中在 `apps/LivoxViewer/state/ParameterUiState.h`。

1. `paramQueryTimer` 每 1 秒触发 `onParamQueryTimeout()`。
2. 当前设备存在时查询内部参数。
3. 响应回调进入主线程解析参数。
4. 参数按类别显示到参数面板：
   - 基本参数。
   - 网络参数。
   - FOV 参数。
   - 外参参数。
   - 状态参数。
5. 用户修改控件后进入 `onParamConfigChanged()`。
6. 根据参数类型调用：
   - `applyIpConfig()`。
   - `applyHostIpConfig()`。
   - `applyFovConfig()`。
   - `applyAttitudeConfig()`。
7. 参数记录开启后创建 CSV 文件，按固定字段顺序周期写入参数值。
8. 停止记录时 flush 并关闭文件。

## 9. 实时点云流程

实时点云入口在 `apps/LivoxViewer/actions/PointCloudActions.cpp`，算法模块在 `libs/PointCloud/`。

1. SDK 点云回调把原始包交给 `decodePointCloudPacket()`。
2. `PointCloudDecoder` 按 Livox 数据类型解析：
   - 笛卡尔高精度点。
   - 笛卡尔低精度点。
   - 球坐标点。
   - 双回波点。
3. 每个点转换为 `PointCloudPoint`，包含坐标、反射率、颜色、tag 等字段。
4. 解码后的帧按设备 handle 放入 `pendingFrames`。
5. `renderTimer` 触发 `onRenderTick()`。
6. 渲染 tick 根据当前积分时间合并滑动窗口内的帧。
7. 合并结果进入 `applyPointCloudPipeline()`：
   - tag 噪点滤波。
   - 着色。
   - 球面深度投影。
   - 平面投影。
   - 图例更新。
8. `presentPointCloudFrame()` 把结果交给 `PointCloudView`。
9. `PointCloudView` 更新 VBO 并触发 OpenGL 重绘。

## 10. 点云显示与交互流程

`libs/PointCloud/src/PointCloudView.cpp` 是 OpenGL 视图层。

支持能力：

- 点云缓冲上传和 shader 渲染。
- 坐标轴、网格、图例和测距线渲染。
- 鼠标旋转、平移、缩放。
- 双击设置视图目标点。
- 世界、前、后、左、右、俯视等视角预设。
- 框选点云并生成点属性表。
- AABB 持久选择。
- 测距模式下选择两个点并计算三维距离。
- 拖放 LVX2、PCAP、PCAPNG、CAP 文件触发离线加载。

## 11. 采集、录制和保存流程

采集状态集中在 `apps/LivoxViewer/state/CaptureSessionState.h`，入口分布在 `CaptureActions.cpp` 与 `PointCloudActions.cpp`。

### 11.1 LOG 与 Debug 采集

1. 用户从工具菜单启动 LOG 或 Debug 采集。
2. 程序检查当前是否已有采集任务。
3. 初始化倒计时、进度条和采集状态。
4. LOG 调用 Livox SDK 日志保存接口。
5. Debug 调用 Livox SDK Debug 点云接口。
6. 到达指定时长后停止采集并恢复 UI。

### 11.2 PCD/LAS 保存

1. 用户选择保存目录和帧数。
2. 程序创建按设备 SN 命名的输出目录。
3. 渲染 tick 中按合并后的帧写出 PCD 或 LAS。
4. 输出函数通过 `libs/Export/PointCloudExport` 完成。
5. 达到指定帧数后自动停止。

### 11.3 LVX2 录制

1. 用户选择 LVX2 保存路径和录制时长。
2. 程序写入 LVX2 public header、private header 和设备信息。
3. 点云回调中把原始 SDK 点云包封装为 LVX2 package。
4. 按 50ms frame 写入 frame header 与 package。
5. 录制倒计时结束后 flush 并关闭文件。

### 11.4 IMU CSV 保存

1. 用户启用设备 IMU 数据发送。
2. 选择保存目录和时长。
3. 程序创建 `IMU_<SN>` 目录和 CSV 文件。
4. IMU 回调逐样本写入 timestamp、gyro、acceleration。
5. 到达时长后 flush 并关闭文件。

## 12. IMU、GPS 和串口流程

IMU/GPS/串口运行状态集中在 `apps/LivoxViewer/state/ImuRuntimeState.h`。

1. IMU 回调更新最新样本。
2. IMU 面板读取最新样本并更新表格、进度条和 ASCII 状态。
3. 用户可打开 IMU 曲线窗口，后台线程周期刷新 gyro 和 acceleration 曲线。
4. GPS 模拟启用后定时生成 GPRMC 类报文并写入日志。
5. 串口转发启用后打开所选串口，后台线程读取 NMEA 报文。
6. 串口线程识别 RMC/GGA/GSA/GSV 等内容并更新日志或状态。

## 13. LVX2 离线播放流程

离线播放状态集中在 `apps/LivoxViewer/state/PlaybackControllerState.h`，统一来源抽象位于 `libs/Playback/include/Playback/PlaybackSource.h`。

1. 用户通过菜单、拖放或文件入口选择 LVX2 文件。
2. 应用关闭当前播放状态并创建加载 token。
3. 后台读取 LVX2 header、设备信息和 frame index。
4. 主线程校验 token，设置 `playbackState.source`、设备列表、帧数、当前帧和设备可见性。
5. 回放条支持：
   - 播放/暂停。
   - 首帧、上一帧、下一帧、尾帧。
   - 关闭。
   - 进度拖动。
   - 速度切换。
   - 逐帧/滑窗模式切换。
6. 显示帧时从 `Lvx2Reader` 读取 frame。
7. `Lvx2PointParser` 解析 package 并应用设备外参。
8. 点云进入统一处理与显示流程。

## 14. PCAP 离线播放流程

PCAP 读取能力位于 `libs/Pcap/`。

1. 用户选择 PCAP、PCAPNG 或 CAP 文件。
2. Windows 使用 Npcap SDK 链接库，运行时依赖本机 Npcap DLL。
3. `PcapParser` 离线读取包。
4. `PcapUdpPacket` 解析 UDP 源/目的 IP、端口和 payload。
5. 端口 56200 的 Push 包用于解析设备 SN、型号、IP 和外参。
6. 端口 56300 的点云包用于解析 Livox 点云数据。
7. `PointParser::FrameBuilder` 按 50ms 聚合帧。
8. `PcapReader` 输出与 LVX2 兼容的 `Playback::Source`。
9. UI 复用同一套回放条、设备列表、设备可见性和点云显示流程。

## 15. LVX2 格式转换流程

入口为 `apps/LivoxViewer/dialogs/FormatConvertDialog.cpp` 和 `apps/LivoxViewer/actions/Lvx2ConvertActions.cpp`。

1. 用户选择源 LVX2、输出目录、输出文件名、转换模式和格式。
2. 转换支持：
   - 合并全部帧到一个文件。
   - 按固定时间片拆分输出。
3. 输出格式支持：
   - PCD。
   - LAS。
   - CSV。
   - TXT。
4. 转换过程读取 LVX2 header、设备信息、frame index 和 package。
5. 解析点云并应用外参。
6. 调用 `libs/Export/PointCloudExport` 写出目标格式。
7. UI 根据进度回调刷新进度。

## 16. 构建、部署和 smoke 流程

当前主机已验证的关键路径：

- Visual Studio 2026 Community：`B:\Program Files\Microsoft Visual Studio\18\Community`
- Qt 6.8.3 MSVC 2022 64-bit：`B:\Qt\6.8.3\msvc2022_64`
- CMake：`B:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe`

推荐命令：

```powershell
& "B:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B build-msvc -A x64 -DCMAKE_PREFIX_PATH="B:\Qt\6.8.3\msvc2022_64"
& "B:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-msvc --config Release --target LivoxViewerQT
& "B:\Qt\6.8.3\msvc2022_64\bin\windeployqt.exe" "B:\Workspace\LivoxViewerQT\build-msvc\Release\LivoxViewerQT.exe"
& "B:\Workspace\LivoxViewerQT\build-msvc\Release\LivoxViewerQT.exe"
```

离线样例约定：

- `testdata/manual/sample.lvx2`
- `testdata/manual/sample.pcap`

真实样例被 `.gitignore` 忽略，不应提交。

## 17. 当前架构特征

当前项目已经完成一轮保守拆分：

- `libs/**` 已基本脱离 `LivoxViewerWindow`，承担纯服务、模型、解析和算法职责。
- 应用层 `apps/LivoxViewer/actions/**` 仍以 `LivoxViewerWindow::` 成员函数承载大量业务。
- `LivoxViewerWindow.h` 已通过多个 `state/*State.h` 收敛状态，但仍持有大量跨模块控件指针、SDK 状态、点云状态和设备状态。
- `PointCloudView.cpp` 是较完整的 OpenGL 视图对象，功能集中但边界相对清晰。
- 后续重构重点应从“把代码移出 libs”转向“拆应用层 controller/panel/dialog 对象，继续缩小 LivoxViewerWindow”。

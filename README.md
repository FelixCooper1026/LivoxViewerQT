# LivoxViewerQT

LivoxViewerQT 是一个基于 Qt/CMake 的 Livox 激光雷达可视化、控制、离线播放和 SLAM 工具。集成 FAST_LIO 风格后端与 M-detector 动态目标检测，支持实时点云、LVX2/PCAP/ROSbag 离线播放，以及在线/离线 SLAM 复验。

## 功能概览

- 实时设备自动发现与连接
- 实时点云显示、着色、框选、测距和裁切
- 设备参数查询/控制命令下发、固件升级
- LOG / Debug 信息采集、LVX2 录制、PCD/LAS 导出、IMU 数据保存
- LVX、LVX2、PCAP/PCAPNG/CAP、ROS1 bag、ROS2 db3/metadata.yaml 离线点云回放
- ROSbag 播放支持 ROS1 uncompressed/lz4 chunk、Livox CustomMsg、Livox/通用 PointCloud2 和无 IMU 纯 XYZ 点云
- 剪枝移植后的 FAST_LIO 后端，支持在线 SLAM 与离线 SLAM
- SLAM 功能包含 LIO/LO 模式建图、轨迹、世界系/机体系点云显示、CSV/TUM 轨迹导出、PCD/LAS 全局地图导出
- 基于 M-detector 的动态目标检测，支持无聚类事件点模式与聚类增强模式
- 动态检测可独立显示瞬时世界系当前帧与动态目标点，不依赖历史地图点云累计
- 点云裁切支持动态检测的瞬时世界系当前帧和动态目标 overlay
- 命令行离线 SLAM 诊断工具 `SlamReplayTool`

## 项目文件结构

```text
LivoxViewerQT/
  CMakeLists.txt                         根 CMake 工程，直接维护应用、库和工具目标
  README.md                              项目概览、构建方式和目录说明
  apps/
    LivoxViewer/                         Qt 主程序
      actions/                           菜单、播放、点云、IMU、SDK、参数等动作实现
      dialogs/                           对话框与窗口工具
      imu/                               IMU 可视化相关实现
      panels/                            侧边栏、状态栏和信息面板
      slam/                              SLAM UI、控制条、状态桥和窗口动作
      state/                             运行时状态结构
      utils/                             应用层工具函数
      widgets/                           自定义 Qt 控件
  libs/
    AppConfig/                           应用配置、配置 JSON、网卡服务
    DynamicObject/                       M-detector 动态目标检测与聚类
    Export/                              通用点云导出
    LivoxCore/                           Livox SDK 封装、设备发现、参数服务
    Lvx/                                 LVX 读取和点云包解析
    Lvx2/                                LVX2 读取、解析和 Playback::Source 实现
    Pcap/                                PCAP 解析、IMU/点云 payload 解析和 Playback::Source 实现
    Playback/                            LVX2/PCAP/ROSbag 共用离线播放抽象
    PointCloud/                          点云数据结构、滤波、着色、OpenGL 视图
    Rosbag/                              ROS1/ROS2 bag reader、消息反序列化、普通播放适配
    Slam/                                SLAM 类型、队列、运行参数、FAST_LIO 后端、数据源和导出
      third_party/fast_lio/              移植的 FAST_LIO/IKFoM/ikd-Tree 代码
      third_party/fast_lio_compat/       ROS/PCL 最小兼容头层
  plugins/
    StlModel/                            STL 模型加载插件
  resources/                             Qt 资源文件
  icons/                                 应用图标
  pics/                                  README/界面相关图片资源
  scripts/
    setup_third_party.ps1                Windows 第三方头文件安装脚本
    setup_third_party.sh                 Linux/macOS 第三方头文件安装脚本
  tools/
    SlamReplayTool/                      离线 SLAM 回归与诊断命令行工具
  livox_sdk_qt/                          项目内嵌 modified Livox SDK 头文件与库
  third-party/
    eigen-3.4.0/                         Eigen 头文件
    lz4-1.10.0/                          官方 LZ4 最小源码，用于 ROS1 bag lz4 chunk 解压
    npcap-sdk-1.16/                      Windows PCAP 构建所需 Npcap SDK
```

## 构建依赖

- CMake `>= 3.16`
- C++17 编译器
- C 编译器，用于构建项目内官方 LZ4 源码
- Qt `>= 6.2`，CMake 中保留 Qt 5.15 回退
- Qt 模块：Core、Widgets、OpenGL、OpenGLWidgets、SerialPort、Charts、Network、Svg、Concurrent、Sql
- Eigen 3.4.0 headers，安装到 `third-party/eigen-3.4.0`
- LZ4 1.10.0 源码，随项目放在 `third-party/lz4-1.10.0`
- OpenMP C++ runtime/compiler support
- Windows：MSVC、Npcap Runtime、`third-party/npcap-sdk-1.16`
- Linux：Qt 开发包、OpenMP、`libpcap-dev`、`pthread`、`dl`、`m`

不需要安装完整 Boost 或完整 PCL。FAST_LIO 固定状态类型已去除 Boost 预处理依赖，PCL 相关类型由项目内 `fast_lio_compat` 最小兼容头层提供。

## 第三方头文件安装

Windows：

```powershell
pwsh -NoProfile -ExecutionPolicy Bypass -File .\scripts\setup_third_party.ps1
```

Linux/macOS：

```bash
bash scripts/setup_third_party.sh
```

强制重装 Eigen：

```bash
bash scripts/setup_third_party.sh --force
```

## 编译与运行

Windows 构建：

```powershell
$buildDir = "build\cmd-msvc-Release"
$qtDir = $env:QT_DIR
if (-not $qtDir) { $qtDir = Split-Path (Split-Path (Get-Command qmake).Source) }
cmake -S . -B $buildDir -G "Visual Studio 17 2022" -A x64 -DCMAKE_PREFIX_PATH="$qtDir"
cmake --build $buildDir --config Release --target LivoxViewerQT SlamReplayTool --parallel
& "$buildDir\Release\LivoxViewerQT.exe"
```

Linux 构建：

```bash
bash scripts/setup_third_party.sh
cmake -S . -B build/cmd-linux-Release -DCMAKE_BUILD_TYPE=Release
cmake --build build/cmd-linux-Release -j
./build/cmd-linux-Release/LivoxViewerQT
```

Linux 下可执行文件会优先从 `./livox_sdk_qt/lib` 查找 Livox SDK 动态库。

## Windows Portable 包

Windows x64 的 portable 包以 `LivoxViewerQT-2.2.0-win64-qt6.8.3-portable.zip` 发布，解压后直接运行 `LivoxViewerQT.exe`。包内包含 Qt 运行时、MSVC 运行库和项目所需的 Livox SDK 动态库；系统仍需安装 Npcap Runtime 才能读取 PCAP 或使用网络抓包能力。

## 离线 SLAM 诊断

`SlamReplayTool` 支持两类用法：

```bat
build\cmd-msvc-Release\Release\SlamReplayTool.exe with_imu.pcap no_imu.pcap
build\cmd-msvc-Release\Release\SlamReplayTool.exe --diagnose --max-frames 300 path\to\data.bag
build\cmd-msvc-Release\Release\SlamReplayTool.exe --diagnose --source-only --max-frames 10 path\to\pointcloud_only.bag
```

`--diagnose` 支持 `.pcap/.pcapng/.bag/.db3/.yaml/.yml`，会输出数据源摘要、帧/点/IMU 覆盖统计和 FAST_LIO 后端处理结果。`--max-frames N` 只限制后端处理帧数，数据源仍完整加载并统计。`--source-only` 只加载数据源并输出帧统计，不启动 FAST_LIO，适合复验无 IMU 的普通点云文件播放。

## 支持的数据源

| 数据源 | 普通离线播放 | 离线 SLAM | 说明 |
|---|---|---|---|
| LVX/LVX2 | 支持 | 支持 | LVX/LVX2 文件仅包含点云，仅支持 LO 模式建图 |
| PCAP/PCAPNG/CAP | 支持 | 支持 | LIO 模式要求 IMU 和点内时间覆盖 |
| ROS1 `.bag` | 支持 | 支持 | 支持 uncompressed 和 lz4 chunk |
| ROS2 `.db3` / `metadata.yaml` | 支持 | 支持 | 支持 SQLite3 storage |

ROSbag LiDAR topic 支持 Livox CustomMsg、Livox driver2 PointCloud2、允许合成点内时间的旧 driver PointCloud2、通用 `x/y/z/intensity/ring/time` PointCloud2，以及普通播放用的纯 `x/y/z` PointCloud2。离线播放可无 IMU；SLAM 模式要求有效 IMU 覆盖和合理点内时间。

## 许可证

本项目采用 GNU General Public License v2.0 only（GPL-2.0-only），详见 [LICENSE](LICENSE)。

# LivoxViewerQT

LivoxViewerQT 是一个基于 Qt/CMake 的 Livox 激光雷达可视化、控制、离线播放和 SLAM 工具。已集成 FAST_LIO 风格后端，支持实时点云、LVX2/PCAP/ROSbag 离线播放，以及在线/离线 SLAM 复验。

## 功能概览

- 实时设备发现与 Livox SDK 初始化
- 实时点云显示、着色、选择、测距和截面查看
- 设备参数查询/下发、网络参数、FOV、外参和状态管理
- LOG / Debug 采集、LVX2 录制、PCD/LAS 导出、IMU CSV 保存
- LVX2、PCAP/PCAPNG/CAP、ROS1 bag、ROS2 db3/metadata.yaml 离线点云播放
- ROSbag 普通播放复用现有播放条、离线点云 tab、IMU 曲线和着色管线
- 在线 SLAM 与离线 SLAM，后端为移植后的 FAST_LIO 流程
- SLAM 轨迹显示、世界系/机体系点云显示、CSV/TUM 轨迹导出、PCD/LAS 地图导出

## 项目文件结构

```text
LivoxViewerQT/
  CMakeLists.txt                         根 CMake 工程，直接维护应用、库和工具目标
  README.md                              项目概览、构建方式和目录说明
  slam_dev_plan.md                       SLAM 集成主开发记录和复验结论
  fastlio_migration_audit.md             FAST_LIO 移植流程、参数和发布语义审计
  rosbag_slam_source_development_plan.md ROSbag 作为离线 SLAM 数据源的开发方案
  rosbag_offline_playback_plan.md        ROSbag 接入普通离线点云播放的开发方案与实施记录
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
      window/                            主窗口拆分实现
  libs/
    AppConfig/                           应用配置、配置 JSON、网卡服务
    Export/                              通用点云导出
    LivoxCore/                           Livox SDK 封装、设备发现、参数服务
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
    dev_build_run.bat                    当前 Windows 开发构建/启动脚本
    setup_third_party.ps1                Windows 第三方头文件安装脚本
    setup_third_party.sh                 Linux/macOS 第三方头文件安装脚本
    package_windows_release*.bat         Windows 打包脚本
  tools/
    SlamPhase4Replay/                    离线 SLAM 回归与诊断命令行工具
  livox_sdk_qt/                          项目内 Livox SDK 头文件与库
  third-party/
    eigen-3.4.0/                         本地 Eigen 头文件，脚本安装产物
    npcap-sdk-1.16/                      Windows PCAP 构建所需 Npcap SDK
```

## 构建依赖

- CMake `>= 3.16`
- C++17 编译器
- Qt `>= 6.2`，CMake 中保留 Qt 5.15 回退
- Qt 模块：Core、Widgets、OpenGL、OpenGLWidgets、SerialPort、Charts、Network、Svg、Concurrent、Sql
- Eigen 3.4.0 headers，安装到 `third-party/eigen-3.4.0`
- OpenMP C++ runtime/compiler support
- Windows：MSVC、Npcap Runtime、`third-party/npcap-sdk-1.16`
- Linux：Qt 开发包、OpenMP、`libpcap-dev`、`pthread`、`dl`、`m`

不需要安装完整 Boost 或完整 PCL。当前 FAST_LIO 固定状态类型已去除 Boost 预处理依赖，PCL 相关类型由项目内 `fast_lio_compat` 最小兼容头层提供。

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

当前 Windows 开发首选脚本：

```bat
scripts\dev_build_run.bat Release
```

该脚本会配置/构建 `LivoxViewerQT`，并启动主程序。当前本地脚本也会同步构建 `SlamPhase4Replay`，但 `scripts/*` 默认被 `.gitignore` 忽略；若需要团队共享脚本规则，应先调整 `.gitignore`。

手动 Windows 构建：

```powershell
cmake -S . -B build\cmd-msvc-Release -G "Visual Studio 17 2022" -A x64 -DCMAKE_PREFIX_PATH="S:\Qt\6.5.3\msvc2019_64"
cmake --build build\cmd-msvc-Release --config Release --target LivoxViewerQT SlamPhase4Replay --parallel
```

Linux 构建：

```bash
bash scripts/setup_third_party.sh
cmake -S . -B build/cmd-linux-Release -DCMAKE_BUILD_TYPE=Release
cmake --build build/cmd-linux-Release -j
./build/cmd-linux-Release/LivoxViewerQT
```

Linux 下可执行文件会优先从 `./livox_sdk_qt/lib` 查找 Livox SDK 动态库。

## 离线 SLAM 诊断

`SlamPhase4Replay` 支持两类用法：

```bat
build\cmd-msvc-Release\Release\SlamPhase4Replay.exe with_imu.pcap no_imu.pcap
build\cmd-msvc-Release\Release\SlamPhase4Replay.exe --diagnose --max-frames 300 path\to\data.bag
```

`--diagnose` 支持 `.pcap/.pcapng/.bag/.db3/.yaml/.yml`，会输出数据源摘要、帧/点/IMU 覆盖统计和 FAST_LIO 后端处理结果。`--max-frames N` 只限制后端处理帧数，数据源仍完整加载并统计。

## 支持的数据源

| 数据源 | 普通离线播放 | 离线 SLAM | 说明 |
|---|---|---|---|
| LVX2 | 支持 | 不作为 SLAM 输入 | 现有回放与转换链路 |
| PCAP/PCAPNG/CAP | 支持 | 支持 | SLAM 要求 IMU 和点内时间覆盖 |
| ROS1 `.bag` | 支持 | 支持 | 仅支持未压缩 ROS1 bag |
| ROS2 `.db3` / `metadata.yaml` | 支持 | 支持 | 仅支持 SQLite3 storage，不支持 MCAP |

ROSbag LiDAR topic 当前支持 Livox CustomMsg、Livox driver2 PointCloud2，以及允许合成点内时间的旧 driver PointCloud2。普通播放可无 IMU，SLAM 模式仍要求有效 IMU 覆盖。

## 开发文档

- [slam_dev_plan.md](slam_dev_plan.md)：SLAM 集成主线、编译规则、复验结果和当前风险记录
- [fastlio_migration_audit.md](fastlio_migration_audit.md)：FAST_LIO 原版流程与当前移植状态审计
- [rosbag_slam_source_development_plan.md](rosbag_slam_source_development_plan.md)：ROSbag 作为 SLAM 输入源的实现计划
- [rosbag_offline_playback_plan.md](rosbag_offline_playback_plan.md)：ROSbag 普通离线点云播放接入计划与实施记录

## 许可证

MIT License，详见 [LICENSE](LICENSE)。

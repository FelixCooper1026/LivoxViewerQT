# LivoxViewerQT

LivoxViewerQT 是一个基于 Qt 的 Livox 激光雷达设备控制与数据可视化软件。集成设备控制、数据回放、SLAM 后端与多种动态目标检测算法，支持实时/离线点云播放以及在线/离线 SLAM 复验。

## 功能概览

- 实时设备自动发现与连接
- 实时点云显示、着色、框选、测距和裁切
- 设备参数查询/控制命令下发、固件升级
- LOG / Debug 信息采集、LVX2 点云录制、PCD/LAS 点云导出、IMU 数据保存
- 点云数据格式转换，支持 LVX to PCD/LAS/CSV/TXT
- Livox PCAP 数据提取，支持提取点云、IMU、推送信息数据
- LVX、LVX2、PCAP/PCAPNG/CAP、ROS1 bag、ROS2 db3/metadata.yaml、MCAP 格式离线点云回放
- ROSbag 播放支持 ROS1 uncompressed/lz4 chunk、ROS1/ROS2 MCAP、Livox CustomMsg、Livox/通用 PointCloud2 和无 IMU 纯 XYZ 点云
- 剪枝移植后的 FAST_LIO 后端，支持在线 SLAM 与离线 SLAM
- SLAM 功能包含 LIO/LO 模式建图、轨迹、世界系/机体系点云显示、CSV/TUM 轨迹导出、PCD/LAS 全局地图导出
- 基于 M-detector/FreeDOM 的动态目标检测 demo
- 命令行离线 SLAM 诊断工具 `SlamReplayTool`

## 项目文件结构

```text
LivoxViewerQT/
  CMakeLists.txt                         主 CMake 工程
  README.md                              项目说明与构建指南
  apps/
    LivoxViewer/                         Qt 主程序
      actions/                           文件、播放、点云、IMU、SDK 和参数操作
      dialogs/                           格式转换及功能对话框
      helpers/                           主窗口辅助组件
      imu/                               IMU 可视化
      panels/                            工具栏、播放栏和信息面板
      slam/                              SLAM 界面、状态桥和窗口操作
      state/                             应用运行状态
      utils/                             应用层公共工具
      widgets/                           点云工作区等自定义控件
  libs/
    AppConfig/                           配置文件与网卡服务
    DynamicFilter/                       动态点过滤接口与控制器
    DynamicObject/                       M-detector 动态目标检测与聚类
    Export/                              PCD、LAS 等点云导出
    FreeDOM/                             FreeDOM 适配与上游实现
    LivoxCore/                           Livox SDK 封装、发现和参数服务
    Lvx/                                 LVX 读取与点云解析
    Lvx2/                                LVX2 读取、写入与播放数据源
    Pcap/                                PCAP、Livox 数据包和 IMU 解析
    Playback/                            离线播放公共接口
    PointCloud/                          点云结构、滤波、着色与 OpenGL 视图
    Rosbag/                              ROS1 bag、ROS2 db3、MCAP 读取与消息解析
    Slam/                                SLAM 数据源、同步、后端和地图导出
      third_party/fast_lio/              移植的 FAST_LIO/IKFoM/ikd-Tree 代码
      third_party/fast_lio_compat/       ROS/PCL 最小兼容头层
  plugins/
    StlModel/                            STL 模型加载插件
  icons/                                 应用图标
  pics/                                  界面图片资源
  resources/                             Qt 资源文件
  scripts/
    compile.bat / compile.sh             配置、编译并启动程序
    setup_third_party.ps1/.sh            第三方依赖安装脚本
    package_windows_release.bat          Windows 发布包生成脚本
    create_deb.sh / create_appimage.sh   Linux 发布包生成脚本
  third-party/                           安装脚本生成的本地依赖目录
  tools/
    SlamReplayTool/                      离线 SLAM 回归与诊断命令行工具
  livox_sdk_qt/                          项目内嵌 modified Livox SDK 头文件与库
```

## 构建依赖

- CMake `>= 3.16`
- C++17 编译器
- C 编译器，用于构建 LZ4 和 Zstd
- Qt `>= 6.2`，CMake 中保留 Qt 5.15 回退
- Qt 模块：Core、Widgets、OpenGL、OpenGLWidgets、SerialPort、Charts、Network、Svg、Concurrent、Sql
- Eigen 3.4.0 headers
- LZ4 1.10.0、MCAP C++ 2.1.3、Zstd 1.5.7
- OpenMP C++ runtime/compiler support
- Windows：MSVC、Npcap Runtime、Npcap SDK 1.16
- Linux：Qt 开发包、OpenMP、`libpcap-dev`、`pthread`、`dl`、`m`

无需预先安装系统 Boost 或完整 PCL。依赖安装脚本会下载 Boost 1.82.0 并编译 GTSAM 所需组件；PCL 接口由项目内 `fast_lio_compat` 最小兼容层提供，无需安装完整 PCL。

## 第三方依赖安装

首次构建前运行对应平台的安装脚本。依赖会安装到 `third-party/`，包括 Eigen、LZ4、MCAP C++、Zstd、OpenCV、Boost 和 GTSAM；Windows 还会安装 Npcap SDK。

Windows：

```powershell
pwsh -NoProfile -ExecutionPolicy Bypass -File .\scripts\setup_third_party.ps1
```

Linux/macOS：

```bash
bash scripts/setup_third_party.sh
```

强制重装全部第三方依赖：

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

## 离线 SLAM 诊断

`SlamReplayTool` 支持两类用法：

```bat
build\cmd-msvc-Release\Release\SlamReplayTool.exe with_imu.pcap no_imu.pcap
build\cmd-msvc-Release\Release\SlamReplayTool.exe --diagnose --max-frames 300 path\to\data.bag
build\cmd-msvc-Release\Release\SlamReplayTool.exe --diagnose --source-only --max-frames 10 path\to\pointcloud_only.bag
```

`--diagnose` 支持 `.pcap/.pcapng/.bag/.db3/.mcap/.yaml/.yml`，会输出数据源摘要、帧/点/IMU 覆盖统计和 FAST_LIO 后端处理结果。`--max-frames N` 只限制后端处理帧数，数据源仍完整加载并统计。`--source-only` 只加载数据源并输出帧统计，不启动 FAST_LIO，适合复验无 IMU 的普通点云文件播放。

## 支持的数据源

| 数据源 | 普通离线播放 | 离线 SLAM | 说明 |
|---|---|---|---|
| LVX/LVX2 | 支持 | 支持 | LVX/LVX2 文件仅包含点云，仅支持 LO 模式建图 |
| PCAP/PCAPNG/CAP | 支持 | 支持 | LIO 模式要求 IMU 和点内时间覆盖 |
| ROS1 `.bag` | 支持 | 支持 | 支持 uncompressed 和 lz4 chunk |
| ROS2 `.db3` / `metadata.yaml` | 支持 | 支持 | 支持 SQLite3 storage |
| `.mcap` / `metadata.yaml` | 支持 | 支持 | 支持 ROS1/ROS2 消息及 uncompressed、LZ4、Zstd |

ROSbag LiDAR topic 支持 Livox CustomMsg、Livox driver2 PointCloud2、允许合成点内时间的旧 driver PointCloud2、通用 `x/y/z/intensity/ring/time` PointCloud2，以及普通播放用的纯 `x/y/z` PointCloud2。离线播放可无 IMU；SLAM 模式要求有效 IMU 覆盖和合理点内时间。

## 许可证

本项目采用 GNU General Public License v2.0 only（GPL-2.0-only），详见 [LICENSE](LICENSE)。

## 致谢

感谢以下开源项目及其贡献者，本项目的开发从中受益：

- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [FAST_LIO](https://github.com/hku-mars/FAST_LIO)
- [M-detector](https://github.com/hku-mars/M-detector)
- [FreeDOM](https://github.com/LC-Robotics/FreeDOM)
- [FAST_LIO_SAM](https://github.com/kahowang/FAST_LIO_SAM)

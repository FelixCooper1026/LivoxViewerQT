# LivoxViewerQT

LivoxViewerQT 是一个基于 Qt/CMake 的 Livox 激光雷达可视化与控制工具，支持实时点云显示、参数管理、数据采集、LVX2/PCAP 离线回放和 LVX2 格式转换。

## 功能特性

- 实时发现设备并初始化 Livox SDK
- OpenGL 点云渲染与多种着色模式：
  - 反射率
  - 距离
  - 高程
  - 纯色
  - 平面投影
- 点云交互：
  - 旋转 / 平移 / 缩放
  - 预设视角
  - 框选与属性查看
  - 三维测距
- 设备与网络管理：
  - 网卡选择
  - 主机 IP 自动配置（可选）
  - 设备状态显示
- 参数管理：
  - 参数查询与下发（基础/网络/FOV/外参/状态）
  - 参数 CSV 记录
- 数据采集：
  - LOG / Debug 采集
  - LVX2 录制
  - PCD / LAS 导出
  - IMU CSV 保存
- 离线回放：
  - LVX2
  - PCAP / PCAPNG / CAP
- LVX2 转换：
  - PCD / LAS / CSV / TXT
- IMU 与 GPS：
  - IMU 数值与曲线显示
  - GPS 模拟
  - 串口 NMEA 转发

## 项目结构

```text
LivoxViewerQT/
  apps/LivoxViewer/                 应用层（UI、Actions、Panels、Dialogs、State）
  libs/AppConfig/                   配置与网卡服务
  libs/Export/                      点云导出（PCD/LAS/CSV/TXT）
  libs/LivoxCore/                   Livox SDK 封装、发现与参数服务
  libs/Lvx2/                        LVX2 读取与解析
  libs/Pcap/                        PCAP 解析与回放读取
  libs/Playback/                    统一回放数据源抽象
  libs/PointCloud/                  点云运行时与 OpenGL 视图
  livox_sdk_qt/                     Livox SDK 头文件与库
  third-party/npcap-sdk-1.16/       Npcap SDK（Windows 构建）
```

## 构建依赖

- CMake `>= 3.16`
- C++17 编译器
- Qt `>= 6.2`（CMake 中保留 Qt 5.15 回退）
- Qt 模块：
  - Core
  - Widgets
  - OpenGL
  - OpenGLWidgets（Qt6）
  - SerialPort
  - Charts
  - Network
  - Svg

### 平台附加依赖

- Windows：
  - MSVC 工具链
  - 已安装 Npcap 运行时
  - `third-party/npcap-sdk-1.16/Lib/x64/` 下可用 `wpcap.lib` / `Packet.lib`
- Linux：
  - `libpcap`
  - `pthread`、`dl`、`m`

## 编译

### Windows（PowerShell）

```powershell
cmake -S . -B build-msvc -A x64 -DCMAKE_PREFIX_PATH="C:\Qt\6.x.x\msvc2022_64"
cmake --build build-msvc --config Release --target LivoxViewerQT
```

如需打包 Qt 运行时：

```powershell
windeployqt build-msvc\Release\LivoxViewerQT.exe
```

### Linux

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/LivoxViewerQT
```

说明：

- Linux 下可执行文件会优先从 `./livox_sdk_qt/lib` 查找 Livox SDK 动态库。
- Linux 下主机网卡 IP 自动修改默认关闭。

## 使用流程

### 实时模式

1. 网线连接主机与 Livox 设备。
2. 启动 `LivoxViewerQT`。
3. 在设备面板选择网卡。
4. 等待设备发现与 SDK 初始化完成。
5. 在主视图进行点云观察、参数配置和采集操作。

### 离线回放

1. 从菜单或拖拽打开 LVX2 / PCAP / PCAPNG / CAP 文件。
2. 使用回放条控制播放、帧位置、速度与模式。
3. 在文件信息面板切换设备可见性。

### LVX2 转换

1. 打开格式转换对话框。
2. 选择源 LVX2 和输出参数。
3. 执行转换并检查输出文件。

## 仓库约定

- `docs/` 与 `testdata/` 为本地目录，默认不纳入版本控制。
- 如需团队共享文档或样例，请另建单独仓库或发布制品通道。

## 许可证

MIT License，详见 [LICENSE](LICENSE)。

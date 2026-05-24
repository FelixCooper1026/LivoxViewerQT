# LivoxViewerQT

LivoxViewerQT 是一个基于 Qt、CMake、Livox SDK2 和 Npcap/libpcap 的 Livox 激光雷达可视化与控制工具。项目支持实时雷达发现、点云显示、参数控制、数据采集、LVX2/PCAP 离线播放和 LVX2 格式转换。

## 功能概览

- 实时发现 Livox 雷达并初始化 Livox SDK2。
- OpenGL 点云显示，支持反射率、距离、高程、纯色、球面深度投影和平面投影。
- 点云交互：旋转、平移、缩放、视角预设、框选、属性表和三维测距。
- 设备管理：网卡选择、主机 IP 自动配置、设备列表和状态显示。
- 参数管理：基本参数、网络参数、FOV、外参、状态参数查询和写入。
- 数据采集：LOG、Debug、LVX2 录制、PCD/LAS 保存、IMU CSV 保存。
- 离线播放：LVX2、PCAP、PCAPNG、CAP。
- 格式转换：LVX2 转 PCD、LAS、CSV、TXT。
- IMU/GPS：IMU 数值和曲线显示、GPS 模拟、串口 NMEA 转发。

## 当前项目结构

```text
LivoxViewerQT/
  apps/LivoxViewer/                 Qt 应用层
    actions/                        主窗口业务动作
    dialogs/                        格式转换、固件升级、滤波等对话框
    panels/                         工具栏、回放条和 dock 面板
    state/                          应用层状态聚合
  libs/AppConfig/                   配置、网卡和应用设置服务
  libs/Export/                      PCD/LAS/CSV/TXT 导出
  libs/LivoxCore/                   Livox SDK、发现、参数服务和共享类型
  libs/Lvx2/                        LVX2 读取和点解析
  libs/Pcap/                        PCAP 离线读取和解析
  libs/Playback/                    LVX2/PCAP 统一播放抽象
  libs/PointCloud/                  点云模型、算法和 OpenGL 视图
  livox_sdk_qt/                     Livox SDK2 头文件和静态库
  third-party/npcap-sdk-1.16/       Npcap SDK
  testdata/manual/                  本地手工测试样例，真实样例不入库
  docs/                             架构、流程和重构文档
```

## 依赖

Windows 当前验证环境：

- Visual Studio 2026 Community
- Qt 6.8.3 `msvc2022_64`
- CMake，使用 Visual Studio 自带版本即可
- Npcap 运行时
- 仓库内的 `livox_sdk_qt/`
- 仓库内的 `third-party/npcap-sdk-1.16/Lib/x64/wpcap.lib`
- 仓库内的 `third-party/npcap-sdk-1.16/Lib/x64/Packet.lib`

Linux 需要：

- CMake 3.16+
- GCC/Clang
- Qt 5.15+ 或 Qt 6.2+，包含 Core、Widgets、OpenGL、SerialPort、Charts、Network、Svg
- libpcap
- pthread、dl、m

## Windows 编译运行

当前机器可直接使用以下命令：

```powershell
& "B:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B build-msvc -A x64 -DCMAKE_PREFIX_PATH="B:\Qt\6.8.3\msvc2022_64"
& "B:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-msvc --config Release --target LivoxViewerQT
& "B:\Qt\6.8.3\msvc2022_64\bin\windeployqt.exe" "B:\Workspace\LivoxViewerQT\build-msvc\Release\LivoxViewerQT.exe"
& "B:\Workspace\LivoxViewerQT\build-msvc\Release\LivoxViewerQT.exe"
```

如果 `cmake.exe` 已加入 PATH，可简化为：

```powershell
cmake -S . -B build-msvc -A x64 -DCMAKE_PREFIX_PATH="B:\Qt\6.8.3\msvc2022_64"
cmake --build build-msvc --config Release --target LivoxViewerQT
```

## Linux 编译运行

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/LivoxViewerQT
```

Linux 下程序通过 RPATH 优先从可执行文件旁的 `livox_sdk_qt/lib` 查找 Livox SDK 动态库。自动修改主机网卡 IP 默认关闭，如需使用需在界面中启用并确保具备权限。

## 手工 smoke 测试

真实离线样例放在：

```text
testdata/manual/sample.lvx2
testdata/manual/sample.pcap
```

这些文件被 `.gitignore` 忽略，不提交到仓库。

每次重构后建议检查：

```powershell
& "B:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-msvc --config Release --target LivoxViewerQT
& "B:\Qt\6.8.3\msvc2022_64\bin\windeployqt.exe" "B:\Workspace\LivoxViewerQT\build-msvc\Release\LivoxViewerQT.exe"
& "B:\Workspace\LivoxViewerQT\build-msvc\Release\LivoxViewerQT.exe"
```

影响离线播放或导出时，还应验证：

- LVX2 样例能加载并播放。
- PCAP 样例能加载并播放。
- 回放条首帧、上一帧、下一帧、尾帧、播放/暂停、速度和模式切换可用。
- 文件信息面板设备显示/隐藏可用。
- LVX2 转换输出 PCD、LAS、CSV、TXT，且文件非空。

## 使用流程

实时设备：

1. 使用网线连接主机和 Livox 雷达。
2. 启动 `LivoxViewerQT.exe`。
3. 在设备面板选择有线网卡。
4. 程序自动发现设备，必要时更新主机 IP 或配置文件。
5. SDK 初始化成功后开始接收点云、IMU、状态和参数。
6. 使用工具栏调整显示、投影、滤波、框选和测距。

离线播放：

1. 通过菜单或拖放打开 LVX2、PCAP、PCAPNG、CAP 文件。
2. 使用底部回放条控制帧、速度和播放模式。
3. 在文件信息面板控制设备点云显示/隐藏。

格式转换：

1. 打开格式转换对话框。
2. 选择 LVX2 源文件、输出目录、输出名、转换模式和目标格式。
3. 执行转换并检查输出文件。

## 文档

- [完整功能流程文档](docs/complete-feature-flow.md)
- [项目结构优化建议](docs/project-structure-optimization.md)
- [多功能耦合文件分析及重构建议](docs/multi-function-coupling-analysis.md)
- [Agent 指南](agent.md)

后续代码重构前应先阅读 `agent.md`。

## 许可证

本项目使用 MIT License，详见 [LICENSE](LICENSE)。

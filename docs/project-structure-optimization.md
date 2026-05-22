# LivoxViewerQT 项目结构优化建议

## 1. 当前结构判断

项目当前已经从单文件工程演进为 `apps` 与 `libs` 目录结构，但模块边界还没有真正闭合。主要问题是多个 `libs/*/src` 文件直接包含 `LivoxViewerWindow.h` 并实现主窗口成员函数，导致所谓库模块仍依赖应用层。

当前最重要的结构风险不是 CMake target 数量，而是 `LivoxViewerWindow` 同时承担 UI、状态、SDK、网络、配置、点云、录制、回放和导出职责。

## 2. 高耦合文件

### `apps/LivoxViewer/LivoxViewerWindow.h`

该头文件保存了主窗口绝大多数状态，包括：

- Qt 控件指针。
- SDK 初始化和设备列表状态。
- 网络发现状态。
- 点云帧缓存。
- 参数查询和配置控件映射。
- PCD、LAS、LVX2、IMU 保存状态。
- LVX2/PCAP 回放状态。
- IMU、GPS、串口线程状态。
- 点云滤波和选择状态。

优化方向：主窗口只保留子模块对象、关键信号连接和窗口生命周期。

### `apps/LivoxViewer/LivoxViewerUi.cpp`

该文件同时负责：

- 主界面布局。
- 工具栏和菜单。
- 多个 Dock。
- 配置生成对话框。
- 格式转换对话框。
- 保存 PCD/LAS/LVX2/IMU 对话框。
- 固件升级流程。
- 点云滤波对话框。
- 设备列表和参数标签更新。

优化方向：拆分为 panel、dialog、action 类，UI 只发信号，不直接承载业务流程。

### `libs/PointCloud/src/PointCloudPipeline.cpp`

该文件混合了：

- SDK 点云包解析。
- 滑动窗口合帧。
- 点云滤波、着色、投影。
- PCD/LAS 写出。
- LVX2 录制。
- IMU 显示、IMU CSV 保存。
- GPS 模拟和串口转发。
- 采集计时器。

优化方向：拆成点云解码、点云 pipeline、导出器、录制器、IMU/GPS 服务。

### `libs/LivoxCore/src/LidarSdkController.cpp`

该文件混合了：

- SDK 初始化和关闭。
- 配置文件查找、迁移、修正。
- 网卡扫描和选择。
- 自动修改主机 IP。
- UDP 广播发现。
- 雷达发现响应解析。

优化方向：拆成 SDK service、discovery service、config service、network service。

### `libs/LivoxCore/src/LidarParameterParser.cpp`

该文件混合了：

- 参数格式化。
- 参数控件同步。
- 参数写入。
- 参数响应处理。
- 参数 CSV 记录。

优化方向：拆成参数模型、参数 parser、参数 command service、参数记录器。

### `libs/Lvx2/src/Lvx2PlaybackController.cpp`

该文件混合了：

- LVX2 文件结构读取。
- package 点云解析。
- 外参应用。
- 帧缓存。
- 播放状态。
- 播放 UI 更新。

优化方向：拆成 `Lvx2Reader` 和通用 `PlaybackController`。

### `libs/Lvx2/src/Lvx2Converter.cpp`

该文件与 LVX2 回放重复实现点解析和外参应用，同时又包含多种输出格式 writer。

优化方向：复用 `Lvx2Reader` 和 `Export` writer。

## 3. 推荐目标结构

```text
apps/LivoxViewer/
  MainWindow.*
  actions/
  panels/
  dialogs/

libs/AppConfig/
  AppSettings.*
  ConfigJsonService.*
  NetworkInterfaceService.*

libs/LivoxCore/
  LidarSdkService.*
  LidarDiscoveryService.*
  LidarConfigService.*
  LidarParameterService.*
  LidarParameterModel.*

libs/PointCloud/
  PointCloudTypes.*
  PointCloudFrame.*
  PointCloudDecoder.*
  PointCloudPipeline.*
  PointCloudColorizer.*
  PointCloudFilter.*
  PointCloudProjection.*
  PointCloudView.*

libs/Recording/
  Lvx2Recorder.*
  ImuCsvRecorder.*
  CaptureSession.*

libs/Playback/
  PlaybackSource.*
  PlaybackController.*
  Lvx2Reader.*
  PcapReader.*
  PlaybackDeviceModel.*

libs/Export/
  PcdWriter.*
  LasWriter.*
  CsvPointWriter.*
  TxtPointWriter.*
```

## 4. 分阶段重构路线

### Stage 0：文档与测试样例约定

- 新增完整功能流程文档。
- 新增项目结构优化建议。
- 忽略 `testdata/manual/*`，避免真实 LVX2/PCAP 样例进入仓库。
- 保留 `testdata/manual/README.md` 说明手动测试样例命名。

### Stage 1：建立真实模块边界

- 修正 `PointCloudView.cpp` 对 `LivoxViewerWindow.h` 的直接依赖。
- 补齐空壳头文件中的接口声明。
- 不移动业务逻辑，不改变行为。

### Stage 2：拆出点云导出与 LVX2 公共解析

- 新增 `libs/Export`。
- 集中实现 PCD、LAS、CSV、TXT 写出。
- LVX2 播放和转换共用点解析与外参 helper。
- 保持文件格式和 UI 行为不变。

### Stage 3：统一离线播放来源

- 新增 `PlaybackSource` 概念。
- `Lvx2Reader` 负责 LVX2。
- `PcapReader` 负责 PCAP。
- 播放控制器只处理帧数、当前帧、速度、模式、设备可见性。

### Stage 4：拆实时点云处理 pipeline

- `PointCloudDecoder` 只负责原始 SDK 包到 `PointCloudFrame`。
- `PointCloudColorizer` 负责着色。
- `PointCloudFilter` 负责 tag 滤波。
- `PointCloudProjection` 负责球面/平面投影。
- `onRenderTick()` 保留调度语义，内部调用独立模块。

### Stage 5：拆 Livox SDK、发现、配置、参数服务

- `LidarSdkService` 管理 SDK 生命周期和回调注册。
- `LidarDiscoveryService` 管理 UDP 发现。
- `LidarConfigService` 管理 `config.json` 查找、迁移、更新。
- `LidarParameterService` 管理参数查询、写入、格式化和记录。

### Stage 6：拆主窗口 UI

- 主窗口拆出工具栏、设备面板、参数面板、回放条、文件信息面板、IMU 面板、日志面板。
- 对话框和菜单 action 独立成文件。
- `LivoxViewerWindow` 只做模块组合和生命周期管理。

## 5. 重构约束

- 保持用户可见行为不变。
- 每阶段只改一个方向。
- 每阶段必须 Release 编译通过。
- 每阶段启动 GUI，确认不立即崩溃。
- 有真实样例时验证 LVX2 与 PCAP 离线回放。
- 不添加隐藏错误的兜底逻辑；失败路径必须显式报错或保持现有错误提示。
- 不提交 `build-msvc/`、`testdata/manual/sample.lvx2`、`testdata/manual/sample.pcap` 和 smoke 输出。

## 6. 阶段验收标准

- `git status` 只包含本阶段预期变更。
- Release 构建成功。
- `windeployqt` 成功完成。
- `LivoxViewerQT.exe` 能启动并保持运行。
- LVX2 样例能加载并显示第一帧。
- PCAP 样例能加载并显示第一帧。
- LVX2 转换到 PCD、LAS、CSV、TXT 后生成非空文件。

# LivoxViewerQT 项目结构优化建议

更新时间：2026-05-23
当前基线提交：`bd75939 refactor: split main window panels and menu actions`

## 1. 当前状态概览

项目已经从早期的主窗口大文件逐步拆出 `apps/` 与 `libs/` 目录，并完成了 Stage 0 到 Stage 6 的第一轮保守重构提交。当前结构比初始状态清晰很多：

- `apps/LivoxViewer/` 已拆出工具栏、回放条、设备面板、参数面板、文件信息面板、IMU 面板、日志面板和菜单动作文件。
- `libs/Export/` 已集中点云导出能力。
- `libs/Lvx2/` 已有 `Lvx2Reader`、`Lvx2PointParser`、`Lvx2Converter`。
- `libs/Pcap/` 已有 `PcapReader` 和 PCAP 解析相关模块。
- `libs/Playback/` 已有统一 `PlaybackSource` 抽象。
- `libs/PointCloud/` 已有 `PointCloudDecoder`、`PointCloudColorizer`、`PointCloudFilter`、`PointCloudProjection` 等处理模块。
- `libs/AppConfig/` 与 `libs/LivoxCore/` 已拆出配置、发现、SDK、参数服务的部分实现。

但需要明确：当前阶段主要完成了“物理拆文件”和“公共能力抽取”，还没有完全完成“业务边界闭合”。多个 `libs/*/src` 文件仍然直接包含 `LivoxViewerWindow.h`，并实现 `LivoxViewerWindow::` 成员函数。这意味着这些文件虽然位于 `libs` 目录下，职责上仍然属于应用层主窗口的一部分。

## 2. 阶段完成情况

| 阶段 | 状态 | 对应提交 | 当前结论 |
| --- | --- | --- | --- |
| Stage 0：文档与测试样例约定 | 已完成 | `a08c0c6` | 已新增功能流程文档和结构优化文档；`.gitignore` 已忽略 `testdata/manual/*` 并保留 README。 |
| Stage 1：建立真实模块边界 | 已完成 | `1c42da5` | 已补齐关键头文件接口并减少部分隐式依赖。 |
| Stage 2：点云导出与 LVX2 公共解析 | 已完成 | `91ce482` | 已新增 `libs/Export` 与 LVX2 点解析 helper，导出逻辑开始集中。 |
| Stage 3：统一离线播放来源 | 已完成 | `4e23419` | 已新增 `PlaybackSource`、`Lvx2Reader`、`PcapReader`，LVX2/PCAP 可使用统一播放来源模型。 |
| Stage 4：拆实时点云 pipeline | 已完成 | `28e2121` | 已新增 decoder、colorizer、filter、projection 模块；实时点云处理主路径已拆出核心处理能力。 |
| Stage 5：拆 SDK、发现、配置、参数服务 | 部分完成 | `f7986ba` | 已有服务类，但 `LidarSdkController.cpp`、`LidarSdkCallbacks.cpp`、`LidarParameterParser.cpp` 仍实现大量主窗口成员函数。 |
| Stage 6：拆主窗口 UI | 部分完成 | `415669e`、`bd75939` | 已拆出 toolbar、playback bar、五个 dock 面板、菜单动作文件、三个独立 dialog 文件，以及帮助、采集/保存、设备动作文件；回放连接和状态栏/定时器仍留在 `MainMenuActions.cpp`。 |

## 3. 已完成项

### 3.1 文档、测试样例和仓库规则

已完成：

- `docs/complete-feature-flow.md`
- `docs/project-structure-optimization.md`
- `testdata/manual/*` 忽略规则
- `testdata/manual/README.md` 保留规则
- Npcap SDK `Lib` 目录忽略规则修复

当前离线 smoke 样例仍按约定放在：

- `testdata/manual/sample.lvx2`
- `testdata/manual/sample.pcap`

这些真实样例不进入仓库。

### 3.2 UI 文件第一轮拆分

已从 `initializeUserInterface()` 中拆出：

- `apps/LivoxViewer/ViewerToolbar.cpp`
- `apps/LivoxViewer/PlaybackBar.cpp`
- `apps/LivoxViewer/DevicePanel.cpp`
- `apps/LivoxViewer/ParameterPanel.cpp`
- `apps/LivoxViewer/FileInfoPanel.cpp`
- `apps/LivoxViewer/ImuPanel.cpp`
- `apps/LivoxViewer/LogPanel.cpp`
- `apps/LivoxViewer/MainMenuActions.cpp`
- `apps/LivoxViewer/FormatConvertDialog.cpp`
- `apps/LivoxViewer/FirmwareUpgradeDialog.cpp`
- `apps/LivoxViewer/PointCloudFilterDialog.cpp`
- `apps/LivoxViewer/HelpActions.cpp`
- `apps/LivoxViewer/CaptureActions.cpp`
- `apps/LivoxViewer/DeviceActions.cpp`

当前 `initializeUserInterface()` 的职责已经收敛为：

- 设置应用字体
- 创建中央点云视图
- 创建工具栏和回放条
- 初始化投影控件状态
- 调用各 panel 创建函数
- 设置 dock 初始尺寸
- 调用菜单和 action 创建函数

这一步没有改变控件文案、信号连接或业务流程。

### 3.3 点云处理模块

已新增或完善：

- `PointCloudDecoder`
- `PointCloudColorizer`
- `PointCloudFilter`
- `PointCloudProjection`
- `PointCloudFrame`
- `PointCloudTypes`

当前实时点云路径已经具备更清晰的处理模块，但 `PointCloudPipeline.cpp` 仍然承载不少应用层成员函数。

### 3.4 离线播放和导出模块

已新增或完善：

- `PlaybackSource`
- `Lvx2Reader`
- `PcapReader`
- `Lvx2PointParser`
- `PointCloudExport`

LVX2/PCAP 离线 smoke 已覆盖：

- LVX2 加载
- PCAP 加载
- 设备数量解析
- 点数解析
- PCD/LAS/CSV/TXT 非空导出

### 3.5 Livox SDK、发现、配置、参数服务

已新增或完善：

- `LidarSdkService`
- `LidarDiscoveryService`
- `LidarParameterService`
- `LidarConfigService`
- `ConfigJsonService`
- `NetworkInterfaceService`
- `AppSettings`

这些服务为后续从主窗口剥离 SDK、网络、配置和参数业务奠定了基础。

## 4. 未完全完成项

### 4.1 `libs` 目录中仍有应用层主窗口实现

当前仍有多个 `libs/*/src` 文件直接包含 `LivoxViewerWindow.h`，并实现 `LivoxViewerWindow::` 成员函数：

- `libs/LivoxCore/src/LidarSdkController.cpp`
- `libs/LivoxCore/src/LidarSdkCallbacks.cpp`
- `libs/LivoxCore/src/LidarParameterParser.cpp`
- `libs/Lvx2/src/Lvx2PlaybackController.cpp`
- `libs/Lvx2/src/Lvx2Converter.cpp`
- `libs/Pcap/src/PcapPlaybackController.cpp`
- `libs/PointCloud/src/PointCloudPipeline.cpp`

这仍然是当前最大的结构问题。下一步应把这些文件中真正属于应用层的主窗口槽函数迁回 `apps/LivoxViewer/` 或拆成应用层 controller/action 文件；`libs` 只保留纯服务、解析、模型和算法。

### 4.2 `MainMenuActions.cpp` 仍然过大

`apps/LivoxViewer/MainMenuActions.cpp` 当前已从约 1179 行降到约 211 行，仍集中包含：

- 文件菜单
- 工具菜单
- 状态栏、计时器和部分 action 连接
- LVX2/PCAP 打开动作
- 离线回放控件连接
- IMU 绘图和点云滤波 action 入口

已拆出的独立 dialog 文件：

- `apps/LivoxViewer/FormatConvertDialog.cpp`
- `apps/LivoxViewer/FirmwareUpgradeDialog.cpp`
- `apps/LivoxViewer/PointCloudFilterDialog.cpp`

已拆出的独立 action 文件：

- `apps/LivoxViewer/HelpActions.cpp`
- `apps/LivoxViewer/CaptureActions.cpp`
- `apps/LivoxViewer/DeviceActions.cpp`

剩余目标是继续拆出文件打开、回放连接、状态栏和定时器初始化，让 `MainMenuActions.cpp` 只保留菜单骨架。

### 4.3 `ParameterPanel.cpp` 仍然偏重

`apps/LivoxViewer/ParameterPanel.cpp` 当前约 488 行，仍集中创建多个参数页、参数控件映射和部分控件连接。后续可以继续拆为：

- `BasicParameterPanel`
- `NetworkParameterPanel`
- `FovParameterPanel`
- `StatusParameterPanel`
- `ParameterRecordControls`

这类拆分应仍保持保守，仅移动 UI 创建代码，不改变参数 key、控件文案和连接。

### 4.4 `LivoxViewerWindow.h` 仍然是中心状态仓库

`apps/LivoxViewer/LivoxViewerWindow.h` 当前仍保存大量状态：

- UI 控件指针
- SDK 状态
- 设备列表
- 参数查询和参数记录状态
- 点云窗口、滤波、投影状态
- LVX2/PCAP 回放状态
- IMU/GPS/串口状态
- 采集、保存、录制状态

后续应逐步把状态移动到明确的 controller/service/panel 对象中。主窗口最终只保留模块对象、信号转发和生命周期管理。

### 4.5 `LidarSdkController.cpp` 仍然过大

`libs/LivoxCore/src/LidarSdkController.cpp` 当前约 1208 行，仍包含 SDK 初始化、发现控制、主机 IP 更新、配置文件更新、设备状态处理等主窗口成员函数。

虽然 Stage 5 已新增服务类，但旧 controller 仍承担大量协调逻辑。后续应把它拆为：

- 应用层 `LidarController` 或 `DeviceController`
- 纯 SDK 服务
- 纯发现服务
- 配置服务
- 网络接口服务

### 4.6 `PointCloudPipeline.cpp` 仍然混合实时、录制、IMU/GPS 和 UI 槽函数

`libs/PointCloud/src/PointCloudPipeline.cpp` 当前约 1177 行，仍包含：

- 采集计时
- GPS 模拟
- 串口转发
- IMU 图表和 IMU 保存
- 渲染 tick 调度
- 点云选择和测量 UI 响应
- LVX2 录制入口

Stage 4 已拆出点云处理算法，但该文件仍不是纯 point cloud library。后续应把应用层槽函数迁出，并进一步拆出 recording、IMU/GPS 和 selection controller。

## 5. 推荐目标结构

当前仍建议向下面结构收敛：

```text
apps/LivoxViewer/
  LivoxViewerWindow.*
  ViewerToolbar.*
  PlaybackBar.*
  panels/
    DevicePanel.*
    ParameterPanel.*
    FileInfoPanel.*
    ImuPanel.*
    LogPanel.*
  actions/
    MainMenuActions.*
    CaptureActions.*
    DeviceActions.*
    HelpActions.*
    PlaybackActions.*
  dialogs/
    ConfigGeneratorDialog.*
    FormatConvertDialog.*
    FirmwareUpgradeDialog.*
    PointCloudFilterDialog.*
    PreferencesDialog.*

libs/AppConfig/
  AppSettings.*
  ConfigJsonService.*
  LidarConfigService.*
  NetworkInterfaceService.*

libs/LivoxCore/
  LidarSdkService.*
  LidarDiscoveryService.*
  LidarParameterService.*
  LidarParameterModel.*
  LidarPacketUtils.*

libs/PointCloud/
  PointCloudTypes.*
  PointCloudFrame.*
  PointCloudDecoder.*
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

说明：当前 CMake 仍保持单一 `LivoxViewerQT` target，这是合理的。等 `libs` 不再实现主窗口成员函数后，再考虑拆静态库 target。

## 6. 下一阶段建议

### Priority 1：完成 Stage 6 的 action/dialog 二次拆分

目标：降低 `MainMenuActions.cpp` 复杂度，不改行为。

建议顺序：

1. 已完成：把格式转换对话框拆到 `FormatConvertDialog.cpp`。
2. 已完成：把固件升级对话框拆到 `FirmwareUpgradeDialog.cpp`。
3. 已完成：把点云滤波对话框拆到 `PointCloudFilterDialog.cpp`。
4. 已完成：把帮助菜单链接拆到 `HelpActions.cpp`。
5. 已完成：把采集/保存菜单动作拆到 `CaptureActions.cpp`。
6. 已完成：把设备菜单动作拆到 `DeviceActions.cpp`。
7. 待完成：把 LVX2/PCAP 文件打开和回放连接拆到 `PlaybackActions.cpp`。

验收标准：

- `MainMenuActions.cpp` 只负责创建菜单骨架和连接 action。
- 每个 dialog/action 文件只承载一个功能方向。
- 菜单文案、默认值、保存路径、错误提示和信号连接保持不变。

### Priority 2：把应用层主窗口成员实现移出 `libs`

目标：让 `libs` 目录真正成为库层。

建议顺序：

1. 新增 `apps/LivoxViewer/controllers/` 或先使用平铺文件。
2. 将 `Lvx2PlaybackController.cpp`、`PcapPlaybackController.cpp` 中的 `LivoxViewerWindow::` 实现移到应用层。
3. 将 `Lvx2Converter.cpp` 中的主窗口 wrapper 移到应用层，库层只保留转换器。
4. 将 `PointCloudPipeline.cpp` 中 UI 槽函数迁移到应用层。
5. 将 `LidarSdkController.cpp`、`LidarSdkCallbacks.cpp`、`LidarParameterParser.cpp` 的主窗口实现逐步迁移。

验收标准：

- `libs/**` 中不再包含 `LivoxViewerWindow.h`。
- `libs/**` 中不再出现 `LivoxViewerWindow::`。
- 主窗口通过 service/controller 对象调用库能力。

### Priority 3：继续拆 `LivoxViewerWindow.h` 状态

目标：减少主窗口头文件的成员变量和 include。

建议顺序：

1. 将 playback 状态封装为 `PlaybackControllerState` 或应用层 controller。
2. 将 capture/record 状态封装为 `CaptureSession`。
3. 将 IMU/GPS/serial 状态封装为独立对象。
4. 将参数控件映射和参数记录状态放入参数面板或参数 controller。
5. 将滤波 dialog 相关状态放入 `PointCloudFilterDialog`。

验收标准：

- `LivoxViewerWindow.h` 只保留跨模块必需的对象指针和少量生命周期状态。
- 大部分 Qt 控件指针由对应 panel/dialog 持有。

### Priority 4：整理源码目录和 CMake

目标：在边界稳定后再做目录移动，避免过早增加 churn。

建议顺序：

1. 将 `apps/LivoxViewer/*Panel.cpp` 移入 `apps/LivoxViewer/panels/`。
2. 将 action 文件移入 `apps/LivoxViewer/actions/`。
3. 将 dialog 文件移入 `apps/LivoxViewer/dialogs/`。
4. 更新 CMake source list。
5. 等 `libs` 完全脱离主窗口后，再考虑拆多个静态库 target。

## 7. 重构约束

- 保持用户可见行为不变。
- 每个阶段只处理一个方向。
- 优先移动代码和建立边界，再做行为性重写。
- 不添加隐藏错误的兜底逻辑。
- 解析失败、文件缺失、SDK 调用失败应显式返回错误或保持现有错误提示。
- 每阶段必须 Release 编译通过。
- 每阶段必须执行 GUI 启动 smoke。
- 有真实样例时继续执行 LVX2/PCAP 离线 smoke。
- 不提交 `build-msvc/`、`testdata/manual/sample.lvx2`、`testdata/manual/sample.pcap` 和 smoke 输出。

## 8. 阶段验收标准

每个后续阶段完成后固定检查：

```powershell
cmake --build build-msvc --config Release
B:\Qt\6.8.3\msvc2022_64\bin\windeployqt.exe B:\Workspace\LivoxViewerQT\build-msvc\Release\LivoxViewerQT.exe
B:\Workspace\LivoxViewerQT\build-msvc\Release\LivoxViewerQT.exe
```

离线 smoke：

```powershell
cmake --build build-msvc\smoke-build --config Release
B:\Workspace\LivoxViewerQT\build-msvc\smoke-build\Release\lvx2_smoke.exe
```

文件检查：

```powershell
git status --short
git diff --check
Get-ChildItem build-msvc\smoke-output -File | Select-Object Name,Length
```

验收结果应至少覆盖：

- Release 构建成功。
- `windeployqt` 成功完成。
- GUI 启动后不立即崩溃。
- LVX2 样例可读取。
- PCAP 样例可读取。
- PCD/LAS/CSV/TXT 输出文件非空。
- 本阶段 commit 只包含本阶段相关源码或文档变更。

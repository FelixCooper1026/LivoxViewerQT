# LivoxViewerQT Agent 指南

本文件是后续自动化代理或人工协作进行代码更改时必须先阅读的项目规则。任何重构、修复、构建或提交前，都应以本文件作为边界约束。

## 1. 通用重构边界

- 优先保持现有用户可见行为不变。
- 每次重构只处理一个明确功能方向，不把无关清理混入同一阶段。
- 先移动代码和收敛边界，再做行为性改写。
- 不添加掩盖错误的兜底逻辑。
- 解析失败、文件缺失、SDK 调用失败、PCAP/LVX2 读取失败必须显式返回失败或保持现有错误提示。
- 不改变菜单文案、按钮文案、默认值、保存路径策略、文件扩展名和导出字段，除非任务明确要求。
- 不改变 LVX2、PCAP、PCD、LAS、CSV、TXT 的现有格式语义。
- 不改变参数 key、参数查询周期、参数写入 payload 和参数 CSV 字段顺序。
- 不改变点云滤波 tag 组合逻辑、着色模式、投影模式和默认显示状态。
- 不回退用户或其他代理已有的未提交改动。

## 2. 当前推荐模块边界

应用层：

- `apps/LivoxViewer/LivoxViewerWindow.h`：只应保留主窗口生命周期、模块组合、少量共享状态和 Qt slot 声明。后续应继续缩小。
- `apps/LivoxViewer/actions/`：当前承载主窗口动作。后续可继续按功能拆成更小文件，最终迁移到 controller 对象。
- `apps/LivoxViewer/panels/`：只负责创建和维护对应 dock/panel UI。
- `apps/LivoxViewer/dialogs/`：只负责具体对话框交互。
- `apps/LivoxViewer/state/`：保存应用层状态聚合结构。新增 state 必须有明确生命周期和所有权。

库层：

- `libs/AppConfig/`：配置文件、应用设置、网卡服务。
- `libs/Export/`：点云导出。
- `libs/LivoxCore/`：Livox SDK、发现、参数服务和基础类型。
- `libs/Lvx2/`：LVX2 文件读取、索引、点解析。
- `libs/Pcap/`：PCAP 离线读取、UDP/Push/点云解析。
- `libs/Playback/`：统一离线播放抽象。
- `libs/PointCloud/`：点云模型、解码、滤波、着色、投影和 OpenGL 视图。

库层不得依赖 `apps/LivoxViewer`，不得包含 `LivoxViewerWindow.h`，不得实现 `LivoxViewerWindow::` 成员函数。

## 3. 代码规范

- 使用 C++17。
- 保持 Qt 现有风格：QObject 所有权、signal/slot、`QMetaObject::invokeMethod` 跨线程回主线程。
- SDK 回调内必须深拷贝需要异步使用的数据。
- 涉及共享状态时保留现有互斥锁或原子变量语义。
- 新增文件要同步加入 `CMakeLists.txt` 的 `SOURCES` 或 `HEADERS`。
- 优先使用现有 service、state、helper，不引入重复解析或重复导出逻辑。
- 新增注释只解释不直观的边界或复杂逻辑，不写显而易见的注释。
- 不在源代码中硬编码新的本机私有路径；README 和文档中的本机编译示例可以保留当前机器路径。
- Windows 路径写在文档中时使用 PowerShell 可直接执行的引用方式。

## 4. 编译和运行方法

当前 Windows 环境：

- Visual Studio 2026 Community：`B:\Program Files\Microsoft Visual Studio\18\Community`
- Qt 6.8.3：`B:\Qt\6.8.3\msvc2022_64`
- CMake：`B:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe`

首次配置或 CMakeLists 改动后执行：

```powershell
& "B:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B build-msvc -A x64 -DCMAKE_PREFIX_PATH="B:\Qt\6.8.3\msvc2022_64"
```

常规 Release 编译：

```powershell
& "B:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-msvc --config Release --target LivoxViewerQT
```

Qt 部署：

```powershell
& "B:\Qt\6.8.3\msvc2022_64\bin\windeployqt.exe" "B:\Workspace\LivoxViewerQT\build-msvc\Release\LivoxViewerQT.exe"
```

GUI smoke：

```powershell
& "B:\Workspace\LivoxViewerQT\build-msvc\Release\LivoxViewerQT.exe"
```

可用脚本方式验证进程能启动且不立即退出：

```powershell
$exe = "B:\Workspace\LivoxViewerQT\build-msvc\Release\LivoxViewerQT.exe"
$p = Start-Process -FilePath $exe -PassThru
Start-Sleep -Seconds 3
if ($p.HasExited) { "EXITED:$($p.ExitCode)" } else { "RUNNING:$($p.Id)"; Stop-Process -Id $p.Id -Force }
```

## 5. 离线样例和 smoke

真实样例位置：

- `testdata/manual/sample.lvx2`
- `testdata/manual/sample.pcap`

这些文件被 `.gitignore` 忽略，不得提交。

重构影响 LVX2、PCAP、播放、点云解析或导出时，应验证：

- LVX2 能加载。
- PCAP 能加载。
- 播放条首帧、上一帧、下一帧、尾帧、播放/暂停、速度和模式切换可用。
- 文件信息面板设备显示/隐藏可用。
- LVX2 转换输出 PCD、LAS、CSV、TXT，且文件非空。

## 6. 每次重构后需要更新的文件

如果改变了模块职责、文件位置、构建方式、测试方式或功能流程，必须同步检查并更新：

- `docs/complete-feature-flow.md`
- `docs/multi-function-coupling-analysis.md`
- `docs/project-structure-optimization.md`
- `README.md`
- `agent.md`

如果只做局部 bug 修复，不改变结构或流程，可以不更新上述文档，但最终回复中要说明验证结果。

## 7. 提交前检查

提交前固定执行：

```powershell
git status --short
git diff --check
```

推荐验证：

```powershell
& "B:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-msvc --config Release --target LivoxViewerQT
```

提交内容应只包含当前阶段相关文件。不要提交：

- `build-msvc/`
- `testdata/manual/sample.lvx2`
- `testdata/manual/sample.pcap`
- smoke 输出文件
- 本机临时日志

## 8. 当前高优先级重构方向

1. 缩小 `LivoxViewerWindow.h`。
2. 拆分 `PointCloudActions.cpp` 的 IMU/GPS/串口、选择/测距、录制/保存职责。
3. 拆分 `LidarSdkActions.cpp` 的 SDK 生命周期、发现、主机网络配置和配置同步职责。
4. 拆分 `LidarParameterActions.cpp` 的参数写入、格式化和记录职责。
5. 将 `ParameterPanel.cpp` 拆成更小的参数页构建模块。
6. 在边界稳定后再考虑拆多个 CMake 静态库 target。

# LivoxViewerQT 完整功能流程文档

## 1. 项目定位

LivoxViewerQT 是一个基于 Qt、CMake 和 Livox SDK2 的激光雷达可视化与控制工具。项目当前构建为单一可执行程序 `LivoxViewerQT`，入口位于 `apps/LivoxViewer/main.cpp`，主窗口核心类为 `LivoxViewerWindow`。

程序面向 Livox 雷达的本机调试、实时点云显示、参数控制、数据采集、LVX2/PCAP 离线回放和格式转换。

## 2. 启动流程

1. `main()` 在 Windows 下调用 `SetDllDirectoryA("C:\\Windows\\System32\\Npcap")`，让 PCAP 运行时优先从 Npcap 安装目录加载。
2. 创建 `QApplication`，设置应用名、版本、组织名和窗口图标。
3. 创建并显示 `LivoxViewerWindow`。
4. `LivoxViewerWindow` 构造阶段初始化 UI、读取视图偏好、刷新网卡与串口、启动雷达发现流程、启动参数轮询定时器。
5. Qt 事件循环接管菜单、按钮、定时器、SDK 回调转发和 OpenGL 渲染刷新。

## 3. 主界面流程

主界面由 `apps/LivoxViewer/LivoxViewerUi.cpp` 构建，核心区域包括：

- 中央点云视图：`PointCloudView` 基于 `QOpenGLWidget` 渲染点云、坐标轴、网格、图例、框选区域和测距结果。
- 视图工具栏：控制积分时间、点大小、着色模式、球面投影、平面投影、可视化暂停、框选、测距、视角预设和重置视图。
- 设备面板：网卡选择、自动修改主机 IP、设备列表、GPS 模拟、串口 GPRMC 转发。
- 参数面板：基本参数、网络参数、FOV 参数、外参参数、状态参数和参数 CSV 记录。
- 文件信息面板：显示 LVX2/PCAP 离线回放中的设备，支持按设备隐藏/显示点云。
- IMU 面板：显示最新 IMU 数值、ASCII 状态和曲线窗口。
- 日志面板：显示运行日志并支持清空。
- 菜单栏：提供配置文件生成、LVX2 播放、PCAP 播放、格式转换、数据采集、点云保存、IMU 保存、固件升级、重启、恢复出厂设置和帮助链接。

## 4. 网络与设备发现流程

1. 程序刷新本机网络接口，过滤不可用、回环、虚拟和无效 IPv4 接口。
2. 用户可在设备面板中选择用于雷达通信的网卡。
3. `startLidarDiscovery()` 创建 UDP socket，默认监听 56000 端口。
4. Windows 下优先绑定到选中的主机 IPv4；Linux 下优先绑定 `AnyIPv4` 并尝试物理绑定到指定网卡。
5. 程序周期性向广播地址发送 Livox 发现命令。
6. 收到雷达响应后解析设备 IP、类型、序列号等信息。
7. 如果设备与主机不在兼容网段，程序根据设置尝试自动配置主机 IP，或提示用户手动调整。
8. 发现到的设备信息进入 `lidarDevices`，设备列表刷新，并选择当前设备。

## 5. 配置文件与 SDK 初始化流程

1. 初始化 SDK 前先解析标准配置目录中的 `config.json`；若不存在，则检查兼容旧路径。
2. 未找到配置文件时打开配置生成向导。
3. 找到配置文件后检查所有设备块中的 `host_net_info[*].host_ip` 是否与当前所选主机 IP 一致。
4. 不一致时自动更新配置文件中的 `host_ip`。
5. 如果通过发现流程获得了设备类型，程序会尝试修正配置文件中的设备类型。
6. 调用 `LivoxLidarSdkInit(configPath)` 初始化 Livox SDK。
7. 注册设备信息、点云、IMU 和状态信息回调。
8. 初始化成功后状态栏切换为连接/采样状态。
9. 关闭时先注销 SDK 回调，再调用 `LivoxLidarSdkUninit()`，避免析构期间回调访问已销毁状态。

## 6. 设备参数流程

1. 设备连接后查询内部参数。
2. SDK 参数响应回调中深拷贝数据，回到主线程解析。
3. 参数按 key、length、value 逐项解析。
4. 状态参数直接更新 label。
5. 可配置参数只在首次或必要时同步到控件，避免覆盖用户正在编辑的值。
6. 用户修改控件后进入 `onParamConfigChanged()`，按参数类型调用对应的 SDK 设置接口。
7. 网络、FOV、外参等复杂参数有独立 apply 方法。
8. 参数记录功能会在 CSV 文件中记录参数表头和周期性参数值。

## 7. 实时点云流程

1. Livox SDK 点云回调接收 `LivoxLidarEthernetPacket`。
2. 回调检查点数、数据类型和长度，深拷贝数据包。
3. 数据包通过 `QMetaObject::invokeMethod` 交回主线程。
4. `decodePointCloudPacket()` 按数据类型解析点：
   - 笛卡尔高精度点。
   - 笛卡尔低精度点。
   - 球坐标点。
   - 双回波点。
5. 每个点转换为 `PointCloudPoint`，包含坐标、颜色、反射率和 tag。
6. 点云帧按设备 handle 放入 `pendingFrames`。
7. `onRenderTick()` 根据当前设备和 `frameIntervalMs` 计算滑动窗口。
8. 窗口内帧被合并为一个 `PointCloudFrame`。
9. 合并帧进入点云处理流程：滤波、着色、投影和图例更新。
10. 处理后的帧通过 `presentPointCloudFrame()` 交给 `PointCloudView` 渲染。

## 8. 点云显示与交互流程

`PointCloudView` 负责 OpenGL 视图层：

- 初始化 shader、点云缓冲、坐标轴缓冲和网格缓冲。
- 支持鼠标旋转、平移、滚轮缩放。
- 支持双击点云设置视图目标点。
- 支持世界、前、后、左、右、俯视等视角预设。
- 支持矩形框选，并把屏幕区域转换为点云选择结果。
- 支持 AABB 持久选择。
- 支持测距模式，用户选择两个点后计算三维距离。
- 支持拖放 LVX2、PCAP、PCAPNG、CAP 文件触发离线加载。

## 9. 点云处理流程

实时点云和离线回放最终都进入相同的处理入口：

1. tag 滤波：根据滤噪列表高亮或移除指定 tag。
2. 反射率着色：按 reflectivity 映射颜色。
3. 距离着色：按点到原点距离归一化映射颜色。
4. 高程着色：按 z 值范围映射颜色。
5. 纯色模式：全部点使用用户选择颜色。
6. 平面投影着色：按投影平面坐标归一化映射颜色。
7. 球面投影和平面投影会在解码或处理阶段改变显示坐标。

## 10. 数据采集与保存流程

### LOG 和 Debug 采集

1. 用户在工具菜单中启动 LOG 或 Debug 采集。
2. 程序检查当前是否已有采集任务。
3. 设置统一采集计时器和进度条。
4. LOG 调用 Livox SDK 日志保存接口。
5. Debug 调用 Livox SDK Debug 点云接口。
6. 到达时长后停止采集并恢复状态。

### PCD/LAS 点云保存

1. 用户选择保存目录和帧数。
2. 程序创建按设备 SN 命名的目标目录。
3. 在实时渲染循环中按合并帧写出 PCD 或 LAS。
4. 每帧文件名使用窗口末尾时间戳。
5. 完成指定帧数后自动停止。

### LVX2 录制

1. 用户选择目标路径和录制时长。
2. 程序写入 LVX2 public/private header 和设备信息。
3. 点云回调中把原始 SDK 点云包封装为 LVX2 package。
4. 每 50ms 写入一个 LVX2 frame header 和对应 package。
5. 录制计时结束后关闭文件。

### IMU CSV 保存

1. 用户启用设备 IMU 数据发送。
2. 选择保存目录和时长。
3. 程序创建 `IMU_<SN>` 目录和 CSV 文件。
4. IMU 回调中逐样本写入 timestamp、gyro 和 acceleration。
5. 计时结束后 flush 并关闭文件。

## 11. IMU、GPS 与串口流程

- IMU 回调只处理当前选中设备的数据。
- 最新 IMU 样本保存在 `latestImu`，由 IMU 表格和曲线线程读取。
- IMU 曲线窗口包含陀螺仪和加速度两组曲线。
- GPS 模拟使用定时器生成 GPRMC 类报文并写入日志。
- 串口转发使用 `QSerialPort` 在线程中读取 NMEA 报文，识别 RMC/GGA/GSA/GSV 等内容并更新日志和状态栏。

## 12. LVX2 离线回放流程

1. 用户通过菜单或拖放选择 LVX2 文件。
2. 程序关闭当前回放状态并设置加载 token。
3. 后台线程读取 LVX2 public header、private header、设备信息和 frame index。
4. 主线程校验 token，打开文件并初始化帧缓存、设备可见性和播放状态。
5. 回放支持逐帧和滑窗模式。
6. 每次显示帧时解析 LVX2 package，读取点云 payload，应用设备外参。
7. 解析出的点云进入统一点云处理与显示流程。
8. 播放条支持播放/暂停、首帧、上一帧、下一帧、尾帧、速度和模式切换。

## 13. PCAP 离线回放流程

1. 用户通过菜单或拖放选择 PCAP、PCAPNG 或 CAP 文件。
2. `PcapParser` 使用 Npcap/libpcap 离线读取包。
3. `PcapUdp` 提取 UDP 源/目标 IP、端口和 payload。
4. 端口 56200 的 Push 包用于解析设备 SN、型号、IP、外参。
5. 端口 56300 的点云包用于解析 Livox 点云数据。
6. `PointParser::FrameBuilder` 按 50ms 聚合帧。
7. 解析结果填充到与 LVX2 回放兼容的缓存结构。
8. PCAP 回放复用当前 LVX2 播放条和设备可见性 UI。

## 14. LVX2 格式转换流程

1. 用户选择源 LVX2、输出目录、输出名、转换模式和目标格式。
2. 转换器读取 LVX2 header、设备信息和 frame index。
3. 支持合并全部帧到一个文件，或按 100ms 拆分输出。
4. 支持输出 PCD、LAS、CSV、TXT。
5. 转换过程中读取每个 package，解析点云、应用外参、写出目标格式。
6. UI 通过进度回调更新转换进度。

## 15. 构建与部署流程

1. CMake 查找 Qt6，失败时尝试 Qt5。
2. 使用 C++17 编译。
3. Windows 链接 Livox SDK 静态库、Npcap SDK 的 `wpcap.lib` 和 `Packet.lib`、`ws2_32`、`winmm`、`delayimp.lib`。
4. Linux 链接 pthread、dl、m 和 libpcap。
5. 构建后复制 `livox_sdk_qt` 到可执行文件目录。
6. Windows 运行时通过 `windeployqt` 部署 Qt DLL 和插件。

## 16. 当前架构特征

当前项目虽然目录上已有 `apps` 与 `libs` 分层，但多个 `libs/*/src` 文件仍直接实现 `LivoxViewerWindow` 的成员函数。实际架构仍以 `LivoxViewerWindow` 为中心，UI、SDK、点云、录制、回放、配置和网络发现耦合较深。

LivoxViewerQT
https://img.shields.io/badge/Qt-5.15%252B-green.svg
https://img.shields.io/badge/OpenGL-3.3%252B-blue.svg
https://img.shields.io/badge/Platform-Windows%2520%257C%2520Linux%2520%257C%2520macOS-lightgrey.svg
https://img.shields.io/badge/License-MIT-yellow.svg

一个基于Qt框架开发的Livox激光雷达可视化与控制软件，支持多种Livox雷达设备，提供实时点云显示、设备管理、数据记录和高级可视化功能。

https://via.placeholder.com/800x400.png?text=LivoxViewerQT+Demo+Screenshot

✨ 功能总览
🎯 核心功能
多设备支持: 自动发现并连接网络中的Livox雷达设备

实时点云可视化: 高性能OpenGL点云渲染，支持大规模点云数据

多种显示模式: 反射率着色、距离着色、高程着色、平面投影等

交互式测量: 支持3D空间距离测量和点选操作

智能网络配置: 自动检测和配置主机IP，确保与雷达设备通信

📊 数据记录
点云录制: 支持LVX2格式录制和PCD/LAS格式导出

IMU数据记录: 实时记录陀螺仪和加速度计数据到CSV

设备日志采集: 支持设备调试日志采集

参数记录: 实时记录设备配置参数变化

⚙️ 设备控制
参数配置: 完整的设备参数可视化配置界面

工作模式控制: 支持采样、待机、睡眠等多种工作模式

FOV配置: 可视化配置雷达视场角

网络设置: 灵活的雷达和主机网络配置

🔧 高级功能
GPS时间同步: 支持GPS模拟和串口GPS数据转发

IMU数据显示: 实时显示IMU数据曲线和数值

点云滤波: 噪点识别和过滤功能

平面投影: 球坐标到平面坐标的投影显示

深度投影: 固定深度投影显示模式

📋 目录
系统要求

安装与编译

快速开始

使用指南

项目结构

故障排除

技术架构

贡献指南

许可证

支持

💻 系统要求
支持的操作系统
Windows: Windows 10/11 (64位)

Linux: Ubuntu 18.04+ (64位)

macOS: macOS 10.14+ (Intel/Apple Silicon)

硬件要求
CPU: Intel i5或同等性能以上

内存: 8GB RAM (推荐16GB)

显卡: 支持OpenGL 3.3+的独立显卡

存储: 至少1GB可用空间

网络: 千兆以太网接口

🛠️ 安装与编译
前置依赖
安装Qt
下载并安装 Qt 5.15+ 或 Qt 6.2+，确保包含以下模块：

Qt Core

Qt GUI

Qt Widgets

Qt OpenGL

Qt Network

Qt SerialPort

Qt Charts

编译步骤
Windows
bash
# 使用Qt Creator
1. 使用Qt Creator打开项目
2. 选择MSVC编译器套件
3. 配置构建目录
4. 构建项目

# 或使用命令行
qmake -tp vc LivoxViewerQT.pro
msbuild LivoxViewerQT.sln /p:Configuration=Release
Linux
bash
# 安装依赖 (Ubuntu/Debian)
sudo apt update
sudo apt install build-essential libgl1-mesa-dev

# 编译
qmake LivoxViewerQT.pro
make -j$(nproc)
macOS
bash
# 使用Homebrew安装Qt (可选)
brew install qt

# 编译
qmake LivoxViewerQT.pro
make -j$(sysctl -n hw.ncpu)

# 运行
open LivoxViewerQT.app
🚀 快速开始
首次运行配置
首次运行时会自动检测网络配置：

程序会自动扫描有线网络接口

检测连接的Livox雷达设备

自动配置主机IP与雷达在同一网段

生成或更新配置文件(config.json)

基本使用流程
连接设备: 使用网线连接主机和Livox雷达

启动程序: 运行LivoxViewerQT

自动发现: 程序会自动发现并连接雷达设备

开始可视化: 连接成功后点云数据将自动显示

📖 使用指南
3D视图控制
旋转视图: 鼠标左键拖拽

平移视图: 鼠标右键拖拽

缩放: 鼠标滚轮

重置视图: 工具栏"重置视角"按钮

点云测量
启用测量模式（工具栏测量按钮）

按住Ctrl+左键选择第一个点

按住Ctrl+左键选择第二个点

查看距离测量结果

区域选择
启用选择模式（工具栏选择按钮）

按住Ctrl+左键拖拽选择区域

查看选中点的属性表格

数据记录
录制点云
点击"录制LVX2"按钮

选择保存路径和时长

开始录制

录制完成后自动保存

导出点云
PCD格式: 适用于PCL库

LAS格式: 适用于专业点云软件

通过文件菜单选择导出格式

IMU数据记录
确保IMU数据发送已开启

点击"保存IMU数据"

选择保存路径和时长

开始记录

📁 项目结构
text
LivoxViewerQT/
├── main.cpp                  # 程序入口
├── mainwindow.h/cpp          # 主窗口类
├── point_widget.cpp          # 点云显示组件
├── sdk_init.cpp              # Livox SDK初始化
├── sdk_callbacks.cpp         # SDK回调处理
├── point_visualize.cpp       # 点云可视化处理
├── resources/                # 资源文件
│   ├── icons/               # 程序图标
│   └── styles/              # 样式表
├── config.json              # 配置文件（自动生成）
├── CMakeLists.txt           # CMake构建配置
├── README.md                # 项目说明
└── LICENSE                  # 许可证文件
🔧 故障排除
常见问题
设备无法连接
检查网线连接

确认主机IP与雷达在同一网段

检查防火墙设置

查看程序日志输出

点云显示异常
确认显卡驱动支持OpenGL 3.3+

检查点云数据格式设置

调整点云大小和颜色模式

性能问题
降低点云积分时间

减小显示点大小

启用点云滤波

日志查看
程序运行日志显示在主界面底部日志区域，包含详细的设备状态和错误信息。

🏗️ 技术架构
前端: Qt Widgets + OpenGL

3D渲染: 自定义OpenGL着色器

设备通信: Livox SDK + UDP广播

数据格式: LVX2、PCD、LAS、CSV

多线程: 异步数据处理和渲染

🤝 贡献指南
我们欢迎社区贡献！请参阅以下指南：

提交Issue
使用清晰的标题描述问题

提供详细的重现步骤

包含系统环境和版本信息

提交Pull Request
Fork本项目

创建功能分支 (git checkout -b feature/AmazingFeature)

提交更改 (git commit -m 'Add some AmazingFeature')

推送到分支 (git push origin feature/AmazingFeature)

开启Pull Request

开发规范
遵循Qt编码规范

添加适当的注释

更新相关文档

确保所有测试通过

📄 许可证
本项目基于Qt框架和Livox SDK开发，具体许可证信息请参考相关依赖组件的许可证条款。

主要许可证: MIT License

💬 支持
如有问题请联系：

📧 邮箱: your-email@example.com

🐛 Issues

📚 项目Wiki

LivoxViewerQT - 让激光雷达数据可视化更简单 ✨

注意: 本项目与Livox Tech无关，是一个独立的开源项目。Livox是Livox Tech的注册商标。
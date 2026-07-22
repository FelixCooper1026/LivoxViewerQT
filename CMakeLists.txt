# LivoxViewerQT - 跨平台CMake配置文件
# 支持Windows (MSVC) 和 Linux (GCC/Clang) 平台

cmake_minimum_required(VERSION 3.16)
project(LivoxViewerQT 
    VERSION 2.3.4
    DESCRIPTION "Livox LiDAR Qt Viewer Application"
    LANGUAGES C CXX
)

# =============================================================================
# 基础配置
# =============================================================================

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置构建类型默认值
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

# 启用详细输出（调试时有用）
option(VERBOSE_BUILD "Enable verbose build output" OFF)
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()

# =============================================================================
# 平台检测和配置
# =============================================================================

# 检测操作系统
if(WIN32)
    set(PLATFORM_NAME "Windows")
    set(IS_WINDOWS TRUE)
elseif(UNIX AND NOT APPLE)
    set(PLATFORM_NAME "Linux")
    set(IS_LINUX TRUE)
elseif(APPLE)
    set(PLATFORM_NAME "macOS")
    set(IS_MACOS TRUE)
else()
    set(PLATFORM_NAME "Unknown")
endif()

# Windows 资源文件支持，用于编译 .rc 图标资源
if(IS_WINDOWS)
    set(CMAKE_VS_GLOBALS "VcpkgEnabled=false")
    enable_language(RC)
endif()

message(STATUS "Building for ${PLATFORM_NAME} platform")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# =============================================================================
# Qt依赖查找
# =============================================================================

# 查找Qt6，如果失败则尝试Qt5
find_package(Qt6 6.2 QUIET COMPONENTS 
    Core Widgets OpenGL OpenGLWidgets SerialPort Charts Network Svg Concurrent Sql
)

if(Qt6_FOUND)
    set(QT_VERSION_MAJOR 6)
    set(QT_FOUND TRUE)
    message(STATUS "Found Qt6: ${Qt6_VERSION}")
else()
    # 尝试Qt5作为备选
    find_package(Qt5 5.15 QUIET COMPONENTS 
        Core Widgets OpenGL SerialPort Charts Network Svg Concurrent Sql
    )
    
    if(Qt5_FOUND)
        set(QT_VERSION_MAJOR 5)
        set(QT_FOUND TRUE)
        message(STATUS "Found Qt5: ${Qt5_VERSION}")
        message(STATUS "Note: Qt6 not found, using Qt5 as fallback")
    else()
        message(FATAL_ERROR "Neither Qt6 nor Qt5 found. Please install Qt development packages.")
    endif()
endif()

# Qt标准项目设置
if(QT_VERSION_MAJOR EQUAL 6)
    qt_standard_project_setup()
endif()

# =============================================================================
# Livox SDK配置
# =============================================================================

# 设置Livox SDK路径
set(LIVOX_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/livox_sdk_qt")
set(LIVOX_INCLUDE_PATH "${LIVOX_SDK_PATH}/include")
set(LIVOX_LIB_PATH "${LIVOX_SDK_PATH}/lib")

# 检查Livox SDK是否存在
if(NOT EXISTS "${LIVOX_SDK_PATH}")
    message(WARNING "Livox SDK directory not found: ${LIVOX_SDK_PATH}")
    set(LIVOX_SDK_AVAILABLE FALSE)
else()
    set(LIVOX_SDK_AVAILABLE TRUE)
    message(STATUS "Found Livox SDK: ${LIVOX_SDK_PATH}")
endif()

# 查找Livox SDK库文件
set(LIVOX_LIBRARY "")
if(LIVOX_SDK_AVAILABLE)
    if(IS_WINDOWS)
        # Windows平台查找.lib文件
        set(LIVOX_LIB_CANDIDATES
            "${LIVOX_LIB_PATH}/livox_lidar_sdk_static.lib"
        )
    else()
        # Linux/Unix平台查找.a文件
        set(LIVOX_LIB_CANDIDATES
            "${LIVOX_LIB_PATH}/liblivox_lidar_sdk_static.a"
        )
    endif()
    
    # 查找第一个存在的库文件
    foreach(lib_candidate ${LIVOX_LIB_CANDIDATES})
        if(EXISTS "${lib_candidate}")
            set(LIVOX_LIBRARY "${lib_candidate}")
            break()
        endif()
    endforeach()
    
    if(LIVOX_LIBRARY)
        message(STATUS "Found Livox library: ${LIVOX_LIBRARY}")
    else()
        message(WARNING "Livox SDK library not found in ${LIVOX_LIB_PATH}")
        message(STATUS "Searched for: ${LIVOX_LIB_CANDIDATES}")
    endif()
endif()

# =============================================================================
# 源文件收集
# =============================================================================

set(SLAM_FAST_LIO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/libs/Slam/third_party/fast_lio")
set(SLAM_FAST_LIO_COMPAT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/libs/Slam/third_party/fast_lio_compat")
set(THIRD_PARTY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third-party")
set(SLAM_EIGEN_INCLUDE_DIR "${THIRD_PARTY_ROOT}/eigen-3.4.0")
if(WIN32)
    set(SETUP_THIRD_PARTY_COMMAND "powershell.exe -NoProfile -ExecutionPolicy Bypass -File \"${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup_third_party.ps1\"")
else()
    set(SETUP_THIRD_PARTY_COMMAND "bash \"${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup_third_party.sh\"")
endif()

if(NOT EXISTS "${SLAM_EIGEN_INCLUDE_DIR}/Eigen/Core")
    message(FATAL_ERROR "Eigen headers are required at ${SLAM_EIGEN_INCLUDE_DIR}. Run: ${SETUP_THIRD_PARTY_COMMAND}")
endif()

set(SLAM_GTSAM_INSTALL_DIR "${THIRD_PARTY_ROOT}/gtsam-4.2.0-install")
list(PREPEND CMAKE_PREFIX_PATH "${SLAM_GTSAM_INSTALL_DIR}")
if(WIN32)
    set(GTSAM_DIR "${SLAM_GTSAM_INSTALL_DIR}/CMake")
else()
    set(GTSAM_DIR "${SLAM_GTSAM_INSTALL_DIR}/lib/cmake/GTSAM")
endif()

set(Eigen3_DIR "${SLAM_EIGEN_INCLUDE_DIR}/share/eigen3/cmake")
set(BOOST_ROOT "${THIRD_PARTY_ROOT}/boost-1.82.0")
set(BOOST_LIBRARYDIR "${BOOST_ROOT}/stage/lib")
set(Boost_DIR "${BOOST_LIBRARYDIR}/cmake/Boost-1.82.0")
set(Boost_USE_STATIC_LIBS ON)
if(MSVC)
    set(Boost_COMPILER "-vc143")
endif()
set(Boost_NO_SYSTEM_PATHS ON)
if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

find_package(OpenMP REQUIRED COMPONENTS CXX)
find_package(GTSAM REQUIRED)

message(STATUS "FAST_LIO source snapshot: ${SLAM_FAST_LIO_ROOT}")
message(STATUS "FAST_LIO Eigen include: ${SLAM_EIGEN_INCLUDE_DIR}")
message(STATUS "FAST_LIO OpenMP: ${OpenMP_CXX_VERSION}")
message(STATUS "FAST_LIO_SAM GTSAM: ${GTSAM_VERSION}")

set(SLAM_FAST_LIO_SAM_SOURCES
    libs/Slam/src/Backends/FastLioSam/FastLioSamPoseGraph.cpp
    libs/Slam/src/Backends/FastLioSam/FastLioSamLoopClosure.cpp
    libs/Slam/src/Backends/FastLioSam/FastLioSamMapBuilder.cpp
)

# 第三方轻量库
set(THIRD_PARTY_LZ4_DIR "${THIRD_PARTY_ROOT}/lz4-1.10.0")
add_library(third_party_lz4 STATIC
    ${THIRD_PARTY_LZ4_DIR}/lz4.c
    ${THIRD_PARTY_LZ4_DIR}/lz4frame.c
    ${THIRD_PARTY_LZ4_DIR}/lz4hc.c
    ${THIRD_PARTY_LZ4_DIR}/xxhash.c
)
target_include_directories(third_party_lz4 PUBLIC ${THIRD_PARTY_LZ4_DIR})

# 基础源文件
set(SOURCES
    apps/LivoxViewer/main.cpp
    apps/LivoxViewer/LivoxViewerUi.cpp
    apps/LivoxViewer/panels/ViewerToolbar.cpp
    apps/LivoxViewer/panels/PlaybackBar.cpp
    apps/LivoxViewer/panels/DevicePanel.cpp
    apps/LivoxViewer/panels/ParameterPanel.cpp
    apps/LivoxViewer/panels/FileInfoPanel.cpp
    apps/LivoxViewer/panels/ImuPanel.cpp
    apps/LivoxViewer/panels/LogPanel.cpp
    apps/LivoxViewer/slam/SlamControlDialog.cpp
    apps/LivoxViewer/slam/SlamUiBridge.cpp
    apps/LivoxViewer/slam/SlamWindowActions.cpp
    apps/LivoxViewer/actions/MainMenuActions.cpp
    apps/LivoxViewer/actions/DeviceActions.cpp
    apps/LivoxViewer/actions/HelpActions.cpp
    apps/LivoxViewer/actions/PlaybackActions.cpp
    apps/LivoxViewer/actions/FileActions.cpp
    apps/LivoxViewer/actions/StatusRuntime.cpp
    apps/LivoxViewer/dialogs/FormatConvertDialog.cpp
    apps/LivoxViewer/dialogs/FirmwareUpgradeDialog.cpp
    apps/LivoxViewer/dialogs/PointCloudFilterDialog.cpp
    apps/LivoxViewer/dialogs/DataCaptureDialogs.cpp
    apps/LivoxViewer/dialogs/PacketCaptureDialog.cpp
    apps/LivoxViewer/dialogs/ConfigGeneratorDialog.cpp
    apps/LivoxViewer/dialogs/ImuVisualizationDialog.cpp
    apps/LivoxViewer/widgets/ImuOrientationView.cpp
    apps/LivoxViewer/widgets/VisualizationWorkspace.cpp
    apps/LivoxViewer/imu/ImuAttitudeEstimator.cpp
    apps/LivoxViewer/utils/DeviceModelResource.cpp
    apps/LivoxViewer/actions/PlaybackControllerActions.cpp
    apps/LivoxViewer/actions/VisualizationWorkspaceActions.cpp
    apps/LivoxViewer/actions/PcapPlaybackActions.cpp
    apps/LivoxViewer/actions/Lvx2ConvertActions.cpp
    apps/LivoxViewer/actions/PointCloudActions.cpp
    apps/LivoxViewer/actions/PointCloudRuntimeActions.cpp
    apps/LivoxViewer/actions/PointCloudRecordingActions.cpp
    apps/LivoxViewer/actions/PointCloudSelectionActions.cpp
    apps/LivoxViewer/actions/ImuActions.cpp
    apps/LivoxViewer/actions/SerialGpsActions.cpp
    apps/LivoxViewer/actions/LidarSdkActions.cpp
    apps/LivoxViewer/actions/LidarSdkCallbacks.cpp
    apps/LivoxViewer/actions/LidarParameterActions.cpp
    apps/LivoxViewer/LivoxViewerActions.cpp
    apps/LivoxViewer/LivoxViewerSettings.cpp
    libs/LivoxCore/src/LidarSdkService.cpp
    libs/LivoxCore/src/LidarDiscoveryService.cpp
    libs/LivoxCore/src/LidarParameterService.cpp
    libs/PointCloud/src/PointCloudView.cpp
    libs/PointCloud/src/PointCloudEdlRenderer.cpp
    libs/PointCloud/src/PointCloudDecoder.cpp
    libs/PointCloud/src/PointCloudColorizer.cpp
    libs/PointCloud/src/PointCloudFilter.cpp
    libs/PointCloud/src/PointCloudProjection.cpp
    libs/PointCloud/src/PointCloudCrossSection.cpp
    plugins/StlModel/StlModelLoader.cpp
    libs/Lvx/src/LvxReader.cpp
    libs/Lvx/src/LvxPointParser.cpp
    libs/Lvx2/src/Lvx2Reader.cpp
    libs/Lvx2/src/Lvx2PointParser.cpp
    libs/Pcap/src/ImuParser.cpp
    libs/Pcap/src/PcapParser.cpp
    libs/Pcap/src/PcapReader.cpp
    libs/Pcap/src/PcapUdpPacket.cpp
    libs/Pcap/src/PointParser.cpp
    libs/Pcap/src/PushMsgParser.cpp
    libs/Rosbag/src/RosbagReader.cpp
    libs/Rosbag/src/Ros2BagReader.cpp
    libs/Rosbag/src/RosMessageParsers.cpp
    libs/Rosbag/src/RosbagPlaybackSource.cpp
    libs/Slam/src/Core/ISlamBackend.cpp
    libs/Slam/src/Core/FastLioInputSynchronizer.cpp
    libs/Slam/src/Core/SlamInputQueue.cpp
    libs/Slam/src/Core/SlamRuntimeConfig.cpp
    libs/Slam/src/Backends/FastLio/FastLioSlamBackend.cpp
    ${SLAM_FAST_LIO_SAM_SOURCES}
    libs/DynamicObject/src/DynamicObjectDetector.cpp
    libs/DynamicObject/src/DynamicObjectCluster.cpp
    libs/Slam/src/Export/SlamMapExport.cpp
    libs/Slam/src/Export/SlamTrajectoryExport.cpp
    libs/Slam/third_party/fast_lio/include/ikd-Tree/ikd_Tree.cpp
    libs/Slam/src/Io/LiveLidarSlamSource.cpp
    libs/Slam/src/Io/LvxSlamSource.cpp
    libs/Slam/src/Io/PcapSlamSource.cpp
    libs/Slam/src/Io/RosbagSlamSource.cpp
    libs/Export/src/PointCloudExport.cpp
    libs/AppConfig/src/AppSettings.cpp
    libs/AppConfig/src/LidarConfigService.cpp
    libs/AppConfig/src/ConfigJsonService.cpp
    libs/AppConfig/src/NetworkInterfaceService.cpp
)

# 头文件
  set(HEADERS
     apps/LivoxViewer/LivoxViewerWindow.h
     apps/LivoxViewer/state/CaptureSessionState.h
     apps/LivoxViewer/state/ImuRuntimeState.h
     apps/LivoxViewer/state/ParameterUiState.h
     apps/LivoxViewer/state/PlaybackControllerState.h
     apps/LivoxViewer/state/PointCloudFilterState.h
     apps/LivoxViewer/widgets/ParameterOptionButtons.h
     apps/LivoxViewer/widgets/SwitchCheckBox.h
     apps/LivoxViewer/widgets/ImuOrientationView.h
     apps/LivoxViewer/widgets/VisualizationWorkspace.h
     apps/LivoxViewer/slam/SlamControlDialog.h
     apps/LivoxViewer/slam/SlamUiBridge.h
     apps/LivoxViewer/dialogs/DialogWindowUtils.h
     apps/LivoxViewer/dialogs/ImuVisualizationDialog.h
    apps/LivoxViewer/imu/ImuAttitudeEstimator.h
    apps/LivoxViewer/utils/DeviceModelResource.h
    apps/LivoxViewer/dialogs/PacketCaptureDialog.h
    libs/LivoxCore/include/LidarDiscoveryService.h
    libs/LivoxCore/include/LidarDeviceInfo.h
    libs/LivoxCore/include/LidarDiagnostics.h
    libs/LivoxCore/include/LidarModelUtils.h
    libs/LivoxCore/include/LidarPacketUtils.h
    libs/LivoxCore/include/LidarParameterService.h
    libs/LivoxCore/include/LidarSdkService.h
    libs/LivoxCore/include/LidarSdkTypes.h
    libs/PointCloud/include/PointCloudTypes.h
    libs/PointCloud/include/PointCloudFrame.h
    libs/PointCloud/include/PointCloudView.h
    libs/PointCloud/include/PointCloudEdlRenderer.h
    libs/PointCloud/include/PointCloudRenderConfig.h
    libs/PointCloud/include/PointCloudDecoder.h
    libs/PointCloud/include/PointCloudColorizer.h
    libs/PointCloud/include/PointCloudFilter.h
    libs/PointCloud/include/PointCloudProjection.h
    libs/PointCloud/include/PointCloudCrossSection.h
    libs/PointCloud/include/PointCloudPipeline.h
    libs/PointCloud/include/PointCloudPersistence.h
    plugins/StlModel/StlModelLoader.h
    libs/Playback/include/PlaybackSource.h
    libs/Lvx/include/LvxTypes.h
    libs/Lvx/include/LvxReader.h
    libs/Lvx/include/LvxPointParser.h
    libs/Lvx2/include/Lvx2Types.h
    libs/Lvx2/include/Lvx2Reader.h
    libs/Lvx2/include/Lvx2PlaybackController.h
    libs/Lvx2/include/Lvx2Converter.h
    libs/Lvx2/include/Lvx2PointParser.h
    libs/Pcap/include/ImuParser.h
    libs/Pcap/include/PcapParser.h
    libs/Pcap/include/PcapReader.h
    libs/Pcap/include/PcapUdpPacket.h
    libs/Pcap/include/PointParser.h
    libs/Pcap/include/PushMsgParser.h
    libs/Pcap/include/PcapPlaybackController.h
    libs/Rosbag/include/RosbagTypes.h
    libs/Rosbag/include/RosbagReader.h
    libs/Rosbag/include/Ros2BagReader.h
    libs/Rosbag/include/RosMessageParsers.h
    libs/Rosbag/include/RosbagPlaybackController.h
    libs/Rosbag/include/RosbagPlaybackSource.h
    libs/Slam/include/Core/ISlamBackend.h
    libs/Slam/include/Core/FastLioInputSynchronizer.h
    libs/Slam/include/Core/SlamInputQueue.h
    libs/Slam/include/Core/SlamRuntimeConfig.h
    libs/Slam/include/Core/SlamTypes.h
    libs/Slam/include/Backends/FastLio/FastLioSlamBackend.h
    libs/Slam/include/Backends/FastLioSam/FastLioSamTypes.h
    libs/Slam/include/Backends/FastLioSam/FastLioSamBackendState.h
    libs/Slam/include/Backends/FastLioSam/FastLioSamPoseGraph.h
    libs/Slam/include/Backends/FastLioSam/FastLioSamLoopClosure.h
    libs/Slam/include/Backends/FastLioSam/FastLioSamMapBuilder.h
    libs/DynamicObject/include/DynamicObjectDetector.h
    libs/DynamicObject/include/DynamicObjectCluster.h
    libs/Slam/include/Export/SlamMapExport.h
    libs/Slam/include/Export/SlamTrajectoryExport.h
    libs/Slam/include/Io/LiveLidarSlamSource.h
    libs/Slam/include/Io/LvxSlamSource.h
    libs/Slam/include/Io/PcapSlamSource.h
    libs/Slam/include/Io/RosbagSlamSource.h
    libs/Slam/include/Visualization/SlamRenderSnapshot.h
    libs/Export/include/PointCloudExport.h
    libs/AppConfig/include/AppSettings.h
    libs/AppConfig/include/LidarConfigService.h
    libs/AppConfig/include/ConfigJsonService.h
    libs/AppConfig/include/NetworkInterfaceService.h
)

# 资源文件：Qt 内部资源，用于运行时窗口图标、按钮图标等
set(RESOURCE_FILES
    icons/icons.qrc
)

# 应用图标配置
set(APP_ICON_RESOURCE "")
set(APP_LINUX_ICON_FILE "${CMAKE_CURRENT_SOURCE_DIR}/resources/app_icon.png")
set(APP_LINUX_ICON_NAME "livoxviewerqt")

# Windows exe 文件图标资源
if(IS_WINDOWS)
    set(APP_ICON_RESOURCE "${CMAKE_CURRENT_SOURCE_DIR}/resources/app_icon.rc")

    if(NOT EXISTS "${APP_ICON_RESOURCE}")
        message(FATAL_ERROR "Windows icon resource not found: ${APP_ICON_RESOURCE}")
    endif()

    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/resources/app_icon.ico")
        message(FATAL_ERROR "Windows icon file not found: ${CMAKE_CURRENT_SOURCE_DIR}/resources/app_icon.ico")
    endif()

    set_source_files_properties("${APP_ICON_RESOURCE}" PROPERTIES LANGUAGE RC)
endif()


# =============================================================================
# 可执行文件创建
# =============================================================================

if(QT_VERSION_MAJOR EQUAL 6)
    # Qt6方式
    add_executable(LivoxViewerQT
        WIN32 MACOSX_BUNDLE
        ${SOURCES}
        ${HEADERS}
        ${APP_ICON_RESOURCE}
    )
    
    # 设置Qt属性
    set_target_properties(LivoxViewerQT PROPERTIES
        AUTOMOC ON
        AUTOUIC OFF
        AUTORCC ON
    )
    
    # 添加UI文件
    qt_wrap_ui(UI_HEADERS ${UI_FILES})
    target_sources(LivoxViewerQT PRIVATE ${UI_HEADERS})
    
    # 添加资源文件
    if(RESOURCE_FILES)
        qt_add_resources(QRC_SOURCES ${RESOURCE_FILES})
        target_sources(LivoxViewerQT PRIVATE ${QRC_SOURCES})
    endif()
    
else()
    # Qt5方式
    add_executable(LivoxViewerQT
        WIN32 MACOSX_BUNDLE
        ${SOURCES}
        ${HEADERS}
        ${APP_ICON_RESOURCE}
    )

    # 处理UI文件
    qt5_wrap_ui(UI_HEADERS ${UI_FILES})
    target_sources(LivoxViewerQT PRIVATE ${UI_HEADERS})

    # 处理资源文件
    if(RESOURCE_FILES)
        qt5_add_resources(QRC_SOURCES ${RESOURCE_FILES})
        target_sources(LivoxViewerQT PRIVATE ${QRC_SOURCES})
    endif()
endif()

set(APP_VERSION_HEADER "${CMAKE_CURRENT_BINARY_DIR}/generated/AppVersion.h")
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/LivoxViewer/AppVersion.h.in
    ${APP_VERSION_HEADER}
    @ONLY
)
target_sources(LivoxViewerQT PRIVATE ${APP_VERSION_HEADER})

target_compile_definitions(LivoxViewerQT PRIVATE
    LIVOX_VIEWER_VERSION="${PROJECT_VERSION}"
    LIVOX_VIEWER_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    LIVOX_VIEWER_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    LIVOX_VIEWER_VERSION_PATCH=${PROJECT_VERSION_PATCH}
    LIVOX_VIEWER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

# =============================================================================
# 包含路径配置
# =============================================================================

target_include_directories(LivoxViewerQT PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/LivoxViewer
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/LivoxCore/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/PointCloud/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/Playback/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/Lvx/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/Lvx2/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/Pcap/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/Rosbag/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/Slam/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/DynamicObject/include
    ${SLAM_FAST_LIO_COMPAT_ROOT}/include
    ${SLAM_FAST_LIO_ROOT}/include
    ${SLAM_FAST_LIO_ROOT}/src
    ${SLAM_EIGEN_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/Export/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/AppConfig/include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

# 添加Livox SDK包含路径
if(LIVOX_SDK_AVAILABLE AND EXISTS "${LIVOX_INCLUDE_PATH}")
    target_include_directories(LivoxViewerQT PRIVATE "${LIVOX_INCLUDE_PATH}")
    message(STATUS "Added Livox SDK include path: ${LIVOX_INCLUDE_PATH}")
endif()

# =============================================================================
# 编译器特定设置
# =============================================================================

# 编译器警告和优化设置
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # GCC/Clang设置
    target_compile_options(LivoxViewerQT PRIVATE
        -Wall -Wextra -Wpedantic
        $<$<CONFIG:Debug>:-g -O0 -DDEBUG>
        $<$<CONFIG:Release>:-O3 -DNDEBUG>
        $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>
    )
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    # MSVC设置
    target_compile_options(LivoxViewerQT PRIVATE
        /MP
        /utf-8
        /W4
        $<$<CONFIG:Debug>:/Od /Zi /DDEBUG>
        $<$<CONFIG:Release>:/O2 /DNDEBUG>
        $<$<CONFIG:RelWithDebInfo>:/O2 /Zi /DNDEBUG>
    )
    
    # MSVC特定定义
    target_compile_definitions(LivoxViewerQT PRIVATE
        _CRT_SECURE_NO_WARNINGS
        NOMINMAX
        WIN32_LEAN_AND_MEAN
    )
endif()

# =============================================================================
# Linux RPATH（避免依赖 LD_LIBRARY_PATH）
# =============================================================================
if(IS_LINUX)
    # 运行时优先从可执行文件旁的 livox_sdk_qt/lib 查找动态库
    set_target_properties(LivoxViewerQT PROPERTIES
        BUILD_RPATH "\$ORIGIN/livox_sdk_qt/lib"
        INSTALL_RPATH "\$ORIGIN/livox_sdk_qt/lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
endif()

# =============================================================================
# 库链接配置
# =============================================================================

# Qt库链接
if(QT_VERSION_MAJOR EQUAL 6)
    target_link_libraries(LivoxViewerQT PRIVATE
        Qt6::Core
        Qt6::Widgets
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        Qt6::SerialPort
        Qt6::Charts
        Qt6::Network
        Qt6::Svg
        Qt6::Concurrent
        Qt6::Sql
    )
else()
    target_link_libraries(LivoxViewerQT PRIVATE
        Qt5::Core
        Qt5::Widgets
        Qt5::OpenGL
        Qt5::SerialPort
        Qt5::Charts
        Qt5::Network
        Qt5::Svg
        Qt5::Concurrent
        Qt5::Sql
    )
endif()

# Livox SDK库链接
if(LIVOX_LIBRARY)
    target_link_libraries(LivoxViewerQT PRIVATE "${LIVOX_LIBRARY}")
    message(STATUS "Linked Livox SDK library: ${LIVOX_LIBRARY}")
else()
    message(WARNING "Livox SDK library not linked - application may not function properly")
endif()

target_link_libraries(LivoxViewerQT PRIVATE OpenMP::OpenMP_CXX third_party_lz4 gtsam)

# 平台特定的系统库
if(IS_LINUX)
    target_link_libraries(LivoxViewerQT PRIVATE
        pthread
        dl
        m  # 数学库
    )
    find_library(PCAP_LIBRARY pcap)
    if(PCAP_LIBRARY)
        target_link_libraries(LivoxViewerQT PRIVATE ${PCAP_LIBRARY})
        message(STATUS "Linked libpcap: ${PCAP_LIBRARY}")
    else()
        message(WARNING "libpcap not found; install libpcap-dev for Pcap playback support")
    endif()
elseif(IS_WINDOWS)
    target_link_libraries(LivoxViewerQT PRIVATE
        ws2_32  # Windows Socket库
        winmm   # Windows多媒体库
    )

    set(NPCAP_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third-party/npcap-sdk-1.16")
    if(EXISTS "${NPCAP_SDK_DIR}/Include/pcap.h")
        message(STATUS "Npcap SDK: ${NPCAP_SDK_DIR}")
        target_include_directories(LivoxViewerQT PRIVATE "${NPCAP_SDK_DIR}/Include")
        target_link_libraries(LivoxViewerQT PRIVATE
            "${NPCAP_SDK_DIR}/Lib/x64/wpcap.lib"
            "${NPCAP_SDK_DIR}/Lib/x64/Packet.lib"
        )
        if(MSVC)
            set_target_properties(LivoxViewerQT PROPERTIES LINK_FLAGS "/DELAYLOAD:wpcap.dll")
            target_link_libraries(LivoxViewerQT PRIVATE delayimp.lib)
        elseif(MINGW)
            set_target_properties(LivoxViewerQT PROPERTIES LINK_FLAGS "-Wl,--delayload=wpcap.dll")
        endif()
    else()
        message(WARNING "Npcap SDK not found at ${NPCAP_SDK_DIR}; Pcap playback will not build on Windows")
    endif()
endif()

set(SLAM_REPLAY_TOOL_SOURCES
    tools/SlamReplayTool/main.cpp
    libs/Lvx/src/LvxReader.cpp
    libs/Lvx/src/LvxPointParser.cpp
    libs/Lvx2/src/Lvx2Reader.cpp
    libs/Lvx2/src/Lvx2PointParser.cpp
    libs/Pcap/src/ImuParser.cpp
    libs/Pcap/src/PcapUdpPacket.cpp
    libs/Pcap/src/PointParser.cpp
    libs/Pcap/src/PushMsgParser.cpp
    libs/Rosbag/src/RosbagReader.cpp
    libs/Rosbag/src/Ros2BagReader.cpp
    libs/Rosbag/src/RosMessageParsers.cpp
    libs/Slam/src/Core/SlamRuntimeConfig.cpp
    libs/Slam/src/Core/FastLioInputSynchronizer.cpp
    libs/Slam/src/Backends/FastLio/FastLioSlamBackend.cpp
    ${SLAM_FAST_LIO_SAM_SOURCES}
    libs/DynamicObject/src/DynamicObjectDetector.cpp
    libs/DynamicObject/src/DynamicObjectCluster.cpp
    libs/Slam/src/Io/PcapSlamSource.cpp
    libs/Slam/src/Io/RosbagSlamSource.cpp
    libs/Slam/src/Io/LvxSlamSource.cpp
    libs/Slam/third_party/fast_lio/include/ikd-Tree/ikd_Tree.cpp
)

add_executable(SlamReplayTool EXCLUDE_FROM_ALL ${SLAM_REPLAY_TOOL_SOURCES})

target_include_directories(SlamReplayTool PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/LivoxCore/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/PointCloud/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/Playback/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/Lvx/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/Lvx2/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/Pcap/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/Rosbag/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/Slam/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/DynamicObject/include
    ${SLAM_FAST_LIO_COMPAT_ROOT}/include
    ${SLAM_FAST_LIO_ROOT}/include
    ${SLAM_FAST_LIO_ROOT}/src
    ${SLAM_EIGEN_INCLUDE_DIR}
)

if(LIVOX_SDK_AVAILABLE AND EXISTS "${LIVOX_INCLUDE_PATH}")
    target_include_directories(SlamReplayTool PRIVATE "${LIVOX_INCLUDE_PATH}")
endif()

if(QT_VERSION_MAJOR EQUAL 6)
    target_link_libraries(SlamReplayTool PRIVATE Qt6::Core Qt6::Gui Qt6::Sql)
else()
    target_link_libraries(SlamReplayTool PRIVATE Qt5::Core Qt5::Gui Qt5::Sql)
endif()

target_link_libraries(SlamReplayTool PRIVATE OpenMP::OpenMP_CXX third_party_lz4 gtsam)

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options(SlamReplayTool PRIVATE /MP /utf-8 /W4 $<$<CONFIG:Debug>:/Od /Zi /DDEBUG> $<$<CONFIG:Release>:/O2 /DNDEBUG>)
    target_compile_definitions(SlamReplayTool PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

if(IS_LINUX)
    target_link_libraries(SlamReplayTool PRIVATE pthread dl m)
    if(PCAP_LIBRARY)
        target_link_libraries(SlamReplayTool PRIVATE ${PCAP_LIBRARY})
    endif()
elseif(IS_WINDOWS)
    target_link_libraries(SlamReplayTool PRIVATE ws2_32 winmm)
    if(EXISTS "${NPCAP_SDK_DIR}/Include/pcap.h")
        target_include_directories(SlamReplayTool PRIVATE "${NPCAP_SDK_DIR}/Include")
        target_link_libraries(SlamReplayTool PRIVATE
            "${NPCAP_SDK_DIR}/Lib/x64/wpcap.lib"
            "${NPCAP_SDK_DIR}/Lib/x64/Packet.lib"
        )
    endif()
endif()

# =============================================================================
# 构建后处理
# =============================================================================

# 复制SDK文件到构建目录
if(LIVOX_SDK_AVAILABLE)
    add_custom_command(TARGET LivoxViewerQT POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${LIVOX_SDK_PATH}"
        "$<TARGET_FILE_DIR:LivoxViewerQT>/livox_sdk_qt"
        COMMENT "Copying Livox SDK files to build directory"
    )
endif()

# 复制配置文件到构建目录
set(CONFIG_FILES
    "config.json"
    "mid360_config.json"
)

foreach(config_file ${CONFIG_FILES})
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${config_file}")
        add_custom_command(TARGET LivoxViewerQT POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/${config_file}"
            "$<TARGET_FILE_DIR:LivoxViewerQT>/${config_file}"
            COMMENT "Copying ${config_file} to build directory"
        )
    endif()
endforeach()

# =============================================================================
# 安装配置
# =============================================================================

if(IS_WINDOWS)
    get_target_property(QT_QMAKE_EXECUTABLE
        Qt${QT_VERSION_MAJOR}::qmake IMPORTED_LOCATION)
    get_filename_component(QT_BIN_DIR "${QT_QMAKE_EXECUTABLE}" DIRECTORY)

    add_custom_command(TARGET LivoxViewerQT POST_BUILD
        COMMAND "${QT_BIN_DIR}/windeployqt.exe"
            "$<IF:$<CONFIG:Debug>,--debug,--release>"
            --no-compiler-runtime
            --no-translations
            --no-system-d3d-compiler
            --no-opengl-sw
            "$<TARGET_FILE:LivoxViewerQT>"
        COMMENT "Deploying Qt runtime to build directory"
    )
endif()

include(GNUInstallDirs)

# 安装可执行文件
install(TARGETS LivoxViewerQT
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

if(IS_WINDOWS)
    # Windows特定安装
    
    # 安装SDK文件
    if(LIVOX_SDK_AVAILABLE)
        install(DIRECTORY "${LIVOX_SDK_PATH}/"
            DESTINATION "${CMAKE_INSTALL_BINDIR}/livox_sdk_qt"
            FILES_MATCHING 
            PATTERN "*.lib" 
            PATTERN "*.dll"
            PATTERN "*.h"
        )
    endif()
    
    # Qt部署脚本
    if(QT_VERSION_MAJOR EQUAL 6)
        qt_generate_deploy_app_script(
            TARGET LivoxViewerQT
            OUTPUT_SCRIPT deploy_script
            NO_UNSUPPORTED_PLATFORM_ERROR
        )
        install(SCRIPT ${deploy_script})
    endif()
    
else()
    # Linux特定安装
    
    # 安装SDK文件
    if(LIVOX_SDK_AVAILABLE)
        install(DIRECTORY "${LIVOX_SDK_PATH}/"
            DESTINATION "${CMAKE_INSTALL_BINDIR}/livox_sdk_qt"
            FILES_MATCHING 
            PATTERN "*.a" 
            PATTERN "*.so*"
            PATTERN "*.h"
        )
    endif()
    
    # 安装配置文件
    foreach(config_file ${CONFIG_FILES})
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${config_file}")
            install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/${config_file}"
                DESTINATION ${CMAKE_INSTALL_BINDIR}
            )
        endif()
    endforeach()
    
    # 安装桌面文件
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/LivoxViewerQT.desktop")
        install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/LivoxViewerQT.desktop"
            DESTINATION ${CMAKE_INSTALL_DATADIR}/applications
        )
    endif()
    
    # 安装 Linux 桌面 / AppImage 图标
    if(EXISTS "${APP_LINUX_ICON_FILE}")
        # 标准 hicolor 图标路径：推荐给 Linux 桌面环境和 AppImage 使用
        install(FILES "${APP_LINUX_ICON_FILE}"
            DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/256x256/apps
            RENAME ${APP_LINUX_ICON_NAME}.png
        )

        # 兼容传统 pixmaps 路径
        install(FILES "${APP_LINUX_ICON_FILE}"
            DESTINATION ${CMAKE_INSTALL_DATADIR}/pixmaps
            RENAME ${APP_LINUX_ICON_NAME}.png
        )
    else()
        message(WARNING "Linux icon file not found: ${APP_LINUX_ICON_FILE}")
    endif()
    
    # 创建启动脚本
    set(LAUNCHER_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/livoxviewerqt-launcher.sh")
    file(WRITE "${LAUNCHER_SCRIPT}"
        "#!/bin/bash\n"
        "# LivoxViewerQT启动脚本\n"
        "SCRIPT_DIR=\"\$(cd \"\$(dirname \"\${BASH_SOURCE[0]}\")\"; pwd)\"\n"
        "export LD_LIBRARY_PATH=\"\$SCRIPT_DIR/livox_sdk_qt/lib:\$LD_LIBRARY_PATH\"\n"
        "cd \"\$SCRIPT_DIR\"\n"
        "exec ./LivoxViewerQT \"\$@\"\n"
    )
    
    install(PROGRAMS "${LAUNCHER_SCRIPT}"
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        RENAME livoxviewerqt
    )
endif()

# =============================================================================
# 构建信息摘要
# =============================================================================

message(STATUS "")
message(STATUS "=== LivoxViewerQT Build Configuration Summary ===")
message(STATUS "Platform: ${PLATFORM_NAME}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Qt Version: ${QT_VERSION_MAJOR}")
if(QT_VERSION_MAJOR EQUAL 6)
    message(STATUS "Qt6 Version: ${Qt6_VERSION}")
else()
    message(STATUS "Qt5 Version: ${Qt5_VERSION}")
endif()
message(STATUS "Livox SDK Available: ${LIVOX_SDK_AVAILABLE}")
if(LIVOX_LIBRARY)
    message(STATUS "Livox Library: ${LIVOX_LIBRARY}")
endif()
message(STATUS "Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "===============================================")
message(STATUS "")

# 构建完成后的提示信息
if(NOT LIVOX_LIBRARY)
    message(STATUS "")
    message(STATUS "WARNING: Livox SDK library not found!")
    message(STATUS "The application may not function properly without it.")
    message(STATUS "Please ensure the Livox SDK is properly installed in:")
    message(STATUS "  ${LIVOX_SDK_PATH}")
    message(STATUS "")
endif()

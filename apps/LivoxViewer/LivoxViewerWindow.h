#ifndef LIVOXVIEWER_LIVOXVIEWERWINDOW_H
#define LIVOXVIEWER_LIVOXVIEWERWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QStatusBar>
#include <QMenuBar>
#include <QAction>
#include <QMessageBox>
#include <QThread>
#include <QMutex>
#include <QMap>
#include <QDateTime>
#include <QMutexLocker>
#include <QMetaObject>
#include <QPointer>
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QVector3D>
#include <QVector>
#include <QQueue>
#include <QElapsedTimer>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPoint>
#include <QTabWidget>
#include <QTabBar>
#include <QScrollArea>
#include <QComboBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QQuaternion>
#include <QDockWidget>
#include <QToolBar>
#include <QToolButton>
#include <QSettings>
#include <QColor>
#include <QFrame>
#include <QTableView>
#include <QFile>
#include <QSlider>
#include <atomic>
#include <thread>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDialog>
#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QUdpSocket>
#include <QHostAddress>
#include <QAtomicInteger>
#include <functional>
#include <memory>
#include <optional>
#include "AppConfig/NetworkInterfaceService.h"
#include "LivoxCore/LidarDeviceInfo.h"
#include "LivoxCore/LidarDiscoveryService.h"
#include "LivoxCore/LidarSdkTypes.h"
#include "LivoxCore/Lvx2Types.h"
#include "Lvx2/Lvx2Converter.h"
#include "Lvx2/Lvx2PlaybackController.h"
#include "Playback/PlaybackSource.h"
#include "PointCloud/PointCloudFrame.h"
#include "PointCloud/PointCloudView.h"
#include "state/CaptureSessionState.h"
#include "state/ImuRuntimeState.h"
#include "state/ParameterUiState.h"
#include "state/PlaybackControllerState.h"
#include "state/PointCloudFilterState.h"

QT_BEGIN_NAMESPACE
class QChartView;
class QChart;
class QDragEnterEvent;
class QDropEvent;
QT_END_NAMESPACE
QT_BEGIN_NAMESPACE

class LivoxViewerWindow : public QMainWindow
{
    Q_OBJECT

public:
    LivoxViewerWindow(QWidget *parent = nullptr);
    ~LivoxViewerWindow();

    using Lvx2PlaybackFrameIndex = Lvx2Playback::FrameIndex;
    using Lvx2PlaybackExtrinsic = Lvx2Playback::Extrinsic;
    using Lvx2PlaybackMode = Lvx2Playback::Mode;
    using PlaybackDeviceInfo = Playback::DeviceInfo;

    enum class RealtimeConnectionState {
        Idle,
        WaitingNetwork,
        Discovering,
        ReconfiguringNetwork,
        WaitingSdkReady,
        InitializingSdk,
        Running,
        Stopping,
        Error
    };

private:
    void initializeUserInterface();
    QWidget* createViewerToolbar(QWidget* parent);
    QWidget* createPlaybackBar(QWidget* parent);
    void createDevicePanel();
    void createParameterPanel();
    void createImuPanel();
    void createFileInfoPanel();
    void createLogPanel();
    void createMenusAndActions();
    void showFormatConvertDialog();
    void showFirmwareUpgradeDialog();
    void showPointCloudFilterDialog();
    void showTimeSyncDialog();
    void showPointCloudCaptureDialog();
    void showImuCaptureDialog();
    void showParameterCaptureDialog();
    void showDebugCaptureDialog();
    void createDeviceActions();
    void createHelpActions();
    void createPlaybackActions(QMenu* toolsMenu);
    void createFileActions();
    void createStatusBarAndTimers();
    void initializeLivoxSdk();
    void shutdownLivoxSdk();
    bool showConfigGeneratorDialog();
    void loadViewPreferences();
    void saveViewPreferences();
    void showPreferencesDialog();
    void applyUiTheme();
    bool shouldUseDarkTheme() const;
    int effectiveColorMode() const;
    void updatePointCloudLegend();
    void updateProjectionControlsVisibility();
    void syncPointCloudVisualizationAction();

    // 点云处理
    void decodePointCloudPacket(uint32_t handle, uint8_t dev_type, const LivoxLidarEthernetPacket* packet);
    void presentPointCloudFrame(const PointCloudFrame& frame);
    void applyPointCloudPipeline(PointCloudFrame& frame);
    QString formatLidarParameterValue(uint16_t key, uint8_t* value, uint16_t length);
    bool loadLvx2PlaybackFile(const QString& filePath);
    bool loadPcapPlaybackFile(const QString& filePath);
    void closeLvx2Playback(bool clearView = true);
    void showLvx2PlaybackFrame(int playbackFrameIndex);
    void updateLvx2PlaybackUi();
    void setLvx2PlaybackPlaying(bool playing);
    void finishPlaybackSourceLoad(const std::shared_ptr<Playback::Source>& source);
    int playbackRawEndIndexForFrame(int playbackFrameIndex, Lvx2PlaybackMode mode, uint64_t intervalMs) const;
    int playbackFrameIndexForRawEndIndex(int rawEndIndex, Lvx2PlaybackMode mode, uint64_t intervalMs) const;
    QString lvx2DeviceTypeToModel(uint8_t deviceType) const;
    void rebuildLvx2DeviceTab();
    int lvx2PlaybackIntervalMs() const;
    using Lvx2ConvertMode = Lvx2Convert::Mode;
    using Lvx2ConvertFormat = Lvx2Convert::Format;
    bool convertLvx2File(const QString& sourcePath,
                         const QString& outputPathNoExt,
                         Lvx2ConvertMode mode,
                         Lvx2ConvertFormat format,
                         const std::function<void(int, int)>& progress);
    bool savePointCloudAsCSV(const QString& filePath, const QVector<PointCloudPoint>& points);
    bool savePointCloudAsTXT(const QString& filePath, const QVector<PointCloudPoint>& points);
    void handlePointCloudRecording(const PointCloudFrame& merged, uint64_t timestampNs);
    bool startPointCloudCapture(PointCloudCaptureFormat format, const QString& baseDir, int captureAmount, QString& errorMessage);
    bool startImuCapture(const QString& baseDir, int durationSec, QString& errorMessage);
    bool startParameterCapture(const QString& baseDir, int durationSec, QString& errorMessage);
    bool startLogCapture(int durationSec, QString& errorMessage);
    bool startDebugPointCloudCapture(int durationSec, QString& errorMessage);
    void stopPointCloudCapture();
    void stopImuCapture();
    void stopParameterCapture();
    void stopLogCapture();
    void stopDebugPointCloudCapture();
    QString debugLogOutputDir() const;
    QString debugPointCloudOutputDir() const;

    // 着色模式
    enum ColorMode {
        ColorByReflectivity = 0,
        ColorByDistance = 1,
        ColorByElevation = 2,
        ColorSolid = 3,
        ColorByLine = 4
    };

    enum ThemeMode {
        ThemeFollowSystem = 0,
        ThemeLight = 1,
        ThemeDark = 2
    };

    // 静态回调函数
    static void onLidarDeviceInfoChange(uint32_t handle, const LivoxLidarInfo* info, void* client_data);
    static void onPointCloudData(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
    static void onImuData(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
    static void onStatusInfo(uint32_t handle, uint8_t dev_type, const char* info, void* client_data);
    static void onAsyncControlResponse(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void* client_data);
    static void onIpConfigResponse(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void* client_data);
    static void onQueryInternalInfoResponse(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void* client_data);
    static QString getLivoxStatusString(livox_status status);
    static QString getRetCodeString(uint8_t ret_code);
    
    // UI components
    QWidget* realtimeDeviceListWidget = nullptr;
    QTabWidget* paramTabWidget;  // 添加QTabWidget成员变量

    QLabel* statusLabel;
    QTextEdit* logText;
    PointCloudView* pointCloudView;
    QLabel* statusLabelBar;

    // Docks and toolbar
    QDockWidget* lidarDevicesDock;
    QDockWidget* paramsDock;
    QDockWidget* imuDock = nullptr;
    QDockWidget* lvx2FileDock = nullptr;
    QDockWidget* logDock;
    QToolBar* mainToolBar;
    QAction* actionStartSdk;
    QAction* actionStopSdk;
    QAction* actionRefresh;
    QAction* actionClearCloud;
    QAction* actionResetView;
    QAction* actionShowImuCharts = nullptr;
    QAction* actionPlayLvx2 = nullptr;
    QAction* actionPlayPcap = nullptr;
    QAction* actionPointCloudVisualization = nullptr;

    // Livox SDK related
    bool sdk_initialized;
    bool sdk_started;
    bool shutting_down = false;
    QTimer* updateTimer;
    QTimer* renderTimer;
    QMutex lidarDeviceMutex;
    QMap<uint32_t, LidarDeviceInfo> lidarDevices;
    uint32_t currentLidarHandle = 0;
    bool hasCurrentLidarHandle = false;
    std::unique_ptr<LivoxLidarIpInfo> pendingLidarIpConfig;
    bool tryGetCurrentDevice(LidarDeviceInfo& out);
    void setCurrentDeviceHandle(uint32_t handle);
    void clearCurrentDevice();

    // 点云组帧相关
    QMap<uint32_t, QQueue<PointCloudFrame>> pendingFrames;
    QMap<uint32_t, uint64_t> lastFrameTimestamp;
    QMap<uint32_t, uint64_t> lastSeenTimestamp; // 最新到达的每设备时间戳（用于滑动窗口）
    QMutex frameMutex;
    uint64_t frameIntervalMs = 100; // 100ms帧间隔

    // 点云回调状态
    bool pointCloudCallbackEnabled;

    // 工作模式状态
    bool isNormalMode;

    // 参数查询相关
    QTimer* paramQueryTimer;
    ParameterUiState parameterState;

    // Menu
    QMenuBar* menuBar;
    QMenu* fileMenu;
    QMenu* deviceMenu;
    QMenu* helpMenu;
    QMenu* viewMenu;
    QAction* exitAction;
    QAction* aboutAction;

    // 点云可视化控制（UI控件指针与状态）
    QSpinBox* pointSizeSpin = nullptr;
    QComboBox* colorModeCombo = nullptr;
    QWidget* projectionControlsGroup = nullptr;
    QDoubleSpinBox* projectionDepthSpin = nullptr;
    QToolButton* projectionDepthCheck = nullptr;
    QToolButton* planarProjectionCheck = nullptr;
    QDoubleSpinBox* planarRadiusSpin = nullptr;
    QTableView* selectionTable = nullptr;
    // 点属性弹窗
    QDockWidget* attrDock = nullptr;
    QTableView* attrTable = nullptr;
    // 采集控制
    CaptureSessionState captureState;
    QPointer<QDialog> pointCloudCaptureDialog;
    QPointer<QDialog> imuCaptureDialog;
    QPointer<QDialog> parameterCaptureDialog;
    QPointer<QDialog> debugCaptureDialog;
    // 多设备升级进度跟踪
    QMap<uint32_t, int> upgradeProgressMap;
    int upgradeTotalDevices = 0;  // 总设备数，用于进度显示
    QMutex upgradeProgressMutex;  // 升级进度映射的互斥锁
	// GPS RMC 模拟
    ImuRuntimeState imuState;
    QWidget* lvx2DeviceListWidget = nullptr;

    int colorMode = ColorByReflectivity;
    int themeMode = ThemeFollowSystem;
    QColor solidColor = QColor(255, 255, 255);
    QVector<QColor> lineColors = {
        QColor(33, 150, 243),
        QColor(46, 204, 113),
        QColor(255, 193, 7),
        QColor(233, 30, 99)
    };
    float pointSizePx = 2.0f;
    float distanceLegendMin = 0.0f;
    float distanceLegendMax = 100.0f;
    float elevationLegendMin = -5.0f;
    float elevationLegendMax = 5.0f;
    // 球坐标深度投影（m）。0 表示使用原始深度
    float projectionDepthMeters = 1.0f;
    // 深度投影启用状态（仅在球坐标时生效）
    bool projectionDepthEnabled = false;

    // 平面投影相关参数
    bool planarProjectionEnabled = false;  // 是否启用平面投影
    float planarProjectionRadius = 10.0f;  // 平面投影半径（米）

    // 点云可视化控制
    bool pointCloudVisualizationEnabled = true;  // 是否启用点云可视化

    // 实时框选支持
    int lastSelectionCount = -1;
    bool selectionRealtimeEnabled = false;

    // 测距暂停播放
    bool measurementModeActive = false;
    bool pointCloudVisualizationBeforeMeasurement = true;
    bool crossSectionModeActive = false;
    bool pointCloudVisualizationBeforeCrossSection = true;
    bool playbackPlayingBeforeCrossSection = false;

    // 更新选中点属性表
    void updateSelectionTableAndLog();

    // PCD 保存
    bool savePointCloudAsPCD(const QString& filePath, const QVector<PointCloudPoint>& points);

    // LAS 保存
    bool savePointCloudAsLAS(const QString& filePath, const QVector<PointCloudPoint>& points);

    // LVX2 录制
    bool startLvx2Recording(const QString& filePath);
    void stopLvx2Recording(bool flushPending);

    PlaybackControllerState playbackState;

    // IMU CSV 采集
    void appendImuCsvRow(quint64 timestamp_ns, float gx, float gy, float gz, float ax, float ay, float az);



    // 点云滤波功能
    PointCloudFilterState filterState;

    // 工具：组合/匹配 tag
    uint8_t makeFilterTag() const {
        return uint8_t((filterState.tagVal76 & 0x3) << 6 | (filterState.tagVal54 & 0x3) << 4 | (filterState.tagVal32 & 0x3) << 2 | (filterState.tagVal10 & 0x3));
    }
    // 更新滤噪列表显示
    void updateNoiseFilterList();

    // 设备发现相关
    void startLidarDiscovery();
    void stopLidarDiscovery();
    void sendLidarBroadcastDiscovery();
    void onLidarDiscoveryResponse(const QByteArray& data, const QHostAddress& sender);
    void handleParsedDiscoveryResponse(const LidarDiscoveryService::DiscoveryResponse& response, const QHostAddress& sender);
    void finalizeDiscoveredLidars();
    bool updateHostIPForDevice(const QString& deviceIP);
    void updateHostIPForDeviceAsync(const NetworkInterfaceService::NetworkInterfaceInfo& iface,
                                    const QString& targetHostIp,
                                    const QString& netmask);
    void waitForHostIpThenInitializeSdk(const QString& interfaceName,
                                        const QString& targetHostIp,
                                        int remainingAttempts);
    bool updateConfigFileIP(const QString& newHostIP);
    bool updateConfigFileDeviceTypesIfNeeded(const QString& configPath);
    QString calculateCompatibleHostIP(const QString& deviceIP);
    void setRealtimeState(RealtimeConnectionState state);
    void enterWaitingNetworkState();
    void scheduleDiscoveryRetry(int delayMs);
    void stopAndDeleteTimer(QTimer*& timer);
    void resetDiscoverySessionState();
    QVector<LidarDeviceInfo> connectedLidarDevicesSnapshot();
    void rebuildRealtimeDeviceCards();
    QWidget* createRealtimeDeviceCard(const LidarDeviceInfo& device);
    bool createAndBindDiscoverySocket(const NetworkInterfaceService::NetworkInterfaceInfo& iface);
    std::optional<NetworkInterfaceService::NetworkInterfaceInfo> selectedLidarInterface() const;
    std::optional<NetworkInterfaceService::NetworkInterfaceInfo> ensureSelectedLidarInterface();
    void selectLidarInterface(const NetworkInterfaceService::NetworkInterfaceInfo& iface);
    void restartRealtimeConnectionForNetworkChange();
    // void printPacketDetails(const QByteArray& data, const QHostAddress& sender);

    // 网络接口选择相关
    QComboBox* networkInterfaceCombo = nullptr;
    QCheckBox* autoConfigHostIpCheck = nullptr;
    QString selectedNetworkIP;
    QString selectedNetworkInterfaceHumanName;
    QString selectedNetworkInterfaceSysName;
    QString selectedInterfaceName;
    QString selectedInterfaceDisplayName;
    QString selectedHostIp;
    QString selectedNetmask;
    QString selectedBroadcast;
    QSet<QString> lastKnownSysNames; // 记录上一次刷新时的网卡系统名称集合
    QElapsedTimer networkWaitLogTimer;
    QString lastNetworkWaitLogMessage;
    void refreshNetworkInterfaces();
    void onNetworkInterfaceChanged(int index);
    QString getSelectedHostIP() const;

    bool autoConfigHostIpEnabled = true;
    bool autoDiscoveryEnabled = true;
    bool manualNetworkConfigPromptActive = false;

    // UDP socket for device discovery
    QUdpSocket* lidarDiscoverySocket;
    QTimer* lidarDiscoveryTimer;
    QTimer* discoveryBroadcastTimer = nullptr;
    QTimer* discoveryTimeoutTimer = nullptr;
    QTimer* discoveryRetryTimer = nullptr;
    QTimer* networkWaitTimer = nullptr;
    QTimer* discoverySettleTimer = nullptr;
    bool lidarDiscoveryActive;
    RealtimeConnectionState realtimeState = RealtimeConnectionState::Idle;
    QSet<QString> localIPv4SetForCurrentSession;
    int discoverySendCount = 0;
    int discoveryBindLogCount = 0;
    int sdkInitRetryCount = 0;
    QString lastAttemptedAutoConfigIp;
    QMap<QString, LidarDiscoveryService::DiscoveryResponse> discoveredLidarsBySn;

private slots:
    void onParamQueryTimeout();
    void onParamConfigChanged(uint16_t key);
    void applyIpConfig(uint16_t key, const QString& ip, const QString& mask, const QString& gateway);
    void applyHostIpConfig(uint16_t key, const QString& ip, int port);
    void applyNtpServerIpConfig(const QString& ip);
    void applyFovConfig(uint16_t key, int yawStart, int yawStop, int pitchStart, int pitchStop);
    void applyAttitudeConfig(uint16_t key, double roll, double pitch, double yaw, int x, int y, int z);
    void onFrameIntervalChanged(int ms);
    void updateFovEnableState(QCheckBox* fov0Check, QCheckBox* fov1Check);
    void updateLidarDeviceList();
    void updateLidarDeviceInfo(const LidarDeviceInfo& device);
    void setActiveRealtimeDevice(uint32_t handle);
    void activateConnectedDevice(const LidarDeviceInfo& device);
    void registerPointCloudDeviceIfNeeded(uint32_t handle, uint8_t dev_type);
    void updateStatus();
    void logMessage(const QString& message);
    void onTabChanged(int index);  // 添加标签页切换槽函数
    void onRenderTick();           // 渲染定时器回调（滑动窗口）
    void onPointSizeChanged(int px);
    void onColorModeClicked(int index);
    void onProjectionDepthToggled(bool enabled);
    void onProjectionDepthChanged(double meters);
    void onPlanarProjectionToggled(bool enabled);
    void onPlanarProjectionRadiusChanged(double radius);
    void onPointCloudVisualizationToggled(bool enabled);
    void onSelectionFinished();
    void onCaptureTick();
    void onActionCaptureImuTriggered();
    void onGpsSimulateToggled(bool enabled);
    void onGpsTick();
    void onImuDisplayButtonClicked();
    void refreshSerialPorts();
    void onSerialEnableToggled(bool enabled);
    void onMeasurementUpdated();
    void onActionShowImuCharts();
    void onLvx2PlaybackTick();
    void onLvx2PlaybackSliderMoved(int value);
    void onLvx2PlaybackFileDropped(const QString& filePath);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

};

#endif // LIVOXVIEWER_LIVOXVIEWERWINDOW_H

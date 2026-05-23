#ifndef LIVOXVIEWER_LIVOXVIEWERWINDOW_H
#define LIVOXVIEWER_LIVOXVIEWERWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QListWidget>
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
#include <QSettings>
#include <QColor>
#include <QFrame>
#include <QTableWidget>
#include <QProgressBar>
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
#include "LivoxCore/LidarDeviceInfo.h"
#include "LivoxCore/LidarSdkTypes.h"
#include "LivoxCore/Lvx2Types.h"
#include "Lvx2/Lvx2Converter.h"
#include "Lvx2/Lvx2PlaybackController.h"
#include "Playback/PlaybackSource.h"
#include "PointCloud/PointCloudFrame.h"
#include "PointCloud/PointCloudView.h"
#include "state/PlaybackControllerState.h"

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
    void createCaptureActions(QMenu* toolsMenu);
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

    // 点云处理
    void decodePointCloudPacket(uint32_t handle, const LivoxLidarEthernetPacket* packet);
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

    // 着色模式
    enum ColorMode {
        ColorByReflectivity = 0,
        ColorByDistance = 1,
        ColorByElevation = 2,
        ColorSolid = 3,
        ColorByPlanarProjection = 4  // 新增：平面投影模式
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
    QListWidget* lidarDeviceList;
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

    // Livox SDK related
    bool sdk_initialized;
    bool sdk_started;
    bool shutting_down = false;
    QTimer* updateTimer;
    QTimer* renderTimer;
    QMutex lidarDeviceMutex;
    QMap<uint32_t, LidarDeviceInfo> lidarDevices;
    LidarDeviceInfo* currentLidarDevice;

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
    QMap<uint16_t, QString> paramValues;
    QMap<uint16_t, QLabel*> paramLabels;

    // 可配置参数控件映射
    QMap<uint16_t, QWidget*> paramControls;

    // 参数配置状态
    QSet<uint16_t> configurableKeys;  // 可配置参数键集合
    QSet<uint16_t> statusKeys;        // 状态参数键集合
    QSet<uint16_t> updatedConfigKeys; // 已更新的可配置参数

    // 参数记录相关
    QPushButton* recordParamsButton = nullptr;
    QFile recordParamsFile;
    bool isRecordingParams = false;
    QString recordParamsFilePath;
    QMap<uint16_t, QString> recordedParamKeys; // 记录所有参数的键和名称映射
    QVector<uint16_t> recordedParamOrder;     // 记录参数的顺序，确保表头和数据对齐

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
    QPushButton* solidColorButton = nullptr;
    QFrame* solidColorPreview = nullptr;
    QWidget* solidColorRow = nullptr;
    QDoubleSpinBox* projectionDepthSpin = nullptr;
    QCheckBox* projectionDepthCheck = nullptr;
    QCheckBox* planarProjectionCheck = nullptr;
    QDoubleSpinBox* planarRadiusSpin = nullptr;
    QTableWidget* selectionTable = nullptr;
    // 点属性弹窗
    QDockWidget* attrDock = nullptr;
    QTableWidget* attrTable = nullptr;
    // 采集控制
    QSpinBox* captureDurationSpin = nullptr;
    QPushButton* btnCaptureLog = nullptr;
    QPushButton* btnCaptureDebug = nullptr;
    QProgressBar* captureProgress = nullptr;
    QTimer* captureTimer = nullptr;
    int captureSecondsRemaining = 0;
    int captureTotalSeconds = 0;
    enum CaptureType { CaptureNone, CaptureLog, CaptureDebug, CaptureLVX2, CaptureIMU } currentCapture = CaptureNone;
    // 多设备升级进度跟踪
    QMap<uint32_t, int> upgradeProgressMap;
    int upgradeTotalDevices = 0;  // 总设备数，用于进度显示
    QMutex upgradeProgressMutex;  // 升级进度映射的互斥锁
	// GPS RMC 模拟
    QCheckBox* gpsSimulateCheck = nullptr;
    QTimer* gpsTimer = nullptr;
    QPushButton* imuDisplayButton = nullptr; // deprecated in UI; kept for backward compatibility
    QTableWidget* imuDataTable = nullptr;
    QTableWidget* lvx2DeviceTable = nullptr;

    // IMU per-axis UI elements
    QProgressBar* gyroBarX = nullptr;
    QProgressBar* gyroBarY = nullptr;
    QProgressBar* gyroBarZ = nullptr;
    QLabel* gyroValX = nullptr;
    QLabel* gyroValY = nullptr;
    QLabel* gyroValZ = nullptr;
    QProgressBar* accBarX = nullptr;
    QProgressBar* accBarY = nullptr;
    QProgressBar* accBarZ = nullptr;
    QLabel* accValX = nullptr;
    QLabel* accValY = nullptr;
    QLabel* accValZ = nullptr;

    // IMU ASCII display
    QLabel* imuAsciiLabel = nullptr;

    // IMU charts
    QChartView* gyroChartView = nullptr;
    QChart* gyroChart = nullptr;
    QLineSeries* gyroSeriesX = nullptr;
    QLineSeries* gyroSeriesY = nullptr;
    QLineSeries* gyroSeriesZ = nullptr;
    QValueAxis* gyroAxisX = nullptr;
    QValueAxis* gyroAxisY = nullptr;

    QChartView* accChartView = nullptr;
    QChart* accChart = nullptr;
    QLineSeries* accSeriesX = nullptr;
    QLineSeries* accSeriesY = nullptr;
    QLineSeries* accSeriesZ = nullptr;
    QValueAxis* accAxisX = nullptr;
    QValueAxis* accAxisY = nullptr;

    QWidget* imuChartWindow = nullptr;

    std::atomic_bool imuDisplayRunning{false};
    std::thread imuDisplayThread;
    std::atomic_bool imuChartRunning{false};
    std::thread imuChartThread;
    QMutex imuSampleMutex;
    struct { float gx=0, gy=0, gz=0, ax=0, ay=0, az=0; bool have=false; } latestImu;
    // 串口转发GPS同步
    QComboBox* serialPortCombo = nullptr;
    QCheckBox* serialEnableCheck = nullptr;
    std::atomic_bool serialRunning{false};
    std::thread serialThread;

    int colorMode = ColorByReflectivity;
    QColor solidColor = QColor(255, 255, 255);
    float pointSizePx = 2.0f;
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

    // 更新选中点属性表
    void updateSelectionTableAndLog();

    // PCD 保存
    QString pcdSaveDir;           // 目标保存目录（PCD_雷达SN）
    int pcdFramesRemaining = 0;   // 待保存帧数
    bool pcdSaveActive = false;   // 是否正在保存
    uint64_t pcdLastSavedTimestamp = 0; // 上一次已保存的帧时间戳，避免重复
    bool savePointCloudAsPCD(const QString& filePath, const QVector<PointCloudPoint>& points);

    // LAS 保存
    QString lasSaveDir;           // 目标保存目录（LAS_雷达SN）
    int lasFramesRemaining = 0;   // 待保存帧数
    bool lasSaveActive = false;   // 是否正在保存
    uint64_t lasLastSavedTimestamp = 0; // 上一次已保存的帧时间戳，避免重复
    bool savePointCloudAsLAS(const QString& filePath, const QVector<PointCloudPoint>& points);

    // LVX2 录制
    QString lvx2SaveDir;          // 目标保存目录（LVX2_雷达SN）
    bool lvx2SaveActive = false;  // 是否正在录制
    QFile lvx2File;               // 当前打开文件
    QVector<QByteArray> lvx2PendingPkgs; // 当前帧待写入包
    uint64_t lvx2FrameStartNs = 0;       // 当前帧起始时间
    uint64_t lvx2FrameIndex = 0;         // 帧序号
    QMutex lvx2Mutex;                     // 录制互斥
    void startLvx2Recording(const QString& filePath, int durationSec);
    void stopLvx2Recording(bool flushPending);

    PlaybackControllerState playbackState;

    // IMU CSV 采集
    QFile imuCsvFile;
    bool imuSaveActive = false;
    int imuSecondsRemaining = 0;
    int imuTotalSeconds = 0;
    QMutex imuCsvMutex;
    void appendImuCsvRow(quint64 timestamp_ns, float gx, float gy, float gz, float ax, float ay, float az);



    // 点云滤波功能
    bool showNoisePoints = false;
    bool removeNoisePoints = false;
    // 滤波参数
    int filterTagVal76 = 0; // bit[7-6]
    int filterTagVal54 = 0; // bit[5-4]
    int filterTagVal32 = 0; // bit[3-2]
    int filterTagVal10 = 0; // bit[1-0]
    // 滤噪列表
    QVector<uint8_t> noiseFilterTags;
    // 控制面板（懒创建）
    QDialog* filterDialog = nullptr;
    QSpinBox* filterSpin76 = nullptr;
    QSpinBox* filterSpin54 = nullptr;
    QSpinBox* filterSpin32 = nullptr;
    QSpinBox* filterSpin10 = nullptr;
    QLabel* filterTagLabel = nullptr;
    QCheckBox* showNoiseCheck = nullptr;
    QCheckBox* removeNoiseCheck = nullptr;
    QListWidget* noiseFilterList = nullptr;
    QPushButton* addNoiseFilterButton = nullptr;
    QPushButton* removeNoiseFilterButton = nullptr;

    // 工具：组合/匹配 tag
    uint8_t makeFilterTag() const {
        return uint8_t((filterTagVal76 & 0x3) << 6 | (filterTagVal54 & 0x3) << 4 | (filterTagVal32 & 0x3) << 2 | (filterTagVal10 & 0x3));
    }
    // 更新滤噪列表显示
    void updateNoiseFilterList();

    // 设备发现相关
    void startLidarDiscovery();
    void stopLidarDiscovery();
    void sendLidarBroadcastDiscovery();
    void onLidarDiscoveryResponse(const QByteArray& data, const QHostAddress& sender);
    bool updateHostIPForDevice(const QString& deviceIP);
    bool updateConfigFileIP(const QString& newHostIP);
    bool updateConfigFileDeviceTypeIfNeeded(const QString& configPath);
    QString calculateCompatibleHostIP(const QString& deviceIP);
    // void printPacketDetails(const QByteArray& data, const QHostAddress& sender);

    // 网络接口选择相关
    QComboBox* networkInterfaceCombo = nullptr;
    QCheckBox* autoConfigHostIpCheck = nullptr;
    QString selectedNetworkIP;
    QString selectedNetworkInterfaceHumanName;
    QString selectedNetworkInterfaceSysName;
    QSet<QString> lastKnownSysNames; // 记录上一次刷新时的网卡系统名称集合
    void refreshNetworkInterfaces();
    void onNetworkInterfaceChanged(int index);
    QString getSelectedHostIP() const;

    bool autoConfigHostIpEnabled = true;
    bool manualNetworkConfigPromptActive = false;

    // UDP socket for device discovery
    QUdpSocket* lidarDiscoverySocket;
    QTimer* lidarDiscoveryTimer;
    bool lidarDiscoveryActive;
    // 最近一次发现到的雷达型号（用于自动校正配置文件中的 Device type）
    uint8_t lastDiscoveredLidarType = 0;
    bool hasLastDiscoveredLidarType = false;

private slots:
    void onParamQueryTimeout();
    void onParamConfigChanged(uint16_t key);
    void applyIpConfig(uint16_t key, const QString& ip, const QString& mask, const QString& gateway);
    void applyHostIpConfig(uint16_t key, const QString& ip, int port);
    void applyFovConfig(uint16_t key, int yawStart, int yawStop, int pitchStart, int pitchStop);
    void applyAttitudeConfig(uint16_t key, double roll, double pitch, double yaw, int x, int y, int z);
    void onFrameIntervalChanged(int ms);
    void updateFovEnableState(QCheckBox* fov0Check, QCheckBox* fov1Check);
    void onLidarDeviceSelected();
    void updateLidarDeviceList();
    void updateLidarDeviceInfo(const LidarDeviceInfo& device);
    void updateStatus();
    void logMessage(const QString& message);
    void addLidarDeviceToList(const LidarDeviceInfo& device);
    void onTabChanged(int index);  // 添加标签页切换槽函数
    void onRenderTick();           // 渲染定时器回调（滑动窗口）
    void onPointSizeChanged(int px);
    void onColorModeChanged(int index);
    void onSolidColorClicked();
    void onProjectionDepthToggled(bool enabled);
    void onProjectionDepthChanged(double meters);
    void onPlanarProjectionToggled(bool enabled);
    void onPlanarProjectionRadiusChanged(double radius);
    void onPointCloudVisualizationToggled(bool enabled);
    void onSelectionFinished();
    void onStartCaptureLog();
    void onStartCaptureDebug();
    void onCaptureTick();
    void onActionCaptureImuTriggered();
    void onGpsSimulateToggled(bool enabled);
    void onGpsTick();
    void onImuDisplayButtonClicked();
    void refreshSerialPorts();
    void onSerialEnableToggled(bool enabled);
    void onMeasurementUpdated();
    void onActionShowImuCharts();
    void onRecordParamsClicked();
    void stopRecordParams(); // 辅助函数
    void onLvx2PlaybackTick();
    void onLvx2PlaybackSliderMoved(int value);
    void onLvx2PlaybackFileDropped(const QString& filePath);

private:
    // Helpers
    QString buildImuAscii(double gx, double gy, double gz, double ax, double ay, double az) const;
};

#endif // LIVOXVIEWER_LIVOXVIEWERWINDOW_H

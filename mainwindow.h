#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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

QT_BEGIN_NAMESPACE
class QChartView;
class QChart;
QT_END_NAMESPACE
QT_BEGIN_NAMESPACE

#pragma pack(push, 1)
struct LVX2PublicHeader {
    char signature[16] = "livox_tech";
    uint8_t version_a = 2;
    uint8_t version_b = 0;
    uint8_t version_c = 0;
    uint8_t version_d = 0;
    uint32_t magic_code = 0xAC0EA767;
};

struct LVX2PrivateHeader {
    uint32_t frame_duration = 50;  // ms
    uint8_t device_count = 1;
};

struct LVX2DeviceInfo {
    char lidar_sn[16] = {};
    char hub_sn[16] = {};
    uint32_t lidar_id = 0;
    uint8_t lidar_type = 247;
    uint8_t device_type = 9;
    uint8_t extrinsic_enable = 1;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct LVX2FrameHeader {
    uint64_t current_offset = 0;
    uint64_t next_offset = 0;
    uint64_t frame_index = 0;
};

struct LVX2PackageHeader {
    uint8_t version = 0;
    uint32_t lidar_id = 0;
    uint8_t lidar_type = 8;
    uint8_t timestamp_type = 0;
    uint64_t timestamp = 0;
    uint16_t udp_counter = 0;
    uint8_t data_type = 0;
    uint32_t data_length = 0;
    uint8_t frame_counter = 0;
    uint8_t reserve[4] = {0};
};
#pragma pack(pop)

// Livox SDK includes
extern "C" {
    #include "livox_lidar_api.h"
}

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

// 设备信息结构
struct DeviceInfo {
    uint32_t handle;
    uint8_t dev_type;
    QString sn;
    QString lidar_ip;
    QString product_info;
    bool is_connected;
    bool is_streaming;
};

// 点云数据结构
struct Point3D {
    float x, y, z;
    float r, g, b;
    uint8_t reflectivity;
    uint8_t tag;
};

struct PointCloudFrame {
    QVector<Point3D> points;
    uint64_t timestamp;
    uint32_t device_handle;
};

// 点云可视化组件
class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT

public:
    explicit PointCloudWidget(QWidget *parent = nullptr);
    ~PointCloudWidget();

    void updatePointCloud(const PointCloudFrame& frame);
    void clearPointCloud();
    void resetView();
    void setPointSize(float sizePixels);
    void setLegend(int mode, float minVal, float maxVal, bool visible);
    QRect currentSelectionRect() const { return m_selectionRect(); }
    QVector<Point3D> currentPoints() const { QMutexLocker locker(const_cast<QMutex*>(&m_pointsMutex)); return m_points; }
    void setSelectionModeEnabled(bool enabled);
    bool isSelectionModeEnabled() const { return m_selectionModeEnabled; }
    QVector<Point3D> pointsInRect(const QRect& rect, int maxPoints = 5000);
    QVector<Point3D> pointsInAabb(const QVector3D& min, const QVector3D& max, int maxPoints = 5000);
    QVector<Point3D> pointsInPersistSelection(int maxPoints = 5000);

    // 世界坐标选择
    void setSelectionAabb(const QVector3D& min, const QVector3D& max) { m_aabbMin = min; m_aabbMax = max; m_selectionLocked = true; update(); }
    void clearSelectionAabb() { m_selectionLocked = false; update(); }
    bool hasSelectionAabb() const { return m_selectionLocked; }
    QPair<QVector3D,QVector3D> selectionAabb() const { return { m_aabbMin, m_aabbMax }; }

    // 测距模式
    void setMeasurementModeEnabled(bool enabled) { m_measureMode = enabled; if (!enabled) { m_haveP1=false; m_haveP2=false; update(); } }
    bool isMeasurementModeEnabled() const { return m_measureMode; }
    bool hasMeasureP1() const { return m_haveP1; }
    bool hasMeasureP2() const { return m_haveP2; }
    QVector3D getMeasureP1() const { return m_p1; }
    QVector3D getMeasureP2() const { return m_p2; }
    double getMeasureDistance() const { return m_haveP1 && m_haveP2 ? (m_p2 - m_p1).length() : 0.0; }

    // 设置平面投影视角（用于平面投影观察）
    void setTopDownView();

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override; // 新增释放事件处理
    void wheelEvent(QWheelEvent *event) override;

private:
    void setupShaders();
    void setupBuffers();
    void setupAxesBuffers(); // 坐标轴缓冲
    QVector3D mapToArcball(const QPoint& p) const; // Arcball 映射
    bool pickNearestPoint(const QPoint& pos, QVector3D& outWorld, QPoint& outScreen, int pixelRadius = 10);

    QOpenGLShaderProgram *m_program;
    QOpenGLBuffer m_vbo;
    QOpenGLVertexArrayObject m_vao;

    // 坐标轴
    QOpenGLBuffer m_axesVbo;
    QOpenGLVertexArrayObject m_axesVao;

    QMatrix4x4 m_projection;
    QMatrix4x4 m_modelView;

    QVector<Point3D> m_points;
    QMutex m_pointsMutex;

    // 相机控制
    float m_distance;
    QVector3D m_rotation;
    QQuaternion m_orientation;      // 轨迹球当前朝向
    QVector3D m_panOffset;          // 平移偏移
    Qt::MouseButton m_activeButton; // 当前按下按钮
    QPoint m_lastMousePos;
    bool m_mousePressed;

    // 点大小
    float m_pointSize = 2.0f;

    // 图例状态
    bool m_legendVisible = false;
    int m_legendMode = 0;     // 与 MainWindow::ColorMode 一致的索引
    float m_legendMin = 0.0f; // 用于距离/高度
    float m_legendMax = 1.0f;

    // 框选（屏幕实时框）
    bool m_selecting = false;
    QPoint m_selStart;
    QPoint m_selEnd;
    QRect m_selectionRect() const { return QRect(m_selStart, m_selEnd).normalized(); }
    bool m_selectionModeEnabled = false;

    // 持久选择（基于框选时的 MVP 与视口、深度范围）
    bool m_selectionLocked = false;
    QVector3D m_aabbMin; // 向后兼容，现已不用于渲染
    QVector3D m_aabbMax; // 向后兼容，现已不用于渲染
    QMatrix4x4 m_selModelView;
    QMatrix4x4 m_selProjection;
    QRect m_selRectLogical; // 选择时屏幕矩形（逻辑像素）
    int m_selViewportW = 0;
    int m_selViewportH = 0;
    float m_selViewZMin = 0.0f;
    float m_selViewZMax = 0.0f;

    // 测距
    bool m_measureMode = false;
    bool m_haveP1 = false;
    bool m_haveP2 = false;
    QVector3D m_p1;
    QVector3D m_p2;
    QPoint m_p1Screen;
    QPoint m_p2Screen;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    void setupUI();
    void setupLivoxSDK();
    void cleanupLivoxSDK();
    bool runConfigGeneratorDialog();

    // 点云处理
    void processPointCloudPacket(uint32_t handle, const LivoxLidarEthernetPacket* packet);
    uint64_t parseTimestamp(const uint8_t* timestamp);
    void publishPointCloudFrame(const PointCloudFrame& frame);
    void calculatePointColor(uint8_t reflectivity, uint8_t tag, float& r, float& g, float& b);
    QString parseParamValue(uint16_t key, uint8_t* value, uint16_t length);

    // 着色模式
    enum ColorMode {
        ColorByReflectivity = 0,
        ColorByDistance = 1,
        ColorByElevation = 2,
        ColorSolid = 3,
        ColorByPlanarProjection = 4  // 新增：平面投影模式
    };

    // 静态回调函数
    static void onDeviceInfoChange(uint32_t handle, const LivoxLidarInfo* info, void* client_data);
    static void onPointCloudData(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
    static void onImuData(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data);
    static void onStatusInfo(uint32_t handle, uint8_t dev_type, const char* info, void* client_data);
    static void onAsyncControlResponse(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse* response, void* client_data);
    static void onQueryInternalInfoResponse(livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse* response, void* client_data);

    Ui::MainWindow *ui;

    // UI components
    QListWidget* deviceList;
    QTabWidget* paramTabWidget;  // 添加QTabWidget成员变量

    QLabel* statusLabel;
    QTextEdit* logText;
    PointCloudWidget* pointCloudWidget;
    QLabel* statusLabelBar;

    // Docks and toolbar
    QDockWidget* devicesDock;
    QDockWidget* paramsDock;
    QDockWidget* logDock;
    QToolBar* mainToolBar;
    QAction* actionStartSdk;
    QAction* actionStopSdk;
    QAction* actionRefresh;
    QAction* actionClearCloud;
    QAction* actionResetView;
    QAction* actionShowImuCharts = nullptr;

    // Livox SDK related
    bool sdk_initialized;
    bool sdk_started;
    bool shutting_down = false;
    QTimer* updateTimer;
    QTimer* renderTimer;
    QMutex deviceMutex;
    QMap<uint32_t, DeviceInfo> devices;
    DeviceInfo* currentDevice;

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
// GPS RMC 模拟
    QCheckBox* gpsSimulateCheck = nullptr;
    QTimer* gpsTimer = nullptr;
    QLabel* imuGyroLabel = nullptr;
    QLabel* imuAccLabel = nullptr;
    QPushButton* imuDisplayButton = nullptr; // deprecated in UI; kept for backward compatibility

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
    bool savePointCloudAsPCD(const QString& filePath, const QVector<Point3D>& points);

    // LAS 保存
    QString lasSaveDir;           // 目标保存目录（LAS_雷达SN）
    int lasFramesRemaining = 0;   // 待保存帧数
    bool lasSaveActive = false;   // 是否正在保存
    uint64_t lasLastSavedTimestamp = 0; // 上一次已保存的帧时间戳，避免重复
    bool savePointCloudAsLAS(const QString& filePath, const QVector<Point3D>& points);

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
        try {
            return uint8_t((filterTagVal76 & 0x3) << 6 | (filterTagVal54 & 0x3) << 4 | (filterTagVal32 & 0x3) << 2 | (filterTagVal10 & 0x3));
        } catch (...) {
            return 0;
        }
    }
    bool filterTagMatches(uint8_t tag) const {
        try {
            // 只检查滤噪列表中的所有Tag值
            for (uint8_t filterTag : noiseFilterTags) {
                if (filterTag == tag) {
                    return true;
                }
            }

            return false;
        } catch (...) {
            return false;
        }
    }
    // 滤波处理
    QVector<Point3D> applyPointCloudFilters(const QVector<Point3D>& inputPoints);

    // 更新滤噪列表显示
    void updateNoiseFilterList();

    // 设备发现相关
    void startDeviceDiscovery();
    void stopDeviceDiscovery();
    void sendBroadcastDiscovery();
    void onDeviceDiscoveryResponse(const QByteArray& data, const QHostAddress& sender);
    bool updateHostIPForDevice(const QString& deviceIP);
    bool updateConfigFileIP(const QString& newHostIP);
    QString calculateCompatibleHostIP(const QString& deviceIP);
    // void printPacketDetails(const QByteArray& data, const QHostAddress& sender);

    // UDP socket for device discovery
    QUdpSocket* discoverySocket;
    QTimer* discoveryTimer;
    bool discoveryActive;

private slots:
    void onParamQueryTimeout();
    void onParamConfigChanged(uint16_t key);
    void applyIpConfig(uint16_t key, const QString& ip, const QString& mask, const QString& gateway);
    void applyHostIpConfig(uint16_t key, const QString& ip, int port);
    void applyFovConfig(uint16_t key, int yawStart, int yawStop, int pitchStart, int pitchStop);
    void applyAttitudeConfig(uint16_t key, double roll, double pitch, double yaw, int x, int y, int z);
    void onFrameIntervalChanged(int ms);
    void updateFovEnableState(QCheckBox* fov0Check, QCheckBox* fov1Check);
    void onDeviceSelected();
    void updateDeviceList();
    void updateDeviceInfo(const DeviceInfo& device);
    void updateStatus();
    void logMessage(const QString& message);
    void addDeviceToList(const DeviceInfo& device);
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

private:
    // Helpers
    QString buildImuAscii(double gx, double gy, double gz, double ax, double ay, double az) const;
};

#endif // MAINWINDOW_H

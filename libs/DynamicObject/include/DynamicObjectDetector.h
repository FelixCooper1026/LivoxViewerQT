#ifndef DYNAMICOBJECT_DYNAMICOBJECTDETECTOR_H
#define DYNAMICOBJECT_DYNAMICOBJECTDETECTOR_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <QVector>

#include <cstdint>
#include <memory>

class DynamicObjectCluster;

enum class DynamicObjectLabel : uint8_t {
    Static = 0,
    Case1 = 1,
    Case2 = 2,
    Case3 = 3,
    Invalid = 255
};

struct DynamicObjectDetectorConfig {
    double horizontalResolutionRad = 0.005;
    double verticalResolutionRad = 0.01;
    double verticalFovDownDeg = -7.0;
    double verticalFovUpDeg = 52.0;
    double horizontalFovRightDeg = -180.0;
    double horizontalFovLeftDeg = 180.0;
    double minRangeM = 0.3;
    double maxRangeM = 100.0;
    double bufferDelaySec = 0.1;
    double depthMapDurationSec = 0.4;
    double frameDurationSec = 0.02;
    int maxDepthMaps = 5;
    int maxPointsPerPixel = 20;
    int minHistoryMaps = 2;
    int neighborPixelRadius = 1;
    double case1DepthMarginM = 0.5;
    double case2DepthMarginM = 0.3;
    double case3DepthMarginM = 0.15;
    int case1VoteThreshold = 3;
    int case2OcclusionChainLength = 3;
    int case3OcclusionChainLength = 3;
    double case2MinVelocityMps = 0.5;
    double case2AccelerationLimitMps2 = 7.0;
    double case2MapConsistencyDepthM = 0.3;
    double case2MapConsistencyHorizontalRad = 0.02;
    double case2MapConsistencyVerticalRad = 0.06;
    double case2OcclusionHorizontalRad = 0.02;
    double case2OcclusionVerticalRad = 0.06;
    double case2DepthConsistencyMeanM = 0.1;
    double case2DepthConsistencyMaxM = 0.5;
    double case2DepthConsistencyHorizontalRad = 0.03;
    double case2DepthConsistencyVerticalRad = 0.06;
    double case2DepthConsistencyScale = 0.005;
    double case3MinVelocityMps = 0.5;
    double case3AccelerationLimitMps2 = 15.0;
    double case3MapConsistencyDepthM = 0.1;
    double case3MapConsistencyHorizontalRad = 0.03;
    double case3MapConsistencyVerticalRad = 0.03;
    double case3OcclusionHorizontalRad = 0.03;
    double case3OcclusionVerticalRad = 0.03;
    double case3DepthConsistencyMeanM = 0.3;
    double case3DepthConsistencyMaxM = 1.0;
    double case3DepthConsistencyHorizontalRad = 0.03;
    double case3DepthConsistencyVerticalRad = 0.03;
    double case3DepthConsistencyScale = 0.005;
    bool clusterEnabled = false;
    double clusterVoxelSizeM = 0.1;
    int clusterExtendVoxel = 3;
    int clusterMinVoxelCount = 1;
    double clusterTrustThreshold = 0.1;
    double clusterGroundDistanceThresholdM = 0.1;
    double clusterGroundMaxAngleDeg = 30.0;
};

struct DynamicObjectPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float worldX = 0.0f;
    float worldY = 0.0f;
    float worldZ = 0.0f;
    uint8_t reflectivity = 0;
    DynamicObjectLabel label = DynamicObjectLabel::Static;
};

struct DynamicObjectDetectorStats {
    int inputPointCount = 0;
    int staticPointCount = 0;
    int dynamicPointCount = 0;
    int case1PointCount = 0;
    int case2PointCount = 0;
    int case3PointCount = 0;
    int invalidPointCount = 0;
    int historyDepthMapCount = 0;
    int originDynamicPointCount = 0;
    int clusterCount = 0;
    int rejectedClusterCount = 0;
    int groundRemovedPointCount = 0;
    double detectorMs = 0.0;
    double clusterMs = 0.0;
    bool clusterEnabled = false;
};

struct DynamicObjectDetectionFrame {
    int64_t timestampNs = 0;
    Eigen::Quaterniond worldFromBodyRotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d worldFromBodyTranslation = Eigen::Vector3d::Zero();
    QVector<DynamicObjectPoint> points;
};

struct DynamicObjectDetectionResult {
    QVector<DynamicObjectPoint> originDynamicPoints;
    QVector<DynamicObjectPoint> dynamicPoints;
    QVector<DynamicObjectLabel> originLabels;
    QVector<DynamicObjectLabel> clusterLabels;
    DynamicObjectDetectorStats stats;
};

class DynamicObjectDetector
{
public:
    explicit DynamicObjectDetector(const DynamicObjectDetectorConfig& config = DynamicObjectDetectorConfig());
    ~DynamicObjectDetector();

    void reset();
    DynamicObjectDetectionResult processFrame(const DynamicObjectDetectionFrame& frame);

private:
    struct PixelProjection {
        int x = 0;
        int y = 0;
        float azimuthRad = 0.0f;
        float elevationRad = 0.0f;
        float depth = 0.0f;
    };

    struct HistoricalPoint {
        Eigen::Vector3d worldPoint = Eigen::Vector3d::Zero();
        int64_t timestampNs = 0;
        DynamicObjectLabel label = DynamicObjectLabel::Static;
        int depthMapId = -1;
        PixelProjection depthMapProjection;
        std::weak_ptr<HistoricalPoint> case2Predecessor;
        int case2PredecessorMapId = -1;
        PixelProjection case2QueryProjection;
        std::weak_ptr<HistoricalPoint> case3Predecessor;
        int case3PredecessorMapId = -1;
        PixelProjection case3QueryProjection;
    };

    struct DepthPixel {
        float minStaticDepth = 0.0f;
        float maxStaticDepth = 0.0f;
        float minAnyDepth = 0.0f;
        float maxAnyDepth = 0.0f;
        bool hasStatic = false;
        bool hasAny = false;
        QVector<std::shared_ptr<HistoricalPoint>> points;
        std::shared_ptr<HistoricalPoint> minAnyPoint;
        std::shared_ptr<HistoricalPoint> maxAnyPoint;
    };

    struct DepthMap {
        int id = -1;
        int64_t timestampNs = 0;
        Eigen::Quaterniond worldFromBodyRotation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d worldFromBodyTranslation = Eigen::Vector3d::Zero();
        QVector<DepthPixel> pixels;
    };

    struct PixelStats {
        float minStaticDepth = 0.0f;
        float maxStaticDepth = 0.0f;
        float minAnyDepth = 0.0f;
        float maxAnyDepth = 0.0f;
        bool hasStatic = false;
        bool hasAny = false;
    };

    struct PointState {
        DynamicObjectPoint point;
        PixelProjection currentProjection;
        std::shared_ptr<HistoricalPoint> historicalPoint;
        DynamicObjectLabel label = DynamicObjectLabel::Static;
        bool valid = false;
    };

    struct BufferedFrame {
        int64_t timestampNs = 0;
        Eigen::Quaterniond worldFromBodyRotation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d worldFromBodyTranslation = Eigen::Vector3d::Zero();
        QVector<PointState> states;
    };

    bool projectBodyPoint(const Eigen::Vector3d& point, PixelProjection* projection) const;
    bool projectWorldPoint(const Eigen::Vector3d& point,
                           const DepthMap& map,
                           PixelProjection* projection) const;
    bool gatherPixelStats(const DepthMap& map, int x, int y, PixelStats* stats) const;
    QVector<std::shared_ptr<HistoricalPoint>> gatherNeighborPoints(
        const DepthMap& map,
        const PixelProjection& projection,
        double horizontalThresholdRad,
        double verticalThresholdRad) const;
    bool detectCase2(const std::shared_ptr<HistoricalPoint>& point);
    bool detectCase3(const std::shared_ptr<HistoricalPoint>& point);
    bool case2Enter(const HistoricalPoint& point,
                    const PixelProjection& projection,
                    const DepthMap& map) const;
    bool case3Enter(const HistoricalPoint& point,
                    const PixelProjection& projection,
                    const DepthMap& map) const;
    bool case2MapConsistent(const HistoricalPoint& point,
                            const PixelProjection& projection,
                            const DepthMap& map) const;
    bool case3MapConsistent(const HistoricalPoint& point,
                            const PixelProjection& projection,
                            const DepthMap& map) const;
    bool case2DepthConsistent(const HistoricalPoint& point, const DepthMap& map) const;
    bool case3DepthConsistent(const HistoricalPoint& point, const DepthMap& map) const;
    bool case2IsOccluded(const HistoricalPoint& point,
                         const PixelProjection& projection,
                         const HistoricalPoint& occluder) const;
    bool case3IsOccluding(const HistoricalPoint& point,
                          const PixelProjection& projection,
                          const HistoricalPoint& occludedPoint) const;
    bool findCase2Occluder(const HistoricalPoint& point,
                           const DepthMap& map,
                           std::shared_ptr<HistoricalPoint>* occluder,
                           PixelProjection* queryProjection) const;
    bool findCase3OccludedPoint(const HistoricalPoint& point,
                                const DepthMap& map,
                                std::shared_ptr<HistoricalPoint>* occludedPoint,
                                PixelProjection* queryProjection) const;
    int depthMapIndex(int id) const;
    DepthMap buildDepthMap(const DynamicObjectDetectionFrame& frame,
                           const QVector<PointState>& states,
                           int mapId) const;
    void mergeBufferedFrame(DepthMap& map, const BufferedFrame& frame) const;
    void appendProjectedPoint(DepthMap& map,
                              const PixelProjection& projection,
                              const std::shared_ptr<HistoricalPoint>& point) const;
    void promoteBufferedFrames(int64_t currentTimestampNs);
    void appendDepthMap(DepthMap&& map);
    int pixelIndex(int x, int y) const;

    DynamicObjectDetectorConfig config_;
    int imageWidth_ = 0;
    int imageHeight_ = 0;
    std::unique_ptr<DynamicObjectCluster> cluster_;
    QVector<BufferedFrame> bufferedFrames_;
    QVector<DepthMap> depthMaps_;
    int nextDepthMapId_ = 0;
};

#endif // DYNAMICOBJECT_DYNAMICOBJECTDETECTOR_H

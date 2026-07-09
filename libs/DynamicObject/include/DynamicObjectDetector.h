#ifndef DYNAMICOBJECT_DYNAMICOBJECTDETECTOR_H
#define DYNAMICOBJECT_DYNAMICOBJECTDETECTOR_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <QVector>

#include <cstdint>

enum class DynamicObjectLabel : uint8_t {
    Static = 0,
    Case1 = 1,
    Case2 = 2,
    Case3 = 3,
    Invalid = 255
};

struct DynamicObjectDetectorConfig {
    int imageWidth = 2048;
    int imageHeight = 128;
    double verticalFovDownDeg = -45.0;
    double verticalFovUpDeg = 45.0;
    double minRangeM = 0.5;
    double maxRangeM = 100.0;
    int maxDepthMaps = 12;
    int minHistoryMaps = 2;
    int neighborPixelRadius = 1;
    double case1DepthMarginM = 0.25;
    double case2DepthMarginM = 0.5;
    double case3DepthMarginM = 0.35;
    int case1VoteThreshold = 2;
    int case2VoteThreshold = 2;
    int case3VoteThreshold = 2;
    bool clusterEnabled = false;
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
    double detectorMs = 0.0;
    bool clusterEnabled = false;
};

struct DynamicObjectDetectionFrame {
    int64_t timestampNs = 0;
    Eigen::Quaterniond worldFromBodyRotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d worldFromBodyTranslation = Eigen::Vector3d::Zero();
    QVector<DynamicObjectPoint> points;
};

struct DynamicObjectDetectionResult {
    QVector<DynamicObjectPoint> dynamicPoints;
    DynamicObjectDetectorStats stats;
};

class DynamicObjectDetector
{
public:
    explicit DynamicObjectDetector(const DynamicObjectDetectorConfig& config = DynamicObjectDetectorConfig());

    void reset();
    DynamicObjectDetectionResult processFrame(const DynamicObjectDetectionFrame& frame);

private:
    struct DepthPixel {
        float minStaticDepth = 0.0f;
        float maxStaticDepth = 0.0f;
        float minAnyDepth = 0.0f;
        float maxAnyDepth = 0.0f;
        bool hasStatic = false;
        bool hasAny = false;
    };

    struct DepthMap {
        int64_t timestampNs = 0;
        Eigen::Quaterniond worldFromBodyRotation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d worldFromBodyTranslation = Eigen::Vector3d::Zero();
        QVector<DepthPixel> pixels;
    };

    struct PixelProjection {
        int x = 0;
        int y = 0;
        float depth = 0.0f;
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
        DynamicObjectLabel label = DynamicObjectLabel::Static;
        bool valid = false;
    };

    bool projectBodyPoint(const Eigen::Vector3d& point, PixelProjection* projection) const;
    bool gatherPixelStats(const DepthMap& map, int x, int y, PixelStats* stats) const;
    DepthMap buildDepthMap(const DynamicObjectDetectionFrame& frame, const QVector<PointState>& states) const;
    void appendDepthMap(DepthMap&& map);
    int pixelIndex(int x, int y) const;

    DynamicObjectDetectorConfig config_;
    QVector<DepthMap> depthMaps_;
};

#endif // DYNAMICOBJECT_DYNAMICOBJECTDETECTOR_H

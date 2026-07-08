#include "PointCloudFilter.h"

namespace PointCloudFilter {

QVector<PointCloudPoint> apply(const QVector<PointCloudPoint>& inputPoints, const Config& config)
{
    if (inputPoints.isEmpty() || (!config.showNoisePoints && !config.removeNoisePoints)) {
        return inputPoints;
    }

    QVector<PointCloudPoint> processedPoints;
    processedPoints.reserve(inputPoints.size());
    for (const PointCloudPoint& point : inputPoints) {
        const bool isNoise = config.noiseTags.contains(point.tag);
        PointCloudPoint processedPoint = point;

        if (config.showNoisePoints && isNoise) {
            processedPoint.r = 1.0f;
            processedPoint.g = 0.0f;
            processedPoint.b = 0.0f;
        }

        if (!config.removeNoisePoints || !isNoise) {
            processedPoints.append(processedPoint);
        }
    }

    return processedPoints;
}

} // namespace PointCloudFilter

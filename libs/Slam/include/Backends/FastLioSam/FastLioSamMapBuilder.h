#ifndef SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMMAPBUILDER_H
#define SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMMAPBUILDER_H

#include "Backends/FastLioSam/FastLioSamBackendState.h"

#include <Eigen/Geometry>

Eigen::Affine3f pclPointToAffine3f(const PointTypePose& point);
Eigen::Affine3f trans2Affine3f(const float transform[6]);
PointCloudXYZI::Ptr transformPointCloud(FastLioAlgorithmState& state,
                                       const PointCloudXYZI::Ptr& cloud,
                                       const PointTypePose* pose);
float pointDistance(const PointType& lhs, const PointType& rhs);
void reconstructIKdTree(FastLioAlgorithmState& state);
void appendOptimizedKeyframeMap(FastLioAlgorithmState& state,
                                const PointCloudXYZI::Ptr& keyframe,
                                const PointTypePose& pose);
QVector<SlamPoint> buildOptimizedGlobalMap(FastLioAlgorithmState& state);

#endif // SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMMAPBUILDER_H

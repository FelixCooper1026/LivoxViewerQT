#ifndef SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMPOSEGRAPH_H
#define SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMPOSEGRAPH_H

#include "Backends/FastLioSam/FastLioSamBackendState.h"

gtsam::Pose3 pclPointTogtsamPose3(const PointTypePose& point);
gtsam::Pose3 trans2gtsamPose(const float transform[6]);
void getCurPose(FastLioAlgorithmState& state);
bool saveFrame(FastLioAlgorithmState& state);
void addOdomFactor(FastLioAlgorithmState& state);
void addLoopFactor(FastLioAlgorithmState& state);
void saveKeyFramesAndFactor(FastLioAlgorithmState& state);
void saveFinalKeyFrameAndFactor(FastLioAlgorithmState& state);
void correctPoses(FastLioAlgorithmState& state);
void updateOptimizedPath(FastLioAlgorithmState& state, const PointTypePose& pose);
void appendFastLioSamOutput(FastLioAlgorithmState& state, SlamOutput* output);
bool flushPendingLoopFactors(FastLioAlgorithmState& state);

#endif // SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMPOSEGRAPH_H

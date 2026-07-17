#ifndef SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMLOOPCLOSURE_H
#define SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMLOOPCLOSURE_H

#include "Backends/FastLioSam/FastLioSamBackendState.h"

bool detectLoopClosureDistance(FastLioAlgorithmState& state, int* latestId, int* closestId);
void loopFindNearKeyframes(FastLioAlgorithmState& state,
                           PointCloudXYZI::Ptr& nearKeyframes,
                           int key,
                           int searchNum);
void performLoopClosure(FastLioAlgorithmState& state);
void updateLoopClosureVisualization(FastLioAlgorithmState& state);
void loopClosureThread(FastLioAlgorithmState* state);
void startLoopClosureThread(FastLioAlgorithmState& state);
void stopLoopClosureThread(FastLioAlgorithmState& state);

#endif // SLAM_BACKENDS_FASTLIOSAM_FASTLIOSAMLOOPCLOSURE_H

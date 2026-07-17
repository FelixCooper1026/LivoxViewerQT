#include "Backends/FastLioSam/FastLioSamLoopClosure.h"

#include "Backends/FastLioSam/FastLioSamMapBuilder.h"
#include "Backends/FastLioSam/FastLioSamPoseGraph.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include <chrono>
#include <cmath>

bool detectLoopClosureDistance(FastLioAlgorithmState& state, int* latestId, int* closestId)
{
    auto& sam = state.sam;
    const int loopKeyCur = static_cast<int>(sam.copyCloudKeyPoses3D->size()) - 1;
    int loopKeyPre = -1;

    {
        std::lock_guard<std::mutex> lock(sam.loopInfoMutex);
        if (sam.loopIndexContainer.find(loopKeyCur) != sam.loopIndexContainer.end()) {
            return false;
        }
    }

    std::vector<int> candidateIndices;
    std::vector<float> candidateSqDistances;
    sam.kdtreeHistoryKeyPoses->setInputCloud(sam.copyCloudKeyPoses3D);
    sam.kdtreeHistoryKeyPoses->radiusSearch(
        sam.copyCloudKeyPoses3D->back(),
        state.config.historyKeyframeSearchRadius,
        candidateIndices,
        candidateSqDistances,
        0);

    for (const int id : candidateIndices) {
        if (std::abs(sam.copyCloudKeyPoses6D->points[static_cast<std::size_t>(id)].time -
                     sam.copyCloudKeyPoses6D->back().time) >
            state.config.historyKeyframeSearchTimeDiff) {
            loopKeyPre = id;
            break;
        }
    }
    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre) {
        return false;
    }

    *latestId = loopKeyCur;
    *closestId = loopKeyPre;
    return true;
}

void loopFindNearKeyframes(FastLioAlgorithmState& state,
                           PointCloudXYZI::Ptr& nearKeyframes,
                           int key,
                           int searchNum)
{
    auto& sam = state.sam;
    nearKeyframes->clear();
    const int poseCount = static_cast<int>(sam.copyCloudKeyPoses6D->size());
    struct KeyframePose {
        PointCloudXYZI::Ptr cloud;
        PointTypePose pose;
    };
    std::vector<KeyframePose> keyframes;
    std::size_t pointCount = 0;

    {
        std::lock_guard<std::mutex> lock(sam.poseMutex);
        for (int offset = -searchNum; offset <= searchNum; ++offset) {
            const int keyNear = key + offset;
            const int cloudCount = static_cast<int>(sam.surfCloudKeyFrames.size());
            if (keyNear < 0 || keyNear >= poseCount || keyNear >= cloudCount) {
                continue;
            }
            const PointCloudXYZI::Ptr& keyframe =
                sam.surfCloudKeyFrames[static_cast<std::size_t>(keyNear)];
            pointCount += keyframe->size();
            keyframes.push_back(
                {keyframe, sam.copyCloudKeyPoses6D->points[static_cast<std::size_t>(keyNear)]});
        }
    }

    nearKeyframes->reserve(pointCount);
    for (const KeyframePose& keyframe : keyframes) {
        *nearKeyframes += *transformPointCloud(
            state,
            keyframe.cloud,
            &keyframe.pose);
    }

    if (nearKeyframes->empty()) {
        return;
    }
    PointCloudXYZI::Ptr cloudTemp(new PointCloudXYZI());
    sam.downSizeFilterICP.setInputCloud(nearKeyframes);
    sam.downSizeFilterICP.filter(*cloudTemp);
    *nearKeyframes = *cloudTemp;
}

void performLoopClosure(FastLioAlgorithmState& state)
{
    auto& sam = state.sam;
    {
        std::lock_guard<std::mutex> lock(sam.poseMutex);
        if (sam.cloudKeyPoses3D->empty()) {
            return;
        }
        *sam.copyCloudKeyPoses3D = *sam.cloudKeyPoses3D;
        *sam.copyCloudKeyPoses6D = *sam.cloudKeyPoses6D;
    }

    int loopKeyCur = -1;
    int loopKeyPre = -1;
    if (!detectLoopClosureDistance(state, &loopKeyCur, &loopKeyPre)) {
        return;
    }

    PointCloudXYZI::Ptr currentKeyframeCloud(new PointCloudXYZI());
    PointCloudXYZI::Ptr previousKeyframeCloud(new PointCloudXYZI());
    loopFindNearKeyframes(state, currentKeyframeCloud, loopKeyCur, 0);
    loopFindNearKeyframes(
        state, previousKeyframeCloud, loopKeyPre, state.config.historyKeyframeSearchNum);
    if (currentKeyframeCloud->empty() || previousKeyframeCloud->empty()) {
        return;
    }

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setInputSource(currentKeyframeCloud);
    icp.setInputTarget(previousKeyframeCloud);
    PointCloudXYZI::Ptr unusedResult(new PointCloudXYZI());
    icp.align(*unusedResult);

    if (!icp.hasConverged() ||
        icp.getFitnessScore() > state.config.historyKeyframeFitnessScore) {
        return;
    }

    const Eigen::Affine3f correctionLidarFrame(icp.getFinalTransformation());
    const Eigen::Affine3f tWrong = pclPointToAffine3f(
        sam.copyCloudKeyPoses6D->points[static_cast<std::size_t>(loopKeyCur)]);
    const Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;

    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    const gtsam::Pose3 poseFrom(
        gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    const gtsam::Pose3 poseTo = pclPointTogtsamPose3(
        sam.copyCloudKeyPoses6D->points[static_cast<std::size_t>(loopKeyPre)]);

    const float noiseScore = static_cast<float>(icp.getFitnessScore());
    gtsam::Vector vector6(6);
    vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    auto constraintNoise = gtsam::noiseModel::Diagonal::Variances(vector6);

    {
        std::lock_guard<std::mutex> lock(sam.loopInfoMutex);
        sam.loopIndexQueue.emplace_back(loopKeyCur, loopKeyPre);
        sam.loopPoseQueue.push_back(poseFrom.between(poseTo));
        sam.loopNoiseQueue.push_back(constraintNoise);
        sam.loopIndexContainer[loopKeyCur] = loopKeyPre;
        sam.pendingLoopClosureEdges.push_back({loopKeyCur, loopKeyPre});
    }
}

bool performDeterministicLoopClosure(FastLioAlgorithmState& state, bool forceLatest)
{
    auto& sam = state.sam;
    int latestId = -1;
    double latestTime = 0.0;
    {
        std::lock_guard<std::mutex> lock(sam.poseMutex);
        if (sam.cloudKeyPoses6D->empty()) {
            return false;
        }
        latestId = static_cast<int>(sam.cloudKeyPoses6D->size()) - 1;
        latestTime = sam.cloudKeyPoses6D->back().time;
    }
    if (latestId == sam.lastDeterministicLoopKeyframeId) {
        return false;
    }

    const double periodSec = 1.0 / state.config.loopClosureFrequency;
    if (!forceLatest && sam.lastDeterministicLoopKeyframeId >= 0 &&
        latestTime - sam.lastDeterministicLoopTime < periodSec) {
        return false;
    }

    sam.lastDeterministicLoopKeyframeId = latestId;
    sam.lastDeterministicLoopTime = latestTime;
    performLoopClosure(state);
    return true;
}

void updateLoopClosureVisualization(FastLioAlgorithmState& state)
{
    static_cast<void>(state);
}

void loopClosureThread(FastLioAlgorithmState* state)
{
    if (!state->config.loopClosureEnableFlag) {
        return;
    }
    const double periodSec = 1.0 / state->config.loopClosureFrequency;
    std::unique_lock<std::mutex> lock(state->sam.loopThreadMutex);
    while (!state->sam.loopThreadCondition.wait_for(
        lock,
        std::chrono::duration<double>(periodSec),
        [state]() { return state->sam.stopRequested.load(); })) {
        lock.unlock();
        performLoopClosure(*state);
        updateLoopClosureVisualization(*state);
        lock.lock();
    }
}

void startLoopClosureThread(FastLioAlgorithmState& state)
{
    if (!state.config.loopClosureEnableFlag || state.config.deterministicOfflineLoopClosure) {
        return;
    }
    state.sam.stopRequested.store(false);
    state.sam.loopThread = std::thread(loopClosureThread, &state);
}

void stopLoopClosureThread(FastLioAlgorithmState& state)
{
    state.sam.stopRequested.store(true);
    state.sam.loopThreadCondition.notify_one();
    if (state.sam.loopThread.joinable()) {
        state.sam.loopThread.join();
    }
}

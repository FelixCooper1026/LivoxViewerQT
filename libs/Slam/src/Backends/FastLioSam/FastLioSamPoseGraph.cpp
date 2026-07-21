#include "Backends/FastLioSam/FastLioSamPoseGraph.h"

#include "Backends/FastLioSam/FastLioSamMapBuilder.h"

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include <cmath>

namespace {

Eigen::Quaterniond eulerToQuat(float roll, float pitch, float yaw)
{
    Eigen::Quaterniond quaternion =
        Eigen::AngleAxisd(double(yaw), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(double(pitch), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(double(roll), Eigen::Vector3d::UnitX());
    quaternion.normalize();
    return quaternion;
}

SlamTrajectoryPoint optimizedTrajectoryPoint(const PointTypePose& pose)
{
    SlamTrajectoryPoint point;
    point.pose.timestampNs = static_cast<int64_t>(pose.time * 1.0e9);
    point.pose.tx = pose.x;
    point.pose.ty = pose.y;
    point.pose.tz = pose.z;
    const Eigen::Quaterniond quaternion = eulerToQuat(pose.roll, pose.pitch, pose.yaw);
    point.pose.qx = quaternion.x();
    point.pose.qy = quaternion.y();
    point.pose.qz = quaternion.z();
    point.pose.qw = quaternion.w();
    point.pose.poseFrame = QStringLiteral("slam_world");
    return point;
}

void updateIsamWithQueuedFactors(FastLioAlgorithmState& state)
{
    auto& sam = state.sam;
    sam.isam->update(sam.gtSAMgraph, sam.initialEstimate);
    sam.isam->update();
    if (sam.aLoopIsClosed) {
        sam.isam->update();
        sam.isam->update();
        sam.isam->update();
        sam.isam->update();
        sam.isam->update();
    }
    sam.gtSAMgraph.resize(0);
    sam.initialEstimate.clear();
    sam.isamCurrentEstimate = sam.isam->calculateBestEstimate();
}

} // namespace

FastLioSamBackendState::FastLioSamBackendState()
    : cloudKeyPoses3D(new pcl::PointCloud<PointType>()),
      cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()),
      copyCloudKeyPoses3D(new pcl::PointCloud<PointType>()),
      copyCloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()),
      fastlioUnoptimizedCloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()),
      kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<PointType>()),
      kdtreeSurroundingKeyPoses(new pcl::KdTreeFLANN<PointType>())
{
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = std::make_unique<gtsam::ISAM2>(parameters);
}

gtsam::Pose3 pclPointTogtsamPose3(const PointTypePose& point)
{
    return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(double(point.roll), double(point.pitch), double(point.yaw)),
        gtsam::Point3(double(point.x), double(point.y), double(point.z)));
}

gtsam::Pose3 trans2gtsamPose(const float transform[6])
{
    return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(transform[0], transform[1], transform[2]),
        gtsam::Point3(transform[3], transform[4], transform[5]));
}

void getCurPose(FastLioAlgorithmState& state)
{
    const Eigen::Vector3d eulerAngle = state.statePoint.rot.matrix().eulerAngles(2, 1, 0);
    state.sam.transformTobeMapped[0] = static_cast<float>(eulerAngle(2));
    state.sam.transformTobeMapped[1] = static_cast<float>(eulerAngle(1));
    state.sam.transformTobeMapped[2] = static_cast<float>(eulerAngle(0));
    state.sam.transformTobeMapped[3] = static_cast<float>(state.statePoint.pos(0));
    state.sam.transformTobeMapped[4] = static_cast<float>(state.statePoint.pos(1));
    state.sam.transformTobeMapped[5] = static_cast<float>(state.statePoint.pos(2));
}

bool saveFrame(FastLioAlgorithmState& state)
{
    auto& sam = state.sam;
    if (sam.cloudKeyPoses3D->empty()) {
        return true;
    }

    const Eigen::Affine3f transStart = pclPointToAffine3f(sam.cloudKeyPoses6D->back());
    const Eigen::Affine3f transFinal = trans2Affine3f(sam.transformTobeMapped);
    const Eigen::Affine3f transBetween = transStart.inverse() * transFinal;

    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
    if (std::abs(roll) < state.config.surroundingKeyframeAddingAngleThreshold &&
        std::abs(pitch) < state.config.surroundingKeyframeAddingAngleThreshold &&
        std::abs(yaw) < state.config.surroundingKeyframeAddingAngleThreshold &&
        std::sqrt(x * x + y * y + z * z) < state.config.surroundingKeyframeAddingDistThreshold) {
        return false;
    }
    return true;
}

void addOdomFactor(FastLioAlgorithmState& state)
{
    auto& sam = state.sam;
    if (sam.cloudKeyPoses3D->empty()) {
        auto priorNoise = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());
        sam.gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(
            0, trans2gtsamPose(sam.transformTobeMapped), priorNoise));
        sam.initialEstimate.insert(0, trans2gtsamPose(sam.transformTobeMapped));
        return;
    }

    auto odometryNoise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    const gtsam::Pose3 poseFrom = pclPointTogtsamPose3(sam.cloudKeyPoses6D->back());
    const gtsam::Pose3 poseTo = trans2gtsamPose(sam.transformTobeMapped);
    const std::size_t currentIndex = sam.cloudKeyPoses3D->size();
    sam.gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        currentIndex - 1, currentIndex, poseFrom.between(poseTo), odometryNoise));
    sam.initialEstimate.insert(currentIndex, poseTo);
}

void addLoopFactor(FastLioAlgorithmState& state)
{
    auto& sam = state.sam;
    std::lock_guard<std::mutex> lock(sam.loopInfoMutex);
    if (sam.loopIndexQueue.empty()) {
        return;
    }
    for (std::size_t i = 0; i < sam.loopIndexQueue.size(); ++i) {
        const int indexFrom = sam.loopIndexQueue[i].first;
        const int indexTo = sam.loopIndexQueue[i].second;
        sam.gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            indexFrom, indexTo, sam.loopPoseQueue[i], sam.loopNoiseQueue[i]));
    }
    sam.loopIndexQueue.clear();
    sam.loopPoseQueue.clear();
    sam.loopNoiseQueue.clear();
    sam.aLoopIsClosed = true;
}

void updateOptimizedPath(FastLioAlgorithmState& state, const PointTypePose& pose)
{
    const SlamTrajectoryPoint point = optimizedTrajectoryPoint(pose);
    state.optimizedPath.push_back(point);
    if (!state.config.deterministicOfflineLoopClosure) {
        state.pendingOptimizedTrajectory.push_back(point);
    }
}

namespace {

void saveCurrentFrameAndFactor(FastLioAlgorithmState& state)
{
    auto& sam = state.sam;

    addOdomFactor(state);
    addLoopFactor(state);
    updateIsamWithQueuedFactors(state);

    const gtsam::Pose3 latestEstimate = sam.isamCurrentEstimate.at<gtsam::Pose3>(
        sam.isamCurrentEstimate.size() - 1);
    PointType thisPose3D;
    thisPose3D.x = static_cast<float>(latestEstimate.translation().x());
    thisPose3D.y = static_cast<float>(latestEstimate.translation().y());
    thisPose3D.z = static_cast<float>(latestEstimate.translation().z());
    thisPose3D.intensity = static_cast<float>(sam.cloudKeyPoses3D->size());

    PointTypePose thisPose6D;
    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;
    thisPose6D.roll = static_cast<float>(latestEstimate.rotation().roll());
    thisPose6D.pitch = static_cast<float>(latestEstimate.rotation().pitch());
    thisPose6D.yaw = static_cast<float>(latestEstimate.rotation().yaw());
    thisPose6D.time = state.measures.lidar_end_time;

    state_ikfom stateUpdated = state.kf.get_x();
    stateUpdated.pos = Eigen::Vector3d(latestEstimate.translation().x(),
                                       latestEstimate.translation().y(),
                                       latestEstimate.translation().z());
    stateUpdated.rot = eulerToQuat(thisPose6D.roll, thisPose6D.pitch, thisPose6D.yaw);
    state.statePoint = stateUpdated;
    state.kf.change_x(stateUpdated);

    PointCloudXYZI::Ptr thisSurfKeyFrame(new PointCloudXYZI());
    pcl::VoxelGrid<PointType> keyframeFilter;
    keyframeFilter.setLeafSize(state.config.mappingSurfLeafSize,
                               state.config.mappingSurfLeafSize,
                               state.config.mappingSurfLeafSize);
    keyframeFilter.setInputCloud(state.featsUndistort);
    keyframeFilter.filter(*thisSurfKeyFrame);
    {
        std::lock_guard<std::mutex> lock(sam.poseMutex);
        sam.cloudKeyPoses3D->push_back(thisPose3D);
        sam.cloudKeyPoses6D->push_back(thisPose6D);
        sam.surfCloudKeyFrames.push_back(thisSurfKeyFrame);
    }

    updateOptimizedPath(state, thisPose6D);
}

} // namespace

void saveKeyFramesAndFactor(FastLioAlgorithmState& state)
{
    if (!saveFrame(state)) {
        return;
    }
    saveCurrentFrameAndFactor(state);
}

void saveFinalKeyFrameAndFactor(FastLioAlgorithmState& state)
{
    if (!state.sam.cloudKeyPoses6D->empty() &&
        state.sam.cloudKeyPoses6D->back().time == state.measures.lidar_end_time) {
        return;
    }
    saveCurrentFrameAndFactor(state);
}

void correctPoses(FastLioAlgorithmState& state)
{
    auto& sam = state.sam;
    if (sam.cloudKeyPoses3D->empty() || !sam.aLoopIsClosed) {
        return;
    }

    const bool publishOptimization = !state.config.deterministicOfflineLoopClosure;
    const gtsam::Pose3 latestPoseBeforeOptimization =
        pclPointTogtsamPose3(sam.cloudKeyPoses6D->back());
    const gtsam::Pose3 currentPoseBeforeOptimization = trans2gtsamPose(sam.transformTobeMapped);
    state.optimizedPath.clear();
    state.pendingOptimizedTrajectory.clear();
    state.optimizedTrajectoryResetPending = publishOptimization;
    state.pendingOptimizedGlobalMapPoints.clear();
    state.optimizedGlobalMapResetPending = publishOptimization;

    const int numPoses = static_cast<int>(sam.isamCurrentEstimate.size());
    {
        std::lock_guard<std::mutex> lock(sam.poseMutex);
        for (int i = 0; i < numPoses; ++i) {
            const gtsam::Pose3 pose = sam.isamCurrentEstimate.at<gtsam::Pose3>(i);
            sam.cloudKeyPoses3D->points[static_cast<std::size_t>(i)].x =
                static_cast<float>(pose.translation().x());
            sam.cloudKeyPoses3D->points[static_cast<std::size_t>(i)].y =
                static_cast<float>(pose.translation().y());
            sam.cloudKeyPoses3D->points[static_cast<std::size_t>(i)].z =
                static_cast<float>(pose.translation().z());

            PointTypePose& pose6D = sam.cloudKeyPoses6D->points[static_cast<std::size_t>(i)];
            pose6D.x = sam.cloudKeyPoses3D->points[static_cast<std::size_t>(i)].x;
            pose6D.y = sam.cloudKeyPoses3D->points[static_cast<std::size_t>(i)].y;
            pose6D.z = sam.cloudKeyPoses3D->points[static_cast<std::size_t>(i)].z;
            pose6D.roll = static_cast<float>(pose.rotation().roll());
            pose6D.pitch = static_cast<float>(pose.rotation().pitch());
            pose6D.yaw = static_cast<float>(pose.rotation().yaw());
            updateOptimizedPath(state, pose6D);
        }
    }

    const gtsam::Pose3 latestPose =
        sam.isamCurrentEstimate.at<gtsam::Pose3>(sam.isamCurrentEstimate.size() - 1);
    const gtsam::Pose3 correctedCurrentPose =
        latestPose.compose(latestPoseBeforeOptimization.inverse()).compose(
            currentPoseBeforeOptimization);
    state_ikfom stateUpdated = state.kf.get_x();
    stateUpdated.pos = Eigen::Vector3d(correctedCurrentPose.translation().x(),
                                       correctedCurrentPose.translation().y(),
                                       correctedCurrentPose.translation().z());
    stateUpdated.rot = eulerToQuat(
        static_cast<float>(correctedCurrentPose.rotation().roll()),
        static_cast<float>(correctedCurrentPose.rotation().pitch()),
        static_cast<float>(correctedCurrentPose.rotation().yaw()));
    state.statePoint = stateUpdated;
    state.kf.change_x(stateUpdated);
    state.eulerCur = SO3ToEuler(state.statePoint.rot);
    state.posLid = state.statePoint.pos + state.statePoint.rot * state.statePoint.offset_T_L_I;
    ++state.poseCorrectionEpoch;

    if (!publishOptimization) {
        state.pendingOptimizedTrajectory.clear();
        state.pendingOptimizedGlobalMapPoints.clear();
    }

    if (!state.finalizing) {
        reconstructIKdTree(state);
    }
    sam.aLoopIsClosed = false;
}

bool flushPendingLoopFactors(FastLioAlgorithmState& state)
{
    {
        std::lock_guard<std::mutex> lock(state.sam.loopInfoMutex);
        if (state.sam.loopIndexQueue.empty()) {
            return false;
        }
    }
    addLoopFactor(state);
    updateIsamWithQueuedFactors(state);
    correctPoses(state);
    return true;
}

void appendFastLioSamOutput(FastLioAlgorithmState& state, SlamOutput* output)
{
    if (output == nullptr) {
        return;
    }
    output->optimizedTrajectoryReset = state.optimizedTrajectoryResetPending;
    output->optimizedTrajectory = state.pendingOptimizedTrajectory;
    state.optimizedTrajectoryResetPending = false;
    state.pendingOptimizedTrajectory.clear();

    output->optimizedGlobalMapReset = state.optimizedGlobalMapResetPending;
    output->optimizedGlobalMapPoints = state.optimizedGlobalMapResetPending
        ? buildOptimizedGlobalMap(state)
        : state.pendingOptimizedGlobalMapPoints;
    state.optimizedGlobalMapResetPending = false;
    state.pendingOptimizedGlobalMapPoints.clear();

    output->keyframeCount = static_cast<int>(state.sam.cloudKeyPoses3D->size());
    {
        std::lock_guard<std::mutex> lock(state.sam.loopInfoMutex);
        output->loopClosureEdges = state.sam.pendingLoopClosureEdges;
        state.sam.pendingLoopClosureEdges.clear();
        output->loopClosureCount = static_cast<int>(state.sam.loopIndexContainer.size());
    }
}

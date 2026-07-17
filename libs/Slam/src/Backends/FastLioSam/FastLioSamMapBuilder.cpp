#include "Backends/FastLioSam/FastLioSamMapBuilder.h"

#include <pcl/common/transforms.h>

#include <algorithm>
#include <cmath>

namespace {

QVector<SlamPoint> toSlamPoints(const PointCloudXYZI& cloud)
{
    QVector<SlamPoint> points;
    points.reserve(static_cast<int>(cloud.points.size()));
    for (const PointType& point : cloud.points) {
        SlamPoint outputPoint;
        outputPoint.x = point.x;
        outputPoint.y = point.y;
        outputPoint.z = point.z;
        outputPoint.reflectivity = static_cast<uint8_t>(
            std::clamp(point.intensity, 0.0f, 255.0f));
        points.push_back(outputPoint);
    }
    return points;
}

PointCloudXYZI::Ptr downsampleCloud(const PointCloudXYZI::Ptr& cloud, float leafSize)
{
    PointCloudXYZI::Ptr downsampled(new PointCloudXYZI());
    pcl::VoxelGrid<PointType> filter;
    filter.setLeafSize(leafSize, leafSize, leafSize);
    filter.setInputCloud(cloud);
    filter.filter(*downsampled);
    return downsampled;
}

} // namespace

Eigen::Affine3f pclPointToAffine3f(const PointTypePose& point)
{
    return pcl::getTransformation(
        point.x, point.y, point.z, point.roll, point.pitch, point.yaw);
}

Eigen::Affine3f trans2Affine3f(const float transform[6])
{
    return pcl::getTransformation(
        transform[3], transform[4], transform[5], transform[0], transform[1], transform[2]);
}

PointCloudXYZI::Ptr transformPointCloud(FastLioAlgorithmState& state,
                                       const PointCloudXYZI::Ptr& cloud,
                                       const PointTypePose* pose)
{
    PointCloudXYZI::Ptr transformedCloud(new PointCloudXYZI());
    transformedCloud->resize(cloud->size());

    Eigen::Isometry3d bodyFromLidar(state.statePoint.offset_R_L_I.toRotationMatrix());
    bodyFromLidar.pretranslate(Eigen::Vector3d(state.statePoint.offset_T_L_I(0),
                                               state.statePoint.offset_T_L_I(1),
                                               state.statePoint.offset_T_L_I(2)));
    Eigen::Isometry3d worldFromBody = Eigen::Isometry3d::Identity();
    worldFromBody.matrix() = pclPointToAffine3f(*pose).matrix().cast<double>();
    const Eigen::Isometry3d worldFromLidar = worldFromBody * bodyFromLidar;
    const int numberOfCores = state.config.numberOfCores;
    const int cloudSize = static_cast<int>(cloud->size());

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i) {
        const PointType& source = cloud->points[static_cast<std::size_t>(i)];
        PointType& target = transformedCloud->points[static_cast<std::size_t>(i)];
        const Eigen::Vector3d transformed =
            worldFromLidar * Eigen::Vector3d(source.x, source.y, source.z);
        target.x = static_cast<float>(transformed.x());
        target.y = static_cast<float>(transformed.y());
        target.z = static_cast<float>(transformed.z());
        target.intensity = source.intensity;
    }
    transformedCloud->width = static_cast<uint32_t>(transformedCloud->size());
    transformedCloud->height = 1;
    return transformedCloud;
}

float pointDistance(const PointType& lhs, const PointType& rhs)
{
    const float dx = lhs.x - rhs.x;
    const float dy = lhs.y - rhs.y;
    const float dz = lhs.z - rhs.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void reconstructIKdTree(FastLioAlgorithmState& state)
{
    auto& sam = state.sam;
    if (sam.reconstructKdTree && sam.updateKdtreeCount > 0) {
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMapPoses(
            new pcl::KdTreeFLANN<PointType>());
        PointCloudXYZI::Ptr subMapKeyPoses(new PointCloudXYZI());
        PointCloudXYZI::Ptr subMapKeyPosesDS(new PointCloudXYZI());
        PointCloudXYZI::Ptr subMapKeyFrames(new PointCloudXYZI());
        PointCloudXYZI::Ptr subMapKeyFramesDS(new PointCloudXYZI());
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;

        {
            std::lock_guard<std::mutex> lock(sam.poseMutex);
            kdtreeGlobalMapPoses->setInputCloud(sam.cloudKeyPoses3D);
            kdtreeGlobalMapPoses->radiusSearch(
                sam.cloudKeyPoses3D->back(),
                state.config.globalMapVisualizationSearchRadius,
                pointSearchIndGlobalMap,
                pointSearchSqDisGlobalMap,
                0);
        }

        for (const int index : pointSearchIndGlobalMap) {
            subMapKeyPoses->push_back(
                sam.cloudKeyPoses3D->points[static_cast<std::size_t>(index)]);
        }

        pcl::VoxelGrid<PointType> downSizeFilterSubMapKeyPoses;
        downSizeFilterSubMapKeyPoses.setLeafSize(
            state.config.globalMapVisualizationPoseDensity,
            state.config.globalMapVisualizationPoseDensity,
            state.config.globalMapVisualizationPoseDensity);
        downSizeFilterSubMapKeyPoses.setInputCloud(subMapKeyPoses);
        downSizeFilterSubMapKeyPoses.filter(*subMapKeyPosesDS);

        for (const PointType& posePoint : subMapKeyPosesDS->points) {
            if (pointDistance(posePoint, sam.cloudKeyPoses3D->back()) >
                state.config.globalMapVisualizationSearchRadius) {
                continue;
            }
            const int keyIndex = static_cast<int>(posePoint.intensity);
            *subMapKeyFrames += *transformPointCloud(
                state,
                sam.surfCloudKeyFrames[static_cast<std::size_t>(keyIndex)],
                &sam.cloudKeyPoses6D->points[static_cast<std::size_t>(keyIndex)]);
        }

        pcl::VoxelGrid<PointType> downSizeFilterGlobalMap;
        downSizeFilterGlobalMap.setLeafSize(
            state.config.globalMapVisualizationLeafSize,
            state.config.globalMapVisualizationLeafSize,
            state.config.globalMapVisualizationLeafSize);
        downSizeFilterGlobalMap.setInputCloud(subMapKeyFrames);
        downSizeFilterGlobalMap.filter(*subMapKeyFramesDS);
        state.ikdtree.reconstruct(subMapKeyFramesDS->points);
        sam.updateKdtreeCount = 0;
    }
    ++sam.updateKdtreeCount;
}

void appendOptimizedKeyframeMap(FastLioAlgorithmState& state,
                                const PointCloudXYZI::Ptr& keyframe,
                                const PointTypePose& pose)
{
    if (!state.config.saveMap) {
        return;
    }
    const PointCloudXYZI::Ptr transformed = transformPointCloud(state, keyframe, &pose);
    const PointCloudXYZI::Ptr downsampled =
        downsampleCloud(transformed, state.config.mappingSurfLeafSize);
    state.pendingOptimizedGlobalMapPoints += toSlamPoints(*downsampled);
}

QVector<SlamPoint> buildOptimizedGlobalMap(FastLioAlgorithmState& state)
{
    if (!state.config.saveMap) {
        return {};
    }

    std::vector<PointCloudXYZI::Ptr> keyframes;
    pcl::PointCloud<PointTypePose>::Ptr poses(new pcl::PointCloud<PointTypePose>());
    {
        std::lock_guard<std::mutex> lock(state.sam.poseMutex);
        keyframes = state.sam.surfCloudKeyFrames;
        *poses = *state.sam.cloudKeyPoses6D;
    }

    PointCloudXYZI::Ptr globalSurfCloud(new PointCloudXYZI());
    const int keyframeCount = static_cast<int>(poses->size());
    for (int i = 0; i < keyframeCount; ++i) {
        *globalSurfCloud += *transformPointCloud(
            state,
            keyframes[static_cast<std::size_t>(i)],
            &poses->points[static_cast<std::size_t>(i)]);
    }
    return toSlamPoints(*downsampleCloud(globalSurfCloud, state.config.mappingSurfLeafSize));
}

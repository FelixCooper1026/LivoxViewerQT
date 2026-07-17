#pragma once

#include "pcl/common/transforms.h"
#include "pcl/kdtree/kdtree_flann.h"

#include <Eigen/Core>
#include <Eigen/SVD>

#include <cmath>
#include <limits>
#include <vector>

namespace pcl {

template <typename SourcePointT, typename TargetPointT>
class IterativeClosestPoint {
public:
    void setMaxCorrespondenceDistance(double distance) { maxCorrespondenceDistance_ = distance; }
    void setMaximumIterations(int iterations) { maximumIterations_ = iterations; }
    void setTransformationEpsilon(double epsilon) { transformationEpsilon_ = epsilon; }
    void setEuclideanFitnessEpsilon(double epsilon) { euclideanFitnessEpsilon_ = epsilon; }
    void setRANSACIterations(int iterations) { ransacIterations_ = iterations; }
    void setInputSource(const typename PointCloud<SourcePointT>::Ptr& source) { source_ = source; }
    void setInputTarget(const typename PointCloud<TargetPointT>::Ptr& target) { target_ = target; }

    void align(PointCloud<SourcePointT>& output)
    {
        converged_ = false;
        fitnessScore_ = std::numeric_limits<double>::max();
        finalTransformation_.setIdentity();

        KdTreeFLANN<TargetPointT> targetTree;
        targetTree.setInputCloud(target_);

        double previousFitness = std::numeric_limits<double>::max();
        std::vector<Eigen::Vector3d> matchedSource;
        std::vector<Eigen::Vector3d> matchedTarget;
        matchedSource.reserve(source_->points.size());
        matchedTarget.reserve(source_->points.size());
        std::vector<int> indices;
        std::vector<float> squaredDistances;
        indices.reserve(1);
        squaredDistances.reserve(1);
        for (int iteration = 0; iteration < maximumIterations_; ++iteration) {
            Eigen::Vector3d sourceCentroid = Eigen::Vector3d::Zero();
            Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
            matchedSource.clear();
            matchedTarget.clear();

            for (const SourcePointT& sourcePoint : source_->points) {
                const Eigen::Vector4f transformed = finalTransformation_ *
                    Eigen::Vector4f(sourcePoint.x, sourcePoint.y, sourcePoint.z, 1.0f);
                TargetPointT query;
                query.x = transformed.x();
                query.y = transformed.y();
                query.z = transformed.z();

                if (targetTree.nearestKSearch(query, 1, indices, squaredDistances) != 1 ||
                    squaredDistances.front() > maxCorrespondenceDistance_ * maxCorrespondenceDistance_) {
                    continue;
                }

                const TargetPointT& targetPoint = target_->points[static_cast<std::size_t>(indices.front())];
                const Eigen::Vector3d sourceVector(transformed.x(), transformed.y(), transformed.z());
                const Eigen::Vector3d targetVector(targetPoint.x, targetPoint.y, targetPoint.z);
                matchedSource.push_back(sourceVector);
                matchedTarget.push_back(targetVector);
                sourceCentroid += sourceVector;
                targetCentroid += targetVector;
            }

            if (matchedSource.size() < 3) {
                transformPointCloud(*source_, output, finalTransformation_);
                return;
            }

            const double inverseCount = 1.0 / static_cast<double>(matchedSource.size());
            sourceCentroid *= inverseCount;
            targetCentroid *= inverseCount;

            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (std::size_t i = 0; i < matchedSource.size(); ++i) {
                covariance += (matchedSource[i] - sourceCentroid) *
                    (matchedTarget[i] - targetCentroid).transpose();
            }
            const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
                covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
            if (rotation.determinant() < 0.0) {
                Eigen::Matrix3d correctedV = svd.matrixV();
                correctedV.col(2) *= -1.0;
                rotation = correctedV * svd.matrixU().transpose();
            }
            const Eigen::Vector3d translation = targetCentroid - rotation * sourceCentroid;

            Eigen::Matrix4f incremental = Eigen::Matrix4f::Identity();
            incremental.block<3, 3>(0, 0) = rotation.cast<float>();
            incremental.block<3, 1>(0, 3) = translation.cast<float>();
            finalTransformation_ = incremental * finalTransformation_;

            double fitness = 0.0;
            for (std::size_t i = 0; i < matchedSource.size(); ++i) {
                const Eigen::Vector3d corrected = rotation * matchedSource[i] + translation;
                fitness += (corrected - matchedTarget[i]).squaredNorm();
            }
            fitness *= inverseCount;
            converged_ = true;
            fitnessScore_ = fitness;

            const double rotationDelta = (rotation - Eigen::Matrix3d::Identity()).squaredNorm();
            const double translationDelta = translation.squaredNorm();
            if (rotationDelta + translationDelta < transformationEpsilon_ ||
                std::abs(previousFitness - fitness) < euclideanFitnessEpsilon_) {
                break;
            }
            previousFitness = fitness;
        }

        transformPointCloud(*source_, output, finalTransformation_);
    }

    bool hasConverged() const { return converged_; }
    double getFitnessScore() const { return fitnessScore_; }
    const Eigen::Matrix4f& getFinalTransformation() const { return finalTransformation_; }

private:
    typename PointCloud<SourcePointT>::Ptr source_;
    typename PointCloud<TargetPointT>::Ptr target_;
    double maxCorrespondenceDistance_ = 1.0;
    int maximumIterations_ = 1;
    double transformationEpsilon_ = 0.0;
    double euclideanFitnessEpsilon_ = 0.0;
    int ransacIterations_ = 0;
    bool converged_ = false;
    double fitnessScore_ = std::numeric_limits<double>::max();
    Eigen::Matrix4f finalTransformation_ = Eigen::Matrix4f::Identity();
};

} // namespace pcl

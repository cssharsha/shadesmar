#include <core/types/keyframe.hpp>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include "gaussian_splatting/utils/point_cloud_utils.hpp"

namespace gaussian_splatting {
namespace utils {

PointCloudUtils::PointCloudUtils() {
    reset();
}

void PointCloudUtils::reset() {
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

PointCloudUtils::~PointCloudUtils() {}

void PointCloudUtils::addPoint(float x, float y, float z) {
    pcl::PointXYZ point;
    point.x = x;
    point.y = y;
    point.z = z;

    cloud_->points.push_back(point);
}

void PointCloudUtils::setupKDTree() {
    cloud_->width = cloud_->points.size();
    cloud_->height = 1;
    kdtree_.setInputCloud(cloud_);
}

int PointCloudUtils::getClosestPoint(const Eigen::Vector3f& point, std::vector<int>& indices,
                                     std::vector<float>& distances) {
    pcl::PointXYZ searchPoint;
    searchPoint.x = point.x();
    searchPoint.y = point.y();
    searchPoint.z = point.z();

    int K = 4;

    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);

    if (kdtree_.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0) {
        indices = pointIdxKNNSearch;
        distances = pointKNNSquaredDistance;
    }

    return indices.size();
}

Eigen::Vector3f PointCloudUtils::computeScaleFromKNN(const Eigen::Vector3f& position) {
    std::vector<int> indices;
    std::vector<float> distances;
    getClosestPoint(position, indices, distances);
    if (indices.size() < 3) {
        return Eigen::Vector3f::Zero();
    }

    float distance = 0.0f;
    for (uint32_t idx{0}; idx < indices.size(); ++idx) {
        float dist = distances[idx];
        if (idx < cloud_->points.size()) {
            distance += dist;
        }
    }
    distance /= indices.size();
    if (distance < 1e-6f) {
        distance = 1e-6f;
    }
    // distance = std::sqrt(distance);
    distance = std::log(distance);
    Eigen::Vector3f scale = Eigen::Vector3f(distance, distance, distance);

    return scale;
}

}  // namespace utils
}  // namespace gaussian_splatting

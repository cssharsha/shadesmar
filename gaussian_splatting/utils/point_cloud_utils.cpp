#include <core/types/keyframe.hpp>
#include <limits>
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
                                     std::vector<float>& distances, int K) {
    pcl::PointXYZ searchPoint;
    searchPoint.x = point.x();
    searchPoint.y = point.y();
    searchPoint.z = point.z();

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
    // Take square root to get actual distance (distances from KNN are squared)
    distance = std::sqrt(distance);
    // Return linear-space scale - log conversion happens in tensor conversion
    Eigen::Vector3f scale = Eigen::Vector3f(distance, distance, distance);

    return scale;
}

BoundingBox PointCloudUtils::computeBoundingBox() const {
    if (cloud_->points.empty()) {
        return BoundingBox();
    }

    Eigen::Vector3f min_pt(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max());
    Eigen::Vector3f max_pt(std::numeric_limits<float>::lowest(),
                           std::numeric_limits<float>::lowest(),
                           std::numeric_limits<float>::lowest());

    for (const auto& point : cloud_->points) {
        min_pt.x() = std::min(min_pt.x(), point.x);
        min_pt.y() = std::min(min_pt.y(), point.y);
        min_pt.z() = std::min(min_pt.z(), point.z);

        max_pt.x() = std::max(max_pt.x(), point.x);
        max_pt.y() = std::max(max_pt.y(), point.y);
        max_pt.z() = std::max(max_pt.z(), point.z);
    }

    return BoundingBox(min_pt, max_pt);
}

std::vector<BoundingBox> PointCloudUtils::subdivideIntoOctants(const BoundingBox& bbox) const {
    std::vector<BoundingBox> octants;
    octants.reserve(8);

    Eigen::Vector3f center = bbox.center();
    Eigen::Vector3f min = bbox.min;
    Eigen::Vector3f max = bbox.max;

    // Create 8 octants by dividing the bounding box at the center
    // Octant ordering: (x,y,z) where each can be min (-) or max (+)
    octants.push_back(BoundingBox(min, center));  // (-, -, -)
    octants.push_back(BoundingBox(Eigen::Vector3f(center.x(), min.y(), min.z()),
                                  Eigen::Vector3f(max.x(), center.y(), center.z())));  // (+, -, -)
    octants.push_back(BoundingBox(Eigen::Vector3f(min.x(), center.y(), min.z()),
                                  Eigen::Vector3f(center.x(), max.y(), center.z())));  // (-, +, -)
    octants.push_back(BoundingBox(Eigen::Vector3f(center.x(), center.y(), min.z()),
                                  Eigen::Vector3f(max.x(), max.y(), center.z())));  // (+, +, -)
    octants.push_back(BoundingBox(Eigen::Vector3f(min.x(), min.y(), center.z()),
                                  Eigen::Vector3f(center.x(), center.y(), max.z())));  // (-, -, +)
    octants.push_back(BoundingBox(Eigen::Vector3f(center.x(), min.y(), center.z()),
                                  Eigen::Vector3f(max.x(), center.y(), max.z())));  // (+, -, +)
    octants.push_back(BoundingBox(Eigen::Vector3f(min.x(), center.y(), center.z()),
                                  Eigen::Vector3f(center.x(), max.y(), max.z())));  // (-, +, +)
    octants.push_back(BoundingBox(center, max));                                    // (+, +, +)

    return octants;
}

}  // namespace utils
}  // namespace gaussian_splatting

#pragma once

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <vector>

namespace gaussian_splatting {
namespace utils {

struct BoundingBox {
    Eigen::Vector3f min;
    Eigen::Vector3f max;

    BoundingBox() : min(Eigen::Vector3f::Zero()), max(Eigen::Vector3f::Zero()) {}
    BoundingBox(const Eigen::Vector3f& min_pt, const Eigen::Vector3f& max_pt)
        : min(min_pt), max(max_pt) {}

    Eigen::Vector3f center() const {
        return (min + max) / 2.0f;
    }

    Eigen::Vector3f size() const {
        return max - min;
    }

    bool contains(const Eigen::Vector3f& point) const {
        return (point.x() >= min.x() && point.x() <= max.x() && point.y() >= min.y() &&
                point.y() <= max.y() && point.z() >= min.z() && point.z() <= max.z());
    }

    // Static helper function to compute center from min/max vectors
    static Eigen::Vector3f computeCenter(const Eigen::Vector3f& min_pt,
                                         const Eigen::Vector3f& max_pt) {
        return (min_pt + max_pt) / 2.0f;
    }
};

class PointCloudUtils {
public:
    PointCloudUtils();
    ~PointCloudUtils();

    void reset();
    void addPoint(float x, float y, float z);
    void setupKDTree();

    int getClosestPoint(const Eigen::Vector3f& point, std::vector<int>& indices,
                        std::vector<float>& distances, int K = 4);
    Eigen::Vector3f computeScaleFromKNN(const Eigen::Vector3f& position);

    // Bounding box computation
    BoundingBox computeBoundingBox() const;
    std::vector<BoundingBox> subdivideIntoOctants(const BoundingBox& bbox) const;

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
};

}  // namespace utils
}  // namespace gaussian_splatting

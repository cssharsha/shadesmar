#pragma once

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <vector>

namespace gaussian_splatting {
namespace utils {

class PointCloudUtils {
public:
    PointCloudUtils();
    ~PointCloudUtils();

    void reset();
    void addPoint(float x, float y, float z);
    void setupKDTree();

    int getClosestPoint(const Eigen::Vector3f& point, std::vector<int>& indices,
                        std::vector<float>& distances);
    Eigen::Vector3f computeScaleFromKNN(const Eigen::Vector3f& position);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
};

}  // namespace utils
}  // namespace gaussian_splatting

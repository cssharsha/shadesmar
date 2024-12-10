#include <core/types/keyframe.hpp>
#include <core/types/pose.hpp>
#include <gtest/gtest.h>

namespace core {
namespace types {
namespace testing {

TEST(TypesTest, PoseConversion) {
  Pose original_pose;
  original_pose.position = Eigen::Vector3d(1, 2, 3);
  original_pose.orientation =
      Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).normalized();
  original_pose.timestamp = 1234.5;

  // Convert to proto and back
  auto proto = original_pose.toProto();
  auto converted_pose = Pose::fromProto(proto);

  EXPECT_NEAR(original_pose.position.x(), converted_pose.position.x(), 1e-9);
  EXPECT_NEAR(original_pose.position.y(), converted_pose.position.y(), 1e-9);
  EXPECT_NEAR(original_pose.position.z(), converted_pose.position.z(), 1e-9);

  EXPECT_NEAR(original_pose.orientation.w(), converted_pose.orientation.w(),
              1e-9);
  EXPECT_NEAR(original_pose.orientation.x(), converted_pose.orientation.x(),
              1e-9);
  EXPECT_NEAR(original_pose.orientation.y(), converted_pose.orientation.y(),
              1e-9);
  EXPECT_NEAR(original_pose.orientation.z(), converted_pose.orientation.z(),
              1e-9);

  EXPECT_DOUBLE_EQ(original_pose.timestamp, converted_pose.timestamp);
}

TEST(TypesTest, PoseOperations) {
  Pose pose1;
  pose1.position = Eigen::Vector3d(1, 0, 0);
  pose1.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

  Pose pose2;
  pose2.position = Eigen::Vector3d(0, 1, 0);

  // Create equivalent Eigen transforms
  Eigen::Isometry3d t1 = Eigen::Isometry3d::Identity();
  t1.translation() = pose1.position;
  t1.linear() = pose1.orientation.toRotationMatrix();

  Eigen::Isometry3d t2 = Eigen::Isometry3d::Identity();
  t2.translation() = pose2.position;
  t2.linear() = pose2.orientation.toRotationMatrix();

  // Test composition
  Pose composed = pose1 * pose2;
  Eigen::Isometry3d t_composed = t1 * t2;

  EXPECT_NEAR((composed.position - t_composed.translation()).norm(), 0, 1e-9);
  EXPECT_NEAR((composed.orientation.matrix() - t_composed.rotation()).norm(), 0,
              1e-9);

  // Test inverse
  Pose inv = pose1.inverse();
  Eigen::Isometry3d t_inv = t1.inverse();

  EXPECT_NEAR((inv.position - t_inv.translation()).norm(), 0, 1e-9);
  EXPECT_NEAR((inv.orientation.matrix() - t_inv.rotation()).norm(), 0, 1e-9);

  // Test identity property
  Pose identity = pose1 * inv;
  Eigen::Isometry3d t_identity = t1 * t_inv;

  EXPECT_NEAR((identity.position - t_identity.translation()).norm(), 0, 1e-9);
  EXPECT_NEAR((identity.orientation.matrix() - t_identity.rotation()).norm(), 0,
              1e-9);
}

TEST(TypesTest, KeyFrameConversion) {
  auto original_kf = std::make_shared<KeyFrame>();
  original_kf->id = 42;
  original_kf->pose.position = Eigen::Vector3d(1, 2, 3);

  // Test with point cloud data
  PointCloud cloud;
  cloud.points.push_back(Eigen::Vector3d(0, 0, 0));
  cloud.points.push_back(Eigen::Vector3d(1, 1, 1));
  cloud.colors.push_back(Eigen::Vector3d(1, 0, 0));
  cloud.colors.push_back(Eigen::Vector3d(0, 1, 0));
  original_kf->data = cloud;

  // Convert to proto and back
  auto proto = original_kf->toProto();
  auto converted_kf = KeyFrame::fromProto(proto);

  EXPECT_EQ(original_kf->id, converted_kf->id);
  EXPECT_TRUE(converted_kf->hasPointCloud());

  const auto &converted_cloud = converted_kf->getPointCloud();
  EXPECT_EQ(cloud.points.size(), converted_cloud.points.size());
  EXPECT_EQ(cloud.colors.size(), converted_cloud.colors.size());
}

} // namespace testing
} // namespace types
} // namespace core
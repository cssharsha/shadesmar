#include <gtest/gtest.h>
#include <core/types/image.hpp>
#include <core/types/keyframe.hpp>
#include <core/types/pose.hpp>
#include <opencv2/core.hpp>

namespace core {
namespace types {
namespace testing {

TEST(TypesTest, PoseConversion) {
    Pose original_pose;
    original_pose.position = Eigen::Vector3d(1, 2, 3);
    original_pose.orientation = Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5).normalized();
    original_pose.timestamp = 1234.5;

    // Convert to proto and back
    auto proto = original_pose.toProto();
    auto converted_pose = Pose::fromProto(proto);

    EXPECT_NEAR(original_pose.position.x(), converted_pose.position.x(), 1e-9);
    EXPECT_NEAR(original_pose.position.y(), converted_pose.position.y(), 1e-9);
    EXPECT_NEAR(original_pose.position.z(), converted_pose.position.z(), 1e-9);

    EXPECT_NEAR(original_pose.orientation.w(), converted_pose.orientation.w(), 1e-9);
    EXPECT_NEAR(original_pose.orientation.x(), converted_pose.orientation.x(), 1e-9);
    EXPECT_NEAR(original_pose.orientation.y(), converted_pose.orientation.y(), 1e-9);
    EXPECT_NEAR(original_pose.orientation.z(), converted_pose.orientation.z(), 1e-9);

    EXPECT_DOUBLE_EQ(original_pose.timestamp, converted_pose.timestamp);
}

TEST(TypesTest, PoseOperations) {
    Pose pose1;
    pose1.position = Eigen::Vector3d(1, 0, 0);
    pose1.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

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
    EXPECT_NEAR((composed.orientation.matrix() - t_composed.rotation()).norm(), 0, 1e-9);

    // Test inverse
    Pose inv = pose1.inverse();
    Eigen::Isometry3d t_inv = t1.inverse();

    EXPECT_NEAR((inv.position - t_inv.translation()).norm(), 0, 1e-9);
    EXPECT_NEAR((inv.orientation.matrix() - t_inv.rotation()).norm(), 0, 1e-9);

    // Test identity property
    Pose identity = pose1 * inv;
    Eigen::Isometry3d t_identity = t1 * t_inv;

    EXPECT_NEAR((identity.position - t_identity.translation()).norm(), 0, 1e-9);
    EXPECT_NEAR((identity.orientation.matrix() - t_identity.rotation()).norm(), 0, 1e-9);
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
    original_kf->depth_data = cloud;

    // Convert to proto and back
    auto proto = original_kf->toProto();
    auto converted_kf = KeyFrame::fromProto(proto);

    EXPECT_EQ(original_kf->id, converted_kf->id);
    EXPECT_TRUE(converted_kf->hasPointCloud());

    const auto& converted_cloud = converted_kf->getPointCloud();
    EXPECT_EQ(cloud.points.size(), converted_cloud.points.size());
    EXPECT_EQ(cloud.colors.size(), converted_cloud.colors.size());
}

TEST(TypesTest, ImageConversion) {
    // Create a test image
    cv::Mat test_mat(480, 640, CV_8UC3);
    test_mat = cv::Scalar(100, 150, 200);

    Image original_image = Image::fromCvMat(test_mat, "bgr8");

    auto proto = original_image.toProto();
    auto converted_image = Image::fromProto(proto);

    // Verify properties
    EXPECT_EQ(converted_image.width, original_image.width);
    EXPECT_EQ(converted_image.height, original_image.height);
    EXPECT_EQ(converted_image.channels, original_image.channels);
    EXPECT_EQ(converted_image.encoding, original_image.encoding);

    // Verify image data - compare each channel separately
    std::vector<cv::Mat> orig_channels, conv_channels;
    cv::split(original_image.data, orig_channels);
    cv::split(converted_image.data, conv_channels);

    for (int i = 0; i < 3; ++i) {
        cv::Mat diff;
        cv::compare(orig_channels[i], conv_channels[i], diff, cv::CMP_NE);
        EXPECT_EQ(cv::countNonZero(diff), 0);
    }
}

TEST(TypesTest, KeyFrameWithImageData) {
    auto kf = std::make_shared<KeyFrame>();
    kf->id = 42;

    // Create test depth image - Add explicit type check
    cv::Mat depth_mat(480, 640, CV_32FC1, 1.0);
    Image depth_image = Image::fromCvMat(depth_mat, "32FC1");
    ASSERT_EQ(depth_image.data.type(), CV_32FC1) << "Original depth image type not preserved";
    kf->depth_data = depth_image;

    // Create test color image
    cv::Mat color_mat(480, 640, CV_8UC3, cv::Scalar(100, 150, 200));
    Image color_image = Image::fromCvMat(color_mat, "bgr8");
    kf->color_data = color_image;

    // Convert to proto and back
    auto proto = kf->toProto();
    auto converted_kf = KeyFrame::fromProto(proto);

    // Check basic properties
    EXPECT_EQ(kf->id, converted_kf->id);
    EXPECT_TRUE(std::holds_alternative<Image>(converted_kf->depth_data));
    EXPECT_TRUE(converted_kf->color_data.has_value());

    // Check depth image
    const auto& conv_depth_img = std::get<Image>(converted_kf->depth_data);
    const auto& orig_depth_img = std::get<Image>(kf->depth_data);
    EXPECT_EQ(conv_depth_img.width, orig_depth_img.width);
    EXPECT_EQ(conv_depth_img.height, orig_depth_img.height);
    EXPECT_EQ(conv_depth_img.encoding, orig_depth_img.encoding);

    // Check color image
    EXPECT_EQ(converted_kf->color_data->width, kf->color_data->width);
    EXPECT_EQ(converted_kf->color_data->height, kf->color_data->height);
    EXPECT_EQ(converted_kf->color_data->encoding, kf->color_data->encoding);

    // Verify image data - compare each channel separately for color image
    std::vector<cv::Mat> orig_color_channels, conv_color_channels;
    cv::split(kf->color_data->data, orig_color_channels);
    cv::split(converted_kf->color_data->data, conv_color_channels);

    // Add size checks before comparison
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(orig_color_channels[i].size(), conv_color_channels[i].size());
        ASSERT_EQ(orig_color_channels[i].type(), conv_color_channels[i].type());

        cv::Mat color_diff;
        cv::compare(orig_color_channels[i], conv_color_channels[i], color_diff, cv::CMP_NE);
        EXPECT_EQ(cv::countNonZero(color_diff), 0);
    }

    // For depth image (single channel) - Add more detailed error messages
    const auto& orig_depth = std::get<Image>(kf->depth_data).data;
    const auto& conv_depth = std::get<Image>(converted_kf->depth_data).data;

    ASSERT_EQ(orig_depth.size(), conv_depth.size()) << "Depth image size mismatch";
    ASSERT_EQ(orig_depth.type(), conv_depth.type())
        << "Depth image type mismatch. Original: " << orig_depth.type()
        << ", Converted: " << conv_depth.type();

    cv::Mat depth_diff;
    cv::compare(orig_depth, conv_depth, depth_diff, cv::CMP_NE);
    EXPECT_EQ(cv::countNonZero(depth_diff), 0);
}

}  // namespace testing
}  // namespace types
}  // namespace core

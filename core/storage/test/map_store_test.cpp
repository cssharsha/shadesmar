#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <core/storage/map_store.hpp>
#include <core/types/factor.hpp>
#include <core/types/keypoint.hpp>
#include <filesystem>
#include <opencv2/core/eigen.hpp>  // Added for Eigen/OpenCV interop
#include <opencv2/opencv.hpp>      // Added for cv::imread, cv::absdiff, etc.
#include "core/proto/factor.pb.h"

namespace core {
namespace storage {
namespace testing {

// Helper function to create a KeyFrame for testing
std::shared_ptr<core::types::KeyFrame> createKeyframe(uint64_t id, const std::string& frame_id,
                                                      const Eigen::Vector3d& pos,
                                                      const Eigen::Quaterniond& q,
                                                      double timestamp = 0.0) {
    auto kf = std::make_shared<core::types::KeyFrame>();
    kf->id = id;
    kf->pose.position = pos;
    kf->pose.orientation = q;
    kf->pose.timestamp = timestamp;
    kf->pose.frame_id = frame_id;

    core::types::CameraInfo c;
    c.frame_id = frame_id;
    c.width = 640;
    c.height = 480;
    c.distortion_model = "plumb_bob";
    // Simple camera matrix for testing
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 500.0;
    K(1, 1) = 500.0;
    K(0, 2) = 320.0;
    K(1, 2) = 240.0;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_rm = K;
    c.k = std::vector<double>(K_rm.data(), K_rm.data() + K_rm.size());
    // Initialize distortion coefficients to empty (no distortion)
    c.d = std::vector<double>(5, 0.0);
    kf->camera_info = c;

    // Create a small test image using the fromCvMat helper
    cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC3);
    core::types::Image img = core::types::Image::fromCvMat(test_image, "rgb8", frame_id);
    kf->color_data = img;

    // Add some test point cloud data
    core::types::PointCloud cloud;
    cloud.points.push_back(Eigen::Vector3d(0, 0, 1));
    cloud.points.push_back(Eigen::Vector3d(1, 0, 1));
    cloud.frame_id = frame_id;
    kf->depth_data = cloud;

    return kf;
}

// Helper function to create a Factor for testing
core::types::Factor createFactor(uint64_t id, proto::FactorType type,
                                 const std::vector<uint64_t>& connected_nodes,
                                 const core::types::Pose& measurement_pose,
                                 double timestamp = 0.0) {
    core::types::Factor factor;
    factor.id = id;
    factor.type = type;
    factor.connected_nodes = connected_nodes;

    // ✅ FIX: Use correct variant indices based on factor type
    if (type == proto::FactorType::PRIOR) {
        factor.measurement.emplace<0>(measurement_pose);  // Index 0 for absolute poses
    } else if (type == proto::FactorType::ODOMETRY || type == proto::FactorType::LOOP_CLOSURE) {
        factor.measurement.emplace<1>(measurement_pose);  // Index 1 for relative poses
    } else {
        factor.measurement.emplace<0>(measurement_pose);  // Default fallback
    }

    factor.information = Eigen::Matrix<double, 6, 6>::Identity();
    factor.timestamp = timestamp;
    factor.description = "Test factor";
    return factor;
}

// Helper function to create a Keypoint for testing
core::types::Keypoint createKeypoint(uint32_t id, const Eigen::Vector3d& position,
                                     const std::vector<core::types::Location>& locations = {}) {
    core::types::Keypoint keypoint(id);
    keypoint.position = position;
    keypoint.locations = locations;
    return keypoint;
}

class MapStoreTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_base_path_ = "test_map";
        store_ = std::make_unique<MapStore>(test_base_path_);
    }

    void TearDown() override {
        // Clean up test files
        std::filesystem::remove(test_base_path_ + ".dat");
        std::filesystem::remove(test_base_path_ + ".idx");
        std::filesystem::remove(test_base_path_ + ".meta");
    }

    std::unique_ptr<MapStore> store_;
    std::string test_base_path_;
};

TEST_F(MapStoreTest, AddAndGetKeyFrame) {
    // Create a test keyframe
    auto kf = createKeyframe(1, "camera_frame", Eigen::Vector3d(1.0, 2.0, 3.0),
                             Eigen::Quaterniond::Identity(), 123.456);

    // Add keyframe to store
    EXPECT_TRUE(store_->addKeyFrame(kf));

    // Retrieve keyframe
    auto retrieved_kf = store_->getKeyFrame(1);
    ASSERT_NE(retrieved_kf, nullptr);
    EXPECT_EQ(retrieved_kf->id, 1);
    EXPECT_NEAR((retrieved_kf->pose.position - Eigen::Vector3d(1.0, 2.0, 3.0)).norm(), 0, 1e-9);
    EXPECT_NEAR(retrieved_kf->pose.timestamp, 123.456, 1e-9);
}

TEST_F(MapStoreTest, SaveAndLoadKeyFrames) {
    // Create and add a keyframe
    auto kf = createKeyframe(1, "camera_frame", Eigen::Vector3d(1.0, 2.0, 3.0),
                             Eigen::Quaterniond::Identity(), 123.456);

    EXPECT_TRUE(store_->addKeyFrame(kf));
    EXPECT_TRUE(store_->saveChanges());

    // Create new store and load
    auto new_store = std::make_unique<MapStore>(test_base_path_);
    EXPECT_TRUE(new_store->loadMap());

    auto loaded_kf = new_store->getKeyFrame(1);
    ASSERT_NE(loaded_kf, nullptr);
    EXPECT_EQ(loaded_kf->id, kf->id);
    EXPECT_NEAR((loaded_kf->pose.position - kf->pose.position).norm(), 0, 1e-9);
    EXPECT_NEAR(loaded_kf->pose.timestamp, kf->pose.timestamp, 1e-9);
}

TEST_F(MapStoreTest, AddAndGetFactor) {
    // Create a test factor
    core::types::Pose measurement_pose;
    measurement_pose.position = Eigen::Vector3d(0.1, 0.2, 0.3);
    measurement_pose.orientation = Eigen::Quaterniond::Identity();
    measurement_pose.timestamp = 456.789;

    auto factor = createFactor(1, proto::FactorType::ODOMETRY, {1, 2}, measurement_pose, 456.789);

    // Add factor to store
    EXPECT_TRUE(store_->addFactor(factor));

    // Retrieve factor
    auto retrieved_factor = store_->getFactor(1);
    ASSERT_TRUE(retrieved_factor.has_value());
    EXPECT_EQ(retrieved_factor->id, 1);
    EXPECT_EQ(retrieved_factor->type, proto::FactorType::ODOMETRY);
    EXPECT_EQ(retrieved_factor->connected_nodes.size(), 2);
    EXPECT_EQ(retrieved_factor->connected_nodes[0], 1);
    EXPECT_EQ(retrieved_factor->connected_nodes[1], 2);
    EXPECT_NEAR(retrieved_factor->timestamp, 456.789, 1e-9);
}

TEST_F(MapStoreTest, SaveAndLoadFactors) {
    // Create a test factor
    core::types::Pose measurement_pose;
    measurement_pose.position = Eigen::Vector3d(0.1, 0.2, 0.3);
    measurement_pose.orientation = Eigen::Quaterniond::Identity();
    measurement_pose.timestamp = 456.789;

    auto factor = createFactor(1, proto::FactorType::ODOMETRY, {1, 2}, measurement_pose, 456.789);

    EXPECT_TRUE(store_->addFactor(factor));
    EXPECT_TRUE(store_->saveChanges());

    // Create new store and load
    auto new_store = std::make_unique<MapStore>(test_base_path_);
    EXPECT_TRUE(new_store->loadMap());

    auto factors = new_store->getAllFactors();
    ASSERT_EQ(factors.size(), 1);
    EXPECT_EQ(factors[0].id, factor.id);
    EXPECT_EQ(factors[0].type, factor.type);
    EXPECT_EQ(factors[0].connected_nodes, factor.connected_nodes);
    EXPECT_NEAR(factors[0].timestamp, factor.timestamp, 1e-9);
}

TEST_F(MapStoreTest, AddAndGetKeyPoint) {
    // Create test locations
    std::vector<core::types::Location> locations;
    core::types::Location loc1{1, "camera_frame"};
    loc1.x = 100.5f;
    loc1.y = 200.5f;
    locations.push_back(loc1);

    // Create a test keypoint
    auto keypoint = createKeypoint(1, Eigen::Vector3d(1.0, 2.0, 3.0), locations);

    // Add keypoint to store
    EXPECT_TRUE(store_->addKeyPoint(keypoint));

    // Retrieve keypoint
    auto retrieved_kp = store_->getKeyPoint(1);
    ASSERT_TRUE(retrieved_kp.has_value());
    EXPECT_EQ(retrieved_kp->id(), 1);
    EXPECT_NEAR((retrieved_kp->position - Eigen::Vector3d(1.0, 2.0, 3.0)).norm(), 0, 1e-9);
    EXPECT_EQ(retrieved_kp->locations.size(), 1);
    EXPECT_EQ(retrieved_kp->locations[0].keyframe_id, 1);
    EXPECT_EQ(retrieved_kp->locations[0].frame_id, "camera_frame");
}

TEST_F(MapStoreTest, SaveAndLoadKeyPoints) {
    // Create test locations
    std::vector<core::types::Location> locations;
    core::types::Location loc1{1, "camera_frame"};
    loc1.x = 100.5f;
    loc1.y = 200.5f;
    locations.push_back(loc1);

    auto keypoint = createKeypoint(1, Eigen::Vector3d(1.0, 2.0, 3.0), locations);

    EXPECT_TRUE(store_->addKeyPoint(keypoint));
    EXPECT_TRUE(store_->saveChanges());

    // Create new store and load
    auto new_store = std::make_unique<MapStore>(test_base_path_);
    EXPECT_TRUE(new_store->loadMap());

    auto keypoints = new_store->getAllKeyPoints();
    ASSERT_EQ(keypoints.size(), 1);
    EXPECT_EQ(keypoints[0].id(), keypoint.id());
    EXPECT_NEAR((keypoints[0].position - keypoint.position).norm(), 0, 1e-9);
    EXPECT_EQ(keypoints[0].locations.size(), keypoint.locations.size());
}

TEST_F(MapStoreTest, GetKeyFramesByTimestamp) {
    // Create keyframes with different timestamps
    auto kf1 = createKeyframe(1, "camera_frame", Eigen::Vector3d(1.0, 0.0, 0.0),
                              Eigen::Quaterniond::Identity(), 100.0);
    auto kf2 = createKeyframe(2, "camera_frame", Eigen::Vector3d(2.0, 0.0, 0.0),
                              Eigen::Quaterniond::Identity(), 100.0);  // Same timestamp
    auto kf3 = createKeyframe(3, "camera_frame", Eigen::Vector3d(3.0, 0.0, 0.0),
                              Eigen::Quaterniond::Identity(), 200.0);

    EXPECT_TRUE(store_->addKeyFrame(kf1));
    EXPECT_TRUE(store_->addKeyFrame(kf2));
    EXPECT_TRUE(store_->addKeyFrame(kf3));

    // Get keyframes by specific timestamp
    auto kfs_at_100 = store_->getKeyFramesByTimestamp(100.0);
    EXPECT_EQ(kfs_at_100.size(), 2);

    auto kfs_at_200 = store_->getKeyFramesByTimestamp(200.0);
    EXPECT_EQ(kfs_at_200.size(), 1);
    EXPECT_EQ(kfs_at_200[0]->id, 3);
}

TEST_F(MapStoreTest, GetKeyFramesByTimestampRange) {
    // Create keyframes with different timestamps
    auto kf1 = createKeyframe(1, "camera_frame", Eigen::Vector3d(1.0, 0.0, 0.0),
                              Eigen::Quaterniond::Identity(), 50.0);
    auto kf2 = createKeyframe(2, "camera_frame", Eigen::Vector3d(2.0, 0.0, 0.0),
                              Eigen::Quaterniond::Identity(), 100.0);
    auto kf3 = createKeyframe(3, "camera_frame", Eigen::Vector3d(3.0, 0.0, 0.0),
                              Eigen::Quaterniond::Identity(), 150.0);
    auto kf4 = createKeyframe(4, "camera_frame", Eigen::Vector3d(4.0, 0.0, 0.0),
                              Eigen::Quaterniond::Identity(), 200.0);

    EXPECT_TRUE(store_->addKeyFrame(kf1));
    EXPECT_TRUE(store_->addKeyFrame(kf2));
    EXPECT_TRUE(store_->addKeyFrame(kf3));
    EXPECT_TRUE(store_->addKeyFrame(kf4));

    // Get keyframes in range [75, 175]
    auto kfs_in_range = store_->getKeyFramesByTimestampRange(75.0, 175.0);
    EXPECT_EQ(kfs_in_range.size(), 2);  // Should get kf2 and kf3
}

TEST_F(MapStoreTest, FindKeyFramesNearPosition) {
    // Create keyframes at different positions
    auto kf1 = createKeyframe(1, "camera_frame", Eigen::Vector3d(0.0, 0.0, 0.0),
                              Eigen::Quaterniond::Identity(), 100.0);
    auto kf2 = createKeyframe(2, "camera_frame", Eigen::Vector3d(1.0, 0.0, 0.0),
                              Eigen::Quaterniond::Identity(), 200.0);
    auto kf3 = createKeyframe(3, "camera_frame", Eigen::Vector3d(5.0, 0.0, 0.0),
                              Eigen::Quaterniond::Identity(), 300.0);

    EXPECT_TRUE(store_->addKeyFrame(kf1));
    EXPECT_TRUE(store_->addKeyFrame(kf2));
    EXPECT_TRUE(store_->addKeyFrame(kf3));

    // Find keyframes near origin within radius 2.0
    Eigen::Vector3d target_pos(0.0, 0.0, 0.0);
    auto nearby_kfs = store_->findKeyFramesNearPosition(target_pos, 2.0);
    EXPECT_EQ(nearby_kfs.size(), 2);  // Should find kf1 and kf2

    // Find keyframes with max results limit
    auto limited_kfs = store_->findKeyFramesNearPosition(target_pos, 2.0, 1);
    EXPECT_EQ(limited_kfs.size(), 1);  // Should find only closest one (kf1)
    EXPECT_EQ(limited_kfs[0]->id, 1);
}

TEST_F(MapStoreTest, GetAllMethods) {
    // Add some test data
    auto kf = createKeyframe(1, "camera_frame", Eigen::Vector3d(1.0, 2.0, 3.0),
                             Eigen::Quaterniond::Identity(), 123.456);

    core::types::Pose measurement_pose;
    measurement_pose.position = Eigen::Vector3d(0.1, 0.2, 0.3);
    measurement_pose.orientation = Eigen::Quaterniond::Identity();
    auto factor = createFactor(1, proto::FactorType::ODOMETRY, {1, 2}, measurement_pose, 456.789);

    auto keypoint = createKeypoint(1, Eigen::Vector3d(1.0, 2.0, 3.0));

    EXPECT_TRUE(store_->addKeyFrame(kf));
    EXPECT_TRUE(store_->addFactor(factor));
    EXPECT_TRUE(store_->addKeyPoint(keypoint));

    // Test getAll methods
    auto all_kfs = store_->getAllKeyFrames();
    EXPECT_EQ(all_kfs.size(), 1);

    auto all_factors = store_->getAllFactors();
    EXPECT_EQ(all_factors.size(), 1);

    auto all_keypoints = store_->getAllKeyPoints();
    EXPECT_EQ(all_keypoints.size(), 1);
}

TEST_F(MapStoreTest, MetadataBounds) {
    // Add keyframes at different positions to test bounds calculation
    auto kf1 = createKeyframe(1, "camera_frame", Eigen::Vector3d(-1.0, -2.0, -3.0),
                              Eigen::Quaterniond::Identity(), 100.0);
    auto kf2 = createKeyframe(2, "camera_frame", Eigen::Vector3d(5.0, 3.0, 1.0),
                              Eigen::Quaterniond::Identity(), 200.0);

    EXPECT_TRUE(store_->addKeyFrame(kf1));
    EXPECT_TRUE(store_->addKeyFrame(kf2));

    const auto& metadata = store_->getMetadata();
    const auto& bounds = metadata.bounds();

    EXPECT_NEAR(bounds.min_x(), -1.0, 1e-9);
    EXPECT_NEAR(bounds.min_y(), -2.0, 1e-9);
    EXPECT_NEAR(bounds.min_z(), -3.0, 1e-9);
    EXPECT_NEAR(bounds.max_x(), 5.0, 1e-9);
    EXPECT_NEAR(bounds.max_y(), 3.0, 1e-9);
    EXPECT_NEAR(bounds.max_z(), 1.0, 1e-9);
}

TEST_F(MapStoreTest, AddKeyFrameWithRealImage) {
    // Test with actual keyframe image from /data/robot/keyframes
    std::string image_path = "/data/robot/keyframes/keyframe_71.png";

    // Check if the image file exists
    if (!std::filesystem::exists(image_path)) {
        GTEST_SKIP() << "Keyframe image not found at: " << image_path;
        return;
    }

    // Load the actual keyframe image
    cv::Mat real_image = cv::imread(image_path, cv::IMREAD_COLOR);
    ASSERT_FALSE(real_image.empty()) << "Failed to load image from: " << image_path;

    // Create keyframe with real image data
    auto kf = std::make_shared<core::types::KeyFrame>();
    kf->id = 71;
    kf->pose.position = Eigen::Vector3d(2.5, 1.5, 0.5);
    kf->pose.orientation = Eigen::Quaterniond::Identity();
    kf->pose.timestamp = 71.0;
    kf->pose.frame_id = "camera_frame";

    // Set up camera info to match the real image dimensions
    core::types::CameraInfo c;
    c.frame_id = "camera_frame";
    c.width = real_image.cols;
    c.height = real_image.rows;
    c.distortion_model = "plumb_bob";
    // Camera matrix for testing
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = 636.6418;
    K(1, 1) = 636.1846;
    K(0, 2) = 635.5804;
    K(1, 2) = 372.6107;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K_rm = K;
    c.k = std::vector<double>(K_rm.data(), K_rm.data() + K_rm.size());
    c.d = std::vector<double>(5, 0.0);  // Initialize distortion coefficients (no distortion)
    kf->camera_info = c;

    // Create image from the loaded cv::Mat
    core::types::Image img = core::types::Image::fromCvMat(real_image, "bgr8", "camera_frame");
    kf->color_data = img;

    // Add some test point cloud data
    core::types::PointCloud cloud;
    cloud.points.push_back(Eigen::Vector3d(0, 0, 1));
    cloud.points.push_back(Eigen::Vector3d(1, 0, 1));
    cloud.frame_id = "camera_frame";
    kf->depth_data = cloud;

    // Add keyframe to store
    EXPECT_TRUE(store_->addKeyFrame(kf));

    // Retrieve and verify keyframe
    auto retrieved_kf = store_->getKeyFrame(71);
    ASSERT_NE(retrieved_kf, nullptr);
    EXPECT_EQ(retrieved_kf->id, 71);
    EXPECT_NEAR((retrieved_kf->pose.position - Eigen::Vector3d(2.5, 1.5, 0.5)).norm(), 0, 1e-9);

    // Verify image data
    ASSERT_TRUE(retrieved_kf->color_data.has_value());
    const auto& retrieved_img = retrieved_kf->color_data.value();
    EXPECT_EQ(retrieved_img.width, real_image.cols);
    EXPECT_EQ(retrieved_img.height, real_image.rows);
    EXPECT_EQ(retrieved_img.encoding, "bgr8");

    // Verify camera info matches image dimensions
    ASSERT_TRUE(retrieved_kf->camera_info.has_value());
    const auto& retrieved_cam_info = retrieved_kf->camera_info.value();
    EXPECT_EQ(retrieved_cam_info.width, real_image.cols);
    EXPECT_EQ(retrieved_cam_info.height, real_image.rows);

    // Test save and load with real image
    EXPECT_TRUE(store_->saveChanges());

    auto new_store = std::make_unique<MapStore>(test_base_path_);
    EXPECT_TRUE(new_store->loadMap());

    auto loaded_kf = new_store->getKeyFrame(71);
    ASSERT_NE(loaded_kf, nullptr);
    EXPECT_EQ(loaded_kf->id, kf->id);
    EXPECT_NEAR((loaded_kf->pose.position - kf->pose.position).norm(), 0, 1e-9);

    // Verify loaded image data
    ASSERT_TRUE(loaded_kf->color_data.has_value());
    const auto& loaded_img = loaded_kf->color_data.value();
    EXPECT_EQ(loaded_img.width, real_image.cols);
    EXPECT_EQ(loaded_img.height, real_image.rows);
    EXPECT_EQ(loaded_img.encoding, "bgr8");

    // Verify actual image content
    cv::Mat loaded_cv_img = loaded_img.data;
    ASSERT_EQ(real_image.type(), loaded_cv_img.type());
    ASSERT_EQ(real_image.size(), loaded_cv_img.size());
    cv::Mat diff;
    cv::absdiff(real_image, loaded_cv_img, diff);
    cv::Mat diff_channels[3];
    cv::split(diff, diff_channels);
    EXPECT_EQ(cv::countNonZero(diff_channels[0]), 0);
    EXPECT_EQ(cv::countNonZero(diff_channels[1]), 0);
    EXPECT_EQ(cv::countNonZero(diff_channels[2]), 0);
}

}  // namespace testing
}  // namespace storage
}  // namespace core

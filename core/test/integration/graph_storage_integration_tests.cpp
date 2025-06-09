#include <gtest/gtest.h>
#include <core/graph/factor_graph.hpp>
#include <core/graph/graph_adapter.hpp>
#include <core/storage/map_store.hpp>
#include <thread>
#include <chrono>

class GraphStorageIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize MapStore with proper constructor
        store_ = std::make_unique<core::storage::MapStore>("/tmp/legacy_test_map");
    }

    void TearDown() override {
        store_.reset();
    }

    std::unique_ptr<core::storage::MapStore> store_;
    core::graph::FactorGraph graph_;

    // Helper functions
    core::types::KeyFrame::Ptr createKeyFrame(uint64_t id, double x, double y, double z) {
        auto kf = std::make_shared<core::types::KeyFrame>();
        kf->id = id;
        kf->pose.position = Eigen::Vector3d(x, y, z);
        kf->pose.orientation = Eigen::Quaterniond::Identity();
        kf->pose.timestamp = id * 1.0;

        // Add some test point cloud data
        core::types::PointCloud cloud;
        cloud.points.push_back(Eigen::Vector3d(x, y, z));
        cloud.colors.push_back(Eigen::Vector3d(1, 0, 0));
        kf->depth_data = cloud;

        return kf;
    }

    core::types::Factor createOdometryFactor(uint64_t id, uint64_t from_id, uint64_t to_id,
                                             const Eigen::Vector3d& relative_translation) {
        core::types::Factor factor;
        factor.id = id;
        factor.type = core::proto::FactorType::ODOMETRY;
        factor.connected_nodes = {from_id, to_id};

        core::types::Pose relative_pose;
        relative_pose.position = relative_translation;
        relative_pose.orientation = Eigen::Quaterniond::Identity();
        factor.measurement.emplace<1>(relative_pose);  // Note: using emplace<1> for relative pose

        factor.information = Eigen::Matrix<double, 6, 6>::Identity();
        return factor;
    }
};

// PHASE 4 COMPREHENSIVE INTEGRATION TEST
class Phase4IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize with clean temporary storage
        store_ = std::make_unique<core::storage::MapStore>("/tmp/phase4_test_map");
        graph_ = std::make_unique<core::graph::FactorGraph>();
        adapter_ = std::make_unique<core::graph::GraphAdapter>(*graph_, *store_);

        // Enable optimization thread for testing
        adapter_->setOptimizationInterval(3);  // Optimize every 3 keyframes
        adapter_->enableOptimizationThread();

        LOG(INFO) << "Phase 4 Integration Test: Setup complete with bridge architecture";
    }

    void TearDown() override {
        // Ensure proper cleanup
        adapter_->disableOptimizationThread();
        adapter_.reset();
        graph_.reset();
        store_.reset();

        LOG(INFO) << "Phase 4 Integration Test: Teardown complete";
    }

    std::unique_ptr<core::storage::MapStore> store_;
    std::unique_ptr<core::graph::FactorGraph> graph_;
    std::unique_ptr<core::graph::GraphAdapter> adapter_;

    // Helper to create test poses with realistic movement
    core::types::Pose createTestPose(double x, double y, double theta, double timestamp) {
        core::types::Pose pose;
        pose.position = Eigen::Vector3d(x, y, 0.0);
        pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
        pose.timestamp = timestamp;
        pose.frame_id = "map";
        return pose;
    }

    // Helper to create test camera info
    core::types::CameraInfo createTestCameraInfo() {
        core::types::CameraInfo camera_info;
        camera_info.width = 640;
        camera_info.height = 480;
        camera_info.k = {500, 0, 320, 0, 500, 240, 0, 0, 1};  // fx, fy, cx, cy
        camera_info.frame_id = "camera";
        return camera_info;
    }

    // Helper to create test image
    core::types::Image createTestImage(int width = 640, int height = 480) {
        core::types::Image image;
        image.width = width;
        image.height = height;
        image.encoding = "mono8";
        image.data = cv::Mat::zeros(height, width, CV_8UC1);
        // Add some simple pattern for ORB tracking
        cv::circle(image.data, cv::Point(width/4, height/4), 20, cv::Scalar(255), -1);
        cv::circle(image.data, cv::Point(3*width/4, height/4), 20, cv::Scalar(255), -1);
        cv::circle(image.data, cv::Point(width/2, 3*height/4), 20, cv::Scalar(255), -1);
        return image;
    }
};

TEST_F(Phase4IntegrationTest, ComprehensiveArchitectureValidation) {
    LOG(INFO) << "=== Phase 4 Test: Comprehensive Architecture Validation ===";

    // Test 1: Bridge Pattern - GraphAdapter coordinates between components
    LOG(INFO) << "Test 1: Verifying GraphAdapter bridge/coordinator pattern";

    auto camera_info = createTestCameraInfo();
    adapter_->handleCameraInfo(camera_info, 0.0);

    // Create a simple trajectory: just a few poses
    std::vector<core::types::Pose> trajectory = {
        createTestPose(0.0, 0.0, 0.0, 1.0),      // Start
        createTestPose(1.0, 0.0, 0.0, 2.0),      // Move east
        createTestPose(2.0, 0.0, 0.0, 3.0),      // Move further east
        createTestPose(3.0, 0.0, 0.0, 4.0),      // Move further east
    };

    // Process trajectory through GraphAdapter
    size_t keyframes_created = 0;
    for (const auto& pose : trajectory) {
        auto image = createTestImage();

        // Handle inputs through GraphAdapter bridge
        adapter_->handleOdometryInput(pose, pose.timestamp);
        adapter_->handleImageInput(image, pose.timestamp);
        adapter_->handleCameraInfo(camera_info, pose.timestamp);

        // Small delay to allow synchronizer to work
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        keyframes_created++;

        LOG(INFO) << "Processed pose " << keyframes_created << ": [" << pose.position.transpose() << "]";
    }

    // Wait briefly for processing to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Test 2: MapStore as single source of truth
    LOG(INFO) << "Test 2: Verifying MapStore as single source of truth";

    auto all_keyframes = store_->getAllKeyFrames();
    auto all_factors = store_->getAllFactors();
    auto all_keypoints = store_->getAllKeyPoints();

    EXPECT_GT(all_keyframes.size(), 0) << "MapStore should contain keyframes from GraphAdapter";
    EXPECT_GT(all_factors.size(), 0) << "MapStore should contain factors from GraphAdapter";

    LOG(INFO) << "MapStore contains: " << all_keyframes.size() << " keyframes, "
              << all_factors.size() << " factors, " << all_keypoints.size() << " keypoints";

    // Test 3: Memory-efficient pose loading
    LOG(INFO) << "Test 3: Verifying memory-efficient pose loading";

    auto poses_only = store_->getAllKeyFramePoses();
    EXPECT_EQ(poses_only.size(), all_keyframes.size())
        << "Memory-efficient pose loading should return same count as full keyframes";

    // Verify poses match without loading full keyframes
    for (const auto& [keyframe_id, pose] : poses_only) {
        auto full_keyframe = store_->getKeyFrame(keyframe_id);
        ASSERT_NE(full_keyframe, nullptr);
        EXPECT_TRUE(pose.position.isApprox(full_keyframe->pose.position, 1e-6))
            << "Memory-efficient pose should match full keyframe pose";
    }

    LOG(INFO) << "Memory-efficient loading verified for " << poses_only.size() << " poses";

    // Test 4: Background sync functionality
    LOG(INFO) << "Test 4: Verifying background sync functionality";

    EXPECT_TRUE(store_->isBackgroundSyncEnabled()) << "Background sync should be enabled";

    // Test 5: Factor-keyframe associations
    LOG(INFO) << "Test 5: Verifying factor-keyframe associations";

    for (const auto& keyframe : all_keyframes) {
        auto associated_factors = store_->getFactorsForKeyFrame(keyframe->id);
        if (!associated_factors.empty()) {
            LOG(INFO) << "Keyframe " << keyframe->id << " has " << associated_factors.size()
                      << " associated factors";

            // Verify reverse mapping works
            for (const auto& factor : associated_factors) {
                bool found_keyframe = false;
                for (uint64_t connected_id : factor.connected_nodes) {
                    if (connected_id == keyframe->id) {
                        found_keyframe = true;
                        break;
                    }
                }
                EXPECT_TRUE(found_keyframe) << "Factor should reference the keyframe";
            }
        }
    }

    // Test 6: Final architecture state validation
    LOG(INFO) << "Test 6: Final architecture state validation";

    auto final_keyframes = store_->getAllKeyFrames();
    auto final_factors = store_->getAllFactors();

    LOG(INFO) << "Final state: " << final_keyframes.size() << " keyframes, "
              << final_factors.size() << " factors";

    // Verify GraphAdapter is not storing data (pure bridge)
    auto map_points_copy = adapter_->getMapPointsCopy();
    auto direct_keypoints = store_->getAllKeyPoints();

    EXPECT_EQ(map_points_copy.size(), direct_keypoints.size())
        << "GraphAdapter should coordinate to same data as MapStore";

    LOG(INFO) << "=== Phase 4 Comprehensive Test: SUCCESS ===";
}

TEST_F(GraphStorageIntegrationTest, OptimizeFromStorageOnly) {
    // Create a simple scenario with three keyframes in a triangle
    auto kf1 = createKeyFrame(1, 0, 0, 0);
    auto kf2 = createKeyFrame(2, 1, 0, 0);
    auto kf3 = createKeyFrame(3, 1, 1, 0);

    // Add keyframes only to storage (new architecture)
    store_->addKeyFrame(kf1);
    store_->addKeyFrame(kf2);
    store_->addKeyFrame(kf3);

    // Create factors
    auto prior = core::types::Factor();
    prior.id = 1;
    prior.type = core::proto::FactorType::PRIOR;
    prior.connected_nodes = {1};
    prior.measurement.emplace<0>(kf1->pose);  // Prior factor uses emplace<0> for pose
    prior.information = Eigen::Matrix<double, 6, 6>::Identity();

    auto odom1 = createOdometryFactor(2, 1, 2, Eigen::Vector3d(1, 0, 0));
    auto odom2 = createOdometryFactor(3, 2, 3, Eigen::Vector3d(0, 1, 0));

    // Add factors only to storage (new architecture)
    store_->addFactor(prior);
    store_->addFactor(odom1);
    store_->addFactor(odom2);

    // Use new stateless optimization from storage
    EXPECT_TRUE(graph_.optimizeFromStorage(*store_));

    // Get optimized poses from FactorGraph
    auto optimized_poses = graph_.getOptimizedPoses();
    EXPECT_EQ(optimized_poses.size(), 3);

    // Update storage with optimized poses (new pattern)
    for (const auto& [keyframe_id, optimized_pose] : optimized_poses) {
        auto keyframe = store_->getKeyFrame(keyframe_id);
        if (keyframe) {
            keyframe->pose = optimized_pose;
            store_->addKeyFrame(keyframe);  // Update in storage
        }
    }

    // Verify optimized poses were updated in storage
    auto updated_kf1 = store_->getKeyFrame(1);
    auto updated_kf2 = store_->getKeyFrame(2);
    auto updated_kf3 = store_->getKeyFrame(3);

    ASSERT_NE(updated_kf1, nullptr);
    ASSERT_NE(updated_kf2, nullptr);
    ASSERT_NE(updated_kf3, nullptr);

    // Check if optimized poses are reasonable (within expected bounds)
    EXPECT_LT(updated_kf1->pose.position.norm(), 0.1);  // Should be near origin
    EXPECT_TRUE(updated_kf2->pose.position.x() > 0.5);  // Should be east of origin
    EXPECT_TRUE(updated_kf3->pose.position.y() > 0.5);  // Should be north of origin
}

TEST_F(GraphStorageIntegrationTest, IncrementalOptimizationFromStorage) {
    // Initial keyframes
    auto kf1 = createKeyFrame(1, 0, 0, 0);
    auto kf2 = createKeyFrame(2, 1, 0, 0);

    // Add initial keyframes to storage only
    store_->addKeyFrame(kf1);
    store_->addKeyFrame(kf2);

    auto prior = core::types::Factor();
    prior.id = 1;
    prior.type = core::proto::FactorType::PRIOR;
    prior.connected_nodes = {1};
    prior.measurement.emplace<0>(kf1->pose);
    prior.information = Eigen::Matrix<double, 6, 6>::Identity();

    auto odom1 = createOdometryFactor(2, 1, 2, Eigen::Vector3d(1, 0, 0));

    store_->addFactor(prior);
    store_->addFactor(odom1);

    // First optimization from storage
    EXPECT_TRUE(graph_.optimizeFromStorage(*store_));

    // Add new keyframe and factor
    auto kf3 = createKeyFrame(3, 2, 0, 0);
    store_->addKeyFrame(kf3);

    auto odom2 = createOdometryFactor(3, 2, 3, Eigen::Vector3d(1, 0, 0));
    store_->addFactor(odom2);

    // Second optimization from storage
    EXPECT_TRUE(graph_.optimizeFromStorage(*store_));

    // Verify all keyframes and factors are present in storage
    EXPECT_EQ(store_->getAllKeyFrames().size(), 3);
    EXPECT_EQ(store_->getAllFactors().size(), 3);

    // Verify factor-keyframe associations work
    auto connected_factors = store_->getFactorsForKeyFrame(2);
    EXPECT_EQ(connected_factors.size(), 2);  // Should be connected to both odom factors
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
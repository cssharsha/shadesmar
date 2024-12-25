#include "ros/rosbag_reader.hpp"
#include <gtest/gtest.h>
#include <cstdlib>
#include <filesystem>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <rosbag2_cpp/writer.hpp>
// #include <rosbag2_cpp/writers/sequential_writer.hpp>
// #include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class RosbagReaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set the ROS logging directory to the test temporary directory
        const char* test_tmpdir = std::getenv("TEST_TMPDIR");
        if (test_tmpdir) {
            setenv("ROS_LOG_DIR", test_tmpdir, 1);
            setenv("ROS_HOME", test_tmpdir, 1);
        }

        rclcpp::InitOptions init_options;
        rclcpp::init(0, nullptr, init_options);

        graph_ = std::make_unique<core::graph::FactorGraph>();
        store_ = std::make_unique<core::storage::MapStore>();

        // Update config with new structure
        config_.odom_topic = "/base/odom";
        config_.color_topic = "/camera/camera/color/image_raw";
        config_.camera_info_topic = "/camera/camera/color/camera_info";
        config_.keyframe_distance_threshold = 0.1;  // 10cm default
    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    std::string test_bag_path_ = std::string(getenv("TEST_SRCDIR")) + "/" +
                                 std::string(getenv("TEST_WORKSPACE")) +
                                 "/ros/test/data/test_sequence.db3";
    ros::Config config_;
    std::unique_ptr<core::graph::FactorGraph> graph_;
    std::unique_ptr<core::storage::MapStore> store_;
};

TEST_F(RosbagReaderTest, Initialize) {
    ros::RosbagReader reader(test_bag_path_, *graph_, *store_, config_);
    EXPECT_TRUE(reader.initialize());
}

TEST_F(RosbagReaderTest, ProcessEntireBag) {
    ros::RosbagReader reader(test_bag_path_, *graph_, *store_, config_);
    EXPECT_TRUE(reader.initialize());
    EXPECT_TRUE(reader.processBag());

    const auto& store = *store_;

    // Verify that keyframes were created
    const auto keyframes = store.getAllKeyFrames();
    EXPECT_GT(keyframes.size(), 0);

    // Verify that factors were created
    const auto factors = store.getAllFactors();
    EXPECT_GT(factors.size(), 0);

    // Verify keyframe spacing
    for (size_t i = 1; i < keyframes.size(); i++) {
        const auto& kf1 = keyframes[i - 1];
        const auto& kf2 = keyframes[i];

        double dx = kf2->pose.position.x() - kf1->pose.position.x();
        double dy = kf2->pose.position.y() - kf1->pose.position.y();
        double dz = kf2->pose.position.z() - kf1->pose.position.z();

        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        EXPECT_GE(distance, config_.keyframe_distance_threshold);
    }
}

TEST_F(RosbagReaderTest, InvalidBagFile) {
    ros::RosbagReader reader("/nonexistent/path.db3", *graph_, *store_, config_);
    EXPECT_FALSE(reader.initialize());
}

TEST_F(RosbagReaderTest, DifferentDistanceThresholds) {
    // Test with larger threshold
    config_.keyframe_distance_threshold = 0.2;  // 20cm
    ros::RosbagReader reader1(test_bag_path_, *graph_, *store_, config_);
    EXPECT_TRUE(reader1.initialize());
    EXPECT_TRUE(reader1.processBag());
    size_t keyframes_with_large_threshold = store_->getAllKeyFrames().size();

    // Reset store and graph
    store_ = std::make_unique<core::storage::MapStore>();
    graph_ = std::make_unique<core::graph::FactorGraph>();

    // Test with smaller threshold
    config_.keyframe_distance_threshold = 0.05;  // 5cm
    ros::RosbagReader reader2(test_bag_path_, *graph_, *store_, config_);
    EXPECT_TRUE(reader2.initialize());
    EXPECT_TRUE(reader2.processBag());
    size_t keyframes_with_small_threshold = store_->getAllKeyFrames().size();

    // We should get more keyframes with a smaller threshold
    EXPECT_GT(keyframes_with_small_threshold, keyframes_with_large_threshold);
}

TEST_F(RosbagReaderTest, SaveAndLoadFactorGraph) {
    // Process the bag first
    ros::RosbagReader reader(test_bag_path_, *graph_, *store_, config_);
    EXPECT_TRUE(reader.initialize());
    EXPECT_TRUE(reader.processBag());

    // Get the original data
    const auto original_keyframes = store_->getAllKeyFrames();
    const auto original_factors = store_->getAllFactors();
    EXPECT_GT(original_keyframes.size(), 0);
    EXPECT_GT(original_factors.size(), 0);

    // Save to a file in the same directory as the bag file
    std::filesystem::path bag_path(test_bag_path_);
    const std::string test_file = (bag_path.parent_path() / "test_map.pb").string();
    EXPECT_TRUE(store_->save(test_file));

    // Create new store and load the file
    auto new_store = std::make_unique<core::storage::MapStore>();
    EXPECT_TRUE(new_store->load(test_file));

    // Compare keyframes
    const auto loaded_keyframes = new_store->getAllKeyFrames();
    EXPECT_EQ(loaded_keyframes.size(), original_keyframes.size());

    for (size_t i = 0; i < original_keyframes.size(); ++i) {
        const auto& original = original_keyframes[i];
        const auto loaded = new_store->getKeyFrame(original->id);
        ASSERT_NE(loaded, nullptr);

        // Compare keyframe properties
        EXPECT_EQ(loaded->id, original->id);
        EXPECT_NEAR((loaded->pose.position - original->pose.position).norm(), 0, 1e-9);
        EXPECT_NEAR(
            (loaded->pose.orientation.coeffs() - original->pose.orientation.coeffs()).norm(), 0,
            1e-9);

        // Compare point clouds if they exist
        if (std::holds_alternative<core::types::PointCloud>(original->depth_data)) {
            const auto& original_cloud = std::get<core::types::PointCloud>(original->depth_data);
            const auto& loaded_cloud = std::get<core::types::PointCloud>(loaded->depth_data);

            EXPECT_EQ(loaded_cloud.points.size(), original_cloud.points.size());
            // Compare first point as a sample
            if (!loaded_cloud.points.empty()) {
                EXPECT_NEAR((loaded_cloud.points[0] - original_cloud.points[0]).norm(), 0, 1e-9);
            }
        }
    }

    // Compare factors
    const auto loaded_factors = new_store->getAllFactors();
    EXPECT_EQ(loaded_factors.size(), original_factors.size());

    for (size_t i = 0; i < original_factors.size(); ++i) {
        const auto& original = original_factors[i];
        const auto& loaded = loaded_factors[i];

        EXPECT_EQ(loaded.id, original.id);
        EXPECT_EQ(loaded.type, original.type);
        EXPECT_EQ(loaded.connected_nodes, original.connected_nodes);

        // Compare information matrices
        EXPECT_NEAR((loaded.information - original.information).norm(), 0, 1e-9);

        // Compare measurements based on factor type
        if (original.type == core::proto::FactorType::ODOMETRY) {
            const auto& original_measurement = std::get<0>(original.measurement);
            const auto& loaded_measurement = std::get<0>(loaded.measurement);
            EXPECT_NEAR((loaded_measurement.position - original_measurement.position).norm(), 0,
                        1e-9);
            EXPECT_NEAR((loaded_measurement.orientation.coeffs() -
                         original_measurement.orientation.coeffs())
                            .norm(),
                        0, 1e-9);
        }
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

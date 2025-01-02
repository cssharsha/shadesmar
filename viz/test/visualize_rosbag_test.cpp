#include <gtest/gtest.h>
#include <iostream>
#include "ros/rosbag_reader.hpp"
#include "viz/rerun_viz.hpp"

class VisualizeBagTest : public ::testing::Test {
protected:
    void SetUp() override {
        visualizer_ = std::make_unique<viz::RerunVisualizer>("rosbag_viz_test");
        ASSERT_TRUE(visualizer_->initialize());

        graph_ = std::make_shared<core::graph::FactorGraph>();
        store_ = std::make_shared<core::storage::MapStore>();
    }

    std::unique_ptr<viz::RerunVisualizer> visualizer_;
    std::shared_ptr<core::graph::FactorGraph> graph_;
    std::shared_ptr<core::storage::MapStore> store_;
};

TEST_F(VisualizeBagTest, ProcessAndVisualizeRosbag) {
    // Configure rosbag reader
    ros::Config config;
    config.odom_topic = "/base/odom";
    config.color_topic = "/camera/camera/color/image_raw";
    config.camera_info_topic = "/camera/camera/color/camera_info";
    config.keyframe_distance_threshold = 0.1;

    // Get path to test bag file from environment variable
    const char* bag_path = std::getenv("TEST_ROSBAG_PATH");
    ASSERT_NE(bag_path, nullptr) << "TEST_ROSBAG_PATH environment variable not set";

    // Create and initialize rosbag reader
    ros::RosbagReader reader(bag_path, *graph_, *store_, config);
    ASSERT_TRUE(reader.initialize());

    // Process the bag and build the factor graph
    ASSERT_TRUE(reader.processBag());

    // Visualize the factor graph
    visualizer_->visualizeFactorGraph(*graph_);
    visualizer_->update();
}

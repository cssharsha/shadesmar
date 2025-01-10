#include <rclcpp/rclcpp.hpp>
#include "ros/ros_adapter.hpp"
#include "viz/rerun_viz.hpp"

int main(int argc, char** argv) {
    // Initialize ROS first
    rclcpp::init(argc, argv);

    std::cout << "Initializing visualizer..." << std::endl;
    auto visualizer = std::make_shared<viz::RerunVisualizer>("live_viz", "localhost", 9999);
    if (!visualizer->initialize()) {
        std::cerr << "Failed to initialize visualizer" << std::endl;
        return 1;
    }

    core::graph::FactorGraph graph;
    core::storage::MapStore store;
    ros::Config config;
    config.odom_topic = "/base/odom";
    config.tf_topic = "/tf";
    config.tf_static_topic = "/tf_static";
    config.color_topic = "/camera/camera/color/image_raw";
    config.camera_info_topic = "/camera/camera/color/camera_info";
    config.keyframe_distance_threshold = 0.05;
    config.odom_frame_id = "odom";
    config.base_link_frame_id = "base_link";
    config.camera_frame_id = "camera_color_optical_frame";
    config.depth_topic = "/camera/camera/depth/image_rect_raw";
    config.depth_camera_info_topic = "/camera/camera/depth/camera_info";
    config.point_cloud_topic = "/camera/pointcloud";

    std::cout << "Creating ROS adapter..." << std::endl;
    auto adapter = std::make_shared<ros::RosAdapter>(config, graph, store, visualizer);
    std::cout << "Initializing adapter..." << std::endl;
    if (!adapter->initialize()) {
        std::cerr << "Failed to initialize adapter" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // Print available topics at startup
    std::cout << "\nDiscovered topics:" << std::endl;
    adapter->printAvailableTopics();

    std::cout << "Starting to spin..." << std::endl;
    rclcpp::spin(adapter);

    rclcpp::shutdown();
    return 0;
}

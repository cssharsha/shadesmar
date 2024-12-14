#include <iostream>
#include "ros/rosbag_reader.hpp"
#include "viz/rerun_viz.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_rosbag>" << std::endl;
        return 1;
    }

    std::cout << "Initializing visualizer..." << std::endl;
    // Initialize visualizer
    viz::RerunVisualizer visualizer("rosbag_viz");
    if (!visualizer.initialize()) {
        std::cerr << "Failed to initialize visualizer" << std::endl;
        return 1;
    }

    // Create factor graph and map store
    auto graph = std::make_shared<core::graph::FactorGraph>();
    auto store = std::make_shared<core::storage::MapStore>();

    // Configure rosbag reader
    ros::Config config;
    config.odom_topic = "/base/odom";
    config.color_topic = "/camera/camera/color/image_raw";
    config.camera_info_topic = "/camera/camera/color/camera_info";
    config.keyframe_distance_threshold = 0.1;  // 10cm between keyframes

    // Create and initialize rosbag reader
    ros::RosbagReader reader(argv[1], *graph, *store, config);
    if (!reader.initialize()) {
        std::cerr << "Failed to initialize rosbag reader" << std::endl;
        return 1;
    }

    // Process the bag and build the factor graph
    std::cout << "Processing rosbag..." << std::endl;
    if (!reader.processBag()) {
        std::cerr << "Failed to process rosbag" << std::endl;
        return 1;
    }

    // Visualize the factor graph
    std::cout << "Visualizing factor graph..." << std::endl;
    visualizer.visualizeFactorGraph(*graph);
    visualizer.update();

    // Keep the program running to maintain visualization
    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();

    return 0;
}

#include <iostream>
#include <thread>
#include "ros/rosbag_reader.hpp"
#include "viz/rerun_viz.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_rosbag>" << std::endl;
        return 1;
    }

    // Initialize visualizer for TCP transport
    std::cout << "Initializing visualizer..." << std::endl;
    auto visualizer = std::make_shared<viz::RerunVisualizer>("rosbag_viz", "localhost", 9999);
    if (!visualizer->initialize()) {
        std::cerr << "Failed to initialize visualizer" << std::endl;
        return 1;
    }

    // Set up data structures and configuration
    core::graph::FactorGraph graph;
    core::storage::MapStore store;

    ros::Config config;
    config.odom_topic = "/base/odom";
    config.tf_topic = "/tf";
    config.tf_static_topic = "/tf_static";
    config.color_topic = "/camera/camera/color/image_raw";
    config.camera_info_topic = "/camera/camera/color/camera_info";
    config.point_cloud_topic = "/camera/pointcloud";
    config.base_link_frame_id = "base_link";
    config.camera_frame_id = "camera_color_optical_frame";
    config.keyframe_distance_threshold = 0.05;  // 5cm between keyframes

    // Create and initialize rosbag reader
    std::cout << "Processing rosbag: " << argv[1] << std::endl;
    ros::RosbagReader reader(argv[1], graph, store, config, visualizer);
    if (!reader.initialize()) {
        std::cerr << "Failed to initialize reader" << std::endl;
        return 1;
    }

    // Process the bag file
    if (!reader.processBag()) {
        std::cerr << "Failed to process bag file" << std::endl;
        return 1;
    }

    // Keep visualization window open until user presses Enter
    std::cout << "\nProcessing complete. Press Enter to exit..." << std::endl;
    bool running = true;
    std::thread input_thread([&running]() {
        std::cin.get();
        running = false;
    });

    // Update visualization at 2Hz until user exits
    while (running) {
        visualizer->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    input_thread.join();
    return 0;
}

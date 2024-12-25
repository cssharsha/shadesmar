#include <iostream>
#include "ros/rosbag_reader.hpp"
#include "viz/rerun_viz.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_rosbag>" << std::endl;
        return 1;
    }

    std::cout << "Initializing visualizer..." << std::endl;
    // Initialize visualizer for TCP transport
    auto visualizer = std::make_shared<viz::RerunVisualizer>("rosbag_viz", "localhost", 9999);
    if (!visualizer->initialize()) {
        std::cerr << "Failed to initialize visualizer" << std::endl;
        return 1;
    }

    bool running = true;
    std::cout << "Press Enter to exit..." << std::endl;

    std::thread input_thread([&running]() {
        std::cin.get();
        running = false;
    });

    core::graph::FactorGraph graph;
    core::storage::MapStore store;
    ros::Config config;
    config.odom_topic = "/base/odom";
    config.tf_topic = "/base/tf";
    config.tf_static_topic = "/base/tf_static";
    config.color_topic = "/camera/camera/color/image_raw";
    config.camera_info_topic = "/camera/camera/color/camera_info";

    // Create RosbagReader with shared_ptr to visualizer
    ros::RosbagReader reader(argv[1], graph, store, config, visualizer);
    if (!reader.initialize()) {
        std::cerr << "Failed to initialize reader" << std::endl;
        return 1;
    }
    reader.processBag();

    // while (running) {
    //     visualizer->update();
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 2 Hz
    // }

    input_thread.join();

    return 0;
}

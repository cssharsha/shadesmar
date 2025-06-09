#include <iostream>
#include <thread>
#include "ros/rosbag_reader.hpp"
#include "viz/rerun_viz.hpp"
#include "viz/config_loader.hpp"

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: " << argv[0] << " <path_to_rosbag> [config_type]" << std::endl;
        std::cerr << "  config_type: 'default', 'tum', 'tum_alt' (default: 'default')" << std::endl;
        std::cerr << "Examples:" << std::endl;
        std::cerr << "  " << argv[0] << " /path/to/bag.db3" << std::endl;
        std::cerr << "  " << argv[0] << " /path/to/tum_bag.db3 tum" << std::endl;
        std::cerr << "  " << argv[0] << " /path/to/tum_bag.db3 tum_alt" << std::endl;
        return 1;
    }

    // Determine configuration type
    std::string config_type = (argc == 3) ? argv[2] : "default";

    // Load configuration
    viz::DatasetConfig dataset_config;
    try {
        if (config_type == "tum") {
            dataset_config = viz::ConfigLoader::loadTUM();
        } else if (config_type == "tum_alt") {
            dataset_config = viz::ConfigLoader::loadTUMAlt();
        } else if (config_type == "default") {
            dataset_config = viz::ConfigLoader::loadDefault();
        } else {
            // Try to load as file path
            dataset_config = viz::ConfigLoader::loadFromFile(config_type);
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to load configuration '" << config_type << "': " << e.what() << std::endl;
        std::cerr << "Available options: 'default', 'tum', 'tum_alt', or path to custom config file" << std::endl;
        return 1;
    }

    // Initialize visualizer for TCP transport
    std::cout << "Initializing visualizer..." << std::endl;
    auto visualizer = std::make_shared<viz::RerunVisualizer>("rosbag_viz", "localhost", 9999);
    if (!visualizer->initialize()) {
        std::cerr << "Failed to initialize visualizer" << std::endl;
        return 1;
    }

    // Set up data structures - only need MapStore, FactorGraph handled internally
    core::storage::MapStore store("/data/robots");

    // Create and initialize rosbag reader with loaded configuration
    std::cout << "Processing rosbag: " << argv[1] << std::endl;
    std::cout << "Using configuration: " << dataset_config.dataset_name
              << " (" << dataset_config.description << ")" << std::endl;

    // Print key configuration details
    const auto& config = dataset_config.ros_config;
    std::cout << "\nTopic Configuration:" << std::endl;
    std::cout << "  Color: " << config.color_topic << std::endl;
    std::cout << "  Camera Info: " << config.camera_info_topic << std::endl;
    std::cout << "  IMU: " << config.imu_topic << std::endl;
    std::cout << "  TF: " << config.tf_topic << std::endl;
    if (!config.odom_topic.empty()) {
        std::cout << "  Odometry: " << config.odom_topic << std::endl;
    } else if (dataset_config.use_tf_for_poses) {
        std::cout << "  Pose source: TF transforms" << std::endl;
    } else {
        std::cout << "  Pose source: Visual odometry" << std::endl;
    }

    std::cout << "\nKeyframe Thresholds:" << std::endl;
    std::cout << "  Distance: " << dataset_config.keyframe_thresholds.distance_threshold << "m" << std::endl;
    std::cout << "  Rotation: " << dataset_config.keyframe_thresholds.rotation_threshold << "°" << std::endl;
    std::cout << "  IMU Motion: " << (dataset_config.keyframe_thresholds.use_imu_motion ? "enabled" : "disabled") << std::endl;
    std::cout << "  Visual Odometry: " << (dataset_config.keyframe_thresholds.use_visual_odometry ? "enabled" : "disabled") << std::endl;

    std::cout << "\nOptimization Configuration:" << std::endl;
    std::cout << "  Background Optimization: " << (dataset_config.enable_background_optimization ? "enabled" : "disabled") << std::endl;
    std::cout << "  Keyframe Interval: " << dataset_config.optimization_keyframe_interval << " keyframes" << std::endl;

    ros::RosbagReader reader(argv[1], store, dataset_config.ros_config, visualizer);

    // Configure keyframe thresholds
    reader.setKeyframeThresholds(dataset_config.keyframe_thresholds);

    // Configure optimization thread
    reader.setOptimizationConfig(dataset_config.enable_background_optimization,
                                dataset_config.optimization_keyframe_interval);

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

#include <iostream>
#include <memory>  // For std::make_shared
#include <string>

#include "core/graph/factor_graph.hpp"
#include "core/storage/map_store.hpp"
#include "kitti/config.hpp"
#include "kitti/kitti_reader.hpp"
#include "viz/rerun_viz.hpp"  // Assuming RerunVisualizer is used

// Basic error handling
void usage() {
    std::cerr << "Usage: kitti_reader_example <sequence_number> [dataset_base_path]" << std::endl;
    std::cerr << "Example: kitti_reader_example 00 /data/robot/kitti/data/odom" << std::endl;
    std::cerr
        << "If dataset_base_path is not provided, it defaults to the one in KittiReaderConfig."
        << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 2 || argc > 3) {
        usage();
        return 1;
    }

    kitti::KittiReaderConfig config;
    config.sequence_number = argv[1];  // e.g., "00", "01", ...

    if (argc == 3) {
        config.dataset_base_path = argv[2];
    }
    // Otherwise, config.dataset_base_path uses its default from config.hpp

    // Set additional configuration parameters
    config.keyframe_distance_threshold = 1.0;  // meters
    config.max_frames = 0;                     // Process all frames (0 = no limit)
    config.frame_delay_ms = 0;                 // No delay between frames
    config.load_images = true;
    config.load_point_clouds = false;  // Not using LiDAR for now

    std::cout << "Running KITTI Reader Example:" << std::endl;
    std::cout << "  Sequence: " << config.sequence_number << std::endl;
    std::cout << "  Dataset Path: " << config.dataset_base_path << std::endl;

    // Initialize core components
    core::graph::FactorGraph graph;
    core::storage::MapStore store;

    // Initialize Rerun visualizer (optional, but good for seeing output)
    std::cout << "Initializing visualizer..." << std::endl;
    auto visualizer = std::make_shared<viz::RerunVisualizer>("rosbag_viz", "localhost", 9999);
    if (!visualizer->initialize()) {
        std::cerr << "Failed to initialize visualizer" << std::endl;
        return 1;
    }

    // Initialize KittiReader
    kitti::KittiReader reader(config, graph, store, visualizer);

    if (!reader.initialize()) {
        std::cerr << "Failed to initialize KittiReader." << std::endl;
        return 1;
    }

    // Process the sequence
    if (!reader.processSequence()) {
        std::cerr << "Failed to process KITTI sequence." << std::endl;
        return 1;
    }

    std::cout << "Finished processing sequence " << config.sequence_number << "." << std::endl;

    // You might want to keep the application running to view data in Rerun,
    // or add a wait here. For a batch processing example, it can just exit.
    std::cout << "Data processing complete. Check Rerun viewer." << std::endl;
    // visualizer->disconnect(); // Optional: explicitly disconnect
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

    return 0;
}

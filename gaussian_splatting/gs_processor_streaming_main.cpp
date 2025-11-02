#include <signal.h>
#include <chrono>
#include <iostream>
#include <logging/logging.hpp>
#include <string>
#include <thread>
#include "gaussian_splatting/streaming_gs_processor.hpp"

std::unique_ptr<gaussian_splatting::StreamingGS> gs_processor;

bool use_training_processor = true;  // Default to training processor

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <map_base_path> [OPTIONS]\n";
    std::cout << "  map_base_path: Base path for map data files (e.g., /data/robot/map)\n";
    std::cout << "\nOptions:\n";
    std::cout
        << "  --no-spatial-partitioning  Train all splats together without spatial partitioning\n";
    std::cout << "  --spatial-partitioning     Use spatial partitioning (default)\n";
    std::cout << "\nExamples:\n";
    std::cout << "  " << program_name << " /data/robot/house11_map\n";
    std::cout << "    # Use spatial partitioning (default)\n";
    std::cout << "  " << program_name << " /data/robot/house11_map --no-spatial-partitioning\n";
    std::cout << "    # Train all splats together without partitioning\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2 || argc > 3) {
        printUsage(argv[0]);
        return 1;
    }

    std::string map_base_path = argv[1];
    bool use_spatial_partitioning = true;  // Default to true

    // Parse command line arguments
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--no-spatial-partitioning") {
            use_spatial_partitioning = false;
            std::cout << "Spatial partitioning disabled - training all splats together"
                      << std::endl;
        } else if (arg == "--spatial-partitioning") {
            use_spatial_partitioning = true;
            std::cout << "Spatial partitioning enabled" << std::endl;
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    std::cout << "Starting Gaussian Splat Processor with map path: " << map_base_path << std::endl;

    // Create and configure the streaming processor configuration
    gaussian_splatting::Config config;
    config.map_base_path = map_base_path;
    config.base_link = "base_link";
    // config.camera_frame = "camera_color_optical_frame";
    config.camera_frame = "camera";
    config.use_spatial_partitioning = use_spatial_partitioning;

    // Create and initialize training processor
    gs_processor = std::make_unique<gaussian_splatting::StreamingGS>(config);
    if (!gs_processor) {
        std::cout << "Should have got the proper processor" << std::endl;
        return 1;
    }
    std::cout << "Inited the main" << std::endl;
    gs_processor->initializeStore();
    std::cout << "Finished all loading!!" << std::endl;
    if (!gs_processor->streamAndTrain()) {
        std::cerr << "Did not train" << std::endl;
    }
    std::cout << "Finished training" << std::endl;

    return 0;
}

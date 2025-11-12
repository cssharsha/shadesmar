// Include streaming_gs_processor.hpp first to avoid macro conflicts with Foxglove
#include "gaussian_splatting/streaming_gs_processor.hpp"

#include <signal.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <logging/logging.hpp>

std::unique_ptr<gaussian_splatting::StreamingGS> gs_processor;

bool use_training_processor = true;  // Default to training processor

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <map_base_path> [OPTIONS]\n";
    std::cout << "  map_base_path: Base path for map data files (e.g., /data/robot/map)\n";
    std::cout << "\nTraining Options:\n";
    std::cout
        << "  --no-spatial-partitioning  Train all splats together without spatial partitioning\n";
    std::cout << "  --spatial-partitioning     Use spatial partitioning (default)\n";
    std::cout << "\nFoxglove Visualization Options:\n";
    std::cout << "  --enable-foxglove          Enable Foxglove interactive renderer (default)\n";
    std::cout << "  --disable-foxglove         Disable Foxglove interactive renderer\n";
    std::cout << "  --foxglove-host <host>     Foxglove server host (default: 0.0.0.0)\n";
    std::cout << "  --foxglove-port <port>     Foxglove server port (default: 8765)\n";
    std::cout << "\nExamples:\n";
    std::cout << "  " << program_name << " /data/robot/house11_map\n";
    std::cout << "    # Use defaults (spatial partitioning, Foxglove enabled on 0.0.0.0:8765)\n";
    std::cout << "  " << program_name << " /data/robot/house11_map --disable-foxglove\n";
    std::cout << "    # Disable Foxglove interactive rendering\n";
    std::cout << "  " << program_name << " /data/robot/house11_map --foxglove-port 9000\n";
    std::cout << "    # Use custom Foxglove port\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string map_base_path = argv[1];
    bool use_spatial_partitioning = true;   // Default to true
    bool enable_foxglove_renderer = true;   // Default to true
    std::string foxglove_host = "0.0.0.0";  // Default bind to all interfaces
    int foxglove_port = 8765;               // Default Foxglove port

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
        } else if (arg == "--enable-foxglove") {
            enable_foxglove_renderer = true;
            std::cout << "Foxglove interactive renderer enabled" << std::endl;
        } else if (arg == "--disable-foxglove") {
            enable_foxglove_renderer = false;
            std::cout << "Foxglove interactive renderer disabled" << std::endl;
        } else if (arg == "--foxglove-host") {
            if (i + 1 < argc) {
                foxglove_host = argv[++i];
                std::cout << "Foxglove host: " << foxglove_host << std::endl;
            } else {
                std::cerr << "Error: --foxglove-host requires a value" << std::endl;
                printUsage(argv[0]);
                return 1;
            }
        } else if (arg == "--foxglove-port") {
            if (i + 1 < argc) {
                foxglove_port = std::stoi(argv[++i]);
                std::cout << "Foxglove port: " << foxglove_port << std::endl;
            } else {
                std::cerr << "Error: --foxglove-port requires a value" << std::endl;
                printUsage(argv[0]);
                return 1;
            }
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

    // Foxglove configuration
    config.enable_foxglove_renderer = enable_foxglove_renderer;
    config.foxglove_host = foxglove_host;
    config.foxglove_port = foxglove_port;

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

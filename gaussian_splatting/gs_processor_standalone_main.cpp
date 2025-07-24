#include <signal.h>
#include <chrono>
#include <iostream>
#include <logging/logging.hpp>
#include <string>
#include <thread>
#include "standalone_gs_processor.hpp"

std::unique_ptr<gaussian_splatting::StandaloneGs> gs_processor;

bool use_training_processor = true;  // Default to training processor

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <map_base_path> [--legacy]\n";
    std::cout << "  map_base_path: Base path for map data files (e.g., /data/robot/map)\n";
    std::cout << "  --legacy: Use legacy GaussianSplatProcessor instead of training processor\n";
    std::cout << "\nExamples:\n";
    std::cout << "  " << program_name
              << " /data/robot/house11_map          # Use training processor (default)\n";
    std::cout << "  " << program_name
              << " /data/robot/house11_map --legacy # Use legacy processor\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2 || argc > 3) {
        printUsage(argv[0]);
        return 1;
    }

    std::string map_base_path = argv[1];

    std::cout << "Starting Gaussian Splat Processor with map path: " << map_base_path << std::endl;

    // Create and configure the streaming processor configuration
    gaussian_splatting::Config config;
    config.map_base_path = map_base_path;

    // Create and initialize training processor
    gs_processor = std::make_unique<gaussian_splatting::StandaloneGs>(config);
    if (!gs_processor) {
        std::cout << "Should have got the proper processor" << std::endl;
        return 1;
    }
    std::cout << "Inited the main" << std::endl;
    gs_processor->initializeStore();
    std::cout << "Finished all loading!!" << std::endl;
    if (!gs_processor->loadAndTrain()) {
        std::cerr << "Did not train" << std::endl;
    }
    std::cout << "Finished training" << std::endl;

    return 0;
}

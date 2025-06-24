#include <iostream>
#include <signal.h>
#include <string>
#include <thread>
#include <chrono>
#include "gs_processor.hpp"
#include "viz/visualize_gs_rerun.hpp"
#include "core/storage/map_store.hpp"
#include <logging/logging.hpp>

std::unique_ptr<gaussian_splatting::GaussianSplatProcessor> g_processor;
std::unique_ptr<viz::GaussianSplatRerunVisualizer> g_visualizer;

void signalHandler(int signal) {
    LOG(INFO) << "Received signal " << signal << ", shutting down Gaussian splat processor...";
    if (g_visualizer) {
        g_visualizer->disconnect();
    }
    if (g_processor) {
        g_processor->stop();
    }
    exit(0);
}

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <map_base_path>\n";
    std::cout << "  map_base_path: Base path for map data files (e.g., /data/robot/map)\n";
    std::cout << "\nExample:\n";
    std::cout << "  " << program_name << " /data/robot/house11_map\n";
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string map_base_path = argv[1];
    
    LOG(INFO) << "Starting Gaussian Splat Processor with map path: " << map_base_path;

    // Set up signal handlers for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        // Create processor configuration
        gaussian_splatting::ProcessorConfig config(map_base_path);
        
        // Create and initialize processor
        g_processor = std::make_unique<gaussian_splatting::GaussianSplatProcessor>(config);
        
        if (!g_processor->initialize()) {
            LOG(ERROR) << "Failed to initialize Gaussian splat processor";
            return 1;
        }

        // Create MapStore instance for the visualizer
        auto map_store = std::make_shared<core::storage::MapStore>(map_base_path);
        
        // Create and initialize Gaussian splat visualizer with shared recording ID
        // This allows both rosbag and GS processes to publish to the same Rerun viewer
        std::string shared_recording_id = "shadesmar_combined";
        g_visualizer = std::make_unique<viz::GaussianSplatRerunVisualizer>(
            map_store, shared_recording_id, "gaussian_splats");
        
        if (!g_visualizer->initialize()) {
            LOG(WARNING) << "Failed to initialize Gaussian splat visualizer, continuing without visualization";
            g_visualizer.reset();
        } else {
            // Attach visualizer callback to processor
            g_processor->setSplatBatchCallback([&]() {
                if (g_visualizer) {
                    g_visualizer->notifyNewSplatBatch();
                }
            });
            LOG(INFO) << "Gaussian splat visualizer initialized with recording_id: " << shared_recording_id
                      << " and attached to processor";
        }

        // Start processing
        if (!g_processor->start()) {
            LOG(ERROR) << "Failed to start Gaussian splat processor";
            return 1;
        }

        LOG(INFO) << "Gaussian splat processor started successfully";
        LOG(INFO) << "Press Ctrl+C to stop...";

        // Keep main thread alive
        while (g_processor->isRunning()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception in Gaussian splat processor: " << e.what();
        return 1;
    }

    LOG(INFO) << "Gaussian splat processor finished";
    return 0;
}
#include <signal.h>
#include <chrono>
#include <iostream>
#include <logging/logging.hpp>
#include <string>
#include <thread>
#include "core/storage/map_store.hpp"
#include "gs_processor.hpp"
#include "streaming_gs_processor.hpp"
// #include "training_gs_processor.hpp"
#include "viz/visualize_gs_rerun.hpp"

std::unique_ptr<gaussian_splatting::GaussianSplatProcessor> g_processor;
std::unique_ptr<gaussian_splatting::DefaultStreamingGSProcessor> g_training_processor;
std::unique_ptr<viz::GaussianSplatRerunVisualizer> g_visualizer;
bool use_training_processor = true;  // Default to training processor

void signalHandler(int signal) {
    LOG(INFO) << "Received signal " << signal << ", shutting down Gaussian splat processor...";
    if (g_visualizer) {
        g_visualizer->disconnect();
    }
    if (g_training_processor) {
        g_training_processor->stop();
    }
    if (g_processor) {
        g_processor->stop();
    }
    exit(0);
}

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

    // Check for legacy flag
    if (argc == 3 && std::string(argv[2]) == "--legacy") {
        use_training_processor = false;
        LOG(INFO) << "Using legacy GaussianSplatProcessor";
    } else {
        LOG(INFO) << "Using TrainingGaussianSplatProcessor (default)";
    }

    LOG(INFO) << "Starting Gaussian Splat Processor with map path: " << map_base_path;

    // Set up signal handlers for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        if (use_training_processor) {
            // Create and configure the streaming processor configuration
            gaussian_splatting::DefaultStreamingGSProcessor::ConfigType training_config;
            training_config.map_base_path = map_base_path;

            // Set device
            if (torch::cuda::is_available()) {
                training_config.device = torch::kCUDA;
            }

            // Configure training settings
            training_config.max_training_iterations = 100;  // As in original code
            training_config.learning_rate = 0.01;
            training_config.training_frequency = 1;

            // Configure sliding window (using default values from SlidingWindowConfig)
            training_config.window_config.max_window_size = 50;
            training_config.window_config.min_window_size = 3;

            // Configure convergence (using default values from ConvergenceConfig)
            training_config.convergence_config.enable_early_stopping = true;
            training_config.convergence_config.convergence_check_interval = 10;

            // Configure splat initialization (using default values from
            // SplatInitializationConfig)
            training_config.init_config.initial_splat_count = 10000;
            training_config.init_config.adaptive_density = true;

            // Configure visualization
            training_config.training_viz_recording_id = "streaming_gs_training";
            training_config.training_viz_host = "127.0.0.1";
            training_config.training_viz_port = 9876;

            // Configure polling intervals
            training_config.main_loop_interval_ms = 500;
            training_config.training_loop_interval_ms = 100;
            training_config.visualization_loop_interval_ms = 1000;

            // Create and initialize training processor
            g_training_processor =
                std::make_unique<gaussian_splatting::DefaultStreamingGSProcessor>(training_config);

            if (!g_training_processor->initialize()) {
                LOG(ERROR) << "Failed to initialize training Gaussian splat processor";
                return 1;
            }
        }

        // Start processing
        if (use_training_processor) {
            if (!g_training_processor->start()) {
                LOG(ERROR) << "Failed to start training Gaussian splat processor";
                return 1;
            }

            LOG(INFO) << "Training Gaussian splat processor started successfully";
            LOG(INFO) << "Training with visualization enabled - check Rerun viewer for progress";
            LOG(INFO) << "Press Ctrl+C to stop...";

            // Keep main thread alive
            while (g_training_processor->isRunning()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        } else {
            if (!g_processor->start()) {
                LOG(ERROR) << "Failed to start legacy Gaussian splat processor";
                return 1;
            }

            LOG(INFO) << "Legacy Gaussian splat processor started successfully";
            LOG(INFO) << "Press Ctrl+C to stop...";

            // Keep main thread alive
            while (g_processor->isRunning()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception in Gaussian splat processor: " << e.what();
        return 1;
    }

    if (use_training_processor) {
        LOG(INFO) << "Training Gaussian splat processor finished";
    } else {
        LOG(INFO) << "Legacy Gaussian splat processor finished";
    }
    return 0;
}

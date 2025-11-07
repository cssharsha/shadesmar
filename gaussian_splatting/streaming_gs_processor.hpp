#pragma once

#include <memory>
#include "core/storage/map_store.hpp"
#include "gaussian_splatting/training/batch_trainer.hpp"
#include "gaussian_splatting/training/training_config.hpp"
#include "gaussian_splatting/visualization/training_visualizer.hpp"
#include "gaussian_splatting/visualization/foxglove_renderer.hpp"
#include "stf/transform_tree.hpp"

namespace gaussian_splatting {

struct Config {
    // Device configuration
    torch::Device device = torch::kCUDA;

    // File paths
    std::string map_base_path;
    std::string debug_output_path = "";  // If empty, derived from map_base_path + "/debug/"
    std::string training_viz_recording_id = "streaming_gs_training";
    std::string training_viz_host = "127.0.0.1";
    int training_viz_port = 9876;

    // Foxglove interactive rendering
    bool enable_foxglove_renderer = true;
    std::string foxglove_host = "0.0.0.0";
    int foxglove_port = 8765;

    std::string camera_frame = "camera";
    std::string base_link = "base_link";

    // Training configuration
    int max_training_iterations = 1000;
    double learning_rate = 0.01;
    int training_frequency = 1;  // Train every N keyframes
    bool use_spatial_partitioning = true;  // If false, train all splats together without batching

    // Scene bounds (fallback if trajectory estimation fails)
    Eigen::Vector3f scene_min{-10.0f, -10.0f, -10.0f};
    Eigen::Vector3f scene_max{10.0f, 10.0f, 10.0f};

    // Polling intervals
    int main_loop_interval_ms = 500;
    int training_loop_interval_ms = 100;
    int visualization_loop_interval_ms = 1000;
};

class StreamingGS {
public:
    StreamingGS(Config& config);
    double getCurrentTimestamp();
    bool loadAndTrain();
    bool streamAndTrain();
    bool executeIncrementalTraining();
    bool initializeVisualization();
    void initializeStore();
    void visualizeAllKeyframes();

private:
    Config config_;
    std::shared_ptr<core::storage::MapStore> map_store_;
    std::shared_ptr<stf::TransformTree> tf_tree_;

    std::atomic<uint64_t> next_splat_id_{0};
    std::atomic<uint64_t> batch_id_{0};

    std::unique_ptr<training::BatchTrainer> batch_trainer_;
    training::TrainingConfig training_config_;

    std::shared_ptr<visualization::RerunTrainingVisualizer> training_visualizer_;
    std::unique_ptr<visualization::FoxgloveRenderer> foxglove_renderer_;
};

}  // namespace gaussian_splatting

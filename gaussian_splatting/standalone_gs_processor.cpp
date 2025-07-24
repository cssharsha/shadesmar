#include <logging/logging.hpp>
#include <memory>

#include "core/storage/map_store.hpp"
#include "gaussian_splatting/training/batch_trainer.hpp"
#include "gaussian_splatting/utils/image_utils.hpp"
#include "gaussian_splatting/utils/initializers.hpp"
#include "standalone_gs_processor.hpp"

namespace gaussian_splatting {

bool StandaloneGs::initializeVisualization() {
    std::cout << "Initializing visualization system..." << std::endl;

    try {
        // Create RerunTrainingVisualizer with config parameters
        training_visualizer_ = std::make_shared<visualization::RerunTrainingVisualizer>(
            config_.training_viz_recording_id, config_.training_viz_host,
            config_.training_viz_port);

        if (!training_visualizer_) {
            std::cerr << "Failed to create RerunTrainingVisualizer" << std::endl;
            return false;
        }

        // Initialize the visualizer
        if (!training_visualizer_->initialize()) {
            std::cerr << "Failed to initialize RerunTrainingVisualizer" << std::endl;
            training_visualizer_.reset();
            return false;
        }

        // Configure visualization settings
        training_visualizer_->enableSplatVisualization(true);
        training_visualizer_->enableBatchVisualization(true);
        training_visualizer_->setVisualizationFrequency(1);  // Update every iteration

        // Set initial training state
        training_visualizer_->visualizeTrainingState("INITIALIZING",
                                                     "Setting up streaming processor");

        std::cout << "Visualization system initialized successfully" << std::endl;
        std::cout << "Recording ID: " << config_.training_viz_recording_id << std::endl;
        LOG(INFO) << "Visualization host: " << config_.training_viz_host << ":"
                  << config_.training_viz_port;

        return true;

    } catch (const std::exception& e) {
        std::cerr << "Exception initializing visualization: " << e.what() << std::endl;
        training_visualizer_.reset();
        return false;
    }
}

StandaloneGs::StandaloneGs(Config& config) : config_(config) {
    training_config_.initial_width = 512;
    training_config_.initial_height = 384;
    training_config_.max_iterations_per_batch = config_.max_training_iterations;
    training_config_.learning_rate = config_.learning_rate;

    LOG(INFO) << "Training configuration set: " << training_config_.initial_width << "x"
              << training_config_.initial_height
              << ", max iterations: " << training_config_.max_iterations_per_batch;
    std::cout << "Using shared recording ID: " << config_.training_viz_recording_id << std::endl;
    // initializeStore();
}

void StandaloneGs::initializeStore() {
    try {
        map_store_ = std::make_shared<core::storage::MapStore>(config_.map_base_path,
                                                               core::storage::ProcessRole::READER);

        if (!map_store_) {
            std::cerr << "Invalid map store init" << std::endl;
            exit(0);
        }

        // tf_tree_ = map_store_->getTransformTree();
        // if (!tf_tree_) {
        //     std::cerr << "Failed to load tf tree" << std::endl;
        //     exit(0);
        // }
        // tf_tree_->printTree();
        tf_tree_ = std::make_shared<stf::TransformTree>();
        tf_tree_->setTransform("base_link", "camera", Eigen::Isometry3d::Identity());
        // tf_tree_->setTransform("camera", "base_link", Eigen::Isometry3d::Identity());
        batch_trainer_ =
            std::make_unique<training::BatchTrainer>(training_config_, map_store_, tf_tree_);
        std::cout << "Batch trainer inited" << std::endl;

        map_store_->syncIndexFromDisk();

        initializeVisualization();
        std::cout << "Finished visualizing" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Unable init map store and tf tree" << e.what() << std::endl;
        exit(0);
    }
    std::cout << "Finished initing stuff" << std::endl;
}

double StandaloneGs::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

void StandaloneGs::visualizeAllKeyframes() {
    std::cout << "Viz all keyframes" << std::endl;
    auto all_keyframes = map_store_->getAllKeyFrames();
    for (const auto& kf : all_keyframes) {
        std::cout << "Keyframe: " << kf->id << std::endl;
        training_visualizer_->visualizeKeyframe(kf->pose, kf->getCameraInfo(),
                                                "/" + std::to_string(kf->id));
    }
}

bool StandaloneGs::loadAndTrain() {
    std::cout << "Getting here at all" << std::endl;
    std::cout << "Getting here at all" << std::endl;
    if (!batch_trainer_) {
        std::cerr << "BatchTrainer not initialized" << std::endl;
        return false;
    }
    std::cout << "Loading and training" << std::endl;

    std::vector<uint64_t> keyframe_ids;

    auto current_timestamp = getCurrentTimestamp();
    core::types::GaussianSplatBatch splat_batch;
    if (!intializeSplatsFromKeypoints(map_store_, batch_id_, current_timestamp, splat_batch,
                                      next_splat_id_, keyframe_ids)) {
        std::cout << "Unbale to load and init gaus splats" << std::endl;
        return false;
    }
    training_visualizer_->visualizeCurrentSplats(splat_batch, 0);
    // while (true) {
    //     std::cout << "Press c to continue" << std::endl;
    //     auto c = getchar();
    //     if (c == 'c') {
    //         break;
    //     }
    //
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }
    // exit(0);

    try {
        std::cout << "Batch first point: " << splat_batch.splats[0].position.transpose()
                  << std::endl;
        auto all_keyframes = map_store_->getAllKeyFrames();
        LOG(INFO) << "Training with " << all_keyframes.size() << " keyframes";
        std::cout << "Training with " << all_keyframes.size() << " keyframes" << std::endl;
        // for (auto& kf : all_keyframes) {
        // std::cout << "Enqueueing keyframe " << kf->id << std::endl;
        training::TrainingResults results;
        // batch_trainer_->trainKeyframe(kf, results);
        batch_trainer_->setupTraining(splat_batch);
        int iteration = 0;
        for (auto& kf : all_keyframes) {
            batch_trainer_->trainKeyframe(kf, results, iteration);
            iteration++;
            std::cout << "Image size: " << results.rendered_image.sizes() << std::endl;
            auto rendered_image = utils::tensorToMat(results.rendered_image[0], false);
            std::string entity_path = "/camera";

            // TODO: Optimize this - copySplatsToBatch causes GPU->CPU->GPU transfer every
            // iteration! Only visualize splats periodically to avoid memory overhead
            batch_trainer_->copySplatsToBatch(splat_batch.splats);
            std::cout << "Splats: " << splat_batch.splats.size() << std::endl;
            training_visualizer_->visualizeCurrentSplats(splat_batch, 0);

            training_visualizer_->visualizeKeyframe(kf->pose, kf->getCameraInfo(), entity_path);
            training_visualizer_->logImage(entity_path, rendered_image, 0);
            utils::writeImageToDirectory(rendered_image, "/data/south-building/debug/rendered/",
                                         std::to_string(kf->id) + ".png");
        }

        std::cout << "Finished logging " << std::endl;
        // }
    } catch (const std::exception& e) {
        std::cerr << "Exception in executeIncrementalTraining: " << e.what() << std::endl;
        return false;
    }
    return true;
}

}  // namespace gaussian_splatting

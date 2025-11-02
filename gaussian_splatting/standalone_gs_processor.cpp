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

        map_store_->syncIndexFromDisk();
        // tf_tree_ = map_store_->getTransformTree();
        // if (!tf_tree_) {
        //     std::cerr << "Failed to load tf tree" << std::endl;
        //     exit(0);
        // }
        // tf_tree_->printTree();
        tf_tree_ = std::make_shared<stf::TransformTree>();
        tf_tree_->setTransform("base_link", "camera", Eigen::Isometry3d::Identity());
        batch_trainer_ =
            std::make_unique<training::BatchTrainer>(training_config_, map_store_, tf_tree_);
        std::cout << "Batch trainer inited" << std::endl;

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
    std::cout << "Loading and training with spatial partitioning" << std::endl;

    std::vector<uint64_t> keyframe_ids;
    auto current_timestamp = getCurrentTimestamp();

    // Initialize all splats from keypoints
    core::types::GaussianSplatBatch full_splat_batch;
    utils::PointCloudUtils point_cloud_utils;

    auto all_keypoints = map_store_->getAllKeyPoints();
    std::cout << "Total keypoints before filtering: " << all_keypoints.size() << std::endl;

    // Filter out keypoints that need triangulation
    std::vector<core::types::Keypoint> valid_keypoints;
    for (const auto& keypoint : all_keypoints) {
        if (!keypoint.needs_triangulation) {
            valid_keypoints.push_back(keypoint);
        }
    }
    std::cout << "Valid keypoints (after triangulation filter): " << valid_keypoints.size()
              << std::endl;

    // Build initial point cloud for computing center
    utils::PointCloudUtils temp_point_cloud;
    for (const auto& keypoint : valid_keypoints) {
        temp_point_cloud.addPoint(keypoint.position.x(), keypoint.position.y(),
                                  keypoint.position.z());
    }
    temp_point_cloud.setupKDTree();

    // Compute initial bounding box and center
    auto initial_bbox = temp_point_cloud.computeBoundingBox();
    Eigen::Vector3f scene_center = initial_bbox.center();
    std::cout << "Initial scene center: [" << scene_center.transpose() << "]" << std::endl;

    // Filter keypoints by distance from center (default 30m)
    std::cout << "\n=== Filtering keypoints by distance from center ===" << std::endl;
    valid_keypoints = filterKeypointsByDistanceFromCenter(valid_keypoints, scene_center, 45.0f);

    // Build point cloud with filtered keypoints for density check
    for (const auto& keypoint : valid_keypoints) {
        point_cloud_utils.addPoint(keypoint.position.x(), keypoint.position.y(),
                                   keypoint.position.z());
    }
    point_cloud_utils.setupKDTree();

    // Filter sparse/non-dense keypoints
    std::cout << "\n=== Filtering sparse keypoints ===" << std::endl;
    valid_keypoints = filterSparseKeypoints(valid_keypoints, point_cloud_utils, 10, 2.0f);

    // Rebuild point cloud with final filtered keypoints
    point_cloud_utils.reset();
    for (const auto& keypoint : valid_keypoints) {
        point_cloud_utils.addPoint(keypoint.position.x(), keypoint.position.y(),
                                   keypoint.position.z());
    }
    point_cloud_utils.setupKDTree();

    std::cout << "\n=== Final filtered keypoint count: " << valid_keypoints.size()
              << " ===" << std::endl;

    // Initialize full splat batch from filtered keypoints
    if (!intializeSplatsFromKeypoints(valid_keypoints, batch_id_, current_timestamp,
                                      full_splat_batch, next_splat_id_, point_cloud_utils)) {
        std::cout << "Unable to load and init gaussian splats" << std::endl;
        return false;
    }

    std::cout << "Initialized " << full_splat_batch.splats.size() << " splats" << std::endl;

    auto all_keyframes = map_store_->getAllKeyFrames();
    LOG(INFO) << "Training with " << all_keyframes.size() << " keyframes";

    // Partition scene into radial sectors with shared center (outside-in viewing)
    // Using more sectors to reduce memory consumption per region
    // auto regions = partitionKeyframesIntoRadialSectors(all_keyframes, valid_keypoints,
    // 16, 22.5f);
    auto regions = partitionKeyframesIntoRadialSectors(all_keyframes, valid_keypoints, 1, 22.5f);

    training_visualizer_->visualizeCurrentSplats(full_splat_batch, 0);
    // for (size_t i = 0; i < regions.size(); i++) {
    //     std::string name = "/bbox_" + std::to_string(i);
    //     training_visualizer_->visualizeBBox(regions[i].bbox_3d, name);
    // }

    try {
        // Train each region separately
        for (size_t region_idx = 0; region_idx < regions.size(); ++region_idx) {
            const auto& region = regions[region_idx];

            std::cout << "\n=== Training Region " << (region_idx + 1) << "/" << regions.size()
                      << " ===" << std::endl;
            std::cout << "Region " << region.region_id << " bounds: min["
                      << region.bbox_3d.min.transpose() << "] max["
                      << region.bbox_3d.max.transpose() << "]" << std::endl;
            training_visualizer_->visualizeBBox(region.bbox_3d, "/bbox");

            // Filter splats for this region
            auto region_splat_batch = filterSplatsByBoundingBox(full_splat_batch, region.bbox_3d);

            if (region_splat_batch.splats.empty()) {
                std::cout << "No splats in region " << (region_idx + 1) << ", skipping..."
                          << std::endl;
                continue;
            }

            std::cout << "Training region " << (region_idx + 1) << " with "
                      << region_splat_batch.splats.size() << " splats and "
                      << region.keyframe_ids.size() << " keyframes" << std::endl;

            // Setup training for this region
            batch_trainer_->setupTraining(region_splat_batch);

            // Visualize region splats
            training_visualizer_->visualizeCurrentSplats(region_splat_batch, region_idx);

            // Train with keyframes in this region for N epochs
            training::TrainingResults results;
            const int num_epochs = 200;

            // Collect keyframe pointers for this region
            std::vector<core::types::KeyFrame::Ptr> region_keyframes;
            for (const auto& kf_id : region.keyframe_ids) {
                auto kf_it = std::find_if(all_keyframes.begin(), all_keyframes.end(),
                                          [kf_id](const auto& kf) { return kf->id == kf_id; });
                if (kf_it != all_keyframes.end()) {
                    region_keyframes.push_back(*kf_it);
                }
            }

            if (region_keyframes.empty()) {
                std::cout << "No valid keyframes in region " << (region_idx + 1) << ", skipping..."
                          << std::endl;
                continue;
            }

            std::cout << "Training for " << num_epochs << " epochs with " << region_keyframes.size()
                      << " keyframes" << std::endl;

            // Training loop: run for num_epochs iterations
            for (int iteration = 0; iteration < num_epochs; ++iteration) {
                // Randomly select a keyframe from this region
                int random_idx = std::rand() % region_keyframes.size();
                auto& kf = region_keyframes[random_idx];

                batch_trainer_->trainKeyframe(kf, results, iteration, training_visualizer_);

                if (iteration % 10 == 0) {
                    std::cout << "Region " << (region_idx + 1) << " - Epoch " << iteration << "/"
                              << num_epochs << std::endl;
                }

                // Visualize results periodically
                // if (iteration % 50 == 0) {
                auto rendered_image = utils::tensorToMat(results.rendered_image[0], false);
                std::string entity_path = "/camera/region_" + std::to_string(region_idx);

                batch_trainer_->copySplatsToBatch(region_splat_batch.splats);
                training_visualizer_->visualizeKeyframe(kf->pose, kf->getCameraInfo(), entity_path);
                training_visualizer_->logImage(entity_path, rendered_image, region_idx);
                // }
            }

            // Copy trained splats back to full batch
            batch_trainer_->copySplatsToBatch(region_splat_batch.splats);
            for (size_t i = 0; i < region_splat_batch.splats.size(); ++i) {
                // Find and update the corresponding splat in full batch
                auto splat_it = std::find_if(
                    full_splat_batch.splats.begin(), full_splat_batch.splats.end(),
                    [&](const auto& s) { return s.id == region_splat_batch.splats[i].id; });
                if (splat_it != full_splat_batch.splats.end()) {
                    *splat_it = region_splat_batch.splats[i];
                }
            }
            training_visualizer_->visualizeCurrentSplats(region_splat_batch, region_idx);

            std::cout << "Finished training region " << (region_idx + 1) << std::endl;
        }

        std::cout << "\n=== Finished training all regions ===" << std::endl;

        // Final visualization with all trained splats
        training_visualizer_->visualizeCurrentSplats(full_splat_batch, 0);

    } catch (const std::exception& e) {
        std::cerr << "Exception in loadAndTrain: " << e.what() << std::endl;
        return false;
    }

    return true;
}

}  // namespace gaussian_splatting

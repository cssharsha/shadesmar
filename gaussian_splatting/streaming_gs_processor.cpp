// Include header first to avoid macro conflicts with foxglove
#include "gaussian_splatting/streaming_gs_processor.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <numeric>
#include <random>
#include <unordered_set>

#include <logging/logging.hpp>

#include "core/storage/map_store.hpp"
#include "gaussian_splatting/training/batch_trainer.hpp"
#include "gaussian_splatting/utils/image_utils.hpp"
#include "gaussian_splatting/utils/initializers.hpp"
#include "gaussian_splatting/utils/splat_logger.hpp"

namespace gaussian_splatting {

bool StreamingGS::initializeVisualization() {
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

        // Initialize FoxgloveRenderer for interactive rendering
        if (config_.enable_foxglove_renderer) {
            LOG(INFO) << "Initializing Foxglove interactive renderer...";
            visualization::FoxgloveRenderer::Config foxglove_config;
            foxglove_config.host = config_.foxglove_host;
            foxglove_config.port = config_.foxglove_port;

            foxglove_renderer_ = std::make_unique<visualization::FoxgloveRenderer>(foxglove_config);

            if (!foxglove_renderer_->initialize()) {
                LOG(ERROR) << "Failed to initialize FoxgloveRenderer";
                foxglove_renderer_.reset();
                // Don't fail entirely - Foxglove is optional
            } else {
                LOG(INFO) << "Foxglove renderer initialized on " << config_.foxglove_host << ":"
                          << config_.foxglove_port;
            }
        }

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

StreamingGS::StreamingGS(Config& config) : config_(config) {
    training_config_.initial_width = 512;
    training_config_.initial_height = 384;
    training_config_.max_iterations_per_batch = config_.max_training_iterations;
    training_config_.learning_rate = config_.learning_rate;

    training_config_.base_link_ = config_.base_link;
    training_config_.camera_frame_ = config_.camera_frame;

    // Set debug output path - derive from map_base_path if not specified
    if (config_.debug_output_path.empty()) {
        training_config_.debug_output_path = config_.map_base_path + "/debug/";
    } else {
        training_config_.debug_output_path = config_.debug_output_path;
    }

    LOG(INFO) << "Training configuration set: " << training_config_.initial_width << "x"
              << training_config_.initial_height
              << ", max iterations: " << training_config_.max_iterations_per_batch;
    LOG(INFO) << "Debug output path: " << training_config_.debug_output_path;
    std::cout << "Using shared recording ID: " << config_.training_viz_recording_id << std::endl;
    // initializeStore();
}

void StreamingGS::initializeStore() {
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

double StreamingGS::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

void StreamingGS::visualizeAllKeyframes() {
    std::cout << "Viz all keyframes" << std::endl;
    auto all_keyframes = map_store_->getAllKeyFrames();
    for (const auto& kf : all_keyframes) {
        std::cout << "Keyframe: " << kf->id << std::endl;
        training_visualizer_->visualizeKeyframe(kf->pose, kf->getCameraInfo(),
                                                "/" + std::to_string(kf->id));
    }
}

bool StreamingGS::loadAndTrain() {
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

    training_visualizer_->visualizeCurrentSplats(full_splat_batch, 0);

    try {
        // Check if spatial partitioning is enabled
        if (!config_.use_spatial_partitioning) {
            // Train all splats together without spatial partitioning
            std::cout << "\n=== Training all splats together (no spatial partitioning) ==="
                      << std::endl;
            std::cout << "Training with " << full_splat_batch.splats.size() << " splats and "
                      << all_keyframes.size() << " keyframes" << std::endl;

            // Setup training for all splats
            batch_trainer_->setupTraining(full_splat_batch);

            // Train with all keyframes for N epochs
            training::TrainingResults results;
            const int num_epochs = 200;

            std::cout << "Training for " << num_epochs << " epochs with " << all_keyframes.size()
                      << " keyframes" << std::endl;

            std::random_device rd;
            std::mt19937 rng(rd());

            // Training loop: run for num_epochs epochs
            for (int epoch = 0; epoch < num_epochs; ++epoch) {
                // Create a shuffled copy of keyframe indices for this epoch
                std::vector<size_t> keyframe_indices(all_keyframes.size());
                std::iota(keyframe_indices.begin(), keyframe_indices.end(), 0);
                std::shuffle(keyframe_indices.begin(), keyframe_indices.end(), rng);

                // Iterate through all keyframes in random order
                for (size_t kf_idx = 0; kf_idx < all_keyframes.size(); ++kf_idx) {
                    auto& kf = all_keyframes[keyframe_indices[kf_idx]];
                    int iteration = epoch * all_keyframes.size() + kf_idx;

                    batch_trainer_->trainKeyframe(kf, results, iteration, training_visualizer_);

                    if (iteration % 10 == 0) {
                        std::cout << "Epoch " << epoch << "/" << num_epochs << " - Keyframe "
                                  << (kf_idx + 1) << "/" << all_keyframes.size() << std::endl;
                    }

                    // Visualize results periodically
                    if (iteration % 5 == 0) {
                        auto rendered_image = utils::tensorToMat(results.rendered_image[0], false);
                        std::string entity_path = "/camera/unified";

                        batch_trainer_->copySplatsToBatch(full_splat_batch.splats);
                        training_visualizer_->visualizeCurrentSplats(full_splat_batch, iteration);
                        training_visualizer_->visualizeKeyframe(kf->pose, kf->getCameraInfo(),
                                                                entity_path);
                        training_visualizer_->logImage(entity_path, rendered_image, iteration);
                        auto rendering_tensors = batch_trainer_->cloneCurrentTensors();
                        foxglove_renderer_->updateRenderingTensors(rendering_tensors);
                        if (foxglove_renderer_->testRenderFromFixedPose()) {
                            LOG(INFO) << "Rendering test successful!";
                            // render_count++;
                        } else {
                            LOG(ERROR) << "Rendering test failed";
                        }

                        // Update Foxglove renderer with cloned training tensors
                        // LOG(INFO) << "Checking Foxglove renderer: ptr="
                        //           << (foxglove_renderer_ != nullptr) << " running="
                        //           << (foxglove_renderer_ ? foxglove_renderer_->isRunning() :
                        //           false);
                        // if (foxglove_renderer_ && foxglove_renderer_->isRunning()) {
                        //     LOG(INFO) << "Cloning training tensors for rendering";
                        //     auto rendering_tensors = batch_trainer_->cloneCurrentTensors();
                        //     foxglove_renderer_->updateRenderingTensors(rendering_tensors);
                        //
                        //     // Test render from fixed pose periodically so Foxglove clients can
                        //     see
                        //     // it
                        //     static int render_count = 0;
                        //     if (render_count <
                        //         5) {  // Render first 5 times to ensure clients receive it
                        //         LOG(INFO)
                        //             << "Testing rendering pipeline with fixed camera pose
                        //             (attempt "
                        //             << (render_count + 1) << ")";
                        //         if (foxglove_renderer_->testRenderFromFixedPose()) {
                        //             LOG(INFO) << "Rendering test successful!";
                        //             render_count++;
                        //         } else {
                        //             LOG(ERROR) << "Rendering test failed";
                        //         }
                        //     }
                        // } else {
                        //     LOG(WARNING) << "Foxglove renderer not available or not running";
                        // }
                    }
                }
            }

            // Copy trained splats back
            batch_trainer_->copySplatsToBatch(full_splat_batch.splats);

            std::cout << "\n=== Finished training all splats ===" << std::endl;

        } else {
            auto regions =
                partitionKeyframesIntoRadialSectors(all_keyframes, valid_keypoints, 16, 22.5f);

            // for (size_t i = 0; i < regions.size(); i++) {
            //     std::string name = "/bbox_" + std::to_string(i);
            //     training_visualizer_->visualizeBBox(regions[i].bbox_3d, name);
            // }

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
                auto region_splat_batch =
                    filterSplatsByBoundingBox(full_splat_batch, region.bbox_3d);

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
                    std::cout << "No valid keyframes in region " << (region_idx + 1)
                              << ", skipping..." << std::endl;
                    continue;
                }

                std::cout << "Training for " << num_epochs << " epochs with "
                          << region_keyframes.size() << " keyframes" << std::endl;

                std::random_device rd;
                std::mt19937 rng(rd());

                // Training loop: run for num_epochs epochs
                for (int epoch = 0; epoch < num_epochs; ++epoch) {
                    // Create a shuffled copy of keyframe indices for this epoch
                    std::vector<size_t> keyframe_indices(region_keyframes.size());
                    std::iota(keyframe_indices.begin(), keyframe_indices.end(), 0);
                    std::shuffle(keyframe_indices.begin(), keyframe_indices.end(), rng);

                    // Iterate through all keyframes in random order
                    for (size_t kf_idx = 0; kf_idx < region_keyframes.size(); ++kf_idx) {
                        auto& kf = region_keyframes[keyframe_indices[kf_idx]];
                        int iteration = epoch * region_keyframes.size() + kf_idx;

                        batch_trainer_->trainKeyframe(kf, results, iteration, training_visualizer_);

                        if (iteration % 10 == 0) {
                            std::cout << "Region " << (region_idx + 1) << " - Epoch " << epoch
                                      << "/" << num_epochs << " - Keyframe " << (kf_idx + 1) << "/"
                                      << region_keyframes.size() << std::endl;
                        }

                        // Visualize results periodically
                        if (iteration % 5 == 0) {
                            auto rendered_image =
                                utils::tensorToMat(results.rendered_image[0], false);
                            std::string entity_path =
                                "/camera/region_" + std::to_string(region_idx);

                            batch_trainer_->copySplatsToBatch(region_splat_batch.splats);
                            training_visualizer_->visualizeCurrentSplats(region_splat_batch,
                                                                         region_idx);
                            training_visualizer_->visualizeKeyframe(kf->pose, kf->getCameraInfo(),
                                                                    entity_path);
                            training_visualizer_->logImage(entity_path, rendered_image, region_idx);
                        }
                    }
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

                std::cout << "Finished training region " << (region_idx + 1) << std::endl;
            }

            std::cout << "\n=== Finished training all regions ===" << std::endl;

            // Final visualization with all trained splats
            training_visualizer_->visualizeCurrentSplats(full_splat_batch, 0);
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception in loadAndTrain: " << e.what() << std::endl;
        return false;
    }

    return true;
}

// Helper function to split keyframes into batches
std::vector<std::vector<uint64_t>> batchKeyframes(
    const std::vector<core::types::KeyFrame::Ptr>& keyframes, size_t batch_size) {
    std::vector<std::vector<uint64_t>> batches;
    std::vector<uint64_t> current_batch;

    for (const auto& kf : keyframes) {
        current_batch.push_back(kf->id);

        if (current_batch.size() >= batch_size) {
            batches.push_back(current_batch);
            current_batch.clear();
        }
    }

    // Add remaining keyframes as the last batch
    if (!current_batch.empty()) {
        batches.push_back(current_batch);
    }

    return batches;
}

// Helper function to group keypoints by keyframe batches
// Note: A keypoint can appear in multiple batches if it's visible from keyframes in different
// batches
std::vector<std::vector<core::types::Keypoint>> groupKeypointsByKeyframeBatches(
    const std::vector<core::types::Keypoint>& all_keypoints,
    const std::vector<std::vector<uint64_t>>& keyframe_batches) {
    std::vector<std::vector<core::types::Keypoint>> keypoint_batches;

    for (const auto& keyframe_batch : keyframe_batches) {
        // Create a set for fast lookup of keyframes in this batch
        std::unordered_set<uint64_t> keyframe_set(keyframe_batch.begin(), keyframe_batch.end());

        std::vector<core::types::Keypoint> batch_keypoints;

        // For each keypoint, check if it's visible from any keyframe in this batch
        for (const auto& keypoint : all_keypoints) {
            bool visible_in_batch = false;

            for (const auto& location : keypoint.locations) {
                if (keyframe_set.find(location.keyframe_id) != keyframe_set.end()) {
                    visible_in_batch = true;
                    break;
                }
            }

            if (visible_in_batch) {
                batch_keypoints.push_back(keypoint);
            }
        }

        keypoint_batches.push_back(batch_keypoints);
    }

    return keypoint_batches;
}

// Helper function to compute bounding box from keypoints
utils::BoundingBox computeBoundingBoxFromKeypoints(
    const std::vector<core::types::Keypoint>& keypoints) {
    if (keypoints.empty()) {
        return utils::BoundingBox{Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                                  Eigen::Vector3f(0.0f, 0.0f, 0.0f)};
    }

    Eigen::Vector3f min_point = keypoints[0].position.cast<float>();
    Eigen::Vector3f max_point = keypoints[0].position.cast<float>();

    for (const auto& keypoint : keypoints) {
        Eigen::Vector3f pos = keypoint.position.cast<float>();

        min_point.x() = std::min(min_point.x(), pos.x());
        min_point.y() = std::min(min_point.y(), pos.y());
        min_point.z() = std::min(min_point.z(), pos.z());

        max_point.x() = std::max(max_point.x(), pos.x());
        max_point.y() = std::max(max_point.y(), pos.y());
        max_point.z() = std::max(max_point.z(), pos.z());
    }

    // Add some padding to the bounding box
    const float padding = 1.0f;
    min_point -= Eigen::Vector3f(padding, padding, padding);
    max_point += Eigen::Vector3f(padding, padding, padding);

    return utils::BoundingBox{min_point, max_point};
}

bool StreamingGS::streamAndTrain() {
    std::cout << "Starting streamAndTrain with keyframe-based batching" << std::endl;

    if (!batch_trainer_) {
        std::cerr << "BatchTrainer not initialized" << std::endl;
        return false;
    }

    auto current_timestamp = getCurrentTimestamp();

    // Load all keyframes and keypoints
    auto all_keyframes = map_store_->getAllKeyFrames();
    auto all_keypoints = map_store_->getAllKeyPoints();

    std::cout << "Total keyframes: " << all_keyframes.size() << std::endl;
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

    // Filter keypoints by distance from center
    std::cout << "\n=== Filtering keypoints by distance from center ===" << std::endl;
    valid_keypoints = filterKeypointsByDistanceFromCenter(valid_keypoints, scene_center, 45.0f);

    // Build point cloud with filtered keypoints for density check
    utils::PointCloudUtils point_cloud_utils;
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

    // Filter keyframes to only include those with color images
    std::vector<core::types::KeyFrame::Ptr> keyframes_with_color;
    size_t keyframes_without_color = 0;
    for (const auto& kf : all_keyframes) {
        if (kf->hasColorImage()) {
            keyframes_with_color.push_back(kf);
        } else {
            keyframes_without_color++;
        }
    }

    std::cout << "\n=== Keyframe color image filtering ===" << std::endl;
    std::cout << "Total keyframes: " << all_keyframes.size() << std::endl;
    std::cout << "Keyframes with color images: " << keyframes_with_color.size() << std::endl;
    std::cout << "Keyframes without color images (skipped): " << keyframes_without_color
              << std::endl;

    if (keyframes_with_color.empty()) {
        std::cerr << "No keyframes with color images available for training" << std::endl;
        return false;
    }

    try {
        // Check if spatial partitioning (batching) is enabled
        if (!config_.use_spatial_partitioning) {
            // Train all splats together without batching
            std::cout << "\n=== Training all splats together (no batching) ===" << std::endl;

            // Initialize gaussian splats for all keypoints
            core::types::GaussianSplatBatch full_splat_batch;
            // if (!intializeSplatsFromKeypoints(valid_keypoints, map_store_, batch_id_,
            //                                   current_timestamp, full_splat_batch,
            //                                   next_splat_id_, point_cloud_utils)) {
            if (!intializeSplatsFromKeypoints(valid_keypoints, batch_id_, current_timestamp,
                                              full_splat_batch, next_splat_id_,
                                              point_cloud_utils)) {
                std::cout << "Unable to initialize gaussian splats" << std::endl;
                return false;
            }

            std::cout << "Initialized " << full_splat_batch.splats.size() << " splats" << std::endl;
            std::cout << "Training with " << keyframes_with_color.size() << " keyframes"
                      << std::endl;

            // Setup training for all splats
            batch_trainer_->setupTraining(full_splat_batch);

            // Visualize initial splats
            training_visualizer_->visualizeCurrentSplats(full_splat_batch, 0);

            // Train with all keyframes for N epochs
            const int num_epochs = 2000;
            training::TrainingResults results;

            std::cout << "Training for " << num_epochs << " epochs with "
                      << keyframes_with_color.size() << " keyframes" << std::endl;

            std::random_device rd;
            std::mt19937 rng(rd());

            for (int epoch = 0; epoch < num_epochs; ++epoch) {
                // Create a shuffled copy of keyframe indices for this epoch
                std::vector<size_t> keyframe_indices(keyframes_with_color.size());
                std::iota(keyframe_indices.begin(), keyframe_indices.end(), 0);
                std::shuffle(keyframe_indices.begin(), keyframe_indices.end(), rng);

                // Iterate through all keyframes in random order
                for (size_t kf_idx = 0; kf_idx < keyframes_with_color.size(); ++kf_idx) {
                    auto& kf = keyframes_with_color[keyframe_indices[kf_idx]];
                    int iteration = epoch * keyframes_with_color.size() + kf_idx;

                    batch_trainer_->trainKeyframe(kf, results, iteration, training_visualizer_);

                    if (iteration % 10 == 0) {
                        std::cout << "Epoch " << epoch << "/" << num_epochs << " - Keyframe "
                                  << (kf_idx + 1) << "/" << keyframes_with_color.size()
                                  << std::endl;
                    }

                    // Visualize results periodically
                    if (iteration % 5 == 0) {
                        auto rendered_image = utils::tensorToMat(results.rendered_image[0], false);
                        std::string entity_path = "/camera/unified";

                        batch_trainer_->copySplatsToBatch(full_splat_batch.splats);
                        training_visualizer_->visualizeCurrentSplats(full_splat_batch, iteration);
                        auto transform_result =
                            tf_tree_->getTransform(config_.base_link, config_.camera_frame);
                        auto camera_pose = kf->pose.getEigenIsometry() * transform_result.transform;
                        core::types::Pose pose;
                        pose.position = camera_pose.translation();
                        pose.orientation = camera_pose.rotation();
                        training_visualizer_->visualizeKeyframe(pose, kf->getCameraInfo(),
                                                                entity_path);
                        training_visualizer_->logImage(entity_path, rendered_image, iteration);

                        // // Update Foxglove renderer with cloned training tensors
                        LOG(INFO) << "Checking Foxglove renderer: ptr="
                                  << (foxglove_renderer_ != nullptr) << " running="
                                  << (foxglove_renderer_ ? foxglove_renderer_->isRunning() : false);
                        if (foxglove_renderer_ && foxglove_renderer_->isRunning()) {
                            auto timestamp_ns = static_cast<uint64_t>(
                                std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::system_clock::now().time_since_epoch())
                                    .count());

                            foxglove_renderer_->updateSplats(full_splat_batch.splats, timestamp_ns);
                            foxglove_renderer_->testRenderFromFixedPose();

                            // Sample and publish 10 keyframe poses evenly from all keyframes
                            const size_t num_poses_to_publish = 10;
                            if (keyframes_with_color.size() > 0 && iteration % 50 == 0) {
                                std::vector<core::types::Pose> sampled_poses;
                                size_t num_keyframes = keyframes_with_color.size();
                                size_t sample_count = std::min(num_poses_to_publish, num_keyframes);

                                for (size_t i = 0; i < sample_count; ++i) {
                                    // Sample evenly across the keyframe list
                                    size_t idx = (i * num_keyframes) / sample_count;
                                    auto& kf = keyframes_with_color[idx];

                                    // Transform from base_link to camera_frame
                                    auto transform_result = tf_tree_->getTransform(
                                        config_.base_link, config_.camera_frame);
                                    auto camera_pose =
                                        kf->pose.getEigenIsometry() * transform_result.transform;

                                    core::types::Pose pose;
                                    pose.position = camera_pose.translation();
                                    pose.orientation = camera_pose.rotation();
                                    pose.frame_id = "world";
                                    pose.timestamp = kf->pose.timestamp;
                                    sampled_poses.push_back(pose);
                                }

                                foxglove_renderer_->publishKeyframePoses(sampled_poses,
                                                                         timestamp_ns);
                                LOG(INFO) << "[StreamingGSProcessor] Published "
                                          << sampled_poses.size() << " keyframe poses to Foxglove";
                            }
                        }
                        // if (foxglove_renderer_ && foxglove_renderer_->isRunning()) {
                        //     LOG(INFO) << "Cloning training tensors for rendering";
                        //     auto rendering_tensors = batch_trainer_->cloneCurrentTensors();
                        //     foxglove_renderer_->updateRenderingTensors(rendering_tensors);
                        //
                        //     // Test render from fixed pose periodically so Foxglove clients can
                        //     see it static int render_count = 0; if (render_count < 5) {  //
                        //     Render first 5 times to ensure clients receive it
                        //         LOG(INFO) << "Testing rendering pipeline with fixed camera pose
                        //         (attempt " << (render_count + 1) << ")"; if
                        //         (foxglove_renderer_->testRenderFromFixedPose()) {
                        //             LOG(INFO) << "Rendering test successful!";
                        //             render_count++;
                        //         } else {
                        //             LOG(ERROR) << "Rendering test failed";
                        //         }
                        //     }
                        // } else {
                        //     LOG(WARNING) << "Foxglove renderer not available or not running";
                        // }
                    }

                    // Log splat state to CSV at first iteration and every 50 iterations
                    if (iteration == 0 || (iteration % 50 == 0 && iteration > 0)) {
                        batch_trainer_->copySplatsToBatch(full_splat_batch.splats);
                        std::string csv_filename =
                            "splats_iter_" + std::to_string(iteration) + ".csv";
                        LOG(INFO) << "Logging " << full_splat_batch.splats.size() << " splats to "
                                  << training_config_.debug_output_path << csv_filename;
                        utils::SplatLogger::writeSplatsToCSV(full_splat_batch.splats,
                                                             training_config_.debug_output_path,
                                                             csv_filename);
                    }
                }
            }

            std::cout << "\n=== Finished training all splats ===" << std::endl;

            // Log final splat state to CSV
            batch_trainer_->copySplatsToBatch(full_splat_batch.splats);
            std::string splat_log_path = training_config_.debug_output_path;
            std::string csv_filename = "splats_final.csv";
            LOG(INFO) << "Logging " << full_splat_batch.splats.size() << " splats to "
                      << splat_log_path << csv_filename;
            if (!utils::SplatLogger::writeSplatsToCSV(full_splat_batch.splats, splat_log_path,
                                                      csv_filename)) {
                LOG(ERROR) << "Failed to write splats to CSV";
            }

        } else {
            // Use batching (original behavior)
            // Split keyframes into batches (only those with color images)
            const size_t keyframes_per_batch = 10;  // Configurable batch size
            auto keyframe_batches = batchKeyframes(keyframes_with_color, keyframes_per_batch);

            std::cout << "\n=== Split keyframes into " << keyframe_batches.size()
                      << " batches ===" << std::endl;

            // Group keypoints by keyframe batches
            auto keypoint_batches =
                groupKeypointsByKeyframeBatches(valid_keypoints, keyframe_batches);

            std::cout << "Grouped keypoints into " << keypoint_batches.size() << " batches"
                      << std::endl;

            // Process each batch
            for (size_t batch_idx = 0; batch_idx < keyframe_batches.size(); ++batch_idx) {
                const auto& keyframe_batch = keyframe_batches[batch_idx];
                const auto& keypoint_batch = keypoint_batches[batch_idx];

                std::cout << "\n=== Processing Batch " << (batch_idx + 1) << "/"
                          << keyframe_batches.size() << " ===" << std::endl;
                std::cout << "Keyframes in batch: " << keyframe_batch.size() << std::endl;
                std::cout << "Keypoints in batch: " << keypoint_batch.size() << std::endl;

                if (keypoint_batch.empty()) {
                    std::cout << "No keypoints in batch " << (batch_idx + 1) << ", skipping..."
                              << std::endl;
                    continue;
                }

                // Initialize gaussian splats for this batch with color extraction from keyframes
                core::types::GaussianSplatBatch batch_splat_batch;
                if (!intializeSplatsFromKeypoints(keypoint_batch, map_store_, batch_id_,
                                                  current_timestamp, batch_splat_batch,
                                                  next_splat_id_, point_cloud_utils)) {
                    std::cout << "Unable to initialize gaussian splats for batch "
                              << (batch_idx + 1) << std::endl;
                    continue;
                }

                std::cout << "Initialized " << batch_splat_batch.splats.size()
                          << " splats for batch" << std::endl;

                // Compute bounding box for visualization
                auto batch_bbox = computeBoundingBoxFromKeypoints(keypoint_batch);
                std::cout << "Batch bounding box: min[" << batch_bbox.min.transpose() << "] max["
                          << batch_bbox.max.transpose() << "]" << std::endl;

                // Visualize bounding box
                training_visualizer_->visualizeBBox(batch_bbox,
                                                    "/batch_bbox_" + std::to_string(batch_idx));

                // Setup training for this batch
                batch_trainer_->setupTraining(batch_splat_batch);

                // Visualize batch splats
                training_visualizer_->visualizeCurrentSplats(batch_splat_batch, batch_idx);

                // Collect keyframe pointers for this batch (all should have color images)
                std::vector<core::types::KeyFrame::Ptr> batch_keyframes;
                for (const auto& kf_id : keyframe_batch) {
                    auto kf_it =
                        std::find_if(keyframes_with_color.begin(), keyframes_with_color.end(),
                                     [kf_id](const auto& kf) { return kf->id == kf_id; });
                    if (kf_it != keyframes_with_color.end()) {
                        batch_keyframes.push_back(*kf_it);
                    }
                }

                if (batch_keyframes.empty()) {
                    std::cout << "No valid keyframes in batch " << (batch_idx + 1)
                              << ", skipping..." << std::endl;
                    continue;
                }

                // Train this batch
                const int num_epochs =
                    (20000 + batch_keyframes.size() - 1) / batch_keyframes.size();
                training::TrainingResults results;

                std::cout << "Training batch for " << num_epochs << " epochs with "
                          << batch_keyframes.size() << " keyframes" << std::endl;

                std::random_device rd;
                std::mt19937 rng(rd());

                for (int epoch = 0; epoch < num_epochs; ++epoch) {
                    // Create a shuffled copy of keyframe indices for this epoch
                    std::vector<size_t> keyframe_indices(batch_keyframes.size());
                    std::iota(keyframe_indices.begin(), keyframe_indices.end(), 0);
                    std::shuffle(keyframe_indices.begin(), keyframe_indices.end(), rng);

                    // Iterate through all keyframes in random order
                    for (size_t kf_idx = 0; kf_idx < batch_keyframes.size(); ++kf_idx) {
                        auto& kf = batch_keyframes[keyframe_indices[kf_idx]];
                        int iteration = epoch * batch_keyframes.size() + kf_idx;

                        batch_trainer_->trainKeyframe(kf, results, iteration, training_visualizer_);

                        if (iteration % 10 == 0) {
                            std::cout << "Batch " << (batch_idx + 1) << " - Epoch " << epoch << "/"
                                      << num_epochs << " - Keyframe " << (kf_idx + 1) << "/"
                                      << batch_keyframes.size() << std::endl;
                        }

                        // Visualize results periodically
                        if (iteration % 5 == 0) {
                            auto rendered_image =
                                utils::tensorToMat(results.rendered_image[0], false);
                            std::string entity_path = "/camera/batch_" + std::to_string(batch_idx);

                            batch_trainer_->copySplatsToBatch(batch_splat_batch.splats);
                            training_visualizer_->visualizeCurrentSplats(batch_splat_batch,
                                                                         batch_idx);
                            auto transform_result =
                                tf_tree_->getTransform(config_.base_link, config_.camera_frame);
                            auto camera_pose =
                                kf->pose.getEigenIsometry() * transform_result.transform;
                            core::types::Pose pose;
                            pose.position = camera_pose.translation();
                            pose.orientation = camera_pose.rotation();
                            training_visualizer_->visualizeKeyframe(pose, kf->getCameraInfo(),
                                                                    entity_path);
                            training_visualizer_->logImage(entity_path, rendered_image, batch_idx);
                        }
                    }
                }

                std::cout << "Finished training batch " << (batch_idx + 1) << std::endl;

                // Increment batch ID for next batch
                batch_id_++;
            }

            std::cout << "\n=== Finished training all batches ===" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception in streamAndTrain: " << e.what() << std::endl;
        return false;
    }

    return true;
}

}  // namespace gaussian_splatting

#include "core/graph/factor_graph.hpp"
#include <chrono>
#include <iomanip>
#include "logging/logging.hpp"

namespace core {
namespace graph {

void FactorGraph::addKeyFrame(const KeyFramePtr& keyframe) {
    keyframes_[keyframe->id] = keyframe;

    // Add to GTSAM initial estimates
    initial_estimates_.insert(gtsam::Symbol('x', keyframe->id), toPose3(keyframe->pose));
}

bool FactorGraph::removeKeyFrame(uint64_t id) {
    auto it = keyframes_.find(id);
    if (it == keyframes_.end()) {
        return false;  // Keyframe not found
    }

    // Remove from keyframes map
    keyframes_.erase(it);

    // Remove from GTSAM initial estimates
    gtsam::Symbol symbol('x', id);
    if (initial_estimates_.exists(symbol)) {
        initial_estimates_.erase(symbol);
    }

    // Note: We don't remove factors here as they might reference this keyframe
    // The optimization will handle missing keyframes gracefully

    return true;
}

void FactorGraph::addFactor(const types::Factor& factor) {
    factors_.push_back(factor);

    switch (factor.type) {
        case proto::FactorType::PRIOR: {
            const auto& pose = std::get<0>(factor.measurement);
            graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
                gtsam::Symbol('x', factor.connected_nodes[0]), toPose3(pose),
                gtsam::noiseModel::Gaussian::Information(factor.information)));
            break;
        }
        case proto::FactorType::ODOMETRY:
        case proto::FactorType::LOOP_CLOSURE: {
            const auto& relative_pose = std::get<1>(factor.measurement);
            graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                gtsam::Symbol('x', factor.connected_nodes[0]),
                gtsam::Symbol('x', factor.connected_nodes[1]), toPose3(relative_pose),
                gtsam::noiseModel::Gaussian::Information(factor.information)));
            break;
        }
        case proto::FactorType::IMU_PREINTEGRATED: {
            if (factor.connected_nodes.size() != 2) {
                throw std::runtime_error("IMU factor requires exactly 2 connected nodes");
            }

            const auto& imu_measurement = std::get<2>(factor.measurement);

            // Create 6x6 noise model for BetweenFactor (3 for rotation + 3 for translation)
            // Extract relevant blocks from the 9x9 IMU covariance matrix
            Eigen::Matrix<double, 6, 6> pose_covariance;
            pose_covariance.setZero();

            // Position part (from preintegrated position covariance)
            pose_covariance.block<3, 3>(3, 3) = imu_measurement.covariance_matrix.block<3, 3>(0, 0);

            // Rotation part (from preintegrated rotation covariance)
            pose_covariance.block<3, 3>(0, 0) = imu_measurement.covariance_matrix.block<3, 3>(6, 6);

            // Ensure positive definiteness
            pose_covariance += Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;

            auto noise_model = gtsam::noiseModel::Gaussian::Covariance(pose_covariance);

            // Create relative pose from IMU preintegration
            types::Pose relative_pose;
            relative_pose.position = imu_measurement.preintegrated_position;
            relative_pose.orientation = imu_measurement.preintegrated_rotation;

            graph_.add(
                gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', factor.connected_nodes[0]),
                                                   gtsam::Symbol('x', factor.connected_nodes[1]),
                                                   toPose3(relative_pose), noise_model));
            break;
        }
        default:
            throw std::runtime_error("Unsupported factor type");
    }
}

bool FactorGraph::optimize() {
    try {
        gtsam::LevenbergMarquardtParams params;
        params.setVerbosity("ERROR");
        params.setMaxIterations(100);
        params.setRelativeErrorTol(1e-5);

        gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_estimates_, params);
        result_ = optimizer.optimize();

        // Update keyframe poses with optimized results
        for (auto& [id, keyframe] : keyframes_) {
            const gtsam::Symbol symbol('x', id);
            if (result_.exists(symbol)) {
                keyframe->pose = fromPose3(result_.at<gtsam::Pose3>(symbol));
            }
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Optimization failed: " << e.what() << std::endl;
        return false;
    }
}

bool FactorGraph::optimizeFromStorage(const core::storage::MapStore& store) {
    try {
        LOG(INFO) << "Building GTSAM graph on-demand from storage for optimization";

        // Create temporary GTSAM structures for this optimization
        gtsam::NonlinearFactorGraph temp_graph;
        gtsam::Values temp_initial_estimates;

        // Load all keyframes from storage and add to temporary estimates
        auto all_keyframes = store.getAllKeyFrames();
        LOG(INFO) << "Loading " << all_keyframes.size() << " keyframes from storage";

        if (all_keyframes.empty()) {
            LOG(WARNING) << "No keyframes found in storage";
            return false;
        }

        // Log all keyframe IDs being loaded
        std::vector<uint64_t> keyframe_ids;
        for (const auto& keyframe : all_keyframes) {
            temp_initial_estimates.insert(gtsam::Symbol('x', keyframe->id),
                                          toPose3(keyframe->pose));
            keyframe_ids.push_back(keyframe->id);
        }

        std::sort(keyframe_ids.begin(), keyframe_ids.end());
        LOG(INFO) << "Keyframe IDs loaded: [" << keyframe_ids.front() << " to "
                  << keyframe_ids.back() << "]";
        LOG(INFO) << "First 10 keyframe IDs: ";
        for (size_t i = 0; i < std::min(size_t(10), keyframe_ids.size()); ++i) {
            LOG(INFO) << "  Keyframe ID: " << keyframe_ids[i];
        }
        if (keyframe_ids.size() > 10) {
            LOG(INFO) << "  ... (and " << (keyframe_ids.size() - 10) << " more)";
        }

        // Load all factors from storage and add to temporary graph
        auto all_factors = store.getAllFactors();
        LOG(INFO) << "Loading " << all_factors.size() << " factors from storage";

        if (all_factors.empty()) {
            LOG(WARNING) << "No factors found in storage";
            return false;
        }

        size_t prior_factors = 0, odometry_factors = 0, loop_factors = 0, imu_factors = 0,
               skipped_factors = 0;

        for (const auto& factor : all_factors) {
            // Log detailed information about each factor
            std::string factor_type_name;
            switch (factor.type) {
                case proto::FactorType::PRIOR:
                    factor_type_name = "PRIOR";
                    break;
                case proto::FactorType::ODOMETRY:
                    factor_type_name = "ODOMETRY";
                    break;
                case proto::FactorType::LOOP_CLOSURE:
                    factor_type_name = "LOOP_CLOSURE";
                    break;
                case proto::FactorType::IMU_PREINTEGRATED:
                    factor_type_name = "IMU_PREINTEGRATED";
                    break;
                default:
                    factor_type_name = "UNKNOWN";
                    break;
            }

            LOG(INFO) << "Processing factor #" << factor.id << " type=" << factor_type_name
                      << " connecting nodes: [";
            for (size_t i = 0; i < factor.connected_nodes.size(); ++i) {
                LOG(INFO) << "  " << factor.connected_nodes[i];
                if (i < factor.connected_nodes.size() - 1) {
                    LOG(INFO) << " -> ";
                }
            }
            LOG(INFO) << "]";

            try {
                switch (factor.type) {
                    case proto::FactorType::PRIOR: {
                        const auto& pose = std::get<0>(factor.measurement);
                        temp_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                            gtsam::Symbol('x', factor.connected_nodes[0]), toPose3(pose),
                            gtsam::noiseModel::Gaussian::Information(factor.information)));
                        prior_factors++;
                        LOG(INFO) << "Added PRIOR factor for keyframe "
                                  << factor.connected_nodes[0];
                        break;
                    }
                    case proto::FactorType::ODOMETRY: {
                        const auto& relative_pose = std::get<1>(factor.measurement);
                        temp_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                            gtsam::Symbol('x', factor.connected_nodes[0]),
                            gtsam::Symbol('x', factor.connected_nodes[1]), toPose3(relative_pose),
                            gtsam::noiseModel::Gaussian::Information(factor.information)));
                        odometry_factors++;
                        LOG(INFO) << "Added ODOMETRY factor between keyframes "
                                  << factor.connected_nodes[0] << " -> "
                                  << factor.connected_nodes[1];
                        break;
                    }
                    case proto::FactorType::LOOP_CLOSURE: {
                        const auto& relative_pose = std::get<1>(factor.measurement);
                        temp_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                            gtsam::Symbol('x', factor.connected_nodes[0]),
                            gtsam::Symbol('x', factor.connected_nodes[1]), toPose3(relative_pose),
                            gtsam::noiseModel::Gaussian::Information(factor.information)));
                        loop_factors++;
                        LOG(INFO) << "Added LOOP_CLOSURE factor between keyframes "
                                  << factor.connected_nodes[0] << " -> "
                                  << factor.connected_nodes[1];
                        break;
                    }
                    case proto::FactorType::IMU_PREINTEGRATED: {
                        if (factor.connected_nodes.size() != 2) {
                            LOG(WARNING)
                                << "IMU factor requires exactly 2 connected nodes, skipping";
                            skipped_factors++;
                            continue;
                        }

                        const auto& imu_measurement = std::get<2>(factor.measurement);

                        // Create 6x6 noise model for BetweenFactor
                        Eigen::Matrix<double, 6, 6> pose_covariance;
                        pose_covariance.setZero();
                        pose_covariance.block<3, 3>(3, 3) =
                            imu_measurement.covariance_matrix.block<3, 3>(0, 0);
                        pose_covariance.block<3, 3>(0, 0) =
                            imu_measurement.covariance_matrix.block<3, 3>(6, 6);
                        pose_covariance += Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;

                        auto noise_model = gtsam::noiseModel::Gaussian::Covariance(pose_covariance);

                        types::Pose relative_pose;
                        relative_pose.position = imu_measurement.preintegrated_position;
                        relative_pose.orientation = imu_measurement.preintegrated_rotation;

                        temp_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                            gtsam::Symbol('x', factor.connected_nodes[0]),
                            gtsam::Symbol('x', factor.connected_nodes[1]), toPose3(relative_pose),
                            noise_model));
                        imu_factors++;
                        LOG(INFO) << "Added IMU factor between keyframes "
                                  << factor.connected_nodes[0] << " -> "
                                  << factor.connected_nodes[1];
                        break;
                    }
                    default:
                        LOG(WARNING) << "Unsupported factor type " << static_cast<int>(factor.type)
                                     << " in optimization, skipping factor #" << factor.id;
                        skipped_factors++;
                        break;
                }
            } catch (const std::exception& e) {
                LOG(WARNING) << "Failed to add factor #" << factor.id << " (" << factor_type_name
                             << ") to GTSAM graph: " << e.what();
                skipped_factors++;
            }
        }

        LOG(INFO) << "Factor breakdown: Prior=" << prior_factors
                  << ", Odometry=" << odometry_factors << ", Loop=" << loop_factors
                  << ", IMU=" << imu_factors << ", Skipped=" << skipped_factors;

        if (temp_graph.size() == 0) {
            LOG(ERROR) << "No valid factors added to GTSAM graph";
            return false;
        }

        // Check for consistency before optimization
        if (temp_initial_estimates.size() == 0) {
            LOG(ERROR) << "No initial estimates for GTSAM optimization";
            return false;
        }

        LOG(INFO) << "Starting GTSAM optimization with " << temp_graph.size() << " factors and "
                  << temp_initial_estimates.size() << " variables";

        // Configure GTSAM optimizer with enhanced parameters
        gtsam::LevenbergMarquardtParams params;
        params.setVerbosity("ERROR");  // Reduce GTSAM output noise
        params.setMaxIterations(100);
        params.setRelativeErrorTol(1e-5);
        params.setAbsoluteErrorTol(1e-5);
        params.setErrorTol(1e-5);
        params.setVerbosityLM("SILENT");  // Suppress LM iteration messages

        // Calculate initial error
        double initial_error = temp_graph.error(temp_initial_estimates);
        LOG(INFO) << "Initial graph error: " << std::scientific << std::setprecision(3)
                  << initial_error;

        // Optimize using temporary structures
        gtsam::LevenbergMarquardtOptimizer optimizer(temp_graph, temp_initial_estimates, params);

        auto optimization_start = std::chrono::high_resolution_clock::now();
        gtsam::Values optimized_result = optimizer.optimize();
        auto optimization_end = std::chrono::high_resolution_clock::now();

        auto gtsam_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            optimization_end - optimization_start);

        // Calculate final error and convergence metrics
        double final_error = temp_graph.error(optimized_result);
        double error_reduction = initial_error - final_error;
        double error_reduction_percent =
            (initial_error > 0) ? (100.0 * error_reduction / initial_error) : 0.0;

        LOG(INFO) << "GTSAM optimization completed in " << gtsam_duration.count() << "ms";
        LOG(INFO) << "Final graph error: " << std::scientific << std::setprecision(3)
                  << final_error;
        LOG(INFO) << "Error reduction: " << std::fixed << std::setprecision(1)
                  << error_reduction_percent << "%";
        LOG(INFO) << "Iterations: " << optimizer.iterations();

        // Validate optimization result
        if (optimized_result.size() != temp_initial_estimates.size()) {
            LOG(ERROR) << "Optimization result size mismatch: expected "
                       << temp_initial_estimates.size() << ", got " << optimized_result.size();
            return false;
        }

        // Check for NaN or infinite values in the result
        bool has_invalid_values = false;
        for (const auto& key_value : optimized_result) {
            gtsam::Pose3 pose = optimized_result.at<gtsam::Pose3>(key_value.key);
            if (!pose.translation().allFinite() || !pose.rotation().matrix().allFinite()) {
                has_invalid_values = true;
                LOG(ERROR) << "Invalid pose values detected in optimization result for variable "
                           << gtsam::Symbol(key_value.key);
                break;
            }
        }

        if (has_invalid_values) {
            LOG(ERROR) << "Optimization produced invalid results (NaN/Inf values)";
            return false;
        }

        // Store the result for retrieval
        result_ = optimized_result;

        // Temporary graph and estimates will be automatically destroyed when leaving scope
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Storage-based optimization failed: " << e.what();
        return false;
    }
}

gtsam::Pose3 FactorGraph::toPose3(const types::Pose& pose) const {
    return gtsam::Pose3(gtsam::Rot3(pose.orientation.matrix()), gtsam::Point3(pose.position));
}

types::Pose FactorGraph::fromPose3(const gtsam::Pose3& pose3) const {
    types::Pose pose;
    pose.position = pose3.translation();
    pose.orientation = Eigen::Quaterniond(pose3.rotation().matrix());
    return pose;
}

KeyFramePtr FactorGraph::getKeyFramePtr(uint64_t id) const {
    auto it = keyframes_.find(id);
    return it != keyframes_.end() ? it->second : nullptr;
}

types::KeyFrame& FactorGraph::getKeyFrame(uint64_t id) const {
    auto it = keyframes_.find(id);
    return *(it->second);
}

std::vector<KeyFramePtr> FactorGraph::getAllKeyFrames() const {
    std::vector<KeyFramePtr> keyframes;
    keyframes.reserve(keyframes_.size());
    for (const auto& [_, keyframe] : keyframes_) {
        keyframes.push_back(keyframe);
    }
    return keyframes;
}

std::vector<types::Factor> FactorGraph::getFactors() const {
    return factors_;
}

gtsam::Vector3 FactorGraph::toVector3(const Eigen::Vector3d& vec) const {
    return gtsam::Vector3(vec.x(), vec.y(), vec.z());
}

gtsam::Matrix3 FactorGraph::toMatrix3(const Eigen::Matrix3d& mat) const {
    gtsam::Matrix3 result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result(i, j) = mat(i, j);
        }
    }
    return result;
}

std::map<uint64_t, types::Pose> FactorGraph::getOptimizedPoses() const {
    std::map<uint64_t, types::Pose> optimized_poses;

    if (result_.empty()) {
        LOG(WARNING) << "No optimization results available";
        return optimized_poses;
    }

    // Extract all optimized poses from the result
    for (const auto& key_value : result_) {
        if (gtsam::Symbol(key_value.key).chr() == 'x') {  // Only pose variables
            uint64_t keyframe_id = gtsam::Symbol(key_value.key).index();
            gtsam::Pose3 optimized_pose3 = result_.at<gtsam::Pose3>(key_value.key);
            optimized_poses[keyframe_id] = fromPose3(optimized_pose3);
        }
    }

    LOG(INFO) << "Extracted " << optimized_poses.size() << " optimized poses";
    return optimized_poses;
}

void FactorGraph::addLandmark(uint32_t landmark_id, const Eigen::Vector3d& position) {
    initial_estimates_.insert(gtsam::Symbol('l', landmark_id), toPoint3(position));
}

void FactorGraph::addProjectionFactor(uint64_t keyframe_id, uint32_t landmark_id,
                                      const Eigen::Vector2d& measurement,
                                      const gtsam::SharedNoiseModel& noise_model,
                                      const gtsam::Cal3_S2& camera_calibration,
                                      const gtsam::Pose3& body_to_camera_transform) {
    gtsam::Point2 measured_point(measurement.x(), measurement.y());

    // Create shared_ptr for camera calibration as required by GTSAM
    auto camera_cal_ptr = boost::make_shared<gtsam::Cal3_S2>(camera_calibration);

    if (body_to_camera_transform.equals(gtsam::Pose3())) {
        // Direct camera frame projection (no body-to-camera transform)
        graph_.add(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(
            measured_point, noise_model, gtsam::Symbol('x', keyframe_id),
            gtsam::Symbol('l', landmark_id), camera_cal_ptr));
    } else {
        // With body-to-camera transform
        graph_.add(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(
            measured_point, noise_model, gtsam::Symbol('x', keyframe_id),
            gtsam::Symbol('l', landmark_id), camera_cal_ptr, body_to_camera_transform));
    }
}

bool FactorGraph::optimizeFromStorageWithLandmarks(
    const core::storage::MapStore& store,
    const std::map<uint32_t, core::types::Keypoint>& map_keypoints) {
    try {
        LOG(INFO) << "Building GTSAM graph with landmarks for full bundle adjustment";

        // Create temporary GTSAM structures for this optimization
        gtsam::NonlinearFactorGraph temp_graph;
        gtsam::Values temp_initial_estimates;

        // Load all keyframes from storage and add to temporary estimates
        auto all_keyframes = store.getAllKeyFrames();
        LOG(INFO) << "Loading " << all_keyframes.size() << " keyframes from storage";

        if (all_keyframes.empty()) {
            LOG(WARNING) << "No keyframes found in storage";
            return false;
        }

        // Add keyframe poses to initial estimates
        std::vector<uint64_t> keyframe_ids;
        for (const auto& keyframe : all_keyframes) {
            temp_initial_estimates.insert(gtsam::Symbol('x', keyframe->id),
                                          toPose3(keyframe->pose));
            keyframe_ids.push_back(keyframe->id);
        }

        std::sort(keyframe_ids.begin(), keyframe_ids.end());
        LOG(INFO) << "Keyframe IDs loaded: [" << keyframe_ids.front() << " to "
                  << keyframe_ids.back() << "]";

        // Add landmark points to initial estimates
        LOG(INFO) << "Adding " << map_keypoints.size() << " landmarks to optimization";
        for (const auto& [landmark_id, keypoint] : map_keypoints) {
            temp_initial_estimates.insert(gtsam::Symbol('l', landmark_id),
                                          toPoint3(keypoint.position));
        }

        // Load all factors from storage and add to temporary graph
        auto all_factors = store.getAllFactors();
        LOG(INFO) << "Loading " << all_factors.size() << " pose factors from storage";

        size_t prior_factors = 0, odometry_factors = 0, loop_factors = 0, imu_factors = 0,
               skipped_factors = 0;

        // Add pose factors (prior, odometry, loop closure, IMU)
        for (const auto& factor : all_factors) {
            try {
                switch (factor.type) {
                    case proto::FactorType::PRIOR: {
                        const auto& pose = std::get<0>(factor.measurement);
                        temp_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                            gtsam::Symbol('x', factor.connected_nodes[0]), toPose3(pose),
                            gtsam::noiseModel::Gaussian::Information(factor.information)));
                        prior_factors++;
                        break;
                    }
                    case proto::FactorType::ODOMETRY:
                    case proto::FactorType::LOOP_CLOSURE: {
                        const auto& relative_pose = std::get<1>(factor.measurement);
                        temp_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                            gtsam::Symbol('x', factor.connected_nodes[0]),
                            gtsam::Symbol('x', factor.connected_nodes[1]), toPose3(relative_pose),
                            gtsam::noiseModel::Gaussian::Information(factor.information)));
                        if (factor.type == proto::FactorType::ODOMETRY) {
                            odometry_factors++;
                        } else {
                            loop_factors++;
                        }
                        break;
                    }
                    case proto::FactorType::IMU_PREINTEGRATED: {
                        if (factor.connected_nodes.size() != 2) {
                            skipped_factors++;
                            continue;
                        }

                        const auto& imu_measurement = std::get<2>(factor.measurement);
                        Eigen::Matrix<double, 6, 6> pose_covariance;
                        pose_covariance.setZero();
                        pose_covariance.block<3, 3>(3, 3) =
                            imu_measurement.covariance_matrix.block<3, 3>(0, 0);
                        pose_covariance.block<3, 3>(0, 0) =
                            imu_measurement.covariance_matrix.block<3, 3>(6, 6);
                        pose_covariance += Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;

                        auto noise_model = gtsam::noiseModel::Gaussian::Covariance(pose_covariance);

                        types::Pose relative_pose;
                        relative_pose.position = imu_measurement.preintegrated_position;
                        relative_pose.orientation = imu_measurement.preintegrated_rotation;

                        temp_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                            gtsam::Symbol('x', factor.connected_nodes[0]),
                            gtsam::Symbol('x', factor.connected_nodes[1]), toPose3(relative_pose),
                            noise_model));
                        imu_factors++;
                        break;
                    }
                    default:
                        skipped_factors++;
                        break;
                }
            } catch (const std::exception& e) {
                LOG(WARNING) << "Failed to add factor #" << factor.id << ": " << e.what();
                skipped_factors++;
            }
        }

        // Add projection factors for visual observations
        size_t projection_factors = 0;
        size_t skipped_projections = 0;

        // Create pixel noise model (typically 1-2 pixels standard deviation)
        auto pixel_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);  // 1 pixel std dev

        for (const auto& [landmark_id, keypoint] : map_keypoints) {
            for (const auto& observation : keypoint.locations) {
                try {
                    // Find the keyframe for this observation
                    auto keyframe_it = std::find_if(all_keyframes.begin(), all_keyframes.end(),
                                                    [&observation](const auto& kf) {
                                                        return kf->id == observation.keyframe_id;
                                                    });

                    if (keyframe_it == all_keyframes.end()) {
                        skipped_projections++;
                        continue;
                    }

                    auto keyframe = *keyframe_it;
                    if (!keyframe->camera_info.has_value()) {
                        skipped_projections++;
                        continue;
                    }

                    // Create camera calibration from keyframe camera info
                    gtsam::Cal3_S2 camera_cal =
                        createCameraCalibration(keyframe->camera_info.value());

                    // Create shared_ptr for camera calibration as required by GTSAM
                    auto camera_cal_ptr = boost::make_shared<gtsam::Cal3_S2>(camera_cal);

                    // Create projection factor
                    gtsam::Point2 measured_point(observation.x, observation.y);
                    temp_graph.add(
                        gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(
                            measured_point, pixel_noise,
                            gtsam::Symbol('x', observation.keyframe_id),
                            gtsam::Symbol('l', landmark_id), camera_cal_ptr));

                    projection_factors++;

                } catch (const std::exception& e) {
                    LOG(WARNING) << "Failed to add projection factor for landmark " << landmark_id
                                 << " in keyframe " << observation.keyframe_id << ": " << e.what();
                    skipped_projections++;
                }
            }
        }

        LOG(INFO) << "Factor breakdown: Prior=" << prior_factors
                  << ", Odometry=" << odometry_factors << ", Loop=" << loop_factors
                  << ", IMU=" << imu_factors << ", Projection=" << projection_factors
                  << ", Skipped=" << (skipped_factors + skipped_projections);

        if (temp_graph.size() == 0) {
            LOG(ERROR) << "No valid factors added to GTSAM graph";
            return false;
        }

        if (temp_initial_estimates.size() == 0) {
            LOG(ERROR) << "No initial estimates for GTSAM optimization";
            return false;
        }

        LOG(INFO) << "Starting GTSAM bundle adjustment with " << temp_graph.size()
                  << " factors and " << temp_initial_estimates.size() << " variables ("
                  << all_keyframes.size() << " poses + " << map_keypoints.size() << " landmarks)";

        // Configure GTSAM optimizer with enhanced parameters for bundle adjustment
        gtsam::LevenbergMarquardtParams params;
        params.setVerbosity("ERROR");
        params.setMaxIterations(100);
        params.setRelativeErrorTol(1e-5);
        params.setAbsoluteErrorTol(1e-5);
        params.setErrorTol(1e-5);
        params.setVerbosityLM("SILENT");

        // Calculate initial error
        double initial_error = temp_graph.error(temp_initial_estimates);
        LOG(INFO) << "Initial bundle adjustment error: " << std::scientific << std::setprecision(3)
                  << initial_error;

        // Optimize using temporary structures
        gtsam::LevenbergMarquardtOptimizer optimizer(temp_graph, temp_initial_estimates, params);

        auto optimization_start = std::chrono::high_resolution_clock::now();
        gtsam::Values optimized_result = optimizer.optimize();
        auto optimization_end = std::chrono::high_resolution_clock::now();

        auto gtsam_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            optimization_end - optimization_start);

        // Calculate final error and convergence metrics
        double final_error = temp_graph.error(optimized_result);
        double error_reduction = initial_error - final_error;
        double error_reduction_percent =
            (initial_error > 0) ? (100.0 * error_reduction / initial_error) : 0.0;

        LOG(INFO) << "GTSAM bundle adjustment completed in " << gtsam_duration.count() << "ms";
        LOG(INFO) << "Final bundle adjustment error: " << std::scientific << std::setprecision(3)
                  << final_error;
        LOG(INFO) << "Error reduction: " << std::fixed << std::setprecision(1)
                  << error_reduction_percent << "%";
        LOG(INFO) << "Iterations: " << optimizer.iterations();

        // Validate optimization result
        if (optimized_result.size() != temp_initial_estimates.size()) {
            LOG(ERROR) << "Optimization result size mismatch: expected "
                       << temp_initial_estimates.size() << ", got " << optimized_result.size();
            return false;
        }

        // Check for NaN or infinite values in the result
        bool has_invalid_values = false;
        for (const auto& key_value : optimized_result) {
            gtsam::Symbol symbol(key_value.key);
            if (symbol.chr() == 'x') {
                gtsam::Pose3 pose = optimized_result.at<gtsam::Pose3>(key_value.key);
                if (!pose.translation().allFinite() || !pose.rotation().matrix().allFinite()) {
                    has_invalid_values = true;
                    LOG(ERROR) << "Invalid pose values detected for keyframe " << symbol.index();
                    break;
                }
            } else if (symbol.chr() == 'l') {
                gtsam::Point3 point = optimized_result.at<gtsam::Point3>(key_value.key);
                if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
                    !std::isfinite(point.z())) {
                    has_invalid_values = true;
                    LOG(ERROR) << "Invalid landmark values detected for landmark "
                               << symbol.index();
                    break;
                }
            }
        }

        if (has_invalid_values) {
            LOG(ERROR) << "Bundle adjustment produced invalid results (NaN/Inf values)";
            return false;
        }

        // Store the result for retrieval
        result_ = optimized_result;

        LOG(INFO) << "Bundle adjustment optimization completed successfully";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Bundle adjustment optimization failed: " << e.what();
        return false;
    }
}

std::map<uint32_t, Eigen::Vector3d> FactorGraph::getOptimizedLandmarks() const {
    std::map<uint32_t, Eigen::Vector3d> optimized_landmarks;

    if (result_.empty()) {
        LOG(WARNING) << "No optimization results available for landmarks";
        return optimized_landmarks;
    }

    // Extract all optimized landmarks from the result
    for (const auto& key_value : result_) {
        if (gtsam::Symbol(key_value.key).chr() == 'l') {  // Only landmark variables
            uint32_t landmark_id = gtsam::Symbol(key_value.key).index();
            gtsam::Point3 optimized_point3 = result_.at<gtsam::Point3>(key_value.key);
            optimized_landmarks[landmark_id] = fromPoint3(optimized_point3);
        }
    }

    LOG(INFO) << "Extracted " << optimized_landmarks.size() << " optimized landmarks";
    return optimized_landmarks;
}

gtsam::Point3 FactorGraph::toPoint3(const Eigen::Vector3d& point) const {
    return gtsam::Point3(point.x(), point.y(), point.z());
}

Eigen::Vector3d FactorGraph::fromPoint3(const gtsam::Point3& point3) const {
    return Eigen::Vector3d(point3.x(), point3.y(), point3.z());
}

gtsam::Cal3_S2 FactorGraph::createCameraCalibration(
    const core::types::CameraInfo& camera_info) const {
    // Extract camera intrinsics from CameraInfo
    if (camera_info.k.size() != 9) {
        throw std::runtime_error("Camera info K matrix must have 9 elements");
    }

    // K matrix is stored in row-major order: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    double fx = camera_info.k[0];
    double fy = camera_info.k[4];
    double cx = camera_info.k[2];
    double cy = camera_info.k[5];
    double s = camera_info.k[1];  // skew parameter (usually 0)

    return gtsam::Cal3_S2(fx, fy, s, cx, cy);
}

}  // namespace graph
}  // namespace core

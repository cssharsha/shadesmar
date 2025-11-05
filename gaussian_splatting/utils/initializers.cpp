#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <random>

#include <logging/logging.hpp>
#include <memory>
#include <string>
#include "core/storage/map_store.hpp"

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>
#include "gaussian_splatting/utils/point_cloud_utils.hpp"
#include "image_utils.hpp"
#include "initializers.hpp"

namespace gaussian_splatting {

GaussianTensors convertSplatBatchToTensors(const core::types::GaussianSplatBatch& splat_batch,
                                           const torch::Device& device) {
    GaussianTensors tensors;
    tensors.fromSplats(splat_batch.splats);
    tensors.to(device);
    return tensors;
}

bool intializeSplatsFromKeypoints(std::shared_ptr<core::storage::MapStore>& map_store,
                                  const uint64_t& current_batch_id, double& current_timestamp,
                                  core::types::GaussianSplatBatch& splat_batch,
                                  std::atomic<uint64_t>& next_splat_id,
                                  std::vector<uint64_t>& keyframe_ids) {
    std::cout << "Initialize splats from keypoints" << std::endl;
    // Get all keypoints from map store (non-blocking read)
    auto all_keypoints = map_store->getAllKeyPoints();
    std::cout << "No keypoints: " << all_keypoints.size() << std::endl;
    if (!all_keypoints.size()) {
        return false;
    }

    // Create splat batch from keypoints
    splat_batch.batch_id = current_batch_id;
    splat_batch.timestamp = current_timestamp;

    utils::PointCloudUtils point_cloud_utils;
    // Filter keypoints that are visible from the given keyframes
    std::set<uint64_t> keyframe_set;
    for (const auto& keypoint : all_keypoints) {
        // Skip keypoints that need triangulation
        if (keypoint.needs_triangulation) {
            throw std::runtime_error("Keypoint " + std::to_string(keypoint.id()) +
                                     " needs triangulation");
        }
        point_cloud_utils.addPoint(keypoint.position.x(), keypoint.position.y(),
                                   keypoint.position.z());

        for (const auto& location : keypoint.locations) {
            keyframe_set.insert(location.keyframe_id);
        }
    }
    std::cout << "Keyframes: " << keyframe_set.size() << std::endl;

    keyframe_ids = std::vector<uint64_t>(keyframe_set.begin(), keyframe_set.end());
    return intializeSplatsFromKeypoints(keyframe_ids, map_store, current_batch_id,
                                        current_timestamp, splat_batch, next_splat_id,
                                        point_cloud_utils);
}

bool intializeSplatsFromKeypoints(const std::vector<uint64_t>& keyframe_ids,
                                  std::shared_ptr<core::storage::MapStore>& map_store,
                                  const uint64_t& current_batch_id, double& current_timestamp,
                                  core::types::GaussianSplatBatch& splat_batch,
                                  std::atomic<uint64_t>& next_splat_id,
                                  utils::PointCloudUtils& point_cloud_utils) {
    std::cout << "Initialize splats from keypoints" << std::endl;
    // Get all keypoints from map store (non-blocking read)
    auto all_keypoints = map_store->getAllKeyPoints();
    std::cout << "No keypoints: " << all_keypoints.size() << std::endl;
    if (!all_keypoints.size()) {
        return false;
    }

    // Create splat batch from keypoints
    splat_batch.batch_id = current_batch_id;
    splat_batch.timestamp = current_timestamp;

    // Store source keyframe IDs
    for (const auto& kf_id : keyframe_ids) {
        splat_batch.source_keyframe_ids.insert(kf_id);
    }

    // Filter keypoints that are visible from the given keyframes
    // Progress bar for keypoints loaded
    size_t keypoints_loaded = 0;
    size_t keypoints_total = all_keypoints.size();
    std::cout << "Loading " << keypoints_total << " keypoints" << std::endl;
    const int barWidth = 50;
    point_cloud_utils.setupKDTree();
    for (const auto& keypoint : all_keypoints) {
        // std::cout << "Loading keypoint: " << keypoint.id() << std::endl;
        // Skip keypoints that need triangulation
        if (keypoint.needs_triangulation) {
            continue;
        }

        core::types::GaussianSplat splat;
        splat.id = next_splat_id++;
        splat.position = keypoint.position;
        splat.source_keypoint_id = keypoint.id();

        // Initialize color by extracting from keyframe observations
        // Convert RGB color to SH DC component
        Eigen::Vector3f rgb_color = keypoint.color.cast<float>() / 255.0f;
        constexpr float C0 = 0.28209479177387814f;
        splat.sh_dc = (rgb_color.array() - 0.5f) / C0;
        // std::cout << "Color: " << rgb_color.transpose() << " -> SH DC: " <<
        // splat.sh_dc.transpose() << std::endl;

        splat.scale =
            point_cloud_utils.computeScaleFromKNN(keypoint.position.cast<float>()).cast<double>();
        // std::cout << "Scale: " << splat.scale.transpose() << std::endl;
        splat.covariance = computeKeypointCovarianceUsingScale(keypoint, splat.scale.cast<float>());

        // Initialize opacity and confidence
        // splat.opacity = computeInitialOpacity(keypoint);
        auto init_opacity = 0.5f;
        // splat.opacity = std::log(init_opacity / (1.0f - init_opacity));
        splat.opacity = init_opacity;
        splat.confidence = computeInitialConfidence(keypoint);
        splat.timestamp = current_timestamp;

        // Initialize higher order spherical harmonics to zero (degree 3: 15 coeffs × 3 RGB = 45)
        splat.sh_rest = Eigen::VectorXf::Zero(45);
        splat.sh_degree = 3;

        splat_batch.splats.push_back(splat);
        // std::cout << "Added splat: " << splat.id << " with position: " <<
        // splat.position.transpose()
        //           << std::endl;
        // std::cout << "Added splat: " << splat_batch.splats.back().id
        //           << " with position: " << splat_batch.splats.back().position.transpose()
        //           << std::endl;
        keypoints_loaded++;
        // Actual progress bar showing progress
        float progress = static_cast<float>(keypoints_loaded) / keypoints_total;
        int pos = static_cast<int>(barWidth * progress);
        std::cout << "\r [";
        for (int i = 0; i < barWidth; ++i) {
            if (i < pos) {
                std::cout << "=";
            } else if (i == pos) {
                std::cout << ">";
            } else {
                std::cout << " ";
            }
        }
        std::cout << "] " << keypoints_loaded << " / " << keypoints_total << " keypoints loaded"
                  << std::flush;
    }
    std::cout << std::endl;
    return true;
}

bool intializeSplatsFromKeypoints(const std::vector<core::types::Keypoint>& keypoints,
                                  const uint64_t& current_batch_id, double& current_timestamp,
                                  core::types::GaussianSplatBatch& splat_batch,
                                  std::atomic<uint64_t>& next_splat_id,
                                  utils::PointCloudUtils& point_cloud_utils) {
    std::cout << "Initialize splats from filtered keypoints" << std::endl;
    std::cout << "Number of keypoints: " << keypoints.size() << std::endl;
    if (keypoints.empty()) {
        return false;
    }

    // Create splat batch from keypoints
    splat_batch.batch_id = current_batch_id;
    splat_batch.timestamp = current_timestamp;

    // Progress bar for keypoints loaded
    size_t keypoints_loaded = 0;
    size_t keypoints_total = keypoints.size();
    std::cout << "Loading " << keypoints_total << " filtered keypoints" << std::endl;
    const int barWidth = 50;
    point_cloud_utils.setupKDTree();

    for (const auto& keypoint : keypoints) {
        core::types::GaussianSplat splat;
        splat.id = next_splat_id++;
        splat.position = keypoint.position;
        splat.source_keypoint_id = keypoint.id();

        // Initialize color by converting RGB to SH DC component
        Eigen::Vector3f rgb_color = keypoint.color.cast<float>() / 255.0f;
        constexpr float C0 = 0.28209479177387814f;
        splat.sh_dc = (rgb_color.array() - 0.5f) / C0;

        splat.scale =
            point_cloud_utils.computeScaleFromKNN(keypoint.position.cast<float>()).cast<double>();
        splat.covariance = computeKeypointCovarianceUsingScale(keypoint, splat.scale.cast<float>());

        // Initialize opacity and confidence
        auto init_opacity = 0.5f;
        splat.opacity = std::log(init_opacity / (1.0f - init_opacity));
        splat.confidence = computeInitialConfidence(keypoint);
        splat.timestamp = current_timestamp;

        // Initialize higher order spherical harmonics to zero (degree 3: 15 coeffs × 3 RGB = 45)
        splat.sh_rest = Eigen::VectorXf::Zero(45);
        splat.sh_degree = 3;

        splat_batch.splats.push_back(splat);

        keypoints_loaded++;
        // Actual progress bar showing progress
        float progress = static_cast<float>(keypoints_loaded) / keypoints_total;
        int pos = static_cast<int>(barWidth * progress);
        std::cout << "\r [";
        for (int i = 0; i < barWidth; ++i) {
            if (i < pos) {
                std::cout << "=";
            } else if (i == pos) {
                std::cout << ">";
            } else {
                std::cout << " ";
            }
        }
        std::cout << "] " << keypoints_loaded << " / " << keypoints_total << " keypoints loaded"
                  << std::flush;
    }
    std::cout << std::endl;
    return true;
}

bool intializeSplatsFromKeypoints(const std::vector<core::types::Keypoint>& keypoints,
                                  std::shared_ptr<core::storage::MapStore>& map_store,
                                  const uint64_t& current_batch_id, double& current_timestamp,
                                  core::types::GaussianSplatBatch& splat_batch,
                                  std::atomic<uint64_t>& next_splat_id,
                                  utils::PointCloudUtils& point_cloud_utils) {
    std::cout << "Initialize splats from filtered keypoints with color extraction" << std::endl;
    std::cout << "Number of keypoints: " << keypoints.size() << std::endl;
    if (keypoints.empty()) {
        return false;
    }

    // Create splat batch from keypoints
    splat_batch.batch_id = current_batch_id;
    splat_batch.timestamp = current_timestamp;

    // Progress bar for keypoints loaded
    size_t keypoints_loaded = 0;
    size_t keypoints_total = keypoints.size();
    std::cout << "Loading " << keypoints_total << " filtered keypoints with color extraction"
              << std::endl;
    const int barWidth = 50;
    point_cloud_utils.setupKDTree();

    for (const auto& keypoint : keypoints) {
        core::types::GaussianSplat splat;
        splat.id = next_splat_id++;
        splat.position = keypoint.position;
        splat.source_keypoint_id = keypoint.id();

        // Extract color from keyframe observations (reads actual images from map_store)
        // and convert to SH DC component
        Eigen::Vector3f rgb_color = extractColorFromKeyframes(keypoint, map_store);
        splat.setColor(rgb_color);

        splat.scale =
            point_cloud_utils.computeScaleFromKNN(keypoint.position.cast<float>()).cast<double>();
        splat.covariance = computeKeypointCovarianceUsingScale(keypoint, splat.scale.cast<float>());

        // Initialize opacity and confidence
        auto init_opacity = 0.5f;
        splat.opacity = std::log(init_opacity / (1.0f - init_opacity));
        splat.confidence = computeInitialConfidence(keypoint);
        splat.timestamp = current_timestamp;

        // Initialize higher order spherical harmonics to zero (degree 3: 15 coeffs × 3 RGB = 45)
        splat.sh_rest = Eigen::VectorXf::Zero(45);
        splat.sh_degree = 3;

        splat_batch.splats.push_back(splat);

        keypoints_loaded++;
        // Actual progress bar showing progress
        float progress = static_cast<float>(keypoints_loaded) / keypoints_total;
        int pos = static_cast<int>(barWidth * progress);
        std::cout << "\r [";
        for (int i = 0; i < barWidth; ++i) {
            if (i < pos) {
                std::cout << "=";
            } else if (i == pos) {
                std::cout << ">";
            } else {
                std::cout << " ";
            }
        }
        std::cout << "] " << keypoints_loaded << " / " << keypoints_total << " keypoints loaded"
                  << std::flush;
    }
    std::cout << std::endl;
    return true;
}

Eigen::Vector3f extractColorFromKeyframes(const core::types::Keypoint& keypoint,
                                          std::shared_ptr<core::storage::MapStore>& map_store) {
    std::vector<Eigen::Vector3f> colors;
    std::vector<float> weights;

    // Extract color from each keyframe observation
    for (const auto& location : keypoint.locations) {
        auto keyframe = map_store->getKeyFrame(location.keyframe_id);
        if (!keyframe)
            continue;

        if (!keyframe->hasColorImage())
            continue;

        // Extract pixel color at keypoint location
        const auto& color_image = keyframe->getColorImage();
        cv::Mat cv_image = color_image.toCvMat();

        // Convert pixel coordinates to image coordinates
        int x = static_cast<int>(location.x);
        int y = static_cast<int>(location.y);

        // Check bounds
        if (x >= 0 && x < cv_image.cols && y >= 0 && y < cv_image.rows) {
            cv::Vec3b pixel = cv_image.at<cv::Vec3b>(y, x);

            // Convert BGR to RGB and normalize
            Eigen::Vector3f color(pixel[2] / 255.0f, pixel[1] / 255.0f, pixel[0] / 255.0f);

            // Use uniform weighting since response is not available in Location struct
            float weight = 1.0f;

            colors.push_back(color);
            weights.push_back(weight);
        }
    }

    // Compute weighted average color
    if (!colors.empty()) {
        Eigen::Vector3f weighted_color = Eigen::Vector3f::Zero();
        float total_weight = 0.0f;

        for (size_t i = 0; i < colors.size(); ++i) {
            weighted_color += weights[i] * colors[i];
            total_weight += weights[i];
        }

        if (total_weight > 0) {
            return weighted_color / total_weight;
        }
    }

    return Eigen::Vector3f(0.7f, 0.7f, 0.7f);
}

Eigen::Matrix3d computeKeypointCovariance(const core::types::Keypoint& keypoint,
                                          std::shared_ptr<core::storage::MapStore>& map_store) {
    // Base covariance scaled by observation quality
    double base_variance = 0.01;

    // Scale based on number of observations (more observations = lower uncertainty)
    if (keypoint.locations.size() > 3) {
        base_variance = 0.005;
    } else if (keypoint.locations.size() > 1) {
        base_variance = 0.008;
    }

    // Compute viewing angle variance - points seen from more diverse angles are more certain
    std::vector<Eigen::Vector3d> viewing_directions;
    for (const auto& location : keypoint.locations) {
        auto keyframe = map_store->getKeyFrame(location.keyframe_id);
        if (keyframe) {
            Eigen::Vector3d view_dir = (keypoint.position - keyframe->pose.position).normalized();
            viewing_directions.push_back(view_dir);
        }
    }

    // Compute viewing angle spread
    if (viewing_directions.size() > 1) {
        double angle_variance = 0.0;
        for (size_t i = 0; i < viewing_directions.size(); ++i) {
            for (size_t j = i + 1; j < viewing_directions.size(); ++j) {
                double angle = std::acos(
                    std::clamp(viewing_directions[i].dot(viewing_directions[j]), -1.0, 1.0));
                angle_variance += angle * angle;
            }
        }
        angle_variance /= (viewing_directions.size() * (viewing_directions.size() - 1) / 2);

        // More diverse viewing angles reduce uncertainty
        double angle_factor = std::exp(-angle_variance * 2.0);
        base_variance *= (0.5 + 0.5 * angle_factor);
    }

    // Scale by distance from cameras (closer points are more certain)
    if (!viewing_directions.empty()) {
        double avg_distance = 0.0;
        int count = 0;
        for (const auto& location : keypoint.locations) {
            auto keyframe = map_store->getKeyFrame(location.keyframe_id);
            if (keyframe) {
                avg_distance += (keypoint.position - keyframe->pose.position).norm();
                count++;
            }
        }
        if (count > 0) {
            avg_distance /= count;
            // Scale variance with distance (farther points are less certain)
            base_variance *= std::max(0.5, std::min(2.0, avg_distance / 5.0));
        }
    }

    return Eigen::Matrix3d::Identity() * base_variance;
}

Eigen::Matrix3d computeKeypointCovarianceUsingScale(const core::types::Keypoint& keypoint,
                                                    const Eigen::Vector3f& scale) {
    // Convert scale to covariance matrix
    // Set an identity direction for now
    Eigen::Quaternionf direction = Eigen::Quaternionf::Identity();
    Eigen::Matrix3d covariance = scaleToCovariance(scale, direction);

    // Add minimum variance to ensure numerical stability
    double min_variance = 1e-6;
    for (int i = 0; i < 3; ++i) {
        if (covariance(i, i) < min_variance) {
            covariance(i, i) = min_variance;
        }
    }

    return covariance;
}

Eigen::Matrix3d scaleToCovariance(const Eigen::Vector3f& scale,
                                  const Eigen::Quaternionf& direction) {
    // Compute the diagonal matrix from squared scaling values
    auto diagonalMatrix = scale.array().square().matrix().asDiagonal();
    auto R = direction.toRotationMatrix();

    // Calculate the covariance matrix by applying the rotation and its inverse
    auto covariance = R * diagonalMatrix * R.inverse();

    return covariance.cast<double>();
}
// bool testScaleCovarianceRoundtrip() {
//     // Test various scale values to ensure 1:1 mapping
//     std::vector<Eigen::Vector3d> test_scales = {
//         Eigen::Vector3d(0.01, 0.01, 0.01),  // Isotropic small
//         Eigen::Vector3d(0.1, 0.05, 0.02),   // Anisotropic
//         Eigen::Vector3d(1.0, 0.5, 0.1),     // Large anisotropic
//         Eigen::Vector3d(0.05, 0.05, 0.05)   // Isotropic medium
//     };
//
//     bool all_tests_passed = true;
//     double tolerance = 1e-10;
//
//     for (const auto& original_scale : test_scales) {
//         // Generate random rotation matrix for testing
//         Eigen::Matrix3d test_rotation = Eigen::Matrix3d::Identity();
//
//         // Test with identity rotation
//         Eigen::Matrix3d covariance = scaleToCovariance(original_scale, test_rotation);
//         Eigen::Matrix3d recovered_rotation;
//         Eigen::Vector3d recovered_scale = covarianceToScale(covariance, recovered_rotation);
//
//         // Check if scales match
//         Eigen::Vector3d scale_diff = (original_scale - recovered_scale).cwiseAbs();
//         double max_scale_error = scale_diff.maxCoeff();
//
//         if (max_scale_error > tolerance) {
//             LOG(ERROR) << "Scale roundtrip test failed! Original: " << original_scale.transpose()
//                        << " Recovered: " << recovered_scale.transpose()
//                        << " Error: " << max_scale_error;
//             all_tests_passed = false;
//         } else {
//             LOG(INFO) << "Scale roundtrip test passed for scale: " << original_scale.transpose();
//         }
//     }
//
//     if (all_tests_passed) {
//         LOG(INFO) << "All scale-covariance roundtrip tests passed!";
//     }
//
//     return all_tests_passed;
// }

float computeInitialOpacity(const core::types::Keypoint& keypoint) {
    // Higher opacity for keypoints with more observations
    float opacity = 0.5f + 0.3f * std::min(1.0f, keypoint.locations.size() / 5.0f);
    return std::clamp(opacity, 0.1f, 0.9f);
}

float computeInitialConfidence(const core::types::Keypoint& keypoint) {
    // Confidence based on observation count and descriptor quality
    float base_confidence = 0.8f;
    if (keypoint.locations.size() > 2) {
        base_confidence = 0.95f;
    } else if (keypoint.locations.size() > 1) {
        base_confidence = 0.85f;
    }

    return std::clamp(base_confidence, 0.3f, 1.0f);
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> estimateSceneBoundsFromTrajectory(
    std::shared_ptr<core::storage::MapStore>& map_store) {
    Eigen::Vector3f scene_min{-100.f, -100.f, -100.f};
    Eigen::Vector3f scene_max{100.f, 100.f, 100.f};

    try {
        auto keyframes = map_store->getAllKeyFrames();
        if (!keyframes.empty()) {
            std::cout << "Estimating scene bounds from " << keyframes.size() << " keyframes"
                      << std::endl;

            Eigen::Vector3f first_pos = keyframes[0]->pose.position.cast<float>();
            scene_min = first_pos;
            scene_max = first_pos;

            // Expand bounds to include all keyframe positions
            for (const auto& kf : keyframes) {
                Eigen::Vector3f pos = kf->pose.position.cast<float>();
                scene_min = scene_min.cwiseMin(pos);
                scene_max = scene_max.cwiseMax(pos);
            }

            std::cout << "Estimated scene bounds from trajectory: min(" << scene_min.transpose()
                      << ") max(" << scene_max.transpose() << ")" << std::endl;
        } else {
            std::cout << "No keyframes available, using default scene bounds" << std::endl;
            scene_min = Eigen::Vector3f(-10.0f, -10.0f, -10.0f);
            scene_max = Eigen::Vector3f(10.0f, 10.0f, 10.0f);
        }

    } catch (const std::exception& e) {
        LOG(WARNING) << "Failed to estimate scene bounds from trajectory: " << e.what()
                     << ", using default bounds";
        scene_min = Eigen::Vector3f(-10.0f, -10.0f, -10.0f);
        scene_max = Eigen::Vector3f(10.0f, 10.0f, 10.0f);
    }

    return std::make_pair(scene_min, scene_max);
}

std::vector<core::types::GaussianSplat> generateRandomSplats(const Eigen::Vector3f& scene_min,
                                                             const Eigen::Vector3f& scene_max,
                                                             int count,
                                                             std::atomic<uint64_t>& next_splat_id,
                                                             double timestamp) {
    std::vector<core::types::GaussianSplat> splats;
    splats.reserve(count);

    std::cout << "Generating " << count << " random splats in bounds: min(" << scene_min.transpose()
              << ") max(" << scene_max.transpose() << ")" << std::endl;

    for (int i = 0; i < count; ++i) {
        core::types::GaussianSplat splat;

        // Generate unique ID
        splat.id = next_splat_id++;

        // Random position within scene bounds
        splat.position = generateRandomPosition(scene_min, scene_max).template cast<double>();

        // Random initial color - convert RGB to SH DC component
        Eigen::Vector3f rgb_color = generateRandomColor();
        splat.setColor(rgb_color);

        // Initial covariance matrix
        splat.covariance = generateInitialCovariance();

        // Initial opacity
        splat.opacity = generateInitialOpacity();

        // Initial confidence (medium for random initialization)
        splat.confidence = 0.5f;

        // Timestamp
        splat.timestamp = timestamp;

        // No source keypoint for random initialization
        splat.source_keypoint_id = 0;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(splat.covariance);
        if (solver.info() == Eigen::Success) {
            splat.rotation = Eigen::Quaterniond(solver.eigenvectors());
            Eigen::Vector3d eigenvalues = solver.eigenvalues();
            splat.scale = eigenvalues.cwiseMax(0).cwiseSqrt();
        } else {
            splat.rotation = Eigen::Quaterniond::Identity();
            splat.scale = Eigen::Vector3d::Ones() * 0.01;
        }

        // Initialize higher order spherical harmonics to zero (degree 3: 15 coeffs × 3 RGB = 45)
        splat.sh_rest = Eigen::VectorXf::Zero(45);
        splat.sh_degree = 3;

        splats.push_back(splat);
    }

    return splats;
}

Eigen::Vector3f generateRandomPosition(const Eigen::Vector3f& min_bounds,
                                       const Eigen::Vector3f& max_bounds) {
    static std::random_device rd;
    static std::mt19937 random_generator(rd());
    static std::uniform_real_distribution<float> uniform_dist(0.0f, 1.0f);

    Eigen::Vector3f position;
    for (int i = 0; i < 3; ++i) {
        position[i] =
            min_bounds[i] + uniform_dist(random_generator) * (max_bounds[i] - min_bounds[i]);
    }
    return position;
}

Eigen::Vector3f generateRandomColor() {
    static std::random_device rd;
    static std::mt19937 random_generator(rd());
    static std::uniform_real_distribution<float> uniform_dist(0.0f, 1.0f);

    // Generate neutral colors with some variation
    float base_brightness = 0.5f + uniform_dist(random_generator) * 0.3f;  // 0.5-0.8
    float color_variation = 0.1f;

    Eigen::Vector3f color;
    color[0] =
        std::clamp(static_cast<float>(base_brightness +
                                      (uniform_dist(random_generator) - 0.5) * color_variation),
                   0.0f, 1.0f);
    color[1] =
        std::clamp(static_cast<float>(base_brightness +
                                      (uniform_dist(random_generator) - 0.5) * color_variation),
                   0.0f, 1.0f);
    color[2] =
        std::clamp(static_cast<float>(base_brightness +
                                      (uniform_dist(random_generator) - 0.5) * color_variation),
                   0.0f, 1.0f);

    return color;
}

Eigen::Matrix3d generateInitialCovariance() {
    static std::random_device rd;
    static std::mt19937 random_generator(rd());
    static std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);

    double initial_variance = 0.01;

    double variance_perturbation = uniform_dist(random_generator) * 0.1 * initial_variance;

    Eigen::Matrix3d covariance =
        Eigen::Matrix3d::Identity() * (initial_variance + variance_perturbation);

    double angle = uniform_dist(random_generator) * 0.1;
    Eigen::AngleAxisd rotation(angle, Eigen::Vector3d::Random().normalized());
    Eigen::Matrix3d R = rotation.toRotationMatrix();

    covariance = R * covariance * R.transpose();

    return covariance;
}

float generateInitialOpacity() {
    static std::random_device rd;
    static std::mt19937 random_generator(rd());
    static std::uniform_real_distribution<float> uniform_dist(0.0f, 1.0f);

    // Random opacity within reasonable default range
    float min_opacity = 0.1f;
    float max_opacity = 0.8f;

    return min_opacity + uniform_dist(random_generator) * (max_opacity - min_opacity);
}

core::types::GaussianSplatBatch filterSplatsByBoundingBox(
    const core::types::GaussianSplatBatch& splat_batch, const utils::BoundingBox& bbox) {
    core::types::GaussianSplatBatch filtered_batch;
    filtered_batch.batch_id = splat_batch.batch_id;
    filtered_batch.timestamp = splat_batch.timestamp;
    filtered_batch.start_keyframe_id = splat_batch.start_keyframe_id;
    filtered_batch.end_keyframe_id = splat_batch.end_keyframe_id;
    filtered_batch.source_keyframe_ids = splat_batch.source_keyframe_ids;

    for (const auto& splat : splat_batch.splats) {
        Eigen::Vector3f position = splat.position.cast<float>();
        if (bbox.contains(position)) {
            filtered_batch.splats.push_back(splat);
        }
    }

    std::cout << "Filtered " << filtered_batch.splats.size() << " splats from "
              << splat_batch.splats.size() << " in bounding box [" << bbox.min.transpose()
              << "] to [" << bbox.max.transpose() << "]" << std::endl;

    return filtered_batch;
}

bool isRegionVisibleFromKeyframe(const core::types::KeyFrame::Ptr& keyframe,
                                 const utils::BoundingBox& bbox, float viewing_distance_threshold) {
    if (!keyframe) {
        return false;
    }

    Eigen::Vector3d camera_position = keyframe->pose.position;
    Eigen::Vector3f camera_pos_f = camera_position.cast<float>();
    Eigen::Vector3f bbox_center = bbox.center();

    // Calculate distance from camera to bounding box center
    float distance_to_center = (camera_pos_f - bbox_center).norm();

    // Check if camera is within viewing distance threshold
    if (distance_to_center > viewing_distance_threshold) {
        return false;
    }

    // Check if the bounding box is in front of the camera
    // Get camera forward direction (assuming camera looks along +Z in camera frame)
    Eigen::Vector3d forward_world = keyframe->pose.orientation * Eigen::Vector3d(0, 0, 1);
    Eigen::Vector3d to_bbox = (bbox_center.cast<double>() - camera_position).normalized();

    // Check if the bounding box is roughly in the camera's field of view
    // Use a generous angle (e.g., 120 degrees = cos(120) = -0.5)
    double dot_product = forward_world.dot(to_bbox);
    if (dot_product < -0.5) {
        return false;  // Bounding box is behind the camera
    }

    return true;
}

std::vector<uint64_t> findKeyframesViewingRegion(
    const std::vector<core::types::KeyFrame::Ptr>& all_keyframes, const utils::BoundingBox& bbox,
    float viewing_distance_threshold) {
    std::vector<uint64_t> viewing_keyframes;

    for (const auto& keyframe : all_keyframes) {
        if (isRegionVisibleFromKeyframe(keyframe, bbox, viewing_distance_threshold)) {
            viewing_keyframes.push_back(keyframe->id);
        }
    }

    std::cout << "Found " << viewing_keyframes.size() << " keyframes viewing region ["
              << bbox.min.transpose() << "] to [" << bbox.max.transpose() << "]" << std::endl;

    return viewing_keyframes;
}

std::vector<core::types::Keypoint> filterKeypointsByDistanceFromCenter(
    const std::vector<core::types::Keypoint>& keypoints, const Eigen::Vector3f& center,
    float max_distance) {
    return keypoints;
    std::vector<core::types::Keypoint> filtered_keypoints;
    filtered_keypoints.reserve(keypoints.size());

    size_t filtered_count = 0;
    for (const auto& keypoint : keypoints) {
        Eigen::Vector3f position = keypoint.position.cast<float>();
        float distance = (position - center).norm();

        if (distance <= max_distance) {
            filtered_keypoints.push_back(keypoint);
        } else {
            filtered_count++;
        }
    }

    std::cout << "Filtered " << filtered_count << " keypoints beyond " << max_distance
              << "m from center [" << center.transpose() << "]" << std::endl;
    std::cout << "Remaining keypoints: " << filtered_keypoints.size() << " / " << keypoints.size()
              << std::endl;

    return filtered_keypoints;
}

std::vector<core::types::Keypoint> filterSparseKeypoints(
    const std::vector<core::types::Keypoint>& keypoints, utils::PointCloudUtils& point_cloud_utils,
    int k_neighbors, float density_threshold) {
    std::vector<core::types::Keypoint> filtered_keypoints;
    filtered_keypoints.reserve(keypoints.size());

    size_t sparse_count = 0;
    std::vector<float> all_avg_distances;
    all_avg_distances.reserve(keypoints.size());

    std::cout << "\n=== Filtering sparse keypoints ===" << std::endl;
    std::cout << "Keypoints: " << keypoints.size() << std::endl;

    for (const auto& keypoint : keypoints) {
        Eigen::Vector3f position = keypoint.position.cast<float>();

        // Get k nearest neighbors
        std::vector<int> indices;
        std::vector<float> distances;
        point_cloud_utils.getClosestPoint(position, indices, distances, k_neighbors + 1);

        if (indices.size() < static_cast<size_t>(k_neighbors)) {
            // Not enough neighbors, consider sparse
            sparse_count++;
            continue;
        }

        // Calculate average distance to k nearest neighbors
        float avg_distance = 0.0f;
        for (size_t i = 0; i < indices.size(); ++i) {
            avg_distance += std::sqrt(distances[i]);
        }
        avg_distance /= indices.size();
        all_avg_distances.push_back(avg_distance);

        // Keep points with high density (low average distance to neighbors)
        if (avg_distance <= density_threshold) {
            filtered_keypoints.push_back(keypoint);
        } else {
            sparse_count++;
        }
    }

    std::cout << "Filtered " << sparse_count
              << " sparse keypoints (density threshold: " << density_threshold << "m)" << std::endl;
    std::cout << "Remaining keypoints: " << filtered_keypoints.size() << " / " << keypoints.size()
              << std::endl;

    // Compute and display histogram of average distances
    if (!all_avg_distances.empty()) {
        std::sort(all_avg_distances.begin(), all_avg_distances.end());

        float min_dist = all_avg_distances.front();
        float max_dist = all_avg_distances.back();
        float median_dist = all_avg_distances[all_avg_distances.size() / 2];

        std::cout << "\n=== Distance Statistics ===" << std::endl;
        std::cout << "Min distance: " << min_dist << "m" << std::endl;
        std::cout << "Max distance: " << max_dist << "m" << std::endl;
        std::cout << "Median distance: " << median_dist << "m" << std::endl;

        // Create histogram with 10 bins
        const int num_bins = 10;
        std::vector<int> histogram(num_bins, 0);
        float bin_width = (max_dist - min_dist) / num_bins;

        if (bin_width > 0) {
            for (float dist : all_avg_distances) {
                int bin = static_cast<int>((dist - min_dist) / bin_width);
                bin = std::min(bin, num_bins - 1);  // Handle edge case for max value
                histogram[bin]++;
            }

            std::cout << "\n=== Distance Histogram ===" << std::endl;
            int max_count = *std::max_element(histogram.begin(), histogram.end());
            const int bar_width = 50;

            for (int i = 0; i < num_bins; ++i) {
                float bin_start = min_dist + i * bin_width;
                float bin_end = min_dist + (i + 1) * bin_width;

                int bar_len = (max_count > 0) ? (histogram[i] * bar_width / max_count) : 0;

                std::cout << std::fixed << std::setprecision(2);
                std::cout << "[" << std::setw(6) << bin_start << " - " << std::setw(6) << bin_end
                          << "): ";

                for (int j = 0; j < bar_len; ++j) {
                    std::cout << "#";
                }
                std::cout << " " << histogram[i] << std::endl;
            }
            std::cout << std::endl;
        }
    }

    return filtered_keypoints;
}

std::pair<int, int> detectDominant2DPlane(
    const std::vector<core::types::KeyFrame::Ptr>& keyframes) {
    if (keyframes.empty()) {
        return {0, 1};  // Default to XY plane
    }

    // Collect all keyframe positions
    std::vector<Eigen::Vector3f> positions;
    positions.reserve(keyframes.size());
    for (const auto& kf : keyframes) {
        positions.push_back(kf->pose.position.cast<float>());
    }

    // Compute variance along each axis
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& pos : positions) {
        mean += pos;
    }
    mean /= positions.size();

    Eigen::Vector3f variance = Eigen::Vector3f::Zero();
    for (const auto& pos : positions) {
        Eigen::Vector3f diff = pos - mean;
        variance += diff.cwiseProduct(diff);
    }
    variance /= positions.size();

    std::cout << "\n=== Keyframe Position Variance Analysis ===" << std::endl;
    std::cout << "Variance along X: " << variance.x() << std::endl;
    std::cout << "Variance along Y: " << variance.y() << std::endl;
    std::cout << "Variance along Z: " << variance.z() << std::endl;

    // Find the axis with minimum variance (this is the "vertical" axis)
    int min_variance_axis = 0;
    float min_var = variance.x();
    if (variance.y() < min_var) {
        min_variance_axis = 1;
        min_var = variance.y();
    }
    if (variance.z() < min_var) {
        min_variance_axis = 2;
    }

    // The other two axes form the dominant 2D plane
    std::vector<int> plane_axes;
    for (int i = 0; i < 3; ++i) {
        if (i != min_variance_axis) {
            plane_axes.push_back(i);
        }
    }

    std::string axis_names[] = {"X", "Y", "Z"};
    std::cout << "Detected dominant 2D plane: " << axis_names[plane_axes[0]] << "-"
              << axis_names[plane_axes[1]] << " (vertical axis: " << axis_names[min_variance_axis]
              << ")" << std::endl;

    return {plane_axes[0], plane_axes[1]};
}

std::vector<KeyframeRegion> partitionKeyframesInto2DGrid(
    const std::vector<core::types::KeyFrame::Ptr>& keyframes,
    const std::vector<core::types::Keypoint>& keypoints, int grid_rows, int grid_cols) {
    std::cout << "\n=== Partitioning keyframes into " << grid_rows << "x" << grid_cols
              << " 2D grid ===" << std::endl;

    // Detect dominant 2D plane
    auto [axis1, axis2] = detectDominant2DPlane(keyframes);
    int vertical_axis = 3 - axis1 - axis2;  // The remaining axis

    // Find min/max bounds along the 2D plane for keyframes
    float min_coord1 = std::numeric_limits<float>::max();
    float max_coord1 = std::numeric_limits<float>::lowest();
    float min_coord2 = std::numeric_limits<float>::max();
    float max_coord2 = std::numeric_limits<float>::lowest();

    for (const auto& kf : keyframes) {
        Eigen::Vector3f pos = kf->pose.position.cast<float>();
        float coord1 = pos[axis1];
        float coord2 = pos[axis2];

        min_coord1 = std::min(min_coord1, coord1);
        max_coord1 = std::max(max_coord1, coord1);
        min_coord2 = std::min(min_coord2, coord2);
        max_coord2 = std::max(max_coord2, coord2);
    }

    // Add padding to ensure all keyframes are included
    float padding_1 = (max_coord1 - min_coord1) * 0.1f;
    float padding_2 = (max_coord2 - min_coord2) * 0.1f;
    min_coord1 -= padding_1;
    max_coord1 += padding_1;
    min_coord2 -= padding_2;
    max_coord2 += padding_2;

    std::cout << "2D grid bounds: axis" << axis1 << " [" << min_coord1 << ", " << max_coord1
              << "], axis" << axis2 << " [" << min_coord2 << ", " << max_coord2 << "]" << std::endl;

    // Find vertical extent from keypoints
    float min_vertical = std::numeric_limits<float>::max();
    float max_vertical = std::numeric_limits<float>::lowest();
    for (const auto& kp : keypoints) {
        float vert_coord = kp.position.cast<float>()[vertical_axis];
        min_vertical = std::min(min_vertical, vert_coord);
        max_vertical = std::max(max_vertical, vert_coord);
    }

    // Add vertical padding
    float vert_padding = (max_vertical - min_vertical) * 0.1f;
    min_vertical -= vert_padding;
    max_vertical += vert_padding;

    std::cout << "Vertical extent (axis " << vertical_axis << "): [" << min_vertical << ", "
              << max_vertical << "]" << std::endl;

    // Create grid cells
    float cell_width_1 = (max_coord1 - min_coord1) / grid_cols;
    float cell_width_2 = (max_coord2 - min_coord2) / grid_rows;

    std::vector<KeyframeRegion> regions;
    int region_id = 0;

    for (int row = 0; row < grid_rows; ++row) {
        for (int col = 0; col < grid_cols; ++col) {
            KeyframeRegion region;
            region.region_id = region_id++;

            // Compute 2D bounds for this cell
            float cell_min_1 = min_coord1 + col * cell_width_1;
            float cell_max_1 = cell_min_1 + cell_width_1;
            float cell_min_2 = min_coord2 + row * cell_width_2;
            float cell_max_2 = cell_min_2 + cell_width_2;

            // Create 3D bounding box
            Eigen::Vector3f bbox_min, bbox_max;
            bbox_min[axis1] = cell_min_1;
            bbox_max[axis1] = cell_max_1;
            bbox_min[axis2] = cell_min_2;
            bbox_max[axis2] = cell_max_2;
            bbox_min[vertical_axis] = min_vertical;
            bbox_max[vertical_axis] = max_vertical;

            region.bbox_3d = utils::BoundingBox(bbox_min, bbox_max);
            region.center_2d =
                Eigen::Vector2f((cell_min_1 + cell_max_1) / 2.0f, (cell_min_2 + cell_max_2) / 2.0f);

            // Assign keyframes to this region
            for (const auto& kf : keyframes) {
                Eigen::Vector3f pos = kf->pose.position.cast<float>();
                float kf_coord1 = pos[axis1];
                float kf_coord2 = pos[axis2];

                if (kf_coord1 >= cell_min_1 && kf_coord1 < cell_max_1 && kf_coord2 >= cell_min_2 &&
                    kf_coord2 < cell_max_2) {
                    region.keyframe_ids.push_back(kf->id);
                }
            }

            // Only add region if it has keyframes
            if (!region.keyframe_ids.empty()) {
                std::cout << "Region " << region.region_id << ": " << region.keyframe_ids.size()
                          << " keyframes" << std::endl;
                regions.push_back(region);
            }
        }
    }

    std::cout << "Created " << regions.size() << " non-empty regions" << std::endl;
    return regions;
}

std::vector<KeyframeRegion> partitionKeyframesIntoRadialSectors(
    const std::vector<core::types::KeyFrame::Ptr>& keyframes,
    const std::vector<core::types::Keypoint>& keypoints, int num_sectors, float overlap_angle) {
    std::cout << "\n=== Partitioning keyframes into " << num_sectors
              << " radial sectors (overlap: " << overlap_angle << " degrees) ===" << std::endl;

    if (keyframes.empty() || keypoints.empty()) {
        return {};
    }

    // Detect dominant 2D plane
    auto [axis1, axis2] = detectDominant2DPlane(keyframes);
    int vertical_axis = 3 - axis1 - axis2;

    // Compute shared center from keypoints (the object being viewed)
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    for (const auto& kp : keypoints) {
        center += kp.position.cast<float>();
    }
    center /= keypoints.size();

    std::cout << "Shared center (from keypoints): [" << center.transpose() << "]" << std::endl;

    // Find vertical extent from keypoints
    float min_vertical = std::numeric_limits<float>::max();
    float max_vertical = std::numeric_limits<float>::lowest();
    for (const auto& kp : keypoints) {
        float vert_coord = kp.position.cast<float>()[vertical_axis];
        min_vertical = std::min(min_vertical, vert_coord);
        max_vertical = std::max(max_vertical, vert_coord);
    }

    float vert_padding = (max_vertical - min_vertical) * 0.1f;
    min_vertical -= vert_padding;
    max_vertical += vert_padding;

    // Find maximum radius from keyframes to center (in 2D plane)
    float max_radius = 0.0f;
    for (const auto& kf : keyframes) {
        Eigen::Vector3f pos = kf->pose.position.cast<float>();
        float dx = pos[axis1] - center[axis1];
        float dy = pos[axis2] - center[axis2];
        float radius = std::sqrt(dx * dx + dy * dy);
        max_radius = std::max(max_radius, radius);
    }

    // Add padding to ensure all points are included
    float horizontal_padding = max_radius * 0.3f;
    max_radius += horizontal_padding;

    std::cout << "Maximum radius from center: " << max_radius << "m" << std::endl;
    std::cout << "Vertical extent (axis " << vertical_axis << "): [" << min_vertical << ", "
              << max_vertical << "]" << std::endl;

    // Create radial sectors with overlap
    std::vector<KeyframeRegion> regions;
    float sector_angle = 360.0f / num_sectors;
    float overlap_rad = overlap_angle * M_PI / 180.0f;
    float sector_angle_rad = sector_angle * M_PI / 180.0f;

    for (int sector = 0; sector < num_sectors; ++sector) {
        KeyframeRegion region;
        region.region_id = sector;

        // Calculate sector angular range with overlap
        float sector_center_angle = sector * sector_angle_rad;
        float angle_start = sector_center_angle - sector_angle_rad / 2.0f - overlap_rad;
        float angle_end = sector_center_angle + sector_angle_rad / 2.0f + overlap_rad;

        // 2D center for this sector (outward from shared center)
        float center_angle = sector * sector_angle_rad;
        region.center_2d =
            Eigen::Vector2f(center[axis1] + max_radius * 0.5f * std::cos(center_angle),
                            center[axis2] + max_radius * 0.5f * std::sin(center_angle));

        // Create bounding box that covers the sector
        // All sectors share the same center but extend outward in their direction
        Eigen::Vector3f bbox_min, bbox_max;

        // Calculate sector extents in 2D
        std::vector<Eigen::Vector2f> sector_corners;
        sector_corners.push_back(Eigen::Vector2f(0, 0));  // Center

        // Sample points along the arc
        int num_arc_samples = 10;
        for (int i = 0; i <= num_arc_samples; ++i) {
            float angle = angle_start + (angle_end - angle_start) * i / num_arc_samples;
            sector_corners.push_back(
                Eigen::Vector2f(max_radius * std::cos(angle), max_radius * std::sin(angle)));
        }

        // Find min/max in 2D
        float min_2d_1 = std::numeric_limits<float>::max();
        float max_2d_1 = std::numeric_limits<float>::lowest();
        float min_2d_2 = std::numeric_limits<float>::max();
        float max_2d_2 = std::numeric_limits<float>::lowest();

        for (const auto& corner : sector_corners) {
            min_2d_1 = std::min(min_2d_1, corner.x());
            max_2d_1 = std::max(max_2d_1, corner.x());
            min_2d_2 = std::min(min_2d_2, corner.y());
            max_2d_2 = std::max(max_2d_2, corner.y());
        }

        // Create 3D bounding box
        bbox_min[axis1] = center[axis1] + min_2d_1;
        bbox_max[axis1] = center[axis1] + max_2d_1;
        bbox_min[axis2] = center[axis2] + min_2d_2;
        bbox_max[axis2] = center[axis2] + max_2d_2;
        bbox_min[vertical_axis] = min_vertical;
        bbox_max[vertical_axis] = max_vertical;

        region.bbox_3d = utils::BoundingBox(bbox_min, bbox_max);

        // Assign keyframes to this sector based on their angle from center
        for (const auto& kf : keyframes) {
            Eigen::Vector3f pos = kf->pose.position.cast<float>();
            float dx = pos[axis1] - center[axis1];
            float dy = pos[axis2] - center[axis2];
            float kf_angle = std::atan2(dy, dx);

            // Normalize angles to [0, 2π]
            auto normalize_angle = [](float angle) {
                while (angle < 0)
                    angle += 2.0f * M_PI;
                while (angle >= 2.0f * M_PI)
                    angle -= 2.0f * M_PI;
                return angle;
            };

            kf_angle = normalize_angle(kf_angle);
            float norm_angle_start = normalize_angle(angle_start);
            float norm_angle_end = normalize_angle(angle_end);

            // Check if keyframe angle is within sector range (handling wraparound)
            bool in_sector = false;
            if (norm_angle_start <= norm_angle_end) {
                in_sector = (kf_angle >= norm_angle_start && kf_angle <= norm_angle_end);
            } else {
                // Sector wraps around 0
                in_sector = (kf_angle >= norm_angle_start || kf_angle <= norm_angle_end);
            }

            if (in_sector) {
                region.keyframe_ids.push_back(kf->id);
            }
        }

        if (!region.keyframe_ids.empty()) {
            std::cout << "Sector " << sector << " (angle " << std::fixed << std::setprecision(1)
                      << (sector * sector_angle) << "°): " << region.keyframe_ids.size()
                      << " keyframes" << std::endl;
            regions.push_back(region);
        }
    }

    std::cout << "Created " << regions.size() << " non-empty sectors" << std::endl;
    return regions;
}

bool initializeRandomSplats(const std::vector<uint64_t>& keyframe_ids,
                            std::shared_ptr<core::storage::MapStore>& map_store,
                            const uint64_t& current_batch_id, double& current_timestamp,
                            core::types::GaussianSplatBatch& splat_batch,
                            std::atomic<uint64_t>& next_splat_id, int splat_count) {
    std::cout << "Initialize random splats" << std::endl;
    std::cout << "Initialize random splats" << std::endl;

    auto all_keypoints = map_store->getAllKeyPoints();
    std::cout << "No keypoints: " << all_keypoints.size() << std::endl;
    if (!all_keypoints.size()) {
        return false;
    }
    try {
        // Estimate scene bounds from camera trajectory
        auto [scene_min, scene_max] = estimateSceneBoundsFromTrajectory(map_store);

        std::cout << "Estimated scene bounds: min(" << scene_min.transpose() << ") max("
                  << scene_max.transpose() << ")" << std::endl;

        // Create splat batch
        splat_batch.batch_id = current_batch_id;
        splat_batch.timestamp = current_timestamp;

        // Store source keyframe IDs
        for (const auto& kf_id : keyframe_ids) {
            splat_batch.source_keyframe_ids.insert(kf_id);
        }

        // Generate random splats within scene bounds
        auto random_splats = generateRandomSplats(scene_min, scene_max, splat_count, next_splat_id,
                                                  current_timestamp);

        std::cout << "Generated " << random_splats.size() << " random splats" << std::endl;

        splat_batch.splats = random_splats;

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error initializing random splats: " << e.what();
        return false;
    }
}

bool loadKeyframesToTensorBatch(const std::vector<uint64_t>& keyframe_ids,
                                const std::shared_ptr<core::storage::MapStore>& map_store,
                                const std::shared_ptr<stf::TransformTree>& tf_tree,
                                const torch::Device& device, const uint32_t& batch_id,
                                training::KeyframeBatch& keyframe_batch) {
    std::cout << "Loading stuff!!" << std::endl;
    if (!tf_tree) {
        LOG(ERROR) << "Transform tree not initialized for camera pose extraction";
        return false;
    }

    // Try to get base_link to camera transform from transform tree
    // std::string camera_frame = "camera_color_optical_frame";  // Default camera frame
    std::string camera_frame = "camera";  // Default camera frame
    // Get camera transform relative to base_link
    Eigen::Isometry3d base_to_camera = Eigen::Isometry3d::Identity();
    try {
        auto transform_result = tf_tree->getTransform("base_link", camera_frame);
        base_to_camera = transform_result.transform;
    } catch (const std::exception& e) {
        LOG(WARNING) << "Failed to get base_link to camera transform: " << e.what()
                     << ", using identity";
        return false;
    }
    auto all_keyframes = map_store->getAllKeyFrames();
    std::cout << "All keyframes: " << all_keyframes.size()
              << " locations keyframe ids:" << keyframe_ids.size() << std::endl;

    std::cout << "Loading more stuff" << std::endl;
    // Prepare tensors for all keyframes
    std::vector<torch::Tensor> image_tensors;
    std::vector<torch::Tensor> pose_tensors;
    std::vector<torch::Tensor> intrinsic_tensors;
    std::vector<core::types::KeyFrame::Ptr> keyframes;
    std::vector<uint64_t> kf_ids;

    image_tensors.reserve(keyframe_ids.size());
    pose_tensors.reserve(keyframe_ids.size());
    intrinsic_tensors.reserve(keyframe_ids.size());

    try {
        uint32_t nk = 0;
        for (auto keyframe : all_keyframes) {
            if (keyframe->id % 5 != 0) {
                continue;
            }
            if (!keyframe) {
                std::cout << "Unable to get the keyframe " << keyframe->id << std::endl;
                continue;
            }

            Eigen::Isometry3d camera_pose;
            Eigen::Isometry3d base_pose = keyframe->pose.getEigenIsometry();
            camera_pose = base_pose * base_to_camera;
            pose_tensors.push_back(convertCameraPoseToTensor(camera_pose, device));

            torch::Tensor image_tensor;
            core::types::CameraInfo camera_info;
            std::cout << "Extracting keyframe images for " << keyframe->id << std::endl;
            if (!extractImageTensor(keyframe, device, image_tensor, camera_info)) {
                LOG(ERROR) << "Failed to extract image tensor from keyframe " << keyframe->id;
                continue;
            }

            // Dumber doing dumb things
            keyframe_batch.image_height = camera_info.height;
            keyframe_batch.image_width = camera_info.width;

            // Convert camera intrinsics to tensor
            torch::Tensor intrinsics_tensor = convertCameraIntrinsicsToTensor(camera_info, device);

            kf_ids.push_back(keyframe->id);
            keyframes.push_back(keyframe);

            image_tensors.push_back(image_tensor);
            intrinsic_tensors.push_back(intrinsics_tensor);
        }
        std::cout << "POse: " << pose_tensors.size()
                  << " intrinsic_tensors: " << intrinsic_tensors.size()
                  << " image_tensors: " << image_tensors.size() << keyframe_batch.keyframes.size()
                  << std::endl;

        assert(image_tensors.size() == intrinsic_tensors.size());
        assert(image_tensors.size() == pose_tensors.size());
        // Clear and initialize the batch
        keyframe_batch.clear();
        keyframe_batch.batch_id = batch_id;

        keyframe_batch.batch_size = image_tensors.size();
        keyframe_batch.device = device;
        keyframe_batch.keyframe_ids = keyframe_ids;
        keyframe_batch.keyframes = keyframes;

        // Stack individual tensors into batch tensors
        keyframe_batch.images = torch::stack(image_tensors, 0);
        keyframe_batch.images = keyframe_batch.images.permute({0, 3, 1, 2});
        keyframe_batch.camera_intrinsics = torch::stack(intrinsic_tensors, 0);  // [N, 3, 3]
        keyframe_batch.camera_poses = torch::stack(pose_tensors, 0);            // [N, 4, 4]
        for (uint32_t i = 0; i < image_tensors.size(); i++) {
            std::string output_dir = "/data/robot/log/input";
            std::string filename = "stacked_" + std::to_string(i) + ".png";
            auto conv_ten_img = utils::tensorToMat(keyframe_batch.images[i], true);
            utils::writeImageToDirectory(conv_ten_img, output_dir, filename);
            filename = "ectored_" + std::to_string(i) + ".png";
            auto conv_img = utils::tensorToMat(image_tensors[i], true);
            utils::writeImageToDirectory(conv_img, output_dir, filename);
        }

        std::cout << "Printing stuff" << std::endl;
        std::cout << "Printing stuff" << std::endl;
        keyframe_batch.print();
        keyframe_batch.to(device);

        return true;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception extracting camera poses: " << e.what();
        return false;
    }
}

bool extractImageTensor(const core::types::KeyFrame::Ptr& keyframe, const torch::Device& device,
                        torch::Tensor& image_tensor, core::types::CameraInfo& camera_info) {
    if (!keyframe) {
        LOG(ERROR) << "Null keyframe provided for image extraction";
        return false;
    }

    std::cout << "EXTRACT!!!" << std::endl;
    try {
        // Extract camera info from keyframe or use default
        if (keyframe->hasCameraInfo()) {
            camera_info = keyframe->getCameraInfo();
        } else {
            LOG(WARNING) << "Unable to get the camera info for " << keyframe->id;
            return false;
        }

        // Extract image data
        cv::Mat image_cv;
        if (keyframe->hasColorImage()) {
            // Prefer color image
            std::cout << "taking the colored image" << std::endl;
            const auto& color_image = keyframe->getColorImage();
            image_cv = color_image.data.clone();
        } else if (keyframe->hasImage()) {
            // Fall back to depth/grayscale image
            const auto& depth_image = keyframe->getImage();
            image_cv = depth_image.data.clone();

            // Convert single channel to 3-channel if needed
            if (image_cv.channels() == 1) {
                cv::cvtColor(image_cv, image_cv, cv::COLOR_GRAY2BGR);
            }
        } else {
            LOG(ERROR) << "Keyframe " << keyframe->id << " has no image data";
            return false;
        }

        // Resize image to training resolution
        std::cout << "Image sizes: " << image_cv.cols << ", " << image_cv.rows << " |  "
                  << camera_info.width << ", " << camera_info.height << std::endl;
        if (image_cv.cols != camera_info.width || image_cv.rows != camera_info.height) {
            cv::resize(image_cv, image_cv, cv::Size(camera_info.width, camera_info.height));
        }

        std::cout << "image type: " << image_cv.type() << " - " << CV_32FC3 << std::endl;
        std::cout << "image type: " << image_cv.type() << " - " << CV_32FC3 << std::endl;
        if (image_cv.type() != CV_32FC3) {
            image_cv.convertTo(image_cv, CV_32FC3, 1.0 / 255.0);
        }

        image_tensor =
            torch::from_blob(image_cv.data, {image_cv.rows, image_cv.cols, 3}, torch::kFloat32);
        std::cout << "Image tensor: " << image_tensor.sizes() << std::endl;
        // image_tensor = image_tensor.permute({2, 0, 1});  // HWC -> CHW
        image_tensor = image_tensor.to(device);
        // std::string filename = std::to_string(keyframe->id) + ".png";

        // // Write to output directory
        // std::string output_dir = "/data/robot/log/input";
        // auto conv_ten_img = utils::tensorToMat(image_tensor, true);
        // utils::writeImageToDirectory(conv_ten_img, output_dir, filename);

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception extracting image tensor from keyframe " << keyframe->id << ": "
                   << e.what();
        return false;
    }
}

torch::Tensor convertCameraIntrinsicsToTensor(const core::types::CameraInfo& camera_info,
                                              const torch::Device& device) {
    // Create 3x3 camera intrinsics matrix
    torch::Tensor intrinsics =
        torch::zeros({3, 3}, torch::TensorOptions().dtype(torch::kFloat32).device(device));

    // Extract from camera matrix K [fx, 0, cx; 0, fy, cy; 0, 0, 1]
    if (camera_info.k.size() >= 9) {
        intrinsics[0][0] = camera_info.k[0];  // fx
        intrinsics[1][1] = camera_info.k[4];  // fy
        intrinsics[0][2] = camera_info.k[2];  // cx
        intrinsics[1][2] = camera_info.k[5];  // cy
    } else {
        // Fallback defaults
        intrinsics[0][0] = 500.0;  // fx
        intrinsics[1][1] = 500.0;  // fy
        intrinsics[0][2] = 320.0;  // cx
        intrinsics[1][2] = 240.0;  // cy
    }
    intrinsics[2][2] = 1.0;  // homogeneous coordinate

    return intrinsics;
}

torch::Tensor convertCameraPoseToTensor(const Eigen::Isometry3d& pose,
                                        const torch::Device& device) {
    // Convert 4x4 pose matrix to torch tensor
    torch::Tensor pose_tensor =
        torch::zeros({4, 4}, torch::TensorOptions().dtype(torch::kFloat32).device(device));

    Eigen::Matrix4d pose_matrix = pose.matrix();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            pose_tensor[i][j] = static_cast<float>(pose_matrix(i, j));
        }
    }

    return pose_tensor;
}

}  // namespace gaussian_splatting

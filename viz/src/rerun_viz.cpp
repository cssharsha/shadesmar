#include "viz/rerun_viz.hpp"
#include "viz/rerun_eigen_adapters.hpp"

#include "logging/logging.hpp"

#include "core/types/keyframe.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rerun.hpp>
#include <rerun/archetypes/image.hpp>
#include <rerun/archetypes/transform3d.hpp>
#include <rerun/datatypes/quaternion.hpp>
#include <rerun/datatypes/vec3d.hpp>
#include <rerun/recording_stream.hpp>
#include <string>
#include <thread>

namespace viz {

RerunVisualizer::RerunVisualizer(const std::string& name, const std::string& host, uint16_t port)
    : name_(name), host_(host), port_(port), rec_(name) {
    rec_.spawn().exit_on_failure();
}

bool RerunVisualizer::initialize(bool save_to_file) {
    try {
        if (save_to_file) {
            std::string recording_path = "/data/" + name_ + ".rrd";
            auto result = rec_.save(recording_path);

            if (result.is_err()) {
                LOG(ERROR) << "FATAL: Failed to save Rerun recording to " << recording_path << ": ";
                std::exit(1);
                return false;
            }
            if (!rec_.is_enabled()) {
                LOG(ERROR) << "Failed to create recording file (is_enabled is false after save)";
                return false;
            }
        } else {
            if (!rec_.is_enabled()) {
                LOG(ERROR)
                    << "Error: Rerun recording stream is not enabled after constructor's spawn().";
            }
            LOG(INFO) << "RerunVisualizer configured to connect to spawned/existing viewer.";
        }

        is_connected_ =
            rec_.is_enabled();  // is_enabled() should be true if save() or spawn() was successful.
        if (is_connected_) {
            LOG(INFO) << "RerunVisualizer initialized. Logging to "
                      << (save_to_file ? "file." : "spawned/connected viewer.");
        }
        return is_connected_;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception during RerunVisualizer::initialize: " << e.what();
        return false;
    }
}

bool RerunVisualizer::isConnected() const {
    return is_connected_;
}

void RerunVisualizer::disconnect() {
    if (is_connected_) {
        try {
            rec_.log("disconnect", rerun::TextLog("disconnecting"));
        } catch (const std::exception&) {
            // Ignore errors during disconnect
        }
        is_connected_ = false;
    }
}

void RerunVisualizer::addPose(const core::types::Pose& pose, const std::string& entity_path,
                              double timestamp) {
    if (!is_connected_)
        return;

    rec_.log(entity_path, toRerunTransform(pose));
}

void RerunVisualizer::addPointCloud(const core::types::PointCloud& cloud,
                                    const std::string& entity_path, double timestamp,
                                    const core::types::Pose& transform) {
    if (!is_connected_)
        return;

    rec_.log(entity_path, toRerunPoints(cloud, transform));
}

void RerunVisualizer::addCamera(const rerun::archetypes::Pinhole& camera,
                                const std::string& entity_path, double timestamp) {
    if (!is_connected_)
        return;

    rec_.log(entity_path, camera);
}

void RerunVisualizer::addImage(const cv::Mat& image, const std::string& entity_path,
                               double timestamp) {
    if (!is_connected_)
        return;
    if (image.channels() == 1) {
        // For monochrome images, Rerun might expect a 2D tensor or specific format.
        // Shape: H, W
        // Buffer: Collection of all uint8_t pixels
        rec_.log(entity_path,
                 rerun::Image({static_cast<size_t>(image.rows), static_cast<size_t>(image.cols)},
                              rerun::TensorBuffer::u8(rerun::Collection<uint8_t>::borrow(
                                  image.data, image.total() * image.channels()))));
    } else {
        // Assuming image.data is RGB or BGR.
        // If BGR, conversion to RGB might be needed if Rerun expects RGB.
        // cv::Mat rgb_image;
        // cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);
        // then use rgb_image.data, rgb_image.rows, rgb_image.cols, rgb_image.channels()

        // Shape: H, W, C
        // Buffer: Collection of all uint8_t pixels (interleaved)
        rec_.log(entity_path,
                 rerun::Image({static_cast<size_t>(image.rows), static_cast<size_t>(image.cols),
                               static_cast<size_t>(image.channels())},
                              rerun::TensorBuffer::u8(rerun::Collection<uint8_t>::borrow(
                                  image.data, image.total() * image.channels()))));
    }
}

void RerunVisualizer::visualizeFromStorage(
    const core::storage::MapStore& map_store,
    uint64_t current_keyframe_id,
    uint64_t previous_keyframe_id,
    size_t trajectory_keyframe_count) {
    if (!is_connected_)
        return;

    LOG(INFO) << "Visualizing three-queue system - current KF: " << current_keyframe_id
              << ", previous KF: " << previous_keyframe_id;

    // ===== GET KEYFRAMES FROM THREE-QUEUE SYSTEM =====

    // Get processed non-optimized keyframes (yellow/orange trajectory)
    auto processed_non_optimized_kfs = map_store.getProcessedNonOptimizedKeyFrames();
    LOG(INFO) << "Processed non-optimized keyframes: " << processed_non_optimized_kfs.size();

    // Get processed optimized keyframes (green trajectory)
    auto processed_optimized_kfs = map_store.getProcessedOptimizedKeyFrames();
    LOG(INFO) << "Processed optimized keyframes: " << processed_optimized_kfs.size();

    // Get additional keyframes from disk if they've been synced
    auto disk_keyframes = map_store.getAllKeyFrames();
    LOG(INFO) << "Total keyframes on disk: " << disk_keyframes.size();

    // Combine processed keyframes for trajectory (processed non-optimized + processed optimized + disk)
    std::vector<std::shared_ptr<core::types::KeyFrame>> all_processed_keyframes;

    // Add processed non-optimized (these are ORB-processed but not optimized yet)
    for (const auto& kf : processed_non_optimized_kfs) {
        all_processed_keyframes.push_back(kf);
    }

    // Add processed optimized (these are ORB-processed and optimized)
    for (const auto& kf : processed_optimized_kfs) {
        all_processed_keyframes.push_back(kf);
    }

    // Add disk keyframes (these have been fully processed and synced)
    for (const auto& kf : disk_keyframes) {
        // Avoid duplicates - only add if not already in processed queues
        bool already_added = false;
        for (const auto& existing_kf : all_processed_keyframes) {
            if (existing_kf->id == kf->id) {
                already_added = true;
                break;
            }
        }
        if (!already_added) {
            all_processed_keyframes.push_back(kf);
        }
    }

    if (all_processed_keyframes.empty()) {
        LOG(WARNING) << "No processed keyframes found for visualization";
        return;
    }

    // Sort by ID for proper trajectory order
    std::sort(all_processed_keyframes.begin(), all_processed_keyframes.end(),
              [](const auto& a, const auto& b) { return a->id < b->id; });

    // ===== GET MAP KEYPOINTS =====

    // Get map keypoints directly from MapStore (single source of truth)
    auto all_keypoints = map_store.getAllKeyPoints();
    std::map<uint32_t, core::types::Keypoint> map_keypoints;
    for (const auto& kp : all_keypoints) {
        map_keypoints[kp.id()] = kp;
    }

    // ===== SET TIMELINE =====

    // Get current keyframe for timeline
    auto current_kf = map_store.getKeyFrame(current_keyframe_id);
    if (!current_kf) {
        // Try to get from processed queues
        current_kf = map_store.getProcessedNonOptimizedKeyFrame(current_keyframe_id);
        if (!current_kf) {
            current_kf = map_store.getProcessedOptimizedKeyFrame(current_keyframe_id);
            if (!current_kf) {
                current_kf = map_store.getUnprocessedKeyFrame(current_keyframe_id);
            }
        }
    }

    // Set timeline and current timestamp for visualization
    if (current_kf) {
        current_timestamp_ = current_kf->pose.timestamp;
        rec_.set_time_sequence("max_keyframe_id", current_kf->pose.timestamp);
        LOG(INFO) << "Set timeline timestamp: " << current_timestamp_;
    } else {
        // Fallback timestamp if no current keyframe found
        current_timestamp_ = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        LOG(WARNING) << "No current keyframe found, using system timestamp: " << current_timestamp_;
    }

    LOG(INFO) << "Using " << all_processed_keyframes.size()
              << " processed keyframes for visualization ("
              << processed_non_optimized_kfs.size() << " non-optimized + "
              << processed_optimized_kfs.size() << " optimized + "
              << (all_processed_keyframes.size() - processed_non_optimized_kfs.size() - processed_optimized_kfs.size())
              << " from disk)";

    // ===== VISUALIZE DIFFERENT PROCESSING STAGES =====

    // Processed Non-Optimized trajectory (Orange - ORB processed but not optimized)
    std::vector<rerun::datatypes::Vec3D> non_optimized_points;
    for (const auto& kf : processed_non_optimized_kfs) {
        auto position = kf->pose.position;
        non_optimized_points.emplace_back(rerun::datatypes::Vec3D{
            static_cast<float>(position.x()),
            static_cast<float>(position.y()),
            static_cast<float>(position.z())});
    }

    // Processed Optimized trajectory (Green - ORB processed and optimized)
    std::vector<rerun::datatypes::Vec3D> optimized_points;
    for (const auto& kf : processed_optimized_kfs) {
        auto position = kf->pose.position;
        optimized_points.emplace_back(rerun::datatypes::Vec3D{
            static_cast<float>(position.x()),
            static_cast<float>(position.y()),
            static_cast<float>(position.z())});
    }

    // Disk-synced trajectory (Blue - fully processed and persisted)
    std::vector<rerun::datatypes::Vec3D> disk_points;
    for (const auto& kf : disk_keyframes) {
        // Only add if not in other queues (to avoid duplicates)
        bool in_other_queue = false;
        for (const auto& proc_kf : processed_non_optimized_kfs) {
            if (proc_kf->id == kf->id) { in_other_queue = true; break; }
        }
        for (const auto& opt_kf : processed_optimized_kfs) {
            if (opt_kf->id == kf->id) { in_other_queue = true; break; }
        }

        if (!in_other_queue) {
            auto position = kf->pose.position;
            disk_points.emplace_back(rerun::datatypes::Vec3D{
                static_cast<float>(position.x()),
                static_cast<float>(position.y()),
                static_cast<float>(position.z())});
        }
    }

    // Visualize each processing stage with different colors
    std::string base_path = "/" + reference_frame_id_ + "/trajectory";

    if (!non_optimized_points.empty()) {
        rec_.log(base_path + "/non_optimized",
                 rerun::Points3D({non_optimized_points})
                     .with_colors({rerun::components::Color(255, 165, 0)}));  // Orange

        if (non_optimized_points.size() > 1) {
            rec_.log(base_path + "/non_optimized",
                     rerun::LineStrips3D({non_optimized_points})
                         .with_colors({rerun::components::Color(255, 165, 0)}));  // Orange
        }
        LOG(INFO) << "Visualized " << non_optimized_points.size() << " non-optimized keyframes (orange)";
    }

    if (!optimized_points.empty()) {
        rec_.log(base_path + "/optimized",
                 rerun::Points3D({optimized_points})
                     .with_colors({rerun::components::Color(0, 255, 0)}));  // Green

        if (optimized_points.size() > 1) {
            rec_.log(base_path + "/optimized",
                     rerun::LineStrips3D({optimized_points})
                         .with_colors({rerun::components::Color(0, 255, 0)}));  // Green
        }
        LOG(INFO) << "Visualized " << optimized_points.size() << " optimized keyframes (green)";
    }

    if (!disk_points.empty()) {
        rec_.log(base_path + "/disk_synced",
                 rerun::Points3D({disk_points})
                     .with_colors({rerun::components::Color(0, 100, 255)}));  // Blue

        if (disk_points.size() > 1) {
            rec_.log(base_path + "/disk_synced",
                     rerun::LineStrips3D({disk_points})
                         .with_colors({rerun::components::Color(0, 100, 255)}));  // Blue
        }
        LOG(INFO) << "Visualized " << disk_points.size() << " disk-synced keyframes (blue)";
    }

    // ===== VISUALIZE CAMERAS (only from processed keyframes) =====

    uint32_t numCameras = 0;
    std::vector<std::shared_ptr<core::types::KeyFrame>> camera_keyframes;

    // ===== GET MOST RECENT CURRENT AND PREVIOUS KEYFRAMES =====
    // Priority order: unprocessed > processed_non_optimized > processed_optimized > disk

    LOG(INFO) << "Looking for current keyframe " << current_keyframe_id << " for camera visualization";

    // Find current keyframe (most recent)
    auto current_processed_kf = map_store.getUnprocessedKeyFrame(current_keyframe_id);
    if (!current_processed_kf) {
        current_processed_kf = map_store.getProcessedNonOptimizedKeyFrame(current_keyframe_id);
        if (!current_processed_kf) {
            current_processed_kf = map_store.getProcessedOptimizedKeyFrame(current_keyframe_id);
            if (!current_processed_kf) {
                current_processed_kf = map_store.getKeyFrame(current_keyframe_id);
            }
        }
    }

    LOG(INFO) << "Looking for previous keyframe " << previous_keyframe_id << " for camera visualization";

    // Find previous keyframe
    auto previous_processed_kf = map_store.getUnprocessedKeyFrame(previous_keyframe_id);
    if (!previous_processed_kf) {
        previous_processed_kf = map_store.getProcessedNonOptimizedKeyFrame(previous_keyframe_id);
        if (!previous_processed_kf) {
            previous_processed_kf = map_store.getProcessedOptimizedKeyFrame(previous_keyframe_id);
            if (!previous_processed_kf) {
                previous_processed_kf = map_store.getKeyFrame(previous_keyframe_id);
            }
        }
    }

    // Add keyframes for camera visualization (most recent first)
    if (current_processed_kf) {
        camera_keyframes.push_back(current_processed_kf);
        LOG(INFO) << "Found current keyframe " << current_keyframe_id << " for camera visualization";
    } else {
        LOG(WARNING) << "Current keyframe " << current_keyframe_id << " not found in any queue";
    }

    if (previous_processed_kf) {
        camera_keyframes.push_back(previous_processed_kf);
        LOG(INFO) << "Found previous keyframe " << previous_keyframe_id << " for camera visualization";
    } else {
        LOG(WARNING) << "Previous keyframe " << previous_keyframe_id << " not found in any queue";
    }

    LOG(INFO) << "Total keyframes for camera visualization: " << camera_keyframes.size();

    auto getStaticTransform = [&](const std::string& source, const std::string& target,
                                  stf::TransformTree::TransformResult& transform) {
        try {
            LOG(INFO) << "DEBUG: Transform query - original: '" << source << "' -> '" << target
                      << "'";

            // First try without cleaning (original frame names)
            try {
                transform = transform_tree_->getTransform(source, target);
                LOG(INFO) << "DEBUG: Transform query SUCCEEDED (original names)";
                return true;
            } catch (const std::exception& e1) {
                LOG(INFO) << "DEBUG: Transform query failed with original names: " << e1.what();
            }

            // Then try with cleaned names (remove leading slashes)
            std::string clean_source = source;
            std::string clean_target = target;
            if (!clean_source.empty() && clean_source[0] == '/') {
                clean_source = clean_source.substr(1);
            }
            if (!clean_target.empty() && clean_target[0] == '/') {
                clean_target = clean_target.substr(1);
            }

            if (clean_source != source || clean_target != target) {
                LOG(INFO) << "DEBUG: Trying cleaned names: '" << clean_source << "' -> '"
                          << clean_target << "'";
                transform = transform_tree_->getTransform(clean_source, clean_target);
                LOG(INFO) << "DEBUG: Transform query SUCCEEDED (cleaned names)";
                return true;
            } else {
                throw std::runtime_error("Transform not found with either naming convention");
            }
        } catch (const std::exception& e) {
            LOG(ERROR) << "Failed to get static transform from " << source << " to " << target
                       << ": " << e.what();
            return false;
        }
        return true;
    };

    for (const auto& kf : camera_keyframes) {
        if (kf->hasColorImage() && kf->hasCameraInfo() && numCameras < NUM_CAMERAS_TO_VIZ) {
            const auto& image_data = kf->getColorImage();
            const auto& K = kf->getCameraInfo();

            LOG(INFO) << "DEBUG: Processing camera visualization for keyframe " << kf->id;
            LOG(INFO) << "DEBUG: Image size: " << image_data.width << "x" << image_data.height
                      << ", encoding: " << image_data.encoding;
            LOG(INFO) << "DEBUG: Camera info - frame: '" << K.frame_id << "', size: "
                      << K.width << "x" << K.height;
            LOG(INFO) << "DEBUG: Camera intrinsics - fx:" << K.k[0] << ", fy:" << K.k[4]
                      << ", cx:" << K.k[2] << ", cy:" << K.k[5];
            LOG(INFO) << "DEBUG: base_link_frame_id_ = '" << base_link_frame_id_ << "'";

            // Query transform from base_link to camera frame
            stf::TransformTree::TransformResult base_to_camera_result;
            if (getStaticTransform(base_link_frame_id_, K.frame_id, base_to_camera_result)) {
                auto camera = rerun::archetypes::Pinhole::from_focal_length_and_resolution(
                    {static_cast<float>(K.k[0]), static_cast<float>(K.k[4])},
                    {static_cast<float>(K.width), static_cast<float>(K.height)});

                // Get base_link to camera transform
                Eigen::Isometry3d base_to_camera_transform = base_to_camera_result.transform;
                core::types::Pose base_to_camera_pose;
                base_to_camera_pose.position = base_to_camera_transform.translation();
                base_to_camera_pose.orientation =
                    Eigen::Quaterniond(base_to_camera_transform.rotation());

                // Compose: camera_pose_in_reference = pose_of_base_link_in_reference *
                // base_link_to_camera
                core::types::Pose camera_pose_in_reference = kf->pose * base_to_camera_pose;

                if (camera_entity_paths_.size() <= numCameras) {
                    const std::string path =
                        "/" + reference_frame_id_ + "/camera" + std::to_string(numCameras);
                    camera_entity_paths_.push_back(path);
                }
                auto camera_path = camera_entity_paths_.at(numCameras);

                LOG(INFO) << "DEBUG: Publishing camera " << numCameras << " at path: " << camera_path;
                LOG(INFO) << "DEBUG: Camera pose - pos: [" << camera_pose_in_reference.position.transpose()
                          << "], quat: [" << camera_pose_in_reference.orientation.coeffs().transpose() << "]";

                addPose(camera_pose_in_reference, camera_path, current_timestamp_);
                addCamera(camera, camera_path, current_timestamp_);
                addImage(image_data.toCvMat(), camera_path, current_timestamp_);

                LOG(INFO) << "DEBUG: Successfully published camera " << numCameras
                          << " with image and pose for keyframe " << kf->id;
                numCameras++;
            } else {
                LOG(WARNING) << "Failed to get transform from " << base_link_frame_id_ << " to "
                             << K.frame_id << " for camera visualization";
            }
        } else {
            // Detailed logging about why camera visualization was skipped
            LOG(INFO) << "DEBUG: Skipping camera visualization for keyframe " << kf->id << ":";
            LOG(INFO) << "  - hasColorImage: " << (kf->hasColorImage() ? "YES" : "NO");
            LOG(INFO) << "  - hasCameraInfo: " << (kf->hasCameraInfo() ? "YES" : "NO");
            LOG(INFO) << "  - numCameras < limit: " << (numCameras < NUM_CAMERAS_TO_VIZ ? "YES" : "NO")
                      << " (" << numCameras << " < " << NUM_CAMERAS_TO_VIZ << ")";
        }
    }

    // ===== VISUALIZE MAP KEYPOINTS =====

    // Visualize map keypoints (magenta points)
    std::vector<rerun::datatypes::Vec3D> map_points;
    for (const auto& map_point : map_keypoints) {
        auto position = map_point.second.position;
        map_points.emplace_back(rerun::datatypes::Vec3D{static_cast<float>(position.x()),
                                                        static_cast<float>(position.y()),
                                                        static_cast<float>(position.z())});
    }

    if (!map_points.empty()) {
        rec_.log(
            "/" + reference_frame_id_ + "/map_points",
            rerun::Points3D({map_points}).with_colors({rerun::components::Color(255, 0, 255)}));  // Magenta
    }

    LOG(INFO) << "Three-queue visualization complete - Cameras: " << numCameras
              << ", Non-optimized KFs: " << non_optimized_points.size()
              << ", Optimized KFs: " << optimized_points.size()
              << ", Disk-synced KFs: " << disk_points.size()
              << ", Map points: " << map_points.size();
}

rerun::Transform3D RerunVisualizer::toRerunTransform(const core::types::Pose& pose) {
    return rerun::Transform3D(
        rerun::datatypes::Vec3D{static_cast<float>(pose.position.x()),
                                static_cast<float>(pose.position.y()),
                                static_cast<float>(pose.position.z())},
        rerun::datatypes::Quaternion{
            static_cast<float>(pose.orientation.x()), static_cast<float>(pose.orientation.y()),
            static_cast<float>(pose.orientation.z()), static_cast<float>(pose.orientation.w())});
}

rerun::Points3D RerunVisualizer::toRerunPoints(const core::types::PointCloud& cloud) {
    return toRerunPoints(cloud, core::types::Pose());
}

rerun::Points3D RerunVisualizer::toRerunPoints(const core::types::PointCloud& cloud,
                                               const core::types::Pose& pose) {
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> colors;

    points.reserve(cloud.points.size());
    if (!cloud.colors.empty()) {
        colors.reserve(cloud.colors.size());
    }

    // Create transform from pose if not identity
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    if (!pose.position.isZero() || !pose.orientation.coeffs().isZero()) {
        transform.translation() = pose.position;
        transform.linear() = pose.orientation.toRotationMatrix();
    }

    for (const auto& p : cloud.points) {
        Eigen::Vector3d transformed_point = transform * p;
        points.push_back(transformed_point.cast<float>());
    }

    for (const auto& c : cloud.colors) {
        colors.push_back(c.cast<float>());
    }

    return rerun::Points3D(points).with_colors(colors);
}

}  // namespace viz

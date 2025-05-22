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

void RerunVisualizer::visualizeFactorGraph(
    const core::graph::FactorGraph& graph,
    const std::map<uint32_t, core::types::Keypoint>& map_keypoints) {
    if (!is_connected_)
        return;
    return;

    auto keyframes = graph.getAllKeyFrames();

    auto latest_position = keyframes.back()->pose;
    // addPose(latest_position, "/world/path");

    rec_.set_time_sequence("max_keyframe_id", keyframes.back()->pose.timestamp);

    std::vector<rerun::datatypes::Vec3D> path_points;
    uint32_t numCameras = 0;
    uint32_t numClouds = 0;
    uint32_t processedKfs = 0;
    uint32_t toPublish = keyframes.size() - numCameras;

    LOG(INFO) << "Current number of keyframes: " << keyframes.size();

    std::string path_points_frame = "";
    for (const auto& kf : keyframes) {
        processedKfs++;

        auto getStaticTransform = [&](const std::string& source, const std::string& target,
                                      stf::TransformTree::TransformResult& transform) {
            try {
                LOG(INFO) << "DEBUG: Transform query - original: '" << source << "' -> '" << target
                          << "'";

                // First try without cleaning (original frame names)
                try {
                    transform = transform_tree_->getTransform(source, target);
                    transform.transform = transform.transform.inverse();
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
                    transform.transform = transform.transform.inverse();
                    LOG(INFO) << "DEBUG: Transform query SUCCEEDED (cleaned names)";
                    return true;
                } else {
                    // No difference after cleaning, so don't try again
                    throw std::runtime_error("Transform not found with either naming convention");
                }
            } catch (const std::exception& e) {
                LOG(ERROR) << "Failed to get static transform from " << source << " to " << target
                           << ": " << e.what();
                return false;
            }
            return true;
        };

        // For trajectory: use pose directly if it's already in reference frame
        core::types::Pose trajectory_pose = kf->pose;
        if (kf->pose.frame_id.empty() || kf->pose.frame_id == reference_frame_id_) {
            // Pose is already in reference frame, use directly
            trajectory_pose = kf->pose;
            if (path_points_frame.empty()) {
                path_points_frame = "/" + reference_frame_id_ + "/trajectory";
            }
        } else {
            // Pose is in different frame - this shouldn't happen with our new implementation
            LOG(WARNING) << "Keyframe pose is in frame " << kf->pose.frame_id << " but expected "
                         << reference_frame_id_ << " for keyframe " << kf->id;
            trajectory_pose = kf->pose;  // Use as-is and hope for the best
        }

        auto position = trajectory_pose.position;
        path_points.emplace_back(rerun::datatypes::Vec3D{static_cast<float>(position.x()),
                                                         static_cast<float>(position.y()),
                                                         static_cast<float>(position.z())});

        if (kf->hasColorImage() && kf->hasCameraInfo() && numCameras < NUM_CAMERAS_TO_VIZ) {
            if (keyframes.size() > NUM_CAMERAS_TO_VIZ &&
                processedKfs < (keyframes.size() - NUM_CAMERAS_TO_VIZ)) {
                continue;
            }
            const auto& image_data = kf->getColorImage();
            const auto& K = kf->getCameraInfo();

            LOG(INFO) << "DEBUG: Attempting camera visualization for keyframe " << kf->id;
            LOG(INFO) << "DEBUG: base_link_frame_id_ = '" << base_link_frame_id_ << "'";
            LOG(INFO) << "DEBUG: camera frame from camera_info = '" << K.frame_id << "'";

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
                core::types::Pose camera_pose_in_reference = trajectory_pose * base_to_camera_pose;

                if (camera_entity_paths_.size() <= numCameras) {
                    const std::string path =
                        "/" + reference_frame_id_ + "/camera" + std::to_string(numCameras);
                    camera_entity_paths_.push_back(path);
                }
                auto camera_path = camera_entity_paths_.at(numCameras);
                addPose(camera_pose_in_reference, camera_path, current_timestamp_);
                addCamera(camera, camera_path, current_timestamp_);
                addImage(image_data.toCvMat(), camera_path, current_timestamp_);

                // std::string matches_path("/data/robot/bags/house11/orb/inlier_matches_" +
                //                          std::to_string(kf->id) + ".png");
                // if (std::filesystem::exists(matches_path)) {
                //     cv::Mat matches_img = cv::imread(matches_path);
                //     if (!matches_img.empty()) {
                //         addImage(matches_img, camera_path, current_timestamp_);
                //     }
                // }

                numCameras++;
            } else {
                LOG(WARNING) << "Failed to get transform from " << base_link_frame_id_ << " to "
                             << K.frame_id << " for camera visualization";
            }
        }

        // if (kf->hasPointCloud()) {
        //     const auto& cloud = kf->getPointCloud();
        //     const std::string cloud_path = "/world/odom/cloud_" + std::to_string(kf->id);
        //     addPointCloud(cloud, cloud_path, current_timestamp_, kf->pose);
        //     numClouds++;
        // }
    }

    std::vector<rerun::datatypes::Vec3D> map_points;
    for (const auto& map_point : map_keypoints) {
        auto position = map_point.second.position;
        map_points.emplace_back(rerun::datatypes::Vec3D{static_cast<float>(position.x()),
                                                        static_cast<float>(position.y()),
                                                        static_cast<float>(position.z())});
    }
    rec_.log(reference_frame_id_,
             rerun::Points3D({map_points}).with_colors({rerun::components::Color(255, 0, 255)}));

    LOG(INFO) << "Num cameras: " << numCameras << " Num clouds: " << numClouds
              << " Num path points: " << path_points.size();

    // Visualize trajectory as connected points (red dots)
    if (!path_points_frame.empty()) {
        rec_.log(path_points_frame,
                 rerun::Points3D({path_points}).with_colors({rerun::components::Color(255, 0, 0)}));

        // Visualize trajectory as connected line (blue line connecting keyframes)
        if (path_points.size() > 1) {
            rec_.log(path_points_frame, rerun::LineStrips3D({path_points})
                                            .with_colors({rerun::components::Color(0, 100, 255)}));
        }
    }
}

void RerunVisualizer::visualizeFromStorage(
    const core::storage::MapStore& map_store,
    const std::map<uint32_t, core::types::Keypoint>& map_keypoints, uint64_t current_keyframe_id,
    uint64_t previous_keyframe_id, size_t trajectory_keyframe_count) {
    if (!is_connected_)
        return;

    LOG(INFO) << "Visualizing from storage - current KF: " << current_keyframe_id
              << ", previous KF: " << previous_keyframe_id
              << ", trajectory count: " << trajectory_keyframe_count;

    // Get all keyframes for trajectory (last N keyframes)
    auto all_keyframes = map_store.getAllKeyFrames();
    if (all_keyframes.empty()) {
        LOG(WARNING) << "No keyframes found in storage for visualization";
        return;
    }

    // Sort keyframes by ID to get the latest ones
    std::sort(all_keyframes.begin(), all_keyframes.end(),
              [](const auto& a, const auto& b) { return a->id < b->id; });

    // Get last N keyframes for trajectory
    std::vector<std::shared_ptr<core::types::KeyFrame>> trajectory_keyframes;
    size_t start_idx = all_keyframes.size() > trajectory_keyframe_count
                           ? all_keyframes.size() - trajectory_keyframe_count
                           : 0;
    // for (size_t i = start_idx; i < all_keyframes.size(); ++i) {
    for (size_t i = 0; i < all_keyframes.size(); ++i) {
        trajectory_keyframes.push_back(all_keyframes[i]);
    }

    // Get current and previous keyframes for detailed visualization
    auto current_kf = map_store.getKeyFrame(current_keyframe_id);
    auto previous_kf = map_store.getKeyFrame(previous_keyframe_id);

    // Set timeline
    if (current_kf) {
        rec_.set_time_sequence("max_keyframe_id", current_kf->pose.timestamp);
    }

    LOG(INFO) << "Using " << trajectory_keyframes.size()
              << " keyframes for trajectory visualization";

    // Visualize trajectory path
    std::vector<rerun::datatypes::Vec3D> path_points;
    std::string path_points_frame = "/" + reference_frame_id_ + "/trajectory";

    for (const auto& kf : trajectory_keyframes) {
        auto position = kf->pose.position;
        path_points.emplace_back(rerun::datatypes::Vec3D{static_cast<float>(position.x()),
                                                         static_cast<float>(position.y()),
                                                         static_cast<float>(position.z())});
    }

    // Visualize camera images for current and previous keyframes only
    uint32_t numCameras = 0;
    std::vector<std::shared_ptr<core::types::KeyFrame>> camera_keyframes;
    if (previous_kf)
        camera_keyframes.push_back(previous_kf);
    if (current_kf)
        camera_keyframes.push_back(current_kf);

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

            LOG(INFO) << "DEBUG: Attempting camera visualization for keyframe " << kf->id;
            LOG(INFO) << "DEBUG: base_link_frame_id_ = '" << base_link_frame_id_ << "'";
            LOG(INFO) << "DEBUG: camera frame from camera_info = '" << K.frame_id << "'";

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
                addPose(camera_pose_in_reference, camera_path, current_timestamp_);
                addCamera(camera, camera_path, current_timestamp_);
                addImage(image_data.toCvMat(), camera_path, current_timestamp_);

                numCameras++;
            } else {
                LOG(WARNING) << "Failed to get transform from " << base_link_frame_id_ << " to "
                             << K.frame_id << " for camera visualization";
            }
        }
    }

    // Visualize map keypoints (all of them)
    std::vector<rerun::datatypes::Vec3D> map_points;
    for (const auto& map_point : map_keypoints) {
        auto position = map_point.second.position;
        map_points.emplace_back(rerun::datatypes::Vec3D{static_cast<float>(position.x()),
                                                        static_cast<float>(position.y()),
                                                        static_cast<float>(position.z())});
    }

    if (!map_points.empty()) {
        rec_.log(
            reference_frame_id_,
            rerun::Points3D({map_points}).with_colors({rerun::components::Color(255, 0, 255)}));
    }

    LOG(INFO) << "Visualization stats - Cameras: " << numCameras
              << ", Trajectory points: " << path_points.size()
              << ", Map points: " << map_points.size();

    // Visualize trajectory as connected points (red dots)
    if (!path_points.empty()) {
        rec_.log(path_points_frame,
                 rerun::Points3D({path_points}).with_colors({rerun::components::Color(255, 0, 0)}));

        // Visualize trajectory as connected line (blue line connecting keyframes)
        if (path_points.size() > 1) {
            rec_.log(path_points_frame, rerun::LineStrips3D({path_points})
                                            .with_colors({rerun::components::Color(0, 100, 255)}));
        }
    }
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

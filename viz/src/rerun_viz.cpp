#include "viz/rerun_viz.hpp"
#include "viz/rerun_eigen_adapters.hpp"

#include "logging/logging.hpp"

#include "core/types/keyframe.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <opencv2/core.hpp>
#include <rerun.hpp>
#include <rerun/archetypes/image.hpp>
#include <rerun/archetypes/transform3d.hpp>
#include <rerun/datatypes/quaternion.hpp>
#include <rerun/datatypes/vec3d.hpp>
#include <rerun/recording_stream.hpp>
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

void RerunVisualizer::visualizeFactorGraph(const core::graph::FactorGraph& graph) {
    if (!is_connected_)
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

    for (const auto& kf : keyframes) {
        processedKfs++;
        auto position = kf->pose.position;
        path_points.emplace_back(rerun::datatypes::Vec3D{static_cast<float>(position.x()),
                                                         static_cast<float>(position.y()),
                                                         static_cast<float>(position.z())});

        auto getStaticTransform = [&](const std::string& source, const std::string& target,
                                      stf::TransformTree::TransformResult& transform) {
            try {
                transform = transform_tree_->getTransform(source, target);
            } catch (const std::exception& e) {
                LOG(ERROR) << "Failed to get static transform: " << e.what();
                return false;
            }
            return true;
        };

        if (kf->hasColorImage() && kf->hasCameraInfo() && numCameras < NUM_CAMERAS_TO_VIZ) {
            if (keyframes.size() > NUM_CAMERAS_TO_VIZ &&
                processedKfs < (keyframes.size() - NUM_CAMERAS_TO_VIZ)) {
                continue;
            }
            const auto& image_data = kf->getColorImage();
            const auto& K = kf->getCameraInfo();
            stf::TransformTree::TransformResult transformResult;
            if (getStaticTransform("base_link", K.frame_id, transformResult)) {
                auto camera = rerun::archetypes::Pinhole::from_focal_length_and_resolution(
                    {static_cast<float>(K.k[0]), static_cast<float>(K.k[4])},
                    {static_cast<float>(K.width), static_cast<float>(K.height)});

                Eigen::Isometry3d transform = transformResult.transform;
                core::types::Pose camera_pose;
                camera_pose.position = transform.translation();
                camera_pose.orientation = Eigen::Quaterniond(transform.rotation());

                camera_pose = kf->pose * camera_pose;

                if (camera_entity_paths_.size() <= numCameras) {
                    const std::string path = "/world/odom/camera_" + std::to_string(numCameras);
                    camera_entity_paths_.push_back(path);
                }
                auto camera_path = camera_entity_paths_.at(numCameras);
                addPose(camera_pose, camera_path, current_timestamp_);
                addCamera(camera, camera_path, current_timestamp_);
                addImage(image_data.toCvMat(), camera_path, current_timestamp_);

                // Log the tf b/w base_link and camera frame
                auto position = kf->pose.position.cast<float>();
                auto camera_position = camera_pose.position.cast<float>();
                std::vector<rerun::datatypes::Vec3D> tf_points{
                    {position.x(), position.y(), position.z()},
                    {camera_position.x(), camera_position.y(), camera_position.z()}};
                rec_.log(camera_path + "/tf", rerun::LineStrips3D({tf_points})
                                                  .with_colors({rerun::components::Color(
                                                      0, 255, 0)}));  // Green color for visibility

                numCameras++;
            }
        }

        if (kf->hasPointCloud()) {
            const auto& cloud = kf->getPointCloud();
            const std::string cloud_path = "/world/odom/cloud_" + std::to_string(kf->id);
            addPointCloud(cloud, cloud_path, current_timestamp_, kf->pose);
            numClouds++;
        }
    }

    LOG(INFO) << "Num cameras: " << numCameras << " Num clouds: " << numClouds
              << " Num path points: " << path_points.size();

    rec_.log("/world/odom/trajectory",
             rerun::Points3D({path_points}).with_colors({rerun::components::Color(255, 0, 0)}));
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

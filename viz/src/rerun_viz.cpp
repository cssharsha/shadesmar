#include "viz/rerun_viz.hpp"
#include "viz/rerun_eigen_adapters.hpp"

#include "core/types/keyframe.hpp"

#include <Eigen/Core>
#include <chrono>
#include <thread>

namespace viz {

RerunVisualizer::RerunVisualizer(const std::string& name, const std::string& host, uint16_t port)
    : name_(name), host_(host), port_(port), rec_(name) {}

bool RerunVisualizer::initialize(bool save_to_file) {
    try {
        if (save_to_file) {
            // Save to a recording file instead of TCP
            std::string recording_path =
                "/mnt/remote-storage/" + name_ + ".rrd";  // .rrd is the Rerun recording format
            auto result = rec_.save(recording_path);

            if (!rec_.is_enabled()) {
                std::cerr << "Failed to create recording file" << std::endl;
                return false;
            }
        } else {
            // Connect to the Rerun server
            rec_.connect_tcp().exit_on_failure();
        }

        // // Send test data
        // rec_.set_time_sequence("time", 0);
        // rec_.log("connection_test", rerun::TextLog("Recording started"));

        is_connected_ = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to create recording: " << e.what() << std::endl;
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

    if (timestamp > 0) {
        setTimestamp(timestamp);
    }

    rec_.log(entity_path, toRerunTransform(pose));

    if (!entity_path.empty()) {
        rec_.log(entity_path + "/label", rerun::TextLog(entity_path));
    }
}

void RerunVisualizer::addPointCloud(const core::types::PointCloud& cloud,
                                    const core::types::Pose& pose) {
    if (!is_connected_)
        return;

    auto points = toRerunPoints(cloud, pose);

    rec_.log("point_clouds", points);
}

void RerunVisualizer::addImage(const cv::Mat& image, const std::string& entity_path,
                               double timestamp) {
    if (!is_connected_)
        return;

    if (timestamp > 0) {
        setTimestamp(timestamp);
    }

    if (image.channels() == 1) {
        rec_.log(entity_path, rerun::Image());
    } else {
        rec_.log(entity_path, rerun::Image::from_rgb24(rerun::Collection<uint8_t>(image),
                                                       rerun::WidthHeight(image.cols, image.rows)));
    }
}

void RerunVisualizer::visualizeFactorGraph(const core::graph::FactorGraph& graph) {
    if (!is_connected_)
        return;

    // Clear previous data (explicitly use the boolean constructor)
    rec_.log("clear", rerun::Clear(false));  // false means non-recursive clear

    // Get all keyframes and visualize them
    auto keyframes = graph.getAllKeyFrames();
    for (const auto& kf : keyframes) {
        // Add timestamp to each pose
        rec_.set_time_sequence(
            "time", static_cast<int64_t>(current_timestamp_ * 1000));  // Convert to milliseconds
        addPose(kf->pose, "keyframe_" + std::to_string(kf->id));
        current_timestamp_ += 0.1;
    }

    // Visualize factors as connections
    auto factors = graph.getFactors();
    for (const auto& factor : factors) {
        if (factor.type == core::proto::FactorType::ODOMETRY ||
            factor.type == core::proto::FactorType::LOOP_CLOSURE) {
            std::vector<Eigen::Vector3d> line_points;
            for (const auto& node_id : factor.connected_nodes) {
                auto kf = graph.getKeyFrame(node_id);
                if (kf) {
                    line_points.push_back(kf->pose.position);
                }
            }

            if (line_points.size() >= 2) {
                rec_.set_time_sequence("time", static_cast<int64_t>(current_timestamp_ * 1000));
                rec_.log("factors", rerun::LineStrips3D({line_points}));
            }
        }
    }

    // Force an update
    update();
}

void RerunVisualizer::visualizeKeyFrame(const core::types::KeyFrame::ConstPtr& keyframe) {
    if (!is_connected_)
        return;

    // Visualize pose
    addPose(keyframe->pose, "keyframe_" + std::to_string(keyframe->id));

    // Visualize data (either point cloud or image)
    if (keyframe->hasPointCloud()) {
        addPointCloud(keyframe->getPointCloud(), keyframe->pose);
    } else if (keyframe->color_data) {
        addImage(keyframe->color_data.value().toCvMat(),
                 "keyframe_" + std::to_string(keyframe->id));
    }

    // Flush after publishing
    update();
}

void RerunVisualizer::clear() {
    if (!is_connected_)
        return;
}

void RerunVisualizer::update() {
    if (!is_connected_)
        return;

    try {
        // Send a marker to ensure data is processed
        rec_.set_time_sequence("time", static_cast<int64_t>(current_timestamp_ * 1000));
        rec_.log("update", rerun::TextLog("update"));

        // Small delay to ensure processing
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } catch (const std::exception& e) {
        std::cerr << "Failed to update: " << e.what() << std::endl;
        is_connected_ = false;
    }
}

void RerunVisualizer::setTimestamp(double timestamp) {
    current_timestamp_ = timestamp;
}

rerun::Transform3D RerunVisualizer::toRerunTransform(const core::types::Pose& pose) {
    return rerun::Transform3D::from_translation_rotation(
        {static_cast<float>(pose.position.x()), static_cast<float>(pose.position.y()),
         static_cast<float>(pose.position.z())},
        rerun::Rotation3D(rerun::datatypes::Quaternion{
            static_cast<float>(pose.orientation.x()), static_cast<float>(pose.orientation.y()),
            static_cast<float>(pose.orientation.z()), static_cast<float>(pose.orientation.w())}));
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

void RerunVisualizer::visualizeOdometryTrajectory(const std::vector<Eigen::Vector3d>& positions) {
    if (!is_connected_)
        return;

    rec_.log("odometry_trajectory", rerun::LineStrips3D({positions}));
    update();
}

void RerunVisualizer::addCamera(const rerun::archetypes::Pinhole& camera,
                                const std::string& entity_path, double timestamp) {
    if (!is_connected_)
        return;

    if (timestamp > 0) {
        setTimestamp(timestamp);
    }

    rec_.log(entity_path, camera);
}

}  // namespace viz

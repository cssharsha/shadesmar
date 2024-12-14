#include "viz/rerun_viz.hpp"
#include "viz/rerun_eigen_adapters.hpp"

#include "core/types/keyframe.hpp"

#include <Eigen/Core>

namespace viz {

RerunVisualizer::RerunVisualizer(const std::string& name) : name_(name), rec_(name) {}

bool RerunVisualizer::initialize() {
    try {
        rec_.spawn().exit_on_failure();
        is_connected_ = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize Rerun: " << e.what() << std::endl;
        return false;
    }
}

bool RerunVisualizer::isConnected() const {
    return is_connected_;
}

void RerunVisualizer::disconnect() {
    is_connected_ = false;
}

void RerunVisualizer::addPose(const core::types::Pose& pose, const std::string& label) {
    if (!is_connected_)
        return;

    rec_.log("poses", toRerunTransform(pose));

    if (!label.empty()) {
        rec_.log("poses/labels", rerun::TextLog(label));
    }
}

void RerunVisualizer::addPointCloud(const core::types::PointCloud& cloud,
                                    const core::types::Pose& pose) {
    if (!is_connected_)
        return;

    auto points = toRerunPoints(cloud, pose);

    rec_.log("point_clouds", points);
}

void RerunVisualizer::addImage(const cv::Mat& image, const std::string& name) {
    if (!is_connected_)
        return;

    // rerun::Image rerun_image;
    if (image.channels() == 1) {
        // rerun_image = rerun::DepthImage(
        //     rerun::Collection<uint16_t>(image),
        //     rerun::WidthHeight(static_cast<size_t>(image.cols),
        //     static_cast<size_t>(image.rows)));
        rec_.log("images/" + name, rerun::Image());
    } else {
        rec_.log("images/" + name,
                 rerun::Image::from_rgb24(rerun::Collection<uint8_t>(image),
                                          rerun::WidthHeight(image.cols, image.rows)));
    }
}

void RerunVisualizer::visualizeFactorGraph(const core::graph::FactorGraph& graph) {
    if (!is_connected_)
        return;

    // Get all keyframes and visualize them
    auto keyframes = graph.getAllKeyFrames();
    for (const auto& kf : keyframes) {
        addPose(kf->pose, "keyframe_" + std::to_string(kf->id));
    }

    // Visualize factors as connections
    auto factors = graph.getFactors();
    for (const auto& factor : factors) {
        if (factor.type == core::proto::FactorType::ODOMETRY ||
            factor.type == core::proto::FactorType::LOOP_CLOSURE) {
            // Draw lines between connected poses
            std::vector<Eigen::Vector3d> line_points;
            for (const auto& node_id : factor.connected_nodes) {
                auto kf = graph.getKeyFrame(node_id);
                if (kf) {
                    line_points.push_back(kf->pose.position);
                }
            }

            if (line_points.size() >= 2) {
                rec_.log("factors", rerun::LineStrips3D({line_points}));
            }
        }
    }
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
}

void RerunVisualizer::clear() {
    if (!is_connected_)
        return;
}

void RerunVisualizer::update() {
    if (!is_connected_)
        return;
}

void RerunVisualizer::setTimestamp(double timestamp) {
    current_timestamp_ = timestamp;
}

rerun::Transform3D RerunVisualizer::toRerunTransform(const core::types::Pose& pose) {
    return rerun::Transform3D::from_translation_rotation(
        {static_cast<float>(pose.position.x()), static_cast<float>(pose.position.y()),
         static_cast<float>(pose.position.z())},
        rerun::Rotation3D(rerun::datatypes::Quaternion{
            static_cast<float>(pose.orientation.w()), static_cast<float>(pose.orientation.x()),
            static_cast<float>(pose.orientation.y()), static_cast<float>(pose.orientation.z())}));
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

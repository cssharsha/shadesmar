#pragma once

#include <cstdint>
#include <rerun.hpp>
#include <rerun/archetypes/pinhole.hpp>
#include <stf/transform_tree.hpp>
#include <memory>
#include <string>
#include <vector>

#include "viz/interface.hpp"
#include "core/storage/map_store.hpp"
#include "core/types/pose.hpp"

namespace viz {

static constexpr uint32_t NUM_CAMERAS_TO_VIZ = 2;

class RerunVisualizer : public VisualizerInterface {
public:
    RerunVisualizer(const std::string& name = "shadesmar", const std::string& host = "localhost",
                    uint16_t port = 9999);

    bool initialize() override {
        return initialize(false);
    }
    bool initialize(bool save_to_file);
    bool isConnected() const override;
    void disconnect() override;

    void addPose(const core::types::Pose& pose, const std::string& label = "") override {}
    void addPointCloud(const core::types::PointCloud& cloud,
                       const core::types::Pose& pose = core::types::Pose()) override {}
    void addImage(const cv::Mat& image, const std::string& name = "image") override {}

    void visualizeFactorGraph(
        const core::graph::FactorGraph& graph,
        const std::map<uint32_t, core::types::Keypoint>& map_keypoints) override;

    void visualizeKeyFrame(const core::types::KeyFrame::ConstPtr& keyframe) override {}
    void clear() override {}
    void update() override {}
    void setTimestamp(double timestamp) override {}

    void addPose(const core::types::Pose& pose, const std::string& entity_path, double timestamp);
    void addImage(const cv::Mat& image, const std::string& entity_path, double timestamp);
    void addPointCloud(const core::types::PointCloud& cloud, const std::string& entity_path,
                       double timestamp, const core::types::Pose& transform);
    void addCamera(const rerun::archetypes::Pinhole& camera, const std::string& entity_path,
                   double timestamp = 0);

    void setTransformTree(std::shared_ptr<stf::TransformTree> transform_tree) {
        transform_tree_ = transform_tree;
    }

    void setFrameIds(const std::string& reference_frame_id, const std::string& camera_frame_id) {
        reference_frame_id_ = reference_frame_id;
        camera_frame_id_ = camera_frame_id;
    }

    void setFrameIds(const std::string& reference_frame_id, const std::string& base_link_frame_id, const std::string& camera_frame_id) {
        reference_frame_id_ = reference_frame_id;
        base_link_frame_id_ = base_link_frame_id;
        camera_frame_id_ = camera_frame_id;
    }

    // New method to visualize from storage with last N keyframes for trajectory
    void visualizeFromStorage(
        const core::storage::MapStore& map_store,
        const std::map<uint32_t, core::types::Keypoint>& map_keypoints,
        uint64_t current_keyframe_id,
        uint64_t previous_keyframe_id,
        size_t trajectory_keyframe_count = 5);

private:
    std::string name_;
    std::string host_;
    uint16_t port_;
    rerun::RecordingStream rec_;
    bool is_connected_ = false;
    double current_timestamp_ = 0;
    std::shared_ptr<stf::TransformTree> transform_tree_;

    std::vector<std::string> camera_entity_paths_;

    // Configurable frame IDs
    std::string reference_frame_id_ = "odom";  // Default to standard odometry frame
    std::string base_link_frame_id_ = "base_link";  // Default robot body frame
    std::string camera_frame_id_ = "camera_color_optical_frame";

    // Helper functions
    rerun::Transform3D toRerunTransform(const core::types::Pose& pose);
    rerun::Points3D toRerunPoints(const core::types::PointCloud& cloud);
    rerun::Points3D toRerunPoints(const core::types::PointCloud& cloud,
                                  const core::types::Pose& pose);
};
}  // namespace viz
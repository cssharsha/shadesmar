#pragma once

#include <cstdint>
#include <map>
#include <vector>
#include "2d/orb_tracker.hpp"
#include "core/graph/factor_graph.hpp"
#include "core/storage/map_store.hpp"
#include "core/types/keyframe.hpp"
#include "message_synchronizer/message_synchronizer.hpp"
#include "stf/transform_tree.hpp"

#include "logging/logging.hpp"
namespace core {
namespace graph {

struct GraphCallbacks {
    std::function<void()> on_graph_updated;
};

class GraphAdapter {
public:
    GraphAdapter(FactorGraph& graph, storage::MapStore& store);
    void handleOdometryInput(const types::Pose& pose, double timestamp);
    void handleImageInput(const types::Image& image, double timestamp);
    void handleCameraInfo(const types::CameraInfo& camera_info, double timestamp);
    void handleLoopClosure(uint64_t from_id, uint64_t to_id, const types::Pose& relative_pose);
    void setKeyframeDistanceThreshold(double threshold);
    void setCallbacks(const GraphCallbacks& callbacks);
    void handlePointCloudInput(const types::PointCloud& cloud, double timestamp);
    void maybeDumpGraph(bool force = false);
    std::shared_ptr<types::KeyFrame> createKeyframe(
        const types::Pose& pose, const std::optional<types::Image>& image = std::nullopt,
        const std::optional<types::CameraInfo>& camera_info = std::nullopt,
        const std::optional<types::PointCloud>& cloud = std::nullopt);

    void setTransformTree(std::shared_ptr<stf::TransformTree> transform_tree) {
        transform_tree_ = transform_tree;
        orb_tracker_.setTransformTree(transform_tree);
    }

    auto& getMapPoints() const {
        return map_keypoints_;
    }

private:
    FactorGraph& graph_;
    storage::MapStore& store_;
    // utils::MessageSynchronizer<types::Image, types::CameraInfo, types::PointCloud> synchronizer_;
    utils::MessageSynchronizer<types::Image, types::CameraInfo> synchronizer_;

    tracking::image::OrbTracker orb_tracker_;

    uint64_t current_keyframe_id_ = 0;
    std::vector<uint64_t> keyframe_ids_with_images_;
    uint64_t last_keyframe_for_orb_ = 0;
    types::Pose last_keyframe_pose_;
    double keyframe_distance_threshold_ = 0.05;

    void addOdometryFactor(uint64_t from_id, uint64_t to_id, const types::Pose& relative_pose);
    GraphCallbacks callbacks_;

    void addKeyframeToGraph(const std::shared_ptr<types::KeyFrame>& keyframe);
    double calculateDistance(const types::Pose& relative_pose);

    double cumulative_distance_;
    double next_dump_distance_;
    size_t odometry_count_ = 0;

    double total_keyframe_distance_ = 0.0;  // Tracks total distance between keyframes
    std::shared_ptr<stf::TransformTree> transform_tree_;

    std::map<uint32_t, core::types::Keypoint> map_keypoints_;
};

}  // namespace graph
}  // namespace core

#pragma once

#include "core/graph/factor_graph.hpp"
#include "core/storage/map_store.hpp"
#include "core/types/keyframe.hpp"
#include "message_synchronizer/message_synchronizer.hpp"

#include "logging/logging.hpp"
namespace core {
namespace graph {

struct GraphCallbacks {
    std::function<void()> on_graph_updated;
};

class GraphAdapter {
public:
    GraphAdapter(FactorGraph& graph, storage::MapStore& store)
        : graph_(graph),
          store_(store),
          synchronizer_(
              [this](const types::Pose& pose, const types::Image& image,
                     const types::CameraInfo& camera_info) {
                  createKeyframe(pose, image, camera_info);
              },
              [this](const types::Pose& pose) { createKeyframe(pose); }, 0.05, 1.0) {}

    // Modify input handlers to use synchronizer
    void handleOdometryInput(const types::Pose& pose, double timestamp) {
        synchronizer_.addPoseMessage(pose, timestamp);
    }

    void handleImageInput(const types::Image& image, double timestamp) {
        synchronizer_.addMessage(image, timestamp);
    }

    void handleCameraInfo(const types::CameraInfo& camera_info, double timestamp) {
        synchronizer_.addMessage(camera_info, timestamp);
    }

    void handleLoopClosure(uint64_t from_id, uint64_t to_id, const types::Pose& relative_pose);

    void setKeyframeDistanceThreshold(double threshold) {
        keyframe_distance_threshold_ = threshold;
        synchronizer_.setDistanceThreshold(threshold);
    }

    void setCallbacks(const GraphCallbacks& callbacks) {
        callbacks_ = callbacks;
    }

    void createKeyframe(const types::Pose& pose) {
        LOG(INFO) << "Creating keyframe with pose: " << pose.position.transpose();
        auto keyframe = std::make_shared<types::KeyFrame>();
        keyframe->id = ++current_keyframe_id_;
        keyframe->pose = pose;

        addKeyframeToGraph(keyframe);
    }

    void createKeyframe(const types::Pose& pose, const types::Image& image,
                        const types::CameraInfo& camera_info) {
        // LOG(INFO) << "Creating keyframe with pose: " << pose.position.transpose();
        auto keyframe = std::make_shared<types::KeyFrame>();
        keyframe->id = ++current_keyframe_id_;
        keyframe->pose = pose;

        if (!image.data.empty()) {
            keyframe->color_data = image;
            keyframe->camera_info = camera_info;
        }

        addKeyframeToGraph(keyframe);
    }

private:
    FactorGraph& graph_;
    storage::MapStore& store_;
    utils::MessageSynchronizer<types::Image, types::CameraInfo> synchronizer_;

    uint64_t current_keyframe_id_ = 0;
    types::Pose last_keyframe_pose_;
    double keyframe_distance_threshold_ = 0.05;

    void addOdometryFactor(uint64_t from_id, uint64_t to_id, const types::Pose& relative_pose);
    GraphCallbacks callbacks_;

    void addKeyframeToGraph(const std::shared_ptr<types::KeyFrame>& keyframe) {
        graph_.addKeyFrame(keyframe);
        store_.addKeyFrame(keyframe);

        if (current_keyframe_id_ > 1) {
            addOdometryFactor(current_keyframe_id_ - 1, current_keyframe_id_,
                              last_keyframe_pose_.inverse() * keyframe->pose);
        }

        last_keyframe_pose_ = keyframe->pose;

        if (callbacks_.on_graph_updated) {
            callbacks_.on_graph_updated();
        }
    }

    void maybeDumpGraph();

    double cumulative_distance_;
    double next_dump_distance_;
};

}  // namespace graph
}  // namespace core

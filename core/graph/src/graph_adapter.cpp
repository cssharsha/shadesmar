#include "core/graph/graph_adapter.hpp"
#include "core/graph/util.hpp"
#include "logging/logging.hpp"

namespace core {
namespace graph {

GraphAdapter::GraphAdapter(FactorGraph& graph, storage::MapStore& store)
    : graph_(graph),
      store_(store),
      synchronizer_(
          [this](const types::Pose& pose, const std::optional<types::Image>& image,
                 const std::optional<types::CameraInfo>& camera_info) {
              createKeyframe(pose, image, camera_info);
          },
          0.05, 1.0) {}

// Modify input handlers to use synchronizer
void GraphAdapter::handleOdometryInput(const types::Pose& pose, double timestamp) {
    synchronizer_.addPoseMessage(pose, timestamp);
}

void GraphAdapter::handleImageInput(const types::Image& image, double timestamp) {
    synchronizer_.addMessage(image, timestamp);
}

void GraphAdapter::handleCameraInfo(const types::CameraInfo& camera_info, double timestamp) {
    synchronizer_.addMessage(camera_info, timestamp);
}

void GraphAdapter::setKeyframeDistanceThreshold(double threshold) {
    keyframe_distance_threshold_ = threshold;
    synchronizer_.setDistanceThreshold(threshold);
}

void GraphAdapter::setCallbacks(const GraphCallbacks& callbacks) {
    callbacks_ = callbacks;
}

void GraphAdapter::createKeyframe(const types::Pose& pose, const std::optional<types::Image>& image,
                                  const std::optional<types::CameraInfo>& camera_info) {
    auto keyframe = std::make_shared<types::KeyFrame>();
    keyframe->id = ++current_keyframe_id_;
    keyframe->pose = pose;

    if (image.has_value() && camera_info.has_value() && !image->data.empty()) {
        keyframe->color_data = *image;
        keyframe->camera_info = *camera_info;
    }

    addKeyframeToGraph(keyframe);
}

double GraphAdapter::calculateDistance(const types::Pose& relative_pose) {
    return relative_pose.position.norm();
}

void GraphAdapter::handleLoopClosure(uint64_t from_id, uint64_t to_id,
                                     const types::Pose& relative_pose) {
    types::Factor loop_factor;
    loop_factor.type = proto::FactorType::LOOP_CLOSURE;
    loop_factor.connected_nodes = {from_id, to_id};
    loop_factor.measurement.emplace<0>(relative_pose);
    loop_factor.information = Eigen::Matrix<double, 6, 6>::Identity();

    graph_.addFactor(loop_factor);
    store_.addFactor(loop_factor);

    // Loop closures don't count towards cumulative distance
    maybeDumpGraph();
}

void GraphAdapter::addOdometryFactor(uint64_t from_id, uint64_t to_id,
                                     const types::Pose& relative_pose) {
    types::Factor odom_factor;
    odom_factor.type = proto::FactorType::ODOMETRY;
    odom_factor.connected_nodes = {from_id, to_id};
    odom_factor.measurement.emplace<0>(relative_pose);
    odom_factor.information = Eigen::Matrix<double, 6, 6>::Identity();

    graph_.addFactor(odom_factor);
    store_.addFactor(odom_factor);

    // Update cumulative distance and maybe dump graph
    cumulative_distance_ += calculateDistance(relative_pose);
    maybeDumpGraph();
}

void GraphAdapter::maybeDumpGraph() {
    constexpr double DUMP_INTERVAL = 1.0;  // meters

    if (cumulative_distance_ >= next_dump_distance_) {
        LOG(INFO) << "Dumping factor graph at distance " << cumulative_distance_ << "m";

        // Create filename with distance
        std::string filename = "/mnt/remote-storage/factor_graph_" +
                               std::to_string(static_cast<int>(cumulative_distance_)) + "m.vtk";

        util::dumpFactorGraph(graph_, filename);
        next_dump_distance_ = cumulative_distance_ + DUMP_INTERVAL;

        LOG(INFO) << "Next graph dump at " << next_dump_distance_ << "m";
    }
}

void GraphAdapter::addKeyframeToGraph(const std::shared_ptr<types::KeyFrame>& keyframe) {
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

}  // namespace graph
}  // namespace core

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
              return this->createKeyframe(pose, image, camera_info, std::nullopt);
          },
          0.05, 5.0) {}

// Modify input handlers to use synchronizer
void GraphAdapter::handleOdometryInput(const types::Pose& pose, double timestamp) {
    odometry_count_++;
    LOG(INFO) << "Graph adapter processing odometry message #" << odometry_count_;
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

std::shared_ptr<types::KeyFrame> GraphAdapter::createKeyframe(
    const types::Pose& pose, const std::optional<types::Image>& image,
    const std::optional<types::CameraInfo>& camera_info,
    const std::optional<types::PointCloud>& cloud) {
    LOG(INFO) << std::fixed << "Creating keyframe at pose: " << pose.position.transpose()
              << " with timestamp: " << pose.timestamp
              << " (image: " << (image.has_value() ? "available" : "unavailable")
              << ", camera_info: " << (camera_info.has_value() ? "available" : "unavailable")
              << ", cloud: " << (cloud.has_value() ? "available" : "unavailable") << ")";

    auto keyframe = std::make_shared<types::KeyFrame>();
    keyframe->id = ++current_keyframe_id_;
    keyframe->pose = pose;

    if (cloud) {
        keyframe->depth_data = *cloud;
    }

    if (image) {
        keyframe->color_data = *image;
    }

    if (camera_info) {
        keyframe->camera_info = *camera_info;
    }

    if (image && camera_info) {
        keyframe_ids_with_images_.push_back(keyframe->id);
    }

    addKeyframeToGraph(keyframe);

    return keyframe;
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
    // maybeDumpGraph();
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
    // maybeDumpGraph();
}

void GraphAdapter::maybeDumpGraph(bool force) {
    constexpr double DUMP_INTERVAL = 1.0;  // meters

    if (cumulative_distance_ >= next_dump_distance_ || force) {
        LOG(INFO) << "Dumping factor graph at distance " << cumulative_distance_ << "m";

        // Create filename with distance
        std::string filename = "/data/robot/factor_graph_" +
                               std::to_string(static_cast<int>(cumulative_distance_)) + "m.vtk";

        util::dumpFactorGraph(graph_, filename);
        next_dump_distance_ = cumulative_distance_ + DUMP_INTERVAL;

        LOG(INFO) << "Next graph dump at " << next_dump_distance_ << "m";
    }
}

void GraphAdapter::addKeyframeToGraph(const std::shared_ptr<types::KeyFrame>& keyframe) {
    LOG(INFO) << "Adding keyframe to graph at pose: " << keyframe->pose.position.transpose()
              << " with timestamp: " << keyframe->pose.timestamp;
    graph_.addKeyFrame(keyframe);
    store_.addKeyFrame(keyframe);

    if (current_keyframe_id_ > 1) {
        types::Pose relative_pose = last_keyframe_pose_.inverse() * keyframe->pose;
        double distance = calculateDistance(relative_pose);
        total_keyframe_distance_ += distance;

        LOG(INFO) << "Distance to previous keyframe: " << std::fixed << std::setprecision(2)
                  << distance << "m, Total distance between keyframes: " << total_keyframe_distance_
                  << "m";

        addOdometryFactor(current_keyframe_id_ - 1, current_keyframe_id_, relative_pose);

        // if (keyframe_ids_with_images_.size() > 1) {
        //     LOG(INFO) << "Performing tracking from ODB";
        //     orb_tracker_(graph_.getKeyFrame(keyframe_ids_with_images_.back()),
        //                  graph_.getKeyFrame(keyframe_ids_with_images_.back() - 1),
        //                  map_keypoints_);
        // }
    }

    last_keyframe_pose_ = keyframe->pose;

    if (callbacks_.on_graph_updated) {
        callbacks_.on_graph_updated();
    }
}

void GraphAdapter::handlePointCloudInput(const types::PointCloud& cloud, double timestamp) {
    // synchronizer_.addMessage(cloud, timestamp);
}

}  // namespace graph
}  // namespace core

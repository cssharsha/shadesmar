#include "ros/rosbag_reader.hpp"
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <opencv2/core.hpp>

namespace ros {

RosbagReader::RosbagReader(const std::string &bagfile,
                          core::graph::FactorGraph& graph,
                          core::storage::MapStore& store,
                          const Config &config)
    : bagfile_(bagfile)
    , config_(config)
    , graph_(graph)
    , store_(store) {}

bool RosbagReader::initialize() {
    try {
        reader_ = std::make_unique<rosbag2_cpp::Reader>();
        reader_->open(bagfile_);
        return true;
    } catch (const std::exception &e) {
        return false;
    }
}

void RosbagReader::processOdometry(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    last_odom_ = msg;

    if (!shouldCreateKeyframe(rclcpp::Time(msg->header.stamp))) {
        return;
    }

    // Create odometry factor between consecutive keyframes
    if (current_keyframe_id_ > 0) {
        // Following pattern from: core/test/integration/graph_storage_integration_tests.cpp
        // startLine: 28 endLine: 42
        core::types::Factor odom_factor;
        odom_factor.id = current_keyframe_id_;
        odom_factor.type = core::proto::FactorType::ODOMETRY;
        odom_factor.connected_nodes = {current_keyframe_id_ - 1, current_keyframe_id_};

        // Get relative pose between consecutive keyframes
        core::types::Pose relative_pose;
        relative_pose.position = Eigen::Vector3d(
            msg->pose.pose.position.x - last_odom_->pose.pose.position.x,
            msg->pose.pose.position.y - last_odom_->pose.pose.position.y,
            msg->pose.pose.position.z - last_odom_->pose.pose.position.z);
        relative_pose.orientation = Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z) *
            Eigen::Quaterniond(
                last_odom_->pose.pose.orientation.w,
                last_odom_->pose.pose.orientation.x,
                last_odom_->pose.pose.orientation.y,
                last_odom_->pose.pose.orientation.z).inverse();

        odom_factor.measurement.emplace<0>(relative_pose);
        odom_factor.information = Eigen::Matrix<double, 6, 6>::Identity();

        graph_.addFactor(odom_factor);
        store_.addFactor(odom_factor);
    }
}

bool RosbagReader::shouldCreateKeyframe(const rclcpp::Time& current_time) {
    if (current_keyframe_id_ == 0) return true;

    double time_diff = (current_time - last_keyframe_time_).seconds();
    return time_diff >= 0.5;
}

void RosbagReader::createKeyframe(
    const std::shared_ptr<sensor_msgs::msg::Image> color_msg,
    const std::shared_ptr<sensor_msgs::msg::Image> depth_msg,
    const std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info_msg) {

    // Following pattern from: core/types/test/types_test.cpp
    // startLine: 78 endLine: 101
    auto keyframe = std::make_shared<core::types::KeyFrame>();
    keyframe->id = ++current_keyframe_id_;
    keyframe->pose = conversions::toPose(*last_odom_);

    // Convert depth to point cloud
    keyframe->data = conversions::toPointCloud(*depth_msg, *camera_info_msg);
    keyframe->camera_info = conversions::toCameraInfo(*camera_info_msg);

    graph_.addKeyFrame(keyframe);
    store_.addKeyFrame(keyframe);

    // Add prior factor for first keyframe
    // Following pattern from: core/test/integration/graph_storage_integration_tests.cpp
    // startLine: 61 endLine: 66
    if (current_keyframe_id_ == 1) {
        core::types::Factor prior;
        prior.id = 0;
        prior.type = core::proto::FactorType::PRIOR;
        prior.connected_nodes = {1};
        prior.measurement.emplace<0>(keyframe->pose);
        prior.information = Eigen::Matrix<double, 6, 6>::Identity();

        graph_.addFactor(prior);
        store_.addFactor(prior);
    }

    last_keyframe_time_ = rclcpp::Time(last_odom_->header.stamp);

    // Optimize periodically following pattern from: core/graph/src/factor_graph.cpp
    // startLine: 45 endLine: 68
    if (current_keyframe_id_ % 10 == 0) {
        optimizeAndStore();
    }
}

void RosbagReader::optimizeAndStore() {
    if (graph_.optimize()) {
        auto optimized_keyframes = graph_.getAllKeyFrames();
        for (const auto& kf : optimized_keyframes) {
            store_.addKeyFrame(kf);
        }
    }
}

bool RosbagReader::processBag() {
    while (reader_->has_next()) {
        auto bag_msg = reader_->read_next();

        if (bag_msg->topic_name == config_.odom_topic) {
            auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
            rclcpp::Serialization<nav_msgs::msg::Odometry> serialization;
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            serialization.deserialize_message(&serialized_msg, odom_msg.get());
            processOdometry(odom_msg);
        } else if (bag_msg->topic_name == config_.color_topic) {
            auto color_msg = std::make_shared<sensor_msgs::msg::Image>();
            rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            serialization.deserialize_message(&serialized_msg, color_msg.get());
            last_color_msg_ = color_msg;
            processImageMessages();
        } else if (bag_msg->topic_name == config_.depth_topic) {
            auto depth_msg = std::make_shared<sensor_msgs::msg::Image>();
            rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            serialization.deserialize_message(&serialized_msg, depth_msg.get());
            last_depth_msg_ = depth_msg;
            processImageMessages();
        } else if (bag_msg->topic_name == config_.camera_info_topic) {
            auto info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
            rclcpp::Serialization<sensor_msgs::msg::CameraInfo> serialization;
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            serialization.deserialize_message(&serialized_msg, info_msg.get());
            last_camera_info_msg_ = info_msg;
            processImageMessages();
        }
    }

    optimizeAndStore();
    return store_.save("/tmp/final_map.pb");
}

void RosbagReader::processImageMessages() {
    if (!last_color_msg_ || !last_depth_msg_ || !last_camera_info_msg_ || !last_odom_) {
        return;
    }

    // Check timestamp synchronization (within 0.01 seconds)
    auto color_time = rclcpp::Time(last_color_msg_->header.stamp);
    auto depth_time = rclcpp::Time(last_depth_msg_->header.stamp);
    auto info_time = rclcpp::Time(last_camera_info_msg_->header.stamp);

    if (std::abs((color_time - depth_time).seconds()) < 0.01 &&
        std::abs((color_time - info_time).seconds()) < 0.01) {
        createKeyframe(last_color_msg_, last_depth_msg_, last_camera_info_msg_);

        // Clear messages after processing
        last_color_msg_.reset();
        last_depth_msg_.reset();
        last_camera_info_msg_.reset();
    }
}

} // namespace ros
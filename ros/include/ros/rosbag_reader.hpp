#pragma once

#include <core/graph/factor_graph.hpp>
#include <core/storage/map_store.hpp>
#include <deque>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "ros/conversions.hpp"
#include "viz/rerun_viz.hpp"

namespace ros {

struct Config {
    std::string odom_topic;
    std::string tf_topic;
    std::string tf_static_topic;
    std::string color_topic;
    std::string camera_info_topic;
    double keyframe_distance_threshold = 0.1;  // 10cm default
};

class RosbagReader {
public:
    RosbagReader(const std::string& bagfile, core::graph::FactorGraph& graph,
                 core::storage::MapStore& store, const Config& config,
                 std::shared_ptr<viz::RerunVisualizer> visualizer = nullptr);

    bool initialize();
    bool processBag();

private:
    void processOdometry(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
    void processStaticTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg);
    void processTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg);
    bool shouldCreateKeyframe(const nav_msgs::msg::Odometry& current_odom);
    void optimizeAndStore();

    std::shared_ptr<viz::RerunVisualizer> visualizer_;

    // Helper function to find closest message by timestamp
    template <typename T>
    std::shared_ptr<T> findClosestMessage(std::deque<std::shared_ptr<T>>& messages,
                                          const rclcpp::Time& target_time, double max_time_diff);

    std::string bagfile_;
    Config config_;
    core::graph::FactorGraph& graph_;
    core::storage::MapStore& store_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;

    // Message queues
    std::deque<std::shared_ptr<sensor_msgs::msg::Image>> color_messages_;
    std::deque<std::shared_ptr<sensor_msgs::msg::CameraInfo>> camera_info_messages_;

    // Tracking
    uint64_t current_keyframe_id_{0};
    core::types::Pose last_keyframe_pose_;

    std::vector<std::pair<std::pair<std::string, std::string>, core::types::Pose>> static_transforms_;
    std::vector<std::pair<std::pair<std::string, std::string>, core::types::Pose>> transforms_;
};

}  // namespace ros

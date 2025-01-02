#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "core/graph/factor_graph.hpp"
#include "core/storage/map_store.hpp"
#include "viz/rerun_viz.hpp"
#include "stf/transform_tree.hpp"

namespace ros {

struct Config {
    std::string odom_topic;
    std::string color_topic;
    std::string camera_info_topic;
    std::string tf_topic = "/tf";
    std::string tf_static_topic = "/tf_static";
    double keyframe_distance_threshold = 0.5;
    std::string odom_frame_id;
    std::string base_link_frame_id;
    std::string camera_frame_id;
};

class RosAdapter : public rclcpp::Node {
public:
    RosAdapter(const Config& config,
              core::graph::FactorGraph& graph,
              core::storage::MapStore& store,
              std::shared_ptr<viz::RerunVisualizer> visualizer = nullptr);

    bool initialize();

    void printAvailableTopics();
    std::vector<std::string> getAvailableTopics();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void tfStaticCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

    void checkFramesInitialized();

    bool shouldCreateKeyframe(const nav_msgs::msg::Odometry& current_odom);
    void optimizeAndStore();


    template <typename T>
    std::shared_ptr<T> findClosestMessage(std::deque<std::shared_ptr<T>>& messages,
                                         const rclcpp::Time& target_time,
                                         double max_time_diff);

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;

    std::string odom_frame_id_;      // Frame ID where odometry is published
    std::string camera_frame_id_;    // Frame ID where camera/image is published
    bool frames_initialized_ = false;

    // Message queues
    std::deque<std::shared_ptr<sensor_msgs::msg::Image>> color_messages_;
    std::deque<std::shared_ptr<sensor_msgs::msg::CameraInfo>> camera_info_messages_;

    // Configuration and state
    Config config_;
    core::graph::FactorGraph& graph_;
    core::storage::MapStore& store_;
    std::shared_ptr<viz::RerunVisualizer> visualizer_;

    core::types::Pose last_keyframe_pose_;
    uint64_t current_keyframe_id_{0};

    rclcpp::TimerBase::SharedPtr discovery_timer_;
    std::set<std::string> discovered_topics_;

    // Transform tree
    stf::TransformTree tf_tree_;
    bool tf_tree_built_ = false;

    rclcpp::TimerBase::SharedPtr frame_check_timer_;
    rclcpp::TimerBase::SharedPtr odom_check_timer_;
};

}  // namespace ros

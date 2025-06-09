#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "core/graph/factor_graph.hpp"
#include "core/graph/graph_adapter.hpp"
#include "core/storage/map_store.hpp"
#include "stf/transform_tree.hpp"
#include "viz/rerun_viz.hpp"

namespace ros {

struct Config {
    std::string odom_topic;
    std::string color_topic;
    std::string camera_info_topic;
    std::string depth_topic;
    std::string depth_camera_info_topic;
    std::string tf_topic = "/tf";
    std::string tf_static_topic = "/tf_static";
    double keyframe_distance_threshold = 0.5;
    std::string odom_frame_id;
    std::string base_link_frame_id;
    std::string camera_frame_id;
    std::string point_cloud_topic;
};

class RosAdapter : public rclcpp::Node {
public:
    RosAdapter(const Config& config,
               core::storage::MapStore& store,
               std::shared_ptr<viz::RerunVisualizer> visualizer = nullptr);
    ~RosAdapter();

    bool initialize();
    void printAvailableTopics();
    std::vector<std::string> getAvailableTopics();

private:
    Config config_;

    // Only store MapStore reference - FactorGraph handled internally by GraphAdapter
    core::storage::MapStore& store_;
    std::shared_ptr<viz::RerunVisualizer> visualizer_;

    // GraphAdapter manages both FactorGraph and MapStore coordination
    core::graph::FactorGraph internal_graph_;  // Internal FactorGraph for GraphAdapter
    core::graph::GraphAdapter graph_adapter_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void tfStaticCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

    void checkFramesInitialized();

    bool shouldCreateKeyframe(const nav_msgs::msg::Odometry& current_odom);
    void optimizeAndStore();

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;

    std::string odom_frame_id_;    // Frame ID where odometry is published
    std::string camera_frame_id_;  // Frame ID where camera/image is published
    bool frames_initialized_ = false;

    // Message queues
    std::deque<std::shared_ptr<sensor_msgs::msg::Image>> color_messages_;
    std::deque<std::shared_ptr<sensor_msgs::msg::CameraInfo>> camera_info_messages_;

    core::types::Pose last_keyframe_pose_;
    uint64_t current_keyframe_id_{0};

    rclcpp::TimerBase::SharedPtr discovery_timer_;
    std::set<std::string> discovered_topics_;

    // Transform tree
    stf::TransformTree tf_tree_;
    bool tf_tree_built_ = false;

    rclcpp::TimerBase::SharedPtr frame_check_timer_;
    rclcpp::TimerBase::SharedPtr odom_check_timer_;

    // // Replace the regular subscribers with message_filters subscribers
    // message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    // message_filters::Subscriber<sensor_msgs::msg::CameraInfo> depth_camera_info_sub_;

    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
    //                                                         sensor_msgs::msg::CameraInfo>
    //     SyncPolicy;
    // typedef message_filters::Synchronizer<SyncPolicy> DepthSync;

    // std::shared_ptr<DepthSync> depth_sync_;

    // Add helper method
    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
                       const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);
    core::types::PointCloud generatePointCloud(
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);

    // Add new subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

    // Add new callback
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    std::fstream pose_file_;
};

}  // namespace ros

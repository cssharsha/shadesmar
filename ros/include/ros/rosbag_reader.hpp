#pragma once

#include <core/graph/factor_graph.hpp>
#include <core/storage/map_store.hpp>
#include <fstream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "core/graph/graph_adapter.hpp"
#include "ros/conversions.hpp"
#include "stf/transform_tree.hpp"
#include "viz/rerun_viz.hpp"

namespace ros {

struct Config {
    std::string odom_topic;
    std::string tf_topic;
    std::string tf_static_topic;
    std::string color_topic;
    std::string camera_info_topic;
    std::string point_cloud_topic;
    std::string imu_topic;                            // IMU topic
    std::string base_link_frame_id;
    std::string camera_frame_id;
    std::string reference_frame_id = "base_link";     // Configurable reference frame
    double keyframe_distance_threshold = 0.1;         // 10cm default
    double keyframe_rotation_threshold = 5.0;         // 5 degrees default
    bool enable_visual_odometry = true;               // Enable essential matrix checking
    bool enable_imu_integration = true;               // Enable IMU integration
    bool use_tf_for_poses = false;                    // Use TF transforms for poses
};

class RosbagReader {
public:
    RosbagReader(const std::string& bagfile, core::graph::FactorGraph& graph,
                 core::storage::MapStore& store, const Config& config,
                 std::shared_ptr<viz::RerunVisualizer> visualizer = nullptr);

    bool initialize();
    bool processBag();

    // Configuration methods
    void setKeyframeThresholds(const core::graph::KeyframeThresholds& thresholds);
    void setOptimizationConfig(bool enable_background_optimization, size_t keyframe_interval);

private:
    // Message processing methods
    void processOdometry(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
    void processStaticTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg);
    void processTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg);
    void processImage(const std::shared_ptr<sensor_msgs::msg::Image> msg);
    void processCameraInfo(const std::shared_ptr<sensor_msgs::msg::CameraInfo> msg);
    void processPointCloud(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg);
    void processImu(const std::shared_ptr<sensor_msgs::msg::Imu> msg);

    // Message deserialization helper
    template <typename T>
    void deserializeMessage(const rosbag2_storage::SerializedBagMessage& bag_msg,
                            std::shared_ptr<T> msg);

    // Member variables
    std::string bagfile_;
    Config config_;
    core::graph::FactorGraph& graph_;
    core::storage::MapStore& store_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    std::shared_ptr<viz::RerunVisualizer> visualizer_;

    // Transform tree handling
    std::shared_ptr<stf::TransformTree> tf_tree_;
    bool tf_tree_built_{false};

    // Graph adapter for handling synchronized messages
    core::graph::GraphAdapter graph_adapter_;
    size_t odometry_count_ = 0;

    std::ofstream pose_file_;
};

}  // namespace ros

#pragma once

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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "core/graph/factor_graph.hpp"
#include "core/graph/graph_adapter.hpp"
#include "core/storage/map_store.hpp"
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
    std::string base_link_frame_id;
    std::string camera_frame_id;
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
    // Message processing methods
    void processOdometry(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
    void processStaticTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg);
    void processTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg);
    void processImage(const std::shared_ptr<sensor_msgs::msg::Image> msg);
    void processCameraInfo(const std::shared_ptr<sensor_msgs::msg::CameraInfo> msg);
    void processPointCloud(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg);

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

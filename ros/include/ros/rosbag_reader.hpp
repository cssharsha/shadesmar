#pragma once

#include <memory>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <core/graph/factor_graph.hpp>
#include <core/storage/map_store.hpp>
#include "ros/conversions.hpp"

namespace ros {

struct Config {
    std::string odom_topic;
    std::string color_topic;
    std::string depth_topic;
    std::string camera_info_topic;
};

class RosbagReader {
public:
    RosbagReader(const std::string& bagfile,
                 core::graph::FactorGraph& graph,
                 core::storage::MapStore& store,
                 const Config& config);

    bool initialize();
    bool processBag();

private:
    void processOdometry(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
    void createKeyframe(const std::shared_ptr<sensor_msgs::msg::Image> color_msg,
                       const std::shared_ptr<sensor_msgs::msg::Image> depth_msg,
                       const std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info_msg);
    void processImageMessages();
    void optimizeAndStore();
    bool shouldCreateKeyframe(const rclcpp::Time& current_time);

    std::string bagfile_;
    Config config_;
    core::graph::FactorGraph& graph_;
    core::storage::MapStore& store_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;

    // Message cache
    std::shared_ptr<nav_msgs::msg::Odometry> last_odom_;
    std::shared_ptr<sensor_msgs::msg::Image> last_color_msg_;
    std::shared_ptr<sensor_msgs::msg::Image> last_depth_msg_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> last_camera_info_msg_;

    // Tracking
    uint64_t current_keyframe_id_{0};
    rclcpp::Time last_keyframe_time_{0, 0};
};

} // namespace ros
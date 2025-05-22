#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include "ros/rosbag_reader.hpp"
#include "core/graph/keyframe_manager.hpp"

namespace viz {

struct DatasetConfig {
    std::string dataset_name;
    std::string description;

    // Topic configuration
    ros::Config ros_config;

    // Keyframe thresholds
    core::graph::KeyframeThresholds keyframe_thresholds;

    // Processing options
    bool enable_depth_processing = false;
    bool enable_point_cloud_processing = true;
    bool use_tf_for_poses = false;

    // Optimization configuration
    bool enable_background_optimization = true;
    size_t optimization_keyframe_interval = 5;
};

class ConfigLoader {
public:
    static DatasetConfig loadFromFile(const std::string& config_path);
    static DatasetConfig loadDefault();
    static DatasetConfig loadTUM();
    static DatasetConfig loadTUMAlt();

private:
    static void parseTopics(const nlohmann::json& json, ros::Config& config);
    static void parseFrameIds(const nlohmann::json& json, ros::Config& config);
    static void parseKeyframeThresholds(const nlohmann::json& json, core::graph::KeyframeThresholds& thresholds);
    static void parseProcessingOptions(const nlohmann::json& json, DatasetConfig& dataset_config);
    static void parseOptimizationOptions(const nlohmann::json& json, DatasetConfig& dataset_config);
};

}  // namespace viz
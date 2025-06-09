#include "viz/config_loader.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>

namespace viz {

DatasetConfig ConfigLoader::loadFromFile(const std::string& config_path) {
    DatasetConfig config;

    std::ifstream file(config_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open config file: " + config_path);
    }

    nlohmann::json json;
    file >> json;

    // Parse basic info
    config.dataset_name = json.value("dataset_name", "unknown");
    config.description = json.value("description", "");

    // Parse each section
    if (json.contains("topics")) {
        parseTopics(json["topics"], config.ros_config);
    }

    if (json.contains("frame_ids")) {
        parseFrameIds(json["frame_ids"], config.ros_config);
    }

    if (json.contains("keyframe_thresholds")) {
        parseKeyframeThresholds(json["keyframe_thresholds"], config.keyframe_thresholds);
    }

    if (json.contains("processing")) {
        parseProcessingOptions(json["processing"], config);
    }

    if (json.contains("processing_options")) {
        parseProcessingOptions(json["processing_options"], config);
    }

    if (json.contains("optimization")) {
        parseOptimizationOptions(json["optimization"], config);
    }

    std::cout << "Loaded configuration: " << config.dataset_name << " - " << config.description << std::endl;

    return config;
}

DatasetConfig ConfigLoader::loadDefault() {
    return loadFromFile("/workspace/viz/config/default.json");
}

DatasetConfig ConfigLoader::loadTUM() {
    return loadFromFile("/workspace/viz/config/tum.json");
}

DatasetConfig ConfigLoader::loadTUMAlt() {
    return loadFromFile("/workspace/viz/config/tum_alt.json");
}

void ConfigLoader::parseTopics(const nlohmann::json& json, ros::Config& config) {
    config.odom_topic = json.value("odom_topic", "");
    config.tf_topic = json.value("tf_topic", "/tf");
    config.tf_static_topic = json.value("tf_static_topic", "/tf_static");
    config.color_topic = json.value("color_topic", "");
    config.camera_info_topic = json.value("camera_info_topic", "");
    config.point_cloud_topic = json.value("point_cloud_topic", "");
    config.imu_topic = json.value("imu_topic", "");

    // Additional topics for depth processing
    if (json.contains("depth_topic")) {
        std::string depth_topic = json["depth_topic"];
        if (!depth_topic.empty()) {
            // Store depth topic in a way that can be accessed later
            // For now, we'll extend the Config struct if needed
        }
    }
}

void ConfigLoader::parseFrameIds(const nlohmann::json& json, ros::Config& config) {
    config.base_link_frame_id = json.value("base_link_frame_id", "base_link");
    config.camera_frame_id = json.value("camera_frame_id", "camera_color_optical_frame");
    config.reference_frame_id = json.value("reference_frame_id", "base_link");
}

void ConfigLoader::parseKeyframeThresholds(const nlohmann::json& json, core::graph::KeyframeThresholds& thresholds) {
    thresholds.distance_threshold = json.value("distance_threshold", 0.05);
    thresholds.rotation_threshold = json.value("rotation_threshold", 10.0);
    thresholds.use_visual_odometry = json.value("enable_visual_odometry", true);
    thresholds.use_imu_motion = json.value("use_imu_motion", true);
    thresholds.max_linear_velocity = json.value("max_linear_velocity", 2.0);
    thresholds.max_angular_velocity = json.value("max_angular_velocity", 1.0);
    thresholds.max_acceleration = json.value("max_acceleration", 5.0);
    thresholds.imu_integration_window = json.value("imu_integration_window", 1.0);
    thresholds.velocity_threshold_scale = json.value("velocity_threshold_scale", 0.5);
}

void ConfigLoader::parseProcessingOptions(const nlohmann::json& json, DatasetConfig& dataset_config) {
    dataset_config.enable_depth_processing = json.value("enable_depth_processing", false);
    dataset_config.enable_point_cloud_processing = json.value("enable_point_cloud_processing", true);
    dataset_config.use_tf_for_poses = json.value("use_tf_for_poses", false);

    // Pass the setting to ros_config as well
    dataset_config.ros_config.use_tf_for_poses = dataset_config.use_tf_for_poses;

    // Read visual odometry and IMU settings
    if (json.contains("enable_visual_odometry")) {
        dataset_config.keyframe_thresholds.use_visual_odometry = json["enable_visual_odometry"];
    }

    if (json.contains("enable_imu_integration")) {
        dataset_config.ros_config.enable_imu_integration = json["enable_imu_integration"];
        dataset_config.keyframe_thresholds.use_imu_motion = json["enable_imu_integration"];
    }
}

void ConfigLoader::parseOptimizationOptions(const nlohmann::json& json, DatasetConfig& dataset_config) {
    dataset_config.enable_background_optimization = json.value("enable_background_optimization", true);
    dataset_config.optimization_keyframe_interval = json.value("keyframe_interval", 5);

    std::cout << "Optimization config: enabled=" << dataset_config.enable_background_optimization
              << ", interval=" << dataset_config.optimization_keyframe_interval << " keyframes" << std::endl;
}

}  // namespace viz
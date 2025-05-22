#include "ros/rosbag_reader.hpp"
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <logging/logging.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "core/graph/util.hpp"
#include "viz/rerun_viz.hpp"

namespace ros {

RosbagReader::RosbagReader(const std::string& bagfile, core::graph::FactorGraph& graph,
                           core::storage::MapStore& store, const Config& config,
                           std::shared_ptr<viz::RerunVisualizer> visualizer)
    : bagfile_(bagfile),
      config_(config),
      graph_(graph),
      store_(store),
      visualizer_(visualizer),
      graph_adapter_(graph, store) {
    tf_tree_ = std::make_shared<stf::TransformTree>();

    // Configure keyframe thresholds using the new comprehensive approach
    core::graph::KeyframeThresholds thresholds;
    thresholds.distance_threshold = config.keyframe_distance_threshold;
    thresholds.rotation_threshold = config.keyframe_rotation_threshold;
    thresholds.use_visual_odometry = config.enable_visual_odometry;
    thresholds.min_inliers = 15;              // Good default for essential matrix
    thresholds.min_translation_visual = 0.1;  // 10cm visual movement

    graph_adapter_.setKeyframeThresholds(thresholds);
    graph_adapter_.setTransformTree(tf_tree_);
    graph_adapter_.setFrameIds(config.reference_frame_id, config.base_link_frame_id,
                               config.camera_frame_id);

    // Configure memory management - keep only recent keyframes in memory
    graph_adapter_.setInMemoryKeyframeLimit(2);  // Keep current + previous keyframes only

    // Step 3: Optimization thread will be configured via setOptimizationConfig() method

    visualizer_->setTransformTree(tf_tree_);
    visualizer_->setFrameIds(config.reference_frame_id, config.base_link_frame_id,
                             config.camera_frame_id);

    // Enable visual odometry mode if no poses available from other sources
    if (config.odom_topic.empty() && !config.use_tf_for_poses) {
        LOG(INFO) << "Enabling visual odometry mode: no odometry topic and TF poses disabled";
        graph_adapter_.enableVisualOdometryMode();
    }

    // Set up graph update callback
    core::graph::GraphCallbacks callbacks;
    callbacks.on_graph_updated = [this, &graph]() {
        if (visualizer_ && visualizer_->isConnected()) {
            auto map_points = graph_adapter_.getMapPointsCopy();  // Thread-safe access
            visualizer_->visualizeFactorGraph(graph, map_points);
        }
    };

    // Set up new storage-based visualization callback
    callbacks.on_storage_updated =
        [this](const core::storage::MapStore& map_store,
               const std::map<uint32_t, core::types::Keypoint>& map_keypoints,
               uint64_t current_keyframe_id, uint64_t previous_keyframe_id) {
            if (visualizer_ && visualizer_->isConnected()) {
                visualizer_->visualizeFromStorage(map_store, map_keypoints, current_keyframe_id,
                                                  previous_keyframe_id);
            }
        };

    graph_adapter_.setCallbacks(callbacks);
}

bool RosbagReader::initialize() {
    if (!std::filesystem::exists(bagfile_)) {
        std::cerr << "Bag file does not exist: " << bagfile_ << std::endl;
        return false;
    }

    try {
        // Create storage options
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bagfile_;
        storage_options.storage_id = "sqlite3";  // Specify the storage format

        // Create converter options
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        reader_ = std::make_unique<rosbag2_cpp::Reader>();
        reader_->open(storage_options, converter_options);

        // Print bag statistics
        const auto metadata = reader_->get_metadata();
        std::cout << "\nRosbag Statistics:" << std::endl;
        std::cout << "Duration: " << metadata.duration.count() / 1e9 << " seconds" << std::endl;
        std::cout << "Message Count: " << metadata.message_count << std::endl;
        std::cout << "\nTopics:" << std::endl;
        for (const auto& topic : metadata.topics_with_message_count) {
            std::cout << " - " << topic.topic_metadata.name << " (" << topic.message_count
                      << " messages, type: " << topic.topic_metadata.type << ")" << std::endl;
        }
        std::cout << std::endl;

        std::filesystem::path bagfile_path(bagfile_);
        std::filesystem::path output_dir = bagfile_path.parent_path();
        std::filesystem::path pose_file_path = output_dir / "odometry_poses.csv";
        pose_file_.open(pose_file_path);
        if (pose_file_.is_open()) {
            pose_file_ << "timestamp,x,y,z,qw,qx,qy,qz\n";  // CSV header
        } else {
            std::cerr << "Failed to open pose output file: " << pose_file_path.c_str() << std::endl;
            return false;
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to open rosbag: " << e.what() << std::endl;
        return false;
    }
}

bool RosbagReader::processBag() {
    LOG(INFO) << "Starting bag processing...";
    LOG(INFO) << "Expected topics:";
    LOG(INFO) << "  - TF Static: " << config_.tf_static_topic;
    LOG(INFO) << "  - TF: " << config_.tf_topic;
    LOG(INFO) << "  - Odometry: "
              << (config_.odom_topic.empty() ? "NONE (using TF)" : config_.odom_topic);
    LOG(INFO) << "  - Color: " << config_.color_topic;
    LOG(INFO) << "  - Camera Info: " << config_.camera_info_topic;
    LOG(INFO) << "  - IMU: " << config_.imu_topic;
    LOG(INFO) << "Frame IDs: " << config_.base_link_frame_id << " -> " << config_.camera_frame_id;

    size_t message_count = 0;
    size_t tf_static_count = 0, tf_count = 0, image_count = 0, camera_info_count = 0, imu_count = 0;

    while (reader_->has_next()) {
        auto bag_msg = reader_->read_next();
        message_count++;

        if (message_count % 1000 == 0) {
            LOG(INFO) << "📊 Processed " << message_count << " messages. Stats: "
                      << "TF_static=" << tf_static_count << ", TF=" << tf_count
                      << ", Images=" << image_count << ", CameraInfo=" << camera_info_count
                      << ", IMU=" << imu_count << ", TF_built=" << (tf_tree_built_ ? "YES" : "NO")
                      << ", OptPending="
                      << (graph_adapter_.isOptimizationInProgress() ? "YES" : "NO")
                      << ", KfSinceOpt=" << graph_adapter_.getKeyframesSinceLastOptimization();
        }

        if (bag_msg->topic_name == config_.tf_static_topic) {
            tf_static_count++;
            auto msg = std::make_shared<tf2_msgs::msg::TFMessage>();
            deserializeMessage(*bag_msg, msg);
            processStaticTransform(msg);
        } else if (bag_msg->topic_name == config_.tf_topic) {
            tf_count++;
            auto msg = std::make_shared<tf2_msgs::msg::TFMessage>();
            deserializeMessage(*bag_msg, msg);
            processTransform(msg);
        } else if (bag_msg->topic_name == config_.odom_topic && !config_.odom_topic.empty()) {
            auto msg = std::make_shared<nav_msgs::msg::Odometry>();
            deserializeMessage(*bag_msg, msg);
            processOdometry(msg);
        } else if (bag_msg->topic_name == config_.color_topic) {
            image_count++;
            if (!tf_tree_built_) {
                LOG(INFO) << "Skipping image message - TF tree not built yet";
            } else {
                auto msg = std::make_shared<sensor_msgs::msg::Image>();
                deserializeMessage(*bag_msg, msg);
                processImage(msg);
            }
        } else if (bag_msg->topic_name == config_.camera_info_topic) {
            camera_info_count++;
            if (!tf_tree_built_) {
                LOG(INFO) << "Skipping camera info message - TF tree not built yet";
            } else {
                auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
                deserializeMessage(*bag_msg, msg);
                processCameraInfo(msg);
            }
        } else if (bag_msg->topic_name == config_.point_cloud_topic &&
                   !config_.point_cloud_topic.empty()) {
            auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            deserializeMessage(*bag_msg, msg);
            processPointCloud(msg);
        } else if (bag_msg->topic_name == config_.imu_topic) {
            imu_count++;
            auto msg = std::make_shared<sensor_msgs::msg::Imu>();
            deserializeMessage(*bag_msg, msg);
            processImu(msg);
        } else {
            // Log unknown topics periodically
            static std::set<std::string> logged_topics;
            if (logged_topics.find(bag_msg->topic_name) == logged_topics.end()) {
                LOG(INFO) << "Unknown topic encountered: " << bag_msg->topic_name;
                logged_topics.insert(bag_msg->topic_name);
            }
        }
    }

    LOG(INFO) << "Final processing stats:";
    LOG(INFO) << "  Total messages: " << message_count;
    LOG(INFO) << "  TF Static: " << tf_static_count;
    LOG(INFO) << "  TF Dynamic: " << tf_count;
    LOG(INFO) << "  Images: " << image_count;
    LOG(INFO) << "  Camera Info: " << camera_info_count;
    LOG(INFO) << "  IMU: " << imu_count;
    LOG(INFO) << "  TF Tree Built: " << (tf_tree_built_ ? "YES" : "NO");

    std::string bagfile_name = std::filesystem::path(bagfile_).stem().string();
    std::time_t now = std::time(nullptr);
    char timestamp[20];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    std::filesystem::path bagfile_path(bagfile_);
    std::filesystem::path output_dir = bagfile_path.parent_path();
    std::string output_filename = "factor_graph_" + bagfile_name + "_" + timestamp + ".vtk";
    std::filesystem::path output_path = output_dir / output_filename;
    core::graph::util::dumpFactorGraph(graph_, output_path.string());
    if (pose_file_.is_open()) {
        pose_file_.close();
    }

    // Save all map data and indices to persistent storage
    LOG(INFO) << "Saving map data and indices to persistent storage...";
    if (!store_.saveChanges()) {
        LOG(WARNING) << "Failed to save map data to persistent storage";
    } else {
        LOG(INFO) << "Successfully saved map data and indices to disk";
    }

    return true;
}

template <typename T>
void RosbagReader::deserializeMessage(const rosbag2_storage::SerializedBagMessage& bag_msg,
                                      std::shared_ptr<T> msg) {
    rclcpp::Serialization<T> serialization;
    rclcpp::SerializedMessage serialized_msg(*bag_msg.serialized_data);
    serialization.deserialize_message(&serialized_msg, msg.get());
}

void RosbagReader::processStaticTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg) {
    for (const auto& transform : msg->transforms) {
        try {
            LOG(INFO) << "STATIC TRANSFORM: [" << transform.header.frame_id << "] -> ["
                      << transform.child_frame_id << "]";

            core::types::Pose pose;
            pose.position = Eigen::Vector3d(transform.transform.translation.x,
                                            transform.transform.translation.y,
                                            transform.transform.translation.z);
            pose.orientation =
                Eigen::Quaterniond(transform.transform.rotation.w, transform.transform.rotation.x,
                                   transform.transform.rotation.y, transform.transform.rotation.z);

            Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();
            transform_matrix.translate(pose.position);
            transform_matrix.rotate(pose.orientation);

            tf_tree_->setTransform(transform.header.frame_id, transform.child_frame_id,
                                   transform_matrix);

            // Keep track of all transforms for debugging
            static std::set<std::pair<std::string, std::string>> static_transforms;
            static_transforms.insert({transform.header.frame_id, transform.child_frame_id});

            // Print summary of available static transforms periodically
            static size_t static_count = 0;
            static_count++;
            if (static_count % 5 == 0) {
                LOG(INFO) << "STATIC TRANSFORMS SUMMARY (" << static_count << " processed):";
                for (const auto& [parent, child] : static_transforms) {
                    LOG(INFO) << "   " << parent << " -> " << child;
                }
            }

            // Check if we have the required transforms
            if (!tf_tree_built_) {
                try {
                    LOG(INFO) << "Checking for required transform: [" << config_.base_link_frame_id
                              << "] -> [" << config_.camera_frame_id << "]";
                    tf_tree_->getTransform(config_.base_link_frame_id, config_.camera_frame_id);
                    tf_tree_built_ = true;
                    LOG(INFO) << "TF tree successfully built! Required transform found.";
                    tf_tree_->printTree();
                } catch (const std::runtime_error& e) {
                    LOG(INFO) << "Still missing required transforms: " << e.what();
                    LOG(INFO) << "Looking for: [" << config_.base_link_frame_id << "] -> ["
                              << config_.camera_frame_id << "]";

                    // Fallback: If we have processed some static transforms and still can't find
                    // the required one, allow processing to continue for TUM-style datasets
                    static size_t static_transform_count = 0;
                    static_transform_count++;
                    if (static_transform_count >=
                        5) {  // After processing several static transforms
                        LOG(INFO) << "FALLBACK: Enabling TF tree for TUM-style processing after "
                                  << static_transform_count << " static transforms";
                        LOG(INFO) << "Available static transforms:";
                        for (const auto& [parent, child] : static_transforms) {
                            LOG(INFO) << "     " << parent << " -> " << child;
                        }
                        tf_tree_built_ = true;  // Force enable to allow processing to continue
                        tf_tree_->printTree();
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Failed to process static transform: " << e.what() << std::endl;
        }
    }
}

void RosbagReader::processTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg) {
    for (const auto& transform : msg->transforms) {
        try {
            LOG(INFO) << "DYNAMIC TRANSFORM: [" << transform.header.frame_id << "] -> ["
                      << transform.child_frame_id << "] at time " << std::fixed
                      << std::setprecision(3) << rclcpp::Time(transform.header.stamp).seconds();

            core::types::Pose pose;
            pose.position = Eigen::Vector3d(transform.transform.translation.x,
                                            transform.transform.translation.y,
                                            transform.transform.translation.z);
            pose.orientation =
                Eigen::Quaterniond(transform.transform.rotation.w, transform.transform.rotation.x,
                                   transform.transform.rotation.y, transform.transform.rotation.z);

            Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();
            transform_matrix.translate(pose.position);
            transform_matrix.rotate(pose.orientation);

            tf_tree_->setTransform(transform.header.frame_id, transform.child_frame_id,
                                   transform_matrix);

            // Keep track of all dynamic transforms for debugging
            static std::set<std::pair<std::string, std::string>> dynamic_transforms;
            dynamic_transforms.insert({transform.header.frame_id, transform.child_frame_id});

            // Print summary of available dynamic transforms periodically
            static size_t dynamic_count = 0;
            dynamic_count++;
            if (dynamic_count % 50 == 0) {  // Every 50 dynamic transforms
                LOG(INFO) << "DYNAMIC TRANSFORMS SUMMARY (" << dynamic_count << " processed):";
                for (const auto& [parent, child] : dynamic_transforms) {
                    LOG(INFO) << "   " << parent << " -> " << child;
                }
            }

            // Check if we have the required transforms (TUM might use dynamic transforms)
            if (!tf_tree_built_) {
                try {
                    LOG(INFO) << "Checking for required transform in dynamic TF: ["
                              << config_.base_link_frame_id << "] -> [" << config_.camera_frame_id
                              << "]";
                    tf_tree_->getTransform(config_.base_link_frame_id, config_.camera_frame_id);
                    tf_tree_built_ = true;
                    LOG(INFO) << "TF tree successfully built from dynamic transforms!";
                    tf_tree_->printTree();
                } catch (const std::runtime_error& e) {
                    // Only log occasionally to avoid spam
                    static size_t failed_checks = 0;
                    failed_checks++;
                    if (failed_checks % 100 == 0) {
                        LOG(INFO) << "Still missing required transforms in dynamic TF ("
                                  << failed_checks << " attempts): " << e.what();
                        LOG(INFO) << "Available dynamic transforms so far:";
                        for (const auto& [parent, child] : dynamic_transforms) {
                            LOG(INFO) << "     " << parent << " -> " << child;
                        }
                    }
                }
            }

            // If no odometry topic configured and TF poses are enabled, use base_link transforms as
            // pose source
            if (config_.odom_topic.empty() && config_.use_tf_for_poses && tf_tree_built_) {
                // Check if this transform gives us base_link pose
                if (transform.child_frame_id == config_.base_link_frame_id ||
                    transform.header.frame_id == config_.base_link_frame_id) {
                    try {
                        // Get pose of base_link in world/map frame
                        core::types::Pose base_pose;

                        if (transform.child_frame_id == config_.base_link_frame_id) {
                            // Transform provides base_link pose directly
                            base_pose = pose;
                        } else {
                            // Need to compute inverse transform
                            Eigen::Isometry3d inv_transform = transform_matrix.inverse();
                            base_pose.position = inv_transform.translation();
                            base_pose.orientation = Eigen::Quaterniond(inv_transform.rotation());
                        }

                        base_pose.timestamp = rclcpp::Time(transform.header.stamp).seconds();

                        LOG(INFO) << std::fixed << "TF-based pose: Position: ["
                                  << base_pose.position.x() << ", " << base_pose.position.y()
                                  << ", " << base_pose.position.z()
                                  << "], Timestamp: " << base_pose.timestamp;

                        // Send pose to graph adapter
                        graph_adapter_.handleOdometryInput(base_pose, base_pose.timestamp);

                        // Log to pose file if available
                        if (pose_file_.is_open()) {
                            pose_file_
                                << std::fixed << base_pose.timestamp << ","
                                << base_pose.position.x() << "," << base_pose.position.y() << ","
                                << base_pose.position.z() << "," << base_pose.orientation.w() << ","
                                << base_pose.orientation.x() << "," << base_pose.orientation.y()
                                << "," << base_pose.orientation.z() << "\n";
                        }

                    } catch (const std::exception& e) {
                        // Failed to extract pose, continue normally
                        LOG(INFO) << "Could not extract pose from transform: " << e.what();
                    }
                }
            }

        } catch (const std::exception& e) {
            std::cerr << "Failed to process transform: " << e.what() << std::endl;
        }
    }
}

void RosbagReader::processOdometry(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    if (!tf_tree_built_) {
        return;
    }

    core::types::Pose pose = conversions::toPose(*msg);
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    LOG(INFO) << std::fixed << "Odometry message details - Position: [" << msg->pose.pose.position.x
              << ", " << msg->pose.pose.position.y << ", " << msg->pose.pose.position.z
              << "], Timestamp: " << timestamp;

    odometry_count_++;
    LOG(INFO) << "Processing odometry message #" << odometry_count_;

    graph_adapter_.handleOdometryInput(pose, timestamp);

    if (pose_file_.is_open()) {
        pose_file_ << std::fixed << timestamp << "," << pose.position.x() << ","
                   << pose.position.y() << "," << pose.position.z() << "," << pose.orientation.w()
                   << "," << pose.orientation.x() << "," << pose.orientation.y() << ","
                   << pose.orientation.z() << "\n";
    }
}

void RosbagReader::processImage(const std::shared_ptr<sensor_msgs::msg::Image> msg) {
    if (!tf_tree_built_) {
        return;
    }

    core::types::Image image = conversions::toImage(*msg);
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    graph_adapter_.handleImageInput(image, timestamp);
}

void RosbagReader::processCameraInfo(const std::shared_ptr<sensor_msgs::msg::CameraInfo> msg) {
    if (!tf_tree_built_) {
        return;
    }

    core::types::CameraInfo camera_info = conversions::toCameraInfo(*msg);
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    graph_adapter_.handleCameraInfo(camera_info, timestamp);
}

void RosbagReader::processPointCloud(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg) {
    if (!tf_tree_built_) {
        return;
    }

    try {
        core::types::PointCloud cloud;
        cloud.points.reserve(msg->width * msg->height);
        cloud.colors.reserve(msg->width * msg->height);

        // Get field offsets
        int x_offset = -1, y_offset = -1, z_offset = -1;
        int r_offset = -1, g_offset = -1, b_offset = -1;

        for (const auto& field : msg->fields) {
            if (field.name == "x")
                x_offset = field.offset;
            else if (field.name == "y")
                y_offset = field.offset;
            else if (field.name == "z")
                z_offset = field.offset;
            else if (field.name == "r")
                r_offset = field.offset;
            else if (field.name == "g")
                g_offset = field.offset;
            else if (field.name == "b")
                b_offset = field.offset;
        }

        if (x_offset == -1 || y_offset == -1 || z_offset == -1) {
            throw std::runtime_error("Point cloud missing position fields");
        }

        const uint8_t* data_ptr = msg->data.data();
        for (size_t i = 0; i < msg->width * msg->height; ++i) {
            const uint8_t* point_ptr = data_ptr + i * msg->point_step;

            float x = *reinterpret_cast<const float*>(point_ptr + x_offset);
            float y = *reinterpret_cast<const float*>(point_ptr + y_offset);
            float z = *reinterpret_cast<const float*>(point_ptr + z_offset);

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }

            cloud.points.emplace_back(x, y, z);

            if (r_offset != -1 && g_offset != -1 && b_offset != -1) {
                uint8_t r = *(point_ptr + r_offset);
                uint8_t g = *(point_ptr + g_offset);
                uint8_t b = *(point_ptr + b_offset);
                cloud.colors.emplace_back(r / 255.0, g / 255.0, b / 255.0);
            } else {
                cloud.colors.emplace_back(1.0, 1.0, 1.0);
            }
        }

        double timestamp = rclcpp::Time(msg->header.stamp).seconds();
        graph_adapter_.handlePointCloudInput(cloud, timestamp);

    } catch (const std::exception& e) {
        std::cerr << "Error processing point cloud: " << e.what() << std::endl;
    }
}

void RosbagReader::processImu(const std::shared_ptr<sensor_msgs::msg::Imu> msg) {
    if (!config_.enable_imu_integration) {
        return;  // IMU integration disabled
    }

    try {
        core::types::ImuData imu_data = ros::conversions::toImuData(*msg);

        double msg_time = rclcpp::Time(msg->header.stamp).seconds();
        graph_adapter_.handleImuInput(imu_data, msg_time);

        static size_t imu_count = 0;
        if (++imu_count % 100 == 0) {  // Log every 100th IMU message
            std::cout << "Processed " << imu_count << " IMU messages. "
                      << "Latest timestamp: " << std::fixed << std::setprecision(3) << msg_time
                      << "s" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Failed to process IMU message: " << e.what() << std::endl;
    }
}

void RosbagReader::setKeyframeThresholds(const core::graph::KeyframeThresholds& thresholds) {
    graph_adapter_.setKeyframeThresholds(thresholds);
    LOG(INFO) << "Updated keyframe thresholds: distance=" << thresholds.distance_threshold
              << "m, rotation=" << thresholds.rotation_threshold << "°"
              << ", IMU motion=" << (thresholds.use_imu_motion ? "enabled" : "disabled");
}

void RosbagReader::setOptimizationConfig(bool enable_background_optimization,
                                         size_t keyframe_interval) {
    if (enable_background_optimization) {
        graph_adapter_.setOptimizationInterval(keyframe_interval);
        graph_adapter_.enableOptimizationThread();
        LOG(INFO) << "Background optimization enabled (interval: " << keyframe_interval
                  << " keyframes)";
    } else {
        graph_adapter_.disableOptimizationThread();
        LOG(INFO) << "Background optimization disabled by configuration";
    }
}

}  // namespace ros

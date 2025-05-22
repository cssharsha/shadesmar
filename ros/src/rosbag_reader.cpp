#include "ros/rosbag_reader.hpp"
#include <cmath>
#include <filesystem>
#include <opencv2/core.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
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
    graph_adapter_.setKeyframeDistanceThreshold(config.keyframe_distance_threshold);
    graph_adapter_.setTransformTree(tf_tree_);
    visualizer_->setTransformTree(tf_tree_);

    // Set up graph update callback
    core::graph::GraphCallbacks callbacks;
    callbacks.on_graph_updated = [this, &graph]() {
        if (visualizer_ && visualizer_->isConnected()) {
            auto& map_points = graph_adapter_.getMapPoints();
            visualizer_->visualizeFactorGraph(graph, map_points);
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
            std::cerr << "Failed to open pose output file" << std::endl;
            return false;
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to open rosbag: " << e.what() << std::endl;
        return false;
    }
}

bool RosbagReader::processBag() {
    while (reader_->has_next()) {
        auto bag_msg = reader_->read_next();

        if (bag_msg->topic_name == config_.tf_static_topic) {
            auto msg = std::make_shared<tf2_msgs::msg::TFMessage>();
            deserializeMessage(*bag_msg, msg);
            processStaticTransform(msg);
        } else if (bag_msg->topic_name == config_.tf_topic) {
            auto msg = std::make_shared<tf2_msgs::msg::TFMessage>();
            deserializeMessage(*bag_msg, msg);
            processTransform(msg);
        } else if (bag_msg->topic_name == config_.odom_topic) {
            auto msg = std::make_shared<nav_msgs::msg::Odometry>();
            deserializeMessage(*bag_msg, msg);
            processOdometry(msg);
        } else if (bag_msg->topic_name == config_.color_topic) {
            auto msg = std::make_shared<sensor_msgs::msg::Image>();
            deserializeMessage(*bag_msg, msg);
            processImage(msg);
        } else if (bag_msg->topic_name == config_.camera_info_topic) {
            auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
            deserializeMessage(*bag_msg, msg);
            processCameraInfo(msg);
        } else if (bag_msg->topic_name == config_.point_cloud_topic) {
            auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            deserializeMessage(*bag_msg, msg);
            processPointCloud(msg);
        }
    }
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

            // Check if we have the required transforms
            if (!tf_tree_built_) {
                try {
                    tf_tree_->getTransform(config_.base_link_frame_id, config_.camera_frame_id);
                    tf_tree_built_ = true;
                    tf_tree_->printTree();
                } catch (const std::runtime_error&) {
                    // Still missing required transforms
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

}  // namespace ros

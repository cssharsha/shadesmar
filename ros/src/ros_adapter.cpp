#include "ros/ros_adapter.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cmath>
#include <filesystem>
#include "core/graph/util.hpp"
#include "ros/conversions.hpp"

namespace ros {

RosAdapter::RosAdapter(const Config& config,
                       core::storage::MapStore& store,
                       std::shared_ptr<viz::RerunVisualizer> visualizer)
    : Node("ros_adapter"), config_(config), store_(store), visualizer_(visualizer),
      graph_adapter_(internal_graph_, store) {
    graph_adapter_.setKeyframeDistanceThreshold(config.keyframe_distance_threshold);

    // Set up clean storage-based callbacks
    core::graph::GraphCallbacks callbacks;
    // REMOVED: on_graph_updated callback - deprecated since FactorGraph is stateless
    // All visualization should use MapStore as single source of truth

    graph_adapter_.setCallbacks(callbacks);
}

bool RosAdapter::initialize() {
    try {
        // Wait longer for static transforms (5 seconds is more reliable than 1)
        rclcpp::sleep_for(std::chrono::seconds(5));

        // Create subscribers with QoS settings for static transforms
        rclcpp::QoS static_tf_qos(10);
        static_tf_qos.reliable();
        static_tf_qos.transient_local();  // Ensures we get the last published message

        tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            config_.tf_static_topic,
            static_tf_qos,  // Use the modified QoS
            std::bind(&RosAdapter::tfStaticCallback, this, std::placeholders::_1));

        // Add debug info for odometry subscription
        RCLCPP_INFO(this->get_logger(), "Creating odometry subscription on topic: %s",
                    config_.odom_topic.c_str());

        // Create subscribers with QoS settings matching the publisher
        rclcpp::QoS odom_qos(10);
        odom_qos.best_effort();          // Match publisher's BEST_EFFORT reliability
        odom_qos.durability_volatile();  // Match publisher's VOLATILE durability

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            config_.odom_topic,
            odom_qos,  // Use the modified QoS profile
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                RCLCPP_DEBUG(this->get_logger(),
                             "Raw odometry message received on topic '%s' from frame '%s'",
                             config_.odom_topic.c_str(), msg->header.frame_id.c_str());
                this->odomCallback(msg);
            });

        if (!odom_sub_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create odometry subscription!");
            return false;
        }

        // Add immediate subscription status check
        RCLCPP_INFO(this->get_logger(),
                    "Odometry subscription created successfully. Subscription count: %ld",
                    this->count_subscribers(config_.odom_topic));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            config_.color_topic, 10,
            std::bind(&RosAdapter::imageCallback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            config_.camera_info_topic, 10,
            std::bind(&RosAdapter::cameraInfoCallback, this, std::placeholders::_1));

        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            config_.tf_topic, 10, std::bind(&RosAdapter::tfCallback, this, std::placeholders::_1));

        // Add point cloud subscriber
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            config_.point_cloud_topic, 10,
            std::bind(&RosAdapter::pointCloudCallback, this, std::placeholders::_1));

        // Add periodic topic discovery
        discovery_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
            auto topics = getAvailableTopics();
            for (const auto& topic : topics) {
                discovered_topics_.insert(topic);
                RCLCPP_INFO(this->get_logger(), "Discovered topic: %s", topic.c_str());
            }

            // Check if all required topics are discovered
            bool all_found = true;
            std::vector<std::string> required_topics = {
                config_.odom_topic,  config_.tf_topic,          config_.tf_static_topic,
                config_.color_topic, config_.camera_info_topic, config_.point_cloud_topic};

            for (const auto& required : required_topics) {
                if (discovered_topics_.find(required) == discovered_topics_.end()) {
                    all_found = false;
                    RCLCPP_WARN(this->get_logger(), "Still waiting for topic: %s",
                                required.c_str());
                }
            }

            if (all_found) {
                RCLCPP_INFO(this->get_logger(), "All required topics discovered!");
                discovery_timer_->cancel();
            }
        });

        // Add a periodic check for odometry messages
        odom_check_timer_ = this->create_wall_timer(std::chrono::seconds(5), [this]() {
            auto sub_count = this->count_subscribers(config_.odom_topic);
            auto pub_count = this->count_publishers(config_.odom_topic);

            RCLCPP_DEBUG(this->get_logger(),
                         "Odometry topic '%s' status:\n"
                         "  - Subscription active: %s\n"
                         "  - Number of subscribers: %ld\n"
                         "  - Number of publishers: %ld",
                         config_.odom_topic.c_str(), odom_sub_ ? "yes" : "no", sub_count,
                         pub_count);

            if (pub_count == 0) {
                RCLCPP_WARN(this->get_logger(), "No publishers found for odometry topic '%s'",
                            config_.odom_topic.c_str());
            }
        });

        // depth_sub_.subscribe(this, config_.depth_topic);
        // depth_camera_info_sub_.subscribe(this, config_.depth_camera_info_topic);
        // depth_sync_ =
        //     std::make_shared<DepthSync>(DepthSync(10), depth_sub_, depth_camera_info_sub_);
        // depth_sync_->registerCallback(std::bind(&RosAdapter::depthCallback, this,
        //                                         std::placeholders::_1, std::placeholders::_2));

        // Create directory if it doesn't exist
        std::filesystem::path dir_path("/mnt/remote-storage");
        if (!std::filesystem::exists(dir_path)) {
            if (!std::filesystem::create_directories(dir_path)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s",
                             dir_path.c_str());
                return false;
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Directory already exists: %s", dir_path.c_str());
        }

        // Try to open the file
        pose_file_.open("/mnt/remote-storage/odometry_live.csv");
        if (pose_file_.is_open()) {
            pose_file_ << "timestamp,x,y,z,qw,qx,qy,qz\n";  // CSV header
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to open pose output file. Check permissions for: %s",
                         "/mnt/remote-storage/odometry_live.csv");
            return false;
        }

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize: %s", e.what());
        return false;
    }
}

void RosAdapter::tfStaticCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    if (!visualizer_ || !visualizer_->isConnected())
        return;

    for (const auto& transform : msg->transforms) {
        core::types::Pose pose;
        pose.position =
            Eigen::Vector3d(transform.transform.translation.x, transform.transform.translation.y,
                            transform.transform.translation.z);
        pose.orientation =
            Eigen::Quaterniond(transform.transform.rotation.w, transform.transform.rotation.x,
                               transform.transform.rotation.y, transform.transform.rotation.z);

        // Convert to Eigen::Isometry3d
        Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();
        transform_matrix.translate(pose.position);
        transform_matrix.rotate(pose.orientation);

        // Add to transform tree
        try {
            RCLCPP_INFO(this->get_logger(), "Adding static transform from %s to %s",
                        transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
            tf_tree_.setTransform(transform.header.frame_id, transform.child_frame_id,
                                  transform_matrix);

            // Log the transform path
            auto result =
                tf_tree_.getTransform(transform.header.frame_id, transform.child_frame_id);

            // Check if base_link to camera transform is available
            try {
                auto base_to_camera =
                    tf_tree_.getTransform(config_.base_link_frame_id, config_.camera_frame_id);
                if (tf_tree_built_) {
                    RCLCPP_INFO(this->get_logger(), "Transform tree built successfully!");
                    tf_tree_.printTree();
                }
                tf_tree_built_ = true;
            } catch (const std::runtime_error&) {
                // Transform not available yet
                tf_tree_built_ = false;
            }
        } catch (const std::runtime_error& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to add static transform: %s", e.what());
        }
    }
}

void RosAdapter::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    if (!visualizer_ || !visualizer_->isConnected())
        return;

    for (const auto& transform : msg->transforms) {
        core::types::Pose pose;
        pose.position =
            Eigen::Vector3d(transform.transform.translation.x, transform.transform.translation.y,
                            transform.transform.translation.z);
        pose.orientation =
            Eigen::Quaterniond(transform.transform.rotation.w, transform.transform.rotation.x,
                               transform.transform.rotation.y, transform.transform.rotation.z);

        // Convert to Eigen::Isometry3d
        Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();
        transform_matrix.translate(pose.position);
        transform_matrix.rotate(pose.orientation);

        // Add to transform tree
        try {
            tf_tree_.setTransform(transform.header.frame_id, transform.child_frame_id,
                                  transform_matrix);

            // Log the transform path
            auto result =
                tf_tree_.getTransform(transform.header.frame_id, transform.child_frame_id);
        } catch (const std::runtime_error& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to add transform: %s", e.what());
        }
    }
}

void RosAdapter::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!tf_tree_built_) {
        RCLCPP_DEBUG(this->get_logger(), "Skipping odom message - waiting for transform tree");
        return;
    }
    if (!frames_initialized_) {
        odom_frame_id_ = msg->header.frame_id;
        RCLCPP_INFO(this->get_logger(), "Odometry frame detected: %s", odom_frame_id_.c_str());
        checkFramesInitialized();
    }

    try {
        double msg_time = rclcpp::Time(msg->header.stamp).seconds();

        // Update transform tree
        Eigen::Isometry3d odom_transform = Eigen::Isometry3d::Identity();
        odom_transform.translate(Eigen::Vector3d(
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        odom_transform.rotate(
            Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                               msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));

        tf_tree_.setTransform(msg->header.frame_id, msg->child_frame_id, odom_transform);
        auto result = tf_tree_.getTransform(msg->header.frame_id, msg->child_frame_id);

        // Send pose to graph adapter
        core::types::Pose current_pose = conversions::toPose(*msg);
        graph_adapter_.handleOdometryInput(current_pose, msg_time);

        // Visualize if available
        if (visualizer_ && visualizer_->isConnected()) {
            std::string odom_path = result.path;
            visualizer_->addPose(current_pose, odom_path, msg_time);
        }

        if (pose_file_.is_open()) {
            pose_file_ << std::fixed << msg_time << "," << current_pose.position.x() << ","
                       << current_pose.position.y() << "," << current_pose.position.z() << ","
                       << current_pose.orientation.w() << "," << current_pose.orientation.x() << ","
                       << current_pose.orientation.y() << "," << current_pose.orientation.z()
                       << "\n";
            pose_file_.flush();  // Ensure data is written immediately
        }
    } catch (const std::runtime_error& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to add odometry transform: %s", e.what());
    }
}

void RosAdapter::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!tf_tree_built_) {
        RCLCPP_DEBUG(this->get_logger(), "Skipping image message - waiting for transform tree");
        return;
    }
    if (!frames_initialized_) {
        camera_frame_id_ = msg->header.frame_id;
        checkFramesInitialized();
    }

    // Convert and send to graph adapter
    double msg_time = rclcpp::Time(msg->header.stamp).seconds();
    core::types::Image image = conversions::toImage(*msg);
    graph_adapter_.handleImageInput(image, msg_time);

    // Visualize if available
    if (visualizer_ && visualizer_->isConnected()) {
        try {
            auto base_to_camera = tf_tree_.getTransform("base_link", camera_frame_id_);
            std::string camera_path = config_.odom_frame_id + "/" + base_to_camera.path;
            std::string image_path = camera_path + "/image";
            cv::Mat cv_image = conversions::toOpenCVImage(*msg);
            visualizer_->addImage(cv_image, image_path, msg_time);
        } catch (const std::runtime_error& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to visualize image: %s", e.what());
        }
    }
}

void RosAdapter::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!tf_tree_built_) {
        RCLCPP_DEBUG(this->get_logger(),
                     "Skipping camera info message - waiting for transform tree");
        return;
    }

    // Convert and send to graph adapter
    double msg_time = rclcpp::Time(msg->header.stamp).seconds();
    core::types::CameraInfo camera_info = conversions::toCameraInfo(*msg);
    graph_adapter_.handleCameraInfo(camera_info, msg_time);

    // Visualize if available
    if (visualizer_ && visualizer_->isConnected()) {
        // Check if principal point is not at (0,0)
        if (std::abs(msg->k[2]) > 1e-6 || std::abs(msg->k[5]) > 1e-6) {
            RCLCPP_DEBUG(this->get_logger(),
                         "Non-zero principal point (%.2f, %.2f) detected. Rerun visualization may "
                         "be inaccurate.",
                         msg->k[2], msg->k[5]);
        }

        // Create pinhole camera using from_focal_length_and_resolution
        auto camera = rerun::archetypes::Pinhole::from_focal_length_and_resolution(
            {static_cast<float>(msg->k[0]),
             static_cast<float>(msg->k[4])},  // focal lengths (fx, fy)
            {static_cast<float>(msg->width), static_cast<float>(msg->height)}
            // resolution (uint32_t)
        );

        stf::TransformTree::TransformResult base_to_camera;
        try {
            base_to_camera = tf_tree_.getTransform(config_.base_link_frame_id, camera_frame_id_);

            // Add the transform visualization
            if (visualizer_ && visualizer_->isConnected()) {
                Eigen::Isometry3d transform = base_to_camera.transform;
                core::types::Pose camera_pose;
                camera_pose.position = transform.translation();
                camera_pose.orientation = Eigen::Quaterniond(transform.rotation());

                std::string transform_path = config_.odom_frame_id + "/" + base_to_camera.path;
                visualizer_->addPose(camera_pose, transform_path,
                                     rclcpp::Time(msg->header.stamp).seconds());
            }
        } catch (const std::runtime_error& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to get transform from %s to %s: %s",
                        config_.base_link_frame_id.c_str(), camera_frame_id_.c_str(), e.what());
            return;
        }
        std::string camera_path = config_.odom_frame_id + "/" + base_to_camera.path;
        visualizer_->addCamera(camera, camera_path, rclcpp::Time(msg->header.stamp).seconds());
    }

    while (camera_info_messages_.size() > 1000) {
        camera_info_messages_.pop_front();
    }
}

void RosAdapter::printAvailableTopics() {
    auto topic_names_and_types = this->get_topic_names_and_types();
    RCLCPP_DEBUG(this->get_logger(), "Available topics:");
    for (const auto& topic : topic_names_and_types) {
        std::string types_str;
        for (const auto& type : topic.second) {
            if (!types_str.empty())
                types_str += ", ";
            types_str += type;
        }
        RCLCPP_DEBUG(this->get_logger(), "  %s [%s]", topic.first.c_str(), types_str.c_str());
    }
}

std::vector<std::string> RosAdapter::getAvailableTopics() {
    std::vector<std::string> topics;
    auto topic_names_and_types = this->get_topic_names_and_types();
    for (const auto& topic : topic_names_and_types) {
        topics.push_back(topic.first);
    }
    return topics;
}

void RosAdapter::checkFramesInitialized() {
    if (!odom_frame_id_.empty() && !camera_frame_id_.empty() && !frames_initialized_) {
        frames_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Frame relationship established: %s -> %s",
                    odom_frame_id_.c_str(), camera_frame_id_.c_str());
    }
}

void RosAdapter::depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
                               const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg) {
    if (!tf_tree_built_) {
        RCLCPP_DEBUG(this->get_logger(), "Skipping depth message - waiting for transform tree");
        return;
    }

    try {
        // Generate point cloud from depth image
        core::types::PointCloud cloud = generatePointCloud(depth_msg, info_msg);

        // Send to graph adapter
        double msg_time = rclcpp::Time(depth_msg->header.stamp).seconds();
        graph_adapter_.handlePointCloudInput(cloud, msg_time);

        // Visualize if available
        if (visualizer_ && visualizer_->isConnected()) {
            try {
                auto base_to_camera =
                    tf_tree_.getTransform("base_link", depth_msg->header.frame_id);

                // // Transform point cloud to base_link frame
                // core::types::PointCloud transformed_cloud;
                // transformed_cloud.points.reserve(cloud.points.size());
                // transformed_cloud.colors = cloud.colors;  // Colors stay the same

                // Eigen::Isometry3d transform = base_to_camera.transform;
                // for (const auto& point : cloud.points) {
                //     transformed_cloud.points.push_back(transform * point);
                // }

                std::string camera_path = "/odom/" + base_to_camera.path;
                std::string cloud_path = camera_path + "/point_cloud";

                core::types::Pose camera_pose;
                camera_pose.position = base_to_camera.transform.translation();
                camera_pose.orientation = Eigen::Quaterniond(base_to_camera.transform.rotation());
                visualizer_->addPointCloud(cloud, cloud_path, msg_time, camera_pose);
            } catch (const std::runtime_error& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to visualize point cloud: %s", e.what());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing depth data: %s", e.what());
    }
}

core::types::PointCloud RosAdapter::generatePointCloud(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
    core::types::PointCloud cloud;

    // Get camera parameters
    double fx = info_msg->k[0];
    double fy = info_msg->k[4];
    double cx = info_msg->k[2];
    double cy = info_msg->k[5];

    // Convert depth image to point cloud
    float* depth_data = reinterpret_cast<float*>(const_cast<uint8_t*>(&depth_msg->data[0]));

    for (uint32_t v = 0; v < depth_msg->height; v++) {
        for (uint32_t u = 0; u < depth_msg->width; u++) {
            float depth = depth_data[v * depth_msg->width + u];

            // Skip invalid measurements
            if (!std::isfinite(depth) || depth <= 0) {
                continue;
            }

            // Back-project to 3D
            double x = (u - cx) * depth / fx;
            double y = (v - cy) * depth / fy;
            double z = depth;

            cloud.points.emplace_back(x, y, z);

            // Add a default color (white)
            cloud.colors.emplace_back(1.0, 1.0, 1.0);
        }
    }

    return cloud;
}

void RosAdapter::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!tf_tree_built_) {
        RCLCPP_DEBUG(this->get_logger(),
                     "Skipping point cloud message - waiting for transform tree");
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

        // Check if we have position fields
        if (x_offset == -1 || y_offset == -1 || z_offset == -1) {
            RCLCPP_ERROR(this->get_logger(), "Point cloud missing position fields");
            return;
        }

        // Convert point cloud data
        const uint8_t* data_ptr = msg->data.data();
        for (size_t i = 0; i < msg->width * msg->height; ++i) {
            const uint8_t* point_ptr = data_ptr + i * msg->point_step;

            // Get position
            float x = *reinterpret_cast<const float*>(point_ptr + x_offset);
            float y = *reinterpret_cast<const float*>(point_ptr + y_offset);
            float z = *reinterpret_cast<const float*>(point_ptr + z_offset);

            // Skip invalid points
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }

            cloud.points.emplace_back(x, y, z);

            // Get color if available
            if (r_offset != -1 && g_offset != -1 && b_offset != -1) {
                uint8_t r = *(point_ptr + r_offset);
                uint8_t g = *(point_ptr + g_offset);
                uint8_t b = *(point_ptr + b_offset);
                cloud.colors.emplace_back(r / 255.0, g / 255.0, b / 255.0);
            } else {
                cloud.colors.emplace_back(1.0, 1.0, 1.0);  // Default white color
            }
        }

        // Send to graph adapter
        double msg_time = rclcpp::Time(msg->header.stamp).seconds();
        graph_adapter_.handlePointCloudInput(cloud, msg_time);

        // Visualize if available
        if (visualizer_ && visualizer_->isConnected()) {
            try {
                auto base_to_sensor = tf_tree_.getTransform("base_link", msg->header.frame_id);
                std::string camera_path = "/odom/" + base_to_sensor.path;
                std::string cloud_path = camera_path + "/point_cloud";

                core::types::Pose camera_pose;
                camera_pose.position = base_to_sensor.transform.translation();
                camera_pose.orientation = Eigen::Quaterniond(base_to_sensor.transform.rotation());
                visualizer_->addPointCloud(cloud, cloud_path, msg_time, camera_pose);
            } catch (const std::runtime_error& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to visualize point cloud: %s", e.what());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
    }
}

RosAdapter::~RosAdapter() {
    if (pose_file_.is_open()) {
        pose_file_.close();
    }
}

}  // namespace ros

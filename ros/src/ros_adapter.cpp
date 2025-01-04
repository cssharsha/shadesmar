#include "ros/ros_adapter.hpp"
#include <cmath>
#include "core/graph/util.hpp"
#include "ros/conversions.hpp"

namespace ros {

RosAdapter::RosAdapter(const Config& config, core::graph::FactorGraph& graph,
                       core::storage::MapStore& store,
                       std::shared_ptr<viz::RerunVisualizer> visualizer)
    : Node("ros_adapter"), config_(config), graph_adapter_(graph, store), visualizer_(visualizer) {
    graph_adapter_.setKeyframeDistanceThreshold(config.keyframe_distance_threshold);

    // Set up graph update callback only
    core::graph::GraphCallbacks callbacks;
    callbacks.on_graph_updated = [this, &graph]() {
        // RCLCPP_INFO(this->get_logger(), "Graph updated");
        if (visualizer_ && visualizer_->isConnected()) {
            visualizer_->visualizeFactorGraph(graph);
        }
    };

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
                config_.odom_topic, config_.tf_topic, config_.tf_static_topic, config_.color_topic,
                config_.camera_info_topic};

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

}  // namespace ros

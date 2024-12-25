#include "ros/ros_adapter.hpp"
#include <cmath>
#include "core/graph/util.hpp"
#include "ros/conversions.hpp"

namespace ros {

RosAdapter::RosAdapter(const Config& config,
                             core::graph::FactorGraph& graph,
                             core::storage::MapStore& store,
                             std::shared_ptr<viz::RerunVisualizer> visualizer)
    : Node("ros_adapter"),
      config_(config),
      graph_(graph),
      store_(store),
      visualizer_(visualizer) {}

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
        odom_qos.best_effort();  // Match publisher's BEST_EFFORT reliability
        odom_qos.durability_volatile();  // Match publisher's VOLATILE durability

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            config_.odom_topic,
            odom_qos,  // Use the modified QoS profile
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                RCLCPP_DEBUG(this->get_logger(),
                    "Raw odometry message received on topic '%s' from frame '%s'",
                    config_.odom_topic.c_str(),
                    msg->header.frame_id.c_str());
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
            config_.tf_topic, 10,
            std::bind(&RosAdapter::tfCallback, this, std::placeholders::_1));

        // Add periodic topic discovery
        discovery_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                auto topics = getAvailableTopics();
                for (const auto& topic : topics) {
                    discovered_topics_.insert(topic);
                    RCLCPP_INFO(this->get_logger(), "Discovered topic: %s", topic.c_str());
                }

                // Check if all required topics are discovered
                bool all_found = true;
                std::vector<std::string> required_topics = {
                    config_.odom_topic,
                    config_.tf_topic,
                    config_.tf_static_topic,
                    config_.color_topic,
                    config_.camera_info_topic
                };

                for (const auto& required : required_topics) {
                    if (discovered_topics_.find(required) == discovered_topics_.end()) {
                        all_found = false;
                        RCLCPP_WARN(this->get_logger(), "Still waiting for topic: %s", required.c_str());
                    }
                }

                if (all_found) {
                    RCLCPP_INFO(this->get_logger(), "All required topics discovered!");
                    discovery_timer_->cancel();
                }
            });

        // Add a periodic check for odometry messages
        odom_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() {
                auto sub_count = this->count_subscribers(config_.odom_topic);
                auto pub_count = this->count_publishers(config_.odom_topic);

                RCLCPP_DEBUG(this->get_logger(),
                    "Odometry topic '%s' status:\n"
                    "  - Subscription active: %s\n"
                    "  - Number of subscribers: %ld\n"
                    "  - Number of publishers: %ld",
                    config_.odom_topic.c_str(),
                    odom_sub_ ? "yes" : "no",
                    sub_count,
                    pub_count);

                if (pub_count == 0) {
                    RCLCPP_WARN(this->get_logger(),
                        "No publishers found for odometry topic '%s'",
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
    if (!visualizer_ || !visualizer_->isConnected()) return;

    for (const auto& transform : msg->transforms) {
        core::types::Pose pose;
        pose.position = Eigen::Vector3d(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );
        pose.orientation = Eigen::Quaterniond(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z
        );

        // Convert to Eigen::Isometry3d
        Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();
        transform_matrix.translate(pose.position);
        transform_matrix.rotate(pose.orientation);

        // Add to transform tree
        try {
            RCLCPP_INFO(this->get_logger(), "Adding static transform from %s to %s",
                       transform.header.frame_id.c_str(),
                       transform.child_frame_id.c_str());
            tf_tree_.setTransform(transform.header.frame_id,
                                transform.child_frame_id,
                                transform_matrix);

            // Log the transform path
            auto result = tf_tree_.getTransform(transform.header.frame_id, transform.child_frame_id);

            // Check if base_link to camera transform is available
            try {
                auto base_to_camera = tf_tree_.getTransform(config_.base_link_frame_id, config_.camera_frame_id);
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
    if (!visualizer_ || !visualizer_->isConnected()) return;

    for (const auto& transform : msg->transforms) {
        core::types::Pose pose;
        pose.position = Eigen::Vector3d(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );
        pose.orientation = Eigen::Quaterniond(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z
        );

        // Convert to Eigen::Isometry3d
        Eigen::Isometry3d transform_matrix = Eigen::Isometry3d::Identity();
        transform_matrix.translate(pose.position);
        transform_matrix.rotate(pose.orientation);

        // Add to transform tree
        try {
            tf_tree_.setTransform(transform.header.frame_id,
                                transform.child_frame_id,
                                transform_matrix);

            // Log the transform path
            auto result = tf_tree_.getTransform(transform.header.frame_id, transform.child_frame_id);
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

    // Convert odometry message to transform matrix
    Eigen::Isometry3d odom_transform = Eigen::Isometry3d::Identity();
    odom_transform.translate(Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    ));
    odom_transform.rotate(Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    ));

    stf::TransformTree::TransformResult result;
    // Add to transform tree
    try {
        tf_tree_.setTransform(msg->header.frame_id,
                             msg->child_frame_id,
                             odom_transform);

        // Get the transform path
        result = tf_tree_.getTransform(msg->header.frame_id, msg->child_frame_id);
    } catch (const std::runtime_error& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to add odometry transform: %s", e.what());
    }

    if (visualizer_ && visualizer_->isConnected()) {
        core::types::Pose current_pose = conversions::toPose(*msg);
        visualizer_->addPose(current_pose, result.path, this->now().seconds());

    }

    if (!shouldCreateKeyframe(*msg)) {
        return;
    }

    // Create keyframe logic (similar to RosbagReader)
    auto keyframe = std::make_shared<core::types::KeyFrame>();
    keyframe->id = ++current_keyframe_id_;
    keyframe->pose = conversions::toPose(*msg);

    auto msg_time = this->now();
    auto color_msg = findClosestMessage(color_messages_, msg_time, 0.1);
    auto camera_info_msg = findClosestMessage(camera_info_messages_, msg_time, 0.1);

    if (color_msg && camera_info_msg) {
        keyframe->color_data = conversions::toImage(*color_msg);
        keyframe->camera_info = conversions::toCameraInfo(*camera_info_msg);
    }

    // Add factors and optimize (similar to RosbagReader)
    if (current_keyframe_id_ > 1) {
        core::types::Factor odom_factor;
        odom_factor.id = current_keyframe_id_;
        odom_factor.type = core::proto::FactorType::ODOMETRY;
        odom_factor.connected_nodes = {current_keyframe_id_ - 1, current_keyframe_id_};

        core::types::Pose relative_pose;
        relative_pose.position = keyframe->pose.position - last_keyframe_pose_.position;
        relative_pose.orientation =
            keyframe->pose.orientation * last_keyframe_pose_.orientation.inverse();

        odom_factor.measurement.emplace<0>(relative_pose);
        odom_factor.information = Eigen::Matrix<double, 6, 6>::Identity();

        graph_.addFactor(odom_factor);
        store_.addFactor(odom_factor);
    }

    graph_.addKeyFrame(keyframe);
    store_.addKeyFrame(keyframe);
    last_keyframe_pose_ = keyframe->pose;

    if (current_keyframe_id_ % 10 == 0) {
        optimizeAndStore();
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

    // Try to get and publish the transform chain
    try {
        // Get transform from base_link to camera from tf_buffer
        auto base_to_camera = tf_tree_.getTransform("base_link", camera_frame_id_);

        if (visualizer_ && visualizer_->isConnected()) {
            // Convert base_link -> camera transform
            core::types::Pose camera_pose;
            camera_pose.position = base_to_camera.transform.translation();
            Eigen::Quaterniond q(base_to_camera.transform.rotation());
            camera_pose.orientation = q;

            // Publish transforms maintaining the hierarchy
            std::string camera_path = config_.odom_frame_id + "/" + base_to_camera.path;

            visualizer_->addPose(camera_pose, camera_path, this->now().seconds());

            // Add the image under the camera transform
            std::string image_path = camera_path + "/image";
            cv::Mat image = conversions::toOpenCVImage(*msg);
            visualizer_->addImage(image, image_path, rclcpp::Time(msg->header.stamp).seconds());
        }
    } catch (const std::runtime_error& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform from TransformTree: %s", e.what());
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform from tf_buffer: %s", e.what());
    }

    color_messages_.push_back(msg);
    while (color_messages_.size() > 100) {
        color_messages_.pop_front();
    }
}

void RosAdapter::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (!tf_tree_built_) {
        RCLCPP_DEBUG(this->get_logger(), "Skipping camera info message - waiting for transform tree");
        return;
    }
    camera_info_messages_.push_back(msg);

    // Visualize camera using Rerun's pinhole camera
    if (visualizer_ && visualizer_->isConnected()) {
        // Check if principal point is not at (0,0)
        if (std::abs(msg->k[2]) > 1e-6 || std::abs(msg->k[5]) > 1e-6) {
            RCLCPP_DEBUG(this->get_logger(),
                "Non-zero principal point (%.2f, %.2f) detected. Rerun visualization may be inaccurate.",
                msg->k[2], msg->k[5]);
        }

        // Create pinhole camera using from_focal_length_and_resolution
        auto camera = rerun::archetypes::Pinhole::from_focal_length_and_resolution(
            {static_cast<float>(msg->k[0]), static_cast<float>(msg->k[4])},  // focal lengths (fx, fy)
            {static_cast<float>(msg->width), static_cast<float>(msg->height)}  // resolution (uint32_t)
        );

        stf::TransformTree::TransformResult base_to_camera;
        try {
            base_to_camera = tf_tree_.getTransform(config_.base_link_frame_id, camera_frame_id_);
        } catch (const std::runtime_error& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to get transform from %s to %s: %s",
                       config_.base_link_frame_id.c_str(), camera_frame_id_.c_str(), e.what());
            return;
        }
        std::string camera_path = config_.odom_frame_id + "/" + base_to_camera.path;
        visualizer_->addCamera(camera, camera_path, rclcpp::Time(msg->header.stamp).seconds());
    }

    while (camera_info_messages_.size() > 100) {
        camera_info_messages_.pop_front();
    }
}

template <typename T>
std::shared_ptr<T> RosAdapter::findClosestMessage(std::deque<std::shared_ptr<T>>& messages,
                                                     const rclcpp::Time& target_time,
                                                     double max_time_diff) {
    while (!messages.empty()) {
        auto& msg = messages.front();
        double time_diff = std::abs((rclcpp::Time(msg->header.stamp) - target_time).seconds());

        if (time_diff <= max_time_diff) {
            auto result = msg;
            messages.pop_front();
            return result;
        } else if (rclcpp::Time(msg->header.stamp) < target_time) {
            messages.pop_front();
        } else {
            break;
        }
    }
    return nullptr;
}

bool RosAdapter::shouldCreateKeyframe(const nav_msgs::msg::Odometry& current_odom) {
    if (current_keyframe_id_ == 0)
        return true;

    double dx = current_odom.pose.pose.position.x - last_keyframe_pose_.position.x();
    double dy = current_odom.pose.pose.position.y - last_keyframe_pose_.position.y();
    double dz = current_odom.pose.pose.position.z - last_keyframe_pose_.position.z();

    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    return distance >= config_.keyframe_distance_threshold;
}

void RosAdapter::optimizeAndStore() {
    if (graph_.optimize()) {
        auto optimized_keyframes = graph_.getAllKeyFrames();
        for (const auto& kf : optimized_keyframes) {
            store_.addKeyFrame(kf);
        }
    }
}

void RosAdapter::printAvailableTopics() {
    auto topic_names_and_types = this->get_topic_names_and_types();
    RCLCPP_DEBUG(this->get_logger(), "Available topics:");
    for (const auto& topic : topic_names_and_types) {
        std::string types_str;
        for (const auto& type : topic.second) {
            if (!types_str.empty()) types_str += ", ";
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

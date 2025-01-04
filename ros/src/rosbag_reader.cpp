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
    : bagfile_(bagfile), config_(config), graph_(graph), store_(store), visualizer_(visualizer) {}

bool RosbagReader::initialize() {
    if (!std::filesystem::exists(bagfile_)) {
        return false;
    }

    try {
        reader_ = std::make_unique<rosbag2_cpp::Reader>();
        reader_->open(bagfile_);

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

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to open rosbag: " << e.what() << std::endl;
        return false;
    }
}

bool RosbagReader::shouldCreateKeyframe(const nav_msgs::msg::Odometry& current_odom) {
    if (current_keyframe_id_ == 0)
        return true;

    // Calculate distance from last keyframe
    double dx = current_odom.pose.pose.position.x - last_keyframe_pose_.position.x();
    double dy = current_odom.pose.pose.position.y - last_keyframe_pose_.position.y();
    double dz = current_odom.pose.pose.position.z - last_keyframe_pose_.position.z();

    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    return distance >= config_.keyframe_distance_threshold;
}

template <typename T>
std::shared_ptr<T> RosbagReader::findClosestMessage(std::deque<std::shared_ptr<T>>& messages,
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

void RosbagReader::processStaticTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg) {
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

        // Create entity path based on frame ids
        std::string entity_path = transform.header.frame_id + "/" + transform.child_frame_id;
        static_transforms_.push_back(std::make_pair(
            std::make_pair(transform.header.frame_id, transform.child_frame_id), pose));
        visualizer_->addPose(pose, entity_path, rclcpp::Time(transform.header.stamp).seconds());
    }
}

void RosbagReader::processTransform(const std::shared_ptr<tf2_msgs::msg::TFMessage> msg) {
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

        // Create entity path based on frame ids
        std::string entity_path = transform.header.frame_id + "/" + transform.child_frame_id;
        transforms_.push_back(std::make_pair(
            std::make_pair(transform.header.frame_id, transform.child_frame_id), pose));
        // visualizer_->addPose(pose, entity_path, rclcpp::Time(msg->header.stamp).seconds());
    }
}

void RosbagReader::processOdometry(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    // Visualize odometry message immediately if visualizer is available
    if (visualizer_ && visualizer_->isConnected()) {
        std::cout << "Sending odometry message to visualizer" << std::endl;
        core::types::Pose current_pose = conversions::toPose(*msg);
        std::string entity_path = msg->child_frame_id;
        // visualizer_->setTimestamp(rclcpp::Time(msg->header.stamp).seconds());
        // visualizer_->addPose(current_pose, entity_path,
        // rclcpp::Time(msg->header.stamp).seconds());

        // for (const auto& transform : static_transforms_) {
        //     visualizer_->addPose(transform.second, transform.first.first + "/" +
        //     transform.first.second, rclcpp::Time(msg->header.stamp).seconds());
        // }
        // for (const auto& transform : transforms_) {
        //     visualizer_->addPose(transform.second, transform.first.first + "/" +
        //     transform.first.second, rclcpp::Time(msg->header.stamp).seconds());
        // }
        // visualizer_->update();
    }

    if (!shouldCreateKeyframe(*msg)) {
        return;
    }

    // Create a new keyframe
    auto keyframe = std::make_shared<core::types::KeyFrame>();
    keyframe->id = ++current_keyframe_id_;
    keyframe->pose = conversions::toPose(*msg);

    // Find closest color image and camera info
    auto msg_time = rclcpp::Time(msg->header.stamp);
    auto color_msg = findClosestMessage(color_messages_, msg_time, 0.1);
    auto camera_info_msg = findClosestMessage(camera_info_messages_, msg_time, 0.1);

    if (color_msg && camera_info_msg) {
        keyframe->color_data = conversions::toImage(*color_msg);
        keyframe->camera_info = conversions::toCameraInfo(*camera_info_msg);
    }

    // Create odometry factor between consecutive keyframes
    if (current_keyframe_id_ > 1) {
        core::types::Factor odom_factor;
        odom_factor.id = current_keyframe_id_;
        odom_factor.type = core::proto::FactorType::ODOMETRY;
        odom_factor.connected_nodes = {current_keyframe_id_ - 1, current_keyframe_id_};

        // Calculate relative pose between consecutive keyframes
        core::types::Pose relative_pose;
        relative_pose.position = keyframe->pose.position - last_keyframe_pose_.position;
        relative_pose.orientation =
            keyframe->pose.orientation * last_keyframe_pose_.orientation.inverse();

        odom_factor.measurement.emplace<0>(relative_pose);
        odom_factor.information = Eigen::Matrix<double, 6, 6>::Identity();

        graph_.addFactor(odom_factor);
        store_.addFactor(odom_factor);
    } else if (current_keyframe_id_ == 1) {
        // Add prior factor for first keyframe
        core::types::Factor prior;
        prior.id = 0;
        prior.type = core::proto::FactorType::PRIOR;
        prior.connected_nodes = {1};
        prior.measurement.emplace<0>(keyframe->pose);
        prior.information = Eigen::Matrix<double, 6, 6>::Identity();

        graph_.addFactor(prior);
        store_.addFactor(prior);
    }

    // Store keyframe and update tracking
    graph_.addKeyFrame(keyframe);
    store_.addKeyFrame(keyframe);
    last_keyframe_pose_ = keyframe->pose;

    if (current_keyframe_id_ % 10 == 0) {
        optimizeAndStore();
    }

    // Visualize keyframe data when created
    // if (visualizer_ && visualizer_->isConnected()) {
    //     if (color_msg) {
    //         std::string image_path = "images/" + color_msg->header.frame_id;
    //         visualizer_->addImage(keyframe->color_data->toCvMat(),
    //                             image_path,
    //                             rclcpp::Time(color_msg->header.stamp).seconds());
    //     }
    //     visualizer_->update();
    // }
}

bool RosbagReader::processBag() {
    while (reader_->has_next()) {
        auto bag_msg = reader_->read_next();

        // Update visualization timestamp based on message timestamp
        if (bag_msg->topic_name == config_.tf_static_topic) {
            auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
            rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            serialization.deserialize_message(&serialized_msg, tf_msg.get());
            processStaticTransform(tf_msg);
        } else if (bag_msg->topic_name == config_.tf_topic) {
            auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
            rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            serialization.deserialize_message(&serialized_msg, tf_msg.get());
            processTransform(tf_msg);
        } else if (bag_msg->topic_name == config_.odom_topic) {
            auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
            rclcpp::Serialization<nav_msgs::msg::Odometry> serialization;
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            serialization.deserialize_message(&serialized_msg, odom_msg.get());
            processOdometry(odom_msg);
        } else if (bag_msg->topic_name == config_.color_topic) {
            auto color_msg = std::make_shared<sensor_msgs::msg::Image>();
            rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            serialization.deserialize_message(&serialized_msg, color_msg.get());
            color_messages_.push_back(color_msg);
        } else if (bag_msg->topic_name == config_.camera_info_topic) {
            auto info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
            rclcpp::Serialization<sensor_msgs::msg::CameraInfo> serialization;
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            serialization.deserialize_message(&serialized_msg, info_msg.get());
            camera_info_messages_.push_back(info_msg);
        }
    }

    optimizeAndStore();
    core::graph::util::dumpFactorGraph(graph_, "/mnt/remote-storage/final_graph.vtk");
    return store_.save("/tmp/final_map.pb");
}

void RosbagReader::optimizeAndStore() {
    if (graph_.optimize()) {
        auto optimized_keyframes = graph_.getAllKeyFrames();
        for (const auto& kf : optimized_keyframes) {
            store_.addKeyFrame(kf);
        }
    }
}

}  // namespace ros

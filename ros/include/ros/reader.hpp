#pragma once

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <core/graph/factor_graph.hpp>
#include <core/storage/map_store.hpp>
#include "converter.hpp"

namespace ros {
namespace reader {

struct BagReaderConfig {
    std::string odometry_topic = "/odom";
    std::string image_topic = "/camera/color/image_raw";
    std::string depth_topic = "/camera/depth/image_raw";
    std::string camera_info_topic = "/camera/color/camera_info";
    std::string pointcloud_topic = "/points";
    bool use_pointcloud = false;  // If true, use pointcloud instead of depth image
    double min_keyframe_distance = 0.5;  // Minimum distance between keyframes
    double min_keyframe_rotation = 0.5;  // Minimum rotation between keyframes in radians
};

class BagReader {
public:
    explicit BagReader(const BagReaderConfig& config) : config_(config) {}

    bool processBag(const std::string& bagfile, 
                   core::graph::FactorGraph& graph,
                   core::storage::MapStore& map_store) {
        rosbag::Bag bag;
        try {
            bag.open(bagfile, rosbag::bagmode::Read);
        } catch (const rosbag::BagException& e) {
            std::cerr << "Error opening bag file: " << e.what() << std::endl;
            return false;
        }

        std::vector<std::string> topics = {
            config_.odometry_topic,
            config_.image_topic,
            config_.camera_info_topic
        };
        if (config_.use_pointcloud) {
            topics.push_back(config_.pointcloud_topic);
        } else {
            topics.push_back(config_.depth_topic);
        }

        rosbag::View view(bag, rosbag::TopicQuery(topics));
        
        geometry_msgs::PoseStampedConstPtr last_pose;
        core::types::KeyFrame::Ptr last_keyframe;
        uint64_t frame_id = 0;

        for (const rosbag::MessageInstance& msg : view) {
            if (msg.getTopic() == config_.odometry_topic) {
                auto odom_msg = msg.instantiate<geometry_msgs::PoseStamped>();
                if (!odom_msg) continue;

                if (shouldCreateKeyFrame(odom_msg, last_pose)) {
                    auto keyframe = std::make_shared<core::types::KeyFrame>();
                    keyframe->id = frame_id++;
                    keyframe->pose = converter::MessageConverter::fromPoseMsg(*odom_msg);
                    
                    // Add to graph and create factors
                    graph.addKeyFrame(keyframe);
                    if (last_keyframe) {
                        core::types::Factor odom_factor;
                        odom_factor.type = core::proto::FactorType::ODOMETRY;
                        odom_factor.connected_nodes = {last_keyframe->id, keyframe->id};
                        odom_factor.measurement = last_keyframe->pose.inverse() * keyframe->pose;
                        odom_factor.information = Eigen::Matrix<double, 6, 6>::Identity();
                        graph.addFactor(odom_factor);
                    }

                    map_store.addKeyFrame(keyframe);
                    last_keyframe = keyframe;
                    last_pose = odom_msg;
                }
            }
            // Handle other message types...
        }

        bag.close();
        return true;
    }

private:
    BagReaderConfig config_;

    bool shouldCreateKeyFrame(const geometry_msgs::PoseStampedConstPtr& current_pose,
                            const geometry_msgs::PoseStampedConstPtr& last_pose) {
        if (!last_pose) return true;

        Eigen::Vector3d curr_pos(
            current_pose->pose.position.x,
            current_pose->pose.position.y,
            current_pose->pose.position.z
        );
        Eigen::Vector3d last_pos(
            last_pose->pose.position.x,
            last_pose->pose.position.y,
            last_pose->pose.position.z
        );

        double distance = (curr_pos - last_pos).norm();
        if (distance >= config_.min_keyframe_distance) return true;

        Eigen::Quaterniond curr_rot(
            current_pose->pose.orientation.w,
            current_pose->pose.orientation.x,
            current_pose->pose.orientation.y,
            current_pose->pose.orientation.z
        );
        Eigen::Quaterniond last_rot(
            last_pose->pose.orientation.w,
            last_pose->pose.orientation.x,
            last_pose->pose.orientation.y,
            last_pose->pose.orientation.z
        );

        double angle = Eigen::AngleAxisd(curr_rot * last_rot.inverse()).angle();
        return angle >= config_.min_keyframe_rotation;
    }
};

} // namespace reader
} // namespace ros
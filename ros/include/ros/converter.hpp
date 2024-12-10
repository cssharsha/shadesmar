#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <core/types/keyframe.hpp>
#include <core/types/pose.hpp>
#include <cv_bridge/cv_bridge.h>

namespace ros {
namespace converter {

class MessageConverter {
public:
    static core::types::Pose fromPoseMsg(const geometry_msgs::PoseStamped& msg) {
        core::types::Pose pose;
        pose.position = Eigen::Vector3d(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        );
        pose.orientation = Eigen::Quaterniond(
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        );
        pose.timestamp = msg.header.stamp.toSec();
        return pose;
    }

    static core::types::PointCloud fromPointCloud2Msg(const sensor_msgs::PointCloud2& msg) {
        core::types::PointCloud cloud;
        
        // Get field offsets
        int x_idx = -1, y_idx = -1, z_idx = -1;
        int r_idx = -1, g_idx = -1, b_idx = -1;
        
        for (size_t i = 0; i < msg.fields.size(); ++i) {
            if (msg.fields[i].name == "x") x_idx = msg.fields[i].offset;
            if (msg.fields[i].name == "y") y_idx = msg.fields[i].offset;
            if (msg.fields[i].name == "z") z_idx = msg.fields[i].offset;
            if (msg.fields[i].name == "r") r_idx = msg.fields[i].offset;
            if (msg.fields[i].name == "g") g_idx = msg.fields[i].offset;
            if (msg.fields[i].name == "b") b_idx = msg.fields[i].offset;
        }

        cloud.points.reserve(msg.width * msg.height);
        if (r_idx >= 0) cloud.colors.reserve(msg.width * msg.height);

        for (size_t i = 0; i < msg.width * msg.height; ++i) {
            const uint8_t* ptr = &msg.data[i * msg.point_step];
            
            float x = *reinterpret_cast<const float*>(ptr + x_idx);
            float y = *reinterpret_cast<const float*>(ptr + y_idx);
            float z = *reinterpret_cast<const float*>(ptr + z_idx);
            
            cloud.points.emplace_back(x, y, z);

            if (r_idx >= 0) {
                float r = ptr[r_idx] / 255.0f;
                float g = ptr[g_idx] / 255.0f;
                float b = ptr[b_idx] / 255.0f;
                cloud.colors.emplace_back(r, g, b);
            }
        }

        return cloud;
    }

    static cv::Mat fromImageMsg(const sensor_msgs::Image& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            return cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
        }
    }
};

} // namespace converter
} // namespace ros
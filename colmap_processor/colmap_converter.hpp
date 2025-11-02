#pragma once

#include <fstream>
#include <memory>
#include <string>

#include <colmap/scene/reconstruction.h>

#include <glog/logging.h>
#include <core/storage/map_store.hpp>

#include "core/types/image.hpp"
#include "core/types/keyframe.hpp"
#include "core/types/pose.hpp"

namespace gs {

class ColmapConverter {
public:
    ColmapConverter(const std::string& colmap_path, const std::string& map_path);
    ColmapConverter(const std::string& colmap_path);
    bool convert();
    bool convertFromText();
    void justQuery() {
        auto all_keyframes = map_store_.getAllKeyFrames();
        LOG(INFO) << "Total keyframes: " << all_keyframes.size();

        auto all_keypoints = map_store_.getAllKeyPoints();
        LOG(INFO) << "Total keypoints: " << all_keypoints.size();
    }

private:
    // Debug helpers: write intermediate rotations/quaternions/translations to JSON files
    void writeDebugPoseJson(const std::string& stage, uint64_t image_id,
                            const Eigen::Quaterniond& q_xyzw, const Eigen::Vector3d& t,
                            const Eigen::Matrix3d& R_world_to_cam,
                            const Eigen::Isometry3d& world_T_cam,
                            const Eigen::Isometry3d& cam_from_world);

    // Aggregated debug for images.txt parsing
    void debugStartImagesAggregate();
    void debugAppendImageEntry(uint64_t image_id, const Eigen::Quaterniond& q_xyzw,
                               const Eigen::Vector3d& t, const Eigen::Matrix3d& R_world_to_cam,
                               const Eigen::Isometry3d& world_T_cam,
                               const Eigen::Isometry3d& cam_from_world);
    void debugFinishImagesAggregate();
    std::ostringstream debug_images_agg_;
    bool debug_images_agg_open_ = false;
    bool debug_images_first_ = true;

    bool parseCameraFile(const std::string& camera_file, core::types::CameraInfo& camera_info);
    bool parseCameraInfo(std::stringstream& line, core::types::CameraInfo& camera_info);
    bool parseImageLine(std::stringstream& line, core::types::KeyFrame::Ptr& keyframe);
    bool parseImageFile(const std::string& image_file, const core::types::CameraInfo& camera_info,
                        std::unordered_map<uint64_t, core::types::KeyFrame::Ptr>& keyframes);
    bool parsePointLine(std::stringstream& point_file);
    bool parsePointFile(const std::string& point_file);
    bool convertImageToKeyframe(const uint32_t& image_id, core::types::KeyFrame::Ptr& keyframe);
    core::types::Pose T_R_C_;
    std::string colmap_path_;
    core::types::CameraInfo camera_info_;
    std::string map_path_;
    colmap::Reconstruction reconstruction_;
    core::storage::MapStore map_store_;

    // Binary reconstruction support
    bool use_binary_ = false;  // True if binary files are available and loaded

    // Store 2D points from images.txt when binary is not available
    // Map: image_id -> vector of 2D points (indexed by point2D_idx)
    std::unordered_map<uint64_t, std::vector<Eigen::Vector2d>> image_to_points2d_;
};

}  // namespace gs

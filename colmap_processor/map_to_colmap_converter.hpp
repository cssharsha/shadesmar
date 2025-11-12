#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <glog/logging.h>
#include <core/storage/map_store.hpp>
#include <opencv2/opencv.hpp>

#include "core/types/image.hpp"
#include "core/types/keyframe.hpp"
#include "core/types/keypoint.hpp"
#include "core/types/pose.hpp"
#include "utils/stf/transform_tree.hpp"

namespace gs {

/**
 * @brief Converts internal map store format to COLMAP text format
 *
 * This converter reads from MapStore and exports:
 * - cameras.txt: Camera intrinsics
 * - images.txt: Camera poses and 2D observations
 * - points3D.txt: 3D points with tracks
 */
class MapToColmapConverter {
public:
    /**
     * @brief Constructor
     * @param map_path Path to the map store
     * @param output_path Path where COLMAP text files will be written
     */
    MapToColmapConverter(const std::string& map_path, const std::string& output_path);

    /**
     * @brief Convert map store to COLMAP format
     * @return true if conversion successful, false otherwise
     */
    bool convert();

    /**
     * @brief Set camera model for export (default: PINHOLE)
     * @param model Camera model name (PINHOLE, SIMPLE_RADIAL, etc.)
     */
    void setCameraModel(const std::string& model) { camera_model_ = model; }

private:
    /**
     * @brief Write cameras.txt file
     * @return true if successful
     */
    bool writeCameraFile();

    /**
     * @brief Write images.txt file
     * @return true if successful
     */
    bool writeImageFile();

    /**
     * @brief Write points3D.txt file
     * @return true if successful
     */
    bool writePointFile();

    /**
     * @brief Extract camera parameters from CameraInfo
     * @param camera_info Input camera info
     * @param params Output parameters vector
     * @return true if extraction successful
     */
    bool extractCameraParams(const core::types::CameraInfo& camera_info,
                            std::vector<double>& params);

    /**
     * @brief Convert pose to COLMAP format (cam_from_world)
     * The COLMAP format represents the transformation from world to camera
     * @param pose Input pose (world_T_cam)
     * @param qw Output quaternion w component
     * @param qx Output quaternion x component
     * @param qy Output quaternion y component
     * @param qz Output quaternion z component
     * @param tx Output translation x component
     * @param ty Output translation y component
     * @param tz Output translation z component
     */
    void poseToColmapFormat(const core::types::Pose& pose,
                           double& qw, double& qx, double& qy, double& qz,
                           double& tx, double& ty, double& tz);

    /**
     * @brief Build index mapping keyframe IDs to image indices
     * This is needed to map 2D point observations to image indices in points3D.txt
     */
    void buildKeyframeIndex();

    /**
     * @brief Build reverse mapping from (keyframe_id, pixel) to point2D_idx
     * This is needed to write the TRACK[] data in points3D.txt
     */
    void buildPoint2DIndex();

    /**
     * @brief Get the static transform from base_link to camera frame
     * @param camera_frame_id The camera frame ID from keyframe
     * @param base_T_camera Output transform from base_link to camera
     * @return true if transform was found
     */
    bool getBaseToCameraTF(const std::string& camera_frame_id, Eigen::Isometry3d& base_T_camera);

    /**
     * @brief Write images from keyframes to output directory
     * @return true if successful
     */
    bool writeImages();

    std::string map_path_;
    std::string output_path_;
    core::storage::MapStore map_store_;

    std::string camera_model_;  // Default: PINHOLE

    // Cached data
    std::vector<core::types::KeyFrame::Ptr> keyframes_;
    std::vector<core::types::Keypoint> keypoints_;

    // Index mappings
    std::unordered_map<uint64_t, size_t> keyframe_id_to_index_;  // keyframe_id -> index in keyframes_

    // Map from (keyframe_id, 2D point observation) to point2D_idx
    // This is used to build the TRACK data for points3D.txt
    // Key: keyframe_id, Value: map from (x,y) to point2D_idx
    std::unordered_map<uint64_t, std::unordered_map<std::string, uint32_t>> point2d_index_;

    // Static transform from base_link to camera
    Eigen::Isometry3d base_T_camera_;
    bool has_base_T_camera_ = false;
};

}  // namespace gs

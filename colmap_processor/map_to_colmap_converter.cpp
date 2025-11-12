#include "map_to_colmap_converter.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

namespace gs {

MapToColmapConverter::MapToColmapConverter(const std::string& map_path,
                                           const std::string& output_path)
    : map_path_(map_path),
      output_path_(output_path),
      map_store_(map_path, core::storage::ProcessRole::DUAL),
      camera_model_("PINHOLE") {
    map_store_.syncIndexFromDisk();
}

bool MapToColmapConverter::convert() {
    LOG(INFO) << "Starting conversion from map store to COLMAP format";
    LOG(INFO) << "Map path: " << map_path_;
    LOG(INFO) << "Output path: " << output_path_;

    // Create output directory structure
    std::filesystem::path output_dir(output_path_);
    std::filesystem::path text_dir = output_dir / "text";
    std::filesystem::path images_dir = output_dir / "images";
    std::filesystem::create_directories(text_dir);
    std::filesystem::create_directories(images_dir);

    // Load all keyframes and keypoints from map store
    auto all_keyframes = map_store_.getAllKeyFrames();
    keypoints_ = map_store_.getAllKeyPoints();

    LOG(INFO) << "Loaded " << all_keyframes.size() << " keyframes";
    LOG(INFO) << "Loaded " << keypoints_.size() << " keypoints";

    if (all_keyframes.empty()) {
        LOG(ERROR) << "No keyframes found in map store";
        return false;
    }

    // Filter keyframes to only include those with valid image data
    keyframes_.clear();
    std::set<uint64_t> valid_keyframe_ids;
    size_t skipped_keyframes = 0;

    for (const auto& kf : all_keyframes) {
        // Only include keyframes that have color_data and non-empty image
        if (kf->color_data.has_value() && !kf->color_data.value().data.empty()) {
            keyframes_.push_back(kf);
            valid_keyframe_ids.insert(kf->id);
        } else {
            skipped_keyframes++;
        }
    }

    LOG(INFO) << "Filtered to " << keyframes_.size() << " keyframes with valid images";
    if (skipped_keyframes > 0) {
        LOG(WARNING) << "Skipped " << skipped_keyframes << " keyframes without valid image data";
    }

    if (keyframes_.empty()) {
        LOG(ERROR) << "No keyframes with valid image data found";
        return false;
    }

    // Filter keypoints to only include those with observations from valid keyframes
    // and at least 2 observations
    std::vector<core::types::Keypoint> filtered_keypoints;
    size_t skipped_points = 0;
    size_t skipped_observations = 0;

    for (const auto& kp : keypoints_) {
        core::types::Keypoint filtered_kp = kp;
        filtered_kp.locations.clear();

        // Only include observations from valid keyframes
        for (const auto& loc : kp.locations) {
            if (valid_keyframe_ids.count(loc.keyframe_id) > 0) {
                filtered_kp.locations.push_back(loc);
            } else {
                skipped_observations++;
            }
        }

        // Only include point if it has at least 2 observations from valid keyframes
        if (filtered_kp.locations.size() >= 2) {
            filtered_keypoints.push_back(filtered_kp);
        } else {
            skipped_points++;
        }
    }

    keypoints_ = filtered_keypoints;
    LOG(INFO) << "Filtered to " << keypoints_.size() << " points with valid observations";
    if (skipped_points > 0) {
        LOG(WARNING) << "Skipped " << skipped_points << " points with insufficient observations";
    }
    if (skipped_observations > 0) {
        LOG(WARNING) << "Removed " << skipped_observations << " observations from invalid keyframes";
    }

    // Get the static TF from base_link to camera
    // The camera frame ID should be in the camera_info, not the pose frame_id (which is the world frame)
    std::string camera_frame_id;
    if (!keyframes_.empty() && keyframes_[0]->camera_info.has_value()) {
        camera_frame_id = keyframes_[0]->camera_info->frame_id;
        LOG(INFO) << "Camera frame ID from camera_info: " << camera_frame_id;

        auto transform_tree = map_store_.getTransformTree();
        if (transform_tree) {
            LOG(INFO) << "Transform tree structure:";
            transform_tree->printTree();
        }

        if (getBaseToCameraTF(camera_frame_id, base_T_camera_)) {
            has_base_T_camera_ = true;
            LOG(INFO) << "Found static TF from base_link to " << camera_frame_id;
            LOG(INFO) << "Transform:\n" << base_T_camera_.matrix();
        } else {
            LOG(WARNING) << "Could not get static TF from base_link to " << camera_frame_id
                        << ", poses will not be adjusted";
        }
    } else {
        LOG(WARNING) << "No camera info available, poses will not be adjusted";
    }

    // Build indices for efficient lookups
    buildKeyframeIndex();
    buildPoint2DIndex();

    // Write COLMAP text files
    if (!writeCameraFile()) {
        LOG(ERROR) << "Failed to write cameras.txt";
        return false;
    }

    if (!writeImageFile()) {
        LOG(ERROR) << "Failed to write images.txt";
        return false;
    }

    // Don't write points3D.txt - let COLMAP extract features and triangulate
    // The exported camera poses will serve as priors during reconstruction
    LOG(INFO) << "Skipping points3D.txt - COLMAP will extract features and triangulate";

    // Write images
    if (!writeImages()) {
        LOG(ERROR) << "Failed to write images";
        return false;
    }

    LOG(INFO) << "Conversion completed successfully";
    return true;
}

bool MapToColmapConverter::writeCameraFile() {
    std::filesystem::path camera_file = std::filesystem::path(output_path_) / "text" / "cameras.txt";
    std::ofstream out(camera_file);
    if (!out.is_open()) {
        LOG(ERROR) << "Failed to open " << camera_file << " for writing";
        return false;
    }

    // Write header
    out << "# Camera list with one line of data per camera:\n";
    out << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";
    out << "# Number of cameras: 1\n";

    // Assume all keyframes use the same camera (CAMERA_ID = 1)
    // Get camera info from first keyframe
    if (!keyframes_[0]->camera_info.has_value()) {
        LOG(ERROR) << "First keyframe has no camera info";
        return false;
    }

    const auto& camera_info = keyframes_[0]->camera_info.value();
    std::vector<double> params;
    if (!extractCameraParams(camera_info, params)) {
        LOG(ERROR) << "Failed to extract camera parameters";
        return false;
    }

    // Write camera line: CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]
    out << "1 " << camera_model_ << " " << camera_info.width << " " << camera_info.height;
    for (const auto& param : params) {
        out << " " << std::setprecision(6) << param;
    }
    out << "\n";

    out.close();
    LOG(INFO) << "Wrote " << camera_file;
    return true;
}

bool MapToColmapConverter::writeImageFile() {
    std::filesystem::path image_file = std::filesystem::path(output_path_) / "text" / "images.txt";
    std::ofstream out(image_file);
    if (!out.is_open()) {
        LOG(ERROR) << "Failed to open " << image_file << " for writing";
        return false;
    }

    // Write header
    out << "# Image list with two lines of data per image:\n";
    out << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n";
    out << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n";

    // Calculate mean observations per image
    size_t total_observations = 0;
    for (const auto& kp : keypoints_) {
        total_observations += kp.locations.size();
    }
    double mean_observations = keyframes_.empty() ? 0.0
                              : static_cast<double>(total_observations) / keyframes_.size();
    out << "# Number of images: " << keyframes_.size()
        << ", mean observations per image: " << std::fixed << std::setprecision(1)
        << mean_observations << "\n";

    // Write each keyframe
    // Use sequential IMAGE_IDs (1, 2, 3, ...) with zero-padded filenames
    // so COLMAP's feature_extractor assigns matching IDs
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        const auto& kf = keyframes_[i];
        double qw, qx, qy, qz, tx, ty, tz;
        poseToColmapFormat(kf->pose, qw, qx, qy, qz, tx, ty, tz);

        // Line 1: IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME
        // Use sequential ID (i+1) so it matches COLMAP's feature_extractor
        // Use zero-padded filename so alphabetical order = numerical order
        uint64_t image_id = i + 1;
        std::ostringstream filename;
        filename << "frame_" << std::setw(6) << std::setfill('0') << image_id << ".jpg";
        std::string image_name = filename.str();

        out << image_id << " " << std::setprecision(9)
            << qw << " " << qx << " " << qy << " " << qz << " "
            << tx << " " << ty << " " << tz << " "
            << "1 " << image_name << "\n";

        // Line 2: Empty POINTS2D[] - COLMAP will populate during feature extraction
        out << "\n";
    }

    out.close();
    LOG(INFO) << "Wrote " << image_file;
    return true;
}

bool MapToColmapConverter::writePointFile() {
    std::filesystem::path point_file = std::filesystem::path(output_path_) / "text" / "points3D.txt";
    std::ofstream out(point_file);
    if (!out.is_open()) {
        LOG(ERROR) << "Failed to open " << point_file << " for writing";
        return false;
    }

    // Write header
    out << "# 3D point list with one line of data per point:\n";
    out << "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n";

    // Calculate mean track length
    size_t total_track_length = 0;
    for (const auto& kp : keypoints_) {
        total_track_length += kp.locations.size();
    }
    double mean_track_length = keypoints_.empty() ? 0.0
                               : static_cast<double>(total_track_length) / keypoints_.size();
    out << "# Number of points: " << keypoints_.size()
        << ", mean track length: " << std::fixed << std::setprecision(5)
        << mean_track_length << "\n";

    // Write each 3D point
    for (const auto& kp : keypoints_) {
        // POINT3D_ID X Y Z
        out << kp.id() << " " << std::setprecision(6)
            << kp.position.x() << " " << kp.position.y() << " " << kp.position.z() << " ";

        // R G B
        uint8_t r = 128, g = 128, b = 128;  // Default gray
        if (kp.color.x() >= 0 && kp.color.y() >= 0 && kp.color.z() >= 0) {
            r = static_cast<uint8_t>(kp.color.x());
            g = static_cast<uint8_t>(kp.color.y());
            b = static_cast<uint8_t>(kp.color.z());
        }
        out << static_cast<int>(r) << " " << static_cast<int>(g) << " " << static_cast<int>(b) << " ";

        // ERROR (reprojection error, use 0.0 as default)
        out << "0.0 ";

        // TRACK[] as (IMAGE_ID, POINT2D_IDX)
        // For each location/observation, we need the IMAGE_ID and the index of this 2D point
        // within that image's point list
        for (size_t i = 0; i < kp.locations.size(); ++i) {
            const auto& loc = kp.locations[i];

            // IMAGE_ID is the keyframe_id
            out << loc.keyframe_id << " ";

            // POINT2D_IDX is the index of this observation in the image's 2D point list
            // We need to look this up from our point2d_index_
            std::string key = std::to_string(static_cast<int>(loc.x)) + "_" +
                             std::to_string(static_cast<int>(loc.y));

            uint32_t point2d_idx = 0;
            auto it = point2d_index_.find(loc.keyframe_id);
            if (it != point2d_index_.end()) {
                auto it2 = it->second.find(key);
                if (it2 != it->second.end()) {
                    point2d_idx = it2->second;
                }
            }

            out << point2d_idx;
            if (i < kp.locations.size() - 1) {
                out << " ";
            }
        }
        out << "\n";
    }

    out.close();
    LOG(INFO) << "Wrote " << point_file;
    return true;
}

bool MapToColmapConverter::extractCameraParams(const core::types::CameraInfo& camera_info,
                                              std::vector<double>& params) {
    params.clear();

    if (camera_info.k.empty()) {
        LOG(ERROR) << "Camera intrinsics matrix is empty";
        return false;
    }

    // camera_info.k is stored as a flat 9-element vector (row-major 3x3 matrix)
    // K = [fx  0 cx]
    //     [ 0 fy cy]
    //     [ 0  0  1]

    if (camera_info.k.size() != 9) {
        LOG(ERROR) << "Invalid camera intrinsics size: " << camera_info.k.size();
        return false;
    }

    double fx = camera_info.k[0];
    double fy = camera_info.k[4];
    double cx = camera_info.k[2];
    double cy = camera_info.k[5];

    if (camera_model_ == "PINHOLE") {
        // PINHOLE: fx, fy, cx, cy
        params = {fx, fy, cx, cy};
    } else if (camera_model_ == "SIMPLE_RADIAL") {
        // SIMPLE_RADIAL: f, cx, cy, k
        // Use average of fx and fy for focal length
        double f = (fx + fy) / 2.0;
        double k = camera_info.d.empty() ? 0.0 : camera_info.d[0];
        params = {f, cx, cy, k};
    } else if (camera_model_ == "RADIAL") {
        // RADIAL: f, cx, cy, k1, k2
        double f = (fx + fy) / 2.0;
        double k1 = camera_info.d.size() > 0 ? camera_info.d[0] : 0.0;
        double k2 = camera_info.d.size() > 1 ? camera_info.d[1] : 0.0;
        params = {f, cx, cy, k1, k2};
    } else {
        LOG(ERROR) << "Unsupported camera model: " << camera_model_;
        return false;
    }

    return true;
}

void MapToColmapConverter::poseToColmapFormat(const core::types::Pose& pose,
                                             double& qw, double& qx, double& qy, double& qz,
                                             double& tx, double& ty, double& tz) {
    // Our pose represents world_T_base (base_link pose in world frame)
    // COLMAP expects cam_from_world (transformation from world to camera)

    // Create transformation matrix from pose
    Eigen::Isometry3d world_T_base = Eigen::Isometry3d::Identity();
    world_T_base.rotate(pose.orientation);
    world_T_base.pretranslate(pose.position);

    // Apply static TF from base_link to camera if available
    Eigen::Isometry3d world_T_cam;
    if (has_base_T_camera_) {
        // world_T_camera = world_T_base * base_T_camera
        world_T_cam = world_T_base * base_T_camera_;
    } else {
        // If no static TF, assume pose is already in camera frame
        world_T_cam = world_T_base;
    }

    // Invert to get cam_from_world
    Eigen::Isometry3d cam_from_world = world_T_cam.inverse();

    // Extract quaternion (w, x, y, z) and translation
    Eigen::Quaterniond q(cam_from_world.rotation());
    q.normalize();

    qw = q.w();
    qx = q.x();
    qy = q.y();
    qz = q.z();

    Eigen::Vector3d t = cam_from_world.translation();
    tx = t.x();
    ty = t.y();
    tz = t.z();
}

void MapToColmapConverter::buildKeyframeIndex() {
    keyframe_id_to_index_.clear();
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        keyframe_id_to_index_[keyframes_[i]->id] = i;
    }
    LOG(INFO) << "Built keyframe index with " << keyframe_id_to_index_.size() << " entries";
}

void MapToColmapConverter::buildPoint2DIndex() {
    point2d_index_.clear();

    // For each keyframe, collect all 2D observations and assign indices
    for (const auto& kf : keyframes_) {
        std::vector<std::pair<float, float>> points_2d;  // (x, y)

        // Collect all 2D points for this keyframe
        for (const auto& kp : keypoints_) {
            for (const auto& loc : kp.locations) {
                if (loc.keyframe_id == kf->id) {
                    points_2d.push_back({loc.x, loc.y});
                }
            }
        }

        // Assign indices to each 2D point
        std::unordered_map<std::string, uint32_t> point_map;
        for (uint32_t i = 0; i < points_2d.size(); ++i) {
            std::string key = std::to_string(static_cast<int>(points_2d[i].first)) + "_" +
                             std::to_string(static_cast<int>(points_2d[i].second));
            point_map[key] = i;
        }

        point2d_index_[kf->id] = point_map;
    }

    LOG(INFO) << "Built point2D index for " << point2d_index_.size() << " keyframes";
}

bool MapToColmapConverter::getBaseToCameraTF(const std::string& camera_frame_id,
                                             Eigen::Isometry3d& base_T_camera) {
    auto transform_tree = map_store_.getTransformTree();
    if (!transform_tree) {
        LOG(WARNING) << "Transform tree not available";
        return false;
    }

    try {
        // Get transform from base_link to camera frame
        auto result = transform_tree->getTransform("base_link", camera_frame_id);
        base_T_camera = result.transform;
        LOG(INFO) << "Transform path: " << result.path;
        return true;
    } catch (const std::runtime_error& e) {
        LOG(WARNING) << "Could not get transform from base_link to " << camera_frame_id
                    << ": " << e.what();
        return false;
    }
}

bool MapToColmapConverter::writeImages() {
    std::filesystem::path images_dir = std::filesystem::path(output_path_) / "images";

    size_t images_written = 0;
    size_t images_failed = 0;

    for (size_t i = 0; i < keyframes_.size(); ++i) {
        const auto& kf = keyframes_[i];
        // All keyframes in keyframes_ are guaranteed to have valid image data
        // (filtered in convert() method)
        const auto& img = kf->color_data.value();
        cv::Mat cv_img = img.data;

        // If rgb8, convert to bgr8 for OpenCV (jpg encoding expects BGR)
        if (img.encoding == "rgb8") {
            cv::cvtColor(cv_img, cv_img, cv::COLOR_RGB2BGR);
        }

        // Write image file with sequential ID and zero-padding (matching images.txt)
        uint64_t image_id = i + 1;
        std::ostringstream filename;
        filename << "frame_" << std::setw(6) << std::setfill('0') << image_id << ".jpg";
        std::filesystem::path image_path = images_dir / filename.str();

        if (cv::imwrite(image_path.string(), cv_img)) {
            images_written++;
        } else {
            LOG(ERROR) << "Failed to write image: " << image_path;
            images_failed++;
        }
    }

    LOG(INFO) << "Wrote " << images_written << " images to " << images_dir;
    if (images_failed > 0) {
        LOG(ERROR) << "Failed to write " << images_failed << " images";
    }

    return images_written > 0;
}

}  // namespace gs

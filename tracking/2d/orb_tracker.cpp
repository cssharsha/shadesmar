#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// Eigen must be included before opencv2/core/eigen.hpp
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/eigen.hpp>

#include <core/types/keyframe.hpp>
#include <core/types/pose.hpp>
#include <logging/logging.hpp>
#include <stf/transform_utils.hpp>

#include <2d/orb_tracker.hpp>

namespace tracking {
namespace image {

// Helper function to check if a map point already has an observation in a keyframe
bool hasObservationInKeyframe(const core::types::Keypoint& map_point, uint64_t keyframe_id) {
    for (const auto& location : map_point.locations) {
        if (location.keyframe_id == keyframe_id) {
            return true;
        }
    }
    return false;
}

// Helper function to update existing observation or add new one
void addOrUpdateObservation(core::types::Keypoint& map_point, uint64_t keyframe_id,
                            const std::string& frame_id, float x, float y,
                            const std::string& source = "tracking") {
    // Check if observation already exists for this keyframe
    for (auto& location : map_point.locations) {
        if (location.keyframe_id == keyframe_id) {
            // Update existing observation (keep the first one, or use better criteria)
            LOG(WARNING) << "Skipping duplicate observation for map point " << map_point.id()
                         << " in keyframe " << keyframe_id << " from " << source;
            return;
        }
    }

    // Add new observation
    map_point.locations.push_back({keyframe_id, frame_id, x, y});
    // Only log for debugging when needed
    // LOG(INFO) << "Added observation for map point " << map_point.id()
    //           << " in keyframe " << keyframe_id << " from " << source;
}

// Enhanced observation management with quality-based selection
void addOrUpdateObservationWithQuality(core::types::Keypoint& map_point, uint64_t keyframe_id,
                                       const std::string& frame_id, float x, float y,
                                       double quality_score,
                                       const std::string& source = "tracking") {
    // Check if observation already exists for this keyframe
    for (auto it = map_point.locations.begin(); it != map_point.locations.end(); ++it) {
        if (it->keyframe_id == keyframe_id) {
            // For now, keep the first observation (can be enhanced with quality comparison)
            LOG(WARNING) << "Skipping duplicate observation for map point " << map_point.id()
                         << " in keyframe " << keyframe_id << " from " << source
                         << " (quality=" << quality_score << ")";

            // maybe add this if the new observation has better quality
            // if (quality_score > previous_quality) {
            //     *it = {keyframe_id, frame_id, x, y};
            //     LOG(INFO) << "Replaced observation with better quality";
            // }
            return;
        }
    }

    // Add new observation
    map_point.locations.push_back({keyframe_id, frame_id, x, y});
}

OrbTracker::OrbTracker(uint32_t num_features, float scal_factor, uint32_t levels)
    : num_features_(num_features),
      scale_factor_(scal_factor),
      n_levels_(levels),
      orb_detector_(cv::ORB::create(num_features_, scale_factor_, n_levels_)),
      matcher_(cv::BFMatcher::create(cv::NORM_HAMMING2)),
      base_link_frame_id_("base_link") {}

void OrbTracker::addCameraInfo(const core::types::CameraInfo& cam_info) {
    cam_infos_[cam_info.frame_id] = cam_info;
}

void OrbTracker::addCameraPose(const core::types::Pose& cam_pose, std::string camera_frame) {
    cam_poses_[camera_frame] = cam_pose;
}

// void drawEpipolarLines(const core::types::KeyFrame& prev_frame,
//                        const core::types::KeyFrame& cur_frame,
//                        Eigen::Isometry3d expected_relative_transform) {
//     // Compute fundamental matrix for visualization
//     Eigen::Matrix3d R = expected_relative_transform.rotation();
//     Eigen::Vector3d t = expected_relative_transform.translation();
//
//     cv::Mat R_cv(3, 3, CV_64F);
//     cv::Mat t_cv(3, 1, CV_64F);
//     for (int i = 0; i < 3; ++i) {
//         t_cv.at<double>(i) = t(i);
//         for (int j = 0; j < 3; ++j) {
//             R_cv.at<double>(i, j) = R(i, j);
//         }
//     }
//
//     cv::Mat t_x =
//         (cv::Mat_<double>(3, 3) << 0, -t_cv.at<double>(2), t_cv.at<double>(1),
//         t_cv.at<double>(2),
//          0, -t_cv.at<double>(0), -t_cv.at<double>(1), t_cv.at<double>(0), 0);
//
//     cv::Mat E = t_x * R_cv;
//     cv::Mat K_inv = K.inv();
//     cv::Mat F = K_inv.t() * E * K_inv;
//
//     // Create visualization images
//     cv::Mat prev_img_color = prev_frame.color_data.value().data.clone();
//     cv::Mat cur_img_color = cur_frame.color_data.value().data.clone();
//
//     // Sample some points for epipolar line visualization (max 20 for clarity)
//     int num_lines_to_draw = std::min(20, (int)filtered_matches.size());
//     int step = std::max(1, (int)filtered_matches.size() / num_lines_to_draw);
//
//     // Generate random colors for each correspondence
//     std::vector<cv::Scalar> colors;
//     for (int i = 0; i < num_lines_to_draw; ++i) {
//         colors.push_back(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
//     }
//
//     for (int i = 0; i < num_lines_to_draw; ++i) {
//         int idx = i * step;
//         if (idx >= filtered_matches.size())
//             break;
//
//         cv::Point2f pt_prev = prev_img_kps[filtered_matches[idx].queryIdx].pt;
//         cv::Point2f pt_cur = cur_img_kps[filtered_matches[idx].trainIdx].pt;
//
//         // Draw point in previous image
//         cv::circle(prev_img_color, pt_prev, 5, colors[i], -1);
//
//         // Compute epipolar line in current image: l = F * p_prev
//         cv::Mat p_prev = (cv::Mat_<double>(3, 1) << pt_prev.x, pt_prev.y, 1.0);
//         cv::Mat epiline = F * p_prev;
//
//         double a = epiline.at<double>(0);
//         double b = epiline.at<double>(1);
//         double c = epiline.at<double>(2);
//
//         // Draw epipolar line in current image: ax + by + c = 0
//         // Find two points on the line at image boundaries
//         int img_width = cur_img_color.cols;
//         int img_height = cur_img_color.rows;
//
//         cv::Point2f line_pt1, line_pt2;
//         if (std::abs(b) > 1e-6) {
//             // Line crosses left and right borders
//             line_pt1 = cv::Point2f(0, -(c + a * 0) / b);
//             line_pt2 = cv::Point2f(img_width - 1, -(c + a * (img_width - 1)) / b);
//         } else if (std::abs(a) > 1e-6) {
//             // Line crosses top and bottom borders
//             line_pt1 = cv::Point2f(-(c + b * 0) / a, 0);
//             line_pt2 = cv::Point2f(-(c + b * (img_height - 1)) / a, img_height - 1);
//         } else {
//             continue;  // Skip degenerate lines
//         }
//
//         // Draw the epipolar line
//         cv::line(cur_img_color, line_pt1, line_pt2, colors[i], 2);
//
//         // Draw the matched point in current image
//         cv::circle(cur_img_color, pt_cur, 5, colors[i], -1);
//
//         // Draw a small perpendicular line from point to epipolar line to show distance
//         double dist = std::abs(a * pt_cur.x + b * pt_cur.y + c) / std::sqrt(a * a + b * b);
//         if (dist > 0.5) {  // Only draw if distance is visible
//             // Find closest point on line to pt_cur
//             double x0 = pt_cur.x;
//             double y0 = pt_cur.y;
//             double closest_x = (b * (b * x0 - a * y0) - a * c) / (a * a + b * b);
//             double closest_y = (a * (-b * x0 + a * y0) - b * c) / (a * a + b * b);
//             cv::Point2f closest_pt(closest_x, closest_y);
//
//             cv::line(cur_img_color, pt_cur, closest_pt, colors[i], 1, cv::LINE_AA);
//         }
//     }
//
//     // Concatenate images side by side
//     cv::Mat epipolar_viz;
//     cv::hconcat(prev_img_color, cur_img_color, epipolar_viz);
//
//     // Add text labels
//     cv::putText(epipolar_viz, "Previous Frame (Points)", cv::Point(10, 30),
//                 cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
//     cv::putText(epipolar_viz, "Current Frame (Epipolar Lines)",
//                 cv::Point(prev_img_color.cols + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
//                 cv::Scalar(255, 255, 255), 2);
//
//     cv::imwrite("/data/robot/bags/house11/orb/epipolar/" + std::to_string(prev_frame.id) + "_" +
//                     std::to_string(cur_frame.id) + ".png",
//                 epipolar_viz);
//
//     LOG(INFO) << "Saved epipolar visualization with " << num_lines_to_draw << " correspondences";
// }

std::optional<core::types::Pose> OrbTracker::match(
    const core::types::KeyFrame& prev_frame, const core::types::KeyFrame& cur_frame,
    std::map<uint32_t, core::types::Keypoint>& map_keypoints,
    std::vector<Eigen::Vector3d>& world_points) {

    LOG(INFO) << "=== MATCH FUNCTION: Two-Stage Matching ===";
    LOG(INFO) << "Processing frames: prev=" << prev_frame.id << ", cur=" << cur_frame.id;
    LOG(INFO) << "Map size before matching: " << map_keypoints.size() << " keypoints";

    // Detect and compute features helper
    auto detectAndCompute = [&](const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                                cv::Mat& descriptor) {
        cv::Mat gray;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();
        }

        // Use canny edge detector and filter the detections only along these edges
        cv::Mat edge_mask;
        cv::Canny(gray, edge_mask, 100, 200, 3);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::dilate(gray, edge_mask, kernel);

        orb_detector_->detectAndCompute(gray, edge_mask, keypoints, descriptor);
    };

    // Detect features in both frames
    std::vector<cv::KeyPoint> cur_img_kps;
    cv::Mat cur_img_desc;
    detectAndCompute(cur_frame.color_data.value().data, cur_img_kps, cur_img_desc);

    std::vector<cv::KeyPoint> prev_img_kps;
    cv::Mat prev_img_desc;
    detectAndCompute(prev_frame.color_data.value().data, prev_img_kps, prev_img_desc);

    LOG(INFO) << "Detected keypoints - prev: " << prev_img_kps.size()
              << ", cur: " << cur_img_kps.size();

    // Get camera intrinsics
    auto K = cur_frame.getCameraInfo().getKInEigen();
    cv::Mat K_cv;
    cv::eigen2cv(K, K_cv);

    // ===== STAGE 1: Match current frame features with map keypoints =====
    LOG(INFO) << "--- STAGE 1: Matching current frame with map keypoints ---";
    std::set<int> cur_matched_indices =
        matchCurrentFrameWithMap(cur_img_kps, cur_img_desc, cur_frame, K_cv, map_keypoints);
    LOG(INFO) << "Stage 1 matched " << cur_matched_indices.size()
              << " current frame keypoints with map";

    // ===== STAGE 2: Match remaining features with previous frame =====
    LOG(INFO) << "--- STAGE 2: Matching remaining features with previous frame ---";
    std::optional<core::types::Pose> relative_pose = matchRemainingWithPrevFrame(
        prev_img_kps, prev_img_desc, cur_img_kps, cur_img_desc, cur_matched_indices, prev_frame,
        cur_frame, K_cv, map_keypoints, world_points);

    LOG(INFO) << "Map size after matching: " << map_keypoints.size() << " keypoints (added "
              << (map_keypoints.size() - (map_keypoints.size() - world_points.size())) << " new)";
    LOG(INFO) << "Newly triangulated points: " << world_points.size();

    return relative_pose;
}

// Helper: Get pixel color from image at keypoint location (normalized to [0,1])
Eigen::Vector3d OrbTracker::getPixelColor(const core::types::KeyFrame& frame, float x, float y) {
    const cv::Mat& img = frame.color_data.value().data;

    // Clamp to image bounds
    int px = std::max(0, std::min(static_cast<int>(std::round(x)), img.cols - 1));
    int py = std::max(0, std::min(static_cast<int>(std::round(y)), img.rows - 1));

    if (img.channels() == 3) {
        cv::Vec3b bgr = img.at<cv::Vec3b>(py, px);
        // Convert BGR to RGB and normalize to [0,1]
        return Eigen::Vector3d(bgr[2] / 255.0, bgr[1] / 255.0, bgr[0] / 255.0);
    } else {
        // Grayscale image
        uchar gray = img.at<uchar>(py, px);
        double normalized = gray / 255.0;
        return Eigen::Vector3d(normalized, normalized, normalized);
    }
}

// STAGE 1: Match current frame features with map keypoints
std::set<int> OrbTracker::matchCurrentFrameWithMap(
    const std::vector<cv::KeyPoint>& cur_img_kps, const cv::Mat& cur_img_desc,
    const core::types::KeyFrame& cur_frame, const cv::Mat& K,
    std::map<uint32_t, core::types::Keypoint>& map_keypoints) {

    std::set<int> matched_indices;

    if (map_keypoints.empty()) {
        LOG(INFO) << "Map is empty, skipping Stage 1";
        return matched_indices;
    }

    if (cur_img_kps.empty()) {
        LOG(INFO) << "No current frame keypoints detected";
        return matched_indices;
    }

    // Extract descriptors from map keypoints
    std::vector<uint32_t> map_ids;
    cv::Mat map_descriptors;
    for (const auto& [id, keypoint] : map_keypoints) {
        if (!keypoint.descriptor.empty()) {
            map_ids.push_back(id);
            map_descriptors.push_back(keypoint.descriptor);
        }
    }

    if (map_descriptors.empty()) {
        LOG(INFO) << "No map keypoints have descriptors, skipping Stage 1";
        return matched_indices;
    }

    LOG(INFO) << "Matching " << cur_img_kps.size() << " current frame features against "
              << map_descriptors.rows << " map keypoints";

    // Match current frame descriptors against map descriptors
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher_->knnMatch(cur_img_desc, map_descriptors, knn_matches, 2);

    // Apply ratio test
    std::vector<cv::DMatch> good_matches;
    const float ratio_thresh = 0.75f;
    for (const auto& match_pair : knn_matches) {
        if (match_pair.size() >= 2 &&
            match_pair[0].distance < ratio_thresh * match_pair[1].distance) {
            good_matches.push_back(match_pair[0]);
        }
    }

    LOG(INFO) << "Ratio test passed: " << good_matches.size() << " matches";

    if (good_matches.empty()) {
        return matched_indices;
    }

    // TODO: Apply epipolar filtering using most recent keyframe from map (next step)
    // For now, accept all ratio-test matches

    // Update map keypoints with new observations
    int updates = 0;
    for (const auto& match : good_matches) {
        int cur_kp_idx = match.queryIdx;  // Index in cur_img_kps
        int map_kp_idx = match.trainIdx;  // Index in map_ids array
        uint32_t map_id = map_ids[map_kp_idx];

        auto& map_keypoint = map_keypoints[map_id];

        // Add new location observation
        core::types::Location new_loc;
        new_loc.keyframe_id = cur_frame.id;
        new_loc.frame_id = cur_frame.color_data.value().frame_id;
        new_loc.x = cur_img_kps[cur_kp_idx].pt.x;
        new_loc.y = cur_img_kps[cur_kp_idx].pt.y;
        map_keypoint.locations.push_back(new_loc);

        // Update averaged color: (old_color * old_count + new_color) / new_count
        Eigen::Vector3d pixel_color = getPixelColor(cur_frame, new_loc.x, new_loc.y);
        int old_count = map_keypoint.locations.size() - 1;  // Before adding new location
        if (old_count == 0) {
            // First observation, initialize color
            map_keypoint.color = pixel_color;
        } else {
            // Average with previous observations
            map_keypoint.color =
                (map_keypoint.color * old_count + pixel_color) / map_keypoint.locations.size();
        }

        matched_indices.insert(cur_kp_idx);
        updates++;
    }

    LOG(INFO) << "Updated " << updates << " map keypoints with new observations";

    return matched_indices;
}

// STAGE 2: Match remaining current features with previous frame
std::optional<core::types::Pose> OrbTracker::matchRemainingWithPrevFrame(
    const std::vector<cv::KeyPoint>& prev_img_kps, const cv::Mat& prev_img_desc,
    const std::vector<cv::KeyPoint>& cur_img_kps, const cv::Mat& cur_img_desc,
    const std::set<int>& cur_matched_indices, const core::types::KeyFrame& prev_frame,
    const core::types::KeyFrame& cur_frame, const cv::Mat& K,
    std::map<uint32_t, core::types::Keypoint>& map_keypoints,
    std::vector<Eigen::Vector3d>& world_points) {

    if (prev_img_kps.empty() || cur_img_kps.empty()) {
        LOG(WARNING) << "No keypoints for Stage 2 matching";
        return std::nullopt;
    }

    // Filter out already-matched current frame keypoints
    std::vector<cv::KeyPoint> filtered_cur_kps;
    cv::Mat filtered_cur_desc;
    std::vector<int> filtered_to_original_idx;  // Map filtered index -> original index

    for (size_t i = 0; i < cur_img_kps.size(); ++i) {
        if (cur_matched_indices.find(i) == cur_matched_indices.end()) {
            filtered_cur_kps.push_back(cur_img_kps[i]);
            filtered_cur_desc.push_back(cur_img_desc.row(i));
            filtered_to_original_idx.push_back(i);
        }
    }

    LOG(INFO) << "After filtering: " << filtered_cur_kps.size()
              << " remaining current frame keypoints (filtered out " << cur_matched_indices.size()
              << ")";

    if (filtered_cur_kps.empty()) {
        LOG(WARNING) << "All current frame keypoints were matched in Stage 1, no new features";
        return std::nullopt;
    }

    // Match prev → filtered_cur using ratio test
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher_->knnMatch(prev_img_desc, filtered_cur_desc, knn_matches, 2);

    std::vector<cv::DMatch> good_matches;
    std::vector<cv::Point2f> prev_img_points, cur_img_points;
    std::vector<cv::KeyPoint> good_prev_kps, good_cur_kps;

    const float ratio_thresh = 0.75f;
    for (size_t i = 0; i < knn_matches.size(); ++i) {
        if (knn_matches[i].size() >= 2 &&
            knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
            prev_img_points.push_back(prev_img_kps[knn_matches[i][0].queryIdx].pt);
            cur_img_points.push_back(filtered_cur_kps[knn_matches[i][0].trainIdx].pt);
            good_prev_kps.push_back(prev_img_kps[knn_matches[i][0].queryIdx]);
            good_cur_kps.push_back(filtered_cur_kps[knn_matches[i][0].trainIdx]);
        }
    }

    LOG(INFO) << "Good matches (ratio test): " << good_matches.size();

    if (good_matches.size() < 8) {
        LOG(WARNING) << "Insufficient matches for epipolar geometry (" << good_matches.size()
                     << " < 8)";
        return std::nullopt;
    }

    // Compute fundamental matrix and apply epipolar filtering
    Eigen::Isometry3d T_cur_prev =
        stf::getRelativeWithBaseLink1(cur_frame, prev_frame, *tft_, "base_link");

    Eigen::Matrix3d R = T_cur_prev.rotation();
    Eigen::Vector3d t = T_cur_prev.translation();

    Eigen::Matrix3d t_skew;
    t_skew << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
    Eigen::Matrix3d E_eigen = t_skew * R;

    // Convert K (cv::Mat) to Eigen::Matrix3d for matrix operations
    Eigen::Matrix3d K_eigen;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K_eigen(i, j) = K.at<double>(i, j);
        }
    }
    Eigen::Matrix3d F_eigen = K_eigen.transpose().inverse() * E_eigen * K_eigen.inverse();

    cv::Mat F_cv;
    cv::eigen2cv(F_eigen, F_cv);

    // Epipolar filtering
    std::vector<cv::KeyPoint> inliers1, inliers2;
    std::vector<cv::Point2f> inlier_pts1, inlier_pts2;
    std::vector<cv::DMatch> inlier_matches;
    std::vector<int> inlier_original_cur_indices;
    std::vector<int> inlier_filtered_cur_indices;  // Track filtered index for descriptor access

    cv::Mat ipts1_mat(prev_img_points);
    std::vector<cv::Vec3f> epilines1;
    cv::computeCorrespondEpilines(ipts1_mat, 1, F_cv, epilines1);

    for (size_t i = 0; i < prev_img_points.size(); ++i) {
        cv::Vec3f epiline = epilines1[i];
        cv::Point2f pt = prev_img_points[i];

        double dist = std::abs(epiline[0] * pt.x + epiline[1] * pt.y + epiline[2]) /
                      std::sqrt(epiline[0] * epiline[0] + epiline[1] * epiline[1]);

        const double epipolar_thresh = 2.5;
        if (dist < epipolar_thresh) {
            inlier_pts1.push_back(prev_img_points[i]);
            inlier_pts2.push_back(cur_img_points[i]);
            inliers1.push_back(good_prev_kps[i]);
            inliers2.push_back(good_cur_kps[i]);
            inlier_matches.push_back(
                cv::DMatch(inliers1.size() - 1, inliers2.size() - 1, good_matches[i].distance));

            // Track indices for descriptor access and location mapping
            int filtered_idx = good_matches[i].trainIdx;
            inlier_filtered_cur_indices.push_back(filtered_idx);
            inlier_original_cur_indices.push_back(filtered_to_original_idx[filtered_idx]);
        }
    }

    LOG(INFO) << "Epipolar inliers: " << inlier_pts1.size();

    if (inlier_pts1.size() < 4) {
        LOG(WARNING) << "Insufficient epipolar inliers for triangulation (" << inlier_pts1.size()
                     << " < 4)";
        return std::nullopt;
    }

    // Triangulate inliers (use new overload that returns valid indices)
    std::vector<int> valid_triangulation_indices;
    std::vector<Eigen::Vector3d> triangulated_points = triangulateMatches(
        inlier_pts1, inlier_pts2, prev_frame, cur_frame, K, valid_triangulation_indices);

    LOG(INFO) << "Triangulated " << triangulated_points.size() << " new 3D points (from "
              << inlier_pts1.size() << " inliers, filtered "
              << (inlier_pts1.size() - triangulated_points.size()) << ")";

    // Add triangulated points to map_keypoints
    // valid_triangulation_indices[i] tells us which inlier produced triangulated_points[i]
    for (size_t i = 0; i < triangulated_points.size(); ++i) {
        int inlier_idx = valid_triangulation_indices[i];  // Index into inlier arrays

        // Generate new ID using map size
        uint32_t new_id = map_keypoints.size();

        core::types::Keypoint new_keypoint(new_id);
        new_keypoint.position = triangulated_points[i];

        // Store descriptor from current frame (keep first observation)
        // Use inlier_filtered_cur_indices to correctly index into filtered_cur_desc
        int cur_kp_idx_in_filtered = inlier_filtered_cur_indices[inlier_idx];
        new_keypoint.descriptor = filtered_cur_desc.row(cur_kp_idx_in_filtered).clone();

        // Add two location observations (prev + cur)
        core::types::Location prev_loc;
        prev_loc.keyframe_id = prev_frame.id;
        prev_loc.frame_id = prev_frame.color_data.value().frame_id;
        prev_loc.x = inlier_pts1[inlier_idx].x;
        prev_loc.y = inlier_pts1[inlier_idx].y;
        new_keypoint.locations.push_back(prev_loc);

        core::types::Location cur_loc;
        cur_loc.keyframe_id = cur_frame.id;
        cur_loc.frame_id = cur_frame.color_data.value().frame_id;
        cur_loc.x = inlier_pts2[inlier_idx].x;
        cur_loc.y = inlier_pts2[inlier_idx].y;
        new_keypoint.locations.push_back(cur_loc);

        // Set color from current frame pixel
        new_keypoint.color = getPixelColor(cur_frame, cur_loc.x, cur_loc.y);

        new_keypoint.needs_triangulation = false;

        // Add to map
        map_keypoints[new_id] = new_keypoint;

        // Add to world_points output for callback compatibility
        world_points.push_back(triangulated_points[i]);
    }

    LOG(INFO) << "Added " << triangulated_points.size() << " new keypoints to map";

    // Return relative pose from essential matrix decomposition
    // For now, use the ground truth relative pose from transforms
    // TODO: Could extract pose from essential matrix if needed
    core::types::Pose relative_pose;
    relative_pose.position = T_cur_prev.translation();
    relative_pose.orientation = Eigen::Quaterniond(T_cur_prev.rotation());
    relative_pose.frame_id = prev_frame.pose.frame_id;
    relative_pose.timestamp = cur_frame.pose.timestamp;

    return relative_pose;
}

// DEBUG: Generate synthetic wall features for testing triangulation pipeline
void OrbTracker::generateSyntheticWallFeatures(const core::types::KeyFrame& prev_frame,
                                               const core::types::KeyFrame& cur_frame,
                                               const cv::Mat& K,
                                               std::vector<cv::Point2f>& prev_points,
                                               std::vector<cv::Point2f>& cur_points,
                                               std::vector<Eigen::Vector3d>& world_points) {
    prev_points.clear();
    cur_points.clear();
    world_points.clear();

    if (!tft_) {
        LOG(ERROR) << "Transform tree not available for synthetic wall generation";
        return;
    }

    LOG(INFO) << "=== GENERATING SYNTHETIC WALL FEATURES (DEBUG MODE) ===";

    // Get current frame pose in world coordinates
    auto cur_pose_world = cur_frame.pose.getEigenIsometry();

    // Get base_link to camera transform
    auto cur_cam_frame = cur_frame.color_data.value().frame_id;
    auto T_base_to_cam = tft_->getTransform(base_link_frame_id_, cur_cam_frame).transform;

    // Compute camera position in world frame
    Eigen::Isometry3d T_world_to_cur_cam = cur_pose_world * T_base_to_cam;
    Eigen::Vector3d camera_position = T_world_to_cur_cam.translation();
    Eigen::Vector3d forward_direction = T_world_to_cur_cam.rotation() * Eigen::Vector3d(0, 0, 1);

    LOG(INFO) << "Current camera position (world): " << camera_position.transpose();
    LOG(INFO) << "Camera forward direction: " << forward_direction.transpose();

    // Create wall plane at ~3m in front of current camera
    double wall_distance = 3.0;  // meters
    Eigen::Vector3d wall_center = camera_position + forward_direction * wall_distance;

    LOG(INFO) << "Wall center position (world): " << wall_center.transpose();

    // Define wall dimensions (2m x 2m grid)
    double wall_width = 2.0;
    double wall_height = 2.0;
    int grid_cols = 8;  // 8x8 grid = 64 points
    int grid_rows = 8;

    // Compute wall's right and up vectors (perpendicular to forward direction)
    Eigen::Vector3d world_up(0, 0, 1);  // Assuming Z-up world frame
    Eigen::Vector3d wall_right = forward_direction.cross(world_up).normalized();
    Eigen::Vector3d wall_up = wall_right.cross(forward_direction).normalized();

    LOG(INFO) << "Wall right vector: " << wall_right.transpose();
    LOG(INFO) << "Wall up vector: " << wall_up.transpose();

    // Get transforms for both camera frames
    auto prev_cam_frame = prev_frame.color_data.value().frame_id;
    auto T_world_prev = prev_frame.pose.getEigenIsometry();
    auto T_world_cur = cur_frame.pose.getEigenIsometry();
    auto T_base_to_prev_cam = tft_->getTransform(base_link_frame_id_, prev_cam_frame).transform;
    auto T_base_to_cur_cam = tft_->getTransform(base_link_frame_id_, cur_cam_frame).transform;

    auto T_world_prev_cam = T_world_prev * T_base_to_prev_cam;
    auto T_world_cur_cam = T_world_cur * T_base_to_cur_cam;

    // Get camera intrinsics
    Eigen::Matrix3d K_eigen;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K_eigen(i, j) = K.at<double>(i, j);
        }
    }

    LOG(INFO) << "Generating " << (grid_cols * grid_rows) << " synthetic wall points...";

    // Generate grid of 3D points on the wall
    int valid_points = 0;
    for (int row = 0; row < grid_rows; ++row) {
        for (int col = 0; col < grid_cols; ++col) {
            // Compute 3D point position on wall
            double u = (col - grid_cols / 2.0 + 0.5) / grid_cols;  // [-0.5, 0.5]
            double v = (row - grid_rows / 2.0 + 0.5) / grid_rows;  // [-0.5, 0.5]

            Eigen::Vector3d world_point =
                wall_center + wall_right * (u * wall_width) + wall_up * (v * wall_height);

            // Transform to camera frames
            Eigen::Vector3d point_prev_cam = T_world_prev_cam.inverse() * world_point;
            Eigen::Vector3d point_cur_cam = T_world_cur_cam.inverse() * world_point;

            // Check if point is in front of both cameras
            if (point_prev_cam.z() <= 0.1 || point_cur_cam.z() <= 0.1) {
                continue;  // Skip points behind camera
            }

            // Project to pixel coordinates
            Eigen::Vector3d pixel_prev_homo = K_eigen * point_prev_cam;
            Eigen::Vector3d pixel_cur_homo = K_eigen * point_cur_cam;

            double px_prev = pixel_prev_homo.x() / pixel_prev_homo.z();
            double py_prev = pixel_prev_homo.y() / pixel_prev_homo.z();
            double px_cur = pixel_cur_homo.x() / pixel_cur_homo.z();
            double py_cur = pixel_cur_homo.y() / pixel_cur_homo.z();

            // Check if projections are within image bounds
            int img_width = cur_frame.color_data.value().width;
            int img_height = cur_frame.color_data.value().height;

            if (px_prev >= 0 && px_prev < img_width && py_prev >= 0 && py_prev < img_height &&
                px_cur >= 0 && px_cur < img_width && py_cur >= 0 && py_cur < img_height) {
                prev_points.push_back(cv::Point2f(px_prev, py_prev));
                cur_points.push_back(cv::Point2f(px_cur, py_cur));
                world_points.push_back(world_point);
                valid_points++;
            }
        }
    }

    LOG(INFO) << "Generated " << valid_points << " valid synthetic wall features";
    LOG(INFO) << "  Prev points: " << prev_points.size();
    LOG(INFO) << "  Cur points: " << cur_points.size();
    LOG(INFO) << "  World points: " << world_points.size();

    if (valid_points > 0) {
        LOG(INFO) << "Sample point (world): " << world_points[0].transpose();
        LOG(INFO) << "  Projects to prev: (" << prev_points[0].x << ", " << prev_points[0].y << ")";
        LOG(INFO) << "  Projects to cur: (" << cur_points[0].x << ", " << cur_points[0].y << ")";
    }
}

// DEBUG: Generate synthetic 3D object features for testing triangulation pipeline
void OrbTracker::generateSyntheticObjectFeatures(
    const core::types::KeyFrame& prev_frame, const core::types::KeyFrame& cur_frame,
    const cv::Mat& K, std::vector<cv::Point2f>& prev_points, std::vector<cv::Point2f>& cur_points,
    std::vector<Eigen::Vector3d>& world_points, std::vector<int>& object_ids) {
    prev_points.clear();
    cur_points.clear();
    world_points.clear();
    object_ids.clear();

    if (!tft_) {
        LOG(ERROR) << "Transform tree not available for synthetic object generation";
        return;
    }

    LOG(INFO) << "=== GENERATING SYNTHETIC 3D OBJECTS (DEBUG MODE) ===";

    // Define 6 objects at evenly-spaced positions in trajectory bounding box
    // Bounding box: X=[-4.2, 0.0], Y=[0.0, 3.0], Z=[0.5, 2.0]
    std::vector<SyntheticObject> objects;

    // Object 0: Cube (bottom-left)
    objects.push_back(
        {"cube", Eigen::Vector3d(-3.5, 1.0, 1.0), {}, Eigen::Vector3d(1.0, 0.0, 0.0), 0.3});

    // Object 1: Sphere (middle-left)
    objects.push_back(
        {"sphere", Eigen::Vector3d(-3.5, 2.0, 1.2), {}, Eigen::Vector3d(0.0, 1.0, 0.0), 0.2});

    // Object 2: Pyramid (top-left)
    objects.push_back(
        {"pyramid", Eigen::Vector3d(-3.5, 3.0, 0.8), {}, Eigen::Vector3d(0.0, 0.0, 1.0), 0.4});

    // Object 3: Cylinder (bottom-right)
    objects.push_back(
        {"cylinder", Eigen::Vector3d(-0.5, 1.0, 1.5), {}, Eigen::Vector3d(1.0, 1.0, 0.0), 0.15});

    // Object 4: Octahedron (middle-right)
    objects.push_back(
        {"octahedron", Eigen::Vector3d(-0.5, 2.0, 1.3), {}, Eigen::Vector3d(1.0, 0.0, 1.0), 0.25});

    // Object 5: Torus (top-right)
    objects.push_back(
        {"torus", Eigen::Vector3d(-0.5, 3.0, 1.1), {}, Eigen::Vector3d(0.0, 1.0, 1.0), 0.2});

    // Generate feature points for each object
    for (size_t obj_idx = 0; obj_idx < objects.size(); ++obj_idx) {
        auto& obj = objects[obj_idx];

        if (obj.type == "cube") {
            // 8 corners + 6 face centers
            double s = obj.scale / 2.0;
            // 8 corners
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    for (int k = 0; k < 2; ++k) {
                        obj.feature_points.push_back(obj.center + Eigen::Vector3d((i * 2 - 1) * s,
                                                                                  (j * 2 - 1) * s,
                                                                                  (k * 2 - 1) * s));
                    }
                }
            }
            // 6 face centers
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(s, 0, 0));
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(-s, 0, 0));
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(0, s, 0));
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(0, -s, 0));
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(0, 0, s));
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(0, 0, -s));

        } else if (obj.type == "sphere") {
            // 12 points distributed on sphere surface (fibonacci sphere)
            int num_pts = 12;
            double phi = M_PI * (3.0 - std::sqrt(5.0));  // golden angle
            for (int i = 0; i < num_pts; ++i) {
                double y = 1.0 - (i / double(num_pts - 1)) * 2.0;
                double radius = std::sqrt(1.0 - y * y);
                double theta = phi * i;
                double x = std::cos(theta) * radius;
                double z = std::sin(theta) * radius;
                obj.feature_points.push_back(obj.center + obj.scale * Eigen::Vector3d(x, y, z));
            }

        } else if (obj.type == "pyramid") {
            // 4 base corners + 1 apex
            double s = obj.scale / 2.0;
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(s, s, -s));  // base
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(s, -s, -s));
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(-s, -s, -s));
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(-s, s, -s));
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(0, 0, s));  // apex

        } else if (obj.type == "cylinder") {
            // 6 points on top circle + 6 on bottom circle
            double h = 0.5;  // height
            for (int i = 0; i < 6; ++i) {
                double angle = i * M_PI / 3.0;
                double x = obj.scale * std::cos(angle);
                double y = obj.scale * std::sin(angle);
                obj.feature_points.push_back(obj.center + Eigen::Vector3d(x, y, h / 2));   // top
                obj.feature_points.push_back(obj.center + Eigen::Vector3d(x, y, -h / 2));  // bottom
            }

        } else if (obj.type == "octahedron") {
            // 6 vertices
            double s = obj.scale;
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(s, 0, 0));
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(-s, 0, 0));
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(0, s, 0));
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(0, -s, 0));
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(0, 0, s));
            obj.feature_points.push_back(obj.center + Eigen::Vector3d(0, 0, -s));

        } else if (obj.type == "torus") {
            // 16 points around the torus
            double major_r = obj.scale;
            double minor_r = obj.scale * 0.4;
            for (int i = 0; i < 16; ++i) {
                double u = i * 2.0 * M_PI / 16.0;
                double v = (i % 4) * M_PI / 2.0;
                double x = (major_r + minor_r * std::cos(v)) * std::cos(u);
                double y = (major_r + minor_r * std::cos(v)) * std::sin(u);
                double z = minor_r * std::sin(v);
                obj.feature_points.push_back(obj.center + Eigen::Vector3d(x, y, z));
            }
        }
    }

    // Get transforms for both camera frames
    auto prev_cam_frame = prev_frame.color_data.value().frame_id;
    auto cur_cam_frame = cur_frame.color_data.value().frame_id;

    auto T_world_prev = prev_frame.pose.getEigenIsometry();
    auto T_world_cur = cur_frame.pose.getEigenIsometry();

    auto T_base_to_prev_cam = tft_->getTransform(base_link_frame_id_, prev_cam_frame).transform;
    auto T_base_to_cur_cam = tft_->getTransform(base_link_frame_id_, cur_cam_frame).transform;

    auto T_world_prev_cam = T_world_prev * T_base_to_prev_cam;
    auto T_world_cur_cam = T_world_cur * T_base_to_cur_cam;

    // Get camera intrinsics
    Eigen::Matrix3d K_eigen;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K_eigen(i, j) = K.at<double>(i, j);
        }
    }

    int img_width = cur_frame.color_data.value().width;
    int img_height = cur_frame.color_data.value().height;

    // Project all feature points from all objects
    int total_features = 0;
    std::vector<int> visible_per_object(objects.size(), 0);

    for (size_t obj_idx = 0; obj_idx < objects.size(); ++obj_idx) {
        const auto& obj = objects[obj_idx];

        for (const auto& world_point : obj.feature_points) {
            total_features++;

            // Transform to camera frames
            Eigen::Vector3d point_prev_cam = T_world_prev_cam.inverse() * world_point;
            Eigen::Vector3d point_cur_cam = T_world_cur_cam.inverse() * world_point;

            // Check if point is in front of both cameras
            if (point_prev_cam.z() <= 0.1 || point_cur_cam.z() <= 0.1) {
                continue;  // Skip points behind camera
            }

            // Project to pixel coordinates
            Eigen::Vector3d pixel_prev_homo = K_eigen * point_prev_cam;
            Eigen::Vector3d pixel_cur_homo = K_eigen * point_cur_cam;

            double px_prev = pixel_prev_homo.x() / pixel_prev_homo.z();
            double py_prev = pixel_prev_homo.y() / pixel_prev_homo.z();
            double px_cur = pixel_cur_homo.x() / pixel_cur_homo.z();
            double py_cur = pixel_cur_homo.y() / pixel_cur_homo.z();

            // Check if projections are within image bounds
            if (px_prev >= 0 && px_prev < img_width && py_prev >= 0 && py_prev < img_height &&
                px_cur >= 0 && px_cur < img_width && py_cur >= 0 && py_cur < img_height) {
                prev_points.push_back(cv::Point2f(px_prev, py_prev));
                cur_points.push_back(cv::Point2f(px_cur, py_cur));
                world_points.push_back(world_point);
                object_ids.push_back(obj_idx);
                visible_per_object[obj_idx]++;
            }
        }
    }

    LOG(INFO) << "Generated " << total_features << " feature points from " << objects.size()
              << " objects";
    LOG(INFO) << "Visible features: " << prev_points.size() << " total";

    for (size_t i = 0; i < objects.size(); ++i) {
        LOG(INFO) << "  Object " << i << " (" << objects[i].type << "): " << visible_per_object[i]
                  << "/" << objects[i].feature_points.size() << " features visible";
    }

    if (!world_points.empty()) {
        LOG(INFO) << "Sample: Object " << object_ids[0] << " (" << objects[object_ids[0]].type
                  << ") - point: " << world_points[0].transpose();
    }
}

std::vector<Eigen::Vector3d> OrbTracker::triangulateMatches(
    const std::vector<cv::Point2f>& prev_points, const std::vector<cv::Point2f>& cur_points,
    const core::types::KeyFrame& prev_frame, const core::types::KeyFrame& cur_frame,
    const cv::Mat& K) {
    std::vector<Eigen::Vector3d> world_points;

    if (prev_points.size() != cur_points.size()) {
        LOG(ERROR) << "Point counts don't match: " << prev_points.size() << " vs "
                   << cur_points.size();
        return world_points;
    }

    if (!tft_) {
        LOG(ERROR) << "Transform tree not available for triangulation";
        return world_points;
    }

    try {
        // Get camera frame IDs
        auto prev_cam_frame = prev_frame.color_data.value().frame_id;
        auto cur_cam_frame = cur_frame.color_data.value().frame_id;

        LOG(INFO) << "Printing some debug info:";
        LOG(INFO) << "\n" << prev_frame.getCameraInfo().getKInEigen();
        LOG(INFO) << "\n" << cur_frame.getCameraInfo().getKInEigen();
        LOG(INFO) << "Cam info width: " << prev_frame.getCameraInfo().width;
        LOG(INFO) << "Cam info height: " << prev_frame.getCameraInfo().height;
        LOG(INFO) << prev_frame.getColorImage().height << "x" << prev_frame.getColorImage().width;
        LOG(INFO) << cur_frame.getColorImage().height << "x" << cur_frame.getColorImage().width;

        // Get base_link -> camera transforms
        auto T_base_to_prev_cam = tft_->getTransform(base_link_frame_id_, prev_cam_frame).transform;
        auto T_base_to_cur_cam = tft_->getTransform(base_link_frame_id_, cur_cam_frame).transform;

        // Previous frame: base_link -> camera
        LOG(INFO) << "=== PREVIOUS FRAME: base_link -> " << prev_cam_frame << " ===";
        LOG(INFO) << "  Translationprevuse" << T_base_to_prev_cam.translation().transpose();
        LOG(INFO) << "  Rotation Matrix:\n" << T_base_to_prev_cam.rotation();
        LOG(INFO) << "  Quaternion (w,x,y,z): "
                  << Eigen::Quaterniond(T_base_to_prev_cam.rotation()).w() << ", "
                  << Eigen::Quaterniond(T_base_to_prev_cam.rotation()).x() << ", "
                  << Eigen::Quaterniond(T_base_to_prev_cam.rotation()).y() << ", "
                  << Eigen::Quaterniond(T_base_to_prev_cam.rotation()).z();
        LOG(INFO) << "  RPY (rad): " << stf::getRPY(T_base_to_prev_cam).transpose();

        // Current frame: base_link -> camera
        LOG(INFO) << "=== CURRENT FRAME: base_link -> " << cur_cam_frame << " ===";
        LOG(INFO) << "  Translation " << T_base_to_cur_cam.translation().transpose();
        LOG(INFO) << "  Rotation Matrix:\n" << T_base_to_cur_cam.rotation();
        LOG(INFO) << "  Quaternion (w,x,y,z): "
                  << Eigen::Quaterniond(T_base_to_cur_cam.rotation()).w() << ", "
                  << Eigen::Quaterniond(T_base_to_cur_cam.rotation()).x() << ", "
                  << Eigen::Quaterniond(T_base_to_cur_cam.rotation()).y() << ", "
                  << Eigen::Quaterniond(T_base_to_cur_cam.rotation()).z();
        LOG(INFO) << "  RPY (rad): " << stf::getRPY(T_base_to_cur_cam).transpose();

        // Previous frame pose (in odometry/world frame)
        auto prev_pose_iso = prev_frame.pose.getEigenIsometry();
        LOG(INFO) << "=== PREVIOUS FRAME POSE (in " << prev_frame.pose.frame_id << ") ===";
        LOG(INFO) << "  Translationprevpose: " << prev_pose_iso.translation().transpose();
        LOG(INFO) << "  Rotation Matrix:\n" << prev_pose_iso.rotation();
        LOG(INFO) << "  Quaternion (w,x,y,z): " << prev_frame.pose.orientation.w() << ", "
                  << prev_frame.pose.orientation.x() << ", " << prev_frame.pose.orientation.y()
                  << ", " << prev_frame.pose.orientation.z();
        LOG(INFO) << "  RPY (rad): " << stf::getRPY(prev_pose_iso).transpose();

        // Current frame pose (in odometry/world frame)
        auto cur_pose_iso = cur_frame.pose.getEigenIsometry();
        LOG(INFO) << "=== CURRENT FRAME POSE (in " << cur_frame.pose.frame_id << ") ===";
        LOG(INFO) << "  Translation: " << cur_pose_iso.translation().transpose();
        LOG(INFO) << "  Rotation Matrix:\n" << cur_pose_iso.rotation();
        LOG(INFO) << "  Quaternion (w,x,y,z): " << cur_frame.pose.orientation.w() << ", "
                  << cur_frame.pose.orientation.x() << ", " << cur_frame.pose.orientation.y()
                  << ", " << cur_frame.pose.orientation.z();
        LOG(INFO) << "  RPY (rad): " << stf::getRPY(cur_pose_iso).transpose();

        // Get relative transform from prev_camera to cur_camera using getRelative1
        // This computes: T_prev_cam_to_cur_cam
        Eigen::Isometry3d T_prev_cam_to_cur_cam =
            stf::getRelativeWithBaseLink1(cur_frame, prev_frame, *tft_, "base_link");

        LOG(INFO) << "Relative transform (prev_cam -> cur_cam): translation="
                  << T_prev_cam_to_cur_cam.translation().transpose()
                  << ", rotation_rpy=" << stf::getRPY(T_prev_cam_to_cur_cam).transpose();

        // Setup projection matrices for triangulation in prev_camera frame
        // P_prev = K * [I | 0] (prev camera is the reference frame)
        // P_cur = K * [R | t] (where [R|t] is prev_cam -> cur_cam transform)

        cv::Mat P_prev(3, 4, CV_64F);
        cv::Mat P_cur(3, 4, CV_64F);

        // Previous camera projection: [I | 0]
        cv::Mat Rt_prev = cv::Mat::eye(3, 4, CV_64F);
        P_prev = K * Rt_prev;

        // Current camera projection: [R | t] from relative transform
        Eigen::Matrix3d R_rel = T_prev_cam_to_cur_cam.rotation();
        Eigen::Vector3d t_rel = T_prev_cam_to_cur_cam.translation();

        cv::Mat R_rel_cv(3, 3, CV_64F);
        cv::Mat t_rel_cv(3, 1, CV_64F);

        for (int i = 0; i < 3; ++i) {
            t_rel_cv.at<double>(i) = t_rel(i);
            for (int j = 0; j < 3; ++j) {
                R_rel_cv.at<double>(i, j) = R_rel(i, j);
            }
        }

        cv::Mat Rt_cur(3, 4, CV_64F);
        R_rel_cv.copyTo(Rt_cur.colRange(0, 3));
        t_rel_cv.copyTo(Rt_cur.col(3));

        LOG(INFO) << "K:\n" << K;
        LOG(INFO) << "Rt_cur:\n" << Rt_cur;

        P_cur = K * Rt_cur;

        // Triangulate points using OpenCV
        cv::Mat points_homogeneous;
        cv::triangulatePoints(P_prev, P_cur, prev_points, cur_points, points_homogeneous);
        LOG(INFO) << "points_homogeneous: " << points_homogeneous.rows << "x"
                  << points_homogeneous.cols;
        LOG(INFO) << "points_homogeneous: " << points_homogeneous.t();
        LOG(INFO) << "points_homogeneous: " << points_homogeneous.at<float>(0, 0) << " "
                  << points_homogeneous.at<float>(0, 1) << " " << points_homogeneous.at<float>(0, 2)
                  << " " << points_homogeneous.at<float>(0, 3);
        Eigen::Vector3d point_in_3d = {
            points_homogeneous.at<float>(0, 0) / points_homogeneous.at<float>(3, 0),
            points_homogeneous.at<float>(1, 0) / points_homogeneous.at<float>(3, 0),
            points_homogeneous.at<float>(2, 0) / points_homogeneous.at<float>(3, 0)};
        LOG(INFO) << "point_in_3d: " << point_in_3d.transpose();

        for (int i = 0; i < P_prev.rows; i++) {
            LOG(INFO) << "P_prev row " << i << ": " << P_prev.at<double>(i, 0) << " "
                      << P_prev.at<double>(i, 1) << " " << P_prev.at<double>(i, 2) << " "
                      << P_prev.at<double>(i, 3);
        }

        for (int i = 0; i < P_cur.rows; i++) {
            LOG(INFO) << "P_cur row " << i << ": " << P_cur.at<double>(i, 0) << " "
                      << P_cur.at<double>(i, 1) << " " << P_cur.at<double>(i, 2) << " "
                      << P_cur.at<double>(i, 3);
        }

        for (int i = 0; i < prev_points.size(); i++) {
            LOG(INFO) << "prev_points row " << i << ": " << prev_points[i].x << " "
                      << prev_points[i].y;
        }

        for (int i = 0; i < cur_points.size(); i++) {
            LOG(INFO) << "cur_points row " << i << ": " << cur_points[i].x << " "
                      << cur_points[i].y;
        }

        // Get transforms to convert from camera frames to world frame
        // Transform chain: prev_camera -> base_link -> odom (world)
        auto T_base_prev_cam = tft_->getTransform(base_link_frame_id_, prev_cam_frame).transform;
        auto T_world_prev_base = prev_frame.pose.getEigenIsometry();
        auto T_world_prev_cam = T_world_prev_base * T_base_prev_cam;

        // For current camera (for sanity checks)
        auto T_base_cur_cam = tft_->getTransform(base_link_frame_id_, cur_cam_frame).transform;
        auto T_world_cur_base = cur_frame.pose.getEigenIsometry();
        auto T_world_cur_cam = T_world_cur_base * T_base_cur_cam;

        LOG(INFO) << "Transform prev_cam -> world: translation="
                  << T_world_prev_cam.translation().transpose()
                  << ", rotation_rpy=" << stf::getRPY(T_world_prev_cam).transpose();

        LOG(INFO) << "Points homogeneous: " << points_homogeneous.rows << "x"
                  << points_homogeneous.cols;

        // Convert from homogeneous coordinates and filter valid points
        for (int i = 0; i < points_homogeneous.cols; ++i) {
            double w = points_homogeneous.at<float>(3, i);

            if (std::abs(w) < 1e-6) {
                LOG(WARNING) << "Point " << i << " has near-zero homogeneous coordinate";
                continue;
            }

            // Point is in previous camera frame after triangulation
            Eigen::Vector3d point_in_prev_cam(points_homogeneous.at<float>(0, i) / w,
                                              points_homogeneous.at<float>(1, i) / w,
                                              points_homogeneous.at<float>(2, i) / w);

            // Transform to world coordinates
            Eigen::Vector3d point_world = T_world_prev_cam * point_in_prev_cam;

            // Sanity checks:
            // 1. Point should be in front of both cameras
            // (point_in_prev_cam is already in prev camera frame)
            Eigen::Vector3d point_in_cur_cam = T_world_cur_cam.inverse() * point_world;

            if (point_in_prev_cam.z() <= 0 || point_in_cur_cam.z() <= 0) {
                LOG(WARNING) << "Point " << i << " behind camera (z_prev=" << point_in_prev_cam.z()
                             << ", z_cur=" << point_in_cur_cam.z() << ")";
                continue;
            }

            // 2. Point should be within reasonable distance (e.g., < 100m)
            double dist_from_prev = point_in_prev_cam.norm();
            double dist_from_cur = point_in_cur_cam.norm();

            if (dist_from_prev > 100.0 || dist_from_cur > 100.0) {
                LOG(WARNING) << "Point " << i << " too far (dist_prev=" << dist_from_prev
                             << "m, dist_cur=" << dist_from_cur << "m)";
                continue;
            }

            // 3. Check reprojection error
            // Project point back to image and compare with original observation
            // Convert Eigen to cv::Mat for projection
            cv::Mat point_prev_cv = (cv::Mat_<double>(3, 1) << point_in_prev_cam.x(),
                                     point_in_prev_cam.y(), point_in_prev_cam.z());
            cv::Mat point_cur_cv = (cv::Mat_<double>(3, 1) << point_in_cur_cam.x(),
                                    point_in_cur_cam.y(), point_in_cur_cam.z());

            cv::Mat proj_prev_cv = K * point_prev_cv;
            cv::Mat proj_cur_cv = K * point_cur_cv;

            double proj_prev_x = proj_prev_cv.at<double>(0) / proj_prev_cv.at<double>(2);
            double proj_prev_y = proj_prev_cv.at<double>(1) / proj_prev_cv.at<double>(2);
            double proj_cur_x = proj_cur_cv.at<double>(0) / proj_cur_cv.at<double>(2);
            double proj_cur_y = proj_cur_cv.at<double>(1) / proj_cur_cv.at<double>(2);

            double reproj_error_prev = std::sqrt(std::pow(proj_prev_x - prev_points[i].x, 2) +
                                                 std::pow(proj_prev_y - prev_points[i].y, 2));
            double reproj_error_cur = std::sqrt(std::pow(proj_cur_x - cur_points[i].x, 2) +
                                                std::pow(proj_cur_y - cur_points[i].y, 2));

            if (reproj_error_prev > 5.0 || reproj_error_cur > 5.0) {
                LOG(WARNING) << "Point " << i
                             << " has high reprojection error (prev=" << reproj_error_prev
                             << "px, cur=" << reproj_error_cur << "px)";
                continue;
            }

            // All checks passed - add to output
            world_points.push_back(point_world);
        }

        LOG(INFO) << "Triangulation: " << world_points.size() << "/" << points_homogeneous.cols
                  << " points passed quality checks";

    } catch (const std::exception& e) {
        LOG(ERROR) << "Triangulation failed: " << e.what();
    }

    return world_points;
}

// Overload of triangulateMatches that also returns valid indices
// valid_indices[i] corresponds to the input index that produced world_points[i]
std::vector<Eigen::Vector3d> OrbTracker::triangulateMatches(
    const std::vector<cv::Point2f>& prev_points, const std::vector<cv::Point2f>& cur_points,
    const core::types::KeyFrame& prev_frame, const core::types::KeyFrame& cur_frame,
    const cv::Mat& K, std::vector<int>& valid_indices) {

    std::vector<Eigen::Vector3d> world_points;
    valid_indices.clear();

    if (prev_points.size() != cur_points.size()) {
        LOG(ERROR) << "Point counts don't match: " << prev_points.size() << " vs "
                   << cur_points.size();
        return world_points;
    }

    if (!tft_) {
        LOG(ERROR) << "Transform tree not available for triangulation";
        return world_points;
    }

    try {
        // Get camera frame IDs
        auto prev_cam_frame = prev_frame.color_data.value().frame_id;
        auto cur_cam_frame = cur_frame.color_data.value().frame_id;

        // Get relative transform from prev_camera to cur_camera
        Eigen::Isometry3d T_prev_cam_to_cur_cam =
            stf::getRelativeWithBaseLink1(cur_frame, prev_frame, *tft_, "base_link");

        // Setup projection matrices for triangulation
        cv::Mat P_prev(3, 4, CV_64F);
        cv::Mat P_cur(3, 4, CV_64F);

        cv::Mat Rt_prev = cv::Mat::eye(3, 4, CV_64F);
        P_prev = K * Rt_prev;

        Eigen::Matrix3d R_rel = T_prev_cam_to_cur_cam.rotation();
        Eigen::Vector3d t_rel = T_prev_cam_to_cur_cam.translation();

        cv::Mat R_rel_cv(3, 3, CV_64F);
        cv::Mat t_rel_cv(3, 1, CV_64F);

        for (int i = 0; i < 3; ++i) {
            t_rel_cv.at<double>(i) = t_rel(i);
            for (int j = 0; j < 3; ++j) {
                R_rel_cv.at<double>(i, j) = R_rel(i, j);
            }
        }

        cv::Mat Rt_cur(3, 4, CV_64F);
        R_rel_cv.copyTo(Rt_cur.colRange(0, 3));
        t_rel_cv.copyTo(Rt_cur.col(3));

        P_cur = K * Rt_cur;

        // Triangulate points using OpenCV
        cv::Mat points_homogeneous;
        cv::triangulatePoints(P_prev, P_cur, prev_points, cur_points, points_homogeneous);

        // Get transforms to convert from camera frames to world frame
        auto T_base_prev_cam = tft_->getTransform(base_link_frame_id_, prev_cam_frame).transform;
        auto T_world_prev_base = prev_frame.pose.getEigenIsometry();
        auto T_world_prev_cam = T_world_prev_base * T_base_prev_cam;

        auto T_base_cur_cam = tft_->getTransform(base_link_frame_id_, cur_cam_frame).transform;
        auto T_world_cur_base = cur_frame.pose.getEigenIsometry();
        auto T_world_cur_cam = T_world_cur_base * T_base_cur_cam;

        // Process each triangulated point with quality filtering
        for (int i = 0; i < points_homogeneous.cols; ++i) {
            float w = points_homogeneous.at<float>(3, i);
            if (std::abs(w) < 1e-6) {
                continue;  // Skip points at infinity
            }

            // Point is in previous camera frame after triangulation
            Eigen::Vector3d point_in_prev_cam(points_homogeneous.at<float>(0, i) / w,
                                              points_homogeneous.at<float>(1, i) / w,
                                              points_homogeneous.at<float>(2, i) / w);

            // Transform to world coordinates
            Eigen::Vector3d point_world = T_world_prev_cam * point_in_prev_cam;

            // Sanity checks
            Eigen::Vector3d point_in_cur_cam = T_world_cur_cam.inverse() * point_world;

            // 1. Point should be in front of both cameras
            if (point_in_prev_cam.z() <= 0 || point_in_cur_cam.z() <= 0) {
                continue;  // Skip, but don't log to reduce verbosity
            }

            // 2. Point should be within reasonable distance (< 100m)
            double dist_from_prev = point_in_prev_cam.norm();
            double dist_from_cur = point_in_cur_cam.norm();

            if (dist_from_prev > 100.0 || dist_from_cur > 100.0) {
                continue;  // Skip
            }

            // 3. Check reprojection error
            cv::Mat point_prev_cv = (cv::Mat_<double>(3, 1) << point_in_prev_cam.x(),
                                     point_in_prev_cam.y(), point_in_prev_cam.z());
            cv::Mat point_cur_cv = (cv::Mat_<double>(3, 1) << point_in_cur_cam.x(),
                                    point_in_cur_cam.y(), point_in_cur_cam.z());

            cv::Mat proj_prev_cv = K * point_prev_cv;
            cv::Mat proj_cur_cv = K * point_cur_cv;

            double proj_prev_x = proj_prev_cv.at<double>(0) / proj_prev_cv.at<double>(2);
            double proj_prev_y = proj_prev_cv.at<double>(1) / proj_prev_cv.at<double>(2);
            double proj_cur_x = proj_cur_cv.at<double>(0) / proj_cur_cv.at<double>(2);
            double proj_cur_y = proj_cur_cv.at<double>(1) / proj_cur_cv.at<double>(2);

            double reproj_error_prev = std::sqrt(std::pow(proj_prev_x - prev_points[i].x, 2) +
                                                 std::pow(proj_prev_y - prev_points[i].y, 2));
            double reproj_error_cur = std::sqrt(std::pow(proj_cur_x - cur_points[i].x, 2) +
                                                std::pow(proj_cur_y - cur_points[i].y, 2));

            if (reproj_error_prev > 5.0 || reproj_error_cur > 5.0) {
                continue;  // Skip
            }

            // All checks passed - add to output with corresponding index
            world_points.push_back(point_world);
            valid_indices.push_back(i);  // Track which input index produced this output
        }

        LOG(INFO) << "Triangulation: " << world_points.size() << "/" << points_homogeneous.cols
                  << " points passed quality checks";

    } catch (const std::exception& e) {
        LOG(ERROR) << "Triangulation failed: " << e.what();
    }

    return world_points;
}

std::optional<core::types::Pose> OrbTracker::operator()(
    const core::types::KeyFrame& current_kf, const core::types::KeyFrame& previous_kf,
    std::map<uint32_t, core::types::Keypoint>& map_keypoints) {
    LOG(INFO) << "Step 1";
    if (!current_kf.color_data.has_value() || !previous_kf.color_data.has_value()) {
        return std::nullopt;
    }
    auto rtf = stf::getRelative(previous_kf, current_kf, *tft_);
    LOG(INFO) << "Step 2";
    if (rtf.translation().norm() < 1e-6) {
        LOG(INFO) << "Not actually a keyframe";
        return std::nullopt;
    }
    LOG(INFO) << "Step 3";

    auto cur_frame = current_kf.color_data.value().frame_id;
    LOG(INFO) << "Step 4";
    auto prev_frame = previous_kf.color_data.value().frame_id;
    LOG(INFO) << "Step 5";
    if (cam_poses_.find(cur_frame) == cam_poses_.end() ||
        cam_infos_.find(cur_frame) == cam_infos_.end() ||
        cam_poses_.find(prev_frame) == cam_poses_.end() ||
        cam_infos_.find(prev_frame) == cam_infos_.end()) {
        LOG(ERROR) << "Unable to find camera pose or matrix";
        return std::nullopt;
    }
    LOG(INFO) << "Step 6";
    cv::Mat cur_img = current_kf.color_data.value().data;
    LOG(INFO) << "Step 7";
    cv::Mat prev_img = previous_kf.color_data.value().data;
    LOG(INFO) << "Step 8";

    std::vector<cv::KeyPoint> current_img_keypoints;
    cv::Mat current_img_descriptors;
    orb_detector_->detectAndCompute(cur_img, cv::noArray(), current_img_keypoints,
                                    current_img_descriptors);
    LOG(INFO) << "Step 9";
    std::map<uint32_t, uint32_t> current_img_to_map_keypoint_idx;

    track(current_kf, current_img_keypoints, current_img_descriptors, map_keypoints,
          current_img_to_map_keypoint_idx);

    LOG(INFO) << "dev: Finished tracking!";

    std::vector<cv::KeyPoint> untracked_cur_img_keypoints;
    cv::Mat untracked_cur_img_descriptors;

    for (uint32_t i = 0; i < current_img_keypoints.size(); ++i) {
        // if (current_img_to_map_keypoint_idx.find(i) == current_img_to_map_keypoint_idx.end()) {
        untracked_cur_img_keypoints.push_back(current_img_keypoints[i]);
        untracked_cur_img_descriptors.push_back(current_img_descriptors.row(i));
        // }
    }
    LOG(INFO) << "dev: Finished adding, tracked points: " << current_img_to_map_keypoint_idx.size();

    std::vector<cv::KeyPoint> previous_img_keypoints;
    cv::Mat previous_img_descriptors;
    orb_detector_->detectAndCompute(prev_img, cv::noArray(), previous_img_keypoints,
                                    previous_img_descriptors);
    LOG(INFO) << "dev: Finished detecting orb features from prev img";

    if (previous_img_keypoints.empty() || current_img_keypoints.empty()) {
        LOG(ERROR) << "No keypoints detected in keyframe " << previous_img_keypoints.size() << " "
                   << current_img_keypoints.size();
        return std::nullopt;
    }

    if (previous_img_descriptors.empty() || current_img_descriptors.empty()) {
        LOG(ERROR) << "No descriptors in the keyframe";
        return std::nullopt;
    }

    LOG(INFO) << "ORB: Calling feature matching only (triangulation deferred to graph_adapter.cpp)";

    // NEW ARCHITECTURE: Only match features, defer triangulation to graph_adapter.cpp
    auto cam_info = cam_infos_.find(cur_frame);
    if (cam_info == cam_infos_.end()) {
        LOG(ERROR) << "Unable to find camera info";
        return std::nullopt;
    }
    cv::Mat K(cam_info->second.k);
    K = K.reshape(1, 3);

    // Call feature matching only (no triangulation)
    auto transform =
        matchFeaturesOnly(untracked_cur_img_keypoints, untracked_cur_img_descriptors,
                          previous_img_keypoints, previous_img_descriptors, current_kf, previous_kf,
                          K, current_img_to_map_keypoint_idx, map_keypoints);

    return transform;
}

std::optional<core::types::Pose> OrbTracker::track(
    const core::types::KeyFrame& cur_kf, const std::vector<cv::KeyPoint>& current_img_keypoints,
    const cv::Mat& current_img_descriptors,
    std::map<uint32_t, core::types::Keypoint>& map_keypoints,
    std::map<uint32_t, uint32_t>& current_img_to_map_keypoint_idx) {
    if (map_keypoints.empty()) {
        LOG(ERROR) << "No map points to match against";
        return std::nullopt;
    }

    LOG(INFO) << "dev: Doing tracking with: " << "\n\t Current keypoints: "
              << current_img_keypoints.size() << " desc: " << current_img_descriptors.size()
              << "\n\t Map keypoints: " << map_keypoints.size();
    std::vector<cv::DMatch> matches_to_map;
    cv::Mat map_descriptors_mat;
    std::vector<uint32_t> map_ids_for_descriptors;

    for (const auto& kp : map_keypoints) {
        if (!kp.second.descriptor.empty()) {
            map_descriptors_mat.push_back(kp.second.descriptor);
            map_ids_for_descriptors.push_back(kp.second.id());
        }
    }

    if (map_descriptors_mat.empty() || current_img_descriptors.empty())
        return std::nullopt;

    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher_->knnMatch(current_img_descriptors, map_descriptors_mat, knn_matches, 2);

    std::vector<cv::Point3f> object_points_for_pnp;
    std::vector<cv::Point2f> image_points_for_pnp;
    std::vector<uint32_t> current_image_kp_ids_pnp;
    std::vector<uint32_t> matched_map_kp_ids_pnp;

    const float ratio_thresh = 0.7f;
    for (uint32_t i = 0; i < knn_matches.size(); ++i) {
        if (knn_matches[i].size() == 2 &&
            knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            const cv::DMatch& best_match = knn_matches[i][0];
            uint32_t map_kp_id = map_ids_for_descriptors[best_match.trainIdx];

            object_points_for_pnp.push_back(cv::Point3f(map_keypoints[map_kp_id].position.x(),
                                                        map_keypoints[map_kp_id].position.y(),
                                                        map_keypoints[map_kp_id].position.z()));
            image_points_for_pnp.push_back(current_img_keypoints[best_match.queryIdx].pt);
            current_image_kp_ids_pnp.push_back(best_match.queryIdx);
            matched_map_kp_ids_pnp.push_back(map_kp_id);

            current_img_to_map_keypoint_idx[best_match.queryIdx] = map_kp_id;

            auto& map_keypoint = map_keypoints[map_kp_id];
            addOrUpdateObservation(map_keypoint, cur_kf.id, cur_kf.color_data.value().frame_id,
                                   current_img_keypoints[best_match.queryIdx].pt.x,
                                   current_img_keypoints[best_match.queryIdx].pt.y, "tracking");
        }
    }

    if (object_points_for_pnp.size() < min_matches_for_matching_) {
        LOG(ERROR) << "Not enough matches for PnP";
        return std::nullopt;
    }

    return std::nullopt;
    // TODO: Maybe add the below logic for better pose estimation which
    // seems like an overkill.

    cv::Mat rvec, tvec, R_pnp;
    std::vector<int> inliers_pnp;

    bool pnp_success =
        cv::solvePnPRansac(object_points_for_pnp, image_points_for_pnp,
                           cur_kf.camera_info.value().k, cur_kf.camera_info.value().d, rvec, tvec,
                           false, 100, 8.0, 0.99, inliers_pnp, cv::SOLVEPNP_EPNP);
    if (!pnp_success || inliers_pnp.size() < min_matches_for_matching_ / 2) {
        LOG(ERROR) << "PnP failed";
        return std::nullopt;
    }

    // TODO: If the tracked kps in the current image
    //   is not done above then populate only those
    //   tracked which are inliers here.

    cv::Rodrigues(rvec, R_pnp);
    core::types::Pose transform;
    transform.position.x() = tvec.at<double>(0);
    transform.position.y() = tvec.at<double>(1);
    transform.position.z() = tvec.at<double>(2);

    Eigen::Matrix3d R_pnp_eigen;
    R_pnp_eigen << R_pnp.at<double>(0, 0), R_pnp.at<double>(0, 1), R_pnp.at<double>(0, 2),
        R_pnp.at<double>(1, 0), R_pnp.at<double>(1, 1), R_pnp.at<double>(1, 2),
        R_pnp.at<double>(2, 0), R_pnp.at<double>(2, 1), R_pnp.at<double>(2, 2);
    transform.orientation = Eigen::Quaterniond(R_pnp_eigen);
    transform.frame_id = "relative_tf";
}

std::optional<core::types::Pose> OrbTracker::matchAndTriangulate(
    const std::vector<cv::KeyPoint>& cur_img_kps, const cv::Mat& cur_img_desc,
    const std::vector<cv::KeyPoint>& prev_img_kps, const cv::Mat& prev_img_desc,
    const core::types::KeyFrame& cur_frame, const core::types::KeyFrame& prev_frame,
    const cv::Mat& K, const std::map<uint32_t, uint32_t>& current_img_to_map_keypoint_idx,
    std::map<uint32_t, core::types::Keypoint>& map_keypoints) {
    std::vector<cv::DMatch> matches;
    std::vector<std::vector<cv::DMatch>> knn_matches;

    LOG(INFO) << "Performing matchAndTriangulate with: " << "\n\tCurrent image keypoints: "
              << cur_img_kps.size() << " descs: " << cur_img_desc.size()
              << "\n\tPrevious image keypoints: " << prev_img_kps.size()
              << " descs: " << prev_img_desc.size();
    matcher_->knnMatch(prev_img_desc, cur_img_desc, knn_matches, 2);

    std::vector<cv::DMatch> good_matches;
    const float ratio_thresh = 0.75f;
    for (uint32_t i = 0; i < knn_matches.size(); ++i) {
        if (knn_matches[i].size() == 2 &&
            knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    if (good_matches.size() < min_matches_for_matching_) {
        LOG(ERROR) << "Not enough matches available";
        return std::nullopt;
    }

    cv::Mat img_matches;
    cv::drawMatches(prev_frame.color_data.value().data, prev_img_kps,
                    cur_frame.color_data.value().data, cur_img_kps, good_matches, img_matches,
                    cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::DEFAULT);
    if (!img_matches.empty()) {
        cv::imwrite(
            "/data/robot/bags/house11/orb/matches_" + std::to_string(prev_frame.id) + ".png",
            img_matches);
    } else {
        LOG(ERROR) << "Failed to draw matches";
    }

    LOG(INFO) << "Found good matches: " << good_matches.size();
    std::vector<cv::Point2f> prev_img_points, cur_img_points;
    for (const auto& match : good_matches) {
        prev_img_points.push_back(prev_img_kps[match.queryIdx].pt);
        cur_img_points.push_back(cur_img_kps[match.trainIdx].pt);
    }

    cv::Mat E, R, t, inlier_mask_E;
    LOG(INFO) << "dev: Camera matrix size: " << K.size();
    E = cv::findEssentialMat(prev_img_points, cur_img_points, K, cv::RANSAC, 0.999, 1.0,
                             inlier_mask_E);
    LOG(INFO) << "dev: Camera matrix K:\n" << K << "\nEssential matrix\n" << E;

    if (E.empty()) {
        LOG(ERROR) << "Essential matrix not found";
        return std::nullopt;
    }

    int inlier_count_E =
        cv::recoverPose(E, prev_img_points, cur_img_points, K, R, t, inlier_mask_E);
    if (inlier_count_E < min_matches_for_matching_ / 2) {
        LOG(ERROR) << "Not enough inliers after recovering the pose";
        return std::nullopt;
    }

    if (cv::norm(t) < 1e-3) {
        LOG(ERROR) << "Translation should not be zero";
        return std::nullopt;
    }
    cv::Mat transform(3, 4, CV_64F);
    R.copyTo(transform.colRange(0, 3));
    t.copyTo(transform.col(3));

    Eigen::Matrix3d rotE;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotE(i, j) = R.at<double>(i, j);
        }
    }
    Eigen::Vector3d trE(t.at<double>(0), t.at<double>(1), t.at<double>(2));
    Eigen::Isometry3d essentialTransform;
    essentialTransform.linear() = rotE;
    essentialTransform.translation() = trE;
    LOG(INFO) << "Recovered pose: " << t.t();
    LOG(INFO) << "Essential eigen transform: " << essentialTransform.translation().transpose();

    auto odomTransform = stf::getRelative(cur_frame.pose, prev_frame.pose,
                                          prev_frame.color_data.value().frame_id, *tft_);
    LOG(INFO) << "Actual pose: " << odomTransform.translation().transpose()
              << " rot: " << stf::getRPY(odomTransform);

    cv::Mat points_homogeneous;
    std::vector<cv::Point2f> inlier_pts1, inlier_pts2;
    std::vector<cv::DMatch> inlier_matches_for_triangulation;

    cv::Mat inv_K = K.inv();
    for (uint32_t i = 0; i < good_matches.size(); ++i) {
        if (inlier_mask_E.at<uchar>(i)) {
            inlier_pts1.push_back(prev_img_points[i]);
            inlier_pts2.push_back(cur_img_points[i]);
            inlier_matches_for_triangulation.push_back(good_matches[i]);
        }
    }
    LOG(INFO) << "Inlier matches used in triangulation: "
              << inlier_matches_for_triangulation.size();

    cv::Mat img_matches_inlier;
    cv::drawMatches(prev_frame.color_data.value().data, prev_img_kps,
                    cur_frame.color_data.value().data, cur_img_kps,
                    inlier_matches_for_triangulation, img_matches_inlier, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);
    if (!img_matches_inlier.empty()) {
        cv::imwrite(
            "/data/robot/bags/house11/orb/inlier_matches_" + std::to_string(prev_frame.id) + ".png",
            img_matches_inlier);
    } else {
        LOG(ERROR) << "Failed to draw matches";
    }

    if (inlier_pts1.empty()) {
        LOG(ERROR) << "No points for triangulation";
        return std::nullopt;
    }

    // cv::triangulatePoints(P1, P2, inlier_pts1, inlier_pts2, points_homogeneous);

    std::vector<Eigen::Vector3d> triangulated_points;

    // Determine which transform to use based on visual odometry state
    bool prefer_essential_matrix = false;

    // If visual odometry is enabled, prefer essential matrix since TF tree may be incomplete
    if (visual_odometry_enabled_) {
        prefer_essential_matrix = true;
        LOG(INFO) << "Visual odometry mode: preferring essential matrix for triangulation";
    } else {
        LOG(INFO) << "Standard mode: trying TF tree first, essential matrix as fallback";
    }

    reconstruct.triangulate(prev_frame, cur_frame, inlier_pts1, inlier_pts2, *tft_,
                            essentialTransform, triangulated_points, prefer_essential_matrix,
                            base_link_frame_id_);

    for (uint64_t i = 0; i < triangulated_points.size(); ++i) {
        auto prevImgIdx = inlier_matches_for_triangulation[i].queryIdx;
        auto curImgIdx = inlier_matches_for_triangulation[i].trainIdx;
        if (current_img_to_map_keypoint_idx.find(curImgIdx) ==
            current_img_to_map_keypoint_idx.end()) {
            core::types::Keypoint keypoint(map_keypoints.size());
            keypoint.position = triangulated_points[i];
            keypoint.descriptor = prev_img_desc.row(prevImgIdx).clone();
            addOrUpdateObservation(keypoint, cur_frame.id, cur_frame.color_data.value().frame_id,
                                   cur_img_kps[curImgIdx].pt.x, cur_img_kps[curImgIdx].pt.y,
                                   "triangulation");
            addOrUpdateObservation(keypoint, prev_frame.id, prev_frame.color_data.value().frame_id,
                                   prev_img_kps[prevImgIdx].pt.x, prev_img_kps[prevImgIdx].pt.y,
                                   "triangulation");
            map_keypoints.insert(std::make_pair(keypoint.id(), keypoint));
        }
    }
    // if (map_keypoints.size() > 10) {
    //     exit(1);
    // }

    core::types::Pose tf;
    tf.position.x() = t.at<double>(0);
    tf.position.y() = t.at<double>(1);
    tf.position.z() = t.at<double>(2);

    Eigen::Matrix3d R_eigen;
    R_eigen << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), R.at<double>(1, 0),
        R.at<double>(1, 1), R.at<double>(1, 2), R.at<double>(2, 0), R.at<double>(2, 1),
        R.at<double>(2, 2);
    tf.orientation = Eigen::Quaterniond(R_eigen);
    tf.frame_id = "relative_tf";
    return tf;
}

std::optional<core::types::Pose> OrbTracker::matchFeaturesOnly(
    const std::vector<cv::KeyPoint>& cur_img_kps, const cv::Mat& cur_img_desc,
    const std::vector<cv::KeyPoint>& prev_img_kps, const cv::Mat& prev_img_desc,
    const core::types::KeyFrame& cur_frame, const core::types::KeyFrame& prev_frame,
    const cv::Mat& K, const std::map<uint32_t, uint32_t>& current_img_to_map_keypoint_idx,
    std::map<uint32_t, core::types::Keypoint>& map_keypoints) {
    std::vector<cv::DMatch> matches;
    std::vector<std::vector<cv::DMatch>> knn_matches;

    LOG(INFO) << "Performing matchFeaturesOnly (no triangulation) with: "
              << "\n\tCurrent image keypoints: " << cur_img_kps.size()
              << " descs: " << cur_img_desc.size()
              << "\n\tPrevious image keypoints: " << prev_img_kps.size()
              << " descs: " << prev_img_desc.size();
    matcher_->knnMatch(prev_img_desc, cur_img_desc, knn_matches, 2);

    std::vector<cv::DMatch> good_matches;
    const float ratio_thresh = 0.75f;
    for (uint32_t i = 0; i < knn_matches.size(); ++i) {
        if (knn_matches[i].size() == 2 &&
            knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    LOG(INFO) << "Curent number of mathes: " << knn_matches.size() << " " << good_matches.size();
    if (good_matches.size() < min_matches_for_matching_) {
        LOG(ERROR) << "Not enough matches available for feature creation " << good_matches.size()
                   << "/" << min_matches_for_matching_;
        return std::nullopt;
    }

    LOG(INFO) << "Found good matches: " << good_matches.size();
    std::vector<cv::Point2f> prev_img_points, cur_img_points;
    for (const auto& match : good_matches) {
        prev_img_points.push_back(prev_img_kps[match.queryIdx].pt);
        cur_img_points.push_back(cur_img_kps[match.trainIdx].pt);
    }

    cv::Mat E, R, t, inlier_mask_E;
    LOG(INFO) << "Camera matrix K:\n" << K;
    E = cv::findEssentialMat(prev_img_points, cur_img_points, K, cv::RANSAC, 0.999, 1.0,
                             inlier_mask_E);

    if (E.empty()) {
        LOG(ERROR) << "Essential matrix not found";
        return std::nullopt;
    }

    int inlier_count_E =
        cv::recoverPose(E, prev_img_points, cur_img_points, K, R, t, inlier_mask_E);
    if (inlier_count_E < min_matches_for_matching_ / 2) {
        LOG(ERROR) << "Not enough inliers after recovering the pose";
        return std::nullopt;
    }

    if (cv::norm(t) < 1e-3) {
        LOG(ERROR) << "Translation should not be zero";
        return std::nullopt;
    }

    std::vector<cv::DMatch> inlier_matches_for_keypoint_creation;
    for (uint32_t i = 0; i < good_matches.size(); ++i) {
        if (inlier_mask_E.at<uchar>(i)) {
            inlier_matches_for_keypoint_creation.push_back(good_matches[i]);
        }
    }

    LOG(INFO) << "Inlier matches used for keypoint creation: "
              << inlier_matches_for_keypoint_creation.size();

    // CREATE MAP KEYPOINTS WITHOUT 3D POSITION (defer triangulation to graph_adapter.cpp)
    for (uint64_t i = 0; i < inlier_matches_for_keypoint_creation.size(); ++i) {
        auto prevImgIdx = inlier_matches_for_keypoint_creation[i].queryIdx;
        auto curImgIdx = inlier_matches_for_keypoint_creation[i].trainIdx;

        // Only create new map keypoints for unmatched features
        if (current_img_to_map_keypoint_idx.find(curImgIdx) ==
            current_img_to_map_keypoint_idx.end()) {
            core::types::Keypoint keypoint(map_keypoints.size());
            keypoint.position = Eigen::Vector3d::Zero();  // No 3D position yet
            keypoint.needs_triangulation =
                true;  // Mark for triangulation during batch optimization
            keypoint.descriptor = prev_img_desc.row(prevImgIdx).clone();

            // Add observations from both keyframes
            addOrUpdateObservation(keypoint, cur_frame.id, cur_frame.color_data.value().frame_id,
                                   cur_img_kps[curImgIdx].pt.x, cur_img_kps[curImgIdx].pt.y,
                                   "deferred_matching");
            addOrUpdateObservation(keypoint, prev_frame.id, prev_frame.color_data.value().frame_id,
                                   prev_img_kps[prevImgIdx].pt.x, prev_img_kps[prevImgIdx].pt.y,
                                   "deferred_matching");

            map_keypoints.insert(std::make_pair(keypoint.id(), keypoint));

            LOG(INFO) << "Created map keypoint " << keypoint.id()
                      << " without 3D position (marked for deferred triangulation)";
        }
    }

    // Return relative pose for potential odometry use
    core::types::Pose tf;
    tf.position.x() = t.at<double>(0);
    tf.position.y() = t.at<double>(1);
    tf.position.z() = t.at<double>(2);

    Eigen::Matrix3d R_eigen;
    R_eigen << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), R.at<double>(1, 0),
        R.at<double>(1, 1), R.at<double>(1, 2), R.at<double>(2, 0), R.at<double>(2, 1),
        R.at<double>(2, 2);
    tf.orientation = Eigen::Quaterniond(R_eigen);
    tf.frame_id = "relative_tf";

    LOG(INFO) << "Feature matching completed - created "
              << inlier_matches_for_keypoint_creation.size()
              << " new map keypoints (triangulation deferred to graph_adapter.cpp)";

    return tf;
}

KeyframeDecision OrbTracker::evaluateKeyframeNecessity(const core::types::Image& current_image,
                                                       const core::types::Image& previous_image,
                                                       const core::types::CameraInfo& camera_info) {
    KeyframeDecision decision;
    decision.should_create_keyframe = false;
    decision.inlier_count = 0;
    decision.translation_magnitude = 0.0;
    decision.reason = "Unknown";

    // Extract images
    cv::Mat cur_img = current_image.data;
    cv::Mat prev_img = previous_image.data;

    if (cur_img.empty() || prev_img.empty()) {
        decision.reason = "Empty images provided";
        return decision;
    }

    // Detect ORB features in both images
    std::vector<cv::KeyPoint> current_keypoints, previous_keypoints;
    cv::Mat current_descriptors, previous_descriptors;

    orb_detector_->detectAndCompute(cur_img, cv::noArray(), current_keypoints, current_descriptors);
    orb_detector_->detectAndCompute(prev_img, cv::noArray(), previous_keypoints,
                                    previous_descriptors);

    if (current_keypoints.empty() || previous_keypoints.empty() || current_descriptors.empty() ||
        previous_descriptors.empty()) {
        decision.reason = "Insufficient keypoints detected";
        return decision;
    }

    // Match features between images
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher_->knnMatch(previous_descriptors, current_descriptors, knn_matches, 2);

    std::vector<cv::DMatch> good_matches;
    const float ratio_thresh = 0.75f;
    for (size_t i = 0; i < knn_matches.size(); ++i) {
        if (knn_matches[i].size() == 2 &&
            knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            good_matches.push_back(knn_matches[i][0]);
        }
    }

    if (good_matches.size() < min_matches_for_matching_) {
        decision.reason = "Insufficient good matches (" + std::to_string(good_matches.size()) +
                          " < " + std::to_string(min_matches_for_matching_) + ")";
        return decision;
    }

    // Extract matched points
    std::vector<cv::Point2f> prev_points, cur_points;
    for (const auto& match : good_matches) {
        prev_points.push_back(previous_keypoints[match.queryIdx].pt);
        cur_points.push_back(current_keypoints[match.trainIdx].pt);
    }

    // Get camera matrix
    cv::Mat K(camera_info.k);
    K = K.reshape(1, 3);

    return computeEssentialMatrix(prev_points, cur_points, K);
}

KeyframeDecision OrbTracker::computeEssentialMatrix(const std::vector<cv::Point2f>& prev_points,
                                                    const std::vector<cv::Point2f>& cur_points,
                                                    const cv::Mat& K) {
    KeyframeDecision decision;
    decision.should_create_keyframe = false;
    decision.inlier_count = 0;
    decision.translation_magnitude = 0.0;

    // Compute essential matrix
    cv::Mat E, inlier_mask;
    E = cv::findEssentialMat(prev_points, cur_points, K, cv::RANSAC, 0.999, 1.0, inlier_mask);

    if (E.empty()) {
        decision.reason = "Essential matrix computation failed";
        return decision;
    }

    // Recover pose from essential matrix
    cv::Mat R, t;
    int inlier_count = cv::recoverPose(E, prev_points, cur_points, K, R, t, inlier_mask);

    decision.inlier_count = inlier_count;
    decision.translation_magnitude = cv::norm(t);

    // Convert to Eigen for easier manipulation
    Eigen::Matrix3d R_eigen;
    Eigen::Vector3d t_eigen;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_eigen(i, j) = R.at<double>(i, j);
        }
        t_eigen(i) = t.at<double>(i);
    }

    // Create relative pose
    core::types::Pose relative_pose;
    relative_pose.position = t_eigen;
    relative_pose.orientation = Eigen::Quaterniond(R_eigen);
    decision.relative_pose = relative_pose;

    // Calculate rotation angle (angle-axis representation)
    Eigen::AngleAxisd angle_axis(R_eigen);
    double rotation_angle = std::abs(angle_axis.angle());  // in radians

    // Decision criteria
    bool sufficient_inliers = inlier_count >= min_inliers_for_keyframe_;
    bool sufficient_translation = decision.translation_magnitude >= min_translation_for_keyframe_;
    bool sufficient_rotation = rotation_angle >= (5.0 * M_PI / 180.0);  // 5 degrees in radians

    // Build decision reason
    std::stringstream reason;
    reason << "Inliers: " << inlier_count << "/" << min_inliers_for_keyframe_
           << ", Translation: " << std::fixed << std::setprecision(3)
           << decision.translation_magnitude << "m/" << min_translation_for_keyframe_ << "m"
           << ", Rotation: " << std::fixed << std::setprecision(1)
           << (rotation_angle * 180.0 / M_PI) << "°/5.0°";

    decision.reason = reason.str();

    // Create keyframe if any threshold is exceeded AND we have sufficient inliers
    decision.should_create_keyframe =
        sufficient_inliers && (sufficient_translation || sufficient_rotation);

    return decision;
}

// Visual odometry implementation
void OrbTracker::enableVisualOdometry(bool enable) {
    visual_odometry_enabled_ = enable;
    if (enable) {
        LOG(INFO) << "Visual odometry enabled for OrbTracker";
        // Reset visual odometry state
        vo_initialized_ = false;
        current_visual_pose_ = core::types::Pose();
        last_vo_timestamp_ = 0.0;
    } else {
        LOG(INFO) << "Visual odometry disabled for OrbTracker";
    }
}

void OrbTracker::setVisualOdometryParams(double estimated_speed) {
    vo_estimated_speed_ = estimated_speed;
    LOG(INFO) << "Visual odometry speed parameter set to " << estimated_speed << " m/s";
}

std::optional<core::types::Pose> OrbTracker::estimateVisualOdometryPose(
    const core::types::Image& current_image, const core::types::Image& previous_image,
    const core::types::CameraInfo& camera_info, double timestamp, double previous_timestamp) {
    if (!visual_odometry_enabled_) {
        LOG(WARNING) << "Visual odometry not enabled, call enableVisualOdometry() first";
        return std::nullopt;
    }

    // Convert images to grayscale if needed
    cv::Mat current_gray, previous_gray;

    // Convert current image
    if (current_image.encoding == "rgb8" || current_image.encoding == "bgr8") {
        if (current_image.encoding == "rgb8") {
            cv::cvtColor(current_image.data, current_gray, cv::COLOR_RGB2GRAY);
        } else {
            cv::cvtColor(current_image.data, current_gray, cv::COLOR_BGR2GRAY);
        }
    } else if (current_image.encoding == "mono8" || current_image.encoding == "8UC1") {
        current_gray = current_image.data.clone();
    } else {
        LOG(ERROR) << "Unsupported image encoding for visual odometry: " << current_image.encoding;
        return std::nullopt;
    }

    // Convert previous image
    if (previous_image.encoding == "rgb8" || previous_image.encoding == "bgr8") {
        if (previous_image.encoding == "rgb8") {
            cv::cvtColor(previous_image.data, previous_gray, cv::COLOR_RGB2GRAY);
        } else {
            cv::cvtColor(previous_image.data, previous_gray, cv::COLOR_BGR2GRAY);
        }
    } else if (previous_image.encoding == "mono8" || previous_image.encoding == "8UC1") {
        previous_gray = previous_image.data.clone();
    } else {
        LOG(ERROR) << "Unsupported previous image encoding for visual odometry: "
                   << previous_image.encoding;
        return std::nullopt;
    }

    // Initialize on first call
    if (!vo_initialized_) {
        current_visual_pose_.position = Eigen::Vector3d::Zero();
        current_visual_pose_.orientation = Eigen::Quaterniond::Identity();
        current_visual_pose_.timestamp = previous_timestamp;

        // Detect features in previous image for next iteration
        orb_detector_->detectAndCompute(previous_gray, cv::noArray(), previous_vo_keypoints_,
                                        previous_vo_descriptors_);
        previous_vo_image_ = previous_gray.clone();
        last_vo_timestamp_ = previous_timestamp;
        vo_initialized_ = true;

        // Clear stored data on initialization
        last_vo_data_.clear();

        LOG(INFO) << "Visual odometry initialized with " << previous_vo_keypoints_.size()
                  << " features";
        return current_visual_pose_;
    }

    // Clear previous data
    last_vo_data_.clear();

    // Detect features in current frame
    std::vector<cv::KeyPoint> current_keypoints;
    cv::Mat current_descriptors;
    orb_detector_->detectAndCompute(current_gray, cv::noArray(), current_keypoints,
                                    current_descriptors);

    if (current_keypoints.size() < 50 || previous_vo_keypoints_.size() < 50) {
        LOG(WARNING) << "Insufficient features for visual odometry: current="
                     << current_keypoints.size() << ", previous=" << previous_vo_keypoints_.size();
        return std::nullopt;
    }

    // Match features between frames
    std::vector<cv::DMatch> matches;
    matcher_->match(previous_vo_descriptors_, current_descriptors, matches);

    // Filter good matches
    std::vector<cv::DMatch> good_matches;
    double min_dist = 30.0;
    for (const auto& match : matches) {
        if (match.distance < 2 * min_dist && match.distance < 50) {
            good_matches.push_back(match);
        }
    }

    if (good_matches.size() < 20) {
        LOG(WARNING) << "Insufficient good matches for visual odometry: " << good_matches.size();
        return std::nullopt;
    }

    // Extract matched points
    std::vector<cv::Point2f> prev_pts, curr_pts;
    for (const auto& match : good_matches) {
        prev_pts.push_back(previous_vo_keypoints_[match.queryIdx].pt);
        curr_pts.push_back(current_keypoints[match.trainIdx].pt);
    }

    // Create camera matrix from camera info
    if (camera_info.k.size() != 9) {
        LOG(ERROR) << "Invalid camera matrix size: " << camera_info.k.size();
        return std::nullopt;
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) << camera_info.k[0], camera_info.k[1], camera_info.k[2],
                 camera_info.k[3], camera_info.k[4], camera_info.k[5], camera_info.k[6],
                 camera_info.k[7], camera_info.k[8]);

    // Estimate essential matrix and recover pose
    cv::Mat E = cv::findEssentialMat(prev_pts, curr_pts, K, cv::RANSAC, 0.999, 1.0);
    if (E.empty()) {
        LOG(WARNING) << "Failed to estimate essential matrix";
        return std::nullopt;
    }

    cv::Mat R, t;
    int inliers = cv::recoverPose(E, prev_pts, curr_pts, K, R, t);

    if (inliers < 15) {
        LOG(WARNING) << "Insufficient inliers for pose recovery: " << inliers;
        return std::nullopt;
    }

    // Convert OpenCV matrices to Eigen
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation(i, j) = R.at<double>(i, j);
        }
        translation(i) = t.at<double>(i, 0);
    }

    // Log the raw essential matrix output for debugging
    LOG(INFO) << "Raw essential matrix output from cv::recoverPose:";
    LOG(INFO) << "  Raw translation (unit vector): " << translation.transpose();
    LOG(INFO) << "  Raw translation magnitude: " << translation.norm();
    LOG(INFO) << "  Inlier count: " << inliers;

    // Scale translation using estimated speed
    double dt = timestamp - last_vo_timestamp_;
    LOG(INFO) << "Scaling translation with speed=" << vo_estimated_speed_
              << " m/s, dt=" << std::fixed << std::setprecision(4) << dt << "s";

    // Eigen::Vector3d scaled_translation = translation * vo_estimated_speed_ * dt;
    Eigen::Vector3d scaled_translation = translation;
    LOG(INFO) << "Scaled translation for pose accumulation: " << scaled_translation.transpose();
    LOG(INFO) << "Scaling factor applied: " << (vo_estimated_speed_ * dt);

    // Fix coordinate frame and direction issues for visual odometry:
    // Camera motion is opposite to robot motion in world frame
    // translation = -translation;

    // Update current pose (compose with previous pose)
    Eigen::Quaterniond delta_rotation(rotation);
    auto current_pose_eigen = current_visual_pose_.getEigenIsometry();
    Eigen::Isometry3d relative_transform;
    relative_transform.linear() = rotation;
    relative_transform.translation() = scaled_translation;
    Eigen::Isometry3d next_pose_in_eigen = current_pose_eigen * relative_transform;
    Eigen::Quaterniond next_orientation(next_pose_in_eigen.rotation());
    current_visual_pose_.position = next_pose_in_eigen.translation();
    current_visual_pose_.orientation = next_orientation;
    current_visual_pose_.timestamp = timestamp;

    // Store computed data for potential triangulation use
    last_vo_data_.prev_matched_points = prev_pts;
    last_vo_data_.curr_matched_points = curr_pts;
    last_vo_data_.prev_keypoints = previous_vo_keypoints_;
    last_vo_data_.curr_keypoints = current_keypoints;
    last_vo_data_.prev_descriptors = previous_vo_descriptors_.clone();
    last_vo_data_.curr_descriptors = current_descriptors.clone();

    // Store essential matrix transform with SCALED translation to match pose computation
    Eigen::Isometry3d essential_transform;
    essential_transform.linear() = rotation;
    essential_transform.translation() =
        scaled_translation;  // Use scaled translation, not raw unit vector
    last_vo_data_.essential_transform = essential_transform;
    last_vo_data_.inlier_count = inliers;
    last_vo_data_.data_valid = true;

    LOG(INFO) << "Stored essential matrix transform for triangulation:";
    LOG(INFO) << "  Scaled translation: " << scaled_translation.transpose();
    LOG(INFO) << "  Translation magnitude: " << scaled_translation.norm();
    LOG(INFO) << "  This should now match the scale used in pose accumulation";

    // Update previous frame data for next iteration
    previous_vo_image_ = current_gray.clone();
    previous_vo_keypoints_ = current_keypoints;
    previous_vo_descriptors_ = current_descriptors.clone();
    last_vo_timestamp_ = timestamp;

    LOG(INFO) << "Visual odometry estimated pose: " << current_visual_pose_.position.transpose()
              << " with " << inliers << " inliers from " << good_matches.size() << " matches"
              << " (scaled data stored for triangulation)";

    return current_visual_pose_;
}

// Direct triangulation method for visual odometry mode
void OrbTracker::performDirectTriangulation(
    const core::types::KeyFrame& current_kf, const core::types::KeyFrame& previous_kf,
    std::map<uint32_t, core::types::Keypoint>& map_keypoints) {
    if (!last_vo_data_.data_valid) {
        LOG(WARNING) << "No valid visual odometry data available for direct triangulation";
        return;
    }

    LOG(INFO) << "Performing hybrid direct triangulation using stored visual odometry data";
    LOG(INFO) << "Matched points: " << last_vo_data_.prev_matched_points.size()
              << ", Inliers: " << last_vo_data_.inlier_count;

    // PHASE 1: Use existing track() method for descriptor matching against existing map
    std::map<uint32_t, uint32_t> current_img_to_map_keypoint_idx;
    size_t initial_map_size = map_keypoints.size();

    if (!map_keypoints.empty()) {
        LOG(INFO) << "Tracking against existing map with " << map_keypoints.size() << " points";

        // Call existing track method to match current frame against map
        track(current_kf, last_vo_data_.curr_keypoints, last_vo_data_.curr_descriptors,
              map_keypoints, current_img_to_map_keypoint_idx);

        LOG(INFO) << "Tracking matched " << current_img_to_map_keypoint_idx.size()
                  << " features against existing map";
    } else {
        LOG(INFO) << "Empty map - all features will be triangulated as new points";
    }

    // PHASE 2: Filter to only unmatched features for triangulation
    std::vector<cv::Point2f> unmatched_prev_pts, unmatched_curr_pts;
    std::vector<size_t> unmatched_curr_indices;

    // Go through each matched point pair and check if current point was matched to map
    for (size_t i = 0; i < last_vo_data_.curr_matched_points.size(); ++i) {
        cv::Point2f curr_pt = last_vo_data_.curr_matched_points[i];

        // Find corresponding keypoint index in current frame
        int curr_kp_idx = -1;
        float min_distance = 2.0f;  // Small threshold for exact matching

        for (size_t kp_idx = 0; kp_idx < last_vo_data_.curr_keypoints.size(); ++kp_idx) {
            float dist = cv::norm(last_vo_data_.curr_keypoints[kp_idx].pt - curr_pt);
            if (dist < min_distance) {
                min_distance = dist;
                curr_kp_idx = kp_idx;
            }
        }

        // Check if this keypoint was matched to existing map
        bool is_matched_to_map =
            (curr_kp_idx >= 0) && (current_img_to_map_keypoint_idx.find(curr_kp_idx) !=
                                   current_img_to_map_keypoint_idx.end());

        if (!is_matched_to_map) {
            // This feature pair is unmatched - add to triangulation list
            unmatched_prev_pts.push_back(last_vo_data_.prev_matched_points[i]);
            unmatched_curr_pts.push_back(last_vo_data_.curr_matched_points[i]);
            unmatched_curr_indices.push_back(curr_kp_idx >= 0 ? curr_kp_idx : i);
        }
    }

    LOG(INFO) << "Features for triangulation: " << unmatched_prev_pts.size() << " (out of "
              << last_vo_data_.curr_matched_points.size() << " total matches)";

    // PHASE 3: Create map keypoints for unmatched features (defer triangulation to batch
    // optimization)
    if (!unmatched_prev_pts.empty()) {
        LOG(INFO) << "Creating " << unmatched_prev_pts.size()
                  << " new map keypoints without 3D position (deferred triangulation to "
                     "graph_adapter.cpp)";

        // Create new map keypoints WITHOUT 3D position (defer triangulation to batch optimization)
        size_t points_added = 0;

        for (size_t i = 0; i < unmatched_curr_indices.size(); ++i) {
            size_t curr_kp_idx = unmatched_curr_indices[i];

            if (curr_kp_idx < last_vo_data_.curr_keypoints.size()) {
                // Create new map keypoint WITHOUT 3D position - NEW ARCHITECTURE
                core::types::Keypoint keypoint(map_keypoints.size());
                keypoint.position = Eigen::Vector3d::Zero();  // No 3D position yet
                keypoint.needs_triangulation =
                    true;  // Mark for triangulation during batch optimization
                keypoint.descriptor = last_vo_data_.curr_descriptors.row(curr_kp_idx).clone();

                // Add observations from both keyframes
                addOrUpdateObservation(
                    keypoint, current_kf.id, current_kf.color_data.value().frame_id,
                    unmatched_curr_pts[i].x, unmatched_curr_pts[i].y, "deferred_visual_odometry");
                addOrUpdateObservation(
                    keypoint, previous_kf.id, previous_kf.color_data.value().frame_id,
                    unmatched_prev_pts[i].x, unmatched_prev_pts[i].y, "deferred_visual_odometry");

                map_keypoints.insert(std::make_pair(keypoint.id(), keypoint));
                points_added++;
            }
        }

        LOG(INFO) << "Visual odometry: Added " << points_added
                  << " new map keypoints (without 3D position, triangulation deferred to "
                     "graph_adapter.cpp)";
        LOG(INFO) << "Map size: " << initial_map_size << " → " << map_keypoints.size() << " (+"
                  << (map_keypoints.size() - initial_map_size) << ")";
    } else {
        LOG(INFO) << "Visual odometry: No unmatched features to create as map keypoints";
    }

    // Clear the data after use
    last_vo_data_.clear();
}

// New interface using map_store and keyframe_ids
void OrbTracker::performDirectTriangulationWithMapStore(
    uint64_t current_kf_id, uint64_t previous_kf_id, const core::storage::MapStore& map_store,
    std::map<uint32_t, core::types::Keypoint>& map_keypoints) {
    if (!last_vo_data_.data_valid) {
        LOG(WARNING) << "No valid visual odometry data available for direct triangulation";
        return;
    }

    // Get keyframes from map_store
    auto current_kf_ptr = map_store.getKeyFrame(current_kf_id);
    auto previous_kf_ptr = map_store.getKeyFrame(previous_kf_id);

    if (!current_kf_ptr || !previous_kf_ptr) {
        LOG(ERROR) << "Failed to retrieve keyframes " << current_kf_id << " and/or "
                   << previous_kf_id << " from map_store for triangulation";
        return;
    }

    LOG(INFO) << "Performing direct triangulation with map_store for keyframes " << previous_kf_id
              << " → " << current_kf_id;

    // Call the legacy method with retrieved keyframes
    performDirectTriangulation(*current_kf_ptr, *previous_kf_ptr, map_keypoints);
}

// New matchAndTriangulate using map_store
std::optional<core::types::Pose> OrbTracker::matchAndTriangulateWithMapStore(
    const std::vector<cv::KeyPoint>& cur_img_kps, const cv::Mat& cur_img_desc,
    const std::vector<cv::KeyPoint>& prev_img_kps, const cv::Mat& prev_img_desc,
    uint64_t cur_frame_id, uint64_t prev_frame_id, const core::storage::MapStore& map_store,
    const cv::Mat& K, const std::map<uint32_t, uint32_t>& current_img_to_map_keypoint_idx,
    std::map<uint32_t, core::types::Keypoint>& map_keypoints) {
    // Get keyframes from map_store
    auto cur_frame_ptr = map_store.getKeyFrame(cur_frame_id);
    auto prev_frame_ptr = map_store.getKeyFrame(prev_frame_id);

    if (!cur_frame_ptr || !prev_frame_ptr) {
        LOG(ERROR) << "Failed to retrieve keyframes " << cur_frame_id << " and/or " << prev_frame_id
                   << " from map_store for match and triangulate";
        return std::nullopt;
    }

    LOG(INFO) << "Match and triangulate with map_store for keyframes " << prev_frame_id << " → "
              << cur_frame_id;

    // Call the legacy method with retrieved keyframes
    return matchAndTriangulate(cur_img_kps, cur_img_desc, prev_img_kps, prev_img_desc,
                               *cur_frame_ptr, *prev_frame_ptr, K, current_img_to_map_keypoint_idx,
                               map_keypoints);
}

}  // namespace image

}  // namespace tracking

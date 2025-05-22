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

#include <logging/logging.hpp>

#include <2d/orb_tracker.hpp>
#include <stf/transform_utils.hpp>

#include <opencv2/core/eigen.hpp>
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
      matcher_(cv::BFMatcher::create(cv::NORM_HAMMING)),
      orb_detector_(cv::ORB::create(num_features_, scale_factor_, n_levels_)),
      base_link_frame_id_("base_link") {}

void OrbTracker::addCameraInfo(const core::types::CameraInfo& cam_info) {
    cam_infos_[cam_info.frame_id] = cam_info;
}

void OrbTracker::addCameraPose(const core::types::Pose& cam_pose, std::string camera_frame) {
    cam_poses_[camera_frame] = cam_pose;
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
        LOG(ERROR) << "No keypoints detected in keyframe";
        return std::nullopt;
    }

    if (previous_img_descriptors.empty() || current_img_descriptors.empty()) {
        LOG(ERROR) << "No descriptors in the keyframe";
        return std::nullopt;
    }

    LOG(INFO) << "dev: Calling match and triangulate";

    // Assuming the images are from the same camera. Need to mofiy such that
    // it can be from different cameras.
    // cv::Mat K(3, 3, CV_64F, cam_infos_[current_kf.color_data.value().frame_id].k);
    // cv::Mat K(3, 3, CV_64F, current_kf.camera_info.value().k.data());
    auto cam_info = cam_infos_.find(cur_frame);
    if (cam_info == cam_infos_.end()) {
        LOG(ERROR) << "Unable to find camera info";
        return std::nullopt;
    }
    cv::Mat K(cam_info->second.k);
    K = K.reshape(1, 3);
    auto transform =
        matchAndTriangulate(untracked_cur_img_keypoints, untracked_cur_img_descriptors,
                            previous_img_keypoints, previous_img_descriptors, current_kf,
                            previous_kf, K, current_img_to_map_keypoint_idx, map_keypoints);

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

    // PHASE 3: Triangulate only unmatched features
    if (!unmatched_prev_pts.empty()) {
        std::vector<Eigen::Vector3d> triangulated_points;

        // Call triangulation with only unmatched feature pairs
        bool triangulation_success = reconstruct.triangulate(
            previous_kf, current_kf, unmatched_prev_pts, unmatched_curr_pts, *tft_,
            last_vo_data_.essential_transform, triangulated_points, true, base_link_frame_id_);

        LOG(INFO) << "Triangulation produced " << triangulated_points.size()
                  << " new 3D points from " << unmatched_prev_pts.size() << " unmatched features";

        // Create new map keypoints only for successfully triangulated unmatched features
        size_t points_added = 0;

        for (size_t i = 0; i < triangulated_points.size() && i < unmatched_curr_indices.size();
             ++i) {
            size_t curr_kp_idx = unmatched_curr_indices[i];

            if (curr_kp_idx < last_vo_data_.curr_keypoints.size()) {
                // Create new map keypoint
                core::types::Keypoint keypoint(map_keypoints.size());
                keypoint.position = triangulated_points[i];
                keypoint.descriptor = last_vo_data_.curr_descriptors.row(curr_kp_idx).clone();

                // Add observations from both keyframes
                addOrUpdateObservation(
                    keypoint, current_kf.id, current_kf.color_data.value().frame_id,
                    unmatched_curr_pts[i].x, unmatched_curr_pts[i].y, "direct_triangulation");
                addOrUpdateObservation(
                    keypoint, previous_kf.id, previous_kf.color_data.value().frame_id,
                    unmatched_prev_pts[i].x, unmatched_prev_pts[i].y, "direct_triangulation");

                map_keypoints.insert(std::make_pair(keypoint.id(), keypoint));
                points_added++;
            }
        }

        LOG(INFO) << "Added " << points_added << " new map keypoints from triangulation";
        LOG(INFO) << "Map size: " << initial_map_size << " → " << map_keypoints.size() << " (+"
                  << (map_keypoints.size() - initial_map_size) << ")";
    } else {
        LOG(INFO) << "No unmatched features to triangulate";
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

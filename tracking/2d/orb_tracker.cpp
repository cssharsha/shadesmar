#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include <logging/logging.hpp>

#include <2d/orb_tracker.hpp>
namespace tracking {
namespace image {

OrbTracker::OrbTracker(uint32_t num_features, float scal_factor, uint32_t levels)
    : num_features_(num_features),
      scale_factor_(scal_factor),
      n_levels_(levels),
      matcher_(cv::BFMatcher::create(cv::NORM_HAMMING)),
      orb_detector_(cv::ORB::create(num_features_, scale_factor_, n_levels_)) {}

void OrbTracker::addCameraInfo(const core::types::CameraInfo& cam_info) {
    cam_infos_[cam_info.frame_id] = cam_info;
}

std::optional<core::types::Pose> OrbTracker::operator()(
    const core::types::KeyFrame& current_kf, const core::types::KeyFrame& previous_kf,
    std::map<uint32_t, core::types::Keypoint>& map_keypoints) {
    cv::Mat cur_img = current_kf.color_data.value().data;
    cv::Mat prev_img = previous_kf.color_data.value().data;

    std::vector<cv::KeyPoint> current_img_keypoints;
    cv::Mat current_img_descriptors;
    orb_detector_->detectAndCompute(cur_img, cv::noArray(), current_img_keypoints,
                                    current_img_descriptors);
    std::map<uint32_t, uint32_t> current_img_to_map_keypoint_idx;

    track(current_kf, current_img_keypoints, current_img_descriptors, map_keypoints,
          current_img_to_map_keypoint_idx);

    LOG(INFO) << "dev: Finished tracking!";

    std::vector<cv::KeyPoint> untracked_cur_img_keypoints;
    cv::Mat untracked_cur_img_descriptors;

    for (uint32_t i = 0; i < current_img_keypoints.size(); ++i) {
        if (current_img_to_map_keypoint_idx.find(i) == current_img_to_map_keypoint_idx.end()) {
            untracked_cur_img_keypoints.push_back(current_img_keypoints[i]);
            untracked_cur_img_descriptors.push_back(current_img_descriptors.row(i));
        }
    }
    LOG(INFO) << "dev: Finished adding";

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
    cv::Mat K(current_kf.camera_info.value().k);
    K = K.reshape(1, 3);
    auto transform = matchAndTriangulate(untracked_cur_img_keypoints, untracked_cur_img_descriptors,
                                         previous_img_keypoints, previous_img_descriptors,
                                         current_kf, previous_kf, K, map_keypoints);

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
            map_keypoint.locations.push_back({cur_kf.id, cur_kf.color_data.value().frame_id});
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
    const cv::Mat& K, std::map<uint32_t, core::types::Keypoint>& map_keypoints) {
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

    LOG(INFO) << "Found good matches: " << good_matches.size();
    std::vector<cv::Point2f> prev_img_points, cur_img_points;
    for (const auto& match : good_matches) {
        prev_img_points.push_back(prev_img_kps[match.queryIdx].pt);
        cur_img_points.push_back(cur_img_kps[match.queryIdx].pt);
    }

    cv::Mat E, R, t, inlier_mask_E;
    LOG(INFO) << "dev: Camera matrix size: " << K.size();
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

    cv::Mat transform(3, 4, CV_64F);
    R.copyTo(transform.colRange(0, 3));
    t.copyTo(transform.col(3));

    cv::Mat P1 = K * cv::Mat::eye(3, 4, CV_64F);
    cv::Mat P2 = K * transform;

    cv::Mat points_homogeneous;
    std::vector<cv::Point2f> inlier_pts1, inlier_pts2;
    std::vector<cv::DMatch> inlier_matches_for_triangulation;

    for (uint32_t i = 0; i < good_matches.size(); ++i) {
        if (inlier_mask_E.at<uchar>(i)) {
            inlier_pts1.push_back(prev_img_points[i]);
            inlier_pts2.push_back(cur_img_points[i]);
            inlier_matches_for_triangulation.push_back(good_matches[i]);
        }
    }

    if (inlier_pts1.empty()) {
        LOG(ERROR) << "No points for triangulation";
        return std::nullopt;
    }

    cv::triangulatePoints(P1, P2, inlier_pts1, inlier_pts2, points_homogeneous);
    std::vector<uint64_t> triangulated_point_ids;
    std::map<uint64_t, int> current_landmarks_keypoint_idx;

    for (uint64_t i = 0; i < points_homogeneous.cols; ++i) {
        cv::Mat pt4D = points_homogeneous.col(i);
        auto w = pt4D.at<float>(3);
        if (std::abs(w) < 1e-6)
            continue;

        Eigen::Vector3d point_in_prev_camera_frame(pt4D.at<double>(0) / w, pt4D.at<double>(1) / w,
                                                   pt4D.at<double>(2) / w);

        if (point_in_prev_camera_frame.z() < 0)
            continue;

        auto prev_pose = prev_frame.pose;
        auto cur_pose = cur_frame.pose;

        // auto prev_camera_tf =
        //     tf_tree_->getTransform("base_link",
        //     prev_frame.color_data.value().frame_id).transform;
        // auto cur_camera_tf =
        //     tf_tree_->getTransform("base_link", cur_frame.color_data.value().frame_id).transform;

        auto prev_camera_tf = Eigen::Isometry3d::Identity();
        auto point_in_prev_frame = prev_camera_tf * point_in_prev_camera_frame;
        auto cur_camera_tf = Eigen::Isometry3d::Identity();

        Eigen::Vector3d point_in_world_frame = prev_pose * point_in_prev_frame;

        Eigen::Vector3d point_in_current_frame = cur_pose.inverse() * point_in_world_frame;
        auto point_in_current_camera_frame = cur_camera_tf.inverse() * point_in_current_frame;

        if (point_in_current_camera_frame.z() < 0)
            continue;

        core::types::Keypoint keypoint(map_keypoints.size());
        keypoint.position = point_in_world_frame;
        keypoint.descriptor =
            prev_img_desc.row(inlier_matches_for_triangulation[i].queryIdx).clone();
        keypoint.locations.push_back({cur_frame.id, cur_frame.color_data.value().frame_id});
        keypoint.locations.push_back({prev_frame.id, prev_frame.color_data.value().frame_id});
        map_keypoints.insert(std::make_pair(keypoint.id(), keypoint));
    }

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

}  // namespace image

}  // namespace tracking

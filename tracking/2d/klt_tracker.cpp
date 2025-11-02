#include <logging/logging.hpp>
#include <opencv2/imgproc.hpp>

#include "2d/klt_tracker.hpp"

namespace tracking {
namespace image {

KltTracker::KltTracker() {
    klt_detector_ = cv::GFTTDetector::create();
    matcher_ = cv::BFMatcher::create(cv::NORM_L2);
}

std::optional<core::types::Pose> KltTracker::match(const core::types::KeyFrame& prev_frame,
                                                   const core::types::KeyFrame& cur_frame) {
    cv::Mat prev_frame_gray, cur_frame_gray;
    cv::cvtColor(prev_frame.color_data.value().data, prev_frame_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(cur_frame.color_data.value().data, cur_frame_gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::KeyPoint> prev_keypoints, cur_keypoints;
    cv::Mat prev_descriptors, cur_descriptors;

    klt_detector_->detect(prev_frame_gray, prev_keypoints);
    klt_detector_->detect(cur_frame_gray, cur_keypoints);

    if (prev_keypoints.empty() || cur_keypoints.empty()) {
        LOG(WARNING) << "No keypoints detected";
        return std::nullopt;
    }

    // Use calcOpticalFlowPyrLK for KLT tracking
    std::vector<cv::Point2f> prev_points, cur_points;
    for (const auto& kp : prev_keypoints) {
        prev_points.push_back(kp.pt);
    }

    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_frame_gray, cur_frame_gray, prev_points, cur_points, status, err);

    // Filter out bad matches
    std::vector<cv::Point2f> good_prev_points, good_cur_points;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            good_prev_points.push_back(prev_points[i]);
            good_cur_points.push_back(cur_points[i]);
        }
    }

    if (good_prev_points.size() < 10) {
        LOG(WARNING) << "Not enough good matches: " << good_prev_points.size();
        return std::nullopt;
    }

    cv::Mat inliers;
    auto homography = cv::findHomography(good_prev_points, good_cur_points, cv::RANSAC, 3.0, inliers);

    if (homography.empty()) {
        LOG(WARNING) << "Failed to find homography";
        return std::nullopt;
    }

    LOG(INFO) << "Total matches: " << good_prev_points.size();
    LOG(INFO) << "Inlier mask size: " << inliers.rows << " x " << inliers.cols;

    // Build inlier keypoints and matches for visualization
    std::vector<cv::KeyPoint> inlier_prev_kps, inlier_cur_kps;
    std::vector<cv::DMatch> inlier_matches;

    for (int i = 0; i < inliers.rows; ++i) {
        if (inliers.at<uchar>(i)) {
            inlier_prev_kps.push_back(cv::KeyPoint(good_prev_points[i], 1.0f));
            inlier_cur_kps.push_back(cv::KeyPoint(good_cur_points[i], 1.0f));
            inlier_matches.push_back(cv::DMatch(inlier_prev_kps.size() - 1, inlier_cur_kps.size() - 1, 0));
        }
    }

    LOG(INFO) << "Valid inliers: " << inlier_prev_kps.size();

    if (inlier_prev_kps.empty() || inlier_cur_kps.empty()) {
        LOG(WARNING) << "No valid inliers to draw";
        return std::nullopt;
    }

    // Draw matches
    cv::Mat matched_img;
    cv::drawMatches(prev_frame.color_data.value().data, inlier_prev_kps,
                    cur_frame.color_data.value().data, inlier_cur_kps,
                    inlier_matches, matched_img,
                    cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);

    cv::imwrite("/data/robot/bags/house11/klt/matches_" + std::to_string(prev_frame.id) + "_" +
                    std::to_string(cur_frame.id) + ".png",
                matched_img);

    LOG(INFO) << "Homography: " << homography.t();

    return std::nullopt;  // Function should return a value
}

}  // namespace image
}  // namespace tracking

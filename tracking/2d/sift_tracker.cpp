#include <logging/logging.hpp>
#include <opencv2/imgproc.hpp>

#include "2d/sift_tracker.hpp"

namespace tracking {
namespace image {

SiftTracker::SiftTracker() {
    sift_detector_ = cv::SIFT::create();
    matcher_ = cv::BFMatcher::create(cv::NORM_L2);
}

std::optional<core::types::Pose> SiftTracker::match(const core::types::KeyFrame& prev_frame,
                                                    const core::types::KeyFrame& cur_frame) {
    LOG(INFO) << "Cleaner match";
    auto detectAndCompute = [&](const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                                cv::Mat& descriptor) {
        cv::Mat gray;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();
        }
        sift_detector_->detectAndCompute(gray, cv::noArray(), keypoints, descriptor);
    };

    std::vector<cv::KeyPoint> cur_img_kps;
    cv::Mat cur_img_desc;
    detectAndCompute(cur_frame.color_data.value().data, cur_img_kps, cur_img_desc);

    std::vector<cv::KeyPoint> prev_img_kps;
    cv::Mat prev_img_desc;
    detectAndCompute(prev_frame.color_data.value().data, prev_img_kps, prev_img_desc);

    LOG(INFO) << "Detected keypoints - prev: " << prev_img_kps.size()
              << ", cur: " << cur_img_kps.size();

    std::vector<cv::DMatch> matches;
    matcher_->match(prev_img_desc, cur_img_desc, matches);

    if (matches.size() < 4) {
        LOG(WARNING) << "Not enough matches";
        return std::nullopt;
    }

    // Extract point coordinates from keypoints for homography computation
    std::vector<cv::Point2f> prev_points, cur_points;
    for (const auto& match : matches) {
        prev_points.push_back(prev_img_kps[match.queryIdx].pt);
        cur_points.push_back(cur_img_kps[match.trainIdx].pt);
    }

    if (prev_points.size() < 4) {
        LOG(WARNING) << "Not enough valid matches for homography";
        return std::nullopt;
    }

    cv::Mat inlier_mask;
    auto homography = cv::findHomography(prev_points, cur_points, cv::RANSAC, 3.0, inlier_mask);

    if (homography.empty()) {
        LOG(WARNING) << "Failed to find homography";
        return std::nullopt;
    }

    LOG(INFO) << "Total matches: " << matches.size();
    LOG(INFO) << "Inlier mask size: " << inlier_mask.rows << " x " << inlier_mask.cols;
    LOG(INFO) << "Prev keypoints: " << prev_img_kps.size();
    LOG(INFO) << "Cur keypoints: " << cur_img_kps.size();

    std::vector<cv::KeyPoint> inliers1, inliers2;
    std::vector<cv::DMatch> inlier_matches;

    for (uint32_t i = 0; i < prev_img_kps.size(); ++i) {
        if (inlier_mask.at<uchar>(i)) {
            inliers1.push_back(prev_img_kps[i]);
            inliers2.push_back(cur_img_kps[i]);
            inlier_matches.push_back(cv::DMatch(inliers1.size() - 1, inliers2.size() - 1, 0));
        }
    }

    LOG(INFO) << "Valid inliers after bounds checking: " << inliers1.size();

    if (inliers1.empty() || inliers2.empty()) {
        LOG(WARNING) << "No valid inliers to draw";
        return std::nullopt;
    }

    cv::Mat matched_img;
    cv::drawMatches(prev_frame.color_data.value().data, inliers1, cur_frame.color_data.value().data,
                    inliers2, inlier_matches, matched_img, cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);

    cv::imwrite("/data/robot/bags/house11/sift/matches_" + std::to_string(prev_frame.id) + "_" +
                    std::to_string(cur_frame.id) + ".png",
                matched_img);
    LOG(INFO) << "Homography: " << homography.t();

    return std::nullopt;  // Function should return a value
}

}  // namespace image
}  // namespace tracking

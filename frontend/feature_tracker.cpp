#include "feature_tracker.hpp"
#include "core/graph/include/core/graph/factor_graph.hpp"
#include "core/types/include/core/types/keyframe.hpp"

namespace frontend {
namespace camera {

FeatureTracker::FeatureTracker(FeatureType type, std::shared_ptr<core::graph::FactorGraph>& graph)
    : feature_type_(type), graph_(graph) {
    switch (feature_type_) {
        case FeatureType::ORB:
            detector_ = cv::ORB::create(MAX_FEATURES);
            break;
        case FeatureType::GFTT:
            detector_ = cv::GFTTDetector::create(MAX_FEATURES, 0.01, 10, 3, true);
            break;
        case FeatureType::FAST:
            detector_ =
                cv::FastFeatureDetector::create(20, true, cv::FastFeatureDetector::TYPE_9_16);
            break;
    }
}

void FeatureTracker::processKeyframe() {
    auto keyframes = graph_->getKeyframes();

    auto keyframe_to_process = keyframes.begin() + num_keyframes_processed_;

    while (keyframe_to_process != keyframes.end()) {
        auto current_keyframe = keyframe_to_process->second;
        // Extract features for all images in keyframe
        extractFeatures(current_keyframe);

        // If this is the first keyframe, just store it
        if (num_keyframes_processed_ == 0) {
            initializeFirstKeyframe(current_keyframe);
            num_keyframes_processed_++;
            keyframe_to_process = keyframe_to_process + 1;
            continue;
        }

        // Perform intra-keyframe tracking if multiple images exist
        if (current_keyframe.color_data.size() > 1) {
            trackIntraKeyframe(current_keyframe);
        }

        auto prev = keyframe_to_process - 1;
        if (!prev) {
            continue;
        }

        auto prev_keyframe = prev->second;
        if (!prev)
            // Perform inter-keyframe tracking
            trackInterKeyframe(prev_keyframe, current_keyframe);

        // Update 3D landmarks
        updateLandmarks(current_keyframe);
    }
}

void FeatureTracker::extractFeatures(core::types::KeyFrame& keyframe) {
    keyframe.keypoints.resize(keyframe.color_image_count);
    keyframe.descriptors.resize(keyframe.color_image_count);
    keyframe.track_ids.resize(keyframe.color_image_count);

    auto keyframe_images = keyframe.getColorImages();

    for (size_t i = 0; i < keyframe.color_image_count; ++i) {
        std::vector<cv::KeyPoint> kpts;
        auto keyframe_image = keyframe.getColorImage(i);

        if (feature_type_ == FeatureType::ORB) {
            detector_->detect(keyframe_image, kpts);
            cv::Mat desc;
            detector_->compute(keyframe_image, kpts, desc);
            keyframe.setDescriptors(desc, i);
        } else {
            detector_->detect(keyframe_image, kpts);
        }

        // Convert keypoints to Point2f
        std::vector<cv::Point2f> kpts_points;
        kpts_points.reserve(kpts.size());
        for (const auto& kpt : kpts) {
            kpts_points.push_back(kpt.pt);
        }
        keyframe.setKeypoints(kpts_points, i);

        // Initialize track IDs as invalid
        keyframe.track_ids[i].resize(kpts.size(), static_cast<size_t>(-1));
    }
}

void FeatureTracker::trackIntraKeyframe(core::types::KeyFrame& keyframe) {
    if (keyframe.color_image_count <= 1) {
        LOG(INFO) << "Cant do tracking with just one image";
        return;
    }

    for (size_t i = 0; i < keyframe.color_image_count - 1; ++i) {
        std::vector<cv::Point2f> next_points;
        std::vector<uchar> status;
        std::vector<float> error;

        cv::calcOpticalFlowPyrLK(keyframe.getColorImage(i), keyframe.getColorImage(i + 1),
                                 keyframe.getKeypoints(i), next_points, status, error,
                                 cv::Size(21, 21), 3);

        // Update tracks for successfully tracked points
        for (size_t j = 0; j < status.size(); ++j) {
            if (status[j]) {
                if (keyframe.track_ids[i][j] == static_cast<size_t>(-1)) {
                    // Create new track
                    size_t new_track_id = next_track_++;
                    keyframe.track_ids[i][j] = new_track_id;

                    Track new_track;
                    new_track.track_id = new_track_id;
                    new_track.positions.push_back(keyframe.keypoints[i][j]);
                    new_track.frame_ids.push_back(i);
                    new_track.keyframe_ids.push_back(keyframe.id);
                    tracks_[new_track_id] = new_track;
                }

                // Update existing track
                size_t track_id = keyframe.track_ids[i][j];
                tracks_[track_id].positions.push_back(next_points[j]);
                tracks_[track_id].frame_ids.push_back(i + 1);
                tracks_[track_id].keyframe_ids.push_back(keyframe.id);

                // Find if this tracked point matches any keypoint in the next image
                bool matched = false;
                for (size_t k = 0; k < keyframe.getKeypoints(i + 1).size(); ++k) {
                    const auto& kpt = keyframe.getKeypoints(i + 1)[k];
                    float dist = cv::norm(kpt - next_points[j]);
                    if (dist < 2.0) {  // threshold in pixels
                        keyframe.track_ids[i + 1][k] = track_id;
                        matched = true;
                        break;
                    }
                }

                // If no match found, could optionally:
                // 1. Add as new keypoint (current behavior)
                // 2. Discard the track (stricter approach)
                if (!matched) {
                    LOG(WARN) << "Tracked point didn't match any detected keypoint in next frame";
                    // Option 1: Add as new point
                    keyframe.keypoints[i + 1].push_back(next_points[j]);
                    keyframe.track_ids[i + 1].push_back(track_id);
                    // Option 2: Discard track
                    // tracks_[track_id].positions.pop_back();
                    // tracks_[track_id].frame_ids.pop_back();
                    // tracks_[track_id].keyframe_ids.pop_back();
                }
            }
        }
    }
}

void FeatureTracker::trackInterKeyframe(core::types::KeyFrame& prev_keyframe,
                                        core::types::KeyFrame& cur_keyframe) {
    // For each image in previous keyframe
    for (size_t prev_img_idx = 0; prev_img_idx < prev_keyframe.color_image_count; ++prev_img_idx) {
        // Track to each image in current keyframe
        for (size_t cur_img_idx = 0; cur_img_idx < cur_keyframe.color_image_count; ++cur_img_idx) {
            std::vector<cv::Point2f> next_points;
            std::vector<uchar> status;
            std::vector<float> error;

            cv::calcOpticalFlowPyrLK(prev_keyframe.getColorImage(prev_img_idx),
                                     cur_keyframe.getColorImage(cur_img_idx),
                                     prev_keyframe.getKeypoints(prev_img_idx), next_points, status,
                                     error, cv::Size(21, 21), 3);

            // Update tracks for successfully tracked points
            for (size_t j = 0; j < status.size(); ++j) {
                if (status[j]) {
                    size_t track_id = prev_keyframe.track_ids[prev_img_idx][j];
                    // Only continue existing tracks
                    if (track_id != static_cast<size_t>(-1)) {
                        // Find if this tracked point matches any keypoint in the current image
                        bool matched = false;
                        for (size_t k = 0; k < cur_keyframe.getKeypoints(cur_img_idx).size(); ++k) {
                            const auto& kpt = cur_keyframe.getKeypoints(cur_img_idx)[k];
                            float dist = cv::norm(kpt - next_points[j]);
                            if (dist < 2.0) {  // threshold in pixels
                                cur_keyframe.track_ids[cur_img_idx][k] = track_id;

                                // Update track information
                                tracks_[track_id].positions.push_back(kpt);
                                tracks_[track_id].frame_ids.push_back(cur_img_idx);
                                tracks_[track_id].keyframe_ids.push_back(cur_keyframe.id);

                                matched = true;
                                break;
                            }
                        }

                        if (!matched) {
                            LOG(WARNING) << "Inter-keyframe tracked point didn't match any "
                                            "detected keypoint";
                            // Option 1: Add as new point
                            cur_keyframe.keypoints[cur_img_idx].push_back(next_points[j]);
                            cur_keyframe.track_ids[cur_img_idx].push_back(track_id);

                            // Update track information
                            tracks_[track_id].positions.push_back(next_points[j]);
                            tracks_[track_id].frame_ids.push_back(cur_img_idx);
                            tracks_[track_id].keyframe_ids.push_back(cur_keyframe.id);

                            // Option 2: Discard track (commented out)
                            // tracks_[track_id].positions.pop_back();
                            // tracks_[track_id].frame_ids.pop_back();
                            // tracks_[track_id].keyframe_ids.pop_back();
                        }
                    }
                }
            }
        }
    }
}

}  // namespace camera
}  // namespace frontend

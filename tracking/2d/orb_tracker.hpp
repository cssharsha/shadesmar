#include <iostream>
#include <map>
#include <memory>  // For std::unique_ptr
#include <optional>
#include <string>
#include <vector>

// OpenCV for cv::Mat
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <core/types/image.hpp>
#include <core/types/keyframe.hpp>
#include <core/types/keypoint.hpp>
#include <core/types/pose.hpp>
#include <stf/transform_tree.hpp>

namespace tracking {
namespace image {

class OrbTracker {
public:
    OrbTracker(uint32_t num_features = 2000, float scal_factor = 1.2f, uint32_t levels = 8);

    void addCameraInfo(const core::types::CameraInfo& cam_info);
    std::optional<core::types::Pose> operator()(
        const core::types::KeyFrame& cur_kf, const core::types::KeyFrame& prev_kf,
        std::map<uint32_t, core::types::Keypoint>& map_keypoints);
    void setTransformTree(std::shared_ptr<stf::TransformTree>& tf_tree) {
        tf_tree_ = tf_tree;
    }

private:
    std::map<std::string, core::types::CameraInfo> cam_infos_;

    uint32_t num_features_;
    float scale_factor_;
    uint32_t n_levels_;

    uint32_t min_matches_for_matching_ = 50;

    cv::Ptr<cv::ORB> orb_detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    std::shared_ptr<stf::TransformTree> tf_tree_;

    std::optional<core::types::Pose> track(
        const core::types::KeyFrame& cur_kf, const std::vector<cv::KeyPoint>& current_img_keypoints,
        const cv::Mat& current_img_descriptors,
        std::map<uint32_t, core::types::Keypoint>& map_keypoints,
        std::map<uint32_t, uint32_t>& current_img_to_map_keypoint_idx);

    std::optional<core::types::Pose> matchAndTriangulate(
        const std::vector<cv::KeyPoint>& cur_img_kps, const cv::Mat& cur_img_desc,
        const std::vector<cv::KeyPoint>& prev_img_kps, const cv::Mat& prev_img_desc,
        const core::types::KeyFrame& cur_frame, const core::types::KeyFrame& prev_frame,
        const cv::Mat& K, std::map<uint32_t, core::types::Keypoint>& map_keypoints);
};

}  // namespace image

}  // namespace tracking

#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <core/types/keyframe.hpp>
#include <core/types/pose.hpp>
namespace tracking {
namespace image {

class SiftTracker {
public:
    SiftTracker();
    std::optional<core::types::Pose> match(const core::types::KeyFrame& prev_frame,
                                           const core::types::KeyFrame& cur_frame);

private:
    cv::Ptr<cv::SIFT> sift_detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
};

}  // namespace image
}  // namespace tracking

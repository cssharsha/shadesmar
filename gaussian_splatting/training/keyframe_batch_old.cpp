#include "keyframe_batch.hpp"
#include <algorithm>
#include <chrono>
#include <logging/logging.hpp>
#include <opencv2/opencv.hpp>
#include <random>

namespace gaussian_splatting {
namespace training {

bool KeyframeBatch::isValid() const {
    // Add the logs printing all the checks being done here
    LOG(INFO) << "Batch size: " << batch_size << " Keyframe IDs: " << keyframe_ids.size()
              << " Keyframes: " << keyframes.size() << " Images: " << images.defined() << " "
              << images.sizes() << " Camera poses: " << camera_poses.defined() << " "
              << camera_poses.sizes() << " Camera intrinsics: " << camera_intrinsics.defined()
              << " " << camera_intrinsics.sizes() << " Depths: " << depths.defined();
    // return batch_size > 0 && keyframe_ids.size() == batch_size && keyframes.size() == batch_size
    // &&
    //        images.defined() && images.size(0) == static_cast<int64_t>(batch_size) &&
    //        camera_poses.defined() && camera_poses.size(0) == static_cast<int64_t>(batch_size) &&
    //        camera_intrinsics.defined() &&
    //        camera_intrinsics.size(0) == static_cast<int64_t>(batch_size);
    return batch_size > 0 && images.defined() &&
           images.size(0) == static_cast<int64_t>(batch_size) && camera_poses.defined() &&
           camera_poses.size(0) == static_cast<int64_t>(batch_size) &&
           camera_intrinsics.defined() &&
           camera_intrinsics.size(0) == static_cast<int64_t>(batch_size);
}

void KeyframeBatch::print() {
    std::stringstream ss;
    ss << "Keyframe bath " << batch_id << "( " << batch_size << ") info:\n"
       << "Images: " << images.sizes() << "\n"
       << "Camera poses: " << camera_poses.sizes() << "\n"
       << "Camera info: " << camera_intrinsics.sizes();
    LOG(INFO) << ss.str();
}

}  // namespace training
}  // namespace gaussian_splatting

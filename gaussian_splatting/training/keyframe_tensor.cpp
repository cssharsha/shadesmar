#include "gaussian_splatting/training/keyframe_tensor.hpp"

#include <core/types/keyframe.hpp>
#include <cstdint>
#include <logging/logging.hpp>
#include <opencv2/opencv.hpp>
#include "gaussian_splatting/common/tensor_config.hpp"

namespace gaussian_splatting {
namespace training {

KeyframeTensor::KeyframeTensor(std::shared_ptr<core::storage::MapStore>& map_store,
                               std::shared_ptr<stf::TransformTree>& tf_tree, std::string base_link,
                               std::string camera_frame)
    : device_(torch::kCPU),
      map_store_(map_store),
      tf_tree_(tf_tree),
      base_link_(base_link),
      camera_frame_(camera_frame) {
    try {
        auto transform_result = tf_tree_->getTransform(base_link_, camera_frame_);
        base_to_camera_ = transform_result.transform;
        LOG(INFO) << "Base to camera transform: " << base_to_camera_.matrix();
    } catch (const std::exception& e) {
        LOG(WARNING) << "Failed to get base_link to camera transform: " << e.what()
                     << ", using identity";
        throw std::runtime_error("Failed to get " + base_link_ + " to " + camera_frame_.c_str() +
                                 " transform");
    }
}

bool KeyframeTensor::isValid() const {
    LOG(INFO) << " Image: " << image_.defined() << " " << image_.sizes()
              << " Camera pose: " << camera_pose_.defined() << " " << camera_pose_.sizes()
              << " Camera intrinsic: " << camera_intrinsic_.defined() << " "
              << camera_intrinsic_.sizes() << " Depths: " << depth_.defined();
    return image_.defined() && camera_pose_.defined() && camera_intrinsic_.defined();
}

void KeyframeTensor::print() const {
    std::stringstream ss;
    ss << "info :\n "
       << "Images: " << image_.sizes() << "\n"
       << "Camera poses: " << camera_pose_.sizes() << "\n"
       << "Camera info: " << camera_intrinsic_.sizes();
    LOG(INFO) << ss.str();
}

void KeyframeTensor::loadFromStore(uint64_t keyframe_id) {
    keyframe_id_ = keyframe_id;
    // Load from map store directly
    LOG(INFO) << "Loading keyframe " << keyframe_id;
    auto keyframe = map_store_->getKeyFrame(keyframe_id);
    if (!keyframe) {
        LOG(ERROR) << "Failed to load keyframe " << keyframe_id;
        return;
    }
    LOG(INFO) << "Loaded keyframe " << keyframe_id;
    loadFromKeyframe(keyframe);
}

void KeyframeTensor::loadFromKeyframe(const core::storage::KeyFramePtr& keyframe) {
    LOG(INFO) << "Loading keyframe tensor from keyframe " << keyframe->id;

    // Clear previous tensors to free GPU memory before loading new data
    clear();

    keyframe_id_ = keyframe->id;
    auto transform_result = tf_tree_->getTransform(base_link_, camera_frame_);
    auto camera_pose = keyframe->pose.getEigenIsometry() * transform_result.transform;
    camera_pose_ = convertCameraPoseToTensor(camera_pose, device_);
    camera_intrinsic_ = convertCameraIntrinsicsToTensor(keyframe->getCameraInfo(), device_);
    LOG(INFO) << "Converted camera pose and intrinsics";

    // Convert cv::Mat image to a Torch tensor [C,H,W] float32 in [0,1]
    const auto& color_image = keyframe->getColorImage();
    cv::Mat mat = color_image.data;  // cv::Mat copy header (no data copy)
    if (mat.empty()) {
        throw std::runtime_error("Color image is empty for keyframe " +
                                 std::to_string(keyframe_id_));
    }

    // Ensure 3-channel RGB
    if (mat.channels() == 3) {
        if (color_image.encoding == "bgr8") {
            cv::Mat rgb;
            cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
            mat = rgb;  // rgb will keep data alive until we clone the tensor
        }
        // if encoding is already rgb8, keep as-is
    } else if (mat.channels() == 1) {
        cv::Mat rgb;
        cv::cvtColor(mat, rgb, cv::COLOR_GRAY2RGB);
        mat = rgb;
    } else {
        // Fallback: convert to 3-channel RGB
        cv::Mat rgb;
        cv::cvtColor(mat, rgb, cv::COLOR_BGRA2RGB);
        mat = rgb;
    }

    // Create tensor from blob [H,W,C] uint8, then permute to [C,H,W] and normalize
    auto img_tensor_u8 =
        torch::from_blob(mat.data, {mat.rows, mat.cols, mat.channels()}, torch::kUInt8);
    auto img_chw = img_tensor_u8.permute({2, 0, 1});
    image_ = img_chw.to(torch::kFloat32).div_(255.0).clone();  // clone to own memory
}

torch::Tensor KeyframeTensor::convertCameraPoseToTensor(const Eigen::Isometry3d& pose,
                                                        const torch::Device& device) {
    LOG(INFO) << "Converting camera pose to tensor";
    LOG(INFO) << "Base to " << camera_frame_ << ": ";
    // LOG(INFO) << base_to_camera_.matrix();
    // auto camera_in_world = pose * base_to_camera_.inverse();
    // Convert 4x4 pose matrix to torch tensor
    torch::Tensor pose_tensor = torch::zeros({1, 4, 4}, common::getTensorOptions());
    LOG(INFO) << "Camera pose: " << pose.matrix();
    auto camera_in_world = pose.inverse();
    LOG(INFO) << "Camera in world: " << camera_in_world.matrix();
    LOG(INFO) << "Translation: " << camera_in_world.translation().transpose();
    // auto camera_in_world = pose.inverse();
    LOG(INFO) << "Camera in world: " << camera_in_world.matrix();

    Eigen::Matrix4d pose_matrix = camera_in_world.matrix();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            pose_tensor[0][i][j] = pose_matrix(i, j);
        }
    }

    return pose_tensor;
}

torch::Tensor KeyframeTensor::convertCameraIntrinsicsToTensor(
    const core::types::CameraInfo& camera_info, const torch::Device& device) {
    torch::Tensor intrinsics = torch::zeros({1, 3, 3}, common::getTensorOptions());

    assert(camera_info.k.size() == 9);

    intrinsics[0][0][0] = static_cast<common::scalar_t>(camera_info.k[0]);  // fx
    intrinsics[0][1][1] = static_cast<common::scalar_t>(camera_info.k[4]);  // fy
    intrinsics[0][0][2] = static_cast<common::scalar_t>(camera_info.k[2]);  // cx
    intrinsics[0][1][2] = static_cast<common::scalar_t>(camera_info.k[5]);  // cy
    intrinsics[0][2][2] = static_cast<common::scalar_t>(1.0);                // homogeneous coordinate

    image_width_ = camera_info.width;
    image_height_ = camera_info.height;

    return intrinsics;
}

}  // namespace training
}  // namespace gaussian_splatting

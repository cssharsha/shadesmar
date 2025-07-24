#pragma once

#include <torch/torch.h>

#include <cstdint>
#include <memory>
#include <string>

#include <core/storage/map_store.hpp>
#include <stf/transform_tree.hpp>

namespace gaussian_splatting {
namespace training {

/**
 * Represents a batch of keyframes for training with associated camera data
 */
class KeyframeTensor {
public:
    explicit KeyframeTensor(std::shared_ptr<core::storage::MapStore>& map_store,
                            std::shared_ptr<stf::TransformTree>& tf_tree);

    bool isValid() const;

    void print() const;

    void clear() {
        image_ = torch::Tensor();
        camera_pose_ = torch::Tensor();
        camera_intrinsic_ = torch::Tensor();
        depth_ = torch::Tensor();
    }

    // Move batch to specified device
    void to(torch::Device target_device) {
        std::cout << "Moving to device: " << target_device.type() << std::endl;
        if (target_device.is_cuda()) {
            if (!image_.is_cuda()) {
                image_ = image_.to(target_device);
            }
            if (!camera_pose_.is_cuda()) {
                camera_pose_ = camera_pose_.to(target_device);
            }
            if (!camera_intrinsic_.is_cuda()) {
                camera_intrinsic_ = camera_intrinsic_.to(target_device);
            }
            if (depth_.defined() && !depth_.is_cuda()) {
                depth_ = depth_.to(target_device);
            }
        } else {
            if (target_device.is_cpu()) {
                if (!image_.is_cpu()) {
                    image_ = image_.to(target_device);
                }
                if (!camera_pose_.is_cpu()) {
                    camera_pose_ = camera_pose_.to(target_device);
                }
                if (!camera_intrinsic_.is_cpu()) {
                    camera_intrinsic_ = camera_intrinsic_.to(target_device);
                }
                if (depth_.defined() && !depth_.is_cpu()) {
                    depth_ = depth_.to(target_device);
                }
            }
        }
        device_ = target_device;
    }

    void loadFromStore(uint64_t keyframe_id);
    void loadFromKeyframe(const core::storage::KeyFramePtr& keyframe);

    // All the getters
    torch::Tensor& getImage() {
        return image_;
    }
    torch::Tensor& getCameraPose() {
        return camera_pose_;
    }
    torch::Tensor& getCameraIntrinsic() {
        return camera_intrinsic_;
    }
    torch::Tensor& getDepth() {
        return depth_;
    }
    float getImageHeight() {
        return image_height_;
    }
    float getImageWidth() {
        return image_width_;
    }
    uint64_t getKeyframeId() {
        return keyframe_id_;
    }
    torch::Device getDevice() {
        return device_;
    }

private:
    uint64_t keyframe_id_;

    torch::Tensor image_;             // [3, height, width]
    torch::Tensor camera_pose_;       // [4, 4] - world to camera transforms
    torch::Tensor camera_intrinsic_;  // [3, 3] - camera intrinsic matrices
    torch::Tensor depth_;             // [height, width] - optional depth maps

    torch::Device device_;

    float image_height_ = 0.0F;
    float image_width_ = 0.0F;

    std::shared_ptr<core::storage::MapStore> map_store_;
    std::shared_ptr<stf::TransformTree> tf_tree_;
    Eigen::Isometry3d base_to_camera_;
    std::string camera_frame_ = "camera";
    std::string base_link_ = "base_link";

    torch::Tensor convertCameraPoseToTensor(const Eigen::Isometry3d& pose,
                                            const torch::Device& device);

    torch::Tensor convertCameraIntrinsicsToTensor(const core::types::CameraInfo& camera_info,
                                                  const torch::Device& device);
};

}  // namespace training
}  // namespace gaussian_splatting

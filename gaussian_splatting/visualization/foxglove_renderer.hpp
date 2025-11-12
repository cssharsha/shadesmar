#pragma once

// Include foxglove-dependent headers first to avoid macro conflicts
#include "viz/foxglove_server.hpp"

#include "core/types/gaussian_splat.hpp"
#include "core/types/image.hpp"
#include "core/types/pose.hpp"
#include "gaussian_splatting/rendering/rasterizer.hpp"
#include "gaussian_splatting/training/gaussian_tensors.hpp"

#include <torch/torch.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace gaussian_splatting {
namespace visualization {

class FoxgloveRenderer {
public:
    struct Config {
        Config() = default;

        std::string host = "0.0.0.0";
        uint16_t port = 8765;
        std::string pointcloud_topic = "/gaussian_splats/pointcloud";
        std::string image_topic = "/rendering/output";
        std::string camera_pose_topic = "/viewer/camera_pose";
        std::string camera_info_topic = "/viewer/camera_info";
        std::string keyframe_poses_topic = "/gaussian_splats/keyframe_poses";
        std::string frame_id = "world";

        int image_width = 1280;
        int image_height = 720;
        torch::Device device = torch::kCUDA;

        // Coordinate frame transformation flag
        // If true, transforms points/poses from Z-forward (camera) to Z-up (world) for publishing
        // and inverse transform for received poses
        bool apply_coordinate_transform = false;
    };

    explicit FoxgloveRenderer(const Config& config);
    ~FoxgloveRenderer();

    FoxgloveRenderer(const FoxgloveRenderer&) = delete;
    FoxgloveRenderer& operator=(const FoxgloveRenderer&) = delete;
    FoxgloveRenderer(FoxgloveRenderer&&) = delete;
    FoxgloveRenderer& operator=(FoxgloveRenderer&&) = delete;

    bool initialize();
    void shutdown();
    void updateSplats(const std::vector<core::types::GaussianSplat>& splats, uint64_t timestamp_ns);
    void updateRenderingTensors(const GaussianTensors& tensors);
    void publishKeyframePoses(const std::vector<core::types::Pose>& poses, uint64_t timestamp_ns);
    bool isRunning() const;
    bool testRenderFromFixedPose();

private:
    core::types::PointCloud convertSplatsToPointCloud(
        const std::vector<core::types::GaussianSplat>& splats);

    void onCameraPoseReceived(const core::types::Pose& pose);
    void onCameraInfoReceived(const core::types::CameraInfo& info);

    core::types::Image renderFromPose(const core::types::Pose& camera_pose);
    torch::Tensor poseToTorchTensor(const core::types::Pose& pose);

    Config config_;
    std::unique_ptr<viz::FoxgloveServer> server_;
    mutable std::mutex mutex_;

    GaussianTensors current_tensors_;
    bool has_splats_;

    core::types::Pose last_received_pose_;
    bool has_received_pose_;

    core::types::CameraInfo last_received_camera_info_;
    bool has_received_camera_info_;

    rendering::DifferentiableRasterizer rasterizer_;
    torch::Tensor camera_intrinsics_tensor_;

    // Coordinate frame transformation matrices
    // R_camera_to_world: Z-forward (camera frame) -> Z-up (world frame)
    Eigen::Matrix3d R_camera_to_world_;
    // R_world_to_camera: Z-up (world frame) -> Z-forward (camera frame)
    Eigen::Matrix3d R_world_to_camera_;
};

}  // namespace visualization
}  // namespace gaussian_splatting

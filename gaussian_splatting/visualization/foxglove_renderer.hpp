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

/**
 * @brief Foxglove-based interactive renderer for Gaussian splats
 *
 * This class bridges between Gaussian splatting and Foxglove visualization.
 * It converts GaussianSplats to point clouds, subscribes to camera poses from
 * Foxglove viewer, renders from those viewpoints, and publishes the rendered
 * images back to Foxglove.
 */
class FoxgloveRenderer {
public:
    struct Config {
        std::string host = "0.0.0.0";
        uint16_t port = 8765;
        std::string pointcloud_topic = "/gaussian_splats/pointcloud";
        std::string image_topic = "/rendering/output";
        std::string camera_pose_topic = "/viewer/camera_pose";
        std::string frame_id = "world";

        // Camera parameters for rendering
        int image_width = 1280;
        int image_height = 720;
        Eigen::Matrix3d camera_intrinsics;
        torch::Device device = torch::kCUDA;

        // Constructor with default intrinsics for 1280x720
        Config() {
            camera_intrinsics << 960.0, 0.0, 640.0, 0.0, 960.0, 360.0, 0.0, 0.0, 1.0;
        }
    };

    explicit FoxgloveRenderer(const Config& config = Config());
    ~FoxgloveRenderer();

    // Disable copy and move
    FoxgloveRenderer(const FoxgloveRenderer&) = delete;
    FoxgloveRenderer& operator=(const FoxgloveRenderer&) = delete;
    FoxgloveRenderer(FoxgloveRenderer&&) = delete;
    FoxgloveRenderer& operator=(FoxgloveRenderer&&) = delete;

    /**
     * @brief Initialize the Foxglove server and start visualization
     * @return true if initialization succeeded, false otherwise
     */
    bool initialize();

    /**
     * @brief Shutdown the Foxglove server
     */
    void shutdown();

    /**
     * @brief Update the Gaussian splats to visualize
     *
     * This will convert the splats to point cloud format and publish to Foxglove,
     * and also store them internally for rendering.
     *
     * @param splats The vector of Gaussian splats to visualize
     * @param timestamp_ns Nanosecond timestamp
     */
    void updateSplats(const std::vector<core::types::GaussianSplat>& splats, uint64_t timestamp_ns);

    /**
     * @brief Update rendering tensors directly (thread-safe for concurrent training)
     *
     * This accepts a cloned copy of training tensors for safe concurrent rendering.
     * No conversion from splats is needed. The tensors are atomically swapped.
     *
     * @param tensors Cloned GaussianTensors for rendering
     */
    void updateRenderingTensors(const GaussianTensors& tensors);

    /**
     * @brief Check if the renderer is currently running
     * @return true if running, false otherwise
     */
    bool isRunning() const;

    /**
     * @brief Render from the last received camera pose
     *
     * Renders from the last camera pose received via onCameraPoseReceived()
     * and publishes the result to Foxglove. If no pose has been received yet,
     * falls back to a default identity pose (camera at origin, looking forward).
     * Only works if splats have been loaded via updateSplats() or updateRenderingTensors().
     *
     * @return true if rendering succeeded, false otherwise
     */
    bool testRenderFromFixedPose();

private:
    /**
     * @brief Convert GaussianSplats to PointCloudData for visualization
     */
    viz::FoxgloveServer::PointCloudData convertSplatsToPointCloud(
        const std::vector<core::types::GaussianSplat>& splats);

    /**
     * @brief Callback for receiving camera pose from Foxglove viewer
     *
     * Stores the received camera pose for later use by testRenderFromFixedPose().
     * Does not trigger rendering or publishing - that happens when
     * testRenderFromFixedPose() is explicitly called.
     *
     * @param pose The camera pose from the Foxglove viewer
     */
    void onCameraPoseReceived(core::types::Pose& pose);

    /**
     * @brief Render the scene from a given camera pose (gradient-free)
     */
    core::types::Image renderFromPose(const core::types::Pose& camera_pose);

    /**
     * @brief Convert Eigen camera pose to torch tensor
     */
    torch::Tensor poseToTorchTensor(const core::types::Pose& pose);

    Config config_;
    std::unique_ptr<viz::FoxgloveServer> server_;
    mutable std::mutex mutex_;

    // Current splats stored as tensors for rendering
    GaussianTensors current_tensors_;
    bool has_splats_;

    // Last received camera pose from Foxglove viewer
    core::types::Pose last_received_pose_;
    bool has_received_pose_;

    // Rasterizer for gradient-free rendering
    rendering::DifferentiableRasterizer rasterizer_;

    // Camera intrinsics tensor (cached)
    torch::Tensor camera_intrinsics_tensor_;
};

}  // namespace visualization
}  // namespace gaussian_splatting

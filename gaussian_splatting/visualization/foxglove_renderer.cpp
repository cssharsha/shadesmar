#include "gaussian_splatting/visualization/foxglove_renderer.hpp"

#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "common/logging/logging.hpp"

namespace gaussian_splatting {
namespace visualization {

FoxgloveRenderer::FoxgloveRenderer(const Config& config)
    : config_(config), has_splats_(false), has_received_pose_(false) {
    server_ = std::make_unique<viz::FoxgloveServer>(config_.host, config_.port);

    // Prepare camera intrinsics tensor with batch dimension [1, 3, 3]
    camera_intrinsics_tensor_ = torch::zeros({1, 3, 3}, torch::kFloat32);
    camera_intrinsics_tensor_[0][0][0] = config_.camera_intrinsics(0, 0);  // fx
    camera_intrinsics_tensor_[0][0][1] = config_.camera_intrinsics(0, 1);
    camera_intrinsics_tensor_[0][0][2] = config_.camera_intrinsics(0, 2);  // cx
    camera_intrinsics_tensor_[0][1][0] = config_.camera_intrinsics(1, 0);
    camera_intrinsics_tensor_[0][1][1] = config_.camera_intrinsics(1, 1);  // fy
    camera_intrinsics_tensor_[0][1][2] = config_.camera_intrinsics(1, 2);  // cy
    camera_intrinsics_tensor_[0][2][0] = config_.camera_intrinsics(2, 0);
    camera_intrinsics_tensor_[0][2][1] = config_.camera_intrinsics(2, 1);
    camera_intrinsics_tensor_[0][2][2] = config_.camera_intrinsics(2, 2);

    camera_intrinsics_tensor_ = camera_intrinsics_tensor_.to(config_.device);
}

FoxgloveRenderer::~FoxgloveRenderer() {
    shutdown();
}

bool FoxgloveRenderer::initialize() {
    if (!server_->initialize()) {
        LOG(ERROR) << "[FoxgloveRenderer] Failed to initialize Foxglove server";
        return false;
    }

    // Subscribe to camera pose updates
    server_->subscribeToCameraPose(config_.camera_pose_topic, [this](core::types::Pose& pose) {
        this->onCameraPoseReceived(pose);
    });

    LOG(INFO) << "[FoxgloveRenderer] Initialized successfully";
    return true;
}

void FoxgloveRenderer::shutdown() {
    if (server_) {
        server_->shutdown();
    }
    LOG(INFO) << "[FoxgloveRenderer] Shutdown complete";
}

bool FoxgloveRenderer::isRunning() const {
    return server_ && server_->isRunning();
}

bool FoxgloveRenderer::testRenderFromFixedPose() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_splats_) {
        LOG(WARNING) << "[FoxgloveRenderer] Cannot render: no splats loaded";
        return false;
    }

    // Determine which pose to use for rendering
    core::types::Pose render_pose;
    if (has_received_pose_) {
        // Use the last received camera pose from Foxglove viewer
        render_pose = last_received_pose_;
        LOG(INFO) << "[FoxgloveRenderer] Rendering from last received camera pose: "
                  << render_pose.position.transpose();
    } else {
        // Fall back to a default identity pose if no pose has been received yet
        render_pose.position = Eigen::Vector3d(0.0, 0.0, 0.0);
        render_pose.orientation = Eigen::Quaterniond::Identity();
        render_pose.frame_id = config_.frame_id;
        render_pose.timestamp = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
        LOG(WARNING)
            << "[FoxgloveRenderer] No camera pose received yet, using default identity pose";
    }

    // Render from the selected pose
    auto rendered_image = renderFromPose(render_pose);

    if (rendered_image.data.empty()) {
        LOG(ERROR) << "[FoxgloveRenderer] Render failed: empty image";
        return false;
    }

    // Publish the rendered image
    auto now = std::chrono::system_clock::now();
    uint64_t timestamp_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

    server_->publishImage(config_.image_topic, rendered_image, timestamp_ns);

    LOG(INFO) << "[FoxgloveRenderer] Render complete and published to " << config_.image_topic;
    return true;
}

void FoxgloveRenderer::updateSplats(const std::vector<core::types::GaussianSplat>& splats,
                                    uint64_t timestamp_ns) {
    LOG(INFO) << "[FoxgloveRenderer] updateSplats called with " << splats.size() << " splats";

    if (splats.empty()) {
        LOG(WARNING) << "[FoxgloveRenderer] Received empty splat vector";
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    current_tensors_.clear();

    // Convert splats to point cloud and publish
    auto pointcloud_data = convertSplatsToPointCloud(splats);
    server_->publishPointCloud(config_.pointcloud_topic, pointcloud_data, timestamp_ns);

    // Update internal tensors for rendering
    bool success = current_tensors_.fromSplats(splats);
    if (!success) {
        LOG(ERROR) << "[FoxgloveRenderer] Failed to convert splats to tensors";
        return;
    }

    current_tensors_.to(config_.device);
    has_splats_ = true;
    // testRenderFromFixedPose();

    LOG(INFO) << "[FoxgloveRenderer] Updated " << splats.size() << " splats, "
              << "tensor has " << current_tensors_.number_of_splats() << " splats";
}

void FoxgloveRenderer::updateRenderingTensors(const GaussianTensors& tensors) {
    LOG(INFO) << "[FoxgloveRenderer] updateRenderingTensors called with "
              << tensors.number_of_splats() << " splats";

    std::lock_guard<std::mutex> lock(mutex_);

    // Atomic swap: replace current rendering tensors with cloned copy
    current_tensors_ = tensors;
    has_splats_ = true;

    LOG(INFO) << "[FoxgloveRenderer] Rendering tensors updated, "
              << "num_splats=" << current_tensors_.number_of_splats();
}

viz::FoxgloveServer::PointCloudData FoxgloveRenderer::convertSplatsToPointCloud(
    const std::vector<core::types::GaussianSplat>& splats) {
    viz::FoxgloveServer::PointCloudData data;
    data.frame_id = config_.frame_id;

    data.positions.reserve(splats.size() * 3);
    data.colors.reserve(splats.size() * 4);
    data.radii.reserve(splats.size());

    // Transform from camera frame (Z-forward) to world frame (Z-up)
    // Rotation: 90° pitch down around X-axis
    // Camera: X-right, Y-down, Z-forward -> World: X-forward, Y-left, Z-up
    Eigen::Matrix3f R_camera_to_world;
    R_camera_to_world << 0, 0, 1,  // X_world = Z_camera
        -1, 0, 0,                  // Y_world = -X_camera
        0, -1, 0;                  // Z_world = -Y_camera

    for (const auto& splat : splats) {
        // Transform position to world frame
        Eigen::Vector3f pos_camera = splat.position.cast<float>();
        Eigen::Vector3f pos_world = R_camera_to_world * pos_camera;

        data.positions.push_back(pos_world.x());
        data.positions.push_back(pos_world.y());
        data.positions.push_back(pos_world.z());

        // Color from SH DC component
        Eigen::Vector3f rgb = splat.getColor();
        data.colors.push_back(static_cast<uint8_t>(rgb.x() * 255.0f));
        data.colors.push_back(static_cast<uint8_t>(rgb.y() * 255.0f));
        data.colors.push_back(static_cast<uint8_t>(rgb.z() * 255.0f));
        data.colors.push_back(static_cast<uint8_t>(splat.opacity * 255.0f));

        // Radius (use mean of scales)
        float radius = static_cast<float>(splat.scale.mean());
        data.radii.push_back(radius);
    }

    return data;
}

void FoxgloveRenderer::onCameraPoseReceived(core::types::Pose& pose) {
    std::lock_guard<std::mutex> lock(mutex_);

    LOG(INFO) << "[FoxgloveRenderer] Received camera pose: " << pose.position.transpose() << " at "
              << pose.timestamp;

    // Store the received pose for later rendering
    last_received_pose_ = pose;
    last_received_pose_.frame_id = config_.frame_id;
    has_received_pose_ = true;

    LOG(INFO) << "[FoxgloveRenderer] Camera pose updated and stored";
}

core::types::Image FoxgloveRenderer::renderFromPose(const core::types::Pose& camera_pose) {
    // Disable gradient computation for rendering
    torch::NoGradGuard no_grad;

    LOG(INFO) << "[FoxgloveRenderer] renderFromPose: has_splats_=" << has_splats_
              << " num_splats=" << current_tensors_.number_of_splats();

    // Convert pose to torch tensor
    torch::Tensor camera_pose_tensor = poseToTorchTensor(camera_pose);
    LOG(INFO) << "Camera pose tensor: " << camera_pose_tensor.device();
    LOG(INFO) << "Camera intrinsics tensor: " << camera_intrinsics_tensor_.device();

    // Perform rasterization
    rendering::RasterizationOutput output =
        rasterizer_.rasterize(current_tensors_, camera_pose_tensor, camera_intrinsics_tensor_,
                              config_.image_width, config_.image_height);

    if (!output.success) {
        LOG(ERROR) << "[FoxgloveRenderer] Rasterization failed";
        return core::types::Image();
    }

    // Convert rendered tensor to OpenCV Mat
    // rendered_image is [H, W, 3] in range [0, 1]
    torch::Tensor image_cpu = output.rendered_image.to(torch::kCPU);

    // Convert to uint8 [0, 255]
    image_cpu = (image_cpu * 255.0f).clamp(0, 255).to(torch::kUInt8);

    // Get data pointer and create cv::Mat
    cv::Mat cv_image(config_.image_height, config_.image_width, CV_8UC3,
                     image_cpu.data_ptr<uint8_t>());

    // Clone to ensure data ownership
    cv::Mat cv_image_clone = cv_image.clone();

    // Convert RGB to BGR for OpenCV
    cv::cvtColor(cv_image_clone, cv_image_clone, cv::COLOR_RGB2BGR);

    // Create core::types::Image
    core::types::Image result =
        core::types::Image::fromCvMat(cv_image_clone, "bgr8", config_.frame_id);

    return result;
}

torch::Tensor FoxgloveRenderer::poseToTorchTensor(const core::types::Pose& pose) {
    // Convert Eigen pose to 4x4 transformation matrix with batch dimension [1, 4, 4]
    Eigen::Isometry3d isometry = pose.getEigenIsometry();

    // Invert the pose to get camera-to-world (same as KeyframeTensor does)
    Eigen::Isometry3d camera_in_world = isometry.inverse();
    Eigen::Matrix4d pose_matrix = camera_in_world.matrix();

    // Create tensor with batch dimension [1, 4, 4]
    torch::Tensor pose_tensor = torch::zeros({1, 4, 4}, torch::kFloat32);

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            pose_tensor[0][i][j] = static_cast<float>(pose_matrix(i, j));
        }
    }
    LOG(INFO) << "Camera pose tensor: " << pose_tensor.device();

    return pose_tensor.to(config_.device);
}

}  // namespace visualization
}  // namespace gaussian_splatting

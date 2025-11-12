#include "gaussian_splatting/visualization/foxglove_renderer.hpp"

#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "common/logging/logging.hpp"

namespace gaussian_splatting {
namespace visualization {

FoxgloveRenderer::FoxgloveRenderer(const Config& config)
    : config_(config),
      has_splats_(false),
      has_received_pose_(false),
      has_received_camera_info_(false) {
    server_ = std::make_unique<viz::FoxgloveServer>(config_.host, config_.port);

    // Prepare camera intrinsics tensor. It will be populated when the first
    // CameraInfo message is received.
    camera_intrinsics_tensor_ = torch::zeros({1, 3, 3}, torch::kFloat32).to(config_.device);

    // Initialize coordinate frame transformation matrices
    // R_camera_to_world: Z-forward (camera) -> Z-up (world)
    R_camera_to_world_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;

    // R_world_to_camera: Z-up (world) -> Z-forward (camera)
    // This is the transpose/inverse since the matrix is orthogonal
    R_world_to_camera_ = R_camera_to_world_.transpose();
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
    server_->subscribeToCameraPose(
        config_.camera_pose_topic,
        [this](const core::types::Pose& pose) { this->onCameraPoseReceived(pose); });

    // Subscribe to camera info updates
    server_->subscribeToCameraInfo(
        config_.camera_info_topic,
        [this](const core::types::CameraInfo& info) { this->onCameraInfoReceived(info); });

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

    if (!has_splats_ || !has_received_camera_info_) {
        LOG(WARNING)
            << "[FoxgloveRenderer] Cannot render: no splats loaded or no camera info received.";
        return false;
    }

    core::types::Pose render_pose;
    if (has_received_pose_) {
        render_pose = last_received_pose_;
        LOG(INFO) << "[FoxgloveRenderer] Rendering from last received camera pose: "
                  << render_pose.position.transpose();
    } else {
        render_pose.position = Eigen::Vector3d(0.0, 0.0, 0.0);
        render_pose.orientation = Eigen::Quaterniond::Identity();
        render_pose.frame_id = config_.frame_id;
        render_pose.timestamp = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
        LOG(WARNING)
            << "[FoxgloveRenderer] No camera pose received yet, using default identity pose";
    }

    auto rendered_image = renderFromPose(render_pose);

    if (rendered_image.data.empty()) {
        LOG(ERROR) << "[FoxgloveRenderer] Render failed: empty image";
        return false;
    }

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

    auto pointcloud_data = convertSplatsToPointCloud(splats);
    server_->publishPointCloud(config_.pointcloud_topic, pointcloud_data, timestamp_ns);

    bool success = current_tensors_.fromSplats(splats);
    if (!success) {
        LOG(ERROR) << "[FoxgloveRenderer] Failed to convert splats to tensors";
        return;
    }

    current_tensors_.to(config_.device);
    has_splats_ = true;

    LOG(INFO) << "[FoxgloveRenderer] Updated " << splats.size() << " splats, "
              << "tensor has " << current_tensors_.number_of_splats() << " splats";
}

void FoxgloveRenderer::updateRenderingTensors(const GaussianTensors& tensors) {
    LOG(INFO) << "[FoxgloveRenderer] updateRenderingTensors called with "
              << tensors.number_of_splats() << " splats";

    std::lock_guard<std::mutex> lock(mutex_);
    current_tensors_ = tensors;
    has_splats_ = true;

    LOG(INFO) << "[FoxgloveRenderer] Rendering tensors updated, "
              << "num_splats=" << current_tensors_.number_of_splats();
}

void FoxgloveRenderer::publishKeyframePoses(const std::vector<core::types::Pose>& poses,
                                            uint64_t timestamp_ns) {
    if (poses.empty()) {
        LOG(WARNING) << "[FoxgloveRenderer] Cannot publish empty keyframe poses";
        return;
    }

    // Apply coordinate transform if enabled (camera frame -> world frame)
    std::vector<core::types::Pose> transformed_poses = poses;
    if (config_.apply_coordinate_transform) {
        for (auto& pose : transformed_poses) {
            // Transform position
            pose.position = R_camera_to_world_ * pose.position;

            // Transform orientation (quaternion)
            Eigen::Quaterniond q(pose.orientation.w(), pose.orientation.x(), pose.orientation.y(),
                                 pose.orientation.z());
            Eigen::Quaterniond q_transformed(R_camera_to_world_ * q.toRotationMatrix());
            pose.orientation = q_transformed;
        }
    }

    server_->publishKeyframePoses(config_.keyframe_poses_topic, transformed_poses, timestamp_ns);
    LOG(INFO) << "[FoxgloveRenderer] Published " << transformed_poses.size() << " keyframe poses"
              << (config_.apply_coordinate_transform ? " (with coordinate transform)" : "");
}

core::types::PointCloud FoxgloveRenderer::convertSplatsToPointCloud(
    const std::vector<core::types::GaussianSplat>& splats) {
    core::types::PointCloud cloud;
    cloud.frame_id = config_.frame_id;

    cloud.points.reserve(splats.size());
    cloud.colors.reserve(splats.size());

    for (const auto& splat : splats) {
        Eigen::Vector3d pos = splat.position;

        // Apply coordinate transform if enabled (camera frame -> world frame)
        if (config_.apply_coordinate_transform) {
            pos = R_camera_to_world_ * pos;
        }

        // Add position as Eigen::Vector3d
        cloud.points.emplace_back(pos.x(), pos.y(), pos.z());

        // Add color as Eigen::Vector3d (normalized 0-1)
        Eigen::Vector3f rgb = splat.getColor();
        cloud.colors.emplace_back(rgb.x(), rgb.y(), rgb.z());
    }

    return cloud;
}

void FoxgloveRenderer::onCameraPoseReceived(const core::types::Pose& pose) {
    std::lock_guard<std::mutex> lock(mutex_);

    last_received_pose_ = pose;
    last_received_pose_.frame_id = config_.frame_id;

    // Apply inverse coordinate transform if enabled (world frame -> camera frame)
    // The received pose is in world frame (Z-up), but we need it in camera frame (Z-forward) for
    // rendering
    if (config_.apply_coordinate_transform) {
        // Transform position
        last_received_pose_.position = R_world_to_camera_ * pose.position;

        // Transform orientation (quaternion)
        Eigen::Quaterniond q(pose.orientation.w(), pose.orientation.x(), pose.orientation.y(),
                             pose.orientation.z());
        Eigen::Quaterniond q_transformed(R_world_to_camera_ * q.toRotationMatrix());
        last_received_pose_.orientation = q_transformed;

        LOG(INFO) << "[FoxgloveRenderer] Received camera pose (world frame): position=["
                  << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z()
                  << "], transformed to camera frame: position=["
                  << last_received_pose_.position.x() << ", " << last_received_pose_.position.y()
                  << ", " << last_received_pose_.position.z() << "]";
    } else {
        LOG(INFO) << "[FoxgloveRenderer] Received camera pose: position=[" << pose.position.x()
                  << ", " << pose.position.y() << ", " << pose.position.z()
                  << "], orientation=[w=" << pose.orientation.w() << ", x=" << pose.orientation.x()
                  << ", y=" << pose.orientation.y() << ", z=" << pose.orientation.z()
                  << "], frame_id=" << config_.frame_id;
    }

    has_received_pose_ = true;
}

void FoxgloveRenderer::onCameraInfoReceived(const core::types::CameraInfo& info) {
    std::lock_guard<std::mutex> lock(mutex_);

    last_received_camera_info_ = info;
    has_received_camera_info_ = true;

    // Update the intrinsics tensor from the k matrix (9-element row-major array)
    camera_intrinsics_tensor_[0][0][0] = info.k[0];  // fx
    camera_intrinsics_tensor_[0][0][1] = info.k[1];  // 0
    camera_intrinsics_tensor_[0][0][2] = info.k[2];  // cx
    camera_intrinsics_tensor_[0][1][0] = info.k[3];  // 0
    camera_intrinsics_tensor_[0][1][1] = info.k[4];  // fy
    camera_intrinsics_tensor_[0][1][2] = info.k[5];  // cy
    camera_intrinsics_tensor_[0][2][0] = info.k[6];  // 0
    camera_intrinsics_tensor_[0][2][1] = info.k[7];  // 0
    camera_intrinsics_tensor_[0][2][2] = info.k[8];  // 1

    LOG(INFO) << "[FoxgloveRenderer] Received camera info: width=" << info.width
              << ", height=" << info.height << ", frame_id=" << info.frame_id
              << ", fx=" << info.k[0] << ", fy=" << info.k[4] << ", cx=" << info.k[2]
              << ", cy=" << info.k[5] << ", distortion_model=" << info.distortion_model;
}

core::types::Image FoxgloveRenderer::renderFromPose(const core::types::Pose& camera_pose) {
    torch::NoGradGuard no_grad;

    LOG(INFO) << "[FoxgloveRenderer] ===== Starting render call =====";
    LOG(INFO) << "[FoxgloveRenderer] Render camera pose: position=[" << camera_pose.position.x()
              << ", " << camera_pose.position.y() << ", " << camera_pose.position.z()
              << "], orientation=[w=" << camera_pose.orientation.w()
              << ", x=" << camera_pose.orientation.x() << ", y=" << camera_pose.orientation.y()
              << ", z=" << camera_pose.orientation.z() << "], frame_id=" << camera_pose.frame_id;

    if (has_received_camera_info_) {
        LOG(INFO) << "[FoxgloveRenderer] Render camera info: width="
                  << last_received_camera_info_.width
                  << ", height=" << last_received_camera_info_.height
                  << ", fx=" << camera_intrinsics_tensor_[0][0][0].item<double>()
                  << ", fy=" << camera_intrinsics_tensor_[0][1][1].item<double>()
                  << ", cx=" << camera_intrinsics_tensor_[0][0][2].item<double>()
                  << ", cy=" << camera_intrinsics_tensor_[0][1][2].item<double>();
    } else {
        LOG(WARNING) << "[FoxgloveRenderer] No camera info received yet";
    }

    LOG(INFO) << "[FoxgloveRenderer] has_splats_=" << has_splats_
              << ", num_splats=" << current_tensors_.number_of_splats();

    torch::Tensor camera_pose_tensor = poseToTorchTensor(camera_pose);
    LOG(INFO) << "Camera pose tensor: " << camera_pose_tensor.device();
    LOG(INFO) << "Camera intrinsics tensor: " << camera_intrinsics_tensor_.device();

    rendering::RasterizationOutput output =
        rasterizer_.rasterize(current_tensors_, camera_pose_tensor, camera_intrinsics_tensor_,
                              config_.image_width, config_.image_height);

    if (!output.success) {
        LOG(ERROR) << "[FoxgloveRenderer] Rasterization failed";
        return core::types::Image();
    }

    torch::Tensor image_cpu = output.rendered_image.to(torch::kCPU);
    image_cpu = (image_cpu * 255.0f).clamp(0, 255).to(torch::kUInt8);

    cv::Mat cv_image(config_.image_height, config_.image_width, CV_8UC3,
                     image_cpu.data_ptr<uint8_t>());
    cv::Mat cv_image_clone = cv_image.clone();
    cv::cvtColor(cv_image_clone, cv_image_clone, cv::COLOR_RGB2BGR);

    return core::types::Image::fromCvMat(cv_image_clone, "bgr8", config_.frame_id);
}

torch::Tensor FoxgloveRenderer::poseToTorchTensor(const core::types::Pose& pose) {
    Eigen::Isometry3d isometry = pose.getEigenIsometry();
    // Eigen::Isometry3d camera_in_world = isometry.inverse();
    Eigen::Isometry3d camera_in_world = isometry;
    Eigen::Matrix4d pose_matrix = camera_in_world.matrix();

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

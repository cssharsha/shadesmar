// Helper methods borrowed from the original implementation
// This file is included by training_gs_processor.cpp

// Helper method to extract color from keyframe observations
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
Eigen::Vector3f TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                              RasterizationT>::extractColorFromKeyframes(
    const core::types::Keypoint& keypoint,
    const std::unordered_map<uint64_t, core::types::KeyFrame::Ptr>& keyframe_map) {
    
    std::vector<Eigen::Vector3f> colors;
    std::vector<float> weights;
    
    // Extract color from each keyframe observation
    for (const auto& location : keypoint.locations) {
        auto kf_it = keyframe_map.find(location.keyframe_id);
        if (kf_it == keyframe_map.end()) continue;
        
        const auto& keyframe = kf_it->second;
        if (!keyframe->hasColorImage()) continue;
        
        // Extract pixel color at keypoint location
        const auto& color_image = keyframe->getColorImage();
        cv::Mat cv_image = color_image.toCvMat();
        
        // Convert pixel coordinates to image coordinates
        int x = static_cast<int>(location.x);
        int y = static_cast<int>(location.y);
        
        // Check bounds
        if (x >= 0 && x < cv_image.cols && y >= 0 && y < cv_image.rows) {
            cv::Vec3b pixel = cv_image.at<cv::Vec3b>(y, x);
            
            // Convert BGR to RGB and normalize
            Eigen::Vector3f color(pixel[2] / 255.0f, pixel[1] / 255.0f, pixel[0] / 255.0f);
            
            // Use uniform weighting since response is not available in Location struct
            float weight = 1.0f;
            
            colors.push_back(color);
            weights.push_back(weight);
        }
    }
    
    // Compute weighted average color
    if (!colors.empty()) {
        Eigen::Vector3f weighted_color = Eigen::Vector3f::Zero();
        float total_weight = 0.0f;
        
        for (size_t i = 0; i < colors.size(); ++i) {
            weighted_color += weights[i] * colors[i];
            total_weight += weights[i];
        }
        
        if (total_weight > 0) {
            return weighted_color / total_weight;
        }
    }
    
    // Fallback to neutral color if no valid observations
    return Eigen::Vector3f(0.7f, 0.7f, 0.7f);
}

// Helper method to compute covariance from keypoint observations
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
Eigen::Matrix3d TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                              RasterizationT>::computeKeypointCovariance(
    const core::types::Keypoint& keypoint,
    const std::unordered_map<uint64_t, core::types::KeyFrame::Ptr>& keyframe_map) {
    
    // Base covariance scaled by observation quality
    double base_variance = 0.01;
    
    // Scale based on number of observations (more observations = lower uncertainty)
    if (keypoint.locations.size() > 3) {
        base_variance = 0.005;
    } else if (keypoint.locations.size() > 1) {
        base_variance = 0.008;
    }
    
    // Compute viewing angle variance - points seen from more diverse angles are more certain
    std::vector<Eigen::Vector3d> viewing_directions;
    for (const auto& location : keypoint.locations) {
        auto kf_it = keyframe_map.find(location.keyframe_id);
        if (kf_it != keyframe_map.end()) {
            const auto& keyframe = kf_it->second;
            Eigen::Vector3d view_dir = (keypoint.position - keyframe->pose.position).normalized();
            viewing_directions.push_back(view_dir);
        }
    }
    
    // Compute viewing angle spread
    if (viewing_directions.size() > 1) {
        double angle_variance = 0.0;
        for (size_t i = 0; i < viewing_directions.size(); ++i) {
            for (size_t j = i + 1; j < viewing_directions.size(); ++j) {
                double angle = std::acos(std::clamp(viewing_directions[i].dot(viewing_directions[j]), -1.0, 1.0));
                angle_variance += angle * angle;
            }
        }
        angle_variance /= (viewing_directions.size() * (viewing_directions.size() - 1) / 2);
        
        // More diverse viewing angles reduce uncertainty
        double angle_factor = std::exp(-angle_variance * 2.0);
        base_variance *= (0.5 + 0.5 * angle_factor);
    }
    
    // Scale by distance from cameras (closer points are more certain)
    if (!viewing_directions.empty()) {
        double avg_distance = 0.0;
        int count = 0;
        for (const auto& location : keypoint.locations) {
            auto kf_it = keyframe_map.find(location.keyframe_id);
            if (kf_it != keyframe_map.end()) {
                const auto& keyframe = kf_it->second;
                avg_distance += (keypoint.position - keyframe->pose.position).norm();
                count++;
            }
        }
        if (count > 0) {
            avg_distance /= count;
            // Scale variance with distance (farther points are less certain)
            base_variance *= std::max(0.5, std::min(2.0, avg_distance / 5.0));
        }
    }
    
    return Eigen::Matrix3d::Identity() * base_variance;
}

// Helper method to compute initial opacity
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
float TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::computeInitialOpacity(const core::types::Keypoint& keypoint) {
    // Higher opacity for keypoints with more observations
    float opacity = 0.5f + 0.3f * std::min(1.0f, keypoint.locations.size() / 5.0f);
    return std::clamp(opacity, 0.1f, 0.9f);
}

// Helper method to compute initial confidence
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
float TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::computeInitialConfidence(const core::types::Keypoint& keypoint) {
    // Confidence based on observation count and descriptor quality
    float base_confidence = 0.8f;
    if (keypoint.locations.size() > 2) {
        base_confidence = 0.95f;
    } else if (keypoint.locations.size() > 1) {
        base_confidence = 0.85f;
    }
    
    return std::clamp(base_confidence, 0.3f, 1.0f);
}

// Scene bounds estimation
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
std::pair<Eigen::Vector3f, Eigen::Vector3f> TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT, RasterizationT>::estimateSceneBounds() {
    Eigen::Vector3f scene_min = config_.scene_min;
    Eigen::Vector3f scene_max = config_.scene_max;

    try {
        // Try to estimate bounds from existing keyframes
        auto keyframes = map_store_->getAllKeyFrames();
        if (!keyframes.empty()) {
            LOG(INFO) << "Estimating scene bounds from " << keyframes.size() << " keyframes";

            // Initialize with first keyframe position
            Eigen::Vector3f first_pos = keyframes[0]->pose.position.cast<float>();
            scene_min = first_pos;
            scene_max = first_pos;

            // Expand bounds to include all keyframe positions
            for (const auto& kf : keyframes) {
                Eigen::Vector3f pos = kf->pose.position.cast<float>();
                scene_min = scene_min.cwiseMin(pos);
                scene_max = scene_max.cwiseMax(pos);
            }

            // Add padding around camera trajectory
            Eigen::Vector3f padding(2.0f, 2.0f, 2.0f);
            scene_min -= padding;
            scene_max += padding;

            LOG(INFO) << "Estimated scene bounds from keyframes: min(" << scene_min.transpose()
                      << ") max(" << scene_max.transpose() << ")";
        } else {
            LOG(INFO) << "No keyframes available, using config scene bounds";
        }

        // Also try to incorporate existing keypoints if available
        auto keypoints = map_store_->getAllKeyPoints();
        if (!keypoints.empty() && keyframes.empty()) {
            LOG(INFO) << "Estimating scene bounds from " << keypoints.size() << " keypoints";

            // Initialize with first keypoint
            Eigen::Vector3f first_pos = keypoints[0].position.cast<float>();
            scene_min = first_pos;
            scene_max = first_pos;

            // Expand to include all keypoints
            for (const auto& kp : keypoints) {
                if (!kp.needs_triangulation) {  // Only use triangulated keypoints
                    Eigen::Vector3f pos = kp.position.cast<float>();
                    scene_min = scene_min.cwiseMin(pos);
                    scene_max = scene_max.cwiseMax(pos);
                }
            }

            // Add padding
            Eigen::Vector3f padding(1.0f, 1.0f, 1.0f);
            scene_min -= padding;
            scene_max += padding;
        }

    } catch (const std::exception& e) {
        LOG(WARNING) << "Failed to estimate scene bounds from map data: " << e.what()
                     << ", using config bounds";
    }

    return std::make_pair(scene_min, scene_max);
}

// Image processing
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::extractImageTensor(const core::types::KeyFrame::Ptr& keyframe,
                                        torch::Tensor& image_tensor,
                                        core::types::CameraInfo& camera_info) {
    try {
        // Check if keyframe has color image data
        if (!keyframe->hasColorImage()) {
            LOG(ERROR) << "Keyframe " << keyframe->id << " has no color image";
            return false;
        }

        // Get camera info
        if (!keyframe->hasCameraInfo()) {
            LOG(ERROR) << "Keyframe " << keyframe->id << " has no camera info";
            return false;
        }

        camera_info = keyframe->getCameraInfo();
        const auto& color_image = keyframe->getColorImage();

        // Convert OpenCV Mat to LibTorch tensor
        cv::Mat cv_image = color_image.toCvMat();

        // Ensure image is in RGB format (OpenCV uses BGR by default)
        if (cv_image.channels() == 3) {
            cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2RGB);
        }

        // Resize to training resolution
        cv::Mat resized_image;
        cv::resize(cv_image, resized_image,
                   cv::Size(training_config_.initial_width, training_config_.initial_height));

        // Convert to float and normalize to [0, 1]
        cv::Mat float_image;
        resized_image.convertTo(float_image, CV_32F, 1.0 / 255.0);

        // Convert to LibTorch tensor (HWC -> CHW format)
        image_tensor =
            torch::from_blob(float_image.data,
                             {training_config_.initial_height, training_config_.initial_width, 3},
                             torch::kFloat32)
                .clone();

        // Permute dimensions from HWC to CHW (channels first for neural networks)
        image_tensor = image_tensor.permute({2, 0, 1});

        // Move to configured device
        image_tensor = image_tensor.to(config_.device);

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to extract image tensor: " << e.what();
        return false;
    }
}

// Camera parameter extraction
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::extractCameraPoses(const std::vector<uint64_t>& keyframe_ids,
                                        std::vector<Eigen::Isometry3d>& camera_poses,
                                        const std::string& target_frame) {
    try {
        camera_poses.clear();
        camera_poses.reserve(keyframe_ids.size());

        if (!transform_tree_) {
            LOG(ERROR) << "Transform tree not available for camera pose extraction";
            return false;
        }

        // Get static transform from base_link to camera frame
        auto getTransformFromTree = [&](const std::string& camera_frame) {
            try {
                auto transform_result = transform_tree_->getTransform("base_link", "camera");
                return transform_result.transform;
            } catch (const std::exception& e) {
                LOG(WARNING) << "Failed to get base_link to camera transform: " << e.what()
                             << ". Using identity transform.";
                return Eigen::Isometry3d::Identity();
            }
        };

        for (const auto& kf_id : keyframe_ids) {
            auto keyframe = map_store_->getKeyFrame(kf_id);
            if (!keyframe) {
                LOG(WARNING) << "Failed to load keyframe " << kf_id << " for pose extraction";
                continue;
            }

            Eigen::Isometry3d T_baselink_camera = getTransformFromTree("camera_color_optical_frame");

            // Get keyframe's pose in world frame (this is base_link pose in odom frame)
            Eigen::Isometry3d T_world_baselink = keyframe->pose.getEigenIsometry();

            // Chain transforms: T_world_camera = T_world_baselink * T_baselink_camera
            Eigen::Isometry3d T_world_camera = T_world_baselink * T_baselink_camera;

            // Transform to target frame if different from world/odom
            if (target_frame != "world" && target_frame != "odom") {
                try {
                    auto transform_result = transform_tree_->getTransform(target_frame, "odom");
                    T_world_camera = transform_result.transform * T_world_camera;
                } catch (const std::exception& e) {
                    LOG(WARNING) << "Failed to transform to target frame " << target_frame << ": "
                                 << e.what() << ". Using odom frame.";
                }
            }

            camera_poses.push_back(T_world_camera);
        }

        return !camera_poses.empty();

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error extracting camera poses: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
torch::Tensor TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::convertCameraIntrinsicsToTensor(const core::types::CameraInfo& camera_info) {
    try {
        // Convert camera matrix to tensor
        Eigen::Matrix3d K = camera_info.getKInEigen();

        // Create tensor from camera matrix
        torch::Tensor K_tensor =
            torch::from_blob(K.data(), {3, 3}, torch::kFloat64).clone().to(torch::kFloat32);

        // Move to configured device
        K_tensor = K_tensor.to(config_.device);

        return K_tensor;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error converting camera intrinsics to tensor: " << e.what();
        return torch::Tensor();
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
torch::Tensor TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::convertCameraPoseToTensor(const Eigen::Isometry3d& pose) {
    try {
        // Convert pose to 4x4 homogeneous transformation matrix
        Eigen::Matrix4d pose_matrix = pose.matrix();

        // Create tensor from pose matrix
        torch::Tensor pose_tensor = torch::from_blob(pose_matrix.data(), {4, 4}, torch::kFloat64)
                                        .clone()
                                        .to(torch::kFloat32);

        // Move to configured device
        pose_tensor = pose_tensor.to(config_.device);

        return pose_tensor;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error converting camera pose to tensor: " << e.what();
        return torch::Tensor();
    }
}

// Rendering and training
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::renderCurrentSplats(const std::vector<core::types::GaussianSplat>& splats,
                                         const torch::Tensor& camera_pose,
                                         const torch::Tensor& camera_intrinsics,
                                         torch::Tensor& rendered_image) {
    try {
        if (!rasterizer_) {
            LOG(ERROR) << "Rasterizer not initialized";
            return false;
        }

        // Prepare splats for rasterization
        rendering::RasterizationInput input = rendering::prepareSplatsForRasterization(
            splats, camera_pose, camera_intrinsics, config_.device);

        if (!input.isValid()) {
            LOG(ERROR) << "Failed to prepare splats for rasterization";
            return false;
        }

        // Perform rasterization
        rendering::RasterizationOutput output = rasterizer_->rasterize(input);

        if (!output.isValid()) {
            LOG(ERROR) << "Rasterization failed";
            return false;
        }

        rendered_image = output.rendered_image;

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error during splat rendering: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
float TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::computeRenderingLoss(const torch::Tensor& rendered_image,
                                          const torch::Tensor& ground_truth_image) {
    try {
        // Ensure images are on the same device and have compatible shapes
        torch::Tensor gt_image = ground_truth_image.to(config_.device);

        if (rendered_image.sizes() != gt_image.sizes()) {
            LOG(WARNING) << "Image size mismatch - rendered: " << rendered_image.sizes()
                         << " vs ground truth: " << gt_image.sizes();

            // Resize ground truth to match rendered image
            gt_image =
                torch::nn::functional::interpolate(
                    gt_image.unsqueeze(0),
                    torch::nn::functional::InterpolateFuncOptions()
                        .size(std::vector<int64_t>{rendered_image.size(1), rendered_image.size(2)})
                        .mode(torch::kBilinear)
                        .align_corners(false))
                    .squeeze(0);
        }

        // Compute L1 loss (photometric loss)
        torch::Tensor l1_loss = torch::nn::functional::l1_loss(rendered_image, gt_image);

        // Compute SSIM-based loss for structural similarity
        torch::Tensor rendered_mean = torch::mean(rendered_image, {1, 2}, true);
        torch::Tensor gt_mean = torch::mean(gt_image, {1, 2}, true);

        torch::Tensor rendered_var = torch::var(rendered_image, {1, 2}, true);
        torch::Tensor gt_var = torch::var(gt_image, {1, 2}, true);

        torch::Tensor covariance =
            torch::mean((rendered_image - rendered_mean) * (gt_image - gt_mean), {1, 2}, true);

        float c1 = 0.01f * 0.01f;  // stability constants
        float c2 = 0.03f * 0.03f;

        torch::Tensor ssim = (2 * rendered_mean * gt_mean + c1) * (2 * covariance + c2) /
                             ((rendered_mean * rendered_mean + gt_mean * gt_mean + c1) *
                              (rendered_var + gt_var + c2));

        torch::Tensor ssim_loss = 1.0f - torch::mean(ssim);

        // Combined loss (weighted combination)
        float alpha = 0.8f;  // Weight for L1 loss
        float beta = 0.2f;   // Weight for SSIM loss

        torch::Tensor total_loss = alpha * l1_loss + beta * ssim_loss;

        float loss_value = common::itemAs(total_loss);

        return loss_value;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error computing rendering loss: " << e.what();
        return std::numeric_limits<float>::max();
    }
}
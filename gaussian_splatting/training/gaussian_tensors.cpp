#include "gaussian_tensors.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <cmath>
#include <iostream>
#include <logging/logging.hpp>
#include <vector>
#include "gaussian_splatting/common/tensor_config.hpp"

namespace gaussian_splatting {

bool GaussianTensors::isValid() const {
    // For now just return true for everything
    return true;
}

bool GaussianTensors::fromSplats(const std::vector<core::types::GaussianSplat>& splats) {
    LOG(INFO) << "Initilixing tensors from " << splats.size() << " splats";
    int64_t num_splats = splats.size();
    if (num_splats == 0) {
        return false;  // Return default-constructed (empty) tensors
    }

    int64_t sh_dim = 0;
    if (!splats.empty()) {
        sh_dim = splats[0].sh_coefficients.size();
    }
    LOG(INFO) << "SH dims: " << sh_dim;

    auto long_opts = torch::TensorOptions().dtype(torch::kInt64);
    auto tensor_opts = common::getTensorOptions();

    // Create tensors with configured precision
    positions = torch::empty({num_splats, 3}, tensor_opts);
    covariances = torch::empty({num_splats, 3, 3}, tensor_opts);
    colors = torch::empty({num_splats, 3}, tensor_opts);
    opacities = torch::empty({num_splats, 1}, tensor_opts);
    scales = torch::empty({num_splats, 3}, tensor_opts);
    rotations = torch::empty({num_splats, 4}, tensor_opts);
    confidences = torch::empty({num_splats, 1}, tensor_opts);

    auto* positions_ptr = common::dataPtrAs(positions);
    auto* covariances_ptr = common::dataPtrAs(covariances);
    auto* colors_ptr = common::dataPtrAs(colors);
    auto* opacities_ptr = common::dataPtrAs(opacities);
    auto* scales_ptr = common::dataPtrAs(scales);
    auto* rotations_ptr = common::dataPtrAs(rotations);
    auto* confidences_ptr = common::dataPtrAs(confidences);

    // Initialize opacities in logit space: logit(0.1)
    LOG(INFO) << "Opacity size: " << opacities.sizes();
    for (int64_t i = 0; i < num_splats; ++i) {
        const auto& splat = splats[i];

        // Set opacity in logit space: logit(x) = log(x / (1 - x))
        opacities_ptr[i] = splat.opacity;
        confidences_ptr[i] = splat.confidence;

        // Copy positions (cast from double to float)
        positions_ptr[i * 3 + 0] = splat.position.x();
        positions_ptr[i * 3 + 1] = splat.position.y();
        positions_ptr[i * 3 + 2] = splat.position.z();

        // Copy colors
        colors_ptr[i * 3 + 0] = static_cast<common::scalar_t>(splat.color.x());
        colors_ptr[i * 3 + 1] = static_cast<common::scalar_t>(splat.color.y());
        colors_ptr[i * 3 + 2] = static_cast<common::scalar_t>(splat.color.z());

        // Use KNN-computed scales in log-space
        scales_ptr[i * 3 + 0] = static_cast<common::scalar_t>(std::log(splat.scale.x()));
        scales_ptr[i * 3 + 1] = static_cast<common::scalar_t>(std::log(splat.scale.y()));
        scales_ptr[i * 3 + 2] = static_cast<common::scalar_t>(std::log(splat.scale.z()));
        rotations_ptr[i * 4 + 0] = splat.rotation.w();
        rotations_ptr[i * 4 + 1] = splat.rotation.x();
        rotations_ptr[i * 4 + 2] = splat.rotation.y();
        rotations_ptr[i * 4 + 3] = splat.rotation.z();

        // Copy covariances
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                covariances_ptr[i * 9 + j * 3 + k] =
                    static_cast<common::scalar_t>(splat.covariance(j, k));
            }
        }

        if (i == 0) {
            LOG(INFO) << "Position: " << splat.position.transpose();
            LOG(INFO) << "Tensor position: [" << positions_ptr[0] << ", " << positions_ptr[1]
                      << ", " << positions_ptr[2] << "]";
            LOG(INFO) << "Color: " << splat.color.transpose();
            LOG(INFO) << "Tensor color: [" << colors_ptr[0] << ", " << colors_ptr[1] << ", "
                      << colors_ptr[2] << "]";
            LOG(INFO) << "Scale: " << splat.scale.transpose();
            LOG(INFO) << "Tensor scale: [" << scales_ptr[0] << ", " << scales_ptr[1] << ", "
                      << scales_ptr[2] << "]";
            LOG(INFO) << "Rotation: " << splat.rotation.coeffs().transpose();
            LOG(INFO) << "Tensor rotation: [" << rotations_ptr[0] << ", " << rotations_ptr[1]
                      << ", " << rotations_ptr[2] << ", " << rotations_ptr[3] << "]";
            LOG(INFO) << "SH coeffs: " << splat.sh_coefficients.transpose();
            LOG(INFO) << "Opacity: " << splat.opacity;
            LOG(INFO) << "Tensor opacity: " << opacities_ptr[0];
        }
    }
    // Setup spherical harmonics from colors
    // feature shape would be (sh_degree + 1) * (sh_degree + 1)
    const int sh_degree = 3;  // Standard degree 3 for Gaussian Splatting
    const int64_t feature_shape = (sh_degree + 1) * (sh_degree + 1);  // 16 for degree 3

    // Shape: [N, K, 3] where K is number of SH coefficients
    auto shs = torch::zeros({colors.size(0), feature_shape, 3}, tensor_opts);

    // Convert RGB to SH DC component: (rgb - 0.5) / C0 where C0 = 0.28209479177387814 (Y_0^0)
    // This matches gsplat's Python implementation in utils.py line 94
    constexpr double C0 = 0.28209479177387814f;
    torch::Tensor harmonics_colors = (colors - 0.5) / C0;
    LOG(INFO) << "Colors: " << colors[0];
    LOG(INFO) << "Harmonics colors: " << harmonics_colors[0];

    // Assign DC component (first SH coefficient): shs[:, 0, :] = harmonics_colors
    shs.index_put_({torch::indexing::Slice(), 0, torch::indexing::Slice()}, harmonics_colors);

    // Extract sh_0: first coefficient with dimension preserved [N, 1, 3]
    sh_0 = shs.index({torch::indexing::Slice(), torch::indexing::Slice(0, 1),
                      torch::indexing::Slice()})
               .contiguous();

    // Extract sh_N: remaining coefficients [N, K-1, 3]
    sh_N = shs.index({torch::indexing::Slice(), torch::indexing::Slice(1, torch::indexing::None),
                      torch::indexing::Slice()})
               .contiguous();

    // Set scene scale by getting the mean of the camera positions
    // and then getting the median of the distance between each camera
    // and the mean
    auto mean_position = torch::mean(positions, 0);
    scene_scale = static_cast<float>(
        common::itemAs(torch::median(torch::norm(positions - mean_position, 2, 1))));

    LOG(INFO) << "Mean position: " << mean_position;
    LOG(INFO) << "Scene scale: " << scene_scale;

    LOG(INFO) << "Did all the conversions";
    return true;
}

void GaussianTensors::to(const torch::Device& device) {
    LOG(INFO) << "Converting tensors to device " << device.type();
    // source_keypoint_ids = source_keypoint_ids.to(device);
    if (positions.device() != device) {
        positions = positions.to(device);
    }
    // colors = colors.to(device);
    if (opacities.device() != device) {
        opacities = opacities.to(device);
    }
    if (scales.device() != device) {
        scales = scales.to(device);
    }
    if (rotations.device() != device) {
        rotations = rotations.to(device);
    }
    if (sh_0.device() != device) {
        sh_0 = sh_0.to(device);
    }
    if (sh_N.device() != device) {
        sh_N = sh_N.to(device);
    }
    // confidences = confidences.to(device);
    // timestamps = timestamps.to(device);
}

void GaussianTensors::setRequiresGrad(bool requires_grad) {
    positions.requires_grad_(requires_grad);
    colors.requires_grad_(requires_grad);
    opacities.requires_grad_(requires_grad);
    scales.requires_grad_(requires_grad);
    rotations.requires_grad_(requires_grad);
    sh_0.requires_grad_(requires_grad);
    sh_N.requires_grad_(requires_grad);
}

void GaussianTensors::printGradInfo() const {
    LOG(INFO) << "Printing the grads";

    LOG(INFO) << "Positions: " << positions.grad();
}

std::vector<core::types::GaussianSplat> GaussianTensors::toSplats() {
    if (!isValid()) {
        return {};
    }

    // NoGradGuard ensures we don't track gradients for this conversion
    torch::NoGradGuard no_grad;

    // Copy all tensors to CPU without affecting the GPU tensors or their gradients
    auto positions_cpu = positions.to(torch::kCPU);
    auto opacities_cpu = opacities.to(torch::kCPU);
    auto scales_cpu = scales.to(torch::kCPU);
    auto rotations_cpu = rotations.to(torch::kCPU);
    auto sh_0_cpu = sh_0.to(torch::kCPU);
    auto confidences_cpu = confidences.to(torch::kCPU);

    int64_t num_splats = positions_cpu.size(0);
    std::vector<core::types::GaussianSplat> splats(num_splats);
    LOG(INFO) << "Converting " << num_splats << " splats to CPU for visualization";

    // Convert SH back to colors on CPU: colors = sh_0 * C0 + 0.5
    // sh_0 shape is [N, 1, 3], squeeze to [N, 3]
    // This is the inverse of: sh_0 = (colors - 0.5) / C0
    constexpr float C0 = 0.28209479177387814f;
    constexpr float BRIGHTNESS_MULTIPLIER = 2.0f;
    torch::Tensor reconstructed_colors = (sh_0_cpu.squeeze(1) * C0 + 0.5f) * BRIGHTNESS_MULTIPLIER;
    reconstructed_colors = reconstructed_colors.clamp(0.0f, 1.0f);

    // Get data pointers from the CPU tensors
    const auto* positions_ptr = common::dataPtrAs(positions_cpu);
    const auto* reconstructed_colors_ptr = common::dataPtrAs(reconstructed_colors);
    const auto* opacities_ptr = common::dataPtrAs(opacities_cpu);
    const auto* scales_ptr = common::dataPtrAs(scales_cpu);
    const auto* rotations_ptr = common::dataPtrAs(rotations_cpu);
    const auto* confidences_ptr = common::dataPtrAs(confidences_cpu);

    for (int64_t i = 0; i < num_splats; ++i) {
        // Convert opacity from logit space back to [0, 1]: sigmoid(x) = 1 / (1 + exp(-x))
        float opacity_logit = opacities_ptr[i];
        splats[i].opacity = 1.0f / (1.0f + std::exp(-opacity_logit));
        splats[i].confidence = confidences_ptr[i];

        // Cast float tensor values to double for positions
        splats[i].position = Eigen::Vector3d(positions_ptr[i * 3 + 0], positions_ptr[i * 3 + 1],
                                             positions_ptr[i * 3 + 2]);

        // Colors reconstructed from SH
        splats[i].color = Eigen::Vector3f(static_cast<float>(reconstructed_colors_ptr[i * 3 + 0]),
                                          static_cast<float>(reconstructed_colors_ptr[i * 3 + 1]),
                                          static_cast<float>(reconstructed_colors_ptr[i * 3 + 2]));

        // Convert scales from log space back to linear: exp(log_scale)
        // This matches Python: scales = torch.exp(self.splats["scales"])
        splats[i].scale =
            Eigen::Vector3d(std::exp(scales_ptr[i * 3 + 0]),
                           std::exp(scales_ptr[i * 3 + 1]),
                           std::exp(scales_ptr[i * 3 + 2]));
        // splats[i].scale = Eigen::Vector3d(2., 2., 2.);

        // Cast float tensor values to double for rotations and normalize
        splats[i].rotation = Eigen::Quaterniond(rotations_ptr[i * 4 + 0], rotations_ptr[i * 4 + 1],
                                                rotations_ptr[i * 4 + 2], rotations_ptr[i * 4 + 3])
                                 .normalized();
    }

    LOG(INFO) << "Finished converting " << splats.size() << " splats for visualization";

    // Note: We don't modify the GPU tensors at all, they remain on CUDA with gradients intact
    return splats;
}

}  // namespace gaussian_splatting

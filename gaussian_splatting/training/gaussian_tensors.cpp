#include "gaussian_tensors.hpp"
#include <cmath>
#include <iostream>
#include <logging/logging.hpp>
#include <vector>

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
    // auto double_opts = torch::TensorOptions().dtype(torch::kFloat64);
    auto double_opts = torch::TensorOptions().dtype(torch::kFloat32);
    auto double_opts1 = torch::TensorOptions().dtype(torch::kFloat64);
    auto float_opts = torch::TensorOptions().dtype(torch::kFloat32);

    positions = torch::empty({num_splats, 3}, double_opts);
    covariances = torch::empty({num_splats, 3, 3}, double_opts);
    colors = torch::empty({num_splats, 3}, float_opts);
    opacities = torch::empty({num_splats, 1}, float_opts);
    scales = torch::empty({num_splats, 3}, double_opts);
    rotations = torch::empty({num_splats, 4}, double_opts);
    confidences = torch::empty({num_splats, 1}, float_opts);

    auto* positions_ptr = positions.data_ptr<float>();
    auto* covariances_ptr = covariances.data_ptr<float>();
    auto* colors_ptr = colors.data_ptr<float>();
    auto* opacities_ptr = opacities.data_ptr<float>();
    auto* scales_ptr = scales.data_ptr<float>();
    auto* rotations_ptr = rotations.data_ptr<float>();
    auto* confidences_ptr = confidences.data_ptr<float>();

    // Initialize opacities in logit space: logit(0.1)
    std::cout << "Opacity size: " << opacities.sizes() << std::endl;
    for (int64_t i = 0; i < num_splats; ++i) {
        const auto& splat = splats[i];

        // Set opacity in logit space: logit(x) = log(x / (1 - x))
        opacities_ptr[i] = splat.opacity;
        confidences_ptr[i] = splat.confidence;

        // Copy positions (cast from double to float)
        positions_ptr[i * 3 + 0] = static_cast<float>(splat.position.x());
        positions_ptr[i * 3 + 1] = static_cast<float>(splat.position.y());
        positions_ptr[i * 3 + 2] = static_cast<float>(splat.position.z());

        // Copy colors (already float, check if normalization needed)
        colors_ptr[i * 3 + 0] = splat.color.x();
        colors_ptr[i * 3 + 1] = splat.color.y();
        colors_ptr[i * 3 + 2] = splat.color.z();

        // Copy scales (cast from double to float)
        scales_ptr[i * 3 + 0] = 0.1;
        scales_ptr[i * 3 + 1] = 0.1;
        scales_ptr[i * 3 + 2] = 0.1;

        // scales_ptr[i * 3 + 0] = static_cast<float>(splat.scale.x());
        // scales_ptr[i * 3 + 1] = static_cast<float>(splat.scale.y());
        // scales_ptr[i * 3 + 2] = static_cast<float>(splat.scale.z());
        // Copy rotations (cast from double to float)
        rotations_ptr[i * 4 + 0] = static_cast<float>(splat.rotation.w());
        rotations_ptr[i * 4 + 1] = static_cast<float>(splat.rotation.x());
        rotations_ptr[i * 4 + 2] = static_cast<float>(splat.rotation.y());
        rotations_ptr[i * 4 + 3] = static_cast<float>(splat.rotation.z());

        // Copy covariances (cast from double to float)
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                covariances_ptr[i * 9 + j * 3 + k] = static_cast<float>(splat.covariance(j, k));
            }
        }

        if (i == 0) {
            std::cout << "Position: " << splat.position.transpose() << std::endl;
            std::cout << "Tensor position: [" << positions_ptr[0] << ", " << positions_ptr[1]
                      << ", " << positions_ptr[2] << "]" << std::endl;
            std::cout << "Color: " << splat.color.transpose() << std::endl;
            std::cout << "Tensor color: [" << colors_ptr[0] << ", " << colors_ptr[1] << ", "
                      << colors_ptr[2] << "]" << std::endl;
            std::cout << "Scale: " << splat.scale.transpose() << std::endl;
            std::cout << "Tensor scale: [" << scales_ptr[0] << ", " << scales_ptr[1] << ", "
                      << scales_ptr[2] << "]" << std::endl;
            std::cout << "Rotation: " << splat.rotation.coeffs().transpose() << std::endl;
            std::cout << "Tensor rotation: [" << rotations_ptr[0] << ", " << rotations_ptr[1]
                      << ", " << rotations_ptr[2] << ", " << rotations_ptr[3] << "]" << std::endl;
            std::cout << "SH coeffs: " << splat.sh_coefficients.transpose() << std::endl;
            std::cout << "Opacity: " << splat.opacity << std::endl;
            std::cout << "Tensor opacity: " << opacities_ptr[0] << std::endl;
        }
    }
    // Setup spherical harmonics from colors
    // feature shape would be (sh_degree + 1) * (sh_degree + 1)
    const int sh_degree = 3;  // Standard degree 3 for Gaussian Splatting
    const int64_t feature_shape = (sh_degree + 1) * (sh_degree + 1);  // 16 for degree 3

    // Shape: [N, K, 3] where K is number of SH coefficients
    auto shs = torch::zeros({colors.size(0), feature_shape, 3}, float_opts);

    // Convert RGB to SH DC component: (rgb - 0.5) / C0 where C0 = 0.28209479177387814 (Y_0^0)
    // This matches gsplat's Python implementation in utils.py line 94
    constexpr float C0 = 0.28209479177387814f;
    torch::Tensor harmonics_colors = (colors - 0.5f) / C0;
    std::cout << "Colors: " << colors[0] << std::endl;
    std::cout << "Harmonics colors: " << harmonics_colors[0] << std::endl;

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

    LOG(INFO) << "Did all the conversions";
    return true;
}

void GaussianTensors::to(const torch::Device& device) {
    std::cout << "Converting tensors to device " << device.type() << std::endl;
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
    std::cout << "Printing the grads" << std::endl;

    LOG(INFO) << "Positions: " << positions.grad();
    std::cout << "Positions: " << positions.grad() << std::endl;
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
    std::cout << "Converting " << num_splats << " splats to CPU for visualization" << std::endl;

    // Convert SH back to colors on CPU: colors = sh_0 * C0 + 0.5
    // sh_0 shape is [N, 1, 3], squeeze to [N, 3]
    // This is the inverse of: sh_0 = (colors - 0.5) / C0
    constexpr float C0 = 0.28209479177387814f;
    torch::Tensor reconstructed_colors = sh_0_cpu.squeeze(1) * C0 + 0.5f;
    reconstructed_colors = reconstructed_colors.clamp(0.0f, 1.0f);

    // Get data pointers from the CPU tensors
    const auto* positions_ptr = positions_cpu.data_ptr<float>();
    const auto* reconstructed_colors_ptr = reconstructed_colors.data_ptr<float>();
    const auto* opacities_ptr = opacities_cpu.data_ptr<float>();
    const auto* scales_ptr = scales_cpu.data_ptr<float>();
    const auto* rotations_ptr = rotations_cpu.data_ptr<float>();
    const auto* confidences_ptr = confidences_cpu.data_ptr<float>();

    for (int64_t i = 0; i < num_splats; ++i) {
        // Convert opacity from logit space back to [0, 1]: sigmoid(x) = 1 / (1 + exp(-x))
        float opacity_logit = opacities_ptr[i];
        splats[i].opacity = 1.0f / (1.0f + std::exp(-opacity_logit));
        splats[i].confidence = confidences_ptr[i];

        // Cast float tensor values to double for positions
        splats[i].position = Eigen::Vector3d(static_cast<double>(positions_ptr[i * 3 + 0]),
                                             static_cast<double>(positions_ptr[i * 3 + 1]),
                                             static_cast<double>(positions_ptr[i * 3 + 2]));

        // Colors reconstructed from SH
        splats[i].color = Eigen::Vector3f(reconstructed_colors_ptr[i * 3 + 0],
                                          reconstructed_colors_ptr[i * 3 + 1],
                                          reconstructed_colors_ptr[i * 3 + 2]);

        // Convert scales from log space back to linear: exp(log_scale)
        // This matches Python: scales = torch.exp(self.splats["scales"])
        splats[i].scale = Eigen::Vector3d(static_cast<double>(scales_ptr[i * 3 + 0]),
                                          static_cast<double>(scales_ptr[i * 3 + 1]),
                                          static_cast<double>(scales_ptr[i * 3 + 2]));

        // Cast float tensor values to double for rotations and normalize
        splats[i].rotation = Eigen::Quaterniond(static_cast<double>(rotations_ptr[i * 4 + 0]),
                                                static_cast<double>(rotations_ptr[i * 4 + 1]),
                                                static_cast<double>(rotations_ptr[i * 4 + 2]),
                                                static_cast<double>(rotations_ptr[i * 4 + 3]))
                                 .normalized();
    }

    std::cout << "Finished converting " << splats.size() << " splats for visualization"
              << std::endl;

    // Note: We don't modify the GPU tensors at all, they remain on CUDA with gradients intact
    return splats;
}

}  // namespace gaussian_splatting

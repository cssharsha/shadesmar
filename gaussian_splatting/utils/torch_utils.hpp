#pragma once

#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include <string>

namespace gaussian_splatting {
namespace utils {

/**
 * @brief Converts quaternions to rotation matrices.
 *
 * @param rotations A tensor of quaternions with shape [N, 4] (w, x, y, z).
 * @return A tensor of rotation matrices with shape [N, 3, 3].
 */
inline torch::Tensor quaternion_to_rotation_matrix(const torch::Tensor& rotations) {
    std::cout << "Doing quaternion to rotation matrix" << std::endl;
    auto norm = torch::sqrt(torch::sum(rotations * rotations, -1, true));
    std::cout << "Norm: " << norm.sizes() << std::endl;
    auto q = rotations / norm;
    std::cout << "Q: " << q.sizes() << std::endl;

    auto w = q.index({torch::indexing::Slice(), 0});
    auto x = q.index({torch::indexing::Slice(), 1});
    auto y = q.index({torch::indexing::Slice(), 2});
    auto z = q.index({torch::indexing::Slice(), 3});
    std::cout << "W: " << w.sizes() << std::endl;

    auto R = torch::empty({q.size(0), 3, 3}, q.options());
    std::cout << "R: " << R.sizes() << std::endl;

    R.slice(1, 0, 1).slice(2, 0, 1) = (1.0 - 2.0 * (y * y + z * z)).view({-1, 1, 1});
    R.slice(1, 0, 1).slice(2, 1, 2) = (2.0 * (x * y - w * z)).view({-1, 1, 1});
    R.slice(1, 0, 1).slice(2, 2, 3) = (2.0 * (x * z + w * y)).view({-1, 1, 1});

    R.slice(1, 1, 2).slice(2, 0, 1) = (2.0 * (x * y + w * z)).view({-1, 1, 1});
    R.slice(1, 1, 2).slice(2, 1, 2) = (1.0 - 2.0 * (x * x + z * z)).view({-1, 1, 1});
    R.slice(1, 1, 2).slice(2, 2, 3) = (2.0 * (y * z - w * x)).view({-1, 1, 1});

    R.slice(1, 2, 3).slice(2, 0, 1) = (2.0 * (x * z - w * y)).view({-1, 1, 1});
    R.slice(1, 2, 3).slice(2, 1, 2) = (2.0 * (y * z + w * x)).view({-1, 1, 1});
    R.slice(1, 2, 3).slice(2, 2, 3) = (1.0 - 2.0 * (x * x + y * y)).view({-1, 1, 1});
    std::cout << "R: " << R.sizes() << std::endl;

    return R;
}

/**
 * @brief Computes the 3D covariance matrix from scaling and rotation.
 *
 * @param scales A tensor of scaling factors with shape [N, 3].
 * @param rotations A tensor of quaternions with shape [N, 4].
 * @return A tensor of 3D covariance matrices with shape [N, 3, 3].
 */
inline torch::Tensor get_covariance_from_scaling_rotation(const torch::Tensor& scales,
                                                          const torch::Tensor& rotations) {
    auto R = quaternion_to_rotation_matrix(rotations);
    auto S = torch::diag_embed(torch::exp(scales));  // Use exp for non-negativity

    // Covariance = R * S * S^T * R^T
    auto cov = torch::matmul(R, torch::matmul(S, S.transpose(1, 2)));
    cov = torch::matmul(cov, R.transpose(1, 2));

    return cov;
}

/**
 * @brief Computes the Jacobian of the perspective projection.
 *
 * @param K The camera intrinsics matrix [3, 3].
 * @param means_camera The 3D points in camera coordinates [N, 3].
 * @return The Jacobian matrix for each point [N, 2, 3].
 */
inline torch::Tensor get_projection_jacobian(const torch::Tensor& K,
                                             const torch::Tensor& means_camera) {
    auto fx = K[0][0];
    auto fy = K[1][1];

    auto X = means_camera.slice(1, 0, 1);
    auto Y = means_camera.slice(1, 1, 2);
    auto Z = means_camera.slice(1, 2, 3);
    auto Z2 = Z * Z;

    auto J = torch::zeros({means_camera.size(0), 2, 3}, means_camera.options());
    J.slice(1, 0, 1).slice(2, 0, 1) = fx / Z;
    J.slice(1, 0, 1).slice(2, 2, 3) = -fx * X / Z2;
    J.slice(1, 1, 2).slice(2, 1, 2) = fy / Z;
    J.slice(1, 1, 2).slice(2, 2, 3) = -fy * Y / Z2;

    return J;
}

inline std::string dtype_str(const torch::Tensor& t) {
    return c10::toString(t.scalar_type());  // e.g. "Float", "Half", "Byte"
}

inline size_t logical_nbytes(const torch::Tensor& t) {
#if TORCH_VERSION_MAJOR >= 2
    return t.nbytes();  // counts numel() * element_size()
#else
    return static_cast<size_t>(t.numel()) * static_cast<size_t>(t.element_size());
#endif
}

// Bytes spanned by this view (covers strides, not allocator overhead).
inline size_t view_span_nbytes(const torch::Tensor& t) {
    if (t.numel() == 0)
        return 0;
    const auto sizes = t.sizes();
    const auto strides = t.strides();
    std::cout << "Sizes: " << sizes << std::endl;
    std::cout << "Strides: " << strides << std::endl;
    int64_t start = t.storage_offset();  // in elements
    int64_t end = start;                 // in elements
    for (int64_t i = 0; i < sizes.size(); ++i) {
        if (sizes[i] == 0)
            return 0;
    }
    // Max linear index touched by the view:
    for (int64_t i = 0; i < sizes.size(); ++i) {
        end += (sizes[i] - 1) * strides[i];
    }
    return static_cast<size_t>(end - start + 1) * static_cast<size_t>(t.element_size());
}

}  // namespace utils
}  // namespace gaussian_splatting

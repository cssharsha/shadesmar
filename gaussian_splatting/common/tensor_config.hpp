#pragma once

#include <torch/torch.h>

namespace gaussian_splatting {
namespace common {

constexpr bool USE_DOUBLE_PRECISION = false;

using scalar_t = std::conditional_t<USE_DOUBLE_PRECISION, double, float>;

inline constexpr torch::Dtype TENSOR_DTYPE =
    USE_DOUBLE_PRECISION ? torch::kFloat64 : torch::kFloat32;

inline torch::TensorOptions getTensorOptions() {
    return torch::TensorOptions().dtype(TENSOR_DTYPE);
}

inline torch::TensorOptions getTensorOptions(const torch::Device& device) {
    return torch::TensorOptions().dtype(TENSOR_DTYPE).device(device);
}

template <typename TensorLike>
inline scalar_t itemAs(const TensorLike& tensor) {
    if constexpr (USE_DOUBLE_PRECISION) {
        return tensor.template item<double>();
    } else {
        return tensor.template item<float>();
    }
}

template <typename TensorLike>
inline scalar_t* dataPtrAs(TensorLike& tensor) {
    if constexpr (USE_DOUBLE_PRECISION) {
        return tensor.template data_ptr<double>();
    } else {
        return tensor.template data_ptr<float>();
    }
}

template <typename TensorLike>
inline const scalar_t* dataPtrAs(const TensorLike& tensor) {
    if constexpr (USE_DOUBLE_PRECISION) {
        return tensor.template data_ptr<double>();
    } else {
        return tensor.template data_ptr<float>();
    }
}

template <size_t Dim, typename TensorLike>
inline auto accessorAs(TensorLike& tensor) {
    if constexpr (USE_DOUBLE_PRECISION) {
        return tensor.template accessor<double, Dim>();
    } else {
        return tensor.template accessor<float, Dim>();
    }
}

}  // namespace common
}  // namespace gaussian_splatting

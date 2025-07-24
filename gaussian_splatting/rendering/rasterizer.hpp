#pragma once

#include <Common.h>
#include <torch/torch.h>
#include "gaussian_splatting/training/gaussian_tensors.hpp"
#include "gaussian_splatting/training/keyframe_tensor.hpp"

namespace gaussian_splatting {
namespace rendering {

struct RasterizationOutput {
    bool success;
    // The rendered RGB image. Shape: [H, W, 3]
    torch::Tensor rendered_image;
    // The alpha channel of the rendered image. Shape: [H, W, 1]
    torch::Tensor alpha_channel;
    // The radii of the projected Gaussians. Shape: [N]
    torch::Tensor radii;
    torch::Tensor means2d;
    torch::Tensor depths;
    torch::Tensor visibility;
    float width, height;
};

class DifferentiableRasterizer {
public:
    DifferentiableRasterizer() = default;

    RasterizationOutput rasterize(const GaussianTensors& gaussians,
                                  const torch::Tensor& camera_pose,
                                  const torch::Tensor& camera_intrinsics, int image_width,
                                  int image_height);
    RasterizationOutput rasterize(GaussianTensors& gaussians,
                                  training::KeyframeTensor& keyframe_tensor);
};

enum class RasterizeStepStatus {
    FINISHED,
    INITIALIZED,
    NOT_INITIALIZED,
};

class ProjectGaussians : public torch::autograd::Function<ProjectGaussians> {
public:
    static struct Config {
        uint32_t image_width;
        uint32_t image_height;
        float eps2d;
        float near_plane;
        float far_plane;
        float radius_clip;
        bool calc_compensations;
        gsplat::CameraModelType camera_model;
        RasterizeStepStatus rasterize_step_status = RasterizeStepStatus::NOT_INITIALIZED;
    } config;

    static torch::autograd::tensor_list forward(torch::autograd::AutogradContext* ctx,
                                                torch::Tensor means, torch::Tensor rotations,
                                                torch::Tensor scales, torch::Tensor opacities,
                                                torch::Tensor camera_pose,
                                                torch::Tensor camera_intrinsics);

    static torch::autograd::tensor_list backward(torch::autograd::AutogradContext* ctx,
                                                 const torch::autograd::tensor_list& grad_outputs);
};

class SphericalHarmonics : public torch::autograd::Function<SphericalHarmonics> {
public:
    static torch::autograd::tensor_list forward(torch::autograd::AutogradContext* ctx,
                                                torch::Tensor sh_degree_tensor, torch::Tensor dirs,
                                                torch::Tensor coeffs);

    static torch::autograd::tensor_list backward(torch::autograd::AutogradContext* ctx,
                                                 torch::autograd::tensor_list grad_outputs);
};

class Rasterization : public torch::autograd::Function<Rasterization> {
public:
    static struct Config {
        uint32_t image_width;
        uint32_t image_height;
        int32_t tile_size;
        RasterizeStepStatus rasterize_step_status = RasterizeStepStatus::NOT_INITIALIZED;
    } config;
    static torch::autograd::tensor_list forward(torch::autograd::AutogradContext* ctx,
                                                torch::Tensor means2d, torch::Tensor conics,
                                                torch::Tensor colors, torch::Tensor opacities,
                                                torch::Tensor bg_color, torch::Tensor isect_offsets,
                                                torch::Tensor flatten_ids);

    static torch::autograd::tensor_list backward(torch::autograd::AutogradContext* ctx,
                                                 torch::autograd::tensor_list grad_outputs);
};

}  // namespace rendering
}  // namespace gaussian_splatting

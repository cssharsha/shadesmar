#include <torch/torch.h>

#include <Ops.h>  // mind the gsplat header!
#include <logging/logging.hpp>

#include "gaussian_splatting/gsplat/gsplat/cuda/include/Ops.h"
#include "gaussian_splatting/rendering/rasterizer.hpp"
#include "gaussian_splatting/utils/image_utils.hpp"
#include "gaussian_splatting/utils/torch_utils.hpp"

namespace gaussian_splatting {
namespace rendering {
ProjectGaussians::Config ProjectGaussians::config;
Rasterization::Config Rasterization::config;

RasterizationOutput DifferentiableRasterizer::rasterize(const GaussianTensors& gaussians,
                                                        const torch::Tensor& camera_pose,
                                                        const torch::Tensor& camera_intrinsics,
                                                        int image_width, int image_height) {
    std::cout << "Not implemented" << std::endl;
    return RasterizationOutput();
}

RasterizationOutput DifferentiableRasterizer::rasterize(GaussianTensors& gaussians,
                                                        training::KeyframeTensor& keyframe_tensor) {
    std::cout << "Training for keyframe: " << keyframe_tensor.getKeyframeId() << std::endl;

    // Do some transformations to opacities and scales before passing to rasterization
    auto opacities = gaussians.get_opacities();
    opacities = torch::sigmoid(opacities);

    auto scales = gaussians.get_scales();
    // scales = torch::exp(scales);
    //================= Step 1: Project Gaussians to 2D ==================
    ProjectGaussians::config.image_width = keyframe_tensor.getImageWidth();
    ProjectGaussians::config.image_height = keyframe_tensor.getImageHeight();
    ProjectGaussians::config.eps2d = 0.3f;
    ProjectGaussians::config.near_plane = 0.01f;
    ProjectGaussians::config.far_plane = 10000.f;
    ProjectGaussians::config.radius_clip = 0.0f;
    ProjectGaussians::config.calc_compensations = false;
    ProjectGaussians::config.camera_model = gsplat::CameraModelType::PINHOLE;
    ProjectGaussians::config.rasterize_step_status = RasterizeStepStatus::INITIALIZED;
    auto proj_results = ProjectGaussians::apply(
        gaussians.get_positions(), gaussians.get_rotations(), scales, opacities,
        keyframe_tensor.getCameraPose(), keyframe_tensor.getCameraIntrinsic());

    // return {radii, xys, depths, conics, compensations};
    auto radii = proj_results[0];
    auto xys = proj_results[1];
    auto depths = proj_results[2];
    auto conics = proj_results[3];
    auto compensations = proj_results[4];

    // TODO: Apply opacities based on compensations
    //=====================ProjectGaussians done=========================================

    auto xys_with_grad = xys.contiguous();
    xys_with_grad.requires_grad_(true);
    xys_with_grad.retain_grad();

    {
        torch::NoGradGuard no_grad;
        // Create image tensor [1, H, W] and mark pixels where Gaussians project to
        auto xys_image =
            torch::zeros({1, static_cast<int>(keyframe_tensor.getImageHeight()),
                          static_cast<int>(keyframe_tensor.getImageWidth())},
                         torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU));

        auto xys_cpu = xys.to(torch::kCPU);
        auto xys_accessor = xys_cpu.accessor<float, 3>();
        std::cout << "xys size: " << xys_cpu.sizes() << std::endl;

        for (int i = 0; i < xys_cpu.size(1); i++) {
            auto xy = xys_cpu[0][i];
            auto x = static_cast<size_t>(xy[0].item<float>());
            auto y = static_cast<size_t>(xy[1].item<float>());

            if (x >= 0 && x < keyframe_tensor.getImageWidth() && y >= 0 &&
                y < keyframe_tensor.getImageHeight()) {
                xys_image[0][y][x] = 1.0;
            }
        }

        auto num_pixels_set = (xys_image > 0).sum().item<int64_t>();
        std::cout << "Number of pixels set to 1: " << num_pixels_set << " out of " << xys.size(0)
                  << " Gaussians" << std::endl;

        utils::writeImageToDirectory(utils::tensorToMat(xys_image), "/data/south-building/debug/",
                                     std::to_string(keyframe_tensor.getKeyframeId()) + "_xys.png");
    }

    //================== Step 2: Do spherical harmonics ==================
    auto world_to_cam = torch::inverse(keyframe_tensor.getCameraPose());
    std::cout << "World to cam: " << world_to_cam << std::endl;

    auto world_to_cam_T = world_to_cam.index(
        {torch::indexing::Slice(), torch::indexing::Slice(torch::indexing::None, 3), 3});
    std::cout << "World to cam T: " << world_to_cam_T << std::endl;

    // Add batch dimension to dirs to match shs shape
    auto dirs = (gaussians.get_positions() - world_to_cam_T).unsqueeze(0);
    std::cout << "Dirs[0][0]: " << dirs[0][0] << std::endl;
    std::cout << "Dirs shape: " << dirs.sizes() << std::endl;

    // Create masks based on radii
    // std::cout << "Radii: " << radii << std::endl;
    // auto masks = (radii > 0).all(-1);

    // For now, just use DC coefficient (sh_0) directly since higher order terms are zero
    // sh_0 is stored as: (rgb - 0.5) / sqrt(4*pi)
    // Reconstruct: rgb = sh_0 * sqrt(4*pi) + 0.5
    // auto sh_0 = gaussians.get_sh_0();  // Shape: [N, 1, 3]
    // auto colors = sh_0.squeeze(1) * std::sqrt(M_PI * 4) + 0.5f;
    // colors = torch::clamp(colors, 0.0f, 1.0f);

    // TODO: When you add higher-order SH terms, uncomment this:
    auto sh_degree_tensor =
        torch::tensor({gaussians.get_sh_degree()},
                      torch::TensorOptions().dtype(torch::kInt32).device(dirs.device()));
    std::cout << "SH degree tensor: " << sh_degree_tensor << std::endl;
    auto shs = gaussians.get_sh_coefficients().unsqueeze(0);
    auto colors = SphericalHarmonics::apply(sh_degree_tensor, dirs, shs)[0];
    colors = torch::clamp_min(colors + 0.5f, 0.0f);
    std::cout << "Colors shape: " << colors.sizes() << std::endl;
    std::cout << "Colors stats: Min=" << colors.min().item<float>()
              << ", Max=" << colors.max().item<float>() << ", Mean=" << colors.mean().item<float>()
              << std::endl;

    // Visualize colors by rendering them at their projected 2D positions
    {
        torch::NoGradGuard no_grad;
        // Create RGB image tensor [1, H, W, 3] to accumulate colors
        auto color_image =
            torch::zeros({1, static_cast<int>(keyframe_tensor.getImageHeight()),
                          static_cast<int>(keyframe_tensor.getImageWidth()), 3},
                         torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU));

        // Create count image to track how many gaussians project to each pixel
        auto count_image =
            torch::zeros({1, static_cast<int>(keyframe_tensor.getImageHeight()),
                          static_cast<int>(keyframe_tensor.getImageWidth())},
                         torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU));

        auto xys_cpu = xys.to(torch::kCPU);
        // auto colors_cpu = colors.to(torch::kCPU).unsqueeze(0);
        auto colors_cpu = colors.to(torch::kCPU);
        // auto colors_cpu = gaussians.get_colors().to(torch::kCPU).unsqueeze(0);
        auto opacities_cpu = opacities.to(torch::kCPU);

        std::cout << "xys_cpu size: " << xys_cpu.sizes() << std::endl;
        std::cout << "colors_cpu size: " << colors_cpu.sizes() << std::endl;
        std::cout << "opacities_cpu size: " << opacities_cpu.sizes() << std::endl;

        // Simple per-pixel color assignment
        for (int i = 0; i < xys_cpu.size(1); i++) {
            auto xy = xys_cpu[0][i];
            int x = static_cast<int>(xy[0].item<float>());
            int y = static_cast<int>(xy[1].item<float>());

            if (x >= 0 && x < static_cast<int>(keyframe_tensor.getImageWidth()) && y >= 0 &&
                y < static_cast<int>(keyframe_tensor.getImageHeight())) {
                // Get color for this Gaussian (colors_cpu is [1, N, 3], so index [0][i])
                float r = colors_cpu[0][i][0].item<float>();
                float g = colors_cpu[0][i][1].item<float>();
                float b = colors_cpu[0][i][2].item<float>();
                float alpha = 1.0f;

                // Accumulate color at this pixel
                color_image[0][y][x][0] = color_image[0][y][x][0].item<float>() + r * alpha;
                color_image[0][y][x][1] = color_image[0][y][x][1].item<float>() + g * alpha;
                color_image[0][y][x][2] = color_image[0][y][x][2].item<float>() + b * alpha;
                count_image[0][y][x] = count_image[0][y][x].item<float>() + alpha;
            }
        }

        // Debug: Print stats BEFORE normalization
        auto color_before_norm = color_image.clone();
        std::cout << "BEFORE normalization - Color stats: Min="
                  << color_before_norm.min().item<float>()
                  << ", Max=" << color_before_norm.max().item<float>()
                  << ", Mean=" << color_before_norm.mean().item<float>() << std::endl;
        std::cout << "Count image stats: Min=" << count_image.min().item<float>()
                  << ", Max=" << count_image.max().item<float>()
                  << ", Mean=" << count_image.mean().item<float>() << std::endl;

        // Normalize by count to get average color per pixel
        auto mask = count_image > 0;
        color_image.index_put_(
            {mask.unsqueeze(-1).expand_as(color_image)},
            color_image.index({mask.unsqueeze(-1).expand_as(color_image)}) /
                count_image.index({mask}).unsqueeze(-1).expand({-1, 3}).reshape({-1}));

        // Debug: Print stats AFTER normalization
        std::cout << "AFTER normalization - Color stats: Min=" << color_image.min().item<float>()
                  << ", Max=" << color_image.max().item<float>()
                  << ", Mean=" << color_image.mean().item<float>() << std::endl;

        // Clamp colors to [0, 1] range
        color_image = torch::clamp(color_image, 0.0f, 1.0f);

        auto num_colored_pixels = (count_image > 0).sum().item<int64_t>();
        std::cout << "Number of pixels with color: " << num_colored_pixels << std::endl;
        std::cout << "AFTER clamping - Color stats: Min=" << color_image.min().item<float>()
                  << ", Max=" << color_image.max().item<float>()
                  << ", Mean=" << color_image.mean().item<float>() << std::endl;

        // Save the raw normalized image
        auto color_image_raw = color_image.squeeze(0);
        utils::writeImageToDirectory(
            utils::tensorToMat(color_image_raw, false), "/data/south-building/debug/",
            std::to_string(keyframe_tensor.getKeyframeId()) + "_sh_colors_raw.png");

        // Create a brightness-boosted version for better visualization
        // Apply gamma correction (gamma=0.5) to brighten mid-tones
        auto color_image_boosted = torch::pow(color_image, 0.5f);

        // Also create a linear brightness boost
        auto color_image_bright = color_image * 3.0f;
        color_image_bright = torch::clamp(color_image_bright, 0.0f, 1.0f);

        std::cout << "Boosted image stats: Min=" << color_image_boosted.min().item<float>()
                  << ", Max=" << color_image_boosted.max().item<float>()
                  << ", Mean=" << color_image_boosted.mean().item<float>() << std::endl;

        auto color_image_boosted_squeezed = color_image_boosted.squeeze(0);
        auto color_image_bright_squeezed = color_image_bright.squeeze(0);

        utils::writeImageToDirectory(
            utils::tensorToMat(color_image_boosted_squeezed, false), "/data/south-building/debug/",
            std::to_string(keyframe_tensor.getKeyframeId()) + "_sh_colors_boosted.png");

        utils::writeImageToDirectory(
            utils::tensorToMat(color_image_bright_squeezed, false), "/data/south-building/debug/",
            std::to_string(keyframe_tensor.getKeyframeId()) + "_sh_colors_bright.png");
    }
    //=====================SphericalHarmonics done=========================================

    //================= Step 3: Intersect Gaussians with tiles ==================
    int32_t tile_size = 16;
    const int tile_width = (keyframe_tensor.getImageWidth() + tile_size - 1) / tile_size;
    const int tile_height = (keyframe_tensor.getImageHeight() + tile_size - 1) / tile_size;

    // const auto isect_results = gsplat::intersect_tile(xys_with_grad, radii, depths, {}, {}, 1,
    //                                                   tile_size, tile_width, tile_height, true);
    auto intersections =
        gsplat::intersect_tile(xys_with_grad, radii, depths, {}, {}, xys_with_grad.size(0),
                               tile_size, tile_width, tile_height, true, false);

    const auto tiles_per_gauss = std::get<0>(intersections);
    const auto isect_ids = std::get<1>(intersections);
    const auto flatten_ids = std::get<2>(intersections);
    // std::cout << "tiles_per_gauss: " << tiles_per_gauss << std::endl;

    auto isect_offsets = gsplat::intersect_offset(isect_ids, 1, tile_width, tile_height);
    isect_offsets = isect_offsets.reshape({1, tile_height, tile_width});
    // std::cout << "isect_offsets: " << isect_offsets << std::endl;

    {
        torch::NoGradGuard no_grad;
        auto tile_image =
            torch::zeros({1, tile_height, tile_width},
                         torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU));

        auto isect_offsets_cpu = isect_offsets.to(torch::kCPU);
        auto isect_offsets_accessor = isect_offsets_cpu.accessor<int32_t, 3>();

        for (int i = 0; i < tile_height; i++) {
            for (int j = 0; j < tile_width; j++) {
                int start_idx = isect_offsets_accessor[0][i][j];
                int end_idx;
                if (i == tile_height - 1 && j == tile_width - 1) {
                    end_idx = flatten_ids.size(0);
                } else if (j == tile_width - 1) {
                    end_idx = isect_offsets_accessor[0][i + 1][0];
                } else {
                    end_idx = isect_offsets_accessor[0][i][j + 1];
                }
                int count = end_idx - start_idx;
                tile_image[0][i][j] = std::min(count / 10.0f, 1.0f);  // Normalize for visualization
            }
        }

        auto num_tiles_with_gaussians = (tile_image > 0).sum().item<int64_t>();
        std::cout << "Number of tiles with Gaussians: " << num_tiles_with_gaussians << " out of "
                  << (tile_width * tile_height) << " total tiles" << std::endl;

        utils::writeImageToDirectory(
            utils::tensorToMat(tile_image), "/data/south-building/debug/",
            std::to_string(keyframe_tensor.getKeyframeId()) + "_tile_intersections.png");
    }
    //=====================IntersectGaussians done=========================================

    //=== Step 3: Rasterize the 2D Gaussians ===
    std::cout << "opacities: " << opacities.is_cuda()
              << " compensations: " << compensations.is_cuda() << std::endl;
    // No idea how compensations is not on GPU
    // compensations.to();
    std::cout << "After send opacities: " << opacities.is_cuda()
              << " compensations: " << compensations.is_cuda() << std::endl;
    // auto compensated_opacities = gaussians.get_opacities().unsqueeze(0) * compensations;
    Rasterization::config.image_width = keyframe_tensor.getImageWidth();
    Rasterization::config.image_height = keyframe_tensor.getImageHeight();
    Rasterization::config.tile_size = tile_size;
    Rasterization::config.rasterize_step_status = RasterizeStepStatus::INITIALIZED;
    std::cout << "Set all the sconfigs" << std::endl;

    auto final_bg = at::empty({0}, colors.options().dtype(torch::kFloat32));
    std::cout << "Print sizes of all inputs to rasterization" << std::endl;
    std::cout << "xys_with_grad: " << xys_with_grad.sizes() << std::endl;
    std::cout << "conics: " << conics.sizes() << std::endl;
    std::cout << "colors: " << colors.sizes() << std::endl;
    std::cout << "opacities: " << opacities.sizes() << std::endl;
    std::cout << "final_bg: " << final_bg.sizes() << std::endl;
    std::cout << "isect_offsets: " << isect_offsets.sizes() << std::endl;
    std::cout << "flatten_ids: " << flatten_ids.sizes() << std::endl;

    auto raster_outputs = Rasterization::apply(xys_with_grad, conics, colors, opacities, final_bg,
                                               isect_offsets, flatten_ids);

    auto rendered_image = raster_outputs[0];
    auto rendered_alpha = raster_outputs[1];
    //=== Rasterize to 2D done ===

    {
        torch::NoGradGuard no_grad;
        std::cout << "Rendered image shape: " << rendered_image.sizes() << std::endl;
        std::cout << "Alpha channel shape: " << rendered_alpha.sizes() << std::endl;

        utils::writeImageToDirectory(
            utils::tensorToMat(rendered_image.to(torch::kCPU)[0], false),
            "/data/south-building/debug/",
            std::to_string(keyframe_tensor.getKeyframeId()) + "_rendered_image.png");

        utils::writeImageToDirectory(
            utils::tensorToMat(rendered_alpha.to(torch::kCPU)[0], false),
            "/data/south-building/debug/",
            std::to_string(keyframe_tensor.getKeyframeId()) + "_rendered_alpha.png");

        auto alpha_mean = rendered_alpha.mean().item<float>();
        auto alpha_max = rendered_alpha.max().item<float>();
        auto covered_pixels = (rendered_alpha > 0.1).sum().item<int64_t>();
        std::cout << "Alpha stats - Mean: " << alpha_mean << ", Max: " << alpha_max
                  << ", Covered pixels (alpha>0.1): " << covered_pixels << std::endl;
    }

    RasterizationOutput out;
    out.success = true;
    out.width = keyframe_tensor.getImageWidth();
    out.height = keyframe_tensor.getImageHeight();
    out.rendered_image = rendered_image;
    out.alpha_channel = rendered_alpha;
    out.means2d = xys_with_grad;
    out.radii = std::get<0>(radii.squeeze(0).max(-1));
    out.visibility = out.radii > 0;
    return out;
}

torch::autograd::tensor_list ProjectGaussians::forward(torch::autograd::AutogradContext* ctx,
                                                       torch::Tensor xyzs, torch::Tensor rotations,
                                                       torch::Tensor scales,
                                                       torch::Tensor opacities,
                                                       torch::Tensor camera_pose,
                                                       torch::Tensor camera_intrinsics) {
    if (config.rasterize_step_status != RasterizeStepStatus::INITIALIZED) {
        std::cerr << "ProjectGaussians config not initialized" << std::endl;
        throw std::runtime_error("ProjectGaussians config not initialized");
    }
    std::cout << "Printing all dimensions: " << std::endl;
    std::cout << "means: " << xyzs.dim() << std::endl;
    std::cout << "rotations: " << rotations.dim() << std::endl;
    std::cout << "scales: " << scales.dim() << std::endl;
    std::cout << "opacities: " << opacities.dim() << std::endl;
    std::cout << "camera_pose: " << camera_pose.dim() << std::endl;
    std::cout << "camera_intrinsics: " << camera_intrinsics.dim() << std::endl;

    // Make all torch tensors contiguous
    xyzs = xyzs.contiguous();
    rotations = rotations.contiguous();
    scales = scales.contiguous();
    opacities = opacities.contiguous();
    camera_pose = camera_pose.contiguous();
    camera_intrinsics = camera_intrinsics.contiguous();

    // Project 3D Gaussians to 2D
    auto proj_results = gsplat::projection_ewa_3dgs_fused_fwd(
        xyzs, {}, rotations, scales, {} /*opacities*/, camera_pose, camera_intrinsics,
        config.image_width, config.image_height, config.eps2d, config.near_plane, config.far_plane,
        config.radius_clip, config.calc_compensations, config.camera_model);

    auto radii = std::get<0>(proj_results).contiguous();
    auto xys = std::get<1>(proj_results).contiguous();
    auto depths = std::get<2>(proj_results).contiguous();
    auto conics = std::get<3>(proj_results).contiguous();
    auto compensations = std::get<4>(proj_results).contiguous();

    // {
    //     torch::NoGradGuard no_grad;
    //     std::cout << "Sizes: " << xyzs.sizes() << " " << xys.sizes() << std::endl;
    //     std::cout << "Proj example:\nxyz: " << xyzs[0] << "\txy: " << xys[0][0]
    //               << "\ncamera pose: " << camera_pose[0]
    //               << "\ncamera intrinsics: " << camera_intrinsics[0] << "\nradii: " << radii[0]
    //               << std::endl;
    //     xys = xys.to(torch::kCUDA);
    // }

    if (!compensations.defined()) {
        compensations = at::empty({0});
    }
    ctx->save_for_backward({xyzs, rotations, scales, opacities, camera_pose, camera_intrinsics,
                            radii, conics, compensations});

    return {radii, xys, depths, conics, compensations};
}

torch::autograd::tensor_list ProjectGaussians::backward(
    torch::autograd::AutogradContext* ctx, const torch::autograd::tensor_list& grad_outputs) {
    auto saved = ctx->get_saved_variables();
    auto xyzs = saved[0];
    auto rotations = saved[1];
    auto scales = saved[2];
    auto opacities = saved[3];
    auto camera_pose = saved[4];
    auto camera_intrinsics = saved[5];
    auto radii = saved[6];
    auto conics = saved[7];
    auto compensations = saved[8];

    auto grad_radii = grad_outputs[0];
    auto grad_xys = grad_outputs[1];
    auto grad_depths = grad_outputs[2];
    auto grad_conics = grad_outputs[3];
    auto grad_compensations = grad_outputs[4];

    c10::optional<at::Tensor> grad_compensations_opt;
    if (compensations.defined() && compensations.numel() > 0) {
        grad_compensations_opt = grad_compensations.to(torch::kCUDA).contiguous();
    }
    c10::optional<at::Tensor> compensations_opt;
    if (compensations.defined() && compensations.numel() > 0) {
        compensations_opt = compensations;
    }

    auto proj_grads = gsplat::projection_ewa_3dgs_fused_bwd(
        xyzs, {}, rotations, scales, camera_pose, camera_intrinsics, config.image_width,
        config.image_height, config.eps2d, config.camera_model, radii, conics, compensations_opt,
        grad_xys, grad_depths, grad_conics, grad_compensations_opt, ctx->needs_input_grad(4));

    auto grad_xyzs = std::get<0>(proj_grads).contiguous();
    auto grad_rotations = std::get<2>(proj_grads).contiguous();
    auto grad_scales = std::get<3>(proj_grads).contiguous();
    // auto grad_camera_pose = std::get<4>(proj_grads).contiguous();

    torch::Tensor grad_opacities;
    if (opacities.defined() && grad_compensations_opt.has_value() &&
        compensations_opt.has_value()) {
        grad_opacities =
            (grad_compensations_opt.value() * compensations_opt.value() / opacities.unsqueeze(0))
                .sum(0);
    }
    return {grad_xyzs,       grad_rotations,  grad_scales,    grad_opacities,
            torch::Tensor(), torch::Tensor(), torch::Tensor()};
}

// SphericalHarmonicsFunction implementation
torch::autograd::tensor_list SphericalHarmonics::forward(torch::autograd::AutogradContext* ctx,
                                                         torch::Tensor sh_degree_tensor,
                                                         torch::Tensor dirs, torch::Tensor coeffs) {
    const int sh_degree = sh_degree_tensor.item<int>();
    const int num_sh_coeffs = (sh_degree + 1) * (sh_degree + 1);

    // Ensure tensors are contiguous
    dirs = dirs.contiguous();
    coeffs = coeffs.contiguous();
    torch::Tensor masks =
        torch::ones(dirs.sizes(), torch::TensorOptions().dtype(torch::kBool).device(dirs.device()));

    // Flatten batch dimensions for CUDA kernel
    auto dirs_flat = dirs.reshape({-1, 3});
    auto coeffs_flat = coeffs.reshape({-1, coeffs.size(-2), 3});
    auto masks_flat = masks.reshape({-1});

    // Call spherical harmonics forward - pass FULL coeffs!
    auto colors = gsplat::spherical_harmonics_fwd(sh_degree, dirs_flat, coeffs_flat, masks_flat);

    auto output_shape = dirs.sizes().vec();
    // output_shape[output_shape.size() - 1] = 3;  // Ensure last dimension is 3
    colors = colors.reshape(output_shape).contiguous();

    TORCH_CHECK(colors.is_cuda(), "colors must be on CUDA after SH computation");

    // Save for backward - save everything as-is
    ctx->save_for_backward({dirs, coeffs, masks});
    ctx->saved_data["sh_degree"] = sh_degree;
    ctx->saved_data["num_bases"] = coeffs.size(-2);  // Save the full K dimension

    return {colors};
}

torch::autograd::tensor_list SphericalHarmonics::backward(
    torch::autograd::AutogradContext* ctx, torch::autograd::tensor_list grad_outputs) {
    auto grad_colors = grad_outputs[0].contiguous();

    auto saved = ctx->get_saved_variables();
    const auto& dirs = saved[0];
    const auto& coeffs = saved[1];
    const auto& masks = saved[2];

    const int sh_degree = ctx->saved_data["sh_degree"].to<int>();
    const int num_bases = ctx->saved_data["num_bases"].to<int>();

    // CUDA kernel expects flattened tensors
    auto dirs_flat = dirs.reshape({-1, 3});
    auto coeffs_flat = coeffs.reshape({-1, num_bases, 3});
    auto masks_flat = masks.reshape({-1});
    auto grad_colors_flat = grad_colors.reshape({-1, 3});

    bool compute_grad_dirs = ctx->needs_input_grad(1);

    auto sh_grads =
        gsplat::spherical_harmonics_bwd(num_bases, sh_degree, dirs_flat, coeffs_flat, masks_flat,
                                        grad_colors_flat, compute_grad_dirs);

    auto grad_coeffs = std::get<0>(sh_grads);
    auto grad_dirs = std::get<1>(sh_grads);

    if (grad_dirs.defined()) {
        grad_dirs = grad_dirs.reshape(dirs.sizes());
    }
    if (grad_coeffs.defined()) {
        grad_coeffs = grad_coeffs.reshape(coeffs.sizes());
    }

    if (!ctx->needs_input_grad(1)) {
        grad_dirs = torch::Tensor();
    }
    if (!ctx->needs_input_grad(2)) {
        grad_coeffs = torch::Tensor();
    }

    // Return gradients in same order as inputs: sh_degree_tensor, dirs, coeffs, masks
    return {torch::Tensor(), grad_dirs, grad_coeffs, torch::Tensor()};
}

torch::autograd::tensor_list Rasterization::forward(torch::autograd::AutogradContext* ctx,
                                                    torch::Tensor means2d, torch::Tensor conics,
                                                    torch::Tensor colors, torch::Tensor opacities,
                                                    torch::Tensor bg_color,
                                                    torch::Tensor isect_offsets,
                                                    torch::Tensor flatten_ids) {
    // Ensure tensors are contiguous
    std::cout << "Calling forward" << std::endl;
    means2d = means2d.contiguous();
    conics = conics.contiguous();
    colors = colors.contiguous();
    opacities = opacities.contiguous();
    isect_offsets = isect_offsets.contiguous();
    flatten_ids = flatten_ids.contiguous();

    std::cout << "means2d: " << means2d.is_cuda() << " conics: " << conics.is_cuda()
              << " colors: " << colors.is_cuda() << " opacities: " << opacities.is_cuda()
              << " bg_color: " << bg_color.is_cuda()
              << " isect_offsets: " << isect_offsets.is_cuda()
              << " flatten_ids: " << flatten_ids.is_cuda() << std::endl;

    // Convert empty tensor to optional for CUDA function
    at::optional<at::Tensor> bg_color_opt;
    if (bg_color.defined() && bg_color.numel() > 0) {
        bg_color_opt = bg_color;
    }

    // Call rasterization with optional background
    // gsplat::rasterize_to_pixels_3dgs_fwd(
    //     const at::Tensor means2d, const at::Tensor conics, const at::Tensor colors,
    //     const at::Tensor opacities, const at::optional<at::Tensor> backgrounds,
    //     const at::optional<at::Tensor> masks, const uint32_t image_width,
    //     const uint32_t image_height, const uint32_t tile_size, const at::Tensor tile_offsets,
    //     const at::Tensor flatten_ids)
    auto raster_results = gsplat::rasterize_to_pixels_3dgs_fwd(
        means2d, conics, colors, opacities, {}, {}, config.image_width, config.image_height,
        config.tile_size, isect_offsets, flatten_ids);

    auto rendered_image = std::get<0>(raster_results).contiguous();
    auto rendered_alpha = std::get<1>(raster_results).to(torch::kFloat32).contiguous();
    auto last_ids = std::get<2>(raster_results).contiguous();

    // Save for backward
    ctx->save_for_backward({means2d, conics, colors, opacities, bg_color, isect_offsets,
                            flatten_ids, rendered_alpha, last_ids});

    return {rendered_image, rendered_alpha, last_ids};
}

torch::autograd::tensor_list Rasterization::backward(torch::autograd::AutogradContext* ctx,
                                                     torch::autograd::tensor_list grad_outputs) {
    std::cout << "Doing rasterization backward" << std::endl;
    auto grad_image = grad_outputs[0].contiguous();
    auto grad_alpha = grad_outputs[1].contiguous();

    auto saved = ctx->get_saved_variables();
    const auto& means2d = saved[0];
    const auto& conics = saved[1];
    const auto& colors = saved[2];
    const auto& opacities = saved[3];
    const auto& bg_color = saved[4];
    const auto& isect_offsets = saved[5];
    const auto& flatten_ids = saved[6];
    const auto& rendered_alpha = saved[7];
    const auto& last_ids = saved[8];
    std::cout << "Fetched all the saved variables" << means2d.sizes() << " " << conics.sizes()
              << " " << colors.sizes() << " " << opacities.sizes() << " " << bg_color.sizes() << " "
              << isect_offsets.sizes() << " " << flatten_ids.sizes() << " "
              << rendered_alpha.sizes() << " " << last_ids.sizes() << std::endl;

    // Extract settings
    const auto width = config.image_width;
    const auto height = config.image_height;
    const auto tile_size = config.tile_size;

    at::optional<at::Tensor> bg_color_opt;
    if (bg_color.defined() && bg_color.numel() > 0) {
        bg_color_opt = bg_color;
    }

    // Call backward
    auto raster_grads = gsplat::rasterize_to_pixels_3dgs_bwd(
        means2d, conics, colors, opacities, bg_color_opt,
        {},  // bg_color_opt might not have value, masks is empty optional
        width, height, tile_size, isect_offsets, flatten_ids, rendered_alpha, last_ids, grad_image,
        grad_alpha,
        false);  // absgrad

    auto grad_means2d_abs = std::get<0>(raster_grads);
    auto grad_means2d = std::get<1>(raster_grads).contiguous();
    auto grad_conics = std::get<2>(raster_grads).contiguous();
    auto grad_colors = std::get<3>(raster_grads).contiguous();
    auto grad_opacities = std::get<4>(raster_grads).contiguous();

    std::cout << "Finished calling the rasterization backward kernel" << std::endl;

    // Background gradient - only compute if bg_color was not empty and needs gradient
    torch::Tensor grad_bg_color;
    if (ctx->needs_input_grad(4) && bg_color.defined() && bg_color.numel() > 0) {
        auto one_minus_alpha = 1.0f - rendered_alpha;
        grad_bg_color = (grad_image * one_minus_alpha).sum({1, 2});
    } else {
        grad_bg_color = torch::Tensor();
    }

    // Check gradient requirements for other inputs
    if (!ctx->needs_input_grad(0)) {
        grad_means2d = torch::Tensor();
    }
    if (!ctx->needs_input_grad(1)) {
        grad_conics = torch::Tensor();
    }
    if (!ctx->needs_input_grad(2)) {
        grad_colors = torch::Tensor();
    }
    if (!ctx->needs_input_grad(3)) {
        grad_opacities = torch::Tensor();
    }

    std::cout << "Finished all the stuff in the backward" << std::endl;

    return {grad_means2d,  grad_conics,     grad_colors,     grad_opacities,
            grad_bg_color, torch::Tensor(), torch::Tensor(), torch::Tensor()};
}
}  // namespace rendering
}  // namespace gaussian_splatting

#include <gtest/gtest.h>
#include <torch/torch.h>
#include <vector>
#include "gaussian_splatting/rendering/rasterizer.hpp"
#include "gaussian_splatting/training/gaussian_tensors.hpp"

class ProjectGaussiansTest : public ::testing::Test {
protected:
    void SetUp() override {
        camera_pose = torch::tensor({{0.4943, -0.2330, -0.8375, 3.5378},
                                     {0.2814, 0.9544, -0.0995, -0.3731},
                                     {0.8225, -0.1865, 0.5373, -1.1434},
                                     {0.0000, 0.0000, 0.0000, 1.0000}},
                                    torch::kFloat32);
        camera_intrinsics = torch::tensor({{2563.9885, 0.0000, 1536.0000},
                                           {0.0000, 2563.9885, 1152.0000},
                                           {0.0000, 0.0000, 1.0000}},
                                          torch::kFloat32);
        xyzs = torch::tensor({-0.8419, -1.8025, -0.0613}, torch::kFloat32);
        colors = torch::tensor({141, 131, 122}, torch::kFloat32);
        opacities = torch::tensor({0.5}, torch::kFloat32);
        scales = torch::tensor({-1.8051, -1.8051, -1.8051}, torch::kFloat32);
        rotations = torch::tensor({0, 0, 0, 1}, torch::kFloat32);
        sh_coefficients = torch::tensor({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, torch::kFloat32);

        xyzs = xyzs.to(torch::kCUDA);
        rotations = rotations.to(torch::kCUDA);
        scales = scales.to(torch::kCUDA);
        opacities = opacities.to(torch::kCUDA);
        colors = colors.to(torch::kCUDA);
        sh_coefficients = sh_coefficients.to(torch::kCUDA);
        camera_pose = camera_pose.to(torch::kCUDA);
        camera_intrinsics = camera_intrinsics.to(torch::kCUDA);
        gaussian_splatting::rendering::ProjectGaussians::config.image_width = 1080;
        gaussian_splatting::rendering::ProjectGaussians::config.image_height = 720;
        gaussian_splatting::rendering::ProjectGaussians::config.eps2d = 0.3f;
        gaussian_splatting::rendering::ProjectGaussians::config.near_plane = 0.01f;
        gaussian_splatting::rendering::ProjectGaussians::config.far_plane = 10000.f;
        gaussian_splatting::rendering::ProjectGaussians::config.radius_clip = 0.0f;
        gaussian_splatting::rendering::ProjectGaussians::config.calc_compensations = false;
        gaussian_splatting::rendering::ProjectGaussians::config.camera_model =
            gsplat::CameraModelType::PINHOLE;
        gaussian_splatting::rendering::ProjectGaussians::config.rasterize_step_status =
            gaussian_splatting::rendering::RasterizeStepStatus::INITIALIZED;
    }
    torch::Tensor camera_pose;
    torch::Tensor camera_intrinsics;
    torch::Tensor xyzs;
    torch::Tensor rotations;
    torch::Tensor scales;
    torch::Tensor opacities;
    torch::Tensor colors;
    torch::Tensor sh_coefficients;
};
torch::Tensor projectPoint(const torch::Tensor& point_3d, const torch::Tensor& camera_pose_4x4,
                           const torch::Tensor& intrinsics_3x3) {
    torch::Tensor point_homo;
    if (point_3d.size(0) == 3) {
        point_homo = torch::cat({point_3d, torch::ones({1}, point_3d.options())}, 0);
    } else {
        point_homo = point_3d;
    }

    torch::Tensor point_cam_homo = torch::matmul(camera_pose_4x4, point_homo);
    torch::Tensor point_cam = point_cam_homo.slice(0, 0, 3);

    torch::Tensor point_image_homo = torch::matmul(intrinsics_3x3, point_cam);

    torch::Tensor z = point_image_homo[2];
    torch::Tensor pixel_coords = point_image_homo.slice(0, 0, 2) / z;

    return pixel_coords;
}

TEST_F(ProjectGaussiansTest, TestProjectGaussians) {
    // auto proj_results = gaussian_splatting::rendering::ProjectGaussians::apply(
    //     xyzs, rotations, scales, opacities, camera_pose, camera_intrinsics);
    // auto xys = proj_results[0].contiguous();
    // Project xyz to xy using the camera model
    auto xy = projectPoint(xyzs, camera_pose, camera_intrinsics);
    std::cout << "xy: " << xy << std::endl;
}

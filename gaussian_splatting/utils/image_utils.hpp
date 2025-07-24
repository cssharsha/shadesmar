#pragma once

#include <opencv2/opencv.hpp>
#include <torch/torch.h>
#include <string>

namespace gaussian_splatting {
namespace utils {

cv::Mat tensorToMat(const torch::Tensor& tensor, bool is_chw = true);

cv::Mat combineImagesHorizontally(const cv::Mat& img1, const cv::Mat& img2);

bool writeImageToDirectory(const cv::Mat& image, const std::string& directory,
                           const std::string& filename);

void plotProjectedPoints(const torch::Tensor& points, const std::string& directory,
                           const std::string& filename, int image_width, int image_height);

}  // namespace utils
}  // namespace gaussian_splatting

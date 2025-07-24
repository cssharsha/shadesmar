#include <filesystem>
#include <logging/logging.hpp>
#include <opencv2/opencv.hpp>
#include "image_utils.hpp"

namespace gaussian_splatting {
namespace utils {

cv::Mat tensorToMat(const torch::Tensor& tensor, bool is_chw) {
    // Move tensor to CPU if it's on GPU
    torch::Tensor cpu_tensor = tensor.to(torch::kCPU);

    // Handle different input formats
    torch::Tensor img_tensor;

    if (cpu_tensor.dim() == 4) {
        // Batch dimension, take first image [B, C, H, W] or [B, H, W, C]
        img_tensor = cpu_tensor[0];
    } else if (cpu_tensor.dim() == 3) {
        // [C, H, W] or [H, W, C]
        img_tensor = cpu_tensor;
    } else {
        LOG(ERROR) << "Unsupported tensor dimensions: " << cpu_tensor.dim();
        return cv::Mat();
    }

    // Convert to float if not already
    if (img_tensor.dtype() != torch::kFloat32) {
        img_tensor = img_tensor.to(torch::kFloat32);
    }

    // Convert CHW to HWC if needed
    if (is_chw && img_tensor.dim() == 3) {
        img_tensor = img_tensor.permute({1, 2, 0});  // [C, H, W] -> [H, W, C]
    }

    // Clamp values to [0, 1] and convert to [0, 255]
    img_tensor = torch::clamp(img_tensor, 0.0, 1.0) * 255.0;
    img_tensor = img_tensor.to(torch::kUInt8);

    // Get tensor data
    int height = img_tensor.size(0);
    int width = img_tensor.size(1);
    int channels = img_tensor.dim() == 3 ? img_tensor.size(2) : 1;

    // Create cv::Mat
    cv::Mat cv_image;
    if (channels == 3) {
        cv_image = cv::Mat(height, width, CV_8UC3, img_tensor.data_ptr<uint8_t>()).clone();
        // Convert RGB to BGR for OpenCV
        cv::cvtColor(cv_image, cv_image, cv::COLOR_RGB2BGR);
    } else if (channels == 1) {
        cv_image = cv::Mat(height, width, CV_8UC1, img_tensor.data_ptr<uint8_t>()).clone();
    } else {
        LOG(ERROR) << "Unsupported number of channels: " << channels;
        return cv::Mat();
    }

    return cv_image;
}

cv::Mat combineImagesHorizontally(const cv::Mat& img1, const cv::Mat& img2) {
    if (img1.empty() || img2.empty()) {
        LOG(ERROR) << "Cannot combine empty images";
        return cv::Mat();
    }

    // Get dimensions
    int height = std::max(img1.rows, img2.rows);
    int total_width = img1.cols + img2.cols;

    // Create output image
    cv::Mat combined(height, total_width, img1.type());

    // Resize images if heights don't match
    cv::Mat img1_resized = img1;
    cv::Mat img2_resized = img2;

    if (img1.rows != height) {
        cv::resize(img1, img1_resized, cv::Size(img1.cols, height));
    }
    if (img2.rows != height) {
        cv::resize(img2, img2_resized, cv::Size(img2.cols, height));
    }

    // Copy images to combined image
    cv::Rect roi1(0, 0, img1_resized.cols, img1_resized.rows);
    cv::Rect roi2(img1_resized.cols, 0, img2_resized.cols, img2_resized.rows);

    img1_resized.copyTo(combined(roi1));
    img2_resized.copyTo(combined(roi2));

    return combined;
}

bool writeImageToDirectory(const cv::Mat& image, const std::string& directory,
                           const std::string& filename) {
    if (image.empty()) {
        LOG(ERROR) << "Cannot write empty image";
        return false;
    }

    try {
        // Create directory if it doesn't exist
        std::filesystem::create_directories(directory);

        // Construct full path
        std::string full_path = std::filesystem::path(directory) / filename;

        // Write image
        bool success = cv::imwrite(full_path, image);

        if (!success) {
            LOG(ERROR) << "Failed to write image to: " << full_path;
            return false;
        }

        LOG(INFO) << "Successfully wrote image to: " << full_path;
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception while writing image: " << e.what();
        return false;
    }
}

void plotProjectedPoints(const torch::Tensor& points, const std::string& directory,
                           const std::string& filename, int image_width, int image_height) {
    torch::NoGradGuard no_grad;

    // Ensure tensor is on CPU
    torch::Tensor cpu_points = points.to(torch::kCPU);

    // Check tensor dimensions
    if (cpu_points.dim() != 3 || cpu_points.size(2) != 2) {
        LOG(ERROR) << "Unsupported tensor dimensions for plotting: " << cpu_points.sizes()
                   << ". Expected shape [M, N, 2].";
        return;
    }

    int num_images = cpu_points.size(0);
    auto points_accessor = cpu_points.accessor<float, 3>();

    for (int i = 0; i < num_images; ++i) {
        // Create a black image
        cv::Mat image = cv::Mat::zeros(image_height, image_width, CV_8UC3);

        int num_points = cpu_points.size(1);
        for (int j = 0; j < num_points; ++j) {
            // Get point coordinates
            float x = points_accessor[i][j][0];
            float y = points_accessor[i][j][1];

            // Draw a small circle for each point
            if (x >= 0 && x < image_width && y >= 0 && y < image_height) {
                cv::circle(image, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), -1); // Green dot
            }
        }

        // Construct filename with index
        std::filesystem::path path(filename);
        std::string new_filename = path.stem().string() + "_" + std::to_string(i) + path.extension().string();

        // Write image to file
        writeImageToDirectory(image, directory, new_filename);
    }
}

}  // namespace utils
}  // namespace gaussian_splatting
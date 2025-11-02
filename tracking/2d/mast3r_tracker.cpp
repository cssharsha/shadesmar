#include "2d/mast3r_tracker.hpp"

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "logging/logging.hpp"
#include "stf/transform_utils.hpp"

// JSON parsing - using nlohmann/json if available, otherwise manual parsing
#ifdef HAS_NLOHMANN_JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#else
// Simple JSON parsing fallback
#include <regex>
#endif

namespace tracking {
namespace image {

// Pimpl idiom for Python service management
struct MASt3RTracker::PythonServiceImpl {
    pid_t process_id = -1;
    int stdin_pipe[2] = {-1, -1};
    int stdout_pipe[2] = {-1, -1};
    int stderr_pipe[2] = {-1, -1};
    bool is_running = false;

    std::string python_executable;
    std::string script_path;
    std::string model_name;
    std::string device;
};

MASt3RTracker::MASt3RTracker(const std::string& model_path,
                             const std::string& python_executable,
                             const std::string& device, size_t max_points)
    : model_path_(model_path),
      python_executable_(python_executable),
      device_(device),
      max_points_(max_points),
      python_service_(std::make_unique<PythonServiceImpl>()) {
    LOG(INFO) << "Initializing MASt3R tracker with model: " << model_path_;
    LOG(INFO) << "Device: " << device_ << ", Max points: " << max_points_;

    python_service_->python_executable = python_executable;
    python_service_->model_name = model_path;
    python_service_->device = device;

    // Determine script path (should be in same directory as this source file)
    // For now, assume it's in the tracking/2d directory
    python_service_->script_path = "tracking/2d/mast3r_inference_service.py";

    if (!initializePythonService()) {
        LOG(ERROR) << "Failed to initialize MASt3R Python service";
    }
}

MASt3RTracker::~MASt3RTracker() {
    shutdownPythonService();
}

bool MASt3RTracker::initializePythonService() {
    LOG(INFO) << "Starting MASt3R Python inference service...";

    // Create pipes for stdin, stdout, stderr
    if (pipe(python_service_->stdin_pipe) != 0) {
        LOG(ERROR) << "Failed to create stdin pipe";
        return false;
    }
    if (pipe(python_service_->stdout_pipe) != 0) {
        LOG(ERROR) << "Failed to create stdout pipe";
        close(python_service_->stdin_pipe[0]);
        close(python_service_->stdin_pipe[1]);
        return false;
    }
    if (pipe(python_service_->stderr_pipe) != 0) {
        LOG(ERROR) << "Failed to create stderr pipe";
        close(python_service_->stdin_pipe[0]);
        close(python_service_->stdin_pipe[1]);
        close(python_service_->stdout_pipe[0]);
        close(python_service_->stdout_pipe[1]);
        return false;
    }

    // Fork process
    python_service_->process_id = fork();

    if (python_service_->process_id == -1) {
        LOG(ERROR) << "Failed to fork process for Python service";
        return false;
    }

    if (python_service_->process_id == 0) {
        // Child process - set up pipes and exec Python
        dup2(python_service_->stdin_pipe[0], STDIN_FILENO);
        dup2(python_service_->stdout_pipe[1], STDOUT_FILENO);
        dup2(python_service_->stderr_pipe[1], STDERR_FILENO);

        // Close unused pipe ends
        close(python_service_->stdin_pipe[0]);
        close(python_service_->stdin_pipe[1]);
        close(python_service_->stdout_pipe[0]);
        close(python_service_->stdout_pipe[1]);
        close(python_service_->stderr_pipe[0]);
        close(python_service_->stderr_pipe[1]);

        // Execute Python script
        execl(python_service_->python_executable.c_str(),
              python_service_->python_executable.c_str(),
              python_service_->script_path.c_str(), "--model",
              python_service_->model_name.c_str(), "--device",
              python_service_->device.c_str(), nullptr);

        // If exec fails
        std::cerr << "Failed to execute Python service" << std::endl;
        exit(1);
    }

    // Parent process - close unused pipe ends
    close(python_service_->stdin_pipe[0]);
    close(python_service_->stdout_pipe[1]);
    close(python_service_->stderr_pipe[1]);

    python_service_->is_running = true;

    // Test connection with ping
    LOG(INFO) << "Testing Python service connection...";
    std::string ping_request = "{\"command\": \"ping\"}\n";
    write(python_service_->stdin_pipe[1], ping_request.c_str(), ping_request.size());

    // Read response (with timeout)
    char buffer[4096];
    ssize_t bytes_read = read(python_service_->stdout_pipe[0], buffer, sizeof(buffer) - 1);

    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        LOG(INFO) << "Python service responded: " << buffer;
        return true;
    } else {
        LOG(ERROR) << "Python service did not respond to ping";
        return false;
    }
}

void MASt3RTracker::shutdownPythonService() {
    if (!python_service_->is_running) {
        return;
    }

    LOG(INFO) << "Shutting down MASt3R Python service...";

    // Send shutdown command
    std::string shutdown_request = "{\"command\": \"shutdown\"}\n";
    write(python_service_->stdin_pipe[1], shutdown_request.c_str(), shutdown_request.size());

    // Close pipes
    close(python_service_->stdin_pipe[1]);
    close(python_service_->stdout_pipe[0]);
    close(python_service_->stderr_pipe[0]);

    // Wait for process to terminate
    int status;
    waitpid(python_service_->process_id, &status, 0);

    python_service_->is_running = false;
    LOG(INFO) << "Python service shut down successfully";
}

MASt3RResult MASt3RTracker::runInference(const core::types::Image& prev_image,
                                        const core::types::Image& cur_image) {
    MASt3RResult result;

    if (!python_service_->is_running) {
        LOG(ERROR) << "Python service is not running";
        return result;
    }

    total_inferences_++;

    // Convert images to base64
    auto encodeImage = [](const cv::Mat& img) -> std::string {
        std::vector<uchar> buf;
        cv::imencode(".png", img, buf);
        std::string base64_str(buf.begin(), buf.end());

        // Base64 encode
        static const std::string base64_chars =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "abcdefghijklmnopqrstuvwxyz"
            "0123456789+/";

        std::string encoded;
        int i = 0;
        int j = 0;
        unsigned char char_array_3[3];
        unsigned char char_array_4[4];

        for (unsigned char c : base64_str) {
            char_array_3[i++] = c;
            if (i == 3) {
                char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
                char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
                char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
                char_array_4[3] = char_array_3[2] & 0x3f;

                for (i = 0; i < 4; i++)
                    encoded += base64_chars[char_array_4[i]];
                i = 0;
            }
        }

        if (i) {
            for (j = i; j < 3; j++)
                char_array_3[j] = '\0';

            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);

            for (j = 0; j < i + 1; j++)
                encoded += base64_chars[char_array_4[j]];

            while (i++ < 3)
                encoded += '=';
        }

        return encoded;
    };

    std::string img1_base64 = encodeImage(prev_image.data);
    std::string img2_base64 = encodeImage(cur_image.data);

    // Build JSON request
    std::ostringstream request_json;
    request_json << "{"
                 << "\"command\": \"inference\","
                 << "\"image1\": \"" << img1_base64 << "\","
                 << "\"image2\": \"" << img2_base64 << "\","
                 << "\"subsample_factor\": " << subsample_factor_ << "}\n";

    std::string request = request_json.str();

    LOG(INFO) << "Sending inference request to Python service (image sizes: " << prev_image.data.size()
              << ", " << cur_image.data.size() << ")";

    // Send request
    ssize_t bytes_written = write(python_service_->stdin_pipe[1], request.c_str(), request.size());
    if (bytes_written != static_cast<ssize_t>(request.size())) {
        LOG(ERROR) << "Failed to write complete request to Python service";
        return result;
    }

    // Read response
    char buffer[1024 * 1024];  // 1MB buffer for response
    ssize_t bytes_read = read(python_service_->stdout_pipe[0], buffer, sizeof(buffer) - 1);

    if (bytes_read <= 0) {
        LOG(ERROR) << "Failed to read response from Python service";
        return result;
    }

    buffer[bytes_read] = '\0';
    std::string response(buffer);

    LOG(INFO) << "Received response from Python service (" << bytes_read << " bytes)";

    // Parse JSON response (simplified parsing - in production use a proper JSON library)
    // For now, just extract the key fields manually
    // TODO: Use nlohmann/json for proper parsing

    if (response.find("\"success\": true") != std::string::npos) {
        result.success = true;
        successful_inferences_++;

        // Extract number of points
        size_t num_points_pos = response.find("\"num_points\":");
        if (num_points_pos != std::string::npos) {
            size_t num_start = response.find_first_of("0123456789", num_points_pos);
            size_t num_end = response.find_first_not_of("0123456789", num_start);
            int num_points = std::stoi(response.substr(num_start, num_end - num_start));
            LOG(INFO) << "MASt3R found " << num_points << " correspondences";
        }

        // TODO: Parse arrays from JSON
        // For a complete implementation, use nlohmann/json library
        LOG(WARNING) << "JSON parsing not fully implemented - returning empty result";
        LOG(WARNING) << "Please integrate nlohmann/json library for complete functionality";
    } else {
        LOG(ERROR) << "MASt3R inference failed: " << response;
    }

    return result;
}

std::optional<core::types::Pose> MASt3RTracker::match(const core::types::KeyFrame& prev_frame,
                                                      const core::types::KeyFrame& cur_frame,
                                                      std::vector<Eigen::Vector3d>& world_points) {
    LOG(INFO) << "MASt3R match: keyframe " << prev_frame.id << " -> " << cur_frame.id;

    if (!prev_frame.color_data.has_value() || !cur_frame.color_data.has_value()) {
        LOG(ERROR) << "Missing color data in keyframes";
        return std::nullopt;
    }

    // Run MASt3R inference
    MASt3RResult result = runInference(prev_frame.color_data.value(), cur_frame.color_data.value());

    if (!result.success || result.points_3d.empty()) {
        LOG(WARNING) << "MASt3R inference failed or returned no points";
        return std::nullopt;
    }

    LOG(INFO) << "MASt3R produced " << result.points_3d.size() << " 3D points";

    // Transform points to world frame
    world_points = transformToWorldFrame(result.points_3d, prev_frame);

    LOG(INFO) << "Transformed " << world_points.size() << " points to world frame";

    // Estimate relative pose
    if (!prev_frame.camera_info.has_value()) {
        LOG(ERROR) << "Missing camera info for pose estimation";
        return std::nullopt;
    }

    auto relative_transform =
        estimateRelativePose(result.points_3d, result.pixels_prev, result.pixels_curr,
                            prev_frame.camera_info.value());

    if (!relative_transform.has_value()) {
        LOG(WARNING) << "Failed to estimate relative pose";
        return std::nullopt;
    }

    // Convert Eigen::Isometry3d to core::types::Pose
    core::types::Pose relative_pose;
    relative_pose.position = relative_transform->translation();
    relative_pose.orientation = Eigen::Quaterniond(relative_transform->rotation());
    relative_pose.frame_id = prev_frame.pose.frame_id;

    LOG(INFO) << "MASt3R match successful: translation=" << relative_pose.position.transpose()
              << ", rotation=[" << relative_pose.orientation.w() << ", "
              << relative_pose.orientation.x() << ", " << relative_pose.orientation.y() << ", "
              << relative_pose.orientation.z() << "]";

    return relative_pose;
}

std::optional<core::types::Pose> MASt3RTracker::matchAndCreateKeypoints(
    const core::types::KeyFrame& prev_frame, const core::types::KeyFrame& cur_frame,
    std::map<uint32_t, core::types::Keypoint>& map_keypoints) {
    std::vector<Eigen::Vector3d> world_points;

    auto relative_pose = match(prev_frame, cur_frame, world_points);

    if (!relative_pose.has_value()) {
        return std::nullopt;
    }

    // Create keypoints from world points
    // TODO: Associate with descriptors for tracking

    LOG(INFO) << "Creating " << world_points.size() << " map keypoints from MASt3R matches";

    for (size_t i = 0; i < world_points.size(); ++i) {
        uint32_t keypoint_id = next_keypoint_id_++;

        core::types::Keypoint keypoint;
        keypoint.position = world_points[i];
        keypoint.needs_triangulation = false;  // Already triangulated by MASt3R

        // Add observations in both frames (TODO: add pixel coordinates from result)
        // For now, just mark as observed in both keyframes
        core::types::Location obs_prev;
        obs_prev.keyframe_id = prev_frame.id;
        obs_prev.frame_id = prev_frame.color_data->frame_id;
        obs_prev.x = 0.0;  // TODO: Use actual pixel coordinates from MASt3R result
        obs_prev.y = 0.0;

        core::types::Location obs_cur;
        obs_cur.keyframe_id = cur_frame.id;
        obs_cur.frame_id = cur_frame.color_data->frame_id;
        obs_cur.x = 0.0;
        obs_cur.y = 0.0;

        keypoint.locations.push_back(obs_prev);
        keypoint.locations.push_back(obs_cur);

        map_keypoints[keypoint_id] = keypoint;
    }

    return relative_pose;
}

std::vector<Eigen::Vector3d> MASt3RTracker::transformToWorldFrame(
    const std::vector<Eigen::Vector3d>& points_camera, const core::types::KeyFrame& reference_frame) {
    std::vector<Eigen::Vector3d> world_points;

    if (!tft_) {
        LOG(WARNING) << "Transform tree not available, returning points in camera frame";
        return points_camera;
    }

    try {
        // Get transform chain: camera -> base_link -> world
        auto camera_frame_id = reference_frame.color_data->frame_id;
        auto T_base_camera = tft_->getTransform(base_link_frame_id_, camera_frame_id).transform;
        auto T_world_base = reference_frame.pose.getEigenIsometry();
        auto T_world_camera = T_world_base * T_base_camera;

        // Transform all points
        world_points.reserve(points_camera.size());
        for (const auto& pt_camera : points_camera) {
            Eigen::Vector3d pt_world = T_world_camera * pt_camera;
            world_points.push_back(pt_world);
        }

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to transform points to world frame: " << e.what();
        return points_camera;
    }

    return world_points;
}

std::optional<Eigen::Isometry3d> MASt3RTracker::estimateRelativePose(
    const std::vector<Eigen::Vector3d>& points_3d, const std::vector<Eigen::Vector2d>& pixels_prev,
    const std::vector<Eigen::Vector2d>& pixels_curr, const core::types::CameraInfo& camera_info) {
    // Use PnP with RANSAC to estimate camera pose from 3D-2D correspondences

    if (points_3d.size() < 4 || points_3d.size() != pixels_curr.size()) {
        LOG(WARNING) << "Insufficient correspondences for pose estimation";
        return std::nullopt;
    }

    // Convert to OpenCV format
    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> image_points;

    for (size_t i = 0; i < points_3d.size(); ++i) {
        object_points.emplace_back(points_3d[i].x(), points_3d[i].y(), points_3d[i].z());
        image_points.emplace_back(pixels_curr[i].x(), pixels_curr[i].y());
    }

    // Camera intrinsics
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = camera_info.k[0];  // fx
    K.at<double>(1, 1) = camera_info.k[4];  // fy
    K.at<double>(0, 2) = camera_info.k[2];  // cx
    K.at<double>(1, 2) = camera_info.k[5];  // cy

    // Distortion coefficients
    cv::Mat dist_coeffs(5, 1, CV_64F, const_cast<double*>(camera_info.d.data()));

    // Solve PnP with RANSAC
    cv::Mat rvec, tvec;
    std::vector<int> inliers;

    bool success = cv::solvePnPRansac(object_points, image_points, K, dist_coeffs, rvec, tvec,
                                      false, 100, 8.0, 0.99, inliers);

    if (!success || inliers.size() < 10) {
        LOG(WARNING) << "PnP RANSAC failed or too few inliers: " << inliers.size();
        return std::nullopt;
    }

    LOG(INFO) << "PnP RANSAC found " << inliers.size() << " / " << points_3d.size() << " inliers";

    // Convert to Eigen
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    Eigen::Matrix3d R_eigen;
    Eigen::Vector3d t_eigen;

    for (int i = 0; i < 3; ++i) {
        t_eigen(i) = tvec.at<double>(i);
        for (int j = 0; j < 3; ++j) {
            R_eigen(i, j) = R.at<double>(i, j);
        }
    }

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = R_eigen;
    transform.translation() = t_eigen;

    return transform;
}

void MASt3RTracker::filterCorrespondences(MASt3RResult& result,
                                         const core::types::CameraInfo& camera_info) {
    // Filter based on confidence threshold
    if (result.confidence_scores.empty()) {
        return;
    }

    std::vector<Eigen::Vector3d> filtered_points;
    std::vector<Eigen::Vector2d> filtered_pixels_prev;
    std::vector<Eigen::Vector2d> filtered_pixels_curr;
    std::vector<float> filtered_confidence;

    for (size_t i = 0; i < result.points_3d.size(); ++i) {
        if (result.confidence_scores[i] >= confidence_threshold_) {
            filtered_points.push_back(result.points_3d[i]);
            filtered_pixels_prev.push_back(result.pixels_prev[i]);
            filtered_pixels_curr.push_back(result.pixels_curr[i]);
            filtered_confidence.push_back(result.confidence_scores[i]);
        }
    }

    result.points_3d = filtered_points;
    result.pixels_prev = filtered_pixels_prev;
    result.pixels_curr = filtered_pixels_curr;
    result.confidence_scores = filtered_confidence;

    LOG(INFO) << "Filtered correspondences by confidence: " << filtered_points.size() << " / "
              << result.confidence_scores.size() << " passed";
}

}  // namespace image
}  // namespace tracking

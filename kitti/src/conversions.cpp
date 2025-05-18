#include "kitti/include/kitti/conversions.hpp"
#include <cmath>    // For M_PI, cos, sin, etc.
#include <sstream>  // For stringstream in oxts parsing
#include <vector>

// For image conversions, if cv::Mat is used directly
#include <opencv2/imgcodecs.hpp>  // For cv::imdecode if raw data is used
#include <opencv2/imgproc.hpp>    // For cv::cvtColor if needed

namespace kitti {
namespace conversions {

Eigen::Isometry3d matrixToIsometry(const Eigen::Matrix<double, 3, 4>& mat) {
    Eigen::Matrix3d R = mat.leftCols<3>();
    Eigen::Vector3d t = mat.rightCols<1>();
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = R;
    T.translation() = t;
    return T;
}

Eigen::Isometry3d matrixToIsometry(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = R;
    T.translation() = t;
    return T;
}

core::types::Pose toPose(const Eigen::Isometry3d& eigen_pose, double timestamp,
                         std::string frame_id) {
    core::types::Pose pose;
    pose.position = eigen_pose.translation();

    Eigen::Quaterniond rotation(eigen_pose.rotation());
    pose.orientation = rotation;
    pose.timestamp = timestamp;
    pose.frame_id = frame_id;
    return pose;
}

core::types::Image toImage(const cv::Mat& cv_img, const std::string& frame_id) {
    core::types::Image core_image;
    if (cv_img.empty()) {
        return core_image;  // Return empty image
    }

    core_image.fromCvMat(cv_img, "rgb8", frame_id);

    return core_image;
}

cv::Mat toCvMat(const core::types::Image& core_img) {
    if (core_img.data.empty()) {
        return cv::Mat();
    }

    int cv_type = -1;
    if (core_img.encoding == "mono8") {
        cv_type = CV_8UC1;
    } else if (core_img.encoding == "bgr8") {
        cv_type = CV_8UC3;
    } else if (core_img.encoding == "rgb8") {  // Handle if you expect rgb8 sometimes
        cv_type = CV_8UC3;
        // If you create a cv::Mat this way, you might need to cvtColor from RGB to BGR if OpenCV
        // functions expect BGR
    } else if (core_img.encoding == "bgra8") {
        cv_type = CV_8UC4;
    } else {
        // Unsupported encoding
        return cv::Mat();
    }

    // Create a Mat pointing to the data, assuming step is correctly set.
    // Use const_cast if the core_img.data is const but cv::Mat needs non-const. Risky.
    // It's safer to copy data if modifications are expected or if lifetime issues arise.
    // cv::Mat mat(core_img.height, core_img.width, cv_type, const_cast<unsigned
    // char*>(core_img.data.data()), core_img.step); For safety, let's copy.
    cv::Mat mat_data_owner(core_img.height, core_img.width, cv_type);
    // TODO Need to get back to this
    // std::memcpy(mat_data_owner.data, core_img.data.data(), core_img.data.size());

    // If core_img was rgb8 and you need bgr8 for OpenCV:
    // if (core_img.encoding == "rgb8" && cv_type == CV_8UC3) {
    //    cv::cvtColor(mat_data_owner, mat_data_owner, cv::COLOR_RGB2BGR);
    // }
    return mat_data_owner;
}

// Geodetic to ECEF conversion
Eigen::Vector3d geodeticToEcef(double lat, double lon, double alt) {
    constexpr double a = 6378137.0;            // WGS84 semi-major axis
    constexpr double f = 1.0 / 298.257223563;  // WGS84 flattening
    constexpr double e_sq = f * (2.0 - f);     // Square of eccentricity

    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    double N = a / std::sqrt(1.0 - e_sq * std::sin(lat_rad) * std::sin(lat_rad));

    double x = (N + alt) * std::cos(lat_rad) * std::cos(lon_rad);
    double y = (N + alt) * std::cos(lat_rad) * std::sin(lon_rad);
    double z = (N * (1.0 - e_sq) + alt) * std::sin(lat_rad);
    return {x, y, z};
}

// ECEF to ENU conversion matrix
Eigen::Matrix3d ecefToEnuMatrix(double lat0_rad, double lon0_rad) {
    Eigen::Matrix3d R_enu;
    R_enu(0, 0) = -std::sin(lon0_rad);
    R_enu(0, 1) = std::cos(lon0_rad);
    R_enu(0, 2) = 0.0;
    R_enu(1, 0) = -std::sin(lat0_rad) * std::cos(lon0_rad);
    R_enu(1, 1) = -std::sin(lat0_rad) * std::sin(lon0_rad);
    R_enu(1, 2) = std::cos(lat0_rad);
    R_enu(2, 0) = std::cos(lat0_rad) * std::cos(lon0_rad);
    R_enu(2, 1) = std::cos(lat0_rad) * std::sin(lon0_rad);
    R_enu(2, 2) = std::sin(lat0_rad);
    return R_enu;
}

core::types::Pose toPoseFromOxts(const std::vector<double>& oxts_data,
                                 const std::vector<double>* first_oxts_data_ptr) {
    core::types::Pose pose;  // This is IMU pose in the world (local ENU) frame
    if (oxts_data.size() < 6)
        return pose;  // Invalid

    double lat = oxts_data[0];
    double lon = oxts_data[1];
    double alt = oxts_data[2];
    double roll = oxts_data[3];   // rotation around x-axis (forward)
    double pitch = oxts_data[4];  // rotation around y-axis (left)
    double yaw = oxts_data[5];    // rotation around z-axis (up)

    pose.orientation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    if (first_oxts_data_ptr && first_oxts_data_ptr->size() >= 3) {
        const auto& first_oxts = *first_oxts_data_ptr;
        double lat0 = first_oxts[0];
        double lon0 = first_oxts[1];
        double alt0 =
            first_oxts[2];  // alt0 is not strictly needed for ENU if relative poses are fine

        Eigen::Vector3d p_ecef = geodeticToEcef(lat, lon, alt);
        Eigen::Vector3d p0_ecef = geodeticToEcef(lat0, lon0, alt0);
        Eigen::Vector3d dp_ecef = p_ecef - p0_ecef;

        Eigen::Matrix3d R_enu = ecefToEnuMatrix(lat0 * M_PI / 180.0, lon0 * M_PI / 180.0);
        pose.position = R_enu * dp_ecef;
    } else {
        // If no reference point, position will be zero (or handle as error)
        pose.position = Eigen::Vector3d::Zero();
    }
    return pose;
}

core::types::PointCloud toPointCloud(const float* velo_data, size_t num_points_with_intensity,
                                     const std::string& frame_id, double timestamp) {
    core::types::PointCloud cloud;
    // TODO: Get back to this
    // cloud.timestamp = timestamp;
    // cloud.frame_id = frame_id;
    //
    // size_t num_points = num_points_with_intensity / 4;  // x,y,z,intensity
    // cloud.points.reserve(num_points);
    // cloud.intensities.reserve(num_points);
    //
    // for (size_t i = 0; i < num_points; ++i) {
    //     cloud.points.emplace_back(velo_data[i * 4], velo_data[i * 4 + 1], velo_data[i * 4 + 2]);
    //     cloud.intensities.push_back(velo_data[i * 4 + 3]);
    // }
    return cloud;
}

core::types::CameraInfo toCameraInfo(const Eigen::Matrix<double, 3, 4>& P_rect,
                                     const std::string& frame_id) {
    std::cout << "conversions::toCameraInfo: Entered" << std::endl;
    core::types::CameraInfo cam_info;
    cam_info.frame_id = frame_id;
    std::cout << "conversions::toCameraInfo: Initializing stuff" << std::endl;
    cam_info.width = static_cast<double>(P_rect(0, 2) * 2);
    cam_info.height = static_cast<double>(P_rect(1, 3) * 2);
    std::cout << "conversions::toCameraInfo: Initialized stuff" << std::endl;

    auto K = P_rect.block<3, 3>(0, 0);
    cam_info.k.resize(6);
    std::cout << "conversions::toCameraInfo: Initialized temp K" << std::endl;
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        cam_info.k.data(), K.rows(), K.cols()) = K;
    std::cout << "conversions::toCameraInfo: Initialized K" << std::endl;

    // KITTI data is usually rectified, so distortion parameters (D) are often zero.
    // If your specific sequence has them, you'll need to parse them from calib_cam_to_cam.txt
    cam_info.d.assign(5, 0.0);  // Assuming 5 distortion coeffs, all zero
    std::cout << "conversions::toCameraInfo: Initialized d" << std::endl;

    // P_rect = [K | K*t] = [fx 0 cx tx; 0 fy cy ty; 0 0 1 tz]
    // For a rectified camera, the P matrix is K_rect * [I | t_stereo] where t_stereo is the
    // baseline for stereo cameras. For a monocular camera, P_rect = [K_rect | 0]. So, the last
    // column should be zero for a single camera setup if K is truly just intrinsics. However, KITTI
    // P_rect_xx includes the extrinsic translation part for stereo. For core::types::CameraInfo, K
    // should be the pure intrinsic matrix. P_rect(0,3) and P_rect(1,3) are K * t_x and K * t_y. If
    // K is [fx 0 cx; 0 fy cy; 0 0 1], then P_rect(0,3) = fx * Tx + cx * Tz. Tz is usually 0 for
    // stereo baseline. For a single camera after rectification, P_rect = [fx' 0 cx' 0; 0 fy' cy' 0;
    // 0 0 1 0] We assume K_cam0 in KittiReader is already the intrinsic part.

    cam_info.distortion_model = "plumb_bob";  // Common model, or "rational_polynomial"
    std::cout << "conversions::toCameraInfo: Initialized model" << std::endl;

    return cam_info;
}

rerun::archetypes::Pinhole toRerunPinhole(const core::types::CameraInfo& cam_info) {
    // Rerun expects focal length and principal point in image coordinates (pixels).
    // K = [fx  s cx]
    //     [ 0 fy cy]
    //     [ 0  0  1]
    // Rerun's from_focal_length_and_resolution takes image_from_camera (K matrix)
    // and resolution. It can also take camera_xyz axis convention.
    // The Pinhole archetype can also be constructed with a 3x3 camera intrinsic matrix directly.
    // rerun::Mat3x3 image_from_camera_mat({{
    //     {fx, cam_info.K(0,1 /*skew*/), cx}},
    //     {0.0f, fy, cy}},
    //     {0.0f, 0.0f, 1.0f}}
    // });

    // Create pinhole camera using from_focal_length_and_resolution
    auto camera = rerun::archetypes::Pinhole::from_focal_length_and_resolution(
        {static_cast<float>(cam_info.k[0]),
         static_cast<float>(cam_info.k[4])},  // focal lengths (fx, fy)
        {static_cast<float>(cam_info.width), static_cast<float>(cam_info.height)}
        // resolution (uint32_t)
    );
    // Simpler way if K is already [fx 0 cx; 0 fy cy; 0 0 1]:
    return camera;
    // TODO: Check Rerun documentation for how it handles camera orientation / XYZ convention.
    // KITTI cameras are typically +Z forward, +X right, +Y down.
    // Rerun default might be different (e.g. +X right, +Y up, +Z backward or forward).
    // You might need to set .with_camera_xyz() if default doesn't match.
}

}  // namespace conversions
}  // namespace kitti

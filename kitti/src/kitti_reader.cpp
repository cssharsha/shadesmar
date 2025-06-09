#include "kitti/include/kitti/kitti_reader.hpp"
#include <algorithm>  // For std::min_element, std::abs
#include <cstdint>
#include <filesystem>  // For path manipulation
#include <iomanip>     // For std::setw, std::setfill
#include <iostream>    // For std::cout, std::cerr
#include <ostream>
#include <sstream>  // For std::ostringstream
#include "kitti/include/kitti/conversions.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

namespace kitti {

namespace fs = std::filesystem;

KittiReader::KittiReader(const Config& config,
                         core::storage::MapStore& store,
                         std::shared_ptr<viz::RerunVisualizer> visualizer)
    : config_(config), store_(store), visualizer_(visualizer),
      graph_adapter_(internal_graph_, store) {
    tf_tree_ = std::make_shared<stf::TransformTree>();
    graph_adapter_.setKeyframeDistanceThreshold(config_.keyframe_distance_threshold);
    graph_adapter_.setTransformTree(tf_tree_);
    visualizer_->setTransformTree(tf_tree_);
    // Set up clean storage-based callbacks
    core::graph::GraphCallbacks callbacks;
    // REMOVED: on_graph_updated callback - deprecated since FactorGraph is stateless
    // All visualization should use MapStore as single source of truth

    graph_adapter_.setCallbacks(callbacks);
}

bool KittiReader::initialize() {
    // Path to the specific sequence (e.g., /data/robot/kitti/data/odom/sequences/00)
    sequence_data_path_ =
        (fs::path(config_.dataset_base_path) / "sequences" / config_.sequence_number);
    // Path to ground truth poses (e.g., /data/robot/kitti/data/odom/poses/00.txt)
    poses_path_ =
        (fs::path(config_.dataset_base_path) / "poses" / (config_.sequence_number + ".txt"));

    std::cout << "Initializing KITTI Reader for sequence: " << config_.sequence_number << std::endl;
    std::cout << "  Sequence data path: " << sequence_data_path_.string() << std::endl;
    std::cout << "  Ground truth poses path: " << poses_path_.string() << std::endl;

    calibration_file_path_ = (sequence_data_path_ / config_.calib_file_name).string();
    times_file_path_ = (sequence_data_path_ / config_.times_file_name).string();
    std::cout << "Coming here 1" << std::endl;

    if (!fs::exists(sequence_data_path_)) {
        std::cout << "Error: Sequence data path does not exist: " << sequence_data_path_.string()
                  << std::endl;
        return false;
    }
    std::cout << "Coming here 2" << std::endl;
    if (!fs::exists(poses_path_)) {
        std::cout << "Error: Ground truth poses path does not exist: " << poses_path_.string()
                  << std::endl;
        // This might be a warning if we can operate without GT (e.g. VO mode)
        // For now, consider it an error if we expect odometry benchmark data.
        return false;
    }
    std::cout << "Coming here 3" << std::endl;
    if (!fs::exists(calibration_file_path_)) {
        std::cout << "Error: Calibration file does not exist: " << calibration_file_path_
                  << std::endl;
        return false;
    }
    std::cout << "Coming here 4" << std::endl;
    if (!fs::exists(times_file_path_)) {
        std::cout << "Error: Timestamps file does not exist: " << times_file_path_ << std::endl;
        return false;
    }
    std::cout << "Coming here 5" << std::endl;

    if (!loadCalibration()) {
        std::cout << "Failed to load calibration data from: " << calibration_file_path_
                  << std::endl;
        return false;
    }
    std::cout << "Coming here 6" << std::endl;

    if (!loadTimestamps()) {
        std::cout << "Failed to load timestamps from: " << times_file_path_ << std::endl;
        return false;
    }
    std::cout << "Coming here 7" << std::endl;

    if (!loadGroundTruthPoses()) {
        std::cout << "Failed to load ground truth poses from: " << poses_path_.string()
                  << std::endl;
        return false;
    }
    std::cout << "Coming here 8" << std::endl;

    current_frame_idx_ = 0;
    max_frames_ = timestamps_s_.size();
    if (max_frames_ == 0) {
        std::cout << "Error: No timestamps loaded, cannot process sequence." << std::endl;
        return false;
    }
    std::cout << "Coming here 9" << std::endl;
    if (ground_truth_poses_.size() != max_frames_ && !ground_truth_poses_.empty()) {
        std::cout << "Warning: Number of ground truth poses (" << ground_truth_poses_.size()
                  << ") does not match number of timestamps (" << max_frames_
                  << "). Using minimum of the two for frame count." << std::endl;
        max_frames_ = std::min(ground_truth_poses_.size(), max_frames_);
    }

    std::cout << "KITTI Reader initialized successfully. Found " << max_frames_
              << " frames for sequence " << config_.sequence_number << std::endl;
    return true;
}

std::string KittiReader::getImagePath(size_t frame_idx, int camera_idx) const {
    std::ostringstream filename;
    filename << std::setw(6) << std::setfill('0') << frame_idx << ".png";
    // Odometry dataset typically has image_0, image_1, image_2, image_3
    return (sequence_data_path_ / ("image_" + std::to_string(camera_idx)) / filename.str())
        .string();
}

std::string KittiReader::getVelodynePath(size_t frame_idx) const {
    std::ostringstream filename;
    filename << std::setw(6) << std::setfill('0') << frame_idx << ".bin";
    return (sequence_data_path_ / "velodyne" / filename.str()).string();
}

// getOxtsPath might not be directly used if relying on ground truth poses from odometry benchmark
// For raw data, it would be:
// return (sequence_data_path_ / "oxts" / "data" / filename.str()).string();

bool KittiReader::loadCalibration() {
    std::ifstream calib_file(calibration_file_path_);
    if (!calib_file.is_open()) {
        std::cout << "Error: Could not open calibration file: " << calibration_file_path_
                  << std::endl;
        return false;
    }

    std::string line;
    std::map<std::string, Eigen::MatrixXd> calib_data_raw;
    std::cout << "loadCalibration: 1" << std::endl;

    while (std::getline(calib_file, line)) {
        std::stringstream ss(line);
        std::string key_str;
        ss >> key_str;
        if (key_str.empty())
            continue;

        // KITTI keys end with ':', remove it. e.g. "P0:", "Tr:"
        if (!key_str.empty() && key_str.back() == ':') {
            key_str.pop_back();
        }

        std::vector<double> values;
        double val;
        while (ss >> val) {
            values.push_back(val);
        }

        Eigen::MatrixXd mat;
        if ((key_str.rfind("P", 0) == 0) && values.size() == 12) {  // P0, P1, P2, P3 are 3x4
            mat = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(values.data());
        } else if (key_str == "Tr" && values.size() == 12) {  // Tr (velo_to_cam) is 3x4
            mat = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(values.data());
        } else {
            std::cout << "Skipping unknown or malformed calib key: " << key_str << " with "
                      << values.size() << " values." << std::endl;
            continue;
        }
        calib_data_raw[key_str] = mat;
    }
    std::cerr << "loadCalibration: calib_data_raw: " << calib_data_raw.size() << std::endl;
    calib_file.close();

    // Extract Camera 0 (left grayscale) projection matrix P0
    std::vector<double> K;
    if (calib_data_raw.count("P0")) {
        std::cout << "loadCalibration: Loading P0" << std::endl;
        calib_data_.P0 = calib_data_raw["P0"];
        std::cout << "loadCalibration: Loading K" << std::endl;
        K_cam0_ = calib_data_.P0.block<3, 3>(0, 0);  // Intrinsic matrix
        std::cout << "loadCalibration: Loading cam info" << std::endl;
        camera_info_cam0_ =
            conversions::toCameraInfo(calib_data_.P0, config_.left_cam_gray_frame_id);
        std::cout << "loadCalibration: Loaded camera_info_cam0_" << std::endl;
    } else {
        std::cout << "P0 (left grayscale camera projection) not found in calib file: "
                  << calibration_file_path_ << std::endl;
        return false;
    }
    std::cerr << "Print something" << std::endl;
    // Extract Camera 1 (right grayscale) projection matrix P1
    if (calib_data_raw.count("P1")) {
        calib_data_.P1 = calib_data_raw["P1"];
    } else {
        std::cerr << "P1 (right grayscale camera projection) not found in calib file: "
                  << calibration_file_path_ << std::endl;
        return false;
    }
    // Optionally load P2, P3 if color cameras are used
    if (calib_data_raw.count("P2"))
        calib_data_.P2 = calib_data_raw["P2"];
    if (calib_data_raw.count("P3"))
        calib_data_.P3 = calib_data_raw["P3"];

    // Extract Velodyne to Camera 0 transform (Tr)
    // This is T_cam0_velo (transform points from velodyne to cam0 frame)
    // Eigen::MatrixXd tr_velo_to_cam0_mat_3x4;
    // if (calib_data_raw.count("Tr")) {
    //     tr_velo_to_cam0_mat_3x4 = calib_data_raw["Tr"];
    // } else {
    //     std::cerr << "Tr (Velodyne to Cam0 transform) not found in calib file: "
    //               << calibration_file_path_ << std::endl;
    //     return false;
    // }
    // // Convert 3x4 P_matrix (R|t) to Isometry3d
    // Eigen::Matrix4d tr_velo_to_cam0_mat_4x4 = Eigen::Matrix4d::Identity();
    // tr_velo_to_cam0_mat_4x4.block<3, 4>(0, 0) = tr_velo_to_cam0_mat_3x4;
    // T_cam0_velodyne_ = Eigen::Isometry3d(tr_velo_to_cam0_mat_4x4);
    T_cam0_velodyne_ = Eigen::Isometry3d::Identity();

    // In KITTI Odometry, the 'vehicle' or 'body' frame, to which ground truth poses refer,
    // is often implicitly the same as the IMU/GPS unit's frame *at the start of the sequence*
    // or a fixed point on the vehicle. The sensor extrinsics (P0, P1, Tr) define how
    // sensors are mounted relative to each other, often with P0 defining the coordinate
    // system for cam0, and Tr relating Velodyne to cam0.
    // For simplicity, we assume ground truth poses are for config_.vehicle_frame_id
    // and other sensors are relative to that.
    // Let's assume CAM0 is the reference for other sensors defined in calib.txt
    // T_vehicle_cam0 will be identity IF poses are given directly for CAM0,
    // OR if CAM0 is the reference point of the "vehicle".
    // More commonly, poses are for the vehicle body (e.g., rear axle center).
    // We need to know T_cam0_vehicle or T_velodyne_vehicle.
    // The `calib.txt` provides T_cam0_velodyne (from Tr).
    // For now, let's set up transforms relative to the vehicle frame specified in config.
    // We need T_cam0_vehicle and T_velodyne_vehicle.
    // The `Tr` in `calib.txt` is T_cam0_velodyne.
    // Let's assume the vehicle frame is the Velodyne frame for simplicity,
    // as Velodyne is often a central sensor. This is a common simplification if
    // precise vehicle reference point is not defined or used.
    // Then T_velodyne_vehicle is Identity.
    // And T_cam0_vehicle = T_cam0_velodyne * T_velodyne_vehicle = T_cam0_velodyne.

    // TF tree: world -> vehicle (from GT poses) -> sensors
    // We need:
    // T_vehicle_cam0_gray_left
    // T_vehicle_velodyne

    // Let's assume the `vehicle_frame_id` is the coordinate system of the Velodyne sensor.
    // This means T_vehicle_velodyne is identity.
    // Then T_vehicle_cam0 = T_velodyne_cam0.inverse() = T_cam0_velodyne.inverse()
    // Which is (Eigen::Isometry3d(tr_velo_to_cam0_mat_4x4)).inverse()

    T_vehicle_velodyne_ = Eigen::Isometry3d::Identity();  // Velodyne IS the vehicle frame
    T_vehicle_cam0_gray_left_ = T_cam0_velodyne_.inverse();

    // Update the transform tree
    // The parent frame for sensors will be the vehicle_frame_id.
    // The pose of vehicle_frame_id in world_frame_id will be updated from ground truth.
    tf_tree_->setTransform(config_.vehicle_frame_id, config_.left_cam_gray_frame_id,
                           T_vehicle_cam0_gray_left_);
    tf_tree_->setTransform(config_.vehicle_frame_id, config_.velodyne_frame_id,
                           T_vehicle_velodyne_);
    // If right camera is used:
    // We need T_cam1_cam0 from P0, P1 baseline. P = K [R | t]. t = -K_inv * P.col(3). Baseline =
    // t_P1_x - t_P0_x For P0 = [f_u 0 c_u P_03; 0 f_v c_v P_13; 0 0 1 P_23] P_x = K_inv * P.col(3)
    // gives translation component of camera X in its own optical frame (if R=I). Better: P1
    // projects points from the world to cam1. P0 projects points from world to cam0. The
    // transformation between cam0 and cam1 can be derived from their projection matrices P0 and P1.
    // P0 = K0 [I|0] (assuming cam0 is the reference for stereo)
    // P1 = K1 [R|t] (where R,t is transform from cam0 to cam1, R=I for rectified stereo)
    // For KITTI, cam0 and cam1 are rectified. P0 = [fx 0 cx | 0; 0 fy cy | 0; 0 0 1 | 0]
    // P1 = [fx 0 cx' | bx*fx; 0 fy cy | 0; 0 0 1 | 0] where bx is baseline.
    // So T_cam0_cam1 is a translation by (bx, 0, 0) in cam0 frame.
    // bx = (P0(0,3) - P1(0,3)) / P0(0,0)  (if P0(0,3) is 0) -> bx = -P1(0,3)/P1(0,0)
    if (calib_data_.P0.cols() == 4 && calib_data_.P1.cols() == 4 && calib_data_.P0(0, 0) != 0) {
        double baseline_x = (calib_data_.P0(0, 3) - calib_data_.P1(0, 3)) /
                            calib_data_.P0(0, 0);  // This is B_x for P1 relative to P0
        Eigen::Isometry3d T_cam0_cam1 = Eigen::Isometry3d::Identity();
        T_cam0_cam1.translation().x() =
            baseline_x;  // Cam1 is to the right of Cam0 (positive x for Cam0)
        T_vehicle_cam1_gray_right_ = T_vehicle_cam0_gray_left_ * T_cam0_cam1;
        tf_tree_->setTransform(config_.vehicle_frame_id, config_.right_cam_gray_frame_id,
                               T_vehicle_cam1_gray_right_);

        std::cout << "  Baseline (Cam0 to Cam1): " << baseline_x << " m" << std::endl;
        std::cout << "  T_vehicle_cam0_gray_left (Cam0 to Vehicle):\n"
                  << T_vehicle_cam0_gray_left_.matrix() << std::endl;
        std::cout << "  T_vehicle_cam1_gray_right (Cam1 to Vehicle):\n"
                  << T_vehicle_cam1_gray_right_.matrix() << std::endl;
    }

    std::cout << "Calibration loaded:" << std::endl;
    std::cout << "  K_cam0 (Left Gray Cam Intrinsics):\n" << K_cam0_ << std::endl;
    std::cout << "  T_cam0_velodyne (Velodyne to Cam0):\n"
              << T_cam0_velodyne_.matrix() << std::endl;

    tf_tree_->printTree();
    return true;
}

bool KittiReader::loadTimestamps() {
    std::ifstream times_file(times_file_path_);
    if (!times_file.is_open()) {
        std::cerr << "Error: Could not open timestamps file: " << times_file_path_ << std::endl;
        return false;
    }
    timestamps_s_.clear();
    std::string line;
    while (std::getline(times_file, line)) {
        try {
            double ts = std::stod(line);
            timestamps_s_.push_back(ts);
        } catch (const std::invalid_argument& ia) {
            std::cerr << "Invalid argument: " << ia.what() << " for line: " << line << std::endl;
        } catch (const std::out_of_range& oor) {
            std::cerr << "Out of Range error: " << oor.what() << " for line: " << line << std::endl;
        }
    }
    times_file.close();
    if (timestamps_s_.empty()) {
        std::cerr << "No timestamps loaded from " << times_file_path_ << std::endl;
        return false;
    }
    std::cout << "Loaded " << timestamps_s_.size() << " timestamps from " << times_file_path_
              << std::endl;
    return true;
}

bool KittiReader::loadGroundTruthPoses() {
    std::ifstream poses_file(poses_path_.string());
    if (!poses_file.is_open()) {
        std::cerr << "Error: Could not open ground truth poses file: " << poses_path_.string()
                  << std::endl;
        return false;
    }
    ground_truth_poses_.clear();
    std::string line;
    while (std::getline(poses_file, line)) {
        std::stringstream ss(line);
        double val;
        std::vector<double> values;
        while (ss >> val) {
            values.push_back(val);
        }
        if (values.size() == 12) {  // Each line is a 3x4 row-major matrix
            Eigen::Matrix<double, 3, 4, Eigen::RowMajor> pose_mat_3x4(values.data());
            Eigen::Matrix4d pose_mat_4x4 = Eigen::Matrix4d::Identity();
            pose_mat_4x4.block<3, 4>(0, 0) = pose_mat_3x4;
            ground_truth_poses_.emplace_back(pose_mat_4x4);
        } else {
            std::cerr << "Warning: Malformed line in pose file " << poses_path_.string()
                      << ". Expected 12 values, got " << values.size() << ". Line: " << line
                      << std::endl;
        }
    }
    poses_file.close();
    if (ground_truth_poses_.empty()) {
        std::cerr << "No ground truth poses loaded from " << poses_path_.string() << std::endl;
        return false;
    }
    std::cout << "Loaded " << ground_truth_poses_.size() << " ground truth poses from "
              << poses_path_.string() << std::endl;
    return true;
}

// ... existing code ...
// Make sure to declare these new member variables in kitti_reader.hpp:
// fs::path sequence_data_path_;
// fs::path poses_path_;
// std::string calibration_file_path_;
// std::string times_file_path_;
// std::vector<double> timestamps_s_;
// std::vector<Eigen::Isometry3d> ground_truth_poses_;
// size_t current_frame_idx_ = 0;
// size_t max_frames_ = 0;
// CalibrationData calib_data_; // struct to hold P0, P1, P2, P3, Tr
// Eigen::Isometry3d T_cam0_velodyne_;
// Eigen::Isometry3d T_vehicle_velodyne_;
// Eigen::Isometry3d T_vehicle_cam0_gray_left_;
// Eigen::Isometry3d T_vehicle_cam1_gray_right_;

// And in kitti_reader.hpp, add/update the CalibrationData struct:
/*
struct CalibrationData {
    Eigen::Matrix<double, 3, 4, Eigen::RowMajor> P0, P1, P2, P3; // Projection matrices
    // Tr is already effectively stored in T_cam0_velodyne_
};
*/
// And ensure K_cam0_ is Eigen::Matrix3d;

// Modify processFrame to use ground_truth_poses_ and timestamps_s_
bool KittiReader::processNextFrame() {
    if (current_frame_idx_ >= max_frames_) {
        std::cout << "Finished processing all frames for sequence " << config_.sequence_number
                  << std::endl;
        return false;  // No more frames
    }

    double timestamp_sec = timestamps_s_[current_frame_idx_];

    // Update vehicle pose in the world
    const Eigen::Isometry3d& T_world_vehicle = ground_truth_poses_[current_frame_idx_];
    auto pose = conversions::toPose(T_world_vehicle, timestamp_sec, config_.vehicle_frame_id);
    tf_tree_->setTransform(config_.world_frame_id, config_.vehicle_frame_id, T_world_vehicle);

    // Add vehicle pose to graph
    graph_adapter_.handleOdometryInput(pose, timestamp_sec);

    std::cout << "Processing frame " << current_frame_idx_ << "/" << max_frames_ - 1
              << " (ts: " << std::fixed << std::setprecision(3) << timestamp_sec << "s)"
              << std::endl;

    if (config_.load_images) {
        // Load left grayscale image (cam0)
        std::string img0_path = getImagePath(current_frame_idx_, 2);  // Camera 0
        if (fs::exists(img0_path)) {
            cv::Mat image0 = cv::imread(img0_path);
            if (!image0.empty()) {
                auto internal_image = conversions::toImage(image0, config_.left_cam_gray_frame_id);
                graph_adapter_.handleImageInput(internal_image, timestamp_sec);
                graph_adapter_.handleCameraInfo(camera_info_cam0_, timestamp_sec);
            } else {
                std::cerr << "Warning: Failed to load image: " << img0_path << std::endl;
            }
        } else {
            std::cerr << "Warning: Image file not found: " << img0_path << std::endl;
        }

        // Optionally load right grayscale image (cam1)
        // std::string img1_path = getImagePath(current_frame_idx_, 1);  // Camera 1
        // if (fs::exists(img1_path)) {
        //     cv::Mat image1 = cv::imread(img1_path, cv::IMREAD_GRAYSCALE);
        //     if (!image1.empty()) {
        //         auto internal_image = conversions::to(image1);
        //         if (visualizer_ && visualizer_->isConnected()) {
        //             visualizer_->logImage(config_.right_cam_gray_frame_id, internal_image.image,
        //                                   internal_image.timestamp_ns);
        //         }
        //     } else {
        //         std::cerr << "Warning: Failed to load image: " << img1_path << std::endl;
        //     }
        // } else {
        //     // This is common if only mono is processed, so not a strong warning.
        //     // std::cout << "Debug: Image file not found: " << img1_path << std::endl;
        // }
    }

    // graph_adapter_.update();  // Process this frame's data

    current_frame_idx_++;
    return true;
}

bool KittiReader::processSequence() {
    std::cout << "Processing sequence " << config_.sequence_number << " with " << max_frames_
              << " frames..." << std::endl;

    // Initialize keyframe selection parameters
    double last_keyframe_distance = 0.0;
    Eigen::Isometry3d last_keyframe_pose = Eigen::Isometry3d::Identity();
    bool first_frame = true;

    // Process all frames in the sequence
    while (processNextFrame()) {
        // Check if we should stop early (for debugging or testing)
        if (config_.max_frames > 0 && current_frame_idx_ >= config_.max_frames) {
            std::cout << "Reached maximum number of frames (" << config_.max_frames
                      << "), stopping." << std::endl;
            break;
        }

        // Optional: Add a delay between frames for visualization
        if (config_.frame_delay_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(config_.frame_delay_ms));
        }
    }

    // Save the factor graph to a VTK file
    std::string sequence_name = "kitti_" + config_.sequence_number;
    std::time_t now = std::time(nullptr);
    char timestamp[20];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));

    fs::path output_dir = fs::path(config_.dataset_base_path) / "output";
    if (!fs::exists(output_dir)) {
        fs::create_directories(output_dir);
    }

    std::string output_filename = "factor_graph_" + sequence_name + "_" + timestamp + ".vtk";
    fs::path output_path = output_dir / output_filename;

    std::cout << "Saving factor graph to " << output_path.string() << std::endl;
    graph_adapter_.maybeDumpGraph(true);

    return true;
}

// Remove or adapt old processFrame(size_t frame_idx) if it exists.
// The new processNextFrame() is the main processing logic per frame.

// Remove or comment out the old loadTimestamps which loaded multiple timestamp files
// The new one loads the main times.txt from the sequence directory.

// Remove oxts related file reading unless specifically needed for raw data processing.
// The ground truth poses from the odometry benchmark are generally preferred.

}  // namespace kitti

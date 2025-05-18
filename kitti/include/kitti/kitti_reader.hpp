#pragma once

#include <filesystem>  // Required for fs::path
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"  // For Eigen::Matrix, Eigen::Isometry3d

#include "core/graph/factor_graph.hpp"
#include "core/graph/graph_adapter.hpp"
#include "core/storage/map_store.hpp"
#include "core/types/image.hpp"
#include "kitti/include/kitti/config.hpp"
#include "stf/transform_tree.hpp"  // For stf::TransformTree
#include "viz/rerun_viz.hpp"

namespace kitti {

namespace fs = std::filesystem;

// To store parsed calibration matrices from calib.txt
struct CalibrationData {
    Eigen::Matrix<double, 3, 4, Eigen::RowMajor> P0, P1, P2,
        P3;  // Projection matrices for cam0 to cam3
    // Tr (velo to cam0 transform) is stored as T_cam0_velodyne_
};

class KittiReader {
public:
    KittiReader(const KittiReaderConfig& config, core::graph::FactorGraph& graph,
                core::storage::MapStore& store, std::shared_ptr<viz::RerunVisualizer> visualizer);

    bool initialize();
    // void run();  // Main loop to process sequence - replaced by processSequence
    bool processNextFrame();  // Processes a single frame
    bool processSequence();   // Processes all frames in the sequence

private:
    bool loadCalibration();
    bool loadTimestamps();
    bool loadGroundTruthPoses();  // New method to load GT poses
    // bool processFrame(size_t frame_idx); // Replaced by processNextFrame()

    // Parsing functions are mostly integrated or placeholders within .cpp
    // core::types::Pose parseImuData(const std::string& line) const; // Not used with GT poses
    // core::types::PointCloud parseVelodyneData(const std::string& path) const; // Integrated
    // core::types::Image loadImage(const std::string& path) const; // Integrated via OpenCV

    // Helper to get full path for a data type and frame index
    // std::string getImagePath(size_t frame_idx, const std::string& camera_folder = "image_00")
    // const; // Updated signature
    std::string getImagePath(size_t frame_idx, int camera_idx) const;
    std::string getVelodynePath(size_t frame_idx) const;

    KittiReaderConfig config_;
    core::graph::GraphAdapter graph_adapter_;
    std::shared_ptr<viz::RerunVisualizer> visualizer_;
    std::shared_ptr<stf::TransformTree> tf_tree_;

    // Paths
    fs::path sequence_data_path_;  // Path to the sequence dir (e.g., .../sequences/00/)
    fs::path poses_path_;          // Path to the ground truth poses file (e.g., .../poses/00.txt)
    std::string calibration_file_path_;  // Full path to calib.txt
    std::string times_file_path_;        // Full path to times.txt

    // Loaded data
    std::vector<double> timestamps_s_;                   // Timestamps in seconds for each frame
    std::vector<Eigen::Isometry3d> ground_truth_poses_;  // World to Vehicle transforms

    CalibrationData calib_data_;  // Stores P0, P1, P2, P3
    Eigen::Matrix3d K_cam0_;      // Intrinsic matrix for camera 0 (derived from P0)
    core::types::CameraInfo camera_info_cam0_;
    // Transforms (relative to vehicle frame, which is often Velodyne for KITTI odom)
    Eigen::Isometry3d
        T_cam0_velodyne_;  // Extrinsic: Velodyne frame to Cam0 frame (from Tr in calib.txt)
    Eigen::Isometry3d
        T_vehicle_velodyne_;  // Extrinsic: Velodyne frame to Vehicle frame (often Identity)
    Eigen::Isometry3d T_vehicle_cam0_gray_left_;  // Extrinsic: Cam0 frame to Vehicle frame
    Eigen::Isometry3d
        T_vehicle_cam1_gray_right_;  // Extrinsic: Cam1 frame to Vehicle frame (if stereo)

    // Processing state
    size_t current_frame_idx_ = 0;
    size_t max_frames_ = 0;

    // Removed old members:
    // Eigen::Isometry3d T_cam0_imu_;
    // Eigen::Isometry3d T_velodyne_imu_;
    // std::vector<double> timestamps_cam0_;
    // std::vector<double> timestamps_velodyne_;
    // std::string sequence_path_; // Replaced by sequence_data_path_
    // std::string calib_imu_to_velo_path_;
    // std::string calib_imu_to_cam_path_;
    // std::string calib_cam_to_cam_path_;
    // std::string times_path_; // Replaced by times_file_path_
    // std::ifstream imu_file_;
    // size_t current_frame_ = 0; // Replaced by current_frame_idx_
};

}  // namespace kitti

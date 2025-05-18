#pragma once

#include <string>
#include <vector>

namespace kitti {

struct KittiReaderConfig {
    // Base path to the KITTI odometry dataset directory (e.g., /data/robot/kitti/data/odom)
    // This path should contain 'sequences' and 'poses' (if using GT) or be adaptable for raw data
    // structure
    std::string dataset_base_path = "/data/robot/kitti/data/odom";
    // Sequence number to process (e.g., "00", "01", ... "10" for odom, or
    // "2011_09_26_drive_0005_sync" for raw)
    std::string sequence_number = "00";

    // File names within the sequence directory
    std::string calib_file_name = "calib.txt";  // For P0-P3, Tr (velo to cam)
    std::string times_file_name = "times.txt";

    // OXTS (GPS/IMU) data configuration (used if not using ground truth poses file)
    std::string oxts_data_dirname = "oxts";  // Typically "oxts" in sequence folder
    // std::string oxts_data_subdirname = "data"; // Often under oxts/data/
    // For KITTI odom, oxts data is usually in <sequence_path>/oxts/data/NNNNNNNNNN.txt
    // For KITTI raw, it's in <drive_path>/oxts/data/NNNNNNNNNN.txt

    // Calibration file for IMU to Velodyne transform.
    // This might be alongside calib.txt in the sequence folder for odometry,
    // or in a general calibration directory for raw data.
    // Example: "calib_imu_to_velo.txt"
    std::string imu_to_velo_calib_file_name =
        "calib_imu_to_velo.txt";  // Relative to sequence_data_path_ or a global calib dir? For now,
                                  // assume in sequence_data_path_

    // Sensor frame IDs
    std::string world_frame_id = "world";
    // vehicle_frame_id is the primary moving frame, its pose is derived from OXTS data.
    // It's often considered co-located with the IMU origin after calibration.
    std::string vehicle_frame_id = "vehicle_link";
    std::string velodyne_frame_id = "velodyne";                 // LiDAR sensor frame
    std::string left_cam_gray_frame_id = "camera_gray_left";    // Grayscale left camera (image_0)
    std::string right_cam_gray_frame_id = "camera_gray_right";  // Grayscale right camera (image_1)
    // The imu_frame_id specified here can be an alias or the same as vehicle_frame_id if OXTS data
    // directly refers to it. std::string imu_frame_id = "imu_link"; // This was the old placeholder

    // Flags to control what data to load
    bool load_images = true;
    bool load_point_clouds = true;

    // Processing parameters
    double keyframe_distance_threshold = 1.0;  // meters (for keyframing)
    int max_frames = 0;                        // 0 means process all frames
    int frame_delay_ms = 0;  // Delay between frames for visualization (0 = no delay)
};

}  // namespace kitti

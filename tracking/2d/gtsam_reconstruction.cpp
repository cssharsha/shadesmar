#include "gtsam_reconstruction.hpp"
#include "triangulation_common.hpp"
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/triangulation.h>
#include <logging/logging.hpp>
#include <stf/transform_utils.hpp>
#include <set>
#include <chrono>
#include <limits>

namespace tracking {
namespace image {

gtsam::Pose3 eigenToGtsamPose3(const Eigen::Isometry3d& eigen_pose) {
    gtsam::Rot3 rotation(eigen_pose.rotation());
    gtsam::Point3 translation(eigen_pose.translation());
    return gtsam::Pose3(rotation, translation);
}

bool GtsamReconstruct::triangulate(const core::types::KeyFrame& kf1,
                                   const core::types::KeyFrame& kf2,
                                   const std::vector<cv::Point2f>& pts1,
                                   const std::vector<cv::Point2f>& pts2,
                                   const stf::TransformTree& tft,
                                   const Eigen::Isometry3d& essentialMatrix,
                                   std::vector<Eigen::Vector3d>& triangulated_points_world,
                                   bool use_essential_mat, const std::string& base_link_frame_id) {
    if (pts1.size() != pts2.size()) {
        LOG(ERROR) << "Should have the same number of points for triangulation";
        return false;
    }

    LOG(INFO) << "GTSAM triangulation for " << pts1.size() << " point pairs";

    // Get camera calibration matrices
    auto K1 = kf1.camera_info.value().getKInEigen();
    auto K2 = kf2.camera_info.value().getKInEigen();

    LOG(INFO) << "K1:\n" << K1.matrix();
    LOG(INFO) << "K2:\n" << K2.matrix();

    if (std::abs(K1.determinant()) <= 1e-9 || std::abs(K2.determinant()) <= 1e-9) {
        LOG(ERROR) << "Unable to invert K - determinant too small";
        return false;
    }

    // Convert Eigen camera matrices to GTSAM Cal3_S2
    gtsam::Cal3_S2 gtsam_K1(K1(0, 0), K1(1, 1), 0.0, K1(0, 2), K1(1, 2));
    gtsam::Cal3_S2 gtsam_K2(K2(0, 0), K2(1, 1), 0.0, K2(0, 2), K2(1, 2));

    // Get base_link transform if available
    auto get_base_link_tf = [&](const std::string& camera_frame) -> Eigen::Isometry3d {
        try {
            auto transform_result = tft.getTransform(base_link_frame_id, camera_frame);

            return transform_result.transform;
        } catch (const std::exception& e) {
            LOG(WARNING) << "Failed to get base_link transform: " << e.what();
            // std::throw(std::runtime_error("Unable to get the base link to camera transform"));
            exit(1);
        }
    };
    gtsam::Pose3 pose1 = eigenToGtsamPose3(kf1.pose.getEigenIsometry() *
                                           get_base_link_tf(kf1.color_data.value().frame_id));
    gtsam::Pose3 pose2 = eigenToGtsamPose3(kf2.pose.getEigenIsometry() *
                                           get_base_link_tf(kf2.color_data.value().frame_id));

    // Write keyframe poses to CSV file (similar to reconstruction.cpp transform writing)
    std::ofstream gtsam_csv("/data/robot/bags/house11/gtsam.csv", std::ios::app);
    if (gtsam_csv.is_open()) {
        // Write header if file is empty (first write)
        gtsam_csv.seekp(0, std::ios::end);
        if (gtsam_csv.tellp() == 0) {
            gtsam_csv << "x,y,z,r,p,y,kf_id,frame_id\n";
        }

        // Write keyframe 1 pose
        Eigen::Isometry3d kf1_world_pose = kf1.pose.getEigenIsometry() * 
                                          get_base_link_tf(kf1.color_data.value().frame_id);
        Eigen::Vector3d kf1_translation = kf1_world_pose.translation();
        Eigen::Vector3d kf1_rpy = stf::getRPY(kf1_world_pose);
        
        gtsam_csv << std::fixed << std::setprecision(6) 
                  << kf1_translation.x() << "," << kf1_translation.y() << "," << kf1_translation.z() << ","
                  << kf1_rpy.x() << "," << kf1_rpy.y() << "," << kf1_rpy.z() << ","
                  << kf1.id << "," << kf1.color_data.value().frame_id << "\n";

        // Write keyframe 2 pose
        Eigen::Isometry3d kf2_world_pose = kf2.pose.getEigenIsometry() * 
                                          get_base_link_tf(kf2.color_data.value().frame_id);
        Eigen::Vector3d kf2_translation = kf2_world_pose.translation();
        Eigen::Vector3d kf2_rpy = stf::getRPY(kf2_world_pose);
        
        gtsam_csv << std::fixed << std::setprecision(6) 
                  << kf2_translation.x() << "," << kf2_translation.y() << "," << kf2_translation.z() << ","
                  << kf2_rpy.x() << "," << kf2_rpy.y() << "," << kf2_rpy.z() << ","
                  << kf2.id << "," << kf2.color_data.value().frame_id << "\n";
        
        gtsam_csv.close();
    } else {
        LOG(WARNING) << "Failed to open /data/robot/bags/house11/gtsam.csv for writing";
    }

    // Create GTSAM cameras
    gtsam::PinholeCamera<gtsam::Cal3_S2> camera1(pose1, gtsam_K1);
    gtsam::PinholeCamera<gtsam::Cal3_S2> camera2(pose2, gtsam_K2);
    gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>> cameras;
    cameras.push_back(camera1);
    cameras.push_back(camera2);


    // Triangulate each point pair
    for (size_t i = 0; i < pts1.size(); ++i) {
        // Create GTSAM measurements
        gtsam::Point2Vector measurements;
        measurements.push_back(gtsam::Point2(pts1[i].x, pts1[i].y));
        measurements.push_back(gtsam::Point2(pts2[i].x, pts2[i].y));

        try {
            // Use GTSAM DLT triangulation
            gtsam::Point3 triangulated_point =
                gtsam::triangulatePoint3(cameras, measurements, 1e-9, false  // rank_tol, optimize
                );

            // Convert back to Eigen
            Eigen::Vector3d point_camera_frame(triangulated_point.x(), triangulated_point.y(),
                                               triangulated_point.z());

            // Apply cheirality check (point should be in front of both cameras)
            if (point_camera_frame.z() <= 0) {
                continue;
            }

            // // Check second camera
            // Eigen::Vector3d point_in_camera2 = T_2_1 * point_camera_frame;
            // if (point_in_camera2.z() <= 0) {
            //     continue;
            // }
            //
            // // Transform to world coordinates
            // Eigen::Vector3d point_base_link = base_link_T_camera * point_camera_frame;
            // Eigen::Vector3d point_world = kf1.pose.getEigenIsometry() * point_base_link;
            //
            // // Basic sanity check on world coordinates
            // if (std::abs(point_world.x()) > 100 || std::abs(point_world.y()) > 100 ||
            //     std::abs(point_world.z()) > 100) {
            //     continue;
            // }

            triangulated_points_world.push_back(point_camera_frame);

        } catch (const std::exception& e) {
            LOG(INFO) << "GTSAM triangulation failed for point " << i << ": " << e.what();
            continue;
        }
    }

    LOG(INFO) << "GTSAM triangulation results:";
    LOG(INFO) << "Input points: " << pts1.size();
    LOG(INFO) << "Successfully triangulated: " << triangulated_points_world.size();
    LOG(INFO) << "Success rate: " << std::fixed << std::setprecision(1)
              << (100.0 * triangulated_points_world.size() / pts1.size()) << "%";

    return triangulated_points_world.size() > 0;
}

// NEW API: Map keypoint triangulation using GTSAM
bool GtsamReconstruct::triangulateFromMapKeypoint(const core::types::Keypoint& map_keypoint,
                                                  const core::storage::MapStore& map_store,
                                                  const stf::TransformTree& tft,
                                                  Eigen::Vector3d& triangulated_point_world,
                                                  const std::string& base_link_frame_id) {
    if (map_keypoint.locations.size() < 2) {
        LOG(ERROR) << "Map keypoint " << map_keypoint.id() << " has less than 2 observations ("
                   << map_keypoint.locations.size() << "), cannot triangulate";
        return false;
    }

    LOG(INFO) << "GTSAM triangulating map keypoint " << map_keypoint.id() << " from "
              << map_keypoint.locations.size() << " observations";

    // Use common utility to build observation data
    std::vector<ObservationData> observations = triangulation_utils::buildObservationData(
        map_keypoint, map_store, tft, base_link_frame_id);

    if (observations.size() < 2) {
        LOG(ERROR) << "Only " << observations.size()
                   << " valid observations available for GTSAM triangulation";
        return false;
    }

    LOG(INFO) << "Using " << observations.size() << " observations for GTSAM triangulation";

    // Perform triangulation
    bool success = false;
    if (observations.size() == 2) {
        // Use GTSAM two-view triangulation method
        success = triangulateFromTwoObservationsGtsam(observations[0], observations[1],
                                                     triangulated_point_world);
    } else {
        // Use GTSAM multi-view triangulation method
        success = triangulateFromMultipleObservationsGtsam(observations, triangulated_point_world);
    }

    // DEBUG: Call ray-based debug visualization (works for both successful and failed triangulations)
    static std::set<uint32_t> debug_generated_keypoints;
    
    // Generate debug visualization for each unique map keypoint (avoid duplicates)
    // This works even if triangulation failed since it's purely ray-based
    if (debug_generated_keypoints.find(map_keypoint.id()) == debug_generated_keypoints.end()) {
        // Use zero vector for failed triangulations (not used in ray-based method anyway)
        Eigen::Vector3d debug_point = success ? triangulated_point_world : Eigen::Vector3d::Zero();
        
        triangulation_utils::debugUnprojectReproject(map_keypoint, observations, debug_point, base_link_frame_id);
        debug_generated_keypoints.insert(map_keypoint.id());
        
        LOG(INFO) << "DEBUG: Generated GTSAM ray-based visualization for map keypoint " << map_keypoint.id() 
                  << " (triangulation " << (success ? "successful" : "failed") 
                  << ", total generated: " << debug_generated_keypoints.size() << ")";
    }

    return success;
}

bool GtsamReconstruct::triangulateFromTwoObservationsGtsam(const ObservationData& obs1,
                                                          const ObservationData& obs2,
                                                          Eigen::Vector3d& triangulated_point_world) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // Convert camera calibration matrices to GTSAM format
        gtsam::Cal3_S2 gtsam_K1(obs1.K(0, 0), obs1.K(1, 1), 0.0, obs1.K(0, 2), obs1.K(1, 2));
        gtsam::Cal3_S2 gtsam_K2(obs2.K(0, 0), obs2.K(1, 1), 0.0, obs2.K(0, 2), obs2.K(1, 2));

        // Convert camera poses to GTSAM format
        gtsam::Pose3 pose1 = eigenToGtsamPose3(obs1.camera_pose_in_odom);
        gtsam::Pose3 pose2 = eigenToGtsamPose3(obs2.camera_pose_in_odom);

        // Create GTSAM cameras
        gtsam::PinholeCamera<gtsam::Cal3_S2> camera1(pose1, gtsam_K1);
        gtsam::PinholeCamera<gtsam::Cal3_S2> camera2(pose2, gtsam_K2);
        gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>> cameras;
        cameras.push_back(camera1);
        cameras.push_back(camera2);

        // Create GTSAM measurements
        gtsam::Point2Vector measurements;
        measurements.push_back(gtsam::Point2(obs1.pixel.x, obs1.pixel.y));
        measurements.push_back(gtsam::Point2(obs2.pixel.x, obs2.pixel.y));

        // GTSAM-specific validation parameters
        const double gtsam_rank_tolerance = 1e-9;  // Numerical tolerance for rank check
        const bool gtsam_optimize = false;         // Use DLT instead of optimization
        const double max_reprojection_error = 5.0; // Max allowed reprojection error (pixels)
        const double max_distance_from_cameras = 100.0; // Max triangulated point distance (meters)
        
        // Use GTSAM DLT triangulation
        gtsam::Point3 triangulated_point = gtsam::triangulatePoint3(cameras, measurements, gtsam_rank_tolerance, gtsam_optimize);

        // Convert back to Eigen
        triangulated_point_world = Eigen::Vector3d(
            triangulated_point.x(), triangulated_point.y(), triangulated_point.z());

        // GTSAM-specific enhanced validation checks
        
        // 1. Distance from cameras check
        double dist_to_cam1 = (obs1.camera_pose_in_odom.translation() - triangulated_point_world).norm();
        double dist_to_cam2 = (obs2.camera_pose_in_odom.translation() - triangulated_point_world).norm();
        if (std::max(dist_to_cam1, dist_to_cam2) > max_distance_from_cameras) {
            LOG(WARNING) << "GTSAM: Point too far from cameras (" << std::max(dist_to_cam1, dist_to_cam2) << "m > " << max_distance_from_cameras << "m)";
            return false;
        }
        
        // 2. Reprojection error validation  
        auto checkReprojectionError = [&](const gtsam::PinholeCamera<gtsam::Cal3_S2>& camera, const gtsam::Point2& measurement) -> double {
            try {
                gtsam::Point2 reprojected = camera.project(triangulated_point);
                return (reprojected - measurement).norm();
            } catch (const std::exception& e) {
                return std::numeric_limits<double>::max(); // Return max error if projection fails
            }
        };
        
        double error1 = checkReprojectionError(camera1, gtsam::Point2(obs1.pixel.x, obs1.pixel.y));
        double error2 = checkReprojectionError(camera2, gtsam::Point2(obs2.pixel.x, obs2.pixel.y));
        double max_error = std::max(error1, error2);
        
        if (max_error > max_reprojection_error) {
            LOG(WARNING) << "GTSAM: High reprojection error (" << max_error << "px > " << max_reprojection_error << "px)";
            return false;
        }
        
        LOG(INFO) << "GTSAM validation: max_dist=" << std::max(dist_to_cam1, dist_to_cam2) << "m, max_reproj_err=" << max_error << "px";

        // 3. Standard cheirality checks: ensure point is in front of both cameras
        Eigen::Vector3d point_camera1 = obs1.camera_pose_in_odom.inverse() * triangulated_point_world;
        if (point_camera1.z() <= 0) {
            LOG(WARNING) << "GTSAM: Point behind first camera";
            return false;
        }

        Eigen::Vector3d point_camera2 = obs2.camera_pose_in_odom.inverse() * triangulated_point_world;
        if (point_camera2.z() <= 0) {
            LOG(WARNING) << "GTSAM: Point behind second camera";
            return false;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        LOG(INFO) << "GTSAM two-view triangulation successful: " << triangulated_point_world.transpose() 
                  << " (time: " << duration.count() << "μs)";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "GTSAM two-view triangulation failed: " << e.what();
        return false;
    }
}

bool GtsamReconstruct::triangulateFromMultipleObservationsGtsam(
    const std::vector<ObservationData>& observations, Eigen::Vector3d& triangulated_point_world) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    try {
        // Build GTSAM cameras and measurements
        gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>> cameras;
        gtsam::Point2Vector measurements;

        for (const auto& obs : observations) {
            // Convert camera calibration to GTSAM format
            gtsam::Cal3_S2 gtsam_K(obs.K(0, 0), obs.K(1, 1), 0.0, obs.K(0, 2), obs.K(1, 2));

            // Convert camera pose to GTSAM format
            gtsam::Pose3 pose = eigenToGtsamPose3(obs.camera_pose_in_odom);

            // Create GTSAM camera
            gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pose, gtsam_K);
            cameras.push_back(camera);

            // Add measurement
            measurements.push_back(gtsam::Point2(obs.pixel.x, obs.pixel.y));
        }

        // GTSAM-specific validation parameters
        const double gtsam_rank_tolerance = 1e-9;  // Numerical tolerance for rank check
        const bool gtsam_optimize = false;         // Use DLT instead of optimization
        const double max_reprojection_error = 5.0; // Max allowed reprojection error (pixels)
        const double max_distance_from_cameras = 100.0; // Max triangulated point distance (meters)

        // Use GTSAM DLT triangulation
        gtsam::Point3 triangulated_point = gtsam::triangulatePoint3(cameras, measurements, gtsam_rank_tolerance, gtsam_optimize);

        // Convert back to Eigen
        triangulated_point_world = Eigen::Vector3d(
            triangulated_point.x(), triangulated_point.y(), triangulated_point.z());

        // GTSAM-specific enhanced validation checks for multi-view
        
        // 1. Distance from cameras check
        double max_distance_to_cameras = 0.0;
        for (const auto& obs : observations) {
            double dist = (obs.camera_pose_in_odom.translation() - triangulated_point_world).norm();
            max_distance_to_cameras = std::max(max_distance_to_cameras, dist);
        }
        if (max_distance_to_cameras > max_distance_from_cameras) {
            LOG(WARNING) << "GTSAM multi-view: Point too far from cameras (" << max_distance_to_cameras << "m > " << max_distance_from_cameras << "m)";
            return false;
        }
        
        // 2. Reprojection error validation for all views
        double max_reprojection_err = 0.0;
        for (size_t i = 0; i < cameras.size(); ++i) {
            try {
                gtsam::Point2 reprojected = cameras[i].project(triangulated_point);
                double error = (reprojected - measurements[i]).norm();
                max_reprojection_err = std::max(max_reprojection_err, error);
            } catch (const std::exception& e) {
                LOG(WARNING) << "GTSAM multi-view: Reprojection failed for camera " << i;
                return false;
            }
        }
        
        if (max_reprojection_err > max_reprojection_error) {
            LOG(WARNING) << "GTSAM multi-view: High reprojection error (" << max_reprojection_err << "px > " << max_reprojection_error << "px)";
            return false;
        }
        
        LOG(INFO) << "GTSAM multi-view validation: max_dist=" << max_distance_to_cameras << "m, max_reproj_err=" << max_reprojection_err << "px";

        // 3. Standard cheirality checks for all cameras
        int valid_cameras = 0;
        for (const auto& obs : observations) {
            Eigen::Vector3d point_in_camera =
                obs.camera_pose_in_odom.inverse() * triangulated_point_world;
            if (point_in_camera.z() > 0) {
                valid_cameras++;
            }
        }

        double valid_ratio = static_cast<double>(valid_cameras) / observations.size();
        if (valid_ratio < 0.5) {
            LOG(WARNING) << "GTSAM multi-view triangulation: only " << valid_cameras << "/"
                         << observations.size() << " cameras see point in front";
            return false;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        LOG(INFO) << "GTSAM multi-view triangulation successful with " << observations.size()
                  << " observations: " << triangulated_point_world.transpose() 
                  << " (time: " << duration.count() << "μs)";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "GTSAM multi-view triangulation failed: " << e.what();
        return false;
    }
}

}  // namespace image

}  // namespace tracking

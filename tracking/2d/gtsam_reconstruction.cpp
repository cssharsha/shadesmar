#include "gtsam_reconstruction.hpp"
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/triangulation.h>
#include <logging/logging.hpp>
#include <stf/transform_utils.hpp>

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

}  // namespace image

}  // namespace tracking

#include <2d/reconstruction.hpp>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <logging/logging.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <set>
#include <stf/transform_utils.hpp>
#include <string>
#include <vector>

namespace tracking {
namespace image {

void printKeyframe(const core::types::KeyFrame& kf) {
    LOG(INFO) << "Keyframe: " << kf.id << ":\nPose:\n"
              << kf.pose.position.transpose() << "\n"
              << kf.pose.orientation.x() << "," << kf.pose.orientation.y() << ","
              << kf.pose.orientation.z() << "," << kf.pose.orientation.w() << ","
              << "\nCamera info:\n"
              << kf.camera_info.value().getKInEigen();
    if (!kf.color_data.value().data.empty())
        cv::imwrite("/data/robot/test_image_" + std::to_string(kf.id) + ".png",
                    kf.color_data.value().data);
}

// bool Reconstruct::triangulateUsingOCV(
//     const core::types::KeyFrame& kf1, const core::types::KeyFrame& kf2,
//     const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2,
//     const stf::TransformTree& tft, const Eigen::Isometry3d& essentialMatrix,
//     std::vector<Eigen::Vector3d>& triangulated_points_world, bool use_essential_mat,
//     const std::string& base_link_frame_id) {}
bool Reconstruct::triangulate(const core::types::KeyFrame& kf1, const core::types::KeyFrame& kf2,
                              const std::vector<cv::Point2f>& pts1,
                              const std::vector<cv::Point2f>& pts2, const stf::TransformTree& tft,
                              const Eigen::Isometry3d& essentialMatrix,
                              std::vector<Eigen::Vector3d>& triangulated_points_world,
                              bool use_essential_mat, const std::string& base_link_frame_id) {
    if (pts1.size() != pts2.size()) {
        LOG(ERROR) << "Should have the same number of points for triangulation";
        return false;
    }
    printKeyframe(kf1);
    printKeyframe(kf2);

    auto tfrel = stf::getRelativeWithBaseLink(kf1, kf2, tft, base_link_frame_id);
    LOG(INFO) << "Relative transform:[" << tfrel.translation().transpose() << "]["
              << stf::getRPY(tfrel).transpose();
    LOG(INFO) << "Keypoints: " << pts1.size() << ", " << pts2.size();
    uint32_t printp = std::ceil(pts1.size() / 2);
    LOG(INFO) << "Point 1: " << pts1.at(printp);
    LOG(INFO) << "Point 2: " << pts2.at(printp);

    // Get base_link to camera transform
    Eigen::Isometry3d base_link_T_camera;
    bool has_camera_transform = false;
    try {
        auto transform_result =
            tft.getTransform(base_link_frame_id, kf1.color_data.value().frame_id);
        base_link_T_camera = transform_result.transform;
        has_camera_transform = true;
    } catch (const std::exception& e) {
        LOG(WARNING) << "Could not get camera transform, using identity: " << e.what();
        base_link_T_camera = Eigen::Isometry3d::Identity();
        has_camera_transform = false;
    }

    // Try to get transform from TF tree first (default robot case)
    Eigen::Isometry3d T_2_1;
    bool using_tf_transform = false;

    if (!use_essential_mat) {
        try {
            // T_2_1 = stf::getRelativeWithBaseLink(kf2, kf1, tft, base_link_frame_id);
            // Use the base_link_T_camera already calculated above
            // (no additional code needed here)

            // Calculate camera poses in odom frame
            Eigen::Isometry3d odom_T_camera1 = kf1.pose.getEigenIsometry() * base_link_T_camera;
            Eigen::Isometry3d odom_T_camera2 = kf2.pose.getEigenIsometry() * base_link_T_camera;

            // Relative camera transform for triangulation (camera2_T_camera1)
            T_2_1 = odom_T_camera2.inverse() * odom_T_camera1;
            using_tf_transform = true;
            LOG(INFO) << "Using TF tree transform for triangulation";
        } catch (const std::exception& e) {
            LOG(WARNING) << "TF tree transform failed, falling back to essential matrix: "
                         << e.what();
            T_2_1 = essentialMatrix;
            using_tf_transform = false;
        }
    } else {
        T_2_1 = essentialMatrix;
        using_tf_transform = false;
        LOG(INFO) << "Using essential matrix transform for triangulation";
        LOG(INFO) << "[" << essentialMatrix.translation().transpose() << "]["
                  << stf::getRPY(essentialMatrix).transpose();
    }

    // Log which transform we're using
    auto odomTrans = T_2_1;  // Keep variable name for compatibility
    if (using_tf_transform) {
        LOG(INFO) << "TF-based transform: rotation: [" << stf::getRPY(odomTrans).transpose()
                  << "], translation: [" << odomTrans.translation().transpose() << "]";
        LOG(INFO) << "Not usong but Essential matrix: rotation: ["
                  << stf::getRPY(essentialMatrix).transpose() << "], translation: ["
                  << essentialMatrix.translation().transpose() << "]";
    } else {
        LOG(INFO) << "Essential matrix: rotation: [" << stf::getRPY(essentialMatrix).transpose()
                  << "], translation: [" << essentialMatrix.translation().transpose() << "]";
    }

    // Write relative transform T_2_1 to CSV file
    std::ofstream reconstruct_csv("/data/robot/reconstruct.csv", std::ios::app);
    if (reconstruct_csv.is_open()) {
        // Write header if file is empty (first write)
        reconstruct_csv.seekp(0, std::ios::end);
        if (reconstruct_csv.tellp() == 0) {
            reconstruct_csv << "x,y,z,r,p,y,kf1_id,kf2_id,method\n";
        }

        // Get translation and rotation from T_2_1
        Eigen::Vector3d translation = T_2_1.translation();
        Eigen::Vector3d rpy = stf::getRPY(T_2_1);

        // Write transform data: x,y,z,r,p,y,kf1_id,kf2_id,method
        reconstruct_csv << std::fixed << std::setprecision(6) << translation.x() << ","
                        << translation.y() << "," << translation.z() << "," << rpy.x() << ","
                        << rpy.y() << "," << rpy.z() << "," << kf1.id << "," << kf2.id << ","
                        << (using_tf_transform ? "TF" : "Essential") << "\n";
        reconstruct_csv.close();
    } else {
        LOG(WARNING) << "Failed to open /data/robot/reconstruct.csv for writing";
    }

    auto k1 = kf1.camera_info.value().k;
    std::cout << "cam mat vec K: ";
    for (auto stuff : k1) {
        std::cout << stuff << ", ";
    }
    std::cout << std::endl;
    auto K1 = kf1.camera_info.value().getKInEigen();
    auto K2 = kf2.camera_info.value().getKInEigen();

    LOG(INFO) << "Camera info matrices:\n" << K1 << "\n==========\n" << K2;

    if (std::abs(K1.determinant()) <= 1e-9 || std::abs(K2.determinant()) <= 1e-9) {
        LOG(ERROR) << "Unable to invert K - determinant too small";
        return false;
    }

    Eigen::Matrix<double, 3, 4> P1_norm, P2_norm;
    P1_norm.setZero();
    P1_norm.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    LOG(INFO) << "P1_norm:\n" << P1_norm;

    P2_norm.block<3, 3>(0, 0) = T_2_1.rotation();
    P2_norm.col(3) = T_2_1.translation();

    LOG(INFO) << "P2_norm:\n" << P2_norm;

    std::ofstream p3d1("/data/robot/bags/house11/matches/3d_points_prev_" + std::to_string(kf1.id) +
                       ".csv");
    std::ofstream p3d2("/data/robot/bags/house11/matches/3d_points_cur_" + std::to_string(kf2.id) +
                       ".csv");
    std::ofstream p3d3("/data/robot/bags/house11/matches/3d_points_world_" +
                       std::to_string(kf1.id) + ".csv");
    p3d1 << "x y z" << "\n";
    p3d2 << "x y z" << "\n";
    p3d3 << "x y z" << "\n";
    // Remove unused lambda - transform already calculated above
    for (uint32_t i = 0; i < pts1.size(); ++i) {
        const Eigen::Vector2d p1(pts1[i].x, pts1[i].y);
        const Eigen::Vector2d p2(pts2[i].x, pts2[i].y);

        Eigen::Vector2d normalized_p1, normalized_p2;

        // Normalize image points (no coordinate transform here)
        Eigen::Vector3d p1_h(p1.x(), p1.y(), 1.0);
        LOG(INFO) << "Point 1 homo: " << p1_h.transpose();
        Eigen::Vector3d p1_norm_h = K1.inverse() * p1_h;
        LOG(INFO) << "Point 1 norm homo: " << p1_norm_h.transpose();
        normalized_p1 =
            Eigen::Vector2d(p1_norm_h.x() / p1_norm_h.z(), p1_norm_h.y() / p1_norm_h.z());
        LOG(INFO) << "Point 1 norm: " << normalized_p1.transpose();

        Eigen::Vector3d p2_h(p2.x(), p2.y(), 1.0);
        LOG(INFO) << "Point 2 homo: " << p2_h.transpose();
        Eigen::Vector3d p2_norm_h = K2.inverse() * p2_h;
        LOG(INFO) << "Point 2 norm homo: " << p2_norm_h.transpose();
        normalized_p2 =
            Eigen::Vector2d(p2_norm_h.x() / p2_norm_h.z(), p2_norm_h.y() / p2_norm_h.z());
        LOG(INFO) << "Point 2 norm: " << normalized_p2.transpose();

        Eigen::Matrix<double, 4, 4> A;
        A.row(0) = normalized_p1.x() * P1_norm.row(2) - P1_norm.row(0);
        A.row(1) = normalized_p1.y() * P1_norm.row(2) - P1_norm.row(1);
        A.row(2) = normalized_p2.x() * P2_norm.row(2) - P2_norm.row(0);
        A.row(3) = normalized_p2.y() * P2_norm.row(2) - P2_norm.row(1);
        LOG(INFO) << "A:\n" << A;

        Eigen::JacobiSVD<Eigen::Matrix<double, 4, 4>> svd(A, Eigen::ComputeFullV);
        LOG(INFO) << "svd singular values:\n" << svd.singularValues();
        Eigen::Vector4d X_h = svd.matrixV().col(3);  // Last column of V

        Eigen::Vector3d current_point3D_world;
        LOG(INFO) << "Point homo " << i << ": " << X_h.transpose();
        if (std::abs(X_h(3)) > 1e-9) {  // Check W component
            Eigen::Vector3d current_point3D_in_Kf1_camera_frame;
            current_point3D_in_Kf1_camera_frame = X_h.head<3>() / X_h(3);
            p3d1 << current_point3D_in_Kf1_camera_frame.transpose() << "\n";

            LOG(INFO) << "kf1.pose translation: " << kf1.pose.position.transpose();
            LOG(INFO) << "kf1.pose rotation (RPY): "
                      << stf::getRPY(kf1.pose.getEigenIsometry()).transpose();
            LOG(INFO) << "Point in camera frame: "
                      << current_point3D_in_Kf1_camera_frame.transpose();

            // Transform point from camera1 frame to odom frame
            if (has_camera_transform) {
                // Standard robot case: point_camera1 → base_link → odom
                Eigen::Isometry3d odom_T_camera1 = kf1.pose.getEigenIsometry() * base_link_T_camera;
                current_point3D_world = odom_T_camera1 * current_point3D_in_Kf1_camera_frame;
                LOG(INFO) << "Applied robot transformation: odom_T_base_link * base_link_T_camera "
                             "* point_camera";
            } else {
                // Visual odometry case: point_camera1 → odom directly
                current_point3D_world =
                    kf1.pose.getEigenIsometry() * current_point3D_in_Kf1_camera_frame;
                LOG(INFO) << "Applied direct camera transformation: odom_T_camera * point_camera";
            }

            LOG(INFO) << "Final point in world frame: " << current_point3D_world.transpose();

            p3d3 << current_point3D_world.transpose() << "\n";

            if (current_point3D_in_Kf1_camera_frame.z() <= 0) {  // Check in front of first camera
                continue;
            }

            // Cheirality check for the second camera
            Eigen::Vector3d point_in_kf2_camera_frame = T_2_1 * current_point3D_in_Kf1_camera_frame;
            if (point_in_kf2_camera_frame.z() <= 0) {
                continue;
            }

            // Optional: Filter points that are too far or behind the world origin (if
            // meaningful)
            if (current_point3D_world.z() <= 0 || std::abs(current_point3D_world.x()) > 10 ||
                std::abs(current_point3D_world.y()) > 10 ||
                std::abs(current_point3D_world.z()) > 10) {
                // This check might be too restrictive or not general enough.
                // Consider if these world-frame checks are necessary for your application.
                // If odom is world, current_point3D_world.z() <=0 might be fine if camera can
                // look up/down.
                continue;
            }
        } else {
            continue;
        }
        triangulated_points_world.push_back(current_point3D_world);
    }
    p3d1.close();
    p3d2.close();
    p3d3.close();

    LOG(INFO) << "Total number of triangulated points: " << triangulated_points_world.size();

    // Add diagnostic summary
    LOG(INFO) << " TRIANGULATION results:";
    LOG(INFO) << "Input points: " << pts1.size();
    LOG(INFO) << "Successfully triangulated: " << triangulated_points_world.size();
    LOG(INFO) << "Success rate: " << std::fixed << std::setprecision(1)
              << (100.0 * triangulated_points_world.size() / pts1.size()) << "%";
    LOG(INFO) << "Transform method: " << (using_tf_transform ? "TF Tree" : "Essential Matrix");
    LOG(INFO) << "Camera transform: " << (has_camera_transform ? "Available" : "Identity fallback");
    LOG(INFO) << "Keyframe poses: kf1=" << kf1.pose.frame_id << ", kf2=" << kf2.pose.frame_id;

    // Smart frame detection summary (detect from first valid point)
    if (!triangulated_points_world.empty()) {
        bool poses_are_camera_poses = false;
        if (kf1.pose.frame_id.find("camera") != std::string::npos ||
            kf1.pose.frame_id.find("optical") != std::string::npos ||
            kf1.pose.frame_id == kf1.color_data.value().frame_id ||
            (!has_camera_transform && use_essential_mat)) {
            poses_are_camera_poses = true;
        }
        LOG(INFO) << "Frame detection: "
                  << (poses_are_camera_poses ? "CAMERA poses" : "BASE_LINK poses");
        LOG(INFO) << "Expected output frame: reference frame (via "
                  << (poses_are_camera_poses ? "direct camera transform" : "robot transform chain")
                  << ")";
    }
    LOG(INFO) << "============================";

    return triangulated_points_world.size();
}

// Use common debug function from triangulation_utils
// (moved to triangulation_common.cpp)

bool Reconstruct::triangulateFromMapKeypoint(const core::types::Keypoint& map_keypoint,
                                             const core::storage::MapStore& map_store,
                                             const stf::TransformTree& tft,
                                             Eigen::Vector3d& triangulated_point_world,
                                             const std::string& base_link_frame_id) {
    if (map_keypoint.locations.size() < 2) {
        LOG(ERROR) << "Map keypoint " << map_keypoint.id() << " has less than 2 observations ("
                   << map_keypoint.locations.size() << "), cannot triangulate";
        return false;
    }

    LOG(INFO) << "Triangulating map keypoint " << map_keypoint.id() << " from "
              << map_keypoint.locations.size() << " observations";

    // Use common utility to build observation data
    std::vector<ObservationData> observations = triangulation_utils::buildObservationData(
        map_keypoint, map_store, tft, base_link_frame_id);

    if (observations.size() < 2) {
        LOG(ERROR) << "Only " << observations.size()
                   << " valid observations available for triangulation";
        return false;
    }

    LOG(INFO) << "Using " << observations.size() << " observations for triangulation";

    // Perform triangulation
    bool success = false;
    if (observations.size() == 2) {
        // Use existing two-view triangulation method
        success = triangulateFromTwoObservations(observations[0], observations[1],
                                                 triangulated_point_world);
    } else {
        // Use multi-view triangulation method
        success = triangulateFromMultipleObservations(observations, triangulated_point_world);
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
        
        LOG(INFO) << "DEBUG: Generated ray-based visualization for map keypoint " << map_keypoint.id() 
                  << " (triangulation " << (success ? "successful" : "failed") 
                  << ", total generated: " << debug_generated_keypoints.size() << ")";
    }

    return success;
}

bool Reconstruct::triangulateFromTwoObservations(const ObservationData& obs1,
                                                 const ObservationData& obs2,
                                                 Eigen::Vector3d& triangulated_point_world) {
    // CORRECTED: Use full projection matrices in world frame for DLT triangulation
    // P = K * [R|t] where [R|t] transforms from world to camera
    
    // Build full projection matrices (world to image)
    Eigen::Matrix<double, 3, 4> P1, P2;
    
    // For camera 1: P1 = K1 * [R1|t1] where [R1|t1] = world_T_camera1.inverse()
    Eigen::Isometry3d world_T_camera1_inv = obs1.camera_pose_in_odom.inverse();
    P1.block<3, 3>(0, 0) = obs1.K * world_T_camera1_inv.rotation();
    P1.col(3) = obs1.K * world_T_camera1_inv.translation();
    
    // For camera 2: P2 = K2 * [R2|t2] where [R2|t2] = world_T_camera2.inverse()
    Eigen::Isometry3d world_T_camera2_inv = obs2.camera_pose_in_odom.inverse();
    P2.block<3, 3>(0, 0) = obs2.K * world_T_camera2_inv.rotation();
    P2.col(3) = obs2.K * world_T_camera2_inv.translation();

    // Image points (no normalization needed for full projection matrices)
    Eigen::Vector2d p1(obs1.pixel.x, obs1.pixel.y);
    Eigen::Vector2d p2(obs2.pixel.x, obs2.pixel.y);

    // DLT triangulation: Build linear system A * X = 0
    Eigen::Matrix<double, 4, 4> A;
    A.row(0) = p1.x() * P1.row(2) - P1.row(0);
    A.row(1) = p1.y() * P1.row(2) - P1.row(1);
    A.row(2) = p2.x() * P2.row(2) - P2.row(0);
    A.row(3) = p2.y() * P2.row(2) - P2.row(1);

    Eigen::JacobiSVD<Eigen::Matrix<double, 4, 4>> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4d X_h = svd.matrixV().col(3);

    if (std::abs(X_h(3)) <= 1e-9) {
        LOG(ERROR) << "Triangulation failed: point at infinity";
        return false;
    }

    // Get 3D point in world frame (homogeneous to Euclidean)
    triangulated_point_world = X_h.head<3>() / X_h(3);

    // Cheirality checks: ensure point is in front of both cameras
    Eigen::Vector3d point_camera1 = obs1.camera_pose_in_odom.inverse() * triangulated_point_world;
    if (point_camera1.z() <= 0) {
        LOG(WARNING) << "Point behind first camera";
        return false;
    }

    Eigen::Vector3d point_camera2 = obs2.camera_pose_in_odom.inverse() * triangulated_point_world;
    if (point_camera2.z() <= 0) {
        LOG(WARNING) << "Point behind second camera";
        return false;
    }

    LOG(INFO) << "Two-view triangulation successful: " << triangulated_point_world.transpose();
    return true;
}

bool Reconstruct::triangulateFromMultipleObservations(
    const std::vector<ObservationData>& observations, Eigen::Vector3d& triangulated_point_world) {
    // CORRECTED: Use DLT method with full projection matrices
    // Build the system: A * X = 0, where X = [x, y, z, w]^T is the homogeneous 3D point

    int num_observations = observations.size();
    Eigen::MatrixXd A(2 * num_observations, 4);

    for (int i = 0; i < num_observations; ++i) {
        const auto& obs = observations[i];

        // Image point (no normalization needed for full projection matrix)
        Eigen::Vector2d pixel(obs.pixel.x, obs.pixel.y);

        // CORRECTED: Build full projection matrix P = K * [R|t]
        // where [R|t] transforms from world to camera
        Eigen::Matrix<double, 3, 4> P;
        Eigen::Isometry3d world_T_camera_inv = obs.camera_pose_in_odom.inverse();
        P.block<3, 3>(0, 0) = obs.K * world_T_camera_inv.rotation();
        P.col(3) = obs.K * world_T_camera_inv.translation();

        // Add two rows to the system matrix
        A.row(2 * i) = pixel.x() * P.row(2) - P.row(0);
        A.row(2 * i + 1) = pixel.y() * P.row(2) - P.row(1);
    }

    // Solve using SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4d X_h = svd.matrixV().col(3);

    if (std::abs(X_h(3)) <= 1e-9) {
        LOG(ERROR) << "Multi-view triangulation failed: point at infinity";
        return false;
    }

    // Convert from homogeneous coordinates to world frame
    triangulated_point_world = X_h.head<3>() / X_h(3);

    // Perform cheirality checks for all cameras
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
        LOG(WARNING) << "Multi-view triangulation: only " << valid_cameras << "/"
                     << observations.size() << " cameras see point in front";
        return false;
    }

    LOG(INFO) << "Multi-view triangulation successful with " << observations.size()
              << " observations: " << triangulated_point_world.transpose();
    return true;
}

}  // namespace image

}  // namespace tracking

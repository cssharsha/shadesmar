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

// Ray-based debug function for multi-keyframe observation consistency visualization
void debugUnprojectReproject(const core::types::Keypoint& map_keypoint,
                             const std::vector<ObservationData>& observations,
                             const Eigen::Vector3d& triangulated_point_world,
                             const std::string& reference_frame_id) {
    LOG(INFO) << "=== RAY-BASED OBSERVATION CONSISTENCY VISUALIZATION ===";
    LOG(INFO) << "Map keypoint " << map_keypoint.id() << " with " << observations.size() 
              << " observations (triangulation not required)";

    if (observations.size() < 2) {
        LOG(WARNING) << "Need at least 2 observations for ray consistency check";
        return;
    }

    // Collect valid keyframes with their ray intersection data
    struct KeyframeVisualizationData {
        cv::Mat image;
        cv::Point2f original_pixel;
        std::vector<cv::Point2f> ray_intersections;
        std::vector<double> ray_errors;
        std::vector<uint64_t> source_keyframe_ids;
        uint64_t keyframe_id;
        std::string camera_frame_id;
        bool valid;
    };
    
    std::vector<KeyframeVisualizationData> valid_keyframes;
    
    // Process each keyframe as the "target" for ray projections
    for (size_t target_idx = 0; target_idx < observations.size(); ++target_idx) {
        const auto& target_obs = observations[target_idx];
        
        KeyframeVisualizationData kf_data;
        kf_data.keyframe_id = target_obs.keyframe->id;
        kf_data.camera_frame_id = target_obs.camera_frame_id;
        kf_data.original_pixel = target_obs.pixel;
        kf_data.valid = false;
        
        // Skip if no color image
        if (!target_obs.keyframe->hasColorImage()) {
            LOG(WARNING) << "Target keyframe " << target_obs.keyframe->id << " has no color image, skipping";
            continue;
        }
        
        kf_data.image = target_obs.keyframe->color_data->data.clone();
        
        // Process rays from all OTHER keyframes to this target keyframe
        for (size_t source_idx = 0; source_idx < observations.size(); ++source_idx) {
            if (source_idx == target_idx) continue; // Skip self
            
            const auto& source_obs = observations[source_idx];
            
            try {
                // Step 1: Unproject source pixel to 3D ray in source camera frame
                Eigen::Vector3d source_pixel_homo(source_obs.pixel.x, source_obs.pixel.y, 1.0);
                Eigen::Vector3d source_ray_camera = source_obs.K.inverse() * source_pixel_homo;
                source_ray_camera.normalize(); // Unit ray direction
                
                LOG(INFO) << "Source KF " << source_obs.keyframe->id << " pixel (" 
                          << source_obs.pixel.x << "," << source_obs.pixel.y 
                          << ") → ray: " << source_ray_camera.transpose();
                
                // Step 2: Transform ray from source camera frame to target camera frame
                // Transform chain: source_camera → odom → target_camera
                Eigen::Isometry3d source_camera_T_target_camera = 
                    target_obs.camera_pose_in_odom.inverse() * source_obs.camera_pose_in_odom;
                
                // Transform ray direction (rotation only, no translation for direction)
                Eigen::Vector3d target_ray_camera = source_camera_T_target_camera.rotation() * source_ray_camera;
                
                // Transform ray origin (source camera center in target camera frame)
                Eigen::Vector3d source_camera_center_in_target = source_camera_T_target_camera.translation();
                
                // Step 3: Project ray onto target camera's image plane
                // Ray equation: P(t) = origin + t * direction
                // Image plane: Z = focal_length (approximately, for simplicity we use Z = 1 after normalization)
                
                // Find intersection with Z = 1 plane (normalized camera coordinates)
                if (std::abs(target_ray_camera.z()) < 1e-9) {
                    LOG(WARNING) << "Ray parallel to image plane, skipping";
                    continue;
                }
                
                // Parameter t where ray intersects Z = 1 plane
                double t = (1.0 - source_camera_center_in_target.z()) / target_ray_camera.z();
                
                // 3D intersection point in target camera frame
                Eigen::Vector3d intersection_3d = source_camera_center_in_target + t * target_ray_camera;
                
                // Check if intersection is in front of target camera
                if (intersection_3d.z() <= 0) {
                    LOG(WARNING) << "Ray intersection behind target camera, skipping";
                    continue;
                }
                
                // Step 4: Project to image coordinates
                Eigen::Vector3d projected_homo = target_obs.K * intersection_3d;
                cv::Point2f ray_intersection(projected_homo.x() / projected_homo.z(),
                                           projected_homo.y() / projected_homo.z());
                
                // Calculate error from original pixel
                double ray_error = cv::norm(target_obs.pixel - ray_intersection);
                
                LOG(INFO) << "Ray from KF " << source_obs.keyframe->id << " → KF " << target_obs.keyframe->id 
                          << ": intersection (" << ray_intersection.x << "," << ray_intersection.y 
                          << ") error: " << ray_error << "px";
                
                // Store successful ray intersection
                kf_data.ray_intersections.push_back(ray_intersection);
                kf_data.ray_errors.push_back(ray_error);
                kf_data.source_keyframe_ids.push_back(source_obs.keyframe->id);
                
            } catch (const std::exception& e) {
                LOG(WARNING) << "Failed to project ray from KF " << source_obs.keyframe->id 
                             << " to KF " << target_obs.keyframe->id << ": " << e.what();
                continue;
            }
        }
        
        // Mark as valid if we have at least one successful ray intersection
        if (!kf_data.ray_intersections.empty()) {
            kf_data.valid = true;
            valid_keyframes.push_back(kf_data);
        }
    }
    
    if (valid_keyframes.empty()) {
        LOG(WARNING) << "No valid keyframes with ray intersections for visualization";
        return;
    }
    
    // Auto-calculate grid dimensions
    int num_images = valid_keyframes.size();
    int cols = static_cast<int>(std::ceil(std::sqrt(num_images)));
    int rows = static_cast<int>(std::ceil(static_cast<double>(num_images) / cols));
    
    LOG(INFO) << "Creating " << rows << "x" << cols << " grid for " << num_images << " keyframes";
    
    // Determine target size for each cell (keeping aspect ratio)
    int target_cell_width = 400;  // pixels
    int target_cell_height = 300; // pixels
    
    // Create grid image
    cv::Mat grid_image = cv::Mat::zeros(rows * target_cell_height, cols * target_cell_width, CV_8UC3);
    
    // Process each keyframe
    for (int i = 0; i < valid_keyframes.size(); ++i) {
        const auto& kf_data = valid_keyframes[i];
        
        // Calculate grid position
        int row = i / cols;
        int col = i % cols;
        
        // Resize keyframe image to fit cell
        cv::Mat resized_image;
        cv::resize(kf_data.image, resized_image, cv::Size(target_cell_width, target_cell_height));
        
        // Calculate scaling factors for keypoint coordinates
        double scale_x = static_cast<double>(target_cell_width) / kf_data.image.cols;
        double scale_y = static_cast<double>(target_cell_height) / kf_data.image.rows;
        
        // Scale original pixel coordinates
        cv::Point2f scaled_original(kf_data.original_pixel.x * scale_x, 
                                   kf_data.original_pixel.y * scale_y);
        
        // Draw visualization on resized image
        int circle_radius = std::max(3, static_cast<int>(5 * std::min(scale_x, scale_y)));
        int line_thickness = std::max(1, static_cast<int>(2 * std::min(scale_x, scale_y)));
        
        // Draw original point in green (larger circle)
        cv::circle(resized_image, scaled_original, circle_radius + 1, cv::Scalar(0, 255, 0), line_thickness);
        
        // Draw ray intersections in red and connect with purple lines
        double total_error = 0.0;
        for (size_t j = 0; j < kf_data.ray_intersections.size(); ++j) {
            cv::Point2f scaled_intersection(kf_data.ray_intersections[j].x * scale_x,
                                          kf_data.ray_intersections[j].y * scale_y);
            
            // Draw red circle for ray intersection
            cv::circle(resized_image, scaled_intersection, circle_radius, cv::Scalar(0, 0, 255), line_thickness);
            
            // Draw purple line from original to intersection
            cv::line(resized_image, scaled_original, scaled_intersection, cv::Scalar(255, 0, 255), line_thickness);
            
            total_error += kf_data.ray_errors[j];
        }
        
        double avg_error = total_error / kf_data.ray_intersections.size();
        
        // Add text info (scaled font)
        double font_scale = 0.4 * std::min(scale_x, scale_y);
        int font_thickness = std::max(1, static_cast<int>(1 * std::min(scale_x, scale_y)));
        
        std::string keyframe_text = "KF " + std::to_string(kf_data.keyframe_id);
        cv::putText(resized_image, keyframe_text, cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 
                   font_scale, cv::Scalar(255, 255, 255), font_thickness);
        
        std::string rays_text = "Rays: " + std::to_string(kf_data.ray_intersections.size());
        cv::putText(resized_image, rays_text, cv::Point(5, 40), cv::FONT_HERSHEY_SIMPLEX, 
                   font_scale, cv::Scalar(255, 255, 255), font_thickness);
        
        std::string error_text = "Avg: " + std::to_string(static_cast<int>(avg_error)) + "px";
        cv::putText(resized_image, error_text, cv::Point(5, 60), cv::FONT_HERSHEY_SIMPLEX, 
                   font_scale, cv::Scalar(255, 255, 255), font_thickness);
        
        std::string camera_text = kf_data.camera_frame_id;
        cv::putText(resized_image, camera_text, cv::Point(5, 80), cv::FONT_HERSHEY_SIMPLEX, 
                   font_scale * 0.8, cv::Scalar(200, 200, 200), font_thickness);
        
        // Copy resized image to grid
        cv::Rect cell_rect(col * target_cell_width, row * target_cell_height, 
                          target_cell_width, target_cell_height);
        resized_image.copyTo(grid_image(cell_rect));
    }
    
    // Add overall title
    std::string title = "Map Keypoint " + std::to_string(map_keypoint.id()) + 
                       " - Ray Consistency Across " + std::to_string(valid_keyframes.size()) + " Keyframes";
    cv::putText(grid_image, title, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 
               0.8, cv::Scalar(255, 255, 0), 2);
    
    // Add legend
    cv::putText(grid_image, "Green: Original Pixel | Red: Ray Intersections | Purple: Consistency Lines", 
               cv::Point(10, grid_image.rows - 15), cv::FONT_HERSHEY_SIMPLEX, 
               0.5, cv::Scalar(255, 255, 255), 1);
    
    // Calculate overall statistics
    double overall_total_error = 0.0;
    double overall_max_error = 0.0;
    int total_rays = 0;
    
    for (const auto& kf_data : valid_keyframes) {
        for (double error : kf_data.ray_errors) {
            overall_total_error += error;
            overall_max_error = std::max(overall_max_error, error);
            total_rays++;
        }
    }
    
    double overall_avg_error = total_rays > 0 ? overall_total_error / total_rays : 0.0;
    
    // Add statistics
    std::string stats_text = "Overall: " + std::to_string(total_rays) + " rays, Avg: " + 
                            std::to_string(static_cast<int>(overall_avg_error)) + 
                            "px, Max: " + std::to_string(static_cast<int>(overall_max_error)) + "px";
    cv::putText(grid_image, stats_text, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 
               0.6, cv::Scalar(255, 255, 0), 1);
    
    // Save debug image
    std::string filename = "/data/robot/unproject_" + std::to_string(map_keypoint.id()) + ".png";
    cv::imwrite(filename, grid_image);
    
    LOG(INFO) << "Ray-based consistency visualization saved to " << filename;
    LOG(INFO) << "Overall statistics: " << total_rays << " rays, Avg error " << overall_avg_error 
              << "px, Max error " << overall_max_error << "px across " << valid_keyframes.size() << " keyframes";
    LOG(INFO) << "=== END RAY-BASED VISUALIZATION ===";
}

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

    // Extract keyframes, camera info, and pixel coordinates for all observations
    std::vector<ObservationData> observations;
    observations.reserve(map_keypoint.locations.size());

    // Collect observation data
    for (const auto& location : map_keypoint.locations) {
        auto keyframe = map_store.getKeyFrame(location.keyframe_id);
        if (!keyframe) {
            LOG(WARNING) << "Failed to get keyframe " << location.keyframe_id
                         << " for map keypoint " << map_keypoint.id();
            continue;
        }

        if (!keyframe->camera_info.has_value()) {
            LOG(WARNING) << "Keyframe " << location.keyframe_id
                         << " has no camera info for map keypoint " << map_keypoint.id();
            continue;
        }

        ObservationData obs;
        obs.keyframe = keyframe;
        obs.pixel = cv::Point2f(location.x, location.y);
        obs.camera_frame_id = location.frame_id;
        obs.K = keyframe->camera_info.value().getKInEigen();

        // Get camera pose in odom frame
        try {
            // Get base_link to camera transform
            auto transform_result = tft.getTransform(base_link_frame_id, obs.camera_frame_id);
            Eigen::Isometry3d base_link_T_camera = transform_result.transform;

            // Calculate camera pose in odom frame
            obs.camera_pose_in_odom = keyframe->pose.getEigenIsometry() * base_link_T_camera;

            observations.push_back(obs);

            LOG(INFO) << "Added observation from keyframe " << location.keyframe_id << " at pixel ("
                      << location.x << ", " << location.y << ")";
        } catch (const std::exception& e) {
            LOG(WARNING) << "Failed to get camera transform for frame " << obs.camera_frame_id
                         << ": " << e.what();
            continue;
        }
    }

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
        
        debugUnprojectReproject(map_keypoint, observations, debug_point, base_link_frame_id);
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
    // Calculate relative camera transform
    Eigen::Isometry3d camera2_T_camera1 =
        obs2.camera_pose_in_odom.inverse() * obs1.camera_pose_in_odom;

    // Normalize image points
    Eigen::Vector3d p1_h(obs1.pixel.x, obs1.pixel.y, 1.0);
    Eigen::Vector3d p2_h(obs2.pixel.x, obs2.pixel.y, 1.0);

    Eigen::Vector3d p1_norm_h = obs1.K.inverse() * p1_h;
    Eigen::Vector3d p2_norm_h = obs2.K.inverse() * p2_h;

    Eigen::Vector2d normalized_p1(p1_norm_h.x() / p1_norm_h.z(), p1_norm_h.y() / p1_norm_h.z());
    Eigen::Vector2d normalized_p2(p2_norm_h.x() / p2_norm_h.z(), p2_norm_h.y() / p2_norm_h.z());

    // Set up projection matrices for triangulation
    Eigen::Matrix<double, 3, 4> P1_norm, P2_norm;
    P1_norm.setZero();
    P1_norm.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    P2_norm.block<3, 3>(0, 0) = camera2_T_camera1.rotation();
    P2_norm.col(3) = camera2_T_camera1.translation();

    // DLT triangulation
    Eigen::Matrix<double, 4, 4> A;
    A.row(0) = normalized_p1.x() * P1_norm.row(2) - P1_norm.row(0);
    A.row(1) = normalized_p1.y() * P1_norm.row(2) - P1_norm.row(1);
    A.row(2) = normalized_p2.x() * P2_norm.row(2) - P2_norm.row(0);
    A.row(3) = normalized_p2.y() * P2_norm.row(2) - P2_norm.row(1);

    Eigen::JacobiSVD<Eigen::Matrix<double, 4, 4>> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4d X_h = svd.matrixV().col(3);

    if (std::abs(X_h(3)) <= 1e-9) {
        LOG(ERROR) << "Triangulation failed: point at infinity";
        return false;
    }

    // Get 3D point in camera1 frame
    Eigen::Vector3d point_camera1 = X_h.head<3>() / X_h(3);

    // Cheirality checks
    if (point_camera1.z() <= 0) {
        LOG(WARNING) << "Point behind first camera";
        return false;
    }

    Eigen::Vector3d point_camera2 = camera2_T_camera1 * point_camera1;
    if (point_camera2.z() <= 0) {
        LOG(WARNING) << "Point behind second camera";
        return false;
    }

    // Transform to world frame
    triangulated_point_world = obs1.camera_pose_in_odom * point_camera1;

    LOG(INFO) << "Two-view triangulation successful: " << triangulated_point_world.transpose();
    return true;
}

bool Reconstruct::triangulateFromMultipleObservations(
    const std::vector<ObservationData>& observations, Eigen::Vector3d& triangulated_point_world) {
    // Use DLT method for multiple observations
    // Build the system: A * X = 0, where X = [x, y, z, w]^T is the homogeneous 3D point

    int num_observations = observations.size();
    Eigen::MatrixXd A(2 * num_observations, 4);

    for (int i = 0; i < num_observations; ++i) {
        const auto& obs = observations[i];

        // Normalize image point
        Eigen::Vector3d p_h(obs.pixel.x, obs.pixel.y, 1.0);
        Eigen::Vector3d p_norm_h = obs.K.inverse() * p_h;
        Eigen::Vector2d normalized_p(p_norm_h.x() / p_norm_h.z(), p_norm_h.y() / p_norm_h.z());

        // Build projection matrix for this camera
        Eigen::Matrix<double, 3, 4> P;
        P.block<3, 3>(0, 0) = obs.camera_pose_in_odom.rotation();
        P.col(3) = obs.camera_pose_in_odom.translation();

        // Add two rows to the system matrix
        A.row(2 * i) = normalized_p.x() * P.row(2) - P.row(0);
        A.row(2 * i + 1) = normalized_p.y() * P.row(2) - P.row(1);
    }

    // Solve using SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4d X_h = svd.matrixV().col(3);

    if (std::abs(X_h(3)) <= 1e-9) {
        LOG(ERROR) << "Multi-view triangulation failed: point at infinity";
        return false;
    }

    // Convert from homogeneous coordinates
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

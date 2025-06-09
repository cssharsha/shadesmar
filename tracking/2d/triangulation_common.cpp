#include "triangulation_common.hpp"
#include <cmath>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <logging/logging.hpp>

namespace tracking {
namespace image {
namespace triangulation_utils {

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
    std::string filename = "/data/robot/unproject_gtsam_" + std::to_string(map_keypoint.id()) + ".png";
    cv::imwrite(filename, grid_image);
    
    LOG(INFO) << "Ray-based consistency visualization saved to " << filename;
    LOG(INFO) << "Overall statistics: " << total_rays << " rays, Avg error " << overall_avg_error 
              << "px, Max error " << overall_max_error << "px across " << valid_keyframes.size() << " keyframes";
    LOG(INFO) << "=== END RAY-BASED VISUALIZATION ===";
}

std::vector<ObservationData> buildObservationData(
    const core::types::Keypoint& map_keypoint,
    const core::storage::MapStore& map_store,
    const stf::TransformTree& tft,
    const std::string& base_link_frame_id) {
    
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

        // Get camera pose in odom frame using standard coordinate frame transformation
        try {
            obs.camera_pose_in_odom = computeCameraPoseInOdom(*keyframe, tft, obs.camera_frame_id, base_link_frame_id);
            observations.push_back(obs);

            LOG(INFO) << "Added observation from keyframe " << location.keyframe_id << " at pixel ("
                      << location.x << ", " << location.y << ")";
        } catch (const std::exception& e) {
            LOG(WARNING) << "Failed to get camera transform for frame " << obs.camera_frame_id
                         << ": " << e.what();
            continue;
        }
    }
    
    return observations;
}

Eigen::Isometry3d computeCameraPoseInOdom(
    const core::types::KeyFrame& keyframe,
    const stf::TransformTree& tft,
    const std::string& camera_frame_id,
    const std::string& base_link_frame_id) {
    
    // Get base_link to camera transform
    auto transform_result = tft.getTransform(base_link_frame_id, camera_frame_id);
    Eigen::Isometry3d base_link_T_camera = transform_result.transform;

    // Calculate camera pose in odom frame: odom_T_camera = odom_T_base_link * base_link_T_camera
    return keyframe.pose.getEigenIsometry() * base_link_T_camera;
}

}  // namespace triangulation_utils
}  // namespace image
}  // namespace tracking
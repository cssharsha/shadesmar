#pragma once

#include <fstream>
#include <string>
#include <filesystem>
#include "core/storage/map_store.hpp"

namespace core {
namespace graph {
namespace util {

inline void dumpFactorGraph(const core::storage::MapStore& store, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    // Get data from store
    auto keyframes = store.getAllKeyFrames();
    auto factors = store.getAllFactors();
    auto map_keypoints = store.getAllKeyPoints();

    // Calculate total points: keyframes + map keypoints
    size_t total_points = keyframes.size() + map_keypoints.size();

    // Write VTK header
    file << "# vtk DataFile Version 3.0\n";
    file << "Factor Graph with Map Keypoints and Reprojection Factors\n";
    file << "ASCII\n";
    file << "DATASET POLYDATA\n";

    // Write all points: keyframes first, then map keypoints
    file << "POINTS " << total_points << " float\n";
    
    // Write keyframe positions
    for (const auto& kf : keyframes) {
        file << kf->pose.position.x() << " " << kf->pose.position.y() << " "
             << kf->pose.position.z() << "\n";
    }
    
    // Write map keypoint positions (landmarks)
    for (const auto& keypoint : map_keypoints) {
        if (!keypoint.needs_triangulation && keypoint.position.norm() > 1e-6) {
            file << keypoint.position.x() << " " << keypoint.position.y() << " "
                 << keypoint.position.z() << "\n";
        } else {
            // For untriangulated points, place them at origin or skip
            file << "0.0 0.0 0.0\n";
        }
    }

    // Create mapping from IDs to point indices
    std::map<uint64_t, size_t> keyframe_id_to_index;
    for (size_t i = 0; i < keyframes.size(); ++i) {
        keyframe_id_to_index[keyframes[i]->id] = i;
    }
    
    std::map<uint32_t, size_t> keypoint_id_to_index;
    for (size_t i = 0; i < map_keypoints.size(); ++i) {
        keypoint_id_to_index[map_keypoints[i].id()] = keyframes.size() + i;  // Offset by keyframes
    }

    // Count different types of edges
    int odometry_edges = 0;
    int loop_closure_edges = 0;
    int imu_edges = 0;
    int reprojection_edges = 0;
    
    // Count pose factors
    for (const auto& factor : factors) {
        if (factor.type == proto::FactorType::ODOMETRY) odometry_edges++;
        else if (factor.type == proto::FactorType::LOOP_CLOSURE) loop_closure_edges++;
        else if (factor.type == proto::FactorType::IMU_PREINTEGRATED) imu_edges++;
    }
    
    // Count reprojection factors (keyframe observations of map keypoints)
    for (const auto& keypoint : map_keypoints) {
        if (!keypoint.needs_triangulation && keypoint.position.norm() > 1e-6) {
            for (const auto& location : keypoint.locations) {
                if (keyframe_id_to_index.count(location.keyframe_id)) {
                    reprojection_edges++;
                }
            }
        }
    }
    
    int total_edges = odometry_edges + loop_closure_edges + imu_edges + reprojection_edges;
    
    file << "LINES " << total_edges << " " << total_edges * 3 << "\n";

    // Write pose factor edges (odometry, loop closure, IMU)
    for (const auto& factor : factors) {
        if ((factor.type == proto::FactorType::ODOMETRY ||
             factor.type == proto::FactorType::LOOP_CLOSURE ||
             factor.type == proto::FactorType::IMU_PREINTEGRATED) &&
            factor.connected_nodes.size() >= 2) {
            
            auto it1 = keyframe_id_to_index.find(factor.connected_nodes[0]);
            auto it2 = keyframe_id_to_index.find(factor.connected_nodes[1]);
            
            if (it1 != keyframe_id_to_index.end() && it2 != keyframe_id_to_index.end()) {
                file << "2 " << it1->second << " " << it2->second << "\n";
            }
        }
    }
    
    // Write reprojection factor edges (keyframe to map keypoint observations)
    for (const auto& keypoint : map_keypoints) {
        if (!keypoint.needs_triangulation && keypoint.position.norm() > 1e-6) {
            size_t keypoint_index = keypoint_id_to_index[keypoint.id()];
            
            for (const auto& location : keypoint.locations) {
                auto it = keyframe_id_to_index.find(location.keyframe_id);
                if (it != keyframe_id_to_index.end()) {
                    // Connect keyframe to map keypoint
                    file << "2 " << it->second << " " << keypoint_index << "\n";
                }
            }
        }
    }

    // Write point data to distinguish keyframes from landmarks
    file << "POINT_DATA " << total_points << "\n";
    file << "SCALARS point_type int 1\n";
    file << "LOOKUP_TABLE default\n";
    
    // Keyframes = 0, Map keypoints = 1
    for (size_t i = 0; i < keyframes.size(); ++i) {
        file << "0\n";  // Keyframe
    }
    for (size_t i = 0; i < map_keypoints.size(); ++i) {
        file << "1\n";  // Map keypoint
    }

    // Write edge data to distinguish factor types
    file << "CELL_DATA " << total_edges << "\n";
    file << "SCALARS factor_type int 1\n";
    file << "LOOKUP_TABLE default\n";
    
    // Write pose factor types
    for (const auto& factor : factors) {
        if ((factor.type == proto::FactorType::ODOMETRY ||
             factor.type == proto::FactorType::LOOP_CLOSURE ||
             factor.type == proto::FactorType::IMU_PREINTEGRATED) &&
            factor.connected_nodes.size() >= 2) {
            
            auto it1 = keyframe_id_to_index.find(factor.connected_nodes[0]);
            auto it2 = keyframe_id_to_index.find(factor.connected_nodes[1]);
            
            if (it1 != keyframe_id_to_index.end() && it2 != keyframe_id_to_index.end()) {
                file << static_cast<int>(factor.type) << "\n";
            }
        }
    }
    
    // Write reprojection factor type (use value 10 for reprojection factors)
    for (const auto& keypoint : map_keypoints) {
        if (!keypoint.needs_triangulation && keypoint.position.norm() > 1e-6) {
            for (const auto& location : keypoint.locations) {
                auto it = keyframe_id_to_index.find(location.keyframe_id);
                if (it != keyframe_id_to_index.end()) {
                    file << "10\n";  // Reprojection factor
                }
            }
        }
    }

    file.close();
}

// Utility function to determine the appropriate output directory for graph files
inline std::string getGraphOutputDirectory() {
    // Priority order for output directories:
    // 1. Check if bag directory exists (most likely current dataset)
    // 2. Check if specific dataset directories exist
    // 3. Fall back to default /data/robot
    
    std::vector<std::string> candidate_dirs = {
        "/data/robot/bags/house11",          // Current bag directory
        "/data/robot/bags",                  // Bags parent directory  
        "/data/robot/kitti/data/odom",      // KITTI dataset directory
        "/data/robot"                        // Default fallback
    };
    
    for (const auto& dir : candidate_dirs) {
        if (std::filesystem::exists(dir) && std::filesystem::is_directory(dir)) {
            return dir;
        }
    }
    
    // If none exist, create and return default
    std::filesystem::create_directories("/data/robot");
    return "/data/robot";
}

// Enhanced graph dumping function that uses appropriate output directory
inline void dumpFactorGraphWithAutoPath(const core::storage::MapStore& store, 
                                        const std::string& filename_prefix = "factor_graph") {
    std::string output_dir = getGraphOutputDirectory();
    std::string full_path = output_dir + "/" + filename_prefix + ".vtk";
    
    try {
        dumpFactorGraph(store, full_path);
        // Also create a simple text summary
        std::string summary_path = output_dir + "/" + filename_prefix + "_summary.txt";
        std::ofstream summary(summary_path);
        if (summary.is_open()) {
            auto keyframes = store.getAllKeyFrames();
            auto factors = store.getAllFactors();
            auto map_keypoints = store.getAllKeyPoints();
            
            summary << "Factor Graph Summary\n";
            summary << "===================\n";
            summary << "Output Directory: " << output_dir << "\n";
            summary << "VTK File: " << full_path << "\n\n";
            summary << "Graph Statistics:\n";
            summary << "- Keyframes: " << keyframes.size() << "\n";
            summary << "- Map Keypoints: " << map_keypoints.size() << "\n";
            summary << "- Total Factors: " << factors.size() << "\n\n";
            
            // Count factor types
            std::map<std::string, int> factor_counts;
            for (const auto& factor : factors) {
                switch (factor.type) {
                    case proto::FactorType::ODOMETRY:
                        factor_counts["Odometry"]++;
                        break;
                    case proto::FactorType::LOOP_CLOSURE:
                        factor_counts["Loop Closure"]++;
                        break;
                    case proto::FactorType::IMU_PREINTEGRATED:
                        factor_counts["IMU"]++;
                        break;
                    default:
                        factor_counts["Other"]++;
                        break;
                }
            }
            
            // Count reprojection factors
            int reprojection_count = 0;
            for (const auto& keypoint : map_keypoints) {
                if (!keypoint.needs_triangulation && keypoint.position.norm() > 1e-6) {
                    reprojection_count += keypoint.locations.size();
                }
            }
            factor_counts["Reprojection"] = reprojection_count;
            
            summary << "Factor Type Breakdown:\n";
            for (const auto& [type, count] : factor_counts) {
                summary << "- " << type << ": " << count << "\n";
            }
            
            summary << "\nVisualization Guide:\n";
            summary << "- Blue points: Keyframes (camera poses)\n";
            summary << "- Red points: Map keypoints (landmarks)\n";
            summary << "- Thick lines: Pose factors (odometry/loop closure/IMU)\n";
            summary << "- Thin lines: Reprojection factors (observations)\n";
            summary.close();
        }
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to dump factor graph: " + std::string(e.what()));
    }
}

}  // namespace util
}  // namespace graph
}  // namespace core

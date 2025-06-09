#include <core/storage/map_store.hpp>
#include <core/types/keyframe.hpp>
#include <core/types/factor.hpp>
#include <logging/logging.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>
#include <map>

struct TransformData {
    double x, y, z, r, p, yaw;
    uint64_t kf1_id, kf2_id;
    std::string method;

    TransformData(double x_, double y_, double z_, double r_, double p_, double yaw_,
                  uint64_t kf1_id_, uint64_t kf2_id_, const std::string& method_)
        : x(x_), y(y_), z(z_), r(r_), p(p_), yaw(yaw_),
          kf1_id(kf1_id_), kf2_id(kf2_id_), method(method_) {}
};

struct OdometryTrajectoryPoint {
    uint64_t keyframe_id;
    Eigen::Vector3d position;
    Eigen::Vector3d rotation; // RPY
    double timestamp;

    OdometryTrajectoryPoint(uint64_t id, const Eigen::Vector3d& pos, const Eigen::Vector3d& rot, double ts)
        : keyframe_id(id), position(pos), rotation(rot), timestamp(ts) {}
};

std::vector<TransformData> readReconstructCSV(const std::string& csv_path) {
    std::vector<TransformData> transforms;
    std::ifstream file(csv_path);

    if (!file.is_open()) {
        LOG(ERROR) << "Failed to open CSV file: " << csv_path;
        return transforms;
    }

    std::string line;
    bool first_line = true;

    while (std::getline(file, line)) {
        if (first_line) {
            // Skip header line
            first_line = false;
            continue;
        }

        std::stringstream ss(line);
        std::string item;
        std::vector<std::string> tokens;

        while (std::getline(ss, item, ',')) {
            tokens.push_back(item);
        }

        if (tokens.size() >= 9) {
            try {
                double x = std::stod(tokens[0]);
                double y = std::stod(tokens[1]);
                double z = std::stod(tokens[2]);
                double r = std::stod(tokens[3]);
                double p = std::stod(tokens[4]);
                double yaw_val = std::stod(tokens[5]);
                uint64_t kf1_id = std::stoull(tokens[6]);
                uint64_t kf2_id = std::stoull(tokens[7]);
                std::string method = tokens[8];

                transforms.emplace_back(x, y, z, r, p, yaw_val, kf1_id, kf2_id, method);
            } catch (const std::exception& e) {
                LOG(WARNING) << "Failed to parse CSV line: " << line << ", error: " << e.what();
            }
        }
    }

    LOG(INFO) << "Read " << transforms.size() << " transform entries from " << csv_path;
    return transforms;
}

std::vector<OdometryTrajectoryPoint> buildOdometryTrajectory(const core::storage::MapStore& map_store) {
    std::vector<OdometryTrajectoryPoint> trajectory;

    // Get all factors from the map store
    auto all_factors = map_store.getAllFactors();
    LOG(INFO) << "Total factors in map store: " << all_factors.size();

    // Filter to get only odometry factors and sort by connected nodes
    std::vector<core::types::Factor> odom_factors;
    for (const auto& factor : all_factors) {
        if (factor.type == core::proto::FactorType::ODOMETRY) {
            odom_factors.push_back(factor);
        }
    }

    LOG(INFO) << "Found " << odom_factors.size() << " odometry factors";

    if (odom_factors.empty()) {
        LOG(WARNING) << "No odometry factors found in map store";
        return trajectory;
    }

    // Sort odometry factors by the first connected node (should be sequential)
    std::sort(odom_factors.begin(), odom_factors.end(),
              [](const core::types::Factor& a, const core::types::Factor& b) {
                  return a.connected_nodes[0] < b.connected_nodes[0];
              });

    // Get keyframes to get timestamps
    auto all_keyframes = map_store.getAllKeyFrames();
    std::map<uint64_t, double> keyframe_timestamps;
    for (const auto& kf : all_keyframes) {
        keyframe_timestamps[kf->id] = kf->pose.timestamp;
    }

    // Build cumulative trajectory from odometry factors
    Eigen::Vector3d cumulative_position(0, 0, 0);
    Eigen::Vector3d cumulative_rotation(0, 0, 0);

    // Add starting point (origin)
    if (!odom_factors.empty()) {
        uint64_t first_kf_id = odom_factors[0].connected_nodes[0];
        double first_timestamp = keyframe_timestamps.count(first_kf_id) ?
                                keyframe_timestamps[first_kf_id] : 0.0;
        trajectory.emplace_back(first_kf_id, cumulative_position, cumulative_rotation, first_timestamp);
    }

    // Accumulate transforms from odometry factors
    for (const auto& factor : odom_factors) {
        if (factor.connected_nodes.size() != 2) {
            LOG(WARNING) << "Odometry factor " << factor.id << " does not have exactly 2 connected nodes";
            continue;
        }

        // Extract relative pose from factor measurement
        const auto& relative_pose = std::get<1>(factor.measurement);

        // Convert quaternion to Euler angles (RPY)
        Eigen::Vector3d rpy = relative_pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);

        // Accumulate the transform
        cumulative_position += relative_pose.position;
        cumulative_rotation += rpy;

        uint64_t to_kf_id = factor.connected_nodes[1];
        double timestamp = keyframe_timestamps.count(to_kf_id) ?
                          keyframe_timestamps[to_kf_id] : 0.0;

        trajectory.emplace_back(to_kf_id, cumulative_position, cumulative_rotation, timestamp);

        LOG(INFO) << "Odometry factor " << factor.id << ": "
                  << factor.connected_nodes[0] << " -> " << factor.connected_nodes[1]
                  << " relative_pos: [" << relative_pose.position.transpose() << "]"
                  << " cumulative_pos: [" << cumulative_position.transpose() << "]";
    }

    LOG(INFO) << "Built odometry trajectory with " << trajectory.size() << " points";
    return trajectory;
}

void writeTrajectoryData(const std::vector<core::storage::KeyFramePtr>& keyframes,
                        const std::vector<TransformData>& transforms,
                        const std::vector<OdometryTrajectoryPoint>& odometry_trajectory,
                        const std::string& output_path) {
    std::ofstream output(output_path);
    if (!output.is_open()) {
        LOG(ERROR) << "Failed to open output file: " << output_path;
        return;
    }

    // Write header
    output << "type,id,x,y,z,roll,pitch,yaw,timestamp,method\n";

    // Write keyframe poses
    for (const auto& kf : keyframes) {
        if (!kf) continue;

        // Convert quaternion to RPY
        Eigen::Quaterniond q = kf->pose.orientation;
        Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2); // Roll, Pitch, Yaw

        output << std::fixed << std::setprecision(6)
               << "keyframe," << kf->id << ","
               << kf->pose.position.x() << ","
               << kf->pose.position.y() << ","
               << kf->pose.position.z() << ","
               << rpy.x() << ","
               << rpy.y() << ","
               << rpy.z() << ","
               << kf->pose.timestamp << ","
               << "ground_truth\n";
    }

    // Write transform data as cumulative trajectory
    Eigen::Vector3d cumulative_position(0, 0, 0);
    Eigen::Vector3d cumulative_rotation(0, 0, 0);

    for (const auto& transform : transforms) {
        // Accumulate transforms to create trajectory
        cumulative_position.x() += transform.x;
        cumulative_position.y() += transform.y;
        cumulative_position.z() += transform.z;
        cumulative_rotation.x() += transform.r;
        cumulative_rotation.y() += transform.p;
        cumulative_rotation.z() += transform.yaw;

        output << std::fixed << std::setprecision(6)
               << "transform," << transform.kf2_id << ","
               << cumulative_position.x() << ","
               << cumulative_position.y() << ","
               << cumulative_position.z() << ","
               << cumulative_rotation.x() << ","
               << cumulative_rotation.y() << ","
               << cumulative_rotation.z() << ","
               << "0,"  // No timestamp for transforms
               << transform.method << "\n";
    }

    // Write odometry trajectory
    for (const auto& odom_point : odometry_trajectory) {
        output << std::fixed << std::setprecision(6)
               << "odometry," << odom_point.keyframe_id << ","
               << odom_point.position.x() << ","
               << odom_point.position.y() << ","
               << odom_point.position.z() << ","
               << odom_point.rotation.x() << ","
               << odom_point.rotation.y() << ","
               << odom_point.rotation.z() << ","
               << odom_point.timestamp << ","
               << "odometry_factors\n";
    }

    LOG(INFO) << "Trajectory comparison data written to: " << output_path;
}

void printTrajectoryStats(const std::vector<core::storage::KeyFramePtr>& keyframes,
                         const std::vector<TransformData>& transforms,
                         const std::vector<OdometryTrajectoryPoint>& odometry_trajectory) {
    LOG(INFO) << "\n=== TRAJECTORY COMPARISON STATISTICS ===";

    // Keyframe statistics
    if (!keyframes.empty()) {
        auto first_kf = keyframes.front();
        auto last_kf = keyframes.back();

        Eigen::Vector3d total_displacement = last_kf->pose.position - first_kf->pose.position;
        double total_distance = 0.0;

        for (size_t i = 1; i < keyframes.size(); ++i) {
            if (keyframes[i] && keyframes[i-1]) {
                total_distance += (keyframes[i]->pose.position - keyframes[i-1]->pose.position).norm();
            }
        }

        LOG(INFO) << "KEYFRAME TRAJECTORY:";
        LOG(INFO) << "  Total keyframes: " << keyframes.size();
        LOG(INFO) << "  Start position: [" << first_kf->pose.position.transpose() << "]";
        LOG(INFO) << "  End position: [" << last_kf->pose.position.transpose() << "]";
        LOG(INFO) << "  Total displacement: [" << total_displacement.transpose() << "]";
        LOG(INFO) << "  Total distance traveled: " << total_distance << "m";
        LOG(INFO) << "  Duration: " << (last_kf->pose.timestamp - first_kf->pose.timestamp) << "s";
    }

    // Transform statistics
    if (!transforms.empty()) {
        Eigen::Vector3d cumulative_position(0, 0, 0);
        double cumulative_distance = 0.0;
        int tf_count = 0;
        int essential_count = 0;

        for (const auto& transform : transforms) {
            Eigen::Vector3d step(transform.x, transform.y, transform.z);
            cumulative_position += step;
            cumulative_distance += step.norm();

            if (transform.method == "TF") tf_count++;
            else if (transform.method == "Essential") essential_count++;
        }

        LOG(INFO) << "TRANSFORM TRAJECTORY:";
        LOG(INFO) << "  Total transforms: " << transforms.size();
        LOG(INFO) << "  TF transforms: " << tf_count;
        LOG(INFO) << "  Essential matrix transforms: " << essential_count;
        LOG(INFO) << "  Final position: [" << cumulative_position.transpose() << "]";
        LOG(INFO) << "  Total distance: " << cumulative_distance << "m";
        LOG(INFO) << "  Average step size: " << (cumulative_distance / transforms.size()) << "m";
    }

    // Odometry trajectory statistics
    if (!odometry_trajectory.empty()) {
        auto first_odom = odometry_trajectory.front();
        auto last_odom = odometry_trajectory.back();

        Eigen::Vector3d total_displacement = last_odom.position - first_odom.position;
        double total_distance = 0.0;

        for (size_t i = 1; i < odometry_trajectory.size(); ++i) {
            total_distance += (odometry_trajectory[i].position - odometry_trajectory[i-1].position).norm();
        }

        LOG(INFO) << "ODOMETRY TRAJECTORY:";
        LOG(INFO) << "  Total odometry points: " << odometry_trajectory.size();
        LOG(INFO) << "  Start position: [" << first_odom.position.transpose() << "]";
        LOG(INFO) << "  End position: [" << last_odom.position.transpose() << "]";
        LOG(INFO) << "  Total displacement: [" << total_displacement.transpose() << "]";
        LOG(INFO) << "  Total distance: " << total_distance << "m";
        if (last_odom.timestamp > first_odom.timestamp) {
            LOG(INFO) << "  Duration: " << (last_odom.timestamp - first_odom.timestamp) << "s";
        }
    }

    LOG(INFO) << "======================================\n";
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    std::string csv_path = "/data/robot/reconstruct.csv";
    std::string map_base_path = "/data/robots";
    std::string output_path = "/data/robot/trajectory_comparison.csv";

    // Parse command line arguments
    if (argc > 1) {
        csv_path = argv[1];
    }
    if (argc > 2) {
        map_base_path = argv[2];
    }
    if (argc > 3) {
        output_path = argv[3];
    }

    LOG(INFO) << "Starting trajectory comparison tool";
    LOG(INFO) << "CSV path: " << csv_path;
    LOG(INFO) << "Map base path: " << map_base_path;
    LOG(INFO) << "Output path: " << output_path;

    // Read transform data from CSV
    auto transforms = readReconstructCSV(csv_path);
    if (transforms.empty()) {
        LOG(WARNING) << "No transform data found in CSV file";
    }

    // Load keyframes from MapStore
    core::storage::MapStore map_store(map_base_path);
    if (!map_store.loadMap()) {
        LOG(ERROR) << "Failed to load map from " << map_base_path;
        return 1;
    }

    auto keyframes = map_store.getAllKeyFrames();
    if (keyframes.empty()) {
        LOG(ERROR) << "No keyframes found in map store";
        return 1;
    }

    // Sort keyframes by ID for consistent trajectory
    std::sort(keyframes.begin(), keyframes.end(),
              [](const core::storage::KeyFramePtr& a, const core::storage::KeyFramePtr& b) {
                  return a->id < b->id;
              });

    // Build odometry trajectory from factors
    auto odometry_trajectory = buildOdometryTrajectory(map_store);

    // Print statistics
    printTrajectoryStats(keyframes, transforms, odometry_trajectory);

    // Write comparison data
    writeTrajectoryData(keyframes, transforms, odometry_trajectory, output_path);

    LOG(INFO) << "Trajectory comparison completed successfully";
    LOG(INFO) << "Use Python plotting script to visualize: scripts/plot_trajectory.py " << output_path;

    return 0;
}
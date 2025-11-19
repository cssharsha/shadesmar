#include "colmap_viz.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>

namespace gs {

void ColmapViz::initialize(const std::string& input_dir) {
    viz_ = std::make_shared<viz::RerunVisualizer>("colmap", "building", "127.0.0.1", 9876);
    if (!viz_->initialize()) {
        std::cerr << "Failed to initialize visualizer" << std::endl;
        exit(0);
    }

    try {
        map_store_ = std::make_shared<core::storage::MapStore>(input_dir,
                                                               core::storage::ProcessRole::READER);

        if (!map_store_) {
            std::cerr << "Invalid map store init" << std::endl;
            exit(0);
        }
        tf_tree_ = std::make_shared<stf::TransformTree>();
        tf_tree_->setTransform("base_link", "camera", Eigen::Isometry3d::Identity());
        map_store_->syncIndexFromDisk();
    } catch (const std::exception& e) {
        std::cerr << "Unable init map store and tf tree" << e.what() << std::endl;
        exit(0);
    }
    std::cout << "Finished initing stuff" << std::endl;
}

void ColmapViz::publishKeypoints() {
    auto all_keypoints = map_store_->getAllKeyPoints();
    std::cout << "No keypoints: " << all_keypoints.size() << std::endl;
    if (!all_keypoints.size()) {
        return;
    }
    if (!viz_->isConnected()) {
        std::cerr << "Cannot visualize keypoints without Rerun visualizer" << std::endl;
        return;
    }

    std::ofstream file("/data/south-building/text/keypoints.csv");
    if (!file.is_open()) {
        std::cerr << "Failed to open keypoints.csv for writing" << std::endl;
        return;
    }
    core::types::PointCloud cloud;
    cloud.points.reserve(all_keypoints.size());
    cloud.colors.reserve(all_keypoints.size());

    for (const auto& kp : all_keypoints) {
        const auto& position = kp.position;
        cloud.points.emplace_back(position);
        std::cout << "Keypoint color: " << kp.color.transpose() << std::endl;
        cloud.colors.emplace_back(kp.color);
        // cloud.colors.emplace_back(
        //     Eigen::Vector3d(static_cast<float>(kp.descriptor.at<uint8_t>(0, 0)),
        //                     static_cast<float>(kp.descriptor.at<uint8_t>(0, 1)),
        //                     static_cast<float>(kp.descriptor.at<uint8_t>(0, 2))));
        // cloud.colors.emplace_back(Eigen::Vector3d(1.0f, 1.0f, 1.0f));
        file << kp.id() << "," << position.x() << "," << position.y() << "," << position.z()
             << std::endl;
    }

    viz_->addPointCloud(cloud, "/map/keypoints", 0.0);
}

void ColmapViz::visualizeCameras() {
    auto all_keyframes = map_store_->getAllKeyFrames();
    std::cout << "No keyframes: " << all_keyframes.size() << std::endl;
    if (!all_keyframes.size()) {
        return;
    }
    if (!viz_->isConnected()) {
        std::cerr << "Cannot visualize keypoints without Rerun visualizer" << std::endl;
        return;
    }

    std::vector<Eigen::Vector3f> points;
    points.reserve(all_keyframes.size());

    for (const auto& kf : all_keyframes) {
        viz_->addPose(kf->pose, "/map/cameras/" + std::to_string(kf->id), 0.0);
        viz_->addCamera(kf->camera_info.value(), "/map/cameras/" + std::to_string(kf->id), 0.0);
        if (kf->id % 10 == 0) {
            viz_->addImage(kf->color_data.value().toCvMat(),
                           "/map/cameras/" + std::to_string(kf->id), 0.0);
        }
    }
}

void ColmapViz::visualizeKeyframesAsPoints() {
    auto all_keyframes = map_store_->getAllKeyFrames();
    std::cout << "No keyframes: " << all_keyframes.size() << std::endl;
    if (!all_keyframes.size()) {
        return;
    }
    if (!viz_->isConnected()) {
        std::cerr << "Cannot visualize keyframes as points without Rerun visualizer" << std::endl;
        return;
    }

    std::ofstream file("/data/south-building/text/keyframes.csv");
    if (!file.is_open()) {
        std::cerr << "Failed to open keyframes.csv for writing" << std::endl;
        return;
    }
    core::types::PointCloud cloud;
    cloud.points.reserve(all_keyframes.size());

    for (const auto& kf : all_keyframes) {
        if (!kf) {
            std::cerr << "Failed to load keyframe " << kf->id << std::endl;
            continue;
        }
        file << kf->id << "," << kf->pose.position.x() << "," << kf->pose.position.y() << ","
             << kf->pose.position.z() << std::endl;
        cloud.points.emplace_back(kf->pose.position);
        cloud.colors.emplace_back(Eigen::Vector3d(1.0f, 0.0f, 1.0f));
    }
    viz_->addPointCloud(cloud, "/map/keyframes", 0.0, core::types::Pose());
}

void ColmapViz::visuaulize() {
    publishKeypoints();
    visualizeCameras();
}

void ColmapViz::exportForOpen3D(const std::string& out_dir) {
    namespace fs = std::filesystem;
    try {
        if (!map_store_) {
            std::cerr << "Map store not initialized" << std::endl;
            return;
        }

        // Ensure output directory exists
        fs::create_directories(out_dir);

        // 1) Export keypoints as ASCII PLY (x y z r g b)
        auto all_keypoints = map_store_->getAllKeyPoints();
        const auto ply_path = fs::path(out_dir) / "points.ply";
        {
            std::ofstream ply(ply_path);
            if (!ply) {
                std::cerr << "Failed to open PLY file for writing: " << ply_path << std::endl;
            } else {
                ply << "ply\n";
                ply << "format ascii 1.0\n";
                ply << "element vertex " << all_keypoints.size() << "\n";
                ply << "property float x\n";
                ply << "property float y\n";
                ply << "property float z\n";
                ply << "property uchar red\n";
                ply << "property uchar green\n";
                ply << "property uchar blue\n";
                ply << "end_header\n";

                ply << std::fixed << std::setprecision(6);
                for (const auto& kp : all_keypoints) {
                    const auto& p = kp.position;
                    // Keypoints: green
                    ply << static_cast<float>(p.x()) << ' ' << static_cast<float>(p.y()) << ' '
                        << static_cast<float>(p.z()) << ' ' << 0 << ' ' << 255 << ' ' << 0 << '\n';
                }
                std::cout << "Wrote keypoints to " << ply_path << " (" << all_keypoints.size()
                          << ")" << std::endl;
            }
        }

        // 2) Export keyframe trajectory and orientations as JSON
        auto all_keyframes = map_store_->getAllKeyFrames();
        std::sort(all_keyframes.begin(), all_keyframes.end(),
                  [](const auto& a, const auto& b) { return a->id < b->id; });

        const auto traj_path = fs::path(out_dir) / "trajectory.json";
        {
            std::ofstream js(traj_path);
            if (!js) {
                std::cerr << "Failed to open trajectory.json for writing: " << traj_path
                          << std::endl;
            } else {
                js << std::fixed << std::setprecision(6);
                js << "{\n  \"keyframes\": [\n";
                for (size_t i = 0; i < all_keyframes.size(); ++i) {
                    const auto& kf = all_keyframes[i];
                    const auto& pos = kf->pose.position;
                    const auto& q = kf->pose.orientation;
                    js << "    {\n";
                    js << "      \"id\": " << kf->id << ",\n";
                    js << "      \"position\": [" << pos.x() << ", " << pos.y() << ", " << pos.z()
                       << "],\n";
                    js << "      \"orientation\": [" << q.w() << ", " << q.x() << ", " << q.y()
                       << ", " << q.z() << "]\n";
                    js << "    }" << (i + 1 < all_keyframes.size() ? "," : "") << "\n";
                }
                js << "  ]\n}";
                std::cout << "Wrote trajectory to " << traj_path << " (" << all_keyframes.size()
                          << ")" << std::endl;
            }
        }

        // Optional convenience: export keyframe positions as a small PLY line points
        const auto kf_points_path = fs::path(out_dir) / "keyframes.ply";
        {
            std::ofstream ply(kf_points_path);
            if (ply) {
                ply << "ply\nformat ascii 1.0\n";
                ply << "element vertex " << all_keyframes.size() << "\n";
                ply << "property float x\nproperty float y\nproperty float z\n";
                ply << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
                ply << "end_header\n";
                ply << std::fixed << std::setprecision(6);
                for (const auto& kf : all_keyframes) {
                    const auto& p = kf->pose.position;
                    // Magenta for keyframes
                    ply << static_cast<float>(p.x()) << ' ' << static_cast<float>(p.y()) << ' '
                        << static_cast<float>(p.z()) << ' ' << 255 << ' ' << 0 << ' ' << 255
                        << '\n';
                }
                std::cout << "Wrote keyframe positions to " << kf_points_path << std::endl;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception during exportForOpen3D: " << e.what() << std::endl;
    }
}

}  // namespace gs

#include <core/storage/map_store.hpp>
#include <logging/logging.hpp>
#include <iostream>

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <map_base_path>" << std::endl;
        return 1;
    }

    std::string map_base_path = argv[1];

    LOG(INFO) << "Loading map from: " << map_base_path;

    // Load map store
    core::storage::MapStore map_store(map_base_path);
    if (!map_store.loadMap()) {
        LOG(ERROR) << "Failed to load map from " << map_base_path;
        return 1;
    }

    LOG(INFO) << "Map loaded successfully";

    // Check keyframes
    auto all_keyframes = map_store.getAllKeyFrames();
    LOG(INFO) << "Total keyframes: " << all_keyframes.size();

    // Check keypoints
    auto all_keypoints = map_store.getAllKeyPoints();
    LOG(INFO) << "Total keypoints: " << all_keypoints.size();

    // Check factors
    auto all_factors = map_store.getAllFactors();
    LOG(INFO) << "Total factors: " << all_factors.size();

    // Sample a few keypoints if available
    if (!all_keypoints.empty()) {
        LOG(INFO) << "Sample keypoints (first 5):";
        for (size_t i = 0; i < std::min(size_t(5), all_keypoints.size()); ++i) {
            const auto& kp = all_keypoints[i];
            LOG(INFO) << "  Keypoint " << kp.id() << ": " << kp.locations.size() << " detections";
        }
    }

    // Check if keyframes have image data
    if (!all_keyframes.empty()) {
        int with_images = 0;
        for (const auto& kf : all_keyframes) {
            if (kf && kf->color_data.has_value()) {
                with_images++;
            }
        }
        LOG(INFO) << "Keyframes with image data: " << with_images << "/" << all_keyframes.size();
    }

    return 0;
}
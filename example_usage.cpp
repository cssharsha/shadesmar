// Example usage of the new cleaner keyframe creation system
#include "core/graph/graph_adapter.hpp"
#include "core/graph/keyframe_manager.hpp"
#include "2d/orb_tracker.hpp"

int main() {
    // Create the graph components
    core::graph::FactorGraph graph;
    core::storage::MapStore store;
    core::graph::GraphAdapter adapter(graph, store);

    // Configure keyframe creation thresholds
    core::graph::KeyframeThresholds thresholds;
    thresholds.distance_threshold = 0.1;        // 10cm distance threshold
    thresholds.rotation_threshold = 10.0;       // 10 degree rotation threshold
    thresholds.min_inliers = 50;                // Minimum 50 inliers for visual odometry
    thresholds.min_translation_visual = 0.05;   // 5cm visual translation threshold
    thresholds.use_visual_odometry = true;      // Enable essential matrix evaluation

    // Apply the thresholds
    adapter.setKeyframeThresholds(thresholds);

    // Set up transform tree for ORB tracker
    auto transform_tree = std::make_shared<stf::TransformTree>();
    adapter.setTransformTree(transform_tree);

    // Example: Process incoming data
    // The system will automatically decide when to create keyframes based on:
    // 1. Distance threshold (geometric)
    // 2. Rotation threshold (geometric)
    // 3. Essential matrix analysis (visual odometry)

    core::types::Pose pose;
    core::types::Image image;
    core::types::CameraInfo camera_info;

    // This will create a keyframe if any threshold is exceeded
    auto keyframe = adapter.createKeyframe(pose, image, camera_info);

    if (keyframe) {
        std::cout << "Keyframe created with ID: " << keyframe->id << std::endl;
    } else {
        std::cout << "Keyframe creation skipped" << std::endl;
    }

    return 0;
}
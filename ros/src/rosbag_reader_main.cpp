#include "ros/rosbag_reader.hpp"

int main(int argc, char** argv) {
    ros::Config config;
    config.odom_topic = "/base/odom";
    config.color_topic = "/camera/camera/color/image_raw";
    config.camera_info_topic = "/camera/camera/color/camera_info";
    config.keyframe_distance_threshold = 0.1;  // 10cm default

    auto graph = std::make_shared<core::graph::FactorGraph>();
    auto store = std::make_shared<core::storage::MapStore>();

    ros::RosbagReader reader(argv[1], *graph, *store, config);
    if (!reader.initialize()) {
        std::cerr << "Failed to initialize rosbag reader" << std::endl;
        return 1;
    }
    reader.processBag();
    return 0;
}

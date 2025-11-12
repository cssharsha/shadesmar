#include <glog/logging.h>
#include "map_to_colmap_converter.hpp"

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;

    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <map-store-path> <output-colmap-path>" << std::endl;
        std::cerr << "Example: " << argv[0] << " /path/to/map /path/to/output" << std::endl;
        std::cerr << "\nThis will create:" << std::endl;
        std::cerr << "  <output-colmap-path>/text/cameras.txt" << std::endl;
        std::cerr << "  <output-colmap-path>/text/images.txt" << std::endl;
        std::cerr << "  <output-colmap-path>/text/points3D.txt" << std::endl;
        return 1;
    }

    std::string map_path = argv[1];
    std::string output_path = argv[2];

    LOG(INFO) << "Converting map store to COLMAP format";
    LOG(INFO) << "Map path: " << map_path;
    LOG(INFO) << "Output path: " << output_path;

    auto converter = std::make_unique<gs::MapToColmapConverter>(map_path, output_path);

    // You can optionally set the camera model (default is PINHOLE)
    // converter->setCameraModel("SIMPLE_RADIAL");
    // converter->setCameraModel("RADIAL");

    if (converter->convert()) {
        LOG(INFO) << "Conversion successful!";
        LOG(INFO) << "COLMAP text files written to: " << output_path << "/text/";
        return 0;
    } else {
        LOG(ERROR) << "Conversion failed.";
        return 1;
    }
}

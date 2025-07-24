#include <glog/logging.h>
#include "colmap_converter.hpp"

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;

    if (!(argc == 2 || argc == 3)) {
        std::cerr << "Usage: " << argv[0] << " <path-to-colmap-database>" << std::endl;
        return 1;
    }

    std::unique_ptr<gs::ColmapConverter> converter;
    if (argc == 3) {
        converter = std::make_unique<gs::ColmapConverter>(argv[1], argv[2]);
    } else {
        converter = std::make_unique<gs::ColmapConverter>(argv[1]);
    }
    if (converter->convertFromText()) {
        LOG(INFO) << "Conversion successful.";
    } else {
        LOG(ERROR) << "Conversion failed.";
        return 1;
    }
    // converter->justQuery();

    return 0;
}

#include "colmap_viz.hpp"
#include <string>
#include <iostream>

int main(int argc, char** argv) {
    // Parse arguments
    std::string input_dir;
    std::string export_dir;

    // Arg handling: --input <input_dir> (required), --export_o3d <out_dir> (optional)
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--input" && i + 1 < argc) {
            input_dir = argv[++i];
        } else if (arg == "--export_o3d" && i + 1 < argc) {
            export_dir = argv[++i];
        }
    }

    if (input_dir.empty()) {
        std::cerr << "Usage: " << argv[0] << " --input <input_dir> [--export_o3d <out_dir>]" << std::endl;
        return 1;
    }

    gs::ColmapViz viz;
    viz.initialize(input_dir);

    if (!export_dir.empty()) {
        std::cout << "Exporting for Open3D to: " << export_dir << std::endl;
        viz.exportForOpen3D(export_dir);
        return 0;
    }

    // Default behavior: use Rerun visualizer
    viz.visuaulize();
    std::cout << "Rerun visualization running. Press Ctrl+C to exit." << std::endl;
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

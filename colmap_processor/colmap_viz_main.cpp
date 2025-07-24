#include "colmap_viz.hpp"
#include <string>
#include <iostream>

int main(int argc, char** argv) {
    gs::ColmapViz viz;
    viz.initialize();
    // Simple arg handling: --export_o3d <out_dir>
    // If provided, export files for Open3D viewer and exit.
    if (argc >= 3 && std::string(argv[1]) == "--export_o3d") {
        std::string out_dir = argv[2];
        std::cout << "Exporting for Open3D to: " << out_dir << std::endl;
        viz.exportForOpen3D(out_dir);
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

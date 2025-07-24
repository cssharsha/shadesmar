#pragma once

#include <memory>
#include "core/storage/map_store.hpp"
#include "stf/transform_tree.hpp"
#include "viz/rerun_viz.hpp"
namespace gs {

class ColmapViz {
public:
    ColmapViz() = default;
    void visuaulize();
    void initialize();
    // Export minimal artifacts for a simple Python/Open3D viewer
    // Writes point cloud and trajectory files into the given directory.
    // - points.ply: ASCII PLY with x y z r g b
    // - trajectory.json: list of keyframe poses (id, position, orientation)
    void exportForOpen3D(const std::string& out_dir);

private:
    std::shared_ptr<viz::RerunVisualizer> viz_;
    std::shared_ptr<core::storage::MapStore> map_store_;
    std::shared_ptr<stf::TransformTree> tf_tree_;
    void publishKeypoints();
    void visualizeCameras();
    void visualizeKeyframesAsPoints();
};
}  // namespace gs

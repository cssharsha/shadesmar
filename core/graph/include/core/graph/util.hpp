#pragma once

#include <fstream>
#include <string>
#include "core/graph/factor_graph.hpp"

namespace core {
namespace graph {
namespace util {

inline void dumpFactorGraph(const FactorGraph& graph, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    // Write VTK header
    file << "# vtk DataFile Version 3.0\n";
    file << "Factor Graph\n";
    file << "ASCII\n";
    file << "DATASET POLYDATA\n";

    // Write vertices (keyframe positions)
    auto keyframes = graph.getAllKeyFrames();
    file << "POINTS " << keyframes.size() << " float\n";
    for (const auto& kf : keyframes) {
        file << kf->pose.position.x() << " "
             << kf->pose.position.y() << " "
             << kf->pose.position.z() << "\n";
    }

    // Write edges (factors)
    auto factors = graph.getFactors();
    int num_edges = 0;
    for (const auto& factor : factors) {
        if (factor.type == proto::FactorType::ODOMETRY ||
            factor.type == proto::FactorType::LOOP_CLOSURE) {
            num_edges++;
        }
    }

    file << "LINES " << num_edges << " " << num_edges * 3 << "\n";

    // Create a map of keyframe IDs to indices
    std::map<uint64_t, size_t> id_to_index;
    for (size_t i = 0; i < keyframes.size(); ++i) {
        id_to_index[keyframes[i]->id] = i;
    }

    // Write edges
    for (const auto& factor : factors) {
        if (factor.type == proto::FactorType::ODOMETRY ||
            factor.type == proto::FactorType::LOOP_CLOSURE) {
            // Each line starts with number of points (2) followed by the point indices
            file << "2 "
                 << id_to_index[factor.connected_nodes[0]] << " "
                 << id_to_index[factor.connected_nodes[1]] << "\n";
        }
    }

    // Write factor types as cell data
    file << "CELL_DATA " << num_edges << "\n";
    file << "SCALARS factor_type int 1\n";
    file << "LOOKUP_TABLE default\n";
    for (const auto& factor : factors) {
        if (factor.type == proto::FactorType::ODOMETRY ||
            factor.type == proto::FactorType::LOOP_CLOSURE) {
            file << static_cast<int>(factor.type) << "\n";
        }
    }

    file.close();
}

}  // namespace util
}  // namespace graph
}  // namespace core

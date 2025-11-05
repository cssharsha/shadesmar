#include "splat_logger.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <logging/logging.hpp>

namespace gaussian_splatting {
namespace utils {

std::string SplatLogger::getSplatCSVHeader(int sh_degree) {
    std::stringstream ss;
    ss << "splat_id,pos_x,pos_y,pos_z,scale_x,scale_y,scale_z,";
    ss << "rot_w,rot_x,rot_y,rot_z,opacity,";

    // Add SH DC component (sh0)
    ss << "sh0_r,sh0_g,sh0_b";

    // Add higher order SH coefficients (shN)
    // For degree 3: 15 coefficients (indices 0-14)
    const int num_higher_order = (sh_degree + 1) * (sh_degree + 1) - 1;
    for (int i = 0; i < num_higher_order; ++i) {
        ss << ",shN_" << i << "_r,shN_" << i << "_g,shN_" << i << "_b";
    }

    ss << "\n";
    return ss.str();
}

std::string SplatLogger::splatToCSVRow(const core::types::GaussianSplat& splat) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);

    // ID
    ss << splat.id << ",";

    // Position (3 values)
    ss << splat.position.x() << "," << splat.position.y() << "," << splat.position.z() << ",";

    // Scale (3 values)
    ss << splat.scale.x() << "," << splat.scale.y() << "," << splat.scale.z() << ",";

    // Rotation quaternion (4 values: w, x, y, z)
    ss << splat.rotation.w() << "," << splat.rotation.x() << ","
       << splat.rotation.y() << "," << splat.rotation.z() << ",";

    // Opacity (1 value)
    ss << splat.opacity << ",";

    // SH DC component (sh0_r, sh0_g, sh0_b)
    ss << splat.sh_dc.x() << "," << splat.sh_dc.y() << "," << splat.sh_dc.z();

    // Higher order SH coefficients (shN_i_r, shN_i_g, shN_i_b)
    // sh_rest is stored as [sh1_r, sh1_g, sh1_b, sh2_r, sh2_g, sh2_b, ...]
    const int num_higher_order = splat.sh_rest.size() / 3;
    for (int i = 0; i < num_higher_order; ++i) {
        ss << "," << splat.sh_rest(i * 3 + 0)   // r
           << "," << splat.sh_rest(i * 3 + 1)   // g
           << "," << splat.sh_rest(i * 3 + 2);  // b
    }

    ss << "\n";
    return ss.str();
}

bool SplatLogger::writeSplatsToCSV(const std::vector<core::types::GaussianSplat>& splats,
                                    const std::string& output_path,
                                    const std::string& filename) {
    if (splats.empty()) {
        LOG(WARNING) << "No splats to write to CSV";
        return false;
    }

    try {
        // Create directory if it doesn't exist
        std::filesystem::create_directories(output_path);

        std::string full_path = output_path;
        if (full_path.back() != '/') {
            full_path += '/';
        }
        full_path += filename;

        std::ofstream file(full_path);
        if (!file.is_open()) {
            LOG(ERROR) << "Failed to open CSV file for writing: " << full_path;
            return false;
        }

        // Get SH degree from first splat (all should be the same)
        int sh_degree = 3;  // Default to degree 3
        if (!splats.empty()) {
            sh_degree = splats[0].sh_degree;
        }

        // Write header
        file << getSplatCSVHeader(sh_degree);

        // Write splat data
        for (const auto& splat : splats) {
            file << splatToCSVRow(splat);
        }

        file.close();
        LOG(INFO) << "Successfully wrote " << splats.size() << " splats to " << full_path;
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception writing splats to CSV: " << e.what();
        return false;
    }
}

bool SplatLogger::appendSplatsToCSV(const std::vector<core::types::GaussianSplat>& splats,
                                     const std::string& output_path,
                                     const std::string& filename) {
    if (splats.empty()) {
        LOG(WARNING) << "No splats to append to CSV";
        return false;
    }

    try {
        std::string full_path = output_path;
        if (full_path.back() != '/') {
            full_path += '/';
        }
        full_path += filename;

        std::ofstream file(full_path, std::ios::app);
        if (!file.is_open()) {
            LOG(ERROR) << "Failed to open CSV file for appending: " << full_path;
            return false;
        }

        // Write splat data (no header)
        for (const auto& splat : splats) {
            file << splatToCSVRow(splat);
        }

        file.close();
        LOG(INFO) << "Successfully appended " << splats.size() << " splats to " << full_path;
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception appending splats to CSV: " << e.what();
        return false;
    }
}

}  // namespace utils
}  // namespace gaussian_splatting

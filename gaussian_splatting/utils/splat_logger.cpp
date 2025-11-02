#include "splat_logger.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <logging/logging.hpp>

namespace gaussian_splatting {
namespace utils {

std::string SplatLogger::getSplatCSVHeader(int max_sh_coeffs) {
    std::stringstream ss;
    ss << "id,pos_x,pos_y,pos_z,scale_x,scale_y,scale_z,";
    ss << "rot_w,rot_x,rot_y,rot_z,";
    ss << "color_r,color_g,color_b,opacity";

    // Add SH coefficient columns
    for (int i = 0; i < max_sh_coeffs; ++i) {
        ss << ",sh_" << i;
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

    // Color (3 values)
    ss << splat.color.x() << "," << splat.color.y() << "," << splat.color.z() << ",";

    // Opacity (1 value)
    ss << splat.opacity;

    // SH coefficients (variable number)
    for (int i = 0; i < splat.sh_coefficients.size(); ++i) {
        ss << "," << splat.sh_coefficients(i);
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

        // Find maximum SH coefficient size
        int max_sh_coeffs = 0;
        for (const auto& splat : splats) {
            max_sh_coeffs = std::max(max_sh_coeffs, static_cast<int>(splat.sh_coefficients.size()));
        }

        // Write header
        file << getSplatCSVHeader(max_sh_coeffs);

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

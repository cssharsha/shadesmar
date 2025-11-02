#pragma once

#include <string>
#include <vector>
#include "core/types/gaussian_splat.hpp"

namespace gaussian_splatting {
namespace utils {

class SplatLogger {
public:
    // Write Gaussian splats to a CSV file
    // CSV format: id,pos_x,pos_y,pos_z,scale_x,scale_y,scale_z,rot_w,rot_x,rot_y,rot_z,
    //             color_r,color_g,color_b,opacity,sh_0,sh_1,...,sh_N
    static bool writeSplatsToCSV(const std::vector<core::types::GaussianSplat>& splats,
                                  const std::string& output_path,
                                  const std::string& filename = "splats.csv");

    // Append splats to an existing CSV file (without header)
    static bool appendSplatsToCSV(const std::vector<core::types::GaussianSplat>& splats,
                                   const std::string& output_path,
                                   const std::string& filename = "splats.csv");

private:
    static std::string getSplatCSVHeader(int max_sh_coeffs);
    static std::string splatToCSVRow(const core::types::GaussianSplat& splat);
};

}  // namespace utils
}  // namespace gaussian_splatting

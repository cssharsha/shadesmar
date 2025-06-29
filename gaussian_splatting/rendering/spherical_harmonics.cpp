#include "spherical_harmonics.hpp"
#include <logging/logging.hpp>
#include <cmath>

namespace gaussian_splatting {
namespace rendering {

SphericalHarmonics::SphericalHarmonics(int sh_degree, torch::Device device)
    : sh_degree_(sh_degree), device_(device) {
    
    if (sh_degree_ > MAX_SH_DEGREE) {
        LOG(WARNING) << "SH degree " << sh_degree_ << " exceeds maximum " << MAX_SH_DEGREE;
        sh_degree_ = MAX_SH_DEGREE;
    }
    
    LOG(INFO) << "Initializing SphericalHarmonics with degree " << sh_degree_;
}

torch::Tensor SphericalHarmonics::evaluateSH(const torch::Tensor& sh_coeffs,
                                            const torch::Tensor& viewing_directions) const {
    if (device_.is_cuda() && sh_coeffs.is_cuda() && viewing_directions.is_cuda()) {
        return cuda::evaluateSHCuda(sh_coeffs, viewing_directions, sh_degree_);
    }
    
    // CPU implementation
    auto dirs = normalizeDirections(viewing_directions);
    auto sh_basis = computeSHBasis(dirs, sh_degree_);
    
    // sh_coeffs: [N, num_coeffs, 3]
    // sh_basis: [N, num_coeffs]
    // output: [N, 3]
    auto output = torch::sum(sh_coeffs * sh_basis.unsqueeze(-1), 1);
    
    return output;
}

torch::Tensor SphericalHarmonics::computeSHBasis(const torch::Tensor& directions, int degree) const {
    if (device_.is_cuda() && directions.is_cuda()) {
        return cuda::computeSHBasisCuda(directions, degree);
    }
    
    // CPU implementation
    auto dirs = normalizeDirections(directions);
    int num_points = dirs.size(0);
    int total_coeffs = getTotalSHCoeffs(degree);
    
    auto opts = torch::TensorOptions().device(device_).dtype(torch::kFloat32);
    auto sh_basis = torch::zeros({num_points, total_coeffs}, opts);
    
    int coeff_idx = 0;
    
    // Degree 0 (l=0)
    if (degree >= 0) {
        auto y0 = evaluateY0(dirs);
        sh_basis.slice(1, coeff_idx, coeff_idx + 1) = y0;
        coeff_idx += 1;
    }
    
    // Degree 1 (l=1)
    if (degree >= 1) {
        auto y1 = evaluateY1(dirs);
        sh_basis.slice(1, coeff_idx, coeff_idx + 3) = y1;
        coeff_idx += 3;
    }
    
    // Degree 2 (l=2)
    if (degree >= 2) {
        auto y2 = evaluateY2(dirs);
        sh_basis.slice(1, coeff_idx, coeff_idx + 5) = y2;
        coeff_idx += 5;
    }
    
    // Degree 3 (l=3)
    if (degree >= 3) {
        auto y3 = evaluateY3(dirs);
        sh_basis.slice(1, coeff_idx, coeff_idx + 7) = y3;
        coeff_idx += 7;
    }
    
    // Degree 4 (l=4)
    if (degree >= 4) {
        auto y4 = evaluateY4(dirs);
        sh_basis.slice(1, coeff_idx, coeff_idx + 9) = y4;
        coeff_idx += 9;
    }
    
    return sh_basis;
}

int SphericalHarmonics::getNumSHCoeffs(int degree) {
    if (degree < 0 || degree > MAX_SH_DEGREE) {
        return 0;
    }
    return 2 * degree + 1;
}

int SphericalHarmonics::getTotalSHCoeffs(int degree) {
    if (degree < 0) return 0;
    if (degree > MAX_SH_DEGREE) degree = MAX_SH_DEGREE;
    
    int total = 0;
    for (int l = 0; l <= degree; ++l) {
        total += getNumSHCoeffs(l);
    }
    return total;
}

torch::Tensor SphericalHarmonics::evaluateY0(const torch::Tensor& dirs) const {
    // Y_0^0 = 1/(2*sqrt(pi))
    const float Y00 = 0.28209479177387814f; // 1/(2*sqrt(pi))
    
    auto result = torch::full({dirs.size(0), 1}, Y00, 
                             torch::TensorOptions().device(device_).dtype(torch::kFloat32));
    return result;
}

torch::Tensor SphericalHarmonics::evaluateY1(const torch::Tensor& dirs) const {
    auto x = dirs.select(1, 0);
    auto y = dirs.select(1, 1);
    auto z = dirs.select(1, 2);
    
    // Y_1^{-1} = sqrt(3/(4*pi)) * y
    // Y_1^0 = sqrt(3/(4*pi)) * z
    // Y_1^1 = sqrt(3/(4*pi)) * x
    const float C1 = 0.4886025119029199f; // sqrt(3/(4*pi))
    
    auto y1m1 = C1 * y;
    auto y10 = C1 * z;
    auto y11 = C1 * x;
    
    return torch::stack({y1m1, y10, y11}, 1);
}

torch::Tensor SphericalHarmonics::evaluateY2(const torch::Tensor& dirs) const {
    auto x = dirs.select(1, 0);
    auto y = dirs.select(1, 1);
    auto z = dirs.select(1, 2);
    
    auto xx = x * x;
    auto yy = y * y;
    auto zz = z * z;
    auto xy = x * y;
    auto yz = y * z;
    auto xz = x * z;
    
    // SH degree 2 coefficients
    const float C20 = 0.31539156525252005f; // sqrt(5/(16*pi))
    const float C21 = 0.5462742152960396f;  // sqrt(15/(4*pi))
    const float C22 = 0.5462742152960396f;  // sqrt(15/(4*pi))
    const float C23 = 0.5462742152960396f;  // sqrt(15/(4*pi))
    const float C24 = 0.27313710764801976f; // sqrt(15/(16*pi))
    
    auto y2m2 = C24 * (xx - yy);
    auto y2m1 = C21 * xy;
    auto y20 = C20 * (2.0f * zz - xx - yy);
    auto y21 = C22 * yz;
    auto y22 = C23 * xz;
    
    return torch::stack({y2m2, y2m1, y20, y21, y22}, 1);
}

torch::Tensor SphericalHarmonics::evaluateY3(const torch::Tensor& dirs) const {
    auto x = dirs.select(1, 0);
    auto y = dirs.select(1, 1);
    auto z = dirs.select(1, 2);
    
    // Simplified degree 3 implementation
    auto result = torch::zeros({dirs.size(0), 7}, 
                              torch::TensorOptions().device(device_).dtype(torch::kFloat32));
    
    // Placeholder - proper implementation would require more complex calculations
    return result;
}

torch::Tensor SphericalHarmonics::evaluateY4(const torch::Tensor& dirs) const {
    auto x = dirs.select(1, 0);
    auto y = dirs.select(1, 1);
    auto z = dirs.select(1, 2);
    
    // Simplified degree 4 implementation
    auto result = torch::zeros({dirs.size(0), 9}, 
                              torch::TensorOptions().device(device_).dtype(torch::kFloat32));
    
    // Placeholder - proper implementation would require more complex calculations
    return result;
}

torch::Tensor SphericalHarmonics::normalizeDirections(const torch::Tensor& directions) const {
    auto norms = directions.norm(2, -1, true);
    return directions / (norms + 1e-8f);
}

} // namespace rendering
} // namespace gaussian_splatting
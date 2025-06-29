#pragma once

#include <torch/torch.h>
#include "../training/training_config.hpp"

namespace gaussian_splatting {
namespace rendering {

class SphericalHarmonics {
public:
    static constexpr int MAX_SH_DEGREE = 4;
    static constexpr int SH_COEFFS_PER_DEGREE[] = {1, 3, 5, 7, 9}; // l=0,1,2,3,4
    
    explicit SphericalHarmonics(int sh_degree, torch::Device device);
    
    // Evaluate SH coefficients for given viewing directions
    torch::Tensor evaluateSH(const torch::Tensor& sh_coeffs,
                           const torch::Tensor& viewing_directions) const;
    
    // Compute SH basis functions
    torch::Tensor computeSHBasis(const torch::Tensor& directions, int degree) const;
    
    // Get number of SH coefficients for given degree
    static int getNumSHCoeffs(int degree);
    
    // Get total number of coefficients up to degree
    static int getTotalSHCoeffs(int degree);
    
private:
    int sh_degree_;
    torch::Device device_;
    
    // SH basis evaluation functions
    torch::Tensor evaluateY0(const torch::Tensor& dirs) const;
    torch::Tensor evaluateY1(const torch::Tensor& dirs) const;
    torch::Tensor evaluateY2(const torch::Tensor& dirs) const;
    torch::Tensor evaluateY3(const torch::Tensor& dirs) const;
    torch::Tensor evaluateY4(const torch::Tensor& dirs) const;
    
    // Helper functions
    torch::Tensor normalizeDirections(const torch::Tensor& directions) const;
};

// CUDA kernel declarations
#ifdef __CUDACC__
__device__ float evaluateSHBasisDevice(int l, int m, float x, float y, float z);

__global__ void evaluateSHKernel(
    const float* sh_coeffs,
    const float* viewing_directions,
    float* output,
    int num_points,
    int sh_degree,
    int num_coeffs
);

__global__ void computeSHBasisKernel(
    const float* directions,
    float* sh_basis,
    int num_points,
    int sh_degree,
    int total_coeffs
);
#endif

// CUDA wrapper functions
namespace cuda {
    torch::Tensor evaluateSHCuda(const torch::Tensor& sh_coeffs,
                                const torch::Tensor& viewing_directions,
                                int sh_degree);
    
    torch::Tensor computeSHBasisCuda(const torch::Tensor& directions, 
                                   int degree);
}

} // namespace rendering
} // namespace gaussian_splatting
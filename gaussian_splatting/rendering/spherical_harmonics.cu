#include "spherical_harmonics.hpp"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

namespace gaussian_splatting {
namespace rendering {

__device__ float evaluateSHBasisDevice(int l, int m, float x, float y, float z) {
    if (l == 0) {
        // Y_0^0 = 1/(2*sqrt(pi))
        return 0.28209479177387814f;
    }
    else if (l == 1) {
        if (m == -1) return 0.4886025119029199f * y;      // Y_1^{-1}
        else if (m == 0) return 0.4886025119029199f * z;  // Y_1^0
        else if (m == 1) return 0.4886025119029199f * x;  // Y_1^1
    }
    else if (l == 2) {
        if (m == -2) return 0.27313710764801976f * (x*x - y*y);          // Y_2^{-2}
        else if (m == -1) return 0.5462742152960396f * x * y;            // Y_2^{-1}
        else if (m == 0) return 0.31539156525252005f * (2.0f*z*z - x*x - y*y); // Y_2^0
        else if (m == 1) return 0.5462742152960396f * y * z;             // Y_2^1
        else if (m == 2) return 0.5462742152960396f * x * z;             // Y_2^2
    }
    
    return 0.0f;
}

__global__ void evaluateSHKernel(
    const float* sh_coeffs,
    const float* viewing_directions,
    float* output,
    int num_points,
    int sh_degree,
    int num_coeffs
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;
    
    // Normalize viewing direction
    float x = viewing_directions[idx * 3 + 0];
    float y = viewing_directions[idx * 3 + 1];
    float z = viewing_directions[idx * 3 + 2];
    
    float norm = sqrtf(x*x + y*y + z*z) + 1e-8f;
    x /= norm;
    y /= norm;
    z /= norm;
    
    // Evaluate SH basis and multiply with coefficients
    float result_r = 0.0f, result_g = 0.0f, result_b = 0.0f;
    
    int coeff_idx = 0;
    for (int l = 0; l <= sh_degree; ++l) {
        for (int m = -l; m <= l; ++m) {
            float sh_val = evaluateSHBasisDevice(l, m, x, y, z);
            
            result_r += sh_coeffs[idx * num_coeffs * 3 + coeff_idx * 3 + 0] * sh_val;
            result_g += sh_coeffs[idx * num_coeffs * 3 + coeff_idx * 3 + 1] * sh_val;
            result_b += sh_coeffs[idx * num_coeffs * 3 + coeff_idx * 3 + 2] * sh_val;
            
            coeff_idx++;
        }
    }
    
    output[idx * 3 + 0] = result_r;
    output[idx * 3 + 1] = result_g;
    output[idx * 3 + 2] = result_b;
}

__device__ int getTotalSHCoeffsDevice(int degree) {
    if (degree < 0) return 0;
    if (degree > 4) degree = 4;
    
    int total = 0;
    for (int l = 0; l <= degree; ++l) {
        total += 2 * l + 1;
    }
    return total;
}

__global__ void computeSHBasisKernel(
    const float* directions,
    float* sh_basis,
    int num_points,
    int sh_degree,
    int total_coeffs
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;
    
    // Normalize direction
    float x = directions[idx * 3 + 0];
    float y = directions[idx * 3 + 1];
    float z = directions[idx * 3 + 2];
    
    float norm = sqrtf(x*x + y*y + z*z) + 1e-8f;
    x /= norm;
    y /= norm;
    z /= norm;
    
    // Compute SH basis functions
    int coeff_idx = 0;
    for (int l = 0; l <= sh_degree; ++l) {
        for (int m = -l; m <= l; ++m) {
            float sh_val = evaluateSHBasisDevice(l, m, x, y, z);
            sh_basis[idx * total_coeffs + coeff_idx] = sh_val;
            coeff_idx++;
        }
    }
}

namespace cuda {

torch::Tensor evaluateSHCuda(const torch::Tensor& sh_coeffs,
                            const torch::Tensor& viewing_directions,
                            int sh_degree) {
    int num_points = viewing_directions.size(0);
    int num_coeffs = SphericalHarmonics::getTotalSHCoeffs(sh_degree);
    
    auto output = torch::zeros({num_points, 3}, 
                              torch::TensorOptions().device(sh_coeffs.device()).dtype(torch::kFloat32));
    
    const int block_size = 256;
    const int num_blocks = (num_points + block_size - 1) / block_size;
    
    evaluateSHKernel<<<num_blocks, block_size>>>(
        sh_coeffs.data_ptr<float>(),
        viewing_directions.data_ptr<float>(),
        output.data_ptr<float>(),
        num_points,
        sh_degree,
        num_coeffs
    );
    
    cudaDeviceSynchronize();
    
    return output;
}

torch::Tensor computeSHBasisCuda(const torch::Tensor& directions, int degree) {
    int num_points = directions.size(0);
    int total_coeffs = SphericalHarmonics::getTotalSHCoeffs(degree);
    
    auto sh_basis = torch::zeros({num_points, total_coeffs},
                                torch::TensorOptions().device(directions.device()).dtype(torch::kFloat32));
    
    const int block_size = 256;
    const int num_blocks = (num_points + block_size - 1) / block_size;
    
    computeSHBasisKernel<<<num_blocks, block_size>>>(
        directions.data_ptr<float>(),
        sh_basis.data_ptr<float>(),
        num_points,
        degree,
        total_coeffs
    );
    
    cudaDeviceSynchronize();
    
    return sh_basis;
}

} // namespace cuda
} // namespace rendering
} // namespace gaussian_splatting
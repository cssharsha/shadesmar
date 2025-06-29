#include "batch_gpu_manager.hpp"
#include <logging/logging.hpp>

namespace gaussian_splatting {
namespace utils {

BatchGPUManager::BatchGPUManager(const training::TrainingConfig& config) 
    : config_(config), device_(torch::kCPU) {
    
    // Try to use CUDA if available
    if (torch::cuda::is_available()) {
        device_ = torch::kCUDA;
        LOG(INFO) << "BatchGPUManager using CUDA device";
    } else {
        LOG(WARNING) << "CUDA not available, using CPU";
    }
}

BatchGPUManager::~BatchGPUManager() {
    // Cleanup handled by torch tensors
}

bool BatchGPUManager::loadBatchToGPU(const core::types::GaussianSplatBatch& batch, GPUBatchData& gpu_data) {
    LOG(INFO) << "Loading batch " << batch.batch_id << " to GPU (" << batch.size() << " splats)";
    
    if (batch.splats.empty()) {
        LOG(ERROR) << "Empty batch cannot be loaded to GPU";
        return false;
    }
    
    // Check memory requirements
    size_t estimated_size = batch.size() * sizeof(float) * 32; // Rough estimate
    if (!hasEnoughMemoryForBatch(estimated_size)) {
        LOG(ERROR) << "Insufficient GPU memory for batch " << batch.batch_id;
        return false;
    }
    
    // Convert batch to tensors
    if (!convertBatchToTensors(batch, gpu_data)) {
        LOG(ERROR) << "Failed to convert batch to tensors";
        return false;
    }
    
    // Move to GPU
    try {
        gpu_data.positions = gpu_data.positions.to(device_);
        gpu_data.rotations = gpu_data.rotations.to(device_);
        gpu_data.scales = gpu_data.scales.to(device_);
        gpu_data.opacities = gpu_data.opacities.to(device_);
        gpu_data.sh_coeffs = gpu_data.sh_coeffs.to(device_);
        gpu_data.colors = gpu_data.colors.to(device_);
        gpu_data.device = device_;
        
        LOG(INFO) << "Successfully loaded batch " << batch.batch_id << " to GPU";
        return true;
        
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to move batch to GPU: " << e.what();
        gpu_data.clear();
        return false;
    }
}

bool BatchGPUManager::convertBatchToTensors(const core::types::GaussianSplatBatch& batch, GPUBatchData& gpu_data) {
    int num_splats = batch.size();
    
    // Initialize tensors
    auto opts = torch::TensorOptions().dtype(torch::kFloat32);
    gpu_data.positions = torch::zeros({num_splats, 3}, opts);
    gpu_data.rotations = torch::zeros({num_splats, 4}, opts);
    gpu_data.scales = torch::zeros({num_splats, 3}, opts);
    gpu_data.opacities = torch::zeros({num_splats, 1}, opts);
    gpu_data.colors = torch::zeros({num_splats, 3}, opts);
    
    // Get accessors for efficient data copy
    auto pos_acc = gpu_data.positions.accessor<float, 2>();
    auto rot_acc = gpu_data.rotations.accessor<float, 2>();
    auto scale_acc = gpu_data.scales.accessor<float, 2>();
    auto opacity_acc = gpu_data.opacities.accessor<float, 2>();
    auto color_acc = gpu_data.colors.accessor<float, 2>();
    
    // Copy data from splats
    for (int i = 0; i < num_splats; ++i) {
        const auto& splat = batch.splats[i];
        
        // Position
        pos_acc[i][0] = splat.position[0];
        pos_acc[i][1] = splat.position[1];
        pos_acc[i][2] = splat.position[2];
        
        // Rotation (initialize as identity quaternion)
        rot_acc[i][0] = 1.0f; // w
        rot_acc[i][1] = 0.0f; // x
        rot_acc[i][2] = 0.0f; // y
        rot_acc[i][3] = 0.0f; // z
        
        // Scale (initialize from covariance)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(splat.covariance);
        auto eigenvalues = eigen_solver.eigenvalues();
        scale_acc[i][0] = std::sqrt(std::max(1e-6, eigenvalues[0]));
        scale_acc[i][1] = std::sqrt(std::max(1e-6, eigenvalues[1]));
        scale_acc[i][2] = std::sqrt(std::max(1e-6, eigenvalues[2]));
        
        // Opacity
        opacity_acc[i][0] = splat.opacity;
        
        // Color
        color_acc[i][0] = splat.color[0];
        color_acc[i][1] = splat.color[1];
        color_acc[i][2] = splat.color[2];
    }
    
    // Initialize SH coefficients
    int sh_dim = config_.sh_degree * config_.sh_degree;
    gpu_data.sh_coeffs = torch::zeros({num_splats, sh_dim, 3}, opts);
    initializeSHCoefficients(gpu_data.sh_coeffs, gpu_data.colors, num_splats);
    
    gpu_data.num_gaussians = num_splats;
    gpu_data.batch_id = batch.batch_id;
    
    return true;
}

void BatchGPUManager::initializeSHCoefficients(torch::Tensor& sh_coeffs, const torch::Tensor& colors, int num_gaussians) {
    // Initialize first SH coefficient (DC component) with RGB colors
    auto sh_acc = sh_coeffs.accessor<float, 3>();
    auto color_acc = colors.accessor<float, 2>();
    
    for (int i = 0; i < num_gaussians; ++i) {
        sh_acc[i][0][0] = color_acc[i][0];
        sh_acc[i][0][1] = color_acc[i][1];
        sh_acc[i][0][2] = color_acc[i][2];
        
        // Higher order coefficients initialized to zero
        for (int j = 1; j < config_.sh_degree * config_.sh_degree; ++j) {
            sh_acc[i][j][0] = 0.0f;
            sh_acc[i][j][1] = 0.0f;
            sh_acc[i][j][2] = 0.0f;
        }
    }
}

size_t BatchGPUManager::getCurrentGPUMemoryUsage() {
    // LibTorch C++ doesn't expose detailed memory stats like Python API
    // Return 0 for now, can be implemented with CUDA driver API if needed
    return 0;
}

bool BatchGPUManager::hasEnoughMemoryForBatch(size_t estimated_batch_size) {
    // Simple heuristic: allow batches up to config limit
    return estimated_batch_size <= config_.max_gpu_memory_per_batch;
}

void BatchGPUManager::clearBatchFromGPU(GPUBatchData& gpu_data) {
    LOG(INFO) << "Clearing batch " << gpu_data.batch_id << " from GPU";
    gpu_data.clear();
    // Note: LibTorch C++ doesn't expose empty_cache like Python API
}

bool BatchGPUManager::isDeviceAvailable() const {
    return device_.is_cuda() ? torch::cuda::is_available() : true;
}

} // namespace utils
} // namespace gaussian_splatting
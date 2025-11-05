#include "batch_gpu_manager.hpp"
#include <logging/logging.hpp>
#include "gaussian_splatting/training/keyframe_batch.hpp"
#include "gaussian_splatting/utils/torch_utils.hpp"

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

BatchGPUManager::~BatchGPUManager() {}

bool BatchGPUManager::loadBatchToTensors(const core::types::GaussianSplatBatch& batch,
                                         GaussianTensors& gpu_tensors) {
    LOG(INFO) << "Loading batch " << batch.batch_id << " to GPU (" << batch.size() << " splats)";

    if (batch.splats.empty()) {
        LOG(ERROR) << "Empty batch cannot be loaded to GPU";
        return false;
    }

    // Check memory requirements
    size_t estimated_size = batch.size() * sizeof(float) * 32;  // Rough estimate
    LOG(INFO) << "Estimated memory size: " << estimated_size
              << ", threshold: " << config_.max_gpu_memory_per_batch;
    if (!hasEnoughMemoryForBatch(estimated_size)) {
        LOG(ERROR) << "Insufficient GPU memory for batch " << batch.batch_id;
        return false;
    }

    // Convert batch to tensors
    if (!gpu_tensors.fromSplats(batch.splats)) {
        LOG(ERROR) << "Failed to convert batch to tensors";
        return false;
    }
    LOG(INFO) << "Did all the initializations correctly";
    return true;
}

bool BatchGPUManager::loadToGPU(GaussianTensors& gpu_tensors) {
    std::cout << "Computing positions size" << std::endl;
    auto positions_size = utils::logical_nbytes(gpu_tensors.get_positions()) / 1024.0 / 1024.0;
    std::cout << "Computing scales size" << std::endl;
    auto scales_size = utils::logical_nbytes(gpu_tensors.get_scales()) / 1024.0 / 1024.0;
    std::cout << "Computing opacities size" << std::endl;
    auto opacities_size = utils::logical_nbytes(gpu_tensors.get_opacities()) / 1024.0 / 1024.0;
    std::cout << "Computing sh_coefficients size" << std::endl;
    auto sh_coefficients_size =
        utils::logical_nbytes(gpu_tensors.get_sh_coefficients()) / 1024.0 / 1024.0;
    std::cout << "Computing rotations size" << std::endl;
    auto rotations_size = utils::logical_nbytes(gpu_tensors.get_rotations()) / 1024.0 / 1024.0;

    auto total_size = positions_size + scales_size + opacities_size +
                      sh_coefficients_size + rotations_size;
    LOG(INFO) << "Loading tensors to GPU: positions=" << positions_size
              << ", scales=" << scales_size
              << ", opacities=" << opacities_size << ", sh_coefficients=" << sh_coefficients_size
              << ", rotations=" << rotations_size << " : total=" << total_size;

    if (positions_size + scales_size + opacities_size + sh_coefficients_size +
            rotations_size >
        config_.max_gpu_memory_per_batch) {
        LOG(ERROR) << "Batch too large to fit in GPU memory";
        return false;
    }

    // Load to GPU
    gpu_tensors.to(device_);

    return true;
}

bool BatchGPUManager::loadToGPU(training::KeyframeBatch& batch) {
    std::cout << "Computing images size" << std::endl;
    auto images_size = utils::logical_nbytes(batch.images) / 1024.0 / 1024.0;
    std::cout << "Computing camera_poses size" << std::endl;
    auto camera_poses_size = utils::logical_nbytes(batch.camera_poses) / 1024.0 / 1024.0;
    std::cout << "Computing camera_intrinsics size" << std::endl;
    auto camera_intrinsics_size = utils::logical_nbytes(batch.camera_intrinsics) / 1024.0 / 1024.0;
    // std::cout << "Computing depths size" << std::endl;
    // auto depths_size = utils::logical_nbytes(batch.depths) / 1024.0 / 1024.0;
    auto depths_size = 0;

    auto total_size = images_size + camera_poses_size + camera_intrinsics_size;
    LOG(INFO) << "Loading batch to GPU: images=" << images_size
              << ", camera_poses=" << camera_poses_size
              << ", camera_intrinsics=" << camera_intrinsics_size << ", depths=" << depths_size
              << " : total=" << total_size;

    if (images_size + camera_poses_size + camera_intrinsics_size + depths_size >
        config_.max_gpu_memory_per_batch) {
        LOG(ERROR) << "Batch too large to fit in GPU memory";
        return false;
    }
    batch.to(device_);
    return true;
}

size_t BatchGPUManager::getCurrentGPUMemoryUsage() {
    // Should probablye get a cuda api to get this
    return 0;
}

bool BatchGPUManager::hasEnoughMemoryForBatch(size_t estimated_batch_size) {
    // Simple heuristic: allow batches up to config limit
    return estimated_batch_size <= config_.max_gpu_memory_per_batch;
}

void BatchGPUManager::clearBatchFromGPU(GaussianTensors& gaussian_tensors) {
    // Note: LibTorch C++ doesn't expose empty_cache like Python API
}

bool BatchGPUManager::isDeviceAvailable() const {
    return device_.is_cuda() ? torch::cuda::is_available() : true;
}

}  // namespace utils
}  // namespace gaussian_splatting

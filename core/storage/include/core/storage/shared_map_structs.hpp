#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>

namespace core {
namespace storage {

// Memory-efficient shared memory header
// Only contains indexes and metadata, actual data stays on disk
struct SharedMapStoreHeader {
    // Magic number for validation
    static constexpr uint32_t MAGIC_NUMBER = 0x4D415053; // "MAPS"
    uint32_t magic;
    uint32_t version;
    
    // Version counters for cache invalidation (atomic for lock-free reads)
    std::atomic<uint64_t> keyframe_index_version;
    std::atomic<uint64_t> keypoint_index_version; 
    std::atomic<uint64_t> splat_index_version;
    
    // Current counts
    std::atomic<uint64_t> total_keyframes;
    std::atomic<uint64_t> total_keypoints;
    std::atomic<uint64_t> total_splats;
    
    // Process health status
    std::atomic<bool> vslam_process_healthy;
    std::atomic<bool> gs_process_healthy;
    std::atomic<uint64_t> last_vslam_update_ns;
    std::atomic<uint64_t> last_gs_update_ns;
    
    // Data availability flags
    std::atomic<bool> tf_tree_available;
    std::atomic<bool> keyframes_available;
    std::atomic<bool> keypoints_available;
    std::atomic<uint64_t> data_version;  // Incremented when new data is written
    
    // Capacity limits for shared arrays
    uint32_t max_keyframes;
    uint32_t max_keypoints_per_batch;
    uint32_t max_splat_batches;
    
    SharedMapStoreHeader() {
        magic = MAGIC_NUMBER;
        version = 1;
        keyframe_index_version.store(0);
        keypoint_index_version.store(0);
        splat_index_version.store(0);
        total_keyframes.store(0);
        total_keypoints.store(0);
        total_splats.store(0);
        vslam_process_healthy.store(false);
        gs_process_healthy.store(false);
        last_vslam_update_ns.store(0);
        last_gs_update_ns.store(0);
        tf_tree_available.store(false);
        keyframes_available.store(false);
        keypoints_available.store(false);
        data_version.store(0);
        max_keyframes = 10000;
        max_keypoints_per_batch = 1000;
        max_splat_batches = 100;
    }
};

// Shared index for keyframes (not the actual keyframe data)
struct SharedKeyFrameIndex {
    uint64_t id;
    uint64_t disk_offset;      // Position in .dat file
    uint32_t data_size;        // Size on disk
    uint64_t timestamp_ns;
    bool is_optimized;
    bool is_valid;             // For sparse arrays
    
    SharedKeyFrameIndex() : id(0), disk_offset(0), data_size(0), 
                           timestamp_ns(0), is_optimized(false), is_valid(false) {}
};

// Shared index for keypoint batches
struct SharedKeyPointIndex {
    uint64_t batch_id;
    uint64_t keyframe_id;      // Associated keyframe
    uint64_t disk_offset;
    uint32_t data_size;
    uint32_t num_keypoints;
    bool is_valid;
    
    SharedKeyPointIndex() : batch_id(0), keyframe_id(0), disk_offset(0), 
                           data_size(0), num_keypoints(0), is_valid(false) {}
};

// Shared index for gaussian splat batches
struct SharedSplatIndex {
    uint64_t batch_id;
    uint64_t disk_offset;
    uint32_t data_size;
    uint32_t num_gaussians;
    uint64_t timestamp_ns;
    bool is_valid;
    
    SharedSplatIndex() : batch_id(0), disk_offset(0), data_size(0), 
                        num_gaussians(0), timestamp_ns(0), is_valid(false) {}
};

// Complete shared memory layout
struct SharedMapStoreRegion {
    SharedMapStoreHeader header;
    
    // Variable-length arrays follow header
    // Actual size determined by header.max_* values
    SharedKeyFrameIndex keyframe_indices[];
    // SharedKeyPointIndex keypoint_indices[] follows after keyframes
    // SharedSplatIndex splat_indices[] follows after keypoints
    
    // Helper methods to calculate offsets
    static size_t getKeyPointIndicesOffset(uint32_t max_keyframes) {
        return sizeof(SharedMapStoreHeader) + 
               max_keyframes * sizeof(SharedKeyFrameIndex);
    }
    
    static size_t getSplatIndicesOffset(uint32_t max_keyframes, uint32_t max_keypoints) {
        return getKeyPointIndicesOffset(max_keyframes) + 
               max_keypoints * sizeof(SharedKeyPointIndex);
    }
    
    static size_t getTotalSize(uint32_t max_keyframes, uint32_t max_keypoints, uint32_t max_splats) {
        return getSplatIndicesOffset(max_keyframes, max_keypoints) + 
               max_splats * sizeof(SharedSplatIndex);
    }
};

} // namespace storage
} // namespace core
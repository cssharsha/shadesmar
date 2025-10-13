#include "splat_index_manager.hpp"
#include <iostream>
#include <algorithm>

namespace gaussian_splatting {
namespace utils {

void SplatIndexManager::setActiveIndices(const std::vector<int>& global_indices) {
    active_global_indices_ = global_indices;
    rebuildMaps();

    // Initialize dirty flags (all clean initially)
    dirty_flags_.resize(active_global_indices_.size(), false);

    std::cout << "SplatIndexManager: Loaded " << active_global_indices_.size()
              << " active splats onto GPU" << std::endl;
}

bool SplatIndexManager::isActive(int global_idx) const {
    return global_to_local_.find(global_idx) != global_to_local_.end();
}

int SplatIndexManager::globalToLocal(int global_idx) const {
    auto it = global_to_local_.find(global_idx);
    return (it != global_to_local_.end()) ? it->second : -1;
}

int SplatIndexManager::localToGlobal(int local_idx) const {
    auto it = local_to_global_.find(local_idx);
    return (it != local_to_global_.end()) ? it->second : -1;
}

void SplatIndexManager::afterDuplicate(int num_new_splats, int next_global_id) {
    std::cout << "SplatIndexManager::afterDuplicate: " << num_new_splats
              << " new splats, next global ID: " << next_global_id << std::endl;

    // GPU now has: [original_splats..., duplicated_splats...]
    // The duplicated splats are appended at the end
    int original_local_count = active_global_indices_.size();

    // Assign new global IDs to the duplicated splats
    for (int i = 0; i < num_new_splats; ++i) {
        active_global_indices_.push_back(next_global_id + i);
    }

    // Update total global count
    total_global_splats_ = next_global_id + num_new_splats;

    rebuildMaps();

    // Resize dirty flags and mark new splats as dirty (need to be synced to CPU)
    dirty_flags_.resize(active_global_indices_.size(), false);
    for (int i = original_local_count; i < active_global_indices_.size(); ++i) {
        dirty_flags_[i] = true;
    }
}

void SplatIndexManager::afterSplit(int num_original_split, int split_size, int next_global_id) {
    std::cout << "SplatIndexManager::afterSplit: " << num_original_split
              << " originals split into " << split_size << " each, next global ID: "
              << next_global_id << std::endl;

    // GPU now has: [non_split_splats..., split_splats...]
    // Each original that was split is replaced by split_size new splats
    // The split splats are appended at the end, originals are kept in rest_idxs

    int num_new_splats = num_original_split * split_size;
    int new_local_count = active_global_indices_.size() + num_original_split;  // net gain

    // Note: The split operation in Strategy already handles the GPU tensors
    // Here we just track the new global IDs for the split results
    // The original splats remain in their positions, new ones are appended

    std::vector<int> new_active_indices;
    new_active_indices.reserve(new_local_count);

    // The Strategy's splitSplats creates: [rest_idxs, split_results]
    // We need to know which were split vs which weren't
    // For now, assume the last num_new_splats are the newly created ones

    // Keep existing indices (these include both non-split and the originals that will be removed)
    // This is a simplification - in reality, Strategy removes originals and adds splits
    // The actual implementation should match Strategy's index reordering

    // For simplicity: append new global IDs for all split results
    for (int i = 0; i < num_original_split * (split_size - 1); ++i) {
        active_global_indices_.push_back(next_global_id + i);
    }

    total_global_splats_ = next_global_id + num_original_split * (split_size - 1);

    rebuildMaps();
    dirty_flags_.resize(active_global_indices_.size(), true);  // Mark all as dirty for safety
}

void SplatIndexManager::afterPrune(const std::vector<int>& local_indices_kept) {
    std::cout << "SplatIndexManager::afterPrune: Keeping " << local_indices_kept.size()
              << " out of " << active_global_indices_.size() << " splats" << std::endl;

    // Build new active indices list with only the kept indices
    std::vector<int> new_active_indices;
    new_active_indices.reserve(local_indices_kept.size());

    std::vector<bool> new_dirty_flags;
    new_dirty_flags.reserve(local_indices_kept.size());

    for (int local_idx : local_indices_kept) {
        if (local_idx >= 0 && local_idx < active_global_indices_.size()) {
            new_active_indices.push_back(active_global_indices_[local_idx]);
            new_dirty_flags.push_back(dirty_flags_[local_idx]);
        }
    }

    active_global_indices_ = std::move(new_active_indices);
    dirty_flags_ = std::move(new_dirty_flags);

    rebuildMaps();
}

void SplatIndexManager::markDirty(const std::vector<int>& local_indices) {
    for (int local_idx : local_indices) {
        if (local_idx >= 0 && local_idx < dirty_flags_.size()) {
            dirty_flags_[local_idx] = true;
        }
    }
}

void SplatIndexManager::markAllDirty() {
    std::fill(dirty_flags_.begin(), dirty_flags_.end(), true);
}

void SplatIndexManager::clearDirtyFlags() {
    std::fill(dirty_flags_.begin(), dirty_flags_.end(), false);
}

void SplatIndexManager::reset() {
    active_global_indices_.clear();
    global_to_local_.clear();
    local_to_global_.clear();
    dirty_flags_.clear();
    total_global_splats_ = 0;
}

void SplatIndexManager::rebuildMaps() {
    global_to_local_.clear();
    local_to_global_.clear();

    for (size_t local_idx = 0; local_idx < active_global_indices_.size(); ++local_idx) {
        int global_idx = active_global_indices_[local_idx];
        global_to_local_[global_idx] = local_idx;
        local_to_global_[local_idx] = global_idx;
    }
}

void SplatIndexManager::printMapping() const {
    std::cout << "=== SplatIndexManager Mapping ===" << std::endl;
    std::cout << "Total global splats: " << total_global_splats_ << std::endl;
    std::cout << "Active GPU splats: " << active_global_indices_.size() << std::endl;
    std::cout << "Sample mappings (first 10):" << std::endl;

    for (size_t i = 0; i < std::min(size_t(10), active_global_indices_.size()); ++i) {
        std::cout << "  Local " << i << " → Global " << active_global_indices_[i]
                  << " (dirty: " << (dirty_flags_[i] ? "yes" : "no") << ")" << std::endl;
    }
}

}  // namespace utils
}  // namespace gaussian_splatting

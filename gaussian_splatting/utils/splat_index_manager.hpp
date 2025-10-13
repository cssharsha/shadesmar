#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace gaussian_splatting {
namespace utils {

/**
 * Manages index mapping between CPU (full dataset) and GPU (active subset) splats.
 *
 * Key responsibilities:
 * - Track which global (CPU) splats are active on GPU
 * - Provide bidirectional index mapping: global_idx ↔ local_idx
 * - Handle index updates after densification/pruning operations
 * - Track dirty flags for synchronization
 */
class SplatIndexManager {
public:
    SplatIndexManager() = default;

    // Initialize with a set of active global indices
    void setActiveIndices(const std::vector<int>& global_indices);

    // Query mapping
    bool isActive(int global_idx) const;
    int globalToLocal(int global_idx) const;  // Returns -1 if not active
    int localToGlobal(int local_idx) const;   // Returns -1 if invalid

    // Get all active indices
    const std::vector<int>& getActiveGlobalIndices() const { return active_global_indices_; }
    size_t getActiveCount() const { return active_global_indices_.size(); }

    // Update operations after densification/pruning
    // These are called from Strategy after split/duplicate/prune operations

    // Called after duplicateSplats: adds new global indices for duplicated splats
    // Parameters:
    //   num_new_splats: number of splats that were duplicated on GPU
    //   next_global_id: next available global ID in CPU storage
    void afterDuplicate(int num_new_splats, int next_global_id);

    // Called after splitSplats: adds new global indices for split splats
    // Each split creates (split_size - 1) new splats per original
    // Parameters:
    //   num_original_split: number of original splats that were split on GPU
    //   split_size: how many splats each original became (typically 2)
    //   next_global_id: next available global ID in CPU storage
    void afterSplit(int num_original_split, int split_size, int next_global_id);

    // Called after pruneSplats: removes pruned indices
    // Parameters:
    //   local_indices_kept: indices (in GPU/local space) that were kept after pruning
    void afterPrune(const std::vector<int>& local_indices_kept);

    // Mark splats as dirty (need CPU ← GPU sync)
    void markDirty(const std::vector<int>& local_indices);
    void markAllDirty();
    const std::vector<bool>& getDirtyFlags() const { return dirty_flags_; }
    void clearDirtyFlags();

    // Get statistics
    int getTotalGlobalSplats() const { return total_global_splats_; }
    void setTotalGlobalSplats(int count) { total_global_splats_ = count; }

    // Clear all state
    void reset();

    // Debug
    void printMapping() const;

private:
    // Active global indices (CPU indices) currently loaded on GPU
    std::vector<int> active_global_indices_;

    // Bidirectional mapping
    std::unordered_map<int, int> global_to_local_;  // global_idx → local_idx
    std::unordered_map<int, int> local_to_global_;  // local_idx → global_idx

    // Dirty flags: true if GPU splat needs to be synced back to CPU
    std::vector<bool> dirty_flags_;

    // Total count of splats in CPU storage (for bounds checking)
    int total_global_splats_ = 0;

    // Helper: rebuild maps after indices change
    void rebuildMaps();
};

}  // namespace utils
}  // namespace gaussian_splatting

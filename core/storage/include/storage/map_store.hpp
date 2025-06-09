namespace storage {

class MapStore {
public:
    // ... existing public methods ...

    // Three-queue system methods
    void addToUnprocessedCache(const KeyFramePtr& keyframe);
    void moveToProcessedNonOptimized(uint64_t keyframe_id);
    void moveToProcessedOptimized(uint64_t keyframe_id);
    void moveBatchToProcessedOptimized(const std::vector<uint64_t>& keyframe_ids);

    // Queue-specific accessors
    std::vector<KeyFramePtr> getUnprocessedKeyFrames() const;
    std::vector<KeyFramePtr> getProcessedNonOptimizedKeyFrames() const;
    std::vector<KeyFramePtr> getProcessedOptimizedKeyFrames() const;

    KeyFramePtr getUnprocessedKeyFrame(uint64_t id) const;
    KeyFramePtr getProcessedNonOptimizedKeyFrame(uint64_t id) const;
    KeyFramePtr getProcessedOptimizedKeyFrame(uint64_t id) const;

    // Queue state queries
    size_t getUnprocessedCount() const;
    size_t getProcessedNonOptimizedCount() const;
    size_t getProcessedOptimizedCount() const;

    bool hasUnprocessedKeyFrame(uint64_t id) const;
    bool hasProcessedNonOptimizedKeyFrame(uint64_t id) const;
    bool hasProcessedOptimizedKeyFrame(uint64_t id) const;

    // Map point eviction
    void evictOldMapPoints(uint64_t current_keyframe_id, size_t max_age = 30);

    // Batch operations for optimization
    std::vector<KeyFramePtr> getBatchForOptimization(size_t max_batch_size = 50) const;
    void markBatchAsOptimized(const std::vector<uint64_t>& keyframe_ids);

// ... existing code ...

private:
    // Three-queue system caches
    mutable std::map<uint64_t, KeyFramePtr> unprocessed_cache_;
    mutable std::map<uint64_t, KeyFramePtr> processed_non_optimized_cache_;
    mutable std::map<uint64_t, KeyFramePtr> processed_optimized_cache_;

    // Queue-specific mutexes
    mutable std::shared_mutex unprocessed_cache_mutex_;
    mutable std::shared_mutex processed_non_optimized_cache_mutex_;
    mutable std::shared_mutex processed_optimized_cache_mutex_;

    // Map point eviction tracking
    mutable std::map<uint32_t, uint64_t> map_point_last_seen_;
    mutable std::mutex map_point_eviction_mutex_;

    // ... existing private members ...

    // Helper methods for queue management
    void moveKeyFrameBetweenCaches(uint64_t keyframe_id,
                                   std::map<uint64_t, KeyFramePtr>& from_cache,
                                   std::shared_mutex& from_mutex,
                                   std::map<uint64_t, KeyFramePtr>& to_cache,
                                   std::shared_mutex& to_mutex);

    void updateMapPointLastSeen(uint64_t keyframe_id);
    void evictMapPointToDisk(uint32_t map_point_id);

    // Modified background sync to handle processed optimized queue
    void syncProcessedOptimizedToDisk();
};

// ... existing code ...
}
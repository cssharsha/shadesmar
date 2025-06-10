#include <core/storage/map_store.hpp>
#include <chrono>
#include <iomanip>
#include <logging/logging.hpp>

namespace core {
namespace storage {

MapStore::MapStore(const std::string& map_base_filepath) {
    if (!map_base_filepath.empty()) {
        initializeFilePaths(map_base_filepath);
    }
    metadata_.set_version("1.0-disk");
    metadata_.set_created_by("MapStoreDisk");
    auto* bounds = metadata_.mutable_bounds();
    bounds->set_min_x(std::numeric_limits<double>::max());
    bounds->set_min_y(std::numeric_limits<double>::max());
    bounds->set_min_z(std::numeric_limits<double>::max());
    bounds->set_max_x(std::numeric_limits<double>::lowest());
    bounds->set_max_y(std::numeric_limits<double>::lowest());
    bounds->set_max_z(std::numeric_limits<double>::lowest());

    // Initialize dirty flags
    keyframes_dirty_ = false;
    factors_dirty_ = false;
    keypoints_dirty_ = false;
    splat_batches_dirty_ = false;
    metadata_dirty_ = false;

    // Initialize atomic queue system
    initializeAtomicQueues();
    
    // Start write worker thread
    startWriteWorker();

    LOG(INFO) << "MapStore initialized with callback-based write system and single write worker";
}

MapStore::~MapStore() {
    // Stop write worker first
    stopWriteWorker();
    
    // Wait for any in-progress write operation to complete
    if (write_in_progress_.load()) {
        LOG(INFO) << "Waiting for final write operation to complete before MapStore destruction";
        waitForWriteCompletion();
    }

    // Perform final sync if there are dirty changes or pending data in queues
    bool has_pending_data = false;
    {
        std::shared_lock<std::shared_mutex> unprocessed_lock(unprocessed_cache_mutex_);
        std::shared_lock<std::shared_mutex> non_opt_lock(processed_non_optimized_queue_mutex_);
        std::shared_lock<std::shared_mutex> opt_lock(processed_optimized_queue_mutex_);

        has_pending_data = !unprocessed_cache_.empty() ||
                          !processed_non_optimized_queue_->empty() ||
                          !processed_optimized_queue_->empty();
    }

    if (keyframes_dirty_.load() || factors_dirty_.load() || keypoints_dirty_.load() ||
        splat_batches_dirty_.load() || metadata_dirty_.load() || has_pending_data) {
        LOG(INFO) << "Performing final sync before MapStore destruction";
        performAtomicSync();
    }

    LOG(INFO) << "MapStore destroyed with callback-based write system";
}

bool MapStore::initializeFilePaths(const std::string& map_base_filepath) {
    base_filepath_ = map_base_filepath;
    data_filepath_ = base_filepath_ + ".dat";
    index_filepath_ = base_filepath_ + ".idx";
    metadata_filepath_ = base_filepath_ + ".meta";
    splat_data_filepath_ = base_filepath_ + ".splats";
    splat_index_filepath_ = base_filepath_ + ".splat_idx";
    transform_tree_filepath_ = base_filepath_ + ".tf_tree";
    LOG(INFO) << "MapStore initialized with data file: " << data_filepath_ << std::endl;
    return true;
}

bool MapStore::openDataFileForAppend(std::fstream& file_stream) {
    if (data_filepath_.empty()) {
        LOG(ERROR) << "Data filepath not set." << std::endl;
        return false;
    }
    file_stream.open(data_filepath_, std::ios::out | std::ios::app | std::ios::binary);
    if (!file_stream.is_open()) {
        LOG(ERROR) << "Failed to open data file for append: " << data_filepath_ << std::endl;
        return false;
    }
    return true;
}

bool MapStore::openDataFileForRead(std::fstream& file_stream) const {
    if (data_filepath_.empty()) {
        LOG(ERROR) << "Data filepath not set." << std::endl;
        return false;
    }
    file_stream.open(data_filepath_, std::ios::in | std::ios::binary);
    if (!file_stream.is_open()) {
        LOG(ERROR) << "Failed to open data file for read: " << data_filepath_ << std::endl;
        return false;
    }
    return true;
}

template <typename ProtoType>
bool MapStore::writeProtoMessage(std::fstream& stream, const ProtoType& message,
                                 proto::FileLocation& out_location) {
    std::string serialized_data;
    if (!message.SerializeToString(&serialized_data)) {
        LOG(ERROR) << "Failed to serialize message." << std::endl;
        return false;
    }

    uint32_t message_len = static_cast<uint32_t>(serialized_data.length());

    stream.seekp(0, std::ios::end);  // Go to end of file
    out_location.set_offset(static_cast<uint64_t>(stream.tellp()));
    out_location.set_length(message_len);

    stream.write(reinterpret_cast<const char*>(&message_len), sizeof(message_len));
    stream.write(serialized_data.data(), message_len);

    if (stream.fail()) {
        LOG(ERROR) << "Failed to write message to data file." << std::endl;
        return false;
    }
    return true;
}

template <typename ProtoType, typename CppType>
std::optional<CppType> MapStore::readProtoMessage(
    const proto::FileLocation& location,
    std::function<CppType(const ProtoType&)> fromProtoConverter) const {
    std::fstream data_file;
    if (!openDataFileForRead(data_file)) {
        return std::nullopt;
    }

    data_file.seekg(location.offset());
    if (data_file.fail()) {
        LOG(ERROR) << "Failed to seek to offset " << location.offset() << std::endl;
        return std::nullopt;
    }

    uint32_t message_len_on_disk;
    data_file.read(reinterpret_cast<char*>(&message_len_on_disk), sizeof(message_len_on_disk));
    if (data_file.fail() || message_len_on_disk != location.length()) {
        LOG(ERROR) << "Failed to read message length or length mismatch. Expected: "
                   << location.length() << ", Got from disk prefix: " << message_len_on_disk
                   << " at offset " << location.offset() << std::endl;
        return std::nullopt;
    }

    std::vector<char> buffer(location.length());
    data_file.read(buffer.data(), location.length());
    if (data_file.fail()) {
        LOG(ERROR) << "Failed to read message data of length " << location.length() << std::endl;
        return std::nullopt;
    }

    ProtoType proto_message;
    if (!proto_message.ParseFromArray(buffer.data(), buffer.size())) {
        LOG(ERROR) << "Failed to parse message from disk." << std::endl;
        return std::nullopt;
    }

    return fromProtoConverter(proto_message);
}

void MapStore::updateMetadataBounds(const types::Pose& pose) {
    auto* bounds = metadata_.mutable_bounds();
    bounds->set_min_x(std::min(bounds->min_x(), pose.position.x()));
    bounds->set_min_y(std::min(bounds->min_y(), pose.position.y()));
    bounds->set_min_z(std::min(bounds->min_z(), pose.position.z()));
    bounds->set_max_x(std::max(bounds->max_x(), pose.position.x()));
    bounds->set_max_y(std::max(bounds->max_y(), pose.position.y()));
    bounds->set_max_z(std::max(bounds->max_z(), pose.position.z()));
}

bool MapStore::addKeyFrame(const KeyFramePtr& keyframe) {
    if (!keyframe)
        return false;

    // Only update in-memory structures - no disk I/O
    std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);

    bool is_update = keyframe_pending_writes_.count(keyframe->id) > 0 ||
                     keyframe_locations_.count(keyframe->id) > 0;

    if (is_update) {
        LOG(INFO) << "KeyFrame ID " << keyframe->id
                  << " marked for update with optimized pose";
    } else {
        LOG(INFO) << "KeyFrame ID " << keyframe->id
                  << " added to cache, pending disk write";
    }

    // Add to pending writes (will be written during next sync)
    keyframe_pending_writes_[keyframe->id] = keyframe;

    // Update cache immediately - inline to avoid recursive mutex lock
    cacheKeyFrameInternal(keyframe->id, keyframe);

    // Update timestamp index for new keyframes
    if (!is_update) {
        timestamp_to_keyframe_ids_[keyframe->pose.timestamp].push_back(keyframe->id);
    }

    // Update spatial index entry
    proto::KeyFrameIndexEntry kf_index_entry;
    kf_index_entry.set_keyframe_id(keyframe->id);
    kf_index_entry.set_timestamp(keyframe->pose.timestamp);
    // Note: location will be set during sync when written to disk

    bool found_in_spatial = false;
    for (auto& entry : keyframe_spatial_index_entries_) {
        if (entry.keyframe_id() == keyframe->id) {
            entry = kf_index_entry;  // Update existing
            found_in_spatial = true;
            break;
        }
    }
    if (!found_in_spatial) {
        keyframe_spatial_index_entries_.push_back(kf_index_entry);
    }

    updateMetadataBounds(keyframe->pose);

    // Mark as dirty for sync
    keyframes_dirty_ = true;
    metadata_dirty_ = true;

    LOG(INFO) << "KeyFrame " << keyframe->id << " cached and marked for sync";
    return true;
}

bool MapStore::addFactor(const types::Factor& factor) {
    // Only update in-memory structures - no disk I/O
    std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);

    if (factor_pending_writes_.count(factor.id) || factor_locations_.count(factor.id)) {
        LOG(INFO) << "Factor ID " << factor.id << " already exists, marking for update";
    }

    LOG(INFO) << "Adding factor " << factor.id << " to cache, pending disk write";

    // Add to pending writes (will be written during next sync)
    factor_pending_writes_[factor.id] = factor;

    // Update cache immediately - inline to avoid recursive mutex lock
    factor_cache_[factor.id] = factor;

    // Update reverse mapping: associate this factor with connected keyframes
    for (uint64_t keyframe_id : factor.connected_nodes) {
        auto& factor_list = keyframe_to_factor_ids_[keyframe_id];
        // Avoid duplicates
        if (std::find(factor_list.begin(), factor_list.end(), factor.id) == factor_list.end()) {
            factor_list.push_back(factor.id);
        }
    }

    LOG(INFO) << "Factor " << factor.id << " associated with " << factor.connected_nodes.size()
              << " keyframes and cached for sync";

    // Mark as dirty for sync
    factors_dirty_ = true;
    return true;
}

bool MapStore::addKeyPoint(const types::Keypoint& keypoint) {
    // Only update in-memory structures - no disk I/O
    std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);

    if (keypoint_pending_writes_.count(keypoint.id()) || keypoint_locations_.count(keypoint.id())) {
        LOG(INFO) << "KeyPoint ID " << keypoint.id() << " already exists, marking for update";
    }

    LOG(INFO) << "Adding keypoint " << keypoint.id() << " to cache, pending disk write";

    // Add to pending writes (will be written during next sync)
    keypoint_pending_writes_[keypoint.id()] = keypoint;

    // Update cache immediately - inline to avoid recursive mutex lock
    keypoint_cache_[keypoint.id()] = keypoint;

    // Mark as dirty for sync
    keypoints_dirty_ = true;
    return true;
}

size_t MapStore::getCurrentCacheSize() const {
    std::shared_lock<std::shared_mutex> lock(cache_mutex_);
    return keyframe_cache_.size();
}

KeyFramePtr MapStore::getKeyFrame(uint64_t id) const {
    // First check cache
    {
        std::shared_lock<std::shared_mutex> lock(cache_mutex_);
        auto cache_it = keyframe_cache_.find(id);
        if (cache_it != keyframe_cache_.end()) {
            // Found in cache, need to update LRU - upgrade to unique lock
            lock.unlock();
            std::unique_lock<std::shared_mutex> write_lock(cache_mutex_);

            // Recheck that it's still in cache after lock upgrade
            cache_it = keyframe_cache_.find(id);
            if (cache_it != keyframe_cache_.end()) {
                // Move to front of LRU list (mark as recently used)
                auto lru_it = keyframe_lru_map_.find(id);
                if (lru_it != keyframe_lru_map_.end()) {
                    keyframe_lru_list_.erase(lru_it->second);
                    keyframe_lru_list_.push_front(id);
                    lru_it->second = keyframe_lru_list_.begin();
                }
                return cache_it->second;
            }
            // Fall through to disk load if evicted between locks
        }
    }

    // Cache miss - load from disk
    auto it = keyframe_locations_.find(id);
    if (it == keyframe_locations_.end()) {
        return nullptr;
    }
    auto opt_kf_proto = readProtoMessage<proto::KeyFrame, proto::KeyFrame>(
        it->second, [](const proto::KeyFrame& p) { return p; });
    if (opt_kf_proto) {
        auto keyframe = types::KeyFrame::fromProto(opt_kf_proto.value());
        if (keyframe) {
            cacheKeyFrame(id, keyframe);
        }
        return keyframe;
    }
    return nullptr;
}

std::optional<types::Factor> MapStore::getFactor(uint64_t id) const {
    // First check cache
    {
        std::shared_lock<std::shared_mutex> lock(cache_mutex_);
        auto cache_it = factor_cache_.find(id);
        if (cache_it != factor_cache_.end()) {
            return cache_it->second;
        }
    }

    // Cache miss - load from disk
    auto it = factor_locations_.find(id);
    if (it == factor_locations_.end()) {
        return std::nullopt;
    }
    auto factor_opt = readProtoMessage<proto::Factor, types::Factor>(it->second, types::Factor::fromProto);
    if (factor_opt) {
        cacheFactor(id, factor_opt.value());
    }
    return factor_opt;
}

std::optional<types::Keypoint> MapStore::getKeyPoint(uint32_t id) const {
    // First check cache
    {
        std::shared_lock<std::shared_mutex> lock(cache_mutex_);
        auto cache_it = keypoint_cache_.find(id);
        if (cache_it != keypoint_cache_.end()) {
            return cache_it->second;
        }
    }

    // Cache miss - load from disk
    auto it = keypoint_locations_.find(id);
    if (it == keypoint_locations_.end()) {
        return std::nullopt;
    }
    auto keypoint_opt = readProtoMessage<proto::Keypoint, types::Keypoint>(it->second,
                                                              types::Keypoint::fromProto);
    if (keypoint_opt) {
        cacheKeyPoint(id, keypoint_opt.value());
    }
    return keypoint_opt;
}

bool MapStore::hasKeyPoint(uint32_t id) const {
    // Check cache first
    {
        std::shared_lock<std::shared_mutex> lock(cache_mutex_);
        if (keypoint_cache_.find(id) != keypoint_cache_.end()) {
            return true;
        }
        if (keypoint_pending_writes_.find(id) != keypoint_pending_writes_.end()) {
            return true;
        }
    }
    
    // Check disk locations
    return keypoint_locations_.find(id) != keypoint_locations_.end();
}

std::vector<KeyFramePtr> MapStore::getAllKeyFrames() const {
    std::vector<KeyFramePtr> all_kfs;
    for (const auto& pair : keyframe_locations_) {
        KeyFramePtr kf = getKeyFrame(pair.first);
        if (kf) {
            all_kfs.push_back(kf);
        } else {
            LOG(INFO) << "Unable to read kf for " << pair.first;
        }
    }
    LOG(INFO) << "Read " << all_kfs.size() << " of " << keyframe_locations_.size();
    return all_kfs;
}

std::vector<types::Factor> MapStore::getAllFactors() const {
    std::vector<types::Factor> all_factors;
    for (const auto& pair : factor_locations_) {
        auto factor = getFactor(pair.first);
        if (factor) {
            all_factors.push_back(factor.value());
        }
    }
    return all_factors;
}

std::vector<types::Keypoint> MapStore::getAllKeyPoints() const {
    std::vector<types::Keypoint> all_keypoints;
    for (const auto& pair : keypoint_locations_) {
        auto kp = getKeyPoint(pair.first);
        if (kp) {
            all_keypoints.push_back(kp.value());
        }
    }
    return all_keypoints;
}

std::vector<KeyFramePtr> MapStore::getKeyFramesByTimestamp(double timestamp) const {
    std::vector<KeyFramePtr> result_kfs;
    auto it = timestamp_to_keyframe_ids_.find(timestamp);
    if (it != timestamp_to_keyframe_ids_.end()) {
        for (uint64_t kf_id : it->second) {
            KeyFramePtr kf = getKeyFrame(kf_id);
            if (kf) {
                result_kfs.push_back(kf);
            }
        }
    }
    return result_kfs;
}

std::vector<KeyFramePtr> MapStore::getKeyFramesByTimestampRange(double start_timestamp,
                                                                double end_timestamp) const {
    std::vector<KeyFramePtr> result_kfs;
    for (auto it = timestamp_to_keyframe_ids_.lower_bound(start_timestamp);
         it != timestamp_to_keyframe_ids_.end() && it->first <= end_timestamp; ++it) {
        for (uint64_t kf_id : it->second) {
            KeyFramePtr kf = getKeyFrame(kf_id);
            if (kf) {
                result_kfs.push_back(kf);
            }
        }
    }
    return result_kfs;
}

std::vector<KeyFramePtr> MapStore::findKeyFramesNearPosition(const Eigen::Vector3d& target_position,
                                                             double radius, int max_results) const {
    std::vector<KeyFramePtr> nearby_kfs;
    std::vector<std::pair<double, uint64_t>> sorted_kf_ids;
    double radius_squared = radius * radius;

    for (const auto& kf_idx_entry : keyframe_spatial_index_entries_) {
        KeyFramePtr kf = getKeyFrame(kf_idx_entry.keyframe_id());
        if (kf) {
            double dist_sq = (kf->pose.position - target_position).squaredNorm();
            if (dist_sq <= radius_squared) {
                sorted_kf_ids.push_back({dist_sq, kf->id});
            }
        }
    }

    std::sort(sorted_kf_ids.begin(), sorted_kf_ids.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    for (const auto& dist_id_pair : sorted_kf_ids) {
        if (max_results != -1 && static_cast<int>(nearby_kfs.size()) >= max_results) {
            break;
        }
        KeyFramePtr kf = getKeyFrame(
            dist_id_pair.second);  // Fetch again, or pass KeyFramePtr if already fetched
        if (kf) {
            nearby_kfs.push_back(kf);
        }
    }
    return nearby_kfs;
}

bool MapStore::saveChanges() {
    std::lock_guard<std::mutex> lock(file_operations_mutex_);  // Protect file operations

    if (metadata_filepath_.empty() || index_filepath_.empty()) {
        LOG(ERROR) << "Filepaths not initialized. Cannot save." << std::endl;
        return false;
    }

    // Write all pending data to disk first
    bool write_success = writePendingDataToDisk();
    if (!write_success) {
        LOG(ERROR) << "Failed to write pending data to disk during saveChanges";
        return false;
    }

    // Update metadata timestamp
    metadata_.set_creation_timestamp_seconds(
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());

    // Save metadata
    std::fstream meta_stream(metadata_filepath_,
                             std::ios::out | std::ios::trunc | std::ios::binary);
    if (!meta_stream.is_open() || !metadata_.SerializeToOstream(&meta_stream)) {
        LOG(ERROR) << "Failed to save metadata to " << metadata_filepath_ << std::endl;
        return false;
    }
    meta_stream.close();
    LOG(INFO) << "Metadata saved to " << metadata_filepath_ << std::endl;

    // Build and save index
    proto::MapDiskIndex disk_index;
    for (const auto& pair : keyframe_locations_) {
        bool found_in_spatial = false;
        for (const auto& spatial_entry : keyframe_spatial_index_entries_) {
            if (spatial_entry.keyframe_id() == pair.first) {
                *disk_index.add_keyframe_entries() = spatial_entry;
                found_in_spatial = true;
                break;
            }
        }
        if (!found_in_spatial) {
            proto::KeyFrameIndexEntry* entry = disk_index.add_keyframe_entries();
            entry->set_keyframe_id(pair.first);
            *entry->mutable_location() = pair.second;
        }
    }
    for (const auto& pair : factor_locations_) {
        proto::FactorIndexEntry* entry = disk_index.add_factor_entries();
        entry->set_factor_id(pair.first);
        *entry->mutable_location() = pair.second;
    }
    for (const auto& pair : keypoint_locations_) {
        proto::KeyPointIndexEntry* entry = disk_index.add_keypoint_entries();
        entry->set_keypoint_id(pair.first);
        *entry->mutable_location() = pair.second;
    }

    std::fstream index_stream(index_filepath_, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!index_stream.is_open() || !disk_index.SerializeToOstream(&index_stream)) {
        LOG(ERROR) << "Failed to save index to " << index_filepath_ << std::endl;
        return false;
    }
    index_stream.close();
    LOG(INFO) << "Index saved to " << index_filepath_ << std::endl;

    // Clear dirty flags after successful save
    keyframes_dirty_ = false;
    factors_dirty_ = false;
    keypoints_dirty_ = false;
    metadata_dirty_ = false;

    return true;
}

bool MapStore::writePendingDataToDisk() {
    // Write all pending keyframes, factors, and keypoints to disk
    // This method assumes file_operations_mutex_ is already locked

    std::fstream data_file;
    if (!openDataFileForAppend(data_file)) {
        LOG(ERROR) << "Failed to open data file for writing pending data";
        return false;
    }

    size_t keyframes_written = 0, factors_written = 0, keypoints_written = 0;

    // Write pending keyframes
    if (keyframes_dirty_.load()) {
        std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);
        for (auto& [keyframe_id, keyframe] : keyframe_pending_writes_) {
            proto::KeyFrame kf_proto = keyframe->toProto();
            proto::FileLocation location;
            if (writeProtoMessage(data_file, kf_proto, location)) {
                keyframe_locations_[keyframe_id] = location;

                // Update spatial index with actual disk location
                for (auto& entry : keyframe_spatial_index_entries_) {
                    if (entry.keyframe_id() == keyframe_id) {
                        *entry.mutable_location() = location;
                        break;
                    }
                }
                keyframes_written++;
            } else {
                LOG(ERROR) << "Failed to write keyframe " << keyframe_id << " to disk";
            }
        }
        keyframe_pending_writes_.clear();
    }

    // Write pending factors
    if (factors_dirty_.load()) {
        std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);
        for (auto& [factor_id, factor] : factor_pending_writes_) {
            proto::Factor factor_proto = factor.toProto();
            proto::FileLocation location;
            if (writeProtoMessage(data_file, factor_proto, location)) {
                factor_locations_[factor_id] = location;
                factors_written++;
            } else {
                LOG(ERROR) << "Failed to write factor " << factor_id << " to disk";
            }
        }
        factor_pending_writes_.clear();
    }

    // Write pending keypoints
    if (keypoints_dirty_.load()) {
        std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);
        for (auto& [keypoint_id, keypoint] : keypoint_pending_writes_) {
            proto::Keypoint kp_proto = keypoint.toProto();
            proto::FileLocation location;
            if (writeProtoMessage(data_file, kp_proto, location)) {
                keypoint_locations_[keypoint_id] = location;
                keypoints_written++;
            } else {
                LOG(ERROR) << "Failed to write keypoint " << keypoint_id << " to disk";
            }
        }
        keypoint_pending_writes_.clear();
    }

    data_file.close();

    LOG(INFO) << "Wrote pending data to disk: " << keyframes_written << " keyframes, "
              << factors_written << " factors, " << keypoints_written << " keypoints";

    return true;
}

bool MapStore::performAtomicSync() {
    // Use atomic sync to prevent corruption during concurrent operations
    std::lock_guard<std::mutex> lock(file_operations_mutex_);

    auto start_time = std::chrono::steady_clock::now();

    // Check if there's actually anything to sync (including processed optimized queue)
    bool has_processed_optimized = false;
    {
        std::shared_lock<std::shared_mutex> popt_lock(processed_optimized_queue_mutex_);
        has_processed_optimized = !processed_optimized_queue_->empty();
    }

    if (!keyframes_dirty_.load() && !factors_dirty_.load() &&
        !keypoints_dirty_.load() && !metadata_dirty_.load() && !has_processed_optimized) {
        LOG(INFO) << "No dirty data or processed optimized keyframes found, skipping sync";
        return true;
    }

    LOG(INFO) << "Performing atomic sync - dirty flags: keyframes="
              << (keyframes_dirty_.load() ? "YES" : "NO")
              << ", factors=" << (factors_dirty_.load() ? "YES" : "NO")
              << ", keypoints=" << (keypoints_dirty_.load() ? "YES" : "NO")
              << ", metadata=" << (metadata_dirty_.load() ? "YES" : "NO")
              << ", processed_optimized=" << (has_processed_optimized ? "YES" : "NO");

    bool success = true;

    // First, sync the processed optimized queue (highest priority)
    if (has_processed_optimized) {
        try {
            syncProcessedOptimizedToDisk();
            LOG(INFO) << "Successfully synced processed optimized queue";
        } catch (const std::exception& e) {
            LOG(ERROR) << "Failed to sync processed optimized queue: " << e.what();
            success = false;
        }
    }

    // Then perform the regular save operation for pending writes
    if (success && (keyframes_dirty_.load() || factors_dirty_.load() ||
                   keypoints_dirty_.load() || metadata_dirty_.load())) {
        success = saveChanges();
    }

    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    if (success) {
        LOG(INFO) << "Atomic sync completed in " << duration.count() << "ms";
    } else {
        LOG(ERROR) << "Atomic sync failed after " << duration.count() << "ms";
    }

    return success;
}

void MapStore::cacheKeyFrame(uint64_t id, const KeyFramePtr& keyframe) const {
    std::unique_lock<std::shared_mutex> lock(cache_mutex_);
    cacheKeyFrameInternal(id, keyframe);
}

void MapStore::cacheKeyFrameInternal(uint64_t id, const KeyFramePtr& keyframe) const {
    // Assumes cache_mutex_ is already locked

    // Check if already in cache
    if (keyframe_cache_.find(id) != keyframe_cache_.end()) {
        // Already cached, just update LRU position
        auto lru_it = keyframe_lru_map_.find(id);
        if (lru_it != keyframe_lru_map_.end()) {
            keyframe_lru_list_.erase(lru_it->second);
            keyframe_lru_list_.push_front(id);
            lru_it->second = keyframe_lru_list_.begin();
        }
        // Update the cached keyframe with new data
        keyframe_cache_[id] = keyframe;
        return;
    }

    // Evict if cache is full
    while (keyframe_cache_.size() >= max_cache_size_) {
        evictLRUKeyFrame();
    }

    // Add to cache and LRU tracking
    keyframe_cache_[id] = keyframe;
    keyframe_lru_list_.push_front(id);
    keyframe_lru_map_[id] = keyframe_lru_list_.begin();

    LOG(INFO) << "Cached keyframe " << id << " (cache size: " << keyframe_cache_.size() << "/" << max_cache_size_ << ")";
}

void MapStore::cacheFactor(uint64_t id, const types::Factor& factor) const {
    std::unique_lock<std::shared_mutex> lock(cache_mutex_);
    factor_cache_[id] = factor;
    // Factors don't have LRU eviction for now - they're typically smaller
}

void MapStore::cacheKeyPoint(uint32_t id, const types::Keypoint& keypoint) const {
    std::unique_lock<std::shared_mutex> lock(cache_mutex_);
    keypoint_cache_[id] = keypoint;
    // Keypoints don't have LRU eviction for now - they're typically smaller
}

void MapStore::evictLRUKeyFrame() const {
    // Assumes cache_mutex_ is already locked for writing
    if (keyframe_lru_list_.empty()) {
        return;
    }

    uint64_t lru_id = keyframe_lru_list_.back();
    keyframe_lru_list_.pop_back();
    keyframe_lru_map_.erase(lru_id);
    keyframe_cache_.erase(lru_id);

    LOG(INFO) << "Evicted keyframe " << lru_id << " from cache (LRU)";
}

std::vector<uint64_t> MapStore::getFactorIdsForKeyFrame(uint64_t keyframe_id) const {
    auto it = keyframe_to_factor_ids_.find(keyframe_id);
    if (it != keyframe_to_factor_ids_.end()) {
        return it->second;
    }
    return {};  // Return empty vector if no factors found
}

std::vector<types::Factor> MapStore::getFactorsForKeyFrame(uint64_t keyframe_id) const {
    std::vector<types::Factor> factors;
    auto factor_ids = getFactorIdsForKeyFrame(keyframe_id);

    for (uint64_t factor_id : factor_ids) {
        auto factor_opt = getFactor(factor_id);
        if (factor_opt) {
            factors.push_back(factor_opt.value());
        } else {
            LOG(WARNING) << "Failed to load factor " << factor_id << " for keyframe " << keyframe_id;
        }
    }

    LOG(INFO) << "Retrieved " << factors.size() << " factors for keyframe " << keyframe_id
              << " (from " << factor_ids.size() << " associated factor IDs)";

    return factors;
}

bool MapStore::updateOptimizedPoses(const std::map<uint64_t, types::Pose>& optimized_poses) {
    auto start_time = std::chrono::high_resolution_clock::now();

    LOG(INFO) << "Updating " << optimized_poses.size() << " optimized poses in MapStore";

    size_t poses_updated = 0;
    size_t poses_not_found = 0;
    double total_position_change = 0.0;
    double max_position_change = 0.0;

    {
        std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);

        for (const auto& [keyframe_id, optimized_pose] : optimized_poses) {
            bool found = false;
            double position_change = 0.0;

            // Update in-memory cache if present
            auto cache_it = keyframe_cache_.find(keyframe_id);
            if (cache_it != keyframe_cache_.end()) {
                position_change = (cache_it->second->pose.position - optimized_pose.position).norm();
                cache_it->second->pose = optimized_pose;
                found = true;
            }

            // Check pending writes
            auto pending_it = keyframe_pending_writes_.find(keyframe_id);
            if (pending_it != keyframe_pending_writes_.end()) {
                if (!found) {
                    position_change = (pending_it->second->pose.position - optimized_pose.position).norm();
                }
                pending_it->second->pose = optimized_pose;
                found = true;
            }

            // If not in cache or pending, need to load, update, and mark for write
            if (!found && keyframe_locations_.count(keyframe_id)) {
                // Load from disk, update pose, and add to pending writes
                auto keyframe = getKeyFrameFromDisk(keyframe_id);
                if (keyframe) {
                    position_change = (keyframe->pose.position - optimized_pose.position).norm();
                    keyframe->pose = optimized_pose;
                    keyframe_pending_writes_[keyframe_id] = keyframe;
                    cacheKeyFrameInternal(keyframe_id, keyframe);
                    found = true;
                }
            }

            if (found) {
                poses_updated++;
                total_position_change += position_change;
                max_position_change = std::max(max_position_change, position_change);

                if (position_change > 0.01) {
                    LOG(INFO) << "Updated pose for keyframe " << keyframe_id
                              << ": Δpos=" << std::fixed << std::setprecision(3) << position_change << "m";
                }
            } else {
                LOG(WARNING) << "Keyframe " << keyframe_id << " not found for pose update";
                poses_not_found++;
            }
        }

        // Mark keyframes as dirty for sync
        if (poses_updated > 0) {
            keyframes_dirty_ = true;
            metadata_dirty_ = true;  // Bounds may have changed
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    double avg_position_change = (poses_updated > 0) ? (total_position_change / poses_updated) : 0.0;

    LOG(INFO) << "Optimized pose update summary:";
    LOG(INFO) << "    Total poses: " << optimized_poses.size();
    LOG(INFO) << "    Updated: " << poses_updated;
    LOG(INFO) << "    Not found: " << poses_not_found;
    LOG(INFO) << "    Update time: " << duration.count() << "ms";

    if (poses_updated > 0) {
        LOG(INFO) << "    Position changes: max=" << std::fixed << std::setprecision(3)
                  << max_position_change << "m, avg=" << avg_position_change << "m";
    }

    // Trigger background sync if enabled
    if (sync_thread_running_.load()) {
        requestSync();
    }

    return poses_not_found == 0;
}

bool MapStore::updateOptimizedLandmarks(const std::map<uint32_t, Eigen::Vector3d>& optimized_landmarks) {
    auto start_time = std::chrono::high_resolution_clock::now();

    LOG(INFO) << "Updating " << optimized_landmarks.size() << " optimized landmarks in MapStore";

    size_t landmarks_updated = 0;
    size_t landmarks_not_found = 0;
    double total_position_change = 0.0;
    double max_position_change = 0.0;

    {
        std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);

        for (const auto& [landmark_id, optimized_position] : optimized_landmarks) {
            bool found = false;
            double position_change = 0.0;

            // Update in-memory cache if present
            auto cache_it = keypoint_cache_.find(landmark_id);
            if (cache_it != keypoint_cache_.end()) {
                position_change = (cache_it->second.position - optimized_position).norm();
                cache_it->second.position = optimized_position;
                found = true;
            }

            // Check pending writes
            auto pending_it = keypoint_pending_writes_.find(landmark_id);
            if (pending_it != keypoint_pending_writes_.end()) {
                if (!found) {
                    position_change = (pending_it->second.position - optimized_position).norm();
                }
                pending_it->second.position = optimized_position;
                found = true;
            }

            // If not in cache or pending, need to load, update, and mark for write
            if (!found && keypoint_locations_.count(landmark_id)) {
                auto keypoint_opt = getKeyPointFromDisk(landmark_id);
                if (keypoint_opt) {
                    auto keypoint = keypoint_opt.value();
                    position_change = (keypoint.position - optimized_position).norm();
                    keypoint.position = optimized_position;
                    keypoint_pending_writes_[landmark_id] = keypoint;
                    keypoint_cache_[landmark_id] = keypoint; // Inline caching to avoid recursive lock
                    found = true;
                }
            }

            // Mark as dirty for new write system
            if (found) {
                markMapPointDirty(landmark_id);
                landmarks_updated++;
                total_position_change += position_change;
                max_position_change = std::max(max_position_change, position_change);

                if (position_change > 0.05) {
                    LOG(INFO) << "Updated position for landmark " << landmark_id
                              << ": Δpos=" << std::fixed << std::setprecision(3) << position_change << "m";
                }
            } else {
                LOG(WARNING) << "Landmark " << landmark_id << " not found for position update";
                landmarks_not_found++;
            }
        }

        // Mark keypoints as dirty for sync
        if (landmarks_updated > 0) {
            keypoints_dirty_ = true;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    double avg_position_change = (landmarks_updated > 0) ? (total_position_change / landmarks_updated) : 0.0;

    LOG(INFO) << "Optimized landmark update summary:";
    LOG(INFO) << "    Total landmarks: " << optimized_landmarks.size();
    LOG(INFO) << "    Updated: " << landmarks_updated;
    LOG(INFO) << "    Not found: " << landmarks_not_found;
    LOG(INFO) << "    Update time: " << duration.count() << "ms";

    if (landmarks_updated > 0) {
        LOG(INFO) << "    Position changes: max=" << std::fixed << std::setprecision(3)
                  << max_position_change << "m, avg=" << avg_position_change << "m";
    }

    // Trigger background sync if enabled
    if (sync_thread_running_.load()) {
        requestSync();
    }

    return landmarks_not_found == 0;
}

bool MapStore::triggerImmediateSync() {
    auto start_time = std::chrono::high_resolution_clock::now();

    LOG(INFO) << "Triggering immediate sync to disk after optimization";

    bool success = saveChanges();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    if (success) {
        LOG(INFO) << "Immediate sync completed successfully in " << duration.count() << "ms";
    } else {
        LOG(ERROR) << "Immediate sync failed after " << duration.count() << "ms";
    }

    return success;
}

void MapStore::enableBackgroundSync(std::chrono::seconds sync_interval) {
    if (sync_thread_running_.load()) {
        LOG(INFO) << "Background sync thread already running, updating interval";
        setSyncInterval(sync_interval);
        return;
    }

    sync_interval_ = sync_interval;
    should_stop_sync_ = false;
    sync_thread_running_ = true;
    sync_requested_ = false;

    sync_thread_ = std::make_unique<std::thread>(&MapStore::syncThreadLoop, this);

    LOG(INFO) << "Background sync thread enabled with " << sync_interval.count() << "s interval";
}

void MapStore::disableBackgroundSync() {
    if (!sync_thread_running_.load()) {
        return;
    }

    should_stop_sync_ = true;
    sync_thread_running_ = false;

    // Signal the thread to wake up and exit
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        sync_condition_.notify_all();
    }

    if (sync_thread_ && sync_thread_->joinable()) {
        sync_thread_->join();
    }

    sync_thread_.reset();

    LOG(INFO) << "Background sync thread disabled";
}

void MapStore::setSyncInterval(std::chrono::seconds interval) {
    sync_interval_ = interval;
    LOG(INFO) << "Background sync interval updated to " << interval.count() << "s";
}

void MapStore::syncThreadLoop() {
    LOG(INFO) << "Background sync thread started";

    size_t sync_attempts = 0;
    size_t successful_syncs = 0;
    size_t failed_syncs = 0;
    auto thread_start = std::chrono::steady_clock::now();

    while (sync_thread_running_.load() && !should_stop_sync_.load()) {
        std::unique_lock<std::mutex> lock(sync_mutex_);

        // Wait for sync interval or explicit sync request
        bool sync_requested = sync_condition_.wait_for(lock, sync_interval_, [this]() {
            return sync_requested_.load() || should_stop_sync_.load();
        });

        if (should_stop_sync_.load()) {
            break;
        }

        lock.unlock();

        // Perform sync (either periodic or requested)
        sync_attempts++;
        auto sync_start = std::chrono::steady_clock::now();

        try {
            if (performAtomicSync()) {
                successful_syncs++;

                auto sync_end = std::chrono::steady_clock::now();
                auto sync_duration = std::chrono::duration_cast<std::chrono::milliseconds>(sync_end - sync_start);

                std::string sync_type = sync_requested ? "requested" : "periodic";
                LOG(INFO) << "Background sync #" << sync_attempts << " (" << sync_type
                          << ") completed successfully in " << sync_duration.count() << "ms";

                // Log periodic statistics
                if (successful_syncs % 10 == 0) {
                    auto current_time = std::chrono::steady_clock::now();
                    auto total_runtime = std::chrono::duration_cast<std::chrono::minutes>(current_time - thread_start);
                    double success_rate = (sync_attempts > 0) ? (100.0 * successful_syncs / sync_attempts) : 0.0;

                    LOG(INFO) << "Background sync stats (runtime: " << total_runtime.count() << " min):";
                    LOG(INFO) << "    Total syncs: " << sync_attempts;
                    LOG(INFO) << "    Successful: " << successful_syncs << " (" << std::fixed
                              << std::setprecision(1) << success_rate << "%)";
                    LOG(INFO) << "    Failed: " << failed_syncs;
                }
            } else {
                failed_syncs++;
                LOG(WARNING) << "Background sync #" << sync_attempts << " failed";
            }

        } catch (const std::exception& e) {
            failed_syncs++;
            LOG(ERROR) << "Background sync #" << sync_attempts << " exception: " << e.what();
        } catch (...) {
            failed_syncs++;
            LOG(ERROR) << "Background sync #" << sync_attempts << " unknown exception";
        }

        // Reset the sync request flag
        sync_requested_ = false;
    }

    auto thread_end = std::chrono::steady_clock::now();
    auto total_runtime = std::chrono::duration_cast<std::chrono::minutes>(thread_end - thread_start);

    LOG(INFO) << "Background sync thread ended after " << total_runtime.count() << " minutes";
    LOG(INFO) << "Final sync stats: " << successful_syncs << "/" << sync_attempts << " successful syncs";
}

void MapStore::requestSync() {
    if (!sync_thread_running_.load()) {
        LOG(WARNING) << "Background sync not enabled, cannot request sync";
        return;
    }

    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        sync_requested_ = true;
    }
    sync_condition_.notify_one();

    LOG(INFO) << "Background sync requested";
}

std::map<uint64_t, types::Pose> MapStore::getAllKeyFramePoses() const {
    std::map<uint64_t, types::Pose> poses;

    LOG(INFO) << "Extracting poses from " << keyframe_locations_.size() << " keyframes efficiently";

    for (const auto& [keyframe_id, location] : keyframe_locations_) {
        // Load only the keyframe data we need, extract pose, then discard
        auto opt_kf_proto = readProtoMessage<proto::KeyFrame, proto::KeyFrame>(
            location, [](const proto::KeyFrame& p) { return p; });

        if (opt_kf_proto) {
            // Extract only the pose data without creating full KeyFrame object
            const auto& kf_proto = opt_kf_proto.value();

            types::Pose pose;
            pose.position.x() = kf_proto.pose().position().x();
            pose.position.y() = kf_proto.pose().position().y();
            pose.position.z() = kf_proto.pose().position().z();

            pose.orientation.x() = kf_proto.pose().orientation().x();
            pose.orientation.y() = kf_proto.pose().orientation().y();
            pose.orientation.z() = kf_proto.pose().orientation().z();
            pose.orientation.w() = kf_proto.pose().orientation().w();

            pose.timestamp = kf_proto.pose().timestamp();
            pose.frame_id = kf_proto.pose().frame_id();

            poses[keyframe_id] = pose;
        } else {
            LOG(WARNING) << "Failed to load pose for keyframe " << keyframe_id;
        }
    }

    LOG(INFO) << "Extracted " << poses.size() << " poses efficiently (no full keyframes loaded)";
    return poses;
}

KeyFramePtr MapStore::getKeyFrameFromDisk(uint64_t id) const {
    // Direct disk access without cache updates - used for optimization updates
    auto it = keyframe_locations_.find(id);
    if (it == keyframe_locations_.end()) {
        return nullptr;
    }
    auto opt_kf_proto = readProtoMessage<proto::KeyFrame, proto::KeyFrame>(
        it->second, [](const proto::KeyFrame& p) { return p; });
    if (opt_kf_proto) {
        return types::KeyFrame::fromProto(opt_kf_proto.value());
    }
    return nullptr;
}

std::optional<types::Keypoint> MapStore::getKeyPointFromDisk(uint32_t id) const {
    // Direct disk access without cache updates - used for optimization updates
    auto it = keypoint_locations_.find(id);
    if (it == keypoint_locations_.end()) {
        return std::nullopt;
    }
    return readProtoMessage<proto::Keypoint, types::Keypoint>(it->second, types::Keypoint::fromProto);
}

bool MapStore::loadMap() {
    if (metadata_filepath_.empty() || index_filepath_.empty() || data_filepath_.empty()) {
        LOG(ERROR) << "Filepaths not initialized. Cannot load." << std::endl;
        return false;
    }
    clearDataAndIndices();

    // Load Metadata
    std::fstream meta_stream(metadata_filepath_, std::ios::in | std::ios::binary);
    if (!meta_stream.is_open() || !metadata_.ParseFromIstream(&meta_stream)) {
        LOG(ERROR) << "Failed to load metadata from " << metadata_filepath_ << std::endl;
        if (meta_stream.is_open())
            meta_stream.close();
    } else {
        meta_stream.close();
        LOG(INFO) << "Metadata loaded from " << metadata_filepath_ << std::endl;
    }

    proto::MapDiskIndex disk_index;
    std::fstream index_stream(index_filepath_, std::ios::in | std::ios::binary);
    if (!index_stream.is_open() || !disk_index.ParseFromIstream(&index_stream)) {
        LOG(ERROR) << "Failed to load index from " << index_filepath_
                   << ". Map data will be inaccessible." << std::endl;
        if (index_stream.is_open())
            index_stream.close();
        return false;
    }
    index_stream.close();

    for (const auto& entry : disk_index.keyframe_entries()) {
        keyframe_locations_[entry.keyframe_id()] = entry.location();
        keyframe_spatial_index_entries_.push_back(entry);  // Populate enriched index
    }
    for (const auto& entry : disk_index.factor_entries()) {
        factor_locations_[entry.factor_id()] = entry.location();
    }
    for (const auto& entry : disk_index.keypoint_entries()) {
        keypoint_locations_[entry.keypoint_id()] = entry.location();
    }

    rebuildTransientIndices();
    LOG(INFO) << "Index loaded from " << index_filepath_
              << ". KF entries: " << keyframe_locations_.size() << std::endl;
    return true;
}

void MapStore::rebuildTransientIndices() {
    timestamp_to_keyframe_ids_.clear();
    keyframe_to_factor_ids_.clear();  // Clear factor-keyframe associations

    for (const auto& kf_idx_entry : keyframe_spatial_index_entries_) {
        timestamp_to_keyframe_ids_[kf_idx_entry.timestamp()].push_back(kf_idx_entry.keyframe_id());
    }
    for (auto& pair_val : timestamp_to_keyframe_ids_) {
        std::sort(pair_val.second.begin(), pair_val.second.end());
        pair_val.second.erase(std::unique(pair_val.second.begin(), pair_val.second.end()),
                              pair_val.second.end());
    }

    // Rebuild factor-keyframe associations by loading all factors
    LOG(INFO) << "Rebuilding factor-keyframe associations from " << factor_locations_.size() << " factors";
    for (const auto& [factor_id, location] : factor_locations_) {
        auto factor_opt = readProtoMessage<proto::Factor, types::Factor>(location, types::Factor::fromProto);
        if (factor_opt) {
            const auto& factor = factor_opt.value();
            for (uint64_t keyframe_id : factor.connected_nodes) {
                keyframe_to_factor_ids_[keyframe_id].push_back(factor_id);
            }
        } else {
            LOG(WARNING) << "Failed to read factor " << factor_id << " during index rebuild";
        }
    }

    // Log association statistics
    size_t total_associations = 0;
    for (const auto& [keyframe_id, factor_ids] : keyframe_to_factor_ids_) {
        total_associations += factor_ids.size();
    }
    LOG(INFO) << "Rebuilt factor-keyframe associations: " << keyframe_to_factor_ids_.size()
              << " keyframes with " << total_associations << " total factor associations";
}

void MapStore::clearDataAndIndices() {
    keyframe_locations_.clear();
    factor_locations_.clear();
    keypoint_locations_.clear();
    timestamp_to_keyframe_ids_.clear();
    keyframe_spatial_index_entries_.clear();
    keyframe_to_factor_ids_.clear();  // Clear factor-keyframe associations

    // Clear pending writes and caches
    {
        std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);
        keyframe_pending_writes_.clear();
        factor_pending_writes_.clear();
        keypoint_pending_writes_.clear();
        keyframe_cache_.clear();
        factor_cache_.clear();
        keypoint_cache_.clear();
        keyframe_lru_list_.clear();
        keyframe_lru_map_.clear();
    }

    // Reset dirty flags
    keyframes_dirty_ = false;
    factors_dirty_ = false;
    keypoints_dirty_ = false;
    metadata_dirty_ = false;

    metadata_.Clear();
    metadata_.set_version("1.0-disk");
    metadata_.set_created_by("MapStoreDisk");
    auto* bounds = metadata_.mutable_bounds();
    bounds->set_min_x(std::numeric_limits<double>::max());
    bounds->set_min_y(std::numeric_limits<double>::max());
    bounds->set_min_z(std::numeric_limits<double>::max());
    bounds->set_max_x(std::numeric_limits<double>::lowest());
    bounds->set_max_y(std::numeric_limits<double>::lowest());
    bounds->set_max_z(std::numeric_limits<double>::lowest());

    LOG(INFO) << "In-memory indices, caches, and pending writes cleared." << std::endl;
}

// ===== NEW CALLBACK-BASED WRITE SYSTEM =====

void MapStore::initializeAtomicQueues() {
    // Initialize queue pointers with empty maps
    processed_non_optimized_queue_ = std::make_unique<std::unordered_map<uint64_t, KeyFramePtr>>();
    processed_optimized_queue_ = std::make_unique<std::unordered_map<uint64_t, KeyFramePtr>>();

    LOG(INFO) << "Queue system initialized";
}

// ===== NEW SINGLE-THREADED WRITE WORKER =====

void MapStore::startWriteWorker() {
    if (write_worker_running_.load()) {
        LOG(WARNING) << "Write worker already running";
        return;
    }
    
    should_stop_write_worker_ = false;
    write_worker_running_ = true;
    
    write_worker_thread_ = std::make_unique<std::thread>(&MapStore::writeWorkerLoop, this);
    LOG(INFO) << "Write worker thread started";
}

void MapStore::stopWriteWorker() {
    if (!write_worker_running_.load()) {
        return;
    }
    
    should_stop_write_worker_ = true;
    write_worker_running_ = false;
    
    // Signal worker to wake up and exit
    {
        std::lock_guard<std::mutex> lock(write_queue_mutex_);
        write_available_.notify_all();
    }
    
    if (write_worker_thread_ && write_worker_thread_->joinable()) {
        write_worker_thread_->join();
    }
    
    write_worker_thread_.reset();
    LOG(INFO) << "Write worker thread stopped";
}

void MapStore::enableWriteWorker() {
    startWriteWorker();
}

void MapStore::disableWriteWorker() {
    stopWriteWorker();
}

void MapStore::writeWorkerLoop() {
    LOG(INFO) << "Write worker loop started";
    
    size_t write_attempts = 0;
    size_t successful_writes = 0;
    size_t failed_writes = 0;
    auto thread_start = std::chrono::steady_clock::now();
    
    while (write_worker_running_.load() && !should_stop_write_worker_.load()) {
        std::unique_lock<std::mutex> lock(write_queue_mutex_);
        
        // Wait for write requests
        write_available_.wait(lock, [this]() {
            return !pending_write_queue_.empty() || should_stop_write_worker_.load();
        });
        
        if (should_stop_write_worker_.load()) {
            break;
        }
        
        if (!pending_write_queue_.empty()) {
            auto batch = std::move(pending_write_queue_.front());
            pending_write_queue_.pop();
            lock.unlock();
            
            write_attempts++;
            write_in_progress_ = true;
            
            LOG(INFO) << "Write worker processing batch #" << write_attempts 
                      << " with " << batch->size() << " keyframes";
            
            auto write_start = std::chrono::high_resolution_clock::now();
            
            try {
                writeBatchToDiskComplete(std::move(batch));
                successful_writes++;
                
                auto write_end = std::chrono::high_resolution_clock::now();
                auto write_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                    write_end - write_start);
                
                LOG(INFO) << "Write worker batch #" << write_attempts 
                          << " completed successfully in " << write_duration.count() << "ms";
                
            } catch (const std::exception& e) {
                failed_writes++;
                write_in_progress_ = false;
                
                // Signal any waiting threads
                {
                    std::lock_guard<std::mutex> completion_lock(write_completion_mutex_);
                    write_completed_.notify_all();
                }
                
                // Terminate as requested - no graceful handling
                LOG(ERROR) << "Write worker batch #" << write_attempts << " failed: " << e.what();
                throw std::runtime_error("Disk write operation failed: " + std::string(e.what()));
            }
            
            write_in_progress_ = false;
            
            // Signal completion to any waiting threads
            {
                std::lock_guard<std::mutex> completion_lock(write_completion_mutex_);
                write_completed_.notify_all();
            }
            
            // Log periodic statistics
            if (write_attempts % 5 == 0) {
                auto current_time = std::chrono::steady_clock::now();
                auto total_runtime = std::chrono::duration_cast<std::chrono::minutes>(
                    current_time - thread_start);
                double success_rate = (write_attempts > 0) ? 
                    (100.0 * successful_writes / write_attempts) : 0.0;
                
                LOG(INFO) << "Write worker stats (runtime: " << total_runtime.count() << " min):";
                LOG(INFO) << "    Total writes: " << write_attempts;
                LOG(INFO) << "    Successful: " << successful_writes << " (" << std::fixed 
                          << std::setprecision(1) << success_rate << "%)";
                LOG(INFO) << "    Failed: " << failed_writes;
            }
        }
    }
    
    auto thread_end = std::chrono::steady_clock::now();
    auto total_runtime = std::chrono::duration_cast<std::chrono::minutes>(thread_end - thread_start);
    
    LOG(INFO) << "Write worker loop ended after " << total_runtime.count() << " minutes";
    LOG(INFO) << "Final write stats: " << successful_writes << "/" << write_attempts 
              << " successful writes";
}

void MapStore::queueWriteRequest(std::unique_ptr<std::unordered_map<uint64_t, KeyFramePtr>> batch) {
    std::lock_guard<std::mutex> lock(write_queue_mutex_);
    
    // Check queue size limit
    if (pending_write_queue_.size() >= MAX_WRITE_QUEUE_SIZE) {
        LOG(WARNING) << "Write queue is full (" << pending_write_queue_.size() 
                     << "/" << MAX_WRITE_QUEUE_SIZE << "), dropping oldest request";
        pending_write_queue_.pop();  // Drop oldest request
    }
    
    pending_write_queue_.push(std::move(batch));
    write_available_.notify_one();
    
    LOG(INFO) << "Queued write request (queue size: " << pending_write_queue_.size() 
              << "/" << MAX_WRITE_QUEUE_SIZE << ")";
}

void MapStore::waitForWriteCompletion() {
    if (!write_in_progress_.load()) {
        return;  // No write in progress
    }
    
    LOG(INFO) << "Waiting for write operation to complete (write slower than frame collection)";
    
    std::unique_lock<std::mutex> lock(write_completion_mutex_);
    write_completed_.wait(lock, [this]() {
        return !write_in_progress_.load();
    });
    
    LOG(INFO) << "Write operation completed, continuing processing";
}

void MapStore::swapAndWriteToDisk() {
    LOG(INFO) << "Starting callback-based queue swap and write to disk";

    // Atomically swap the PROCESSED OPTIMIZED queue with an empty one
    std::unique_ptr<std::unordered_map<uint64_t, KeyFramePtr>> queue_to_write;
    {
        std::unique_lock<std::shared_mutex> lock(processed_optimized_queue_mutex_);
        queue_to_write = std::move(processed_optimized_queue_);
        processed_optimized_queue_ = std::make_unique<std::unordered_map<uint64_t, KeyFramePtr>>();
    }

    if (queue_to_write->empty()) {
        LOG(INFO) << "No keyframes in processed optimized queue to write";
        return;
    }

    LOG(INFO) << "Swapped " << queue_to_write->size() 
              << " keyframes from processed optimized queue for background write";

    // Queue the write request for the background worker
    queueWriteRequest(std::move(queue_to_write));
    
    // If write queue is getting backed up, wait for completion
    {
        std::lock_guard<std::mutex> lock(write_queue_mutex_);
        if (pending_write_queue_.size() >= MAX_WRITE_QUEUE_SIZE - 1) {
            LOG(INFO) << "Write queue is nearly full, waiting for write completion";
        }
    }
    
    // Wait if we're at the limit to prevent overwhelming the write worker
    if (write_in_progress_.load()) {
        waitForWriteCompletion();
    }
}

void MapStore::writeBatchToDiskComplete(std::unique_ptr<std::unordered_map<uint64_t, KeyFramePtr>> keyframes_to_write) {
    auto start_time = std::chrono::high_resolution_clock::now();

    LOG(INFO) << "Writing complete batch of " << keyframes_to_write->size() 
              << " keyframes to disk (data + index + metadata)";

    // Step 1: Add all keyframes to pending writes
    {
        std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);

        for (const auto& [id, keyframe] : *keyframes_to_write) {
            keyframe_pending_writes_[id] = keyframe;

            // Also update the cache
            cacheKeyFrameInternal(id, keyframe);

            // Update bounds
            updateMetadataBounds(keyframe->pose);
        }

        keyframes_dirty_ = true;
        metadata_dirty_ = true;
    }

    // Step 2: Write all dirty map points (only changed ones) - as requested
    size_t dirty_map_points_count = 0;
    {
        std::shared_lock<std::shared_mutex> dirty_lock(dirty_map_points_mutex_);
        std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);

        for (uint32_t dirty_id : dirty_map_points_) {
            auto cache_it = keypoint_cache_.find(dirty_id);
            if (cache_it != keypoint_cache_.end()) {
                keypoint_pending_writes_[dirty_id] = cache_it->second;
                dirty_map_points_count++;
            }
        }

        if (!dirty_map_points_.empty()) {
            keypoints_dirty_ = true;
            LOG(INFO) << "Added " << dirty_map_points_count << " dirty map points to write batch";
        }
    }

    // Step 3: Write data file (keyframes, factors, keypoints)
    std::lock_guard<std::mutex> file_lock(file_operations_mutex_);
    
    bool data_write_success = writePendingDataToDisk();
    if (!data_write_success) {
        throw std::runtime_error("Failed to write pending data to disk");
    }

    // Step 4: Write index file (.idx) - FIXED: This was missing in original implementation
    if (index_filepath_.empty()) {
        throw std::runtime_error("Index filepath not initialized");
    }

    proto::MapDiskIndex disk_index;
    for (const auto& pair : keyframe_locations_) {
        bool found_in_spatial = false;
        for (const auto& spatial_entry : keyframe_spatial_index_entries_) {
            if (spatial_entry.keyframe_id() == pair.first) {
                *disk_index.add_keyframe_entries() = spatial_entry;
                found_in_spatial = true;
                break;
            }
        }
        if (!found_in_spatial) {
            proto::KeyFrameIndexEntry* entry = disk_index.add_keyframe_entries();
            entry->set_keyframe_id(pair.first);
            *entry->mutable_location() = pair.second;
        }
    }
    for (const auto& pair : factor_locations_) {
        proto::FactorIndexEntry* entry = disk_index.add_factor_entries();
        entry->set_factor_id(pair.first);
        *entry->mutable_location() = pair.second;
    }
    for (const auto& pair : keypoint_locations_) {
        proto::KeyPointIndexEntry* entry = disk_index.add_keypoint_entries();
        entry->set_keypoint_id(pair.first);
        *entry->mutable_location() = pair.second;
    }

    std::fstream index_stream(index_filepath_, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!index_stream.is_open() || !disk_index.SerializeToOstream(&index_stream)) {
        throw std::runtime_error("Failed to write index file: " + index_filepath_);
    }
    index_stream.close();

    // Step 5: Write metadata file (.meta) - as requested for every batch
    metadata_.set_creation_timestamp_seconds(
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    std::fstream meta_stream(metadata_filepath_, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!meta_stream.is_open() || !metadata_.SerializeToOstream(&meta_stream)) {
        throw std::runtime_error("Failed to write metadata file: " + metadata_filepath_);
    }
    meta_stream.close();

    // Step 6: Clear dirty flags and dirty map points after successful write
    keyframes_dirty_ = false;
    factors_dirty_ = false;
    keypoints_dirty_ = false;
    metadata_dirty_ = false;

    {
        std::unique_lock<std::shared_mutex> dirty_lock(dirty_map_points_mutex_);
        dirty_map_points_.clear();
    }

    // Clear the batch queue
    keyframes_to_write->clear();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    LOG(INFO) << "Complete batch write finished in " << duration.count() << "ms"
              << " (keyframes + " << dirty_map_points_count << " map points + index + metadata)";
}

// Map point dirty tracking
void MapStore::markMapPointDirty(uint32_t id) {
    std::unique_lock<std::shared_mutex> lock(dirty_map_points_mutex_);
    dirty_map_points_.insert(id);
}

bool MapStore::isMapPointDirty(uint32_t id) const {
    std::shared_lock<std::shared_mutex> lock(dirty_map_points_mutex_);
    return dirty_map_points_.count(id) > 0;
}

// ===== THREE-QUEUE SYSTEM IMPLEMENTATION (Updated for atomic pointers) =====

void MapStore::addToUnprocessedCache(const KeyFramePtr& keyframe) {
    std::unique_lock<std::shared_mutex> lock(unprocessed_cache_mutex_);
    unprocessed_cache_[keyframe->id] = keyframe;

    LOG(INFO) << "Added keyframe " << keyframe->id << " to unprocessed cache (size: "
              << unprocessed_cache_.size() << ")";
}

void MapStore::moveToProcessedNonOptimized(uint64_t keyframe_id) {
    // Move from unprocessed cache to processed non-optimized queue
    std::unique_lock<std::shared_mutex> unprocessed_lock(unprocessed_cache_mutex_);

    auto it = unprocessed_cache_.find(keyframe_id);
    if (it == unprocessed_cache_.end()) {
        LOG(WARNING) << "Keyframe " << keyframe_id << " not found in unprocessed cache";
        return;
    }

    auto keyframe = it->second;
    unprocessed_cache_.erase(it);
    unprocessed_lock.unlock();

    // Add to processed non-optimized queue (mutex-protected access)
    {
        std::unique_lock<std::shared_mutex> queue_lock(processed_non_optimized_queue_mutex_);
        (*processed_non_optimized_queue_)[keyframe_id] = keyframe;
    }

    LOG(INFO) << "Moved keyframe " << keyframe_id << " to processed non-optimized queue";

    // Update map point last seen tracking
    updateMapPointLastSeen(keyframe_id);
}

void MapStore::moveBatchToProcessedOptimized(const std::vector<uint64_t>& keyframe_ids) {
    // Move batch from processed non-optimized to processed optimized queue
    std::unique_lock<std::shared_mutex> non_opt_lock(processed_non_optimized_queue_mutex_);
    std::unique_lock<std::shared_mutex> opt_lock(processed_optimized_queue_mutex_);

    size_t moved_count = 0;
    for (uint64_t keyframe_id : keyframe_ids) {
        auto it = processed_non_optimized_queue_->find(keyframe_id);
        if (it != processed_non_optimized_queue_->end()) {
            (*processed_optimized_queue_)[keyframe_id] = it->second;
            processed_non_optimized_queue_->erase(it);
            moved_count++;
        }
    }

    LOG(INFO) << "Moved batch of " << moved_count << "/" << keyframe_ids.size()
              << " keyframes to processed optimized queue";
}

void MapStore::moveToProcessedOptimized(uint64_t keyframe_id) {
    // This method is deprecated - use moveBatchToProcessedOptimized instead
    std::vector<uint64_t> single_keyframe = {keyframe_id};
    moveBatchToProcessedOptimized(single_keyframe);
}

// Queue-specific accessors
std::vector<KeyFramePtr> MapStore::getUnprocessedKeyFrames() const {
    std::shared_lock<std::shared_mutex> lock(unprocessed_cache_mutex_);
    std::vector<KeyFramePtr> result;
    result.reserve(unprocessed_cache_.size());

    for (const auto& [id, keyframe] : unprocessed_cache_) {
        result.push_back(keyframe);
    }

    return result;
}

std::vector<KeyFramePtr> MapStore::getProcessedNonOptimizedKeyFrames() const {
    std::shared_lock<std::shared_mutex> lock(processed_non_optimized_queue_mutex_);
    std::vector<KeyFramePtr> result;
    result.reserve(processed_non_optimized_queue_->size());

    for (const auto& [id, keyframe] : *processed_non_optimized_queue_) {
        result.push_back(keyframe);
    }

    return result;
}

std::vector<KeyFramePtr> MapStore::getProcessedOptimizedKeyFrames() const {
    std::shared_lock<std::shared_mutex> lock(processed_optimized_queue_mutex_);
    std::vector<KeyFramePtr> result;
    result.reserve(processed_optimized_queue_->size());

    for (const auto& [id, keyframe] : *processed_optimized_queue_) {
        result.push_back(keyframe);
    }

    return result;
}

KeyFramePtr MapStore::getUnprocessedKeyFrame(uint64_t id) const {
    std::shared_lock<std::shared_mutex> lock(unprocessed_cache_mutex_);
    auto it = unprocessed_cache_.find(id);
    return (it != unprocessed_cache_.end()) ? it->second : nullptr;
}

KeyFramePtr MapStore::getProcessedNonOptimizedKeyFrame(uint64_t id) const {
    std::shared_lock<std::shared_mutex> lock(processed_non_optimized_queue_mutex_);
    auto it = processed_non_optimized_queue_->find(id);
    return (it != processed_non_optimized_queue_->end()) ? it->second : nullptr;
}

KeyFramePtr MapStore::getProcessedOptimizedKeyFrame(uint64_t id) const {
    std::shared_lock<std::shared_mutex> lock(processed_optimized_queue_mutex_);
    auto it = processed_optimized_queue_->find(id);
    return (it != processed_optimized_queue_->end()) ? it->second : nullptr;
}

// Queue state queries
size_t MapStore::getUnprocessedCount() const {
    std::shared_lock<std::shared_mutex> lock(unprocessed_cache_mutex_);
    return unprocessed_cache_.size();
}

size_t MapStore::getProcessedNonOptimizedCount() const {
    std::shared_lock<std::shared_mutex> lock(processed_non_optimized_queue_mutex_);
    return processed_non_optimized_queue_->size();
}

size_t MapStore::getProcessedOptimizedCount() const {
    std::shared_lock<std::shared_mutex> lock(processed_optimized_queue_mutex_);
    return processed_optimized_queue_->size();
}

bool MapStore::hasUnprocessedKeyFrame(uint64_t id) const {
    std::shared_lock<std::shared_mutex> lock(unprocessed_cache_mutex_);
    return unprocessed_cache_.find(id) != unprocessed_cache_.end();
}

bool MapStore::hasProcessedNonOptimizedKeyFrame(uint64_t id) const {
    std::shared_lock<std::shared_mutex> lock(processed_non_optimized_queue_mutex_);
    return processed_non_optimized_queue_->find(id) != processed_non_optimized_queue_->end();
}

bool MapStore::hasProcessedOptimizedKeyFrame(uint64_t id) const {
    std::shared_lock<std::shared_mutex> lock(processed_optimized_queue_mutex_);
    return processed_optimized_queue_->find(id) != processed_optimized_queue_->end();
}

// Map point eviction
void MapStore::evictOldMapPoints(uint64_t current_keyframe_id, size_t max_age) {
    std::lock_guard<std::mutex> eviction_lock(map_point_eviction_mutex_);
    std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);

    std::vector<uint32_t> points_to_evict;

    for (const auto& [map_point_id, last_seen_keyframe_id] : map_point_last_seen_) {
        if (current_keyframe_id - last_seen_keyframe_id > max_age) {
            points_to_evict.push_back(map_point_id);
        }
    }

    cache_lock.unlock();  // Release cache lock before disk operations

    LOG(INFO) << "Evicting " << points_to_evict.size() << " old map points (max age: "
              << max_age << " keyframes)";

    for (uint32_t map_point_id : points_to_evict) {
        evictMapPointToDisk(map_point_id);
        map_point_last_seen_.erase(map_point_id);
    }
}

// Batch operations for optimization
std::vector<KeyFramePtr> MapStore::getBatchForOptimization(size_t max_batch_size) const {
    std::shared_lock<std::shared_mutex> lock(processed_non_optimized_queue_mutex_);
    std::vector<KeyFramePtr> batch;
    batch.reserve(std::min(max_batch_size, processed_non_optimized_queue_->size()));

    // Get keyframes in order of ID (oldest first)
    std::vector<std::pair<uint64_t, KeyFramePtr>> sorted_keyframes;
    for (const auto& [id, keyframe] : *processed_non_optimized_queue_) {
        sorted_keyframes.emplace_back(id, keyframe);
    }

    std::sort(sorted_keyframes.begin(), sorted_keyframes.end());

    for (size_t i = 0; i < std::min(max_batch_size, sorted_keyframes.size()); ++i) {
        batch.push_back(sorted_keyframes[i].second);
    }

    LOG(INFO) << "Created optimization batch with " << batch.size() << " keyframes";
    return batch;
}

void MapStore::markBatchAsOptimized(const std::vector<uint64_t>& keyframe_ids) {
    // Move batch to processed optimized queue
    moveBatchToProcessedOptimized(keyframe_ids);

    // Mark all current map points as dirty for writing after optimization
    {
        std::shared_lock<std::shared_mutex> cache_lock(cache_mutex_);
        std::unique_lock<std::shared_mutex> dirty_lock(dirty_map_points_mutex_);

        // Mark all map points as dirty so they get written to disk
        for (const auto& [id, keypoint] : keypoint_cache_) {
            dirty_map_points_.insert(id);
        }

        if (!keypoint_cache_.empty()) {
            LOG(INFO) << "Marked " << keypoint_cache_.size() << " map points as dirty for disk write";
        }
    }

    // Trigger callback-based write to disk
    swapAndWriteToDisk();

    LOG(INFO) << "Marked batch as optimized and triggered write to disk";
}

// Helper methods for queue management
void MapStore::updateMapPointLastSeen(uint64_t keyframe_id) {
    std::lock_guard<std::mutex> eviction_lock(map_point_eviction_mutex_);

    // Use shared lock for reading cache since we're just checking observations
    std::shared_lock<std::shared_mutex> cache_lock(cache_mutex_);

    // Update last seen for all map points observed in this keyframe
    for (const auto& [map_point_id, keypoint] : keypoint_cache_) {
        for (const auto& observation : keypoint.locations) {
            if (observation.keyframe_id == keyframe_id) {
                map_point_last_seen_[map_point_id] = keyframe_id;
                break;
            }
        }
    }
}

void MapStore::evictMapPointToDisk(uint32_t map_point_id) {
    // Note: This method is called from updateMapPointLastSeen which already holds map_point_eviction_mutex_
    // and may be called while cache_lock is held, so we need to be careful about lock ordering

    std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);

    auto it = keypoint_cache_.find(map_point_id);
    if (it != keypoint_cache_.end()) {
        // Add to pending writes for disk storage - inline to avoid recursive lock
        keypoint_pending_writes_[map_point_id] = it->second;
        keypoints_dirty_ = true;

        // Remove from in-memory cache
        keypoint_cache_.erase(it);

        LOG(INFO) << "Evicted map point " << map_point_id << " to disk";
    }
}

void MapStore::syncProcessedOptimizedToDisk() {
    auto start_time = std::chrono::high_resolution_clock::now();

    LOG(INFO) << "Syncing processed optimized queue to disk";

    // Get the current size of the processed optimized queue
    size_t queue_size = 0;
    {
        std::shared_lock<std::shared_mutex> lock(processed_optimized_queue_mutex_);
        queue_size = processed_optimized_queue_->size();
    }

    if (queue_size == 0) {
        LOG(INFO) << "No keyframes in processed optimized queue to sync";
        return;
    }

    // Atomically swap the processed optimized queue with an empty one
    std::unique_ptr<std::unordered_map<uint64_t, KeyFramePtr>> queue_to_write;
    {
        std::unique_lock<std::shared_mutex> lock(processed_optimized_queue_mutex_);
        queue_to_write = std::move(processed_optimized_queue_);
        processed_optimized_queue_ = std::make_unique<std::unordered_map<uint64_t, KeyFramePtr>>();
    }

    LOG(INFO) << "Swapped " << queue_to_write->size() << " keyframes from processed optimized queue";

    // Add all keyframes to pending writes
    {
        std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);

        for (const auto& [id, keyframe] : *queue_to_write) {
            keyframe_pending_writes_[id] = keyframe;
            
            // Update cache
            cacheKeyFrameInternal(id, keyframe);
            
            // Update bounds
            updateMetadataBounds(keyframe->pose);
        }

        keyframes_dirty_ = true;
        metadata_dirty_ = true;
    }

    // Write all dirty map points
    {
        std::shared_lock<std::shared_mutex> dirty_lock(dirty_map_points_mutex_);
        std::unique_lock<std::shared_mutex> cache_lock(cache_mutex_);

        for (uint32_t dirty_id : dirty_map_points_) {
            auto cache_it = keypoint_cache_.find(dirty_id);
            if (cache_it != keypoint_cache_.end()) {
                keypoint_pending_writes_[dirty_id] = cache_it->second;
            }
        }

        if (!dirty_map_points_.empty()) {
            keypoints_dirty_ = true;
            LOG(INFO) << "Added " << dirty_map_points_.size() << " dirty map points to sync";
        }
    }

    // Perform the actual disk write
    bool write_success = writePendingDataToDisk();
    if (!write_success) {
        LOG(ERROR) << "Failed to write processed optimized queue to disk";
        throw std::runtime_error("Failed to sync processed optimized queue to disk");
    }

    // Update metadata and index
    bool index_success = saveChanges();
    if (!index_success) {
        LOG(ERROR) << "Failed to update metadata and index after writing processed optimized queue";
    }

    // Clear dirty map points after successful write
    {
        std::unique_lock<std::shared_mutex> dirty_lock(dirty_map_points_mutex_);
        dirty_map_points_.clear();
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    LOG(INFO) << "Synced processed optimized queue to disk in " << duration.count() << "ms"
              << " (" << queue_to_write->size() << " keyframes)";
}

// Gaussian splat storage methods
bool MapStore::addGaussianSplatBatch(const types::GaussianSplatBatch& batch) {
    // TODO: Implement full splat batch storage
    LOG(INFO) << "addGaussianSplatBatch placeholder - batch " << batch.batch_id 
              << " with " << batch.size() << " splats";
    return true;
}

std::optional<types::GaussianSplatBatch> MapStore::getGaussianSplatBatch(uint32_t batch_id) const {
    // TODO: Implement splat batch loading from disk
    LOG(INFO) << "getGaussianSplatBatch placeholder - batch " << batch_id;
    return std::nullopt;
}

bool MapStore::hasGaussianSplatBatch(uint32_t batch_id) const {
    // TODO: Implement splat batch existence check
    return false;
}

std::vector<types::GaussianSplatBatch> MapStore::getAllGaussianSplatBatches() const {
    // TODO: Implement loading all splat batches
    LOG(INFO) << "getAllGaussianSplatBatches placeholder";
    return {};
}

// Transform tree storage methods
bool MapStore::setTransformTree(std::shared_ptr<stf::TransformTree> tf_tree) {
    std::unique_lock<std::shared_mutex> lock(transform_tree_mutex_);
    transform_tree_ = tf_tree;
    // TODO: Mark transform tree as dirty for disk write
    return true;
}

std::shared_ptr<stf::TransformTree> MapStore::getTransformTree() const {
    std::shared_lock<std::shared_mutex> lock(transform_tree_mutex_);
    return transform_tree_;
}

bool MapStore::saveTransformTreeToDisk() const {
    std::shared_lock<std::shared_mutex> lock(transform_tree_mutex_);
    if (!transform_tree_) {
        return false;
    }
    
    // TODO: Implement transform tree disk storage
    LOG(INFO) << "saveTransformTreeToDisk placeholder";
    return transform_tree_->saveToFile(transform_tree_filepath_);
}

// Splat storage helper methods
bool MapStore::writeSplatBatchesToDisk() {
    // TODO: Implement splat batch disk writing
    LOG(INFO) << "writeSplatBatchesToDisk placeholder";
    return true;
}

bool MapStore::loadSplatBatchFromDisk(uint32_t batch_id, types::GaussianSplatBatch& batch) const {
    // TODO: Implement splat batch loading from disk
    LOG(INFO) << "loadSplatBatchFromDisk placeholder - batch " << batch_id;
    return false;
}

void MapStore::markSplatBatchDirty(uint32_t batch_id) {
    // TODO: Mark splat batch for disk write
    splat_batches_dirty_ = true;
}

}  // namespace storage
}  // namespace core


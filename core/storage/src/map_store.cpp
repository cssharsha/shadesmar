#include <core/storage/map_store.hpp>
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
}

MapStore::~MapStore() {}

bool MapStore::initializeFilePaths(const std::string& map_base_filepath) {
    base_filepath_ = map_base_filepath;
    data_filepath_ = base_filepath_ + ".dat";
    index_filepath_ = base_filepath_ + ".idx";
    metadata_filepath_ = base_filepath_ + ".meta";
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

    std::lock_guard<std::mutex> lock(file_operations_mutex_);  // Protect file operations

    bool is_update = keyframe_locations_.count(keyframe->id) > 0;
    if (is_update) {
        LOG(INFO) << "KeyFrame ID " << keyframe->id
                  << " already exists, updating with optimized pose" << std::endl;
    }

    std::fstream data_file;
    if (!openDataFileForAppend(data_file))
        return false;

    proto::KeyFrame kf_proto = keyframe->toProto();
    proto::FileLocation location;
    if (!writeProtoMessage(data_file, kf_proto, location)) {
        return false;
    }
    data_file.close();

    keyframe_locations_[keyframe->id] = location;

    // Only add to timestamp index if this is a new keyframe
    if (!is_update) {
        timestamp_to_keyframe_ids_[keyframe->pose.timestamp].push_back(keyframe->id);
    }

    // Handle spatial index entries - update existing or add new
    proto::KeyFrameIndexEntry kf_index_entry;
    kf_index_entry.set_keyframe_id(keyframe->id);
    *kf_index_entry.mutable_location() = location;
    kf_index_entry.set_timestamp(keyframe->pose.timestamp);

    if (is_update) {
        // Find and update existing spatial index entry
        bool found = false;
        for (auto& entry : keyframe_spatial_index_entries_) {
            if (entry.keyframe_id() == keyframe->id) {
                entry = kf_index_entry;  // Update with new location and timestamp
                found = true;
                break;
            }
        }
        if (!found) {
            // This shouldn't happen but add it if missing
            keyframe_spatial_index_entries_.push_back(kf_index_entry);
            LOG(WARNING) << "Keyframe " << keyframe->id
                         << " not found in spatial index during update, adding";
        }
    } else {
        // New keyframe - add to spatial index
        keyframe_spatial_index_entries_.push_back(kf_index_entry);
    }

    updateMetadataBounds(keyframe->pose);
    return true;
}

bool MapStore::addFactor(const types::Factor& factor) {
    std::lock_guard<std::mutex> lock(file_operations_mutex_);  // Protect file operations

    if (factor_locations_.count(factor.id)) {
        LOG(INFO) << "Factor ID " << factor.id << " already exists." << std::endl;
    }
    std::fstream data_file;
    if (!openDataFileForAppend(data_file))
        return false;

    LOG(INFO) << "Adding the factor " << factor.id << ", " << factor.type;

    proto::Factor factor_proto = factor.toProto();
    proto::FileLocation location;
    if (!writeProtoMessage(data_file, factor_proto, location))
        return false;
    data_file.close();

    factor_locations_[factor.id] = location;
    LOG(INFO) << "Wrote stuff to stuff";
    return true;
}

bool MapStore::addKeyPoint(const types::Keypoint& keypoint) {
    std::lock_guard<std::mutex> lock(file_operations_mutex_);  // Protect file operations

    if (keypoint_locations_.count(keypoint.id())) {
        LOG(INFO) << "KeyPoint ID " << keypoint.id() << " already exists." << std::endl;
    }
    std::fstream data_file;
    if (!openDataFileForAppend(data_file))
        return false;

    proto::Keypoint kp_proto = keypoint.toProto();
    proto::FileLocation location;
    if (!writeProtoMessage(data_file, kp_proto, location))
        return false;
    data_file.close();

    keypoint_locations_[keypoint.id()] = location;
    return true;
}

KeyFramePtr MapStore::getKeyFrame(uint64_t id) const {
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

std::optional<types::Factor> MapStore::getFactor(uint64_t id) const {
    auto it = factor_locations_.find(id);
    if (it == factor_locations_.end()) {
        return std::nullopt;
    }
    return readProtoMessage<proto::Factor, types::Factor>(it->second, types::Factor::fromProto);
}

std::optional<types::Keypoint> MapStore::getKeyPoint(uint32_t id) const {
    auto it = keypoint_locations_.find(id);
    if (it == keypoint_locations_.end()) {
        return std::nullopt;
    }
    return readProtoMessage<proto::Keypoint, types::Keypoint>(it->second,
                                                              types::Keypoint::fromProto);
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
    metadata_.set_creation_timestamp_seconds(
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    std::fstream meta_stream(metadata_filepath_,
                             std::ios::out | std::ios::trunc | std::ios::binary);
    if (!meta_stream.is_open() || !metadata_.SerializeToOstream(&meta_stream)) {
        LOG(ERROR) << "Failed to save metadata to " << metadata_filepath_ << std::endl;
        return false;
    }
    meta_stream.close();
    LOG(INFO) << "Metadata saved to " << metadata_filepath_ << std::endl;

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
    return true;
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

    for (const auto& kf_idx_entry : keyframe_spatial_index_entries_) {
        timestamp_to_keyframe_ids_[kf_idx_entry.timestamp()].push_back(kf_idx_entry.keyframe_id());
    }
    for (auto& pair_val : timestamp_to_keyframe_ids_) {
        std::sort(pair_val.second.begin(), pair_val.second.end());
        pair_val.second.erase(std::unique(pair_val.second.begin(), pair_val.second.end()),
                              pair_val.second.end());
    }
}

void MapStore::clearDataAndIndices() {
    keyframe_locations_.clear();
    factor_locations_.clear();
    keypoint_locations_.clear();
    timestamp_to_keyframe_ids_.clear();
    keyframe_spatial_index_entries_.clear();

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

    // std::remove(data_filepath_.c_str());
    // std::remove(index_filepath_.c_str());
    // std::remove(metadata_filepath_.c_str());
    LOG(INFO) << "In-memory indices and data cleared." << std::endl;
}

}  // namespace storage
}  // namespace core

#pragma once

#include <fstream>
#include <memory>
#include "core/types/factor.hpp"
#include "core/types/keyframe.hpp"
#include "core/types/keypoint.hpp"

#include "core/proto/map_storage_index.pb.h"

#include <Eigen/Core>

namespace core {
namespace storage {

using KeyFramePtr = types::KeyFrame::Ptr;
class MapStore {
public:
    MapStore(const std::string& map_base_filepth);
    ~MapStore();

    bool initializeFilePaths(const std::string& map_base_filepath);

    bool addKeyFrame(const KeyFramePtr& keyframe);
    bool addFactor(const types::Factor& factor);
    bool addKeyPoint(const types::Keypoint& keypoint);

    KeyFramePtr getKeyFrame(uint64_t id) const;
    std::optional<types::Factor> getFactor(uint64_t id) const;  // Optional in case not found
    std::optional<types::Keypoint> getKeyPoint(uint32_t id) const;

    std::vector<KeyFramePtr> getAllKeyFrames() const;
    std::vector<types::Factor> getAllFactors() const;
    std::vector<types::Keypoint> getAllKeyPoints() const;

    std::vector<KeyFramePtr> getKeyFramesByTimestamp(double timestamp) const;
    std::vector<KeyFramePtr> getKeyFramesByTimestampRange(double start_timestamp,
                                                          double end_timestamp) const;
    std::vector<KeyFramePtr> findKeyFramesNearPosition(const Eigen::Vector3d& target_position,
                                                       double radius, int max_results = -1) const;

    bool saveChanges();
    bool loadMap();

    const proto::MapDiskMetadata& getMetadata() const {
        return metadata_;
    }
    void clearDataAndIndices();

private:
    std::mutex file_operations_mutex_;  // Protect all file operations from concurrent access

    std::string base_filepath_;
    std::string data_filepath_;
    std::string index_filepath_;
    std::string metadata_filepath_;

    proto::MapDiskMetadata metadata_;
    std::map<uint64_t, proto::FileLocation> keyframe_locations_;
    std::map<uint64_t, proto::FileLocation> factor_locations_;
    std::map<uint32_t, proto::FileLocation> keypoint_locations_;

    std::map<double, std::vector<uint64_t>> timestamp_to_keyframe_ids_;
    std::vector<proto::KeyFrameIndexEntry> keyframe_spatial_index_entries_;

    bool openDataFileForAppend(std::fstream& file_stream);
    bool openDataFileForRead(std::fstream& file_stream) const;

    template <typename ProtoType>
    bool writeProtoMessage(std::fstream& stream, const ProtoType& message,
                           proto::FileLocation& out_location);

    template <typename ProtoType, typename CppType>
    std::optional<CppType> readProtoMessage(
        const proto::FileLocation& location,
        std::function<CppType(const ProtoType&)> fromProtoConverter) const;

    void updateMetadataBounds(const types::Pose& pose);
    void rebuildTransientIndices();
    void updateBounds();
};

}  // namespace storage
}  // namespace core

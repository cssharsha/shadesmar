#pragma once

#include <fstream>
#include "core/proto/map.pb.h"
#include "core/types/factor.hpp"
#include "core/types/keyframe.hpp"

namespace core {
namespace storage {

using KeyFramePtr = types::KeyFrame::Ptr;
class MapStore {
public:
    MapStore() {
        map_proto_.mutable_metadata()->set_version("1.0");
        map_proto_.mutable_metadata()->set_created_by("Shadesmar");
    }

    void addKeyFrame(const KeyFramePtr& keyframe);
    KeyFramePtr getKeyFrame(uint64_t id) const;
    std::vector<KeyFramePtr> getAllKeyFrames() const;

    void addFactor(const types::Factor& factor);
    types::Factor getFactor(uint64_t id) const;
    std::vector<types::Factor> getAllFactors() const;

    std::vector<types::Factor> getConnectedFactors(uint64_t keyframe_id) const;

    bool save(const std::string& filename);
    bool load(const std::string& filename);

    void updateBounds();

private:
    proto::Map map_proto_;
    std::map<uint64_t, size_t> keyframe_index_;  // Maps keyframe ID to index in proto
    std::map<uint64_t, size_t> factor_index_;    // Maps factor ID to index in proto
};

}  // namespace storage
}  // namespace core

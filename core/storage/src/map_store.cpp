#include "core/storage/map_store.hpp"

namespace core {
namespace storage {

void MapStore::addKeyFrame(const KeyFramePtr& keyframe) {
    keyframe_index_[keyframe->id] = map_proto_.keyframes_size();
    *map_proto_.add_keyframes() = keyframe->toProto();
    updateBounds();
}

KeyFramePtr MapStore::getKeyFrame(uint64_t id) const {
    auto it = keyframe_index_.find(id);
    if (it == keyframe_index_.end()) return nullptr;
    return types::KeyFrame::fromProto(map_proto_.keyframes(it->second));
}

std::vector<KeyFramePtr> MapStore::getAllKeyFrames() const {
    std::vector<KeyFramePtr> keyframes;
    keyframes.reserve(map_proto_.keyframes_size());
    for (const auto& kf_proto : map_proto_.keyframes()) {
        keyframes.push_back(types::KeyFrame::fromProto(kf_proto));
    }
    return keyframes;
}

void MapStore::addFactor(const types::Factor& factor) {
    factor_index_[factor.id] = map_proto_.factors_size();
    *map_proto_.add_factors() = factor.toProto();
}

types::Factor MapStore::getFactor(uint64_t id) const {
    auto it = factor_index_.find(id);
    if (it == factor_index_.end()) throw std::runtime_error("Factor not found");
    return types::Factor::fromProto(map_proto_.factors(it->second));
}

std::vector<types::Factor> MapStore::getAllFactors() const {
    std::vector<types::Factor> factors;
    factors.reserve(map_proto_.factors_size());
    for (const auto& factor_proto : map_proto_.factors()) {
        factors.push_back(types::Factor::fromProto(factor_proto));
    }
    return factors;
}

std::vector<types::Factor> MapStore::getConnectedFactors(uint64_t keyframe_id) const {
    std::vector<types::Factor> connected_factors;
    for (const auto& factor_proto : map_proto_.factors()) {
        for (const auto& node_id : factor_proto.connected_nodes()) {
            if (node_id == keyframe_id) {
                connected_factors.push_back(types::Factor::fromProto(factor_proto));
                break;
            }
        }
    }
    return connected_factors;
}

bool MapStore::save(const std::string& filename) {
    // Update metadata
    auto* metadata = map_proto_.mutable_metadata();
    metadata->set_creation_timestamp(
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count()
    );

    std::ofstream output(filename, std::ios::binary);
    return map_proto_.SerializeToOstream(&output);
}

bool MapStore::load(const std::string& filename) {
    std::ifstream input(filename, std::ios::binary);
    if (!map_proto_.ParseFromIstream(&input)) return false;

    // Rebuild indices
    keyframe_index_.clear();
    factor_index_.clear();

    for (int i = 0; i < map_proto_.keyframes_size(); ++i) {
        keyframe_index_[map_proto_.keyframes(i).id()] = i;
    }

    for (int i = 0; i < map_proto_.factors_size(); ++i) {
        factor_index_[map_proto_.factors(i).id()] = i;
    }

    return true;
}

void MapStore::updateBounds() {
    if (map_proto_.keyframes_size() == 0) return;

    auto* bounds = map_proto_.mutable_metadata()->mutable_bounds();
    bounds->set_min_x(std::numeric_limits<double>::max());
    bounds->set_min_y(std::numeric_limits<double>::max());
    bounds->set_min_z(std::numeric_limits<double>::max());
    bounds->set_max_x(std::numeric_limits<double>::lowest());
    bounds->set_max_y(std::numeric_limits<double>::lowest());
    bounds->set_max_z(std::numeric_limits<double>::lowest());

    for (const auto& kf : map_proto_.keyframes()) {
        const auto& pos = kf.pose().position();
        bounds->set_min_x(std::min(bounds->min_x(), pos.x()));
        bounds->set_min_y(std::min(bounds->min_y(), pos.y()));
        bounds->set_min_z(std::min(bounds->min_z(), pos.z()));
        bounds->set_max_x(std::max(bounds->max_x(), pos.x()));
        bounds->set_max_y(std::max(bounds->max_y(), pos.y()));
        bounds->set_max_z(std::max(bounds->max_z(), pos.z()));
    }
}

} // namespace storage
} // namespace core
#pragma once

#include <cstdint>
#include <utility>
#include "core/proto/geometry.pb.h"
#include "core/proto/keypoint.pb.h"

namespace core {
namespace types {

using location = std::pair<uint32_t, std::string>;
struct Location {
    uint64_t keyframe_id;
    std::string frame_id;
};

struct Keypoint {
    Eigen::Vector3d position;
    cv::Mat descriptor;
    std::vector<Location> locations;
    // Add additional info like ellipsoid if required later

    Keypoint() {}
    Keypoint(uint32_t id) : keypoint_id(id) {}

    static Keypoint fromProto(const proto::Keypoint& keypoint_proto) {
        Keypoint point(keypoint_proto.id());
        point.position =
            Eigen::Vector3d(keypoint_proto.position().x(), keypoint_proto.position().y(),
                            keypoint_proto.position().z());

        for (const auto& location : keypoint_proto.locations()) {
            Location loc{location.keyframe_id(), location.frame_id()};
            point.locations.emplace_back(std::move(loc));
        }
        return point;
    }

    proto::Keypoint toProto() const {
        proto::Keypoint keypoint_proto;
        auto* point_proto = keypoint_proto.mutable_position();
        point_proto->set_x(position.x());
        point_proto->set_y(position.y());
        point_proto->set_z(position.z());

        for (const auto& location : locations) {
            auto* l = keypoint_proto.add_locations();
            l->set_keyframe_id(location.keyframe_id);
            l->set_frame_id(location.frame_id);
        }
    }

    uint32_t id() const {
        return keypoint_id;
    }

private:
    uint32_t keypoint_id;
};
}  // namespace types
}  // namespace core

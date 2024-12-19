#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "core/proto/geometry.pb.h"

namespace core {
namespace types {

struct Pose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d position{0, 0, 0};
    Eigen::Quaterniond orientation{1, 0, 0, 0};
    double timestamp{0};

    // Proto conversion
    static Pose fromProto(const proto::Pose& pose_proto) {
        Pose pose;
        pose.position = Eigen::Vector3d(pose_proto.position().x(), pose_proto.position().y(),
                                        pose_proto.position().z());
        pose.orientation =
            Eigen::Quaterniond(pose_proto.orientation().w(), pose_proto.orientation().x(),
                               pose_proto.orientation().y(), pose_proto.orientation().z());
        pose.timestamp = pose_proto.timestamp();
        return pose;
    }

    proto::Pose toProto() const {
        proto::Pose pose_proto;
        auto* position_proto = pose_proto.mutable_position();
        position_proto->set_x(position.x());
        position_proto->set_y(position.y());
        position_proto->set_z(position.z());

        auto* orientation_proto = pose_proto.mutable_orientation();
        orientation_proto->set_w(orientation.w());
        orientation_proto->set_x(orientation.x());
        orientation_proto->set_y(orientation.y());
        orientation_proto->set_z(orientation.z());

        pose_proto.set_timestamp(timestamp);
        return pose_proto;
    }

    // Convenience methods
    Pose inverse() const {
        Pose inv;
        inv.orientation = orientation.conjugate();
        inv.position = -(inv.orientation * position);
        inv.timestamp = timestamp;
        return inv;
    }

    Pose operator*(const Pose& other) const {
        Pose result;
        result.orientation = orientation * other.orientation;
        result.position = position + orientation * other.position;
        result.timestamp = other.timestamp;
        return result;
    }
};

}  // namespace types
}  // namespace core

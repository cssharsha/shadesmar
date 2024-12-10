#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "core/proto/factor.pb.h"
#include "core/proto/geometry.pb.h"

namespace core {
namespace types {

struct Factor {
  uint64_t id;
  proto::FactorType type;
  std::vector<uint64_t> connected_nodes;

  // Factor measurement (relative or absolute pose)
  std::variant<Pose, Pose> measurement;

  // Information matrix (inverse covariance)
  Eigen::Matrix<double, 6, 6> information;
  double timestamp;
  std::string description;

  proto::Factor toProto() const {
    proto::Factor factor_proto;
    factor_proto.set_id(id);
    factor_proto.set_type(type);

    for (const auto &node_id : connected_nodes) {
      factor_proto.add_connected_nodes(node_id);
    }

    if (type == proto::FactorType::PRIOR) {
      *factor_proto.mutable_absolute_pose() =
          std::get<0>(measurement).toProto();
    } else {
      *factor_proto.mutable_relative_pose() =
          std::get<0>(measurement).toProto();
    }

    for (int i = 0; i < 36; ++i) {
      factor_proto.add_information_matrix(information.data()[i]);
    }

    factor_proto.set_timestamp(timestamp);
    factor_proto.set_description(description);
    return factor_proto;
  }

  static Factor fromProto(const proto::Factor &factor_proto) {
    Factor factor;
    factor.id = factor_proto.id();
    factor.type = factor_proto.type();

    factor.connected_nodes.reserve(factor_proto.connected_nodes_size());
    for (const auto &node_id : factor_proto.connected_nodes()) {
      factor.connected_nodes.push_back(node_id);
    }

    if (factor_proto.has_absolute_pose()) {
      factor.measurement.emplace<0>(
          Pose::fromProto(factor_proto.absolute_pose()));
    } else {
      factor.measurement.emplace<0>(
          Pose::fromProto(factor_proto.relative_pose()));
    }

    factor.information =
        Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
            factor_proto.information_matrix().data());

    factor.timestamp = factor_proto.timestamp();
    factor.description = factor_proto.description();
    return factor;
  }
};

} // namespace types
} // namespace core
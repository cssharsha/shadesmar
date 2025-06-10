#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <vector>
#include "core/proto/gaussian_splat.pb.h"

namespace core {
namespace types {

struct GaussianSplat {
    uint32_t id;
    Eigen::Vector3d position;        // μ (mean position)
    Eigen::Matrix3d covariance;      // Σ (3x3 covariance matrix)  
    Eigen::Vector3f color;           // RGB color [0.0, 1.0]
    float opacity;                   // α (alpha/opacity) [0.0, 1.0]
    uint32_t source_keypoint_id;     // ID of keypoint that generated this splat
    float confidence;                // Quality/confidence metric
    double timestamp;                // Creation timestamp

    GaussianSplat()
        : id(0), 
          position(Eigen::Vector3d::Zero()),
          covariance(Eigen::Matrix3d::Identity()),
          color(Eigen::Vector3f::Zero()),
          opacity(1.0f),
          source_keypoint_id(0),
          confidence(1.0f),
          timestamp(0.0) {}

    GaussianSplat(uint32_t splat_id, const Eigen::Vector3d& pos, const Eigen::Matrix3d& cov,
                  const Eigen::Vector3f& rgb, float alpha, uint32_t keypoint_id = 0)
        : id(splat_id),
          position(pos),
          covariance(cov),
          color(rgb),
          opacity(alpha),
          source_keypoint_id(keypoint_id),
          confidence(1.0f),
          timestamp(0.0) {}

    // Convert to protobuf message
    void toProto(proto::GaussianSplat& proto_splat) const {
        proto_splat.set_id(id);
        
        auto* pos_proto = proto_splat.mutable_position();
        pos_proto->set_x(position.x());
        pos_proto->set_y(position.y());
        pos_proto->set_z(position.z());
        
        // Store covariance as 9 elements in row-major order
        proto_splat.clear_covariance();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                proto_splat.add_covariance(covariance(i, j));
            }
        }
        
        auto* color_proto = proto_splat.mutable_color();
        color_proto->set_x(color.x());
        color_proto->set_y(color.y());
        color_proto->set_z(color.z());
        
        proto_splat.set_opacity(opacity);
        proto_splat.set_source_keypoint_id(source_keypoint_id);
        proto_splat.set_confidence(confidence);
        proto_splat.set_timestamp(timestamp);
    }

    // Create from protobuf message
    static GaussianSplat fromProto(const proto::GaussianSplat& proto_splat) {
        GaussianSplat splat;
        splat.id = proto_splat.id();
        
        const auto& pos_proto = proto_splat.position();
        splat.position = Eigen::Vector3d(pos_proto.x(), pos_proto.y(), pos_proto.z());
        
        // Reconstruct covariance matrix from 9 elements
        if (proto_splat.covariance_size() == 9) {
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    splat.covariance(i, j) = proto_splat.covariance(i * 3 + j);
                }
            }
        } else {
            splat.covariance = Eigen::Matrix3d::Identity();
        }
        
        const auto& color_proto = proto_splat.color();
        splat.color = Eigen::Vector3f(color_proto.x(), color_proto.y(), color_proto.z());
        
        splat.opacity = proto_splat.opacity();
        splat.source_keypoint_id = proto_splat.source_keypoint_id();
        splat.confidence = proto_splat.confidence();
        splat.timestamp = proto_splat.timestamp();
        
        return splat;
    }

    // Validate splat parameters
    bool isValid() const {
        // Check position is finite
        if (!position.allFinite()) return false;
        
        // Check covariance is positive semi-definite
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.eigenvalues().minCoeff() < 0) return false;
        
        // Check color range [0, 1]
        if (color.minCoeff() < 0.0f || color.maxCoeff() > 1.0f) return false;
        
        // Check opacity range [0, 1]
        if (opacity < 0.0f || opacity > 1.0f) return false;
        
        return true;
    }

    // Get ellipsoid axes and scales for visualization
    void getEllipsoidParameters(Eigen::Vector3d& scales, Eigen::Matrix3d& rotation) const {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        scales = solver.eigenvalues().cwiseSqrt();
        rotation = solver.eigenvectors();
    }
};

struct GaussianSplatBatch {
    uint32_t batch_id;
    std::vector<GaussianSplat> splats;
    uint64_t start_keyframe_id;
    uint64_t end_keyframe_id;
    double timestamp;

    GaussianSplatBatch()
        : batch_id(0), start_keyframe_id(0), end_keyframe_id(0), timestamp(0.0) {}

    // Convert to protobuf message
    void toProto(proto::GaussianSplatBatch& proto_batch) const {
        proto_batch.set_batch_id(batch_id);
        proto_batch.set_start_keyframe_id(start_keyframe_id);
        proto_batch.set_end_keyframe_id(end_keyframe_id);
        proto_batch.set_timestamp(timestamp);
        proto_batch.set_splat_count(splats.size());
        
        proto_batch.clear_splats();
        for (const auto& splat : splats) {
            auto* splat_proto = proto_batch.add_splats();
            splat.toProto(*splat_proto);
        }
    }

    // Create from protobuf message
    static GaussianSplatBatch fromProto(const proto::GaussianSplatBatch& proto_batch) {
        GaussianSplatBatch batch;
        batch.batch_id = proto_batch.batch_id();
        batch.start_keyframe_id = proto_batch.start_keyframe_id();
        batch.end_keyframe_id = proto_batch.end_keyframe_id();
        batch.timestamp = proto_batch.timestamp();
        
        batch.splats.reserve(proto_batch.splats_size());
        for (const auto& splat_proto : proto_batch.splats()) {
            batch.splats.push_back(GaussianSplat::fromProto(splat_proto));
        }
        
        return batch;
    }

    size_t size() const { return splats.size(); }
    bool empty() const { return splats.empty(); }
};

}  // namespace types
}  // namespace core
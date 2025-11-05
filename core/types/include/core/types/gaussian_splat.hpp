#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <set>
#include <vector>
#include "core/proto/gaussian_splat.pb.h"

namespace core {
namespace types {

struct GaussianSplat {
    uint32_t id;
    Eigen::Vector3d position;    // μ (mean position)
    Eigen::Matrix3d covariance;  // Σ (3x3 covariance matrix)
    float opacity;               // α (alpha/opacity) [0.0, 1.0]
    Eigen::Vector3d scale;
    Eigen::Quaterniond rotation;

    // Spherical harmonics representation
    Eigen::Vector3f sh_dc;       // DC component (0th degree) - base color in SH space
    Eigen::VectorXf sh_rest;     // Higher order coefficients (degrees 1-3)
    int sh_degree = 3;           // SH degree (0-3)

    uint32_t source_keypoint_id;  // ID of keypoint that generated this splat
    float confidence;             // Quality/confidence metric
    double timestamp;             // Creation timestamp

    GaussianSplat()
        : id(0),
          position(Eigen::Vector3d::Zero()),
          covariance(Eigen::Matrix3d::Identity()),
          opacity(1.0f),
          scale(Eigen::Vector3d::Ones()),
          rotation(Eigen::Quaterniond::Identity()),
          sh_dc(Eigen::Vector3f::Zero()),
          sh_rest(Eigen::VectorXf::Zero(45)),  // 15 coeffs × 3 RGB for degree 3
          source_keypoint_id(0),
          confidence(1.0f),
          timestamp(0.0) {}

    GaussianSplat(uint32_t splat_id, const Eigen::Vector3d& pos, const Eigen::Matrix3d& cov,
                  const Eigen::Vector3f& rgb, float alpha, uint32_t keypoint_id = 0)
        : id(splat_id),
          position(pos),
          covariance(cov),
          opacity(alpha),
          scale(Eigen::Vector3d::Ones()),
          rotation(Eigen::Quaterniond::Identity()),
          sh_rest(Eigen::VectorXf::Zero(45)),
          source_keypoint_id(keypoint_id),
          confidence(1.0f),
          timestamp(0.0) {
        // Convert RGB to SH DC component: sh_dc = (rgb - 0.5) / C0
        constexpr float C0 = 0.28209479177387814f;
        sh_dc = (rgb.array() - 0.5f) / C0;
    }

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

        proto_splat.set_opacity(opacity);
        proto_splat.set_source_keypoint_id(source_keypoint_id);
        proto_splat.set_confidence(confidence);
        proto_splat.set_timestamp(timestamp);

        auto* scale_proto = proto_splat.mutable_scale();
        scale_proto->set_x(scale.x());
        scale_proto->set_y(scale.y());
        scale_proto->set_z(scale.z());

        auto* rotation_proto = proto_splat.mutable_rotation();
        rotation_proto->set_w(rotation.w());
        rotation_proto->set_x(rotation.x());
        rotation_proto->set_y(rotation.y());
        rotation_proto->set_z(rotation.z());

        // Store spherical harmonics DC component
        auto* sh_dc_proto = proto_splat.mutable_sh_dc();
        sh_dc_proto->set_x(sh_dc.x());
        sh_dc_proto->set_y(sh_dc.y());
        sh_dc_proto->set_z(sh_dc.z());

        // Store higher order SH coefficients
        proto_splat.clear_sh_rest();
        for (int i = 0; i < sh_rest.size(); ++i) {
            proto_splat.add_sh_rest(sh_rest(i));
        }

        proto_splat.set_sh_degree(sh_degree);
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

        splat.opacity = proto_splat.opacity();
        splat.source_keypoint_id = proto_splat.source_keypoint_id();
        splat.confidence = proto_splat.confidence();
        splat.timestamp = proto_splat.timestamp();

        const auto& scale_proto = proto_splat.scale();
        splat.scale = Eigen::Vector3d(scale_proto.x(), scale_proto.y(), scale_proto.z());

        const auto& rotation_proto = proto_splat.rotation();
        splat.rotation = Eigen::Quaterniond(rotation_proto.w(), rotation_proto.x(),
                                            rotation_proto.y(), rotation_proto.z());

        // Load spherical harmonics DC component
        const auto& sh_dc_proto = proto_splat.sh_dc();
        splat.sh_dc = Eigen::Vector3f(sh_dc_proto.x(), sh_dc_proto.y(), sh_dc_proto.z());

        // Load higher order SH coefficients
        splat.sh_rest.resize(proto_splat.sh_rest_size());
        for (int i = 0; i < proto_splat.sh_rest_size(); ++i) {
            splat.sh_rest(i) = proto_splat.sh_rest(i);
        }

        splat.sh_degree = proto_splat.sh_degree();

        return splat;
    }

    // Get RGB color from SH DC component
    Eigen::Vector3f getColor() const {
        constexpr float C0 = 0.28209479177387814f;
        Eigen::Vector3f color = (sh_dc.array() * C0 + 0.5f).matrix();
        return color.cwiseMax(0.0f).cwiseMin(1.0f);  // Clamp to [0, 1]
    }

    // Set SH DC component from RGB color
    void setColor(const Eigen::Vector3f& rgb) {
        constexpr float C0 = 0.28209479177387814f;
        sh_dc = (rgb.array() - 0.5f) / C0;
    }

    // Validate splat parameters
    bool isValid() const {
        // Check position is finite
        if (!position.allFinite())
            return false;

        // Check covariance is positive semi-definite
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.eigenvalues().minCoeff() < 0)
            return false;

        // Check SH DC component is finite
        if (!sh_dc.allFinite())
            return false;

        // Check opacity range [0, 1]
        if (opacity < 0.0f || opacity > 1.0f)
            return false;

        return true;
    }

    // Get ellipsoid axes and scales for visualization
    void getEllipsoidParameters(Eigen::Vector3d& scales, Eigen::Matrix3d& rotation) const {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        scales = solver.eigenvalues().cwiseSqrt();
        rotation = solver.eigenvectors();
    }

    void getScales(Eigen::Vector3d& scales) const {
        scales = scale;
    }
};

struct GaussianSplatBatch {
    uint32_t batch_id;
    std::vector<GaussianSplat> splats;
    uint64_t start_keyframe_id;
    uint64_t end_keyframe_id;
    std::set<uint64_t> source_keyframe_ids;  // Set of keyframe IDs that generated this batch
    double timestamp;

    GaussianSplatBatch() : batch_id(0), start_keyframe_id(0), end_keyframe_id(0), timestamp(0.0) {}

    // Convert to protobuf message
    void toProto(proto::GaussianSplatBatch& proto_batch) const {
        proto_batch.set_batch_id(batch_id);
        proto_batch.set_start_keyframe_id(start_keyframe_id);
        proto_batch.set_end_keyframe_id(end_keyframe_id);
        proto_batch.set_timestamp(timestamp);
        proto_batch.set_splat_count(splats.size());

        // Store source keyframe IDs
        proto_batch.clear_source_keyframe_ids();
        for (const auto& keyframe_id : source_keyframe_ids) {
            proto_batch.add_source_keyframe_ids(keyframe_id);
        }

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

        // Load source keyframe IDs
        for (const auto& keyframe_id : proto_batch.source_keyframe_ids()) {
            batch.source_keyframe_ids.insert(keyframe_id);
        }

        batch.splats.reserve(proto_batch.splats_size());
        for (const auto& splat_proto : proto_batch.splats()) {
            batch.splats.push_back(GaussianSplat::fromProto(splat_proto));
        }

        return batch;
    }

    size_t size() const {
        return splats.size();
    }
    bool empty() const {
        return splats.empty();
    }
};

}  // namespace types
}  // namespace core

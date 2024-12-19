#pragma once

#include "core/proto/geometry.pb.h"
#include "core/proto/keyframe.pb.h"
#include "core/proto/sensor_data.pb.h"
#include "core/types/image.hpp"
#include "core/types/pose.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>

#include <optional>
#include <variant>

namespace core {
namespace types {

struct PointCloud {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> colors;

    proto::PointCloud toProto() const {
        proto::PointCloud cloud_proto;
        for (const auto& point : points) {
            auto* p = cloud_proto.add_points();
            p->set_x(point.x());
            p->set_y(point.y());
            p->set_z(point.z());
        }
        for (const auto& color : colors) {
            auto* c = cloud_proto.add_colors();
            c->set_x(color.x());
            c->set_y(color.y());
            c->set_z(color.z());
        }
        return cloud_proto;
    }

    static PointCloud fromProto(const proto::PointCloud& cloud_proto) {
        PointCloud cloud;
        cloud.points.reserve(cloud_proto.points_size());
        cloud.colors.reserve(cloud_proto.colors_size());

        for (const auto& point : cloud_proto.points()) {
            cloud.points.emplace_back(point.x(), point.y(), point.z());
        }
        for (const auto& color : cloud_proto.colors()) {
            cloud.colors.emplace_back(color.x(), color.y(), color.z());
        }
        return cloud;
    }
};

class KeyFrame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<KeyFrame>;
    using ConstPtr = std::shared_ptr<const KeyFrame>;

    uint64_t id;
    Pose pose;
    std::variant<PointCloud, Image> depth_data;
    std::optional<Image> color_data;
    std::optional<proto::CameraInfo> camera_info;

    proto::KeyFrame toProto() const {
        proto::KeyFrame kf_proto;
        kf_proto.set_id(id);
        *kf_proto.mutable_pose() = pose.toProto();

        if (std::holds_alternative<PointCloud>(depth_data)) {
            *kf_proto.mutable_point_cloud() = std::get<PointCloud>(depth_data).toProto();
        } else {
            *kf_proto.mutable_depth_image() = std::get<Image>(depth_data).toProto();
        }

        if (color_data) {
            *kf_proto.mutable_color_image() = color_data->toProto();
        }

        if (camera_info) {
            *kf_proto.mutable_camera_info() = *camera_info;
        }

        return kf_proto;
    }

    static Ptr fromProto(const proto::KeyFrame& kf_proto) {
        auto kf = std::make_shared<KeyFrame>();
        kf->id = kf_proto.id();
        kf->pose = Pose::fromProto(kf_proto.pose());

        if (kf_proto.has_point_cloud()) {
            kf->depth_data = PointCloud::fromProto(kf_proto.point_cloud());
        } else if (kf_proto.has_depth_image()) {
            kf->depth_data = Image::fromProto(kf_proto.depth_image());
        }

        if (kf_proto.has_color_image()) {
            kf->color_data = Image::fromProto(kf_proto.color_image());
        }

        if (kf_proto.has_camera_info()) {
            kf->camera_info = kf_proto.camera_info();
        }

        return kf;
    }

    bool hasPointCloud() const {
        return std::holds_alternative<PointCloud>(depth_data);
    }

    const PointCloud& getPointCloud() const {
        if (!hasPointCloud()) {
            throw std::runtime_error("Point cloud not available");
        }
        return std::get<PointCloud>(depth_data);
    }
};

}  // namespace types
}  // namespace core

#pragma once

#include "core/proto/geometry.pb.h"
#include "core/proto/keyframe.pb.h"
#include "core/proto/sensor_data.pb.h"
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
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      points;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      colors;

  proto::PointCloud toProto() const {
    proto::PointCloud cloud_proto;
    for (const auto &point : points) {
      auto *p = cloud_proto.add_points();
      p->set_x(point.x());
      p->set_y(point.y());
      p->set_z(point.z());
    }
    for (const auto &color : colors) {
      auto *c = cloud_proto.add_colors();
      c->set_x(color.x());
      c->set_y(color.y());
      c->set_z(color.z());
    }
    return cloud_proto;
  }

  static PointCloud fromProto(const proto::PointCloud &cloud_proto) {
    PointCloud cloud;
    cloud.points.reserve(cloud_proto.points_size());
    cloud.colors.reserve(cloud_proto.colors_size());

    for (const auto &point : cloud_proto.points()) {
      cloud.points.emplace_back(point.x(), point.y(), point.z());
    }
    for (const auto &color : cloud_proto.colors()) {
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
  std::variant<PointCloud, cv::Mat> data;
  std::optional<proto::CameraInfo> camera_info;

  proto::KeyFrame toProto() const {
    proto::KeyFrame kf_proto;
    kf_proto.set_id(id);
    *kf_proto.mutable_pose() = pose.toProto();

    if (std::holds_alternative<PointCloud>(data)) {
      *kf_proto.mutable_point_cloud() = std::get<PointCloud>(data).toProto();
    } else {
      const cv::Mat &img = std::get<cv::Mat>(data);
      auto *image_proto = kf_proto.mutable_image();
      image_proto->set_data(img.data, img.total() * img.elemSize());
      image_proto->set_width(img.cols);
      image_proto->set_height(img.rows);
      image_proto->set_channels(img.channels());
      image_proto->set_encoding(img.depth() == CV_8U ? "rgb8" : "bgr8");
    }

    if (camera_info) {
      *kf_proto.mutable_camera_info() = *camera_info;
    }

    return kf_proto;
  }

  static Ptr fromProto(const proto::KeyFrame &kf_proto) {
    auto kf = std::make_shared<KeyFrame>();
    kf->id = kf_proto.id();
    kf->pose = Pose::fromProto(kf_proto.pose());

    if (kf_proto.has_point_cloud()) {
      kf->data = PointCloud::fromProto(kf_proto.point_cloud());
    } else if (kf_proto.has_image()) {
      const auto &img_proto = kf_proto.image();
      cv::Mat img(img_proto.height(), img_proto.width(),
                  CV_8UC(img_proto.channels()),
                  const_cast<void *>(
                      static_cast<const void *>(img_proto.data().data())));
      kf->data = img.clone();
    }

    if (kf_proto.has_camera_info()) {
      kf->camera_info = kf_proto.camera_info();
    }

    return kf;
  }

  bool hasPointCloud() const {
    return std::holds_alternative<PointCloud>(data);
  }

  const PointCloud &getPointCloud() const {
    if (!hasPointCloud()) {
      throw std::runtime_error("Point cloud not available");
    }
    return std::get<PointCloud>(data);
  }
};

} // namespace types
} // namespace core
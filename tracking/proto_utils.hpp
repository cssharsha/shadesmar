#pragma once

#include "tracking/proto/tracking_test_case.pb.h"
#include "core/proto/keyframe.pb.h"
#include "utils/stf/proto/transform_tree.pb.h"

#include <core/types/keyframe.hpp>
#include "utils/stf/transform_tree.hpp"
#include <opencv2/core/types.hpp>
#include <vector>
#include <string>

namespace tracking {
namespace proto_utils {

// Eigen::Isometry3d <-> core.proto.Pose
core::proto::Pose toProto(const Eigen::Isometry3d& isometry);
Eigen::Isometry3d fromProto(const core::proto::Pose& pose_proto);

// core::types::Image <-> core.proto.Image
core::proto::Image toProto(const core::types::Image& image_data);
core::types::Image fromProto(const core::proto::Image& image_proto);

// core::types::CameraInfo <-> core.proto.CameraInfo
core::proto::CameraInfo toProto(const core::types::CameraInfo& camera_info);
core::types::CameraInfo fromProto(const core::proto::CameraInfo& camera_info_proto);

// core::types::KeyFrame <-> core.proto.KeyFrame
core::proto::KeyFrame toProto(const core::types::KeyFrame& keyframe);
core::types::KeyFrame fromProto(const core::proto::KeyFrame& keyframe_proto);

// stf::TransformTree <-> stf.proto.TransformTreeSnapshot
stf::proto::TransformTreeSnapshot toProto(const stf::TransformTree& transform_tree);
stf::TransformTree fromProto(const stf::proto::TransformTreeSnapshot& snapshot_proto);

// std::vector<cv::Point2f> <-> tracking.proto.MatchedPoints
tracking::proto::MatchedPoints toProto(const std::vector<cv::Point2f>& points1,
                                       const std::vector<cv::Point2f>& points2);
void fromProto(const tracking::proto::MatchedPoints& matches_proto,
               std::vector<cv::Point2f>& points1,
               std::vector<cv::Point2f>& points2);

// Load/Save TrackingTestCase
bool saveTrackingTestCase(const tracking::proto::TrackingTestCase& test_case, const std::string& filename);
tracking::proto::TrackingTestCase loadTrackingTestCase(const std::string& filename);

} // namespace proto_utils
} // namespace tracking
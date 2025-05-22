#pragma once

#include <Eigen/Geometry>

#include <core/types/keyframe.hpp>
#include "transform_tree.hpp"

namespace stf {

Eigen::Vector3d getRPY(const Eigen::Isometry3d& iso);
Eigen::Isometry3d getRelative(const core::types::Pose& src_pose, const core::types::Pose& trg_pose);

// Legacy functions with hardcoded "base_link" - kept for backward compatibility
Eigen::Isometry3d getRelative(const core::types::KeyFrame& src_frame,
                              const core::types::KeyFrame& trg_frame, const TransformTree& tft);

Eigen::Isometry3d getRelative(const core::types::Pose& src_pose, const core::types::Pose& trg_pose,
                              const std::string& frame_id, const TransformTree& tft);

// New configurable functions with explicit base_link_frame_id parameter
Eigen::Isometry3d getRelativeWithBaseLink(const core::types::KeyFrame& src_frame,
                                          const core::types::KeyFrame& trg_frame, const TransformTree& tft,
                                          const std::string& base_link_frame_id);

Eigen::Isometry3d getRelativeWithBaseLink(const core::types::Pose& src_pose, const core::types::Pose& trg_pose,
                                          const std::string& frame_id, const TransformTree& tft,
                                          const std::string& base_link_frame_id);

// Functions for transforming poses to different reference frames
Eigen::Isometry3d getRelativeInReferenceFrame(const core::types::KeyFrame& src_frame,
                                              const core::types::KeyFrame& trg_frame, const TransformTree& tft,
                                              const std::string& reference_frame_id);

Eigen::Isometry3d getRelativeInReferenceFrame(const core::types::Pose& src_pose, const core::types::Pose& trg_pose,
                                              const std::string& frame_id, const TransformTree& tft,
                                              const std::string& reference_frame_id);

}  // namespace stf

#include <cmath>
#include <stf/transform_utils.hpp>

#include <core/types/keyframe.hpp>
#include <logging/logging.hpp>
#include <stf/transform_tree.hpp>
namespace stf {

Eigen::Vector3d getRPY(const Eigen::Isometry3d& iso) {
    Eigen::Vector3d rpy_rad = iso.rotation().eulerAngles(2, 1, 0);
    Eigen::Vector3d rpy;
    rpy.x() = rpy_rad[0] * 180.0 / M_PI;
    rpy.y() = rpy_rad[1] * 180.0 / M_PI;
    rpy.z() = rpy_rad[2] * 180.0 / M_PI;
    return rpy;
}

Eigen::Isometry3d getRelative(const core::types::Pose& src_pose,
                              const core::types::Pose& trg_pose) {
    auto src_pose_eigen = src_pose.getEigenIsometry();
    auto trg_pose_eigen = trg_pose.getEigenIsometry();

    return src_pose_eigen.inverse() * trg_pose_eigen;
}

// Legacy function with hardcoded "base_link" - kept for backward compatibility
Eigen::Isometry3d getRelative(const core::types::KeyFrame& src_frame,
                              const core::types::KeyFrame& trg_frame, const TransformTree& tft) {
    LOG(INFO) << "Asdasd";
    return getRelativeWithBaseLink(src_frame, trg_frame, tft, "base_link");
}

// Legacy function with hardcoded "base_link" - kept for backward compatibility
Eigen::Isometry3d getRelative(const core::types::Pose& src_pose, const core::types::Pose& trg_pose,
                              const std::string& frame_id, const TransformTree& tft) {
    return getRelativeWithBaseLink(src_pose, trg_pose, frame_id, tft, "base_link");
}

// New configurable function with explicit base_link_frame_id parameter
Eigen::Isometry3d getRelativeWithBaseLink(const core::types::KeyFrame& src_frame,
                                          const core::types::KeyFrame& trg_frame,
                                          const TransformTree& tft,
                                          const std::string& base_link_frame_id) {
    LOG(INFO) << "has value: " << src_frame.color_data.has_value() << " "
              << trg_frame.color_data.value().frame_id;
    LOG(INFO) << "base link " << base_link_frame_id << " -> "
              << trg_frame.color_data.value().frame_id;
    auto T_base_link_t_camera_src =
        tft.getTransform(base_link_frame_id, trg_frame.color_data.value().frame_id).transform;
    LOG(INFO) << "Transform: [" << T_base_link_t_camera_src.translation().transpose() << "]["
              << getRPY(T_base_link_t_camera_src).transpose();

    auto T_odom_base_link_src = src_frame.pose.getEigenIsometry();
    LOG(INFO) << "ODOM bae link: [" << T_odom_base_link_src.translation().transpose() << "]["
              << getRPY(T_odom_base_link_src).transpose();
    auto T_odom_camera_src = T_odom_base_link_src * T_base_link_t_camera_src;
    LOG(INFO) << "ODMO to cam frame: " << T_odom_camera_src.translation().transpose() << "]["
              << getRPY(T_odom_camera_src).transpose();

    auto T_base_link_t_camera_trg =
        tft.getTransform(base_link_frame_id, trg_frame.color_data.value().frame_id).transform;
    auto T_odom_base_link_trg = trg_frame.pose.getEigenIsometry();
    LOG(INFO) << "base link " << base_link_frame_id << " -> "
              << trg_frame.color_data.value().frame_id;
    LOG(INFO) << "Transform: [" << T_base_link_t_camera_trg.translation().transpose() << "]["
              << getRPY(T_base_link_t_camera_trg).transpose();
    LOG(INFO) << "ODOM bae link: [" << T_odom_base_link_trg.translation().transpose() << "]["
              << getRPY(T_odom_base_link_trg).transpose();
    auto T_odom_camera_trg = T_odom_base_link_trg * T_base_link_t_camera_trg;
    LOG(INFO) << "ODMO to cam frame: " << T_odom_camera_trg.translation().transpose() << "]["
              << getRPY(T_odom_camera_trg).transpose();

    return T_odom_camera_src.inverse() * T_odom_camera_trg;
}

// New configurable function with explicit base_link_frame_id parameter
Eigen::Isometry3d getRelativeWithBaseLink(const core::types::Pose& src_pose,
                                          const core::types::Pose& trg_pose,
                                          const std::string& frame_id, const TransformTree& tft,
                                          const std::string& base_link_frame_id) {
    auto T_base_link_t_camera = tft.getTransform(base_link_frame_id, frame_id).transform;

    auto T_odom_base_link_src = src_pose.getEigenIsometry();
    auto T_odom_camera_src = T_odom_base_link_src * T_base_link_t_camera;

    auto T_odom_base_link_trg = trg_pose.getEigenIsometry();
    auto T_odom_camera_trg = T_odom_base_link_trg * T_base_link_t_camera;

    return T_odom_camera_src.inverse() * T_odom_camera_trg;
}

// Functions for transforming poses to different reference frames
Eigen::Isometry3d getRelativeInReferenceFrame(const core::types::KeyFrame& src_frame,
                                              const core::types::KeyFrame& trg_frame,
                                              const TransformTree& tft,
                                              const std::string& reference_frame_id) {
    auto T_reference_camera_src =
        tft.getTransform(reference_frame_id, src_frame.color_data.value().frame_id).transform;

    auto T_odom_reference_src = src_frame.pose.getEigenIsometry();
    auto T_odom_camera_src = T_odom_reference_src * T_reference_camera_src;

    auto T_reference_camera_trg =
        tft.getTransform(reference_frame_id, trg_frame.color_data.value().frame_id).transform;
    auto T_odom_reference_trg = trg_frame.pose.getEigenIsometry();
    auto T_odom_camera_trg = T_odom_reference_trg * T_reference_camera_trg;

    return T_odom_camera_src.inverse() * T_odom_camera_trg;
}

Eigen::Isometry3d getRelativeInReferenceFrame(const core::types::Pose& src_pose,
                                              const core::types::Pose& trg_pose,
                                              const std::string& frame_id, const TransformTree& tft,
                                              const std::string& reference_frame_id) {
    auto T_reference_camera = tft.getTransform(reference_frame_id, frame_id).transform;

    auto T_odom_reference_src = src_pose.getEigenIsometry();
    auto T_odom_camera_src = T_odom_reference_src * T_reference_camera;

    auto T_odom_reference_trg = trg_pose.getEigenIsometry();
    auto T_odom_camera_trg = T_odom_reference_trg * T_reference_camera;

    return T_odom_camera_src.inverse() * T_odom_camera_trg;
}

}  // namespace stf

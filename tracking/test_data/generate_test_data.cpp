#include "tracking/proto_utils.hpp"
#include "core/types/keyframe.hpp" // For core::types::Pose, CameraInfo etc.
#include "stf/transform_tree.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

// Helper to create a KeyFrame proto
core::proto::KeyFrame create_kf_proto(uint64_t id, const Eigen::Isometry3d& odom_T_baselink,
                                      const std::string& camera_frame_id,
                                      const std::vector<double>& K_vec) {
    core::proto::KeyFrame kf_proto;
    kf_proto.set_id(id);

    // Pose (odom_T_baselink)
    *kf_proto.mutable_pose() = tracking::proto_utils::toProto(odom_T_baselink);
    kf_proto.mutable_pose()->set_frame_id("odom"); // Assuming odom is the world frame for pose

    // CameraInfo
    auto* ci_proto = kf_proto.mutable_camera_info();
    ci_proto->set_frame_id(camera_frame_id);
    ci_proto->set_width(640); // Example
    ci_proto->set_height(480); // Example
    for(double k_val : K_vec) ci_proto->add_k(k_val);
    // Add distortion if any, e.g., ci_proto->set_distortion_model("plumb_bob"); ci_proto->add_d(0.0);

    // Color Image (optional, can be empty if not testing tracker)
    // auto* img_proto = kf_proto.mutable_color_image();
    // img_proto->set_width(640); img_proto->set_height(480); img_proto->set_encoding("bgr8");
    // img_proto->set_frame_id(camera_frame_id);
    // std::vector<uint8_t> dummy_image_data(640 * 480 * 3, 128); // Dummy gray image
    // img_proto->set_data(dummy_image_data.data(), dummy_image_data.size());

    return kf_proto;
}

// Helper to project points (simplified, assumes point.z > 0)
Eigen::Vector2d project(const Eigen::Vector3d& point_c, const Eigen::Matrix3d& K_eig) {
    Eigen::Vector3d projected_homo = K_eig * (point_c / point_c.z());
    return Eigen::Vector2d(projected_homo.x(), projected_homo.y());
}

int main(int argc, char** argv) {
    tracking::proto::TrackingTestCase test_case;

    std::string camera_optical_frame = "camera_optical_frame";
    std::vector<double> K_values = {500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
    Eigen::Matrix3d K_eigen;
    K_eigen << K_values[0], K_values[1], K_values[2],
               K_values[3], K_values[4], K_values[5],
               K_values[6], K_values[7], K_values[8];

    // KeyFrame 1
    Eigen::Isometry3d odom_T_baselink_kf1 = Eigen::Isometry3d::Identity();
    *test_case.mutable_keyframe1() = create_kf_proto(1, odom_T_baselink_kf1, camera_optical_frame, K_values);

    // KeyFrame 2
    Eigen::Isometry3d odom_T_baselink_kf2 = Eigen::Isometry3d::Identity();
    odom_T_baselink_kf2.translation().x() = 0.2; // Baselink moved 0.2m in X
    *test_case.mutable_keyframe2() = create_kf_proto(2, odom_T_baselink_kf2, camera_optical_frame, K_values);

    // Transform Tree
    stf::proto::TransformTreeSnapshot* tft_proto = test_case.mutable_transform_tree();
    // Edge 1: odom -> base_link (can be static or updated with kf poses)
    // For simplicity, assume odom is the parent of base_link from kf1.pose
    // If kf.pose.frame_id is "odom", this means odom_T_kf.frame_id (implicit base_link for that kf)
    // To keep it simple: let's define base_link relative to odom as identity for KF1, and displaced for KF2
    // However, the TransformTree usually holds *static* or slowly changing transforms.
    // The KeyFrame poses are odom_T_base_link.
    // The important static transform is base_link_T_camera_optical_frame.

    Eigen::Isometry3d base_link_T_camera = Eigen::Isometry3d::Identity(); // Simplification
    // base_link_T_camera.translation() = Eigen::Vector3d(0.05, 0, 0.02); // Example offset

    stf::proto::TransformEdge* edge1 = tft_proto->add_edges();
    edge1->set_parent_frame_id("base_link");
    edge1->set_child_frame_id(camera_optical_frame);
    *edge1->mutable_transform() = tracking::proto_utils::toProto(base_link_T_camera);

    // Matched Points & Ground Truth
    // Define some 3D points in KF1's CAMERA frame
    std::vector<Eigen::Vector3d> points_in_kf1_camera_frame;
    points_in_kf1_camera_frame.push_back(Eigen::Vector3d(0.1, 0.05, 2.0));
    points_in_kf1_camera_frame.push_back(Eigen::Vector3d(-0.1, -0.05, 1.5));

    tracking::proto::MatchedPoints* matches_proto = test_case.mutable_matches();

    // Calculate odom_T_camera_kf1 and odom_T_camera_kf2
    Eigen::Isometry3d odom_T_camera_kf1 = odom_T_baselink_kf1 * base_link_T_camera;
    Eigen::Isometry3d odom_T_camera_kf2 = odom_T_baselink_kf2 * base_link_T_camera;

    // Relative transform camera1_T_camera2
    Eigen::Isometry3d camera1_T_camera2 = odom_T_camera_kf1.inverse() * odom_T_camera_kf2;

    for (const auto& p_c1 : points_in_kf1_camera_frame) {
        // Point in KF1 pixel coords
        Eigen::Vector2d px1 = project(p_c1, K_eigen);
        auto* p1_proto = matches_proto->add_points1();
        p1_proto->set_x(px1.x()); p1_proto->set_y(px1.y());

        // Point in KF2 camera coords
        Eigen::Vector3d p_c2 = camera1_T_camera2.inverse() * p_c1; // Transform point from C1 to C2 frame
        if (p_c2.z() <= 0) {
            std::cerr << "Point behind camera 2! Check test data setup." << std::endl; continue;
        }
        Eigen::Vector2d px2 = project(p_c2, K_eigen);
        auto* p2_proto = matches_proto->add_points2();
        p2_proto->set_x(px2.x()); p2_proto->set_y(px2.y());

        // Ground truth: point in world (odom) frame
        Eigen::Vector3d p_world = odom_T_camera_kf1 * p_c1;
        auto* gt_proto = test_case.add_ground_truth_points_world();
        gt_proto->set_x(p_world.x());
        gt_proto->set_y(p_world.y());
        gt_proto->set_z(p_world.z());
    }

    std::string output_filename = "tracking/test_data/pipeline_case_1.pb";
    if (tracking::proto_utils::saveTrackingTestCase(test_case, output_filename)) {
        std::cout << "Successfully saved test case to " << output_filename << std::endl;
    } else {
        std::cerr << "Failed to save test case to " << output_filename << std::endl;
        return 1;
    }

    return 0;
}
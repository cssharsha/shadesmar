#include "tracking/proto_utils.hpp"
#include <fstream>
#include <vector>
#include <opencv2/imgcodecs.hpp> // For imencode, imdecode

namespace tracking {
namespace proto_utils {

// Eigen::Isometry3d <-> core.proto.Pose
core::proto::Pose toProto(const Eigen::Isometry3d& isometry) {
    core::proto::Pose pose_proto;
    pose_proto.mutable_position()->set_x(isometry.translation().x());
    pose_proto.mutable_position()->set_y(isometry.translation().y());
    pose_proto.mutable_position()->set_z(isometry.translation().z());
    Eigen::Quaterniond q(isometry.linear());
    pose_proto.mutable_orientation()->set_w(q.w());
    pose_proto.mutable_orientation()->set_x(q.x());
    pose_proto.mutable_orientation()->set_y(q.y());
    pose_proto.mutable_orientation()->set_z(q.z());
    return pose_proto;
}

Eigen::Isometry3d fromProto(const core::proto::Pose& pose_proto) {
    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    isometry.translation() = Eigen::Vector3d(pose_proto.position().x(),
                                             pose_proto.position().y(),
                                             pose_proto.position().z());
    isometry.linear() = Eigen::Quaterniond(pose_proto.orientation().w(),
                                           pose_proto.orientation().x(),
                                           pose_proto.orientation().y(),
                                           pose_proto.orientation().z()).toRotationMatrix();
    return isometry;
}

// core::types::Image <-> core.proto.Image
core::proto::Image toProto(const core::types::Image& image_data) {
    core::proto::Image image_proto;
    if (!image_data.data.empty()) {
        std::vector<uchar> buf;
        bool success = cv::imencode(".jpg", image_data.data, buf);
        if (success) {
            image_proto.set_data(buf.data(), buf.size());
        }
    }
    image_proto.set_width(image_data.data.cols);
    image_proto.set_height(image_data.data.rows);
    image_proto.set_channels(image_data.data.channels());
    image_proto.set_encoding(image_data.encoding);
    image_proto.set_frame_id(image_data.frame_id);
    return image_proto;
}

core::types::Image fromProto(const core::proto::Image& image_proto) {
    core::types::Image image_data;
    if (!image_proto.data().empty()) {
        std::vector<uchar> buf(image_proto.data().begin(), image_proto.data().end());
        image_data.data = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
    }
    image_data.encoding = image_proto.encoding();
    image_data.frame_id = image_proto.frame_id();
    return image_data;
}

// core::types::CameraInfo <-> core.proto.CameraInfo
core::proto::CameraInfo toProto(const core::types::CameraInfo& camera_info) {
    core::proto::CameraInfo proto_info;
    proto_info.set_width(camera_info.width);
    proto_info.set_height(camera_info.height);
    proto_info.set_distortion_model(camera_info.distortion_model);
    proto_info.set_frame_id(camera_info.frame_id);
    for (double val : camera_info.k) {
        proto_info.add_k(val);
    }
    for (double val : camera_info.d) {
        proto_info.add_d(val);
    }
    return proto_info;
}

core::types::CameraInfo fromProto(const core::proto::CameraInfo& camera_info_proto) {
    core::types::CameraInfo info;
    info.width = camera_info_proto.width();
    info.height = camera_info_proto.height();
    info.distortion_model = camera_info_proto.distortion_model();
    info.frame_id = camera_info_proto.frame_id();
    info.k.assign(camera_info_proto.k().begin(), camera_info_proto.k().end());
    info.d.assign(camera_info_proto.d().begin(), camera_info_proto.d().end());
    return info;
}

// core::types::KeyFrame <-> core.proto.KeyFrame
core::proto::KeyFrame toProto(const core::types::KeyFrame& keyframe) {
    core::proto::KeyFrame proto_kf;
    proto_kf.set_id(keyframe.id);
    *proto_kf.mutable_pose() = toProto(keyframe.pose.getEigenIsometry());
    if (keyframe.camera_info.has_value()) {
        *proto_kf.mutable_camera_info() = toProto(keyframe.camera_info.value());
    }
    if (keyframe.color_data.has_value()) {
        *proto_kf.mutable_color_image() = toProto(keyframe.color_data.value());
    }
    return proto_kf;
}

core::types::KeyFrame fromProto(const core::proto::KeyFrame& keyframe_proto) {
    core::types::KeyFrame kf;
    kf.id = keyframe_proto.id();

    // Convert Eigen::Isometry3d to Pose manually
    Eigen::Isometry3d isometry = fromProto(keyframe_proto.pose());
    kf.pose.position = isometry.translation();
    kf.pose.orientation = Eigen::Quaterniond(isometry.linear());

    if (keyframe_proto.has_camera_info()) {
        kf.camera_info = fromProto(keyframe_proto.camera_info());
    }
    if (keyframe_proto.has_color_image()) {
        kf.color_data = fromProto(keyframe_proto.color_image());
    }
    return kf;
}

// stf::TransformTree <-> stf.proto.TransformTreeSnapshot
stf::proto::TransformTreeSnapshot toProto(const stf::TransformTree& transform_tree) {
    stf::proto::TransformTreeSnapshot snapshot_proto;
    std::vector<stf::TransformTree::Edge> all_edges = transform_tree.getAllEdges();
    for (const auto& edge : all_edges) {
        stf::proto::TransformEdge* edge_proto = snapshot_proto.add_edges();
        edge_proto->set_parent_frame_id(edge.parent);
        edge_proto->set_child_frame_id(edge.child);
        *edge_proto->mutable_transform() = toProto(edge.transform);
    }
    return snapshot_proto;
}

stf::TransformTree fromProto(const stf::proto::TransformTreeSnapshot& snapshot_proto) {
    stf::TransformTree transform_tree;
    for (const auto& edge_proto : snapshot_proto.edges()) {
        transform_tree.setTransform(edge_proto.parent_frame_id(),
                                    edge_proto.child_frame_id(),
                                    fromProto(edge_proto.transform()));
    }
    return transform_tree;
}

// std::vector<cv::Point2f> <-> tracking.proto.MatchedPoints
tracking::proto::MatchedPoints toProto(const std::vector<cv::Point2f>& points1,
                                       const std::vector<cv::Point2f>& points2) {
    tracking::proto::MatchedPoints matches_proto;
    for (const auto& p : points1) {
        auto* pt_proto = matches_proto.add_points1();
        pt_proto->set_x(p.x);
        pt_proto->set_y(p.y);
    }
    for (const auto& p : points2) {
        auto* pt_proto = matches_proto.add_points2();
        pt_proto->set_x(p.x);
        pt_proto->set_y(p.y);
    }
    return matches_proto;
}

void fromProto(const tracking::proto::MatchedPoints& matches_proto,
               std::vector<cv::Point2f>& points1,
               std::vector<cv::Point2f>& points2) {
    points1.clear();
    points2.clear();
    for (const auto& pt_proto : matches_proto.points1()) {
        points1.emplace_back(pt_proto.x(), pt_proto.y());
    }
    for (const auto& pt_proto : matches_proto.points2()) {
        points2.emplace_back(pt_proto.x(), pt_proto.y());
    }
}

// Load/Save TrackingTestCase
bool saveTrackingTestCase(const tracking::proto::TrackingTestCase& test_case, const std::string& filename) {
    std::fstream output(filename, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!output) {
        return false;
    }
    return test_case.SerializeToOstream(&output);
}

tracking::proto::TrackingTestCase loadTrackingTestCase(const std::string& filename) {
    tracking::proto::TrackingTestCase test_case;
    std::fstream input(filename, std::ios::in | std::ios::binary);
    if (!input) {
        return test_case;
    }
    if (!test_case.ParseFromIstream(&input)) {
        return test_case;
    }
    return test_case;
}

} // namespace proto_utils
} // namespace tracking
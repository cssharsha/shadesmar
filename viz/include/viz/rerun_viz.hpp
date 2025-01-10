#pragma once

#include <rerun.hpp>
#include <rerun/archetypes/pinhole.hpp>

#include "viz/interface.hpp"

namespace viz {

class RerunVisualizer : public VisualizerInterface {
public:
    RerunVisualizer(const std::string& name = "shadesmar", const std::string& host = "localhost",
                    uint16_t port = 9999);

    bool initialize() override {
        return initialize(false);
    }
    bool initialize(bool save_to_file);
    bool isConnected() const override;
    void disconnect() override;

    void addPose(const core::types::Pose& pose, const std::string& label = "") override {}
    void addPointCloud(const core::types::PointCloud& cloud,
                       const core::types::Pose& pose = core::types::Pose()) override {}
    void addImage(const cv::Mat& image, const std::string& name = "image") override {}

    void visualizeFactorGraph(const core::graph::FactorGraph& graph) override;
    void visualizeKeyFrame(const core::types::KeyFrame::ConstPtr& keyframe) override;
    void visualizeOdometryTrajectory(const std::vector<Eigen::Vector3d>& positions);

    void clear() override;
    void update() override;
    void setTimestamp(double timestamp) override;

    // Add new method to get time range
    std::pair<double, double> getTimeRange(const core::graph::FactorGraph& graph);

    void addPose(const core::types::Pose& pose, const std::string& entity_path, double timestamp);
    void addImage(const cv::Mat& image, const std::string& entity_path, double timestamp);
    void addPointCloud(const core::types::PointCloud& cloud, const std::string& entity_path,
                       double timestamp, const core::types::Pose& transform);
    void addCamera(const rerun::archetypes::Pinhole& camera, const std::string& entity_path,
                   double timestamp = 0);

private:
    std::string name_;
    std::string host_;
    uint16_t port_;
    rerun::RecordingStream rec_;
    bool is_connected_ = false;
    double current_timestamp_ = 0;

    // Helper functions
    rerun::Transform3D toRerunTransform(const core::types::Pose& pose);
    rerun::Points3D toRerunPoints(const core::types::PointCloud& cloud);
    rerun::Points3D toRerunPoints(const core::types::PointCloud& cloud,
                                  const core::types::Pose& pose);
};
}  // namespace viz

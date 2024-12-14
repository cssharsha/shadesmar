#pragma once

#include <rerun.hpp>
#include "viz/interface.hpp"

namespace viz {

class RerunVisualizer : public VisualizerInterface {
public:
    RerunVisualizer(const std::string& name = "shadesmar");

    bool initialize() override;
    bool isConnected() const override;
    void disconnect() override;

    void addPose(const core::types::Pose& pose, const std::string& label = "") override;
    void addPointCloud(const core::types::PointCloud& cloud,
                       const core::types::Pose& pose = core::types::Pose()) override;
    void addImage(const cv::Mat& image, const std::string& name = "image") override;

    void visualizeFactorGraph(const core::graph::FactorGraph& graph) override;
    void visualizeKeyFrame(const core::types::KeyFrame::ConstPtr& keyframe) override;

    void clear() override;
    void update() override;
    void setTimestamp(double timestamp) override;

    // Add new method to get time range
    std::pair<double, double> getTimeRange(const core::graph::FactorGraph& graph);

private:
    std::string name_;
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

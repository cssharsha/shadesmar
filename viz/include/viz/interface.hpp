#pragma once

#include <memory>
#include <string>
#include "core/graph/factor_graph.hpp"
#include "core/types/keyframe.hpp"
#include "core/types/pose.hpp"

namespace viz {

class VisualizerInterface {
public:
    virtual ~VisualizerInterface() = default;

    // Connection management
    virtual bool initialize() = 0;
    virtual bool isConnected() const = 0;
    virtual void disconnect() = 0;

    // Basic visualization primitives
    virtual void addPose(const core::types::Pose& pose, const std::string& label = "") = 0;
    virtual void addPointCloud(const core::types::PointCloud& cloud,
                               const core::types::Pose& pose = core::types::Pose()) = 0;
    virtual void addImage(const cv::Mat& image, const std::string& name = "image") = 0;

    // Higher level visualization
    virtual void visualizeFactorGraph(const core::graph::FactorGraph& graph) = 0;
    virtual void visualizeKeyFrame(const core::types::KeyFrame::ConstPtr& keyframe) = 0;

    // Frame management
    virtual void clear() = 0;
    virtual void update() = 0;
    virtual void setTimestamp(double timestamp) = 0;
};

}  // namespace viz

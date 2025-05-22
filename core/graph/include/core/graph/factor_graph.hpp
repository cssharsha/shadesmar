#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <core/types/factor.hpp>
#include <core/types/keyframe.hpp>

namespace core {
namespace graph {

using KeyFramePtr = types::KeyFrame::Ptr;

class FactorGraph {
public:
    FactorGraph() = default;

    void addKeyFrame(const KeyFramePtr& keyframe);
    void addFactor(const types::Factor& factor);
    bool optimize();

    KeyFramePtr getKeyFramePtr(uint64_t id) const;
    types::KeyFrame& getKeyFrame(uint64_t id) const;
    std::vector<KeyFramePtr> getAllKeyFrames() const;
    std::vector<types::Factor> getFactors() const;

private:
    std::map<uint64_t, KeyFramePtr> keyframes_;
    std::vector<types::Factor> factors_;

    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_estimates_;
    gtsam::Values result_;

    void updateGTSAMGraph();
    gtsam::Pose3 toPose3(const types::Pose& pose) const;
    types::Pose fromPose3(const gtsam::Pose3& pose3) const;
};

}  // namespace graph
}  // namespace core
